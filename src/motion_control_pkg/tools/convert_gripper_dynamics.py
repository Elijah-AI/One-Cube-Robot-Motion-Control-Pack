#!/usr/bin/env python3
"""Offline OneCube CCS gripper -> Tianji OnSetTool dynamics (no robot I/O).

Inputs must be the robot's installed URDF and MvKDCfg. The gripper subtree
must contain only fixed joints or joints locked at zero. All URDF inertias
are about their own COM; the SDK output inertia is about the flange origin.
Requires Python 3 and NumPy. This utility only writes the requested report.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                             math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def vector(text):
    v = np.array([float(x) for x in text.split()])
    if v.shape != (3,) or not np.isfinite(v).all():
        raise ValueError("expected three finite coordinates")
    return v


def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, :3] = rotation(vector(element.get("rpy", "0 0 0")))
        t[:3, 3] = vector(element.get("xyz", "0 0 0"))
    return t


def mdh(row):
    # Craig MDH: Rx(alpha) Tx(a) Rz(theta) Tz(d); degrees/mm -> radians/m.
    alpha, a, d, theta = row
    alpha, theta = np.deg2rad([alpha, theta])
    rx = rotation([alpha, 0, 0])
    t = np.eye(4)
    t[:3, :3] = rx @ rotation([0, 0, theta])
    t[:3, 3] = np.array([a, 0, 0])/1000 + rx @ np.array([0, 0, d/1000])
    return t


def read_ccs_dh(path):
    rows = [[float(v) for v in line.strip().rstrip(",").split(",")]
            for line in Path(path).read_text().splitlines() if line.strip()]
    if len(rows) != 26 or any(not np.isfinite(row).all() for row in rows):
        raise ValueError("expected two finite 13-row MvKDCfg arm records")
    arms = []
    for offset in [0, 13]:
        if rows[offset][0] != 1017:
            raise ValueError("this converter requires the OneCube Pilot-CCS model (1017)")
        dh = [row[:4] for row in rows[offset+1:offset+9]]
        # This is the verified CCS/URDF terminal-frame convention. Do not
        # silently apply the mapping to another arm model or flange layout.
        if not np.allclose(dh[7], [90, 0, 95, 90], atol=1e-9, rtol=0):
            raise ValueError("unverified CCS flange DH; review the frame mapping first")
        arms.append(dh)
    return arms


def check_inertia(tensor, label):
    if not np.isfinite(tensor).all():
        raise ValueError(label + ": non-finite inertia")
    # PSD of the second moment enforces the principal-moment triangle
    # inequalities as well as positivity, including off-diagonal terms.
    second_moment = np.trace(tensor)/2*np.eye(3) - tensor
    if np.linalg.eigvalsh(second_moment).min() < -1e-12:
        raise ValueError(label + ": physically inconsistent central inertia")


def parallel_axis(mass, com):
    return mass * ((com @ com)*np.eye(3) - np.outer(com, com))


def convert_arm(robot, dh, side):
    links = {x.get("name"): x for x in robot.findall("link")}
    joints = {x.get("name"): x for x in robot.findall("joint")}
    letter = side.upper()
    # Validate the URDF arm frames against this robot's SDK MDH. The URDF
    # stores pi/2 as 1.5708, so allow only that small rounding discrepancy.
    frame_checks = []
    for index in range(1, 8):
        joint = joints[f"Joint{index}_{letter}"]
        expected_parent = (f"arm-{side}-base_link" if index == 1
                           else f"Link{index-1}_{letter}")
        if (joint.get("type") != "revolute" or
                joint.find("parent").get("link") != expected_parent or
                joint.find("child").get("link") != f"Link{index}_{letter}" or
                not np.array_equal(vector(joint.find("axis").get("xyz")), [0, 0, 1])):
            raise ValueError("unverified arm chain: " + joint.get("name"))
        actual, expected = origin(joint.find("origin")), mdh(dh[index-1])
        rotation_error = float(np.max(np.abs(actual[:3, :3]-expected[:3, :3])))
        translation_error = float(np.linalg.norm(actual[:3, 3]-expected[:3, 3]))
        if rotation_error > 1.5e-5 or translation_error > 1e-9:
            raise ValueError("URDF/SDK arm frame mismatch: " + joint.get("name"))
        frame_checks.append({"joint": joint.get("name"),
                             "rotation_max_error": rotation_error,
                             "translation_error_m": translation_error})

    root = f"gripper_base_{side}_link"
    mount = joints[f"gripper_base_{side}_joint"]
    if (mount.get("type") != "fixed" or
            mount.find("parent").get("link") != f"Link7_{letter}" or
            mount.find("child").get("link") != root):
        raise ValueError("unverified gripper mounting joint")
    # F_T_G includes the URDF's 0.36386 mm lateral mount offset and its
    # literal rounded RPY; G is not silently equated with the SDK flange F.
    flange_to_root = np.linalg.inv(mdh(dh[7])) @ origin(mount.find("origin"))
    queue = [(root, flange_to_root)]
    parts, seen, locked_joints = [], set(), []
    for name, placement in queue:
        if name in seen:
            raise ValueError("repeated/cyclic gripper link: " + name)
        seen.add(name)
        inertial = links[name].find("inertial")
        if inertial is None:
            raise ValueError("missing inertial for gripper link: " + name)
        mass = float(inertial.find("mass").get("value"))
        if not math.isfinite(mass) or mass <= 0:
            raise ValueError("invalid mass for " + name)
        i = {k: float(v) for k, v in inertial.find("inertia").attrib.items()}
        central = np.array([[i["ixx"], i["ixy"], i["ixz"]],
                            [i["ixy"], i["iyy"], i["iyz"]],
                            [i["ixz"], i["iyz"], i["izz"]]])
        check_inertia(central, name)
        com_frame = placement @ origin(inertial.find("origin"))
        com = com_frame[:3, 3]
        central = com_frame[:3, :3] @ central @ com_frame[:3, :3].T
        parts.append({"link": name, "mass_kg": mass, "com_flange_m": com.tolist(),
                      "central_inertia_flange_axes_kg_m2": central.tolist()})
        for joint in joints.values():
            if joint.find("parent").get("link") != name:
                continue
            if joint.get("type") != "fixed":
                limit = joint.find("limit")
                if (joint.get("type") not in ("prismatic", "revolute") or limit is None or
                        float(limit.get("lower", "nan")) != 0 or
                        float(limit.get("upper", "nan")) != 0):
                    raise ValueError("gripper joint must be locked at zero: " + joint.get("name"))
                locked_joints.append(joint.get("name"))
            queue.append((joint.find("child").get("link"),
                          placement @ origin(joint.find("origin"))))

    mass = sum(p["mass_kg"] for p in parts)
    com = sum(p["mass_kg"]*np.array(p["com_flange_m"]) for p in parts)/mass
    inertia = sum(np.array(p["central_inertia_flange_axes_kg_m2"]) +
                  parallel_axis(p["mass_kg"], np.array(p["com_flange_m"])) for p in parts)
    central = inertia - parallel_axis(mass, com)
    check_inertia(central, root + " aggregate")
    dynamics = [mass, *(com*1000), inertia[0, 0], inertia[0, 1], inertia[0, 2],
                inertia[1, 1], inertia[1, 2], inertia[2, 2]]
    return {"root_link": root, "mass_kg": mass, "com_flange_mm": (com*1000).tolist(),
            "inertia_at_flange_kg_m2": inertia.tolist(),
            "inertia_at_combined_com_kg_m2": central.tolist(),
            "central_principal_moments_kg_m2": np.linalg.eigvalsh(central).tolist(),
            "sdk_dynamics": [float(x) for x in dynamics],
            "sdk_flange_mdh_deg_mm": dh[7],
            "flange_T_gripper_base_m": flange_to_root.tolist(),
            "arm_frame_checks": frame_checks, "locked_zero_joints": locked_joints,
            "parts": parts}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--sdk-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="new JSON report; never overwritten")
    args = parser.parse_args()
    robot = ET.parse(args.urdf).getroot()
    dh = read_ccs_dh(args.sdk_config)
    report = {"source_kind": "nominal installed URDF; not measured payload identification",
              "urdf": str(args.urdf.resolve()),
              "urdf_sha256": hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
              "sdk_config": str(args.sdk_config.resolve()),
              "sdk_config_sha256": hashlib.sha256(args.sdk_config.read_bytes()).hexdigest(),
              "order": ["M", "MCP_X", "MCP_Y", "MCP_Z", "I_XX", "I_XY", "I_XZ", "I_YY", "I_YZ", "I_ZZ"],
              "units": "kg, mm, kg*m^2; COM and inertia referred to SDK flange origin and axes",
              "left": convert_arm(robot, dh[0], "l"),
              "right": convert_arm(robot, dh[1], "r")}
    with args.output.open("x") as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write("\n")
    for side in ["left", "right"]:
        values = ", ".join(format(v, ".14g") for v in report[side]["sdk_dynamics"])
        print(f"{side}_tool_dynamics: [{values}]")


if __name__ == "__main__":
    main()
