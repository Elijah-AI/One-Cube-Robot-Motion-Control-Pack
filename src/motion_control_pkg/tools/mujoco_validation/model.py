"""Load the exact source URDF into MuJoCo for fixed-base kinematic validation.

Collision DAE meshes are deliberately excluded: this model does not certify
collision clearance, dynamics, ground contact, balance, or physical tracking.
"""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


def numbers(value):
    return np.array([float(x) for x in value.split()])


def fmt(value):
    return " ".join(format(float(x), ".17g") for x in np.asarray(value).ravel())


def compile_model(bundle, output):
    output = Path(output)
    urdf = Path(bundle["urdf"]).resolve(strict=True)
    raw = urdf.read_bytes()
    if raw != bundle["urdf_xml"].encode():
        raise ValueError("URDF changed since C++ export; regenerate the trajectories")
    tree = ET.fromstring(raw)
    limits, locked = {}, {}
    for joint in tree.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        if joint.get("type") not in ("revolute", "prismatic") or joint.find("mimic") is not None:
            raise ValueError("unsupported joint: " + joint.get("name"))
        limit = joint.find("limit")
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        limits[joint.get("name")] = dict(lower=lower, upper=upper, velocity=float(limit.get("velocity")))
        if lower == upper:
            # The current source has zero-locked wheel joints. Fail if this
            # changes, rather than dropping a nonzero fixed transformation.
            if lower != 0:
                raise ValueError("nonzero locked joint needs explicit transform")
            locked[joint.get("name")] = lower
            joint.set("type", "fixed")
    for link in tree.findall("link"):
        for collision in link.findall("collision"):
            link.remove(collision)
    mesh_hashes = {}
    for mesh in tree.findall(".//mesh"):
        uri = mesh.get("filename")
        path = (urdf.parents[2] / uri[len("package://"):]) if uri.startswith("package://") else urdf.parent / uri
        path = path.resolve(strict=True)
        mesh.set("filename", str(path))
        mesh_hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    extension = tree.find("mujoco")
    if extension is not None:
        tree.remove(extension)
    ET.SubElement(ET.SubElement(tree, "mujoco"), "compiler", discardvisual="false",
                  fusestatic="false", balanceinertia="false", strippath="false")
    imported = mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))
    neutral = mujoco.MjData(imported)
    mujoco.mj_forward(imported, neutral)
    wheel_bottoms = []
    for i in range(imported.ngeom):
        body_name = mujoco.mj_id2name(imported, mujoco.mjtObj.mjOBJ_BODY, int(imported.geom_bodyid[i])) or ""
        if "wheel" in body_name and imported.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = imported.geom_dataid[i]
            begin, count = imported.mesh_vertadr[mesh], imported.mesh_vertnum[mesh]
            vertices = imported.mesh_vert[begin:begin+count]
            world_vertices = vertices @ neutral.geom_xmat[i].reshape(3, 3).T + neutral.geom_xpos[i]
            wheel_bottoms.append(float(world_vertices[:, 2].min()))
    ground_height = min(wheel_bottoms) if wheel_bottoms else -.14
    # MuJoCo performs the URDF transform and inertia conversion itself.
    intermediate = output / "robot_import.xml"
    mujoco.mj_saveLastXML(str(intermediate), imported)
    scene = ET.parse(intermediate).getroot()
    # mj_saveLastXML rounds body quaternions in its human-readable output.
    # Preserve the imported model's double-precision kinematic transforms;
    # large waist angles expose that otherwise small serialization error.
    for body in scene.iter("body"):
        i = mujoco.mj_name2id(imported, mujoco.mjtObj.mjOBJ_BODY, body.get("name"))
        if i < 0:
            raise ValueError("serialized body missing from imported model")
        body.set("pos", fmt(imported.body_pos[i]))
        body.set("quat", fmt(imported.body_quat[i]))
    for joint in scene.findall(".//joint"):
        name = joint.get("name")
        if name is None:  # Optional default joint properties.
            continue
        i = mujoco.mj_name2id(imported, mujoco.mjtObj.mjOBJ_JOINT, name)
        if i < 0:
            raise ValueError("serialized joint missing from imported model")
        joint.set("pos", fmt(imported.jnt_pos[i]))
        joint.set("axis", fmt(imported.jnt_axis[i]))
    option = scene.find("option")
    if option is None:
        option = ET.SubElement(scene, "option")
    option.set("timestep", str(bundle["dt_sec"]))
    for geom in scene.findall(".//geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
    world = scene.find("worldbody")
    if bundle.get("constraint_mode") == "rigid_grasp":
        box = ET.SubElement(world, "body", name="carried_box", mocap="true", pos="0 0 1")
        ET.SubElement(box, "geom", name="box_visual", type="box", size=".18 .30 .12",
                      rgba=".65 .40 .18 .85", contype="0", conaffinity="0")
        ET.SubElement(box, "site", type="box", pos="0 0 .121", size=".17 .29 .001", rgba=".8 .58 .27 .6")
    ET.SubElement(world, "light", pos="2 -3 4", dir="-2 3 -4", directional="true", diffuse=".6 .6 .6")
    ET.SubElement(world, "light", pos="-2 1 3", dir="2 -1 -3", diffuse=".25 .25 .25")
    ET.SubElement(world, "geom", name="display_ground", type="plane", size="3 3 .1",
                  pos=fmt([0, 0, ground_height]), rgba=".12 .16 .20 1", contype="0", conaffinity="0")
    ET.SubElement(scene.find("asset"), "texture", type="skybox", builtin="gradient",
                  rgb1=".13 .17 .23", rgb2=".04 .06 .09", width="512", height="3072")
    bodies = {b.get("name"): b for b in world.iter("body")}
    for side, color in (("left", ".1 .75 1 .65"), ("right", "1 .55 .12 .65")):
        link = "gripper_base_l_link" if side == "left" else "gripper_base_r_link"
        tool = np.array(bundle[side + "_tool"]).reshape(4, 4)
        q = Rotation.from_matrix(tool[:3, :3]).as_quat()[[3, 0, 1, 2]]
        body = ET.SubElement(bodies[link], "body", name=side + "_tcp_frame",
                             pos=fmt(tool[:3, 3]), quat=fmt(q))
        if bundle.get("constraint_mode") == "rigid_grasp":
            # Schematic vacuum cup: its contact point is exactly the calibrated
            # TCP, with the stem opposite the +Z suction direction. These are
            # display-only shapes, not measured tooling or contact constraints.
            ET.SubElement(body, "site", name=side + "_tcp", type="cylinder",
                          size=".036 .001", rgba=color)
            ET.SubElement(body, "geom", type="cylinder", fromto="0 0 -.075 0 0 -.025",
                          size=".010", rgba=".25 .28 .32 1", contype="0", conaffinity="0")
            for radius, z in ((.016, -.025), (.023, -.017), (.030, -.010)):
                ET.SubElement(body, "geom", type="cylinder", pos=fmt([0, 0, z]),
                              size=fmt([radius, .004]), rgba=color,
                              contype="0", conaffinity="0")
        else:
            ET.SubElement(body, "site", name=side + "_tcp", type="box", size=".11 .07 .004", rgba=color)
            ET.SubElement(body, "site", type="cylinder", fromto="0 0 0 0 0 .12", size=".004", rgba=color)
    visual = scene.find("visual")
    if visual is None:
        visual = ET.SubElement(scene, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    ET.SubElement(visual, "headlight", ambient=".2 .2 .2", diffuse=".3 .3 .3")
    path = output / "scene.xml"
    ET.indent(scene)
    ET.ElementTree(scene).write(path, encoding="unicode")
    model = mujoco.MjModel.from_xml_path(str(path))
    manifest = dict(urdf=str(urdf), urdf_sha256=hashlib.sha256(raw).hexdigest(),
                    mesh_sha256=mesh_hashes, limits=limits, locked=locked,
                    mujoco_version=mujoco.__version__, fixed_base=True,
                    collisions=False, dynamics=False)
    return model, manifest


class Robot:
    def __init__(self, model, bundle, manifest):
        self.model, self.data = model, mujoco.MjData(model)
        self.names = bundle["joint_names"]
        if len(self.names) != len(set(self.names)) or set(self.names) != set(manifest["limits"]):
            raise ValueError("C++/URDF joint name set mismatch")
        self.mapping = []
        for i, name in enumerate(self.names):
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint < 0:
                if name not in manifest["locked"]:
                    raise ValueError("missing MuJoCo joint: " + name)
            else:
                if model.jnt_type[joint] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                    raise ValueError("non-scalar joint")
                self.mapping.append((i, int(model.jnt_qposadr[joint])))
        if len(self.mapping) != model.nq:
            raise ValueError("unmapped MuJoCo joint")
        self.sites = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, side + "_tcp") for side in ("left", "right")]
        if min(self.sites) < 0:
            raise ValueError("missing TCP sites")
        self.box_anchor = None
        self.box_mocap = None
        if bundle.get("constraint_mode") == "rigid_grasp":
            self.set_q(bundle["scenarios"][0]["samples"][0]["q"])
            (lp, lr), (rp, _) = self.tcp()
            center = (lp+rp)/2 - np.array([0, 0, .12])
            self.box_anchor = (lr.T @ (center-lp), lr.T)
            self.box_mocap = int(model.body_mocapid[model.body("carried_box").id])

    def set_q(self, q):
        if len(q) != len(self.names) or not np.isfinite(q).all():
            raise ValueError("invalid full joint sample")
        self.data.qpos[:] = self.model.qpos0
        for source, dest in self.mapping:
            self.data.qpos[dest] = q[source]
        mujoco.mj_forward(self.model, self.data)
        if self.box_anchor is not None:
            p, r = self.tcp()[0]
            offset, rotation = self.box_anchor
            self.data.mocap_pos[self.box_mocap] = p + r @ offset
            self.data.mocap_quat[self.box_mocap] = Rotation.from_matrix(r @ rotation).as_quat()[[3, 0, 1, 2]]
            mujoco.mj_forward(self.model, self.data)

    def tcp(self):
        return [(self.data.site_xpos[i].copy(), self.data.site_xmat[i].reshape(3, 3).copy()) for i in self.sites]


def tilt(rotation, sign):
    normal = rotation[:, 2] * sign
    return np.arctan2(np.linalg.norm(normal[:2]), normal[2])
