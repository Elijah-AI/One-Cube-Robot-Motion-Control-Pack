#!/usr/bin/env python3
"""Independent MuJoCo FK checks, A/B video, and an interactive trajectory viewer."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np


def check_scene(robot, scene, bundle, manifest, output):
    from model import tilt
    names = bundle["joint_names"]
    q = np.array([s["q"] for s in scene["samples"]])
    if q.ndim != 2 or q.shape[1] != len(names) or len(q) < 2 or not np.isfinite(q).all():
        raise ValueError("invalid trajectory")
    dt = bundle["dt_sec"]
    if dt != .004 or abs(scene["duration_sec"] - (len(q)-1)*dt) > 1e-9:
        raise ValueError("trajectory clock mismatch")
    arm_indices = [i for i, n in enumerate(names) if n.startswith("Joint")]
    baseline = q.copy()
    baseline[:, arm_indices] = q[0, arm_indices]
    rigid = bundle.get("constraint_mode") == "rigid_grasp"
    if rigid:
        baseline = np.array([s["legacy_q"] for s in scene["samples"]])
    lower = np.array([manifest["limits"][n]["lower"] for n in names])
    upper = np.array([manifest["limits"][n]["upper"] for n in names])
    velocity = np.array([manifest["limits"][n]["velocity"] for n in names])
    moving = velocity > 0
    position_excess = float(max(0., np.max(lower-q), np.max(q-upper)))
    velocity_ratio = float(np.max(np.abs(np.diff(q, axis=0)[:, moving]) / (dt*velocity[moving])))
    robot.set_q(q[0])
    signs = [1 if r[2, 2] >= 0 else -1 for _, r in robot.tcp()]
    (lp, lr), (rp, rr) = robot.tcp()
    initial_relative_position, initial_relative_rotation = lr.T @ (rp-lp), lr.T @ rr
    initial_distance = np.linalg.norm(rp-lp)
    def grasp_errors(tcp):
        (lp, lr), (rp, rr) = tcp
        delta = initial_relative_rotation.T @ lr.T @ rr
        from scipy.spatial.transform import Rotation
        return np.array([abs(lp[2]-rp[2]), np.linalg.norm(lr.T@(rp-lp)-initial_relative_position),
                         abs(np.linalg.norm(rp-lp)-initial_distance), Rotation.from_matrix(delta).magnitude()])
    midpoint_grasp = np.zeros(4)
    rows, fk_position, fk_rotation, midpoint_tilt = [], 0., 0., 0.
    for i, sample in enumerate(q):
        robot.set_q(baseline[i])
        raw_tilt = [tilt(r, sign) for (_, r), sign in zip(robot.tcp(), signs)]
        raw_grasp = grasp_errors(robot.tcp())
        robot.set_q(sample)
        tcp = robot.tcp()
        corrected = [tilt(r, sign) for (_, r), sign in zip(tcp, signs)]
        for side, (p, r) in zip(("left", "right"), tcp):
            cpp = np.array(scene["samples"][i][side + "_tcp"]).reshape(4, 4)
            fk_position = max(fk_position, float(np.linalg.norm(p-cpp[:3, 3])))
            fk_rotation = max(fk_rotation, float(np.linalg.norm(r-cpp[:3, :3])))
        rows.append([i*dt, sample[names.index("body_joint")],
                     *[sample[names.index("leg_joint"+str(j))] for j in (1, 2, 3)],
                     *corrected, *raw_tilt, *tcp[0][0], *tcp[1][0], *grasp_errors(tcp), *raw_grasp])
        if i:
            robot.set_q((q[i-1]+sample)/2)
            midpoint_tilt = max(midpoint_tilt, *[tilt(r, sign) for (_, r), sign in zip(robot.tcp(), signs)])
            midpoint_grasp = np.maximum(midpoint_grasp, grasp_errors(robot.tcp()))
    rows = np.array(rows)
    target_errors = {}
    robot.set_q(q[-1])
    for side, (p, r) in zip(("left", "right"), robot.tcp()):
        if side + "_target" in scene:
            goal = np.array(scene[side + "_target"]).reshape(4, 4)
            target_errors[side + "_position_m"] = float(np.linalg.norm(p-goal[:3, 3]))
            relative = r @ goal[:3, :3].T
            target_errors[side + "_orientation_rad"] = float(np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1)))
    result = dict(name=scene["name"], pipeline=scene["pipeline"], samples=len(q),
                  duration_sec=scene["duration_sec"], checked_midpoints=len(q)-1,
                  max_left_tilt_rad=float(rows[:, 5].max()), max_right_tilt_rad=float(rows[:, 6].max()),
                  max_midpoint_tilt_rad=float(midpoint_tilt),
                  uncompensated_max_tilt_rad=float(rows[:, 7:9].max()),
                  max_fk_position_difference_m=fk_position, max_fk_rotation_matrix_difference=fk_rotation,
                  max_joint_limit_excess_rad=position_excess, max_urdf_velocity_ratio=velocity_ratio,
                  waist_start_deg=float(np.rad2deg(rows[0, 1])), waist_end_deg=float(np.rad2deg(rows[-1, 1])),
                  leg_joint_excursion_rad=np.ptp(rows[:, 2:5], axis=0).tolist(),
                  arm_max_change_rad=float(np.abs(q[:, arm_indices]-q[0, arm_indices]).max()),
                  target_errors=target_errors)
    result["constraint_mode"] = bundle.get("constraint_mode", "tcp_level")
    result["initial_contact_distance_m"] = float(initial_distance)
    result["max_grasp_errors"] = dict(zip(("height_gap_m", "relative_position_m", "distance_change_m", "relative_rotation_rad"),
                                         np.maximum(rows[:, 15:19].max(axis=0), midpoint_grasp).tolist()))
    result["legacy_max_grasp_errors"] = dict(zip(("height_gap_m", "relative_position_m", "distance_change_m", "relative_rotation_rad"),
                                                rows[:, 19:23].max(axis=0).tolist()))
    checks = dict(finite=True, urdf_position_limits=position_excess < 1e-8,
                  urdf_velocity_limits=velocity_ratio <= 1.001,
                  tcp_level=max(rows[:, 5:7].max(), midpoint_tilt) <= bundle["tolerance_rad"],
                  independent_fk=fk_position < 1e-9 and fk_rotation < 1e-9)
    if scene["name"] in ("waist_175", "leg_waist_175"):
        checks["waist_0_to_175"] = abs(result["waist_start_deg"]) < 1e-6 and abs(result["waist_end_deg"]-175) < 1e-6
    if scene["name"] in ("leg_bend", "leg_waist_175"):
        checks["leg_motion_preserved"] = .299 <= result["leg_joint_excursion_rad"][0] <= .301
        checks["baseline_detects_tilt"] = result["uncompensated_max_tilt_rad"] > .15
        checks["arms_compensate"] = result["arm_max_change_rad"] > .02
        if rigid:
            del checks["baseline_detects_tilt"]
            checks["legacy_breaks_grasp"] = bool(rows[:, 20].max() > .005 or rows[:, 22].max() > .02)
    if rigid:
        checks["calibrated_suction_faces_down"] = signs == [-1, -1] and max(rows[:, 5:7].max(), midpoint_tilt) <= bundle["grasp_orientation_tolerance_rad"]
        maximum = np.maximum(rows[:, 15:19].max(axis=0), midpoint_grasp)
        checks["same_contact_height"] = maximum[0] <= bundle["grasp_position_tolerance_m"]
        checks["constant_relative_position"] = maximum[1] <= bundle["grasp_position_tolerance_m"]
        checks["constant_distance"] = maximum[2] <= bundle["grasp_position_tolerance_m"]
        checks["constant_relative_orientation"] = maximum[3] <= bundle["grasp_orientation_tolerance_rad"]
    if target_errors:
        checks["wbc_tcp_target"] = all(v < (.002 if k.endswith("_m") else .02) for k, v in target_errors.items())
    if rigid and scene["name"] == "wbc_movel":
        begin = .5*(rows[0, 9:12]+rows[0, 12:15])
        end = .5*(np.array(scene["left_target"]).reshape(4, 4)[:3, 3] +
                  np.array(scene["right_target"]).reshape(4, 4)[:3, 3])
        delta = end-begin
        center = .5*(rows[:, 9:12]+rows[:, 12:15])
        alpha = np.clip((center-begin)@delta/max(delta@delta, 1e-18), 0, 1)
        maximum = float(np.linalg.norm(center-begin-alpha[:, None]*delta, axis=1).max())
        result["object_line_max_error_m"] = maximum
        checks["object_center_follows_line"] = maximum <= .003
    result["checks"] = {k: bool(v) for k, v in checks.items()}
    result["passed"] = all(checks.values())
    header = ["time_sec", "waist_rad", "leg1_rad", "leg2_rad", "leg3_rad", "left_tilt_rad", "right_tilt_rad",
              "left_baseline_tilt_rad", "right_baseline_tilt_rad", "left_x", "left_y", "left_z", "right_x", "right_y", "right_z"]
    header += [prefix + metric for prefix in ("grasp_", "baseline_")
               for metric in ("height_gap_m", "relative_position_m", "distance_change_m", "relative_rotation_rad")]
    with (output / (scene["name"] + ".csv")).open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
    return result, rows, q, baseline


def plot_results(results, output, tolerance):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(results), 3, figsize=(15, 3.1*len(results)), squeeze=False)
    for ax, (result, rows, _, _) in zip(axes, results):
        t = rows[:, 0]
        ax[0].plot(t, np.rad2deg(rows[:, 5:7]), label=["left compensated", "right compensated"])
        ax[0].plot(t, np.rad2deg(rows[:, 7:9]), "--", alpha=.6, label=["left arms frozen", "right arms frozen"])
        ax[0].axhline(np.rad2deg(tolerance), color="red", linestyle=":", label="tolerance")
        ax[0].set(title=result["name"], ylabel="TCP tilt (deg)")
        if result.get("constraint_mode") == "rigid_grasp":
            ax[0].clear()
            ax[0].plot(t, rows[:, 15:18]*1000, label=["height gap", "relative position", "distance drift"])
            ax[0].plot(t, rows[:, 20]*1000, "--", label="old level-only relative drift")
            ax[0].set(title=result["name"], ylabel="Grasp error (mm)")
        ax[1].plot(t, np.rad2deg(rows[:, 1:5]), label=["waist", "leg J1", "leg J2", "leg J3"])
        ax[1].set(ylabel="Joint angle (deg)")
        ax[2].plot(t, rows[:, [11, 14]], label=["left TCP", "right TCP"])
        ax[2].set(ylabel="TCP height (m)")
        for a in ax:
            a.set_xlabel("Trajectory time (s)")
            a.grid(alpha=.25)
            a.legend(fontsize=7)
    fig.suptitle("Production C++ trajectories / Independent MuJoCo FK\nFixed base; kinematics only; no collision or balance certification")
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(output / "validation.png", dpi=150)
    plt.close(fig)


def camera():
    import mujoco
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [.05, 0, 1.05]
    cam.distance, cam.azimuth, cam.elevation = 4.0, 135, -16
    return cam


def render_video(robot, bundle, results, output, speed):
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw, ImageFont
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_path, 19) if Path(font_path).exists() else ImageFont.load_default()
    cam = camera()
    fps = 30
    # Software EGL on WSL can spend most render time on multisampling and
    # shadows. Keep the original meshes and lighting; these effects do not
    # participate in the independent kinematic checks.
    robot.model.vis.quality.offsamples = 0
    with mujoco.Renderer(robot.model, height=560, width=640) as renderer, imageio.get_writer(
            str(output / "comparison.mp4"), fps=fps, codec="libx264", quality=8, macro_block_size=16) as video:
        for result, rows, q, baseline in results:
            frames = max(2, int(np.ceil(result["duration_sec"] / speed * fps)))
            rigid = bundle.get("constraint_mode") == "rigid_grasp"
            peak = int(np.argmax(rows[:, 20] if rigid else rows[:, 7:9].max(axis=1)))
            saved = False
            for frame in range(frames+1):
                i = min(len(q)-1, round(frame/frames*(len(q)-1)))
                images = []
                for value in (q[i], baseline[i]):
                    robot.set_q(value)
                    renderer.update_scene(robot.data, camera=cam)
                    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
                    images.append(renderer.render().copy())
                canvas = Image.new("RGB", (1280, 720), (15, 23, 32))
                canvas.paste(Image.fromarray(images[0]), (0, 48))
                canvas.paste(Image.fromarray(images[1]), (640, 48))
                draw = ImageDraw.Draw(canvas)
                draw.text((18, 12), "RIGID TWO-SUCTION BOX GRASP" if rigid else "CURRENT C++ TCP COMPENSATION", font=font, fill=(82, 214, 242))
                draw.text((660, 12), "LEGACY: keep_tcp_level" if rigid else "REFERENCE: ARMS FROZEN", font=font, fill=(255, 183, 96))
                for y, text in enumerate([
                    f"{result['name']}   t={rows[i, 0]:.2f}s   waist={np.rad2deg(rows[i, 1]):.1f} deg   playback={speed:g}x",
                    (f"Gap: {rows[i, 15]*1000:.3f} mm  Relative drift: {rows[i, 16]*1000:.3f} mm  |  Old: {rows[i, 19]*1000:.2f} / {rows[i, 20]*1000:.2f} mm" if rigid else
                     f"TCP tilt L/R: {np.rad2deg(rows[i, 5]):.4f} / {np.rad2deg(rows[i, 6]):.4f} deg     |     frozen: {np.rad2deg(rows[i, 7]):.2f} / {np.rad2deg(rows[i, 8]):.2f} deg"),
                    ("Suction TCP +Z faces down | Cyan: left cup | Orange: right cup" if rigid else
                     "Fixed-base kinematic validation | Cyan: left TCP plane | Orange: right TCP plane"),
                    ("Kinematic simulation | Schematic cups and box | No vacuum or load dynamics" if rigid else
                     "No contact dynamics, collision clearance or physical tracking acceptance"),
                ]):
                    draw.text((18, 612+25*y), text, font=font, fill=(230, 235, 240))
                video.append_data(np.asarray(canvas))
                if not saved and i >= peak:
                    canvas.save(output / (result["name"] + "_comparison.png"))
                    saved = True
            print("RENDERED", result["name"], flush=True)
    from video_clips import make_clips
    make_clips([r[0] for r in results], output, speed, fps)


def show_viewer(robot, results, speed):
    import mujoco.viewer
    state = dict(scene=0, paused=False, baseline=False, position=0.)
    def key(code):
        if code == 32:
            state["paused"] = not state["paused"]
        elif code in (ord("N"), ord("n")):
            state["scene"] = (state["scene"]+1) % len(results)
            state["position"] = 0.
        elif code in (ord("C"), ord("c")):
            state["baseline"] = not state["baseline"]
        elif code in (ord("R"), ord("r")):
            state["position"] = 0.
        elif code in (262, 263):
            state["position"] = max(0., state["position"]+(1 if code == 262 else -1))
    print("Viewer: Space pause, N next, C toggle comparison, R restart, arrows seek; mouse rotates/zooms.")
    with mujoco.viewer.launch_passive(robot.model, robot.data, key_callback=key) as viewer:
        with viewer.lock():
            cam = camera()
            viewer.cam.lookat[:] = cam.lookat
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = cam.distance, cam.azimuth, cam.elevation
        previous = time.monotonic()
        while viewer.is_running():
            now = time.monotonic()
            if not state["paused"]:
                state["position"] += (now-previous)*speed
            previous = now
            result, rows, q, baseline = results[state["scene"]]
            if state["position"] > result["duration_sec"]+1.:
                state["scene"] = (state["scene"]+1) % len(results)
                state["position"] = 0.
                continue
            i = min(len(q)-1, int(state["position"]/.004))
            with viewer.lock():
                robot.set_q(baseline[i] if state["baseline"] else q[i])
                robot.data.time = rows[i, 0]
            cols = (7, 8) if state["baseline"] else (5, 6)
            rigid = result.get("constraint_mode") == "rigid_grasp"
            mode = (("Legacy: keep_tcp_level" if state["baseline"] else "Rigid box grasp") if rigid else
                    ("Arms frozen" if state["baseline"] else "TCP compensation"))
            metric_cols = (19, 20) if state["baseline"] else (15, 16)
            metric = (f"{rows[i, metric_cols[0]]*1000:.3f} / {rows[i, metric_cols[1]]*1000:.3f} mm" if rigid else
                      f"{np.rad2deg(rows[i, cols[0]]):.4f} / {np.rad2deg(rows[i, cols[1]]):.4f} deg")
            viewer.set_texts((mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                "Scene\nMode\nTime / waist\n"+("Height / relative drift" if rigid else "Left / right tilt")+"\nControls",
                f"{result['name']}\n{mode} (kinematic)\n"
                f"{rows[i, 0]:.2f}s / {np.rad2deg(rows[i, 1]):.1f} deg\n"
                f"{metric}\n"
                "Space pause | N next | C compare | R restart"))
            viewer.sync()
            time.sleep(.01)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", default="all", choices=("all", "leg_bend", "waist_175", "leg_waist_175", "wbc_movep", "wbc_movel"))
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--speed", type=float, default=2.)
    args = parser.parse_args()
    if not np.isfinite(args.speed) or args.speed <= 0:
        parser.error("speed must be positive and finite")
    if not args.viewer:
        os.environ.setdefault("MUJOCO_GL", "egl")
    from model import Robot, compile_model
    bundle = json.loads(args.trajectory.read_text())
    if bundle.get("schema_version") != 1 or bundle.get("mode") != "fixed_base_kinematic_validation":
        raise ValueError("unsupported bundle")
    manifest_file = args.trajectory.parent / "source_manifest.json"
    if not manifest_file.exists():
        raise ValueError("missing source_manifest.json; use run_mujoco_validation.sh to export")
    source_manifest = json.loads(manifest_file.read_text())
    repo = Path(bundle["urdf"]).parents[3]
    for relative, expected in source_manifest.items():
        if hashlib.sha256((repo / relative).read_bytes()).hexdigest() != expected:
            raise ValueError("export is stale; rebuild after source change: " + relative)
    args.output.mkdir(parents=True, exist_ok=True)
    model, manifest = compile_model(bundle, args.output)
    robot = Robot(model, bundle, manifest)
    scenes = [s for s in bundle["scenarios"] if args.case in ("all", s["name"])]
    if not scenes:
        raise ValueError("no matching scenario")
    results = [check_scene(robot, s, bundle, manifest, args.output) for s in scenes]
    negative_passed = all(bundle["negative_checks"].get(k) is True for k in (
        "waist_180_rejected", "uncompensated_tilt_rejected"))
    report = dict(passed=all(r[0]["passed"] for r in results) and negative_passed, mode=bundle["mode"],
                  trajectory_sha256=hashlib.sha256(args.trajectory.read_bytes()).hexdigest(),
                  source_manifest=str(manifest_file.resolve()),
                  validation_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                     for p in sorted(Path(__file__).parent.glob("*.py"))},
                  scene_sha256=hashlib.sha256((args.output / "scene.xml").read_bytes()).hexdigest(),
                  tolerance_rad=bundle["tolerance_rad"], tool_profile=bundle["tool_profile"],
                  constraint_mode=bundle.get("constraint_mode", "tcp_level"),
                  negative_checks=bundle["negative_checks"], model=manifest,
                  scenarios=[r[0] for r in results])
    (args.output / "report.json").write_text(json.dumps(report, indent=2)+"\n")
    plot_results(results, args.output, bundle["tolerance_rad"])
    for result, *_ in results:
        print(("PASS " if result["passed"] else "FAIL ") + result["name"], result["checks"], flush=True)
    if not report["passed"]:
        raise SystemExit("MuJoCo verification failed; inspect report.json")
    if args.video:
        render_video(robot, bundle, results, args.output, args.speed)
    if args.viewer:
        show_viewer(robot, results, args.speed)


if __name__ == "__main__":
    main()
