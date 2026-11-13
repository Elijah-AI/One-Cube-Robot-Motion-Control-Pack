#!/usr/bin/env python3
"""Fault injection against the independent validator, using an exported fixture."""
import copy
import json
from pathlib import Path
import sys
import tempfile
from model import Robot, compile_model
from validate import check_scene


def main():
    bundle = json.loads(Path(sys.argv[1]).read_text())
    scene = next(s for s in bundle["scenarios"] if s["name"] == "leg_waist_175")
    with tempfile.TemporaryDirectory(prefix="mujoco_regression_") as tmp:
        output = Path(tmp)
        model, manifest = compile_model(bundle, output)
        robot = Robot(model, bundle, manifest)
        result, *_ = check_scene(robot, scene, bundle, manifest, output)
        assert result["passed"], "positive control failed"
        arm = bundle["joint_names"].index("Joint1_L")
        bad = copy.deepcopy(scene)
        bad["samples"][len(bad["samples"])//2]["q"][arm] = manifest["limits"]["Joint1_L"]["upper"] + .1
        result, *_ = check_scene(robot, bad, bundle, manifest, output)
        assert not result["passed"] and not result["checks"]["urdf_position_limits"]
        print("PASS rejects injected joint-limit violation")
        altered = copy.deepcopy(bundle)
        a, b = altered["joint_names"].index("Joint2_L"), altered["joint_names"].index("Joint2_R")
        altered["joint_names"][a], altered["joint_names"][b] = altered["joint_names"][b], altered["joint_names"][a]
        result, *_ = check_scene(Robot(model, altered, manifest), scene, altered, manifest, output)
        assert not result["passed"] and not result["checks"]["independent_fk"]
        print("PASS detects swapped left/right joint mapping")
        altered = copy.deepcopy(bundle)
        altered["urdf_xml"] += "\n"
        try:
            compile_model(altered, output)
        except ValueError as exc:
            assert "URDF changed" in str(exc)
        else:
            raise AssertionError("stale URDF accepted")
        print("PASS rejects stale URDF snapshot")
        altered = copy.deepcopy(bundle)
        altered["dt_sec"] = .005
        try:
            check_scene(robot, scene, altered, manifest, output)
        except ValueError as exc:
            assert "clock mismatch" in str(exc)
        else:
            raise AssertionError("wrong trajectory clock accepted")
        print("PASS rejects changed trajectory clock")


if __name__ == "__main__":
    main()
