"""Export once and pin source, executable and URDF identity for reproducibility."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def hashes(repo):
    paths = list((repo / "src/motion_control_pkg").rglob("*.cpp"))
    paths += list((repo / "src/motion_control_pkg/include").rglob("*.hpp"))
    paths += list((repo / "src/com_interfaces").rglob("*.action"))
    paths += list((repo / "src/com_interfaces/include").rglob("*.hpp"))
    paths += list((repo / "src/onecube_robot_description/urdf").glob("*.urdf"))
    paths += [repo / "build/motion_control_pkg/wbc_mujoco_export"]
    paths += list((repo / "src/motion_control_pkg/tools/mujoco_validation").glob("*.json"))
    return {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(paths))}


if __name__ == "__main__":
    repo, output = map(lambda x: Path(x).resolve(), sys.argv[1:3])
    before = hashes(repo)
    temporary = output / "trajectories.pending.json"
    with (output / "planner.log").open("w") as log:
        subprocess.run([str(repo / "build/motion_control_pkg/wbc_mujoco_export"), str(repo), str(temporary), *sys.argv[3:]],
                       stdout=log, stderr=subprocess.STDOUT, timeout=300, check=True)
    if before != hashes(repo):
        raise RuntimeError("source changed during export; rebuild and retry")
    temporary.replace(output / "trajectories.json")
    (output / "source_manifest.json").write_text(json.dumps(before, indent=2)+"\n")
    print("Exported production trajectories. Log:", output / "planner.log", flush=True)
