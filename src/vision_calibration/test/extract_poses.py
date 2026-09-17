#!/usr/bin/env python3
import re, glob, os

data_dir = "/home/neardi/ros2_ws/src/vision_calibration/data_cali/20260902_085806"
out_yaml = os.path.join(data_dir, "poses.yaml")
out_csv  = os.path.join(data_dir, "poses.csv")

files = sorted(glob.glob(os.path.join(data_dir, "pose_*.yaml")))

def extract(text, key):
    m = re.search(rf"{key}\s*:\s*\[(.*?)\]", text)
    if not m:
        raise ValueError(f"{key} not found in text")
    return [float(x.strip()) for x in m.group(1).split(",") if x.strip()]

trans, quats = [], []
with open(out_csv, "w") as fcsv:
    fcsv.write("index,tx,ty,tz,qx,qy,qz,qw\n")
    for i, fp in enumerate(files, 1):
        with open(fp) as f:
            txt = f.read()
        t = extract(txt, "tcp_translation")
        if i == 1:
            print(f"[debug] first file: {fp}")
        q = extract(txt, "tcp_quaternion_xyzw")
        trans.append(t)
        quats.append(q)
        fcsv.write(f"{i},{t[0]},{t[1]},{t[2]},{q[0]},{q[1]},{q[2]},{q[3]}\n")

with open(out_yaml, "w") as f:
    f.write("%YAML:1.0\n")
    f.write("tcp_poses:\n")
    for t, q in zip(trans, quats):
        f.write(f"   - [{t[0]}, {t[1]}, {t[2]}, {q[0]}, {q[1]}, {q[2]}, {q[3]}]\n")

print(f"OK: {len(trans)} poses")
print(f"  -> {out_yaml}")
print(f"  -> {out_csv}")