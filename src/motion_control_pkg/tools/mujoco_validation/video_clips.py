"""Extract exact scene frame ranges from the rendered comparison video."""
import argparse
import json
from pathlib import Path
import subprocess
import numpy as np
import imageio_ffmpeg


def make_clips(scenes, output, speed, fps=30):
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    video = output / "comparison.mp4"
    count, _ = imageio_ffmpeg.count_frames_and_secs(str(video))
    expected = [max(2, int(np.ceil(s["duration_sec"]/speed*fps)))+1 for s in scenes]
    if count != sum(expected):
        raise ValueError(f"video is incomplete: {count} frames, expected {sum(expected)}")
    index, begin = [], 0
    for scene, frames in zip(scenes, expected):
        clip = output / (scene["name"] + ".mp4")
        subprocess.run([ffmpeg, "-y", "-v", "error", "-i", str(video), "-vf",
                        f"trim=start_frame={begin}:end_frame={begin+frames},setpts=N/({fps}*TB)",
                        "-r", str(fps), "-fps_mode", "cfr", "-an", "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p",
                        "-movflags", "+faststart", str(clip)], check=True)
        actual, duration = imageio_ffmpeg.count_frames_and_secs(str(clip))
        if actual != frames:
            raise ValueError("incomplete clip: " + str(clip))
        index.append(dict(name=scene["name"], file=clip.name, frames=frames,
                          video_duration_sec=duration, trajectory_duration_sec=scene["duration_sec"],
                          playback_speed=speed, begin_frame=begin, end_frame=begin+frames))
        begin += frames
    (output / "videos.json").write_text(json.dumps(index, indent=2)+"\n")
    print("Verified comparison.mp4 and", len(index), "scene clips", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speed", type=float, default=2.)
    args = parser.parse_args()
    if not np.isfinite(args.speed) or args.speed <= 0:
        parser.error("speed must be finite and positive")
    bundle = json.loads((args.output / "trajectories.json").read_text())
    make_clips(bundle["scenarios"], args.output, args.speed)
