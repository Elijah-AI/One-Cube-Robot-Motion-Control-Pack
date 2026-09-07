#!/usr/bin/env python3
"""ROS 2 dependency manager. Only the install subcommand changes the system."""
import argparse
import datetime
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
KINDS = ("apt", "pip", "rosdep")


def load_config(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("version") != 1:
        raise ValueError("清单 version 必须为 1")
    for kind in KINDS:
        values = data.get(kind)
        if not isinstance(values, list):
            raise ValueError(f"{kind} 必须为字符串列表")
        for value in values:
            validate(kind, value)
    skips = data.get("skip_rosdep", {})
    if not isinstance(skips, dict):
        raise ValueError("skip_rosdep 必须为 key: 原因 字典")
    for key, reason in skips.items():
        validate("rosdep", key)
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError(f"跳过 {key} 必须注明原因")
    notes = data.get("manual", [])
    if not isinstance(notes, list) or not all(isinstance(x, str) for x in notes):
        raise ValueError("manual 必须为字符串列表")
    return data


def validate(kind, value):
    if not isinstance(value, str) or not value.strip() or value.startswith("-") or "\n" in value:
        raise ValueError(f"无效的 {kind} 依赖: {value!r}")
    # Permit the ROS distribution placeholder only for apt packages.
    candidate = value.replace("{ros_distro}", "humble") if kind == "apt" else value
    if kind != "pip" and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+:/=~-]*", candidate):
        raise ValueError(f"无效的 {kind} 名称: {value!r}")


def save_config(path, data):
    # A failed write must not truncate the dependency manifest.
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                     prefix=".deps-", delete=False) as stream:
        temp = Path(stream.name)
        json.dump(data, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    try:
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


class Runner:
    def __init__(self, apply, log=None):
        self.apply = apply
        self.log = log

    def say(self, message):
        print(message, flush=True)
        if self.log:
            self.log.write(message + "\n")
            self.log.flush()

    def run(self, command):
        self.say("$ " + shlex.join([str(x) for x in command]))
        if not self.apply:
            return
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, bufsize=1) as process:
            for line in process.stdout:
                self.say(line.rstrip("\n"))
            code = process.wait()
        if code:
            raise RuntimeError(f"命令失败，退出码 {code}；修复后可重新运行 install")


def execute(args, config, runner):
    workspace = args.workspace.resolve()
    if (workspace / "src").is_dir():
        source_paths = [workspace / "src"]
    else:
        source_paths = sorted(p.parent for p in workspace.glob("*/package.xml")
                              if p.parent.name not in {"build", "install", "log", ".deps_venv"})
        if not source_paths:
            raise ValueError(f"找不到工作区源码目录或根目录下的 ROS 包: {workspace}")
    distro = args.ros_distro
    if not re.fullmatch(r"[a-z][a-z0-9_]*", distro):
        raise ValueError("无效 ROS 发行版")
    if runner.apply:
        if os.geteuid() == 0:
            raise ValueError("请以普通用户运行程序；仅 apt 和 rosdep init 内部使用 sudo")
        if not Path(f"/opt/ros/{distro}/setup.bash").exists():
            raise ValueError(f"请先安装 ROS 2 {distro} 并配置其 apt 源")
        if not shutil.which("apt-get") or not shutil.which("sudo"):
            raise ValueError("需要支持 apt-get 和 sudo 的 Ubuntu/Debian 环境")
    runner.say(f"工作区: {workspace}\nROS: {distro}\n清单: {args.manifest.resolve()}")
    for key, reason in config.get("skip_rosdep", {}).items():
        runner.say(f"显式跳过 rosdep key {key}: {reason}")
    apt = list(dict.fromkeys([
        "python3-rosdep", "python3-colcon-common-extensions", "python3-venv",
        *[x.replace("{ros_distro}", distro) for x in config["apt"]],
    ]))
    if not args.no_update:
        runner.run(["sudo", "apt-get", "update"])
    runner.run(["sudo", "apt-get", "install", "-y", "--no-remove", *apt])
    if not Path("/etc/ros/rosdep/sources.list.d/20-default.list").exists():
        runner.run(["/usr/bin/sudo", "/usr/bin/rosdep", "init"])
    if not args.no_update:
        runner.run(["/usr/bin/rosdep", "update", "--rosdistro", distro])
    rosdep_args = ["--from-paths", *[str(path) for path in source_paths], "--ignore-src", "--rosdistro", distro]
    if config.get("skip_rosdep"):
        rosdep_args += ["--skip-keys", " ".join(config["skip_rosdep"])]
    runner.run(["/usr/bin/rosdep", "install", *rosdep_args, "-y"])
    if config["rosdep"]:
        runner.run(["/usr/bin/rosdep", "install", *config["rosdep"], "--rosdistro", distro, "-y"])
    if config["pip"]:
        venv = workspace / ".deps_venv"
        if not (venv / "bin/python").exists():
            runner.run(["/usr/bin/python3", "-m", "venv", "--system-site-packages", str(venv)])
        requirements = []
        for item in config["pip"]:
            if item.endswith(".whl") and "://" not in item:
                wheel = Path(item).expanduser()
                if not wheel.is_absolute():
                    wheel = args.manifest.resolve().parent / wheel
                if runner.apply and not wheel.is_file():
                    raise ValueError(f"找不到 wheel: {wheel}")
                item = str(wheel)
            requirements.append(item)
        runner.run([str(venv / "bin/python"), "-m", "pip", "install", *requirements])
        runner.run([str(venv / "bin/python"), "-m", "pip", "check"])
        runner.say(f"使用 Python 依赖前执行: source {shlex.quote(str(venv / 'bin/activate'))}")
    runner.run(["/usr/bin/rosdep", "check", *rosdep_args])
    if config["rosdep"]:
        runner.run(["/usr/bin/rosdep", "check", *config["rosdep"], "--rosdistro", distro])
    runner.say("公共依赖安装与 rosdep 检查完成。" if runner.apply else "仅预览，未执行以上命令。")
    for note in config.get("manual", []):
        runner.say("[需另行核对的厂家/硬件依赖] " + note)
    runner.say("以上结果不代表工作区编译、SDK ABI 或机器人运行验证通过。")


def main(argv=None):
    parser = argparse.ArgumentParser(description="批量管理 ROS 2 工作区外部依赖（默认仅预览）")
    parser.add_argument("--workspace", type=Path, default=HERE.parent.parent)
    parser.add_argument("--manifest", type=Path, default=HERE / "dependencies.json")
    parser.add_argument("--ros-distro", default=os.environ.get("ROS_DISTRO", "humble"))
    parser.add_argument("--no-update", action="store_true", help="复用现有 apt / rosdep 索引")
    sub = parser.add_subparsers(dest="action")
    sub.add_parser("plan", help="打印安装计划，不联网、不安装")
    install = sub.add_parser("install", help="实际安装；可以重复运行")
    install.add_argument("--dry-run", action="store_true", help="仅预览")
    sub.add_parser("list", help="显示依赖清单")
    add = sub.add_parser("add", help="追加依赖到清单，不立即安装")
    add.add_argument("kind", choices=KINDS)
    add.add_argument("packages", nargs="+")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.manifest)
        if args.action == "add":
            for package in args.packages:
                validate(args.kind, package)
                if package not in config[args.kind]:
                    config[args.kind].append(package)
            save_config(args.manifest, config)
            print(f"已更新 {args.manifest}；执行 install 才会安装")
            return 0
        if args.action == "list":
            print(json.dumps(config, indent=2, ensure_ascii=False))
            return 0
        apply = args.action == "install" and not args.dry_run
        if apply:
            directory = args.workspace.resolve() / "log" / "dependencies"
            directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            with (directory / f"install_{stamp}.log").open("w", encoding="utf-8") as log:
                runner = Runner(True, log)
                runner.say(f"日志: {log.name}")
                try:
                    execute(args, config, runner)
                except Exception as error:
                    runner.say(f"失败: {error}")
                    return 1
        else:
            execute(args, config, Runner(False))
        return 0
    except (OSError, ValueError, RuntimeError) as error:
        print(f"失败: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n已中断；可重新执行 install。", file=sys.stderr)
        sys.exit(130)
