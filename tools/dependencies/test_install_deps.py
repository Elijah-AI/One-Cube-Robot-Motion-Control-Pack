"""Offline regression: never invoke apt, sudo, rosdep or pip."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import install_deps as deps


class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "src").mkdir()
        self.manifest = self.root / "dependencies.json"
        self.config = {"version": 1, "apt": ["ros-{ros_distro}-pcl-ros"],
                       "pip": [], "rosdep": [], "skip_rosdep": {}}
        self.write()
        self.base = ["--workspace", str(self.root), "--manifest", str(self.manifest)]

    def write(self):
        self.manifest.write_text(json.dumps(self.config))

    def run_cli(self, *args):
        output = io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            code = deps.main([*self.base, *args])
        return code, output.getvalue()

    def test_default_and_dry_run_never_spawn_or_write(self):
        before = sorted(self.root.rglob("*"))
        with patch.object(deps.subprocess, "Popen", side_effect=AssertionError("spawn")):
            for args in [(), ("install", "--dry-run")]:
                code, output = self.run_cli(*args)
                self.assertEqual(code, 0, output)
                self.assertIn("ros-humble-pcl-ros", output)
                self.assertIn("rosdep check", output)
        self.assertEqual(before, sorted(self.root.rglob("*")))

    def test_add_is_persistent_deduplicated_and_does_not_install(self):
        with patch.object(deps.subprocess, "Popen", side_effect=AssertionError("spawn")):
            self.assertEqual(self.run_cli("add", "apt", "libpcl-dev", "libpcl-dev")[0], 0)
        self.assertEqual(deps.load_config(self.manifest)["apt"].count("libpcl-dev"), 1)

    def test_flat_repository_ignores_build_and_install(self):
        (self.root / "src").rmdir()
        for name in ["com_interfaces", "task_manager", "build", "install", "log", ".deps_venv"]:
            (self.root / name).mkdir()
            (self.root / name / "package.xml").write_text("<package/>")
        with patch.object(deps.subprocess, "Popen", side_effect=AssertionError("spawn")):
            code, output = self.run_cli()
        self.assertEqual(code, 0, output)
        self.assertIn(f"--from-paths {self.root / 'com_interfaces'} {self.root / 'task_manager'} --ignore-src", output)
        self.assertNotIn(str(self.root / "build"), output)
        self.assertNotIn(str(self.root / "install"), output)

    def test_missing_sources_fails_before_commands(self):
        (self.root / "src").rmdir()
        with patch.object(deps.subprocess, "Popen", side_effect=AssertionError("spawn")):
            code, output = self.run_cli()
        self.assertEqual(code, 1, output)
        self.assertNotIn("$ ", output)

    def test_invalid_config_fails_before_commands(self):
        self.config["apt"] = ["--allow-unauthenticated"]
        self.write()
        with patch.object(deps.subprocess, "Popen", side_effect=AssertionError("spawn")):
            self.assertEqual(self.run_cli("install")[0], 1)

    def test_failed_install_stops_and_logs(self):
        calls = []

        def fail(runner, command):
            calls.append(command)
            raise RuntimeError("simulated apt failure")

        with patch.object(deps.os, "geteuid", return_value=1000), \
                patch.object(deps.shutil, "which", return_value="/usr/bin/mock"), \
                patch.object(Path, "exists", return_value=True), \
                patch.object(deps.Runner, "run", fail):
            code, output = self.run_cli("install")
        self.assertEqual(code, 1, output)
        self.assertEqual(len(calls), 1)
        log = next((self.root / "log/dependencies").glob("*.log"))
        self.assertIn("simulated apt failure", log.read_text())

    def test_pip_plan_uses_venv_and_resolves_wheel(self):
        self.config["pip"] = ["./vendor.whl", "example>=1"]
        self.write()
        code, output = self.run_cli("plan")
        self.assertEqual(code, 0, output)
        self.assertIn("--system-site-packages", output)
        self.assertIn(str(self.root / "vendor.whl"), output)
        self.assertIn(".deps_venv/bin/python -m pip check", output)

    def test_no_update_and_extra_rosdep(self):
        self.config["rosdep"] = ["diagnostic_updater"]
        self.write()
        code, output = self.run_cli("--no-update", "plan")
        self.assertEqual(code, 0, output)
        self.assertNotIn("apt-get update", output)
        self.assertNotIn("rosdep update", output)
        self.assertIn("rosdep check diagnostic_updater", output)


if __name__ == "__main__":
    unittest.main()
