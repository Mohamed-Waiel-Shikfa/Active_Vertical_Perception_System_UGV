"""Argument/lifecycle contract tests; fake Docker is NOT runtime evidence."""
import json
import os
import shlex
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("bash"), "Bash is required")
class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="greenhouse launcher ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.project = self.root / "project with spaces"
        (self.project / "config").mkdir(parents=True)
        for name in ("run.sh", "config/greenhouse.json", "config/robot.example.yaml", "config/actuator.json"):
            shutil.copyfile(ROOT / name, self.project / name)
        self.setup = self.project / "robot setup"
        self.setup.mkdir()
        (self.setup / "robot.yaml").write_text("preserve: my existing configuration\n")
        self.bin = self.root / "bin"
        self.bin.mkdir()
        fake = self.bin / "docker"
        fake.write_text("""#!/usr/bin/env python3
import json, os, sys
with open(os.environ['DOCKER_TEST_LOG'], 'a') as f:
    f.write(json.dumps(sys.argv[1:]) + '\\n')
if sys.argv[1] == 'info':
    if os.environ.get('DOCKER_TEST_FAIL') == 'daemon': sys.exit(1)
    print('linux')
elif sys.argv[1:3] == ['image', 'inspect']:
    if os.environ.get('DOCKER_TEST_FAIL') == 'image': sys.exit(1)
    if '--format' in sys.argv: print(os.environ.get('DOCKER_TEST_SOURCE', ''))
elif sys.argv[1:3] == ['container', 'inspect']:
    sys.exit(0 if os.environ.get('DOCKER_TEST_FAIL') == 'conflict' else 1)
elif sys.argv[1] == 'run':
    sys.exit(int(os.environ.get('DOCKER_TEST_EXIT', '0')))
""")
        fake.chmod(0o755)
        self.log = self.root / "docker-calls.jsonl"
        self.env = dict(os.environ, PATH=str(self.bin) + os.pathsep + os.environ["PATH"],
                        DOCKER_TEST_LOG=str(self.log), HOME=str(self.root))
        self.env["DOCKER_TEST_SOURCE"] = self.run_launcher("--source-fingerprint").stdout.strip()

    def run_launcher(self, *args):
        return subprocess.run(["bash", str(self.project / "run.sh"), *args],
                              cwd=self.root, env=self.env, text=True, capture_output=True)

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def test_bash_syntax_and_lf(self):
        for path in (ROOT / "run.sh", ROOT / "docker" / "start.sh", ROOT / "tools" / "teleop.sh"):
            subprocess.run(["bash", "-n", str(path)], check=True)
            self.assertNotIn(b"\r", path.read_bytes())

    def test_help_and_invalid_options_do_not_call_docker(self):
        self.assertEqual(self.run_launcher("--help").returncode, 0)
        for args in (("--unknown",), ("--name",), ("--name", "bad name"), ("--headless", "--rviz"),
                     ("--no-build", "--build-only"), ("--", "world:=warehouse")):
            self.assertNotEqual(self.run_launcher(*args).returncode, 0)
        self.assertEqual(self.calls(), [])

    def test_dry_run_with_spaces_is_side_effect_free(self):
        result = self.run_launcher("--dry-run", "--headless", "--software-rendering")
        self.assertEqual(result.returncode, 0, result.stderr)
        command = shlex.split(result.stdout.splitlines()[-1])
        self.assertIn(f"type=bind,source={self.project}/runtime/clearpath,target=/etc/clearpath", command)
        self.assertIn("SETUP_PATH=/etc/clearpath", command)
        self.assertIn("LIBGL_ALWAYS_SOFTWARE=1", command)
        self.assertFalse((self.project / "runtime").exists())
        self.assertEqual(self.calls(), [])

    def test_existing_configuration_and_exit_code_are_preserved(self):
        before = (self.setup / "robot.yaml").read_bytes()
        self.env["DOCKER_TEST_EXIT"] = "37"
        result = self.run_launcher("--headless", "--no-build", "--setup-dir", str(self.setup),
                                   "--name", "isolated-test", "--", "--debug")
        self.assertEqual(result.returncode, 37, result.stderr)
        self.assertEqual((self.setup / "robot.yaml").read_bytes(), before)
        run = self.calls()[-1]
        self.assertEqual(run[0], "run")
        self.assertIn("--debug", run)
        self.assertIn("isolated-test", run)
        self.assertNotIn("--privileged", run)
        self.assertNotIn("--network", run)

    def test_ros_flags_also_work_without_the_separator(self):
        result = self.run_launcher("--headless", "--no-build", "--setup-dir", str(self.setup), "--show-args")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls()[-1][-1], "--show-args")

    def test_daemon_and_name_conflicts_fail_without_deletion(self):
        for failure in ("daemon", "conflict"):
            self.env["DOCKER_TEST_FAIL"] = failure
            result = self.run_launcher("--headless", "--no-build", "--setup-dir", str(self.setup))
            self.assertNotEqual(result.returncode, 0)
            self.assertNotIn("run", [call[0] for call in self.calls()])
            self.assertFalse(any("rm" in call or "prune" in call for call in self.calls()))

    def test_no_build_refuses_an_image_built_from_other_source(self):
        fingerprint = self.env["DOCKER_TEST_SOURCE"]
        self.assertRegex(fingerprint, "^[0-9a-f]{16}$")
        for label in ("", "0123456789abcdef"):
            self.env["DOCKER_TEST_SOURCE"] = label
            result = self.run_launcher("--headless", "--no-build", "--setup-dir", str(self.setup))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("built from different source", result.stderr)
            self.assertIn("without --no-build", result.stderr)
            self.assertNotIn("run", [call[0] for call in self.calls()])
        result = self.run_launcher("--headless", "--no-build", "--allow-stale-image", "--setup-dir", str(self.setup))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("warning: starting stale image", result.stderr)
        self.assertEqual(self.calls()[-1][0], "run")
        self.assertNotEqual(self.run_launcher("--allow-stale-image", "--dry-run").returncode, 0)

    def test_build_labels_the_image_with_the_source_fingerprint(self):
        self.env["DOCKER_TEST_EXIT"] = "0"
        result = self.run_launcher("--build-only")
        self.assertEqual(result.returncode, 0, result.stderr)
        build = self.calls()[-1]
        self.assertEqual(build[0], "build")
        self.assertIn("GREENHOUSE_SOURCE=" + self.env["DOCKER_TEST_SOURCE"], build)
        before = self.env["DOCKER_TEST_SOURCE"]
        (self.project / "config" / "robot.example.yaml").write_text("changed build input\n")
        self.assertNotEqual(self.run_launcher("--source-fingerprint").stdout.strip(), before)
        (self.project / "config" / "actuator.json").write_text("{}\n")  # runtime-mounted, not a build input
        changed = self.run_launcher("--source-fingerprint").stdout.strip()
        (self.project / "config" / "actuator.json").write_text('{"x": 1}\n')
        self.assertEqual(self.run_launcher("--source-fingerprint").stdout.strip(), changed)

    def test_script_is_mounted_and_announced_to_the_container(self):
        scripts = self.root / "my scripts"
        scripts.mkdir()
        (scripts / "survey.py").write_text("print('hi')\n")
        result = self.run_launcher("--dry-run", "--headless", "--script", str(scripts / "survey.py"))
        self.assertEqual(result.returncode, 0, result.stderr)
        command = shlex.split(result.stdout.splitlines()[-1])
        self.assertIn(f"type=bind,source={scripts},target=/opt/greenhouse/user_script", command)
        self.assertIn("GREENHOUSE_SCRIPT=/opt/greenhouse/user_script/survey.py", command)
        # Relative to the current directory (cwd is self.root in run_launcher).
        relative = self.run_launcher("--dry-run", "--headless", "--script", "my scripts/survey.py")
        self.assertEqual(relative.returncode, 0, relative.stderr)
        self.assertEqual(relative.stdout, result.stdout)

    def test_scenario_is_mounted_read_only_and_announced(self):
        scenarios = self.root / "my scenarios"
        scenarios.mkdir()
        (scenarios / "aisle block.json").write_text("{}\n")
        result = self.run_launcher("--dry-run", "--headless", "--scenario", "my scenarios/aisle block.json")
        self.assertEqual(result.returncode, 0, result.stderr)
        command = shlex.split(result.stdout.splitlines()[-1])
        self.assertIn(f"type=bind,source={scenarios}/aisle block.json,"
                      "target=/opt/greenhouse/config/scenario.json,readonly", command)
        self.assertIn("GREENHOUSE_SCENARIO=/opt/greenhouse/config/scenario.json", command)
        plain = self.run_launcher("--dry-run", "--headless")
        self.assertNotIn("GREENHOUSE_SCENARIO", plain.stdout)
        (scenarios / "notes.txt").write_text("{}\n")
        for args in (("--smoke-test", "--scenario", str(scenarios / "aisle block.json")),
                     ("--scenario", str(scenarios / "notes.txt")), ("--scenario", "missing.json"),
                     ("--scenario",)):
            self.assertNotEqual(self.run_launcher("--dry-run", *args).returncode, 0, args)
        self.assertEqual(self.calls(), [])
        self.assertEqual(self.calls(), [])

    def test_viz_publishes_the_foxglove_port_on_loopback_only(self):
        result = self.run_launcher("--dry-run", "--viz", "--headless")  # the GUI path needs a DISPLAY
        self.assertEqual(result.returncode, 0, result.stderr)
        command = shlex.split(result.stdout.splitlines()[-1])
        self.assertEqual(command[command.index("--publish") + 1], "127.0.0.1:8767:8767")
        self.assertIn("viz:=true", command)
        self.assertNotIn("--network", command)
        plain = shlex.split(self.run_launcher("--dry-run", "--headless").stdout.splitlines()[-1])
        self.assertNotIn("--publish", plain)
        self.assertIn("viz:=false", plain)
        for args in (("--viz", "--generate-only"), ("--viz", "--build-only")):
            self.assertNotEqual(self.run_launcher("--dry-run", *args).returncode, 0, args)
        self.assertEqual(self.calls(), [])

    def test_script_rejects_missing_files_and_other_run_modes(self):
        (self.root / "notes.txt").write_text("x\n")
        (self.root / "ok.py").write_text("\n")
        for args in (("--script", "absent.py"), ("--script", "notes.txt"), ("--script",),
                     ("--smoke-test", "--script", "ok.py"), ("--build-only", "--script", "ok.py")):
            result = self.run_launcher("--dry-run", *args)
            self.assertNotEqual(result.returncode, 0, args)
        self.assertEqual(self.calls(), [])

    def test_missing_setup_is_not_silently_replaced(self):
        result = self.run_launcher("--headless", "--no-build", "--setup-dir", "absent directory")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("setup directory", result.stderr)
        self.assertFalse((self.project / "absent directory").exists())


if __name__ == "__main__":
    unittest.main()
