"""Offline tests for the build-only runner; no SDK or device is invoked."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("idf_build", ROOT / "tools/esp_idf_build.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class BuildRunner(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="cansat-idf-runner-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.base = self.root / "software/esp_idf"
        self.base.mkdir(parents=True)
        self.lock = json.loads((ROOT / "software/esp_idf/sdk.lock.json").read_text())
        self.sdk = self.base / self.lock["sdk_directory"]
        self.python_env = self.base / self.lock["tools_directory"] / self.lock["python_environment"]
        (self.sdk / "tools").mkdir(parents=True)
        (self.sdk / "tools/idf.py").touch()
        (self.python_env / "bin").mkdir(parents=True)
        (self.python_env / "bin/python").touch()
        self.manifest = self.sdk / "tools/tools.json"
        self.manifest.write_text('{"fixture":true}\n')
        self.lock["tools_manifest_sha256"] = hashlib.sha256(self.manifest.read_bytes()).hexdigest()
        (self.base / "sdk.lock.json").write_text(json.dumps(self.lock))
        self.results = {
            ("rev-parse", "HEAD"): self.lock["commit"],
            ("remote", "get-url", "origin"): self.lock["repository"],
            ("status", "--porcelain", "--untracked-files=no"): "",
            ("submodule", "status", "--recursive"): " " + "a" * 40 + " fixture (pinned)",
        }
        mock = patch.object(runner, "git", side_effect=lambda sdk, *args: self.results[args])
        mock.start()
        self.addCleanup(mock.stop)

    def test_pin_verified(self):
        self.assertEqual(runner.verify_installation(self.root)[0], self.lock)

    def test_missing_install(self):
        (self.sdk / "tools/idf.py").unlink()
        with self.assertRaisesRegex(ValueError, "missing"):
            runner.verify_installation(self.root)

    def test_changed_commit_origin_or_tracked_files(self):
        for args in (("rev-parse", "HEAD"), ("remote", "get-url", "origin"),
                     ("status", "--porcelain", "--untracked-files=no")):
            with self.subTest(args=args):
                original = self.results[args]
                self.results[args] = "changed"
                with self.assertRaises(ValueError):
                    runner.verify_installation(self.root)
                self.results[args] = original

    def test_missing_changed_or_conflicted_submodules(self):
        for value in ("", "-missing", "+changed", "Uconflict"):
            with self.subTest(value=value):
                self.results[("submodule", "status", "--recursive")] = value
                with self.assertRaisesRegex(ValueError, "submodules"):
                    runner.verify_installation(self.root)

    def test_changed_manifest(self):
        self.manifest.write_text("changed")
        with self.assertRaisesRegex(ValueError, "manifest"):
            runner.verify_installation(self.root)

    def test_flash_monitor_ports_and_invalid_jobs_rejected(self):
        for args in (["flash"], ["monitor"], ["build", "--port", "/dev/fixture"],
                     ["build", "all", "--jobs", "0"], ["build", "all", "--jobs", "17"]):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    runner.main(args)
                self.assertEqual(result.exception.code, 2)

    def test_command_is_build_only_with_separate_configs(self):
        for role in runner.ROLES:
            command = runner.build_command(self.root, role)
            self.assertEqual(command[-1], "build")
            self.assertIn("Ninja", command)
            self.assertIn("IDF_TARGET=esp32", command)
            self.assertIn(str(self.base / "apps" / role), command)
            self.assertIn(f"SDKCONFIG={self.base / '.local/build-ninja' / role / 'sdkconfig'}", command)
            self.assertNotIn("flash", command)
            self.assertNotIn("monitor", command)

    def test_check_does_not_run_sdk(self):
        verified = runner.verify_installation(self.root)
        with patch.object(runner, "verify_installation", return_value=verified), \
             patch.object(runner.subprocess, "run") as run, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(runner.main(["check"]), 0)
            run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
