"""Host-only tests for the container's mission logic, telemetry format and
command parser. Never opens a port or flashes a board.

Compiles the real container sources (mission_manager.cpp,
container_telemetry.cpp, command_parser.cpp) together with
container_logic_harness.cpp using a C++17 compiler (``c++``, or the ``CXX``
environment variable), then runs each harness case as a subtest.

Run from the repository root:
    python -m unittest discover -s software/tests -p 'test_container_logic.py' -v
"""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
CONTAINER = ROOT / "software/firmware/container"
SOURCES = [
    CONTAINER / "src/mission_manager.cpp",
    CONTAINER / "src/container_telemetry.cpp",
    CONTAINER / "src/command_parser.cpp",
    Path(__file__).resolve().parent / "container_logic_harness.cpp",
]


class ContainerLogicTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get("CXX") or shutil.which("c++")
        if not compiler:
            raise RuntimeError("A C++17 compiler is required (set CXX or install c++).")
        cls.temp = tempfile.TemporaryDirectory(prefix="cansat-container-tests-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.harness = Path(cls.temp.name) / "container_logic_harness"
        command = [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                   f"-I{CONTAINER / 'include'}",
                   f"-I{ROOT / 'software/firmware_lib/inc'}",
                   *map(str, SOURCES), "-o", str(cls.harness)]
        result = subprocess.run(command, capture_output=True, text=True, timeout=120)
        if result.returncode:
            raise RuntimeError(f"Host compile failed:\n{result.stdout}{result.stderr}")
        listing = subprocess.run([str(cls.harness), "--list"], capture_output=True,
                                 text=True, timeout=10)
        cls.cases = listing.stdout.split()

    def test_harness_has_cases(self):
        self.assertGreater(len(self.cases), 10)

    def test_all_cases(self):
        for case in self.cases:
            with self.subTest(case=case):
                result = subprocess.run([str(self.harness), case], capture_output=True,
                                        text=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
