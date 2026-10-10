"""Host CMake contract checks. Not an ESP-IDF configure, compile, or link test."""
from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parents[3]
ROLES = ("container", "pocketqube", "ground_radio")
APPS = ROOT / "software/esp_idf/apps"
if not all((APPS / role / "CMakeLists.txt").is_file() for role in ROLES):
    print("APP_PROJECTS_UNAVAILABLE: expected container, pocketqube, ground_radio", file=sys.stderr)
    raise SystemExit(2)

# Deliberately only a CMake registration recorder. It cannot build firmware or
# validate SDK APIs, Kconfig, toolchain, linker, target memory, or runtime behavior.
PROJECT_RECORDER = r'''
function(idf_component_register)
  cmake_parse_arguments(REG "" "" "SRCS;INCLUDE_DIRS;REQUIRES;PRIV_REQUIRES;REQUIRED_IDF_TARGETS" ${ARGN})
  if(REG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "Unknown registration arguments: ${REG_UNPARSED_ARGUMENTS}")
  endif()
  foreach(path IN LISTS REG_SRCS REG_INCLUDE_DIRS)
    if(NOT EXISTS "${TEST_COMPONENT_DIR}/${path}")
      message(FATAL_ERROR "Missing registered path: ${TEST_COMPONENT_DIR}/${path}")
    endif()
    if(path MATCHES "tests|stubs")
      message(FATAL_ERROR "Host substitutes leaked into firmware")
    endif()
  endforeach()
  if(REG_REQUIRED_IDF_TARGETS AND NOT IDF_TARGET IN_LIST REG_REQUIRED_IDF_TARGETS)
    message(FATAL_ERROR "Unsupported registration target")
  endif()
  message(STATUS "REGISTER ${TEST_COMPONENT} sources=${REG_SRCS} targets=${REG_REQUIRED_IDF_TARGETS}")
  set(CONTRACT_DEPENDS ${REG_REQUIRES} ${REG_PRIV_REQUIRES} PARENT_SCOPE)
  set(COMPONENT_LIB "contract_${TEST_COMPONENT}" PARENT_SCOPE)
endfunction()

function(target_compile_features)
  if(NOT "cxx_std_17" IN_LIST ARGN)
    message(FATAL_ERROR "Expected explicit C++17")
  endif()
endfunction()

macro(project name)
  if(NOT "${name}" STREQUAL "cansat_${TEST_ROLE}")
    message(FATAL_ERROR "Unexpected project identity")
  endif()
  if(NOT IDF_TARGET STREQUAL "esp32" OR NOT COMPONENTS STREQUAL "main")
    message(FATAL_ERROR "Unexpected target/component selection")
  endif()
  set(TEST_COMPONENT "main")
  set(TEST_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}/main")
  include("${TEST_COMPONENT_DIR}/CMakeLists.txt")
  set(pending ${CONTRACT_DEPENDS})
  set(seen main)
  while(pending)
    list(POP_FRONT pending TEST_COMPONENT)
    if(TEST_COMPONENT IN_LIST seen)
      continue()
    endif()
    list(APPEND seen "${TEST_COMPONENT}")
    # Only these SDK dependencies are currently expected; do not fake SDK files.
    if(TEST_COMPONENT STREQUAL "log" OR TEST_COMPONENT STREQUAL "esp_driver_i2c" OR
       TEST_COMPONENT STREQUAL "esp_wifi" OR TEST_COMPONENT STREQUAL "freertos")
      continue()
    endif()
    set(TEST_COMPONENT_DIR "${EXTRA_COMPONENT_DIRS}/${TEST_COMPONENT}")
    include("${TEST_COMPONENT_DIR}/CMakeLists.txt")
    list(APPEND pending ${CONTRACT_DEPENDS})
  endwhile()
  message(STATUS "HOST_CMAKE_CONTRACT ${name} target=${IDF_TARGET}; NOT_SDK_BUILD")
endmacro()
'''


class AppProjects(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="cansat-app-contract-")
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name).resolve()
        self.sdk = self.work / "registration_recorder"
        project = self.sdk / "tools/cmake/project.cmake"
        project.parent.mkdir(parents=True)
        project.write_text(PROJECT_RECORDER, encoding="utf-8")
        (project.parent / "version.cmake").write_text(
            "set(IDF_VERSION_MAJOR 6)\nset(IDF_VERSION_MINOR 1)\nset(IDF_VERSION_PATCH 0)\n",
            encoding="utf-8")
        self.env = os.environ.copy()
        self.env.pop("IDF_TARGET", None)
        self.env["IDF_PATH"] = str(self.sdk)

    def configure(self, role, definitions=(), env=None, script_mode=True):
        command = ["cmake", f"-DTEST_ROLE={role}", *definitions]
        if script_mode:
            command += ["-P", str(APPS / role / "CMakeLists.txt")]
        else:
            command += ["-S", str(APPS / role), "-B", str(self.work / role)]
        return subprocess.run(command, cwd=self.work, env=self.env if env is None else env,
                              capture_output=True, text=True, timeout=30)

    def test_all_three_dependency_closures(self):
        for role in ROLES:
            with self.subTest(role=role):
                result = self.configure(role)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(f"HOST_CMAKE_CONTRACT cansat_{role} target=esp32; NOT_SDK_BUILD", result.stdout)
                self.assertIn("REGISTER main sources=app_main.cpp targets=esp32", result.stdout)
                self.assertIn("REGISTER cansat_app_boot sources=app_boot.cpp targets=esp32", result.stdout)
                self.assertIn("REGISTER cansat_communications", result.stdout)
                self.assertIn("REGISTER cansat_espnow", result.stdout)
                for component in ("cansat_core", "cansat_mission", "cansat_drivers", "cansat_esp_idf"):
                    if role == "ground_radio":
                        self.assertNotIn(f"REGISTER {component} ", result.stdout)
                    else:
                        self.assertIn(f"REGISTER {component} ", result.stdout)

    def test_explicit_esp32_cache_and_environment(self):
        for role in ROLES:
            with self.subTest(role=role):
                env = dict(self.env, IDF_TARGET="esp32")
                result = self.configure(role, ("-DIDF_TARGET=esp32",), env)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_rejects_other_chip_cache(self):
        for role in ROLES:
            with self.subTest(role=role):
                result = self.configure(role, ("-DIDF_TARGET=esp32s3",))
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("require IDF_TARGET=esp32", result.stderr)

    def test_rejects_other_chip_environment_even_with_esp32_cache(self):
        for role in ROLES:
            with self.subTest(role=role):
                env = dict(self.env, IDF_TARGET="esp32s3")
                result = self.configure(role, ("-DIDF_TARGET=esp32",), env)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("conflicting environment target", " ".join(result.stderr.split()))

    def test_missing_sdk_stops_real_cmake_configure(self):
        for role in ROLES:
            with self.subTest(role=role):
                env = self.env.copy()
                env.pop("IDF_PATH")
                # Real configure must stop before toolchain/compiler detection.
                result = self.configure(role, env=env, script_mode=False)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("ESP-IDF is not activated", result.stderr)

    def test_empty_sdk_path(self):
        for role in ROLES:
            with self.subTest(role=role):
                result = self.configure(role, env=dict(self.env, IDF_PATH=""))
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("ESP-IDF is not activated", result.stderr)

    def test_incomplete_sdk_path(self):
        for role in ROLES:
            with self.subTest(role=role):
                result = self.configure(role, env=dict(self.env, IDF_PATH=str(self.work)))
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("IDF_PATH does not contain", result.stderr)

    def test_wrong_sdk_version(self):
        (self.sdk / "tools/cmake/version.cmake").write_text(
            "set(IDF_VERSION_MAJOR 6)\nset(IDF_VERSION_MINOR 0)\nset(IDF_VERSION_PATCH 3)\n",
            encoding="utf-8")
        for role in ROLES:
            with self.subTest(role=role):
                result = self.configure(role)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("Pinned ESP-IDF version 6.1.0 required", result.stderr)


if __name__ == "__main__":
    unittest.main()
