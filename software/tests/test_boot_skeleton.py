"""Host-only regression checks; never open a port or flash a board."""

import configparser
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
ROLES = {
    "ground_station": ("ground", "GROUND_STATION"),
    "pq": ("PQ", "PQ"),
    "container": ("container", "CONTAINER"),
}

ARDUINO_FAKE = r"""
#pragma once
#include <cstdarg>
#include <cstdio>
#include <cstdint>
struct SerialFake {
  unsigned long baud = 0;
  unsigned begin_calls = 0;
  void begin(unsigned long value) { baud = value; ++begin_calls; }
  void printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    std::vprintf(format, args);
    va_end(args);
  }
  void println(const char* line) { std::printf("%s\n", line); }
};
extern SerialFake Serial;
void delay(unsigned long milliseconds);
"""

MAC_FAKE = r"""
#pragma once
#include <cstdint>
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
enum esp_mac_type_t { ESP_MAC_WIFI_STA };
esp_err_t esp_read_mac(uint8_t* mac, esp_mac_type_t type);
"""

HARNESS = r"""
#include <Arduino.h>
#include <esp_mac.h>
#include <cstring>
#include <vector>

SerialFake Serial;
static std::vector<unsigned long> delays;
static bool fail_mac = false;
static unsigned mac_calls = 0;
static bool correct_mac_type = true;

void delay(unsigned long milliseconds) { delays.push_back(milliseconds); }
esp_err_t esp_read_mac(uint8_t* mac, esp_mac_type_t type) {
  ++mac_calls;
  correct_mac_type = type == ESP_MAC_WIFI_STA;
  if (fail_mac) return -42;
  const uint8_t fixture[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
  std::memcpy(mac, fixture, sizeof(fixture));
  return ESP_OK;
}

void setup();
void loop();
int main(int argc, char**) {
  fail_mac = argc > 1;
  setup();
  for (int i = 0; i < 3; ++i) loop();
  if (Serial.baud != 115200 || Serial.begin_calls != 1) return 10;
  if (mac_calls != 1 || !correct_mac_type) return 11;
  if (delays != std::vector<unsigned long>({250, 10, 10, 10})) return 12;
  return 0;
}
"""


class ProjectConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.config = configparser.ConfigParser(interpolation=None)
        self.config.read(ROOT / "platformio.ini", encoding="utf-8")

    def test_common_settings_and_explicit_default(self):
        self.assertEqual(self.config["platformio"]["src_dir"], "software/firmware")
        self.assertEqual(self.config["platformio"]["default_envs"], "ground_station")
        common = self.config["env"]
        self.assertEqual(common["platform"], "platformio/espressif32@8.0.0")
        self.assertEqual(common["framework"], "arduino")
        self.assertEqual(common["board"], "esp32dev")
        self.assertEqual(common["monitor_speed"], "115200")
        self.assertEqual(common["lib_deps"].strip(), "symlink://software/firmware_lib")
        for section in self.config.values():
            self.assertNotIn("upload_port", section)
            self.assertNotIn("monitor_port", section)

    def test_exactly_three_isolated_role_environments(self):
        actual = {name[4:] for name in self.config if name.startswith("env:")}
        self.assertEqual(actual, set(ROLES))
        for env, (folder, _) in ROLES.items():
            with self.subTest(environment=env):
                settings = self.config[f"env:{env}"]
                self.assertEqual(settings["build_src_filter"].split(),
                                 ["-<*>", f"+<{folder}/src/>"])
                self.assertEqual(settings["build_flags"].strip(),
                                 f"-Isoftware/firmware/{folder}/include")
                self.assertTrue((ROOT / "software/firmware" / folder / "src/main.cpp").is_file())

    def test_active_sources_do_not_initialize_hardware(self):
        paths = [ROOT / "software/firmware_lib/src/firmware_boot.cpp"]
        paths.extend(ROOT / "software/firmware" / folder / "src/main.cpp"
                     for folder, _ in ROLES.values())
        for path in paths:
            source = path.read_text(encoding="utf-8")
            with self.subTest(source=str(path)):
                self.assertNotRegex(source, r"\b(?:esp_now_init|esp_wifi_init|esp_wifi_start|"
                                    r"pinMode|digitalWrite|analogWrite|ledcWrite|"
                                    r"xTaskCreate|vTaskSuspend)\s*\(")
                self.assertNotRegex(source, r"(?:WiFi|Wire|SPI|EspNowLink)\s*(?:\.|::)")
                self.assertNotRegex(source, r"\bwhile\s*\(")


class BootBehaviorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get("CXX") or shutil.which("c++")
        if not compiler:
            raise RuntimeError("A C++17 compiler is required (set CXX or install c++).")
        cls.temp = tempfile.TemporaryDirectory(prefix="cansat-boot-tests-")
        cls.addClassCleanup(cls.temp.cleanup)
        build = Path(cls.temp.name)
        (build / "Arduino.h").write_text(ARDUINO_FAKE, encoding="utf-8")
        (build / "esp_mac.h").write_text(MAC_FAKE, encoding="utf-8")
        (build / "harness.cpp").write_text(HARNESS, encoding="utf-8")
        cls.executables = {}
        for env, (folder, _) in ROLES.items():
            executable = build / env
            command = [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                       f"-I{build}", f"-I{ROOT / 'software/firmware_lib/inc'}",
                       f"-I{ROOT / 'software/firmware' / folder / 'include'}",
                       str(ROOT / "software/firmware_lib/src/firmware_boot.cpp"),
                       str(ROOT / "software/firmware" / folder / "src/main.cpp"),
                       str(build / "harness.cpp"), "-o", str(executable)]
            result = subprocess.run(command, capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise RuntimeError(f"Host compile failed for {env}:\n{result.stdout}{result.stderr}")
            cls.executables[env] = executable

    def test_success_banner_for_every_role_and_silent_idle(self):
        for env, (_, label) in ROLES.items():
            with self.subTest(environment=env):
                result = subprocess.run([str(self.executables[env])], capture_output=True,
                                        text=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stderr, "")
                self.assertEqual(result.stdout,
                                 f"ROLE={label}\nMAC=AA:BB:CC:DD:EE:FF\nMODE=BOOT_ONLY\n")

    def test_mac_failure_is_reported_without_hanging_for_every_role(self):
        for env, (_, label) in ROLES.items():
            with self.subTest(environment=env):
                result = subprocess.run([str(self.executables[env]), "fail-mac"],
                                        capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stderr, "")
                self.assertEqual(result.stdout,
                                 f"ROLE={label}\nMAC=UNAVAILABLE (error=-42)\nMODE=BOOT_ONLY\n")


if __name__ == "__main__":
    unittest.main()
