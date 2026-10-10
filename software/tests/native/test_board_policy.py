"""Guard the confirmed board/module preference without inventing a pinout/SKU."""
import json
from pathlib import Path
import sys

root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[3]
path = root / "software/lib/platform/boards.json"
try:
    boards = json.loads(path.read_text())
    policy = boards.get("module_policy")
    if policy is None:
        print("MODULE_POLICY_UNSET", file=sys.stderr)
        sys.exit(2)
    assert policy["preferred_family"] == "ESP32-WROOM-32UE"
    assert policy["idf_target"] == "esp32"
    assert set(policy["scope"]) == {"container", "pocketqube", "ground_radio"}
    assert policy["status"] == "user_preferred_for_majority_not_final_per_target"
    assert policy["exceptions_require_review"] is True
    assert policy["antenna"] == "external_connector"
    assert policy["carrier_board"] == "ESP32-DevKitC V4"
    for field in ("full_part_number", "flash_mb", "psram_mb"):
        assert policy[field] is None, field + " must await user confirmation"
    for target in boards["targets"].values():
        assert target["framework"] == "esp-idf"
        assert target["board"] is None and target["pins"] == {}
    assert boards["status"] == "unconfigured_do_not_flash"
    lock = json.loads((root / "software/esp_idf/sdk.lock.json").read_text())
    assert boards["esp_idf_version"] == lock["version"] == "6.1.0"
    assert boards["vision"]["processor"] is None
    print("MODULE_POLICY: ESP32-DevKitC V4 / ESP32-WROOM-32UE preferred; IDF target=esp32; full SKU/memory/pins unset; vision processor undecided")
except (AssertionError, KeyError, OSError, ValueError) as error:
    print("MODULE_POLICY_INVALID: " + str(error), file=sys.stderr)
    sys.exit(1)
