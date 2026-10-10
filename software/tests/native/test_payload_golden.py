"""Independent binary-layout oracle. Not a GUI adapter or CSV exporter."""
import json
from pathlib import Path
import struct
import subprocess
import sys
import unittest

ROOT = Path(sys.argv.pop(1)).resolve()
CLI = sys.argv.pop(1)
SPEC = json.loads((ROOT / "software/lib/communications/mission_payloads.json").read_text())
CASES = json.loads((ROOT / "software/tests/native/fixtures/mission_payload_vectors.json").read_text())["cases"]


def text(value, width):
    data = value.encode("ascii")
    assert len(data) <= width
    return bytes([len(data)]) + data.ljust(width, b"\0")


def common(c, tag):
    return (struct.pack("<BBQIIBIiIiIiBB", 1, tag, c["mission_time_ms"], c["packet_count"],
                        c["command_count"], ord(c["mode"]), c["valid"], c["altitude_mm"],
                        c["pressure_pa"], c["temperature_milli_c"], c["battery_mv"],
                        c["battery_current_ma"], c["mechanism_state"], c["mechanism_known"])
            + text(c["command_echo"], 24))


def encode_case(case):
    if case["mode"].startswith("cmd_"):
        return (bytes([1, 3, case["opcode"]]) +
                (struct.pack("<" + case["argument_struct"], case["argument"]) if case["argument_struct"] else b""))
    c = case["values"]
    if case["mode"] == "c":
        return common(c, 1) + bytes([c["state"]])
    result = (common(c, 2)
              + struct.pack("<9iqqiH", *c["gyro_mdeg_s"], *c["accel_mm_s2"], *c["mag_milligauss"],
                            c["latitude_nanodeg"], c["longitude_nanodeg"], c["gnss_altitude_mm"], c["gnss_satellites"])
              + text(c["gnss_time"], 32) + struct.pack("<II", *c["solar_mv"]))
    for name in ("stabilization", "science"):
        x = c[name]
        result += struct.pack("<HBB", x["profile"], x["version"], len(x["data"])) + bytes(x["data"])
    return result


class PayloadGolden(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run([CLI], capture_output=True, text=True, check=True)
        cls.native = dict(line.removeprefix("GOLDEN_").split("=", 1)
                          for line in result.stdout.splitlines() if line.startswith("GOLDEN_"))

    def invoke(self, mode, data):
        return subprocess.run([CLI, "--decode", mode, data.hex()], capture_output=True, text=True, check=False)

    def test_independent_encode_decode_vectors(self):
        for case in CASES:
            with self.subTest(case=case["name"]):
                expected = encode_case(case)
                self.assertEqual(self.native[case["name"]], expected.hex())
                decoded = self.invoke(case["mode"], expected)
                self.assertEqual(decoded.returncode, 0, decoded.stderr)
                self.assertEqual(decoded.stdout, expected.hex() + "\n")
                self.assertLessEqual(len(expected), 220)

    def test_schema_offsets_and_sizes(self):
        self.assertEqual(SPEC["payload_version"], 1)
        fields = SPEC["common_telemetry_fields"]
        self.assertEqual([f["offset"] for f in fields], [0,1,2,10,14,18,19,23,27,31,35,39,43,44,45,46])
        self.assertEqual(fields[-1]["offset"] + fields[-1]["bytes"], 70)
        end = 70
        for field in SPEC["pocketqube"]["fields"]:
            self.assertEqual(field["offset"], end)
            end += field["bytes"]
        self.assertEqual(end, 169)
        self.assertEqual(end + 8, SPEC["pocketqube"]["base_payload_bytes"])
        self.assertEqual(end + 8 + SPEC["pocketqube"]["extensions"]["combined_data_budget"], 220)
        self.assertEqual(SPEC["container"]["payload_bytes"], len(encode_case(CASES[0])))
        self.assertEqual(SPEC["pocketqube"]["max_radio_bytes"], len(encode_case(CASES[2])) + 30)
        for case in CASES[3:]:
            entry = next(op for op in SPEC["commands"]["catalog"] if op["id"] == case["opcode"])
            self.assertEqual(len(encode_case(case)), entry["bytes"])
            self.assertIn(case["mode"][-1].upper(), entry["targets"])

    def test_unknown_version_schema_truncation_and_trailing_data(self):
        for case in CASES:
            raw = encode_case(case)
            malformed = [b"", raw[:-1], raw+b"\0", bytes([2])+raw[1:], raw[:1]+b"\x7f"+raw[2:]]
            for bad in malformed:
                with self.subTest(case=case["name"], bytes=len(bad)):
                    self.assertEqual(self.invoke(case["mode"], bad).returncode, 2)

    def test_semantics_and_padding_rejected(self):
        raw = bytearray(encode_case(CASES[0]))
        mutations = [(18,0),(22,128),(27,0),(44,0),(45,25),(46,ord('=')),(69,1),(70,4)]
        # pressure is a multi-byte field; zero all four bytes explicitly.
        for offset, value in mutations:
            bad = bytearray(raw)
            if offset == 27:
                bad[27:31] = b"\0"*4
            else:
                bad[offset] = value
            self.assertEqual(self.invoke("c", bad).returncode, 2, offset)
        pq = bytearray(encode_case(CASES[1]))
        for bad in [pq[:169]+struct.pack("<HBB", 1, 1, 44)+bytes(44)+bytes(4),
                    pq[:169]+struct.pack("<HBB", 1, 1, 0)+bytes(4)]:
            self.assertEqual(self.invoke("p", bad).returncode, 2)
        bad = bytearray(pq); bad[106:114] = struct.pack("<q", 90000000001)
        self.assertEqual(self.invoke("p", bad).returncode, 2)
        bad = bytearray(pq); bad[128] = 33
        self.assertEqual(self.invoke("p", bad).returncode, 2)

    def test_node_scoping_and_command_parameter_rejection(self):
        for mode, data in [
            ("cmd_p",bytes([1,3,1])), ("cmd_p",bytes([1,3,2])),
            ("cmd_c",bytes([1,3,6,4])), ("cmd_p",bytes([1,3,6,1])),
            ("cmd_c",bytes([1,3,7,1])), ("cmd_p",bytes([1,3,7,3])),
            ("cmd_c",bytes([1,3,4,0])), ("cmd_p",bytes([1,3,4,4])),
            ("cmd_p",bytes([1,3,8])+struct.pack("<i",360001)),
            ("cmd_p",bytes([1,3,8])+struct.pack("<i",0)),
            ("cmd_c",bytes([1,3,5])+struct.pack("<I",0)),
            ("cmd_c",bytes([1,3,255]))]:
            self.assertEqual(self.invoke(mode,data).returncode, 2)

    def test_missing_reading_is_distinct_from_valid_zero(self):
        raw = bytes.fromhex(self.native["container_missing"])
        self.assertEqual(len(raw), 71)
        changed = bytearray(raw)
        changed[19:23] = struct.pack("<I", 1 << 4)
        self.assertEqual(self.invoke("c", raw).returncode, 0)
        self.assertEqual(self.invoke("c", changed).returncode, 0)
        self.assertNotEqual(raw, changed)


if __name__ == "__main__":
    unittest.main()
