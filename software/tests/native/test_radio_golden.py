"""Independent Python wire-vector oracle, not a production GUI protocol adapter."""
import json
from pathlib import Path
import struct
import subprocess
import sys
import zlib

spec = json.loads(Path(sys.argv[1]).read_text())
body = struct.pack("<2sBBHBBIIIIH", b"CS", 1, 2, 1234, 0, 1, 11, 22, 1, 0, 3) + bytes([0xA5, 0, 0xFF])
expected = (body + struct.pack("<I", zlib.crc32(body))).hex()
assert expected == spec["golden_command_hex"]
result = subprocess.run([sys.argv[2]], capture_output=True, text=True, check=True)
assert f"GOLDEN_COMMAND={expected}\n" in result.stdout
assert struct.calcsize("<2sBBHBBIIIIH") == spec["header_bytes"]
assert spec["max_payload_bytes"] + spec["header_bytes"] + spec["checksum_bytes"] == spec["max_radio_bytes"]
print("Python struct/zlib and C++ radio golden vector match; 250-byte budget verified")
