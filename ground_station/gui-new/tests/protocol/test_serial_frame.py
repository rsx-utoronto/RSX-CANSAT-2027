import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "code"))
from cansat2027.serial_frame import decode_frame, encode_frame

CPP = sys.argv.pop(1)


class FramingTests(unittest.TestCase):
    def cpp(self, *args, data=None):
        return subprocess.run([CPP, *args], input=data, capture_output=True, check=False)

    def test_both_directions(self):
        for node in ("C", "P"):
            for payload in ("CMD,1011,CAL", "1011C,1.000,42", "x" * 250):
                with self.subTest(node=node, payload=payload):
                    frame = encode_frame(node, payload)
                    native = self.cpp(node, payload)
                    self.assertEqual(native.returncode, 0)
                    self.assertEqual(native.stdout, frame)
                    self.assertEqual(decode_frame(native.stdout).payload, payload)
                    parsed = self.cpp("--decode", data=frame)
                    self.assertEqual(parsed.returncode, 0)
                    self.assertEqual(parsed.stdout, f"{node}\n{payload}\n".encode())

    def test_malformed_frames_match_cpp(self):
        for frame in (b"", b"C:x", b"C:x\r", b"C:x\r\n", b"X:x\n", b"C:\n",
                      b"C:x\nP:y\n", b"C:x\x00y\n", b"C:\xff\n", b"C:" + b"x" * 251 + b"\n"):
            with self.subTest(frame=frame):
                with self.assertRaises(ValueError):
                    decode_frame(frame)
                self.assertEqual(self.cpp("--decode", data=frame).returncode, 2)

    def test_invalid_encoder_inputs(self):
        for node, payload in (("X", "x"), ("", "x"), ("C", ""), ("C", "x" * 251),
                              ("C", "x\ny"), ("C", "x\ry"), ("C", "é")):
            with self.subTest(node=node, payload=payload):
                with self.assertRaises(ValueError):
                    encode_frame(node, payload)
                self.assertEqual(self.cpp(node, payload).returncode, 2)


if __name__ == "__main__":
    unittest.main()
