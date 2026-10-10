"""Experimental ASCII host/bridge framing; not yet connected to the GUI runtime.

Input to decode_frame must be one complete frame, not arbitrary serial chunks.
The eventual serial receiver must bound its buffer and resynchronize on LF.
"""
from dataclasses import dataclass

MAX_PAYLOAD = 250


@dataclass(frozen=True)
class SerialFrame:
    node: str
    payload: str


def encode_frame(node: str, payload: str) -> bytes:
    """Return C/P + colon + 1..250 printable ASCII bytes + LF; raise ValueError otherwise."""
    if node not in ("C", "P"):
        raise ValueError("node must be C or P")
    if not isinstance(payload, str):
        raise ValueError("payload must be text")
    if not 1 <= len(payload) <= MAX_PAYLOAD:
        raise ValueError("payload length must be 1..250")
    if any(not 0x20 <= ord(ch) <= 0x7E for ch in payload):
        raise ValueError("payload must contain printable ASCII only")
    return f"{node}:{payload}\n".encode("ascii")


def decode_frame(frame: bytes) -> SerialFrame:
    """Decode one complete canonical frame; ValueError for bad type, size, text or delimiter."""
    if not isinstance(frame, bytes) or not 4 <= len(frame) <= MAX_PAYLOAD + 3:
        raise ValueError("invalid frame type or length")
    if frame[1:2] != b":" or not frame.endswith(b"\n"):
        raise ValueError("expected node:payload followed by LF")
    try:
        node = frame[:1].decode("ascii")
        payload = frame[2:-1].decode("ascii")
    except UnicodeDecodeError as exc:
        raise ValueError("frame must be ASCII") from exc
    if encode_frame(node, payload) != frame:
        raise ValueError("noncanonical frame")
    return SerialFrame(node, payload)
