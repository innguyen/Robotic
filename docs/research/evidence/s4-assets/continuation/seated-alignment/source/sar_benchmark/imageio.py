"""Lossless diagnostic PNG export from sensor RGB8; no synthetic image content."""
import struct
import zlib
from pathlib import Path


def write_rgb_png(path, width, height, rgb):
    if len(rgb) != width * height * 3:
        raise ValueError("invalid RGB payload")
    def chunk(kind, data):
        return struct.pack("!I", len(data)) + kind + data + struct.pack("!I", zlib.crc32(kind + data) & 0xffffffff)
    raw = b"".join(b"\0" + bytes(rgb[row * width * 3:(row + 1) * width * 3]) for row in range(height))
    payload = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack("!2I5B", width, height, 8, 2, 0, 0, 0))
    payload += chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b"")
    with Path(path).open("xb") as stream:
        stream.write(payload)
