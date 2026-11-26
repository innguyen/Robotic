"""Bounded JSON RPC used only by the trusted host; never handed to the agent."""
import socket
import struct
import zlib

from .contracts import canonical, strict_loads

MAX_BYTES = 16 * 1024 * 1024


def decode_payload(payload):
    if payload.startswith(b"Z"):
        decoder = zlib.decompressobj()
        decoded = decoder.decompress(payload[1:], MAX_BYTES + 1)
        if len(decoded) > MAX_BYTES or decoder.unconsumed_tail or not decoder.eof or decoder.unused_data:
            raise ValueError("invalid/oversized compressed response")
        return decoded
    return payload


def receive_exact(sock, size):
    pieces = []
    while size:
        piece = sock.recv(size)
        if not piece:
            raise ConnectionError("peer disconnected")
        pieces.append(piece)
        size -= len(piece)
    return b"".join(pieces)


class HostClient:
    def __init__(self, port, token, timeout=30):
        self.socket = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.socket.settimeout(timeout)
        self.token = token
        self.sequence = 0

    def call(self, operation, arguments=None):
        request = canonical({"token": self.token, "sequence": self.sequence,
                             "operation": operation, "arguments": arguments or {}})
        if len(request) > MAX_BYTES:
            raise ValueError("request too large")
        self.socket.sendall(struct.pack("!I", len(request)) + request)
        size = struct.unpack("!I", receive_exact(self.socket, 4))[0]
        if not 0 < size <= MAX_BYTES:
            raise ValueError("response too large")
        response = strict_loads(decode_payload(receive_exact(self.socket, size)))
        if response["sequence"] != self.sequence:
            raise ValueError("out-of-order response")
        self.sequence += 1
        if "error" in response:
            raise RuntimeError(response["error"])
        return response["result"]

    def close(self):
        self.socket.close()
