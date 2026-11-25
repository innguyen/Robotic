"""Standard-library-only Godot S0 client. One in-flight request per connection."""
import base64
import json
import math
import socket


class BridgeError(RuntimeError):
    pass


class GodotBridge:
    def __init__(self, host="127.0.0.1", port=17650, timeout=20):
        self.socket = socket.create_connection((host, port), timeout)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.stream = self.socket.makefile("rb")
        self.sequence = 0
        self.epoch = None

    def close(self):
        self.stream.close()
        self.socket.close()

    def request(self, op, **fields):
        self.sequence += 1
        payload = dict(op=op, id=self.sequence, **fields)
        self.socket.sendall((json.dumps(payload, allow_nan=False) + "\n").encode())
        line = self.stream.readline(2_000_000)
        if not line.endswith(b"\n"):
            raise BridgeError("Disconnected or oversized response")
        response = json.loads(line)
        if response.get("id") != self.sequence:
            raise BridgeError("Response ID mismatch")
        if not response.get("ok"):
            raise BridgeError(response.get("error", "Unknown engine error"))
        observation = response["observation"]
        validate_observation(observation)
        self.epoch = observation["epoch"]
        return observation

    def observe(self):
        return self.request("observe")

    def reset(self, scenario="clear", seed=7):
        return self.request("reset", scenario=scenario, seed=seed)

    def step(self, vx=0.0, vy=0.0, wz=0.0, ticks=6):
        if self.epoch is None:
            self.observe()
        return self.request("step", epoch=self.epoch, velocity=[vx, vy, wz], ticks=ticks)


def validate_observation(obs):
    if obs["protocol"] != 1:
        raise BridgeError("Unsupported protocol")
    stamp = obs["simulation_timestamp"]
    if not math.isfinite(stamp) or abs(stamp - obs["tick"] / 60) > 1e-8:
        raise BridgeError("Invalid simulation clock")
    for name in ("pose", "scan", "camera"):
        if obs[name]["stamp"] != stamp:
            raise BridgeError("Unsynchronized sample")
    if not all(math.isfinite(obs["pose"][key]) for key in ("x", "y", "z", "yaw")):
        raise BridgeError("Invalid self pose")
    scan = obs["scan"]
    if len(scan["ranges"]) != 192 or any(
        value is not None and (not math.isfinite(value) or not 0 <= value <= scan["range_max"] + 1e-5)
        for value in scan["ranges"]
    ):
        raise BridgeError("Invalid scan")
    image = obs["camera"]
    if image["valid"]:
        raw = base64.b64decode(image["data"], validate=True)
        if len(raw) != image["width"] * image["height"] * 3:
            raise BridgeError("Invalid RGB8 length")
    elif image["data"] is not None:
        raise BridgeError("Invalid camera status")


def red_detection(camera):
    """Tiny fixture-specific RGB detector, not a human perception model/oracle."""
    if not camera["valid"]:
        raise BridgeError("RGB rendering unavailable")
    pixels = base64.b64decode(camera["data"], validate=True)
    width = camera["width"]
    indices = [i // 3 for i in range(0, len(pixels), 3)
               if pixels[i] > 150 and pixels[i + 1] < 85 and pixels[i + 2] < 85]
    if len(indices) < 8:
        return None
    u = sum(i % width for i in indices) / len(indices)
    focal = width / (2 * math.tan(camera["horizontal_fov"] / 2))
    return dict(pixels=len(indices), u=u, bearing=math.atan(((width - 1) / 2 - u) / focal))
