"""Independent protocol/detection regressions, no Godot or external packages needed."""
import base64
import copy
import json
import math
import socket
import threading
import unittest

from bridge import BridgeError, GodotBridge, red_detection, validate_observation


def sample():
    return dict(protocol=1, epoch=3, tick=6, simulation_timestamp=.1,
                pose=dict(x=0, y=0, z=.11, yaw=0, stamp=.1),
                scan=dict(stamp=.1, range_max=6, ranges=[None] * 192),
                camera=dict(stamp=.1, valid=False, data=None))


class BridgeTests(unittest.TestCase):
    def test_sensor_freshness_and_bad_numeric_values(self):
        obs = sample()
        validate_observation(obs)
        for invalid in (float("nan"), float("inf"), -1, 7):
            bad = copy.deepcopy(obs)
            bad["scan"]["ranges"][10] = invalid
            with self.assertRaises(BridgeError):
                validate_observation(bad)
        for sensor in ("pose", "scan", "camera"):
            bad = copy.deepcopy(obs)
            bad[sensor]["stamp"] = 0
            with self.assertRaises(BridgeError):
                validate_observation(bad)

    def test_rgb_size_is_checked(self):
        obs = sample()
        obs["camera"].update(valid=True, width=160, height=120, data=base64.b64encode(b"x").decode())
        with self.assertRaises(BridgeError):
            validate_observation(obs)

    def test_red_detection_uses_only_pixels_and_camera_intrinsics(self):
        # Red patch to the right must produce negative FLU bearing.
        data = bytearray(160 * 120 * 3)
        for y in range(50, 60):
            for x in range(110, 120):
                data[(y * 160 + x) * 3] = 230
        camera = dict(valid=True, width=160, height=120, horizontal_fov=1.012290966,
                      data=base64.b64encode(data).decode())
        detection = red_detection(camera)
        self.assertEqual(detection["pixels"], 100)
        self.assertAlmostEqual(detection["u"], 114.5)
        self.assertLess(detection["bearing"], 0)
        camera["data"] = base64.b64encode(bytes(len(data))).decode()
        self.assertIsNone(red_detection(camera))
        camera["valid"] = False
        with self.assertRaises(BridgeError):
            red_detection(camera)

    def test_fragmented_response_and_request_epoch(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        received, errors = [], []
        def engine():
            try:
                with listener.accept()[0] as conn, conn.makefile("rb") as stream:
                    for _ in range(2):
                        request = json.loads(stream.readline())
                        received.append(request)
                        response = (json.dumps(dict(ok=True, id=request["id"], observation=sample())) + "\n").encode()
                        for offset in range(0, len(response), 13):
                            conn.sendall(response[offset:offset + 13])
            except Exception as error:
                errors.append(error)
        thread = threading.Thread(target=engine, daemon=True)
        thread.start()
        client = GodotBridge(port=listener.getsockname()[1])
        try:
            client.step(.1, .2, .3)
        finally:
            client.close()
            thread.join(3)
            listener.close()
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors)
        self.assertEqual([r["op"] for r in received], ["observe", "step"])
        self.assertEqual(received[1]["epoch"], 3)
        self.assertEqual(received[1]["velocity"], [.1, .2, .3])

    def test_response_id_mismatch(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        def engine():
            with listener.accept()[0] as conn:
                conn.recv(4096)
                conn.sendall(b'{"ok":true,"id":999}\n')
        thread = threading.Thread(target=engine, daemon=True)
        thread.start()
        client = GodotBridge(port=listener.getsockname()[1])
        try:
            with self.assertRaisesRegex(BridgeError, "ID mismatch"):
                client.observe()
        finally:
            client.close()
            thread.join(3)
            listener.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
