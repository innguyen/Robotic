import copy
import math
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from sar_benchmark.contracts import ContractError, canonical
from sar_benchmark.scenario_contract import validate_scenario
from sar_benchmark.scenarios import scenario
from sar_benchmark.transport import HostClient, receive_exact

from test_pipeline import observation


class IntegrationTests(unittest.TestCase):
    def test_source_snapshot_excludes_generated_editor_state(self):
        from sar_benchmark.runner import source_hashes
        names = source_hashes()
        self.assertTrue(names)
        self.assertFalse(any("/Saved/" in name or "/Intermediate/" in name or "__pycache__" in name for name in names))

    def test_scenarios_validate_and_private_canary_rejected(self):
        for kind in ("clean", "occluded", "absent", "multi", "sensor_black"):
            validate_scenario(scenario(kind))
        for mutation in (lambda c: c.update(seed=True), lambda c: c.update(teleport=[1, 1, 1]),
                         lambda c: c["victims"][0].update(anchor_m=[math.nan, 0, 0]),
                         lambda c: c["boxes"][0].update(size_m=[0, 1, 1])):
            config = scenario()
            mutation(config)
            with self.assertRaises(ContractError):
                validate_scenario(config)

    def test_fragmented_host_transport_and_error(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        def server():
            connection, _ = listener.accept()
            with connection:
                size = struct.unpack("!I", receive_exact(connection, 4))[0]
                request = receive_exact(connection, size)
                self.assertIn(b'"token":"secret"', request)
                payload = canonical({"sequence": 0, "result": {"ok": True}})
                data = struct.pack("!I", len(payload)) + payload
                for piece in (data[:1], data[1:7], data[7:]):
                    connection.sendall(piece)
        worker = threading.Thread(target=server)
        worker.start()
        client = HostClient(listener.getsockname()[1], "secret")
        self.assertEqual(client.call("hello"), {"ok": True})
        client.close()
        worker.join(2)
        listener.close()

    def test_real_policy_worker_uses_public_json_only(self):
        from sar_benchmark.runner import PolicyProcess
        worker = PolicyProcess("sweep", 0)
        try:
            response = worker.act(observation())
            self.assertEqual(response["epoch"], 1)
            self.assertEqual(response["reports"], [])
        finally:
            result = worker.close()
        self.assertEqual(result["exit_code"], 0)

    def test_policy_watchdog_covers_worker_that_never_reads_large_input(self):
        from sar_benchmark.runner import PolicyProcess
        import subprocess
        real_popen = subprocess.Popen
        def sleeper(command,**kwargs):
            return real_popen([sys.executable,"-c","import time; time.sleep(60)"],**kwargs)
        with patch("sar_benchmark.runner.subprocess.Popen",side_effect=sleeper):
            worker = PolicyProcess("sweep",0)
        errors = []
        def call():
            try: worker.act({"large_input":"x"*2_000_000},timeout=.2)
            except Exception as error: errors.append(error)
        started = time.monotonic()
        task = threading.Thread(target=call,daemon=True)
        task.start();task.join(4)
        stalled = task.is_alive()
        if stalled:
            worker.process.kill();worker.process.wait(timeout=3)
            task.join(2)
        try:
            self.assertFalse(stalled,"stdin write escaped the watchdog")
            self.assertIsInstance(errors[0],TimeoutError)
            self.assertLess(time.monotonic()-started,4)
        finally:
            worker.close()

    def test_optional_core_search_consumes_public_sensors(self):
        try:
            import numpy
        except ImportError:
            self.skipTest("optional core policy requires existing NumPy environment")
        from sar_benchmark.core_policy import CoreSearchPolicy
        policy = CoreSearchPolicy(0)
        try:
            for tick in range(3):
                obs = observation(tick=tick)
                action = policy.act(obs)
                self.assertEqual(len(action["twist"]), 3)
                self.assertTrue(all(math.isfinite(v) for v in action["twist"]))
                self.assertEqual(action["reports"], [])
        finally:
            policy.close()

    def test_optional_repo_onnx_inference_without_hardware(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("optional ONNX adapter requires existing OpenCV environment")
        from sar_benchmark.perception import OnnxPersonDetector
        model = Path(__file__).resolve().parents[2] / "models/yolov5n.onnx"
        detector = OnnxPersonDetector(model)
        self.assertEqual(detector.detect([0] * (80 * 60 * 3), 80, 60), [])
        self.assertEqual(len(detector.model_sha256), 64)


if __name__ == "__main__":
    unittest.main()
