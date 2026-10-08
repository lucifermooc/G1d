"""Offline contract checks; never opens a robot connection."""

import base64
import io
import json
import struct
import tempfile
import unittest
from types import SimpleNamespace

from PIL import Image

from g1d_g1_core import BridgeError, G1DCompat, laser_to_scan, map_to_g1_image


def explore(cells=b"\x00\x64\x01\x64"):
    return struct.pack("<ffIIf", 1.5, -2.0, 2, 2, 0.05) + bytes(12) + struct.pack("<I", 4) + cells


class FakeRobot:
    def __init__(self):
        self.calls = []
        self.current = {"action_id": 7, "state": {"status": 1, "result": 0}}
        self.next_action_id = 7

    def binary(self, path):
        self.calls.append(("GET_BINARY", path, None))
        return explore() if path.endswith("explore") else b"STCM"

    def json(self, method, path, **kwargs):
        self.calls.append((method, path, kwargs.get("json")))
        if method == "POST" and path.endswith("/actions"):
            result = {"action_id": self.next_action_id}
            self.current = {"action_id": self.next_action_id,
                            "state": {"status": 1, "result": 0}}
            self.next_action_id += 1
            return result
        if path.endswith(":current") or path.startswith("/api/core/motion/v1/actions/"):
            return self.current
        raise AssertionError((method, path))

    def request(self, method, path, **kwargs):
        self.calls.append((method, path, kwargs.get("json")))
        if method == "GET" and path.startswith("/api/core/motion/v1/actions/"):
            return SimpleNamespace(json=lambda: self.current)
        return SimpleNamespace()


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.robot = FakeRobot()
        self.bridge = G1DCompat(self.robot, self.temp.name)

    def test_map_bootstrap_and_jpeg_orientation(self):
        self.bridge.discover_current_map()
        self.bridge.discover_current_map()
        self.assertEqual(len(self.bridge.list_maps()), 1)
        self.assertEqual(self.bridge.current_map_id(), 1)
        self.assertEqual(self.robot.calls.count(("GET_BINARY", self.bridge.MAP_PATH, None)), 1)
        m = map_to_g1_image(explore())
        self.assertEqual((m["x0"], m["y0"], m["width"], m["height"]),
                         (1.5, -2.0, 2, 2))
        image = Image.open(io.BytesIO(base64.b64decode(m["base64"])))
        self.assertEqual(image.size, (2, 2))
        self.assertLess(image.getpixel((0, 0)), image.getpixel((0, 1)))
        self.assertRaises(BridgeError, map_to_g1_image, explore()[:-1])

    def test_navigation_status_and_ownership(self):
        self.bridge.navigate(2.0, 3.0, 0.7)
        sent = self.robot.calls[-1]
        self.assertEqual(sent[2]["options"]["target"], {"x": 2.0, "y": 3.0, "z": 0})
        self.assertEqual(sent[2]["options"]["move_options"]["yaw"], 0.7)
        self.robot.current["state"] = {"status": 4, "result": 0}
        self.assertEqual(self.bridge.task_status(), (2, 0))
        self.bridge.patrol([{"x": 1, "y": 2}, {"x": 3, "y": 4}])
        self.assertEqual(len(self.robot.calls[-1][2]["options"]["targets"]), 2)
        self.robot.current["action_id"] = 99
        self.assertRaises(BridgeError, self.bridge.cancel)
        self.assertEqual(self.bridge.task_status(), (1, 1))
        self.robot.current["action_id"] = 8
        self.bridge.pause()
        self.assertEqual(self.bridge.task_status(), (4, 1))
        delete_count = sum(c[:2] == ("DELETE", self.bridge.CURRENT_ACTION) for c in self.robot.calls)
        self.bridge.cancel()
        self.assertEqual(self.bridge.task_status(), (5, 1))
        self.assertEqual(sum(c[:2] == ("DELETE", self.bridge.CURRENT_ACTION)
                             for c in self.robot.calls), delete_count)

    def test_scan_and_route_storage(self):
        low, step, ranges = laser_to_scan({"laser_points": [
            {"angle": 0.0, "distance": 1.0, "valid": True},
            {"angle": 1.0, "distance": 3.0, "valid": False},
            {"angle": 2.0, "distance": 2.0, "valid": True}]})
        self.assertEqual((low, step, ranges), (0.0, 1.0, [1.0, 0.0, 2.0]))
        route = json.dumps([{"x": 1, "y": 2}])
        self.bridge.add_route(1, "map", route)
        restarted = G1DCompat(self.robot, self.temp.name)
        self.assertEqual(restarted.routes(1)[0]["point_list"], route)


if __name__ == "__main__":
    unittest.main()
