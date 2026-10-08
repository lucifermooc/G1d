"""Offline contract checks; never opens a robot connection."""

import base64
import io
import json
import struct
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from PIL import Image

from g1d_g1_core import G1_MAP_PALETTE, BridgeError, G1DCompat, laser_to_scan, map_to_g1_image


def explore(cells=b"\x00\x64\x01\x64"):
    return struct.pack("<ffIIf", 1.5, -2.0, 2, 2, 0.05) + bytes(12) + struct.pack("<I", 4) + cells


class FakeRobot:
    def __init__(self):
        self.calls = []
        self.current = {"action_id": 7, "state": {"status": 1, "result": 0}}
        self.next_action_id = 7
        self.slam = {"mapping": True, "localization": True}  # G1D factory state seen on the real robot

    def binary(self, path):
        self.calls.append(("GET_BINARY", path, None))
        return explore() if path.endswith("explore") else b"STCM"

    def json(self, method, path, **kwargs):
        self.calls.append((method, path, kwargs.get("json")))
        if method == "GET" and path.startswith("/api/core/slam/v1/") and path.endswith("/:enable"):
            return self.slam[path.split("/")[-2]]
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
        if method == "PUT" and path.startswith("/api/core/slam/v1/") and path.endswith("/:enable"):
            self.slam[path.split("/")[-2]] = kwargs["json"]["enable"]
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
        # G1 colours (unknown 0 / obstacle 1 / free 2), nearest match because JPEG is lossy; image rows are flipped
        nearest = lambda rgb: min(range(3), key=lambda k: sum(  # noqa: E731
            (a - b) ** 2 for a, b in zip(rgb, G1_MAP_PALETTE[3 * k:3 * k + 3])))
        self.assertEqual([nearest(image.getpixel(uv)) for uv in ((0, 0), (1, 0), (0, 1), (1, 1))], [1, 2, 0, 2])
        self.assertRaises(BridgeError, map_to_g1_image, explore()[:-1])

    def test_robot_status_follows_g1d_switches_and_mode_set(self):
        self.assertEqual(self.bridge.robot_status(), "mapping+idle")
        self.bridge.set_mode("patrol")
        self.assertEqual(self.bridge.robot_status(), "localization+patrol")
        self.bridge.set_mode("remote_control")
        self.assertEqual(self.bridge.robot_status(), "localization+remote_control")
        self.bridge.set_mode("mapping")
        self.assertEqual(self.bridge.robot_status(), "mapping+idle")
        self.assertRaises(BridgeError, self.bridge.set_mode, "dance")
        self.assertEqual(self.bridge.robot_status(), "mapping+idle")
        self.robot.slam = {"mapping": False, "localization": False}
        self.assertEqual(self.bridge.robot_status(), "idle+idle")

    def test_new_mapping_backs_up_then_clears_and_localization_restores_current_map(self):
        stcm_put = ("PUT", self.bridge.STCM_PATH)
        self.bridge.discover_current_map()  # map 1 = what the robot holds
        self.bridge.set_mode("continue_mapping")
        self.assertNotIn(("DELETE", "/api/core/slam/v1/maps", None), self.robot.calls)
        n = len(self.robot.calls)
        self.bridge.set_mode("mapping")
        self.assertEqual([c[:2] for c in self.robot.calls[n:]],
                         [("GET_BINARY", self.bridge.MAP_PATH), ("GET_BINARY", self.bridge.STCM_PATH),
                          ("DELETE", "/api/core/slam/v1/maps"), ("PUT", "/api/core/slam/v1/mapping/:enable")])
        backup = sorted(p.suffix for p in (Path(self.temp.name) / "backup").iterdir())
        self.assertEqual(backup, [".explore", ".stcm"])
        # Cancel the new map: localizing puts map 1 back first, once.
        n = len(self.robot.calls)
        self.bridge.set_mode("idle")
        self.bridge.set_mode("localization")
        self.bridge.set_mode("patrol")
        self.assertEqual([c[:2] for c in self.robot.calls[n:]].count(stcm_put), 1)

    def test_save_ends_mapping_and_saved_map_stays_loaded(self):
        stcm_put = ("PUT", self.bridge.STCM_PATH)
        self.bridge.discover_current_map()
        self.bridge.set_mode("mapping")
        self.bridge.save_map()
        self.assertFalse(self.robot.slam["mapping"])
        new_id = self.bridge.add_map("二楼")
        self.assertEqual(self.bridge.current_map_id(), new_id)
        n = len(self.robot.calls)
        self.bridge.set_mode("localization")
        self.assertNotIn(stcm_put, [c[:2] for c in self.robot.calls[n:]])  # the new map is already on the robot
        self.bridge.set_map(1)
        self.assertEqual(self.robot.calls[-1][:2], stcm_put)
        self.assertEqual(self.bridge.current_map_id(), 1)

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

    def test_point_description_round_trip(self):
        # 语义地图用的 description 存在 POI metadata 里，增、改、查都要带上
        poi = {"id": "u1", "pose": {"x": 1.0, "y": 2.0, "yaw": 0.0},
               "metadata": {"display_name": "茶水间", "description": "接水 喝水"}}
        json_orig = self.robot.json
        self.robot.json = lambda method, path, **kw: [poi] if path == self.bridge.POI_PATH else json_orig(method, path, **kw)
        row = self.bridge.list_points()[0]
        listed = json.loads(row["point_list"])
        self.assertEqual((listed["name"], listed["description"]), ("茶水间", "接水 喝水"))

        new_point = {"position": {"x": 3, "y": 4}, "orientation": {"w": 1}, "name": "前台", "description": "访客登记"}
        self.bridge.add_point(new_point)
        self.assertEqual(self.robot.calls[-1][2]["metadata"], {"display_name": "前台", "description": "访客登记"})

        self.bridge.update_point(row["id"], dict(new_point, description="改过的描述"))
        self.assertEqual(self.robot.calls[-1][2]["metadata"]["description"], "改过的描述")

        no_desc = {k: v for k, v in new_point.items() if k != "description"}
        self.bridge.update_point(row["id"], no_desc)      # 更新时没带描述 → 保留原描述
        self.assertEqual(self.robot.calls[-1][2]["metadata"]["description"], "改过的描述")

        poi["metadata"] = {"display_name": "老点位"}  # 旧点位没有 description 字段
        self.assertEqual(json.loads(self.bridge.list_points()[0]["point_list"])["description"], "")

    def test_patrol_loops_until_cancelled(self):
        # 和 G1 一样：走完一圈不结束，自动从第一个点再走；单段失败从最近的点重试，连续失败才放弃
        route = [{"x": 1, "y": 2}, {"x": 3, "y": 4}, {"x": 5, "y": 6}]
        lap = lambda: [(t["x"], t["y"]) for t in self.robot.calls[-1][2]["options"]["targets"]]  # noqa: E731
        self.bridge.pose = lambda: {"x": 3.1, "y": 4.1}
        self.bridge.patrol(route)
        for _ in range(3):
            self.robot.current["state"] = {"status": 4, "result": 0}
            self.assertEqual(self.bridge.task_status(), (1, 1))
            self.assertEqual(lap(), [(1, 2), (3, 4), (5, 6)])
            self.assertEqual(self.bridge.task["action_id"], self.robot.current["action_id"])

        for _ in range(self.bridge.PATROL_RETRIES):
            self.robot.current["state"] = {"status": 4, "result": -1}
            self.assertEqual(self.bridge.task_status(), (1, 1))
            self.assertEqual(lap(), [(3, 4), (5, 6), (1, 2)])
        self.robot.current["state"] = {"status": 4, "result": 0}  # 一圈成功，失败计数清零
        self.assertEqual(self.bridge.task_status(), (1, 1))
        for _ in range(self.bridge.PATROL_RETRIES):
            self.robot.current["state"] = {"status": 4, "result": -1}
            self.assertEqual(self.bridge.task_status(), (1, 1))
        sent = len(self.robot.calls)
        self.robot.current["state"] = {"status": 4, "result": -1}
        self.assertEqual(self.bridge.task_status(), (3, 1))
        self.assertEqual(len(self.robot.calls), sent + 1)  # 只查了状态，没有再下发

        self.bridge.patrol(route)
        self.robot.current["state"] = {"status": 4, "result": -2}  # 被别处取消：停下，不再续圈
        self.assertEqual(self.bridge.task_status(), (5, 1))
        self.bridge.navigate(1.0, 2.0)
        self.robot.current["state"] = {"status": 4, "result": 0}  # 单点导航仍然到了就结束
        self.assertEqual(self.bridge.task_status(), (2, 0))

    def test_point_and_wall_edits_persist_map(self):
        # POI / 虚拟墙只在 G1D 内存里的地图上：改完要让 G1D 落盘，并刷新 8550 上的 .stcm 快照
        save = ("POST", "/api/multi-floor/map/v1/stcm/:save", None)
        snapshot = Path(self.temp.name) / "1.stcm"
        self.bridge.discover_current_map()
        snapshot.write_bytes(b"OLD")
        self.robot.calls.clear()
        self.bridge.add_point({"position": {"x": 1, "y": 2}, "orientation": {"w": 1}, "name": "前台"})
        self.assertEqual(self.robot.calls[1:], [save, ("GET_BINARY", self.bridge.STCM_PATH, None)])
        self.assertEqual(snapshot.read_bytes(), b"STCM")

        self.robot.json = lambda method, path, **kw: []  # no walls yet
        self.robot.calls.clear()
        self.bridge.set_walls([{"start": {"x": 0, "y": 0}, "end": {"x": 1, "y": 0}}])
        self.assertIn(save, self.robot.calls)

        self.bridge.loaded_id = None  # 新建图进行中：不落盘，交给 save_map
        self.robot.calls.clear()
        self.bridge.add_point({"position": {"x": 1, "y": 2}, "orientation": {"w": 1}})
        self.assertNotIn(save, self.robot.calls)


if __name__ == "__main__":
    unittest.main()
