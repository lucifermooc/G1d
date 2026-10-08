"""Offline contract checks; never opens a robot connection."""

import base64
import io
import json
import os
import struct
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import requests
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


class DockRobot(FakeRobot):
    """FakeRobot with G1D homedocks, robot pose and power status."""

    DOCKS = "/api/core/slam/v1/homedocks"

    def __init__(self):
        super().__init__()
        self.docks = []
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.docking_status = "on_dock"
        self.home = None

    def json(self, method, path, **kwargs):
        body = kwargs.get("json")
        if path == self.DOCKS and method == "GET":
            self.calls.append((method, path, body))
            return [dict(d) for d in self.docks]
        if path == self.DOCKS and method == "POST":
            self.calls.append((method, path, body))
            self.docks.append(body)
            return True
        if path == self.DOCKS + "/:register":
            self.calls.append((method, path, body))
            entry = {"id": f"reg{len(self.calls)}", "pose": dict(self.pose), "metadata": body["metadata"]}
            self.docks.append(entry)
            return entry
        if path == "/api/core/slam/v1/homepose":
            self.calls.append((method, path, body))
            if method == "PUT":
                self.home = body
                return True
            if self.home is None:
                raise requests.HTTPError("404 Home dock not found")
            return dict(self.home)
        if path == "/api/core/slam/v1/localization/pose":
            return dict(self.pose)
        if path == "/api/core/system/v1/power/status":
            return {"dockingStatus": self.docking_status, "batteryPercentage": 50, "isCharging": False}
        return super().json(method, path, **kwargs)

    def request(self, method, path, **kwargs):
        if path.startswith(self.DOCKS + "/"):
            dock_id = path.rsplit("/", 1)[1]
            if method == "DELETE":
                self.docks = [d for d in self.docks if d["id"] != dock_id]
            elif method == "PUT":
                dock = next(d for d in self.docks if d["id"] == dock_id)
                dock.update(kwargs["json"])
        elif path == self.DOCKS and method == "DELETE":
            self.docks = []
        return super().request(method, path, **kwargs)


class DockTests(unittest.TestCase):
    SAVE = ("POST", "/api/multi-floor/map/v1/stcm/:save", None)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        env = mock.patch.dict(os.environ)
        env.start()
        self.addCleanup(env.stop)
        os.environ.pop("G1D_LOW_BATTERY", None)
        self.robot = DockRobot()
        self.bridge = G1DCompat(self.robot, self.temp.name)
        self.bridge.discover_current_map()
        self.snapshot = Path(self.temp.name) / "1.stcm"

    def place(self, x, y, yaw=0.0, name="充电桩"):
        return self.bridge.add_dock(x, y, yaw, name)

    def test_dock_edits_refresh_map_snapshot(self):
        # 桩和 POI 一样在 G1D 的 STCM 里：改完要刷新快照，否则切换地图再切回来桩就没了
        for edit in (lambda: self.place(1, 2),
                     lambda: self.bridge.register_dock(),
                     lambda: self.bridge.delete_dock(self.robot.docks[0]["id"]),
                     self.bridge.clear_docks):
            self.snapshot.write_bytes(b"OLD")
            self.robot.calls.clear()
            edit()
            self.assertIn(self.SAVE, self.robot.calls)
            self.assertEqual(self.snapshot.read_bytes(), b"STCM")
        self.assertEqual(self.robot.docks, [])

    def test_placed_dock_is_provisional_until_removed(self):
        dock_id = self.place(1, 2, 0.5, "二号桩")
        sent = self.robot.docks[0]
        self.assertEqual((sent["id"], sent["pose"], sent["metadata"]),
                         (dock_id, {"x": 1.0, "y": 2.0, "yaw": 0.5}, {"display_name": "二号桩"}))
        self.assertEqual([d["provisional"] for d in self.bridge.docks()], [True])
        restarted = G1DCompat(self.robot, self.temp.name)  # 标记存在 catalog 里，适配层重启后还在
        self.assertTrue(restarted.docks()[0]["provisional"])
        self.bridge.delete_dock(dock_id)
        self.assertEqual(self.bridge.catalog["provisional_docks"], [])

    def test_dock_goes_to_nearest_or_chosen_dock(self):
        self.assertRaises(BridgeError, self.bridge.start_dock)
        a = self.place(0, 0)
        self.robot.calls.clear()
        _, target = self.bridge.start_dock()
        self.assertEqual(target, a)
        self.assertNotIn("/api/core/slam/v1/homepose", [c[1] for c in self.robot.calls])  # 只有一个桩不用选
        b = self.place(5, 0, 1.0)
        self.robot.pose = {"x": 4.0, "y": 0.5, "yaw": 0.0}
        self.robot.calls.clear()
        _, target = self.bridge.start_dock()
        self.assertEqual(target, b)
        homepose = next(c for c in self.robot.calls if c[1] == "/api/core/slam/v1/homepose")
        self.assertEqual(homepose[2], {"x": 5.0, "y": 0.0, "z": 0, "yaw": 1.0, "pitch": 0, "roll": 0})
        self.assertEqual(self.robot.calls[-1][2]["action_name"], "slamtec.agent.actions.GoHomeAction")
        self.assertEqual(self.bridge.start_dock(a)[1], a)
        self.assertRaises(BridgeError, self.bridge.start_dock, "missing")
        self.assertEqual(self.bridge.dock_pose(), {"x": 5.0, "y": 0.0, "yaw": 1.0})

    def test_current_dock_follows_home_pose(self):
        # 当前使用的桩 = G1D 的 home pose；只有一个桩时就是它；回桩会把去的那个设为当前
        self.assertEqual(self.bridge.list_docks(), [])
        a = self.place(0, 0)
        self.assertEqual([d["current"] for d in self.bridge.list_docks()], [True])
        b = self.place(5, 0)
        self.assertEqual([d["current"] for d in self.bridge.list_docks()], [False, False])  # 还没选过
        self.bridge.use_dock(b)
        self.assertEqual({d["id"]: d["current"] for d in self.bridge.list_docks()}, {a: False, b: True})
        self.bridge.start_dock(a)
        self.assertEqual({d["id"]: d["current"] for d in self.bridge.list_docks()}, {a: True, b: False})
        self.assertRaises(BridgeError, self.bridge.use_dock, "missing")

    def test_low_battery_ends_task_and_returns_to_current_dock(self):
        main = self.place(0, 0, name="主桩")
        self.place(5, 0, name="二号桩")
        self.bridge.use_dock(main)
        self.robot.pose = {"x": 4.9, "y": 0.0, "yaw": 0.0}  # 离二号桩更近，但要回的是当前使用的主桩
        self.bridge.navigate(3.0, 3.0)
        nav_id = self.bridge.task["action_id"]
        low = {"batteryPercentage": 8, "isCharging": False, "dockingStatus": "not_on_dock"}
        self.assertIsNone(self.bridge.low_battery_dock(dict(low, batteryPercentage=10), 0))  # 不低于阈值
        self.assertIsNone(self.bridge.low_battery_dock(dict(low, isCharging=True), 0))      # 在充电
        self.assertIsNone(self.bridge.low_battery_dock(dict(low, dockingStatus="on_dock"), 0))
        self.robot.calls.clear()
        action_id, dock_id = self.bridge.low_battery_dock(low, 0)
        self.assertEqual(dock_id, main)
        sent = [c[:2] for c in self.robot.calls]
        self.assertLess(sent.index(("DELETE", self.bridge.CURRENT_ACTION)),  # 先结束导航，再回桩
                        sent.index(("POST", self.bridge.ACTION_PATH)))
        self.assertEqual(self.robot.calls[-1][2]["action_name"], "slamtec.agent.actions.GoHomeAction")
        self.assertNotEqual(action_id, nav_id)
        self.assertEqual(self.bridge.task["status"], 5)  # 导航任务报告为已取消，巡逻也不会续圈

        # 一直没充上电：每 LOW_BATTERY_RETRY 秒重试，最多 LOW_BATTERY_ATTEMPTS 次
        retry = self.bridge.LOW_BATTERY_RETRY
        self.assertIsNone(self.bridge.low_battery_dock(low, retry / 2))
        self.assertIsNotNone(self.bridge.low_battery_dock(low, retry))
        self.assertIsNotNone(self.bridge.low_battery_dock(low, 2 * retry))
        self.assertIsNone(self.bridge.low_battery_dock(low, 10 * retry))
        # 充上电（或电量回升到阈值 + 2% 以上）后清零，下次低电量会重新触发
        self.bridge.low_battery_dock(dict(low, isCharging=True), 10 * retry)
        self.assertIsNotNone(self.bridge.low_battery_dock(low, 10 * retry))

    def test_low_battery_threshold_setting(self):
        self.assertEqual(self.bridge.low_battery_percent(), 10.0)  # 默认 10%
        self.bridge.low_battery_percent(20)
        self.assertEqual(G1DCompat(self.robot, self.temp.name).low_battery_percent(), 20.0)  # 存在 catalog 里
        self.assertRaises(BridgeError, self.bridge.low_battery_percent, 100)
        with mock.patch.dict(os.environ, {"G1D_LOW_BATTERY": "15"}):
            self.assertEqual(G1DCompat(DockRobot(), tempfile.mkdtemp(dir=self.temp.name)).low_battery_percent(), 15.0)
        low = {"batteryPercentage": 1, "isCharging": False, "dockingStatus": "not_on_dock"}
        self.place(0, 0)
        self.bridge.low_battery_percent(0)  # 0 = 关闭
        self.assertIsNone(self.bridge.low_battery_dock(low, 0))
        self.bridge.low_battery_percent(10)
        self.bridge.clear_docks()
        self.assertRaises(BridgeError, self.bridge.low_battery_dock, low, 0)  # 没有桩：报错，由适配层记日志

    def test_first_docking_replaces_estimated_pose(self):
        dock_id = self.place(2.0, 1.0, 0.0, "二号桩")
        self.robot.pose = {"x": 2.3, "y": 1.2, "yaw": 3.1}  # 真实上桩位置
        self.robot.docking_status = "not_on_dock"
        self.assertIsNone(self.bridge.finish_dock(dock_id))  # 动作说成功但不在桩上：不改
        self.robot.docking_status = "on_dock"
        updated = self.bridge.finish_dock(dock_id)
        self.assertEqual(updated["id"], dock_id)
        self.assertEqual(self.robot.docks, [{"id": dock_id, "pose": {"x": 2.3, "y": 1.2, "yaw": 3.1},
                                             "metadata": {"display_name": "二号桩"}}])
        self.assertFalse(self.bridge.docks()[0]["provisional"])
        self.robot.calls.clear()
        self.assertIsNone(self.bridge.finish_dock(dock_id))  # 已是真实位置：以后回桩不再改
        self.assertNotIn(self.SAVE, self.robot.calls)

    def test_register_merges_duplicates_of_the_same_dock(self):
        far = self.place(10, 10)
        near = self.place(0.4, 0, 0, "主桩")
        self.robot.docks.append({"id": "fw", "pose": {"x": 0.1, "y": 0, "yaw": 0},  # 固件自己登记的重复桩
                                 "metadata": {"display_name": "auto"}})
        entry = self.bridge.register_dock(keep_id=near)
        self.assertEqual(entry["id"], near)
        self.assertEqual(sorted(d["id"] for d in self.robot.docks), sorted([far, near]))
        self.assertEqual(entry["metadata"]["display_name"], "主桩")
        # 远处没有桩时注册成新桩，名字用传进来的
        self.robot.pose = {"x": -5.0, "y": 0.0, "yaw": 0.0}
        entry = self.bridge.register_dock("三号桩")
        self.assertEqual(len(self.robot.docks), 3)
        self.assertEqual(entry["metadata"], {"display_name": "三号桩"})


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
