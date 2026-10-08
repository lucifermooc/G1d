"""g1d_semantic 的离线测试：假编码器 + 内存向量库，不连机器人、不加载模型。

需要 qdrant-client，以及能 import semantic_map_ros（source ROS 工作区或设置 SEMANTIC_MAP_ROS_SRC）。
"""
import hashlib
import json
import math
import os
import tempfile
import unittest

import numpy as np

os.environ.setdefault("SEMANTIC_LOG", os.path.join(tempfile.mkdtemp(), "log.jsonl"))

from g1d_g1_core import numeric_poi_id  # noqa: E402
from g1d_semantic import SemanticService, pois_to_rows  # noqa: E402


def fake_encode(texts):
    """按字符二元组哈希成 1024 维向量（与 BGE-M3 同维度，SemanticIndex 固定 1024），意思相近 ≈ 共享字的句子分数高。"""
    out = []
    for t in texts:
        v = np.zeros(1024)
        for a, b in zip(t, t[1:] + " "):
            v[int(hashlib.md5((a + b).encode()).hexdigest(), 16) % 1024] += 1
        n = np.linalg.norm(v)
        out.append((v / n if n else v).tolist())
    return out


def poi(pid, name, desc="", x=0.0, y=0.0, yaw=0.0):
    return {"id": pid, "pose": {"x": x, "y": y, "yaw": yaw},
            "metadata": {"display_name": name, "description": desc}}


class SemanticTests(unittest.TestCase):
    def setUp(self):
        from qdrant_client import QdrantClient
        self.pois = [poi("u-water", "茶水间", "饮水机 接水 喝水 泡茶", 3.0, 1.0, math.pi / 2),
                     poi("u-meet", "会议室", "开会 汇报 投影", -2.0, 4.0),
                     {"id": "no-pose", "metadata": {"display_name": "没有位置"}}]
        self.nav = []
        self.svc = SemanticService(lambda: self.pois, lambda x, y, yaw: self.nav.append((x, y, yaw)),
                                   encode=fake_encode, client=QdrantClient(":memory:"))
        self.svc.threshold = 0.2

    def test_rows_use_g1d_ids_and_description(self):
        rows = pois_to_rows(self.pois)
        self.assertEqual(len(rows), 2)                       # 没有 pose 的 POI 跳过
        self.assertEqual(rows[0]["id"], numeric_poi_id("u-water"))
        p = json.loads(rows[0]["point_list"])
        self.assertEqual((p["name"], p["description"]), ("茶水间", "饮水机 接水 喝水 泡茶"))

    def test_search_does_not_navigate(self):
        r = self.svc.handle("search", "我想喝水", go=False)
        self.assertTrue(r["found"])
        self.assertEqual(r["best"]["name"], "茶水间")
        self.assertFalse(r["navigating"])
        self.assertEqual(self.nav, [])

    def test_go_navigates_to_best(self):
        r = self.svc.handle("go", json.dumps({"text": "去接水喝水", "source": "asr"}), go=True)
        self.assertTrue(r["navigating"])
        self.assertEqual(r["source"], "asr")
        x, y, yaw = self.nav[-1]
        self.assertEqual((x, y), (3.0, 1.0))
        self.assertAlmostEqual(yaw, math.pi / 2, places=3)

    def test_not_found_never_navigates(self):
        r = self.svc.handle("go", "今天股票涨了吗", go=True)
        self.assertFalse(r["found"])
        self.assertEqual(self.nav, [])

    def test_poi_change_triggers_rebuild(self):
        self.assertTrue(self.svc.handle("search", "开会", go=False)["rebuilt"])
        self.assertFalse(self.svc.handle("search", "开会", go=False)["rebuilt"])
        self.pois[1]["metadata"]["description"] = "打印 复印"
        self.assertTrue(self.svc.handle("search", "开会", go=False)["rebuilt"])

    def test_every_request_is_logged(self):
        self.svc.handle("search", "我想喝水", go=False)
        with self.assertRaises(ValueError):
            self.svc.handle("search", "  ", go=False)
        hist = self.svc.history()
        self.assertEqual([h["ok"] for h in hist[-2:]], [True, False])
        self.assertEqual(hist[-2]["best"], "茶水间")

    def test_chassis_error_is_reported(self):
        def boom():
            raise RuntimeError("未连接底盘")
        self.svc.get_pois = boom
        with self.assertRaises(RuntimeError):
            self.svc.handle("search", "喝水", go=False)
        self.assertEqual(self.svc.history()[-1]["error"], "未连接底盘")


if __name__ == "__main__":
    unittest.main()
