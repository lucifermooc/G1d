"""G1D 底盘控制台（app.py）的语义地图：一句话 → 最相关的 POI → 可选导航。不依赖 ROS。

复用 semantic_map_ros 的核心逻辑（SemanticIndex / DebugLog / Embedder，都是纯 Python），流程与 ROS 节点
semantic_map_server 相同：解析 → 检查文本 → 搜索 → 可选导航 → 写调试记录。
点位来自 G1D POI（名称 = metadata.display_name，描述 = metadata.description，与 g1d_g1_core 一致），
点位 id 用 g1d_g1_core.numeric_poi_id，和 ROS 适配层看到的 id 相同。

环境变量：
  SEMANTIC_MAP_ROS_SRC   semantic_map_ros 源码目录，默认本目录下的 ros2_ws/src/semantic_map_ros（不用 source ROS 工作区）
  SEMANTIC_MODEL         模型目录或名称，默认 ~/models/bge-m3
  SEMANTIC_QDRANT_PATH   向量库目录（本地文件模式），默认 ~/maps/semantic_qdrant_flask
  SEMANTIC_LOG           调试记录文件，默认 ~/maps/semantic_map_flask_log.jsonl
  SEMANTIC_THRESHOLD     找到的最低分，默认 0.52
"""
import json
import os
import sys
import threading
import time
from datetime import datetime

from g1d_g1_core import numeric_poi_id, quaternion_from_yaw

_src = os.path.expanduser(os.environ.get("SEMANTIC_MAP_ROS_SRC") or os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "ros2_ws", "src", "semantic_map_ros"))
if os.path.isdir(_src) and _src not in sys.path:
    sys.path.insert(0, _src)

from semantic_map_ros.core import SemanticIndex, inspect_text, parse_point_rows, parse_request  # noqa: E402
from semantic_map_ros.debug_log import DebugLog, visible  # noqa: E402

MAP_ID = 0  # G1D 只有"当前地图"；点位变化由签名检测，变了就重建


def pois_to_rows(pois):
    """G1D POI 列表 → /get_map_point_list 同格式的行（parse_point_rows 的输入）。"""
    rows = []
    for poi in pois or []:
        pose = poi.get("pose") or {}
        if "x" not in pose or "y" not in pose:
            continue
        meta = poi.get("metadata") or {}
        point = {"position": {"x": pose["x"], "y": pose["y"], "z": 0},
                 "orientation": quaternion_from_yaw(float(pose.get("yaw") or 0)),
                 "name": meta.get("display_name", poi.get("id", "")),
                 "description": meta.get("description", "")}
        rows.append({"id": numeric_poi_id(poi["id"]), "frame_id": "map",
                     "point_list": json.dumps(point, ensure_ascii=False)})
    return rows


class SemanticService:
    """get_pois() 返回 G1D POI 列表；navigate(x, y, yaw) 下发导航。两者都由 app.py 提供（需要已连接底盘）。"""

    def __init__(self, get_pois, navigate, encode=None, client=None):
        self.get_pois = get_pois
        self.navigate = navigate
        self.threshold = float(os.environ.get("SEMANTIC_THRESHOLD", "0.52"))
        self.top_k = 3
        self.debug_log = DebugLog(os.environ.get("SEMANTIC_LOG", "~/maps/semantic_map_flask_log.jsonl"), 200)
        self._encode = encode          # 测试时传假编码器；正常为 None，第一次用到时才加载 BGE-M3
        self._client = client
        self._embedder = None
        self._index = None
        self.lock = threading.Lock()
        self.load_error = None

    # ---------- 懒加载：模型 ~2 GB，第一次搜索才加载 ----------
    def _ensure_loaded(self):
        if self._index is not None:
            return
        if self._encode is None:
            from semantic_map_ros.embedder import Embedder
            self._embedder = Embedder(os.path.expanduser(os.environ.get("SEMANTIC_MODEL", "~/models/bge-m3")), False)
            self._embedder.load()
            self._encode = self._embedder.encode
        if self._client is None:
            from qdrant_client import QdrantClient
            path = os.path.expanduser(os.environ.get("SEMANTIC_QDRANT_PATH", "~/maps/semantic_qdrant_flask"))
            os.makedirs(path, exist_ok=True)
            self._client = QdrantClient(path=path)
        self._index = SemanticIndex(self._client, self._encode, "g1d_semantic")

    def status(self):
        return {"model_loaded": self._index is not None, "load_error": self.load_error,
                "threshold": self.threshold,
                "model": os.path.expanduser(os.environ.get("SEMANTIC_MODEL", "~/models/bge-m3"))}

    def preload(self):
        """后台加载模型（页面打开语义面板时调用），加载失败记下原因。"""
        def run():
            try:
                with self.lock:
                    self._ensure_loaded()
                self.load_error = None
            except Exception as e:
                self.load_error = str(e)
        threading.Thread(target=run, daemon=True).start()

    # ---------- 核心 ----------
    def _ensure_index(self, force=False):
        places = parse_point_rows(pois_to_rows(self.get_pois()))
        rebuilt = False
        if force or not self._index.is_fresh(MAP_ID, places):
            self._index.rebuild(MAP_ID, places)
            rebuilt = True
        return places, rebuilt

    def rebuild(self):
        with self.lock:
            self._ensure_loaded()
            places, _ = self._ensure_index(force=True)
        return {"count": len(places)}

    def _search(self, text):
        with self.lock:
            self._ensure_loaded()
            places, rebuilt = self._ensure_index()
            if not places:
                return {"query": text, "found": False, "best": None, "candidates": [],
                        "rebuilt": rebuilt, "threshold": self.threshold}
            result = self._index.search(MAP_ID, text, self.threshold, self.top_k)
        result.update(rebuilt=rebuilt, threshold=self.threshold)
        return result

    def handle(self, entry, data, go):
        """entry: search / go；data: 一句话或 {"text": ..., "source": ...}。无论成败都写一条调试记录。"""
        t0 = time.monotonic()
        raw, source = parse_request(data, "web-" + entry)
        text, issues = inspect_text(raw)
        record = {"time": datetime.now().isoformat(timespec="milliseconds"), "entry": entry,
                  "source": source, "raw": raw, "raw_visible": visible(raw), "length": len(raw),
                  "issues": issues, "text": text, "go": go}
        try:
            if not text:
                raise ValueError("empty query")
            result = self._search(text)
            result["navigating"] = False
            if go and result["found"]:
                b = result["best"]
                self.navigate(b["x"], b["y"], b["yaw"])
                result["navigating"] = True
            result.update(source=source, issues=issues)
            best = result["best"]
            record.update(ok=True, found=result["found"], best=best["name"] if best else None,
                          threshold=result["threshold"],
                          candidates=[{"id": c["id"], "name": c["name"], "score": c["score"]}
                                      for c in result["candidates"]],
                          rebuilt=result["rebuilt"], navigating=result["navigating"])
            return result
        except Exception as e:
            record.update(ok=False, error=str(e))
            raise
        finally:
            record["elapsed_ms"] = round((time.monotonic() - t0) * 1000)
            self.debug_log.add(record)

    def history(self):
        return self.debug_log.recent()
