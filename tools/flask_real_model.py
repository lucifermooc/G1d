"""用真实 BGE-M3（CPU）测 g1d_semantic：假 POI、不连机器人、导航只记录不下发。"""
import os, sys, tempfile, time
os.environ["SEMANTIC_QDRANT_PATH"] = tempfile.mkdtemp()
os.environ["SEMANTIC_LOG"] = os.path.join(tempfile.mkdtemp(), "log.jsonl")
sys.path.insert(0, os.path.expanduser("~/G1D/slam_ws"))
from g1d_semantic import SemanticService
pois = [{"id": i, "pose": {"x": x, "y": 0.0, "yaw": 0.0}, "metadata": {"display_name": n, "description": d}}
        for i, n, d, x in [("a", "茶水间", "可以接水、喝水、泡茶", 1.0), ("b", "会议室", "开会、讨论", 2.0),
                            ("c", "前台", "访客登记、拿快递", 3.0)]]
nav = []
s = SemanticService(lambda: pois, lambda x, y, yaw: nav.append((x, y)))
t = time.time(); s.rebuild(); print(f"加载+建库 {time.time()-t:.1f}s")
for q in ["口渴了想喝点水", "有人来访", "今天天气怎么样"]:
    r = s.handle("search", q, go=False)
    print(q, "->", r["best"]["name"] if r["found"] else "没找到", [f"{c['name']} {c['score']:.3f}" for c in r["candidates"]])
r = s.handle("go", "我要开会", go=True); print("go 我要开会 ->", r["best"]["name"], "导航目标(只记录):", nav)
