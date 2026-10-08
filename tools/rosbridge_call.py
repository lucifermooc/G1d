#!/usr/bin/env python3
"""像网页一样通过 rosbridge（ws://localhost:9090）调用语义地图服务，打印结果。只调 search / history，不导航。"""
import json
import sys

from tornado import gen, ioloop, websocket


@gen.coroutine
def main():
    ws = yield websocket.websocket_connect("ws://localhost:9090")
    queries = sys.argv[1:] or ["我想喝水", "我要开会", "有人来访", "今天天气怎么样"]
    for i, q in enumerate(queries):
        ws.write_message(json.dumps({"op": "call_service", "id": f"q{i}", "service": "/semantic_map/search",
                                     "args": {"data": json.dumps({"text": q, "source": "claude-test"}, ensure_ascii=False)}}))
        while True:
            msg = json.loads((yield ws.read_message()))
            if msg.get("id") == f"q{i}":
                break
        v = msg.get("values", {})
        if not v.get("success"):
            print(f"{q} -> 失败 {v.get('message')}")
            continue
        r = json.loads(v["message"])
        top = "，".join(f"{c['name']} {c['score']:.3f}" for c in r["candidates"])
        print(f"{q} -> {'找到 ' + r['best']['name'] if r['found'] else '没找到'}  [{top}]")
    ws.write_message(json.dumps({"op": "call_service", "id": "h", "service": "/semantic_map/history", "args": {}}))
    while True:
        msg = json.loads((yield ws.read_message()))
        if msg.get("id") == "h":
            break
    hist = json.loads(msg["values"]["message"])
    print(f"history: {len(hist)} 条，最后一条 {hist[-1]['raw']} elapsed {hist[-1]['elapsed_ms']} ms")


ioloop.IOLoop.current().run_sync(main, timeout=180)
