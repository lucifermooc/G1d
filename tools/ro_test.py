#!/usr/bin/env python3
"""像网页一样经 rosbridge 测试 G1D 适配层 + 语义地图：G1D_HOST=192.168.128.59 python3 ro_test.py [句子...]
（默认 localhost）。只调用只读服务、只订阅话题；
不调用 /semantic_map/go、/mode_set、/nav_to_pose、/patrol_path、/cmd_vel_remote_ctrl 等任何会让机器人动或改状态的接口。"""
import base64
import json
import os
import socket
import struct
import sys
import time

FORBIDDEN = ("/semantic_map/go", "/mode_set", "/nav_to_pose", "/patrol_path", "/goal_pose_", "/cmd_vel",
             "/cmd_dock", "/initialpose", "/patrol_control", "/set_current_map_id", "/stop_patrol")


class WS:
    """最小 WebSocket 客户端（只支持文本帧，够 rosbridge 用）。"""

    def __init__(self, host="localhost", port=9090):
        self.s = socket.create_connection((host, port), timeout=30)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall((f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        head = b""
        while b"\r\n\r\n" not in head:
            head += self.s.recv(1)
        assert b" 101 " in head.split(b"\r\n")[0], head
        self.buf = b""

    def _read(self, n):
        while len(self.buf) < n:
            chunk = self.s.recv(65536)
            if not chunk:
                raise ConnectionError("closed")
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def send(self, obj):
        data = json.dumps(obj, ensure_ascii=False).encode()
        mask = os.urandom(4)
        n = len(data)
        hdr = bytes([0x81]) + (bytes([0x80 | n]) if n < 126 else
                               bytes([0x80 | 126]) + struct.pack(">H", n) if n < 65536 else
                               bytes([0x80 | 127]) + struct.pack(">Q", n))
        self.s.sendall(hdr + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def recv(self):
        msg = b""
        while True:
            b0, b1 = self._read(2)
            n = b1 & 0x7F
            if n == 126:
                n = struct.unpack(">H", self._read(2))[0]
            elif n == 127:
                n = struct.unpack(">Q", self._read(8))[0]
            payload = self._read(n)
            op = b0 & 0x0F
            if op == 0x9:          # ping -> 忽略
                continue
            msg += payload
            if b0 & 0x80:
                return json.loads(msg)


ws = WS(os.environ.get("G1D_HOST", "localhost"))
seq = 0


def call(service, args=None, timeout=60):
    global seq
    assert not service.startswith(FORBIDDEN), service
    seq += 1
    ws.send({"op": "call_service", "id": f"c{seq}", "service": service, "args": args or {}})
    t0 = time.time()
    while True:
        m = ws.recv()
        if m.get("op") == "service_response" and m.get("id") == f"c{seq}":
            return m.get("result", True), m.get("values", {}), time.time() - t0
        if time.time() - t0 > timeout:
            raise TimeoutError(service)


def one_message(topic, typ, timeout=10):
    ws.send({"op": "subscribe", "id": f"s-{topic}", "topic": topic, "type": typ})
    t0 = time.time()
    try:
        while time.time() - t0 < timeout:
            m = ws.recv()
            if m.get("op") == "publish" and m.get("topic") == topic:
                return m["msg"]
    finally:
        ws.send({"op": "unsubscribe", "id": f"s-{topic}", "topic": topic})
    return None


def show(name, ok, text):
    print(f"[{'OK ' if ok else 'ERR'}] {name}: {text}")


ok, v, dt = call("/get_map_list")
maps = json.loads(v.get("map_list") or "[]")
show("/get_map_list", ok and v.get("success"), f"{maps} ({dt:.1f}s)")

ok, v, dt = call("/get_current_map_id")
map_id = v.get("map_id", 0)
show("/get_current_map_id", ok and v.get("success"), f"id={map_id} name={v.get('map_name')!r}")

ok, v, dt = call("/get_map_image", {"id": map_id})
info = (v.get("map") or {}).get("info", {})
show("/get_map_image", ok and v.get("success"),
     f"{info.get('width')}x{info.get('height')} res={info.get('resolution')} "
     f"origin=({info.get('origin', {}).get('position', {}).get('x')}, "
     f"{info.get('origin', {}).get('position', {}).get('y')}) data={len((v.get('map') or {}).get('data', []))}B ({dt:.1f}s)")

# 地图图片配色：按离 G1 三种颜色（未探索 / 障碍 / 空地）哪个最近统计像素
try:
    import io
    from PIL import Image
    d = (v.get("map") or {}).get("data", [])
    raw = base64.b64decode(bytes(d) if isinstance(d, list) else base64.b64decode(d))
    img = Image.open(io.BytesIO(raw)).convert("RGB")
    pal = {"未探索": (82, 108, 170), "障碍": (0, 0, 0), "空地": (127, 145, 200), "灰度白": (255, 255, 255)}
    cnt = dict.fromkeys(pal, 0)
    for px in img.getdata():
        cnt[min(pal, key=lambda k: sum((a - b) ** 2 for a, b in zip(px, pal[k])))] += 1
    show("地图配色", cnt["灰度白"] == 0, f"{img.mode} {img.size} {cnt}")
except Exception as e:
    show("地图配色", False, repr(e))

for name, dtype in (("/get_map_point_list", "waypoint_node"), ("/get_map_waypoint_list", "waypoint")):
    ok, v, dt = call(name, {"map_id": map_id, "data_type": dtype})
    rows = json.loads(v.get("message") or "[]") if v.get("success") else v.get("message")
    show(name, ok and v.get("success"), f"{len(rows) if isinstance(rows, list) else rows} 条")

ok, v, dt = call("/get_ip_addresses")
show("/get_ip_addresses", ok, v.get("message") or v.get("result"))

for topic, typ, fmt in [
    ("/base_link_pose", "geometry_msgs/msg/PoseStamped",
     lambda m: f"x={m['pose']['position']['x']:.3f} y={m['pose']['position']['y']:.3f} frame={m['header']['frame_id']}"),
    ("/scan", "sensor_msgs/msg/LaserScan",
     lambda m: f"{len(m['ranges'])} 束 angle=[{m['angle_min']:.2f},{m['angle_max']:.2f}]"),
    ("/battery_state", "sensor_msgs/msg/BatteryState",
     lambda m: f"percentage={m['percentage']} status={m['power_supply_status']}"),
    ("/map_base64", "nav_msgs/msg/OccupancyGrid",
     lambda m: f"{m['info']['width']}x{m['info']['height']} data={len(m['data'])}B"),
    ("/task_status", "aid_robot_msgs/msg/AidTaskStatus", lambda m: json.dumps(m)),
    ("/dock_state", "std_msgs/msg/String", lambda m: m["data"]),
    ("/robot_status", "std_msgs/msg/String", lambda m: m["data"]),
]:
    m = one_message(topic, typ)
    show(topic, m is not None, fmt(m) if m else "10 s 内没有消息")

queries = sys.argv[1:] or ["我想喝水", "我要开会"]
for q in queries:
    ok, v, dt = call("/semantic_map/search", {"data": json.dumps({"text": q, "source": "claude-ro-test"},
                                                                  ensure_ascii=False)})
    if v.get("success"):
        r = json.loads(v["message"])
        top = "，".join(f"{c['name']} {c['score']:.3f}" for c in r["candidates"]) or "无候选"
        show(f"search {q}", True, f"{'找到 ' + r['best']['name'] if r['found'] else '没找到'} [{top}] ({dt:.1f}s)")
    else:
        show(f"search {q}", False, f"{v.get('message')} ({dt:.1f}s)")

ok, v, dt = call("/semantic_map/history")
hist = json.loads(v.get("message") or "[]")
show("/semantic_map/history", ok, f"{len(hist)} 条" + (f"，最后一条 {hist[-1].get('raw')!r}" if hist else ""))
