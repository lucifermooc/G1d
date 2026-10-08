#!/usr/bin/env python3
"""
障碍物过滤节点 + 激光点云实时可视化
- ROS2: 订阅 INPUT_TOPIC → 检测前方障碍 → 转发到 OUTPUT_TOPIC
- Web:  http://0.0.0.0:SERVICE_PORT → 点云渲染（绿色=安全，红色=危险）
"""

import math, re, subprocess, threading, time
import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from flask import Flask, jsonify, Response

# ══════════════════════════════════════════════════════
# 宏定义
# ══════════════════════════════════════════════════════
INPUT_TOPIC = "/aidlux/cmd_vel"  # 接收用户指令的话题
OUTPUT_TOPIC = "/cmd_vel_no_limit"  # 转发到底盘的话题（无速度限制）

ROBOT_IP = "192.168.123.163"  # 底盘 IP
ROBOT_PORT = 1448  # 底盘 REST API 端口
SERVICE_PORT = 5200  # Web 可视化端口

# 机器人本体尺寸（单位：m）
ROBOT_WIDTH = 0.557  # 左右宽度 557 mm
ROBOT_DEPTH = 0.525  # 前后深度 525 mm（正面朝向）

# 障碍检测参数
OBSTACLE_CLEARANCE = 0.15  # 前端边缘外多少米内算危险 (m)
# 检测宽度与机器人同宽（abs(y) < ROBOT_WIDTH/2）

LASER_HZ = 20  # 激光 HTTP 轮询频率 (Hz)
LASER_TIMEOUT = 0.5  # 激光数据超时 (s)
BLOCK_ON_TIMEOUT = False  # 超时时是否拦截指令
POINT_RADIUS = 2  # 点云渲染半径（px）
# ══════════════════════════════════════════════════════

ROBOT_BASE = f"http://{ROBOT_IP}:{ROBOT_PORT}"

# 由尺寸派生
_HALF_W = ROBOT_WIDTH / 2  # 0.2785 m
_HALF_D = ROBOT_DEPTH / 2  # 0.2625 m  (从激光中心到前端)
# 危险区：x 在 [_HALF_D, _HALF_D + OBSTACLE_CLEARANCE]，|y| < _HALF_W

_scan_cache = {"points": [], "ts": 0.0}
_scan_lock = threading.Lock()

flask_app = Flask(__name__)


# ── 激光 HTTP 轮询线程 ─────────────────────────────────
def laser_loop():
    interval = 1.0 / LASER_HZ
    while True:
        t0 = time.time()
        try:
            r = requests.get(f"{ROBOT_BASE}/api/core/system/v1/laserscan", timeout=2)
            raw = r.json().get("laser_points", [])
            pts = []
            for p in raw:
                if not p.get("valid", False):
                    continue
                d, a = p["distance"], p["angle"]
                if d <= 0:
                    continue
                x = d * math.cos(a)
                y = d * math.sin(a)
                # 危险：位于机器人正前方边缘外 OBSTACLE_CLEARANCE 以内
                danger = _HALF_D < x < _HALF_D + OBSTACLE_CLEARANCE and abs(y) < _HALF_W
                pts.append({"x": x, "y": y, "danger": danger})
            with _scan_lock:
                _scan_cache["points"] = pts
                _scan_cache["ts"] = time.time()
        except Exception:
            pass
        time.sleep(max(0, interval - (time.time() - t0)))


# ── Flask ─────────────────────────────────────────────
@flask_app.route("/api/scan")
def api_scan():
    with _scan_lock:
        return jsonify(_scan_cache)


@flask_app.route("/")
def index():
    return Response(HTML, mimetype="text/html")


# ── ROS2 节点 ─────────────────────────────────────────
class ObstacleFilter(Node):
    def __init__(self):
        super().__init__("obstacle_filter")
        self._sub = self.create_subscription(Twist, INPUT_TOPIC, self._on_cmd, 10)
        self._pub = self.create_publisher(Twist, OUTPUT_TOPIC, 10)
        self._blocked_last = False
        self.get_logger().info(
            f"障碍过滤: {INPUT_TOPIC} → {OUTPUT_TOPIC}  "
            f"机器人={ROBOT_WIDTH*1000:.0f}×{ROBOT_DEPTH*1000:.0f}mm  "
            f"前端外 {OBSTACLE_CLEARANCE*100:.0f}cm 内拦截"
        )

    def _has_front_obstacle(self) -> bool:
        with _scan_lock:
            age = time.time() - _scan_cache["ts"]
            pts = list(_scan_cache["points"])
        if age > LASER_TIMEOUT:
            return BLOCK_ON_TIMEOUT
        return any(p["danger"] for p in pts)

    def _on_cmd(self, msg: Twist):
        if msg.linear.x > 0 and self._has_front_obstacle():
            if not self._blocked_last:
                self.get_logger().warning("前方障碍，指令拦截")
                self._blocked_last = True
            return
        if self._blocked_last:
            self.get_logger().info("障碍清除，恢复转发")
            self._blocked_last = False
        self._pub.publish(msg)


# ── 工具 ──────────────────────────────────────────────
def get_ip(iface):
    try:
        out = subprocess.run(
            ["ip", "addr", "show", iface], capture_output=True, text=True
        ).stdout
        m = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", out)
        return m.group(1) if m else "N/A"
    except Exception:
        return "N/A"


def main():
    threading.Thread(target=laser_loop, daemon=True).start()
    threading.Thread(
        target=lambda: flask_app.run(host="0.0.0.0", port=SERVICE_PORT, debug=False),
        daemon=True,
    ).start()

    eth0 = get_ip("eth0")
    wlan0 = get_ip("wlan0")
    print("═" * 56)
    print("  激光点云 + 障碍过滤")
    print(f"  可视化:  http://127.0.0.1:{SERVICE_PORT}")
    print(f"  eth0:    http://{eth0}:{SERVICE_PORT}")
    print(f"  wlan0:   http://{wlan0}:{SERVICE_PORT}")
    print(f"  底盘:    {ROBOT_BASE}")
    print(f"  过滤:    {INPUT_TOPIC} → {OUTPUT_TOPIC}")
    print(
        f"  机器人:  {ROBOT_WIDTH*1000:.0f}mm(宽) × {ROBOT_DEPTH*1000:.0f}mm(深/正面)"
    )
    print(
        f"  危险区:  前端边缘外 {OBSTACLE_CLEARANCE*100:.0f}cm，宽度 {ROBOT_WIDTH*100:.0f}cm"
    )
    print("═" * 56)

    rclpy.init()
    node = ObstacleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── 前端 HTML ─────────────────────────────────────────
HTML = (
    """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>激光点云 + 障碍检测</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body { background:#fff; display:flex; flex-direction:column; height:100vh; font-family:system-ui; }
#toolbar { display:flex; align-items:center; gap:16px; padding:8px 16px;
           border-bottom:1px solid #ddd; font-size:13px; color:#555; flex-shrink:0; }
#toolbar label { display:flex; align-items:center; gap:6px; }
#toolbar input[type=range] { width:100px; }
#info { margin-left:auto; color:#888; font-size:12px; }
#wrap { flex:1; overflow:hidden; }
canvas { width:100%; height:100%; display:block; }
</style>
</head>
<body>
<div id="toolbar">
  <b>激光点云</b>
  <label>量程 <input type="range" id="rangeSlider" min="1" max="20" value="8" step="0.5">
    <span id="rangeVal">8 m</span></label>
  <label><input type="checkbox" id="chkGrid"  checked> 网格</label>
  <label><input type="checkbox" id="chkRobot" checked> 机器人</label>
  <label><input type="checkbox" id="chkZone"  checked> 危险区</label>
  <b id="status" style="color:#22aa44">✓ 通行</b>
  <span id="info">—</span>
</div>
<div id="wrap"><canvas id="c"></canvas></div>
<script>
const REFRESH_HZ   = MACRO_HZ;
const POINT_RADIUS = MACRO_PR;
const HALF_D       = MACRO_HD;   // 机器人前半深（激光中心→前端），m
const HALF_W       = MACRO_HW;   // 机器人半宽，m
const CLEARANCE    = MACRO_CL;   // 前端外危险间距，m

const canvas = document.getElementById('c');
const ctx    = canvas.getContext('2d');
const wrap   = document.getElementById('wrap');

let points = [], maxRange = 8, showGrid = true, showRobot = true, showZone = true;
let lastTs = 0, fps = 0, fpsCount = 0, fpsTimer = Date.now();

document.getElementById('rangeSlider').addEventListener('input', e => {
  maxRange = parseFloat(e.target.value);
  document.getElementById('rangeVal').textContent = maxRange + ' m';
});
document.getElementById('chkGrid') .addEventListener('change', e => showGrid  = e.target.checked);
document.getElementById('chkRobot').addEventListener('change', e => showRobot = e.target.checked);
document.getElementById('chkZone') .addEventListener('change', e => showZone  = e.target.checked);

function resize() {
  canvas.width  = wrap.clientWidth  * devicePixelRatio;
  canvas.height = wrap.clientHeight * devicePixelRatio;
}
window.addEventListener('resize', resize);
resize();

function draw() {
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H / 2;
  const scale = Math.min(W, H) / 2 / maxRange;
  const dpr = devicePixelRatio;

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#fff';
  ctx.fillRect(0, 0, W, H);

  // 网格
  if (showGrid) {
    ctx.strokeStyle = '#eee'; ctx.lineWidth = 1;
    for (let r = 1; r <= maxRange; r++) {
      ctx.beginPath(); ctx.arc(cx, cy, r * scale, 0, Math.PI * 2); ctx.stroke();
    }
    ctx.strokeStyle = '#ddd';
    ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.stroke();
    ctx.fillStyle = '#bbb';
    ctx.font = (11 * dpr) + 'px system-ui';
    ctx.textAlign = 'left';
    for (let r = 1; r <= maxRange; r++)
      ctx.fillText(r + 'm', cx + r * scale + 3, cy - 3);
  }

  // 机器人本体（557×525mm 矩形，正前方=+x=右）
  if (showRobot) {
    const hw = HALF_W * scale;   // 半宽（y 轴）
    const hd = HALF_D * scale;   // 半深（x 轴）
    // 机体
    ctx.fillStyle   = '#1a3a5a';
    ctx.strokeStyle = '#5ab4f0';
    ctx.lineWidth   = 1.5 * dpr;
    ctx.beginPath();
    ctx.roundRect(cx - hd, cy - hw, hd * 2, hw * 2, 4 * dpr);
    ctx.fill(); ctx.stroke();
    // 前端高亮条（正面）
    const sw = Math.max(3 * dpr, hd * 0.12);
    ctx.fillStyle = '#7fd4ff';
    ctx.beginPath();
    ctx.roundRect(cx + hd - sw, cy - hw + 4*dpr, sw, hw * 2 - 8*dpr, [0, 3*dpr, 3*dpr, 0]);
    ctx.fill();
  }

  // 危险区（前端边缘外 CLEARANCE，同机器人宽）
  if (showZone) {
    const hw  = HALF_W     * scale;
    const x0  = cx + HALF_D * scale;
    const zw  = CLEARANCE  * scale;
    ctx.fillStyle   = 'rgba(255,60,60,0.12)';
    ctx.strokeStyle = 'rgba(255,60,60,0.5)';
    ctx.lineWidth   = 1.5;
    ctx.fillRect  (x0, cy - hw, zw, hw * 2);
    ctx.strokeRect(x0, cy - hw, zw, hw * 2);
  }

  // 激光点（绿=安全，红=危险）
  let hasDanger = false;
  for (const p of points) {
    const px = cx + p.x * scale;
    const py = cy - p.y * scale;
    if (px < 0 || px > W || py < 0 || py > H) continue;
    if (p.danger) { hasDanger = true; ctx.fillStyle = '#e03030'; }
    else            ctx.fillStyle = '#22aa44';
    ctx.beginPath();
    ctx.arc(px, py, POINT_RADIUS * dpr, 0, Math.PI * 2);
    ctx.fill();
  }

  // 状态
  const s = document.getElementById('status');
  if (hasDanger) { s.textContent = '⚠ 障碍'; s.style.color = '#e03030'; }
  else           { s.textContent = '✓ 通行'; s.style.color = '#22aa44'; }

  fpsCount++;
  const now = Date.now();
  if (now - fpsTimer >= 1000) { fps = fpsCount; fpsCount = 0; fpsTimer = now; }
  document.getElementById('info').textContent =
    '点数: ' + points.length + '  FPS: ' + fps + '  量程: ±' + maxRange + 'm';
}

async function poll() {
  try {
    const r = await fetch('/api/scan');
    const d = await r.json();
    if (d.ts !== lastTs) { points = d.points; lastTs = d.ts; }
  } catch(e) {}
  draw();
}

setInterval(poll, Math.round(1000 / REFRESH_HZ));
poll();
</script>
</body>
</html>
""".replace("MACRO_HZ", str(LASER_HZ))
    .replace("MACRO_PR", str(POINT_RADIUS))
    .replace("MACRO_HD", str(_HALF_D))
    .replace("MACRO_HW", str(_HALF_W))
    .replace("MACRO_CL", str(OBSTACLE_CLEARANCE))
)


if __name__ == "__main__":
    main()
