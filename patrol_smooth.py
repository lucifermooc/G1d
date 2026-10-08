#!/usr/bin/env python3
"""
平滑巡逻服务（FollowPathPointsAction）
POST /start  — 开始巡逻
POST /stop   — 停止
GET  /status — 状态
"""

import math, re, signal, subprocess, sys, threading, time
import requests
from flask import Flask, jsonify

# ══════════════════════════════════════════════════════
# 宏定义
# ══════════════════════════════════════════════════════
ROBOT_IP = "192.168.123.163"  # 底盘 IP
ROBOT_PORT = 1448  # 底盘 REST API 端口
SERVICE_PORT = 5101  # 本服务监听端口（与 patrol.py 不同，可同时运行）

POI_START_ID = 1  # 巡逻起始 POI ID（整数，含）
POI_END_ID = 10  # 巡逻结束 POI ID（整数，含），超出实际数量时自动截断到全部
DIRECTION = "asc"  # 巡逻方向：asc 升序 | desc 降序

QUEUE_LOOPS = 3  # 每批次发几圈的点；圈数越大，重发间隔的停顿越稀少
SPEED = 0.3  # 巡逻速度 m/s；启动时写入底盘，仅本次运行有效，重启恢复默认
POLL_INTERVAL = 0.5  # 轮询 action 状态间隔（秒）
PAUSED_TIMEOUT = 12.0  # Action 长时间处于 Paused 视为卡住
STALL_TIMEOUT = 20.0  # Action 长时间无位姿/速度进展视为卡住
STALL_RETRY_LIMIT = 3  # 连续卡住的自动恢复次数
STALL_RETRY_DELAY = 2.0  # 自动恢复前等待时间
SPEED_EPSILON = 0.02  # 小于该速度视为静止
# ══════════════════════════════════════════════════════

ROBOT_BASE = f"http://{ROBOT_IP}:{ROBOT_PORT}"
app = Flask(__name__)

_stop_event = threading.Event()
_patrol_thread = None
_lock = threading.Lock()
_status = {"running": False, "current_poi": None, "message": "idle"}
_last_poi_idx = None  # 停止时记录最近 POI 的索引


# ── 底盘请求 ───────────────────────────────────────────
def rget(path):
    return requests.get(f"{ROBOT_BASE}{path}", timeout=5)


def rpost(path, body=None):
    return requests.post(f"{ROBOT_BASE}{path}", json=body, timeout=5)


def rdelete(path):
    return requests.delete(f"{ROBOT_BASE}{path}", timeout=3)


def safe_json(resp):
    try:
        return resp.json()
    except Exception:
        return None


# ── POI 查询 ───────────────────────────────────────────
def get_patrol_pois():
    all_pois = rget("/api/core/artifact/v1/pois").json()
    in_range, all_int = [], []
    for p in all_pois:
        try:
            pid = int(p["id"])
            all_int.append((pid, p))
            if POI_START_ID <= pid <= POI_END_ID:
                in_range.append((pid, p))
        except (ValueError, KeyError):
            pass
    chosen = in_range if in_range else all_int
    chosen.sort(key=lambda x: x[0], reverse=(DIRECTION == "desc"))
    return [p for _, p in chosen]


# ── 当前机器人位置 ──────────────────────────────────────
def get_robot_pose():
    try:
        return rget("/api/core/slam/v1/localization/pose").json()
    except Exception:
        return None


def get_robot_speed():
    try:
        return rget("/api/core/motion/v1/speed").json()
    except Exception:
        return None


def get_robot_health():
    try:
        return rget("/api/core/system/v1/robot/health").json()
    except Exception:
        return None


def get_robot_events():
    try:
        events = rget("/api/platform/v1/events").json()
        return events if isinstance(events, list) else []
    except Exception:
        return []


def has_motion_progress(prev_pose, pose, speed):
    if speed:
        vx = abs(float(speed.get("vx", 0.0)))
        vy = abs(float(speed.get("vy", 0.0)))
        omega = abs(float(speed.get("omega", 0.0)))
        if max(vx, vy, omega) >= SPEED_EPSILON:
            return True

    if prev_pose and pose and "x" in prev_pose and "x" in pose:
        dist = math.hypot(pose["x"] - prev_pose["x"], pose["y"] - prev_pose["y"])
        yaw_delta = abs(float(pose.get("yaw", 0.0)) - float(prev_pose.get("yaw", 0.0)))
        return dist >= 0.03 or yaw_delta >= 0.08

    return False


def summarize_diagnostics(health, events, state=None):
    parts = []

    reason = (state or {}).get("reason")
    if reason:
        parts.append(f"action={reason}")

    if health and health.get("hasFatal"):
        parts.append("底盘存在 Fatal 健康告警")
    elif health and health.get("hasError"):
        parts.append("底盘存在 Error 健康告警")

    interesting = {
        "PATH_OCCUPIED",
        "ROBOT_BLOCKED",
        "LOCALIZATION_ANOMALY",
        "DEVICE_ERROR",
        "LOW_BATTERY",
        "BRAKE_RELEASED",
        "CURRENT_POSE_OCCUPIED",
        "CLIFF_DETECTED",
        "BUMPER_TRIGGERED",
    }
    event_types = [e.get("type") for e in events if e.get("type") in interesting]
    if event_types:
        parts.append("event=" + ",".join(event_types[-3:]))

    if not parts:
        return "长时间无进展"
    return " | ".join(parts)


def abort_current_action():
    try:
        rdelete("/api/core/motion/v1/actions/:current")
    except Exception:
        pass


# ── 找最近 POI 的下一个索引（避免停止后倒退）──────────────
def next_poi_idx_after_stop(pois, pose):
    if not pose or "x" not in pose:
        return 0
    best, best_d = 0, float("inf")
    for i, p in enumerate(pois):
        pp = p.get("pose", {})
        if not pp:
            continue
        d = math.hypot(pp["x"] - pose["x"], pp["y"] - pose["y"])
        if d < best_d:
            best_d, best = d, i
    return (best + 1) % len(pois)


# ── 启动时设置巡逻速度 ─────────────────────────────────────
def set_speed():
    try:
        resp = requests.put(
            f"{ROBOT_BASE}/api/core/system/v1/parameter",
            json={"param": "base.max_moving_speed", "value": str(SPEED)},
            timeout=5,
        )
        resp.raise_for_status()
    except Exception:
        pass


# ── 构建送出的点队列 ──────────────────────────────────────
def build_queue(pois, start_idx):
    """从 start_idx 开始，循环 QUEUE_LOOPS 圈，返回 (path_points, end_idx)"""
    n = len(pois)
    pts = []
    for i in range(n * QUEUE_LOOPS):
        p = pois[(start_idx + i) % n]
        pose = p.get("pose", {})
        pts.append({"x": pose["x"], "y": pose["y"]})
    end_idx = (start_idx + n * QUEUE_LOOPS) % n
    return pts, end_idx


# ── 发送 SeriesMoveToAction（一次送多圈点）─────────────
def send_follow(path_points):
    targets = [{"x": p["x"], "y": p["y"], "z": 0} for p in path_points]
    resp = rpost(
        "/api/core/motion/v1/actions",
        {
            "action_name": "slamtec.agent.actions.SeriesMoveToAction",
            "options": {
                "targets": targets,
                "move_options": {"mode": 0},
            },
        },
    )
    resp.raise_for_status()

    data = safe_json(resp)
    if isinstance(data, dict):
        state = data.get("state", {})
        if state.get("status") == 4 and state.get("result") != 0:
            raise RuntimeError(state.get("reason") or "动作创建后立即失败")
    return data


# ── 轮询等待结束 ───────────────────────────────────────
def poll_until_done():
    """返回包含 result/reason 的字典。"""
    last_pose = None
    last_progress_at = time.monotonic()
    paused_at = None
    last_stage = None

    while not _stop_event.is_set():
        try:
            r = rget("/api/core/motion/v1/actions/:current")
            if r.status_code == 404:
                return {"result": "success", "reason": "action finished"}

            r.raise_for_status()
            info = r.json()
            state = info.get("state", {})
            status = state.get("status")
            stage = info.get("stage")
            pose = get_robot_pose()
            speed = get_robot_speed()

            if has_motion_progress(last_pose, pose, speed) or stage != last_stage or status == 0:
                last_progress_at = time.monotonic()

            if status == 3:
                paused_at = paused_at or time.monotonic()
                if time.monotonic() - paused_at >= PAUSED_TIMEOUT:
                    health = get_robot_health()
                    events = get_robot_events()
                    return {
                        "result": "stalled",
                        "reason": summarize_diagnostics(health, events, state),
                    }
            else:
                paused_at = None

            if status == 4:
                reason = state.get("reason") or "action done"
                if state.get("result") == 0:
                    return {"result": "success", "reason": reason}
                return {"result": "failed", "reason": reason}

            if time.monotonic() - last_progress_at >= STALL_TIMEOUT:
                health = get_robot_health()
                events = get_robot_events()
                return {
                    "result": "stalled",
                    "reason": summarize_diagnostics(health, events, state),
                }

            last_pose = pose or last_pose
            last_stage = stage
        except Exception:
            pass
        time.sleep(POLL_INTERVAL)
    return {"result": "stopped", "reason": "stop requested"}


# ── 巡逻主循环 ─────────────────────────────────────────
def patrol_loop(start_idx):
    global _status, _last_poi_idx
    stall_retries = 0

    try:
        pois = get_patrol_pois()
    except Exception as e:
        with _lock:
            _status = {
                "running": False,
                "current_poi": None,
                "message": f"获取 POI 失败: {e}",
            }
        return

    if not pois:
        with _lock:
            _status = {"running": False, "current_poi": None, "message": "无可用 POI"}
        return

    set_speed()
    idx = start_idx % len(pois)
    print(
        f"[smooth] POI数={len(pois)}，每批={len(pois) * QUEUE_LOOPS}点（{QUEUE_LOOPS}圈），速度={SPEED}m/s，从索引 {idx}（id={pois[idx]['id']}）出发"
    )

    while not _stop_event.is_set():
        pts, next_idx = build_queue(pois, idx)
        poi_id = str(pois[idx]["id"])

        with _lock:
            _status["current_poi"] = poi_id
            _status["message"] = f"→ 索引 {idx}，队列 {len(pts)} 点"
            _last_poi_idx = idx

        print(f"[smooth] 发送 {len(pts)} 个路径点，起点索引 {idx}")
        try:
            send_follow(pts)
        except Exception as e:
            print(f"[smooth] 发送失败: {e}")
            time.sleep(1)
            continue

        result = poll_until_done()
        outcome = result.get("result")
        reason = result.get("reason") or ""

        if outcome == "stopped":
            pose = get_robot_pose()
            with _lock:
                _last_poi_idx = next_poi_idx_after_stop(pois, pose)
            break

        if outcome == "stalled":
            stall_retries += 1
            print(f"[smooth] 路径跟随卡住，准备恢复: {reason}")
            with _lock:
                _status["message"] = (
                    f"巡逻受阻，尝试恢复 {stall_retries}/{STALL_RETRY_LIMIT}: {reason}"
                )

            abort_current_action()
            pose = get_robot_pose()
            idx = next_poi_idx_after_stop(pois, pose)

            if stall_retries >= STALL_RETRY_LIMIT:
                with _lock:
                    _status = {
                        "running": False,
                        "current_poi": None,
                        "message": f"巡逻暂停: {reason}",
                    }
                print(f"[smooth] 连续卡住达到上限，停止巡逻: {reason}")
                break

            time.sleep(STALL_RETRY_DELAY)
            continue

        stall_retries = 0

        if outcome == "failed":
            print(f"[smooth] 路径跟随失败，从当前位置续: {reason}")
            abort_current_action()
            pose = get_robot_pose()
            idx = next_poi_idx_after_stop(pois, pose)
        else:
            idx = next_idx

    abort_current_action()

    with _lock:
        if _status.get("running"):
            _status = {"running": False, "current_poi": None, "message": "已停止"}
    print("[smooth] 巡逻结束")


# ── HTTP 接口 ──────────────────────────────────────────
@app.route("/start", methods=["POST"])
def start_patrol():
    global _patrol_thread

    with _lock:
        if _patrol_thread and _patrol_thread.is_alive():
            return jsonify({"ok": False, "message": "已在巡逻中"})
        start_idx = _last_poi_idx if _last_poi_idx is not None else 0

    _stop_event.clear()
    with _lock:
        _status.update({"running": True, "current_poi": None, "message": "启动中"})

    _patrol_thread = threading.Thread(
        target=patrol_loop, args=(start_idx,), daemon=True
    )
    _patrol_thread.start()
    return jsonify({"ok": True, "message": f"平滑巡逻已开始，起点索引={start_idx}"})


@app.route("/stop", methods=["POST"])
def stop_patrol():
    _stop_event.set()
    abort_current_action()
    return jsonify({"ok": True, "message": "停止指令已发送"})


@app.route("/status")
def get_status():
    with _lock:
        st = dict(_status)
    st["running"] = bool(_patrol_thread and _patrol_thread.is_alive())
    return jsonify(st)


# ── 入口 ───────────────────────────────────────────────
def get_ip(iface):
    try:
        out = subprocess.run(
            ["ip", "addr", "show", iface], capture_output=True, text=True
        ).stdout
        m = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", out)
        return m.group(1) if m else "N/A"
    except Exception:
        return "N/A"


def emergency_stop(sig, frame):
    print("\n[smooth] 收到中断，立即停止机器人...")
    try:
        requests.delete(f"{ROBOT_BASE}/api/core/motion/v1/actions/:current", timeout=3)
        print("[smooth] 已发送停止指令")
    except Exception as e:
        print(f"[smooth] 停止指令发送失败: {e}")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, emergency_stop)
    signal.signal(signal.SIGTERM, emergency_stop)

    print("正在拉取 POI 信息...")
    try:
        _init_pois = get_patrol_pois()
        poi_count = len(_init_pois)
        queue_size = poi_count * QUEUE_LOOPS
    except Exception as e:
        print(f"警告：POI 拉取失败（{e}），启动后再试")
        poi_count = "?"
        queue_size = "?"

    eth0 = get_ip("eth0")
    wlan0 = get_ip("wlan0")
    print("═" * 52)
    print("  平滑巡逻服务（SeriesMoveToAction 多圈批量）")
    print(f"  本地:     http://127.0.0.1:{SERVICE_PORT}")
    print(f"  eth0:     http://{eth0}:{SERVICE_PORT}")
    print(f"  wlan0:    http://{wlan0}:{SERVICE_PORT}")
    print(f"  底盘:   {ROBOT_BASE}")
    actual_ids = [str(p["id"]) for p in _init_pois] if isinstance(poi_count, int) else ["?"]
    actual_range = f"{actual_ids[0]} ~ {actual_ids[-1]}" if actual_ids else "?"
    print(f"  POI 定义范围: {POI_START_ID} ~ {POI_END_ID}  方向: {DIRECTION}")
    print(f"  POI 实际范围: {actual_range}  数量: {poi_count}")
    print(f"  队列: {QUEUE_LOOPS} 圈 × {poi_count} = {queue_size} 点/批次")
    print(f"  速度: {SPEED} m/s")
    print("═" * 52)
    app.run(host="0.0.0.0", port=SERVICE_PORT, debug=False)

