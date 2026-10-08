#!/usr/bin/env python3

from flask import Flask, jsonify, request, send_file
import requests
import json
import os
import struct
import numpy as np
import base64
import uuid
import hashlib
from concurrent.futures import ThreadPoolExecutor
from PIL import Image
import io

app = Flask(__name__)
chassis_url = None

# 地图导出目录（与 app.py 同级的 map/ 文件夹）
APP_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_DIR = os.path.join(APP_DIR, "map")
MAP_EXTS = (".pgm", ".yaml", ".stcm")


def next_map_base():
    """返回下一个可用基名：map, map1, map2 …（三个后缀都不冲突）"""
    os.makedirs(MAP_DIR, exist_ok=True)
    i = 0
    while True:
        base = "map" if i == 0 else f"map{i}"
        if not any(os.path.exists(os.path.join(MAP_DIR, base + e)) for e in MAP_EXTS):
            return base
        i += 1


def req(method, path, **kwargs):
    fn = getattr(requests, method)
    return fn(f"{chassis_url}{path}", timeout=5, **kwargs)


def safe(fn):
    try:
        return fn()
    except Exception:
        return None


def parallel_gets(*paths):
    def fetch(path):
        return safe(lambda: req("get", path).json())
    with ThreadPoolExecutor(max_workers=len(paths)) as ex:
        return list(ex.map(fetch, paths))


@app.route("/")
def index():
    return send_file("index.html")


# ── 连接 ──────────────────────────────────────────────────────────
@app.route("/api/connect", methods=["POST"])
def connect():
    global chassis_url
    ip = request.json.get("ip", "").strip()
    url = f"http://{ip}:1448"
    try:
        resp = requests.get(f"{url}/api/core/system/v1/capabilities", timeout=3)
        caps = resp.json()
        chassis_url = url
        return jsonify({"ok": True, "capabilities": caps})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


# ── 地图 ──────────────────────────────────────────────────────────
@app.route("/api/map")
def get_map():
    if not chassis_url:
        return jsonify({"error": "not connected"}), 400
    try:
        data = req("get", "/api/core/slam/v1/maps/explore").content
        x0, y0 = struct.unpack_from("<ff", data, 0)
        cols, rows = struct.unpack_from("<II", data, 8)
        res = struct.unpack_from("<f", data, 16)[0]

        if cols == 0 or rows == 0:
            mapping, localization = parallel_gets(
                "/api/core/slam/v1/mapping/:enable",
                "/api/core/slam/v1/localization/:enable",
            )
            return jsonify({"empty": True,
                            "mapping": mapping, "localization": localization})

        grid_raw  = data[36: 36 + cols * rows]
        grid_hash = hashlib.md5(grid_raw).hexdigest()[:12]

        # Fetch metadata in parallel
        pose, quality, mapping, localization = parallel_gets(
            "/api/core/slam/v1/localization/pose",
            "/api/core/slam/v1/localization/quality",
            "/api/core/slam/v1/mapping/:enable",
            "/api/core/slam/v1/localization/:enable",
        )

        # If client already has this map version, skip image generation/transfer
        if request.args.get('h') == grid_hash:
            return jsonify({
                "unchanged": True,
                "hash": grid_hash,
                "pose": pose,
                "quality": quality,
                "mapping": mapping,
                "localization": localization,
            })

        grid = np.frombuffer(grid_raw, dtype=np.uint8).reshape((rows, cols))
        # 0=未探索→灰, 1-99=障碍→纯黑, 100-255=可通行→纯白
        ch = np.full((rows, cols), 160, dtype=np.uint8)
        ch[(grid >= 1) & (grid < 100)] = 0
        ch[grid >= 100] = 255
        rgb  = np.stack([ch, ch, ch], axis=-1)

        img = Image.fromarray(np.flipud(rgb))
        buf = io.BytesIO()
        img.save(buf, "PNG", optimize=True)
        img_b64 = base64.b64encode(buf.getvalue()).decode()

        return jsonify({
            "img": img_b64,
            "hash": grid_hash,
            "x0": x0, "y0": y0,
            "cols": cols, "rows": rows,
            "res": res,
            "pose": pose,
            "quality": quality,
            "mapping": mapping,
            "localization": localization,
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# 建图 / 定位 独立开关（互不影响，可同时开）
@app.route("/api/mapping/enable", methods=["PUT"])
def set_mapping():
    enable = bool((request.json or {}).get("enable"))
    req("put", "/api/core/slam/v1/mapping/:enable", json={"enable": enable})
    return jsonify({"ok": True})


@app.route("/api/localization/enable", methods=["PUT"])
def set_localization():
    enable = bool((request.json or {}).get("enable"))
    req("put", "/api/core/slam/v1/localization/:enable", json={"enable": enable})
    return jsonify({"ok": True})


@app.route("/api/mapping/save", methods=["POST"])
def mapping_save():
    req("post", "/api/multi-floor/map/v1/stcm/:save")
    return jsonify({"ok": True})


@app.route("/api/map/load", methods=["POST"])
def map_load():
    req("post", "/api/multi-floor/map/v1/stcm/:reload")
    return jsonify({"ok": True})


@app.route("/api/map/export", methods=["POST"])
def export_map():
    """导出地图到本地 map/ 文件夹，三件套同基名：
       <base>.pgm  + <base>.yaml  → 给 ROS 用
       <base>.stcm                → 机器人原生复合地图，给导入用（无损）
    """
    if not chassis_url:
        return jsonify({"ok": False, "error": "未连接底盘"}), 400
    try:
        data = req("get", "/api/core/slam/v1/maps/explore").content
        x0, y0 = struct.unpack_from("<ff", data, 0)
        cols, rows = struct.unpack_from("<II", data, 8)
        res = struct.unpack_from("<f", data, 16)[0]
        if cols == 0 or rows == 0:
            return jsonify({"ok": False, "error": "地图为空"}), 400
        grid_raw = data[36: 36 + cols * rows]

        grid = np.frombuffer(grid_raw, dtype=np.uint8).reshape((rows, cols))
        pgm = np.full((rows, cols), 205, dtype=np.uint8)
        pgm[(grid >= 1) & (grid < 100)] = 0    # 障碍 → 黑
        pgm[grid >= 100] = 254                  # 可通行 → 白
        pgm = np.flipud(pgm)                    # ROS 图像坐标 Y 轴翻转

        base = next_map_base()
        pgm_path  = os.path.join(MAP_DIR, base + ".pgm")
        yaml_path = os.path.join(MAP_DIR, base + ".yaml")
        stcm_path = os.path.join(MAP_DIR, base + ".stcm")

        # map.pgm（P5 二进制）
        with open(pgm_path, "wb") as f:
            f.write(f"P5\n{cols} {rows}\n255\n".encode())
            f.write(pgm.tobytes())

        # map.yaml（ROS 元数据，image 字段指向同基名 pgm）
        with open(yaml_path, "w") as f:
            f.write(
                f"image: {base}.pgm\n"
                f"resolution: {res:.4f}\n"
                f"origin: [{x0:.4f}, {y0:.4f}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )

        # map.stcm（机器人原生复合地图，无损，供导入还原）
        files = [base + ".pgm", base + ".yaml"]
        stcm = safe(lambda: req("get", "/api/core/slam/v1/maps/stcm").content)
        if stcm:
            with open(stcm_path, "wb") as f:
                f.write(stcm)
            files.append(base + ".stcm")

        return jsonify({"ok": True, "name": base, "dir": MAP_DIR,
                        "files": files, "stcm": bool(stcm)})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@app.route("/api/map/clear", methods=["DELETE"])
def map_clear():
    req("delete", "/api/core/slam/v1/maps")
    return jsonify({"ok": True})


# ── 综合状态（电量 + 当前动作）────────────────────────────────────
@app.route("/api/status")
def get_status():
    power, action, floor = parallel_gets(
        "/api/core/system/v1/power/status",
        "/api/core/motion/v1/actions/:current",
        "/api/multi-floor/map/v1/floors/:current",
    )
    return jsonify({"power": power, "action": action, "floor": floor})


# ── 标注数据（POI + 区域 + 墙 + 充电桩，一次并行拉取）────────────
@app.route("/api/annotations")
def get_annotations():
    pois_data, areas_data, walls_data, docks_data = parallel_gets(
        "/api/core/artifact/v1/pois",
        "/api/core/artifact/v1/rectangle-areas/forbidden_area",
        "/api/core/artifact/v1/lines/walls",
        "/api/core/slam/v1/homedocks",
    )
    return jsonify({
        "pois":  pois_data  if isinstance(pois_data,  list) else [],
        "areas": areas_data if isinstance(areas_data, list) else [],
        "walls": walls_data if isinstance(walls_data, list) else [],
        "docks": docks_data if isinstance(docks_data, list) else [],
    })


# ── 导航 ──────────────────────────────────────────────────────────
@app.route("/api/navigate", methods=["POST"])
def navigate():
    d = request.json
    target    = {"x": d["x"], "y": d["y"], "z": 0}
    move_opts = {"mode": 0}
    yaw = d.get("yaw")
    if yaw is not None:
        move_opts["yaw"]   = float(yaw)
        move_opts["flags"] = ["with_yaw"]
    r = req("post", "/api/core/motion/v1/actions", json={
        "action_name": "slamtec.agent.actions.MoveToAction",
        "options": {"target": target, "move_options": move_opts},
    })
    return jsonify(r.json())


@app.route("/api/map/reset-origin", methods=["POST"])
def reset_origin():
    pose = safe(lambda: req("get", "/api/core/slam/v1/localization/pose").json())
    if not pose or "x" not in pose:
        return jsonify({"ok": False, "error": "无法获取当前位置"}), 400
    r = req("put", "/api/core/slam/v1/maps/origin",
            json={"new_origin": {"x": pose["x"], "y": pose["y"]}})
    return jsonify({"ok": r.status_code < 300})


@app.route("/api/stop", methods=["POST"])
def stop():
    requests.delete(f"{chassis_url}/api/core/motion/v1/actions/:current", timeout=3)
    return jsonify({"ok": True})


@app.route("/api/home", methods=["POST"])
def go_home():
    r = req("post", "/api/core/motion/v1/actions", json={
        "action_name": "slamtec.agent.actions.GoHomeAction",
        "options": {},
    })
    return jsonify(r.json())


@app.route("/api/relocalize", methods=["POST"])
def relocalize():
    d = request.json or {}
    options = {}
    if all(k in d for k in ['x1', 'y1', 'x2', 'y2']):
        x1, y1, x2, y2 = d['x1'], d['y1'], d['x2'], d['y2']
        options['area'] = {
            'x': min(x1, x2), 'y': min(y1, y2),
            'width': abs(x2 - x1), 'height': abs(y2 - y1),
        }
    r = req("post", "/api/core/motion/v1/actions", json={
        "action_name": "slamtec.agent.actions.RecoverLocalizationAction",
        "options": options,
    })
    return jsonify(r.json())


@app.route("/api/path")
def get_path():
    r = safe(lambda: req("get", "/api/core/motion/v1/path").json())
    return jsonify(r or [])


# ── 遥控 ──────────────────────────────────────────────────────────
@app.route("/api/speed", methods=["PUT"])
def set_teleop_speed():
    d = request.json or {}
    if "linear" in d:
        safe(lambda: req("put", "/api/core/system/v1/parameter",
                         json={"param": "base.max_moving_speed", "value": str(d["linear"])}))
    return jsonify({"ok": True})


@app.route("/api/move", methods=["POST"])
def move():
    direction = request.json.get("direction")
    req("post", "/api/core/motion/v1/actions", json={
        "action_name": "slamtec.agent.actions.MoveByAction",
        "options": {"direction": direction, "duration": 600},
    })
    return jsonify({"ok": True})


# ── 充电桩 ─────────────────────────────────────────────────────────
@app.route("/api/docks")
def get_docks():
    r = safe(lambda: req("get", "/api/core/slam/v1/homedocks").json())
    return jsonify(r or [])


@app.route("/api/docks/register", methods=["POST"])
def register_dock():
    d = request.json or {}
    r = req("post", "/api/core/slam/v1/homedocks/:register", json={
        "metadata": {"display_name": d.get("name", "充电桩")}
    })
    return jsonify(r.json())


@app.route("/api/docks", methods=["POST"])
def add_dock():
    d = request.json
    r = req("post", "/api/core/slam/v1/homedocks", json={
        "id": d.get("id") or str(uuid.uuid4()),
        "pose": {"x": d["x"], "y": d["y"], "yaw": d.get("yaw", 0)},
        "metadata": {"display_name": d.get("name", "充电桩")},
    })
    return jsonify(r.json())


@app.route("/api/docks/<dock_id>", methods=["DELETE"])
def delete_dock(dock_id):
    req("delete", f"/api/core/slam/v1/homedocks/{dock_id}")
    return jsonify({"ok": True})


@app.route("/api/docks/clear", methods=["DELETE"])
def clear_docks():
    req("delete", "/api/core/slam/v1/homedocks")
    return jsonify({"ok": True})


# ── POI ───────────────────────────────────────────────────────────
@app.route("/api/pois")
def get_pois():
    r = req("get", "/api/core/artifact/v1/pois")
    return jsonify(r.json())


@app.route("/api/pois", methods=["POST"])
def add_poi():
    d = request.json
    name = d.get("name", "POI")
    body = {
        "id": d.get("id") or name or str(uuid.uuid4()),
        "metadata": {"display_name": name, "description": d.get("description", "")},
    }
    if "x" in d:
        pose = {"x": d["x"], "y": d["y"]}
        if "yaw" in d and d["yaw"] is not None:
            pose["yaw"] = float(d["yaw"])
        body["pose"] = pose
    req("post", "/api/core/artifact/v1/pois", json=body)
    return jsonify({"ok": True})


@app.route("/api/pois/<poi_id>", methods=["PUT"])
def update_poi(poi_id):
    """修改 POI 的名称 / 描述（描述给语义地图用，存在 metadata.description）。"""
    d = request.json or {}
    poi = req("get", f"/api/core/artifact/v1/pois/{poi_id}").json()
    meta = poi.setdefault("metadata", {})
    if "name" in d:
        meta["display_name"] = d["name"]
    if "description" in d:
        meta["description"] = d["description"]
    r = req("put", f"/api/core/artifact/v1/pois/{poi_id}", json=poi)
    return jsonify({"ok": r.status_code < 300})


@app.route("/api/pois/<poi_id>", methods=["DELETE"])
def delete_poi(poi_id):
    req("delete", f"/api/core/artifact/v1/pois/{poi_id}")
    return jsonify({"ok": True})


@app.route("/api/pois/navigate/<poi_id>", methods=["POST"])
def navigate_poi(poi_id):
    poi = req("get", f"/api/core/artifact/v1/pois/{poi_id}").json()
    pose = poi.get("pose", {})
    target = {"x": pose["x"], "y": pose["y"], "z": 0}
    move_opts = {"mode": 0}
    if "yaw" in pose and pose["yaw"] is not None:
        move_opts["yaw"]   = float(pose["yaw"])
        move_opts["flags"] = ["with_yaw"]
    r = req("post", "/api/core/motion/v1/actions", json={
        "action_name": "slamtec.agent.actions.MoveToAction",
        "options": {"target": target, "move_options": move_opts},
    })
    return jsonify(r.json())


# ── 禁行区 ─────────────────────────────────────────────────────────
@app.route("/api/health")
def get_health():
    r = safe(lambda: req("get", "/api/core/system/v1/robot/health").json())
    return jsonify(r or {})


@app.route("/api/health/<int:error_code>", methods=["DELETE"])
def clear_error(error_code):
    req("delete", f"/api/core/system/v1/robot/health/{error_code}")
    return jsonify({"ok": True})


# ── 禁行区 ─────────────────────────────────────────────
@app.route("/api/areas")
def get_areas():
    r = safe(lambda: req("get", "/api/core/artifact/v1/rectangle-areas/forbidden_area").json())
    return jsonify(r or [])


@app.route("/api/areas", methods=["POST"])
def add_area():
    d = request.json
    x1, y1, x2, y2 = d["x1"], d["y1"], d["x2"], d["y2"]
    w = abs(x2 - x1)
    h = abs(y2 - y1)
    if w >= h:
        start = {"x": min(x1, x2), "y": (y1 + y2) / 2}
        end   = {"x": max(x1, x2), "y": (y1 + y2) / 2}
        half_width = h / 2
    else:
        start = {"x": (x1 + x2) / 2, "y": min(y1, y2)}
        end   = {"x": (x1 + x2) / 2, "y": max(y1, y2)}
        half_width = w / 2
    r = req("post", "/api/core/artifact/v1/rectangle-areas/forbidden_area", json={
        "area": {"start": start, "end": end, "half_width": half_width},
        "metadata": {"escape_distance": "0.1"},
    })
    return jsonify(r.json())


@app.route("/api/areas/<int:area_id>", methods=["DELETE"])
def delete_area(area_id):
    req("delete", f"/api/core/artifact/v1/rectangle-areas/forbidden_area/{area_id}")
    return jsonify({"ok": True})


@app.route("/api/areas/clear", methods=["DELETE"])
def clear_areas():
    req("delete", "/api/core/artifact/v1/rectangle-areas/forbidden_area")
    return jsonify({"ok": True})


# ── 虚拟墙 ─────────────────────────────────────────────────────────
@app.route("/api/walls")
def get_walls():
    r = safe(lambda: req("get", "/api/core/artifact/v1/lines/walls").json())
    return jsonify(r or [])


@app.route("/api/walls", methods=["POST"])
def add_wall():
    d = request.json
    # Line schema: flat object with id (integer, ignored on create), start, end
    wall_obj = {
        "id": 0,
        "start": {"x": d["x1"], "y": d["y1"]},
        "end":   {"x": d["x2"], "y": d["y2"]},
    }
    try:
        r = req("post", "/api/core/artifact/v1/lines/walls", json=[wall_obj])
        return jsonify({"ok": r.status_code < 300})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/walls/<wall_id>", methods=["DELETE"])
def delete_wall(wall_id):
    req("delete", f"/api/core/artifact/v1/lines/walls/{wall_id}")
    return jsonify({"ok": True})


@app.route("/api/walls/clear", methods=["DELETE"])
def clear_walls():
    req("delete", "/api/core/artifact/v1/lines/walls")
    return jsonify({"ok": True})


# ── 语义地图 ───────────────────────────────────────────────────────
# 一句话 → 最相关的 POI（名称 + 描述）→ 可选导航。依赖 semantic_map_ros / qdrant-client / FlagEmbedding，
# 没装时这几个接口返回 503 和原因，控制台其它功能不受影响。模型第一次用到时才加载。
_semantic = None
_semantic_error = None


def _semantic_pois():
    if not chassis_url:
        raise RuntimeError("未连接底盘")
    return req("get", "/api/core/artifact/v1/pois").json()


def _semantic_navigate(x, y, yaw):
    r = req("post", "/api/core/motion/v1/actions", json={
        "action_name": "slamtec.agent.actions.MoveToAction",
        "options": {"target": {"x": x, "y": y, "z": 0},
                    "move_options": {"mode": 0, "yaw": float(yaw), "flags": ["with_yaw"]}},
    })
    if r.status_code >= 300:
        raise RuntimeError(f"导航下发失败：HTTP {r.status_code}")


def semantic():
    global _semantic, _semantic_error
    if _semantic is None and _semantic_error is None:
        try:
            from g1d_semantic import SemanticService
            _semantic = SemanticService(_semantic_pois, _semantic_navigate)
        except Exception as e:
            _semantic_error = f"语义地图不可用：{e}"
    if _semantic is None:
        raise RuntimeError(_semantic_error)
    return _semantic


def _semantic_request(fn):
    try:
        return jsonify({"ok": True, "result": fn()})
    except RuntimeError as e:
        return jsonify({"ok": False, "error": str(e)}), 503 if "不可用" in str(e) else 400
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


def _semantic_text():
    d = request.json or {}
    text = d.get("text", "")
    return json.dumps({"text": text, "source": d["source"]}, ensure_ascii=False) if d.get("source") else text


@app.route("/api/semantic/status")
def semantic_status():
    def run():
        s = semantic()
        st = s.status()
        if not st["model_loaded"] and request.args.get("preload"):
            s.preload()
        return st
    return _semantic_request(run)


@app.route("/api/semantic/search", methods=["POST"])
def semantic_search():
    return _semantic_request(lambda: semantic().handle("search", _semantic_text(), go=False))


@app.route("/api/semantic/go", methods=["POST"])
def semantic_go():
    return _semantic_request(lambda: semantic().handle("go", _semantic_text(), go=True))


@app.route("/api/semantic/rebuild", methods=["POST"])
def semantic_rebuild():
    return _semantic_request(lambda: semantic().rebuild())


@app.route("/api/semantic/history")
def semantic_history():
    return _semantic_request(lambda: semantic().history())


if __name__ == "__main__":
    # 端口可用环境变量改（8550 上 5000 被系统服务占用，用 G1D_CONSOLE_PORT=5001）
    app.run(host="0.0.0.0", port=int(os.environ.get("G1D_CONSOLE_PORT", "5000")), debug=False, threaded=True)
