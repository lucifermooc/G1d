"""G1 navigation interface data conversion backed by the G1D HTTP API.

This module has no ROS dependency so its conversions can be checked offline.
"""

import base64
import io
import json
import math
import os
import struct
import threading
import time
import uuid
import zlib
from pathlib import Path

import requests
from PIL import Image


class BridgeError(RuntimeError):
    pass


def yaw_from_quaternion(q):
    x, y, z, w = (float(q.get(k, 0 if k != "w" else 1)) for k in ("x", "y", "z", "w"))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}


def parse_explore(raw):
    """Decode G1D's 36 byte header and occupancy bytes."""
    if len(raw) < 36:
        raise BridgeError("G1D map response is shorter than its header")
    x0, y0, width, height, resolution = struct.unpack_from("<ffIIf", raw)
    cells = width * height
    if not width or not height or cells > 100_000_000:
        raise BridgeError("G1D map dimensions are invalid or empty")
    if not math.isfinite(resolution) or resolution <= 0 or len(raw) < 36 + cells:
        raise BridgeError("G1D map data is incomplete")
    return {"x0": x0, "y0": y0, "width": width, "height": height,
            "resolution": resolution, "grid": raw[36:36 + cells]}


def map_to_g1_image(raw):
    """Return the JPEG/base64 OccupancyGrid payload expected by G1 changeStr()."""
    m = parse_explore(raw)
    pixels = bytearray(len(m["grid"]))
    for i, value in enumerate(m["grid"]):
        pixels[i] = 255 if value >= 100 else (0 if value >= 1 else 150)
    image = Image.frombytes("L", (m["width"], m["height"]), bytes(pixels))
    image = image.transpose(Image.FLIP_TOP_BOTTOM)
    output = io.BytesIO()
    image.save(output, "JPEG", quality=95)
    m["base64"] = base64.b64encode(output.getvalue())
    return m


def numeric_poi_id(value):
    text = str(value)
    if text.isdecimal() and 0 < int(text) < 2**32:
        return int(text)
    return (zlib.crc32(text.encode("utf-8")) & 0x7FFFFFFF) or 1


def laser_to_scan(observation):
    """Bin G1D angle/distance samples into a ROS LaserScan's uniform angles."""
    points = [p for p in observation.get("laser_points", [])
              if math.isfinite(float(p.get("angle", float("nan"))))]
    if len(points) < 2:
        raise BridgeError("G1D laser scan has fewer than two points")
    angles = sorted(float(p["angle"]) for p in points)
    minimum, maximum = angles[0], angles[-1]
    increment = (maximum - minimum) / (len(points) - 1)
    if increment <= 0:
        raise BridgeError("G1D laser scan angles are invalid")
    ranges = [0.0] * len(points)
    for point in points:
        distance = float(point.get("distance", 0))
        if not point.get("valid", True) or not math.isfinite(distance) or distance <= 0:
            continue
        index = min(len(points) - 1, max(0, round((float(point["angle"]) - minimum) / increment)))
        if ranges[index] == 0 or distance < ranges[index]:
            ranges[index] = distance
    return minimum, increment, ranges


class RobotHttp:
    def __init__(self, robot_ip, timeout=5):
        self.base = f"http://{robot_ip}:1448"
        self.timeout = timeout

    def request(self, method, path, **kwargs):
        response = requests.request(method, self.base + path, timeout=self.timeout, **kwargs)
        response.raise_for_status()
        return response

    def json(self, method, path, **kwargs):
        response = self.request(method, path, **kwargs)
        return response.json() if response.content else {}

    def binary(self, path):
        return self.request("GET", path).content


class G1DCompat:
    """Implements the navigation subset of the G1 frontend contract."""

    MAP_PATH = "/api/core/slam/v1/maps/explore"
    STCM_PATH = "/api/core/slam/v1/maps/stcm"
    ACTION_PATH = "/api/core/motion/v1/actions"
    CURRENT_ACTION = ACTION_PATH + "/:current"
    POI_PATH = "/api/core/artifact/v1/pois"
    WALL_PATH = "/api/core/artifact/v1/lines/walls"

    def __init__(self, robot, data_dir):
        self.robot = robot
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.catalog_path = self.data_dir / "catalog.json"
        self.lock = threading.RLock()
        self.catalog = self._read_catalog()
        self.pending = None
        self.task = {"status": 0, "task_type": 0, "targets": [], "yaw": None, "action_id": None}

    def _read_catalog(self):
        if self.catalog_path.exists():
            return json.loads(self.catalog_path.read_text(encoding="utf-8"))
        return {"maps": [], "current_id": 0, "next_id": 1, "routes": [], "next_route_id": 1}

    def _save_catalog(self):
        temp = self.catalog_path.with_suffix(".tmp")
        temp.write_text(json.dumps(self.catalog, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(temp, self.catalog_path)

    def _entry(self, map_id):
        return next((m for m in self.catalog["maps"] if m["id"] == int(map_id)), None)

    def current_map_id(self):
        return self.catalog["current_id"]

    def list_maps(self):
        return [{"id": m["id"], "name": m["name"],
                 "create_timestamp": m["create_timestamp"]} for m in self.catalog["maps"]]

    def discover_current_map(self):
        """Import the robot's existing map once into the local G1 map catalog."""
        with self.lock:
            if self.catalog["maps"]:
                return
            raw = self.robot.binary(self.MAP_PATH)
            parse_explore(raw)
            stcm = self.robot.binary(self.STCM_PATH)
            if not stcm:
                raise BridgeError("G1D returned an empty STCM map")
            map_id = self.catalog["next_id"]
            (self.data_dir / f"{map_id}.explore").write_bytes(raw)
            (self.data_dir / f"{map_id}.stcm").write_bytes(stcm)
            self.catalog["maps"].append({"id": map_id, "name": "G1D当前地图",
                                         "create_timestamp": int(time.time())})
            self.catalog["current_id"] = map_id
            self.catalog["next_id"] = map_id + 1
            self._save_catalog()

    def save_map(self):
        self.robot.request("POST", "/api/multi-floor/map/v1/stcm/:save")
        raw = self.robot.binary(self.MAP_PATH)
        parse_explore(raw)
        stcm = self.robot.binary(self.STCM_PATH)
        if not stcm:
            raise BridgeError("G1D returned an empty STCM map")
        with self.lock:
            self.pending = (raw, stcm)

    def add_map(self, name):
        name = name.strip()
        if not name:
            raise BridgeError("map name is empty")
        with self.lock:
            if any(m["name"] == name for m in self.catalog["maps"]):
                raise BridgeError("map name already exists")
            if self.pending is None:
                raw = self.robot.binary(self.MAP_PATH)
                parse_explore(raw)
                stcm = self.robot.binary(self.STCM_PATH)
            else:
                raw, stcm = self.pending
            if not stcm:
                raise BridgeError("G1D returned an empty STCM map")
            map_id = self.catalog["next_id"]
            (self.data_dir / f"{map_id}.explore").write_bytes(raw)
            (self.data_dir / f"{map_id}.stcm").write_bytes(stcm)
            self.catalog["maps"].append({"id": map_id, "name": name,
                                         "create_timestamp": int(time.time())})
            self.catalog["next_id"] = map_id + 1
            self.catalog["current_id"] = map_id
            self.pending = None
            self._save_catalog()
            return map_id

    def map_image(self, map_id):
        entry = self._entry(map_id)
        if entry is None:
            raise BridgeError(f"map {map_id} does not exist")
        if map_id == self.current_map_id():
            try:
                return map_to_g1_image(self.robot.binary(self.MAP_PATH))
            except (requests.RequestException, BridgeError):
                pass
        return map_to_g1_image((self.data_dir / f"{map_id}.explore").read_bytes())

    def live_map(self):
        return map_to_g1_image(self.robot.binary(self.MAP_PATH))

    def set_map(self, map_id):
        if self._entry(map_id) is None:
            raise BridgeError(f"map {map_id} does not exist")
        if map_id != self.current_map_id():
            stcm = (self.data_dir / f"{map_id}.stcm").read_bytes()
            self.robot.request("PUT", self.STCM_PATH, data=stcm,
                               headers={"Content-Type": "application/octet-stream"})
            with self.lock:
                self.catalog["current_id"] = map_id
                self._save_catalog()

    def rename_map(self, map_id, name):
        with self.lock:
            entry = self._entry(map_id)
            if entry is None or not name.strip():
                raise BridgeError("map does not exist or name is empty")
            entry["name"] = name.strip()
            self._save_catalog()

    def delete_map(self, map_id):
        with self.lock:
            if map_id == self.current_map_id():
                raise BridgeError("cannot delete the active map")
            if self._entry(map_id) is None:
                raise BridgeError("map does not exist")
            self.catalog["maps"] = [m for m in self.catalog["maps"] if m["id"] != map_id]
            self.catalog["routes"] = [r for r in self.catalog["routes"] if r["map_id"] != map_id]
            self._save_catalog()
            for suffix in ("explore", "stcm"):
                (self.data_dir / f"{map_id}.{suffix}").unlink(missing_ok=True)

    def set_mode(self, action):
        if action in ("mapping", "continue_mapping"):
            self.robot.request("PUT", "/api/core/slam/v1/mapping/:enable", json={"enable": True})
        elif action in ("localization", "patrol"):
            self.robot.request("PUT", "/api/core/slam/v1/mapping/:enable", json={"enable": False})
            self.robot.request("PUT", "/api/core/slam/v1/localization/:enable", json={"enable": True})
        elif action == "idle":
            self.cancel()
            self.robot.request("PUT", "/api/core/slam/v1/mapping/:enable", json={"enable": False})
        elif action == "remote_control":
            self.cancel()
        else:
            raise BridgeError(f"unsupported G1 mode: {action}")

    def pose(self):
        return self.robot.json("GET", "/api/core/slam/v1/localization/pose")

    def set_pose(self, x, y, yaw):
        return self.robot.request("PUT", "/api/core/slam/v1/localization/pose",
                                  json={"x": x, "y": y, "z": 0, "yaw": yaw})

    def _send_action(self, name, options):
        result = self.robot.json("POST", self.ACTION_PATH,
                                 json={"action_name": f"slamtec.agent.actions.{name}",
                                       "options": options})
        if isinstance(result, dict) and result.get("result_code", 0) != 0:
            raise BridgeError(result.get("result_description") or str(result["result_code"]))
        return result

    def navigate(self, x, y, yaw=None):
        move = {"mode": 0}
        if yaw is not None:
            move.update({"yaw": yaw, "flags": ["with_yaw"]})
        result = self._send_action("MoveToAction", {"target": {"x": x, "y": y, "z": 0},
                                                     "move_options": move})
        with self.lock:
            self.task = {"status": 1, "task_type": 0, "targets": [{"x": x, "y": y}],
                         "yaw": yaw, "action_id": result.get("action_id")}

    def patrol(self, targets):
        if not targets:
            raise BridgeError("patrol route has no points")
        payload = [{"x": float(p["x"]), "y": float(p["y"]), "z": 0} for p in targets]
        result = self._send_action("SeriesMoveToAction", {"targets": payload,
                                                           "move_options": {"mode": 0}})
        with self.lock:
            self.task = {"status": 1, "task_type": 1, "targets": targets,
                         "yaw": None, "action_id": result.get("action_id")}

    def _abort_own_action(self):
        current = self.robot.json("GET", self.CURRENT_ACTION)
        own_id = self.task.get("action_id")
        if own_id is not None and current.get("action_id") != own_id:
            raise BridgeError("current G1D action belongs to another controller")
        self.robot.request("DELETE", self.CURRENT_ACTION)

    def cancel(self):
        with self.lock:
            if self.task["status"] in (1, 4):
                if self.task["status"] == 1:
                    self._abort_own_action()
                self.task["status"] = 5

    def pause(self):
        with self.lock:
            if self.task["status"] == 1:
                self._abort_own_action()
                self.task["status"] = 4

    def resume(self):
        with self.lock:
            if self.task["status"] != 4:
                return
            targets = list(self.task["targets"])
            task_type, yaw = self.task["task_type"], self.task["yaw"]
        if task_type == 0:
            self.navigate(targets[0]["x"], targets[0]["y"], yaw)
        else:
            pose = self.pose()
            closest = min(range(len(targets)),
                          key=lambda i: (targets[i]["x"] - pose["x"]) ** 2 +
                                        (targets[i]["y"] - pose["y"]) ** 2)
            self.patrol(targets[closest:] + targets[:closest])

    def task_status(self):
        with self.lock:
            status = self.task["status"]
            task_type = self.task["task_type"]
        if status != 1:
            return status, task_type
        try:
            action_id = self.task.get("action_id")
            path = f"{self.ACTION_PATH}/{action_id}" if action_id is not None else self.CURRENT_ACTION
            response = self.robot.request("GET", path)
        except requests.HTTPError as exc:
            if exc.response is not None and exc.response.status_code == 404:
                status = 3
            else:
                raise
        else:
            state = response.json().get("state", {})
            code = state.get("status")
            if code == 4:
                status = (5 if state.get("result") == -2 else
                          2 if state.get("result") == 0 else 3)
            elif code == 3:
                status = 4
            else:
                status = 1
        with self.lock:
            self.task["status"] = status
        return status, task_type

    def teleop(self, direction):
        if direction is None:
            return
        if self.task["status"] == 1:
            raise BridgeError("cancel navigation before remote control")
        return self._send_action("MoveByAction", {"direction": direction, "duration": 600})

    def pois(self):
        return self.robot.json("GET", self.POI_PATH)

    def list_points(self):
        points = []
        for poi in self.pois():
            pose = poi.get("pose") or {}
            if "x" not in pose or "y" not in pose:
                continue
            yaw = float(pose.get("yaw", 0))
            point = {"position": {"x": pose["x"], "y": pose["y"], "z": 0},
                     "orientation": quaternion_from_yaw(yaw),
                     "name": poi.get("metadata", {}).get("display_name", poi["id"])}
            points.append({"id": numeric_poi_id(poi["id"]), "frame_id": "map",
                           "point_list": json.dumps(point, ensure_ascii=False)})
        return points

    def _poi_by_numeric_id(self, point_id):
        return next((p for p in self.pois() if numeric_poi_id(p["id"]) == point_id), None)

    def add_point(self, point):
        position = point["position"]
        yaw = yaw_from_quaternion(point.get("orientation", {}))
        self.robot.request("POST", self.POI_PATH,
                           json={"id": str(uuid.uuid4()),
                                 "pose": {"x": position["x"], "y": position["y"], "yaw": yaw},
                                 "metadata": {"display_name": point.get("name", "位置点")}})

    def update_point(self, point_id, point):
        poi = self._poi_by_numeric_id(point_id)
        if poi is None:
            raise BridgeError("point does not exist")
        position = point["position"]
        poi["pose"] = {"x": position["x"], "y": position["y"],
                       "yaw": yaw_from_quaternion(point.get("orientation", {}))}
        poi.setdefault("metadata", {})["display_name"] = point.get("name", "位置点")
        self.robot.request("PUT", self.POI_PATH + "/" + poi["id"], json=poi)

    def delete_point(self, point_id):
        poi = self._poi_by_numeric_id(point_id)
        if poi is None:
            raise BridgeError("point does not exist")
        self.robot.request("DELETE", self.POI_PATH + "/" + poi["id"])

    def routes(self, map_id):
        return [r for r in self.catalog["routes"] if r["map_id"] == map_id]

    def add_route(self, map_id, frame_id, data):
        with self.lock:
            route_id = self.catalog["next_route_id"]
            self.catalog["next_route_id"] = route_id + 1
            self.catalog["routes"].append({"id": route_id, "map_id": map_id,
                                           "frame_id": frame_id, "point_list": data})
            self._save_catalog()

    def update_route(self, route_id, data):
        with self.lock:
            route = next((r for r in self.catalog["routes"] if r["id"] == route_id), None)
            if route is None:
                raise BridgeError("route does not exist")
            route["point_list"] = data
            self._save_catalog()

    def delete_route(self, route_id):
        with self.lock:
            self.catalog["routes"] = [r for r in self.catalog["routes"] if r["id"] != route_id]
            self._save_catalog()

    def walls(self):
        return self.robot.json("GET", self.WALL_PATH)

    def set_walls(self, lines):
        old = self.walls()
        self.robot.request("DELETE", self.WALL_PATH)
        if lines:
            body = [{"id": 0, "start": {"x": line["start"]["x"], "y": line["start"]["y"]},
                     "end": {"x": line["end"]["x"], "y": line["end"]["y"]}} for line in lines]
            try:
                self.robot.request("POST", self.WALL_PATH, json=body)
            except Exception:
                if old:
                    self.robot.request("POST", self.WALL_PATH, json=old)
                raise

    def dock_pose(self):
        docks = self.robot.json("GET", "/api/core/slam/v1/homedocks")
        return (docks[0].get("pose") or {}) if docks else None

    def register_dock(self):
        return self.robot.json("POST", "/api/core/slam/v1/homedocks/:register",
                               json={"metadata": {"display_name": "G1D充电桩"}})

    def start_dock(self):
        return self._send_action("GoHomeAction", {})["action_id"]

    def action_result(self, action_id):
        return self.robot.json("GET", f"{self.ACTION_PATH}/{action_id}").get("state", {})

    def cancel_dock(self, action_id):
        current = self.robot.json("GET", self.CURRENT_ACTION)
        if current.get("action_id") != action_id:
            raise BridgeError("current G1D action is not the docking action")
        self.robot.request("DELETE", self.CURRENT_ACTION)

    def power_status(self):
        return self.robot.json("GET", "/api/core/system/v1/power/status")
