#!/usr/bin/env python3
"""Expose the G1 navigation ROS contract while controlling a G1D over HTTP.

Run this node with aid_robot_msgs and rosbridge_server from the G1 ROS workspace.
Do not run the original G1 map manager, waypoint follower or mode server in the
same ROS domain: those nodes own the service names implemented here.
"""

import json
import os
import socket
import threading
import time

import rclpy
from aid_robot_msgs.msg import AidTaskStatus, StartToEndPoint
from aid_robot_msgs.srv import (
    DrawPicture, ForbiddenGet, ForbiddenSet, GetCurrentMap, GetDockPose, GetString,
    MapImage, MapLinkedDataList, MapList, MapOperation, MapOperationAdd,
    OperationAdd, OperationDelete, OperationUpdate, PatrolControl, SetString,
    SetCurrentMap, StatusChange,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String
from sensor_msgs.msg import BatteryState, LaserScan
from tf2_msgs.msg import TFMessage

from g1d_g1_core import (BridgeError, G1DCompat, RobotHttp, laser_to_scan,
                        quaternion_from_yaw, yaw_from_quaternion)


def set_ros_map(msg, data):
    msg.header.frame_id = "map"
    msg.info.width = data["width"]
    msg.info.height = data["height"]
    msg.info.resolution = data["resolution"]
    msg.info.origin.position.x = data["x0"]
    msg.info.origin.position.y = data["y0"]
    msg.info.origin.orientation.w = 1.0
    # G1's changeStr() interprets OccupancyGrid.data as base64 ASCII JPEG.
    msg.data = list(data["base64"])


class G1DROSBridge(Node):
    def __init__(self):
        super().__init__("g1d_g1_compat_bridge")
        robot_ip = self.declare_parameter("robot_ip", os.getenv("G1D_ROBOT_IP", "192.168.123.163")).value
        data_dir = self.declare_parameter("data_dir", os.getenv("G1D_BRIDGE_DATA", "bridge_data")).value
        self.api = G1DCompat(RobotHttp(robot_ip), data_dir)
        self.robot_ip = robot_ip
        self.group = ReentrantCallbackGroup()
        self._last_map_poll = 0.0
        self._last_pose_poll = 0.0
        self._last_action_poll = 0.0
        self._last_scan_poll = 0.0
        self._last_power_poll = 0.0
        self._dock_action_id = None
        self._pending_dock_result = None
        self._teleop_direction = None
        self._teleop_action_id = None
        self._teleop_at = 0.0
        self._teleop_lock = threading.RLock()

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map_base64", map_qos)
        self.pose_pub = self.create_publisher(PoseStamped, "/base_link_pose", 10)
        self.task_pub = self.create_publisher(AidTaskStatus, "/task_status", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.dock_state_pub = self.create_publisher(String, "/dock_state", 10)
        self.dock_result_pub = self.create_publisher(String, "/dock_result", 10)

        self.create_service(StatusChange, "/mode_set", self.on_mode, callback_group=self.group)
        self.create_service(MapList, "/get_map_list", self.on_map_list, callback_group=self.group)
        self.create_service(GetCurrentMap, "/get_current_map_id", self.on_current_map, callback_group=self.group)
        self.create_service(SetCurrentMap, "/set_current_map_id", self.on_set_map, callback_group=self.group)
        self.create_service(MapImage, "/get_map_image", self.on_map_image, callback_group=self.group)
        self.create_service(MapOperation, "/aid_save_map", self.on_save_map, callback_group=self.group)
        self.create_service(MapOperationAdd, "/add_map", self.on_add_map, callback_group=self.group)
        for name in ("/delete_map", "/delete_point", "/delete_waypoint"):
            self.create_service(OperationDelete, name, self.on_delete, callback_group=self.group)
        for name in ("/update_map", "/update_point", "/update_waypoint"):
            self.create_service(OperationUpdate, name, self.on_update, callback_group=self.group)
        for name in ("/add_point", "/add_waypoint"):
            self.create_service(OperationAdd, name, self.on_add, callback_group=self.group)
        for name in ("/get_map_point_list", "/get_map_waypoint_list"):
            self.create_service(MapLinkedDataList, name, self.on_data_list, callback_group=self.group)
        self.create_service(ForbiddenGet, "/get_forbidden", self.on_get_forbidden, callback_group=self.group)
        self.create_service(ForbiddenSet, "/set_forbidden", self.on_set_forbidden, callback_group=self.group)
        self.create_service(DrawPicture, "/aid_draw_forbidden_line", self.on_draw_wall,
                            callback_group=self.group)
        self.create_service(DrawPicture, "/map_editor", self.on_map_editor,
                            callback_group=self.group)
        self.create_service(GetDockPose, "/get_dock_pose", self.on_get_dock, callback_group=self.group)
        self.create_service(SetString, "/cmd_dock", self.on_dock, callback_group=self.group)
        self.create_service(GetString, "/get_ip_addresses", self.on_get_ip, callback_group=self.group)
        self.create_service(PatrolControl, "/patrol_control", self.on_patrol_control, callback_group=self.group)

        self.create_subscription(PoseStamped, "/nav_to_pose", self.on_nav, 10, callback_group=self.group)
        self.create_subscription(Path, "/patrol_path", self.on_patrol, 10, callback_group=self.group)
        self.create_subscription(PoseStamped, "/goal_pose_", self.on_recover, 10, callback_group=self.group)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 10,
                                 callback_group=self.group)
        self.create_subscription(Twist, "/cmd_vel_remote_ctrl", self.on_teleop, 10, callback_group=self.group)
        self.create_subscription(Empty, "/stop_patrol", self.on_stop, 10, callback_group=self.group)
        self.create_timer(1.0, self.poll, callback_group=self.group)
        self.get_logger().info(f"G1 interface -> G1D HTTP {robot_ip}:1448; data={data_dir}")

    def _error(self, name, exc):
        self.get_logger().error(f"{name}: {exc}")

    def _check_current_map(self, map_id):
        if int(map_id) != self.api.current_map_id() or not map_id:
            raise BridgeError("select this map before editing its G1D POIs or walls")

    def on_mode(self, request, response):
        try:
            self.api.set_mode(request.action)
            response.message = "ok"
        except Exception as exc:
            response.message = str(exc)
            self._error("mode_set", exc)
        return response

    def on_map_list(self, request, response):
        try:
            self.api.discover_current_map()
            response.map_list = json.dumps(self.api.list_maps(), ensure_ascii=False)
            response.success = True
        except Exception as exc:
            response.map_list = "[]"
            self._error("get_map_list", exc)
        return response

    def on_current_map(self, request, response):
        try:
            self.api.discover_current_map()
            entry = self.api._entry(self.api.current_map_id())
            response.success = entry is not None
            if entry:
                response.map_id = entry["id"]
                response.map_name = entry["name"]
                response.map_file = str(self.api.data_dir / f"{entry['id']}.explore")
        except Exception as exc:
            self._error("get_current_map_id", exc)
        return response

    def on_set_map(self, request, response):
        try:
            self.api.set_map(request.id)
            response.success = True
        except Exception as exc:
            self._error("set_current_map_id", exc)
        return response

    def on_map_image(self, request, response):
        try:
            self.api.discover_current_map()
            data = self.api.map_image(request.id)
            set_ros_map(response.map, data)
            response.map_file = str(self.api.data_dir / f"{request.id}.explore")
            response.success = True
        except Exception as exc:
            self._error("get_map_image", exc)
        return response

    def on_save_map(self, request, response):
        try:
            self.api.save_map()
            response.success, response.message = True, "ok"
        except Exception as exc:
            response.message = str(exc)
            self._error("aid_save_map", exc)
        return response

    def on_add_map(self, request, response):
        try:
            self.api.add_map(request.map_name)
            response.success, response.message = True, "ok"
        except Exception as exc:
            response.message = str(exc)
            self._error("add_map", exc)
        return response

    def on_delete(self, request, response):
        try:
            if request.data_type == "map":
                self.api.delete_map(request.id)
            elif request.data_type == "waypoint_node":
                self.api.delete_point(request.id)
            elif request.data_type == "waypoint":
                self.api.delete_route(request.id)
            else:
                raise BridgeError(f"unsupported delete type: {request.data_type}")
            response.success, response.message = True, "ok"
        except Exception as exc:
            response.message = str(exc)
            self._error("delete", exc)
        return response

    def on_update(self, request, response):
        try:
            data = json.loads(request.data)
            if request.data_type == "map":
                self.api.rename_map(request.id, data["name"])
            elif request.data_type == "waypoint_node":
                self.api.update_point(request.id, data)
            elif request.data_type == "waypoint":
                self.api.update_route(request.id, data["point_list"])
            else:
                raise BridgeError(f"unsupported update type: {request.data_type}")
            response.success, response.message = True, "ok"
        except Exception as exc:
            response.message = str(exc)
            self._error("update", exc)
        return response

    def on_add(self, request, response):
        try:
            self._check_current_map(request.map_id)
            if request.data_type == "waypoint_node":
                self.api.add_point(json.loads(request.data))
            elif request.data_type == "waypoint":
                self.api.add_route(request.map_id, request.frame_id, request.data)
            else:
                raise BridgeError(f"unsupported add type: {request.data_type}")
            response.success, response.message = True, "ok"
        except Exception as exc:
            response.message = str(exc)
            self._error("add", exc)
        return response

    def on_data_list(self, request, response):
        try:
            self._check_current_map(request.map_id)
            if request.data_type == "waypoint_node":
                rows = self.api.list_points()
            elif request.data_type == "waypoint":
                rows = [{"id": r["id"], "frame_id": r["frame_id"],
                         "point_list": r["point_list"]} for r in self.api.routes(request.map_id)]
            else:
                raise BridgeError(f"unsupported list type: {request.data_type}")
            response.message = json.dumps(rows, ensure_ascii=False)
            response.success = True
        except Exception as exc:
            response.message = "[]"
            self._error("get linked data", exc)
        return response

    def on_get_forbidden(self, request, response):
        try:
            self._check_current_map(request.map_id)
            for wall in self.api.walls():
                line = StartToEndPoint()
                line.start.x, line.start.y = float(wall["start"]["x"]), float(wall["start"]["y"])
                line.end.x, line.end.y = float(wall["end"]["x"]), float(wall["end"]["y"])
                response.lines.append(line)
            response.success, response.message = True, "ok"
        except Exception as exc:
            response.message = str(exc)
            self._error("get_forbidden", exc)
        return response

    def on_set_forbidden(self, request, response):
        try:
            self._check_current_map(request.map_id)
            lines = [{"start": {"x": line.start.x, "y": line.start.y},
                      "end": {"x": line.end.x, "y": line.end.y}} for line in request.lines]
            self.api.set_walls(lines)
            response.success, response.message = True, "ok"
        except Exception as exc:
            response.message = str(exc)
            self._error("set_forbidden", exc)
        return response

    def on_draw_wall(self, request, response):
        try:
            self._check_current_map(request.map_id)
            if request.type != "line":
                raise BridgeError("only forbidden lines are supported")
            # The frontend also calls /set_forbidden, which owns G1D walls.
            response.success, response.message = True, "G1D walls are stored by set_forbidden"
        except Exception as exc:
            response.message = str(exc)
            self._error("aid_draw_forbidden_line", exc)
        return response

    def on_map_editor(self, request, response):
        response.success = False
        response.message = "G1D HTTP API does not provide occupancy-grid erasing"
        return response

    def on_get_dock(self, request, response):
        try:
            self._check_current_map(request.map_id)
            dock = self.api.dock_pose()
            if dock:
                response.point.position.x = float(dock["x"])
                response.point.position.y = float(dock["y"])
                q = quaternion_from_yaw(float(dock.get("yaw", 0)))
                for key, value in q.items():
                    setattr(response.point.orientation, key, value)
                response.success, response.message = True, "ok"
            else:
                response.message = "no dock"
        except Exception as exc:
            response.message = str(exc)
            self._error("get_dock_pose", exc)
        return response

    def on_dock(self, request, response):
        try:
            if request.data == "set_dock_pose":
                self.api.register_dock()
                self._pending_dock_result = "set_dock_pose_succeeded"
            elif request.data == "dock":
                self._dock_action_id = self.api.start_dock()
            elif request.data == "cancel_dock":
                if self._dock_action_id is not None:
                    self.api.cancel_dock(self._dock_action_id)
                    self._dock_action_id = None
            else:
                raise BridgeError(f"unsupported dock command: {request.data}")
            response.success, response.message = True, "ok"
        except Exception as exc:
            response.message = str(exc)
            self._error("cmd_dock", exc)
        return response

    def on_get_ip(self, request, response):
        response.success = True
        response.message = "ok"
        response.result = socket.gethostbyname(socket.gethostname())
        return response

    def on_patrol_control(self, request, response):
        try:
            command = request.cmd.lower()
            if command == "cancel":
                self.api.cancel()
            elif command == "pause":
                self.api.pause()
            elif command == "resume":
                self.api.resume()
            else:
                raise BridgeError(f"unsupported patrol command: {command}")
            response.success, response.message = True, "ok"
        except Exception as exc:
            response.message = str(exc)
            self._error("patrol_control", exc)
        return response

    def on_nav(self, msg):
        try:
            pos = msg.pose.position
            q = msg.pose.orientation
            yaw = yaw_from_quaternion({"x": q.x, "y": q.y, "z": q.z, "w": q.w})
            self.api.navigate(pos.x, pos.y, yaw)
        except Exception as exc:
            self._error("nav_to_pose", exc)

    def on_patrol(self, msg):
        try:
            targets = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in msg.poses]
            self.api.patrol(targets)
        except Exception as exc:
            self._error("patrol_path", exc)

    def on_recover(self, msg):
        try:
            self.api._send_action("RecoverLocalizationAction", {})
        except Exception as exc:
            self._error("goal_pose_", exc)

    def on_initialpose(self, msg):
        try:
            pos = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = yaw_from_quaternion({"x": q.x, "y": q.y, "z": q.z, "w": q.w})
            self.api.set_pose(pos.x, pos.y, yaw)
        except Exception as exc:
            self._error("initialpose", exc)

    def on_teleop(self, msg):
        try:
            forward, turn = msg.linear.x, msg.angular.z
            direction = (0 if forward > 0.01 else 1 if forward < -0.01 else
                         3 if turn > 0.01 else 2 if turn < -0.01 else None)
            with self._teleop_lock:
                if direction is None:
                    if self._teleop_action_id is not None:
                        current = self.api.robot.json("GET", self.api.CURRENT_ACTION)
                        if current.get("action_id") == self._teleop_action_id:
                            self.api.robot.request("DELETE", self.api.CURRENT_ACTION)
                    self._teleop_direction = None
                    self._teleop_action_id = None
                elif direction != self._teleop_direction or time.monotonic() - self._teleop_at > 0.3:
                    result = self.api.teleop(direction)
                    self._teleop_direction = direction
                    self._teleop_action_id = result.get("action_id")
                    self._teleop_at = time.monotonic()
        except Exception as exc:
            self._error("cmd_vel_remote_ctrl", exc)

    def on_stop(self, msg):
        try:
            self.api.cancel()
        except Exception as exc:
            self._error("stop_patrol", exc)

    def poll(self):
        now = time.monotonic()
        if now - self._last_pose_poll >= 1.0:
            self._last_pose_poll = now
            try:
                pose = self.api.pose()
                msg = PoseStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.x = float(pose["x"])
                msg.pose.position.y = float(pose["y"])
                msg.pose.position.z = float(pose.get("z", 0))
                q = quaternion_from_yaw(float(pose.get("yaw", 0)))
                for key, value in q.items():
                    setattr(msg.pose.orientation, key, value)
                self.pose_pub.publish(msg)
                transform = TransformStamped()
                transform.header = msg.header
                transform.child_frame_id = "base_link"
                transform.transform.translation.x = msg.pose.position.x
                transform.transform.translation.y = msg.pose.position.y
                transform.transform.translation.z = msg.pose.position.z
                transform.transform.rotation = msg.pose.orientation
                self.tf_pub.publish(TFMessage(transforms=[transform]))
            except Exception as exc:
                self._error("pose poll", exc)
        if now - self._last_scan_poll >= 1.0:
            self._last_scan_poll = now
            try:
                observation = self.api.robot.json("GET", "/api/core/system/v1/laserscan")
                angle_min, angle_increment, ranges = laser_to_scan(observation)
                msg = LaserScan()
                msg.header.frame_id = "base_link"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.angle_min = angle_min
                msg.angle_increment = angle_increment
                msg.angle_max = angle_min + angle_increment * (len(ranges) - 1)
                msg.range_min = 0.01
                msg.range_max = max(ranges) + 1.0
                msg.ranges = ranges
                self.scan_pub.publish(msg)
            except Exception as exc:
                self._error("scan poll", exc)
        if now - self._last_map_poll >= 3.0:
            self._last_map_poll = now
            try:
                data = self.api.live_map()
                msg = OccupancyGrid()
                set_ros_map(msg, data)
                msg.header.stamp = self.get_clock().now().to_msg()
                self.map_pub.publish(msg)
            except Exception as exc:
                self._error("map poll", exc)
        if now - self._last_action_poll >= 1.0:
            self._last_action_poll = now
            try:
                status, task_type = self.api.task_status()
                msg = AidTaskStatus()
                msg.status = status
                msg.task_type = task_type
                self.task_pub.publish(msg)
            except Exception as exc:
                self._error("action poll", exc)
        if now - self._last_power_poll >= 2.0:
            self._last_power_poll = now
            try:
                power = self.api.power_status()
                battery = BatteryState()
                battery.header.stamp = self.get_clock().now().to_msg()
                battery.percentage = float(power["batteryPercentage"]) / 100.0
                battery.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING
                                               if power.get("isCharging") else
                                               BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
                self.battery_pub.publish(battery)
                dock_state = ("charging" if power.get("isCharging") else
                              "goto_dock_pose" if self._dock_action_id is not None else "undock")
                self.dock_state_pub.publish(String(data=dock_state))
            except Exception as exc:
                self._error("power poll", exc)
        if self._pending_dock_result is not None:
            self.dock_result_pub.publish(String(data=self._pending_dock_result))
            self._pending_dock_result = None
        if self._dock_action_id is not None:
            try:
                state = self.api.action_result(self._dock_action_id)
                if state.get("status") == 4:
                    result = "dock_succeeded" if state.get("result") == 0 else "dock_failed"
                    self.dock_result_pub.publish(String(data=result))
                    self._dock_action_id = None
            except Exception as exc:
                self._error("dock poll", exc)


def main():
    rclpy.init()
    node = G1DROSBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
