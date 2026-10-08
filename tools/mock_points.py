#!/usr/bin/env python3
"""临时测试用：模拟适配层的 /get_current_map_id 和 /get_map_point_list（不连机器人），测完就停。"""
import json

import rclpy
from aid_robot_msgs.srv import GetCurrentMap, MapLinkedDataList
from rclpy.node import Node

POINTS = [
    (101, "茶水间", "有饮水机和咖啡机，可以接水、喝水、泡茶", 3.0, 1.0),
    (102, "会议室", "开会、讨论、投影汇报", -2.0, 4.0),
    (103, "前台", "访客登记、拿快递、问路", 0.5, -3.0),
    (104, "洗手间", "上厕所、洗手", -4.0, -1.0),
]


class Mock(Node):
    def __init__(self):
        super().__init__("mock_points_for_test")
        self.create_service(GetCurrentMap, "/get_current_map_id", self.cur)
        self.create_service(MapLinkedDataList, "/get_map_point_list", self.points)

    def cur(self, req, res):
        res.success, res.map_id, res.map_name, res.map_file = True, 7, "测试地图", ""
        return res

    def points(self, req, res):
        rows = [{"id": i, "frame_id": "map", "point_list": json.dumps(
            {"position": {"x": x, "y": y, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
             "name": n, "description": d}, ensure_ascii=False)} for i, n, d, x, y in POINTS]
        res.success, res.message = True, json.dumps(rows, ensure_ascii=False)
        return res


rclpy.init()
rclpy.spin(Mock())
