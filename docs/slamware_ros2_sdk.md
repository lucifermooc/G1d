# Slamware ROS2 SDK 接口文档

> 基于 `slamware_ros2_sdk_linux-aarch64-gcc11`，包版本 2.0.0

---

## 架构

```
底盘 Slamware 引擎（内部 C++ SDK）
        ↕  librpos_robotplatforms_rpslamware.a
slamware_ros_sdk_server_node（ROS2 节点）
        ↕  Topic / Service
ROS2 程序 / 控制脚本
```

HTTP API（端口 1448）和 ROS2 topic 是**同一套引擎的两个出口**，任意一个控制都会影响另一个看到的状态。

---


## 话题发布频率

| 话题 | 周期 | 频率 |
|------|------|------|
| `/slamware_ros_sdk_server_node/odom` | 0.05s | 20Hz |
| `/slamware_ros_sdk_server_node/robot_pose` | 0.05s | 20Hz |
| `/slamware_ros_sdk_server_node/scan` | 0.1s | 10Hz |
| `/slamware_ros_sdk_server_node/map` | 0.2s | 5Hz |
| `/slamware_ros_sdk_server_node/global_plan_path` | 0.1s | 10Hz |
| `/slamware_ros_sdk_server_node/imu_raw_data` | 0.1s | 10Hz |
| `/slamware_ros_sdk_server_node/virtual_walls` | 1.0s | 1Hz |
| `/slamware_ros_sdk_server_node/virtual_tracks` | 1.0s | 1Hz |
| `/slamware_ros_sdk_server_node/basic_sensors_values` | 1.0s | 1Hz |

---

## 输出话题（节点发布）

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `.../map` | `nav_msgs/OccupancyGrid` | 占据栅格地图（建图时实时更新） |
| `.../map_metadata` | `nav_msgs/MapMetaData` | 地图元数据（分辨率、尺寸、原点） |
| `.../scan` | `sensor_msgs/LaserScan` | 激光雷达 2D 扫描 |
| `.../odom` | `nav_msgs/Odometry` | 里程计（需定位激活） |
| `.../robot_pose` | `geometry_msgs/PoseStamped` | 机器人位姿（需定位激活） |
| `.../global_plan_path` | `nav_msgs/Path` | 规划路径（导航时） |
| `.../imu_raw_data` | `sensor_msgs/Imu` | IMU 原始数据 |
| `.../imu_raw_mag_data` | `sensor_msgs/MagneticField` | IMU 磁力计 |
| `.../robot_basic_state` | `slamware_ros_sdk/RobotBasicState` | 电量、温度、建图/定位状态 |
| `.../robot_device_info` | `slamware_ros_sdk/RobotDeviceInfo` | 设备型号、固件版本等 |
| `.../basic_sensors_info` | `slamware_ros_sdk/BasicSensorInfoArray` | 传感器配置信息 |
| `.../basic_sensors_values` | `slamware_ros_sdk/BasicSensorValueDataArray` | 传感器实时数值 |
| `.../virtual_walls` | `slamware_ros_sdk/Line2DFlt32Array` | 虚拟墙列表 |
| `.../virtual_tracks` | `slamware_ros_sdk/Line2DFlt32Array` | 虚拟轨道列表 |
| `.../state` | `slamware_ros_sdk/...` | 节点整体状态 |

---

## 控制话题（节点订阅）

### SLAM 控制

| 话题 | 消息类型 | 对应 HTTP API |
|------|----------|--------------|
| `.../set_map_localization` | `SetMapLocalizationRequest` | `PUT /slam/v1/localization/:enable` |
| `.../set_map_update` | `SetMapUpdateRequest` | `PUT /slam/v1/mapping/:enable` |
| `.../recover_localization` | `RecoverLocalizationRequest` | `RecoverLocalizationAction` |
| `.../sync_map` | `SyncMapRequest` | — |
| `.../clear_map` | `ClearMapRequest` | `DELETE /slam/v1/maps` |
| `.../set_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `PUT /slam/v1/localization/pose` |

#### SetMapLocalizationRequest
```
bool enabled
```

#### SetMapUpdateRequest
```
bool enabled
MapKind kind   # 0=EXPLORERMAP（常用），3=SLAMMAP
```

#### RecoverLocalizationRequest
```
RectFlt32 area             # 搜索范围（全为0则全图搜索）
LocalizationOptions options
  OptionalInt32 max_time_ms
  OptionalLocalizationMovement mvmt_type  # 0=不动, 1=只旋转, 2=任意移动
```

---

### 运动控制

| 话题 | 消息类型 | 说明 | 对应 HTTP API |
|------|----------|------|--------------|
| `.../move_to` | `MoveToRequest` | 导航到目标点 | `MoveToAction` |
| `.../move_to_locations` | `MoveToLocationsRequest` | 依次经过多个点 | — |
| `.../move_by_direction` | `MoveByDirectionRequest` | 按方向遥控 | `MoveByAction` |
| `.../move_by_theta` | `MoveByThetaRequest` | 按角度遥控 | — |
| `.../rotate` | `RotateRequest` | 旋转指定角度 | — |
| `.../rotate_to` | `RotateToRequest` | 旋转到指定朝向 | — |
| `.../move_line` | `MoveLineRequest` | 沿线段移动 | — |
| `.../move_lines` | `MoveLinesRequest` | 沿折线移动 | — |
| `.../go_home` | `GoHomeRequest` | 回充电桩 | `GoHomeAction` |
| `.../cancel_action` | `CancelActionRequest` | 取消当前动作 | `DELETE /actions/:current` |

#### MoveToRequest
```
geometry_msgs/Point location   # 目标坐标 (x, y, z=0)
MoveOptions options
  MoveOptionFlag opt_flags     # 见下方标志位
  OptionalFlt64 speed_ratio    # 速度比例 0~1
float32 yaw                    # 到达时的朝向（弧度）
```

#### MoveByDirectionRequest
```
ActionDirection direction
  # FORWARD=0, BACKWARD=1, TURNRIGHT=2, TURNLEFT=3
MoveOptions options
```

#### MoveOptionFlag 标志位
```
NONE                       = 0
APPENDING                  = 1   # 追加到队列（不打断当前）
MILESTONE                  = 2   # 里程碑点
NO_SMOOTH                  = 4   # 不做轨迹平滑
KEY_POINTS                 = 8   # 关键点模式
PRECISE                    = 16  # 精确到达
WITH_YAW                   = 32  # 到达时控制朝向
RETURN_UNREACHABLE_DIRECTLY= 64  # 不可达时立即返回
KEY_POINTS_WITH_OA         = 128 # 关键点+避障
```

---

### 虚拟障碍物

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `.../add_line` | `AddLineRequest` | 添加单条虚拟线 |
| `.../add_lines` | `AddLinesRequest` | 批量添加虚拟线 |
| `.../remove_line` | `RemoveLineRequest` | 删除指定 ID 的线 |
| `.../clear_lines` | `ClearLinesRequest` | 清空某类线 |
| `.../move_line` | `MoveLineRequest` | 修改线位置 |
| `.../move_lines` | `MoveLinesRequest` | 批量修改 |

#### AddLineRequest
```
ArtifactUsage usage
  # VIRTUAL_WALL=0, VIRTUAL_TRACK=1
Line2DFlt32 line
  uint32 id
  Vec2DFlt32 start   # (x, y)
  Vec2DFlt32 end     # (x, y)
```

---

## Services（服务调用）

| 服务 | 类型 | 说明 |
|------|------|------|
| `/sync_get_stcm` | `SyncGetStcm` | 下载 STCM 地图文件（含 POI、禁区、充电桩等全部数据） |
| `/sync_set_stcm` | `SyncSetStcm` | 上传 STCM 地图文件并设置初始位姿 |

#### SyncGetStcm
```
# Request: 空
---
# Response
uint8[] raw_stcm    # STCM 二进制内容
```

#### SyncSetStcm
```
# Request
uint8[] raw_stcm
geometry_msgs/Pose robot_pose   # 机器人初始位姿
---
# Response: 空
```

---

## RobotBasicState 消息
```
bool is_map_building_enabled    # 是否在建图
bool is_localization_enabled    # 是否在定位
int32 localization_quality      # 定位质量 0~100
float32 board_temperature       # 主板温度（℃）
float32 battery_percentage      # 电量百分比
bool is_dc_in                   # 是否接外部电源
bool is_charging                # 是否在充电
```

---

## 使用示例

### 开启建图（ROS2 命令行）
```bash
ros2 topic pub --once /slamware_ros_sdk_server_node/set_map_update \
  slamware_ros_sdk/msg/SetMapUpdateRequest \
  "{enabled: true, kind: {kind: 0}}"
```

### 开启定位
```bash
ros2 topic pub --once /slamware_ros_sdk_server_node/set_map_localization \
  slamware_ros_sdk/msg/SetMapLocalizationRequest \
  "{enabled: true}"
```

### 导航到目标点
```bash
ros2 topic pub --once /slamware_ros_sdk_server_node/move_to \
  slamware_ros_sdk/msg/MoveToRequest \
  "{location: {x: 1.5, y: 2.0, z: 0.0}, options: {opt_flags: {flags: 0}}, yaw: 0.0}"
```

### 回充电桩
```bash
ros2 topic pub --once /slamware_ros_sdk_server_node/go_home \
  slamware_ros_sdk/msg/GoHomeRequest "{}"
```

### 自动重定位（原地旋转搜索）
```bash
ros2 topic pub --once /slamware_ros_sdk_server_node/recover_localization \
  slamware_ros_sdk/msg/RecoverLocalizationRequest \
  "{area: {x: 0, y: 0, w: 0, h: 0}, options: {mvmt_type: {movement: 1}}}"
```

### 添加虚拟墙
```bash
ros2 topic pub --once /slamware_ros_sdk_server_node/add_line \
  slamware_ros_sdk/msg/AddLineRequest \
  "{usage: {usage: 0}, line: {id: 1, start: {x: 1.0, y: 0.0}, end: {x: 1.0, y: 3.0}}}"
```

### 下载 STCM 地图（Python）
```python
import rclpy
from rclpy.node import Node
from slamware_ros_sdk.srv import SyncGetStcm

rclpy.init()
node = Node('map_backup')
cli = node.create_client(SyncGetStcm, '/sync_get_stcm')
cli.wait_for_service()
future = cli.call_async(SyncGetStcm.Request())
rclpy.spin_until_future_complete(node, future)
with open('backup.stcm', 'wb') as f:
    f.write(bytes(future.result().raw_stcm))
print('地图已保存')
```

---

## 与 HTTP API 对照

| 功能 | HTTP API | ROS2 |
|------|----------|------|
| 开始建图 | `PUT /slam/v1/mapping/:enable` | pub `set_map_update` |
| 开始定位 | `PUT /slam/v1/localization/:enable` | pub `set_map_localization` |
| 重定位 | `RecoverLocalizationAction` | pub `recover_localization` |
| 导航 | `MoveToAction` | pub `move_to` |
| 遥控 | `MoveByAction` | pub `move_by_direction` |
| 回桩 | `GoHomeAction` | pub `go_home` |
| 停止 | `DELETE /actions/:current` | pub `cancel_action` |
| 虚拟墙 | `POST /lines/walls` | pub `add_line` |
| 地图 | `GET /maps/explore`（二进制） | sub `/map`（OccupancyGrid） |
| 位姿 | `GET /localization/pose` | sub `/robot_pose` |
| 下载地图 | `GET /maps/stcm` | service `/sync_get_stcm` |
| 上传地图 | `PUT /maps/stcm` | service `/sync_set_stcm` |

> HTTP API 支持 POI、禁行区、充电桩管理，ROS SDK 暂不支持这些接口。
