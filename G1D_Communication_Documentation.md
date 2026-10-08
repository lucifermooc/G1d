## G1D 通信接口文档（前端 / 其他模块 ↔ G1D）

> 2026-09-30 版。适用于装在 G1D 上的高通 8550 板卡（AidLux，ROS 2 Humble）运行的 G1D 服务。
> 接口名、消息类型和 JSON 格式与《G1 通信接口文档》一致，同一个前端可以同时连 G1 和 G1D。
> 标 **⚠** 的地方是 G1D 和 G1 行为不同、或者使用时容易踩坑的地方，汇总见《附录 A》。

### 第零章 系统结构与连接方式

#### 0.1 结构

````
前端 / 其他模块（ASR 等）
   │  WebSocket：ws://192.168.128.59:9090（rosbridge v2 协议，JSON）
   ▼
8550 板卡（ROS 2 Humble，ROS_DOMAIN_ID=71）
   ├─ g1d_g1_ros_bridge     G1 接口适配层：把 G1 的 ROS 服务 / 话题翻译成 G1D 的 HTTP 请求
   ├─ semantic_map_server   语义地图（第五章）
   └─ 网页：g1_web 控制台 :8090，G1 原版 Vue 前端 :8091，G1D 底盘控制台 :5001（第六章）
   │  HTTP：8550 网线 eth0 192.168.123.100 → G1D 192.168.123.163:1448
   ▼
G1D 底盘（Slamtec 导航底盘，型号 ares2-ys，软件 6.3.3）
````

G1D 自己完成建图、定位和路径规划；8550 上**没有** Nav2 和激光定位，适配层只负责接口翻译。

#### 0.2 地址和端口

| 用途 | 地址 |
|---|---|
| rosbridge（本文档的主要接口） | `ws://192.168.128.59:9090` |
| g1_web 控制台 | `http://192.168.128.59:8090/`（自动连同一台机器的 9090） |
| G1 原版 Vue 前端（备用） | `http://192.168.128.59:8091/`（在"设置"页把 ROS 地址填成 `192.168.128.59`） |
| G1D 底盘控制台（备用，HTTP 接口见第六章） | `http://192.168.128.59:5001/` |
| G1D 原生 HTTP 接口 | `http://192.168.123.163:1448`（只有机器人内部网段能访问，见 `docs/api_quick_reference.md`） |

`192.168.128.59` 是 8550 的 WiFi 地址，以现场为准。

**备用**：G1D 自带的开发计算单元 PC2（Jetson Orin NX，`192.168.123.164`，ROS 2 Foxy）上装了同一套服务、同样的端口和接口，
默认不启动，8550 出问题时再启用。PC2 只在机器人内部网段，外面要经 8550 访问（SSH 跳板 / 隧道）。⚠ 同一时间只用一套。

#### 0.3 rosbridge 调用约定

- 发布话题 `op: publish`，订阅 `op: subscribe`，调用服务 `op: call_service`。
- ⚠ 服务的 `args` 必须是**对象**：`{"action": "mapping"}`，不能写成数组。
- ⚠ **类型是 `string` 的字段，内容是 JSON 时也必须传字符串**（先 `JSON.stringify`），例如 `/add_point` 的 `data`。
- ⚠ 服务请求里**不能带服务定义里没有的字段**，否则 rosbridge 直接报错。
- ⚠ 适配层每秒向 G1D 查询一次，所以位姿、任务状态、模式状态这些话题是 **1 Hz**，状态变化最多晚约 1 秒。
- ⚠ ROS 2 只在板子本机内通信（`ROS_LOCALHOST_ONLY=1`，`ROS_DOMAIN_ID=71`，FastDDS），**其他机器请用 rosbridge**。
  这样 8550 和 PC2 不会互相收到对方的消息。确实要跨机器直连时，启动前设 `G1D_LOCALHOST_ONLY=0`。

#### 0.4 哪些接口会让机器人动

| 类别 | 接口 |
|---|---|
| **会让 G1D 移动** | `/nav_to_pose`、`/patrol_path`、`/patrol_control` 的 `resume`、`/cmd_vel_remote_ctrl`、`/cmd_dock` 的 `dock`、`/semantic_map/go`、`/goal_pose_`（全局重定位，是否转动未实测）；`/semantic_map/text_in`（默认找到就导航；启动参数 `text_in_action:=search` 时只搜索）。底盘控制台：`/api/navigate`、`/api/pois/navigate/<id>`、`/api/move`、`/api/home`、`/api/relocalize`、`/api/semantic/go` |
| 不会动，但会改 G1D 的状态 | `/mode_set`（`mapping` 会**清空 G1D 上的地图**）、`/initialpose`（立即改位姿）、`/set_current_map_id`（会**替换 G1D 上的地图**）、`/aid_save_map`、位置点增删改、`/set_forbidden`、`/cmd_dock` 的 `set_dock_pose`；底盘控制台里的开关、编辑、清空类接口 |
| 只读 | 其余所有接口 |

---

### 第一章 状态

#### 1.1 重定位（设置机器人位姿）⚠ 在 G1D 上立即生效

**Topic**: `/initialpose`　**Message Type**: `geometry_msgs/msg/PoseWithCovarianceStamped`

````
{
  "op": "publish",
  "topic": "/initialpose",
  "msg": {
    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
    "pose": {
      "pose": {
        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.7071068, "w": 0.7071068}
      },
      "covariance": [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685]
    }
  }
}
````

- 适配层把 x、y 和由四元数算出的朝向直接写进 G1D 的定位位姿（`PUT /api/core/slam/v1/localization/pose`）。
  `covariance` 和 `z` 不使用。
- ⚠ **立即生效，没有匹配校验**。位姿给错了，之后的导航也会跟着错。

**全局重定位**：**Topic** `/goal_pose_`　**Message Type** `geometry_msgs/msg/PoseStamped`

收到任意一条消息（内容不使用），就让 G1D 执行 `RecoverLocalizationAction`，在整张地图里重新找自己的位置。
⚠ 这是 G1D 的运动动作，执行时是否会原地转动没有实测过。

G1 文档里的 `/start_init_pose` 在 G1D 上没有节点订阅，发了没有反应。

#### 1.2 机器人当前位置

**Topic**: `/base_link_pose`　**Message Type**: `geometry_msgs/msg/PoseStamped`（1 Hz）

````
{"op": "subscribe", "topic": "/base_link_pose", "type": "geometry_msgs/msg/PoseStamped"}
````

接收：

````
{
  "op": "publish",
  "topic": "/base_link_pose",
  "msg": {
    "header": {"stamp": {"sec": 1790739886, "nanosec": 464323573}, "frame_id": "map"},
    "pose": {
      "position": {"x": 0.172, "y": 1.815, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.6816, "w": 0.7317}
    }
  }
}
````

同时发布 `/tf`（`map → base_link`，内容相同，1 Hz）。

#### 1.3 电量

**Topic**: `/battery_state`　**Message Type**: `sensor_msgs/msg/BatteryState`（每 2 秒一次）

````
{"op": "subscribe", "topic": "/battery_state", "type": "sensor_msgs/msg/BatteryState"}
````

⚠ 只有两个字段有意义，其余字段都是 0 或空：

| 字段 | 含义 |
|---|---|
| `percentage` | 电量，**0~1**（0.68 = 68%） |
| `power_supply_status` | `1` 充电中，`2` 未充电 |

#### 1.4 机器人模式状态

**Topic**: `/robot_status`　**Message Type**: `std_msgs/msg/String`（1 Hz）

````
{"op": "subscribe", "topic": "/robot_status", "type": "std_msgs/msg/String"}
````

`msg.data` 的格式是 `"<SLAM 状态>+<控制模式>"`，例如 `"localization+patrol"`：

| 部分 | 取值 | G1D 上怎么来的 |
|---|---|---|
| SLAM 状态 | `mapping` / `localization` / `idle` | ⚠ 直接读 G1D 的两个开关：建图开关开着就是 `mapping`；否则定位开关开着是 `localization`；都关是 `idle` |
| 控制模式 | `patrol` / `remote_control` / `idle` | 最近一次 `/mode_set` 设置的值（第四章） |

⚠ G1D 出厂时建图和定位两个开关都是开的，这时显示 `mapping+idle`，G1D 会一边定位一边持续修改地图。
导航前建议先 `/mode_set patrol`（前端会自动做），把建图开关关掉。

#### 1.5 激光

**Topic**: `/scan`　**Message Type**: `sensor_msgs/msg/LaserScan`（1 Hz，`frame_id` = `base_link`）

由 G1D 的激光数据按角度重新排成等间隔扫描，约 1800~1900 束。只用于显示（例如重定位时对照墙面）。

#### 1.6 IP 地址 ⚠（在 8550 上不可用）

**Service**: `/get_ip_addresses`　**Service Type**: `aid_robot_msgs/srv/GetString`

返回 `{"success": true, "message": "ok", "result": "127.0.1.1"}`：取的是主机名对应的地址，8550 上是 `127.0.1.1`，没有实际用处。

---
### 第二章 地图管理

⚠ **G1D 自己只保存"当前这一张地图"**。"多张地图"是适配层在 8550 上维护的目录（`~/G1D/bridge_data/`：`catalog.json`
加上每张图的 `.explore` 栅格快照和 `.stcm` 复合地图）：

- 第一次调用任何地图接口时，自动把 G1D 上现有的地图导入为一张名为"G1D当前地图"的地图。
- "切换地图"就是把保存的 `.stcm` 上传到 G1D，替换 G1D 上的地图。
- ⚠ 不要删除 `~/G1D/bridge_data/`，否则地图名、旧地图和巡逻路线都会丢失。
- ⚠ 如果有别的程序（G1D 官方 App、底盘控制台）在适配层之外换了 G1D 的地图，适配层不知道，这时请在前端对要用的地图重新"使用"一次。
  G1D 断电重启后加载的是它自己最后一次保存的地图，同样建议重新"使用"一次。

#### 2.1 地图位置点管理

⚠ 位置点就是 **G1D 的 POI**（兴趣点），直接存在 G1D 上。一个位置点的内容（`data` / `point_list`）是一个 **JSON 字符串**，格式与 G1 相同：

````
{
  "position": {"x": 1.0, "y": 2.0, "z": 0.0},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
  "name": "茶水间",
  "description": "有饮水机和咖啡机，可以接水、喝水、泡茶"
}
````

- `description` 是语义地图用的可选字段（第五章）。
- 在 G1D 上存成：`id`（UUID）、`pose {x, y, yaw}`、`metadata {display_name, description}`。`z` 不保存，`orientation` 换算成朝向角 `yaw`。
- ⚠ 接口里的位置点 `id`（uint32）由 G1D 的 UUID 算出：`crc32(UUID) & 0x7FFFFFFF`。同一个点的 id 不会变，但**不是**自增的小整数。
- POI 和虚拟墙是 G1D 地图（`.stcm`）里的图层。每次增 / 删 / 改位置点、`/set_forbidden` 之后，适配层都会让 G1D 保存地图
  （`POST /api/multi-floor/map/v1/stcm/:save`），并刷新 8550 上这张图的 `.stcm` 快照，这样 G1D 重启、切走再切回来，点位都还在。
  新建图还没保存时加的点不在这时落盘，而是随 2.2.3 保存地图一起保存。
- 用别的工具（G1D 官方 App、底盘控制台）建的 POI 也会出现在列表里；没有 `display_name` 时名称显示为 UUID。
  ⚠ 底盘控制台的 `/api/pois` 增删改**不会**自动保存，改完要调用 `/api/mapping/save`，否则 G1D 重启后就没了。
- ⚠ **只能编辑当前地图的位置点**：`map_id` 必须等于 `/get_current_map_id` 返回的 id，否则返回
  `success: false`，`message` 为 `"select this map before editing its G1D POIs or walls"`。

##### 2.1.1 新增位置点

**Service**: `/add_point`　**Service Type**: `aid_robot_msgs/srv/OperationAdd`

````
- 入参：
  - uint32 map_id     当前地图 id
  - string frame_id   固定 "map"
  - string data       位置点内容（⚠ JSON 字符串）
  - string data_type  固定 "waypoint_node"
- 返回：
  - bool success
  - string message    成功 "ok"；失败时是原因
````

````
{
  "op": "call_service",
  "service": "/add_point",
  "args": {
    "map_id": 1,
    "frame_id": "map",
    "data": "{\"position\":{\"x\":1.0,\"y\":2.0,\"z\":0.0},\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0},\"name\":\"茶水间\",\"description\":\"可以接水、喝水\"}",
    "data_type": "waypoint_node"
  }
}
````

响应：`{"op": "service_response", "service": "/add_point", "values": {"success": true, "message": "ok"}, "result": true}`

⚠ 响应里没有新点的 id，要拿 id 请重新查一次列表（2.1.4）。

##### 2.1.2 删除位置点

**Service**: `/delete_point`　**Service Type**: `aid_robot_msgs/srv/OperationDelete`

````
{"op": "call_service", "service": "/delete_point", "args": {"id": 1587334518, "data_type": "waypoint_node"}}
````

响应：`{"success": true, "message": "ok"}`；id 不存在时 `{"success": false, "message": "point does not exist"}`。

##### 2.1.3 修改位置点

**Service**: `/update_point`　**Service Type**: `aid_robot_msgs/srv/OperationUpdate`

````
{
  "op": "call_service",
  "service": "/update_point",
  "args": {
    "id": 1587334518,
    "data": "{\"position\":{\"x\":1.5,\"y\":2.0,\"z\":0.0},\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.7071068,\"w\":0.7071068},\"name\":\"茶水间\",\"description\":\"可以接水、喝水、泡茶\"}",
    "data_type": "waypoint_node"
  }
}
````

- 坐标、朝向、名称按传入的内容整体替换。
- ⚠ 和 G1 不同：`data` 里**没有** `description` 字段时，保留原来的描述（方便不认识描述字段的旧前端）；
  要清空描述请传 `"description": ""`。

响应：`{"success": true, "message": "ok"}`；id 不存在时 `{"success": false, "message": "point does not exist"}`。

##### 2.1.4 获取位置点列表

**Service**: `/get_map_point_list`　**Service Type**: `aid_robot_msgs/srv/MapLinkedDataList`

````
{"op": "call_service", "service": "/get_map_point_list", "args": {"map_id": 1, "data_type": "waypoint_node"}}
````

⚠ `data_type` 必须是 `"waypoint_node"`（`"waypoint"` 是巡逻路线，见 2.3）。响应的 `message` 是 JSON 字符串，
解析后每一项的 `point_list` **又是一个 JSON 字符串**，需要再解析一次：

````
{
  "op": "service_response",
  "service": "/get_map_point_list",
  "values": {
    "success": true,
    "message": "[{\"id\": 1587334518, \"frame_id\": \"map\", \"point_list\": \"{\\\"position\\\": {\\\"x\\\": 1.0, \\\"y\\\": 2.0, \\\"z\\\": 0}, \\\"orientation\\\": {\\\"x\\\": 0.0, \\\"y\\\": 0.0, \\\"z\\\": 0.0, \\\"w\\\": 1.0}, \\\"name\\\": \\\"茶水间\\\", \\\"description\\\": \\\"可以接水、喝水\\\"}\"}]"
  },
  "result": true
}
````

````js
const points = JSON.parse(res.message).map(row => ({ id: row.id, ...JSON.parse(row.point_list) }));
````

失败时（例如 `map_id` 不是当前地图、连不上 G1D）`success: false`、`message: "[]"`。

#### 2.2 地图管理

编号沿用 G1 文档。

##### 2.2.1 开始建图（新地图）⚠ 会清空 G1D 上的地图

````
{"op": "call_service", "service": "/mode_set", "args": {"action": "mapping"}}
````

G1D 上依次执行：

1. 把 G1D 上现有的地图备份到 8550 的 `~/G1D/bridge_data/backup/<时间>.explore/.stcm`（保留最近 10 份；地图是空的就跳过）。
2. 清空 G1D 上的地图（`DELETE /api/core/slam/v1/maps`）。
3. 打开建图开关，从空白开始建图。

- 响应：`{"message": "ok"}`；失败时 `message` 是原因（⚠ G1 返回 `"err"`，G1D 返回具体原因）。
- ⚠ 已经在建图时再调用 `mapping`，会再清空一次（前一次没保存的建图结果只剩备份）。
- ⚠ 备份只能在 8550 上手动恢复，没有接口。要保留的地图请先按 2.2.3 保存。
- 清空后 G1D 上原来的 POI、虚拟墙会不会一起被清掉，以及机器人位姿会不会回到原点，**没有核实过**。

##### 2.2.1b 继续建图（G1D 支持，G1 后端不支持）

````
{"op": "call_service", "service": "/mode_set", "args": {"action": "continue_mapping"}}
````

只打开 G1D 的建图开关，在 G1D 当前的地图上接着建。保存方式同 2.2.3（需要一个新的地图名字）。

##### 2.2.2 取消建图（不保存）

````
{"op": "call_service", "service": "/mode_set", "args": {"action": "idle"}}
````

取消正在执行的导航任务，关掉建图开关，不保存。
⚠ 刚建了一半的地图还留在 G1D 上；下一次 `/mode_set localization` 或 `patrol` 时，适配层会把"当前地图"重新上传到 G1D（第四章）。

##### 2.2.3 保存地图（两步）

**第 1 步**：**Service** `/aid_save_map`　**Type** `aid_robot_msgs/srv/MapOperation`

````
{"op": "call_service", "service": "/aid_save_map", "args": {"map_file_name": "/maps/1790604029000"}}
````

- 让 G1D 把地图保存到它自己的存储里（`POST /api/multi-floor/map/v1/stcm/:save`），并把地图取回 8550 暂存；然后**关掉建图开关**（和 G1 一样，保存后结束建图）。
- ⚠ `map_file_name` 在 G1D 上不使用，填什么都可以（字段必须有）。
- 响应：`{"success": true, "message": "ok"}`；失败时 `message` 是原因。

**第 2 步**：**Service** `/add_map`　**Type** `aid_robot_msgs/srv/MapOperationAdd`

````
{"op": "call_service", "service": "/add_map", "args": {"map_name": "一楼办公室", "map_file": "/maps/1790604029000"}}
````

- 把第 1 步暂存的地图写进目录，并**设为当前地图**。没有做第 1 步时，直接取 G1D 当前的地图。
- ⚠ `map_file` 不使用。
- 响应：`{"success": true, "message": "ok"}`；重名时 `"map name already exists"`，名字为空时 `"map name is empty"`。

##### 2.2.4 实时地图

**Topic**: `/map_base64`　**Message Type**: `nav_msgs/msg/OccupancyGrid`

````
{"op": "subscribe", "topic": "/map_base64", "type": "nav_msgs/msg/OccupancyGrid", "compression": "png", "throttle_rate": 3000}
````

- ⚠ **每 3 秒发布一次 G1D 当前的地图，不管是不是在建图**（QoS 为 transient local，订阅后马上收到最近一张）。
- ⚠ `msg.data` 不是栅格数据，而是 **JPEG 图片的 base64 字符串逐字节放进去**（与 G1 相同，格式和颜色见 2.2.15）。
- ⚠ 地图大时每张很大（20 m × 53 m 约 150 KB），只在需要时订阅，并加 `compression: "png"`、`throttle_rate`。

##### 2.2.5 切换地图 ⚠ 会替换 G1D 上的地图

**Service**: `/set_current_map_id`　**Service Type**: `aid_robot_msgs/srv/SetCurrentMap`

````
{"op": "call_service", "service": "/set_current_map_id", "args": {"id": 2}}
````

- 响应：`{"success": true}`（只有这一个字段）。
- 要切换的地图不是 G1D 上现有的那张时，把它的 `.stcm` 上传到 G1D，替换 G1D 上的地图。
  ⚠ G1D 上没保存的修改会丢失，而且这一步**不做备份**。
- 接着调用 `/mode_set {"action": "localization"}`。切换后请确认机器人位置对不对，不对就按 1.1 重定位。

##### 2.2.9 禁行线（= G1D 虚拟墙）

**① 保存**：**Service** `/set_forbidden`　**Type** `aid_robot_msgs/srv/ForbiddenSet`

````
{
  "op": "call_service",
  "service": "/set_forbidden",
  "args": {
    "map_id": 1,
    "frame_id": "map",
    "lines": [{"start": {"x": 1.0, "y": 2.0, "z": 0.0}, "end": {"x": 3.0, "y": 4.0, "z": 0.0}}]
  }
}
````

- 直接写到 G1D 的虚拟墙（`/api/core/artifact/v1/lines/walls`），**整体覆盖**原有的虚拟墙；`lines` 为空就是全部删除。
- 只能编辑当前地图（同 2.1）；`z` 不使用。
- 响应：`{"success": true, "message": "ok"}`；失败时 `message` 是原因。

**读取**：**Service** `/get_forbidden`　**Type** `aid_robot_msgs/srv/ForbiddenGet`

````
{"op": "call_service", "service": "/get_forbidden", "args": {"map_id": 1}}
````

响应：`{"success": true, "message": "ok", "lines": [{"start": {...}, "end": {...}}]}`

**② `/aid_draw_forbidden_line`**（`aid_robot_msgs/srv/DrawPicture`）：⚠ 在 G1D 上什么都不做，
`type` 为 `"line"` 时直接返回 `{"success": true, "message": "G1D walls are stored by set_forbidden"}`。
保留它只是为了兼容 G1 前端的调用顺序。

##### 2.2.10 橡皮擦 ⚠（G1D 不支持）

**Service** `/map_editor`（`aid_robot_msgs/srv/DrawPicture`）：G1D 的 HTTP 接口不能修改栅格地图，总是返回
`{"success": false, "message": "G1D HTTP API does not provide occupancy-grid erasing"}`。

##### 2.2.11 删除地图

**Service**: `/delete_map`　**Service Type**: `aid_robot_msgs/srv/OperationDelete`

````
{"op": "call_service", "service": "/delete_map", "args": {"id": 2, "data_type": "map"}}
````

- 删除 8550 上保存的这张图和它的巡逻路线，不影响 G1D。
- ⚠ 不能删除当前地图：`{"success": false, "message": "cannot delete the active map"}`。
- id 不存在：`"map does not exist"`。

##### 2.2.12 重命名地图

**Service**: `/update_map`　**Service Type**: `aid_robot_msgs/srv/OperationUpdate`

````
{"op": "call_service", "service": "/update_map", "args": {"id": 1, "data": "{\"name\": \"一楼办公室\"}", "data_type": "map"}}
````

响应：`{"success": true, "message": "ok"}`；地图不存在或名字为空时 `"map does not exist or name is empty"`。

##### 2.2.13 获取地图列表

**Service**: `/get_map_list`　**Service Type**: `aid_robot_msgs/srv/MapList`

````
{"op": "call_service", "service": "/get_map_list", "args": {}}
````

响应（`map_list` 是 JSON 字符串）：

````
{"values": {"success": true, "map_list": "[{\"id\": 1, \"name\": \"G1D当前地图\", \"create_timestamp\": 1790737444}]"}}
````

##### 2.2.14 获取当前地图

**Service**: `/get_current_map_id`　**Service Type**: `aid_robot_msgs/srv/GetCurrentMap`

````
{"op": "call_service", "service": "/get_current_map_id", "args": {}}
````

响应：

````
{"values": {"success": true, "map_id": 1, "map_name": "G1D当前地图", "map_file": "/home/aidlux/G1D/bridge_data/1.explore"}}
````

##### 2.2.15 获取地图图片

**Service**: `/get_map_image`　**Service Type**: `aid_robot_msgs/srv/MapImage`

````
{"op": "call_service", "service": "/get_map_image", "args": {"id": 1}}
````

- 响应：`{"success": true, "map": <OccupancyGrid>, "map_file": "..."}`。
- 要的是 G1D 上现有的那张图时，返回 G1D 的实时地图；否则返回 8550 上保存的快照。
- ⚠ `map.data` 是 **JPEG 图片的 base64 字符串逐字节放进去**，不是占据栅格：

````js
const b64 = new TextDecoder().decode(new Uint8Array(map.data));
img.src = "data:image/jpg;base64," + b64;
// 像素 ↔ 地图坐标：u = (x - origin.x) / resolution；v = height - (y - origin.y) / resolution
````

- 分辨率 0.05 m/像素；`map.info` 里有 `width`、`height`、`resolution` 和 `origin`。
- 颜色与 G1 一致：障碍物黑色 RGB(0,0,0)，空闲区 RGB(127,145,200)，未探索区 RGB(82,108,170)。

#### 2.3 巡逻路线存储

`/add_waypoint`、`/update_waypoint`、`/delete_waypoint`（服务类型同 2.1 的增删改），
`/get_map_waypoint_list`（同 2.1.4，`data_type` 为 `"waypoint"`）。

- ⚠ G1D 没有路线数据库，路线只存在 8550 的目录里，`data` 原样保存、原样返回（格式由前端自己定）。
- 新增：`map_id` 必须是当前地图。修改：`data` 是 `{"point_list": <路线内容>}` 的 JSON 字符串。
- 列表每一项：`{"id": 1, "frame_id": "map", "point_list": <新增时的 data>}`。
- 存路线不会让机器人动。执行巡逻用 3.2 的 `/patrol_path`。

---
### 第三章 导航管理

G1D 自己规划路径，适配层把目标转成 G1D 的运动动作（Action）。
⚠ 和 G1 不同，G1D 不需要先进入导航模式也能导航；但前端在导航前会调用 `/mode_set {"action": "patrol"}`，
把建图开关关掉，避免边走边改地图，推荐保留这个做法。

#### 3.1 单点导航

##### 3.1.1 开始导航

**Topic**: `/nav_to_pose`　**Message Type**: `geometry_msgs/msg/PoseStamped`

````
{
  "op": "publish",
  "topic": "/nav_to_pose",
  "msg": {
    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
    "pose": {
      "position": {"x": 2.0, "y": 3.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }
}
````

- 转成 G1D 的 `MoveToAction`：目标点 (x, y)，到达后转到 `orientation` 对应的朝向。
- 进度看 `/task_status`（3.3）。
- ⚠ 已有任务在执行时，适配层会直接下发新目标，不会先取消旧任务。G1D 是否打断旧任务取决于固件，**没有实测过**。
  稳妥做法和 G1 一样：先 `/patrol_control {"cmd": "cancel"}` 再发。
- 话题没有应答。下发失败（例如 G1D 不在线）只记在 8550 的 `~/g1d_logs/bridge.log` 里，`/task_status` 不会变成"执行中"。

##### 3.1.2 `/move_base_simple/goal` ⚠（不支持）

G1D 上没有节点订阅这个话题。请用 3.1.1。

##### 3.1.3 暂停 / 继续 / 取消（单点和巡逻共用）⚠

**Service**: `/patrol_control`　**Service Type**: `aid_robot_msgs/srv/PatrolControl`

````
{"op": "call_service", "service": "/patrol_control", "args": {"cmd": "pause"}}
{"op": "call_service", "service": "/patrol_control", "args": {"cmd": "resume"}}
{"op": "call_service", "service": "/patrol_control", "args": {"cmd": "cancel"}}
````

- ⚠ G1D 没有真正的"暂停"：`pause` 是**取消 G1D 当前的动作**并记住目标，机器人停下；`resume` 是重新下发。
  单点导航重新去原来的目标；巡逻从离机器人最近的那个点开始，再按原顺序走完剩下的点。
- ⚠ 只能控制通过本接口（`/nav_to_pose`、`/patrol_path`）发起的任务。G1D 当前的动作如果是别的程序发起的
  （底盘控制台、G1D 官方 App），返回 `{"success": false, "message": "current G1D action belongs to another controller"}`，
  这时请用底盘控制台的 `/api/stop`（第六章）。
- 没有任务时调用也返回 `success: true`，什么都不做。`cmd` 不认识时返回 `"unsupported patrol command: <cmd>"`。
- 成功响应：`{"success": true, "message": "ok"}`。

另有 **Topic** `/stop_patrol`（`std_msgs/msg/Empty`）：收到后等同于 `cancel`。

#### 3.2 路线巡逻

##### 3.2.1 多点巡逻

**Topic**: `/patrol_path`　**Message Type**: `nav_msgs/msg/Path`

````
{
  "op": "publish",
  "topic": "/patrol_path",
  "msg": {
    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
    "poses": [
      {"header": {"frame_id": "map"}, "pose": {"position": {"x": 2.0, "y": 3.0, "z": 0.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
      {"header": {"frame_id": "map"}, "pose": {"position": {"x": 5.0, "y": 1.0, "z": 0.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}
    ]
  }
}
````

- 转成 G1D 的 `SeriesMoveToAction`，按顺序依次经过各点。
- **和 G1 一样一直循环**：走到最后一个点后，适配层马上再下发一圈，从第一个点开始（两个点就是来回往返）。
  直到 `cancel` / `pause`（3.1.3）、`/stop_patrol`、或者发了新的导航 / 巡逻才停；巡逻中 `/task_status` 一直是"执行中"。
- 某一段没走到（G1D 动作结果失败），从离机器人最近的点重新开始这一圈；**连续失败 3 次**才停下，`/task_status` 变成"失败"。
  走完一整圈后失败次数清零。G1D 报告动作被取消（例如别的程序取消了它）时直接停下，不再续圈。
- 续圈靠适配层每秒一次的状态轮询，圈与圈之间会有不到 1 秒的停顿；适配层（8550 上的服务）停了，巡逻也只会走完当前这一圈。
- 不支持 G1 的圈数 / 时长限制（`patrol_count` / `patrol_duration`）：这边的 `PatrolControl.srv` 只有 `cmd`。
- ⚠ 各点的 `orientation` 不使用。至少要有 1 个点。
- 暂停 / 继续 / 取消见 3.1.3。

#### 3.3 任务状态

**Topic**: `/task_status`　**Message Type**: `aid_robot_msgs/msg/AidTaskStatus`（1 Hz）

````
{"op": "subscribe", "topic": "/task_status", "type": "aid_robot_msgs/msg/AidTaskStatus"}
````

接收：`{"msg": {"status": 1, "task_type": 0}}`

| status | 含义 | 在 G1D 上的来源 | task_type | 含义 |
|---|---|---|---|---|
| 0 | 空闲 | 还没有发过任务 | 0 | 单点导航 |
| 1 | 执行中 | G1D 动作正在执行 | 1 | 巡逻 |
| 2 | 成功 | G1D 动作结束，结果成功（巡逻不会出现，它会一直续圈） | | |
| 3 | 失败 | G1D 动作结束，结果失败；或动作已经不存在（巡逻是连续失败 3 次） | | |
| 4 | 暂停 | 调用了 `pause`，或 G1D 报告动作暂停 | | |
| 5 | 取消 | 调用了 `cancel`，或 G1D 报告动作被取消 | | |

⚠ 只反映通过本接口发起的任务；底盘控制台、G1D 官方 App 发起的动作不会显示在这里。任务结束后保持最后的状态，直到下一个任务。

#### 3.4 遥控 ⚠（只有方向，没有速度）

**Topic**: `/cmd_vel_remote_ctrl`　**Message Type**: `geometry_msgs/msg/Twist`

````
{"op": "publish", "topic": "/cmd_vel_remote_ctrl", "msg": {"linear": {"x": 0.3, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}}
````

| 消息 | G1D 动作 |
|---|---|
| `linear.x` > 0.01 | 前进 |
| `linear.x` < -0.01 | 后退 |
| `angular.z` > 0.01（且没有前后速度） | 左转 |
| `angular.z` < -0.01（且没有前后速度） | 右转 |
| 全部为 0 | 立即停止 |

- ⚠ 数值大小不起作用，速度由 G1D 自己的最大速度参数决定（可用底盘控制台 `/api/speed` 调）。
- ⚠ 每条指令只让 G1D 动 600 ms，**按住期间要持续发送（建议 5~10 Hz）**，松开时发一条全 0。
- ⚠ 有导航任务在执行时遥控会被拒绝（只记日志）；先 `/patrol_control cancel`，或 `/mode_set remote_control`（它会先取消任务）。

#### 3.5 回充

**Service**: `/cmd_dock`　**Service Type**: `aid_robot_msgs/srv/SetString`

| `data` | 作用 |
|---|---|
| `"set_dock_pose"` | 把机器人当前位置注册为充电桩（G1D `homedocks/:register`） |
| `"dock"` | **机器人自己回充电桩**（G1D `GoHomeAction`） |
| `"cancel_dock"` | 取消回充 |

````
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "dock"}}
````

响应：`{"success": true, "message": "ok"}`。⚠ `undock`（离桩）不支持。

**充电桩位置**：**Service** `/get_dock_pose`（`aid_robot_msgs/srv/GetDockPose`，`args: {"map_id": 1}`，只能查当前地图）。
有充电桩时返回 `{"success": true, "message": "ok", "point": <Pose>}`；没有时 `{"success": false, "message": "no dock"}`。

**充电状态**：**Topic** `/dock_state`（`std_msgs/msg/String`，每 2 秒一次）：`charging` 充电中 / `goto_dock_pose` 正在回充 / `undock` 不在充电。

**回充结果**：**Topic** `/dock_result`（`std_msgs/msg/String`）：`set_dock_pose_succeeded`、`dock_succeeded`、`dock_failed`。

---

### 第四章 模式设置

**Service**: `/mode_set`　**Service Type**: `aid_robot_msgs/srv/StatusChange`

````
{"op": "call_service", "service": "/mode_set", "args": {"action": "patrol"}}
````

响应：`{"message": "ok"}`；失败时 `message` 是具体原因（⚠ G1 返回 `"err"`）。

⚠ G1D 切换模式只是开关几个功能，**一两秒内完成**，不像 G1 要重启定位 / 导航进程。

| action | G1D 上做的事 | 之后的 `/robot_status` |
|---|---|---|
| `mapping` | 备份 → 清空 G1D 上的地图 → 打开建图开关（2.2.1） | `mapping+idle` |
| `continue_mapping` | 打开建图开关，在现有地图上继续建 | `mapping+idle` |
| `localization` | 如果 G1D 上的地图不是"当前地图"，先把当前地图上传回去；然后关建图、开定位 | `localization+idle` |
| `patrol` | 同 `localization`（导航前调用） | `localization+patrol` |
| `remote_control` | 取消正在执行的导航任务 | `<SLAM 状态>+remote_control` |
| `idle` | 取消任务，关掉建图开关（定位保持打开） | `localization+idle` |

其它值返回 `"unsupported G1 mode: <action>"`。

**典型流程**

- 新建一张地图：`mapping` → 推着或遥控机器人把场地走一遍 → `/aid_save_map` → `/add_map`（自动成为当前地图）
  → `localization` → 给位置点写名称和描述（2.1、第五章）。
- 在现有地图上补建：`continue_mapping` → 走需要补的区域 → `/aid_save_map` → `/add_map`（新名字）。
- 使用某张地图：`/set_current_map_id` → `localization` → 确认位置（不对就 1.1 重定位）→ `patrol` → 导航 / 巡逻。

---
### 第五章 语义地图

说一句话（例如"我想喝水"），在**当前地图的位置点**里找到意思最接近的一个（例如"茶水间"），可以直接让机器人过去。
由 8550 上的 `semantic_map_server` 节点提供，接口与 G1 完全相同。

工作方式：

1. 位置点就是 2.1 的位置点（G1D POI）。**给位置点写上 `description`**（写成人会怎么说，例如"口渴""接水"），匹配会准很多。
2. 每个点的"名称 + 描述"由 BGE-M3 模型转成向量；**坐标不参与匹配**。
3. 搜索时把这句话也转成向量，找最相似的点；最高分低于阈值（默认 0.52）就算"没找到"，机器人不动。
4. 位置点有增删改后，**下一次搜索会自动重建**，一般不需要手动重建。

8550 上的部署：模型 `~/models/bge-m3`，CPU 推理（开机时预加载约 5 秒，之后每句约 0.1~1 秒；备用的 PC2 用 GPU，每句约 0.1 秒）；向量库用本地文件
`~/maps/semantic_qdrant`（不需要 Qdrant 服务）；调试记录 `~/maps/semantic_map_log.jsonl`。

##### 5.1 搜索（不会让机器人动）

**Service**: `/semantic_map/search`　**Service Type**: `aid_robot_msgs/srv/SetString`

````
{"op": "call_service", "service": "/semantic_map/search", "args": {"data": "我想喝水"}}
````

响应：`success` 表示服务是否执行成功；`message` 是 JSON 字符串：

````
{
  "values": {
    "success": true,
    "message": "{\"query\": \"我想喝水\", \"found\": true, \"map_id\": 1, \"threshold\": 0.52, \"rebuilt\": false,
                \"best\": {\"id\": 1587334518, \"name\": \"茶水间\", \"description\": \"有饮水机和咖啡机，可以接水、喝水、泡茶\",
                          \"frame_id\": \"map\", \"x\": 12.5, \"y\": 4.0, \"z\": 0.0, \"yaw\": 1.5708,
                          \"orientation\": {\"x\": 0.0, \"y\": 0.0, \"z\": 0.7071, \"w\": 0.7071}, \"score\": 0.6767},
                \"candidates\": [ ...前 3 名，格式同 best... ],
                \"source\": \"search\", \"issues\": []}"
  },
  "result": true
}
````

| 字段 | 含义 |
|---|---|
| `found` | 最高分 ≥ 阈值时为 true |
| `best` | 找到时是第一名，否则为 null |
| `candidates` | 前 3 名（不管是否达到阈值，方便调试）；当前地图没有位置点时为 `[]` |
| `score` | 余弦相似度，越接近 1 越像 |
| `rebuilt` | 这次搜索前是否因为位置点有变化而自动重建了语义库 |
| `source` | 请求来源（见 5.5），调试用 |
| `issues` | 收到的原文里发现的问题，例如 `首尾有空白：换行`、`含不可见字符：零宽空格`；没问题时为 `[]` |

`data` 可以是一句普通文本，也可以是 JSON 字符串 `{"text": "我想喝水", "source": "asr"}`（见 5.5）。
送进模型前只去掉不可见字符和首尾空白，不改错字、不删标点。

失败时 `success: false`，`message` 是原因，例如 `no current map (select a map first)`、
`service /get_map_point_list not available`、`empty query`。

##### 5.2 一句话导航（会让机器人动）

**Service**: `/semantic_map/go`　**Service Type**: `aid_robot_msgs/srv/SetString`

````
{"op": "call_service", "service": "/semantic_map/go", "args": {"data": "我要上厕所"}}
````

- 先做 5.1 的搜索。找到时，如果已有任务在执行或暂停，先 `/patrol_control cancel`，再把该点发到 `/nav_to_pose`
  （和 3.1.1 完全相同，进度看 `/task_status`）。
- 响应的 `message` 同 5.1，多一个字段 `"navigating": true/false`（没找到时为 false，机器人不动）。
- 建议调用前先 `/mode_set patrol`（第三章开头），网页会自动做。

##### 5.3 手动重建语义库

**Service**: `/semantic_map/rebuild`　**Service Type**: `std_srvs/srv/Trigger`

````
{"op": "call_service", "service": "/semantic_map/rebuild", "args": {}}
````

响应：`{"success": true, "message": "{\"map_id\": 1, \"count\": 6}"}`（`count` 是写入的位置点数）。

##### 5.4 参数与调阈值

参数在 `ros2_ws/src/semantic_map_ros/config/semantic_map.yaml`；8550 的启动脚本 `deploy/run_g1d.sh` 用 launch 参数覆盖了其中几个。
⚠ launch 参数会覆盖配置文件里的同名参数（`qdrant_path`、`model_name`、`use_fp16`、`score_threshold`、`text_in_action`），
要改这几个请改 `run_g1d.sh` 里的 launch 参数。

| 参数 | 8550 上的值 | 说明 |
|---|---|---|
| `qdrant_path` | `~/maps/semantic_qdrant` | 非空 = 本地文件模式（不连 Qdrant 服务） |
| `model_name` | `~/models/bge-m3` | 本地模型目录 |
| `use_fp16` | false | CPU 推理 |
| `score_threshold` | 0.52 | 最高分低于它算"没找到" |
| `top_k` | 3 | 返回的候选数 |
| `text_in_topic` | `/semantic_map/text_in` | 话题输入（5.5），填 `""` 关闭 |
| `text_in_action` | `go` | 话题输入收到后：`go` 找到就导航；`search` 只搜索 |
| `log_file` | `~/maps/semantic_map_log.jsonl` | 调试记录文件（5.6），超过 20 MB 滚动成 `.1` |
| `history_size` | 200 | `/semantic_map/history` 返回的最近条数 |

用真实位置点调阈值：`python3 -m semantic_map_ros.evaluate --queries <句子.json>`（格式见
`config/eval_queries_example.json`），它会给出推荐阈值。

##### 5.5 其他模块（ASR 等）怎么把文本发过来

两种方式任选。**建议带上 `source`**，调试记录里才分得清是谁发的。

**方式一：调服务**（能拿到应答，知道找没找到）：同 5.1 / 5.2，`data` 用 JSON 字符串带上来源：

````
{"op": "call_service", "service": "/semantic_map/go",
 "args": {"data": "{\"text\": \"我想喝水\", \"source\": \"asr\"}"}}
````

ROS 2 节点里直接调：`ros2 service call /semantic_map/search aid_robot_msgs/srv/SetString "{data: '我想喝水'}"`

**方式二：发话题**（只管发，不等结果；适合 ASR 识别完一句就往外丢）

**Topic**: `/semantic_map/text_in`　**Message Type**: `std_msgs/msg/String`

````
{"op": "publish", "topic": "/semantic_map/text_in", "msg": {"data": "我想喝水"}}
````

- `data` 同样可以是 `{"text": "...", "source": "asr"}`；不带 `source` 时来源记为 `topic`。
- ⚠ 默认**找到就导航**（`text_in_action: go`，和 5.2 一样会让机器人动），结果看 5.6 的调试记录。
  只想搜索时，在 `run_g1d.sh` 的语义地图启动参数里加 `text_in_action:=search` 后重启服务。
- 话题没有应答；出错（例如发来空内容、没有选地图）只记在调试记录里。

##### 5.6 调试：看别人发来的原文和匹配得分

每一次请求（网页、服务、话题，成功或失败）都会留下一条记录，用来判断问题出在哪一边：
**原文就不对**（错字、多了空格 / 换行 / 乱码）是发送方的问题；**原文没问题但得分低或匹配错**，是位置点描述或阈值的问题。

看记录的三种方法：

1. **网页**：g1_web 控制台"点位与语义"页 → 调试记录。实时显示所有来源的请求，看不见的字符用黄底标出；
   上面的"模拟 ASR"输入框可以往 `/semantic_map/text_in` 发一句，检查话题链路。
2. **话题** `/semantic_map/debug`（`std_msgs/msg/String`，每条一个 JSON）。
3. **文件** 8550 上的 `~/maps/semantic_map_log.jsonl`（每行一条，重启不丢）。

另有服务 `/semantic_map/history`（`std_srvs/srv/Trigger`），`message` 是最近 200 条记录的 JSON 数组。

一条记录：

````
{"time": "2026-09-29T10:15:02.318", "entry": "topic", "source": "asr",
 "raw": " 我想​喝水\n", "raw_visible": "␠我想⟨U+200B⟩喝水\\n", "length": 7,
 "issues": ["首尾有空白：换行、空格", "含不可见字符：零宽空格"], "text": "我想喝水", "go": false,
 "ok": true, "map_id": 1, "found": true, "best": "茶水间", "threshold": 0.52,
 "candidates": [{"id": 1587334518, "name": "茶水间", "score": 0.6767}, {"id": 902133, "name": "会议室", "score": 0.41},
                {"id": 55123, "name": "前台", "score": 0.39}],
 "rebuilt": false, "navigating": false, "elapsed_ms": 86}
````

| 字段 | 含义 |
|---|---|
| `entry` | 从哪个入口进来：`search` / `go` 服务，或 `topic` 话题 |
| `source` | 发送方自己报的来源；没报时等于 `entry` |
| `raw` | 收到的原文，一个字符都不改 |
| `raw_visible` | 同上，但把看不见的字符写出来：`␠` 首尾空格、`\n` 换行、`⟨U+200B⟩` 零宽空格等 |
| `issues` | 原文的问题（首尾空白、不可见字符、中间换行、乱码替换符 U+FFFD、清理后为空） |
| `text` | 实际送进模型的文本 |
| `candidates` | 前 3 名及得分；`found=false` 时可以看到最高分差阈值多少 |
| `ok` / `error` | 是否处理成功；失败原因 |
| `navigating` | 是否已发出导航目标 |
| `elapsed_ms` | 处理耗时（含模型推理） |

---
### 第六章 底盘控制台 HTTP 接口（端口 5001，备用）

G1D 原有的网页控制台（Flask），直接调 G1D 的 HTTP 接口，**不经过第一~五章的适配层**。一般用 rosbridge 接口即可，
这一章给需要直接用 HTTP 的场景。

- 基地址：`http://192.168.128.59:5001`，请求和响应都是 JSON。
- ⚠ **先连接**：`POST /api/connect`，body `{"ip": "192.168.123.163"}`，成功返回 `{"ok": true, "capabilities": {...}}`。
  连接状态只保存在进程内存里，服务重启后要重新连接；没连接时其它接口会报错（例如 `未连接底盘`）。
- ⚠ 这里发起的运动**不会**出现在 `/task_status` 里，`/patrol_control` 也取消不了，要停请用 `POST /api/stop`。
- 大部分写操作成功时返回 `{"ok": true}`；下发动作类接口返回 G1D 的原始应答（含 `action_id`）。

**状态与地图**

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/status` | 电量、当前动作、当前楼层 |
| GET | `/api/map?h=<hash>` | 地图：`{"img": <PNG base64，黑=障碍、白=空闲、灰=未探索>, "hash", "x0", "y0", "cols", "rows", "res", "pose", "quality", "mapping", "localization"}`；`h` 等于当前 hash 时只返回 `{"unchanged": true, "hash", "pose", ...}`；地图为空时 `{"empty": true, ...}` |
| GET | `/api/annotations` | 一次取回 POI、禁行区、虚拟墙、充电桩 |
| GET | `/api/path` | 当前剩余路径点 |
| GET | `/api/health`，DELETE `/api/health/<错误码>` | 查看 / 清除 G1D 的健康告警 |

**建图 / 地图文件**（不会动，会改 G1D 状态）

| 方法 | 路径 | 说明 |
|---|---|---|
| PUT | `/api/mapping/enable`，body `{"enable": true}` | 建图开关（与定位开关互不影响） |
| PUT | `/api/localization/enable`，body `{"enable": true}` | 定位开关 |
| POST | `/api/mapping/save` | 让 G1D 保存当前地图到它自己的存储 |
| POST | `/api/map/load` | 让 G1D 重新加载它保存的地图 |
| POST | `/api/map/export` | 把地图导出到 8550 的 `slam_ws/map/`（`.pgm` + `.yaml` + `.stcm`） |
| DELETE | `/api/map/clear` | ⚠ **清空 G1D 上的地图**（不备份） |
| POST | `/api/map/reset-origin` | 把地图原点移到机器人当前位置 |

**运动**（⚠ 会让机器人动，除了 `/api/stop` 和 `/api/speed`）

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/api/navigate`，body `{"x": 2.0, "y": 3.0, "yaw": 1.57}` | 导航到目标点（`yaw` 可省略） |
| POST | `/api/pois/navigate/<poi_id>` | 导航到某个 POI（`poi_id` 是 G1D 里 POI 的 id：适配层建的是 UUID，底盘控制台建的默认用名称） |
| POST | `/api/move`，body `{"direction": 0}` | 遥控一次 600 ms：0 前、1 后、2 右转、3 左转 |
| POST | `/api/home` | 回充电桩 |
| POST | `/api/relocalize`，body 可选 `{"x1", "y1", "x2", "y2"}` | 重定位（给矩形就在矩形内搜索，不给就全图） |
| POST | `/api/stop` | 停止 G1D 当前的动作 |
| PUT | `/api/speed`，body `{"linear": 0.5}` | 设置最大移动速度（m/s） |

**POI（位置点）、充电桩、禁行区、虚拟墙**（不会动，会改 G1D 状态）

| 方法 | 路径 | 说明 |
|---|---|---|
| GET / POST | `/api/pois` | 列表 / 新增，body `{"name": "茶水间", "description": "接水、喝水", "x": 1.0, "y": 2.0, "yaw": 0}` |
| PUT | `/api/pois/<poi_id>`，body `{"name": "...", "description": "..."}` | 改名称和 / 或描述（只改传了的字段） |
| DELETE | `/api/pois/<poi_id>` | 删除 |
| GET / POST | `/api/docks` | 列表 / 按坐标新增充电桩，body `{"x", "y", "yaw", "name"}` |
| POST | `/api/docks/register`，body `{"name": "充电桩"}` | 把当前位置注册为充电桩 |
| DELETE | `/api/docks/<id>`、`/api/docks/clear` | 删除一个 / 全部 |
| GET / POST | `/api/areas` | 禁行区列表 / 新增，body `{"x1", "y1", "x2", "y2"}`（矩形对角） |
| DELETE | `/api/areas/<id>`、`/api/areas/clear` | 删除一个 / 全部 |
| GET / POST | `/api/walls` | 虚拟墙列表 / 新增，body `{"x1", "y1", "x2", "y2"}` |
| DELETE | `/api/walls/<id>`、`/api/walls/clear` | 删除一条 / 全部 |

POI 就是 2.1 的位置点：两边读写的是同一份 `metadata.display_name` / `metadata.description`。

**语义地图**（与第五章功能相同，但独立运行：向量库 `~/maps/semantic_qdrant_flask`，调试记录 `~/maps/semantic_map_flask_log.jsonl`）

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/semantic/status?preload=1` | `{"model_loaded", "load_error", "threshold", "model"}`；带 `preload=1` 时在后台加载模型 |
| POST | `/api/semantic/search`，body `{"text": "我想喝水", "source": "asr"}` | 搜索，不会动 |
| POST | `/api/semantic/go`，同上 | ⚠ 搜索，找到就导航 |
| POST | `/api/semantic/rebuild` | 重建，返回 `{"count": N}` |
| GET | `/api/semantic/history` | 最近的调试记录 |

语义接口的响应：成功 `{"ok": true, "result": <同 5.1 的结果对象，这里是对象不是字符串>}`；
失败 `{"ok": false, "error": "原因"}`，HTTP 400（例如 `未连接底盘`），依赖没装好时 HTTP 503。

---

### 附录 A 与 G1 接口的差异汇总

| 项目 | G1 | G1D |
|---|---|---|
| 重定位 `/initialpose` | 暂未实现 / 未验证 | 立即设置 G1D 的位姿，不做匹配校验（1.1） |
| `/robot_status` 的 SLAM 状态 | 正在运行的进程（建图 / 定位） | G1D 建图、定位两个开关；出厂两个都开，显示 `mapping`（1.4） |
| 开始建图 `mapping` | 启动 Lightning 建一张新图 | 先备份，再清空 G1D 上的地图重新建（2.2.1） |
| 继续建图 `continue_mapping` | 后端不支持 | 支持（2.2.1b） |
| 取消建图 `idle` | 停止建图 | 关掉建图开关；建了一半的图留在 G1D 上，下次进入定位时换回当前地图（2.2.2） |
| `/aid_save_map` | `map_file_name` 决定保存目录；保存后回到 idle | `map_file_name` 不使用；G1D 保存后关掉建图开关（2.2.3） |
| `/add_map` 的 `map_file` | 必须和第 1 步一致 | 不使用 |
| 位置点存储和 id | 地图数据库，id 自增 | G1D POI，id = `crc32(UUID) & 0x7FFFFFFF`（2.1） |
| 位置点编辑范围 | 任意地图 | 只能编辑当前地图（2.1） |
| `/update_point` 没带 `description` | 整体替换（描述被清空） | 保留原描述（2.1.3） |
| 切换地图 `/set_current_map_id` | 只改数据库，要再 `localization` 才加载 | 立即把地图上传到 G1D，替换 G1D 上的地图（2.2.5） |
| 禁行线 | `/set_forbidden` 存库 + `/aid_draw_forbidden_line` 画禁行图 | 直接写 G1D 虚拟墙；`/aid_draw_forbidden_line` 什么都不做（2.2.9） |
| 橡皮擦 `/map_editor` | 支持 | 不支持（2.2.10） |
| `/map_base64` | 建图时发布 | 一直每 3 秒发布（2.2.4） |
| 导航前提 | 必须在定位模式（Nav2 在运行） | 不需要，但建议先 `patrol` 关掉建图（第三章） |
| 有任务时再发 `/nav_to_pose` | 新目标被忽略 | 直接下发，G1D 是否打断旧任务未实测（3.1.1） |
| `pause` / `resume` | 暂停 / 继续 | 取消 G1D 动作并记住目标 / 重新下发（3.1.3） |
| 巡逻 `/patrol_path` | 一直循环；朝向改成朝向下一个点 | 一直循环（适配层自动续圈）；朝向不使用（3.2.1） |
| 遥控 `/cmd_vel_remote_ctrl` | 见 G1 文档 | 只认方向，数值不起作用；每条动 600 ms，要持续发送（3.4） |
| `/mode_set` 失败时的 `message` | `"err"` | 具体原因（第四章） |
| 模式切换耗时 | 要重启定位 / 导航进程 | 一两秒 |
| 状态话题频率 | 见 G1 文档 | 位姿、任务、模式 1 Hz；电量、充电状态每 2 秒 |
| `/get_ip_addresses` | 读 wlan0 的地址 | 返回 `127.0.1.1`，不可用（1.6） |

---

### 附录 B 运行与维护（8550）

- **开机自启**：systemd 服务 `g1d`（开机自动启动 rosbridge、语义地图、三个网页和适配层；任何一个进程退出，5 秒后整体重启）。
  - 查看：`systemctl status g1d`、`journalctl -u g1d -f`
  - 重启 / 停止：`bash ~/start_g1d.sh` / `bash ~/stop_g1d.sh`（不需要密码；停止后下次开机仍会自启）
- **日志**：`~/g1d_logs/`，每个服务一个文件（`bridge.log`、`semantic_map.log`、`rosbridge.log` 等），每次启动时覆盖。
- **G1D 关机或网线断开时**：适配层不会退出，每秒在 `bridge.log` 记一次连接错误；G1D 恢复后自动接上，不用重启服务。
- **代码**：全部在 `slam_ws` 里（适配层 `g1d_g1_ros_bridge.py` / `g1d_g1_core.py`，网页 `g1_web/`，ROS 包 `ros2_ws/src/`，
  部署脚本 `deploy/`，说明 `deploy/README.md`）；从电脑同步到 8550 用 `bash deploy/sync_to_8550.sh`。
- ⚠ **重启服务会让适配层忘记正在执行的任务**（之后 `pause` / `cancel` 控制不了它），重启前先确认没有导航任务在执行。
