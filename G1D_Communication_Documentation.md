## G1D 通信接口文档（前端 / 其他模块 ↔ G1D）

> 2026-10-08 版。适用于 G1D 上运行的 G1D 服务（当前运行在 G1D 自带的 PC2 板卡上，见 0.2）。
> 接口名、消息类型和 JSON 格式与《G1 通信接口文档》一致，同一个前端可以同时连 G1 和 G1D。
> 标 **⚠** 的地方是 G1D 和 G1 行为不同、或者使用时容易踩坑的地方，汇总见《附录 A》。

**2026-10-08 更新内容**（之前看过本文档的，只看这些即可）：

| 内容 | 位置 |
|---|---|
| 充电桩接口重写：多个充电桩、查询列表、注册、在地图上放置、删除、指定回哪个桩、当前使用的桩 | 3.5.1 ~ 3.5.8 |
| **低电量自动回充**：电量低于阈值（默认 10%，可设置）自动结束导航 / 巡逻，回当前使用的桩 | 3.5.9 |
| 新的回充事件 `low_battery_dock`；怎么判断"要去充电了"（给语音播报等用） | 3.5.10 |
| `/get_dock_pose` 有多个桩时返回最近的桩 | 3.5.11 |
| ⚠ 订阅话题**不要加 `compression`**（`png` / `cbor`），板子上的 rosbridge 用压缩会一条消息都收不到 | 0.3、2.2.4 |
| 服务地址改为 `192.168.128.113` | 0.2 |
| 底盘控制台（5001）的充电桩接口改为经过适配层 | 第六章 |

### 第零章 系统结构与连接方式

#### 0.1 结构

````
前端 / 其他模块（ASR 等）
   │  WebSocket：ws://192.168.128.113:9090（rosbridge v2 协议，JSON）
   ▼
板卡：G1D 自带的 PC2（当前使用）或 8550（ROS_DOMAIN_ID=71）
   ├─ g1d_g1_ros_bridge     G1 接口适配层：把 G1 的 ROS 服务 / 话题翻译成 G1D 的 HTTP 请求
   ├─ semantic_map_server   语义地图（第五章）
   └─ 网页：g1_web 控制台 :8090，G1D 底盘控制台 :5001（第六章）
   │  HTTP（机器人内部网线）→ G1D 底盘 192.168.123.163:1448
   ▼
G1D 底盘（Slamtec 导航底盘，型号 ares2-ys，软件 6.3.3）
````

G1D 自己完成建图、定位和路径规划；板卡上**没有** Nav2 和激光定位，适配层只负责接口翻译。

#### 0.2 地址和端口

| 用途 | 地址 |
|---|---|
| rosbridge（本文档的主要接口） | `ws://192.168.128.113:9090` |
| g1_web 控制台 | `http://192.168.128.113:8090/`（自动连同一台机器的 9090） |
| G1D 底盘控制台（备用，HTTP 接口见第六章） | `http://192.168.128.113:5001/` |
| G1D 原生 HTTP 接口 | `http://192.168.123.163:1448`（只有机器人内部网段能访问，见 `docs/api_quick_reference.md`） |

`192.168.128.113` 是 G1D 自带的 PC2 板卡（宇树板卡，Jetson Orin NX，ROS 2 Foxy）的 WiFi 地址，以现场为准。

**另一套**：8550 板卡（AidLux，ROS 2 Humble，WiFi `192.168.128.59`）上装了同一套服务、同样的端口和接口。
⚠ 同一时间只用一套（两套都连着同一台 G1D）。

#### 0.3 rosbridge 调用约定

- 发布话题 `op: publish`，订阅 `op: subscribe`，调用服务 `op: call_service`。
- ⚠ 服务的 `args` 必须是**对象**：`{"action": "mapping"}`，不能写成数组。
- ⚠ **类型是 `string` 的字段，内容是 JSON 时也必须传字符串**（先 `JSON.stringify`），例如 `/add_point` 的 `data`。
- ⚠ 服务请求里**不能带服务定义里没有的字段**，否则 rosbridge 直接报错。
- ⚠ **订阅话题时不要加 `compression`**（`png`、`cbor` 都不行）。PC2 上的 rosbridge（1.3.2）这两种压缩都有 bug，
  加了以后**一条消息都收不到**，也不报错（地图预览不显示、电量显示 `--` 都是这个原因）。不加压缩，用默认的 JSON 即可。
- ⚠ 适配层每秒向 G1D 查询一次，所以位姿、任务状态、模式状态这些话题是 **1 Hz**，状态变化最多晚约 1 秒。
- ⚠ ROS 2 只在板子本机内通信（`ROS_LOCALHOST_ONLY=1`，`ROS_DOMAIN_ID=71`，FastDDS），**其他机器请用 rosbridge**。
  这样 8550 和 PC2 不会互相收到对方的消息。确实要跨机器直连时，启动前设 `G1D_LOCALHOST_ONLY=0`。

#### 0.4 哪些接口会让机器人动

| 类别 | 接口 |
|---|---|
| **会让 G1D 移动** | ⚠ **低电量自动回充**（不需要调用接口，电量低于阈值时适配层自动执行，3.5.9）；`/nav_to_pose`、`/patrol_path`、`/patrol_control` 的 `resume`、`/cmd_vel_remote_ctrl`、`/cmd_dock` 的 `dock`、`/semantic_map/go`、`/goal_pose_`（全局重定位，是否转动未实测）；`/semantic_map/text_in`（默认找到就导航；启动参数 `text_in_action:=search` 时只搜索）。底盘控制台：`/api/navigate`、`/api/pois/navigate/<id>`、`/api/move`、`/api/home`、`/api/relocalize`、`/api/semantic/go` |
| 不会动，但会改 G1D 的状态 | `/mode_set`（`mapping` 会**清空 G1D 上的地图**）、`/initialpose`（立即改位姿）、`/set_current_map_id`（会**替换 G1D 上的地图**）、`/aid_save_map`、位置点增删改、`/set_forbidden`、`/cmd_dock` 的 `set_dock_pose` / `add_dock` / `use_dock` / `delete_dock` / `clear_docks` / `low_battery`（设置阈值时）；底盘控制台里的开关、编辑、清空类接口 |
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

⚠ 不要加 `compression: "cbor"`（见 0.3）。电量低于阈值时适配层会自动回充，见 3.5.9。

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

#### 1.6 IP 地址 ⚠（在 G1D 上不可用）

**Service**: `/get_ip_addresses`　**Service Type**: `aid_robot_msgs/srv/GetString`

返回 `{"success": true, "message": "ok", "result": "127.0.1.1"}`：取的是主机名对应的地址，板卡上是 `127.0.1.1` 一类的本机地址，没有实际用处。

---
### 第二章 地图管理

⚠ **G1D 自己只保存"当前这一张地图"**。"多张地图"是适配层在板卡上维护的目录（`~/G1D/bridge_data/`：`catalog.json`
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
  （`POST /api/multi-floor/map/v1/stcm/:save`），并刷新板卡上这张图的 `.stcm` 快照，这样 G1D 重启、切走再切回来，点位都还在。
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

1. 把 G1D 上现有的地图备份到板卡的 `~/G1D/bridge_data/backup/<时间>.explore/.stcm`（保留最近 10 份；地图是空的就跳过）。
2. 清空 G1D 上的地图（`DELETE /api/core/slam/v1/maps`）。
3. 打开建图开关，从空白开始建图。

- 响应：`{"message": "ok"}`；失败时 `message` 是原因（⚠ G1 返回 `"err"`，G1D 返回具体原因）。
- ⚠ 已经在建图时再调用 `mapping`，会再清空一次（前一次没保存的建图结果只剩备份）。
- ⚠ 备份只能在板卡上手动恢复，没有接口。要保留的地图请先按 2.2.3 保存。
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

- 让 G1D 把地图保存到它自己的存储里（`POST /api/multi-floor/map/v1/stcm/:save`），并把地图取回板卡暂存；然后**关掉建图开关**（和 G1 一样，保存后结束建图）。
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
{"op": "subscribe", "topic": "/map_base64", "type": "nav_msgs/msg/OccupancyGrid", "throttle_rate": 3000, "queue_length": 1}
````

- ⚠ **每 3 秒发布一次 G1D 当前的地图，不管是不是在建图**（QoS 为 transient local，订阅后马上收到最近一张）。
- ⚠ `msg.data` 不是栅格数据，而是 **JPEG 图片的 base64 字符串逐字节放进去**（与 G1 相同，格式和颜色见 2.2.15）。
- ⚠ 地图大时每张很大（20 m × 53 m 约 150 KB，JSON 传输约 270 KB），只在需要时订阅，可以加 `throttle_rate` 降低频率。
- ⚠ **不要加 `compression: "png"`**：PC2 上的 rosbridge 用 PNG 压缩会一条都收不到（0.3）。

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

- 删除板卡上保存的这张图和它的巡逻路线，不影响 G1D。
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
{"values": {"success": true, "map_id": 1, "map_name": "G1D当前地图", "map_file": "/home/unitree/G1D/bridge_data/1.explore"}}
````

##### 2.2.15 获取地图图片

**Service**: `/get_map_image`　**Service Type**: `aid_robot_msgs/srv/MapImage`

````
{"op": "call_service", "service": "/get_map_image", "args": {"id": 1}}
````

- 响应：`{"success": true, "map": <OccupancyGrid>, "map_file": "..."}`。
- 要的是 G1D 上现有的那张图时，返回 G1D 的实时地图；否则返回板卡上保存的快照。
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

- ⚠ G1D 没有路线数据库，路线只存在板卡上的目录里，`data` 原样保存、原样返回（格式由前端自己定）。
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
- 话题没有应答。下发失败（例如 G1D 不在线）只记在板卡的 `~/g1d_logs/bridge.log` 里，`/task_status` 不会变成"执行中"。

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
- 续圈靠适配层每秒一次的状态轮询，圈与圈之间会有不到 1 秒的停顿；适配层（板卡上的服务）停了，巡逻也只会走完当前这一圈。
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

#### 3.5 回充与充电桩

这一节的接口都由适配层提供，前端只需要连 rosbridge。

**先了解几个概念**

- **充电桩记录存在 G1D 当前的地图里**（和位置点一样）。每张地图有自己的一组桩，切换地图后桩也跟着换。
  地图上没有桩记录时，机器人不知道桩在哪，回充一定失败。
- ⚠ **桩不会自动出现在地图上**：从桩上出发建图、建完开回桩上，都**不会**自动生成桩记录（2026-10-08 实测）。
  必须用 3.5.4 注册，或用 3.5.5 在地图上放置。
- 每个桩有一个 `id`（字符串，UUID），后面的命令用它指定桩。
- **当前使用的桩**（`current: true`）：不指定桩、低电量自动回充时去的就是它（3.5.9）。只有一个桩时就是这个桩；
  有多个桩时，是最近一次回充去的桩，或者用 `use_dock` 指定的桩。g1_web 地图上标大写 **S**，其它桩标小写 **s**。
- **估计位置的桩**（`provisional: true`）：用 `add_dock` 在地图上放置的桩，位置是人估计的。
  第一次回这个桩成功、底盘确认在桩上之后，适配层自动把它改成真实的上桩位置（`id` 和名称不变），之后 `provisional` 变成 `false`。
  回桩时最后一段靠底盘的激光 / 红外识别桩来对准，所以估计位置不需要很准，但朝向要大致对。

##### 3.5.1 调用格式

**Service**: `/cmd_dock`　**Service Type**: `aid_robot_msgs/srv/SetString`

````
string data      # 命令
---
bool success     # 是否成功
string message   # 成功时是 "ok" 或 JSON 字符串；失败时是原因
````

`data` 有两种写法：

1. **G1 原有的三个命令**：直接写命令名，和 G1 完全兼容。

   ````
   {"op": "call_service", "service": "/cmd_dock", "args": {"data": "dock"}}
   ````

2. **G1D 扩展命令**：`data` 是一个 **JSON 对象序列化后的字符串**，`cmd` 字段是命令名，其它字段是参数。
   ⚠ `data` 的类型是 `string`，要先 `JSON.stringify`，不能直接放对象：

   ````
   {"op": "call_service", "service": "/cmd_dock",
    "args": {"data": "{\"cmd\": \"dock\", \"dock_id\": \"b02beea4-7cde-4418-abc4-b1f6a509d94e\"}"}}
   ````

   JavaScript 写法：`args: {data: JSON.stringify({cmd: "dock", dock_id: id})}`

**返回**：

| 情况 | `success` | `message` |
|---|---|---|
| 成功，命令没有返回内容 | `true` | `"ok"` |
| 成功，命令有返回内容（只有 JSON 写法） | `true` | JSON 字符串，要再 `JSON.parse` 一次 |
| 失败 | `false` | 失败原因（英文），见各命令说明 |

⚠ 用写法 1 调用时，`message` 一律是 `"ok"`（和 G1 一致）；要拿到返回内容请用写法 2。命令名写错时返回 `"unsupported dock command: <data>"`。

##### 3.5.2 命令一览

| 命令 | 机器人会动吗 | 作用 | 成功时的 `message`（JSON 写法） |
|---|---|---|---|
| `list_docks` | 不会 | 查询当前地图的全部充电桩 | 桩数组（3.5.3） |
| `set_dock_pose` | 不会 | 机器人停在桩上时，把当前位置注册为充电桩 | 桩信息 |
| `add_dock` | 不会 | 按坐标在地图上放置充电桩（估计位置） | `{"id": "<新桩 id>"}` |
| `use_dock` | 不会 | 把某个桩设为当前使用的桩 | `"ok"` |
| `delete_dock` | 不会 | 删除一个桩 | `"ok"` |
| `clear_docks` | 不会 | 删除当前地图的全部桩 | `"ok"` |
| `dock` | ⚠ **会** | 回充：回指定的桩，不指定时回最近的桩 | `{"dock_id": "<去的桩>"}` |
| `cancel_dock` | 会停下 | 取消回充 | `"ok"` |
| `low_battery` | 不会 | 读取 / 设置低电量自动回充的阈值 | `{"percent": 10.0}` |

修改桩的命令（注册、放置、删除）会自动保存地图，切换地图再切回来桩不会丢。建图过程中改的桩，随 2.2.3 保存地图一起保存。

##### 3.5.3 查询充电桩列表 `list_docks`

````
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "{\"cmd\": \"list_docks\"}"}}
````

（也可以写成 `{"data": "list_docks"}`，这个命令两种写法都返回列表。）

返回（`message` 是字符串，下面是 `JSON.parse` 之后的内容）：

````
[
  {"id": "b02beea4-7cde-4418-abc4-b1f6a509d94e",
   "pose": {"x": -0.582, "y": 0.011, "yaw": -0.025},
   "metadata": {"display_name": "主桩"},
   "provisional": false,
   "current": true},
  {"id": "5c1d0e2a-9b7f-4a51-8c3e-2f6d4b1a7e90",
   "pose": {"x": 6.2, "y": -3.5, "yaw": 1.571},
   "metadata": {"display_name": "二号桩"},
   "provisional": true,
   "current": false}
]
````

| 字段 | 含义 |
|---|---|
| `id` | 桩的 id，其它命令用它指定桩 |
| `pose.x` / `pose.y` | 桩的位置（地图坐标，米）。准确地说是**机器人停在桩上充电时的位置** |
| `pose.yaw` | **机器人停在桩上时车头的朝向**，弧度，逆时针为正（0 = 地图 x 轴正方向） |
| `metadata.display_name` | 桩的名称 |
| `provisional` | `true` = 估计位置，还没上过这个桩（见本节开头） |
| `current` | `true` = 当前使用的桩；最多一个桩是 `true`，有多个桩但从没回充过时可能全是 `false` |

当前地图没有桩时返回 `[]`。

##### 3.5.4 注册充电桩（机器人在桩上）`set_dock_pose`

把机器人现在的位置记为一个充电桩。

````
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "{\"cmd\": \"set_dock_pose\", \"name\": \"主桩\"}"}}
````

| 参数 | 必填 | 说明 |
|---|---|---|
| `name` | 否 | 桩的名称，不填时叫 `G1D充电桩` |

- G1 写法：`{"data": "set_dock_pose"}`（不能指定名称）。
- 成功返回桩信息（字段同 3.5.3，没有 `provisional` / `current`），同时 `/dock_result` 发一条 `set_dock_pose_succeeded`。
- ⚠ **调用前确认两件事**：机器人**已经停在桩上**（`/dock_state` 是 `charging`）；**定位正确**（机器人在地图上的位置和实际一致）。
  定位不对时注册，桩会被记在错误的位置。
- 1 米内已经有桩时，视为同一个桩：**更新那个桩的位置**，保留它的 `id`（填了 `name` 就改名），不会重复添加。
  （思岚要求桩周围 2 米内没有障碍物，所以 1 米内不可能有两个桩。）

##### 3.5.5 在地图上放置充电桩 `add_dock`

机器人不在桩旁边时，用坐标直接添加一个桩，常用于添加第二个、第三个桩。

````
{"op": "call_service", "service": "/cmd_dock",
 "args": {"data": "{\"cmd\": \"add_dock\", \"x\": 6.2, \"y\": -3.5, \"yaw\": 1.571, \"name\": \"二号桩\"}"}}
````

| 参数 | 必填 | 说明 |
|---|---|---|
| `x`、`y` | 是 | 桩的位置（地图坐标，米），即机器人停在桩上时的位置 |
| `yaw` | 否 | **机器人停在桩上时车头的朝向**，弧度，默认 0。回桩时机器人先按这个朝向开到桩前，偏差太大可能找不到桩 |
| `name` | 否 | 桩的名称，默认 `充电桩` |

- 返回 `{"id": "<新桩 id>"}`。新桩是**估计位置**（`provisional: true`），第一次回这个桩成功后自动校正。
- 机器人不会动。

##### 3.5.6 设为当前使用的桩 `use_dock`

````
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "{\"cmd\": \"use_dock\", \"dock_id\": \"<id>\"}"}}
````

- 机器人不会动。之后不指定桩的回充、低电量自动回充都去这个桩。
- `dock_id` 不存在时失败：`"dock <id> does not exist"`。

##### 3.5.7 删除充电桩 `delete_dock` / `clear_docks`

````
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "{\"cmd\": \"delete_dock\", \"dock_id\": \"<id>\"}"}}
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "{\"cmd\": \"clear_docks\"}"}}
````

`clear_docks` 删除**当前地图**的全部桩，没有确认步骤，请在前端做好二次确认。

##### 3.5.8 回充 `dock` / 取消回充 `cancel_dock`

````
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "dock"}}
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "{\"cmd\": \"dock\", \"dock_id\": \"<id>\"}"}}
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "cancel_dock"}}
````

| 参数 | 必填 | 说明 |
|---|---|---|
| `dock_id` | 否 | 回哪个桩。**不填时回离机器人最近的桩**（直线距离） |

- ⚠ **机器人会动**：先导航到桩前，再对准、上桩。
- 返回 `{"dock_id": "<实际去的桩>"}`（G1 写法返回 `"ok"`）。**返回成功只表示已经出发**，回充的结果看 `/dock_result`（3.5.10）。
- 有多个桩时，去的那个桩会变成当前使用的桩。
- 失败原因：当前地图没有桩 `"no dock on this map"`；`dock_id` 不存在 `"dock <id> does not exist"`。
- 回充成功后，如果去的是估计位置的桩，适配层自动把它校正为真实位置。
- ⚠ 正在导航或巡逻时调用 `dock`：G1D 会用回充替换当前动作，但这种情况没有实测过，建议先 `/patrol_control cancel`。
  （低电量自动回充会自动先取消任务，见 3.5.9。）
- `cancel_dock`：取消通过 `/cmd_dock`（或低电量自动回充）发起的回充，机器人停下。没有在回充时调用也返回 `"ok"`，什么都不做。
  如果这时 G1D 正在执行的不是回充动作（例如回充途中又发了导航），返回 `"current G1D action is not the docking action"`。
- 不支持 `undock`（离桩）。

##### 3.5.9 低电量自动回充 `low_battery`

适配层每 2 秒检查一次电量，**不需要前端调用任何接口**。同时满足下面所有条件时自动回充：

1. 电量 **低于阈值**（默认 **10%**）；
2. 不在充电，也不在桩上；
3. 当前没有在回充。

触发后依次：

1. **结束正在执行的导航 / 巡逻任务**（暂停中的也算），`/task_status` 报告为"取消"（5），巡逻不会再继续；
2. `/dock_result` 发一条 **`low_battery_dock`**；
3. 回**当前使用的桩**（没有标记当前桩时回最近的桩），`/dock_state` 变成 `goto_dock_pose`；
4. 回充结束后 `/dock_result` 照常发 `dock_succeeded` 或 `dock_failed`。

- 没回成功时，每 2 分钟重试一次，最多 3 次。开始充电、或电量回升到**阈值 + 2%** 以上后，重新计数。
- 当前地图没有桩时，机器人不会动，只在适配层日志里记录错误。
- 回充被别的动作打断（例如回充途中又发了导航）时算一次失败，电量仍低时 2 分钟后再试（共最多 3 次）。这种情况没有实测过。

**读取 / 设置阈值**：

````
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "{\"cmd\": \"low_battery\"}"}}
{"op": "call_service", "service": "/cmd_dock", "args": {"data": "{\"cmd\": \"low_battery\", \"percent\": 15}"}}
````

| 参数 | 必填 | 说明 |
|---|---|---|
| `percent` | 否 | 不填 = 只读取；填了 = 设置。单位 %，范围 0~99，**0 = 关闭自动回充** |

- 返回 `{"percent": 15.0}`（设置后的值）。超出范围返回 `"low battery threshold must be 0-99 %"`。
- 设置立即生效，保存在适配层的 `catalog.json` 里，重启后仍有效。从没设置过时用环境变量 `G1D_LOW_BATTERY`，也没有就是 10。

##### 3.5.10 充电状态和回充结果（话题）

````
{"op": "subscribe", "topic": "/dock_state", "type": "std_msgs/msg/String"}
{"op": "subscribe", "topic": "/dock_result", "type": "std_msgs/msg/String"}
````

**`/dock_state`**（`std_msgs/msg/String`，**每 2 秒一条**，表示现在的状态）：

| `data` | 含义 |
|---|---|
| `charging` | 正在充电 |
| `goto_dock_pose` | 正在回充（已出发，还没充上电） |
| `undock` | 不在充电，也没有在回充（⚠ 停在桩上但没在充电时也是 `undock`） |

**`/dock_result`**（`std_msgs/msg/String`，**有事件时才发一条**）：

| `data` | 什么时候发 |
|---|---|
| `set_dock_pose_succeeded` | `set_dock_pose` 注册成功 |
| `low_battery_dock` | 低电量自动回充开始（G1D 新增，G1 没有） |
| `dock_succeeded` | 回充成功，已经上桩 |
| `dock_failed` | 回充失败（包括回充途中被别的动作打断）。⚠ 用 `cancel_dock` 主动取消时**不发** |

**判断"要去充电了"**（例如开始回充时播报语音）：订阅 `/dock_state`，记住上一条的值。
**上一条不是 `goto_dock_pose`、这一条是 `goto_dock_pose`** 的那一刻就是开始回充。手动回充、别的程序调用 `/cmd_dock`、
低电量自动回充，三种情况都会这样变化。

- 回充途中每 2 秒还会收到 `goto_dock_pose`，只在变化的那一刻处理一次。
- 刚订阅时收到的第一条没有"上一条"：如果它就是 `goto_dock_pose`，说明订阅之前就已经在回充了，按需要决定处不处理。
- 要知道是不是低电量引起的：同时订阅 `/dock_result`，开始回充前后会收到一条 `low_battery_dock`。
- 状态每 2 秒更新一次，所以从发出回充到收到 `goto_dock_pose` 最多晚约 2 秒。

````
let last = null;
dockState.subscribe((m) => {
  if (m.data === "goto_dock_pose" && last !== null && last !== "goto_dock_pose") {
    // 要去充电了：在这里播报语音、更新界面等
  }
  last = m.data;
});
````

##### 3.5.11 查询充电桩位置 `/get_dock_pose`（G1 原有接口）

**Service**: `/get_dock_pose`　**Service Type**: `aid_robot_msgs/srv/GetDockPose`

````
{"op": "call_service", "service": "/get_dock_pose", "args": {"map_id": 9}}
````

- `map_id` 必须是当前地图的 id（`/get_current_map_id`），否则返回 `"select this map before editing its G1D POIs or walls"`。
- 有桩时返回 `{"success": true, "message": "ok", "point": <geometry_msgs/Pose>}`，⚠ 有多个桩时返回**离机器人最近**的那个；
  没有桩时返回 `{"success": false, "message": "no dock"}`。
- 要拿到全部桩、名称和 id，请用 `list_docks`（3.5.3）。

##### 3.5.12 推荐的使用流程

**主桩**（2026-10-08 实测可用）：

1. 机器人从桩上出发建图（2.2.1），建完保存（2.2.3）。从桩上出发的好处是地图原点就在桩附近。
2. 把机器人开回桩上，等 `/dock_state` 变成 `charging`。
3. 确认定位正确：机器人在地图上的位置和实际一致（不对就先按 1.1 重定位）。
4. 调用 `set_dock_pose` 注册主桩。之后 `list_docks` 能看到它，`current: true`。

**其它桩**：

1. 用 `add_dock` 在地图上放置（坐标 + 车头朝向）。
2. 调用 `dock` 并指定这个桩，让机器人回一次。成功后位置自动校正，`provisional` 变成 `false`。

**低电量自动回充**：确认当前使用的桩（`list_docks` 里 `current: true` 的那个）是你希望它回去的桩；
需要改阈值时调用 `low_battery`。

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
由板卡上的 `semantic_map_server` 节点提供，接口与 G1 完全相同。

工作方式：

1. 位置点就是 2.1 的位置点（G1D POI）。**给位置点写上 `description`**（写成人会怎么说，例如"口渴""接水"），匹配会准很多。
2. 每个点的"名称 + 描述"由 BGE-M3 模型转成向量；**坐标不参与匹配**。
3. 搜索时把这句话也转成向量，找最相似的点；最高分低于阈值（默认 0.52）就算"没找到"，机器人不动。
4. 位置点有增删改后，**下一次搜索会自动重建**，一般不需要手动重建。

部署：模型 `~/models/bge-m3`；PC2 用 GPU 推理（每句约 0.1 秒），8550 用 CPU 推理（开机时预加载约 5 秒，之后每句约 0.1~1 秒）；向量库用本地文件
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

参数在 `ros2_ws/src/semantic_map_ros/config/semantic_map.yaml`；板卡上的启动脚本 `deploy/run_g1d.sh` 用 launch 参数覆盖了其中几个。
⚠ launch 参数会覆盖配置文件里的同名参数（`qdrant_path`、`model_name`、`use_fp16`、`score_threshold`、`text_in_action`），
要改这几个请改 `run_g1d.sh` 里的 launch 参数。

| 参数 | 板卡上的值 | 说明 |
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
3. **文件** 板卡上的 `~/maps/semantic_map_log.jsonl`（每行一条，重启不丢）。

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

- 基地址：`http://192.168.128.113:5001`，请求和响应都是 JSON。
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
| POST | `/api/map/export` | 把地图导出到板卡的 `slam_ws/map/`（`.pgm` + `.yaml` + `.stcm`） |
| DELETE | `/api/map/clear` | ⚠ **清空 G1D 上的地图**（不备份） |
| POST | `/api/map/reset-origin` | 把地图原点移到机器人当前位置 |

**运动**（⚠ 会让机器人动，除了 `/api/stop` 和 `/api/speed`）

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/api/navigate`，body `{"x": 2.0, "y": 3.0, "yaw": 1.57}` | 导航到目标点（`yaw` 可省略） |
| POST | `/api/pois/navigate/<poi_id>` | 导航到某个 POI（`poi_id` 是 G1D 里 POI 的 id：适配层建的是 UUID，底盘控制台建的默认用名称） |
| POST | `/api/move`，body `{"direction": 0}` | 遥控一次 600 ms：0 前、1 后、2 右转、3 左转 |
| POST | `/api/home`，body `{}` 或 `{"dock_id": "<id>"}` | 回充电桩，不带 `dock_id` 时回最近的桩；返回 `{"dock_id": "<去的桩>"}` |
| POST | `/api/relocalize`，body 可选 `{"x1", "y1", "x2", "y2"}` | 重定位（给矩形就在矩形内搜索，不给就全图） |
| POST | `/api/stop` | 停止 G1D 当前的动作 |
| PUT | `/api/speed`，body `{"linear": 0.5}` | 设置最大移动速度（m/s） |

**POI（位置点）、充电桩、禁行区、虚拟墙**（不会动，会改 G1D 状态）

| 方法 | 路径 | 说明 |
|---|---|---|
| GET / POST | `/api/pois` | 列表 / 新增，body `{"name": "茶水间", "description": "接水、喝水", "x": 1.0, "y": 2.0, "yaw": 0}` |
| PUT | `/api/pois/<poi_id>`，body `{"name": "...", "description": "..."}` | 改名称和 / 或描述（只改传了的字段） |
| DELETE | `/api/pois/<poi_id>` | 删除 |
| GET | `/api/docks` | 充电桩列表，格式同 3.5.3 |
| POST | `/api/docks`，body `{"x": 6.2, "y": -3.5, "yaw": 1.571, "name": "二号桩"}` | 在地图上放置充电桩（估计位置，同 3.5.5），返回 `{"id"}` |
| POST | `/api/docks/register`，body `{"name": "主桩"}` | 机器人在桩上时注册当前位置（同 3.5.4），返回桩信息 |
| DELETE | `/api/docks/<id>`、`/api/docks/clear` | 删除一个 / 全部，返回 `{"ok": true}` |
| GET / POST | `/api/areas` | 禁行区列表 / 新增，body `{"x1", "y1", "x2", "y2"}`（矩形对角） |
| DELETE | `/api/areas/<id>`、`/api/areas/clear` | 删除一个 / 全部 |
| GET / POST | `/api/walls` | 虚拟墙列表 / 新增，body `{"x1", "y1", "x2", "y2"}` |
| DELETE | `/api/walls/<id>`、`/api/walls/clear` | 删除一条 / 全部 |

POI 就是 2.1 的位置点：两边读写的是同一份 `metadata.display_name` / `metadata.description`。

⚠ 充电桩接口（`/api/home`、`/api/docks*`）**不直接改底盘，而是经 rosbridge 调用适配层的 `/cmd_dock`**（3.5），
所以要求适配层在运行；失败时返回 `{"error": "原因"}`，HTTP 502。这样做是为了让桩和适配层保存的地图保持一致。

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
| 充电桩数量 | 一个 | 每张地图可以有多个，`/cmd_dock` 的 JSON 扩展命令管理（3.5） |
| `/cmd_dock` 的 `dock` | 回充电桩 | 回最近的桩，或用 JSON 写法指定桩（3.5.8） |
| `/get_dock_pose` | 返回充电桩 | 有多个桩时返回离机器人最近的（3.5.11） |
| 低电量自动回充 | 无 | 有，默认 10%，可设置（3.5.9） |
| `/dock_result` 的取值 | `set_dock_pose_succeeded` / `dock_succeeded` / `dock_failed` | 多一个 `low_battery_dock`（3.5.10） |

---

### 附录 B 运行与维护（PC2 / 8550 相同）

- **开机自启**：systemd 服务 `g1d`（开机自动启动 rosbridge、语义地图、三个网页和适配层；任何一个进程退出，5 秒后整体重启）。
  - 查看：`systemctl status g1d`、`journalctl -u g1d -f`
  - 重启 / 停止：`bash ~/start_g1d.sh` / `bash ~/stop_g1d.sh`（不需要密码；停止后下次开机仍会自启）
- **日志**：`~/g1d_logs/`，每个服务一个文件（`bridge.log`、`semantic_map.log`、`rosbridge.log` 等），每次启动时覆盖。
  低电量自动回充的触发和错误记在 `bridge.log`。
- **适配层数据**：`~/G1D/bridge_data/catalog.json` 保存地图目录、巡逻路线、估计位置的桩、低电量回充阈值；不要手工改。
- **G1D 关机或网线断开时**：适配层不会退出，每秒在 `bridge.log` 记一次连接错误；G1D 恢复后自动接上，不用重启服务。
- **代码**：全部在 `slam_ws` 里（适配层 `g1d_g1_ros_bridge.py` / `g1d_g1_core.py`，网页 `g1_web/`，ROS 包 `ros2_ws/src/`，
  部署脚本 `deploy/`，说明 `deploy/README.md`）；从电脑同步到板卡用 `bash deploy/sync_to_8550.sh <用户@IP>`（PC2：`unitree@192.168.128.113`）。
- ⚠ **重启服务会让适配层忘记正在执行的任务**（之后 `pause` / `cancel` 控制不了它），重启前先确认没有导航任务在执行。
