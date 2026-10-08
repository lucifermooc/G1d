# 用 G1 接口驱动 G1D：本地适配层

## 目的和结构

`aidrobo_client-haier` 前端调用的是 ROS2 服务和话题，WebSocket 入口为 `ws://<rosIP>:9090`。G1D 工程的底层是机器人 `:1448` HTTP API。这里增加一个独立的 ROS2 节点，把前端已有的 G1 导航接口翻译为 G1D HTTP 请求：

```text
aidrobo_client-haier 原页面
       │  ROSLIB，G1 服务/话题名称
       ▼
rosbridge_websocket :9090
       │
       ▼
g1d_g1_ros_bridge.py ── g1d_g1_core.py ── HTTP :1448 ── G1D
       │
       └── bridge_data/：地图 STCM 快照、栅格快照、地图和巡逻路线目录
```

前端源码和调用点不需要改。适配层需使用 G1 工作空间中的 `aid_robot_msgs` 类型，但不要在同一个 `ROS_DOMAIN_ID` 同时运行原 G1 的 `map_manager_server`、`waypoint_manage`、`robot_status_manager` 等服务拥有者。

## 接口对照

| G1 前端接口 | 适配层转换 | 说明 |
| --- | --- | --- |
| `/mode_set` | PUT mapping/localization enable | 支持 `mapping`、`continue_mapping`、`localization`、`patrol`、`remote_control`、`idle` |
| `/map_base64`、`/get_map_image` | GET `/api/core/slam/v1/maps/explore` | 解析 36 字节头和栅格，翻转 Y 轴后编码成前端预期的 JPEG base64 字节，写入 `OccupancyGrid.data` |
| `/get_map_list`、`/get_current_map_id`、`/set_current_map_id`、`/aid_save_map`、`/add_map` | GET/PUT STCM、POST `stcm/:save`，本地目录 | 首次读取自动把机器人当前地图导入目录；地图切换会 PUT 对应 STCM 到机器人 |
| `/base_link_pose`、`/tf` | GET localization pose | 发布 G1 的位姿话题和 `map → base_link` 变换 |
| `/scan` | GET `/api/core/system/v1/laserscan` | 按角度装箱为 `LaserScan`，供重定位页显示 |
| `/initialpose`、`/goal_pose_` | PUT localization pose、POST RecoverLocalizationAction | 手动位姿和全局重定位 |
| `/nav_to_pose` | POST MoveToAction | 坐标和四元数偏航角转为 G1D 目标点 |
| `/patrol_path` | POST SeriesMoveToAction | 多目标点依次导航 |
| `/patrol_control`、`/stop_patrol`、`/task_status` | DELETE 当前 Action、按 ID GET Action 状态 | 暂停通过取消当前 Action 实现；恢复会重新下发剩余路线，从当前位置最近的巡逻点开始 |
| `/cmd_vel_remote_ctrl` | POST MoveByAction | 方向遥控，动作时长 600 ms；零速度时只停止本桥接层发出的动作 |
| 位置点服务 | `/api/core/artifact/v1/pois` | G1 的 `uint32` 点 ID 用 G1D UUID 的稳定散列转换；只有当前地图可编辑 |
| 巡逻路线服务 | 本地 `catalog.json` | G1D 没有 G1 风格的路线数据库 |
| `/get_forbidden`、`/set_forbidden` | GET/DELETE/POST walls | 禁行线对应 G1D 虚拟墙；仅当前地图可编辑 |
| `/get_dock_pose`、`/cmd_dock`、`/dock_result`、`/dock_state` | homedocks、GoHomeAction、power status | 支持注册当前位置、回桩、取消；`undock` 暂不支持 |
| `/battery_state`、`/get_ip_addresses` | power status、本机 IP | 首页状态字段 |

## 已知边界

- `map_editor` 的栅格橡皮擦无 G1D HTTP 等效接口，服务会明确返回失败；`aid_draw_forbidden_line` 是前端额外调用，实际虚拟墙由 `/set_forbidden` 保存。
- G1 的摄像头、人跟随、标定、扫码配对、离桩等专有功能未映射。导航规划线 `/plan` 暂未发布。
- G1D 原生地图只有当前 STCM；“多地图”是适配层保存的本地快照。不要清理 `bridge_data/`，否则地图名、路线和旧地图快照会丢失。若有其他控制端在适配层之外切换 G1D 地图，目录中的“当前地图”不会自动同步。
- 切换 STCM 地图后需要确认定位；必要时在前端重定位页重新设置机器人位姿，再开始导航。
- G1D HTTP 是否接受具体固件版本的 Action 参数、地图 STCM 切换和虚拟墙批量更新，仍需机器恢复可达后验证。当前完成的是本地静态和假数据验证。
- 前端 `map_base64` 沿用 G1 原项目的非常规格式：`OccupancyGrid.data` 放的是 JPEG base64 ASCII，而不是真正的占据率数组。标准地图消费者不能直接按 ROS 占据栅格解释它。

## 机器可达后的启动方式

以下命令是后续运行说明；当前机器不可达时不要执行桥接节点。前端可以单独启动，但不会收到机器人数据。

1. 在终端 A 启动适配层。使用独立的 ROS domain；`G1D_ROBOT_IP` 按现场网络修改。`G1D_BRIDGE_DATA` 使用固定的绝对目录，保证重启后地图不丢。

   ```bash
   cd /home/lmw/project/unitree/G1D/slam_ws
   source /opt/ros/humble/setup.bash
   source /home/lmw/project/unitree/G1/lightning_ws/install/setup.bash
   export ROS_DOMAIN_ID=71
   export G1D_ROBOT_IP=192.168.123.163
   export G1D_BRIDGE_DATA=/home/lmw/project/unitree/G1D/slam_ws/bridge_data
   python3 g1d_g1_ros_bridge.py
   ```

2. 在终端 B 启动 rosbridge，保持相同的 `ROS_DOMAIN_ID`。

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/lmw/project/unitree/G1/lightning_ws/install/setup.bash
   export ROS_DOMAIN_ID=71
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
   ```

3. 在终端 C 启动 G1 原前端。

   ```bash
   cd /home/lmw/project/unitree/G1D/slam_ws/aidrobo_client-haier
   npm run dev
   ```

   浏览器打开开发服务器打印的地址。若前端与 rosbridge 不在同一台主机，在前端“设置”页面把 ROS IP 设为运行 rosbridge 的主机 IP。这个 IP 不是 G1D 机器人 IP；机器人 IP 由 `G1D_ROBOT_IP` 设置。

## 已执行的离线检查

```bash
cd /home/lmw/project/unitree/G1D/slam_ws
python3 -m unittest -v test_g1d_g1_core
source /opt/ros/humble/setup.bash
source /home/lmw/project/unitree/G1/lightning_ws/install/setup.bash
python3 -c 'import g1d_g1_ros_bridge; print("ROS imports ok")'
```

这些检查不连接机器人，也不启动 DDS 节点。
