# 宇树 G1D 底盘 API 快速参考

> 适用：API v1.1.0 | 所有接口无需鉴权

---

## 一、系统资源（system）

| 需求 | 方法 | 路径 |
|------|------|------|
| 查询底盘支持的能力 | GET | `/api/core/system/v1/capabilities` |
| 查询电源/充电状态 | GET | `/api/core/system/v1/power/status` |
| 关机 / 延时重启 | POST | `/api/core/system/v1/power/:shutdown` |
| 休眠底盘（暂停激光雷达） | POST | `/api/core/system/v1/power/:hibernate` |
| 唤醒底盘 | POST | `/api/core/system/v1/power/:wakeup` |
| 重启模块（软/硬/底盘） | POST | `/api/core/system/v1/power/:restartmodule` |
| 获取设备信息 | GET | `/api/core/system/v1/robot/info` |
| 获取设备健康状态 | GET | `/api/core/system/v1/robot/health` |
| 清除错误状态 | DELETE | `/api/core/system/v1/robot/health/{error_code}` |
| 获取激光观测帧 | GET | `/api/core/system/v1/laserscan` |
| 读取系统参数 | GET | `/api/core/system/v1/parameter?param=<名称>` |
| 设置系统参数 | PUT | `/api/core/system/v1/parameter` |
| 查询网络状态 | GET | `/api/core/system/v1/network/status` |
| 设置网络模式（AP/STA/禁用WiFi等） | PUT | `/api/core/system/v1/network/status` |
| 查询路由优先级 | GET | `/api/core/system/v1/network/route` |
| 设置路由优先级（wifi/4g） | PUT | `/api/core/system/v1/network/route` |
| 设置以太网静态 IP | PUT | `/api/core/system/v1/network/ip` |
| 删除静态 IP 配置 | DELETE | `/api/core/system/v1/network/ip` |
| 获取 4G APN | GET | `/api/core/system/v1/network/apn` |
| 设置 4G APN | PUT | `/api/core/system/v1/network/apn` |
| 设置 Cube 配置 | PUT | `/api/core/system/v1/cube/config` |
| 设置 LED 灯光效果 | POST | `/api/core/system/v1/light/control` |
| 开启 / 关闭 AEB 自动紧急制动 | POST | `/api/core/system/v1/aeb/control` |
| 控制千斤顶（Up/Down/Stop） | POST | `/api/core/system/v1/jack/status` |
| 查询千斤顶状态 | GET | `/api/core/system/v1/jack/status` |
| 获取 IMU ADC 原始值 | GET | `/api/core/system/v1/rawadcimu` |
| 获取 IMU 原始值 | GET | `/api/core/system/v1/rawimu` |
| 获取电池包电流/温度 | GET | `/api/core/system/v1/battery/pack` |

### 系统参数名速查

| param 值 | 含义 |
|----------|------|
| `base.max_moving_speed` | 最大线速度 (m/s) |
| `base.max_angular_speed` | 最大角速度 (rad/s) |
| `base.emergency_stop` | 急停 on/off |
| `base.brake_release` | 刹车释放 on/off |
| `docking.docked_register_strategy` | 充电桩注册策略 |

---

## 二、定位与建图（slam）

| 需求 | 方法 | 路径 |
|------|------|------|
| 获取底盘当前位姿 | GET | `/api/core/slam/v1/localization/pose` |
| 强制设置底盘位姿 | PUT | `/api/core/slam/v1/localization/pose` |
| 获取里程计位姿 | GET | `/api/core/slam/v1/localization/odopose` |
| 获取定位质量（0~100） | GET | `/api/core/slam/v1/localization/quality` |
| 查询是否开启定位 | GET | `/api/core/slam/v1/localization/:enable` |
| 开启 / 暂停定位 | PUT | `/api/core/slam/v1/localization/:enable` |
| 重置定位状态 | POST | `/api/core/slam/v1/localization/status/:reset` |
| 查询是否开启建图 | GET | `/api/core/slam/v1/mapping/:enable` |
| 开启 / 暂停建图 | PUT | `/api/core/slam/v1/mapping/:enable` |
| 查询闭环检测状态 | GET | `/api/core/slam/v1/loopclosure/:enable` |
| 开启 / 暂停闭环检测 | PUT | `/api/core/slam/v1/loopclosure/:enable` |
| 获取充电桩位置 | GET | `/api/core/slam/v1/homepose` |
| 设置充电桩位置 | PUT | `/api/core/slam/v1/homepose` |
| 获取所有充电桩信息 | GET | `/api/core/slam/v1/homedocks` |
| 设置所有充电桩 | PUT | `/api/core/slam/v1/homedocks` |
| 添加充电桩 | POST | `/api/core/slam/v1/homedocks` |
| 清空充电桩 | DELETE | `/api/core/slam/v1/homedocks` |
| 注册当前位置为充电桩 | POST | `/api/core/slam/v1/homedocks/:register` |
| 编辑充电桩 | PUT | `/api/core/slam/v1/homedocks/{dock_id}` |
| 删除某个充电桩 | DELETE | `/api/core/slam/v1/homedocks/{dock_id}` |
| 获取 IMU 数据 | GET | `/api/core/slam/v1/imu` |
| 获取已知区域（地图范围） | GET | `/api/core/slam/v1/knownarea` |
| 获取栅格地图（二进制流） | GET | `/api/core/slam/v1/maps/explore` |
| 获取复合地图（stcm 文件） | GET | `/api/core/slam/v1/maps/stcm` |
| 上传并设置复合地图 | PUT | `/api/core/slam/v1/maps/stcm` |
| 清空地图 | DELETE | `/api/core/slam/v1/maps` |
| 移动地图原点 | PUT | `/api/core/slam/v1/maps/origin` |

---

## 三、地图语义元素（artifact）

### 虚拟线段（tracks / walls）

| 需求 | 方法 | 路径 |
|------|------|------|
| 获取虚拟轨道或虚拟墙 | GET | `/api/core/artifact/v1/lines/{usage}` |
| 添加线段 | POST | `/api/core/artifact/v1/lines/{usage}` |
| 修改线段 | PUT | `/api/core/artifact/v1/lines/{usage}` |
| 清空某类线段 | DELETE | `/api/core/artifact/v1/lines/{usage}` |
| 删除指定 ID 线段 | DELETE | `/api/core/artifact/v1/lines/{usage}/{id}` |

> `{usage}` 取值：`tracks`（虚拟轨道）| `walls`（虚拟墙）

### 矩形区域

| 需求 | 方法 | 路径 |
|------|------|------|
| 获取矩形区域 | GET | `/api/core/artifact/v1/rectangle-areas/{usage}` |
| 添加矩形区域 | POST | `/api/core/artifact/v1/rectangle-areas/{usage}` |
| 清空某类矩形区域 | DELETE | `/api/core/artifact/v1/rectangle-areas/{usage}` |
| 编辑矩形区域 | PUT | `/api/core/artifact/v1/rectangle-areas/{usage}/{id}` |
| 删除矩形区域 | DELETE | `/api/core/artifact/v1/rectangle-areas/{usage}/{id}` |

> `{usage}` 取值：`forbidden_area`（禁行）| `elevator_area`（电梯）| `dangerous_area`（危险/减速）| `coverage_area`（清扫覆盖）| `maintenance_area`（运维）| `sensor_disable_area`（传感器禁用）| `restricted_area`（限行）

### POI 兴趣点

| 需求 | 方法 | 路径 |
|------|------|------|
| 获取所有 POI | GET | `/api/core/artifact/v1/pois` |
| 添加 POI | POST | `/api/core/artifact/v1/pois` |
| 清空所有 POI | DELETE | `/api/core/artifact/v1/pois` |
| 优化 POI 位姿（闭环后调用） | POST | `/api/core/artifact/v1/pois/:adjust` |
| 根据 ID 查找 POI | GET | `/api/core/artifact/v1/pois/{poi_id}` |
| 修改 POI | PUT | `/api/core/artifact/v1/pois/{poi_id}` |
| 删除 POI | DELETE | `/api/core/artifact/v1/pois/{poi_id}` |

### 激光地标（固件 ≥ 5.1.1）

| 需求 | 方法 | 路径 |
|------|------|------|
| 获取激光地标 | GET | `/api/core/artifact/v1/laser-landmarks` |
| 清空激光地标 | DELETE | `/api/core/artifact/v1/laser-landmarks` |
| 设置激光地标 | PUT | `/api/core/artifact/v1/laser-landmarks` |
| 获取地标自动更新状态 | GET | `/api/core/artifact/v1/laser-landmarks/:update` |
| 设置地标自动更新 | PUT | `/api/core/artifact/v1/laser-landmarks/:update` |
| 删除部分激光地标 | POST | `/api/core/artifact/v1/laser-landmarks/:remove` |

---

## 四、运动控制（motion）

| 需求 | 方法 | 路径 |
|------|------|------|
| 查询所有支持的 Action 类型 | GET | `/api/core/motion/v1/action-factories` |
| 获取当前运动行为 | GET | `/api/core/motion/v1/actions/:current` |
| 终止当前运动 | DELETE | `/api/core/motion/v1/actions/:current` |
| **创建运动行为** | POST | `/api/core/motion/v1/actions` |
| 查询指定 Action 状态 | GET | `/api/core/motion/v1/actions/{action_id}` |
| 获取剩余路径点 | GET | `/api/core/motion/v1/path` |
| 获取剩余目标点 | GET | `/api/core/motion/v1/milestones` |
| 获取当前速度 | GET | `/api/core/motion/v1/speed` |
| 获取到目的地剩余时间 | GET | `/api/core/motion/v1/time` |
| 搜索路径（不执行） | POST | `/api/core/motion/v1/:search_path` |
| 获取所有运动策略 | GET | `/api/core/motion/v1/strategies` |
| 获取当前运动策略 | GET | `/api/core/motion/v1/strategies/:current` |
| 设置运动策略 | PUT | `/api/core/motion/v1/strategies/:current` |

### 创建运动行为（POST `/api/core/motion/v1/actions`）常用 action_name

| action_name | 用途 | 关键参数 |
|-------------|------|----------|
| `agent.actions.MoveToAction` | 自主导航到目标点 | `target(x,y,z)`, `move_options` |
| `agent.actions.SeriesMoveToAction` | 依次经过多个目标点 | `targets[]`, `move_options` |
| `agent.actions.MoveByAction` | 遥控移动（需定时调用） | `direction`(0前/1后/2右/3左), `duration`(ms) |
| `agent.actions.GoHomeAction` | 自主回充电桩 | `flags`(dock/no_dock) |
| `agent.actions.RotateToAction` | 原地旋转到指定朝向 | `angle`(弧度) |
| `agent.actions.RotateAction` | 原地旋转指定角度 | `angle`(弧度, 正=逆时针) |
| `agent.actions.MoveToTagAction` | 精准对接标签/货架 | `target`, `tag_type`(0=二维码/1=激光/2=反光板/3=货架) |
| `agent.actions.BackOffFromTagAction` | 从标签前后退 | `backup_mode`, `backup_distance` |
| `agent.actions.RecoverLocalizationAction` | 重定位 | `area`(局部) 或空(全局) |
| `agent.actions.FollowPathPointsAction` | 跟随指定路径点 | `path_points[]` |
| `agent.actions.EnterElevatorAction` | 进电梯 | `elevator_id` |
| `agent.actions.LeaveElevatorAction` | 出电梯 | `elevator_id`, `target` |
| `agent.actions.ReturnToParkingAction` | 返回待命点 | `target(poi_name)` |
| `agent.actions.MultiFloorMoveAction` | 跨楼层移动 | `target(building,floor,pose)` |

### move_options 导航模式

| mode 值 | 含义 |
|---------|------|
| `0` | 自由导航 |
| `1` | 严格轨道模式（遇障碍停止等待） |
| `2` | 轨道优先模式（遇障碍下轨绕行） |

### Action 状态码

| status | 含义 |
|--------|------|
| `0` | NewBorn（刚创建） |
| `1` | Working（执行中） |
| `3` | Paused（暂停） |
| `4` | Done（已结束） |

| result | 含义 |
|--------|------|
| `0` | 成功 |
| `-1` | 失败 |
| `-2` | 被中止 |

---

## 五、固件升级（firmware）

| 需求 | 方法 | 路径 |
|------|------|------|
| 查询云端可升级版本 | GET | `/api/core/firmware/v1/newversion` |
| 查询是否开启自动升级 | GET | `/api/core/firmware/v1/autoupdate/:enable` |
| 开启 / 关闭自动升级 | PUT | `/api/core/firmware/v1/autoupdate/:enable` |
| 从云端下载并升级 | POST | `/api/core/firmware/v1/autoupdate/:start` |
| 上传固件包并升级 | POST | `/api/core/firmware/v1/update/:start` |
| 查询升级进度 | GET | `/api/core/firmware/v1/progress` |

---

## 六、运行统计（statistics）

| 需求 | 方法 | 路径 |
|------|------|------|
| 获取累计行驶里程（米） | GET | `/api/core/statistics/v1/odometry` |
| 获取累计运行时间（秒） | GET | `/api/core/statistics/v1/runtime` |

---

## 七、传感器控制（sensors）

| 需求 | 方法 | 路径 |
|------|------|------|
| 启用 / 禁用深度摄像头 | PUT | `/api/core/sensors/v1/depth/:enable` |
| 获取禁用传感器掩码 | GET | `/api/core/sensors/v1/masks` |
| 设置传感器启用 / 禁用 | PUT | `/api/core/sensors/v1/masks` |

---

## 八、安卓应用管理（application）

| 需求 | 方法 | 路径 |
|------|------|------|
| 获取已安装的 APP 列表 | GET | `/api/core/application/v1/apps` |
| 安装 APP | POST | `/api/core/application/v1/apps` |
| 卸载 APP | DELETE | `/api/core/application/v1/apps/{app_name}` |

---

## 九、平台通用（platform）

| 需求 | 方法 | 路径 |
|------|------|------|
| 获取系统时间戳（ms） | GET | `/api/platform/v1/timestamp` |
| 获取底盘事件列表 | GET | `/api/platform/v1/events` |

### 常见事件类型

| 事件 | 说明 |
|------|------|
| `DEVICE_ERROR` | 设备故障 |
| `ON_DOCK` / `OFF_DOCK` | 上桩 / 下桩 |
| `START_CHARGING` / `STOP_CHARGING` | 充电开始 / 停止 |
| `ROBOT_BLOCKED` | 连续被阻挡（默认3分钟） |
| `MAP_LOOP_CLOSURE` | 建图闭环完成 |
| `LOW_BATTERY` | 电量过低，即将回桩 |
| `CLIFF_DETECTED` | 检测到悬崖 |
| `BUMPER_TRIGGERED` | 碰撞传感器触发 |
| `LOCALIZATION_ANOMALY` | 定位异常 |

---

## 十、多楼层管理（multi-floor）

| 需求 | 方法 | 路径 |
|------|------|------|
| 获取地图加载状态 | GET | `/api/multi-floor/status` |
| 获取所有楼层信息 | GET | `/api/multi-floor/map/v1/floors` |
| 获取底盘当前楼层 | GET | `/api/multi-floor/map/v1/floors/:current` |
| 手动设置底盘楼层（搬运场景） | PUT | `/api/multi-floor/map/v1/floors/:current` |
| 获取多楼层 POI | GET | `/api/multi-floor/map/v1/pois` |
| 查找最近 POI | POST | `/api/multi-floor/map/v1/pois/:search_nearby` |
| 查询 POI 最优遍历顺序 | POST | `/api/multi-floor/map/v1/pois/:dispatch` |
| 获取多楼层充电桩 | GET | `/api/multi-floor/map/v1/homedocks` |
| 获取当前绑定的充电桩 | GET | `/api/multi-floor/map/v1/homedocks/:current` |
| 绑定充电桩 | PUT | `/api/multi-floor/map/v1/homedocks/:current` |
| 查找最近充电桩 | POST | `/api/multi-floor/map/v1/homedocks/:search_nearby` |
| 获取电梯区域信息 | GET | `/api/multi-floor/map/v1/elevators` |
| 获取某个电梯信息 | GET | `/api/multi-floor/map/v1/elevators/{elevator_id}` |
| 获取底盘与电梯位置关系 | GET | `/api/multi-floor/map/v1/elevators/{elevator_id}/pose_relation` |
| 上传地图到底盘（持久化不加载） | POST | `/api/multi-floor/map/v1/stcm` |
| 删除保存的地图文件 | DELETE | `/api/multi-floor/map/v1/stcm` |
| 持久化保存当前地图 | POST | `/api/multi-floor/map/v1/stcm/:save` |
| 重新加载地图 | POST | `/api/multi-floor/map/v1/stcm/:reload` |
| 保存并重新加载（sync） | POST | `/api/multi-floor/map/v1/stcm/:sync` |
| 解绑云端场景 | POST | `/api/multi-floor/map/v1/scene/unbind` |
| 轨道图搜索路径点 | POST | `/api/multi-floor/map/v1/search_path_points` |
| 根据 POI 名称重置底盘位姿 | PUT | `/api/multi-floor/localization/v1/pose` |
| 根据充电桩重置底盘定位 | PUT | `/api/multi-floor/localization/v1/homedock` |

---

## 十一、工业搬运（industry）

| 需求 | 方法 | 路径 |
|------|------|------|
| 创建任务模板 | POST | `/api/industry/v1/tasks/templates` |
| 获取所有任务模板 | GET | `/api/industry/v1/tasks/templates` |
| 删除任务模板 | DELETE | `/api/industry/v1/tasks/templates/{key_id}` |
| 创建工业配送任务 | POST | `/api/industry/v1/tasks` |
| 查询任务信息 | GET | `/api/industry/v1/tasks` |
| 推送任务事件（更新状态） | POST | `/api/industry/v1/tasks/events` |

### 任务动作类型（action）

| 值 | 说明 |
|----|------|
| `to_point` | 移动到点 |
| `accurate_docking_columnar_tag` | 对接货架 |
| `accurate_docking_reflect_tag` | 对接反光板 |
| `carry_move` | 搬运货架 |
| `rise` | 顶起货架 |
| `down` | 放下货架并退出 |
| `back_off_from_shelf` | 从货架下退出 |
| `go_home` | 回桩 |
| `return_to_parking` | 返回待命点 |

---

## 附：本机已启用能力

> 基地址：`http://192.168.123.163:1448`

| name | 版本 | 说明 |
|------|------|------|
| `slamware.agent.core` | 6.1.1 | 定位导航等底盘核心功能 |
| `slamware.agent.multi_floor` | 6.1.1 | 多楼层地图管理和跨楼层移动 |
| `slamware.agent.platform` | 5.0.0 | 日志采集等平台功能 |
| `slamware.agent.swarm` | 4.0.0 | 集群多机调度（文档暂无详细接口） |
