# G1D 语义地图（三个前端都能用）

说一句话（"我想喝水"），在位置点里找意思最接近的一个（"茶水间"），可选让机器人过去。
匹配用的是**位置点名称 + 描述**，描述写成人平时会怎么说，匹配会准很多。
接口说明见仓库根目录的 `G1D_Communication_Documentation.md`（第五章、第六章）。

| 前端 | 语义地图走哪条路 | 入口 |
|---|---|---|
| **g1_web 控制台（主用）**（`g1_web/www`，和 G1 用同一份代码，源头在 `unitree-G1/lightning_ws/src/g1_web/www`） | rosbridge → `semantic_map_ros` 节点 → 适配层取点位 / 发导航 | "点位与语义"标签 |
| G1 原版 Vue 前端（`aidrobo_client-haier`，备用） | 同上 | 功能页 → **语义地图** |
| G1D 底盘控制台（`app.py` + `index.html`，备用） | Flask `/api/semantic/*` → `g1d_semantic.py`（**不依赖 ROS**）→ 直接读 G1D POI / 下发 MoveToAction | 右侧 **语义** 标签 |

三个前端读写的是同一份描述：G1D POI 的 `metadata.description`（名称在 `metadata.display_name`）。

## 在 8550（192.168.128.59，AidLux / Ubuntu 22.04 / ROS humble）上的部署

部署脚本、开机自启、目录说明见 `deploy/README.md`。简要：

| 位置 | 内容 |
|---|---|
| `~/G1D/slam_ws` | 本目录（从电脑 `bash deploy/sync_to_8550.sh` 同步过去） |
| `~/g1d_ws` | ROS 编译工作区，`src/` 下的包链接到 `slam_ws/ros2_ws/src` |
| `~/models/bge-m3` | BGE-M3 模型（CPU 推理，加载约 5 s，每句约 0.1~1 s） |
| `~/maps/semantic_qdrant` | ROS 节点的向量库（qdrant-client 本地文件模式，机器上没有 Docker） |
| `~/maps/semantic_qdrant_flask` | 底盘控制台的向量库（和上面分开，两个进程不能同时锁同一个目录） |
| `~/maps/semantic_map_log.jsonl`、`semantic_map_flask_log.jsonl` | 调试记录（每次请求一行） |
| `~/G1D/bridge_data` | 适配层的地图目录和快照（不要删） |

开机自启的 systemd 服务 `g1d` 启动：rosbridge 9090、语义地图节点、g1_web **8090**、Vue **8091**、底盘控制台 **5001**、适配层（连 G1D）。
8080 / 8081 / 5000 被 AidLux 自带的 filebrowser / ttyd / gstd 占用。

浏览器打开（电脑上开了代理的话，给 192.168.x.x 设成直连，否则会 502）：
- g1_web：`http://192.168.128.59:8090/`（自动连页面所在主机的 9090）。也可以在电脑本地开 g1_web 页面：
  `python3 deploy/serve.py 8090 --directory g1_web/www`，然后打开 `http://localhost:8090/?host=192.168.128.59`。
- Vue：`http://192.168.128.59:8091/`，ROS 地址在"设置"页填 `192.168.128.59`。
- 底盘控制台：`http://192.168.128.59:5001/`，先填 G1D 的 IP（192.168.123.163）点"连接"。

## 使用

1. 给位置点填描述：g1_web"点位与语义"页新建或编辑点位时填；Vue"语义地图"页右侧或新建点位弹窗里填；底盘控制台 POI 标签新建时填，或点"描述"按钮修改。
2. 先用"搜索"多试几句，确认匹配对了再用"去那里"。

## 改动记录

### 2026-09-30：建图接口、g1_web 适配 G1D、代码收回 slam_ws

- `/mode_set mapping` = 新建地图：先把 G1D 上的地图备份到 `bridge_data/backup/`（保留 10 份），再 `DELETE /api/core/slam/v1/maps` 清空后建图；
  新增 `continue_mapping` = 在现有地图上继续建。`localization` / `patrol` 时如果 G1D 上已经不是当前地图，先把当前地图传回去。
  `/aid_save_map` 保存后关掉建图开关（和 G1 一致）。g1_web 加"继续建图"按钮，"开始建图"要点两次确认。
- 适配层地图图片改用 G1 的配色（未探索 82,108,170 / 障碍黑 / 空地 127,145,200，JPEG 4:4:4）。
- 适配层新增 `/robot_status`（1 Hz）：SLAM 状态读 G1D 的建图 / 定位开关，控制模式记最近一次 `/mode_set`。
- g1_web 重定位提示：G1D 上 `/initialpose` 直接设置底盘位姿，立即生效。
- 部署：g1_web、`serve.py`、ROS 包、部署脚本、测试工具都收进本目录；开机自启 systemd 服务 `g1d`（`deploy/`）。
- 测试：`test_robot_status_follows_g1d_switches_and_mode_set`、`test_new_mapping_backs_up_then_clears_and_localization_restores_current_map`、
  `test_save_ends_mapping_and_saved_map_stays_loaded`、地图配色断言。

### 2026-09-29：语义地图

- `g1d_g1_core.py`：点位读写带 `description`（存 POI metadata）；更新时没带描述则保留原描述。测试 `test_point_description_round_trip`。
- `app.py`：`/api/semantic/{status,search,go,rebuild,history}`；新增 POI 可带描述；`PUT /api/pois/<id>` 改名称 / 描述；端口可配。
- `g1d_semantic.py`：底盘控制台用的语义地图服务（复用 `semantic_map_ros` 的 `SemanticIndex` / `DebugLog` / `Embedder`，模型第一次用到才加载）。测试 `test_g1d_semantic.py`（假编码器，不连机器人）。
- `index.html`：右侧"语义"标签；POI 列表显示 / 修改描述。
- Vue：`src/views/utility/semantic.vue`（新页面）、`src/router/index.js`、`src/views/utility/index.vue`（菜单）、
  `src/components/map/edit.vue`（新建点位弹窗加描述）、`static2/js/rosApi.js`（语义地图服务 / 话题）。改完 `npm run build`（Node 16）。
- 改动前的原文件备份在 `.orig_backup_20260929/`。

## 测试（8550 上）

```bash
source /opt/ros/humble/setup.bash; source ~/g1d_ws/install/setup.bash
cd ~/G1D/slam_ws && python3 -m unittest test_g1d_g1_core test_g1d_semantic
cd ~/G1D/slam_ws/ros2_ws && PYTHONPATH=src/semantic_map_ros python3 -m pytest src/semantic_map_ros/test -p no:anyio -p no:cacheprovider
G1D_HOST=192.168.128.59 python3 tools/ro_test.py   # 在电脑上：经 rosbridge 只读检查全部接口（不导航、不改状态）
```
