# 8550 / PC2 部署

G1D 在 8550 板卡（AidLux / Ubuntu 22.04 / ROS 2 Humble，主用）和 G1D 自带的开发计算单元 PC2（Jetson Orin NX /
Ubuntu 20.04 / ROS 2 Foxy，备用）上运行的全部代码都在 `slam_ws` 里，两块板子上的 `~/G1D/slam_ws` 只从这里同步。
接口说明见 `../G1D_Communication_Documentation.md`。

## 服务名称和开关

两块板子上的服务都叫 **`g1d`**（systemd 服务），一个服务管整套：rosbridge、语义地图、三个网页、G1 接口适配层（连 G1D）。

| | 8550（主用） | PC2（备用） |
|---|---|---|
| 登录 | `ssh aidlux@192.168.128.59` | `ssh -J aidlux@192.168.128.59 unitree@192.168.123.164`（经 8550 跳） |
| 开机自启 | **是** | **否**（装了服务，不自启） |
| 平时状态 | 运行中 | 停着 |

在板子上：

| 操作 | 命令 | 要密码吗 |
|---|---|---|
| 启动 / 重启 | `bash ~/start_g1d.sh`（= `sudo systemctl restart g1d`） | 不要 |
| 停止 | `bash ~/stop_g1d.sh`（= `sudo systemctl stop g1d`，再清理残留进程） | 不要 |
| 看状态 | `systemctl status g1d` | 不要 |
| 看日志 | `journalctl -u g1d -f`（启停记录）；`~/g1d_logs/<服务>.log`（各服务的日志） | 不要 |
| 设为开机自启 | `sudo systemctl enable g1d` | 要 |
| 取消开机自启 | `sudo systemctl disable g1d`（不影响当前运行；要同时停掉用 `sudo systemctl disable --now g1d`） | 要 |

- 不用密码的只有 `start` / `stop` / `restart g1d` 这三条（`/etc/sudoers.d/g1d-service`），其它 `systemctl` 操作要输密码
  （8550 是 `aidlux` 的密码，PC2 是 `unitree` 的密码）。
- 停止后，开机自启的设置不变：8550 下次开机还会自己起来。
- 重启 / 停止前确认机器人没有在执行导航任务：重启后适配层不记得之前的任务，`pause` / `cancel` 控制不了它。

⚠ **同一时间只用一套**：两套都连着同一台 G1D，同时开着的话两边的前端都能给机器人下命令。切换方法：

```bash
# 8550 → PC2
ssh aidlux@192.168.128.59 'bash ~/stop_g1d.sh'
ssh -J aidlux@192.168.128.59 unitree@192.168.123.164 'bash ~/start_g1d.sh'
# PC2 → 8550
ssh -J aidlux@192.168.128.59 unitree@192.168.123.164 'bash ~/stop_g1d.sh'
ssh aidlux@192.168.128.59 'bash ~/start_g1d.sh'
```

要长期改用 PC2：在 8550 上 `sudo systemctl disable --now g1d`，在 PC2 上 `sudo systemctl enable --now g1d`。

PC2 的服务要先装一次（只装、不自启、不启动），在电脑上运行，按提示输入 PC2 的密码：

```bash
ssh -t -J aidlux@192.168.128.59 unitree@192.168.123.164 'sudo bash ~/G1D/slam_ws/deploy/install_autostart.sh --no-enable'
```

还没装的时候，`bash ~/start_g1d.sh robot` 也能启动（不经 systemd，在后台运行，不会开机自启），`bash ~/stop_g1d.sh` 停止。

## 目录

| 路径 | 内容 |
|---|---|
| `g1d_g1_ros_bridge.py`、`g1d_g1_core.py` | G1 接口适配层（G1 的 ROS 服务 / 话题 → G1D HTTP :1448） |
| `app.py`、`index.html`、`g1d_semantic.py` | G1D 底盘控制台（Flask，备用） |
| `g1_web/www/` | g1_web 控制台（主用），复制自 `unitree-G1/lightning_ws/src/g1_web/www`，改那边后复制过来 |
| `aidrobo_client-haier/dist/` | G1 原版 Vue 前端的打包结果（备用） |
| `ros2_ws/src/` | `aid_robot_msgs`、`semantic_map_ros`（复制自 unitree-G1 仓库），8550 上在 `~/g1d_ws` 编译 |
| `deploy/` | 本目录：启动、自启、同步、安装脚本 |
| `tools/` | 测试工具：`ro_test.py`（只读检查全部接口）、`mock_points.py`（假点位服务）等 |

## 脚本

| 脚本 | 在哪运行 | 作用 |
|---|---|---|
| `sync_to_8550.sh [用户@IP] [跳板]` | 电脑 | 同步 `slam_ws` 到板子；把 `~/g1d_ws/src` 的包、`~/start_g1d.sh`、`~/stop_g1d.sh` 链接到这里。PC2：`bash deploy/sync_to_8550.sh unitree@192.168.123.164 aidlux@192.168.128.59` |
| `run_g1d.sh [robot]` | 8550 | 前台运行全部服务（systemd 调它）；任何一个进程退出就全部停掉，由 systemd 重启 |
| `start_g1d.sh` / `stop_g1d.sh` | 8550 | 重启 / 停止。装了自启时控制 systemd 服务 `g1d`（不用密码）；没装时后台运行 `run_g1d.sh` |
| `install_autostart.sh [--no-enable]` | 板子，`sudo` | 装 systemd 服务 `g1d`（`g1d.service`）和免密启停规则 `/etc/sudoers.d/g1d-service`；`--no-enable` 只装不自启（PC2） |
| `setup_8550.sh` | 8550 | 新机器装依赖：rosbridge、nav2-msgs、CPU 版 torch、FlagEmbedding、qdrant-client、BGE-M3 模型、Node 16 |
| `setup_pc2.sh`、`pc2_constraints.txt` | PC2 | 装依赖（不用 sudo）：Python 包进 `~/g1d_pydeps`、NVIDIA 的 JetPack 版 torch、rosbridge 1.3.2 源码（见下面"PC2"） |
| `fetch_bge_m3.sh` | 8550 | 从 hf-mirror 下载 / 续传 BGE-M3 到 `~/models/bge-m3` |
| `apt_retry.sh` | 8550 | 系统自动更新占着 dpkg 锁时，循环重试装 apt 包 |
| `serve.py` | 任意 | 不缓存的静态网页服务 |

## 服务

- 端口：rosbridge 9090、g1_web 8090、Vue 8091、底盘控制台 5001（8080 / 8081 / 5000 被 AidLux 自带服务占用）。
- `ROS_DOMAIN_ID=71`，`ROS_LOCALHOST_ONLY=1`（只在本机通信，8550 和 PC2 不会互相收到消息；`G1D_LOCALHOST_ONLY=0` 可关），
  FastDDS；适配层连 `G1D_ROBOT_IP`（默认 192.168.123.163，8550 的 eth0 是 192.168.123.100）。
- 日志：`~/g1d_logs/<服务>.log`（每次启动覆盖），`journalctl -u g1d`。
- 数据：`~/G1D/bridge_data`（适配层的地图目录、快照、建图前的备份，不要删）、`~/maps/`（语义地图向量库和调试记录）。

## 更新代码

```bash
bash deploy/sync_to_8550.sh                 # 电脑上
# 改了 ros2_ws/src 时，在 8550 上先停服务再编译（编译会覆盖正在用的库）：
bash ~/stop_g1d.sh; source /opt/ros/humble/setup.bash; cd ~/g1d_ws && colcon build
bash ~/start_g1d.sh                         # 8550 上重启生效
```

⚠ 重启前确认没有导航任务在执行：重启后适配层不记得之前的任务，`pause` / `cancel` 控制不了它。

## 新机器从头装

1. `bash deploy/sync_to_8550.sh aidlux@<IP>`
2. 8550 上：`bash ~/G1D/slam_ws/deploy/setup_8550.sh`（apt 那段要 sudo 密码；dpkg 被占用时用 `apt_retry.sh`）
3. `source /opt/ros/humble/setup.bash; cd ~/g1d_ws && colcon build`
4. `sudo bash ~/G1D/slam_ws/deploy/install_autostart.sh`

## PC2（备用，2026-09-30 部署）

G1D 的开发计算单元 PC2：`unitree@192.168.123.164`，只在机器人内部网段，从电脑要经 8550 跳：`ssh -J aidlux@192.168.128.59 unitree@192.168.123.164`。
PC2 上还跑着宇树自己的程序（`slamware_service`、`teleimager`、`unitree_eai_agent`），不要动它们；我们的东西都不改系统环境：

- Python 包在 `~/g1d_pydeps`（`PYTHONUSERBASE`），只有 `run_g1d.sh` 起的进程用；系统 Python、宇树的 conda 环境不受影响。
- 版本约束见 `pc2_constraints.txt`（Python 3.8：numpy 1.24、transformers 4.46、FlagEmbedding 1.3.5、torch 2.0 JetPack 版）。
- `run_g1d.sh` 在 PC2 上自动：用 Foxy、`/usr/bin/python3`（`~/.bashrc` 会激活 conda 的 3.10，要避开）、GPU 半精度推理、
  预加载 scikit-learn 自带的 libgomp（Jetson 上不预加载会报 "cannot allocate memory in static TLS block"）。
- rosbridge 1.3.2 从源码编在 `~/g1d_ws`（Foxy 已停更，apt 源不好用）。
- 服务 `g1d` 装了但**不开机自启**。要用：`bash ~/start_g1d.sh`；改成开机自启：`sudo systemctl enable --now g1d`。
  ⚠ 同一时间只用一套（8550 或 PC2），两套都连着同一台 G1D。
- 电脑访问 PC2 的网页：`ssh -N -J aidlux@192.168.128.59 -L 8090:localhost:8090 -L 9090:localhost:9090 unitree@192.168.123.164`，
  然后打开 `http://localhost:8090/`。

PC2 不能上网。重新装依赖时把电脑的代理借给它（电脑代理端口 7897 为例）：

```bash
ssh -N -J aidlux@192.168.128.59 -R 17897:127.0.0.1:7897 unitree@192.168.123.164 &     # 电脑上
G1D_PROXY=http://127.0.0.1:17897 bash ~/G1D/slam_ws/deploy/setup_pc2.sh              # PC2 上
source /opt/ros/foxy/setup.bash; cd ~/g1d_ws && colcon build                         # PC2 上
```

模型不用重新下载：从 8550 拷 `~/models/bge-m3`（2.2 GB）。
