# PC2 部署

G1D 在自带的开发计算单元 PC2（Jetson Orin NX / JetPack 5.1.1 / Ubuntu 20.04 / ROS 2 Foxy / Python 3.8）上运行的全部代码
都在 `slam_ws` 里，PC2 上的 `~/G1D/slam_ws` 只从这里同步。接口说明见 `../G1D_Communication_Documentation.md`。

## 机器

| | 登录 | G1D 底盘固件 | 状态 |
|---|---|---|---|
| G1D_01 | `ssh -p 30522 unitree@47.108.223.180` | Slamtec `ares2-ys`，6.3.3-rc4-3399 | 已部署，开机自启 |
| G1D_02 | `ssh -p 30523 unitree@47.108.223.180` | UNITREE `Ares2_Unitree_G1d`，6.3.3-rc9-3588 | 已部署，开机自启 |
| G1D_03 | `ssh -p 30525 unitree@47.108.223.180` | UNITREE `Ares2_Unitree_G1d`，6.3.3-rc9-3588 | 已部署（2026-10-22），开机自启 |

都经公网转发登录；每台机器人内网都一样：PC2 的 eth0 是 192.168.123.164，G1D 底盘在 192.168.123.163。
底盘固件版本看 `curl http://192.168.123.163:1448/api/core/system/v1/robot/info`。

建议在电脑的 `~/.ssh/config` 里给每台机器起个别名，下面的命令都用别名：

```
Host g1d-pc2
    HostName 47.108.223.180
    Port 30522
    User unitree
```

## 服务名称和开关

服务叫 **`g1d`**（systemd 服务），一个服务管整套：rosbridge、语义地图、三个网页、G1 接口适配层（连 G1D）。开机自启。

在 PC2 上：

| 操作 | 命令 | 要密码吗 |
|---|---|---|
| 启动 / 重启 | `bash ~/start_g1d.sh`（= `sudo systemctl restart g1d`） | 不要 |
| 停止 | `bash ~/stop_g1d.sh`（= `sudo systemctl stop g1d`，再清理残留进程） | 不要 |
| 看状态 | `systemctl status g1d` | 不要 |
| 看日志 | `journalctl -u g1d -f`（启停记录）；`~/g1d_logs/<服务>.log`（各服务的日志） | 不要 |
| 设为开机自启 | `sudo systemctl enable g1d` | 要 |
| 取消开机自启 | `sudo systemctl disable g1d`（不影响当前运行；要同时停掉用 `sudo systemctl disable --now g1d`） | 要 |

- 不用密码的只有 `start` / `stop` / `restart g1d` 这三条（`/etc/sudoers.d/g1d-service`），其它 `systemctl` 操作要输 `unitree` 的密码。
- 停止后，开机自启的设置不变：下次开机还会自己起来。
- 重启 / 停止前确认机器人没有在执行导航任务：重启后适配层不记得之前的任务，`pause` / `cancel` 控制不了它。
- 还没装服务的时候，`bash ~/start_g1d.sh robot` 也能启动（不经 systemd，在后台运行，不会开机自启），`bash ~/stop_g1d.sh` 停止。

电脑访问 PC2 的网页：`ssh -N -L 8090:localhost:8090 -L 9090:localhost:9090 g1d-pc2`，然后打开 `http://localhost:8090/`。

PC2 上还跑着宇树自己的程序（`slamware_service`、`teleimager`、`unitree_eai_agent`），不要动它们；我们的东西都不改系统环境：

- Python 包在 `~/g1d_pydeps`（`PYTHONUSERBASE`），只有 `run_g1d.sh` 起的进程用；系统 Python、宇树的 conda 环境不受影响。
- `run_g1d.sh` 自动：用 Foxy、`/usr/bin/python3`（`~/.bashrc` 会激活 conda 的 3.10，要避开）、GPU 半精度推理、
  预加载 scikit-learn 自带的 libgomp（Jetson 上不预加载会报 "cannot allocate memory in static TLS block"）。

## 目录

| 路径 | 内容 |
|---|---|
| `g1d_g1_ros_bridge.py`、`g1d_g1_core.py` | G1 接口适配层（G1 的 ROS 服务 / 话题 → G1D HTTP :1448） |
| `app.py`、`index.html`、`g1d_semantic.py` | G1D 底盘控制台（Flask，备用） |
| `g1_web/www/` | g1_web 控制台（主用），复制自 `unitree-G1/lightning_ws/src/g1_web/www`，改那边后复制过来 |
| `ros2_ws/src/` | `aid_robot_msgs`、`semantic_map_ros`（复制自 unitree-G1 仓库），PC2 上在 `~/g1d_ws` 编译 |
| `deploy/` | 本目录：启动、自启、同步、安装脚本 |
| `tools/` | 测试工具：`ro_test.py`（只读检查全部接口）、`mock_points.py`（假点位服务）等 |

## 脚本

| 脚本 | 在哪运行 | 作用 |
|---|---|---|
| `sync_to_8550.sh <用户@IP 或别名>` | 电脑 | 同步 `slam_ws` 到 PC2；把 `~/g1d_ws/src` 的包、`~/start_g1d.sh`、`~/stop_g1d.sh` 链接到这里。要写目标：`bash deploy/sync_to_8550.sh g1d-pc2` |
| `run_g1d.sh [robot]` | PC2 | 前台运行全部服务（systemd 调它）；任何一个进程退出就全部停掉，由 systemd 重启 |
| `start_g1d.sh` / `stop_g1d.sh` | PC2 | 重启 / 停止。装了自启时控制 systemd 服务 `g1d`（不用密码）；没装时后台运行 `run_g1d.sh` |
| `install_autostart.sh [--no-enable]` | PC2，`sudo` | 装 systemd 服务 `g1d`（`g1d.service`）和免密启停规则 `/etc/sudoers.d/g1d-service`，设为开机自启并启动；`--no-enable` 只装不自启 |
| `setup_pc2.sh`、`pc2_constraints.txt` | PC2 | 装依赖（不用 sudo）：Python 包进 `~/g1d_pydeps`、NVIDIA 的 JetPack 版 torch、rosbridge 1.3.2 源码 |
| `fetch_bge_m3.sh` | PC2 | 从 hf-mirror 下载 / 续传 BGE-M3 到 `~/models/bge-m3` |
| `serve.py` | 任意 | 不缓存的静态网页服务 |

## 服务

- 端口：rosbridge 9090、g1_web 8090、Vue 8091、底盘控制台 5001。Vue 服务的目录 `aidrobo_client-haier/dist/` 不在仓库里，8091 目前没有内容。
- rosbridge 带 `call_services_in_new_thread:=true`：语义检索这类长 service 默认占住消息线程，取消 / 遥测全排队，改成每次调用独立线程。
- `ROS_DOMAIN_ID=71`，`ROS_LOCALHOST_ONLY=1`（只在本机通信，`G1D_LOCALHOST_ONLY=0` 可关），FastDDS；
  适配层连 `G1D_ROBOT_IP`（默认 192.168.123.163）。
- 日志：`~/g1d_logs/<服务>.log`（每次启动覆盖），`journalctl -u g1d`。
- 数据：`~/G1D/bridge_data`（适配层的地图目录、快照、建图前的备份，不要删）、`~/maps/`（语义地图向量库和调试记录）。

## 更新代码

```bash
bash deploy/sync_to_8550.sh g1d-pc2        # 电脑上
# 改了 ros2_ws/src 时，在 PC2 上先停服务再编译（编译会覆盖正在用的库）：
bash ~/stop_g1d.sh; source /opt/ros/foxy/setup.bash; cd ~/g1d_ws && colcon build
bash ~/start_g1d.sh                        # PC2 上重启生效
```

⚠ 重启前确认没有导航任务在执行：重启后适配层不记得之前的任务，`pause` / `cancel` 控制不了它。

## 新机器从头装

1. `bash deploy/sync_to_8550.sh <别名>`
2. PC2 上：`bash ~/G1D/slam_ws/deploy/setup_pc2.sh`（版本约束见 `pc2_constraints.txt`：numpy 1.24、transformers 4.46、
   FlagEmbedding 1.3.5、torch 2.0 JetPack 版）
3. 模型：从已部署的 PC2 拷 `~/models/bge-m3`（2.2 GB），或 `bash ~/G1D/slam_ws/deploy/fetch_bge_m3.sh`
4. `source /opt/ros/foxy/setup.bash; cd ~/g1d_ws && colcon build`（rosbridge 1.3.2 从源码编在这里，Foxy 已停更，apt 源不好用）
5. `sudo bash ~/G1D/slam_ws/deploy/install_autostart.sh`

PC2 不能上网时，把电脑的代理借给它（电脑代理端口 7897 为例）：

```bash
ssh -N -R 17897:127.0.0.1:7897 <别名> &                                           # 电脑上
G1D_PROXY=http://127.0.0.1:17897 bash ~/G1D/slam_ws/deploy/setup_pc2.sh          # PC2 上
```
