#!/bin/bash
# G1D 整套服务，前台运行（开机自启的 systemd 服务 g1d.service 就是调它）。
# 任何一个进程退出，就停掉其余进程并以非 0 退出，由 systemd 5 秒后整体重启。
#   bash run_g1d.sh          不连机器人（不启动适配层）
#   bash run_g1d.sh robot    同时启动适配层 g1d_g1_ros_bridge.py，连 G1D（$G1D_ROBOT_IP，默认 192.168.123.163）
# 端口：rosbridge 9090，g1_web 控制台 8090（主用），G1 原版 Vue 前端 8091（备用），G1D 底盘控制台 5001（备用）
# 日志：~/g1d_logs/<服务>.log（每次启动覆盖）；本脚本自己的输出在 journalctl -u g1d
# 8550（Humble / Python 3.10）和 G1D 的 PC2（Foxy / Python 3.8 / Jetson）共用：ROS 版本自动找；
# 有 ~/g1d_pydeps 就用它里面的 Python 包（PC2）；Jetson 上语义模型用 GPU 半精度（G1D_FP16 可改）。
D=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)   # slam_ws
L=$HOME/g1d_logs
mkdir -p "$L" "$HOME/maps" "$HOME/G1D/bridge_data"
# 从激活了 conda 的终端手动启动时，别让 conda 的 python 顶掉系统 python（ROS 只认系统的）
PATH=$(printf %s "$PATH" | tr ':' '\n' | grep -v conda | paste -sd: -)
PY=${G1D_PYTHON:-/usr/bin/python3}
for distro in humble foxy; do
  [ -f /opt/ros/$distro/setup.bash ] && { source /opt/ros/$distro/setup.bash; break; }
done
source "$HOME/g1d_ws/install/setup.bash"
if [ -d "$HOME/g1d_pydeps" ]; then
  export PYTHONUSERBASE=$HOME/g1d_pydeps
  # Jetson：scikit-learn 自带的 libgomp 后加载会报 "cannot allocate memory in static TLS block"，先预加载
  GOMP=$(ls "$PYTHONUSERBASE"/lib/python3*/site-packages/scikit_learn.libs/libgomp-*.so* 2>/dev/null | head -1)
  [ -n "$GOMP" ] && export LD_PRELOAD="$GOMP${LD_PRELOAD:+:$LD_PRELOAD}"
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-71} HF_HUB_OFFLINE=1
export RMW_IMPLEMENTATION=${G1D_RMW:-rmw_fastrtps_cpp}   # 固定下来：~/.bashrc 里可能设了别的，手动启动和开机自启要一致
# 只在本机通信：几个进程都在这台机器上。8550 和 PC2 在同一网段、同一个 ROS_DOMAIN_ID，不加这个会互相收到对方的
# /nav_to_pose 等消息，两套同时开着时两个适配层会一起给 G1D 下命令。
export ROS_LOCALHOST_ONLY=${G1D_LOCALHOST_ONLY:-1}
ROBOT_IP=${G1D_ROBOT_IP:-192.168.123.163}
FP16=${G1D_FP16:-$([ -e /etc/nv_tegra_release ] && echo true || echo false)}

declare -A NAME   # pid -> 服务名
# ros2 launch 要靠 SIGINT 才会正常关掉它的子节点，但脚本里 & 出去的进程默认忽略 SIGINT；本脚本自己是被 & 启动的时候
# （手动 start_g1d.sh），bash 的 trap 也改不回来。用 Python 恢复成默认再 exec，和 bash / coreutils 版本无关。
SIGINT_DEFAULT=("$PY" -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])')
run() {  # run <服务名> <命令...>
  local name=$1; shift
  "${SIGINT_DEFAULT[@]}" "$@" > "$L/$name.log" 2>&1 < /dev/null &
  NAME[$!]=$name
}
# ★ 长 service（语义检索）默认占住消息线程，取消/遥测全排队；每次调用独立线程。
run rosbridge    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 call_services_in_new_thread:=true
run semantic_map ros2 launch semantic_map_ros semantic_map.launch.py model_name:="$HOME/models/bge-m3" \
                   qdrant_path:="$HOME/maps/semantic_qdrant" use_fp16:="$FP16"
run g1_web       "$PY" "$D/deploy/serve.py" 8090 --directory "$D/g1_web/www"
run vue          "$PY" "$D/deploy/serve.py" 8091 --directory "$D/aidrobo_client-haier/dist"
run console      env -C "$D" G1D_CONSOLE_PORT=5001 "$PY" app.py
if [ "$1" = robot ]; then
  run bridge     env -C "$D" G1D_ROBOT_IP="$ROBOT_IP" G1D_BRIDGE_DATA="$HOME/G1D/bridge_data" "$PY" g1d_g1_ros_bridge.py
fi

stop() {  # launch 收到 SIGINT 会正常关掉它的子节点；剩下的由 systemd 按 cgroup 清理（手动运行时由 stop_g1d.sh 兜底）
  local p alive
  kill -INT "${!NAME[@]}" 2>/dev/null
  for _ in 1 2 3 4 5 6 7 8; do
    alive=
    for p in "${!NAME[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ -z "$alive" ] && return
    sleep 1
  done
  kill -9 "${!NAME[@]}" 2>/dev/null
}
trap 'stop; exit 0' INT TERM
echo "$(date '+%F %T') 已启动：${NAME[*]}（${1:-不连机器人}${1:+，G1D $ROBOT_IP}）"
wait -n   # bash 5.0（PC2）没有 wait -p，退出后再逐个找是谁
DEAD=
for p in "${!NAME[@]}"; do kill -0 "$p" 2>/dev/null || { DEAD=$p; break; }; done
echo "$(date '+%F %T') ${NAME[$DEAD]:-某个服务} 退出了，停掉全部。它最后的日志："
[ -n "$DEAD" ] && tail -n 20 "$L/${NAME[$DEAD]}.log"
stop
exit 1
