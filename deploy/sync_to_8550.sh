#!/bin/bash
# 在电脑上运行：把整个 slam_ws 同步到 8550（或 G1D 的 PC2）的 ~/G1D/slam_ws（板子上的代码只从这里来）。
#   bash deploy/sync_to_8550.sh [aidlux@192.168.128.59]
#   bash deploy/sync_to_8550.sh unitree@192.168.123.164 aidlux@192.168.128.59   # PC2，第二个参数是跳板（经 8550）
# 同时：~/g1d_ws/src 里的 ROS 包链接到 slam_ws/ros2_ws/src；~/start_g1d.sh、~/stop_g1d.sh 链接到 deploy/。
# 之后：改了 ros2_ws/src 要在 8550 上重新编译（README.md）；然后 bash ~/start_g1d.sh 重启生效。
set -e
H=${1:-aidlux@192.168.128.59}
SSH="ssh${2:+ -J $2}"
D=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
rsync -a --delete --exclude='__pycache__/' --exclude='.pytest_cache/' --exclude='node_modules/' \
  --exclude='dist.prev/' --exclude='.orig_backup_*/' --exclude='nohup.out' -e "$SSH" "$D/" "$H:G1D/slam_ws/"
$SSH "$H" 'set -e; mkdir -p ~/g1d_ws/src
  for p in aid_robot_msgs semantic_map_ros; do
    if [ ! -L ~/g1d_ws/src/$p ]; then
      [ -e ~/g1d_ws/src/$p ] && mv ~/g1d_ws/src/$p ~/g1d_ws_src_old_$p   # 旧的实体目录挪出工作区
      ln -s ~/G1D/slam_ws/ros2_ws/src/$p ~/g1d_ws/src/$p
    fi
  done
  ln -sf ~/G1D/slam_ws/deploy/start_g1d.sh ~/start_g1d.sh
  ln -sf ~/G1D/slam_ws/deploy/stop_g1d.sh ~/stop_g1d.sh
  echo "同步完成：$(hostname) ~/G1D/slam_ws"'
