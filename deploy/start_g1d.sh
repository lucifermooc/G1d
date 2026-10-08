#!/bin/bash
# 启动 / 重启 G1D 整套服务（端口和日志见 run_g1d.sh）。
#   已装开机自启（install_autostart.sh）：重启 systemd 服务 g1d（总是连 G1D，参数忽略）
#   没装：后台运行 run_g1d.sh；bash start_g1d.sh robot 连 G1D，不带参数不连机器人
D=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
if systemctl cat g1d >/dev/null 2>&1; then
  if sudo -n systemctl restart g1d; then
    sleep 3; echo "g1d 服务已重启（连 G1D）。状态：systemctl status g1d，日志 ~/g1d_logs/"
  else
    echo "没有免密权限，请运行：sudo systemctl restart g1d"; exit 1
  fi
  exit 0
fi
bash "$D/stop_g1d.sh" >/dev/null 2>&1
mkdir -p ~/g1d_logs
# 经 Python 把 SIGINT 恢复成默认再启动：否则 & 出去的 run_g1d.sh 忽略 SIGINT，stop_g1d.sh 没法让它按顺序停
setsid nohup /usr/bin/python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  bash "$D/run_g1d.sh" "$1" > ~/g1d_logs/run.log 2>&1 < /dev/null &
sleep 3
cat ~/g1d_logs/run.log
