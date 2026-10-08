#!/bin/bash
# 停掉 G1D 整套服务，包括适配层。装了开机自启时先停 systemd 服务 g1d（下次开机仍会自启）。
if systemctl is-active --quiet g1d 2>/dev/null; then
  sudo -n systemctl stop g1d || { echo "没有免密权限，请运行：sudo systemctl stop g1d"; exit 1; }
fi
# 手动运行（run_g1d.sh）或残留的进程；方括号写法避免匹配到自己
P="[r]un_g1d.sh [r]osbridge_websocket [r]osapi_node [s]emantic_map.launch.py [s]emantic_map_server
   [s]lam_ws/deploy/serve.py [p]ython3.app.py [g]1d_g1_ros_bridge.py"
for sig in INT 9; do
  for pat in $P; do
    pids=$(pgrep -f "$pat"); [ -n "$pids" ] && kill -$sig $pids 2>/dev/null
  done
  [ $sig = INT ] && sleep 3
done
echo stopped
