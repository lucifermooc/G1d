#!/bin/bash
# 安装 systemd 服务 g1d（需要 sudo）：
#   sudo bash ~/G1D/slam_ws/deploy/install_autostart.sh               装好、设为开机自启、立即启动（8550，主用）
#   sudo bash ~/G1D/slam_ws/deploy/install_autostart.sh --no-enable   只装好，不开机自启、不启动（PC2，备用）
# 都会装 /etc/sudoers.d/g1d-service：允许当前用户不输密码 start / stop / restart g1d（只限这三条），
# 这样 start_g1d.sh / stop_g1d.sh 和远程维护不用密码。备用机要用时：bash ~/start_g1d.sh（或 sudo systemctl enable --now g1d 改成开机自启）。
# 卸载：sudo systemctl disable --now g1d; sudo rm /etc/systemd/system/g1d.service /etc/sudoers.d/g1d-service
set -e
[ "$(id -u)" = 0 ] || { echo "请用 sudo 运行：sudo bash $0 $*"; exit 1; }
D=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
U=${SUDO_USER:-aidlux}
H=$(getent passwd "$U" | cut -d: -f6)

# g1d.service 按 8550 的 aidlux 写的，换成实际用户和家目录
sed -e "s|/home/aidlux|$H|g" -e "s|^User=aidlux|User=$U|" "$D/g1d.service" > /etc/systemd/system/g1d.service
chmod 644 /etc/systemd/system/g1d.service
printf '%s ALL=(root) NOPASSWD: /usr/bin/systemctl start g1d, /usr/bin/systemctl stop g1d, /usr/bin/systemctl restart g1d\n' "$U" \
  > /etc/sudoers.d/g1d-service
chmod 440 /etc/sudoers.d/g1d-service
visudo -cf /etc/sudoers.d/g1d-service
systemctl daemon-reload

if [ "$1" = "--no-enable" ]; then
  systemctl disable g1d 2>/dev/null || true
  echo "已安装服务 g1d（$U，$H），没有设开机自启，也没有启动。要用时：bash ~/start_g1d.sh"
  exit 0
fi
systemctl enable g1d
sudo -u "$U" bash "$D/stop_g1d.sh" >/dev/null 2>&1 || true   # 先停掉手动启动的实例，再交给 systemd
systemctl restart g1d
sleep 3
systemctl --no-pager status g1d | head -5
echo "已装开机自启。以后用 bash ~/start_g1d.sh / bash ~/stop_g1d.sh（或 sudo systemctl restart g1d）启停。"
