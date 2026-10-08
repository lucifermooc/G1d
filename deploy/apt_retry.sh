#!/bin/bash
# 等系统自动更新（unattended-upgrades）放开 dpkg 锁后再装 rosbridge
for i in $(seq 180); do
  sudo apt-get install -y -o DPkg::Lock::Timeout=10 ros-humble-rosbridge-server ros-humble-nav2-msgs && { echo APT_EXIT=0; exit 0; }
  echo "retry $i $(date +%T)"; sleep 10
done
echo APT_EXIT=1
