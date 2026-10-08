#!/bin/bash
# 8550（AidLux, Ubuntu 22.04, ROS humble）上的依赖安装。每一段各自写日志，可重复运行。
# 用法：bash ~/G1D/slam_ws/deploy/setup_8550.sh（apt 那段要 sudo；系统自动更新占着 dpkg 锁时改用 apt_retry.sh）
D=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
L=~/deploy_logs; mkdir -p $L ~/models

# 1) rosbridge
( sudo DEBIAN_FRONTEND=noninteractive apt-get update -q
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-rosbridge-server ros-humble-nav2-msgs
  echo APT_EXIT=$? ) > $L/apt.log 2>&1 &

# 2) Python：CPU 版 torch（官方 CPU 源），其余从阿里云；锁 numpy / setuptools 保护系统 OpenCV 和 colcon
( for i in 1 2 3 4 5; do
    python3 -m pip install --user -c "$D/semantic_map_constraints.txt" torch --index-url https://download.pytorch.org/whl/cpu \
      --timeout 60 --retries 10 --progress-bar off && break; sleep 10; done
  for i in 1 2 3 4 5; do
    python3 -m pip install --user -c "$D/semantic_map_constraints.txt" "FlagEmbedding>=1.3" "qdrant-client>=1.12" \
      -i https://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com --timeout 60 --retries 10 --progress-bar off && break; sleep 10; done
  echo PIP_EXIT=$? ) > $L/pip.log 2>&1 &

# 3) BGE-M3 模型
( bash "$D/fetch_bge_m3.sh" ) > $L/model.log 2>&1 &

# 4) Node.js 16（Vue 2 + webpack 老项目用），装到 ~/node16，不用 sudo
( cd ~ && for i in 1 2 3; do
    curl -sfL --retry 5 -o node16.tar.xz https://registry.npmmirror.com/-/binary/node/v16.20.2/node-v16.20.2-linux-arm64.tar.xz && break; sleep 5; done
  mkdir -p ~/node16 && tar -xJf node16.tar.xz -C ~/node16 --strip-components=1 && rm -f node16.tar.xz
  ~/node16/bin/node --version && echo NODE_OK ) > $L/node.log 2>&1 &

wait
echo SETUP_DONE
