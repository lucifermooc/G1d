#!/bin/bash
# G1D 开发计算单元 PC2（Jetson Orin NX，JetPack 5.1.1 / Ubuntu 20.04 / ROS 2 Foxy / Python 3.8）装依赖，不需要 sudo，可重复运行。
#   G1D_PROXY=http://127.0.0.1:17897 bash ~/G1D/slam_ws/deploy/setup_pc2.sh
# PC2 不能上网：在电脑上 ssh -J aidlux@<8550> -R 17897:127.0.0.1:<电脑代理端口> unitree@192.168.123.164 把代理借给它。
# - Python 包装到 ~/g1d_pydeps（PYTHONUSERBASE），只有 G1D 的进程用（run_g1d.sh 会设置），不影响系统 Python 和宇树的程序
# - rosbridge_suite 1.3.2（Foxy 能用的最后一版）从源码编译进 ~/g1d_ws（Foxy 已停更，apt 源不好用，也免 sudo）
# - 模型 ~/models/bge-m3：从 8550 拷，或者 bash fetch_bge_m3.sh（走同一个代理）
set -e
D=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
L=~/deploy_logs; mkdir -p $L ~/g1d_wheels ~/g1d_ws/src
export PYTHONUSERBASE=$HOME/g1d_pydeps
PY=/usr/bin/python3
PIP="$PY -m pip install --user --progress-bar off -i https://pypi.tuna.tsinghua.edu.cn/simple --timeout 60 --retries 10"
[ -n "$G1D_PROXY" ] && PIP="$PIP --proxy $G1D_PROXY" && export https_proxy=$G1D_PROXY http_proxy=$G1D_PROXY

echo "== pip（系统的 20.0 太老，解析不了依赖）"
$PIP --upgrade "pip<25"

echo "== torch 2.0.0（NVIDIA 给 JetPack 5.1.1 编的 CUDA 版）"
W=torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
if [ ! -s ~/g1d_wheels/$W ]; then
  curl -fL --retry 5 -o ~/g1d_wheels/$W.part https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/$W
  mv ~/g1d_wheels/$W.part ~/g1d_wheels/$W
fi
$PIP ~/g1d_wheels/$W   # 本地 wheel 不能加约束；装好后后面的步骤用约束锁住它

echo "== 语义地图、底盘控制台、rosbridge 的 Python 依赖"
# pandas：系统自带的 0.25 和新 numpy 不兼容，datasets 会用到，另装一份
$PIP -c "$D/pc2_constraints.txt" "FlagEmbedding>=1.3" "qdrant-client>=1.12" flask pymongo tornado "pandas>=1.5"

echo "== rosbridge_suite 源码"
[ -d ~/g1d_ws/src/rosbridge_suite ] || git clone -q --depth 1 --branch 1.3.2 \
  https://github.com/RobotWebTools/rosbridge_suite.git ~/g1d_ws/src/rosbridge_suite

echo "== 检查（和 run_g1d.sh 一样预加载 scikit-learn 的 libgomp，否则 Jetson 上报 static TLS）"
GOMP=$(ls "$PYTHONUSERBASE"/lib/python3*/site-packages/scikit_learn.libs/libgomp-*.so* 2>/dev/null | head -1)
[ -n "$GOMP" ] && export LD_PRELOAD="$GOMP"
$PY - <<'EOF'
import torch, flask, bson, tornado, qdrant_client, FlagEmbedding, transformers, numpy
print("torch", torch.__version__, "cuda", torch.cuda.is_available(), "| numpy", numpy.__version__,
      "| transformers", transformers.__version__, "| qdrant", qdrant_client.__version__ if hasattr(qdrant_client, "__version__") else "")
EOF
echo SETUP_PC2_DONE
