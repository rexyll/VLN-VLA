#!/usr/bin/env bash
# vm_provision.sh —— VM 编译/运行环境安装（Gazebo Harmonic 栈）
set -ex
export DEBIAN_FRONTEND=noninteractive
apt-get update -o Dpkg::Lock::Timeout=120
apt-get install -y -o Dpkg::Lock::Timeout=120 build-essential pkg-config curl
# 检查 gz-sim8 开发头文件（插件编译需要）
apt-get install -y -o Dpkg::Lock::Timeout=120 libgz-sim8-dev || echo NO-LIBGZ-SIM8-DEV
# Python 依赖（torch CPU + pillow）；Ubuntu 24.04 需 break-system-packages
pip3 install --quiet --break-system-packages torch --index-url https://download.pytorch.org/whl/cpu
pip3 install --quiet --break-system-packages pillow
python3 -c "import torch, numpy, PIL; print('VM-PY-DEPS-OK torch', torch.__version__)"
gz sim --version 2>/dev/null | head -2 || true
echo PROVISION-DONE
