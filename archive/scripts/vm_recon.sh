#!/usr/bin/env bash
# recon.sh —— VM 环境侦察（部署前置检查）
echo ===SYS===
uname -a
grep PRETTY /etc/os-release
nproc
free -h | sed -n 2p
df -h / | tail -1
echo ===SUDO===
echo 111111 | sudo -S whoami 2>/dev/null
echo ===NET===
timeout 8 curl -sI http://archive.ubuntu.com/ubuntu/ 2>/dev/null | head -1
echo ===APT-GAZEBO===
apt-cache policy gazebo11 2>/dev/null | sed -n 1,3p
echo ===INSTALLED===
which gzserver gazebo gz Xvfb xvfb-run 2>/dev/null
dpkg -l 2>/dev/null | grep -iE "gazebo|ros-" | head -5
echo ===PY===
python3 -V
pip3 -V 2>&1 | head -1
python3 -c "import numpy; print(numpy.__version__)" 2>&1 | tail -1
echo ===GUI===
echo DISPLAY=$DISPLAY
echo ===DONE===
