#!/usr/bin/env bash
# tmp_recon.sh —— 本次任务的环境侦察
echo ===SYS===
uname -a; grep PRETTY /etc/os-release; nproc; free -h | sed -n 2p; df -h / | tail -1
echo ===ROBOT-SW===
which gz gzserver gazebo ros2 roscore 2>/dev/null
gz sim --version 2>/dev/null | head -3
echo ===GUI/DESKTOP===
echo "DISPLAY=$DISPLAY"
loginctl list-sessions --no-legend 2>/dev/null
pgrep -af "gnome-shell|xfce4-session|kde|lxde|mate-session" | head -3
ls /tmp/.X11-unix/ 2>/dev/null
which xterm gnome-terminal xdg-open 2>/dev/null
echo ===NET===
timeout 8 curl -sI http://archive.ubuntu.com/ubuntu/ 2>/dev/null | head -1
timeout 8 curl -sI https://alphacephei.com/vosk/ 2>/dev/null | head -1
echo ===PY===
python3 -V; pip3 -V 2>&1 | head -1
python3 -c "import vosk; print('vosk OK')" 2>&1 | tail -1
python3 -c "import pyaudio; print('pyaudio OK')" 2>&1 | tail -1
python3 -c "import numpy; print('numpy', numpy.__version__)" 2>&1 | tail -1
echo ===AUDIO===
which aplay arecord espeak-ng 2>/dev/null
echo ===HOME===
ls ~ 2>/dev/null
echo ===DONE===
