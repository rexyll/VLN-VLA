#!/usr/bin/env bash
echo ===GZ-PY===
python3 -c "import gz.transport13; print('gz.transport13 OK')" 2>&1 | tail -1
python3 -c "import gz.math7; print('gz.math7 OK')" 2>&1 | tail -1
python3 -c "import gz.msgs10.twist_pb2; print('gz.msgs10 twist OK')" 2>&1 | tail -1
python3 -c "import gz.msgs10.odometry_pb2; print('gz.msgs10 odometry OK')" 2>&1 | tail -1
python3 -c "import gz.msgs10.laserscan_pb2; print('gz.msgs10 laserscan OK')" 2>&1 | tail -1
echo ===PREV-GZ-USE===
ls ~/code/test 2>/dev/null | head -20
grep -rl "gz sim\|gzserver" ~/code/test/*.sh 2>/dev/null | head -5
echo $GZ_SIM_RESOURCE_PATH
ls /usr/share/gz/sim* 2>/dev/null | head
echo ===TOOLS===
which ffmpeg espeak-ng wget 2>/dev/null
aplay -l 2>&1 | head -6
echo ===SESSION===
loginctl show-session 3 -p Type -p Display -p State 2>/dev/null
echo ===DONE===
