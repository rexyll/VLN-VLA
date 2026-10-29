#!/bin/bash
# tmp_m1_fixbin.sh —— P6 修复: 重推模型与音频后, 校验并恢复
set -e
cd ~
echo "=== 恢复 Vosk 模型 ==="
rm -rf vla_demo/vosk-model-small-cn-0.22 vosk-model.zip
# 模型 zip 由本地另行推送为 ~/vosk-model.zip
unzip -q -o vosk-model.zip
mv vosk-model-small-cn-0.22 vla_demo/
rm vosk-model.zip
md5sum vla_demo/vosk-model-small-cn-0.22/ivector/final.ie \
       vla_demo/vosk-model-small-cn-0.22/am/final.mdl
echo "=== 恢复音频(重编译会从 src 复制) ==="
md5sum vla_ws/src/vla_follow/audio/follow_2m.wav
echo "=== 仅文本类型 sed(G8 合规) ==="
find vla_ws/src \( -name '*.py' -o -name '*.sh' -o -name '*.xml' \
  -o -name '*.sdf' -o -name '*.yml' -o -name '*.cfg' \) -exec sed -i 's/\r$//' {} +
chmod +x vla_demo/*.sh
echo "=== 重建 ==="
source /opt/ros/jazzy/setup.bash
cd vla_ws && rm -rf build install && colcon build 2>&1 | tail -1
echo FIX-DONE
