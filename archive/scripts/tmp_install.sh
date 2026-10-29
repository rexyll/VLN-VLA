#!/bin/bash
# tmp_install.sh —— 安装 vosk + 下载中文小模型
set -e
mkdir -p ~/vla_demo/logs ~/vla_demo/audio
pip3 install --break-system-packages -q vosk 2>&1 | tail -2
cd ~/vla_demo
if [ ! -d vosk-model-small-cn-0.22 ]; then
  wget -q https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip -O vosk-model.zip
  unzip -q -o vosk-model.zip && rm vosk-model.zip
fi
python3 -c "from vosk import Model; m=Model('$HOME/vla_demo/vosk-model-small-cn-0.22'); print('VOSK-MODEL-OK')"
echo INSTALL-DONE
