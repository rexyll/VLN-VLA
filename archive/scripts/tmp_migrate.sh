#!/bin/bash
# tmp_migrate.sh —— 项目整体迁移到 /home/earl/workspace/VLA (Z:\VLA)
set -e
SRC_WS="$HOME/vla_ws"
SRC_DEMO="$HOME/vla_demo"
SRC_MODEL="$HOME/vla_demo/vosk-model-small-cn-0.22"
DST="/home/earl/workspace/VLA"

mkdir -p "$DST"
echo "=== 1. 迁移源码包(带 .git 不存在, 直接拷贝) ==="
rm -rf "$DST/vla_ws"
mkdir -p "$DST/vla_ws"
cp -r "$SRC_WS/src" "$DST/vla_ws/src"
echo "=== 2. 迁移 demo 运行资产(音频/快照/配置/脚本, 不含模型) ==="
rm -rf "$DST/vla_demo"
mkdir -p "$DST/vla_demo"
cp -r "$SRC_DEMO/audio" "$DST/vla_demo/audio" 2>/dev/null || mkdir -p "$DST/vla_demo/audio"
cp -r "$SRC_DEMO/snapshots" "$DST/vla_demo/snapshots" 2>/dev/null || true
cp -f "$SRC_DEMO"/*.sh "$DST/vla_demo/" 2>/dev/null || true
[ -f "$SRC_DEMO/vision_config.json" ] && cp -f "$SRC_DEMO/vision_config.json" "$DST/vla_demo/"
echo "=== 3. 迁移 Vosk 模型 ==="
rm -rf "$DST/vla_demo/vosk-model-small-cn-0.22"
mv "$SRC_MODEL" "$DST/vla_demo/vosk-model-small-cn-0.22"
echo "=== 4. 迁移日志 ==="
mkdir -p "$DST/logs"
cp -f "$SRC_WS/logs/"*.log "$DST/logs/" 2>/dev/null || true
echo "=== 5. 权限与校验 ==="
chown -R earl:earl "$DST"
md5sum "$DST/vla_demo/vosk-model-small-cn-0.22/ivector/final.ie" | awk '{print "模型md5:", $1}'
ls "$DST/vla_ws/src/vla_follow" | head
ls "$DST/vla_demo" | head
echo MIGRATE-DONE
