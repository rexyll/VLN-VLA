#!/bin/bash
# tools/package.sh —— 把 VLA 项目打成可分发的压缩包
#
# 产物: dist/VLA-<日期>-<git短sha或直接 latest>.tar.gz
# 内容: 源码(vla_ws/src) + 运行时(vla_demo) + 文档(docs) + 测试(tests)
#       + 工具(tools) + 已编译产物(vla_ws/install)
# 排除: archive/(历史归档)、build/、log/、__pycache__、vosk 大模型
#
# 用法: bash tools/package.sh [标签]
set -u
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
cd "$VLA_HOME" || exit 1

TAG=${1:-latest}
STAMP=$(date +%Y%m%d)
DIST="$VLA_HOME/dist"
NAME="VLA-${STAMP}-${TAG}"
mkdir -p "$DIST"

WORK=$(mktemp -d)
mkdir -p "$WORK/$NAME"

# ---- 需要打包的内容 ----
cp -r vla_ws/src      "$WORK/$NAME/" 2>/dev/null
cp -r vla_ws/install  "$WORK/$NAME/" 2>/dev/null
cp -r vla_demo        "$WORK/$NAME/" 2>/dev/null
cp -r docs            "$WORK/$NAME/" 2>/dev/null
cp -r tests           "$WORK/$NAME/" 2>/dev/null
cp -r tools           "$WORK/$NAME/" 2>/dev/null
[ -f README.md ] && cp README.md "$WORK/$NAME/"

# ---- 清理体积大/无用的东西 ----
rm -rf "$WORK/$NAME"/vla_demo/vosk-model-small-cn-0.22
rm -rf "$WORK/$NAME"/vla_demo/snapshots
rm -rf "$WORK/$NAME"/vla_demo/maps
rm -rf "$WORK/$NAME"/vla_ws/install/*/lib/python3.12/site-packages/*/__pycache__
find "$WORK/$NAME" -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
find "$WORK/$NAME" -name '*.pyc' -delete 2>/dev/null
rm -rf "$WORK/$NAME"/tests/out
# 知识库附带示例代码里的训练权重(单个 13MB), 与交付无关
find "$WORK/$NAME"/docs -name '*.pt' -delete 2>/dev/null
find "$WORK/$NAME"/docs -name '*.pth' -delete 2>/dev/null
find "$WORK/$NAME"/docs -name '*.ckpt' -delete 2>/dev/null
find "$WORK/$NAME"/docs -type d -name runs -prune -exec rm -rf {} + 2>/dev/null

# ---- 版本戳 ----
cat > "$WORK/$NAME/VERSION.txt" <<EOF
VLA 人形跟随演示包
打包时间: $(date '+%Y-%m-%d %H:%M:%S')
标签:     $TAG
平台:     ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04)
内容:     vla_ws/(源码+已编译) vla_demo/(运行时) docs/ tests/ tools/
EOF

tar -czf "$DIST/$NAME.tar.gz" -C "$WORK" "$NAME"
rm -rf "$WORK"
ls -lh "$DIST/$NAME.tar.gz"
echo ">>> 打包完成: $DIST/$NAME.tar.gz"
