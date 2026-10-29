#!/bin/bash
# vmsync_run.sh —— 两端代码/文档同步的一键入口
#
#   用法:  ./vmsync_run.sh status   # 查看开关状态(不连VM)
#         ./vmsync_run.sh enable   # 打开同步工具
#         ./vmsync_run.sh disable  # 关闭同步工具 (默认状态)
#         ./vmsync_run.sh scan     # 只看差异
#         ./vmsync_run.sh sync     # 执行双向同步
#         ./vmsync_run.sh verify   # 全量 md5 校验
#
# 本地根: F:/workspace/_coding_workspace/VLA
# 远端根: /home/earl/workspace/VLA  (VM 192.168.255.128, user earl)
#
# 注意: 同步工具默认【关闭】, scan/sync/verify 会被闸门拒绝。
#       长期使用请先 ./vmsync_run.sh enable; 用完建议再 disable。
set -e
PY="C:/Users/Administrator/.workbuddy/binaries/python/envs/default/Scripts/python.exe"
HERE="$(cd "$(dirname "$0")" && pwd)"
MODE="${1:-status}"
cd "$HERE"
case "$MODE" in
  status)  "$PY" vmsync.py status ;;
  enable)  "$PY" vmsync.py enable ;;
  disable) "$PY" vmsync.py disable ;;
  scan)    "$PY" vmsync.py scan ;;
  sync)    "$PY" vmsync.py sync ;;
  verify)  "$PY" vmsync.py verify ;;
  *) echo "用法: $0 [status|enable|disable|scan|sync|verify]"; exit 1 ;;
esac
