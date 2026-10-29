#!/bin/bash
tgt="$1"
for p in $(pgrep -f "$tgt"); do
  [ "$p" = "$$" ] && continue
  cmd=$(tr "\0" " " < /proc/$p/cmdline 2>/dev/null)
  case "$cmd" in *"kill_nodes"*) continue;; *bash*) continue;; esac
  kill "$p" 2>/dev/null && echo "killed $p $cmd"
done
