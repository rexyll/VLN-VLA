#!/usr/bin/env bash
set -u
PID_DIR="${XDG_RUNTIME_DIR:-/tmp}/vla-gazebo-viewer"
for name in websockify x11vnc; do
  file="$PID_DIR/$name.pid"
  if [[ -f "$file" ]]; then
    pid=$(cat "$file")
    kill "$pid" 2>/dev/null || true
    rm -f "$file"
  fi
done
echo "Gazebo web viewer stopped. Gazebo itself was not stopped."
