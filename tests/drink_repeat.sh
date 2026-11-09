#!/usr/bin/env bash
# drink_repeat.sh <N> [tag] [cooldown_seconds]
#
# Sequentially repeat the complete DrinkTask E2E observer test against an
# already-running Gazebo/ROS 2 demo. Each round starts a fresh read-only
# observer, sends one GO_DRINK command, validates the complete task, and
# preserves its JSON result before the next round.
#
# Prerequisites (start once, outside this script):
#   bash tools/start_scene.sh
#   bash tools/start_drink_demo.sh
#
# Usage:
#   bash tests/drink_repeat.sh 3 p104
#
# P91_WARMUP and P91_FOLLOW_WATCH are forwarded to tests/test_drink.py.
# A failed round stops the sequence: do not start another task while the
# previous simulation may still be in an unsafe or incomplete state.

set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# The demo launch scripts source these overlays only in their own processes.
# This runner is a separate shell, so it must initialize ROS/Gazebo Python
# and shared-library paths before importing rclpy and vla_follow.
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy setup not found at /opt/ros/jazzy/setup.bash" >&2
    exit 2
fi
if [[ ! -f "$ROOT/vla_ws/install/setup.bash" ]]; then
    echo "Workspace overlay missing; build vla_ws before running DrinkTask E2E." >&2
    exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$ROOT/vla_ws/install/setup.bash"
mkdir -p "$ROOT/vla_ws/logs"

N="${1:-3}"
TAG="${2:-drink}"
COOLDOWN="${3:-10}"
OUTDIR="$ROOT/tests/out"

if ! [[ "$N" =~ ^[1-9][0-9]*$ ]]; then
    echo "N must be a positive integer" >&2
    exit 2
fi
if ! [[ "$COOLDOWN" =~ ^[0-9]+$ ]]; then
    echo "cooldown_seconds must be a non-negative integer" >&2
    exit 2
fi
if ! [[ "$TAG" =~ ^[A-Za-z0-9._-]+$ ]] || [[ "$TAG" == "." || "$TAG" == ".." ]]; then
    echo "tag may contain only letters, digits, dot, underscore and hyphen" >&2
    exit 2
fi

mkdir -p "$OUTDIR"
SUMMARY="$OUTDIR/drink_repeat_${TAG}.json"

# Keep prior E2E evidence immutable: reusing a tag must never overwrite
# archived round JSON or the previous aggregate report.
if [[ -e "$SUMMARY" || -L "$SUMMARY" ]]; then
    echo "Repeat summary already exists: $SUMMARY; choose a new tag." >&2
    exit 2
fi
for ((i=1; i<=N; i++)); do
    SNAPSHOT="$OUTDIR/drink_${TAG}_${i}.json"
    if [[ -e "$SNAPSHOT" || -L "$SNAPSHOT" ]]; then
        echo "Round evidence already exists: $SNAPSHOT; choose a new tag." >&2
        exit 2
    fi
done

RESULT="$ROOT/vla_ws/logs/test_drink.json"
SNAPSHOTS=()
FAILED=0

for ((i=1; i<=N; i++)); do
    echo
    echo "========== DrinkTask E2E round $i/$N ($TAG) =========="
    rm -f "$RESULT"

    set +e
    python3 tests/test_drink.py
    RC=$?
    set -e

    SNAPSHOT="$OUTDIR/drink_${TAG}_${i}.json"
    if [[ -s "$RESULT" ]]; then
        cp "$RESULT" "$SNAPSHOT"
        SNAPSHOTS+=("$SNAPSHOT")
    else
        echo "Round $i produced no result JSON (exit=$RC)." >&2
        FAILED=1
        break
    fi

    if (( RC != 0 )); then
        echo "Round $i failed (exit=$RC); stopping to avoid overlapping an incomplete task." >&2
        FAILED=1
        break
    fi

    if (( i < N && COOLDOWN > 0 )); then
        echo "Cooldown: ${COOLDOWN}s"
        sleep "$COOLDOWN"
    fi
done

python3 - "$SUMMARY" "$N" "${SNAPSHOTS[@]}" <<'PY'
import json
import os
import statistics
import sys

summary_path, requested, *paths = sys.argv[1:]
rows = []
for path in paths:
    try:
        with open(path, encoding="utf-8") as stream:
            result = json.load(stream)
    except (OSError, ValueError) as exc:
        print("Invalid round result %s: %s" % (path, exc), file=sys.stderr)
        continue

    follow = result.get("follow") or {}
    rows.append({
        "file": os.path.basename(path),
        "ok": bool(result.get("ok")),
        "elapsed_s": result.get("elapsed_s"),
        "sim_elapsed_s": result.get("sim_elapsed_s"),
        "cup_home_mm": result.get("cup_home_mm"),
        "cup_tilt_deg": result.get("cup_tilt_deg"),
        "follow_travel_m": follow.get("travel_m"),
        "follow_end_distance_m": follow.get("d_end_m"),
        "follow_in_band_pct": follow.get("in_band_pct"),
        "stages": result.get("stages", []),
        "fails": result.get("fails", []),
    })

def numeric_summary(key):
    values = [row[key] for row in rows if isinstance(row.get(key), (int, float))]
    if not values:
        return None
    return {
        "median": round(statistics.median(values), 3),
        "min": round(min(values), 3),
        "max": round(max(values), 3),
    }

report = {
    "requested_rounds": int(requested),
    "completed_rounds": len(rows),
    "passed_rounds": sum(row["ok"] for row in rows),
    "all_passed": len(rows) == int(requested) and all(row["ok"] for row in rows),
    "metrics": {
        key: numeric_summary(key) for key in (
            "elapsed_s", "sim_elapsed_s", "cup_home_mm", "cup_tilt_deg",
            "follow_travel_m", "follow_end_distance_m", "follow_in_band_pct"
        )
    },
    "rounds": rows,
}
os.makedirs(os.path.dirname(summary_path), exist_ok=True)
with open(summary_path, "w", encoding="utf-8") as stream:
    json.dump(report, stream, ensure_ascii=False, indent=2)
print(json.dumps(report, ensure_ascii=False, indent=2))
PY

echo "Repeat summary: $SUMMARY"
if (( FAILED != 0 )); then
    exit 1
fi
python3 - "$SUMMARY" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as stream:
    result = json.load(stream)
sys.exit(0 if result.get("all_passed") else 1)
PY
