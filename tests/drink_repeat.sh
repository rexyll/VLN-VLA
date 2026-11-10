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

# Never follow a pre-existing output-directory symlink outside the repository.
if [[ -L "$OUTDIR" ]]; then
    echo "DrinkTask output directory must not be a symlink: $OUTDIR" >&2
    exit 2
fi
mkdir -p "$OUTDIR"
TESTS_REAL="$(realpath -e "$ROOT/tests")"
OUTDIR_REAL="$(realpath -e "$OUTDIR")"
case "$OUTDIR_REAL/" in
    "$TESTS_REAL"/*) ;;
    *)
        echo "DrinkTask output directory escapes tests/: $OUTDIR_REAL" >&2
        exit 2
        ;;
esac

# All runs share one live Gazebo demo and fixed observer output paths. A
# process-wide lock prevents concurrent batches (even with different tags)
# from issuing overlapping GO_DRINK commands or overwriting live screenshots.
if ! command -v flock >/dev/null 2>&1; then
    echo "flock is required to serialize DrinkTask repeat runs." >&2
    exit 2
fi
LOCKFILE="$OUTDIR/.drink_repeat.lock"
if [[ -L "$LOCKFILE" ]]; then
    echo "DrinkTask lock file must not be a symlink: $LOCKFILE" >&2
    exit 2
fi
# Append mode avoids truncating an existing lock inode (including hardlinks).
exec 9>>"$LOCKFILE"
if ! flock -n 9; then
    echo "Another DrinkTask repeat run is active; refusing concurrent execution." >&2
    exit 2
fi

SUMMARY="$OUTDIR/drink_repeat_${TAG}.json"

# Keep prior E2E evidence immutable: reusing a tag must never overwrite
# archived round JSON or the previous aggregate report.
if [[ -e "$SUMMARY" || -L "$SUMMARY" ]]; then
    echo "Repeat summary already exists: $SUMMARY; choose a new tag." >&2
    exit 2
fi
for ((i=1; i<=N; i++)); do
    SNAPSHOT="$OUTDIR/drink_${TAG}_${i}.json"
    SHOTDIR="$OUTDIR/drink_${TAG}_${i}_shots"
    if [[ -e "$SNAPSHOT" || -L "$SNAPSHOT" ]]; then
        echo "Round evidence already exists: $SNAPSHOT; choose a new tag." >&2
        exit 2
    fi
    if [[ -e "$SHOTDIR" || -L "$SHOTDIR" ]]; then
        echo "Round screenshot archive already exists: $SHOTDIR; choose a new tag." >&2
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
        # Archive each round's screenshots alongside its JSON. The observer
        # writes fixed filenames under vla_ws/logs, so leaving paths untouched
        # would make older JSON point at images overwritten by later rounds.
        if ! python3 - "$RESULT" "$SNAPSHOT" "$ROOT" "$OUTDIR/drink_${TAG}_${i}_shots" "$RC" <<'PY'
import json
import os
import shutil
import sys
import tempfile

source_json, snapshot_json, root, shot_dir, observer_exit_code = sys.argv[1:]
sys.path.insert(0, os.path.join(root, "tests"))
from drink_evidence import (archive_observer_screenshots,
                            validate_observer_result, write_json_atomic,
                            load_json_strict)

archive_published = False
try:
    result = load_json_strict(source_json)
    # Reject scalar/array/null JSON and malformed screenshot fields before
    # mutating the observer payload or creating an archive directory.
    validate_observer_result(result)
    # Persist the process outcome with the archived result so per-round
    # status and aggregate passed_rounds cannot disagree.
    result["observer_exit_code"] = int(observer_exit_code)
    archive_observer_screenshots(result, root, shot_dir)
    archive_published = True

    # Publish the JSON only after the complete screenshot archive exists.
    # The shared helper also closes the raw fd if fdopen fails.
    write_json_atomic(result, snapshot_json)
except (OSError, ValueError) as exc:
    if archive_published:
        shutil.rmtree(shot_dir, ignore_errors=True)
    print("DrinkTask evidence archival failed: %s" % exc, file=sys.stderr)
    sys.exit(3)
PY
        then
            echo "Round $i evidence archival failed (test exit=$RC)." >&2
            FAILED=1
            break
        fi
        SNAPSHOTS+=("$SNAPSHOT")

        # Do not start another round merely because the observer process exited
        # zero: the persisted task result and archived evidence are authoritative.
        if ! python3 - "$SNAPSHOT" "$ROOT" <<'PY'
import os
import sys
root = sys.argv[2]
sys.path.insert(0, os.path.join(root, "tests"))
from drink_evidence import load_json_strict, summarize_round_evidence
result = load_json_strict(sys.argv[1])
summary = summarize_round_evidence(result, root)
if not summary["ok"]:
    print("Round result failed task/evidence acceptance; stopping repeat batch.",
          file=sys.stderr)
    sys.exit(1)
PY
        then
            echo "Round $i failed acceptance; stopping before the next task." >&2
            FAILED=1
            break
        fi
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

python3 - "$SUMMARY" "$N" "$ROOT" "$FAILED" "$TAG" "${SNAPSHOTS[@]}" <<'PY'
import json
import os
import sys

summary_path, requested, root, failed, tag, *paths = sys.argv[1:]
sys.path.insert(0, os.path.join(root, "tests"))
from drink_evidence import (summarize_round_evidence, summarize_numeric_values,
                            summarize_batch_status,
                            finite_numeric_value, write_json_atomic,
                            load_json_strict)

rows = []
invalid_rounds = []
for path in paths:
    try:
        result = load_json_strict(path)
    except (OSError, ValueError) as exc:
        message = str(exc)
        print("Invalid round result %s: %s" % (path, message), file=sys.stderr)
        invalid_rounds.append({"file": os.path.basename(path), "error": message})
        continue

    if not isinstance(result, dict):
        message = "expected a JSON object"
        print("Invalid round result %s: %s" % (path, message), file=sys.stderr)
        invalid_rounds.append({"file": os.path.basename(path), "error": message})
        continue

    follow = result.get("follow")
    if not isinstance(follow, dict):
        follow = {}
    evidence = summarize_round_evidence(result, root)
    rows.append({
        "file": os.path.basename(path),
        "ok": evidence["ok"],
        "task_ok": evidence["task_ok"],
        "process_ok": evidence["process_ok"],
        "observer_exit_code": evidence["observer_exit_code"],
        "archive_ok": evidence["archive_ok"],
        "screenshots_archived": evidence["screenshots_archived"],
        "shots": evidence["shots"],
        "elapsed_s": finite_numeric_value(result.get("elapsed_s")),
        "sim_elapsed_s": finite_numeric_value(result.get("sim_elapsed_s")),
        "cup_home_mm": finite_numeric_value(result.get("cup_home_mm")),
        "cup_tilt_deg": finite_numeric_value(result.get("cup_tilt_deg")),
        "follow_travel_m": finite_numeric_value(follow.get("travel_m")),
        "follow_end_distance_m": finite_numeric_value(follow.get("d_end_m")),
        "follow_in_band_pct": finite_numeric_value(follow.get("in_band_pct")),
        "stages": result.get("stages", []),
        "shot_labels": result.get("shot_labels", []),
        "missing_shot_labels": evidence["missing_shot_labels"],
        "fails": result.get("fails", []),
    })

def numeric_summary(key):
    return summarize_numeric_values(row.get(key) for row in rows)

represented = {row["file"] for row in rows}
represented.update(item["file"] for item in invalid_rounds)
missing_rounds = [
    {"round": index, "file": "drink_%s_%d.json" % (tag, index)}
    for index in range(1, int(requested) + 1)
    if "drink_%s_%d.json" % (tag, index) not in represented
]
batch_status = summarize_batch_status(
    int(requested), rows, invalid_rounds, missing_rounds)
report = {
    **batch_status,
    "runner_failed": int(failed) != 0,
    "all_passed": batch_status["all_passed"] and int(failed) == 0,
    "metrics": {
        key: numeric_summary(key) for key in (
            "elapsed_s", "sim_elapsed_s", "cup_home_mm", "cup_tilt_deg",
            "follow_travel_m", "follow_end_distance_m", "follow_in_band_pct"
        )
    },
    "rounds": rows,
}
os.makedirs(os.path.dirname(summary_path), exist_ok=True)
write_json_atomic(report, summary_path)
print(json.dumps(report, ensure_ascii=False, indent=2))
PY

echo "Repeat summary: $SUMMARY"
if (( FAILED != 0 )); then
    exit 1
fi
python3 - "$SUMMARY" "$ROOT" <<'PY'
import os
import sys
summary_path, root = sys.argv[1:]
sys.path.insert(0, os.path.join(root, "tests"))
from drink_evidence import load_json_strict
result = load_json_strict(summary_path)
sys.exit(0 if result.get("all_passed") else 1)
PY
