#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND="$ROOT/.aura_runtime/backend"
PLANNER_DIR="$BACKEND/mrim_task/mrim_planner"
SCRIPTS="$PLANNER_DIR/scripts"
CONFIG="$PLANNER_DIR/config"
OFFLINE="$BACKEND/simulation/run_offline.sh"

MODE="${1:-rrt}"
case "$MODE" in
  rrt|astar|student|rrtstar) ;;
  *)
    echo "[AURA LAB 1] Unknown mode: $MODE"
    echo "Valid modes: rrt astar student rrtstar"
    exit 2
    ;;
esac

ACTIVE_PLANNER="$SCRIPTS/planner.py"
LAB1_PLANNER="$SCRIPTS/planner_lab1_aura.py"
ACTIVE_CONFIG="$CONFIG/virtual.yaml"
LAB1_CONFIG="$CONFIG/virtual_lab1.yaml"

for f in "$ACTIVE_PLANNER" "$LAB1_PLANNER" "$ACTIVE_CONFIG" "$LAB1_CONFIG" "$OFFLINE"; do
  [[ -f "$f" ]] || { echo "[AURA LAB 1] Missing: $f"; exit 1; }
done

BACKUP_PLANNER="$(mktemp /tmp/aura_lab1_planner.XXXXXX.py)"
BACKUP_CONFIG="$(mktemp /tmp/aura_lab1_virtual.XXXXXX.yaml)"
TMP_CONFIG="$(mktemp /tmp/aura_lab1_selected.XXXXXX.yaml)"
cp "$ACTIVE_PLANNER" "$BACKUP_PLANNER"
cp "$ACTIVE_CONFIG" "$BACKUP_CONFIG"
cp "$LAB1_CONFIG" "$TMP_CONFIG"

cleanup() {
  local status=$?
  cp "$BACKUP_PLANNER" "$ACTIVE_PLANNER" 2>/dev/null || true
  cp "$BACKUP_CONFIG" "$ACTIVE_CONFIG" 2>/dev/null || true
  rm -f "$BACKUP_PLANNER" "$BACKUP_CONFIG" "$TMP_CONFIG" || true
  echo
  echo "[AURA LAB 1] Restored original planner.py and virtual.yaml"
  exit "$status"
}
trap cleanup EXIT INT TERM

python3 - "$TMP_CONFIG" "$MODE" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
mode = sys.argv[2]
lines = p.read_text(encoding="utf-8").splitlines()
inside = False
changed = False
for i, line in enumerate(lines):
    stripped = line.strip()
    if line.startswith("path_planner:"):
        inside = True
        continue
    if inside:
        if line and not line[0].isspace() and not stripped.startswith("#"):
            break
        if line.startswith("  method:"):
            lines[i] = "  method: {}".format(mode)
            changed = True
            break
if not changed:
    raise SystemExit("Could not find path_planner.method in {}".format(p))
p.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

cp "$LAB1_PLANNER" "$ACTIVE_PLANNER"
cp "$TMP_CONFIG" "$ACTIVE_CONFIG"

echo "=============================================================="
echo " AURA LAB 1 - MOTION PLANNING"
echo "=============================================================="
echo " Local planner : $MODE"
echo " Student logic : assignment + heading + temporal deconfliction"
echo " Config        : virtual_lab1.yaml (temporary activation)"
echo "=============================================================="
echo

cd "$BACKEND"
bash "$OFFLINE"
