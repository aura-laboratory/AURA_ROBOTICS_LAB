#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND="$ROOT/.aura_runtime/backend"
PLANNER_DIR="$BACKEND/mrim_task/mrim_planner"
SCRIPTS="$PLANNER_DIR/scripts"
CONFIG="$PLANNER_DIR/config"
OFFLINE="$BACKEND/simulation/run_offline.sh"

METHOD="${1:-greedy}"
case "$METHOD" in
  tsp|random|greedy|q_learning|dqn) ;;
  *)
    echo "[AURA LAB 3] Unknown method: $METHOD"
    echo "Valid methods: tsp random greedy q_learning dqn"
    exit 2
    ;;
esac

ACTIVE_PLANNER="$SCRIPTS/planner.py"
LAB3_PLANNER="$SCRIPTS/planner_lab3_aura.py"
ACTIVE_CONFIG="$CONFIG/virtual.yaml"
LAB3_CONFIG="$CONFIG/virtual_lab3.yaml"
STUDENT_AGENT="$SCRIPTS/student_agent.py"

for f in "$ACTIVE_PLANNER" "$LAB3_PLANNER" "$ACTIVE_CONFIG" "$LAB3_CONFIG" "$STUDENT_AGENT" "$OFFLINE"; do
  [[ -f "$f" ]] || { echo "[AURA LAB 3] Missing: $f"; exit 1; }
done

BACKUP_PLANNER="$(mktemp /tmp/aura_lab3_planner.XXXXXX.py)"
BACKUP_CONFIG="$(mktemp /tmp/aura_lab3_virtual.XXXXXX.yaml)"
TMP_CONFIG="$(mktemp /tmp/aura_lab3_selected.XXXXXX.yaml)"
cp "$ACTIVE_PLANNER" "$BACKUP_PLANNER"
cp "$ACTIVE_CONFIG" "$BACKUP_CONFIG"
cp "$LAB3_CONFIG" "$TMP_CONFIG"

cleanup() {
  local status=$?
  cp "$BACKUP_PLANNER" "$ACTIVE_PLANNER" 2>/dev/null || true
  cp "$BACKUP_CONFIG" "$ACTIVE_CONFIG" 2>/dev/null || true
  rm -f "$BACKUP_PLANNER" "$BACKUP_CONFIG" "$TMP_CONFIG" || true
  echo
  echo "[AURA LAB 3] Restored original planner.py and virtual.yaml"
  exit "$status"
}
trap cleanup EXIT INT TERM

python3 - "$TMP_CONFIG" "$METHOD" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
method = sys.argv[2]
lines = p.read_text(encoding="utf-8").splitlines()
inside = False
changed = False
for i, line in enumerate(lines):
    stripped = line.strip()
    if line.startswith("decision_making:"):
        inside = True
        continue
    if inside:
        if line and not line[0].isspace() and not stripped.startswith("#"):
            break
        if line.startswith("  method:"):
            lines[i] = "  method: {}".format(method)
            changed = True
            break
if not changed:
    raise SystemExit("Could not find decision_making.method in {}".format(p))
p.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

cp "$LAB3_PLANNER" "$ACTIVE_PLANNER"
cp "$TMP_CONFIG" "$ACTIVE_CONFIG"

echo "=============================================================="
echo " AURA LAB 3 - DECISION MAKING / RL"
echo "=============================================================="
echo " Decision method : $METHOD"
echo " Config          : virtual_lab3.yaml (temporary activation)"
echo "=============================================================="
echo

cd "$BACKEND"
bash "$OFFLINE"
