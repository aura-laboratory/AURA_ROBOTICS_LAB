#!/usr/bin/env bash
set -Eeuo pipefail

LAB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AURA_ROOT="${1:-$HOME/Downloads/aura-lab}"
BACKEND="$AURA_ROOT/.aura_runtime/backend"
PATCH="$LAB_DIR/backend"
ROOT_FILES="$LAB_DIR/root"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$AURA_ROOT/.aura_course_backups/lab3_decision_making_$STAMP"

[[ -d "$BACKEND/mrim_task/mrim_planner" ]] || {
  echo "[ERROR] Backend not found at: $BACKEND"
  echo "See the repository README_INSTALLATION.md first."
  exit 1
}

mkdir -p "$BACKUP"

backup_target() {
  local target="$1"
  if [[ -f "$target" ]]; then
    local rel="${target#$AURA_ROOT/}"
    mkdir -p "$BACKUP/$(dirname "$rel")"
    cp -a "$target" "$BACKUP/$rel"
  fi
}

echo "[AURA] Installing lab3_decision_making into: $AURA_ROOT"

while IFS= read -r -d '' src; do
  rel="${src#$PATCH/}"
  dst="$BACKEND/$rel"
  backup_target "$dst"
  mkdir -p "$(dirname "$dst")"
  cp -a "$src" "$dst"
done < <(find "$PATCH" -type f -print0)

while IFS= read -r -d '' src; do
  rel="${src#$ROOT_FILES/}"
  dst="$AURA_ROOT/$rel"
  backup_target "$dst"
  mkdir -p "$(dirname "$dst")"
  cp -a "$src" "$dst"
  [[ "$dst" == *.sh ]] && chmod +x "$dst"
done < <(find "$ROOT_FILES" -type f -print0)

# Python scripts launched by ROS should be executable.
find "$BACKEND/mrim_task/mrim_planner/scripts" -maxdepth 1 -type f   \( -name 'aura_estimator_node.py' -o -name 'planner_lab*_aura.py' \)   -exec chmod +x {} + 2>/dev/null || true

echo

echo "[AURA] lab3_decision_making installed."
echo "[AURA] Backup of overwritten files: $BACKUP"
echo "[AURA] Read: $LAB_DIR/README.md"
