#!/usr/bin/env bash
set -euo pipefail

# AURA Lab 2
# Localization / State Estimation / SLAM
#
# Student usage:
#   ./run_lab2.sh
#   ./run_lab2.sh ekf
#   ./run_lab2.sh particle_filter
#   ./run_lab2.sh ekf_slam
#   ./run_lab2.sh graph_slam
#
# This wrapper starts the normal Lab-1 offline environment, temporarily
# activates the Lab-2 RViz configuration, and launches StudentEstimator.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND="$ROOT/.aura_runtime/backend"
METHOD="${1:-dead_reckoning}"

OFFLINE="$BACKEND/simulation/run_offline.sh"
APPTAINER="$BACKEND/simulation/apptainer.sh"

BASE_RVIZ="$BACKEND/mrim_task/mrim_manager/config/rviz/offline.rviz"
LAB2_RVIZ="$BACKEND/mrim_task/mrim_manager/config/rviz/offline_lab2.rviz"

LAB2_LAUNCH="$BACKEND/mrim_task/mrim_planner/launch/aura_lab2.launch"
LAB2_NODE="$BACKEND/mrim_task/mrim_planner/scripts/aura_estimator_node.py"
STUDENT_ESTIMATOR="$BACKEND/mrim_task/mrim_planner/scripts/student_estimator.py"

for required in \
  "$OFFLINE" \
  "$APPTAINER" \
  "$BASE_RVIZ" \
  "$LAB2_RVIZ" \
  "$LAB2_LAUNCH" \
  "$LAB2_NODE" \
  "$STUDENT_ESTIMATOR"
do
  if [[ ! -f "$required" ]]; then
    echo "[AURA] Missing required Lab 2 file:"
    echo "       $required"
    exit 1
  fi
done

# Preserve the original Lab-1 RViz configuration.
RVIZ_BACKUP="$(mktemp /tmp/aura_offline_rviz.XXXXXX)"
cp "$BASE_RVIZ" "$RVIZ_BACKUP"

OFFLINE_PID=""

cleanup() {
  echo
  echo "[AURA] Cleaning up Lab 2..."

  # Always restore the untouched Lab-1 visualization.
  if [[ -f "$RVIZ_BACKUP" ]]; then
    cp "$RVIZ_BACKUP" "$BASE_RVIZ" 2>/dev/null || true
    rm -f "$RVIZ_BACKUP" || true
  fi

  if [[ -n "${OFFLINE_PID}" ]] && kill -0 "$OFFLINE_PID" 2>/dev/null; then
    kill "$OFFLINE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Activate the cumulative Lab-2 visualization before MrimManager starts RViz.
cp "$LAB2_RVIZ" "$BASE_RVIZ"

echo "[AURA] Lab 2 - Localization / State Estimation / SLAM"
echo "[AURA] Estimator: $METHOD"
echo "[AURA] Visualization: Lab 1 + Lab 2 overlays"
echo "[AURA] Starting offline environment..."
echo

bash "$OFFLINE" &
OFFLINE_PID=$!

# Join the same ROS master from a second container command and launch Lab 2.
bash "$APPTAINER" <<AURA_CONTAINER
echo "[AURA] Waiting for ROS master..."
until rostopic list >/dev/null 2>&1; do
  sleep 1
done

echo "[AURA] ROS master is ready."

# Offline playback normally exposes /visualization/odom_1.
# Real-time simulation exposes /multirotor_simulator/uav1/odom.
if rostopic list | grep -qx "/visualization/odom_1"; then
  TRUTH_TOPIC="/visualization/odom_1"
elif rostopic list | grep -qx "/multirotor_simulator/uav1/odom"; then
  TRUTH_TOPIC="/multirotor_simulator/uav1/odom"
else
  echo "[AURA] ERROR: no supported UAV1 odometry source found."
  rostopic list | grep -E "odom|ground_truth" || true
  exit 2
fi

echo "[AURA] Estimation source: \$TRUTH_TOPIC"
echo "[AURA] Launching StudentEstimator: $METHOD"

roslaunch mrim_planner aura_lab2.launch \
  method:="$METHOD" \
  truth_topic:="\$TRUTH_TOPIC"
AURA_CONTAINER
