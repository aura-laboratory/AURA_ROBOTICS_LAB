#!/usr/bin/env bash
set -Eeuo pipefail

# ================================================================
# AURA Robotics Lab - Full Backend Installer
# ================================================================
#
# Installs the upstream CTU MRS Summer School backend into:
#
#   ~/Downloads/aura-lab/.aura_runtime/backend
#
# The script:
#   - checks the host environment,
#   - creates the required AURA folders,
#   - clones the full upstream backend,
#   - checks out the course-tested backend commit,
#   - runs the upstream install.sh script.
#
# Run from the AURA_ROBOTICS_LAB repository:
#
#   chmod +x install_aura_backend.sh
#   ./install_aura_backend.sh
#
# ================================================================

BACKEND_URL="https://github.com/ctu-mrs/summer-school-2026.git"
BACKEND_COMMIT="cff98a147c64316d5e3d30f61f954821c42685cf"

AURA_ROOT="${HOME}/Downloads/aura-lab"
RUNTIME_DIR="${AURA_ROOT}/.aura_runtime"
BACKEND_DIR="${RUNTIME_DIR}/backend"

echo "=============================================================="
echo " AURA Robotics Lab - Backend Installation"
echo "=============================================================="
echo

# ------------------------------------------------
# 1. Linux check
# ------------------------------------------------

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "[AURA] ERROR:"
  echo "This installer must be executed inside a Linux environment."
  echo
  echo "Supported approaches:"
  echo "  - Linux"
  echo "  - Windows 11 + WSL2 + Linux"
  echo "  - Windows + Linux VM"
  echo "  - macOS + Linux VM"
  exit 1
fi

echo "[AURA] Linux environment detected."

# ------------------------------------------------
# 2. Required commands
# ------------------------------------------------

if ! command -v git >/dev/null 2>&1; then
  echo "[AURA] ERROR: git is not installed."
  echo "Install Git first and run this script again."
  exit 1
fi

echo "[AURA] Git detected: $(git --version)"

# ------------------------------------------------
# 3. OS information / Apptainer note
# ------------------------------------------------

OS_ID="unknown"
OS_LIKE=""

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  OS_ID="${ID:-unknown}"
  OS_LIKE="${ID_LIKE:-}"
fi

echo "[AURA] Linux distribution: ${OS_ID}"

if [[ "${OS_ID}" != "ubuntu" && "${OS_ID}" != "debian" && "${OS_LIKE}" != *"ubuntu"* && "${OS_LIKE}" != *"debian"* ]]; then
  echo
  echo "[AURA] NOTE:"
  echo "The upstream project requires Apptainer to be installed manually"
  echo "on non-Ubuntu-compatible Linux distributions."
  echo
  if ! command -v apptainer >/dev/null 2>&1 && ! command -v singularity >/dev/null 2>&1; then
    echo "[AURA] ERROR: Apptainer/Singularity was not found."
    echo "Install Apptainer first, then run this script again."
    exit 1
  fi
fi

# ------------------------------------------------
# 4. Disk space information
# ------------------------------------------------

echo
echo "[AURA] The upstream backend requires approximately 6 GB of disk space."
echo "[AURA] Installation target:"
echo "       ${BACKEND_DIR}"
echo

# ------------------------------------------------
# 5. Create target folders
# ------------------------------------------------

mkdir -p "${RUNTIME_DIR}"

# ------------------------------------------------
# 6. Clone or validate backend
# ------------------------------------------------

if [[ -d "${BACKEND_DIR}/.git" ]]; then
  echo "[AURA] Existing backend repository found."
  echo "[AURA] Reusing:"
  echo "       ${BACKEND_DIR}"
  echo

  cd "${BACKEND_DIR}"

  ORIGIN_URL="$(git remote get-url origin 2>/dev/null || true)"

  if [[ -n "${ORIGIN_URL}" ]]; then
    echo "[AURA] Existing origin: ${ORIGIN_URL}"
  fi

  echo "[AURA] Fetching backend revisions..."
  git fetch --all --tags
else
  if [[ -e "${BACKEND_DIR}" ]]; then
    echo "[AURA] ERROR:"
    echo "The backend path already exists but is not a Git repository:"
    echo "  ${BACKEND_DIR}"
    echo
    echo "Move or remove that directory before running this installer."
    exit 1
  fi

  echo "[AURA] Cloning full backend..."
  git clone "${BACKEND_URL}" "${BACKEND_DIR}"
  cd "${BACKEND_DIR}"
fi

# ------------------------------------------------
# 7. Checkout tested course version
# ------------------------------------------------

echo
echo "[AURA] Checking out course backend version:"
echo "       ${BACKEND_COMMIT}"

git checkout "${BACKEND_COMMIT}"

CURRENT_COMMIT="$(git rev-parse HEAD)"

if [[ "${CURRENT_COMMIT}" != "${BACKEND_COMMIT}" ]]; then
  echo "[AURA] ERROR: backend commit verification failed."
  exit 1
fi

echo "[AURA] Backend version verified."

# ------------------------------------------------
# 8. Run upstream installer
# ------------------------------------------------

if [[ ! -f "${BACKEND_DIR}/install.sh" ]]; then
  echo "[AURA] ERROR: upstream install.sh was not found."
  exit 1
fi

chmod +x "${BACKEND_DIR}/install.sh"

echo
echo "=============================================================="
echo " Starting upstream backend installer"
echo "=============================================================="
echo

./install.sh

echo
echo "=============================================================="
echo " AURA backend installation completed"
echo "=============================================================="
echo
echo "Backend:"
echo "  ${BACKEND_DIR}"
echo
echo "Next:"
echo "  Open the README of Lab 1, Lab 2, or Lab 3"
echo "  and run that lab's installation script."
echo
echo "PyCharm:"
echo "  cd ${BACKEND_DIR}"
echo "  ./simulation/pycharm.sh"
echo
