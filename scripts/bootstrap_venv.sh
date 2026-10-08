#!/usr/bin/env bash
set -euo pipefail

# Bootstrap the dVRK workspace Python environment used by dvrk_pybullet.
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

BASE_BOOTSTRAP="${SCRIPT_DIR}/../../dvrk_simulator_base/scripts/bootstrap_venv.sh"
if [[ ! -f "${BASE_BOOTSTRAP}" ]]; then
    BASE_PREFIX="$(ros2 pkg prefix dvrk_simulator_base 2>/dev/null || true)"
    if [[ -n "${BASE_PREFIX}" && -f "${BASE_PREFIX}/share/dvrk_simulator_base/scripts/bootstrap_venv.sh" ]]; then
        BASE_BOOTSTRAP="${BASE_PREFIX}/share/dvrk_simulator_base/scripts/bootstrap_venv.sh"
    fi
fi

if [[ ! -f "${BASE_BOOTSTRAP}" ]]; then
    echo "error: bootstrap_venv.sh from dvrk_simulator_base not found" >&2
    exit 2
fi

# PyBullet's isolated source build cannot see NumPy headers. Pip's negatively
# named setting uses 0 to disable isolation; install build dependencies first.
DVRK_BOOTSTRAP_BUILD_REQUIREMENTS="${SCRIPT_DIR}/../requirements-build.txt" \
PIP_NO_BUILD_ISOLATION=0 "${BASE_BOOTSTRAP}" \
    ".venv-pybullet" \
    "${SCRIPT_DIR}/../requirements.txt" \
    "PyBullet" \
    "-c 'import pybullet; print(\"PyBullet ready!\")'" \
    "$@"

# Locate the same workspace environment as the shared bootstrap, including
# installed scripts. Do not rely on the caller's working directory or Python.
WORKSPACE_CURSOR="${SCRIPT_DIR}"
while [[ "$(basename "${WORKSPACE_CURSOR}")" != "src" && "$(basename "${WORKSPACE_CURSOR}")" != "install" ]]; do
    if [[ "${WORKSPACE_CURSOR}" == "/" ]]; then
        echo "error: could not locate the PyBullet workspace" >&2
        exit 2
    fi
    WORKSPACE_CURSOR="$(dirname "${WORKSPACE_CURSOR}")"
done
VENV_PYTHON="$(dirname "${WORKSPACE_CURSOR}")/.venv-pybullet/bin/python"
if ! "${VENV_PYTHON}" -c 'import pybullet, sys; sys.exit(not pybullet.isNumpyEnabled())'; then
    echo "Rebuilding PyBullet with NumPy image support for efficient stereo rendering..."
    PYBULLET_VERSION="$("${VENV_PYTHON}" -c 'from importlib.metadata import version; print(version("pybullet"))')"
    "${VENV_PYTHON}" -m pip install --force-reinstall --no-deps \
        --no-build-isolation --no-binary=pybullet --no-cache-dir "pybullet==${PYBULLET_VERSION}"
fi
"${VENV_PYTHON}" -c 'import pybullet; assert pybullet.isNumpyEnabled(), "PyBullet lacks NumPy image support"; print("PyBullet NumPy image support enabled")'
