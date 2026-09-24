#!/usr/bin/env bash
set -euo pipefail

# Bootstrap the dVRK workspace Python environment used by dvrk_pybullet.
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "${SCRIPT_PATH}")"

find_workspace_root() {
    local cursor="${SCRIPT_DIR}"
    while [[ "${cursor}" != "/" ]]; do
        case "$(basename "${cursor}")" in
            src|install)
                dirname "${cursor}"
                return 0
                ;;
        esac
        cursor="$(dirname "${cursor}")"
    done
    return 1
}

workspace_root="$(find_workspace_root || true)"
if [[ -z "${workspace_root}" || ! -d "${workspace_root}" ]]; then
    echo "error: could not determine workspace root from ${SCRIPT_PATH}" >&2
    exit 2
fi
workspace_root="$(readlink -f "${workspace_root}")"

VENV_DIR="${workspace_root}/.venv"
REQUIREMENTS_FILE="${SCRIPT_DIR}/../requirements.txt"
if [[ ! -f "${REQUIREMENTS_FILE}" ]]; then
    REQUIREMENTS_FILE="${SCRIPT_DIR}/requirements.txt"
fi

if [[ ! -f "${REQUIREMENTS_FILE}" ]]; then
    echo "error: requirements.txt not found" >&2
    exit 2
fi
REQUIREMENTS_FILE="$(readlink -f "${REQUIREMENTS_FILE}")"
PYTHON="${PYTHON:-python3}"

echo "Bootstrapping PyBullet virtual environment at: ${VENV_DIR}"
echo "Requirements file: ${REQUIREMENTS_FILE}"

# Check for non-interactive flag (-y / --yes)
PROCEED=false
for arg in "$@"; do
    case "$arg" in
        -y|--yes)
            PROCEED=true
            ;;
    esac
done

if [ "${PROCEED}" = false ]; then
    if [ -t 0 ]; then
        read -r -p "Create venv and install using pip? [y/N] " response
        case "${response}" in
            [yY][eE][sS]|[yY])
                ;;
            *)
                echo "Operation cancelled."
                exit 0
                ;;
        esac
    fi
fi

if [[ -e "${VENV_DIR}" && ! -f "${VENV_DIR}/pyvenv.cfg" ]]; then
    echo "error: refusing to use an existing non-venv path: ${VENV_DIR}" >&2
    exit 2
fi

if [[ ! -f "${VENV_DIR}/pyvenv.cfg" ]]; then
    echo "Creating virtual environment at ${VENV_DIR} with system site packages..."
    "${PYTHON}" -m venv --system-site-packages "${VENV_DIR}"
else
    echo "Reusing existing virtual environment at ${VENV_DIR}."
fi

VENV_PYTHON="${VENV_DIR}/bin/python"
if [[ ! -x "${VENV_PYTHON}" ]]; then
    echo "error: venv Python is missing: ${VENV_PYTHON}" >&2
    exit 2
fi

echo "Installing dependencies from ${REQUIREMENTS_FILE}..."
"${VENV_PYTHON}" -m pip install --upgrade pip
"${VENV_PYTHON}" -m pip install -r "${REQUIREMENTS_FILE}"

echo
echo "PyBullet environment bootstrapped successfully."
echo "Test with: ${VENV_PYTHON} -c 'import pybullet; print(\"PyBullet ready!\")'"
