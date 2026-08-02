#!/usr/bin/env sh

set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
venv_dir="$script_dir/.venv"

if ! command -v python3 >/dev/null 2>&1; then
    echo "error: python3 was not found" >&2
    exit 1
fi

echo "Creating the somg Python environment..."
python3 -m venv --prompt somg "$venv_dir"

echo "Installing spotctl and its dependencies..."
"$venv_dir/bin/python" -m pip install -e "$script_dir"

echo
echo "Setup complete. Activate the environment with:"
echo "  source $venv_dir/bin/activate"
echo
echo "Then check the URT-2 connection with:"
echo "  spotctl ports"
echo "  spotctl --help"
