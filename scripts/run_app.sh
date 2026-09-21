#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
app_dir="${repo_root}/app"
godot_bin="${GODOT:-godot}"

if ! command -v "${godot_bin}" >/dev/null; then
    echo "error: '${godot_bin}' not found. Run ./scripts/bootstrap_fedora.sh or set GODOT." >&2
    exit 1
fi

pinned="$(cat "${app_dir}/GODOT_VERSION")"
installed="$("${godot_bin}" --version)"
if [[ "${installed}" != "${pinned%%-*}"* ]]; then
    echo "warning: Godot ${installed} does not match the pinned ${pinned}" >&2
fi

# A fresh checkout must be imported once before the project can run
if [[ ! -d "${app_dir}/.godot" ]]; then
    "${godot_bin}" --headless --path "${app_dir}" --import >/dev/null 2>&1 || true
fi

exec "${godot_bin}" --path "${app_dir}" "$@"
