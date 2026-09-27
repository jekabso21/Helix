#!/usr/bin/env bash
# Builds the Godot frame publisher extension into app/bin/. Separate from the CMake presets
# because godot-cpp is a large dependency that rarely changes.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
godot_cpp="${repo_root}/third_party/godot-cpp"
build_dir="${repo_root}/build/extension"
api_file="${build_dir}/extension_api.json"
godot_bin="${GODOT:-godot}"

if [[ ! -e "${godot_cpp}/CMakeLists.txt" ]]; then
    echo "error: ${godot_cpp} is empty. Run: git submodule update --init third_party/godot-cpp" >&2
    exit 1
fi

mkdir -p "${build_dir}"
# Target the exact engine build in use, not godot-cpp's bundled snapshot
(cd "${build_dir}" && "${godot_bin}" --headless --dump-extension-api >/dev/null 2>&1)
if [[ ! -s "${api_file}" ]]; then
    echo "error: '${godot_bin} --headless --dump-extension-api' produced no extension_api.json" >&2
    exit 1
fi

cmake -S "${repo_root}/app/extension" -B "${build_dir}/cmake" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DFPVSIM_GODOT_CPP_DIR="${godot_cpp}" \
    -DGODOTCPP_CUSTOM_API_FILE="${api_file}" \
    -DGODOTCPP_USE_STATIC_CPP=OFF   # Fedora does not ship libstdc++-static by default
cmake --build "${build_dir}/cmake" -j"$(nproc)"

mkdir -p "${repo_root}/app/bin"
echo "built $(ls -1 "${repo_root}"/app/bin/libfpvsim_extension.so)"
