#!/usr/bin/env bash
set -euo pipefail

packages=(
    gcc gcc-c++ clang clang-tools-extra cmake ninja-build make ccache git openssl
    libasan libubsan
    sdl2-compat-devel
    gstreamer1-devel gstreamer1-plugins-base-devel
    gstreamer1-plugins-good gstreamer1-plugins-ugly-free
    uv ruff pre-commit
    godot
    python3-websockify
)

sudo dnf install -y "${packages[@]}"

# Fedora's uv does not download interpreters on demand
uv python install 3.12

echo
echo "Godot installed: $(godot --version 2>/dev/null || echo unknown)"
echo "The app pins its version in app/GODOT_VERSION; they must match."
