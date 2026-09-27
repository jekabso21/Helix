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

# Video outputs need pieces Fedora does not ship: x264enc lives in RPM Fusion free, and
# v4l2loopback is an out-of-tree kernel module. Opt in, because both add a third-party repo
# or build a module against the running kernel.
if [[ "${1:-}" == "--video-outputs" ]]; then
    release="$(rpm -E %fedora)"
    sudo dnf install -y \
        "https://mirrors.rpmfusion.org/free/fedora/rpmfusion-free-release-${release}.noarch.rpm"
    sudo dnf install -y gstreamer1-plugins-ugly akmod-v4l2loopback \
        "kernel-devel-$(uname -r)" gstreamer1-plugins-bad-free
    sudo akmods --force
    # one loopback device the camera outputs can write to
    echo "options v4l2loopback devices=1 video_nr=10 card_label=fpvsim exclusive_caps=1" |
        sudo tee /etc/modprobe.d/fpvsim-v4l2loopback.conf >/dev/null
    echo "v4l2loopback" | sudo tee /etc/modules-load.d/fpvsim-v4l2loopback.conf >/dev/null
    sudo modprobe -r v4l2loopback 2>/dev/null || true
    sudo modprobe v4l2loopback
    echo "v4l2loopback ready: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
fi

# Fedora's uv does not download interpreters on demand
uv python install 3.12

echo
echo "Godot installed: $(godot --version 2>/dev/null || echo unknown)"
echo "The app pins its version in app/GODOT_VERSION; they must match."
