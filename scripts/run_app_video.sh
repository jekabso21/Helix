#!/usr/bin/env bash
# Starts the app with the raw video output on and keeps a GStreamer window attached to it
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
port="${FPVSIM_RAW_VIDEO_PORT:-5700}"
width="${FPVSIM_RAW_VIDEO_WIDTH:-640}"
height="${FPVSIM_RAW_VIDEO_HEIGHT:-360}"
sink="${FPVSIM_VIDEO_SINK:-autovideosink sync=false}"

if ! command -v gst-launch-1.0 >/dev/null; then
    echo "error: gst-launch-1.0 not found. Run ./scripts/bootstrap_fedora.sh." >&2
    exit 1
fi

"${repo_root}/scripts/run_app.sh" "$@" -- --raw-video &
app_pid=$!
gst_pid=""

cleanup() {
    if [[ -n "${gst_pid}" ]] && kill -0 "${gst_pid}" 2>/dev/null; then
        kill "${gst_pid}" 2>/dev/null || true
    fi
    if kill -0 "${app_pid}" 2>/dev/null; then
        kill "${app_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

listening() {
    (exec 3<>"/dev/tcp/127.0.0.1/${port}") 2>/dev/null
}

# The frame server only listens while a session runs; reattach the window for every session
while kill -0 "${app_pid}" 2>/dev/null; do
    if listening; then
        gst-launch-1.0 -q tcpclientsrc host=127.0.0.1 port="${port}" \
            ! rawvideoparse format=rgb width="${width}" height="${height}" framerate=60/1 \
            ! videoconvert ! ${sink} &
        gst_pid=$!
        wait "${gst_pid}" || true
        gst_pid=""
    fi
    sleep 1
done
