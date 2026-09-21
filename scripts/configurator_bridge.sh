#!/usr/bin/env bash
set -euo pipefail

ws_port="${1:-6761}"
uart_port="${2:-5761}"

if ! command -v websockify >/dev/null; then
    echo "error: websockify not found. Run ./scripts/bootstrap_fedora.sh" >&2
    exit 1
fi

# Look at the listening sockets; a probe connection would occupy the single UART client slot
if ! ss -ltn | grep -q ":${uart_port} "; then
    echo "warning: nothing listens on TCP ${uart_port} yet; start Betaflight SITL first" >&2
fi

cat <<EOF
Bridging ws://127.0.0.1:${ws_port} -> Betaflight SITL UART on TCP ${uart_port}
Open https://app.betaflight.com, enable manual connection in its options,
and connect to: ws://127.0.0.1:${ws_port}
A SITL UART serves one client at a time. Stop with Ctrl+C.
EOF

exec websockify "127.0.0.1:${ws_port}" "127.0.0.1:${uart_port}"
