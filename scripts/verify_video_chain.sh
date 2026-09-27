#!/usr/bin/env bash
# Acceptance check for the camera path: the app renders and publishes a frame, simvideo passes it
# to an output, and the bytes that arrive are the bytes the panel showed.
#
#   ./scripts/verify_video_chain.sh [session.yaml]
#
# Uses a raw RGB file output on purpose: v4l2loopback in YUY2 converts colour, so byte identity
# can only be shown on an output that does not. Run ./scripts/build_extension.sh first.
set -uo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
session="${1:-configs/sessions/ci_hover.yaml}"
work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT

if [[ ! -e "${repo_root}/app/bin/libfpvsim_extension.so" ]]; then
    echo "error: build the frame publisher first: ./scripts/build_extension.sh" >&2
    exit 1
fi

cat > "${work}/start.py" <<'PY'
import json, socket, sys
def call(method, params=None):
    s = socket.create_connection(("127.0.0.1", 7740), timeout=90)
    s.sendall((json.dumps({"id": 1, "method": method, "params": params or {}}) + "\n").encode())
    f = s.makefile("r")
    while True:
        line = f.readline()
        if not line:
            return {"ok": False}
        message = json.loads(line)
        if message.get("id") == 1:
            s.close()
            return message
started = call("start", {"path": sys.argv[1]})
if not started.get("ok"):
    print("start failed:", started.get("error")); sys.exit(1)
print(call("status")["result"]["run_dir"])
PY

cat > "${work}/grab.py" <<'PY'
import struct, sys
blob = open("/dev/shm/fpvsim." + sys.argv[1], "rb").read()
magic, version, _fmt, _w, height, stride, slots = struct.unpack_from("<IHHIIII", blob, 0)
slot_size, latest = struct.unpack_from("<QQ", blob, 24)
if magic != 0x46565046 or version != 1:
    print("not a frame ring"); sys.exit(1)
if latest == 0:
    print("no frame published yet"); sys.exit(1)
slot = 128 + slot_size * ((latest - 1) % slots)
begin, = struct.unpack_from("<Q", blob, slot)
end, = struct.unpack_from("<Q", blob, slot + slot_size - 8)
if not begin == end == latest:
    print("torn read"); sys.exit(2)
open(sys.argv[2], "wb").write(blob[slot + 128 : slot + 128 + stride * height])
print("ring frame: sequence %d, %d bytes" % (latest, stride * height))
PY

"${repo_root}/scripts/run_app.sh" -- --screenshot "${work}/app.png" --screenshot-after 45 \
    > "${work}/app.log" 2>&1 &
app=$!
sleep 7
run_dir="$(python3 "${work}/start.py" "${session}" | tail -1)"
if [[ ! -d "${run_dir}" ]]; then
    echo "error: no run directory; see ${work}/app.log" >&2
    kill "${app}" 2>/dev/null
    exit 1
fi
echo "run directory: ${run_dir}"
sleep 8

camera="$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['cameras'][0]['name'])" \
    "${run_dir}/resolved/cameras.json")"
python3 - "${run_dir}/resolved/cameras.json" "${work}/output.raw" <<'PY'
import json, sys
path = sys.argv[1]
document = json.load(open(path))
document["cameras"][0]["outputs"] = [{"pipeline": "filesink location=" + sys.argv[2], "enabled": True}]
json.dump(document, open(path, "w"), indent=2)
PY

"${repo_root}/build/dev/simvideo/simvideo" --cameras "${run_dir}/resolved/cameras.json" \
    > "${work}/simvideo.log" 2>&1 &
video=$!
sleep 4
for _ in 1 2 3; do
    python3 "${work}/grab.py" "${camera}" "${work}/frame.bin" && break
    sleep 0.3
done
sleep 1
kill "${video}" 2>/dev/null; wait "${video}" 2>/dev/null
kill "${app}" 2>/dev/null; wait "${app}" 2>/dev/null

python3 - "${work}/frame.bin" "${work}/output.raw" <<'PY'
import sys
frame = open(sys.argv[1], "rb").read()
data = open(sys.argv[2], "rb").read()
size = len(frame)
frames = len(data) // size if size else 0
match = next((i for i in range(frames) if data[i * size:(i + 1) * size] == frame), -1)
print("output received %d frames of %d bytes" % (frames, size))
if match >= 0:
    print("PASS: the published frame arrived byte-identical (output frame %d)" % match)
else:
    print("FAIL: no byte-identical frame in the output")
    sys.exit(1)
PY
