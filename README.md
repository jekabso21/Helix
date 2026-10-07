# fpvsim

Graphical software-in-the-loop simulator for fast FPV drones that run Betaflight. The real
Betaflight firmware runs on the PC as a SITL binary; fpvsim provides the physics, sensors,
environment, camera video, link emulation, failure injection and test automation around it.

`fpvsim` is a working name. Linux only; Fedora is the development platform.

> **Status: early development.** This project is not ready for use. Do not rely on it for
> production work or for developing, testing or validating firmware, hardware or flight
> behaviour. Interfaces, configs and results change without notice, and the physics is not yet
> implemented or validated.

## Demo

![fpvsim demo: flying the FPV view with the Betaflight OSD, input, telemetry and motor outputs](demo/2026-10-07-new-ui.gif)

[Full video](demo/2026-10-07-new-ui.mp4): the app flown with a RadioMaster Boxer. The controller
is detected and its mapping loaded on its own; the FPV view shows the published camera feed with
the Betaflight OSD, the Telemetry dock follows the flight live (FC state, rates, battery, motors),
a crash ends on Betaflight's stats screen, and the raw camera output plays in a GStreamer window.

[Earlier video](demo/2026-09-22-first-flight.mp4): the first flight, with chase and FPV views and
the video output in a GStreamer window.

## Setup

```bash
git submodule update --init third_party/betaflight
./scripts/bootstrap_fedora.sh       # system dependencies (dnf, needs sudo)
./scripts/bf_build.sh               # Betaflight SITL -> build/betaflight/betaflight_SITL.elf
cmake --preset dev && cmake --build --preset dev
ctest --preset dev
cd tools && uv sync && uv run pytest && cd ..    # add -m integration for the SITL flight test
godot --headless --path app -s tests/run_tests.gd  # GDScript unit tests
pre-commit install
```

CMake presets: `dev` (GCC, Debug, ASan and UBSan), `clang` (Clang, warnings as errors),
`release`, `ci` (GCC, warnings as errors).

## Run

```bash
./scripts/run_app.sh                                        # GUI app; starts simctl serve itself
./scripts/run_app_video.sh                                  # GUI app plus a GStreamer window with the camera output
./scripts/run_app.sh -- --attach                            # GUI attached to a running simctl run
./scripts/run_app.sh -- --screenshot /tmp/app.png           # save the window content and quit
./scripts/run_app.sh -- --ui-scale 1.25                     # start at another UI scale (Ctrl + / Ctrl - / Ctrl 0 in the app)
./scripts/run_app.sh -- --tab Failures                      # open a left dock tab (Input, Drone, Environment, Failures, Scenario)
uv run --project tools simctl validate configs/sessions/ci_hover.yaml
uv run --project tools simctl run configs/sessions/ci_hover.yaml --headless   # hover test flight, run dir in runs/
uv run --project tools python tools/spikes/spin_motors.py   # arm Betaflight SITL, read motors
uv run --project tools python tools/spikes/radio_passthrough.py --duration 60   # USB radio -> SITL
./scripts/configurator_bridge.sh                            # WebSocket bridge for app.betaflight.com
uv run --project tools simctl latency --consumer "v4l2src device=/dev/video10"  # sim-to-output video latency
uv run --project tools simctl frame-timestamps               # frame sim time against the live state
```

## License

Betaflight is GPLv3. It runs as a separate process and is never linked into fpvsim binaries.
The patches in `patches/betaflight/` are GPLv3 like Betaflight.
