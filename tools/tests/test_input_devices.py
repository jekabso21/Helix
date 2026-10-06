from pathlib import Path

from simtools.input_devices import joystick_names

# trimmed from a real /proc/bus/input/devices: keyd's virtual pointer also gets a js node
PROC = """I: Bus=0003 Vendor=1209 Product=4f54 Version=0111
N: Name="OpenTX Radiomaster Boxer Joystick"
P: Phys=usb-0000:00:14.0-2/input0
H: Handlers=event3 js1
B: EV=1b
B: ABS=7ff

I: Bus=0006 Vendor=0fac Product=1ade Version=0001
N: Name="keyd virtual pointer"
H: Handlers=mouse1 event24 js0
B: EV=f

I: Bus=0011 Vendor=0001 Product=0001 Version=ab41
N: Name="AT Translated Set 2 keyboard"
H: Handlers=sysrq kbd leds event2
B: EV=120013
"""


def test_only_joysticks_that_are_not_pointers_are_listed(tmp_path: Path) -> None:
    proc = tmp_path / "devices"
    proc.write_text(PROC)
    assert joystick_names(proc) == ["OpenTX Radiomaster Boxer Joystick"]


def test_a_missing_device_table_lists_nothing(tmp_path: Path) -> None:
    assert joystick_names(tmp_path / "absent") == []
