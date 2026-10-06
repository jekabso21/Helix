import re
from pathlib import Path

PROC_DEVICES = Path("/proc/bus/input/devices")


def joystick_names(proc_devices: Path = PROC_DEVICES) -> list[str]:
    """Names of the connected joysticks, as the kernel (and so SDL) reports them.

    Pointers that also get a js node, such as keyd's virtual pointer, are left out: SDL does not
    offer them as joysticks either.
    """
    try:
        text = proc_devices.read_text()
    except OSError:
        return []
    names: list[str] = []
    for block in text.split("\n\n"):
        name = re.search(r'^N: Name="(.*)"$', block, re.MULTILINE)
        handlers = re.search(r"^H: Handlers=(.*)$", block, re.MULTILINE)
        if name is None or handlers is None:
            continue
        kinds = handlers.group(1).split()
        if any(re.fullmatch(r"js\d+", k) for k in kinds) and not any(
            re.fullmatch(r"mouse\d+", k) for k in kinds
        ):
            names.append(name.group(1))
    return names
