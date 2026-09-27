from dataclasses import dataclass, field

MSP_DISPLAYPORT = 182
MSP_SET_OSD_CANVAS = 188

DP_HEARTBEAT = 0
DP_RELEASE = 1
DP_CLEAR_SCREEN = 2
DP_WRITE_STRING = 3
DP_DRAW_SCREEN = 4
DP_OPTIONS = 5
DP_SYS = 6

ATTR_BLINK = 1 << 6
ATTR_FONT_MASK = 0x03

SD_COLS, SD_ROWS = 30, 16
HD_COLS, HD_ROWS = 53, 20
MAX_COLS, MAX_ROWS = 63, 31

OSD_PROFILE_BITS_POS = 11


def osd_pos(x: int, y: int, profile: int = 1) -> int:
    """Betaflight OSD_POS(x, y) with the element visible in one profile (settings.c `osd_*_pos`)."""
    if not (0 <= x <= MAX_COLS and 0 <= y <= MAX_ROWS):
        raise ValueError(f"OSD position out of range: {x}, {y}")
    return (
        (x & 31) | ((y & 31) << 5) | ((x >> 5) << 10) | (1 << (profile - 1 + OSD_PROFILE_BITS_POS))
    )


@dataclass
class OsdGrid:
    """Character canvas driven by MSP DisplayPort frames; codes are Betaflight font indices."""

    cols: int = HD_COLS
    rows: int = HD_ROWS
    codes: list[list[int]] = field(default_factory=list[list[int]])
    attrs: list[list[int]] = field(default_factory=list[list[int]])
    draws: int = 0
    frames: int = 0

    def __post_init__(self) -> None:
        self.clear()

    def clear(self) -> None:
        self.codes = [[0x20] * self.cols for _ in range(self.rows)]
        self.attrs = [[0] * self.cols for _ in range(self.rows)]

    def apply(self, payload: bytes) -> bool:
        """Applies one MSP_DISPLAYPORT payload; True when the screen should be presented."""
        if not payload:
            return False
        self.frames += 1
        sub = payload[0]
        if sub == DP_CLEAR_SCREEN:
            self.clear()
        elif sub == DP_WRITE_STRING and len(payload) >= 4:
            row, col, attr = payload[1], payload[2], payload[3]
            if row < self.rows:
                for i, code in enumerate(payload[4:]):
                    if col + i >= self.cols:
                        break
                    self.codes[row][col + i] = code
                    self.attrs[row][col + i] = attr
        elif sub == DP_DRAW_SCREEN:
            self.draws += 1
            return True
        return False

    def text_rows(self) -> list[str]:
        """Printable view for logs and tests: font codes outside ASCII become '?'."""
        return [
            "".join(chr(c) if 0x20 <= c < 0x7F else "?" for c in row).rstrip() for row in self.codes
        ]

    def to_json(self) -> dict[str, object]:
        return {"cols": self.cols, "rows": self.rows, "codes": self.codes, "attrs": self.attrs}


def canvas_payload(cols: int, rows: int) -> bytes:
    return bytes([cols, rows])
