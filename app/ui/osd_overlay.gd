extends Control
## Draws the Betaflight OSD canvas (MSP DisplayPort, via the backend's osd topic) over the camera image

const BLINK_HZ := 2.0
const ATTR_BLINK := 0x40
## The horizon ladder is nine one-row glyphs, index 0 at the top of the cell (osd_elements.c)
const AH_BAR_FIRST := 0x80
const AH_BAR_COUNT := 9

## Betaflight font codes that are not ASCII, shown with the closest Unicode glyph
const SYMBOLS := {
	0x01: "▮",  # rssi
	0x02: "▶", 0x03: "◀",  # AH right / left
	0x04: "⇧",  # throttle
	0x05: "⌂",  # over home
	0x06: "V", 0x07: "㎃",  # volt, mAh
	0x0C: "m", 0x0D: "°F", 0x0E: "°C", 0x0F: "ft",
	0x10: "▤", 0x11: "⌂",  # bblog, homeflag
	0x13: "-",  # AH decoration / ladder
	0x14: "⟲", 0x15: "⤢",  # roll, pitch
	0x18: "N", 0x19: "S", 0x1A: "E", 0x1B: "W", 0x1C: "|", 0x1D: "·",
	0x1E: "(", 0x1F: ")",  # sat
	0x57: "W",
	0x60: "↓", 0x61: "↙", 0x62: "↙", 0x63: "↙", 0x64: "→", 0x65: "↗", 0x66: "↗", 0x67: "↗",
	0x68: "↑", 0x69: "↖", 0x6A: "↖", 0x6B: "↖", 0x6C: "←", 0x6D: "↙", 0x6E: "↙", 0x6F: "↙",
	0x70: "▷", 0x71: "⇉",  # speed, distance
	0x72: "─", 0x73: "┼", 0x74: "─",  # crosshairs
	0x75: "▴", 0x76: "▾",
	0x79: "⏱", 0x7A: "🌡", 0x7B: "▯", 0x7D: "km", 0x7E: "mi", 0x7F: "⤒",
	0x80: "▁", 0x81: "▂", 0x82: "▃", 0x83: "▄", 0x84: "▅", 0x85: "▆", 0x86: "▇", 0x87: "█", 0x88: "─",
	0x89: "⌖", 0x8A: "[", 0x8B: "▮", 0x8C: "▯", 0x8D: " ", 0x8E: "]", 0x8F: "]",
	0x90: "▮", 0x91: "▮", 0x92: "▮", 0x93: "▯", 0x94: "▯", 0x95: "▯", 0x96: "▯",
	0x97: "⚡", 0x98: "⌖", 0x99: "ft/s", 0x9A: "A", 0x9B: "⏲", 0x9C: "✈", 0x9D: "mph", 0x9E: "km/h", 0x9F: "m/s",
}

var cols: int = 53
var rows: int = 20
var codes: Array = []
var attrs: Array = []
var draws: int = 0

var _font: Font = null


func _ready() -> void:
	var system := SystemFont.new()
	system.font_names = PackedStringArray(["DejaVu Sans Mono", "Liberation Mono", "Noto Sans Mono", "monospace"])
	_font = system
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	BackendClient.osd.connect(set_canvas)
	BackendClient.status.connect(_on_status)


func _on_status(data: Dictionary) -> void:
	if data.get("state", "") != "running":
		clear()


func clear() -> void:
	codes = []
	attrs = []
	queue_redraw()


func set_canvas(data: Dictionary) -> void:
	cols = int(data.get("cols", cols))
	rows = int(data.get("rows", rows))
	codes = data.get("codes", [])
	attrs = data.get("attrs", [])
	draws += 1
	queue_redraw()


static func is_horizon_bar(code: int) -> bool:
	return code >= AH_BAR_FIRST and code < AH_BAR_FIRST + AH_BAR_COUNT


## The Betaflight font keeps ASCII except 0x60-0x6F, which hold the arrows
static func glyph(code: int) -> String:
	if code == 0x20 or code == 0x00:
		return ""
	if SYMBOLS.has(code):
		return SYMBOLS[code]
	if code >= 0x20 and code < 0x7F:
		return char(code)
	return "▪"


func _draw() -> void:
	if codes.is_empty():
		return
	var cell := Vector2(size.x / cols, size.y / rows)
	var font_size := int(cell.y * 0.9)
	var blink_on := fmod(Time.get_ticks_msec() / 1000.0 * BLINK_HZ, 1.0) < 0.5
	for r in mini(rows, codes.size()):
		var row: Array = codes[r]
		var row_attrs: Array = attrs[r] if r < attrs.size() else []
		for c in mini(cols, row.size()):
			var code := int(row[c])
			var attr: int = int(row_attrs[c]) if c < row_attrs.size() else 0
			if (attr & ATTR_BLINK) != 0 and not blink_on:
				continue
			if is_horizon_bar(code):
				_draw_horizon_bar(c, r, code - AH_BAR_FIRST, cell)
				continue
			var text := glyph(code)
			if text == "":
				continue
			var at := Vector2(c * cell.x, (r + 0.85) * cell.y)
			draw_string(_font, at + Vector2(1.5, 1.5), text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, Color(0, 0, 0, 0.9))
			draw_string(_font, at, text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, Color(1, 1, 1))


## A ladder glyph is a thin line across the cell, not a filled block
func _draw_horizon_bar(col: int, row: int, index: int, cell: Vector2) -> void:
	var y := (row + (index + 0.5) / float(AH_BAR_COUNT)) * cell.y
	var from := Vector2(col * cell.x, y)
	var to := Vector2((col + 1) * cell.x, y)
	var thickness := maxf(cell.y * 0.09, 1.0)
	draw_line(from + Vector2(1.0, 1.5), to + Vector2(1.0, 1.5), Color(0, 0, 0, 0.9), thickness)
	draw_line(from, to, Color(1, 1, 1), thickness)


func _process(_delta: float) -> void:
	if not codes.is_empty():
		queue_redraw()  # blink
