extends SceneTree
## Writes res://ui/theme.tres: godot --headless --path app -s tools/build_theme.gd
## The colours, fonts and sizes live here; panels only name theme types and variations.

const OUT := "res://ui/theme.tres"

const BG := Color("#141618")
const SURFACE := Color("#1d2023")
const TEXT := Color("#e6e7e9")
const DIVIDER := Color(0.902, 0.906, 0.914, 0.16)
const ROW_DIVIDER := Color(0.902, 0.906, 0.914, 0.08)
const N100 := Color("#1a1c1f")
const N200 := Color("#24272a")
const N300 := Color("#33363a")
const N400 := Color("#4a4d51")
const N500 := Color("#6b6e72")
const N600 := Color("#8e9195")
const N700 := Color("#aeb1b4")
const N800 := Color("#cfd1d3")
const ACCENT := Color("#5980a6")
const A100 := Color("#1d2d3d")
const A200 := Color("#2c455d")
const A500 := Color("#749dc4")
const A600 := Color("#94bce3")
const A700 := Color("#b5d9fd")
const A800 := Color("#d6ebff")
const A900 := Color("#eef6ff")
const CHIP := Color(0.063, 0.071, 0.078, 0.72)
const VIEW_BG := Color("#101214")
const ERROR := Color("#e07a6e")
const WARNING := Color("#e0c06e")

const RADIUS := 4

var theme := Theme.new()
var body: Font
var body_medium: Font
var heading: Font
var mono: Font


func _initialize() -> void:
	_fonts()
	_palette()
	_base()
	_labels()
	_buttons()
	_inputs()
	_sliders()
	_panels()
	_scroll_and_separators()
	_containers()
	var error := ResourceSaver.save(theme, OUT)
	print("theme written to %s: %s" % [OUT, error_string(error)])
	quit(0 if error == OK else 1)


func _fonts() -> void:
	# glyphs the latin subsets lack (arrows, triangles) come from whatever the system has
	var fallback := SystemFont.new()
	fallback.font_names = PackedStringArray(["DejaVu Sans", "Noto Sans", "sans-serif"])
	body = _font("res://ui/fonts/Barlow-Regular.woff2", fallback)
	body_medium = _font("res://ui/fonts/Barlow-Medium.woff2", fallback)
	heading = _font("res://ui/fonts/BarlowCondensed-SemiBold.woff2", fallback)
	mono = _font("res://ui/fonts/JetBrainsMono.woff2", fallback)


func _font(path: String, fallback: Font) -> FontVariation:
	var variation := FontVariation.new()
	variation.base_font = load(path)
	variation.fallbacks = [fallback]
	return variation


func _spaced(base: Font, glyph_spacing: int) -> FontVariation:
	var variation := FontVariation.new()
	variation.base_font = base
	variation.spacing_glyph = glyph_spacing
	return variation


## Custom-drawn controls (bars, sparklines, dots, frame corners) read their colours from here
func _palette() -> void:
	var colors := {
		"bg": BG, "surface": SURFACE, "text": TEXT, "divider": DIVIDER,
		"track": N200, "track_strong": N300, "centre": N500, "muted": N600,
		"accent": ACCENT, "accent_line": A500, "accent_soft": A100,
		"plot_bg": N100, "plot_zero": N400, "view_bg": VIEW_BG,
		"corner": Color(TEXT, 0.55), "off": N300, "error": ERROR, "warning": WARNING,
		"chip_bg": CHIP, "chip_text": Color("#e9e9ea"), "pip_border": Color(1, 1, 1, 0.55),
	}
	for key: String in colors:
		theme.set_color(key, "Palette", colors[key])
	theme.set_font("mono", "Palette", mono)
	theme.set_font("heading", "Palette", heading)


func _base() -> void:
	theme.default_font = body
	theme.default_font_size = 13
	var root := StyleBoxFlat.new()
	root.bg_color = BG
	theme.set_stylebox("panel", "Panel", root)
	theme.set_stylebox("panel", "PanelContainer", StyleBoxEmpty.new())
	var tooltip := _box(SURFACE, DIVIDER, 0, 6, 4)
	theme.set_stylebox("panel", "TooltipPanel", tooltip)
	theme.set_color("font_color", "TooltipLabel", TEXT)
	theme.set_font_size("font_size", "TooltipLabel", 12)


func _labels() -> void:
	theme.set_color("font_color", "Label", TEXT)
	_label("Muted", body, 12, N700)
	_label("Hint", body, 11, N600)
	_label("Strong", body_medium, 12, TEXT)
	_label("Kicker", _spaced(heading, 1), 11, A700)
	_label("Heading", _spaced(heading, 0), 15, TEXT)
	_label("Brand", heading, 20, TEXT)
	_label("StateValue", heading, 20, TEXT)
	_label("StateValueDark", heading, 20, BG)
	_label("TileCaption", _spaced(body, 1), 10, N700)
	_label("TileCaptionDark", _spaced(body, 1), 10, Color(BG, 0.8))
	_label("Mono", mono, 12, TEXT)
	_label("MonoMuted", mono, 11, N600)
	_label("MonoSoft", mono, 11, N800)
	_label("MonoLarge", mono, 17, TEXT)
	_label("MonoClock", mono, 16, TEXT)
	_label("ChipText", _spaced(mono, 1), 11, Color("#e9e9ea"))
	_label("TagText", _spaced(heading, 1), 11, N800)
	_label("TagTextAccent", _spaced(body_medium, 1), 11, A800)
	_label("TagTextArmed", _spaced(heading, 1), 11, A900)
	_label("Error", body, 12, ERROR)
	theme.set_type_variation("Code", "RichTextLabel")
	theme.set_font("normal_font", "Code", mono)
	theme.set_font_size("normal_font_size", "Code", 11)
	theme.set_color("default_color", "Code", N800)
	theme.set_stylebox("normal", "Code", _box(N100, DIVIDER, 0, 10, 8))


func _label(variation: String, font: Font, size: int, color: Color) -> void:
	theme.set_type_variation(variation, "Label")
	theme.set_font("font", variation, font)
	theme.set_font_size("font_size", variation, size)
	theme.set_color("font_color", variation, color)


func _buttons() -> void:
	# secondary is the default: a hairline outline that fills faintly on hover
	_button_colors("Button", TEXT, TEXT, Color(TEXT, 0.45))
	theme.set_font("font", "Button", heading)
	theme.set_font_size("font_size", "Button", 14)
	_button_boxes("Button", _box(Color(0, 0, 0, 0), DIVIDER, RADIUS, 12, 5),
		_box(Color(TEXT, 0.07), DIVIDER, RADIUS, 12, 5),
		_box(Color(TEXT, 0.14), DIVIDER, RADIUS, 12, 5),
		_box(Color(0, 0, 0, 0), Color(DIVIDER, 0.08), RADIUS, 12, 5))

	_variation("PrimaryButton", "Button")
	_button_colors("PrimaryButton", BG, BG, Color(BG, 0.6))
	_button_boxes("PrimaryButton", _box(ACCENT, ACCENT, RADIUS, 14, 5),
		_box(A600, A600, RADIUS, 14, 5), _box(A700, A700, RADIUS, 14, 5),
		_box(Color(ACCENT, 0.45), Color(ACCENT, 0), RADIUS, 14, 5))

	# armed while waiting for a control to move
	_variation("ActiveButton", "Button")
	_button_colors("ActiveButton", BG, BG, BG)
	_button_boxes("ActiveButton", _box(ACCENT, ACCENT, RADIUS, 8, 3),
		_box(A600, A600, RADIUS, 8, 3), _box(A700, A700, RADIUS, 8, 3), _box(ACCENT, ACCENT, RADIUS, 8, 3))

	_variation("SmallButton", "Button")
	theme.set_font("font", "SmallButton", body)
	theme.set_font_size("font_size", "SmallButton", 12)
	_button_boxes("SmallButton", _box(Color(0, 0, 0, 0), DIVIDER, RADIUS, 8, 3),
		_box(Color(TEXT, 0.07), DIVIDER, RADIUS, 8, 3),
		_box(Color(TEXT, 0.14), DIVIDER, RADIUS, 8, 3),
		_box(Color(0, 0, 0, 0), Color(DIVIDER, 0.08), RADIUS, 8, 3))

	_variation("TabButton", "Button")
	_button_colors("TabButton", N600, TEXT, N500)
	theme.set_color("font_pressed_color", "TabButton", TEXT)
	theme.set_color("font_hover_pressed_color", "TabButton", TEXT)
	var tab := _box(Color(0, 0, 0, 0), Color(0, 0, 0, 0), 0, 6, 0)
	var tab_hover := _box(Color(ACCENT, 0.1), Color(0, 0, 0, 0), 0, 6, 0)
	var tab_on := _box(Color(0, 0, 0, 0), ACCENT, 0, 6, 0)
	tab_on.set_border_width_all(0)
	tab_on.border_width_bottom = 2
	_button_boxes("TabButton", tab, tab_hover, tab_on, tab)
	theme.set_stylebox("hover_pressed", "TabButton", tab_on)

	_variation("SegButton", "Button")
	theme.set_font_size("font_size", "SegButton", 13)
	_button_colors("SegButton", TEXT, TEXT, N500)
	theme.set_color("font_pressed_color", "SegButton", BG)
	theme.set_color("font_hover_pressed_color", "SegButton", BG)
	var seg := _box(Color(0, 0, 0, 0), Color(0, 0, 0, 0), 0, 14, 5)
	var seg_hover := _box(Color(TEXT, 0.07), Color(0, 0, 0, 0), 0, 14, 5)
	var seg_on := _box(ACCENT, ACCENT, 0, 14, 5)
	_button_boxes("SegButton", seg, seg_hover, seg_on, seg)
	theme.set_stylebox("hover_pressed", "SegButton", seg_on)

	_variation("GhostButton", "Button")
	theme.set_font("font", "GhostButton", body)
	theme.set_font_size("font_size", "GhostButton", 12)
	_button_colors("GhostButton", A700, A900, N500)
	var ghost := _box(Color(0, 0, 0, 0), Color(0, 0, 0, 0), 0, 4, 2)
	_button_boxes("GhostButton", ghost, _box(Color(ACCENT, 0.1), Color(0, 0, 0, 0), 0, 4, 2), _box(Color(ACCENT, 0.18), Color(0, 0, 0, 0), 0, 4, 2), ghost)

	# the dock collapse handles: full height, icon only
	_variation("IconButton", "Button")
	_button_colors("IconButton", N700, TEXT, N500)
	var icon := _box(Color(0, 0, 0, 0), Color(0, 0, 0, 0), 0, 0, 0)
	_button_boxes("IconButton", icon, _box(Color(ACCENT, 0.1), Color(0, 0, 0, 0), 0, 0, 0), _box(Color(ACCENT, 0.18), Color(0, 0, 0, 0), 0, 0, 0), icon)

	# the picture-in-picture thumbnail is a button so a click swaps the views
	_variation("PipButton", "Button")
	var pip := StyleBoxFlat.new()
	pip.bg_color = Color.BLACK
	pip.set_border_width_all(1)
	pip.border_color = Color(1, 1, 1, 0.55)
	pip.shadow_color = Color(0, 0, 0, 0.55)
	pip.shadow_size = 8
	pip.shadow_offset = Vector2(0, 3)
	var pip_hover := pip.duplicate() as StyleBoxFlat
	pip_hover.border_color = Color(1, 1, 1, 0.9)
	_button_boxes("PipButton", pip, pip_hover, pip_hover, pip)

	theme.set_font("font", "CheckBox", body)
	theme.set_font_size("font_size", "CheckBox", 12)
	_button_colors("CheckBox", N700, TEXT, N500)
	theme.set_color("font_pressed_color", "CheckBox", N700)
	for state: String in ["normal", "hover", "pressed", "disabled", "hover_pressed"]:
		theme.set_stylebox(state, "CheckBox", _box(Color(0, 0, 0, 0), Color(0, 0, 0, 0), 0, 2, 2))
		theme.set_stylebox(state, "CheckButton", _box(Color(0, 0, 0, 0), Color(0, 0, 0, 0), 0, 2, 2))
	theme.set_font("font", "CheckButton", body)
	theme.set_font_size("font_size", "CheckButton", 12)
	_button_colors("CheckButton", TEXT, TEXT, N500)
	theme.set_color("font_pressed_color", "CheckButton", TEXT)

	theme.set_font("font", "OptionButton", body)
	theme.set_font_size("font_size", "OptionButton", 13)
	_button_colors("OptionButton", TEXT, TEXT, N500)
	theme.set_color("font_pressed_color", "OptionButton", TEXT)
	var field := _box(SURFACE, DIVIDER, 0, 8, 5)
	var field_hover := _box(SURFACE, Color(TEXT, 0.45), 0, 8, 5)
	_button_boxes("OptionButton", field, field_hover, field_hover, _box(SURFACE, Color(DIVIDER, 0.08), 0, 8, 5))
	theme.set_constant("arrow_margin", "OptionButton", 8)
	theme.set_color("modulate_arrow", "OptionButton", N700)

	_variation("SmallOption", "OptionButton")
	theme.set_font_size("font_size", "SmallOption", 12)
	var small := _box(SURFACE, DIVIDER, 0, 6, 3)
	_button_boxes("SmallOption", small, _box(SURFACE, Color(TEXT, 0.45), 0, 6, 3), _box(SURFACE, Color(TEXT, 0.45), 0, 6, 3), _box(SURFACE, Color(DIVIDER, 0.08), 0, 6, 3))

	theme.set_stylebox("panel", "PopupMenu", _box(SURFACE, DIVIDER, 0, 4, 4))
	theme.set_stylebox("hover", "PopupMenu", _box(Color(ACCENT, 0.25), Color(0, 0, 0, 0), 0, 4, 2))
	theme.set_color("font_color", "PopupMenu", TEXT)
	theme.set_color("font_hover_color", "PopupMenu", TEXT)
	theme.set_color("font_disabled_color", "PopupMenu", N500)
	theme.set_font("font", "PopupMenu", body)
	theme.set_font_size("font_size", "PopupMenu", 13)


func _variation(name: String, base: String) -> void:
	theme.set_type_variation(name, base)


func _button_colors(type: String, normal: Color, hover: Color, disabled: Color) -> void:
	theme.set_color("font_color", type, normal)
	theme.set_color("font_hover_color", type, hover)
	theme.set_color("font_focus_color", type, normal)
	theme.set_color("font_pressed_color", type, hover)
	theme.set_color("font_hover_pressed_color", type, hover)
	theme.set_color("font_disabled_color", type, disabled)
	theme.set_color("icon_normal_color", type, normal)
	theme.set_color("icon_hover_color", type, hover)
	theme.set_color("icon_pressed_color", type, hover)
	theme.set_color("icon_disabled_color", type, disabled)


func _button_boxes(type: String, normal: StyleBox, hover: StyleBox, pressed: StyleBox, disabled: StyleBox) -> void:
	theme.set_stylebox("normal", type, normal)
	theme.set_stylebox("hover", type, hover)
	theme.set_stylebox("pressed", type, pressed)
	theme.set_stylebox("hover_pressed", type, pressed)
	theme.set_stylebox("disabled", type, disabled)
	var focus := StyleBoxFlat.new()
	focus.draw_center = false
	focus.set_border_width_all(1)
	focus.border_color = Color(ACCENT, 0.9)
	focus.set_corner_radius_all((normal as StyleBoxFlat).corner_radius_top_left if normal is StyleBoxFlat else 0)
	theme.set_stylebox("focus", type, focus)


func _inputs() -> void:
	var field := _box(SURFACE, DIVIDER, 0, 8, 5)
	var focus := _box(SURFACE, ACCENT, 0, 8, 5)
	theme.set_stylebox("normal", "LineEdit", field)
	theme.set_stylebox("focus", "LineEdit", focus)
	theme.set_stylebox("read_only", "LineEdit", field)
	theme.set_font("font", "LineEdit", body)
	theme.set_font_size("font_size", "LineEdit", 13)
	theme.set_color("font_color", "LineEdit", TEXT)
	theme.set_color("font_placeholder_color", "LineEdit", N500)
	theme.set_color("caret_color", "LineEdit", ACCENT)
	theme.set_color("selection_color", "LineEdit", Color(ACCENT, 0.3))

	theme.set_stylebox("background", "ProgressBar", _box(N200, Color(0, 0, 0, 0), 0, 0, 0))
	theme.set_stylebox("fill", "ProgressBar", _box(ACCENT, Color(0, 0, 0, 0), 0, 0, 0))
	theme.set_color("font_color", "ProgressBar", TEXT)


func _sliders() -> void:
	var track := StyleBoxFlat.new()
	track.bg_color = N300
	track.content_margin_top = 2
	track.content_margin_bottom = 2
	var fill := StyleBoxFlat.new()
	fill.bg_color = ACCENT
	fill.content_margin_top = 2
	fill.content_margin_bottom = 2
	var fill_hover := fill.duplicate() as StyleBoxFlat
	fill_hover.bg_color = A600
	theme.set_stylebox("slider", "HSlider", track)
	theme.set_stylebox("grabber_area", "HSlider", fill)
	theme.set_stylebox("grabber_area_highlight", "HSlider", fill_hover)


func _panels() -> void:
	_panel("Tile", _box(Color(0, 0, 0, 0), DIVIDER, 0, 10, 7))
	_panel("TileArmed", _box(A900, DIVIDER, 0, 10, 7))
	_panel("Row", _box(Color(0, 0, 0, 0), DIVIDER, 0, 10, 6))
	_panel("CodeBox", _box(SURFACE, DIVIDER, 0, 8, 6))
	_panel("Chip", _box(CHIP, Color(0, 0, 0, 0), 0, 8, 3))
	_panel("Tag", _box(N200, Color(0, 0, 0, 0), 3, 10, 3))
	_panel("TagAccent", _box(A100, Color(0, 0, 0, 0), 3, 10, 3))
	_panel("TagArmed", _box(A200, Color(0, 0, 0, 0), 3, 10, 3))
	_panel("Segment", _box(Color(0, 0, 0, 0), DIVIDER, RADIUS, 0, 0))
	_panel("ViewFrame", _box(VIEW_BG, DIVIDER, 0, 1, 1))
	var dock := StyleBoxFlat.new()
	dock.bg_color = BG
	_panel("Dock", dock)
	var header := StyleBoxFlat.new()
	header.bg_color = BG
	header.border_color = DIVIDER
	header.border_width_bottom = 1
	_panel("DockHeader", header)
	_panel("TopBar", header.duplicate())
	var row := StyleBoxFlat.new()
	row.bg_color = Color(0, 0, 0, 0)
	row.border_color = ROW_DIVIDER
	row.border_width_bottom = 1
	row.content_margin_left = 4
	row.content_margin_right = 4
	_panel("TableRow", row)


func _panel(name: String, box: StyleBox) -> void:
	theme.set_type_variation(name, "PanelContainer")
	theme.set_stylebox("panel", name, box)


func _scroll_and_separators() -> void:
	for type: String in ["VScrollBar", "HScrollBar"]:
		theme.set_stylebox("scroll", type, _box(Color(0, 0, 0, 0), Color(0, 0, 0, 0), 0, 4, 4))
		theme.set_stylebox("grabber", type, _box(N300, Color(0, 0, 0, 0), 0, 4, 4))
		theme.set_stylebox("grabber_highlight", type, _box(N400, Color(0, 0, 0, 0), 0, 4, 4))
		theme.set_stylebox("grabber_pressed", type, _box(N500, Color(0, 0, 0, 0), 0, 4, 4))
	for type: String in ["HSeparator", "VSeparator"]:
		var line := StyleBoxLine.new()
		line.color = DIVIDER
		line.thickness = 1
		line.vertical = type == "VSeparator"
		theme.set_stylebox("separator", type, line)
		theme.set_constant("separation", type, 1)


func _box(fill: Color, border: Color, radius: int, pad_x: float, pad_y: float) -> StyleBoxFlat:
	var box := StyleBoxFlat.new()
	box.bg_color = fill
	box.border_color = border
	box.set_border_width_all(1 if border.a > 0.0 else 0)
	box.set_corner_radius_all(radius)
	box.content_margin_left = pad_x
	box.content_margin_right = pad_x
	box.content_margin_top = pad_y
	box.content_margin_bottom = pad_y
	box.anti_aliasing = radius > 0
	return box


## Spacing from the design's 3.4 px scale, so layouts name a rhythm instead of a number
func _containers() -> void:
	for entry: Array in [["Stack", "VBoxContainer", 0], ["Tight", "VBoxContainer", 3], ["Section", "VBoxContainer", 7],
			["Sections", "VBoxContainer", 20], ["Strip", "HBoxContainer", 0], ["Gap4", "HBoxContainer", 4],
			["Gap6", "HBoxContainer", 6], ["Gap8", "HBoxContainer", 8], ["Gap10", "HBoxContainer", 10],
			["Gap14", "HBoxContainer", 14]]:
		theme.set_type_variation(entry[0], entry[1])
		theme.set_constant("separation", entry[0], entry[2])
	for entry: Array in [["Grid2", 7, 7], ["Grid3", 10, 4], ["Grid4", 10, 6], ["Motors", 10, 10]]:
		theme.set_type_variation(entry[0], "GridContainer")
		theme.set_constant("h_separation", entry[0], entry[1])
		theme.set_constant("v_separation", entry[0], entry[2])
	theme.set_type_variation("HFlow4", "HFlowContainer")
	theme.set_constant("h_separation", "HFlow4", 4)
	theme.set_constant("v_separation", "HFlow4", 4)
	for entry: Array in [["Pad", 14, 14], ["BarPad", 14, 0], ["ChipRow", 12, 10]]:
		theme.set_type_variation(entry[0], "MarginContainer")
		for side: String in ["left", "right"]:
			theme.set_constant("margin_" + side, entry[0], entry[1])
		for side: String in ["top", "bottom"]:
			theme.set_constant("margin_" + side, entry[0], entry[2])
