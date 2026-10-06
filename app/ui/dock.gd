class_name Dock
extends VBoxContainer
## A side dock: a tab strip (or a title) over its pages, folding down to a narrow handle.
## The pages are the dock's children in the scene and stay where they are: moving a page out of
## the tree would run its exit handlers.

signal collapsed_changed(collapsed: bool)

const HANDLE_WIDTH := 36.0
const HEADER_HEIGHT := 40.0

@export var right_side := false
@export var title := ""
@export var expanded_width := 340.0

var collapsed := false
var current := 0
var _pages: Array[Control] = []
var _tabs: Array[Button] = []
var _meta: Label = null
var _header: Control = null
var _handle: Button = null
var _handle_label: VerticalLabel = null


func _ready() -> void:
	theme_type_variation = "Stack"
	size_flags_horizontal = Control.SIZE_FILL
	for child in get_children():
		if child is Control:
			_pages.append(child)
			child.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_header = _build_header()
	add_child(_header)
	move_child(_header, 0)
	_handle = _build_handle()
	add_child(_handle)
	select_page(0)
	_apply_collapsed()


func page_names() -> PackedStringArray:
	var names := PackedStringArray()
	for page in _pages:
		names.append(str(page.name))
	return names


func select_page(index: int) -> void:
	current = clampi(index, 0, _pages.size() - 1)
	for i in _pages.size():
		_pages[i].visible = i == current and not collapsed
	for i in _tabs.size():
		_tabs[i].set_pressed_no_signal(i == current)
	if _handle_label != null:
		_handle_label.text = _active_name()


func set_meta_text(text: String) -> void:
	if _meta != null:
		_meta.text = text


func set_collapsed(value: bool) -> void:
	if value == collapsed:
		return
	collapsed = value
	_apply_collapsed()
	collapsed_changed.emit(collapsed)


func toggle() -> void:
	set_collapsed(not collapsed)


func set_expanded_width(width: float) -> void:
	expanded_width = width
	_apply_collapsed()


func _active_name() -> String:
	if title != "":
		return title
	return str(_pages[current].name) if current < _pages.size() else ""


func _apply_collapsed() -> void:
	if _header == null:
		return
	_header.visible = not collapsed
	for i in _pages.size():
		_pages[i].visible = i == current and not collapsed
	_handle.visible = collapsed
	custom_minimum_size.x = HANDLE_WIDTH if collapsed else expanded_width


func _build_header() -> Control:
	var header := PanelContainer.new()
	header.theme_type_variation = "DockHeader"
	header.custom_minimum_size.y = HEADER_HEIGHT
	var strip := HBoxContainer.new()
	strip.theme_type_variation = "Strip"
	header.add_child(strip)
	var collapse := _icon_button("Collapse panel", false)
	if right_side:
		strip.add_child(collapse)
		strip.add_child(VSeparator.new())
		var heading := Label.new()
		heading.theme_type_variation = "Heading"
		heading.text = "  " + _active_name()
		strip.add_child(heading)
		var spacer := Control.new()
		spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		strip.add_child(spacer)
		_meta = Label.new()
		_meta.theme_type_variation = "MonoMuted"
		strip.add_child(_meta)
		var gap := Control.new()
		gap.custom_minimum_size.x = 12
		strip.add_child(gap)
	else:
		var group := ButtonGroup.new()
		for i in _pages.size():
			var tab := Button.new()
			tab.theme_type_variation = "TabButton"
			tab.text = str(_pages[i].name)
			tab.toggle_mode = true
			tab.button_group = group
			tab.focus_mode = Control.FOCUS_NONE
			tab.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			tab.clip_text = true
			tab.tooltip_text = tab.text
			tab.pressed.connect(select_page.bind(i))
			strip.add_child(tab)
			_tabs.append(tab)
		strip.add_child(VSeparator.new())
		strip.add_child(collapse)
	return header


func _build_handle() -> Button:
	var handle := Button.new()
	handle.theme_type_variation = "IconButton"
	handle.size_flags_vertical = Control.SIZE_EXPAND_FILL
	handle.tooltip_text = "Expand panel"
	handle.focus_mode = Control.FOCUS_NONE
	handle.pressed.connect(set_collapsed.bind(false))
	var column := VBoxContainer.new()
	column.theme_type_variation = "Section"
	column.mouse_filter = Control.MOUSE_FILTER_IGNORE
	column.set_anchors_preset(Control.PRESET_TOP_WIDE)
	column.offset_top = 12
	column.alignment = BoxContainer.ALIGNMENT_BEGIN
	var icon := DockIcon.new()
	icon.divider_left = not right_side
	icon.points_left = right_side
	icon.size_flags_horizontal = Control.SIZE_SHRINK_CENTER
	column.add_child(icon)
	_handle_label = VerticalLabel.new()
	_handle_label.size_flags_horizontal = Control.SIZE_SHRINK_CENTER
	column.add_child(_handle_label)
	handle.add_child(column)
	handle.mouse_entered.connect(icon.queue_redraw)
	handle.mouse_exited.connect(icon.queue_redraw)
	return handle


func _icon_button(tooltip: String, expand: bool) -> Button:
	var button := Button.new()
	button.theme_type_variation = "IconButton"
	button.tooltip_text = tooltip
	button.focus_mode = Control.FOCUS_NONE
	button.custom_minimum_size = Vector2(HANDLE_WIDTH, HEADER_HEIGHT - 1.0)
	var icon := DockIcon.new()
	icon.divider_left = not right_side
	icon.points_left = not right_side if not expand else right_side
	icon.set_anchors_preset(Control.PRESET_CENTER)
	icon.position = Vector2(10, 11.5)
	button.add_child(icon)
	button.pressed.connect(set_collapsed.bind(true))
	button.mouse_entered.connect(icon.queue_redraw)
	button.mouse_exited.connect(icon.queue_redraw)
	return button
