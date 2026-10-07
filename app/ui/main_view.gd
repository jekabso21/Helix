extends VBoxContainer
## Main view: the world from the camera rig or a published camera feed, the other one as a
## picture-in-picture, HUD chips over it and the motor outputs under it

var _camera_panel: Node = null
var _feed_main := false
var _tabs: Array[Button] = []
var _group := ButtonGroup.new()
var _motor_bars: Array[LevelBar] = []
var _motor_values: Array[Label] = []

@onready var _tab_row: HBoxContainer = $ViewBar/Segment/Tabs
@onready var _meta: Label = $ViewBar/Meta
@onready var _stage: Control = $Frame/Stage
@onready var _container: SubViewportContainer = $Frame/Stage/Viewport
@onready var _viewport: SubViewport = $Frame/Stage/Viewport/SubViewport
@onready var _feed: TextureRect = $Frame/Stage/Feed
@onready var _backdrop: ColorRect = $Frame/Stage/FeedBackdrop
@onready var _view_chip: Label = $Frame/Stage/Hud/Row/View/Text
@onready var _armed_chip: Label = $Frame/Stage/Hud/Row/Armed/Text
@onready var _alt_chip: Label = $Frame/Stage/Hud/Row/Alt/Text
@onready var _speed_chip: Label = $Frame/Stage/Hud/Row/Speed/Text
@onready var _wind_chip: Label = $Frame/Stage/Hud/Row/Wind/Text
@onready var _failures_chip: PanelContainer = $Frame/Stage/Hud/Row/Failures
@onready var _failures_text: Label = $Frame/Stage/Hud/Row/Failures/Text
@onready var _pip: Button = $Frame/Stage/Pip
@onready var _pip_picture: TextureRect = $Frame/Stage/Pip/Picture
@onready var _pip_name: Label = $Frame/Stage/Pip/Name/Text
@onready var _motors: GridContainer = $Motors
@onready var _rig: Node3D = $Frame/Stage/Viewport/SubViewport/World/CameraRig


func _ready() -> void:
	_rig.mode_changed.connect(func(_mode: int) -> void: _refresh())
	_pip.pressed.connect(swap)
	SimLink.telemetry.connect(_on_telemetry)
	_build_motors(4)
	_rebuild_tabs()
	var arrow := Node3D.new()
	arrow.set_script(preload("res://scripts/wind_arrow.gd"))
	arrow.name = "WindArrow"
	$Frame/Stage/Viewport/SubViewport/World.add_child(arrow)
	resized.connect(_fit_motor_columns)
	_container.set_anchors_preset(Control.PRESET_TOP_LEFT)
	_stage.resized.connect(_fit_viewport)
	_fit_viewport()


## The UI may be scaled up; the 3D view still renders one pixel per screen pixel, so it is drawn
## at screen size and scaled back down into the stage
func _fit_viewport() -> void:
	var factor := get_window().content_scale_factor if is_inside_tree() else 1.0
	_container.position = Vector2.ZERO
	_container.scale = Vector2.ONE / factor
	_container.size = _stage.size * factor


func world_viewport() -> SubViewport:
	return _viewport


## The camera panel renders and publishes the feeds; this view only shows their textures
func set_camera_source(panel: Node) -> void:
	_camera_panel = panel
	panel.feeds_changed.connect(_rebuild_tabs)
	panel.selection_changed.connect(func(_index: int) -> void: _refresh())
	_rebuild_tabs()


func showing_feed() -> bool:
	return _feed_main and _selected_feed() != null


func show_world(mode: int = -1) -> void:
	_feed_main = false
	if mode >= 0 and mode != _rig.mode:
		_rig.set_mode(mode)
	_refresh()


func show_camera(index: int) -> void:
	if _camera_panel == null:
		return
	_feed_main = true
	if index != _camera_panel.selected_index():
		_camera_panel.select_camera(index)
	_refresh()


func swap() -> void:
	if _feed_main:
		show_world()
	elif _selected_feed() != null:
		show_camera(_camera_panel.selected_index())


func _unhandled_key_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo:
		return
	match key.keycode:
		KEY_1, KEY_2, KEY_3, KEY_4, KEY_5:
			show_world(key.keycode - KEY_1)
		KEY_6:
			if _camera_panel != null:
				show_camera(_camera_panel.selected_index())
		KEY_V:
			swap()
		_:
			return
	get_viewport().set_input_as_handled()


func _selected_feed() -> Variant:
	return _camera_panel.selected_feed() if _camera_panel != null else null


func _world_name() -> String:
	var names: Array = _rig.MODE_NAMES
	# the rig's own FPV mode is never picked from here; the camera tabs show the real feed
	return names[_rig.mode] if _rig.mode != _rig.Mode.FPV else names[0]


func _rebuild_tabs() -> void:
	for child in _tab_row.get_children():
		child.queue_free()
	_tabs.clear()
	var labels: Array[String] = [_world_name()]
	if _camera_panel != null:
		for name: String in _camera_panel.camera_names():
			labels.append("FPV " + name)
	for i in labels.size():
		if i > 0:
			_tab_row.add_child(VSeparator.new())
		var tab := Button.new()
		tab.theme_type_variation = "SegButton"
		tab.text = labels[i]
		tab.toggle_mode = true
		tab.button_group = _group
		tab.focus_mode = Control.FOCUS_NONE
		if i == 0:
			tab.pressed.connect(show_world)
		else:
			tab.pressed.connect(show_camera.bind(i - 1))
		_tab_row.add_child(tab)
		_tabs.append(tab)
	_refresh()


func _refresh() -> void:
	if not is_node_ready():
		return
	var feed: Variant = _selected_feed()
	var feed_main := _feed_main and feed != null
	_feed.visible = feed_main
	# the feed keeps its aspect, so the rest of the view is black rather than the world behind it
	_backdrop.visible = feed_main
	_feed.texture = feed.post_viewport.get_texture() if feed_main else null
	if not _tabs.is_empty():
		_tabs[0].text = _world_name()
		var pressed := 0
		if feed_main:
			pressed = 1 + int(_camera_panel.selected_index())
		for i in _tabs.size():
			_tabs[i].set_pressed_no_signal(i == pressed)
	if feed_main:
		_view_chip.text = "FPV · %s" % feed.name
		_meta.text = "%s · %d×%d@%d" % [feed.name, feed.size.x, feed.size.y, roundi(feed.fps)]
	else:
		_view_chip.text = _world_name().to_upper()
		_meta.text = "%s · mode %d · fov %d°" % [_world_name().to_lower(), _rig.mode + 1, roundi(_rig.camera.fov)]
	_pip.visible = feed != null
	if feed != null:
		_pip_picture.texture = _viewport.get_texture() if feed_main else feed.post_viewport.get_texture()
		_pip_name.text = _world_name().to_upper() if feed_main else "FPV"


func _process(_delta: float) -> void:
	var state := SimLink.last_state
	if state == null:
		_armed_chip.text = "NO STATE"
	elif state.crashed:
		_armed_chip.text = "CRASHED"
	else:
		_armed_chip.text = "ARMED" if state.armed else "DISARMED"
	# a rebuilt feed has a new texture
	if _feed.visible:
		var feed: Variant = _selected_feed()
		if feed != null and _feed.texture != feed.post_viewport.get_texture():
			_refresh()


## Four motor cells side by side need room; a narrow view stacks them two by two
func _fit_motor_columns() -> void:
	if _motor_bars.is_empty():
		return
	var columns := _motor_bars.size() if size.x >= 120.0 * _motor_bars.size() else maxi(1, ceili(_motor_bars.size() / 2.0))
	if _motors.columns != columns:
		_motors.columns = columns


func _on_telemetry(data: Dictionary) -> void:
	var flight: Dictionary = data["flight"]
	_alt_chip.text = "ALT %.2f m" % float(flight["altitude_agl_m"])
	_speed_chip.text = "SPD %.1f m/s" % float(flight["ground_speed_mps"])
	var environment: Variant = data.get("environment")
	if environment is Dictionary:
		var wind: Array = (environment as Dictionary)["wind_ned_mps"]
		var speed := Vector2(float(wind[0]), float(wind[1])).length()
		var from_deg := fposmod(rad_to_deg(atan2(-float(wind[1]), -float(wind[0]))), 360.0)
		_wind_chip.text = "WIND %.1f m/s %03d°" % [speed, roundi(from_deg)] if speed >= 0.05 else "WIND calm"
	var failing: Array = data["sim"].get("active_failures", []).filter(func(f: Dictionary) -> bool: return bool(f.get("in_effect", true)))
	_failures_chip.visible = not failing.is_empty()
	_failures_text.text = "%d FAILURE%s" % [failing.size(), "" if failing.size() == 1 else "S"]
	var motors: Array = data["motors"]
	if motors.size() != _motor_bars.size() and not motors.is_empty():
		_build_motors(motors.size())
	for i in mini(motors.size(), _motor_bars.size()):
		var command := float(motors[i]["command"])
		_motor_bars[i].value = command
		_motor_values[i].text = "%d%%" % roundi(command * 100.0)


func _build_motors(count: int) -> void:
	for child in _motors.get_children():
		child.queue_free()
	_motor_bars.clear()
	_motor_values.clear()
	_motors.columns = count
	for i in count:
		var cell := PanelContainer.new()
		cell.theme_type_variation = "Row"
		cell.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		var row := HBoxContainer.new()
		row.theme_type_variation = "Gap10"
		cell.add_child(row)
		var name := Label.new()
		name.theme_type_variation = "Kicker"
		name.text = "M%d" % (i + 1)
		row.add_child(name)
		var bar := LevelBar.new()
		bar.custom_minimum_size.y = 6
		bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		bar.size_flags_vertical = Control.SIZE_SHRINK_CENTER
		row.add_child(bar)
		var value := Label.new()
		value.theme_type_variation = "Mono"
		value.text = "0%"
		value.custom_minimum_size.x = 34
		value.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		row.add_child(value)
		_motors.add_child(cell)
		_motor_bars.append(bar)
		_motor_values.append(value)
	_fit_motor_columns()
