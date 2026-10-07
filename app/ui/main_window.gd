extends Control

const CAMERA_FEED_WINDOW_SIZE := Vector2i(640, 360)
const LEFT_DOCK_FRACTION := 0.21
const RIGHT_DOCK_FRACTION := 0.225
const LEFT_DOCK_RANGE := Vector2(260.0, 380.0)
const RIGHT_DOCK_RANGE := Vector2(280.0, 400.0)
## Below these widths (in UI pixels, after scaling) the docks fold to their handles
const FOLD_LEFT_BELOW := 1100.0
const FOLD_BOTH_BELOW := 850.0
const MIN_WINDOW := Vector2(720.0, 520.0)
const UI_SCALE_STEPS := [0.75, 0.8, 0.9, 1.0, 1.1, 1.25, 1.4, 1.5, 1.75, 2.0]
const SETTINGS_PATH := "user://ui.cfg"

var _camera_feed_window: Window = null
var _camera_only: bool = false
var _ui_scale := 1.0
var _fold_level := -1

@onready var _left_dock: Dock = $Layout/Body/LeftDock
@onready var _right_dock: Dock = $Layout/Body/RightDock
@onready var _left_rule: Control = $Layout/Body/LeftRule
@onready var _center: Control = $Layout/Body/Center
@onready var _main_view: Control = $Layout/Body/Center/MainView
@onready var _telemetry: Node = $Layout/Body/RightDock/Telemetry/Pad/Sections


func _ready() -> void:
	drop_joypad_ui_events()
	_apply_ui_scale(_initial_ui_scale(), false)
	var feed_panel := camera_panel()
	feed_panel.use_world(_main_view.world_viewport().world_3d)
	_main_view.set_camera_source(feed_panel)
	_telemetry.rate_changed.connect(_right_dock.set_meta_text)
	_right_dock.set_meta_text("waiting for simcore")
	resized.connect(_layout_for_size)
	_layout_for_size.call_deferred()
	var args := OS.get_cmdline_user_args()
	if args.has("--camera-only"):
		_toggle_camera_only.call_deferred()
	# --tab <name> opens a left dock tab, for demos and screenshots
	var tab_index := args.find("--tab")
	if tab_index >= 0 and tab_index + 1 < args.size():
		var page := Array(_left_dock.page_names()).find(args[tab_index + 1])
		if page >= 0:
			_left_dock.select_page.call_deferred(page)
	# --view 1..6 picks a main view mode, the same numbers as the keys
	var view_index := args.find("--view")
	if view_index >= 0 and view_index + 1 < args.size():
		var view := int(args[view_index + 1])
		if view == 6:
			_main_view.show_camera.call_deferred(feed_panel.selected_index())
		elif view >= 1 and view <= 5:
			_main_view.show_world.call_deferred(view - 1)


## Godot maps every gamepad to UI focus navigation by default. The radio plugged in for simcore is
## a gamepad too, so without this its sticks walk the focus around and its buttons press things.
static func drop_joypad_ui_events() -> void:
	for action: StringName in InputMap.get_actions():
		for event in InputMap.action_get_events(action):
			if event is InputEventJoypadButton or event is InputEventJoypadMotion:
				InputMap.action_erase_event(action, event)


func camera_panel() -> Node:
	return _telemetry.camera_panel()


## Dock widths follow the window so nothing overflows on small or tiled windows; widths are in UI
## pixels, so the same rules hold at any UI scale
func _layout_for_size() -> void:
	var width := size.x
	$Layout/TopBar.fit_width(width)
	_left_dock.set_expanded_width(clampf(width * LEFT_DOCK_FRACTION, LEFT_DOCK_RANGE.x, LEFT_DOCK_RANGE.y))
	_right_dock.set_expanded_width(clampf(width * RIGHT_DOCK_FRACTION, RIGHT_DOCK_RANGE.x, RIGHT_DOCK_RANGE.y))
	# fold only when crossing a breakpoint, so a dock opened by hand stays open
	var level := fold_level(width)
	if level != _fold_level and not _camera_only:
		_fold_level = level
		_left_dock.set_collapsed(level >= 1)
		_right_dock.set_collapsed(level >= 2)


static func fold_level(width: float) -> int:
	if width < FOLD_BOTH_BELOW:
		return 2
	return 1 if width < FOLD_LEFT_BELOW else 0


## --ui-scale wins, then the saved choice, then the monitor's own scale
func _initial_ui_scale() -> float:
	var args := OS.get_cmdline_user_args()
	var index := args.find("--ui-scale")
	if index >= 0 and index + 1 < args.size():
		return float(args[index + 1])
	var settings := ConfigFile.new()
	if settings.load(SETTINGS_PATH) == OK:
		return float(settings.get_value("ui", "scale", 1.0))
	return maxf(DisplayServer.screen_get_scale(), 1.0)


static func next_ui_scale(current: float, direction: int) -> float:
	if direction == 0:
		return 1.0
	var steps: Array = UI_SCALE_STEPS
	if direction > 0:
		for step: float in steps:
			if step > current + 0.001:
				return step
		return steps[-1]
	for i in range(steps.size() - 1, -1, -1):
		if steps[i] < current - 0.001:
			return steps[i]
	return steps[0]


func _apply_ui_scale(value: float, save: bool) -> void:
	_ui_scale = clampf(value, UI_SCALE_STEPS[0], UI_SCALE_STEPS[-1])
	var window := get_window()
	window.content_scale_factor = _ui_scale
	window.min_size = Vector2i(MIN_WINDOW * _ui_scale)
	if _camera_feed_window != null:
		_camera_feed_window.content_scale_factor = _ui_scale
	if save:
		var settings := ConfigFile.new()
		settings.load(SETTINGS_PATH)
		settings.set_value("ui", "scale", _ui_scale)
		settings.save(SETTINGS_PATH)
		$Layout/TopBar.show_message("UI scale %d %%  (Ctrl + and Ctrl - to change, Ctrl 0 to reset)" % roundi(_ui_scale * 100.0))


func _unhandled_key_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo:
		return
	if key.ctrl_pressed:
		match key.keycode:
			KEY_EQUAL, KEY_PLUS, KEY_KP_ADD:
				_apply_ui_scale(next_ui_scale(_ui_scale, 1), true)
			KEY_MINUS, KEY_KP_SUBTRACT:
				_apply_ui_scale(next_ui_scale(_ui_scale, -1), true)
			KEY_0, KEY_KP_0:
				_apply_ui_scale(next_ui_scale(_ui_scale, 0), true)
			_:
				return
		get_viewport().set_input_as_handled()
		return
	match key.keycode:
		KEY_TAB:
			_toggle_docks()
		KEY_F:
			_toggle_camera_feed_window()
		KEY_C:
			_toggle_camera_only()


func _toggle_docks() -> void:
	var collapse := not (_left_dock.collapsed and _right_dock.collapsed)
	_left_dock.set_collapsed(collapse)
	_right_dock.set_collapsed(collapse)


## Camera only: the world view stops rendering and the camera feed takes the window, for runs
## where the video is what matters and the GPU should not spend frames on the third person view
func _toggle_camera_only() -> void:
	_camera_only = not _camera_only
	_center.visible = not _camera_only
	_main_view.world_viewport().render_target_update_mode = (
		SubViewport.UPDATE_DISABLED if _camera_only else SubViewport.UPDATE_ALWAYS
	)
	_left_dock.visible = not _camera_only
	_left_rule.visible = not _camera_only
	_right_dock.set_collapsed(false)
	# with the centre gone the telemetry dock takes the rest of the window
	_right_dock.size_flags_horizontal = Control.SIZE_EXPAND_FILL if _camera_only else Control.SIZE_FILL
	if _camera_only and _camera_feed_window == null:
		_toggle_camera_feed_window()


func _toggle_camera_feed_window() -> void:
	if _camera_feed_window != null:
		_camera_feed_window.queue_free()
		_camera_feed_window = null
		return
	_camera_feed_window = Window.new()
	_camera_feed_window.title = "fpvsim camera feed"
	_camera_feed_window.size = Vector2i(Vector2(CAMERA_FEED_WINDOW_SIZE) * _ui_scale)
	_camera_feed_window.content_scale_factor = _ui_scale
	_camera_feed_window.close_requested.connect(_toggle_camera_feed_window)
	var background := Panel.new()
	background.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_camera_feed_window.add_child(background)
	var pad := MarginContainer.new()
	pad.theme_type_variation = "Pad"
	pad.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_camera_feed_window.add_child(pad)
	var panel: Control = load("res://ui/camera_panel.tscn").instantiate()
	pad.add_child(panel)
	add_child(_camera_feed_window)
	panel.mirror(camera_panel())
