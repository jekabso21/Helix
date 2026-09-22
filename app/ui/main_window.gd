extends Control

const CAMERA_FEED_WINDOW_SIZE := Vector2i(640, 360)

var _camera_feed_window: Window = null

@onready var _left_dock: Control = $Layout/Body/LeftDock
@onready var _right_dock: Control = $Layout/Body/Center/RightDock
@onready var _plots: Control = $Layout/Plots


func _unhandled_key_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo:
		return
	match key.keycode:
		KEY_TAB:
			_toggle_docks()
		KEY_F:
			_toggle_camera_feed_window()


func _toggle_docks() -> void:
	var docks_visible := not _left_dock.visible
	_left_dock.visible = docks_visible
	_right_dock.visible = docks_visible
	_plots.visible = docks_visible


func _toggle_camera_feed_window() -> void:
	if _camera_feed_window != null:
		_camera_feed_window.queue_free()
		_camera_feed_window = null
		return
	_camera_feed_window = Window.new()
	_camera_feed_window.title = "fpvsim camera feed"
	_camera_feed_window.size = CAMERA_FEED_WINDOW_SIZE
	_camera_feed_window.close_requested.connect(_toggle_camera_feed_window)
	var panel: Control = load("res://ui/camera_panel.tscn").instantiate()
	panel.set("world_view_path", NodePath(""))
	panel.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_camera_feed_window.add_child(panel)
	add_child(_camera_feed_window)
	var main_viewport: SubViewport = $Layout/Body/Center/MainView/SubViewport
	var feed_viewport: SubViewport = panel.get_node("Container/SubViewport")
	feed_viewport.world_3d = main_viewport.world_3d
