extends Control

const CAMERA_FEED_WINDOW_SIZE := Vector2i(640, 360)

var _camera_feed_window: Window = null
var _camera_only: bool = false

const LEFT_DOCK_FRACTION := 0.22
const RIGHT_DOCK_FRACTION := 0.32

@onready var _body: HSplitContainer = $Margin/Layout/Body
@onready var _center: HSplitContainer = $Margin/Layout/Body/Center
@onready var _left_dock: Control = $Margin/Layout/Body/LeftDock
@onready var _right_dock: Control = $Margin/Layout/Body/Center/RightDock
@onready var _plots: Control = $Margin/Layout/Plots


func _ready() -> void:
	var camera_panel := get_node_or_null("Margin/Layout/Body/Center/RightDock/CameraFeed")
	if camera_panel != null:
		$Margin/Layout/Body/Center/MainView.set_camera_source(camera_panel)
	resized.connect(_layout_for_size)
	_layout_for_size.call_deferred()
	var args := OS.get_cmdline_user_args()
	if args.has("--camera-only"):
		_toggle_camera_only.call_deferred()
	# --view 1..6 picks a main view mode, the same numbers as the keys
	var view_index := args.find("--view")
	if view_index >= 0 and view_index + 1 < args.size():
		var rig := get_node_or_null("Margin/Layout/Body/Center/MainView/SubViewport/World/CameraRig")
		if rig != null:
			rig.set_mode.call_deferred(int(args[view_index + 1]) - 1)


## Dock widths follow the window so nothing overflows on small or tiled windows
func _layout_for_size() -> void:
	var width := size.x
	_body.split_offset = int(width * LEFT_DOCK_FRACTION)
	_center.split_offset = int(-width * RIGHT_DOCK_FRACTION)


func _unhandled_key_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo:
		return
	match key.keycode:
		KEY_TAB:
			_toggle_docks()
		KEY_F:
			_toggle_camera_feed_window()
		KEY_C:
			_toggle_camera_only()


func _toggle_docks() -> void:
	var docks_visible := not _left_dock.visible
	_left_dock.visible = docks_visible
	_right_dock.visible = docks_visible
	_plots.visible = docks_visible


## Camera only: the world view stops rendering and the camera feed takes the window, for runs
## where the video is what matters and the GPU should not spend frames on the third person view
func _toggle_camera_only() -> void:
	var main_view: Control = $Margin/Layout/Body/Center/MainView
	var main_viewport: SubViewport = $Margin/Layout/Body/Center/MainView/SubViewport
	_camera_only = not _camera_only
	main_view.visible = not _camera_only
	main_viewport.render_target_update_mode = (
		SubViewport.UPDATE_DISABLED if _camera_only else SubViewport.UPDATE_ALWAYS
	)
	_left_dock.visible = not _camera_only
	_plots.visible = not _camera_only
	_right_dock.visible = true
	if _camera_only and _camera_feed_window == null:
		_toggle_camera_feed_window()


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
	var main_viewport: SubViewport = $Margin/Layout/Body/Center/MainView/SubViewport
	panel.use_world(main_viewport.world_3d)
