extends VBoxContainer
## FPV preview rendered at the camera resolution; optional raw RGB output over TCP for GStreamer

const CAMERA_MOUNT_FRD := Vector3(0.06, 0.0, -0.02)
const CAMERA_UPTILT_DEG := 35.0
const CAMERA_SIZE := Vector2i(640, 360)
const RAW_PORT := 5700

var _raw_port: int = RAW_PORT

@export var world_view_path: NodePath

@onready var _header: Label = $Header
@onready var _viewport: SubViewport = $SubViewport
@onready var _camera: Camera3D = $SubViewport/Camera3D
@onready var _view: TextureRect = $View
@onready var _raw_toggle: CheckButton = $OutputRow/RawTcp
@onready var _command: LineEdit = $OutputRow/Command

var _raw: RawVideoOut = null


func _ready() -> void:
	var world_view := get_node_or_null(world_view_path)
	if world_view != null:
		var main_viewport: SubViewport = world_view.get_node("SubViewport")
		_viewport.world_3d = main_viewport.world_3d
	_viewport.size = CAMERA_SIZE
	_view.texture = _viewport.get_texture()
	_camera.current = true
	var args := OS.get_cmdline_user_args()
	var port_index := args.find("--raw-port")
	if port_index >= 0 and port_index + 1 < args.size():
		_raw_port = int(args[port_index + 1])
	_raw_toggle.toggled.connect(_on_raw_toggled)
	if args.has("--raw-video"):
		_raw_toggle.button_pressed = true
	# sync=false: frames are shown as they arrive; with the clock, every dropped frame would add lag
	_command.text = "gst-launch-1.0 tcpclientsrc host=127.0.0.1 port=%d ! rawvideoparse format=rgb width=%d height=%d framerate=60/1 ! videoconvert ! autovideosink sync=false" % [_raw_port, CAMERA_SIZE.x, CAMERA_SIZE.y]


func _exit_tree() -> void:
	if _raw != null:
		_raw.stop()


func _on_raw_toggled(on: bool) -> void:
	if on:
		_raw = RawVideoOut.new()
		if not _raw.start(_raw_port):
			_header.text = _raw.error
			_raw = null
			_raw_toggle.set_pressed_no_signal(false)
	elif _raw != null:
		_raw.stop()
		_raw = null


func _process(_delta: float) -> void:
	var state := SimLink.last_state
	var time_text := "no state yet"
	if state != null:
		var rotation := state.godot_rotation()
		var mount := Basis(rotation) * Frames.godot_body_from_frd(CAMERA_MOUNT_FRD)
		_camera.global_position = state.godot_position() + mount
		var tilt := Quaternion(Vector3.RIGHT, deg_to_rad(CAMERA_UPTILT_DEG))
		_camera.quaternion = rotation * tilt
		time_text = "t %.1f s" % (state.sim_time_ns * 1e-9)
	var status := ""
	if _raw != null:
		_raw.push(_viewport.get_texture().get_image().get_data())
		status = "  raw: %d written, %d dropped%s" % [
			_raw.frames_written, _raw.frames_dropped, "  " + _raw.error if _raw.error != "" else ""
		]
	_header.text = "main_fpv %dx%d  %s%s" % [CAMERA_SIZE.x, CAMERA_SIZE.y, time_text, status]
