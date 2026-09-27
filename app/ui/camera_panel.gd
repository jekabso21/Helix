extends VBoxContainer
## The camera feed: rendered at the configured resolution and published to the frame ring,
## with an optional raw RGB output over TCP for GStreamer

const FALLBACK_MOUNT_FRD := Vector3(0.06, 0.0, -0.02)
const FALLBACK_UPTILT_DEG := 35.0
const FALLBACK_SIZE := Vector2i(640, 360)
const FALLBACK_HFOV_DEG := 110.0
const RAW_PORT := 5700

var _raw_port: int = RAW_PORT

var camera_name := "main_fpv"
var camera_size := FALLBACK_SIZE
var camera_fps := 60.0
var mount_position_frd := FALLBACK_MOUNT_FRD
var mount_q_frd_from_camera := Quaternion(Vector3(0, 1, 0), deg_to_rad(FALLBACK_UPTILT_DEG))

var _publisher: Variant = null
var _shown: ImageTexture = null
var _shown_image: Image = null
var _frame_index := 0
var _next_publish_ns := 0
var _published := 0
var _publish_error := ""
var _run_dir := ""

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
	_apply_camera(FALLBACK_SIZE, FALLBACK_HFOV_DEG)
	_view.texture = _viewport.get_texture()
	_camera.current = true
	BackendClient.status.connect(_on_status)
	var args := OS.get_cmdline_user_args()
	var port_index := args.find("--raw-port")
	if port_index >= 0 and port_index + 1 < args.size():
		_raw_port = int(args[port_index + 1])
	_raw_toggle.toggled.connect(_on_raw_toggled)
	if args.has("--raw-video"):
		_raw_toggle.button_pressed = true
	# sync=false: frames are shown as they arrive; with the clock, every dropped frame would add lag
	_update_command()


## Godot's fov is vertical unless the camera keeps its width, which is what an hfov means here
func _apply_camera(size: Vector2i, hfov_deg: float) -> void:
	camera_size = size
	_viewport.size = size
	_camera.keep_aspect = Camera3D.KEEP_WIDTH
	_camera.fov = hfov_deg


func _update_command() -> void:
	# sync=false: frames are shown as they arrive; with the clock, every dropped frame would add lag
	_command.text = "gst-launch-1.0 tcpclientsrc host=127.0.0.1 port=%d ! rawvideoparse format=rgb width=%d height=%d framerate=%d/1 ! videoconvert ! autovideosink sync=false" % [_raw_port, camera_size.x, camera_size.y, roundi(camera_fps)]


func _on_status(data: Dictionary) -> void:
	var run_dir := str(data.get("run_dir", ""))
	var running: bool = data.get("state", "") == "running" and run_dir != ""
	if not running:
		_close_publisher()
		_run_dir = ""
		return
	if run_dir == _run_dir:
		return
	_run_dir = run_dir
	_load_camera(run_dir)


func _load_camera(run_dir: String) -> void:
	var cameras: Variant = _read_json(run_dir + "/resolved/cameras.json")
	if not (cameras is Dictionary) or (cameras as Dictionary).get("cameras", []).is_empty():
		return
	var camera: Dictionary = (cameras as Dictionary)["cameras"][0]
	camera_name = str(camera["name"])
	camera_fps = float(camera["fps"])
	var optics: Dictionary = camera.get("optics", {})
	var hfov_deg: float = rad_to_deg(float(optics.get("hfov_rad", deg_to_rad(FALLBACK_HFOV_DEG))))
	_apply_camera(Vector2i(int(camera["width"]), int(camera["height"])), hfov_deg)
	_load_mount(run_dir, camera_name)
	_update_command()
	_open_publisher()


## The mount comes from the compiled model, so moving the camera part moves the view
func _load_mount(run_dir: String, wanted: String) -> void:
	var drone: Variant = _read_json(run_dir + "/resolved/drone.json")
	if not (drone is Dictionary):
		return
	for entry: Dictionary in (drone as Dictionary).get("cameras", []):
		if str(entry.get("name", "")) != wanted:
			continue
		var p: Array = entry["position_frd_m"]
		mount_position_frd = Vector3(p[0], p[1], p[2])
		var q: Array = entry["q_frd_from_camera"]
		mount_q_frd_from_camera = Quaternion(q[1], q[2], q[3], q[0])  # stored (w, x, y, z)
		return


func _read_json(path: String) -> Variant:
	var text := FileAccess.get_file_as_string(path)
	return JSON.parse_string(text) if text != "" else null


func _open_publisher() -> void:
	_close_publisher()
	if not ClassDB.class_exists("FramePublisher"):
		_publish_error = "extension not built"
		return
	_publisher = ClassDB.instantiate("FramePublisher")
	if not _publisher.open(camera_name, camera_size.x, camera_size.y, camera_fps):
		_publish_error = str(_publisher.last_error())
		_publisher = null
		return
	_publish_error = ""
	_frame_index = 0
	_published = 0
	_next_publish_ns = 0


func _close_publisher() -> void:
	if _publisher != null:
		_publisher.close()
		_publisher = null
	_shown = null
	_shown_image = null
	_view.texture = _viewport.get_texture()


func _exit_tree() -> void:
	if _raw != null:
		_raw.stop()
	_close_publisher()


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
		var mount := Basis(rotation) * Frames.godot_body_from_frd(mount_position_frd)
		_camera.global_position = state.godot_position() + mount
		_camera.quaternion = rotation * Frames.godot_quat_from_frd(mount_q_frd_from_camera)
		time_text = "t %.1f s" % (state.sim_time_ns * 1e-9)

	var wants_publish := _publisher != null and state != null and state.sim_time_ns >= _next_publish_ns
	var status := ""
	if _raw != null or wants_publish:
		var pixels := _viewport.get_texture().get_image().get_data()
		if _raw != null:
			_raw.push(pixels)
			status = "  raw: %d written, %d dropped%s" % [
				_raw.frames_written, _raw.frames_dropped, "  " + _raw.error if _raw.error != "" else ""
			]
		if wants_publish:
			_publish(pixels, state)
			_show_published(pixels)
	if _publisher != null:
		status += "  ring: %d frames" % _published
	elif _publish_error != "":
		status += "  ring: " + _publish_error
	_header.text = "%s %dx%d@%d  %s%s" % [
		camera_name, camera_size.x, camera_size.y, roundi(camera_fps), time_text, status
	]


## The panel shows the bytes that went to the outputs, so what you see is what they receive
func _show_published(pixels: PackedByteArray) -> void:
	if _shown_image == null or _shown_image.get_width() != camera_size.x or _shown_image.get_height() != camera_size.y:
		_shown_image = Image.create_from_data(camera_size.x, camera_size.y, false, Image.FORMAT_RGB8, pixels)
		_shown = ImageTexture.create_from_image(_shown_image)
		_view.texture = _shown
		return
	_shown_image.set_data(camera_size.x, camera_size.y, false, Image.FORMAT_RGB8, pixels)
	_shown.update(_shown_image)


## Frames carry the sim time they represent and the camera pose that produced them
func _publish(pixels: PackedByteArray, state: RenderState) -> void:
	var q_ned_from_frd := Quaternion(state.q_x, state.q_y, state.q_z, state.q_w)
	var position_ned := state.position_ned + q_ned_from_frd * mount_position_frd
	var q_ned_from_camera := q_ned_from_frd * mount_q_frd_from_camera
	if _publisher.publish(pixels, state.sim_time_ns, _frame_index, position_ned, q_ned_from_camera) != 0:
		_published += 1
	_frame_index += 1
	var period_ns := int(1e9 / maxf(camera_fps, 1.0))
	# never chase a backlog after a pause or a reset
	_next_publish_ns = maxi(state.sim_time_ns + 1, _next_publish_ns + period_ns)
