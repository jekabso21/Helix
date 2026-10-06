extends VBoxContainer
## Every configured camera is rendered at its own resolution and published to its frame ring;
## the panel shows the one picked in the selector, with an optional raw RGB output over TCP

const OSD_OVERLAY := preload("res://ui/osd_overlay.gd")
const POST_SHADER := preload("res://shaders/camera_post.gdshader")
const FALLBACK_MOUNT_FRD := Vector3(0.06, 0.0, -0.02)
const FALLBACK_UPTILT_DEG := 35.0
const FALLBACK_SIZE := Vector2i(640, 360)
const FALLBACK_HFOV_DEG := 110.0
const RAW_PORT := 5700


## One camera: the two render stages, the ring it publishes to and the optics it applies
class Feed:
	var name := "main_fpv"
	var size := FALLBACK_SIZE
	var fps := 60.0
	var half_hfov_rad := deg_to_rad(FALLBACK_HFOV_DEG) * 0.5
	var mount_position_frd := FALLBACK_MOUNT_FRD
	var mount_q_frd_from_camera := Quaternion(Vector3(0, 1, 0), deg_to_rad(FALLBACK_UPTILT_DEG))
	var distortion_k := Vector4.ZERO
	var rolling_shutter_s := 0.0
	var shutter_s := 0.0
	var noise_base := 0.0
	var burn_in := false
	var osd := true
	var viewport: SubViewport = null
	var camera: Camera3D = null
	var post_viewport: SubViewport = null
	var post: TextureRect = null
	var publisher: Variant = null
	var publish_error := ""
	var frame_index := 0
	var next_publish_ns := 0
	var published := 0

	func aspect() -> float:
		return float(size.x) / float(size.y)

	func pixels() -> PackedByteArray:
		return post_viewport.get_texture().get_image().get_data()


var _raw_port: int = RAW_PORT
var _feeds: Array[Feed] = []
var _selected := 0
var _shown: ImageTexture = null
var _shown_image: Image = null
var _run_dir := ""
var _world: World3D = null
var _raw: RawVideoOut = null
var _source: Node = null

@export var world_view_path: NodePath

@onready var _header: Label = $HeaderRow/Header
@onready var _selector: OptionButton = $HeaderRow/Camera
@onready var _feed_root: Node = $Feeds
@onready var _view: TextureRect = $View
@onready var _outputs: VBoxContainer = $Outputs
@onready var _raw_toggle: CheckButton = $OutputRow/RawTcp
@onready var _command: LineEdit = $OutputRow/Command


func _ready() -> void:
	var world_view := get_node_or_null(world_view_path)
	if world_view != null:
		var main_viewport: SubViewport = world_view.get_node("SubViewport")
		_world = main_viewport.world_3d
	_build_feeds([{}])
	BackendClient.status.connect(_on_status)
	BackendClient.video.connect(_on_video)
	var args := OS.get_cmdline_user_args()
	var port_index := args.find("--raw-port")
	if port_index >= 0 and port_index + 1 < args.size():
		_raw_port = int(args[port_index + 1])
	_raw_toggle.toggled.connect(_on_raw_toggled)
	_selector.item_selected.connect(_on_camera_selected)
	if args.has("--raw-video"):
		_raw_toggle.button_pressed = true
	_update_command()


## A pop-out copy of the panel renders the same world as the main window
func use_world(world: World3D) -> void:
	_world = world
	for feed in _feeds:
		feed.viewport.world_3d = world


## The pop-out window shows what another panel renders instead of rendering the cameras again:
## a second set of feeds would also open a second publisher on every ring
func mirror(source: Node) -> void:
	_source = source
	_free_feeds()
	if _raw != null:
		_raw.stop()
		_raw = null
	_selector.visible = false
	_outputs.visible = false
	$OutputRow.visible = false


func _exit_tree() -> void:
	if _raw != null:
		_raw.stop()
	_free_feeds()


## The panel renders a plain forward camera until a session says what the cameras are
func _build_feeds(configs: Array) -> void:
	_free_feeds()
	for config: Dictionary in configs:
		_feeds.append(_make_feed(config))
	_selected = clampi(_selected, 0, _feeds.size() - 1)
	_selector.clear()
	for feed in _feeds:
		_selector.add_item(feed.name)
	_selector.select(_selected)
	_selector.visible = _feeds.size() > 1
	_show_feed_texture()
	_update_command()


func _make_feed(config: Dictionary) -> Feed:
	var feed := Feed.new()
	if not config.is_empty():
		feed.name = str(config["name"])
		feed.size = Vector2i(int(config["width"]), int(config["height"]))
		feed.fps = float(config["fps"])
		feed.burn_in = bool(config.get("burn_in_counter", false))
		var optics: Dictionary = config.get("optics", {})
		feed.half_hfov_rad = float(optics.get("hfov_rad", deg_to_rad(FALLBACK_HFOV_DEG))) * 0.5
		var distortion: Dictionary = optics.get("distortion", {})
		if str(distortion.get("model", "none")) == "fisheye":
			var k: Array = distortion.get("k", [0, 0, 0, 0])
			feed.distortion_k = Vector4(k[0], k[1], k[2], k[3])
		feed.rolling_shutter_s = float(optics.get("rolling_shutter_readout_s", 0.0))
		feed.shutter_s = float((optics.get("exposure", {}) as Dictionary).get("shutter_s", 0.0))
		feed.noise_base = float((optics.get("noise", {}) as Dictionary).get("base", 0.0))
		feed.osd = bool(config.get("osd", true))

	feed.viewport = SubViewport.new()
	feed.viewport.size = feed.size
	feed.viewport.own_world_3d = false
	feed.viewport.handle_input_locally = false
	feed.viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	if _world != null:
		feed.viewport.world_3d = _world
	feed.camera = Camera3D.new()
	# Godot's fov is vertical unless the camera keeps its width, which is what an hfov means here
	feed.camera.keep_aspect = Camera3D.KEEP_WIDTH
	feed.camera.fov = rad_to_deg(feed.half_hfov_rad) * 2.0
	feed.viewport.add_child(feed.camera)
	if feed.osd:
		var overlay := Control.new()
		overlay.set_script(OSD_OVERLAY)
		overlay.set_anchors_preset(Control.PRESET_FULL_RECT)
		feed.viewport.add_child(overlay)

	feed.post_viewport = SubViewport.new()
	feed.post_viewport.size = feed.size
	feed.post_viewport.own_world_3d = true
	feed.post_viewport.disable_3d = true
	feed.post_viewport.handle_input_locally = false
	feed.post_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	feed.post = TextureRect.new()
	feed.post.texture = feed.viewport.get_texture()
	feed.post.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	feed.post.stretch_mode = TextureRect.STRETCH_SCALE
	feed.post.set_anchors_preset(Control.PRESET_FULL_RECT)
	var material := ShaderMaterial.new()
	material.shader = POST_SHADER
	material.set_shader_parameter("half_hfov_rad", feed.half_hfov_rad)
	material.set_shader_parameter("aspect", feed.aspect())
	feed.post.material = material
	feed.post_viewport.add_child(feed.post)

	_feed_root.add_child(feed.viewport)
	_feed_root.add_child(feed.post_viewport)
	feed.camera.current = true
	return feed


func _free_feeds() -> void:
	for feed in _feeds:
		_close_publisher(feed)
		feed.viewport.queue_free()
		feed.post_viewport.queue_free()
	_feeds.clear()
	_shown = null
	_shown_image = null


func selected_feed() -> Feed:
	return _feeds[_selected] if _selected < _feeds.size() else null


func _on_camera_selected(index: int) -> void:
	_selected = clampi(index, 0, _feeds.size() - 1)
	_shown = null
	_shown_image = null
	_show_feed_texture()
	_update_command()
	_clear_outputs()


## Without published bytes to show, the panel falls back to the feed's own post-processed texture
func _show_feed_texture() -> void:
	var feed := selected_feed()
	if feed != null:
		_view.texture = feed.post_viewport.get_texture()


func _update_command() -> void:
	var feed := selected_feed()
	if feed == null:
		return
	# sync=false: frames are shown as they arrive; with the clock, every dropped frame would add lag
	_command.text = "gst-launch-1.0 tcpclientsrc host=127.0.0.1 port=%d ! rawvideoparse format=rgb width=%d height=%d framerate=%d/1 ! videoconvert ! autovideosink sync=false" % [_raw_port, feed.size.x, feed.size.y, roundi(feed.fps)]


func _on_status(data: Dictionary) -> void:
	if _source != null:
		return
	var run_dir := str(data.get("run_dir", ""))
	var running: bool = data.get("state", "") == "running" and run_dir != ""
	if not running:
		_build_feeds([{}])
		_clear_outputs()
		_run_dir = ""
		return
	if run_dir == _run_dir:
		return
	_run_dir = run_dir
	_load_cameras(run_dir)


func _load_cameras(run_dir: String) -> void:
	var cameras: Variant = _read_json(run_dir + "/resolved/cameras.json")
	if not (cameras is Dictionary) or (cameras as Dictionary).get("cameras", []).is_empty():
		return
	_build_feeds((cameras as Dictionary)["cameras"])
	var drone: Variant = _read_json(run_dir + "/resolved/drone.json")
	for feed in _feeds:
		var mount := mount_from_drone(drone, feed.name)
		if not mount.is_empty():
			feed.mount_position_frd = mount["position_frd"]
			feed.mount_q_frd_from_camera = mount["q_frd_from_camera"]
		_open_publisher(feed)


## The mount comes from the compiled model, so moving the camera part moves the view
static func mount_from_drone(drone: Variant, wanted: String) -> Dictionary:
	if not (drone is Dictionary):
		return {}
	for entry: Dictionary in (drone as Dictionary).get("cameras", []):
		if str(entry.get("name", "")) != wanted:
			continue
		var p: Array = entry["position_frd_m"]
		var q: Array = entry["q_frd_from_camera"]  # stored (w, x, y, z)
		return {
			"position_frd": Vector3(p[0], p[1], p[2]),
			"q_frd_from_camera": Quaternion(q[1], q[2], q[3], q[0]),
		}
	return {}


func _read_json(path: String) -> Variant:
	var text := FileAccess.get_file_as_string(path)
	return JSON.parse_string(text) if text != "" else null


## simvideo reports every output twice a second; a failed one is named without hiding the others,
## and the switch on each row turns that output off or on while the session runs
func _on_video(data: Dictionary) -> void:
	if _source != null:
		return
	var feed := selected_feed()
	if feed == null or str(data.get("camera", "")) != feed.name:
		return
	var outputs: Array = data.get("outputs", [])
	while _outputs.get_child_count() < outputs.size():
		_outputs.add_child(_make_output_row(_outputs.get_child_count()))
	while _outputs.get_child_count() > outputs.size():
		var extra := _outputs.get_child(_outputs.get_child_count() - 1)
		_outputs.remove_child(extra)
		extra.queue_free()
	for i in outputs.size():
		var output: Dictionary = outputs[i]
		var row := _outputs.get_child(i)
		var toggle: CheckButton = row.get_child(0)
		var label: Label = row.get_child(1)
		toggle.set_pressed_no_signal(str(output.get("state", "")) != "disabled")
		label.text = output_summary(output)
		label.tooltip_text = str(output.get("pipeline", ""))
		label.add_theme_color_override("font_color", output_colour(str(output.get("state", ""))))


func _make_output_row(index: int) -> HBoxContainer:
	var row := HBoxContainer.new()
	var toggle := CheckButton.new()
	toggle.tooltip_text = "Run this output"
	toggle.toggled.connect(func(on: bool) -> void: _set_output_enabled(index, on))
	row.add_child(toggle)
	var label := Label.new()
	label.clip_text = true
	label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(label)
	return row


func _set_output_enabled(index: int, enabled: bool) -> void:
	var feed := selected_feed()
	if feed == null:
		return
	BackendClient.request("set_output_enabled", {
		"camera": feed.name, "index": index, "enabled": enabled
	})


static func output_summary(output: Dictionary) -> String:
	var text := "out %d  %s  %.1f fps" % [
		int(output.get("index", 0)), str(output.get("state", "")), float(output.get("fps", 0.0))
	]
	var bitrate: Variant = output.get("bitrate_bps")
	if bitrate != null:
		text += "  %.1f Mbit/s" % (float(bitrate) * 1e-6)
	var error := str(output.get("last_error", ""))
	if error != "" and error != "<null>":
		text += "  " + error
	return text


static func output_colour(state: String) -> Color:
	match state:
		"error":
			return Color(1.0, 0.45, 0.4)
		"starting", "restarting":
			return Color(1.0, 0.85, 0.4)
		"disabled":
			return Color(0.6, 0.6, 0.6)
		_:
			return Color(0.8, 0.9, 0.8)


func _clear_outputs() -> void:
	for child in _outputs.get_children():
		_outputs.remove_child(child)
		child.queue_free()


func _open_publisher(feed: Feed) -> void:
	_close_publisher(feed)
	if not ClassDB.class_exists("FramePublisher"):
		feed.publish_error = "extension not built"
		return
	feed.publisher = ClassDB.instantiate("FramePublisher")
	if not feed.publisher.open(feed.name, feed.size.x, feed.size.y, feed.fps):
		feed.publish_error = str(feed.publisher.last_error())
		feed.publisher = null
		return
	feed.publisher.set_burn_in_counter(feed.burn_in)
	feed.publish_error = ""
	feed.frame_index = 0
	feed.published = 0
	feed.next_publish_ns = 0


func _close_publisher(feed: Feed) -> void:
	if feed.publisher != null:
		feed.publisher.close()
		feed.publisher = null


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
	if _source != null:
		if is_instance_valid(_source):
			_view.texture = _source._view.texture
			_header.text = _source._header.text
		return
	var state := SimLink.last_state
	var time_text := "no state yet"
	if state != null:
		time_text = "t %.1f s" % (state.sim_time_ns * 1e-9)
	var selected := selected_feed()
	var status := ""
	for feed in _feeds:
		_aim(feed, state)
		_update_post(feed, state)
		var wants_publish := feed.publisher != null and state != null and state.sim_time_ns >= feed.next_publish_ns
		var is_selected := feed == selected
		if not wants_publish and not (is_selected and _raw != null):
			continue
		var pixels := feed.pixels()
		if is_selected and _raw != null:
			_raw.push(pixels)
			status = "  raw: %d written, %d dropped%s" % [
				_raw.frames_written, _raw.frames_dropped, "  " + _raw.error if _raw.error != "" else ""
			]
		if wants_publish:
			_publish(feed, pixels, state)
			if is_selected:
				_show_published(feed, pixels)
	if selected != null:
		if selected.publisher != null:
			status += "  ring: %d frames" % selected.published
			if selected.burn_in:
				status += " (counter burned in)"
		elif selected.publish_error != "":
			status += "  ring: " + selected.publish_error
		_header.text = "%s %dx%d@%d  %s%s" % [
			selected.name, selected.size.x, selected.size.y, roundi(selected.fps), time_text, status
		]


func _aim(feed: Feed, state: RenderState) -> void:
	if state == null:
		return
	var rotation := state.godot_rotation()
	var mount := Basis(rotation) * Frames.godot_body_from_frd(feed.mount_position_frd)
	feed.camera.global_position = state.godot_position() + mount
	feed.camera.quaternion = rotation * Frames.godot_quat_from_frd(feed.mount_q_frd_from_camera)


## Rolling shutter and motion blur both come from how far the camera turns during an interval
static func rotation_uv_shift(rate_frd: Vector3, seconds: float, half_hfov: float, aspect: float) -> Vector2:
	if seconds <= 0.0 or half_hfov <= 0.0:
		return Vector2.ZERO
	var half_width := tan(half_hfov)          # half-width of the image plane at unit distance
	var half_height := half_width / aspect
	# yaw right moves the scene left, so the sample point moves right; pitch up moves it down
	var yaw := rate_frd.z * seconds
	var pitch := rate_frd.y * seconds
	return Vector2(yaw / (2.0 * half_width), pitch / (2.0 * half_height))


func _update_post(feed: Feed, state: RenderState) -> void:
	var material: ShaderMaterial = feed.post.material
	var rate := Vector3.ZERO
	if state != null:
		# the camera turns with the body; its own mount is fixed, so the body rate is the camera rate
		rate = feed.mount_q_frd_from_camera.inverse() * state.angular_rate_frd
	var blur := rotation_uv_shift(rate, feed.shutter_s, feed.half_hfov_rad, feed.aspect())
	material.set_shader_parameter("distortion_k", feed.distortion_k)
	material.set_shader_parameter("rolling_shutter_shift", rotation_uv_shift(rate, feed.rolling_shutter_s, feed.half_hfov_rad, feed.aspect()))
	material.set_shader_parameter("motion_blur_shift", blur)
	material.set_shader_parameter("motion_blur_taps", 8 if blur.length() > 0.001 else 1)
	material.set_shader_parameter("noise_sigma", feed.noise_base)
	material.set_shader_parameter("noise_seed", float(feed.frame_index % 1024))


## The panel shows the bytes that went to the outputs, so what you see is what they receive
func _show_published(feed: Feed, pixels: PackedByteArray) -> void:
	if _shown_image == null or _shown_image.get_width() != feed.size.x or _shown_image.get_height() != feed.size.y:
		_shown_image = Image.create_from_data(feed.size.x, feed.size.y, false, Image.FORMAT_RGB8, pixels)
		_shown = ImageTexture.create_from_image(_shown_image)
		_view.texture = _shown
		return
	_shown_image.set_data(feed.size.x, feed.size.y, false, Image.FORMAT_RGB8, pixels)
	_shown.update(_shown_image)


## Frames carry the sim time they represent and the camera pose that produced them
func _publish(feed: Feed, pixels: PackedByteArray, state: RenderState) -> void:
	var q_ned_from_frd := Quaternion(state.q_x, state.q_y, state.q_z, state.q_w)
	var position_ned := state.position_ned + q_ned_from_frd * feed.mount_position_frd
	var q_ned_from_camera := q_ned_from_frd * feed.mount_q_frd_from_camera
	if feed.publisher.publish(pixels, state.sim_time_ns, feed.frame_index, position_ned, q_ned_from_camera) != 0:
		feed.published += 1
	feed.frame_index += 1
	var period_ns := int(1e9 / maxf(feed.fps, 1.0))
	# never chase a backlog after a pause or a reset
	feed.next_publish_ns = maxi(state.sim_time_ns + 1, feed.next_publish_ns + period_ns)
