extends Node3D
## Benchmark: options after "--" are --no-readback, --no-popout, --frames N

const CAMERA_SIZE := Vector2i(1280, 720)
const WARMUP_FRAMES := 60
const BOX_COUNT := 400

var _frames_to_measure: int = 600
var _do_readback: bool = true
var _do_popout: bool = true
var _camera_viewport: SubViewport
var _frame: int = 0
var _readback_ms: PackedFloat64Array = []
var _frame_ms: PackedFloat64Array = []
var _last_frame_usec: int = 0
var _bytes_read: int = 0


func _ready() -> void:
	var args := OS.get_cmdline_user_args()
	_do_readback = not args.has("--no-readback")
	_do_popout = not args.has("--no-popout")
	var frames_index := args.find("--frames")
	if frames_index >= 0 and frames_index + 1 < args.size():
		_frames_to_measure = int(args[frames_index + 1])

	_build_world()
	var main_camera := Camera3D.new()
	main_camera.position = Vector3(0.0, 12.0, 30.0)
	main_camera.rotation_degrees = Vector3(-20.0, 0.0, 0.0)
	add_child(main_camera)

	_camera_viewport = SubViewport.new()
	_camera_viewport.size = CAMERA_SIZE
	_camera_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_camera_viewport.world_3d = get_viewport().world_3d
	add_child(_camera_viewport)
	var fpv_camera := Camera3D.new()
	fpv_camera.position = Vector3(0.0, 3.0, 20.0)
	fpv_camera.fov = 120.0
	_camera_viewport.add_child(fpv_camera)

	if _do_popout:
		_open_popout()


func _build_world() -> void:
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50.0, 30.0, 0.0)
	light.shadow_enabled = true
	add_child(light)
	var ground := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(200.0, 200.0)
	ground.mesh = plane
	add_child(ground)
	var box := BoxMesh.new()
	var rng := RandomNumberGenerator.new()
	rng.seed = 1
	for _index in BOX_COUNT:
		var instance := MeshInstance3D.new()
		instance.mesh = box
		instance.position = Vector3(
			rng.randf_range(-60.0, 60.0), rng.randf_range(0.5, 8.0), rng.randf_range(-60.0, 10.0)
		)
		add_child(instance)


func _open_popout() -> void:
	var window := Window.new()
	window.title = "fpvsim spike: camera feed pop-out"
	window.size = Vector2i(640, 360)
	var feed := TextureRect.new()
	feed.texture = _camera_viewport.get_texture()
	feed.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	feed.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	feed.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	window.add_child(feed)
	add_child(window)


func _process(_delta: float) -> void:
	var now := Time.get_ticks_usec()
	_frame += 1
	var measuring := _frame > WARMUP_FRAMES
	if measuring and _last_frame_usec > 0:
		_frame_ms.append(float(now - _last_frame_usec) / 1000.0)
	_last_frame_usec = now

	if _do_readback:
		var start := Time.get_ticks_usec()
		var image := _camera_viewport.get_texture().get_image()
		var data := image.get_data()
		if measuring:
			_readback_ms.append(float(Time.get_ticks_usec() - start) / 1000.0)
			_bytes_read = data.size()

	if _frame >= WARMUP_FRAMES + _frames_to_measure:
		_report()
		get_tree().quit()


func _stats(values: PackedFloat64Array) -> String:
	if values.is_empty():
		return "n/a"
	var sorted := values.duplicate()
	sorted.sort()
	var total := 0.0
	for value in sorted:
		total += value
	var p99 := sorted[int(float(sorted.size() - 1) * 0.99)]
	return "mean %.2f  p99 %.2f  max %.2f ms" % [total / sorted.size(), p99, sorted[-1]]


func _report() -> void:
	var window_size := get_window().size
	var mean_frame_ms := 0.0
	for value in _frame_ms:
		mean_frame_ms += value
	mean_frame_ms /= maxf(1.0, float(_frame_ms.size()))
	print("S8 readback bench: readback=%s popout=%s" % [_do_readback, _do_popout])
	print("  display server: %s, screen refresh %.1f Hz, main window %dx%d, windows open: %d" % [
		DisplayServer.get_name(), DisplayServer.screen_get_refresh_rate(),
		window_size.x, window_size.y, DisplayServer.get_window_list().size(),
	])
	print("  camera viewport: %dx%d, %d bytes per frame" % [
		CAMERA_SIZE.x, CAMERA_SIZE.y, _bytes_read
	])
	print("  frame time: %s  (%.1f fps)" % [_stats(_frame_ms), 1000.0 / maxf(0.001, mean_frame_ms)])
	print("  readback (get_image + get_data): %s" % _stats(_readback_ms))
