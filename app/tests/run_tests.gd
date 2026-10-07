extends SceneTree
## Headless unit tests: godot --headless --path app -s tests/run_tests.gd

var failures: int = 0
var checks: int = 0


func _initialize() -> void:
	_test_frames()
	_test_render_state_golden()
	_test_drone_glb()
	_test_plot_series()
	_test_osd_overlay()
	_test_frame_publisher()
	_test_burn_in_counter()
	_test_camera_panel_follows_the_config()
	_test_camera_rotation_shift()
	_test_output_status_lines()
	_test_osd_font()
	await _test_camera_selector()
	await _test_telemetry_dock()
	_test_window_scaling()
	_test_input_profile_detection()
	_test_controller_does_not_drive_the_ui()
	await _test_environment_tab()
	_test_world_cameras_follow_the_drone()
	await _test_camera_popout_mirrors_the_dock()
	await _test_osd_overlay_takes_the_running_session()
	print("%d checks, %d failures" % [checks, failures])
	quit(1 if failures > 0 else 0)


func check(condition: bool, message: String) -> void:
	checks += 1
	if not condition:
		failures += 1
		push_error("FAIL: " + message)
		printerr("FAIL: " + message)


func near(a: Vector3, b: Vector3, tol: float = 1e-6) -> bool:
	return (a - b).length() < tol


func _test_frames() -> void:
	check(near(Frames.godot_from_ned(Vector3(1, 0, 0)), Vector3(0, 0, -1)), "north is -Z")
	check(near(Frames.godot_from_ned(Vector3(0, 1, 0)), Vector3(1, 0, 0)), "east is +X")
	check(near(Frames.godot_from_ned(Vector3(0, 0, 1)), Vector3(0, -1, 0)), "down is -Y")
	var v := Vector3(1.5, -2.0, 3.0)
	check(near(Frames.ned_from_godot(Frames.godot_from_ned(v)), v), "ned round trip")
	# yaw right 90 deg: nose from north to east
	var half := PI / 4.0
	var q := Frames.godot_quat_from_ned_frd(cos(half), 0.0, 0.0, sin(half))
	var nose := q * Vector3(0, 0, -1)
	check(near(nose, Vector3(1, 0, 0)), "yaw 90 points the nose east, got %s" % nose)
	check(abs(Frames.heading_rad(q) - PI / 2.0) < 1e-6, "heading 90 deg")
	# nose up 20 deg (positive pitch about FRD y) lifts the Godot nose (+Y)
	var pitch := deg_to_rad(20.0) / 2.0
	var qp := Frames.godot_quat_from_ned_frd(cos(pitch), 0.0, sin(pitch), 0.0)
	check((qp * Vector3(0, 0, -1)).y > 0.3, "nose up raises the nose")
	# FRD y is the right axis, so a positive rotation about it pitches the nose up
	var uptilt := Quaternion(Vector3(0, 1, 0), deg_to_rad(35.0))
	var g := Frames.godot_quat_from_frd(uptilt)
	var look := g * Vector3(0, 0, -1)
	check(look.y > 0.4 and look.z < -0.7, "an FRD uptilt tilts the Godot camera up, got %s" % look)
	var downtilt := Frames.godot_quat_from_frd(Quaternion(Vector3(0, 1, 0), deg_to_rad(-35.0)))
	check((downtilt * Vector3(0, 0, -1)).y < -0.4, "and a negative one tilts it down")
	check(absf(g.length() - 1.0) < 1e-6, "the converted rotation stays a unit quaternion")


func _test_render_state_golden() -> void:
	var data := FileAccess.get_file_as_bytes("res://../tests/golden/proto/render_state_v1.bin")
	check(data.size() == RenderState.MESSAGE_SIZE, "golden size %d" % data.size())
	var s := RenderState.decode(data)
	check(s != null, "golden decodes")
	if s == null:
		return
	check(s.seq == 7, "seq")
	check(s.sim_time_ns == 1234567890, "sim time")
	check(near(s.position_ned, Vector3(1.5, -2.25, -10.0)), "position")
	check(is_equal_approx(s.q_y, -0.2), "quaternion y")
	check(s.armed and not s.crashed and s.motor_count == 4, "flags")
	check(is_equal_approx(s.motor_rpm[3], 23000.0), "motor rpm")
	check(is_equal_approx(s.sun_intensity, 0.875), "sun intensity")
	check(s.precip_type == 2 and is_equal_approx(s.precip_intensity, 0.25), "precipitation")
	check(near(s.wind_ned, Vector3(5, -1, 0)), "wind")
	check(s.video_fault_flags == 5, "video flags")
	check(near(s.godot_position(), Vector3(-2.25, 10.0, -1.5)), "godot position")
	var truncated := data.slice(0, 100)
	check(RenderState.decode(truncated) == null, "rejects short input")


func _test_drone_glb() -> void:
	var drone: Variant = load("res://scripts/drone_node.gd").new()
	var base := ProjectSettings.globalize_path("res://../tests/golden/config/")
	check(drone.load_model(base + "reference_5in.drone.glb", base + "reference_5in.drone.json"), "golden glb loads")
	check(drone.part_names().has("battery") and drone.part_names().has("nose_marker"), "parts found: %s" % [drone.part_names()])
	var nose_body: Vector3 = drone.part_position_body("nose_marker")
	check(nose_body.z < -0.1 and absf(nose_body.x) < 1e-3, "nose marker is at the Godot body nose (-Z), got %s" % nose_body)
	check(drone.part_position_body("battery").y > 0.0, "the top-mounted battery sits above the CG")
	check(drone.part_position_body("motor2").z < 0.0 and drone.part_position_body("motor2").x > 0.0, "motor 2 is front right")
	check(drone.model.get("motors", []).size() == 4, "compiled json read")
	var state := RenderState.new()
	var max_rpm: float = drone.model["motors"][0]["motor"]["max_speed_radps"] * 60.0 / TAU
	state.motor_rpm = PackedFloat32Array([max_rpm, 0.0, max_rpm / 2.0, 0.0])
	var f: PackedFloat32Array = drone.thrust_fractions(state)
	check(absf(f[0] - 1.0) < 1e-3 and f[1] == 0.0 and absf(f[2] - 0.25) < 1e-3, "thrust fractions %s" % [f])
	drone.queue_free()


func _test_plot_series() -> void:
	var s := PlotSeries.new(10.0)
	for i in 30:
		s.push(float(i), float(i) * 0.5)
	check(s.size() == 11, "window keeps the last 10 s (got %d samples)" % s.size())
	check(s.times[0] == 19.0 and s.latest() == 14.5, "oldest kept sample is t=19, latest value 14.5")
	check(s.min_value() == 9.5 and s.max_value() == 14.5, "min and max over the window")
	s.push(3.0, 1.0)
	check(s.size() == 1 and s.latest() == 1.0, "time going backwards clears the window")


## The telemetry dock keeps 10 s of body rates and battery voltage for its sparklines
func _test_telemetry_dock() -> void:
	var dock: Node = load("res://ui/telemetry_panel.tscn").instantiate()
	await _add_to_root(dock)
	var panel: Node = dock.get_node("Pad/Sections")
	var sample := {
		"sim": {"sim_time_ns": 2_000_000_000, "paused": false, "mode": "realtime", "overruns": 0, "crashed": false},
		"motors": [{"command": 0.2, "rpm": 100.0}, {"command": 0.3, "rpm": 100.0}, {"command": 0.4, "rpm": 100.0}, {"command": 0.5, "rpm": 100.0, "current_a": 3.0}],
		"flight": {"rates_frd_radps": [0.1, -0.2, 0.3], "altitude_agl_m": 5.0, "ground_speed_mps": 1.0, "climb_mps": 0.0, "roll_rad": 0.0, "pitch_rad": 0.0, "heading_rad": 0.0},
		"battery": {"voltage_v": 24.5, "current_a": 6.0, "consumed_mah": 12.0, "soc": 0.9},
	}
	panel._on_telemetry(sample)
	sample["sim"]["sim_time_ns"] = 2_100_000_000
	panel._on_telemetry(sample)
	check(absf(panel._rates[1].latest() - rad_to_deg(-0.2)) < 1e-6, "rates stored in deg/s")
	check(panel._voltage.size() == 2 and panel._voltage.latest() == 24.5, "voltage history filled")
	check(panel._battery_tiles["amps"].text == "6.0 A", "battery tiles, got %s" % panel._battery_tiles["amps"].text)
	check(panel._motor_rows.size() == 4 and panel._motor_rows[3].text.contains("50 %"), "one row per motor")
	sample["sim"]["paused"] = true
	panel._on_telemetry(sample)
	check(panel._voltage.size() == 2, "paused telemetry is not appended")
	panel._render_fc({"armed": false, "arming_disable_flags": ["RXLOSS", "MSP"], "pid_cycle_time_us": 125})
	await process_frame
	check(panel._flags.get_child_count() == 2 and panel._loop_value.text == "125 µs", "arming flags as tags and the PID loop time")
	panel._render_fc({"armed": true, "arming_disable_flags": [], "pid_cycle_time_us": 125})
	check(panel._state_value.text == "Armed" and panel._state_tile.theme_type_variation == "TileArmed", "an armed FC stands out")
	dock.queue_free()


func _test_osd_overlay() -> void:
	var overlay: Variant = load("res://ui/osd_overlay.gd")
	check(overlay.glyph(0x41) == "A" and overlay.glyph(0x20) == "", "ascii passes through, blank is empty")
	check(overlay.glyph(0x90) == "▮" and overlay.glyph(0x68) == "↑", "battery and arrow symbols mapped")
	check(overlay.glyph(0xC3) == "▪", "unknown font codes get a placeholder")
	check(overlay.is_horizon_bar(0x80) and overlay.is_horizon_bar(0x88), "horizon ladder codes are drawn as lines")
	check(not overlay.is_horizon_bar(0x7F) and not overlay.is_horizon_bar(0x89), "the ladder range stops at nine codes")
	var panel: Variant = overlay.new()
	panel.set_canvas({"cols": 30, "rows": 16, "codes": [[0x41, 0x20]], "attrs": [[0, 0]]})
	check(panel.cols == 30 and panel.rows == 16 and panel.draws == 1, "canvas stored")
	panel.free()


## Needs ./scripts/build_extension.sh; skipped when the extension is not built
func _test_frame_publisher() -> void:
	if not ClassDB.class_exists("FramePublisher"):
		print("skip: FramePublisher extension not built")
		return
	var publisher: Variant = ClassDB.instantiate("FramePublisher")
	check(publisher.open("selftest_cam", 8, 4, 60.0), "ring opens: %s" % publisher.last_error())
	check(publisher.frame_bytes() == 8 * 4 * 3, "rgb8 frame size %d" % publisher.frame_bytes())
	var pixels := PackedByteArray()
	pixels.resize(publisher.frame_bytes())
	pixels.fill(0xAB)
	var seq: int = publisher.publish(pixels, 123456789, 7, Vector3(1, 2, -3), Quaternion(0, 0, 0, 1))
	check(seq == 1, "first frame publishes as sequence 1, got %d" % seq)
	check(publisher.publish(PackedByteArray(), 1, 1, Vector3.ZERO, Quaternion()) == 0, "a wrong-sized frame is refused")
	# the ring is a plain shared memory object, so the header can be read back as a file
	var f := FileAccess.open("/dev/shm/fpvsim.selftest_cam", FileAccess.READ)
	check(f != null, "the ring exists in /dev/shm")
	if f != null:
		check(f.get_32() == 0x46565046, "header magic is FPVF")
		check(f.get_16() == 1, "layout version 1")
		check(f.get_16() == 2, "pixel format is rgb8")
		check(f.get_32() == 8 and f.get_32() == 4, "size in the header")
		f.seek(128 + 128)
		check(f.get_8() == 0xAB, "the pixels reached the ring")
	publisher.close()


## The counter a latency run reads back out of a video output: marker cell, guard cell, then bits
func _test_burn_in_counter() -> void:
	if not ClassDB.class_exists("FramePublisher"):
		return
	const WIDTH := 20 * 16   # two cells wider than the counter, so its edge can be checked
	const HEIGHT := 16
	var publisher: Variant = ClassDB.instantiate("FramePublisher")
	check(publisher.open("selftest_burnin", WIDTH, HEIGHT, 60.0), "burn-in ring opens")
	publisher.set_burn_in_counter(true)
	check(publisher.burn_in_counter(), "the publisher reports the counter is on")
	var pixels := PackedByteArray()
	pixels.resize(publisher.frame_bytes())
	pixels.fill(0x40)
	publisher.publish(pixels, 1000, 12345, Vector3.ZERO, Quaternion(0, 0, 0, 1))
	var f := FileAccess.open("/dev/shm/fpvsim.selftest_burnin", FileAccess.READ)
	check(f != null, "the burn-in ring exists in /dev/shm")
	if f != null:
		var bits := ""
		for cell in range(18):
			f.seek(128 + 128 + 8 * WIDTH * 3 + (cell * 16 + 8) * 3)
			bits += "1" if f.get_8() >= 128 else "0"
		# 12345 as sixteen bits, most significant first
		check(bits == "10" + "0011000000111001", "counter pattern for frame 12345: " + bits)
		f.seek(128 + 128 + 8 * WIDTH * 3 + (18 * 16 + 4) * 3)
		check(f.get_8() == 0x40, "the counter leaves the rest of the row alone")
	publisher.close()


## The camera panel takes its resolution, fps, field of view and mount from the resolved config,
## so changing the camera YAML or moving the camera part changes what is rendered.
func _test_camera_panel_follows_the_config() -> void:
	var dir := "user://camera_cfg_test"
	DirAccess.make_dir_recursive_absolute(dir + "/resolved")
	var cameras := {
		"schema_version": 1, "host": "127.0.0.1", "status_port": 7730,
		"cameras": [{
			"name": "probe_fpv", "width": 640, "height": 480, "fps": 30.0,
			"pixel_format": "rgb8", "sensor_latency_s": 0.0, "outputs": [],
			"optics": {"hfov_rad": deg_to_rad(90.0)},
		}],
	}
	# camera part pitched 35 deg nose up, as the model compiler writes it
	var uptilt := Quaternion(Vector3(0, 1, 0), deg_to_rad(35.0))
	var drone := {"cameras": [{
		"name": "probe_fpv",
		"position_frd_m": [0.055, 0.0, -0.015],
		"q_frd_from_camera": [uptilt.w, uptilt.x, uptilt.y, uptilt.z],
	}]}
	_write_json(dir + "/resolved/cameras.json", cameras)
	_write_json(dir + "/resolved/drone.json", drone)

	var panel: Variant = load("res://ui/camera_panel.gd")
	var drone_doc: Variant = JSON.parse_string(FileAccess.get_file_as_string(dir + "/resolved/drone.json"))
	var mount: Dictionary = panel.mount_from_drone(drone_doc, "probe_fpv")
	check(mount["position_frd"].is_equal_approx(Vector3(0.055, 0.0, -0.015)), "mount position from the compiled model, got %s" % mount["position_frd"])
	var look := Frames.godot_quat_from_frd(mount["q_frd_from_camera"]) * Vector3(0, 0, -1)
	check(look.y > 0.4, "a 35 deg nose-up camera part tilts the view up, got %s" % look)
	check(panel.mount_from_drone(drone_doc, "absent").is_empty(), "a camera with no mount falls back")


func _write_json(path: String, data: Dictionary) -> void:
	var f := FileAccess.open(path, FileAccess.WRITE)
	f.store_string(JSON.stringify(data))
	f.close()


## Rolling shutter and motion blur are both "how far did the camera turn in this interval"
func _test_camera_rotation_shift() -> void:
	var panel := load("res://ui/camera_panel.gd")
	var half_hfov := deg_to_rad(90.0) / 2.0   # tan(45 deg) = 1, so the maths is easy to follow
	var aspect := 2.0
	check(panel.rotation_uv_shift(Vector3.ZERO, 0.01, half_hfov, aspect) == Vector2.ZERO, "a still camera does not shift")
	check(panel.rotation_uv_shift(Vector3(1, 1, 1), 0.0, half_hfov, aspect) == Vector2.ZERO, "a zero interval does not shift")
	# yaw is about FRD z (down): 0.2 rad over the interval, half-width tan(45) = 1 -> u = 0.1
	var yaw: Vector2 = panel.rotation_uv_shift(Vector3(0, 0, 0.2), 1.0, half_hfov, aspect)
	check(absf(yaw.x - 0.1) < 1e-6 and absf(yaw.y) < 1e-9, "yaw shifts horizontally by rate/2tan, got %s" % yaw)
	# pitch is about FRD y (right); the vertical half-angle is smaller, so the same rate shifts more
	var pitch: Vector2 = panel.rotation_uv_shift(Vector3(0, 0.2, 0), 1.0, half_hfov, aspect)
	check(absf(pitch.y - 0.2) < 1e-6 and absf(pitch.x) < 1e-9, "pitch shifts vertically by aspect x more, got %s" % pitch)
	# roll is not a translation, so it is left out rather than approximated
	check(panel.rotation_uv_shift(Vector3(5.0, 0, 0), 1.0, half_hfov, aspect) == Vector2.ZERO, "roll does not translate the image")
	# twice the interval, twice the shift
	var double: Vector2 = panel.rotation_uv_shift(Vector3(0, 0, 0.2), 2.0, half_hfov, aspect)
	check(absf(double.x - 2.0 * yaw.x) < 1e-9, "the shift scales with the interval")


## The per-output lines the camera panel shows from simvideo reports
func _test_output_status_lines() -> void:
	var panel := load("res://ui/camera_panel.gd")
	var running := {"index": 0, "state": "running", "fps": 59.94, "bitrate_bps": 4000000.0, "last_error": null}
	check(panel.output_summary(running) == "out 0  running  59.9 fps  4.0 Mbit/s", panel.output_summary(running))
	var broken := {"index": 1, "state": "error", "fps": 0.0, "last_error": "could not link"}
	check(panel.output_summary(broken) == "out 1  error  0.0 fps  could not link", panel.output_summary(broken))
	check(panel.output_colour("error").r > panel.output_colour("running").r, "an error stands out")
	check(panel.output_colour("disabled") != panel.output_colour("running"), "a disabled output differs")


## Every configured camera gets its own feed; the selector picks which one the panel shows
func _test_camera_selector() -> void:
	var panel: Node = load("res://ui/camera_panel.tscn").instantiate()
	root.add_child(panel)
	await panel.ready
	var configs := [
		{"name": "main_fpv", "width": 1280, "height": 720, "fps": 60.0, "optics": {"hfov_rad": deg_to_rad(120.0)}},
		{"name": "rear_fpv", "width": 640, "height": 360, "fps": 30.0, "osd": false, "optics": {"hfov_rad": deg_to_rad(90.0)}},
	]
	panel._build_feeds(configs)
	check(panel._feeds.size() == 2, "two cameras give two feeds, got %d" % panel._feeds.size())
	check(panel._selector.item_count == 2 and panel._selector.get_item_text(1) == "rear_fpv", "both cameras are listed")
	check(panel.selected_feed().name == "main_fpv", "the first camera is shown first")
	check(panel.selected_feed().viewport.size == Vector2i(1280, 720), "each feed renders at its own resolution")
	panel._on_camera_selected(1)
	check(panel.selected_feed().name == "rear_fpv", "the selector switches the shown camera")
	check(panel.selected_feed().viewport.size == Vector2i(640, 360), "the second feed has its own size")
	check(panel._feeds[1].osd == false, "a camera can turn the OSD off")
	check(panel._feeds[1].viewport.get_child_count() == 1, "and then gets no OSD overlay node")
	check(panel._feeds[0].viewport.get_child_count() == 2, "while the main camera keeps one")
	panel._build_feeds([
		{"name": "plain", "width": 64, "height": 36, "fps": 30.0, "optics": {"hfov_rad": 1.0, "distortion": {"model": "none", "k": [0.1, 0.2, 0.3, 0.4]}}},
		{"name": "fisheye", "width": 64, "height": 36, "fps": 30.0, "optics": {"hfov_rad": 1.0, "distortion": {"model": "fisheye", "k": [0.1, 0.2, 0.3, 0.4]}}},
	])
	check(panel._feeds[0].distortion_k == Vector4.ZERO, "a camera without a distortion model is a plain pinhole")
	check(panel._feeds[1].distortion_k == Vector4(0.1, 0.2, 0.3, 0.4), "a fisheye camera keeps its coefficients")
	panel._build_feeds(configs)
	panel._on_video({"camera": "rear_fpv", "outputs": [
		{"index": 0, "state": "running", "fps": 30.0},
		{"index": 1, "state": "disabled", "fps": 0.0},
	]})
	check(panel._outputs.get_child_count() == 2, "one row per output")
	var first_toggle: CheckButton = panel._outputs.get_child(0).get_child(0)
	var second_toggle: CheckButton = panel._outputs.get_child(1).get_child(0)
	check(first_toggle.button_pressed and not second_toggle.button_pressed, "the switches follow the reported state")
	panel._on_video({"camera": "main_fpv", "outputs": []})
	check(panel._outputs.get_child_count() == 2, "a report for another camera is ignored")
	# the pop-out window renders the main window's world through the same panel
	var world := World3D.new()
	panel.use_world(world)
	check(panel._feeds[0].viewport.world_3d == world and panel._feeds[1].viewport.world_3d == world, "every feed follows the world it is given")
	panel.queue_free()


## A real flight controller font: black, white and transparent pixels out of a MAX7456 file
func _test_osd_font() -> void:
	var script: Variant = load("res://ui/osd_font.gd")
	var font: Variant = script.new()
	check(not font.is_loaded(), "a fresh font draws nothing")
	check(not font.load_path("/nonexistent/font.mcm") and font.error != "", "a missing file is reported")
	check(not font.load_mcm_text("not a font"), "a file without the header is refused")

	# two glyphs: the first all white, the second black on the first row and transparent below
	var lines := PackedStringArray(["MAX7456"])
	for row in 18:
		for part in 3:
			lines.append("10101010")
	for _pad in 10:
		lines.append("01010101")
	for row in 18:
		for part in 3:
			lines.append("00000000" if row == 0 else "01010101")
	for _pad in 10:
		lines.append("01010101")
	check(font.load_mcm_text("\n".join(lines)), "the font loads: " + font.error)
	check(font.glyph_count == 2 and font.glyph_size == Vector2i(12, 18), "two 12x18 glyphs, got %d %s" % [font.glyph_count, font.glyph_size])
	check(font.region(1) == Rect2(0, 18, 12, 18), "glyph 1 is the second row of the atlas")
	check(font.region(200) == Rect2(0, 18, 12, 18), "a code past the end clamps to the last glyph")
	var image: Image = font.texture.get_image()
	check(image.get_pixel(0, 0) == Color(1, 1, 1, 1), "the first glyph is white")
	check(image.get_pixel(0, 18) == Color(0, 0, 0, 1), "the second glyph starts black")
	check(image.get_pixel(0, 19).a == 0.0, "and is transparent below")

	var atlas: Variant = script.new()
	var strip := Image.create(24, 36 * 256, false, Image.FORMAT_RGBA8)
	check(atlas.load_image(strip) and atlas.glyph_count == 256, "a PNG atlas of 256 stacked glyphs")
	check(atlas.glyph_size == Vector2i(24, 36), "glyph size from the strip, got %s" % atlas.glyph_size)
	check(not script.new().load_image(Image.create(8, 7, false, Image.FORMAT_RGBA8)), "a strip that is not a whole number of glyphs is refused")


## After an await the root can still be setting up children, so nodes join it deferred
func _add_to_root(node: Node) -> void:
	root.add_child.call_deferred(node)
	await node.ready


## The pop-out shows what the docked panel renders: a second renderer would also open a second
## publisher for every camera, and two writers on one ring fight over it
func _test_camera_popout_mirrors_the_dock() -> void:
	var dock: Node = load("res://ui/camera_panel.tscn").instantiate()
	var popout: Node = load("res://ui/camera_panel.tscn").instantiate()
	await _add_to_root(dock)
	await _add_to_root(popout)
	dock._build_feeds([
		{"name": "main_fpv", "width": 320, "height": 180, "fps": 60.0, "optics": {"hfov_rad": deg_to_rad(120.0)}},
		{"name": "rear_fpv", "width": 160, "height": 90, "fps": 30.0, "osd": false, "optics": {"hfov_rad": deg_to_rad(90.0)}},
	])
	popout.mirror(dock)
	check(popout._feeds.is_empty(), "the pop-out renders no camera of its own")
	popout._on_status({"state": "running", "run_dir": ProjectSettings.globalize_path("user://camera_cfg_test")})
	check(popout._feeds.is_empty(), "a running session gives the pop-out no feeds and no publishers")
	popout._process(0.0)
	check(popout._view.texture == dock._view.texture, "the pop-out shows the docked panel's picture")
	dock._on_camera_selected(1)
	popout._process(0.0)
	check(popout._view.texture == dock._view.texture, "and follows the camera picked in the dock")
	check(popout._header.text == dock._header.text, "with the same header line")
	popout.queue_free()
	dock.queue_free()


## An overlay created while the session is already running (the feeds are rebuilt while the
## running status is being delivered) still loads the session's OSD font
func _test_osd_overlay_takes_the_running_session() -> void:
	var script: Variant = load("res://ui/osd_overlay.gd")
	check(script.font_path({"betaflight": {"osd": {"font": null}}}) == "", "a session without a font names no font")
	check(script.font_path({"betaflight": {"osd": {}}}) == "", "nor does one without the key")
	check(script.font_path({"betaflight": {"osd": {"font": "/x/f.mcm"}}}) == "/x/f.mcm", "a configured font is passed on")
	check(script.font_path(null) == "", "an unreadable session names no font")

	var dir := "user://osd_session_test"
	DirAccess.make_dir_recursive_absolute(dir + "/resolved")
	var font_file := ProjectSettings.globalize_path(dir + "/font.png")
	Image.create(24, 36 * 256, false, Image.FORMAT_RGBA8).save_png(font_file)
	_write_json(dir + "/resolved/session.json", {"betaflight": {"osd": {"font": font_file}}})
	var backend: Node = root.get_node("BackendClient")
	var previous: Dictionary = backend.last_status
	backend.last_status = {"state": "running", "run_dir": ProjectSettings.globalize_path(dir)}
	var overlay := Control.new()
	overlay.set_script(script)
	await _add_to_root(overlay)
	check(overlay._atlas.is_loaded(), "an overlay made during a running session uses its OSD font")
	backend.last_status = previous
	overlay.queue_free()


## Docks fold at fixed UI widths and the UI scale walks a fixed list of steps
func _test_window_scaling() -> void:
	var window: Variant = load("res://ui/main_window.gd")
	check(window.fold_level(1600.0) == 0 and window.fold_level(1000.0) == 1 and window.fold_level(800.0) == 2, "docks fold as the window narrows")
	check(window.next_ui_scale(1.0, 1) == 1.1 and window.next_ui_scale(1.0, -1) == 0.9, "one step up and down from 100 %")
	check(window.next_ui_scale(1.17, 1) == 1.25 and window.next_ui_scale(1.17, -1) == 1.1, "an odd scale snaps to the neighbouring steps")
	check(window.next_ui_scale(2.0, 1) == 2.0 and window.next_ui_scale(0.75, -1) == 0.75, "the ends hold")
	check(window.next_ui_scale(1.5, 0) == 1.0, "reset goes back to 100 %")
	var toolbar: Variant = load("res://ui/toolbar.gd")
	check(toolbar.session_label({"name": "ci_hover", "input": "altitude_hold", "duration_s": 36.0}) == "ci_hover  ·  altitude_hold  ·  36 s", "session label")
	check(toolbar.session_label({"name": "dev_gamepad", "input": "gamepad", "duration_s": null}).ends_with("∞"), "an open-ended session")
	check(toolbar.session_label({"name": "dev_gamepad", "input": "gamepad", "duration_s": 0.0}).ends_with("∞"), "duration 0 runs until stopped")
	var sessions := [{"name": "ci_hover"}, {"name": "dev_gamepad"}, {"name": "dev_multicam"}, {"name": "broken", "error": "x"}]
	check(toolbar.preferred_session(sessions, "") == 1, "the gamepad dev session is picked by default")
	check(toolbar.preferred_session(sessions, "dev_multicam") == 2, "the last started session wins")
	check(toolbar.preferred_session(sessions, "broken") == 1 and toolbar.preferred_session(sessions, "gone") == 1, "an invalid or missing last session falls back")
	check(toolbar.preferred_session([{"name": "a"}], "") == 0 and toolbar.preferred_session([], "") == -1, "otherwise the first")
	check(toolbar.clock_text(70_250_000_000) == "t 01:10.25", "clock, got %s" % toolbar.clock_text(70_250_000_000))


## A connected controller is matched to a saved profile by its device name
func _test_input_profile_detection() -> void:
	var panel: Variant = load("res://ui/input_panel.gd")
	var boxer := {"name": "radiomaster_boxer", "device_name_contains": "Radiomaster Boxer", "mapping": {"channels": {}}}
	var xbox := {"name": "xbox", "device_name_contains": "Xbox", "mapping": {"channels": {}}}
	var broken := {"name": "broken", "device_name_contains": "Xbox", "error": "bad"}
	var devices := ["keyd virtual pointer", "OpenTX Radiomaster Boxer Joystick"]
	var found: Dictionary = panel.detect_profile(devices, [xbox, boxer], "")
	check(found.get("profile") == "radiomaster_boxer" and found.get("device") == devices[1], "the boxer is found by name, case-insensitive: %s" % found)
	check(panel.detect_profile(["Xbox Wireless Controller"] + devices, [xbox, boxer], "radiomaster_boxer")["profile"] == "radiomaster_boxer", "the default profile wins when its device is there")
	check(panel.detect_profile(["Xbox Wireless Controller"], [broken, xbox], "")["profile"] == "xbox", "an invalid profile is skipped")
	check(panel.detect_profile(["keyd virtual pointer"], [xbox, boxer], "").is_empty(), "nothing matches, nothing is picked")
	check(panel.detect_profile(devices, [{"name": "any", "device_name_contains": "", "mapping": {}}], "").is_empty(), "a profile without a device name never matches everything")
	var parsed: Dictionary = JSON.parse_string('{"device_name_contains": "Boxer", "channels": {"aux1": {"button": 0, "inverted": false}, "aux2": {"axis": 4, "inverted": true, "deadband": 0.0}}}')
	var clean: Dictionary = panel.clean_mapping(parsed)
	check(typeof(clean["channels"]["aux1"]["button"]) == TYPE_INT and typeof(clean["channels"]["aux2"]["axis"]) == TYPE_INT, "indices are saved as whole numbers")
	check(JSON.stringify(clean).contains('"axis":4,') and clean["channels"]["aux2"]["inverted"] == true, "and nothing else changes: %s" % JSON.stringify(clean))
	check(typeof(parsed["channels"]["aux2"]["axis"]) == TYPE_FLOAT, "the original is left alone")
	check(panel.profile_slug("OpenTX Radiomaster Boxer Joystick") == "opentx_radiomaster_boxer_joystick", "profile name from the device")
	check(panel.profile_slug("  Xbox (Wireless) #2 ") == "xbox_wireless_2" and panel.profile_slug("!!") == "controller", "odd device names still give a file name")


## The radio is read by simcore; in the app its sticks must not move focus or press buttons
func _test_controller_does_not_drive_the_ui() -> void:
	var has_joypad := func() -> bool:
		for action: StringName in InputMap.get_actions():
			for event in InputMap.action_get_events(action):
				if event is InputEventJoypadButton or event is InputEventJoypadMotion:
					return true
		return false
	check(has_joypad.call(), "Godot maps the gamepad to UI navigation out of the box")
	load("res://ui/main_window.gd").drop_joypad_ui_events()
	check(not has_joypad.call(), "no action reacts to a gamepad any more")
	var keys := InputMap.action_get_events("ui_accept").filter(func(e: InputEvent) -> bool: return e is InputEventKey)
	check(not keys.is_empty(), "keyboard navigation is kept")


## The Environment tab shows the wind in the units of the environment file and starts from it
func _test_environment_tab() -> void:
	var script: Variant = load("res://ui/environment_panel.gd")
	check(script.compass(0.0) == "N" and script.compass(270.0) == "W" and script.compass(359.0) == "N" and script.compass(135.0) == "SE", "compass names")
	# a westerly moves air east: NED (0, 5, 0)
	var westerly: Vector2 = script.speed_and_from([0.0, 5.0, 0.0])
	check(absf(westerly.x - 5.0) < 1e-6 and absf(westerly.y - 270.0) < 1e-6, "wind vector back to speed and direction, got %s" % westerly)
	var northerly: Vector2 = script.speed_and_from([-3.0, 0.0, 0.0])
	check(absf(northerly.y) < 1e-6, "a northerly comes from 0 deg, got %s" % northerly)
	check(script.speed_and_from([0.0, 0.0, 0.0]) == Vector2.ZERO, "calm has no direction")
	var arrow: Variant = load("res://scripts/wind_arrow.gd")
	check(absf(arrow.length_for(5.0) - 0.6) < 1e-6 and arrow.length_for(100.0) == 2.5, "the wind arrow grows with speed up to a cap")

	var dir := "user://env_tab_test"
	DirAccess.make_dir_recursive_absolute(dir + "/resolved")
	_write_json(dir + "/resolved/session.json", {"wind": {
		"mean_speed_mps": 6.5, "mean_from_rad": deg_to_rad(225.0), "turbulence_w20_mps": 15.4,
		"turbulence_intensity": "moderate", "profile": {}, "gusts": [],
	}})
	var tab: Node = load("res://ui/environment_panel.tscn").instantiate()
	await _add_to_root(tab)
	var panel: Node = tab.get_node("Pad/Environment")
	panel._on_status({"state": "running", "run_dir": ProjectSettings.globalize_path(dir)})
	check(absf(panel._speed.value - 6.5) < 1e-6 and absf(panel._from.value - 225.0) < 1e-6, "sliders start at the session's wind")
	check(panel._turbulence.get_item_text(panel._turbulence.selected) == "moderate", "and its turbulence level")
	check(panel._send_at < 0.0, "loading the session's wind does not send it back")
	check(panel._from_value.text == "225° SW", "direction label, got %s" % panel._from_value.text)
	panel._on_status({"state": "idle"})
	check(not panel._speed.editable and panel._gust_button.disabled, "nothing to change without a session")
	tab.queue_free()


## Every world camera mode aims at the drone node; an empty path would aim them all at the spawn
func _test_world_cameras_follow_the_drone() -> void:
	var world: Node = load("res://scenes/world.tscn").instantiate()
	var rig: Node = world.get_node("CameraRig")
	check(rig.drone_path == NodePath("../Drone"), "the rig keeps its drone path, got '%s'" % rig.drone_path)
	world.free()
