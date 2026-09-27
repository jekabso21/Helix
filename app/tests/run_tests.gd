extends SceneTree
## Headless unit tests: godot --headless --path app -s tests/run_tests.gd

var failures: int = 0
var checks: int = 0


func _initialize() -> void:
	_test_frames()
	_test_render_state_golden()
	_test_drone_glb()
	_test_plot_series()
	_test_plots_panel()
	_test_osd_overlay()
	_test_frame_publisher()
	_test_camera_panel_follows_the_config()
	_test_camera_rotation_shift()
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


func _test_plots_panel() -> void:
	var panel: Variant = load("res://ui/plots_panel.gd").new()
	panel._ready()
	var sample := {
		"sim": {"sim_time_ns": 2_000_000_000, "paused": false},
		"motors": [{"command": 0.2}, {"command": 0.3}, {"command": 0.4}, {"command": 0.5}],
		"flight": {"rates_frd_radps": [0.1, -0.2, 0.3]},
		"battery": {"voltage_v": 24.5, "current_a": 6.0},
	}
	panel._on_telemetry(sample)
	sample["sim"]["sim_time_ns"] = 2_100_000_000
	panel._on_telemetry(sample)
	check(panel._motors[3].size() == 2 and absf(panel._motors[3].latest() - 0.5) < 1e-6, "motor series filled")
	check(absf(panel._rates[1].latest() - rad_to_deg(-0.2)) < 1e-6, "rates stored in deg/s")
	check(panel._battery[0].latest() == 24.5 and panel._battery[1].latest() == 6.0, "battery series filled")
	sample["sim"]["paused"] = true
	panel._on_telemetry(sample)
	check(panel._motors[0].size() == 2, "paused telemetry is not appended")
	panel.free()


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

	var panel: Variant = load("res://ui/camera_panel.gd").new()
	panel._load_mount(ProjectSettings.globalize_path(dir), "probe_fpv")
	check(panel.mount_position_frd.is_equal_approx(Vector3(0.055, 0.0, -0.015)), "mount position from the compiled model, got %s" % panel.mount_position_frd)
	var look := Frames.godot_quat_from_frd(panel.mount_q_frd_from_camera) * Vector3(0, 0, -1)
	check(look.y > 0.4, "a 35 deg nose-up camera part tilts the view up, got %s" % look)
	panel.free()


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
