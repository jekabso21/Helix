extends SceneTree
## Headless unit tests: godot --headless --path app -s tests/run_tests.gd

var failures: int = 0
var checks: int = 0


func _init() -> void:
	_test_frames()
	_test_render_state_golden()
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
