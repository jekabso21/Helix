extends Node3D
## Main view cameras: 1 chase, 2 side, 3 orbit, 4 free, 5 top, 6 FPV (the drone camera)

enum Mode { CHASE, SIDE, ORBIT, FREE, TOP, FPV }

const MODE_NAMES := ["Chase", "Side", "Orbit", "Free", "Top", "FPV"]
const CHASE_OFFSET := Vector3(0.0, 0.7, 2.0)
const SIDE_OFFSET := Vector3(2.2, 0.5, 0.0)
const TOP_HEIGHT := 12.0
const FREE_SPEED := 8.0

signal mode_changed(mode: int)

@export var drone_path: NodePath

var mode: int = Mode.SIDE
var orbit_yaw: float = 0.6
var orbit_pitch: float = 0.35
var orbit_distance: float = 2.5
var _chase_yaw: float = 0.0
var _free_yaw: float = 0.0
var _free_pitch: float = -0.2
var _dragging: bool = false

@onready var camera: Camera3D = $Camera3D


func _ready() -> void:
	camera.current = true
	set_mode(Mode.CHASE)


func set_mode(new_mode: int) -> void:
	mode = new_mode
	var drone := get_node_or_null(drone_path) as Node3D
	var fpv := drone.get_node_or_null("FpvCamera") as Camera3D if drone != null else null
	if mode == Mode.FPV and fpv != null:
		fpv.current = true
	else:
		camera.current = true
	if mode == Mode.FREE:
		_free_yaw = camera.rotation.y
		_free_pitch = camera.rotation.x
	mode_changed.emit(mode)


func _unhandled_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key != null and key.pressed and not key.echo:
		match key.keycode:
			KEY_1: set_mode(Mode.CHASE)
			KEY_2: set_mode(Mode.SIDE)
			KEY_3: set_mode(Mode.ORBIT)
			KEY_4: set_mode(Mode.FREE)
			KEY_5: set_mode(Mode.TOP)
			KEY_6: set_mode(Mode.FPV)
		return
	var button := event as InputEventMouseButton
	if button != null:
		if button.button_index == MOUSE_BUTTON_RIGHT:
			_dragging = button.pressed
		elif button.pressed and button.button_index == MOUSE_BUTTON_WHEEL_UP:
			orbit_distance = maxf(1.0, orbit_distance * 0.9)
		elif button.pressed and button.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			orbit_distance = minf(60.0, orbit_distance * 1.1)
		return
	var motion := event as InputEventMouseMotion
	if motion != null and _dragging:
		if mode == Mode.ORBIT:
			orbit_yaw -= motion.relative.x * 0.01
			orbit_pitch = clampf(orbit_pitch + motion.relative.y * 0.01, -1.2, 1.4)
		elif mode == Mode.FREE:
			_free_yaw -= motion.relative.x * 0.005
			_free_pitch = clampf(_free_pitch - motion.relative.y * 0.005, -1.5, 1.5)


func _process(delta: float) -> void:
	var drone := get_node_or_null(drone_path) as Node3D
	var target := drone.global_position if drone != null else Vector3.ZERO
	match mode:
		Mode.CHASE:
			_chase(drone, target, delta)
		Mode.SIDE:
			camera.global_position = target + SIDE_OFFSET
			camera.look_at(target, Vector3.UP)
		Mode.ORBIT:
			var offset := Vector3(0.0, 0.0, orbit_distance)
			offset = offset.rotated(Vector3.RIGHT, -orbit_pitch).rotated(Vector3.UP, orbit_yaw)
			camera.global_position = target + offset
			camera.look_at(target, Vector3.UP)
		Mode.TOP:
			camera.global_position = target + Vector3(0.0, TOP_HEIGHT, 0.0)
			camera.look_at(target, Vector3(0.0, 0.0, -1.0))
		Mode.FREE:
			_free(delta)
		Mode.FPV:
			pass


func _chase(drone: Node3D, target: Vector3, delta: float) -> void:
	if drone != null:
		var heading := Frames.heading_rad(drone.quaternion)
		var wanted := -heading  # Godot yaw about +Y is counter-clockwise, heading is clockwise
		_chase_yaw = lerp_angle(_chase_yaw, wanted, minf(1.0, delta * 3.0))
	var offset := CHASE_OFFSET.rotated(Vector3.UP, _chase_yaw)
	camera.global_position = camera.global_position.lerp(target + offset, minf(1.0, delta * 8.0))
	camera.look_at(target + Vector3(0.0, 0.3, 0.0), Vector3.UP)


func _free(delta: float) -> void:
	camera.rotation = Vector3(_free_pitch, _free_yaw, 0.0)
	var direction := Vector3.ZERO
	if Input.is_key_pressed(KEY_W):
		direction -= camera.basis.z
	if Input.is_key_pressed(KEY_S):
		direction += camera.basis.z
	if Input.is_key_pressed(KEY_A):
		direction -= camera.basis.x
	if Input.is_key_pressed(KEY_D):
		direction += camera.basis.x
	if Input.is_key_pressed(KEY_Q):
		direction -= Vector3.UP
	if Input.is_key_pressed(KEY_E):
		direction += Vector3.UP
	if direction.length() > 0.0:
		camera.global_position += direction.normalized() * FREE_SPEED * delta
