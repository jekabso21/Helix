extends VBoxContainer
## FPV preview: a camera in a shared-world SubViewport that follows the drone's camera mount

const CAMERA_MOUNT_FRD := Vector3(0.06, 0.0, -0.02)
const CAMERA_UPTILT_DEG := 35.0

@export var world_view_path: NodePath

@onready var _header: Label = $Header
@onready var _viewport: SubViewport = $Container/SubViewport
@onready var _camera: Camera3D = $Container/SubViewport/Camera3D

var _frames: int = 0


func _ready() -> void:
	var world_view := get_node_or_null(world_view_path)
	if world_view != null:
		var main_viewport: SubViewport = world_view.get_node("SubViewport")
		_viewport.world_3d = main_viewport.world_3d
	_camera.current = true


func _process(_delta: float) -> void:
	var state := SimLink.last_state
	if state == null:
		_header.text = "Camera feed: no state yet"
		return
	var rotation := state.godot_rotation()
	var mount := Basis(rotation) * Frames.godot_body_from_frd(CAMERA_MOUNT_FRD)
	_camera.global_position = state.godot_position() + mount
	var tilt := Quaternion(Vector3.RIGHT, deg_to_rad(CAMERA_UPTILT_DEG))
	_camera.quaternion = rotation * tilt
	_frames += 1
	_header.text = "main_fpv  %dx%d  t %.1f s" % [
		_viewport.size.x, _viewport.size.y, state.sim_time_ns * 1e-9
	]
