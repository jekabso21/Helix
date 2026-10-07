extends Node3D
## World-fixed arrow beside the drone showing where the air moves, longer for stronger wind

const METRES_PER_MPS := 0.12
const MAX_LENGTH_M := 2.5
const HIDE_BELOW_MPS := 0.2
const OFFSET_UP_M := 0.8

var _shaft: MeshInstance3D = null
var _head: MeshInstance3D = null


func _ready() -> void:
	var material := StandardMaterial3D.new()
	material.albedo_color = Color(0.45, 0.75, 1.0, 0.85)
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var shaft_mesh := CylinderMesh.new()
	shaft_mesh.top_radius = 0.015
	shaft_mesh.bottom_radius = 0.015
	shaft_mesh.height = 1.0
	_shaft = MeshInstance3D.new()
	_shaft.mesh = shaft_mesh
	_shaft.material_override = material
	add_child(_shaft)
	var head_mesh := CylinderMesh.new()
	head_mesh.top_radius = 0.0
	head_mesh.bottom_radius = 0.05
	head_mesh.height = 0.12
	_head = MeshInstance3D.new()
	_head.mesh = head_mesh
	_head.material_override = material
	add_child(_head)
	visible = false


## Shaft length in metres for a wind speed; exposed for the tests
static func length_for(speed_mps: float) -> float:
	return minf(speed_mps * METRES_PER_MPS, MAX_LENGTH_M)


func _process(_delta: float) -> void:
	var state := SimLink.last_state
	if state == null:
		visible = false
		return
	var wind := Frames.godot_from_ned(state.wind_ned)
	var speed := wind.length()
	visible = speed >= HIDE_BELOW_MPS
	if not visible:
		return
	var length := length_for(speed)
	var direction := wind / speed
	global_position = state.godot_position() + Vector3(0.0, OFFSET_UP_M, 0.0) - direction * length * 0.5
	# the cylinders are built along +Y; turn +Y onto the wind direction
	var axis := Vector3.UP.cross(direction)
	var angle := acos(clampf(Vector3.UP.dot(direction), -1.0, 1.0))
	global_basis = Basis(axis.normalized(), angle) if axis.length() > 1e-6 else (Basis.IDENTITY if direction.y > 0.0 else Basis(Vector3.RIGHT, PI))
	_shaft.scale = Vector3(1.0, length, 1.0)
	_shaft.position = Vector3(0.0, length * 0.5, 0.0)
	_head.position = Vector3(0.0, length + 0.06, 0.0)
