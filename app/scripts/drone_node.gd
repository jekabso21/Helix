extends Node3D
## Placeholder box drone driven by RenderState; arms drawn in the FRD sense with a red nose

const ARM_M := 0.08
const BODY_SIZE := Vector3(0.15, 0.03, 0.04)

var last_state: RenderState = null


func _ready() -> void:
	_build()
	SimLink.render_state.connect(_on_render_state)


func _build() -> void:
	var body := MeshInstance3D.new()
	var body_mesh := BoxMesh.new()
	body_mesh.size = Vector3(BODY_SIZE.z, BODY_SIZE.y, BODY_SIZE.x)
	body.mesh = body_mesh
	body.material_override = _material(Color(0.25, 0.25, 0.28))
	add_child(body)
	var nose := MeshInstance3D.new()
	var nose_mesh := BoxMesh.new()
	nose_mesh.size = Vector3(0.03, 0.02, 0.03)
	nose.mesh = nose_mesh
	nose.position = Frames.godot_body_from_frd(Vector3(0.085, 0.0, 0.0))
	nose.material_override = _material(Color(0.9, 0.1, 0.1))
	add_child(nose)
	# Betaflight quad X: 1 rear right, 2 front right, 3 rear left, 4 front left
	var corners := [
		Vector3(-ARM_M, ARM_M, 0.0), Vector3(ARM_M, ARM_M, 0.0),
		Vector3(-ARM_M, -ARM_M, 0.0), Vector3(ARM_M, -ARM_M, 0.0),
	]
	for corner: Vector3 in corners:
		var prop := MeshInstance3D.new()
		var disc := CylinderMesh.new()
		disc.top_radius = 0.064
		disc.bottom_radius = 0.064
		disc.height = 0.004
		prop.mesh = disc
		prop.position = Frames.godot_body_from_frd(corner)
		prop.material_override = _material(Color(0.1, 0.5, 0.9, 0.5))
		add_child(prop)
		var arm := MeshInstance3D.new()
		var arm_mesh := BoxMesh.new()
		arm_mesh.size = Vector3(0.012, 0.005, corner.length())
		arm.mesh = arm_mesh
		arm.position = Frames.godot_body_from_frd(corner * 0.5)
		arm.look_at_from_position(arm.position, Frames.godot_body_from_frd(corner), Vector3.UP)
		arm.material_override = _material(Color(0.15, 0.15, 0.15))
		add_child(arm)


func _material(color: Color) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	if color.a < 1.0:
		material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	return material


func _on_render_state(state: RenderState) -> void:
	last_state = state
	position = state.godot_position()
	quaternion = state.godot_rotation()
