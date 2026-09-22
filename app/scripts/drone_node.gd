extends Node3D
## Drone driven by RenderState: the session's compiled drone.glb once it is known, a box until then

const ARM_M := 0.08
const BODY_SIZE := Vector3(0.15, 0.03, 0.04)
const THRUST_ARROW_M := 0.12
const LOAD_BAR_M := 0.06
const RPM_TO_RADPS := TAU / 60.0

var last_state: RenderState = null
var model: Dictionary = {}
var overlays_visible: bool = true

var _glb_path := ""
var _placeholder: Node3D = null
var _model_root: Node3D = null
var _parts: Dictionary = {}
var _highlighted := ""
var _highlight: StandardMaterial3D = null
var _overlays: Node3D = null
var _arrows: Array[MeshInstance3D] = []
var _bars: Array[MeshInstance3D] = []
var _arrow_materials: Array[StandardMaterial3D] = []


func _ready() -> void:
	add_to_group("drone")
	_highlight = _material(Color(1.0, 0.85, 0.2))
	_highlight.emission_enabled = true
	_highlight.emission = Color(0.6, 0.5, 0.1)
	_build_placeholder()
	SimLink.render_state.connect(_on_render_state)
	BackendClient.status.connect(_on_status)
	if not BackendClient.last_status.is_empty():
		_on_status(BackendClient.last_status)


func part_names() -> Array:
	return _parts.keys()


## Position of a part's mesh origin in the Godot body frame, without needing the scene tree
func part_position_body(part_name: String) -> Vector3:
	if not _parts.has(part_name):
		return Vector3.ZERO
	var t := Transform3D.IDENTITY
	var node: Node = _parts[part_name]
	while node != null and node != self:
		var spatial := node as Node3D
		if spatial != null:
			t = spatial.transform * t
		node = node.get_parent()
	return t.origin


func highlight_part(part_name: String) -> void:
	if _highlighted != "" and _parts.has(_highlighted):
		(_parts[_highlighted] as MeshInstance3D).material_override = null
	_highlighted = part_name if _parts.has(part_name) else ""
	if _highlighted != "":
		(_parts[_highlighted] as MeshInstance3D).material_override = _highlight


func set_overlays_visible(on: bool) -> void:
	overlays_visible = on
	if _overlays != null:
		_overlays.visible = on


## Loads a compiled model: glb for the look, json for motor positions and thrust constants
func load_model(glb_path: String, json_path: String) -> bool:
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	if doc.append_from_file(glb_path, state) != OK:
		push_warning("drone: cannot read %s" % glb_path)
		return false
	var root := doc.generate_scene(state) as Node3D
	if root == null:
		return false
	_clear_model()
	# glTF front is +Z, the Godot body nose is -Z: fixed half turn about up
	_model_root = Node3D.new()
	_model_root.rotation.y = PI
	_model_root.add_child(root)
	add_child(_model_root)
	_collect_parts(root)
	if _placeholder != null:
		_placeholder.queue_free()
		_placeholder = null
	_glb_path = glb_path
	model = _read_json(json_path)
	_build_overlays()
	return true


func _on_status(data: Dictionary) -> void:
	var drone: Variant = data.get("drone")
	if drone == null or not drone is Dictionary:
		return
	var glb := str((drone as Dictionary).get("glb", ""))
	if glb == "" or glb == _glb_path:
		return
	load_model(glb, str((drone as Dictionary).get("json", "")))


func _clear_model() -> void:
	if _model_root != null:
		_model_root.queue_free()
		_model_root = null
	if _overlays != null:
		_overlays.queue_free()
		_overlays = null
	_parts.clear()
	_arrows.clear()
	_bars.clear()
	_arrow_materials.clear()
	_highlighted = ""


func _collect_parts(node: Node) -> void:
	var mesh := node as MeshInstance3D
	if mesh != null:
		_parts[mesh.name] = mesh
	for child in node.get_children():
		_collect_parts(child)


func _read_json(path: String) -> Dictionary:
	var text := FileAccess.get_file_as_string(path)
	var parsed: Variant = JSON.parse_string(text) if text != "" else null
	return parsed if parsed is Dictionary else {}


func _build_overlays() -> void:
	_overlays = Node3D.new()
	_overlays.visible = overlays_visible
	add_child(_overlays)
	_overlays.add_child(_marker(Vector3.ZERO, 0.012, Color(1.0, 0.9, 0.1)))  # CG
	var cg: Array = model.get("cg_from_origin_frd_m", [0.0, 0.0, 0.0])
	var origin_frd := -Vector3(cg[0], cg[1], cg[2])
	_overlays.add_child(_marker(Frames.godot_body_from_frd(origin_frd), 0.008, Color(0.6, 0.6, 0.6)))
	for motor: Dictionary in model.get("motors", []):
		var p: Array = motor["position_frd_m"]
		var a: Array = motor["axis_frd"]
		var pivot := Node3D.new()
		pivot.position = Frames.godot_body_from_frd(Vector3(p[0], p[1], p[2]))
		var up := Frames.godot_body_from_frd(Vector3(a[0], a[1], a[2]))
		if not up.is_equal_approx(Vector3.UP):
			pivot.quaternion = Quaternion(Vector3.UP, up)
		_overlays.add_child(pivot)
		var arrow_material := _material(Color(0.2, 0.9, 0.2))
		var arrow := MeshInstance3D.new()
		var shaft := BoxMesh.new()
		shaft.size = Vector3(0.006, 1.0, 0.006)
		arrow.mesh = shaft
		arrow.material_override = arrow_material
		arrow.scale = Vector3(1.0, 0.001, 1.0)
		pivot.add_child(arrow)
		_arrows.append(arrow)
		_arrow_materials.append(arrow_material)
		var bar := MeshInstance3D.new()
		var bar_mesh := BoxMesh.new()
		bar_mesh.size = Vector3(0.02, 1.0, 0.004)
		bar.mesh = bar_mesh
		bar.material_override = _material(Color(0.9, 0.9, 0.9))
		bar.position = Vector3(0.0, 0.0, -0.03)
		bar.scale = Vector3(1.0, 0.001, 1.0)
		pivot.add_child(bar)
		_bars.append(bar)


func _marker(at: Vector3, radius: float, color: Color) -> MeshInstance3D:
	var marker := MeshInstance3D.new()
	var sphere := SphereMesh.new()
	sphere.radius = radius
	sphere.height = 2.0 * radius
	marker.mesh = sphere
	marker.position = at
	marker.material_override = _material(color)
	return marker


## Thrust fraction per motor from the RPM in RenderState and the compiled motor constants
func thrust_fractions(state: RenderState) -> PackedFloat32Array:
	var out := PackedFloat32Array()
	var motors: Array = model.get("motors", [])
	for i in motors.size():
		var fo: Dictionary = (motors[i] as Dictionary)["first_order"]
		var max_speed: float = fo["max_speed_radps"]
		var speed := 0.0
		if i < state.motor_rpm.size():
			speed = state.motor_rpm[i] * RPM_TO_RADPS
		out.append(clampf((speed * speed) / (max_speed * max_speed), 0.0, 1.0))
	return out


func _update_overlays(state: RenderState) -> void:
	if _arrows.is_empty():
		return
	var fractions := thrust_fractions(state)
	for i in mini(fractions.size(), _arrows.size()):
		var f := fractions[i]
		var length := maxf(f * THRUST_ARROW_M, 0.001)
		_arrows[i].scale.y = length
		_arrows[i].position.y = 0.5 * length
		_arrow_materials[i].albedo_color = Color(0.2, 0.9, 0.2).lerp(Color(0.95, 0.15, 0.1), f)
		var bar_h := maxf(f * LOAD_BAR_M, 0.001)
		_bars[i].scale.y = bar_h
		_bars[i].position.y = 0.5 * bar_h


func _build_placeholder() -> void:
	_placeholder = Node3D.new()
	add_child(_placeholder)
	var body := MeshInstance3D.new()
	var body_mesh := BoxMesh.new()
	body_mesh.size = Vector3(BODY_SIZE.z, BODY_SIZE.y, BODY_SIZE.x)
	body.mesh = body_mesh
	body.material_override = _material(Color(0.25, 0.25, 0.28))
	_placeholder.add_child(body)
	var nose := MeshInstance3D.new()
	var nose_mesh := BoxMesh.new()
	nose_mesh.size = Vector3(0.03, 0.02, 0.03)
	nose.mesh = nose_mesh
	nose.position = Frames.godot_body_from_frd(Vector3(0.085, 0.0, 0.0))
	nose.material_override = _material(Color(0.9, 0.1, 0.1))
	_placeholder.add_child(nose)
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
		_placeholder.add_child(prop)
		var arm := MeshInstance3D.new()
		var arm_mesh := BoxMesh.new()
		arm_mesh.size = Vector3(0.012, 0.005, corner.length())
		arm.mesh = arm_mesh
		arm.position = Frames.godot_body_from_frd(corner * 0.5)
		arm.look_at_from_position(arm.position, Frames.godot_body_from_frd(corner), Vector3.UP)
		arm.material_override = _material(Color(0.15, 0.15, 0.15))
		_placeholder.add_child(arm)


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
	_update_overlays(state)
