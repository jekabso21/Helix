extends Node3D
## Flat checkerboard ground with scattered obstacles and a row of gates; deterministic layout

const GROUND_SIZE := 2000.0
const CHECKER_M := 2.0
const OBJECT_COUNT := 400
const SCATTER_RADIUS := 250.0
const GATE_COUNT := 8
const SEED := 7


func _ready() -> void:
	_build_ground()
	_build_north_marker()
	_scatter_objects()
	_build_gates()


func _build_ground() -> void:
	var image := Image.create(2, 2, false, Image.FORMAT_RGB8)
	image.set_pixel(0, 0, Color(0.42, 0.55, 0.34))
	image.set_pixel(1, 1, Color(0.42, 0.55, 0.34))
	image.set_pixel(1, 0, Color(0.33, 0.46, 0.28))
	image.set_pixel(0, 1, Color(0.33, 0.46, 0.28))
	var texture := ImageTexture.create_from_image(image)
	var material := StandardMaterial3D.new()
	material.albedo_texture = texture
	material.texture_filter = BaseMaterial3D.TEXTURE_FILTER_NEAREST
	material.uv1_scale = Vector3(GROUND_SIZE / (2.0 * CHECKER_M), GROUND_SIZE / (2.0 * CHECKER_M), 1.0)
	var ground := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(GROUND_SIZE, GROUND_SIZE)
	ground.mesh = plane
	ground.material_override = material
	add_child(ground)


func _build_north_marker() -> void:
	var north := MeshInstance3D.new()
	var bar := BoxMesh.new()
	bar.size = Vector3(1.0, 8.0, 1.0)
	north.mesh = bar
	north.position = Vector3(0.0, 4.0, -100.0)
	north.material_override = _material(Color(0.9, 0.1, 0.1))
	add_child(north)


func _scatter_objects() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = SEED
	var pillar := CylinderMesh.new()
	pillar.top_radius = 0.4
	pillar.bottom_radius = 0.5
	pillar.height = 1.0
	var trunk := CylinderMesh.new()
	trunk.top_radius = 0.15
	trunk.bottom_radius = 0.25
	trunk.height = 1.0
	var crown := CylinderMesh.new()
	crown.top_radius = 0.0
	crown.bottom_radius = 2.0
	crown.height = 5.0
	var box := BoxMesh.new()
	var colors := [Color(0.8, 0.5, 0.2), Color(0.3, 0.5, 0.8), Color(0.85, 0.85, 0.3), Color(0.7, 0.3, 0.6)]
	for i in OBJECT_COUNT:
		var angle := rng.randf_range(0.0, TAU)
		var radius := 12.0 + rng.randf_range(0.0, 1.0) ** 0.6 * SCATTER_RADIUS
		var at := Vector3(cos(angle) * radius, 0.0, sin(angle) * radius)
		var kind := rng.randi_range(0, 2)
		if kind == 0:
			var height := rng.randf_range(4.0, 25.0)
			_add(pillar, at + Vector3(0, height * 0.5, 0), Vector3(1, height, 1), colors[i % colors.size()])
		elif kind == 1:
			var size := Vector3(rng.randf_range(2, 8), rng.randf_range(2, 12), rng.randf_range(2, 8))
			_add(box, at + Vector3(0, size.y * 0.5, 0), size, Color(0.55, 0.55, 0.6))
		else:
			var trunk_height := rng.randf_range(3.0, 7.0)
			_add(trunk, at + Vector3(0, trunk_height * 0.5, 0), Vector3(1, trunk_height, 1), Color(0.35, 0.22, 0.1))
			_add(crown, at + Vector3(0, trunk_height + 2.5, 0), Vector3(1, 1, 1), Color(0.15, 0.45, 0.15))


func _build_gates() -> void:
	# Gates every 30 m towards North (-Z), 5 m wide, 3 m tall, alternating sideways
	var post := BoxMesh.new()
	for i in GATE_COUNT:
		var z := -30.0 * (i + 1)
		var x := 6.0 * sin(float(i) * 0.9)
		for side: float in [-2.5, 2.5]:
			_add(post, Vector3(x + side, 1.5, z), Vector3(0.2, 3.0, 0.2), Color(1.0, 0.55, 0.0))
		_add(post, Vector3(x, 3.0, z), Vector3(5.2, 0.2, 0.2), Color(1.0, 0.55, 0.0))


func _add(mesh: Mesh, at: Vector3, scale_xyz: Vector3, color: Color) -> void:
	var instance := MeshInstance3D.new()
	instance.mesh = mesh
	instance.position = at
	instance.scale = scale_xyz
	instance.material_override = _material(color)
	add_child(instance)


func _material(color: Color) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	return material
