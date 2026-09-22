extends Node3D
## Flat ground with a grid; the drone and cameras are children of this scene

const GROUND_SIZE := 400.0
const GRID_STEP := 5.0
const GRID_EXTENT := 100.0


func _ready() -> void:
	_build_ground()
	_build_grid()


func _build_ground() -> void:
	var ground := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(GROUND_SIZE, GROUND_SIZE)
	ground.mesh = plane
	var material := StandardMaterial3D.new()
	material.albedo_color = Color(0.36, 0.48, 0.3)
	ground.material_override = material
	add_child(ground)


func _build_grid() -> void:
	var lines := ImmediateMesh.new()
	lines.surface_begin(Mesh.PRIMITIVE_LINES)
	var n := int(GRID_EXTENT / GRID_STEP)
	for i in range(-n, n + 1):
		var offset := i * GRID_STEP
		lines.surface_add_vertex(Vector3(offset, 0.01, -GRID_EXTENT))
		lines.surface_add_vertex(Vector3(offset, 0.01, GRID_EXTENT))
		lines.surface_add_vertex(Vector3(-GRID_EXTENT, 0.01, offset))
		lines.surface_add_vertex(Vector3(GRID_EXTENT, 0.01, offset))
	lines.surface_end()
	var grid := MeshInstance3D.new()
	grid.mesh = lines
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_color = Color(0.2, 0.28, 0.18)
	grid.material_override = material
	add_child(grid)
	# North marker: a red bar on the -Z axis at the edge of the grid
	var north := MeshInstance3D.new()
	var bar := BoxMesh.new()
	bar.size = Vector3(0.5, 0.5, 4.0)
	north.mesh = bar
	north.position = Vector3(0.0, 0.25, -GRID_EXTENT)
	var red := StandardMaterial3D.new()
	red.albedo_color = Color(0.9, 0.1, 0.1)
	north.material_override = red
	add_child(north)
