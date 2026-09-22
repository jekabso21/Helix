extends VBoxContainer
## Drone tab: editable parts, compiled mass properties, static hover loads, Preview and Apply

const COLUMNS := ["part", "mass g", "x mm", "y mm", "z mm"]

var _parts: Array = []
var _overrides: Dictionary = {}
var _rows: Dictionary = {}
var _select_buttons: Dictionary = {}
var _selected := ""
var _updating := false
var _session_running := false

@onready var _summary: RichTextLabel = $Summary
@onready var _table: GridContainer = $Parts
@onready var _preview: Button = $Actions/Preview
@onready var _apply: Button = $Actions/Apply
@onready var _revert: Button = $Actions/Revert
@onready var _overlays: CheckBox = $Actions/Overlays
@onready var _message: Label = $Message


func _ready() -> void:
	_table.columns = COLUMNS.size()
	_preview.pressed.connect(_on_preview)
	_apply.pressed.connect(_on_apply)
	_revert.pressed.connect(_on_revert)
	_overlays.toggled.connect(_on_overlays_toggled)
	BackendClient.status.connect(_on_status)
	BackendClient.response.connect(_on_response)
	_summary.text = "Start a session to load its drone."
	_set_buttons()
	if not BackendClient.last_status.is_empty():
		_on_status(BackendClient.last_status)


func _on_status(data: Dictionary) -> void:
	var running: bool = data.get("state", "") == "running" and data.get("drone") != null
	if running and not _session_running:
		BackendClient.request("get_drone")
	if not running and _session_running:
		_parts = []
		_overrides = {}
		_fill_table()
		_summary.text = "Start a session to load its drone."
	_session_running = running
	_set_buttons()


func _on_response(method: String, ok: bool, result: Dictionary) -> void:
	if method not in ["get_drone", "preview_drone", "apply_drone"]:
		return
	if not ok:
		_message.text = "%s failed: %s" % [method, result.get("message", "")]
		_set_buttons()
		return
	if method != "preview_drone":
		_parts = result.get("parts", [])
		_overrides = result.get("overrides", {})
		_fill_table()
	_render_summary(result, method)
	_message.text = {
		"get_drone": "", "preview_drone": "preview only, nothing applied",
		"apply_drone": "applied revision %d; drone reset to spawn" % int((result.get("drone", {}) as Dictionary).get("revision", 0)),
	}[method]
	_set_buttons()


func _render_summary(result: Dictionary, method: String) -> void:
	var cg: Array = result["cg_from_origin_frd_m"]
	var inertia: Array = result["inertia_frd_kg_m2"]
	var lines: PackedStringArray = []
	lines.append("[b]%s[/b]  %s" % [result.get("name", ""), "preview" if method == "preview_drone" else "applied"])
	lines.append("mass %.1f g  (generated arms, motors, props %.1f g)" % [
		float(result["mass_kg"]) * 1000.0, float(result.get("generated_mass_kg", 0.0)) * 1000.0
	])
	lines.append("CG from origin  x %+.1f  y %+.1f  z %+.1f mm" % [cg[0] * 1000.0, cg[1] * 1000.0, cg[2] * 1000.0])
	lines.append("inertia  Ixx %.2e  Iyy %.2e  Izz %.2e kg m2" % [inertia[0][0], inertia[1][1], inertia[2][2]])
	lines.append("[b]Static hover[/b]")
	for h: Dictionary in result.get("hover", []):
		lines.append("  motor %d  %.3f N  %4.1f%%" % [int(h["bf_index"]), float(h["thrust_n"]), 100.0 * float(h["fraction"])])
	_summary.text = "\n".join(lines)


func _fill_table() -> void:
	_updating = true
	for child in _table.get_children():
		child.queue_free()
	_rows.clear()
	_select_buttons.clear()
	for title: String in COLUMNS:
		var header := Label.new()
		header.text = title
		_table.add_child(header)
	for part: Dictionary in _parts:
		var part_name := str(part["name"])
		var select := Button.new()
		select.text = part_name
		select.toggle_mode = true
		select.button_pressed = part_name == _selected
		select.pressed.connect(_on_select.bind(part_name))
		_table.add_child(select)
		_select_buttons[part_name] = select
		var fields: Array[SpinBox] = []
		var values := [float(part["mass_g"]), part["pos_mm"][0], part["pos_mm"][1], part["pos_mm"][2]]
		for i in values.size():
			var spin := SpinBox.new()
			spin.min_value = 0.1 if i == 0 else -1000.0
			spin.max_value = 5000.0 if i == 0 else 1000.0
			spin.step = 0.1 if i == 0 else 1.0
			spin.value = values[i]
			spin.custom_minimum_size = Vector2(70, 0)
			spin.value_changed.connect(_on_field_changed.bind(part_name))
			_table.add_child(spin)
			fields.append(spin)
		_rows[part_name] = fields
	_updating = false


func _on_select(part_name: String) -> void:
	_selected = "" if _selected == part_name else part_name
	for other_name: String in _select_buttons:
		(_select_buttons[other_name] as Button).button_pressed = other_name == _selected
	var drone := get_tree().get_first_node_in_group("drone")
	if drone != null:
		drone.call("highlight_part", _selected)


func _on_field_changed(_value: float, part_name: String) -> void:
	if _updating:
		return
	var fields: Array = _rows[part_name]
	var entry: Dictionary = _overrides.get("parts", {}).get(part_name, {})
	entry["mass_g"] = fields[0].value
	entry["pos_mm"] = [fields[1].value, fields[2].value, fields[3].value]
	var parts: Dictionary = _overrides.get("parts", {})
	parts[part_name] = entry
	_overrides["parts"] = parts
	_message.text = "edited; Preview to see the effect, Apply to load it into the sim"
	_set_buttons()


func _on_preview() -> void:
	BackendClient.request("preview_drone", {"overrides": _overrides})


func _on_apply() -> void:
	_message.text = "applying..."
	BackendClient.request("apply_drone", {"overrides": _overrides})


func _on_revert() -> void:
	_overrides = {}
	BackendClient.request("get_drone")


func _on_overlays_toggled(on: bool) -> void:
	var drone := get_tree().get_first_node_in_group("drone")
	if drone != null:
		drone.call("set_overlays_visible", on)


func _set_buttons() -> void:
	var have_parts := not _parts.is_empty()
	_preview.disabled = not have_parts
	_apply.disabled = not (have_parts and _session_running)
	_revert.disabled = not have_parts
