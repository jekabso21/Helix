extends VBoxContainer
## Failures tab: inject motor, battery and sensor failures live, see what is active, clear it

const MOTOR_COUNT := 4
const AXES := ["all", "x", "y", "z"]
const LABELS := {
	"motor_out": "Motor out", "motor_degraded": "Weak motor", "esc_desync": "Desync",
	"prop_damage": "Prop", "battery_weak_cell": "Weak cell",
	"battery_high_resistance": "High resistance", "imu_noise": "Noise", "imu_bias": "Bias",
	"imu_stuck": "Stuck", "imu_saturation": "Saturated", "baro_stuck": "Baro stuck",
	"baro_offset": "Baro jump",
}

var _running := false
var _sliders: Dictionary = {}
var _values: Dictionary = {}
var _buttons: Array[Button] = []
var _active_ids: Array = []

@onready var _motors: VBoxContainer = $Motors/Rows
@onready var _motor_params: VBoxContainer = $Motors/Params
@onready var _battery: VBoxContainer = $Battery
@onready var _sensor: OptionButton = $Sensors/Pick/Sensor
@onready var _axis: OptionButton = $Sensors/Pick/Axis
@onready var _sensor_buttons: HFlowContainer = $Sensors/Buttons
@onready var _sensor_params: VBoxContainer = $Sensors/Params
@onready var _timing: VBoxContainer = $Timing
@onready var _active: VBoxContainer = $Active/Rows
@onready var _active_meta: Label = $Active/Head/Meta
@onready var _clear_all: Button = $Active/Head/ClearAll
@onready var _message: Label = $Message


func _ready() -> void:
	_add_slider(_motor_params, "output_pct", "Weak output", 0, 100, 5, 50, "%d %%")
	_add_slider(_motor_params, "period_ms", "Desync every", 50, 2000, 50, 500, "%d ms")
	_add_slider(_motor_params, "dropout_ms", "Desync for", 10, 500, 10, 50, "%d ms")
	_add_slider(_motor_params, "thrust_loss_pct", "Prop thrust loss", 0, 90, 5, 20, "%d %%")
	_add_slider(_motor_params, "vibration_scale", "Prop vibration", 1, 50, 1, 10, "×%d")
	for i in MOTOR_COUNT:
		_motors.add_child(_motor_row(i + 1))
	var battery_buttons := HFlowContainer.new()
	battery_buttons.theme_type_variation = "HFlow4"
	battery_buttons.add_child(_inject_button("Weak cell", func() -> Dictionary: return {"type": "battery_weak_cell", "drop_v": _value("drop_v")}))
	battery_buttons.add_child(_inject_button("High resistance", func() -> Dictionary: return {"type": "battery_high_resistance", "cell_resistance_scale": _value("cell_resistance_scale")}))
	_battery.add_child(battery_buttons)
	_add_slider(_battery, "drop_v", "Cell drop", 0.1, 2.0, 0.1, 0.5, "%.1f V")
	_add_slider(_battery, "cell_resistance_scale", "Resistance", 1, 20, 0.5, 3, "×%.1f")
	for name: String in ["gyro", "accel"]:
		_sensor.add_item(name)
	for name: String in AXES:
		_axis.add_item(name)
	_sensor.item_selected.connect(func(_i: int) -> void: _render_bias_unit())
	_sensor_buttons.add_child(_inject_button("Noise", func() -> Dictionary: return _sensor_request("imu_noise", {"noise_scale": _value("noise_scale")})))
	_sensor_buttons.add_child(_inject_button("Bias", func() -> Dictionary: return _bias_request()))
	_sensor_buttons.add_child(_inject_button("Stuck", func() -> Dictionary: return _sensor_request("imu_stuck", {"axis": AXES[_axis.selected]})))
	_sensor_buttons.add_child(_inject_button("Saturate", func() -> Dictionary: return _sensor_request("imu_saturation", {"range_pct": _value("range_pct")})))
	_sensor_buttons.add_child(_inject_button("Baro stuck", func() -> Dictionary: return {"type": "baro_stuck"}))
	_sensor_buttons.add_child(_inject_button("Baro jump", func() -> Dictionary: return {"type": "baro_offset", "offset_hpa": _value("offset_hpa")}))
	_add_slider(_sensor_params, "noise_scale", "Noise", 1, 50, 1, 5, "×%d")
	_add_slider(_sensor_params, "bias", "Bias", -100, 100, 1, 20, "%+d °/s")
	_add_slider(_sensor_params, "range_pct", "Range left", 5, 100, 5, 25, "%d %%")
	_add_slider(_sensor_params, "offset_hpa", "Baro jump", -5, 5, 0.1, -1, "%+.1f hPa")
	_add_slider(_timing, "duration_s", "Duration", 0, 30, 0.5, 0, "%.1f s")
	_clear_all.pressed.connect(func() -> void: _request("clear_failure", {"all": true}))
	BackendClient.status.connect(_on_status)
	BackendClient.response.connect(_on_response)
	SimLink.telemetry.connect(_on_telemetry)
	_render_bias_unit()
	_on_status(BackendClient.last_status)
	_render_active([])


## A row of buttons for one motor; Betaflight's motor numbers, which the telemetry uses too
func _motor_row(motor: int) -> HBoxContainer:
	var row := HBoxContainer.new()
	row.theme_type_variation = "Gap6"
	var name := Label.new()
	name.theme_type_variation = "Kicker"
	name.text = "M%d" % motor
	name.custom_minimum_size.x = 28
	row.add_child(name)
	row.add_child(_inject_button("Out", func() -> Dictionary: return {"type": "motor_out", "motor": motor}))
	row.add_child(_inject_button("Weak", func() -> Dictionary: return {"type": "motor_degraded", "motor": motor, "output_pct": _value("output_pct")}))
	row.add_child(_inject_button("Desync", func() -> Dictionary: return {"type": "esc_desync", "motor": motor, "period_ms": _value("period_ms"), "dropout_ms": minf(_value("dropout_ms"), _value("period_ms") - 10.0)}))
	row.add_child(_inject_button("Prop", func() -> Dictionary: return {"type": "prop_damage", "motor": motor, "thrust_loss_pct": _value("thrust_loss_pct"), "vibration_scale": _value("vibration_scale")}))
	return row


func _inject_button(text: String, make: Callable) -> Button:
	var button := Button.new()
	button.theme_type_variation = "SmallButton"
	button.text = text
	button.pressed.connect(func() -> void: _inject(make.call()))
	_buttons.append(button)
	return button


func _add_slider(parent: Container, key: String, label: String, lo: float, hi: float, step: float, value: float, format: String) -> void:
	var row := HBoxContainer.new()
	row.theme_type_variation = "Gap10"
	var name := Label.new()
	name.theme_type_variation = "Muted"
	name.text = label
	name.custom_minimum_size.x = 104
	row.add_child(name)
	var slider := HSlider.new()
	slider.min_value = lo
	slider.max_value = hi
	slider.step = step
	slider.value = value
	slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	slider.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	row.add_child(slider)
	var shown := Label.new()
	shown.theme_type_variation = "Mono"
	shown.custom_minimum_size.x = 70
	shown.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	row.add_child(shown)
	parent.add_child(row)
	_sliders[key] = slider
	_values[key] = [shown, format]
	slider.value_changed.connect(func(v: float) -> void: shown.text = (_values[key][1] as String) % v)
	shown.text = format % value


func _value(key: String) -> float:
	return (_sliders[key] as HSlider).value


func _render_bias_unit() -> void:
	var gyro := _sensor.selected <= 0
	_values["bias"][1] = "%+d °/s" if gyro else "%+.1f m/s²"
	var slider: HSlider = _sliders["bias"]
	slider.min_value = -100.0 if gyro else -10.0
	slider.max_value = 100.0 if gyro else 10.0
	slider.step = 1.0 if gyro else 0.1
	slider.value = 20.0 if gyro else 2.0
	(_values["bias"][0] as Label).text = (_values["bias"][1] as String) % slider.value


func _sensor_request(type: String, extra: Dictionary) -> Dictionary:
	var request := {"type": type, "sensor": _sensor.get_item_text(_sensor.selected)}
	request.merge(extra)
	return request


func _bias_request() -> Dictionary:
	var gyro := _sensor.selected <= 0
	var unit := "step_dps" if gyro else "step_mps2"
	return _sensor_request("imu_bias", {"axis": AXES[_axis.selected], unit: _value("bias")})


func _inject(request: Dictionary) -> void:
	var duration := _value("duration_s")
	if duration > 0.0:
		request["duration_s"] = duration
	_request("inject_failure", request)


func _request(method: String, params: Dictionary) -> void:
	if not _running:
		_message.text = "Start a session to inject failures."
		return
	BackendClient.request(method, params)


func _on_response(method: String, ok: bool, result: Dictionary) -> void:
	if method == "inject_failure":
		_message.text = "Failure #%d active from t %.2f s." % [int(result.get("failure_id", 0)), float(result.get("applied_at_ns", 0)) * 1e-9] if ok else "Not injected: %s" % result.get("message", "")
	elif method == "clear_failure":
		_message.text = "Cleared." if ok else "Not cleared: %s" % result.get("message", "")


func _on_status(data: Dictionary) -> void:
	var was_running := _running
	_running = data.get("state", "") == "running"
	if _running and not was_running:
		_message.text = ""
	for button in _buttons:
		button.disabled = not _running
	_clear_all.disabled = not _running
	if not _running:
		_render_active([])
		_message.text = "Start a session to inject failures. Each one changes the drone the way the real fault would; Betaflight has to cope."


func _on_telemetry(data: Dictionary) -> void:
	var list: Array = data["sim"].get("active_failures", [])
	_render_active(list)


func _render_active(list: Array) -> void:
	var ids := list.map(func(f: Dictionary) -> int: return int(f["failure_id"]))
	_active_meta.text = "%d active" % list.size() if not list.is_empty() else "none"
	if ids == _active_ids and not list.is_empty():
		for i in list.size():
			var row := _active.get_child(i)
			(row.get_child(0) as Label).text = describe(list[i], _sim_time(list))
		return
	_active_ids = ids
	for child in _active.get_children():
		child.queue_free()
	for failure: Dictionary in list:
		var row := HBoxContainer.new()
		row.theme_type_variation = "Gap6"
		var text := Label.new()
		text.theme_type_variation = "Mono"
		text.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		text.clip_text = true
		text.text = describe(failure, _sim_time(list))
		text.tooltip_text = text.text
		row.add_child(text)
		var clear := Button.new()
		clear.theme_type_variation = "SmallButton"
		clear.text = "Clear"
		var id := int(failure["failure_id"])
		clear.pressed.connect(func() -> void: _request("clear_failure", {"failure_id": id}))
		row.add_child(clear)
		_active.add_child(row)


func _sim_time(_list: Array) -> float:
	var state := SimLink.last_state
	return state.sim_time_ns * 1e-9 if state != null else 0.0


## One line per failure: what, where, how strong, and how long it has left
static func describe(failure: Dictionary, now_s: float) -> String:
	var target: Dictionary = failure.get("target", {})
	var params: Dictionary = failure.get("params", {})
	var where := ""
	if target.has("motor"):
		where = " M%d" % int(target["motor"])
	elif target.has("sensor"):
		where = " %s%s" % [target["sensor"], "" if str(target.get("axis", "all")) == "all" else " " + str(target["axis"])]
	var detail := ""
	match str(failure.get("type", "")):
		"motor_degraded":
			detail = " %d %%" % roundi(float(params["output_gain"]) * 100.0)
		"esc_desync":
			detail = " %d/%d ms" % [roundi(float(params["dropout_s"]) * 1000.0), roundi(float(params["period_s"]) * 1000.0)]
		"prop_damage":
			detail = " -%d %% ×%d" % [roundi(float(params["thrust_loss"]) * 100.0), roundi(float(params["vibration_scale"]))]
		"battery_weak_cell":
			detail = " -%.1f V" % float(params["drop_v"])
		"battery_high_resistance":
			detail = " ×%.1f" % float(params["cell_resistance_scale"])
		"imu_noise":
			detail = " ×%d" % roundi(float(params["noise_scale"]))
		"imu_bias":
			var step := float(params["step"])
			detail = " %+d °/s" % roundi(rad_to_deg(step)) if str(target.get("sensor", "")) == "gyro" else " %+.1f m/s²" % step
		"imu_saturation":
			detail = " %d %%" % roundi(float(params["range_scale"]) * 100.0)
		"baro_offset":
			detail = " %+.1f hPa" % (float(params["offset_pa"]) / 100.0)
	var timing := ""
	if not bool(failure.get("in_effect", true)):
		timing = " · in %.1f s" % maxf(float(failure.get("start_s", 0.0)) - now_s, 0.0)
	elif failure.get("end_s") != null:
		timing = " · %.1f s left" % maxf(float(failure["end_s"]) - now_s, 0.0)
	var type := str(failure.get("type", ""))
	return "#%d %s%s%s%s" % [int(failure.get("failure_id", 0)), LABELS.get(type, type), where, detail, timing]
