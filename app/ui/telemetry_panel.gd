extends VBoxContainer
## Telemetry dock: FC status from the backend's MSP poller, flight, rates, battery and motors from
## the control API topic, the video outputs, and the tail of simcore's log

const WINDOW_S := 10.0
const LOG_LINES := 8
const LOG_POLL_S := 1.0
const RATE_NAMES := ["p", "q", "r"]

signal rate_changed(text: String)

var _rates: Array[PlotSeries] = []
var _rate_lines: Array[Sparkline] = []
var _rate_values: Array[Label] = []
var _voltage := PlotSeries.new(WINDOW_S)
var _flight_tiles: Dictionary = {}
var _battery_tiles: Dictionary = {}
var _motor_rows: Array[Label] = []
var _message_times: Array[float] = []
var _next_log_poll := 0.0
var _log_pending := false

@onready var _state_tile: PanelContainer = $Fc/Tiles/State
@onready var _state_caption: Label = $Fc/Tiles/State/Box/Caption
@onready var _state_value: Label = $Fc/Tiles/State/Box/Value
@onready var _loop_value: Label = $Fc/Tiles/Loop/Box/Value
@onready var _flags: HFlowContainer = $Fc/Flags
@onready var _flight_meta: Label = $Flight/Head/Meta
@onready var _attitude: Label = $Flight/Attitude
@onready var _battery_meta: Label = $Battery/Head/Meta
@onready var _voltage_line: Sparkline = $Battery/Voltage
@onready var _video_meta: Label = $Video/Head/Meta
@onready var _log_meta: Label = $Log/Head/Meta
@onready var _sim: Label = $Log/Sim
@onready var _log: RichTextLabel = $Log/Text


func camera_panel() -> Node:
	return $Video/CameraFeed


func _ready() -> void:
	for i in 3:
		_rates.append(PlotSeries.new(WINDOW_S))
	_build_rate_rows()
	for key: String in ["alt", "gs", "climb"]:
		_flight_tiles[key] = _tile($Flight/Tiles, {"alt": "ALTITUDE", "gs": "GROUND SPEED", "climb": "CLIMB"}[key])
	for key: String in ["volts", "amps", "used"]:
		_battery_tiles[key] = _tile($Battery/Tiles, {"volts": "VOLTAGE", "amps": "CURRENT", "used": "USED"}[key])
	_voltage_line.series = _voltage
	_show_flight({})
	_show_battery(null)
	_render_fc({})
	SimLink.telemetry.connect(_on_telemetry)
	BackendClient.fc_status.connect(_render_fc)
	BackendClient.status.connect(_on_status)
	BackendClient.response.connect(_on_response)
	camera_panel().feeds_changed.connect(_update_video_meta)
	camera_panel().selection_changed.connect(func(_index: int) -> void: _update_video_meta())
	_update_video_meta()
	_on_status(BackendClient.last_status)


func _build_rate_rows() -> void:
	for i in 3:
		var row := HBoxContainer.new()
		row.theme_type_variation = "Gap10"
		var name := Label.new()
		name.theme_type_variation = "Mono"
		name.text = RATE_NAMES[i]
		name.custom_minimum_size.x = 18
		row.add_child(name)
		var line := Sparkline.new()
		line.series = _rates[i]
		line.custom_minimum_size.y = 32
		line.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(line)
		var value := Label.new()
		value.theme_type_variation = "Mono"
		value.text = "+0.0"
		value.custom_minimum_size.x = 56
		value.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		row.add_child(value)
		$Rates/Rows.add_child(row)
		_rate_lines.append(line)
		_rate_values.append(value)


## A bordered tile: a small caption over a mono value with its unit
func _tile(grid: GridContainer, caption: String) -> Label:
	var tile := PanelContainer.new()
	tile.theme_type_variation = "Tile"
	tile.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var box := VBoxContainer.new()
	box.theme_type_variation = "Stack"
	tile.add_child(box)
	var label := Label.new()
	label.theme_type_variation = "TileCaption"
	label.text = caption
	label.clip_text = true
	label.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
	box.add_child(label)
	var value := Label.new()
	value.theme_type_variation = "MonoLarge"
	value.clip_text = true
	box.add_child(value)
	grid.add_child(tile)
	return value


func _on_telemetry(data: Dictionary) -> void:
	var now := Time.get_ticks_msec() * 1e-3
	_message_times.append(now)
	while _message_times.size() > 1 and _message_times[0] < now - 2.0:
		_message_times.pop_front()
	if _message_times.size() > 1:
		var hz := (_message_times.size() - 1) / maxf(_message_times[-1] - _message_times[0], 1e-3)
		rate_changed.emit("%d Hz" % roundi(hz))
	var sim: Dictionary = data["sim"]
	_sim.text = "%s · overruns %d%s%s" % [
		sim["mode"], int(sim["overruns"]),
		" · PAUSED" if sim["paused"] else "", " · CRASHED" if sim["crashed"] else ""
	]
	_show_flight(data["flight"])
	_show_battery(data.get("battery"))
	_show_motors(data["motors"])
	if sim.get("paused", false):
		return
	var t: float = float(sim.get("sim_time_ns", 0)) * 1e-9
	var rates: Array = data["flight"]["rates_frd_radps"]
	for i in 3:
		var deg := rad_to_deg(float(rates[i]))
		_rates[i].push(t, deg)
		_rate_values[i].text = "%+.1f" % deg
		_rate_lines[i].queue_redraw()
	var battery: Variant = data.get("battery")
	if battery is Dictionary:
		_voltage.push(t, float(battery["voltage_v"]))
		_voltage_line.queue_redraw()


func _show_flight(flight: Dictionary) -> void:
	if flight.is_empty():
		for key: String in _flight_tiles:
			_flight_tiles[key].text = "—"
		return
	_flight_tiles["alt"].text = "%.2f m" % float(flight["altitude_agl_m"])
	_flight_tiles["gs"].text = "%.1f m/s" % float(flight["ground_speed_mps"])
	_flight_tiles["climb"].text = "%+.1f m/s" % float(flight["climb_mps"])
	_attitude.text = "roll %+.1f°  pitch %+.1f°  hdg %03d°" % [
		rad_to_deg(float(flight["roll_rad"])), rad_to_deg(float(flight["pitch_rad"])),
		roundi(fposmod(rad_to_deg(float(flight["heading_rad"])), 360.0))
	]


func _show_battery(battery: Variant) -> void:
	if not (battery is Dictionary):
		for key: String in _battery_tiles:
			_battery_tiles[key].text = "—"
		return
	_battery_tiles["volts"].text = "%.2f V" % float(battery["voltage_v"])
	_battery_tiles["amps"].text = "%.1f A" % float(battery["current_a"])
	_battery_tiles["used"].text = "%d mAh" % roundi(float(battery["consumed_mah"]))
	_battery_meta.text = "SOC %d %%%s" % [roundi(100.0 * float(battery["soc"])), " · CUTOFF" if battery.get("cutoff", false) else ""]


func _show_motors(motors: Array) -> void:
	while _motor_rows.size() < motors.size():
		var label := Label.new()
		label.theme_type_variation = "Mono"
		$Motors/Rows.add_child(label)
		_motor_rows.append(label)
	for i in _motor_rows.size():
		_motor_rows[i].visible = i < motors.size()
		if i < motors.size():
			var motor: Dictionary = motors[i]
			_motor_rows[i].text = "M%d  %3d %%  %6.0f rpm  %5.1f A" % [
				i + 1, roundi(float(motor["command"]) * 100.0), float(motor["rpm"]), float(motor.get("current_a", 0.0))
			]


func _render_fc(fc: Dictionary) -> void:
	for child in _flags.get_children():
		child.queue_free()
	var armed: bool = fc.get("armed", false)
	_state_tile.theme_type_variation = "TileArmed" if armed else "Tile"
	_state_caption.theme_type_variation = "TileCaptionDark" if armed else "TileCaption"
	_state_value.theme_type_variation = "StateValueDark" if armed else "StateValue"
	if fc.is_empty():
		_state_value.text = "No MSP"
		_loop_value.text = "— µs"
		_flags.add_child(_tag("NO MSP", "Tag", "TagText"))
		return
	_state_value.text = "Armed" if armed else "Disarmed"
	_loop_value.text = "%d µs" % int(fc.get("pid_cycle_time_us", 0))
	if armed:
		_flags.add_child(_tag("ARMED", "TagArmed", "TagTextArmed"))
	for flag: String in fc.get("arming_disable_flags", []):
		_flags.add_child(_tag(flag, "Tag", "TagText"))


func _tag(text: String, panel: String, label_type: String) -> PanelContainer:
	var tag := PanelContainer.new()
	tag.theme_type_variation = panel
	var label := Label.new()
	label.theme_type_variation = label_type
	label.text = text
	tag.add_child(label)
	return tag


func _update_video_meta() -> void:
	var feed: Variant = camera_panel().selected_feed()
	_video_meta.text = "" if feed == null else "%s · %d×%d@%d" % [feed.name, feed.size.x, feed.size.y, roundi(feed.fps)]


func _on_status(data: Dictionary) -> void:
	var running: bool = data.get("state", "") == "running"
	var run_dir: Variant = data.get("run_dir")
	_log_meta.text = str(run_dir).get_file() if running and run_dir != null else "—"
	if not running:
		_log.text = "simcore not running. Start a session to stream logs/simcore.log."
		_sim.text = ""
		_render_fc({})


func _process(_delta: float) -> void:
	var now := Time.get_ticks_msec() * 1e-3
	if now < _next_log_poll or _log_pending:
		return
	_next_log_poll = now + LOG_POLL_S
	if BackendClient.last_status.get("state", "") == "running" and BackendClient.is_connected_to_backend:
		_log_pending = true
		BackendClient.request("tail_log", {"process": "simcore", "lines": LOG_LINES})


func _on_response(method: String, ok: bool, result: Dictionary) -> void:
	if method != "tail_log":
		return
	_log_pending = false
	if ok:
		var lines: Array = result.get("lines", [])
		_log.text = "\n".join(PackedStringArray(lines)) if not lines.is_empty() else "simcore has not logged anything yet."
