extends PanelContainer
## Session picker, Start/Stop/Pause/Reset, sim time, session state and process lights

const PROCESSES := ["backend", "betaflight", "simcore", "simvideo"]
const SETTINGS_PATH := "user://ui.cfg"
## Flown with a controller, so it is the session a developer most often wants
const DEFAULT_SESSION := "dev_gamepad"

@onready var _session: OptionButton = $Row/Session
@onready var _start: Button = $Row/Start
@onready var _stop: Button = $Row/Stop
@onready var _pause: Button = $Row/Pause
@onready var _reset: Button = $Row/Reset
@onready var _time: Label = $Row/SimTime
@onready var _state: Label = $Row/State/Text
@onready var _message: Label = $Row/Message
@onready var _lights_box: HBoxContainer = $Row/Lights

var _sessions: Array = []
var _paused: bool = false
var _lights: Dictionary = {}


func _ready() -> void:
	_build_lights()
	_start.pressed.connect(_on_start)
	_stop.pressed.connect(_on_stop)
	_pause.pressed.connect(_on_pause)
	_reset.pressed.connect(func() -> void: SimLink.request("reset"))
	BackendClient.connected.connect(func() -> void:
		_message.text = ""
		BackendClient.request("list_sessions"))
	BackendClient.unreachable.connect(func(seconds: float) -> void:
		if not BackendClient.is_connected_to_backend:
			_message.text = "backend not reachable after %.0f s: run 'uv run --project tools simctl serve' by hand and check its output" % seconds)
	BackendClient.disconnected.connect(_render_lights)
	BackendClient.response.connect(_on_backend_response)
	BackendClient.status.connect(func(_data: Dictionary) -> void: _render_lights())
	BackendClient.video.connect(func(_data: Dictionary) -> void: _render_lights())
	SimLink.api_connected.connect(_render_lights)
	SimLink.api_disconnected.connect(_render_lights)
	SimLink.command_result.connect(_on_command_result)
	SimLink.telemetry.connect(_on_telemetry)
	_render_lights()


func _build_lights() -> void:
	for name: String in PROCESSES:
		var light := HBoxContainer.new()
		light.theme_type_variation = "Gap6"
		var dot := StatusDot.new()
		light.add_child(dot)
		var label := Label.new()
		label.theme_type_variation = "MonoSoft"
		label.text = name
		light.add_child(label)
		var meta := Label.new()
		meta.theme_type_variation = "MonoMuted"
		light.add_child(meta)
		_lights_box.add_child(light)
		_lights[name] = {"box": light, "dot": dot, "label": label, "meta": meta}


func show_message(text: String) -> void:
	_message.text = text


## Narrow windows drop detail first: light details, then names and labels, then the brand
func fit_width(width: float) -> void:
	var tight := width < 1250.0
	var narrow := width < 1000.0
	for name: String in _lights:
		_lights[name]["meta"].visible = width >= 1500.0
		_lights[name]["label"].visible = not tight
	$Row/Version.visible = not tight
	$Row/SessionLabel.visible = not tight
	$Row/Brand.visible = not narrow
	$Row/Rule.visible = not narrow
	$Row.theme_type_variation = "Gap8" if narrow else "Gap14"
	_session.custom_minimum_size.x = 120.0 if narrow else (160.0 if tight else 230.0)
	_pause.custom_minimum_size.x = 0.0 if narrow else 70.0
	_time.custom_minimum_size.x = 0.0 if narrow else 118.0


static func session_label(entry: Dictionary) -> String:
	if entry.has("error"):
		return "%s  ·  invalid" % entry["name"]
	var duration: Variant = entry.get("duration_s")
	# a duration of 0 runs until stopped
	var length := "∞" if duration == null or float(duration) <= 0.0 else "%s s" % str(snappedf(float(duration), 0.1)).trim_suffix(".0")
	return "%s  ·  %s  ·  %s" % [entry["name"], entry.get("input", "?"), length]


## The last session started from this app, else the gamepad dev session, else the first one
static func preferred_session(sessions: Array, last: String) -> int:
	for wanted: String in [last, DEFAULT_SESSION]:
		if wanted == "":
			continue
		for i in sessions.size():
			if str(sessions[i].get("name", "")) == wanted and not sessions[i].has("error"):
				return i
	return 0 if not sessions.is_empty() else -1


static func clock_text(sim_time_ns: int) -> String:
	var seconds := sim_time_ns * 1e-9
	return "t %02d:%05.2f" % [int(seconds / 60.0), fmod(seconds, 60.0)]


func _on_start() -> void:
	if _session.selected < 0:
		_message.text = "choose a session"
		return
	var entry: Dictionary = _sessions[_session.selected]
	_message.text = "starting %s" % entry["name"]
	BackendClient.request("start", {"path": entry["path"]})
	var settings := ConfigFile.new()
	settings.load(SETTINGS_PATH)
	settings.set_value("session", "last", str(entry["name"]))
	settings.save(SETTINGS_PATH)


func _on_stop() -> void:
	_message.text = "stopping"
	BackendClient.request("stop")


func _on_pause() -> void:
	SimLink.request("resume" if _paused else "pause")


func _on_backend_response(method: String, ok: bool, result: Dictionary) -> void:
	if method == "list_sessions" and ok:
		_sessions = result["sessions"]
		_session.clear()
		for entry: Dictionary in _sessions:
			_session.add_item(session_label(entry))
		var settings := ConfigFile.new()
		settings.load(SETTINGS_PATH)
		var pick := preferred_session(_sessions, str(settings.get_value("session", "last", "")))
		if pick >= 0:
			_session.select(pick)
		_session.disabled = _sessions.is_empty()
		if _sessions.is_empty():
			_message.text = "no sessions found"
	elif method == "start":
		_message.text = "" if ok else "start failed: %s" % result.get("message", "")
	elif method == "stop":
		_message.text = "" if ok else "stop failed: %s" % result.get("message", "")
	_render_lights()


func _on_command_result(method: String, ok: bool, result: Dictionary) -> void:
	if method == "pause" and ok:
		_paused = true
	elif method == "resume" and ok:
		_paused = false
	elif method == "reset" and ok:
		_message.text = "reset at t %.2f s" % [float(result["applied_at_ns"]) * 1e-9]
	elif not ok:
		_message.text = "%s: %s" % [method, result.get("message", "")]
	_render_state()


func _on_telemetry(data: Dictionary) -> void:
	var paused: bool = data["sim"]["paused"]
	if paused != _paused:
		_paused = paused
		_render_state()


func _process(_delta: float) -> void:
	var state := SimLink.last_state
	if state != null:
		_time.text = clock_text(state.sim_time_ns)


func _render_state() -> void:
	var state := str(BackendClient.last_status.get("state", "idle"))
	if state == "running" and _paused:
		state = "paused"
	_state.text = state.to_upper()
	_pause.text = "Resume" if _paused else "Pause"


func _render_lights() -> void:
	var status := BackendClient.last_status
	var backend: Dictionary = _lights["backend"]
	backend["dot"].state = StatusDot.State.ON if BackendClient.is_connected_to_backend else StatusDot.State.OFF
	backend["meta"].text = str(BackendClient.port) if BackendClient.is_connected_to_backend else "—"
	var by_name := {}
	for p: Dictionary in status.get("processes", []):
		by_name[p["name"]] = p
	for name: String in ["betaflight", "simcore", "simvideo"]:
		var light: Dictionary = _lights[name]
		var process: Variant = by_name.get(name)
		if process == null:
			light["dot"].state = StatusDot.State.OFF
			light["meta"].text = "—"
			light["box"].tooltip_text = "%s: not running" % name
			continue
		var running: bool = process["state"] == "running"
		light["dot"].state = StatusDot.State.ON if running else StatusDot.State.FAIL
		light["box"].tooltip_text = "%s: %s%s" % [
			name, process["state"],
			"" if process.get("exit_code") == null else " (exit %d)" % int(process["exit_code"])
		]
		light["meta"].text = _process_meta(name, process) if running else "exit"
	_render_state()
	var running: bool = status.get("state", "idle") == "running"
	_start.disabled = running or not BackendClient.is_connected_to_backend
	_stop.disabled = not running
	_pause.disabled = not SimLink.api_is_connected
	_reset.disabled = not SimLink.api_is_connected


func _process_meta(name: String, process: Dictionary) -> String:
	match name:
		"betaflight":
			return "SITL"
		"simcore":
			return "%ds" % int(float(process.get("uptime_s", 0.0)))
		"simvideo":
			var fps: Variant = BackendClient.last_video.get("input_fps")
			return "%dfps" % roundi(float(fps)) if fps != null else "up"
	return ""
