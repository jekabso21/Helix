extends HBoxContainer
## Session picker, Start/Stop/Pause/Reset, sim time and process lights

@onready var _session: OptionButton = $Session
@onready var _start: Button = $Start
@onready var _stop: Button = $Stop
@onready var _pause: Button = $Pause
@onready var _reset: Button = $Reset
@onready var _time: Label = $SimTime
@onready var _lights: Label = $Lights
@onready var _message: Label = $Message

var _sessions: Array = []
var _paused: bool = false


func _ready() -> void:
	_start.pressed.connect(_on_start)
	_stop.pressed.connect(_on_stop)
	_pause.pressed.connect(_on_pause)
	_reset.pressed.connect(func() -> void: SimLink.request("reset"))
	BackendClient.connected.connect(func() -> void: BackendClient.request("list_sessions"))
	BackendClient.disconnected.connect(_render_lights)
	BackendClient.response.connect(_on_backend_response)
	BackendClient.status.connect(func(_data: Dictionary) -> void: _render_lights())
	SimLink.api_connected.connect(_render_lights)
	SimLink.api_disconnected.connect(_render_lights)
	SimLink.command_result.connect(_on_command_result)
	SimLink.telemetry.connect(_on_telemetry)
	_render_lights()


func _on_start() -> void:
	if _session.selected < 0:
		_message.text = "choose a session"
		return
	var entry: Dictionary = _sessions[_session.selected]
	_message.text = "starting %s" % entry["name"]
	BackendClient.request("start", {"path": entry["path"]})


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
			var label: String = entry["name"]
			if entry.has("error"):
				label += " (invalid)"
			_session.add_item(label)
		_session.disabled = _sessions.is_empty()
		if _sessions.is_empty():
			_message.text = "no sessions found"
	elif method == "start":
		_message.text = "running in %s" % result["run_dir"] if ok else "start failed: %s" % result.get("message", "")
	elif method == "stop":
		_message.text = "stopped" if ok else "stop failed: %s" % result.get("message", "")
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
	_pause.text = "Resume" if _paused else "Pause"


func _on_telemetry(data: Dictionary) -> void:
	_paused = data["sim"]["paused"]
	_pause.text = "Resume" if _paused else "Pause"


func _process(_delta: float) -> void:
	var state := SimLink.last_state
	if state != null:
		var seconds := state.sim_time_ns * 1e-9
		_time.text = "t %02d:%06.3f" % [int(seconds / 60.0), fmod(seconds, 60.0)]


func _render_lights() -> void:
	var backend := "backend %s" % ("●" if BackendClient.is_connected_to_backend else "○")
	var status := BackendClient.last_status
	var processes: Array = status.get("processes", [])
	var bf := "○"
	var core := "○"
	for p: Dictionary in processes:
		var mark: String = "●" if p["state"] == "running" else "✖"
		if p["name"] == "betaflight":
			bf = mark
		elif p["name"] == "simcore":
			core = mark
	var api := "●" if SimLink.api_is_connected else "○"
	_lights.text = "%s   BF %s   core %s   api %s" % [backend, bf, core, api]
	var running: bool = status.get("state", "idle") == "running"
	_start.disabled = running or not BackendClient.is_connected_to_backend
	_stop.disabled = not running
	_pause.disabled = not SimLink.api_is_connected
	_reset.disabled = not SimLink.api_is_connected
