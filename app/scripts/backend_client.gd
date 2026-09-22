extends Node
## Autoload: starts `simctl serve` and talks to it over TCP (JSON lines)

signal connected()
signal disconnected()
signal status(data: Dictionary)
signal fc_status(data: Dictionary)
signal response(method: String, ok: bool, result: Dictionary)
signal unreachable(seconds: float)

var host: String = "127.0.0.1"
var port: int = 7740
var is_connected_to_backend: bool = false
var last_status: Dictionary = {}
var last_fc: Dictionary = {}

var _tcp := StreamPeerTCP.new()
var _buffer := PackedByteArray()
var _next_id: int = 1
var _pending: Dictionary = {}
var _serve_pid: int = -1
var _started_at_ms: int = 0
var _reconnect_at_ms: int = 0


func repo_root() -> String:
	return ProjectSettings.globalize_path("res://").path_join("..").simplify_path()


func _ready() -> void:
	if not OS.get_cmdline_user_args().has("--attach"):
		_start_serve()
	set_process(true)


func _exit_tree() -> void:
	if _serve_pid > 0 and _tcp.get_status() == StreamPeerTCP.STATUS_CONNECTED:
		# Ask the backend to stop the session and exit; killing the uv wrapper would orphan it
		_tcp.put_data((JSON.stringify({"id": 0, "method": "shutdown", "params": {}}) + "\n").to_utf8_buffer())
		_tcp.poll()
		OS.delay_msec(200)


func _start_serve() -> void:
	# Godot's working directory is the app folder, so every path handed to uv must be absolute
	var root := repo_root()
	# setpriv makes the kernel send TERM to uv if this app dies without a chance to say shutdown
	var args := PackedStringArray([
		"--pdeathsig", "TERM", "uv", "run", "--project", root.path_join("tools"), "simctl",
		"serve", "--base-dir", root, "--port", str(port),
	])
	_serve_pid = OS.create_process("setpriv", args, false)
	if _serve_pid <= 0:
		push_error("could not start simctl serve (is uv installed?)")
	_started_at_ms = Time.get_ticks_msec()


func _process(_delta: float) -> void:
	_tcp.poll()
	var tcp_status := _tcp.get_status()
	if tcp_status == StreamPeerTCP.STATUS_CONNECTED:
		if not is_connected_to_backend:
			is_connected_to_backend = true
			connected.emit()
			request("subscribe", {"topic": "status"})
			request("subscribe", {"topic": "fc"})
		_read_lines()
		return
	if is_connected_to_backend:
		is_connected_to_backend = false
		_pending.clear()
		disconnected.emit()
	if tcp_status != StreamPeerTCP.STATUS_CONNECTING and Time.get_ticks_msec() >= _reconnect_at_ms:
		_reconnect_at_ms = Time.get_ticks_msec() + 500
		if _started_at_ms > 0 and Time.get_ticks_msec() - _started_at_ms > 5000:
			unreachable.emit((Time.get_ticks_msec() - _started_at_ms) / 1000.0)
		_tcp = StreamPeerTCP.new()
		_tcp.connect_to_host(host, port)


func _read_lines() -> void:
	var available := _tcp.get_available_bytes()
	if available <= 0:
		return
	var chunk: Array = _tcp.get_data(available)
	if chunk[0] != OK:
		return
	_buffer.append_array(chunk[1])
	while true:
		var newline := _buffer.find(10)
		if newline < 0:
			return
		var line := _buffer.slice(0, newline).get_string_from_utf8()
		_buffer = _buffer.slice(newline + 1)
		_handle_line(line)


func _handle_line(line: String) -> void:
	var message: Variant = JSON.parse_string(line)
	if not message is Dictionary:
		return
	var doc: Dictionary = message
	if doc.has("event"):
		var data: Dictionary = doc["data"]
		if doc["event"] == "status":
			last_status = data
			status.emit(data)
		elif doc["event"] == "fc":
			last_fc = data
			fc_status.emit(data)
		return
	var id: int = int(doc.get("id", -1))
	if not _pending.has(id):
		return
	var method: String = _pending[id]
	_pending.erase(id)
	var ok: bool = doc.get("ok", false)
	var payload: Dictionary = doc.get("result", doc.get("error", {}))
	response.emit(method, ok, payload)


## Sends a backend request; the answer arrives as response(method, ok, payload)
func request(method: String, params: Dictionary = {}) -> int:
	if _tcp.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		response.emit(method, false, {"code": "not_connected", "message": "backend not connected"})
		return -1
	var id := _next_id
	_next_id += 1
	_pending[id] = method
	_tcp.put_data((JSON.stringify({"id": id, "method": method, "params": params}) + "\n").to_utf8_buffer())
	return id
