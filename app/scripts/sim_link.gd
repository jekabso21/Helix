extends Node
## Autoload: RenderState over UDP and the simcore control API over TCP (JSON lines)

signal render_state(state: RenderState)
signal telemetry(data: Dictionary)
signal input_raw(data: Dictionary)
signal api_connected()
signal api_disconnected()
signal command_result(method: String, ok: bool, result: Dictionary)

const TELEMETRY_RATE_HZ := 30.0
const INPUT_RAW_RATE_HZ := 20.0

var render_port: int = 7710
var api_host: String = "127.0.0.1"
var api_port: int = 7700
var render_states_received: int = 0
var last_state: RenderState = null
var api_is_connected: bool = false

var _udp := PacketPeerUDP.new()
var _tcp := StreamPeerTCP.new()
var _buffer := PackedByteArray()
var _next_id: int = 1
var _pending: Dictionary = {}
var _reconnect_at_ms: int = 0
var _subscribed: bool = false


func _ready() -> void:
	set_process(true)
	_bind_udp()


func _bind_udp() -> void:
	_udp.close()
	var error := _udp.bind(render_port, "127.0.0.1")
	if error != OK:
		push_warning("RenderState port %d busy: %s" % [render_port, error_string(error)])


func _process(_delta: float) -> void:
	_poll_udp()
	_poll_tcp()


func _poll_udp() -> void:
	while _udp.get_available_packet_count() > 0:
		var state := RenderState.decode(_udp.get_packet())
		if state == null:
			continue
		render_states_received += 1
		last_state = state
		render_state.emit(state)


func _poll_tcp() -> void:
	_tcp.poll()
	var status := _tcp.get_status()
	if status == StreamPeerTCP.STATUS_CONNECTED:
		if not api_is_connected:
			api_is_connected = true
			api_connected.emit()
			_subscribed = false
			request("subscribe", {"topic": "telemetry", "rate_hz": TELEMETRY_RATE_HZ})
			request("subscribe", {"topic": "input_raw", "rate_hz": INPUT_RAW_RATE_HZ})
		_read_lines()
		return
	if api_is_connected:
		api_is_connected = false
		_pending.clear()
		api_disconnected.emit()
	if status != StreamPeerTCP.STATUS_CONNECTING and Time.get_ticks_msec() >= _reconnect_at_ms:
		_reconnect_at_ms = Time.get_ticks_msec() + 1000
		_tcp = StreamPeerTCP.new()
		_tcp.connect_to_host(api_host, api_port)


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
		if doc["event"] == "telemetry":
			telemetry.emit(doc["data"])
		elif doc["event"] == "input_raw":
			input_raw.emit(doc["data"])
		return
	var id: int = int(doc.get("id", -1))
	if not _pending.has(id):
		return
	var method: String = _pending[id]
	_pending.erase(id)
	var ok: bool = doc.get("ok", false)
	var payload: Dictionary = doc.get("result", doc.get("error", {}))
	if method == "subscribe" and ok:
		_subscribed = true
	command_result.emit(method, ok, payload)


## Sends a control API request; the answer arrives as command_result(method, ok, payload)
func request(method: String, params: Dictionary = {}) -> int:
	if _tcp.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		command_result.emit(method, false, {"code": "not_connected", "message": "simcore not connected"})
		return -1
	var id := _next_id
	_next_id += 1
	_pending[id] = method
	var line := JSON.stringify({"id": id, "method": method, "params": params}) + "\n"
	_tcp.put_data(line.to_utf8_buffer())
	return id
