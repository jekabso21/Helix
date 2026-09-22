class_name RawVideoOut
extends RefCounted
## Serves RGB8 frames to one TCP client (gst-launch tcpclientsrc); frames are dropped while nobody reads

const MAX_QUEUED := 2

var port: int = 5700
var frames_written: int = 0
var frames_dropped: int = 0
var error: String = ""

var _server := TCPServer.new()
var _thread := Thread.new()
var _mutex := Mutex.new()
var _semaphore := Semaphore.new()
var _queue: Array[PackedByteArray] = []
var _stop: bool = false


func start(listen_port: int) -> bool:
	port = listen_port
	var result := _server.listen(port, "127.0.0.1")
	if result != OK:
		error = "cannot listen on %d: %s" % [port, error_string(result)]
		return false
	_stop = false
	_thread.start(_run)
	return true


func stop() -> void:
	_mutex.lock()
	_stop = true
	_mutex.unlock()
	_semaphore.post()
	if _thread.is_started():
		_thread.wait_to_finish()
	_server.stop()


func push(frame: PackedByteArray) -> void:
	_mutex.lock()
	if _queue.size() >= MAX_QUEUED:
		_queue.pop_front()
		frames_dropped += 1
	_queue.push_back(frame)
	_mutex.unlock()
	_semaphore.post()


func _run() -> void:
	var client: StreamPeerTCP = null
	while true:
		_semaphore.wait()
		_mutex.lock()
		var stop := _stop
		var frame: PackedByteArray = _queue.pop_front() if not _queue.is_empty() else PackedByteArray()
		_mutex.unlock()
		if stop:
			break
		if client == null and _server.is_connection_available():
			client = _server.take_connection()
			client.set_no_delay(true)
		if client == null:
			continue
		client.poll()
		if client.get_status() != StreamPeerTCP.STATUS_CONNECTED:
			client = null
			continue
		if frame.is_empty():
			continue
		if client.put_data(frame) != OK:
			client.disconnect_from_host()
			client = null
			continue
		frames_written += 1
	if client != null:
		client.disconnect_from_host()
