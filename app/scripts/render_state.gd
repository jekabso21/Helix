class_name RenderState
extends RefCounted
## Decoder for the RenderState message (16-byte header + 192-byte payload, little-endian)

const MAGIC := 0x53565046
const PROTOCOL_VERSION := 1
const MSG_TYPE := 1
const HEADER_SIZE := 16
const PAYLOAD_SIZE := 192
const MESSAGE_SIZE := HEADER_SIZE + PAYLOAD_SIZE
const MOTORS := 8

var seq: int = 0
var sim_time_ns: int = 0
var position_ned := Vector3.ZERO
var q_w: float = 1.0
var q_x: float = 0.0
var q_y: float = 0.0
var q_z: float = 0.0
var velocity_ned := Vector3.ZERO
var angular_rate_frd := Vector3.ZERO
var armed: bool = false
var crashed: bool = false
var motor_count: int = 0
var motor_rpm: PackedFloat32Array = PackedFloat32Array()
var sun_dir_ned := Vector3.ZERO
var sun_intensity: float = 0.0
var fog_density: float = 0.0
var precip_type: int = 0
var precip_intensity: float = 0.0
var wind_ned := Vector3.ZERO
var video_fault_flags: int = 0


static func decode(data: PackedByteArray) -> RenderState:
	if data.size() != MESSAGE_SIZE:
		return null
	if data.decode_u32(0) != MAGIC or data.decode_u16(4) != PROTOCOL_VERSION:
		return null
	if data.decode_u16(6) != MSG_TYPE or data.decode_u32(8) != PAYLOAD_SIZE:
		return null
	var s := RenderState.new()
	s.seq = data.decode_u32(12)
	var o := HEADER_SIZE
	s.sim_time_ns = data.decode_s64(o)
	s.position_ned = _vec3_f64(data, o + 8)
	s.q_w = data.decode_double(o + 32)
	s.q_x = data.decode_double(o + 40)
	s.q_y = data.decode_double(o + 48)
	s.q_z = data.decode_double(o + 56)
	s.velocity_ned = _vec3_f64(data, o + 64)
	s.angular_rate_frd = _vec3_f64(data, o + 88)
	s.armed = data.decode_u8(o + 112) != 0
	s.crashed = data.decode_u8(o + 113) != 0
	s.motor_count = data.decode_u8(o + 114)
	s.motor_rpm.resize(MOTORS)
	for i in MOTORS:
		s.motor_rpm[i] = data.decode_float(o + 116 + 4 * i)
	s.sun_dir_ned = _vec3_f32(data, o + 148)
	s.sun_intensity = data.decode_float(o + 160)
	s.fog_density = data.decode_float(o + 164)
	s.precip_type = data.decode_u8(o + 168)
	s.precip_intensity = data.decode_float(o + 172)
	s.wind_ned = _vec3_f32(data, o + 176)
	s.video_fault_flags = data.decode_u32(o + 188)
	return s


func godot_position() -> Vector3:
	return Frames.godot_from_ned(position_ned)


func godot_rotation() -> Quaternion:
	return Frames.godot_quat_from_ned_frd(q_w, q_x, q_y, q_z)


static func _vec3_f64(data: PackedByteArray, offset: int) -> Vector3:
	return Vector3(
		data.decode_double(offset), data.decode_double(offset + 8), data.decode_double(offset + 16)
	)


static func _vec3_f32(data: PackedByteArray, offset: int) -> Vector3:
	return Vector3(
		data.decode_float(offset), data.decode_float(offset + 4), data.decode_float(offset + 8)
	)
