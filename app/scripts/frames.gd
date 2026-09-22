class_name Frames
extends RefCounted
## NED/FRD to Godot conversions; the only place these mappings appear in the app


static func godot_from_ned(v: Vector3) -> Vector3:
	return Vector3(v.y, -v.z, -v.x)


static func ned_from_godot(v: Vector3) -> Vector3:
	return Vector3(-v.z, v.x, -v.y)


static func godot_body_from_frd(v: Vector3) -> Vector3:
	return Vector3(v.y, -v.z, -v.x)


## q given as (w, x, y, z) for q_ned_from_frd; returns the Godot node rotation
static func godot_quat_from_ned_frd(w: float, x: float, y: float, z: float) -> Quaternion:
	var v := godot_from_ned(Vector3(x, y, z))
	return Quaternion(v.x, v.y, v.z, w)


static func heading_rad(q: Quaternion) -> float:
	# yaw of q_ned_from_frd, positive from North to East, from the Godot-frame quaternion
	var nose := q * Vector3(0.0, 0.0, -1.0)
	var ned := ned_from_godot(nose)
	return atan2(ned.y, ned.x)
