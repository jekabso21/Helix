class_name PlotSeries
extends RefCounted
## Rolling window of (t, value) samples for the live plots; oldest samples fall off the window

var window_s: float
var times: PackedFloat64Array = PackedFloat64Array()
var values: PackedFloat32Array = PackedFloat32Array()


func _init(window_seconds: float = 10.0) -> void:
	window_s = window_seconds


func push(t_s: float, value: float) -> void:
	if not times.is_empty() and t_s < times[times.size() - 1]:
		clear()  # sim time went backwards: a reset or a new session
	times.append(t_s)
	values.append(value)
	var cutoff := t_s - window_s
	var drop := 0
	while drop < times.size() and times[drop] < cutoff:
		drop += 1
	if drop > 0:
		times = times.slice(drop)
		values = values.slice(drop)


func clear() -> void:
	times = PackedFloat64Array()
	values = PackedFloat32Array()


func size() -> int:
	return times.size()


func latest() -> float:
	return values[values.size() - 1] if not values.is_empty() else 0.0


func min_value() -> float:
	var m := INF
	for v in values:
		m = minf(m, v)
	return m


func max_value() -> float:
	var m := -INF
	for v in values:
		m = maxf(m, v)
	return m
