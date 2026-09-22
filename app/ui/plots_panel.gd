extends Control
## Live plots of the last 10 s: motor outputs, body rates, battery voltage and current

const WINDOW_S := 10.0
const MOTOR_COLORS := [Color(0.95, 0.45, 0.2), Color(0.3, 0.8, 0.95), Color(0.6, 0.9, 0.3), Color(0.9, 0.5, 0.9)]
const RATE_COLORS := [Color(0.95, 0.3, 0.3), Color(0.3, 0.9, 0.3), Color(0.4, 0.5, 1.0)]
const BATTERY_COLORS := [Color(1.0, 0.85, 0.2), Color(0.4, 0.8, 1.0)]
const MARGIN := 6.0

var _motors: Array[PlotSeries] = []
var _rates: Array[PlotSeries] = []
var _battery: Array[PlotSeries] = []
var _paused: bool = false
var _font: Font = null


func _ready() -> void:
	for i in 4:
		_motors.append(PlotSeries.new(WINDOW_S))
	for i in 3:
		_rates.append(PlotSeries.new(WINDOW_S))
	for i in 2:
		_battery.append(PlotSeries.new(WINDOW_S))
	_font = get_theme_default_font()
	SimLink.telemetry.connect(_on_telemetry)
	SimLink.api_disconnected.connect(_clear)


func _clear() -> void:
	for s in _motors + _rates + _battery:
		s.clear()
	queue_redraw()


func _on_telemetry(data: Dictionary) -> void:
	var sim: Dictionary = data["sim"]
	_paused = sim.get("paused", false)
	if _paused:
		return
	var t: float = float(sim.get("sim_time_ns", 0)) * 1e-9
	var motors: Array = data["motors"]
	for i in mini(motors.size(), _motors.size()):
		_motors[i].push(t, float(motors[i]["command"]))
	var rates: Array = data["flight"]["rates_frd_radps"]
	for i in 3:
		_rates[i].push(t, rad_to_deg(float(rates[i])))
	var battery: Variant = data.get("battery")
	if battery is Dictionary:
		_battery[0].push(t, float(battery["voltage_v"]))
		_battery[1].push(t, float(battery["current_a"]))
	queue_redraw()


func _draw() -> void:
	var width := (size.x - 4.0 * MARGIN) / 3.0
	var height := size.y - 2.0 * MARGIN
	var motor_label := "motors %%  " + "  ".join(_motors.map(func(s: PlotSeries) -> String: return "%.0f" % (100.0 * s.latest())))
	_draw_strip(Rect2(MARGIN, MARGIN, width, height), _motors, MOTOR_COLORS, 0.0, 1.0, motor_label)
	var span := 50.0
	for s in _rates:
		if s.size() > 0:
			span = maxf(span, maxf(absf(s.min_value()), absf(s.max_value())))
	var rate_label := "rates deg/s  p %+.0f  q %+.0f  r %+.0f" % [_rates[0].latest(), _rates[1].latest(), _rates[2].latest()]
	_draw_strip(Rect2(2.0 * MARGIN + width, MARGIN, width, height), _rates, RATE_COLORS, -span, span, rate_label)
	var battery_rect := Rect2(3.0 * MARGIN + 2.0 * width, MARGIN, width, height)
	var v := _battery[0]
	var v_lo := floorf(v.min_value() - 0.5) if v.size() > 0 else 20.0
	var v_hi := ceilf(v.max_value() + 0.5) if v.size() > 0 else 26.0
	var i_hi := maxf(10.0, ceilf(_battery[1].max_value() / 10.0) * 10.0) if _battery[1].size() > 0 else 10.0
	var battery_label := "battery  %.2f V  %.1f A" % [v.latest(), _battery[1].latest()]
	_draw_strip(battery_rect, [_battery[0]], [BATTERY_COLORS[0]], v_lo, v_hi, battery_label)
	_draw_series(battery_rect, _battery[1], BATTERY_COLORS[1], 0.0, i_hi)
	if _paused:
		draw_string(_font, Vector2(size.x - 70.0, size.y - MARGIN - 2.0), "PAUSED", HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(1, 0.8, 0.3))


func _draw_strip(rect: Rect2, series: Array, colors: Array, lo: float, hi: float, label: String) -> void:
	draw_rect(rect, Color(0.08, 0.08, 0.1))
	draw_rect(rect, Color(0.3, 0.3, 0.35), false)
	if lo < 0.0 and hi > 0.0:
		var y0 := rect.position.y + rect.size.y * (hi / (hi - lo))
		draw_line(Vector2(rect.position.x, y0), Vector2(rect.end.x, y0), Color(0.35, 0.35, 0.4))
	for i in series.size():
		_draw_series(rect, series[i], colors[i], lo, hi)
	draw_string(_font, rect.position + Vector2(4.0, 12.0), label, HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.85, 0.85, 0.85))


func _draw_series(rect: Rect2, s: PlotSeries, color: Color, lo: float, hi: float) -> void:
	if s.size() < 2 or hi <= lo:
		return
	var t_end := s.times[s.size() - 1]
	var points := PackedVector2Array()
	for i in s.size():
		var x := rect.end.x - (t_end - s.times[i]) / WINDOW_S * rect.size.x
		var y := rect.end.y - clampf((s.values[i] - lo) / (hi - lo), 0.0, 1.0) * rect.size.y
		points.append(Vector2(x, y))
	draw_polyline(points, color, 1.0, true)
