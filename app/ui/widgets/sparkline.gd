class_name Sparkline
extends Control
## The last window of one PlotSeries as a line, with an optional dashed zero line

@export var symmetric := true
@export var min_span := 40.0
@export var show_zero := true

var series: PlotSeries = null


func _draw() -> void:
	var rect := Rect2(Vector2.ZERO, size)
	draw_rect(rect, get_theme_color("plot_bg", "Palette"))
	draw_rect(rect, get_theme_color("divider", "Palette"), false, 1.0)
	var lo := -min_span
	var hi := min_span
	if series != null and series.size() > 0:
		if symmetric:
			hi = maxf(min_span, maxf(absf(series.min_value()), absf(series.max_value())))
			lo = -hi
		else:
			var spread := maxf(series.max_value() - series.min_value(), min_span)
			var mid := (series.max_value() + series.min_value()) * 0.5
			lo = mid - spread * 0.6
			hi = mid + spread * 0.6
	if show_zero and lo < 0.0 and hi > 0.0:
		var y0 := size.y * hi / (hi - lo)
		draw_dashed_line(Vector2(0.0, y0), Vector2(size.x, y0), get_theme_color("plot_zero", "Palette"), 1.0, 3.0)
	if series == null or series.size() < 2:
		return
	var t_end := series.times[series.size() - 1]
	var points := PackedVector2Array()
	for i in series.size():
		var x := size.x - (t_end - series.times[i]) / series.window_s * size.x
		var y := size.y - clampf((series.values[i] - lo) / (hi - lo), 0.0, 1.0) * (size.y - 2.0) - 1.0
		points.append(Vector2(x, y))
	draw_polyline(points, get_theme_color("accent_line", "Palette"), 1.5, true)
