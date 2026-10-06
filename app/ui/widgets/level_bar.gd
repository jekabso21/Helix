class_name LevelBar
extends Control
## A thin bar: from the left edge for a level, or from the centre for a signed value

enum Origin { LEFT, CENTRE }

@export var origin: Origin = Origin.LEFT
@export var min_value: float = 0.0
@export var max_value: float = 1.0
@export var framed: bool = false
@export var muted: bool = false

var value: float = 0.0:
	set(v):
		if v != value:
			value = v
			queue_redraw()


func _draw() -> void:
	var rect := Rect2(Vector2.ZERO, size)
	draw_rect(rect, get_theme_color("track", "Palette"))
	if framed:
		draw_rect(rect, get_theme_color("divider", "Palette"), false, 1.0)
	var span := max_value - min_value
	if span <= 0.0:
		return
	var f := clampf((value - min_value) / span, 0.0, 1.0)
	var inset := 1.0 if framed else 0.0
	var fill := get_theme_color("muted" if muted else "accent", "Palette")
	if origin == Origin.CENTRE:
		var lo := minf(f, 0.5)
		var hi := maxf(f, 0.5)
		draw_rect(Rect2(lo * size.x, inset, (hi - lo) * size.x, size.y - 2.0 * inset), fill)
		if framed:
			draw_line(Vector2(size.x * 0.5, 0.0), Vector2(size.x * 0.5, size.y), get_theme_color("centre", "Palette"), 1.0)
	else:
		draw_rect(Rect2(0.0, inset, f * size.x, size.y - 2.0 * inset), fill)
