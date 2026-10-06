class_name DockIcon
extends Control
## The panel glyph on a dock's collapse handle: a frame, its divider and a chevron (24 px grid)

@export var divider_left := true
@export var points_left := true


func _init() -> void:
	custom_minimum_size = Vector2(16, 16)
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func _draw() -> void:
	var parent := get_parent() as Button
	var hovered := parent != null and parent.is_hovered()
	var colour := get_theme_color("font_hover_color" if hovered else "font_color", "IconButton")
	var s := size.x / 24.0
	draw_rect(Rect2(Vector2(3, 4) * s, Vector2(18, 16) * s), colour, false, 1.5)
	var x := 9.0 if divider_left else 15.0
	draw_line(Vector2(x, 4) * s, Vector2(x, 20) * s, colour, 1.5)
	var x0: float
	if divider_left:
		x0 = 15.0 if points_left else 13.0
	else:
		x0 = 11.0 if points_left else 9.0
	var tip := x0 - 2.0 if points_left else x0 + 2.0
	draw_polyline(PackedVector2Array([Vector2(x0, 10) * s, Vector2(tip, 12) * s, Vector2(x0, 14) * s]), colour, 1.5, true)
