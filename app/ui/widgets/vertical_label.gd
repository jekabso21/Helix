class_name VerticalLabel
extends Control
## Text set on its side, top to bottom, for a collapsed dock

var text := "":
	set(v):
		text = v
		_resize()
		queue_redraw()


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_resize()


func _resize() -> void:
	var font := get_theme_font("heading", "Palette")
	if font == null:
		return
	var width := font.get_string_size(text.to_upper(), HORIZONTAL_ALIGNMENT_LEFT, -1, 13).x + 2.0 * text.length()
	custom_minimum_size = Vector2(16, width)


func _draw() -> void:
	var font := get_theme_font("heading", "Palette")
	var colour := get_theme_color("font_color", "IconButton")
	draw_set_transform(Vector2(size.x * 0.5 - 5.0, 0.0), PI * 0.5)
	var x := 0.0
	for ch in text.to_upper():
		draw_string(font, Vector2(x, 0.0), ch, HORIZONTAL_ALIGNMENT_LEFT, -1, 13, colour)
		x += font.get_string_size(ch, HORIZONTAL_ALIGNMENT_LEFT, -1, 13).x + 2.0
	draw_set_transform(Vector2.ZERO)
