class_name StatusDot
extends Control
## An 8 px process light with a soft ring while it runs

enum State { OFF, ON, WARN, FAIL }

var state: State = State.OFF:
	set(v):
		if v != state:
			state = v
			queue_redraw()


func _init() -> void:
	custom_minimum_size = Vector2(12, 12)
	size_flags_vertical = Control.SIZE_SHRINK_CENTER


func _draw() -> void:
	var centre := size * 0.5
	var colour: Color
	match state:
		State.ON:
			colour = get_theme_color("accent_line", "Palette")
			draw_circle(centre, 6.0, get_theme_color("accent_soft", "Palette"))
		State.WARN:
			colour = get_theme_color("warning", "Palette")
		State.FAIL:
			colour = get_theme_color("error", "Palette")
		_:
			colour = get_theme_color("off", "Palette")
	draw_circle(centre, 4.0, colour)
