class_name FrameCorners
extends Control
## Registration marks on the parent's corners, as on a blueprint

const HALF_ARM := 5.5


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	set_anchors_preset(Control.PRESET_FULL_RECT)


func _draw() -> void:
	var colour := get_theme_color("corner", "Palette")
	for corner: Vector2 in [Vector2.ZERO, Vector2(size.x, 0.0), Vector2(0.0, size.y), size]:
		# the cross sits one pixel outside the frame line
		var c := corner + Vector2(-1.0 if corner.x == 0.0 else 1.0, -1.0 if corner.y == 0.0 else 1.0)
		draw_line(c - Vector2(HALF_ARM, 0.0), c + Vector2(HALF_ARM, 0.0), colour, 1.0)
		draw_line(c - Vector2(0.0, HALF_ARM), c + Vector2(0.0, HALF_ARM), colour, 1.0)
