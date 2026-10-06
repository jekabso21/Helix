class_name PlaceholderPanel
extends MarginContainer

@export var body: String = ""


func _ready() -> void:
	$Box/Title.text = str(name).to_upper()
	$Box/Body.text = body
