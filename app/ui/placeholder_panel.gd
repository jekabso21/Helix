class_name PlaceholderPanel
extends PanelContainer

@export var title: String = "Panel"

@onready var _label: Label = $Label


func _ready() -> void:
	_label.text = title
