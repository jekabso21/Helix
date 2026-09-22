extends SubViewportContainer
## Main 3D view: the world scene inside a SubViewport plus a mode caption

@onready var _caption: Label = $Caption
@onready var _rig: Node3D = $SubViewport/World/CameraRig


func _ready() -> void:
	_rig.mode_changed.connect(_on_mode_changed)
	_on_mode_changed(_rig.mode)


func _on_mode_changed(mode: int) -> void:
	var names: Array = _rig.MODE_NAMES
	_caption.text = "%s view   [1-5 modes, right drag, wheel]" % names[mode]
