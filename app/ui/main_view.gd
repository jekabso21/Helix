extends SubViewportContainer
## Main 3D view: the world scene inside a SubViewport plus a mode caption

var _camera_panel: Node = null

@onready var _caption: Label = $Caption
@onready var _feed: TextureRect = $Feed
@onready var _backdrop: ColorRect = $FeedBackdrop
@onready var _rig: Node3D = $SubViewport/World/CameraRig


func _ready() -> void:
	_rig.mode_changed.connect(_on_mode_changed)
	_on_mode_changed(_rig.mode)


## In FPV mode the main view shows the camera feed itself, optics, OSD and all, so it is the
## picture the video outputs carry rather than a second camera that only looks similar
func set_camera_source(panel: Node) -> void:
	_camera_panel = panel
	_on_mode_changed(_rig.mode)


func showing_feed() -> bool:
	return _camera_panel != null and _rig.mode == _rig.Mode.FPV


func _on_mode_changed(mode: int) -> void:
	var names: Array = _rig.MODE_NAMES
	_caption.text = "%s view   [1-6 modes, right drag, wheel]" % names[mode]
	_feed.visible = showing_feed()
	# the feed keeps its aspect, so the rest of the view is black rather than the world behind it
	_backdrop.visible = _feed.visible
	_update_feed_texture()


func _process(_delta: float) -> void:
	if _feed.visible:
		_update_feed_texture()


func _update_feed_texture() -> void:
	if not _feed.visible:
		_feed.texture = null
		return
	var feed: Variant = _camera_panel.selected_feed()
	_feed.texture = feed.post_viewport.get_texture() if feed != null else null
