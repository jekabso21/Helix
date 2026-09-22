extends VBoxContainer
## Channel monitor: the RC channels simcore sends to Betaflight

const NAMES := ["Roll", "Pitch", "Throttle", "Yaw", "AUX1 arm", "AUX2", "AUX3", "AUX4"]

@onready var _source: Label = $Source
var _bars: Array[ProgressBar] = []


func _ready() -> void:
	for name: String in NAMES:
		var row := HBoxContainer.new()
		var label := Label.new()
		label.text = name
		label.custom_minimum_size = Vector2(80, 0)
		row.add_child(label)
		var bar := ProgressBar.new()
		bar.min_value = 1000
		bar.max_value = 2000
		bar.value = 1500
		bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(bar)
		add_child(row)
		_bars.append(bar)
	SimLink.telemetry.connect(_on_telemetry)


func _on_telemetry(data: Dictionary) -> void:
	var input: Dictionary = data["input"]
	var channels: Array = input["channels_us"]
	_source.text = "Source: %s   arm switch %s" % [input["source"], "ON" if input["armed_switch"] else "off"]
	for i in mini(_bars.size(), channels.size()):
		_bars[i].value = channels[i]
