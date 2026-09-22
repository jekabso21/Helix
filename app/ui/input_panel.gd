extends VBoxContainer
## Input tab: device choice, live channels and raw axes, mapping table, calibration wizard, save

const CHANNELS := ["roll", "pitch", "throttle", "yaw", "aux1", "aux2", "aux3", "aux4", "aux5", "aux6", "aux7", "aux8"]
const CHANNEL_LABELS := [
	"Roll (right +)", "Pitch (forward +)", "Throttle (up +)", "Yaw (right +)",
	"AUX1 arm (on +)", "AUX2 angle (on +)", "AUX3", "AUX4", "AUX5", "AUX6", "AUX7", "AUX8",
]
const WIZARD_STEPS := [
	["throttle", "Move the THROTTLE stick all the way UP, then back down"],
	["roll", "Move the ROLL stick fully RIGHT, then centre it"],
	["pitch", "Move the PITCH stick fully FORWARD (away from you), then centre it"],
	["yaw", "Move the YAW stick fully RIGHT, then centre it"],
	["aux1", "Flip the ARM switch to ARMED, then back"],
	["aux2", "Flip the switch you want for ANGLE mode, then back (or Skip)"],
]
const AXIS_THRESHOLD := 0.5
const MAX_SHOWN_AXES := 8

var _device_names: Array = []
var _mapping: Dictionary = {}
var _raw: Dictionary = {}
var _channel_bars: Array[ProgressBar] = []
var _axis_bars: Array[ProgressBar] = []
var _button_labels: Array[Label] = []
var _source_options: Array[OptionButton] = []
var _invert_checks: Array[CheckBox] = []
var _bind_buttons: Array[Button] = []
var _bind_channel: int = -1
var _bind_baseline: Array = []
var _bind_buttons_baseline: Array = []
var _wizard_step: int = -1
var _wizard_baseline: Array = []
var _wizard_buttons_baseline: Array = []
var _wizard_mapping: Dictionary = {}
var _updating_table: bool = false

@onready var _device: OptionButton = $DeviceRow/Device
@onready var _device_status: Label = $DeviceStatus
@onready var _channels_box: VBoxContainer = $Channels
@onready var _raw_box: VBoxContainer = $Raw
@onready var _table: GridContainer = $Mapping
@onready var _wizard_text: Label = $Wizard/Text
@onready var _wizard_next: Button = $Wizard/Buttons/Next
@onready var _wizard_skip: Button = $Wizard/Buttons/Skip
@onready var _wizard_cancel: Button = $Wizard/Buttons/Cancel
@onready var _calibrate: Button = $Actions/Calibrate
@onready var _save_name: LineEdit = $SaveRow/Name
@onready var _save_default: CheckBox = $SaveRow/Default
@onready var _message: Label = $Message


func _ready() -> void:
	_build_channel_bars()
	_build_raw_view()
	_build_table()
	$DeviceRow/Refresh.pressed.connect(_refresh)
	$DeviceRow/Select.pressed.connect(_select_device)
	_calibrate.pressed.connect(_start_wizard)
	_wizard_next.pressed.connect(_wizard_advance)
	_wizard_skip.pressed.connect(func() -> void: _wizard_advance(true))
	_wizard_cancel.pressed.connect(_wizard_end)
	$SaveRow/Save.pressed.connect(_save)
	$Wizard.visible = false
	SimLink.telemetry.connect(_on_telemetry)
	SimLink.input_raw.connect(_on_input_raw)
	SimLink.command_result.connect(_on_command_result)
	SimLink.api_connected.connect(_refresh)
	BackendClient.response.connect(_on_backend_response)
	if SimLink.api_is_connected:
		_refresh()


func _build_channel_bars() -> void:
	for label_text: String in CHANNEL_LABELS:
		var row := HBoxContainer.new()
		var label := Label.new()
		label.text = label_text
		label.custom_minimum_size = Vector2(90, 0)
		row.add_child(label)
		var bar := ProgressBar.new()
		bar.min_value = 1000
		bar.max_value = 2000
		bar.value = 1500
		bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(bar)
		_channels_box.add_child(row)
		_channel_bars.append(bar)


func _build_raw_view() -> void:
	for i in MAX_SHOWN_AXES:
		var row := HBoxContainer.new()
		var label := Label.new()
		label.text = "axis %d" % i
		label.custom_minimum_size = Vector2(90, 0)
		row.add_child(label)
		var bar := ProgressBar.new()
		bar.min_value = -1.0
		bar.max_value = 1.0
		bar.step = 0.01
		bar.value = 0.0
		bar.show_percentage = false
		bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(bar)
		_raw_box.add_child(row)
		_axis_bars.append(bar)
	var buttons := Label.new()
	buttons.text = "buttons: -"
	_raw_box.add_child(buttons)
	_button_labels.append(buttons)


func _build_table() -> void:
	_table.columns = 4
	for i in CHANNELS.size():
		var name := Label.new()
		name.text = CHANNEL_LABELS[i]
		_table.add_child(name)
		var bind := Button.new()
		bind.text = "Bind"
		bind.tooltip_text = "Press, then move this control in its positive direction"
		bind.pressed.connect(_start_bind.bind(i))
		_table.add_child(bind)
		_bind_buttons.append(bind)
		var source := OptionButton.new()
		source.item_selected.connect(func(_index: int) -> void: _table_changed())
		_table.add_child(source)
		_source_options.append(source)
		var invert := CheckBox.new()
		invert.text = "invert"
		invert.toggled.connect(func(_on: bool) -> void: _table_changed())
		_table.add_child(invert)
		_invert_checks.append(invert)
	_fill_source_options(8, 8)


func _start_bind(channel: int) -> void:
	if _raw.is_empty():
		_message.text = "no raw input yet: start a session and select a device first"
		return
	if _bind_channel == channel:
		_end_bind()
		return
	_end_bind()
	_bind_channel = channel
	_bind_baseline = _raw["axes"].duplicate()
	_bind_buttons_baseline = _raw["buttons"].duplicate()
	_bind_buttons[channel].text = "move it..."
	_message.text = "Binding %s: move that control now (%s), or press the button again to cancel" % [
		CHANNELS[channel], CHANNEL_LABELS[channel]
	]


func _end_bind() -> void:
	if _bind_channel >= 0:
		_bind_buttons[_bind_channel].text = "Bind"
	_bind_channel = -1


func _bind_watch(axes: Array, buttons: Array) -> void:
	var detected := _detect_movement(axes, buttons, _bind_baseline, _bind_buttons_baseline)
	if detected.is_empty():
		return
	var channel := _bind_channel
	_end_bind()
	_set_row(channel, detected)
	_message.text = "%s bound to %s%s" % [
		CHANNELS[channel],
		"axis %d" % int(detected["axis"]) if detected.has("axis") else "button %d" % int(detected["button"]),
		" (inverted)" if detected["inverted"] else ""
	]
	_table_changed()


## The axis that moved most past the threshold, or the first button that changed; {} if none yet
func _detect_movement(axes: Array, buttons: Array, axes_baseline: Array, buttons_baseline: Array) -> Dictionary:
	var best := -1
	var best_delta := 0.0
	for i in mini(axes.size(), axes_baseline.size()):
		var delta: float = float(axes[i]) - float(axes_baseline[i])
		if absf(delta) > absf(best_delta):
			best_delta = delta
			best = i
	if best >= 0 and absf(best_delta) > AXIS_THRESHOLD:
		return {"axis": best, "inverted": best_delta < 0.0, "deadband": 0.0}
	for i in mini(buttons.size(), buttons_baseline.size()):
		if int(buttons[i]) != int(buttons_baseline[i]):
			return {"button": i, "inverted": int(buttons_baseline[i]) != 0, "deadband": 0.0}
	return {}


func _set_row(channel: int, source: Dictionary) -> void:
	var was_updating := _updating_table
	_updating_table = true
	var option := _source_options[channel]
	var text: String = "axis %d" % int(source["axis"]) if source.has("axis") else "button %d" % int(source["button"])
	for item in option.item_count:
		if option.get_item_text(item) == text:
			option.selected = item
	_invert_checks[channel].button_pressed = source.get("inverted", false)
	_updating_table = was_updating


func _fill_source_options(axes: int, buttons: int) -> void:
	for option in _source_options:
		var previous := option.selected
		option.clear()
		option.add_item("none")
		for a in axes:
			option.add_item("axis %d" % a)
		for b in buttons:
			option.add_item("button %d" % b)
		option.selected = mini(previous, option.item_count - 1)


func _refresh() -> void:
	SimLink.request("get_input")


func _select_device() -> void:
	if _device.selected < 0 or _device.selected >= _device_names.size():
		return
	var name: String = _device_names[_device.selected]
	_message.text = "selecting %s" % name
	SimLink.request("select_input_device", {"name_contains": name})


func _on_command_result(method: String, ok: bool, result: Dictionary) -> void:
	if method == "get_input" and ok:
		_apply_input_info(result)
	elif method == "select_input_device":
		_message.text = "device selected" if ok else "select failed: %s" % result.get("message", "")
		_refresh()
	elif method == "set_input_mapping":
		_message.text = "mapping applied" if ok else "mapping rejected: %s" % result.get("message", "")
		_refresh()


func _apply_input_info(info: Dictionary) -> void:
	_device_names = info.get("devices", [])
	var current: String = info["device"]["name"]
	_device.clear()
	for name: String in _device_names:
		_device.add_item(name)
		if name == current:
			_device.selected = _device.item_count - 1
	var device: Dictionary = info["device"]
	_device_status.text = "%s   %s   %d axes, %d buttons   source %s" % [
		current if current != "" else "(no device)",
		"connected" if device["connected"] else "not connected",
		int(device["axis_count"]), int(device["button_count"]), info["source"]
	]
	_fill_source_options(maxi(8, int(device["axis_count"])), maxi(8, int(device["button_count"])))
	_mapping = info["mapping"]
	_fill_table_from_mapping()


func _fill_table_from_mapping() -> void:
	_updating_table = true
	var channels: Dictionary = _mapping.get("channels", {})
	for i in CHANNELS.size():
		var option := _source_options[i]
		var invert := _invert_checks[i]
		option.selected = 0
		invert.button_pressed = false
		if channels.has(CHANNELS[i]):
			_set_row(i, channels[CHANNELS[i]])
	_updating_table = false


func _table_changed() -> void:
	if _updating_table:
		return
	var channels := {}
	for i in CHANNELS.size():
		var text := _source_options[i].get_item_text(_source_options[i].selected)
		if text == "none":
			continue
		var parts := text.split(" ")
		var entry := {"inverted": _invert_checks[i].button_pressed, "deadband": 0.0}
		entry[parts[0]] = int(parts[1])
		channels[CHANNELS[i]] = entry
	_mapping = {
		"device_name_contains": _mapping.get("device_name_contains", ""),
		"arm_channel": "aux1",
		"channels": channels,
	}
	SimLink.request("set_input_mapping", {"mapping": _mapping})


func _on_telemetry(data: Dictionary) -> void:
	var channels: Array = data["input"]["channels_us"]
	for i in mini(_channel_bars.size(), channels.size()):
		_channel_bars[i].value = channels[i]


func _on_input_raw(data: Dictionary) -> void:
	_raw = data
	var axes: Array = data["axes"]
	for i in _axis_bars.size():
		_axis_bars[i].value = axes[i] if i < axes.size() else 0.0
	var buttons: Array = data["buttons"]
	var pressed: PackedStringArray = []
	for i in buttons.size():
		if int(buttons[i]) != 0:
			pressed.append(str(i))
	_button_labels[0].text = "buttons pressed: %s" % (", ".join(pressed) if pressed.size() > 0 else "-")
	if _bind_channel >= 0:
		_bind_watch(axes, buttons)
	elif _wizard_step >= 0:
		_wizard_watch(axes, buttons)


# calibration wizard

func _start_wizard() -> void:
	if _raw.is_empty():
		_message.text = "no raw input yet: select a device first"
		return
	_wizard_mapping = {}
	_wizard_step = 0
	$Wizard.visible = true
	_calibrate.disabled = true
	_wizard_begin_step()


func _wizard_begin_step() -> void:
	_wizard_baseline = _raw["axes"].duplicate()
	_wizard_buttons_baseline = _raw["buttons"].duplicate()
	_wizard_text.text = "Step %d/%d: %s" % [_wizard_step + 1, WIZARD_STEPS.size(), WIZARD_STEPS[_wizard_step][1]]
	_wizard_next.disabled = true


func _wizard_watch(axes: Array, buttons: Array) -> void:
	var channel: String = WIZARD_STEPS[_wizard_step][0]
	if _wizard_mapping.has(channel):
		return
	var detected := _detect_movement(axes, buttons, _wizard_baseline, _wizard_buttons_baseline)
	if detected.is_empty():
		return
	_wizard_mapping[channel] = detected
	_wizard_text.text += "\n-> %s%s" % [
		"axis %d" % int(detected["axis"]) if detected.has("axis") else "button %d" % int(detected["button"]),
		" (inverted)" if detected["inverted"] else ""
	]
	_wizard_next.disabled = false


func _wizard_advance(skip: bool = false) -> void:
	if skip:
		_wizard_mapping.erase(WIZARD_STEPS[_wizard_step][0])
	_wizard_step += 1
	if _wizard_step >= WIZARD_STEPS.size():
		_mapping = {
			"device_name_contains": _mapping.get("device_name_contains", ""),
			"arm_channel": "aux1",
			"channels": _wizard_mapping,
		}
		_wizard_end()
		_fill_table_from_mapping()
		SimLink.request("set_input_mapping", {"mapping": _mapping})
		_message.text = "calibration applied; save it below to keep it"
		return
	_wizard_begin_step()


func _wizard_end() -> void:
	_wizard_step = -1
	$Wizard.visible = false
	_calibrate.disabled = false


func _save() -> void:
	var name := _save_name.text.strip_edges()
	if name.is_empty():
		_message.text = "give the profile a name"
		return
	BackendClient.request("save_input_mapping", {"name": name, "mapping": _mapping, "make_default": _save_default.button_pressed})


func _on_backend_response(method: String, ok: bool, result: Dictionary) -> void:
	if method == "save_input_mapping":
		_message.text = "saved %s%s" % [result.get("path", ""), " as default" if result.get("default", false) else ""] if ok else "save failed: %s" % result.get("message", "")
