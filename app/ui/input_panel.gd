extends VBoxContainer
## Input tab: device choice, live channels and raw axes, mapping table, calibration wizard, profiles

const CHANNELS := ["roll", "pitch", "throttle", "yaw", "aux1", "aux2", "aux3", "aux4", "aux5", "aux6", "aux7", "aux8"]
const CHANNEL_NAMES := ["Roll", "Pitch", "Throttle", "Yaw", "AUX1", "AUX2", "AUX3", "AUX4", "AUX5", "AUX6", "AUX7", "AUX8"]
const CHANNEL_HINTS := ["right +", "fwd +", "up +", "right +", "arm", "angle", "", "", "", "", "", ""]
const CHANNEL_LABELS := [
	"Roll (right +)", "Pitch (forward +)", "Throttle (up +)", "Yaw (right +)",
	"AUX1 arm (on +)", "AUX2 angle (on +)", "AUX3", "AUX4", "AUX5", "AUX6", "AUX7", "AUX8",
]
const MAIN_CHANNELS := 6
const WIZARD_STEPS := [
	["throttle", "Move the THROTTLE stick all the way UP, then back down"],
	["roll", "Move the ROLL stick fully RIGHT, then centre it"],
	["pitch", "Move the PITCH stick fully FORWARD (away from you), then centre it"],
	["yaw", "Move the YAW stick fully RIGHT, then centre it"],
	["aux1", "Flip the ARM switch to ARMED, then back"],
	["aux2", "Flip the switch you want for ANGLE mode, then back (or Skip)"],
]
const AXIS_THRESHOLD := 0.5
const SAVE_DELAY_S := 0.5
const IDLE_POLL_S := 2.0
const MAX_SHOWN_AXES := 8

var _device_names: Array = []
var _profiles: Array = []
var _mapping: Dictionary = {}
var _raw: Dictionary = {}
var _channel_bars: Array[LevelBar] = []
var _channel_values: Array[Label] = []
var _aux_values: Array[Label] = []
var _axis_bars: Array[LevelBar] = []
var _axis_values: Array[Label] = []
var _source_options: Array[OptionButton] = []
var _invert_checks: Array[CheckBox] = []
var _bind_buttons: Array[Button] = []
var _aux_rows: Array[Control] = []
var _bind_channel: int = -1
var _bind_baseline: Array = []
var _bind_buttons_baseline: Array = []
var _wizard_step: int = -1
var _wizard_baseline: Array = []
var _wizard_buttons_baseline: Array = []
var _wizard_mapping: Dictionary = {}
var _updating_table: bool = false
var _updating_devices: bool = false
var _aux_open := false
var _input_times: Array[float] = []
## The profile edits are saved into; "" until a controller is matched or a new one gets a profile
var _active_profile := ""
var _default_target := ""
var _profiles_loaded := false
var _detect_pending := true
var _last_info: Dictionary = {}
var _save_at := -1.0
var _next_idle_poll := 0.0
var _idle_key := ""

@onready var _device: OptionButton = $Device/Row/Device
@onready var _device_status: Label = $Device/Status
@onready var _profile: OptionButton = $Device/ProfileRow/Profile
@onready var _rate: Label = $Channels/Head/Rate
@onready var _channels_box: VBoxContainer = $Channels/Rows
@onready var _aux_toggle: Button = $Channels/AuxToggle
@onready var _aux_grid: GridContainer = $Channels/Aux
@onready var _buttons_label: Label = $Raw/Head/Buttons
@onready var _axes_grid: GridContainer = $Raw/Axes
@onready var _table: VBoxContainer = $Mapping/Table
@onready var _wizard: Control = $Mapping/Wizard
@onready var _wizard_text: Label = $Mapping/Wizard/Box/Text
@onready var _wizard_next: Button = $Mapping/Wizard/Box/Buttons/Next
@onready var _wizard_skip: Button = $Mapping/Wizard/Box/Buttons/Skip
@onready var _wizard_cancel: Button = $Mapping/Wizard/Box/Buttons/Cancel
@onready var _calibrate: Button = $Mapping/Actions/Calibrate
@onready var _save_row: Control = $Mapping/SaveRow
@onready var _save_name: LineEdit = $Mapping/SaveRow/Name
@onready var _save_default: CheckBox = $Mapping/SaveRow/Default
@onready var _set_default: Button = $Mapping/Actions/SetDefault
@onready var _message: Label = $Message


func _ready() -> void:
	_build_channel_bars()
	_build_raw_view()
	_build_table()
	$Device/Row/Refresh.pressed.connect(_rescan)
	_device.item_selected.connect(func(_index: int) -> void: _select_device())
	_profile.item_selected.connect(_on_profile_selected)
	_aux_toggle.pressed.connect(_toggle_aux)
	_calibrate.pressed.connect(_start_wizard)
	_wizard_next.pressed.connect(_wizard_advance)
	_wizard_skip.pressed.connect(func() -> void: _wizard_advance(true))
	_wizard_cancel.pressed.connect(_wizard_end)
	$Mapping/Actions/SaveAs.pressed.connect(func() -> void:
		_save_row.visible = not _save_row.visible
		if _save_row.visible:
			_save_name.grab_focus())
	$Mapping/SaveRow/Save.pressed.connect(_save)
	_save_name.text_submitted.connect(func(_text: String) -> void: _save())
	_set_default.pressed.connect(_set_selected_default)
	_wizard.visible = false
	_render_aux_toggle()
	SimLink.telemetry.connect(_on_telemetry)
	SimLink.input_raw.connect(_on_input_raw)
	SimLink.command_result.connect(_on_command_result)
	SimLink.api_connected.connect(func() -> void:
		_detect_pending = true
		_idle_key = ""
		_refresh())
	SimLink.api_disconnected.connect(func() -> void:
		_last_info = {}
		_idle_key = ""
		_next_idle_poll = 0.0)
	BackendClient.response.connect(_on_backend_response)
	BackendClient.connected.connect(_list_profiles)
	if BackendClient.is_connected_to_backend:
		_list_profiles()
	if SimLink.api_is_connected:
		_refresh()


func _build_channel_bars() -> void:
	for i in MAIN_CHANNELS:
		var row := HBoxContainer.new()
		row.theme_type_variation = "Gap10"
		row.custom_minimum_size.y = 24
		var names := HBoxContainer.new()
		names.theme_type_variation = "Gap4"
		names.custom_minimum_size.x = 96
		var name := Label.new()
		name.theme_type_variation = "Strong"
		name.text = CHANNEL_NAMES[i]
		names.add_child(name)
		var hint := Label.new()
		hint.theme_type_variation = "Hint"
		hint.text = CHANNEL_HINTS[i]
		names.add_child(hint)
		row.add_child(names)
		var bar := LevelBar.new()
		bar.origin = LevelBar.Origin.CENTRE
		bar.framed = true
		bar.min_value = 1000
		bar.max_value = 2000
		bar.value = 1500
		bar.custom_minimum_size.y = 10
		bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		bar.size_flags_vertical = Control.SIZE_SHRINK_CENTER
		row.add_child(bar)
		var value := Label.new()
		value.theme_type_variation = "Mono"
		value.text = "1500"
		value.custom_minimum_size.x = 44
		value.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		row.add_child(value)
		_channels_box.add_child(row)
		_channel_bars.append(bar)
		_channel_values.append(value)
	for i in range(MAIN_CHANNELS, CHANNELS.size()):
		var cell := HBoxContainer.new()
		cell.theme_type_variation = "Gap6"
		cell.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		var name := Label.new()
		name.theme_type_variation = "Muted"
		name.text = CHANNEL_NAMES[i]
		name.custom_minimum_size.x = 36
		cell.add_child(name)
		var value := Label.new()
		value.theme_type_variation = "Mono"
		value.text = "1500"
		cell.add_child(value)
		_aux_grid.add_child(cell)
		_aux_values.append(value)


func _build_raw_view() -> void:
	for i in MAX_SHOWN_AXES:
		var cell := VBoxContainer.new()
		cell.theme_type_variation = "Tight"
		cell.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		var top := HBoxContainer.new()
		var name := Label.new()
		name.theme_type_variation = "Hint"
		name.text = "axis %d" % i
		name.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		top.add_child(name)
		var value := Label.new()
		value.theme_type_variation = "MonoMuted"
		value.text = "0.00"
		top.add_child(value)
		cell.add_child(top)
		var bar := LevelBar.new()
		bar.origin = LevelBar.Origin.CENTRE
		bar.muted = true
		bar.min_value = -1.0
		bar.max_value = 1.0
		bar.custom_minimum_size.y = 4
		cell.add_child(bar)
		_axes_grid.add_child(cell)
		_axis_bars.append(bar)
		_axis_values.append(value)


func _build_table() -> void:
	for i in CHANNELS.size():
		var row := PanelContainer.new()
		row.theme_type_variation = "TableRow"
		row.custom_minimum_size.y = 34
		var line := HBoxContainer.new()
		line.theme_type_variation = "Gap8"
		row.add_child(line)
		var name := Label.new()
		name.theme_type_variation = "Strong"
		name.text = CHANNEL_NAMES[i] + ("" if CHANNEL_HINTS[i] == "" or i < 4 else " " + CHANNEL_HINTS[i])
		name.tooltip_text = CHANNEL_LABELS[i]
		name.mouse_filter = Control.MOUSE_FILTER_PASS
		name.clip_text = true
		name.custom_minimum_size.x = 88
		line.add_child(name)
		var source := OptionButton.new()
		source.theme_type_variation = "SmallOption"
		source.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		source.size_flags_vertical = Control.SIZE_SHRINK_CENTER
		source.clip_text = true
		source.fit_to_longest_item = false
		source.custom_minimum_size.x = 60
		source.item_selected.connect(func(_index: int) -> void: _table_changed())
		line.add_child(source)
		_source_options.append(source)
		var invert := CheckBox.new()
		invert.text = "inv"
		invert.tooltip_text = "Invert this channel"
		invert.toggled.connect(func(_on: bool) -> void: _table_changed())
		line.add_child(invert)
		_invert_checks.append(invert)
		var bind := Button.new()
		bind.theme_type_variation = "SmallButton"
		bind.text = "Bind"
		bind.custom_minimum_size.x = 56
		bind.size_flags_vertical = Control.SIZE_SHRINK_CENTER
		bind.tooltip_text = "Press, then move this control in its positive direction"
		bind.pressed.connect(_start_bind.bind(i))
		line.add_child(bind)
		_bind_buttons.append(bind)
		_table.add_child(row)
		if i >= MAIN_CHANNELS:
			_aux_rows.append(row)
	_fill_source_options(8, 8)


func _toggle_aux() -> void:
	_aux_open = not _aux_open
	_render_aux_toggle()


func _render_aux_toggle() -> void:
	_aux_grid.visible = _aux_open
	for row in _aux_rows:
		row.visible = _aux_open
	if _aux_open:
		_aux_toggle.text = "▾ Hide AUX3–8"
		return
	var values := {}
	for label in _aux_values:
		values[label.text] = true
	var summary := "all %s" % _aux_values[0].text if values.size() == 1 else "mixed"
	_aux_toggle.text = "▸ AUX3–8 · %s" % summary


func _start_bind(channel: int) -> void:
	if _raw.is_empty():
		_message.text = "Bind needs live stick input: start a gamepad session such as dev_gamepad first."
		return
	if _bind_channel == channel:
		_end_bind()
		return
	_end_bind()
	_bind_channel = channel
	_bind_baseline = _raw["axes"].duplicate()
	_bind_buttons_baseline = _raw["buttons"].duplicate()
	_bind_buttons[channel].text = "Move…"
	_bind_buttons[channel].theme_type_variation = "ActiveButton"
	_message.text = "Binding %s: move that control now (%s), or press the button again to cancel" % [
		CHANNELS[channel], CHANNEL_LABELS[channel]
	]


func _end_bind() -> void:
	if _bind_channel >= 0:
		_bind_buttons[_bind_channel].text = "Bind"
		_bind_buttons[_bind_channel].theme_type_variation = "SmallButton"
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
		option.selected = clampi(previous, 0, option.item_count - 1)


func _refresh() -> void:
	SimLink.request("get_input")


## simcore lists devices again only when one is selected, so a rescan re-selects by a known
## device name; detection then runs on the fresh list
func _rescan() -> void:
	_detect_pending = true
	_list_profiles()
	if not SimLink.api_is_connected:
		_message.text = "start a session to look for controllers"
		return
	var needle := _profile_needle(_active_profile)
	if needle == "":
		needle = _profile_needle(_default_target)
	if needle == "":
		_refresh()
		return
	SimLink.request("select_input_device", {"name_contains": needle})


## The first profile whose device name occurs in a connected device's name; the preferred profile
## (the one default.yaml points at) is tried first
static func detect_profile(devices: Array, profiles: Array, preferred: String) -> Dictionary:
	var ordered: Array = []
	for entry: Dictionary in profiles:
		if str(entry.get("name", "")) == preferred:
			ordered.push_front(entry)
		else:
			ordered.append(entry)
	for entry: Dictionary in ordered:
		var needle := str(entry.get("device_name_contains", "")).to_lower()
		if not entry.has("mapping") or needle == "":
			continue
		for device: String in devices:
			if device.to_lower().contains(needle):
				return {"profile": str(entry["name"]), "device": device, "mapping": entry["mapping"]}
	return {}


## A mapping read back from JSON has every number as a float; axis and button indices are ints
static func clean_mapping(mapping: Dictionary) -> Dictionary:
	var clean := mapping.duplicate(true)
	var channels: Dictionary = clean.get("channels", {})
	for name: String in channels:
		var source: Dictionary = channels[name]
		for key: String in ["axis", "button"]:
			if source.has(key):
				source[key] = int(source[key])
	return clean


## A file name for a controller that has no profile yet
static func profile_slug(device_name: String) -> String:
	var slug := ""
	for ch in device_name.to_lower():
		var keep := (ch >= "a" and ch <= "z") or (ch >= "0" and ch <= "9")
		if keep:
			slug += ch
		elif not slug.ends_with("_") and slug != "":
			slug += "_"
	slug = slug.trim_suffix("_")
	return slug if slug != "" else "controller"


func _profile_entry(name: String) -> Dictionary:
	for entry: Dictionary in _profiles:
		if str(entry["name"]) == name:
			return entry
	return {}


func _profile_needle(name: String) -> String:
	return str(_profile_entry(name).get("device_name_contains", "")) if name != "" else ""


func _auto_detect() -> void:
	if not _profiles_loaded or _last_info.is_empty() or str(_last_info.get("source", "")) != "gamepad":
		return
	_detect_pending = false
	var device: Dictionary = _last_info["device"]
	var current := str(device["name"])
	var found := detect_profile(_device_names, _profiles, _default_target)
	if found.is_empty():
		_active_profile = ""
		_show_active_profile()
		if _device_names.is_empty():
			_message.text = "No controller found. Plug one in and press Refresh."
		else:
			_message.text = "No saved profile matches these devices. Pick yours above; your mapping changes are saved as a new profile."
		return
	_active_profile = found["profile"]
	_show_active_profile()
	var mapping: Dictionary = (found["mapping"] as Dictionary).duplicate(true)
	if not (device["connected"] and current == found["device"]):
		SimLink.request("select_input_device", {"name_contains": found["device"]})
	SimLink.request("set_input_mapping", {"mapping": mapping})
	_message.text = "Detected %s, using profile %s. Mapping changes are saved to it." % [found["device"], found["profile"]]


func _show_active_profile() -> void:
	for i in _profiles.size():
		if str(_profiles[i]["name"]) == _active_profile:
			_profile.select(i)
			_set_default.disabled = _active_profile == _default_target
			return


func _schedule_save() -> void:
	_save_at = Time.get_ticks_msec() * 1e-3 + SAVE_DELAY_S


func _process(_delta: float) -> void:
	var now := Time.get_ticks_msec() * 1e-3
	if _save_at > 0.0 and now >= _save_at:
		_save_at = -1.0
		_save_active()
	# simcore only lists devices while a gamepad session runs; until then the backend does
	if not _session_has_controller() and now >= _next_idle_poll and BackendClient.is_connected_to_backend:
		_next_idle_poll = now + IDLE_POLL_S
		BackendClient.request("list_input_devices")


func _session_has_controller() -> bool:
	return SimLink.api_is_connected and str(_last_info.get("source", "")) == "gamepad"


func _current_device_name() -> String:
	if _session_has_controller():
		return str(_last_info["device"]["name"])
	if _device.selected >= 0 and _device.selected < _device_names.size():
		return str(_device_names[_device.selected])
	return ""


func _fill_devices(current: String) -> void:
	_device.clear()
	if _device_names.is_empty():
		_device.add_item("No device detected")
		_device.disabled = true
	else:
		_device.disabled = false
	for name: String in _device_names:
		_device.add_item(name)
		if name == current:
			_device.selected = _device.item_count - 1


## No gamepad session: show the controllers and the profile each would use, and let the mapping
## be edited and saved now
func _apply_idle_devices(devices: Array) -> void:
	if _session_has_controller() or not _profiles_loaded:
		return
	_device_names = devices
	var found := detect_profile(devices, _profiles, _default_target)
	var key := "%s|%s" % [",".join(PackedStringArray(devices)), found.get("profile", "")]
	if key == _idle_key:
		return
	_idle_key = key
	_fill_devices(str(found.get("device", "")))
	var session_note := "a gamepad session such as dev_gamepad"
	if SimLink.api_is_connected:
		_device_status.text = "the running session does not use a controller; start %s to fly with it" % session_note
	else:
		_device_status.text = "no session running; start %s to fly with it" % session_note
	if found.is_empty():
		_active_profile = ""
		if devices.is_empty():
			_message.text = "No controller found. Plug one in; it shows up here within a few seconds."
		else:
			_message.text = "No saved profile matches %s. Your first mapping change creates one." % devices[0]
		return
	_active_profile = found["profile"]
	_show_active_profile()
	_mapping = (found["mapping"] as Dictionary).duplicate(true)
	_fill_table_from_mapping()
	_message.text = "Detected %s, profile %s. Mapping changes are saved to it." % [found["device"], found["profile"]]


## Edits go into the active profile's file; a controller without one gets a profile named after it
func _save_active() -> void:
	var mapping := clean_mapping(_mapping)
	var name := _active_profile
	if name == "":
		var device := _current_device_name()
		if device == "":
			_message.text = "Connect a controller to save its mapping."
			return
		name = profile_slug(device)
		mapping["device_name_contains"] = device
		_active_profile = name
	else:
		mapping["device_name_contains"] = _profile_needle(name)
	BackendClient.request("save_input_mapping", {"name": name, "mapping": mapping, "make_default": false})


func _select_device() -> void:
	if _device.selected < 0 or _device.selected >= _device_names.size():
		return
	var name: String = _device_names[_device.selected]
	var live := _session_has_controller()
	if live:
		SimLink.request("select_input_device", {"name_contains": name})
	var found := detect_profile([name], _profiles, _default_target)
	if found.is_empty():
		_active_profile = ""
		_message.text = "%s has no profile yet; your first mapping change creates one." % name
		return
	_active_profile = found["profile"]
	_show_active_profile()
	_mapping = (found["mapping"] as Dictionary).duplicate(true)
	if live:
		SimLink.request("set_input_mapping", {"mapping": _mapping.duplicate(true)})
	else:
		_fill_table_from_mapping()
	_message.text = "Using profile %s for %s." % [found["profile"], name]


func _on_command_result(method: String, ok: bool, result: Dictionary) -> void:
	if method == "get_input" and ok:
		_apply_input_info(result)
	elif method == "select_input_device":
		# success keeps whatever the panel said about the detected profile
		if not ok:
			_message.text = "select failed: %s" % result.get("message", "")
		_refresh()
	elif method == "set_input_mapping":
		if not ok:
			_message.text = "mapping rejected: %s" % result.get("message", "")
		_refresh()


func _apply_input_info(info: Dictionary) -> void:
	_last_info = info
	if not _session_has_controller():
		# an altitude-hold session has no controller; the backend's list stays in charge
		_idle_key = ""
		_next_idle_poll = 0.0
		return
	_device_names = info.get("devices", [])
	_fill_devices(str(info["device"]["name"]))
	var device: Dictionary = info["device"]
	_device_status.text = "%s · %d axes, %d buttons · source %s" % [
		"connected" if device["connected"] else "not connected",
		int(device["axis_count"]), int(device["button_count"]), info["source"]
	]
	_fill_source_options(maxi(8, int(device["axis_count"])), maxi(8, int(device["button_count"])))
	_mapping = info["mapping"]
	_fill_table_from_mapping()
	if _detect_pending:
		_auto_detect()


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
	if _session_has_controller():
		SimLink.request("set_input_mapping", {"mapping": _mapping})
	_schedule_save()


func _on_telemetry(data: Dictionary) -> void:
	var channels: Array = data["input"]["channels_us"]
	for i in mini(_channel_bars.size(), channels.size()):
		_channel_bars[i].value = channels[i]
		_channel_values[i].text = "%d" % roundi(float(channels[i]))
	for i in _aux_values.size():
		var index := MAIN_CHANNELS + i
		_aux_values[i].text = "%d" % roundi(float(channels[index])) if index < channels.size() else "—"
	if not _aux_open:
		_render_aux_toggle()


func _on_input_raw(data: Dictionary) -> void:
	_raw = data
	var now := Time.get_ticks_msec() * 1e-3
	_input_times.append(now)
	while _input_times.size() > 1 and _input_times[0] < now - 2.0:
		_input_times.pop_front()
	if _input_times.size() > 1:
		_rate.text = "rc %d Hz" % roundi((_input_times.size() - 1) / maxf(_input_times[-1] - _input_times[0], 1e-3))
	var axes: Array = data["axes"]
	for i in _axis_bars.size():
		var value: float = float(axes[i]) if i < axes.size() else 0.0
		_axis_bars[i].value = value
		_axis_values[i].text = "%.2f" % value
	var buttons: Array = data["buttons"]
	var pressed: PackedStringArray = []
	for i in buttons.size():
		if int(buttons[i]) != 0:
			pressed.append(str(i))
	_buttons_label.text = "buttons: %s" % (", ".join(pressed) if pressed.size() > 0 else "none")
	if _bind_channel >= 0:
		_bind_watch(axes, buttons)
	elif _wizard_step >= 0:
		_wizard_watch(axes, buttons)


# calibration wizard

func _start_wizard() -> void:
	if _raw.is_empty():
		_message.text = "Calibration needs live stick input: start a gamepad session such as dev_gamepad first."
		return
	_wizard_mapping = {}
	_wizard_step = 0
	_wizard.visible = true
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
		_schedule_save()
		_message.text = "calibration applied"
		return
	_wizard_begin_step()


func _wizard_end() -> void:
	_wizard_step = -1
	_wizard.visible = false
	_calibrate.disabled = false


func _save() -> void:
	var name := _save_name.text.strip_edges()
	if name.is_empty():
		_message.text = "give the profile a name"
		return
	var mapping := clean_mapping(_mapping)
	var needle := _profile_needle(_active_profile)
	mapping["device_name_contains"] = needle if needle != "" else str(_last_info.get("device", {}).get("name", ""))
	_active_profile = name
	BackendClient.request("save_input_mapping", {"name": name, "mapping": mapping, "make_default": _save_default.button_pressed})


func _list_profiles() -> void:
	BackendClient.request("list_input_mappings")


func _on_profile_selected(index: int) -> void:
	if index < 0 or index >= _profiles.size():
		return
	var entry: Dictionary = _profiles[index]
	_set_default.disabled = entry["name"] == _default_target
	if not entry.has("mapping"):
		_message.text = "%s is not a valid mapping: %s" % [entry["name"], entry.get("error", "")]
		return
	_active_profile = str(entry["name"])
	_mapping = (entry["mapping"] as Dictionary).duplicate(true)
	_fill_table_from_mapping()
	if _session_has_controller():
		SimLink.request("set_input_mapping", {"mapping": _mapping})
	_message.text = "Using profile %s. Mapping changes are saved to it." % _active_profile


func _set_selected_default() -> void:
	if _profile.selected < 0 or _profile.selected >= _profiles.size():
		return
	var name: String = _profiles[_profile.selected]["name"]
	BackendClient.request("save_input_mapping", {"name": name, "mapping": _mapping, "make_default": true})


func _on_backend_response(method: String, ok: bool, result: Dictionary) -> void:
	if method == "save_input_mapping":
		_message.text = "Saved to %s%s" % [result.get("path", ""), " (default)" if result.get("default", false) else ""] if ok else "save failed: %s" % result.get("message", "")
		if ok:
			_save_row.visible = false
			_list_profiles()
	elif method == "list_input_devices" and ok:
		_apply_idle_devices(result.get("devices", []))
	elif method == "list_input_mappings" and ok:
		# default.yaml only points at a profile, so it is listed as a mark on that one
		_profiles = []
		_default_target = ""
		for entry: Dictionary in result.get("mappings", []):
			if str(entry["name"]) == "default":
				_default_target = str(entry.get("profile", ""))
			else:
				_profiles.append(entry)
		_profile.clear()
		for entry: Dictionary in _profiles:
			var label := str(entry["name"])
			if label == _default_target:
				label += "  · default"
			if not entry.has("mapping"):
				label += "  (invalid)"
			_profile.add_item(label)
		_profile.disabled = _profiles.is_empty()
		_profiles_loaded = true
		_show_active_profile()
		if _detect_pending:
			_auto_detect()
