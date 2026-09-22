extends PanelContainer
## Telemetry from the control API topic and FC status from the backend's MSP poller

@onready var _text: RichTextLabel = $Text

var _telemetry: Dictionary = {}
var _fc: Dictionary = {}


func _ready() -> void:
	SimLink.telemetry.connect(_on_telemetry)
	BackendClient.fc_status.connect(_on_fc)
	_render()


func _on_telemetry(data: Dictionary) -> void:
	_telemetry = data
	_render()


func _on_fc(data: Dictionary) -> void:
	_fc = data
	_render()


func _render() -> void:
	var lines: PackedStringArray = []
	if _telemetry.is_empty():
		lines.append("[b]Telemetry[/b]  waiting for simcore")
	else:
		var flight: Dictionary = _telemetry["flight"]
		var sim: Dictionary = _telemetry["sim"]
		lines.append("[b]Flight[/b]  alt %.2f m  gs %.1f m/s  climb %+.1f m/s" % [
			flight["altitude_agl_m"], flight["ground_speed_mps"], flight["climb_mps"]
		])
		lines.append("roll %+.1f  pitch %+.1f  hdg %.0f deg" % [
			rad_to_deg(flight["roll_rad"]), rad_to_deg(flight["pitch_rad"]),
			fposmod(rad_to_deg(flight["heading_rad"]), 360.0)
		])
		var motors: Array = _telemetry["motors"]
		var parts: PackedStringArray = []
		for motor: Dictionary in motors:
			parts.append("%d%% %.0f %.1fA" % [roundi(motor["command"] * 100.0), motor["rpm"], motor.get("current_a", 0.0)])
		lines.append("[b]Motors[/b]  " + "  |  ".join(parts))
		var battery: Variant = _telemetry.get("battery")
		if battery is Dictionary:
			lines.append("[b]Battery[/b]  %.2f V  %.1f A  %.0f mAh  %.0f%%%s" % [
				battery["voltage_v"], battery["current_a"], battery["consumed_mah"],
				100.0 * float(battery["soc"]), "  CUTOFF" if battery.get("cutoff", false) else ""
			])
		lines.append("[b]Sim[/b]  %s  overruns %d%s%s" % [
			sim["mode"], sim["overruns"],
			"  PAUSED" if sim["paused"] else "",
			"  CRASHED" if sim["crashed"] else ""
		])
	if _fc.is_empty():
		lines.append("[b]FC[/b]  no MSP status")
	else:
		var flags: Array = _fc["arming_disable_flags"]
		lines.append("[b]FC[/b]  %s  %s" % [
			"ARMED" if _fc["armed"] else "disarmed",
			"ok" if flags.is_empty() else ", ".join(flags)
		])
	_text.text = "\n".join(lines)
