extends VBoxContainer
## Environment tab: mean wind, turbulence and gusts, changed live through the backend

const SEND_DELAY_S := 0.25
const INTENSITIES := ["none", "light", "moderate", "severe"]
const COMPASS := ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]

var _running := false
var _run_dir := ""
var _send_at := -1.0
var _loading := false

@onready var _live: Label = $Wind/Head/Live
@onready var _speed: HSlider = $Wind/Speed/Slider
@onready var _speed_value: Label = $Wind/Speed/Value
@onready var _from: HSlider = $Wind/From/Slider
@onready var _from_value: Label = $Wind/From/Value
@onready var _turbulence: OptionButton = $Turbulence/Level
@onready var _gust_strength: HSlider = $Gust/Strength/Slider
@onready var _gust_strength_value: Label = $Gust/Strength/Value
@onready var _gust_duration: HSlider = $Gust/Duration/Slider
@onready var _gust_duration_value: Label = $Gust/Duration/Value
@onready var _gust_from: HSlider = $Gust/From/Slider
@onready var _gust_from_value: Label = $Gust/From/Value
@onready var _gust_button: Button = $Gust/Fire
@onready var _message: Label = $Message


func _ready() -> void:
	for name: String in INTENSITIES:
		_turbulence.add_item(name)
	_speed.value_changed.connect(func(_v: float) -> void: _wind_changed())
	_from.value_changed.connect(func(_v: float) -> void: _wind_changed())
	_turbulence.item_selected.connect(func(_i: int) -> void: _send({"wind": {"turbulence": {"intensity": INTENSITIES[_turbulence.selected]}}}))
	for slider: HSlider in [_gust_strength, _gust_duration, _gust_from]:
		slider.value_changed.connect(func(_v: float) -> void: _render_labels())
	_gust_button.pressed.connect(_fire_gust)
	BackendClient.status.connect(_on_status)
	BackendClient.response.connect(_on_response)
	SimLink.telemetry.connect(_on_telemetry)
	_render_labels()
	_on_status(BackendClient.last_status)


static func compass(from_deg: float) -> String:
	var index := int(roundf(fposmod(from_deg, 360.0) / 45.0)) % COMPASS.size()
	return COMPASS[index]


## Wind in NED (where the air goes) as speed and the meteorological direction it comes from
static func speed_and_from(wind_ned: Array) -> Vector2:
	var north := float(wind_ned[0])
	var east := float(wind_ned[1])
	var speed := Vector2(north, east).length()
	var from_deg := fposmod(rad_to_deg(atan2(-east, -north)), 360.0) if speed > 1e-6 else 0.0
	return Vector2(speed, from_deg)


func _render_labels() -> void:
	_speed_value.text = "%.1f m/s" % _speed.value
	_from_value.text = "%03d° %s" % [roundi(_from.value), compass(_from.value)]
	_gust_strength_value.text = "%.1f m/s" % _gust_strength.value
	_gust_duration_value.text = "%.1f s" % _gust_duration.value
	_gust_from_value.text = "%03d° %s" % [roundi(_gust_from.value), compass(_gust_from.value)]


func _wind_changed() -> void:
	_render_labels()
	if not _loading:
		_send_at = Time.get_ticks_msec() * 1e-3 + SEND_DELAY_S


func _process(_delta: float) -> void:
	if _send_at > 0.0 and Time.get_ticks_msec() * 1e-3 >= _send_at:
		_send_at = -1.0
		_send({"wind": {"mean": {"speed_mps": _speed.value, "from_deg": _from.value}}})


func _fire_gust() -> void:
	_send({"gust": {"amplitude_mps": _gust_strength.value, "duration_s": _gust_duration.value, "from_deg": _gust_from.value}})


func _send(params: Dictionary) -> void:
	if not _running:
		_message.text = "Start a session to change the wind live."
		return
	BackendClient.request("set_env", params)


func _on_response(method: String, ok: bool, result: Dictionary) -> void:
	if method != "set_env":
		return
	if ok:
		var at: Variant = result.get("applied_at_ns")
		_message.text = "Applied at t %.2f s." % (float(at) * 1e-9) if at != null else "Applied."
	else:
		_message.text = "Not applied: %s" % result.get("message", "")


func _on_status(data: Dictionary) -> void:
	_running = data.get("state", "") == "running"
	_set_enabled(_running)
	var run_dir := str(data.get("run_dir", "")) if data.get("run_dir") != null else ""
	if not _running:
		_run_dir = ""
		_live.text = "no session"
		_message.text = "Start a session to change the wind live. The session's environment file sets where it starts."
		return
	if run_dir != _run_dir:
		_run_dir = run_dir
		_message.text = ""
		_load_session_wind(run_dir)


func _set_enabled(on: bool) -> void:
	for slider: HSlider in [_speed, _from, _gust_strength, _gust_duration, _gust_from]:
		slider.editable = on
	_turbulence.disabled = not on
	_gust_button.disabled = not on


## The controls start where the session's environment file put the wind
func _load_session_wind(run_dir: String) -> void:
	var text := FileAccess.get_file_as_string(run_dir + "/resolved/session.json")
	var session: Variant = JSON.parse_string(text) if text != "" else null
	if not (session is Dictionary) or not (session as Dictionary).has("wind"):
		return
	var wind: Dictionary = session["wind"]
	_loading = true
	_speed.value = float(wind.get("mean_speed_mps", 0.0))
	_from.value = fposmod(rad_to_deg(float(wind.get("mean_from_rad", 0.0))), 360.0)
	_gust_from.value = _from.value
	var intensity := INTENSITIES.find(str(wind.get("turbulence_intensity", "none")))
	_turbulence.select(maxi(intensity, 0))
	_loading = false
	_render_labels()


func _on_telemetry(data: Dictionary) -> void:
	var environment: Variant = data.get("environment")
	if not (environment is Dictionary):
		return
	var now := speed_and_from((environment as Dictionary)["wind_ned_mps"])
	_live.text = "now %.1f m/s from %03d°" % [now.x, roundi(now.y)]
