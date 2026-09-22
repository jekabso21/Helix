extends Node
## Dev autoload: "./scripts/run_app.sh -- --screenshot <path> [--screenshot-after <s>]" saves the viewport and quits

const ARGUMENT := "--screenshot"
const SETTLE_FRAMES := 20


func _ready() -> void:
	var args := OS.get_cmdline_user_args()
	var index := args.find(ARGUMENT)
	if index < 0 or index + 1 >= args.size():
		return
	var delay_index := args.find("--screenshot-after")
	var delay_s := 0.0
	if delay_index >= 0 and delay_index + 1 < args.size():
		delay_s = float(args[delay_index + 1])
	_capture.call_deferred(args[index + 1], delay_s)


func _capture(path: String, delay_s: float) -> void:
	if delay_s > 0.0:
		await get_tree().create_timer(delay_s).timeout
	for _frame in SETTLE_FRAMES:
		await get_tree().process_frame
	await RenderingServer.frame_post_draw
	var image := get_viewport().get_texture().get_image()
	var error := image.save_png(path)
	if error != OK:
		push_error("screenshot failed: %s (%s)" % [path, error_string(error)])
	else:
		print("screenshot %dx%d saved to %s" % [image.get_width(), image.get_height(), path])
	get_tree().quit(0 if error == OK else 1)
