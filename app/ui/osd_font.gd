extends RefCounted
## A real flight controller OSD font: MAX7456 .mcm (12x18 glyphs) or a PNG atlas of stacked glyphs

const MCM_HEADER := "MAX7456"
const MCM_GLYPH_WIDTH := 12
const MCM_GLYPH_HEIGHT := 18
const MCM_LINES_PER_GLYPH := 64
const MCM_PIXELS_PER_LINE := 4

var texture: Texture2D = null
var glyph_size := Vector2i.ZERO
var glyph_count := 0
var error := ""


func is_loaded() -> bool:
	return texture != null and glyph_count > 0


## Glyphs are stacked in one column, so a code is a row in the atlas
func region(code: int) -> Rect2:
	var index := clampi(code, 0, maxi(glyph_count - 1, 0))
	return Rect2(0, index * glyph_size.y, glyph_size.x, glyph_size.y)


func load_path(path: String) -> bool:
	error = ""
	texture = null
	glyph_count = 0
	if path == "":
		return false
	if path.to_lower().ends_with(".mcm"):
		return _load_mcm(path)
	if path.to_lower().ends_with(".png"):
		return _load_png(path)
	error = "unknown font format: " + path.get_file()
	return false


func _load_mcm(path: String) -> bool:
	var text := FileAccess.get_file_as_string(path)
	if text == "":
		error = "cannot read " + path
		return false
	return load_mcm_text(text)


## The file is one bit pair per pixel: 00 black, 10 white, anything else transparent
func load_mcm_text(text: String) -> bool:
	var lines := text.replace("\r", "").split("\n", false)
	if lines.is_empty() or lines[0].strip_edges() != MCM_HEADER:
		error = "not a MAX7456 font"
		return false
	var body: Array = []
	for i in range(1, lines.size()):
		var line := lines[i].strip_edges()
		if line.length() == 8:
			body.append(line)
	glyph_count = body.size() / MCM_LINES_PER_GLYPH
	if glyph_count == 0:
		error = "the font has no complete glyphs"
		return false
	glyph_size = Vector2i(MCM_GLYPH_WIDTH, MCM_GLYPH_HEIGHT)
	var image := Image.create(glyph_size.x, glyph_size.y * glyph_count, false, Image.FORMAT_RGBA8)
	for glyph in glyph_count:
		for row in MCM_GLYPH_HEIGHT:
			var line_index := glyph * MCM_LINES_PER_GLYPH + row * (MCM_GLYPH_WIDTH / MCM_PIXELS_PER_LINE)
			for part in MCM_GLYPH_WIDTH / MCM_PIXELS_PER_LINE:
				var bits: String = body[line_index + part]
				for pixel in MCM_PIXELS_PER_LINE:
					var pair := bits.substr(pixel * 2, 2)
					var colour := Color(0, 0, 0, 0)
					if pair == "00":
						colour = Color(0, 0, 0, 1)
					elif pair == "10":
						colour = Color(1, 1, 1, 1)
					image.set_pixel(part * MCM_PIXELS_PER_LINE + pixel, glyph * glyph_size.y + row, colour)
	texture = ImageTexture.create_from_image(image)
	return true


## HD fonts ship as a PNG with every glyph stacked in one column
func _load_png(path: String) -> bool:
	var image := Image.load_from_file(path)
	if image == null:
		error = "cannot read " + path
		return false
	return load_image(image)


## Analog fonts hold 256 glyphs and HD fonts 512; both are about 2:3, which tells them apart
func load_image(image: Image) -> bool:
	var width := image.get_width()
	var height := image.get_height()
	var best := 0
	var best_distance := INF
	for count: int in [256, 512]:
		if width <= 0 or height % count != 0 or height / count <= 0:
			continue
		var distance: float = absf(float(height / count) / float(width) - 1.5)
		if distance < best_distance:
			best_distance = distance
			best = count
	if best == 0:
		error = "a font atlas must stack 256 or 512 glyphs in one column"
		return false
	glyph_count = best
	glyph_size = Vector2i(width, height / best)
	texture = ImageTexture.create_from_image(image)
	return true
