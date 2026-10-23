///|
/// Canvas and camera settings for SVG output.
pub struct SvgOptions {
  mut width : Int
  mut height : Int
  mut scale : Double
  mut center_x : Double
  mut center_y : Double
  mut background : String
  mut grid : Bool
}

///|
pub fn SvgOptions::new() -> SvgOptions {
  {
    width: 960,
    height: 640,
    scale: 52.0,
    center_x: 0.0,
    center_y: 4.0,
    background: "#0b1020",
    grid: true,
  }
}

///|
pub fn SvgOptions::with_size(
  self : SvgOptions,
  width : Int,
  height : Int,
) -> SvgOptions {
  self.width = width
  self.height = height
  self
}

///|
pub fn SvgOptions::with_scale(self : SvgOptions, scale : Double) -> SvgOptions {
  self.scale = scale
  self
}

///|
pub fn SvgOptions::with_center(
  self : SvgOptions,
  x : Double,
  y : Double,
) -> SvgOptions {
  self.center_x = x
  self.center_y = y
  self
}

///|
pub fn SvgOptions::with_grid(self : SvgOptions, enabled : Bool) -> SvgOptions {
  self.grid = enabled
  self
}

///|
pub fn SvgOptions::with_background(
  self : SvgOptions,
  color : String,
) -> SvgOptions {
  self.background = color
  self
}

///|
/// Palette used for bodies, keyed by `user_data`.
fn palette(index : Int) -> String {
  let colors : Array[String] = [
    "#7dd3fc", "#fbbf24", "#f472b6", "#a78bfa", "#34d399", "#fb7185", "#60a5fa",
    "#f59e0b", "#c084fc", "#4ade80",
  ]
  let i = index % colors.length()
  let j = if i < 0 { i + colors.length() } else { i }
  colors[j]
}

///|
/// Short fixed-point number, to keep the SVG compact.
fn fmt(v : Double) -> String {
  if v.abs() > 1000000.0 {
    return "0"
  }
  let scaled = v * 100.0
  let rounded = if scaled >= 0.0 {
    (scaled + 0.5).to_int()
  } else {
    (scaled - 0.5).to_int()
  }
  (rounded.to_double() / 100.0).to_string()
}

///|
fn screen_x(options : SvgOptions, x : Double) -> Double {
  options.width.to_double() * 0.5 + (x - options.center_x) * options.scale
}

///|
fn screen_y(options : SvgOptions, y : Double) -> Double {
  options.height.to_double() * 0.5 - (y - options.center_y) * options.scale
}

///|
fn write_grid(sb : StringBuilder, options : SvgOptions) -> Unit {
  let half_w = options.width.to_double() * 0.5
  let half_h = options.height.to_double() * 0.5
  let min_x = options.center_x - half_w / options.scale
  let max_x = options.center_x + half_w / options.scale
  let min_y = options.center_y - half_h / options.scale
  let max_y = options.center_y + half_h / options.scale
  sb.write_object("  <g stroke=\"#16233b\" stroke-width=\"1\">\n")
  let mut x = min_x.to_int().to_double()
  while x <= max_x {
    let sx = fmt(screen_x(options, x))
    sb.write_object(
      "    <line x1=\"\{sx}\" y1=\"0\" x2=\"\{sx}\" y2=\"\{options.height}\"/>\n",
    )
    x = x + 1.0
  }
  let mut y = min_y.to_int().to_double()
  while y <= max_y {
    let sy = fmt(screen_y(options, y))
    sb.write_object(
      "    <line x1=\"0\" y1=\"\{sy}\" x2=\"\{options.width}\" y2=\"\{sy}\"/>\n",
    )
    y = y + 1.0
  }
  sb.write_object("  </g>\n")
}

///|
fn write_polygon(
  sb : StringBuilder,
  snapshot : @moonphys.BodySnapshot,
  options : SvgOptions,
) -> Unit {
  let transform = @moonphys.Transform::new(
    snapshot.position(),
    snapshot.rotation(),
  )
  let vertices = snapshot.shape().vertices()
  if vertices.length() < 3 {
    return
  }
  let mut points = ""
  for i in 0..<vertices.length() {
    let world = transform.apply(vertices[i])
    if i > 0 {
      points = points + " "
    }
    points = points +
      fmt(screen_x(options, world.x())) +
      "," +
      fmt(screen_y(options, world.y()))
  }
  let fill = palette(snapshot.user_data())
  let opacity = if snapshot.asleep() { "0.55" } else { "1" }
  sb.write_object(
    "  <polygon points=\"\{points}\" fill=\"\{fill}\" fill-opacity=\"\{opacity}\" stroke=\"#0b1020\" stroke-width=\"1.5\"/>\n",
  )
}

///|
fn write_circle(
  sb : StringBuilder,
  snapshot : @moonphys.BodySnapshot,
  options : SvgOptions,
) -> Unit {
  let radius = snapshot.shape().radius()
  let position = snapshot.position()
  let fill = palette(snapshot.user_data())
  let opacity = if snapshot.asleep() { "0.55" } else { "1" }
  sb.write_object(
    "  <circle cx=\"\{fmt(screen_x(options, position.x()))}\" cy=\"\{fmt(screen_y(options, position.y()))}\" r=\"\{fmt(radius * options.scale)}\" fill=\"\{fill}\" fill-opacity=\"\{opacity}\" stroke=\"#0b1020\" stroke-width=\"1.5\"/>\n",
  )
}

///|
/// Render one frame as an inline SVG element.
pub fn frame_svg(world : @moonphys.World, options : SvgOptions) -> String {
  let sb = StringBuilder::new()
  sb.write_object(
    "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 \{options.width} \{options.height}\" preserveAspectRatio=\"xMidYMid meet\">\n",
  )
  sb.write_object(
    "  <rect width=\"100%\" height=\"100%\" fill=\"\{options.background}\"/>\n",
  )
  if options.grid {
    write_grid(sb, options)
  }
  for snapshot in world.snapshot() {
    if snapshot.shape().is_circle() {
      write_circle(sb, snapshot, options)
    } else {
      write_polygon(sb, snapshot, options)
    }
  }
  sb.write_object("</svg>\n")
  sb.to_string()
}

///|
/// Render one frame as a standalone SVG document.
pub fn svg_document(world : @moonphys.World, options : SvgOptions) -> String {
  let body = frame_svg(world, options)
  "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" + body
}
