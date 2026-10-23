///|
/// Serialise the current visual state of the world as JSON.
///
/// The payload is small and allocation friendly (a single string), so it can be
/// used to debug a simulation, to record a replay, or to hand the frame to a
/// front end that draws it:
///
/// ```json
/// {
///   "time": 2.5,
///   "bodies": [
///     {"i": 0, "shape": "polygon", "v": [[-20, 0], [20, 0], [20, 1], [-20, 1]],
///      "pos": [0, 0.5], "angle": 0, "asleep": true, "tag": 0},
///     {"i": 1, "shape": "circle", "r": 0.4, "pos": [0, 3], "angle": 0,
///      "asleep": false, "tag": 7}
///   ]
/// }
/// ```
///
/// Circles carry `r`, polygons carry their vertices `v`, both are in local
/// space: apply `pos` and `angle` to place them.
pub fn World::snapshot_json(self : World) -> String {
  let out = StringBuilder(size_hint=2048)
  out.write_string("{\"time\":")
  write_json_number(out, self.time())
  out.write_string(",\"bodies\":[")
  let bodies = self.snapshot()
  let mut first = true
  for index, body in bodies {
    if !first {
      out.write_char(',')
    }
    first = false
    write_json_body(out, index, body)
  }
  out.write_string("]}")
  out.to_string()
}

///|
fn write_json_number(out : StringBuilder, value : Double) -> Unit {
  out.write_string(value.to_string())
}

///|
fn write_json_vec(out : StringBuilder, value : Vec2) -> Unit {
  out.write_char('[')
  write_json_number(out, value.x())
  out.write_char(',')
  write_json_number(out, value.y())
  out.write_char(']')
}

///|
fn write_json_body(
  out : StringBuilder,
  index : Int32,
  body : BodySnapshot,
) -> Unit {
  out.write_string("{\"i\":")
  out.write_string(index.to_string())
  out.write_string(",\"shape\":")
  match body.shape() {
    Circle(radius) => {
      out.write_string("\"circle\",\"r\":")
      write_json_number(out, radius)
    }
    Polygon(vertices) => {
      out.write_string("\"polygon\",\"v\":[")
      let mut first = true
      for vertex in vertices {
        if !first {
          out.write_char(',')
        }
        first = false
        write_json_vec(out, vertex)
      }
      out.write_char(']')
    }
  }
  out.write_string(",\"pos\":")
  write_json_vec(out, body.position())
  out.write_string(",\"angle\":")
  write_json_number(out, body.rotation())
  out.write_string(",\"asleep\":")
  out.write_string(if body.asleep() { "true" } else { "false" })
  out.write_string(",\"tag\":")
  out.write_string(body.user_data().to_string())
  out.write_char('}')
}
