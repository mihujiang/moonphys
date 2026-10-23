///|
/// The smallest interesting moonphys program: a bouncy ball dropped on a floor.
///
/// ```bash
/// moon run examples/bounce
/// ```
fn main {
  let world = @moonphys.World::builder()
    .ground(10.0)
    .body(
      @moonphys.Body::ball(0.4)
      .at(0.0, 4.0)
      .with_restitution(0.75)
      .with_friction(0.2),
    )
    .build()
  let ball = 1
  for step in 0..<240 {
    world.step(1.0 / 60.0)
    // Print four times a second so the bounce is visible in the terminal.
    if step % 15 == 0 {
      let body = world.body(ball)
      println(
        "t=" +
        round(world.time(), 100.0).to_string() +
        " y=" +
        round(body.position().y(), 100.0).to_string() +
        " vy=" +
        round(body.velocity().y(), 100.0).to_string(),
      )
    }
  }
  let body = world.body(ball)
  println(
    "rest: y=" +
    round(body.position().y(), 1000.0).to_string() +
    " state=" +
    (if body.awake() { "awake" } else { "asleep" }),
  )
}

///|
/// Round `value` to `scale` steps per unit, so the output stays readable.
fn round(value : Double, scale : Double) -> Double {
  (value * scale).round() / scale
}
