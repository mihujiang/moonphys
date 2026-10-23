///|
/// A five layer box pyramid that settles and then goes to sleep.
///
/// ```bash
/// moon run examples/pyramid
/// ```
///
/// Kinetic energy is the honest progress bar of a rigid body solver: it has to
/// fall to near zero, and the `asleep` count shows the engine noticing that by
/// itself and switching the pile off.
///
/// Six layers and above is where a ten iteration sequential impulse solver
/// starts to hum: the pile keeps a few centimetres per second of residual speed
/// instead of settling. Raise the iterations if you need taller piles.
fn main {
  let layers = 5
  let world = @moonphys.World::builder().ground(30.0).build()
  for layer in 0..<layers {
    let count = layers - layer
    for i in 0..<count {
      let x = (i.to_double() - (count - 1).to_double() / 2.0) * 0.86
      let y = 0.4 + layer.to_double() * 0.8
      ignore(
        world.add_body(
          @moonphys.Body::box(0.42, 0.4).at(x, y).with_friction(0.7),
        ),
      )
    }
  }
  println("bodies=" + world.body_count().to_string())
  for step in 0..<600 {
    world.step(1.0 / 60.0)
    if step % 60 == 59 {
      let mut asleep = 0
      for i in 1..<world.body_count() {
        if !world.body(i).awake() {
          asleep = asleep + 1
        }
      }
      println(
        "t=" +
        round(world.time(), 10.0).to_string() +
        "s ek=" +
        round(world.total_kinetic_energy(), 10000.0).to_string() +
        " asleep=" +
        asleep.to_string() +
        "/" +
        (world.body_count() - 1).to_string(),
      )
    }
  }
  let top = world.body(world.body_count() - 1)
  println("top box y=" + round(top.position().y(), 1000.0).to_string())
}

///|
/// Round `value` to `scale` steps per unit.
fn round(value : Double, scale : Double) -> Double {
  (value * scale).round() / scale
}
