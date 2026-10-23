///|
/// A four segment ragdoll hanging from a static mount, then kicked sideways.
///
/// ```bash
/// moon run examples/ragdoll
/// ```
///
/// It shows the whole joint workflow: add bodies, remember their indices, then
/// wire them up with revolute joints on shared anchor points.
fn main {
  let world = @moonphys.World::new()
  let mount = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.2, 0.1)).at(0.0, 6.0),
  )
  // Head, torso, thigh, shin: each one hangs one joint below the previous.
  let head = world.add_body(
    @moonphys.Body::box(0.3, 0.3).at(0.0, 5.4).with_density(0.6),
  )
  let torso = world.add_body(@moonphys.Body::box(0.3, 0.55).at(0.0, 4.5))
  let thigh = world.add_body(@moonphys.Body::box(0.18, 0.45).at(0.0, 3.45))
  let shin = world.add_body(@moonphys.Body::box(0.16, 0.4).at(0.0, 2.5))
  world.add_revolve_joint(mount, head, @moonphys.Vec2::new(0.0, 5.75))
  world.add_revolve_joint(head, torso, @moonphys.Vec2::new(0.0, 5.0))
  world.add_revolve_joint(torso, thigh, @moonphys.Vec2::new(0.0, 3.9))
  world.add_revolve_joint(thigh, shin, @moonphys.Vec2::new(0.0, 3.0))
  // A kick to the shin gets the whole chain swinging.
  world
  .body(shin)
  .apply_impulse(@moonphys.Vec2::new(6.0, 1.0), @moonphys.Vec2::new(0.0, -0.3))
  println("joints=" + world.joint_count().to_string())
  for step in 0..<600 {
    world.step(1.0 / 60.0)
    if step % 120 == 119 {
      println(
        "t=" +
        round(world.time(), 10.0).to_string() +
        " head.x=" +
        round(world.body(head).position().x(), 100.0).to_string() +
        " shin.x=" +
        round(world.body(shin).position().x(), 100.0).to_string(),
      )
    }
  }
}

///|
/// Round `value` to `scale` steps per unit.
fn round(value : Double, scale : Double) -> Double {
  (value * scale).round() / scale
}
