///|
/// Guards for the demo scenes: they must stay finite and inside the camera.
fn check_scene(world : @moonphys.World, frames : Int32) -> Unit raise {
  for _frame in 0..<frames {
    world.step(DT)
  }
  for snapshot in world.snapshot() {
    let p = snapshot.position()
    // `abs() < bound` is also false for NaN, so this catches solver blow ups.
    assert_true(p.x().abs() < 40.0)
    assert_true(p.y().abs() < 40.0)
    assert_true(p.y() > -0.5)
  }
}

///|
test "pyramid scene stays stable" {
  let world = pyramid_scene()
  check_scene(world, 240)
  // The pyramid still stands after four seconds.
  let mut high = 0
  for snapshot in world.snapshot() {
    if snapshot.position().y() > 2.0 {
      high = high + 1
    }
  }
  assert_true(high >= 5)
}

///|
test "chain and pendulum stay attached" {
  let world = pendulum_scene()
  for _frame in 0..<240 {
    world.step(DT)
  }
  for snapshot in world.snapshot() {
    assert_true(snapshot.position().x().abs() < 20.0)
    assert_true(snapshot.position().y() > -0.5)
    assert_true(snapshot.position().y() < 20.0)
  }
}

///|
test "linear actuator sweeps the crates away" {
  let world = machine_scene()
  check_scene(world, 240)
  // The cart is driven to the right at 3.2 units/s, so the crates it shoves
  // must end up well past their starting column.
  let mut furthest = -100.0
  for snapshot in world.snapshot() {
    let p = snapshot.position()
    if !snapshot.shape().is_circle() && p.y() < 1.0 {
      furthest = furthest.max(p.x())
    }
  }
  assert_true(furthest > 3.0)
}

///|
test "seesaw rests on its angle stop" {
  let world = machine_scene()
  for _frame in 0..<240 {
    world.step(DT)
  }
  let mut beam_y = -1.0
  for snapshot in world.snapshot() {
    let p = snapshot.position()
    if snapshot.shape().is_circle() {
      continue
    }
    if p.x() > 5.0 && p.x() < 8.5 && p.y() > 2.0 {
      beam_y = p.y()
    }
  }
  // Pivoted at y = 8.4 with a -0.6 rad stop, the beam hangs high above the floor.
  assert_true(beam_y > 7.0)
}

///|
test "renderer emits one element per body" {
  let world = machine_scene()
  let svg = @render.frame_svg(world, @render.SvgOptions::new())
  assert_true(svg.length() > 1000)
  for snapshot in world.snapshot() {
    let tag = if snapshot.shape().is_circle() { "<circle" } else { "<polygon" }
    assert_true(svg.contains(tag))
  }
}
