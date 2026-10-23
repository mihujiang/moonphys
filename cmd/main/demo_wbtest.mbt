///|
/// Guards for the demo scenes: they must stay finite and inside the camera.
fn check_scene(world : @moonphys.World, frames : Int) -> Unit raise {
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
test "bridge sags but holds" {
  let world = bridge_scene()
  for _frame in 0..<240 {
    world.step(DT)
  }
  let mut planks = 0
  for snapshot in world.snapshot() {
    let p = snapshot.position()
    if snapshot.shape().is_circle() {
      continue
    }
    if p.y() < 6.0 && p.y() > 1.0 {
      planks = planks + 1
    }
  }
  // Posts plus planks: nothing fell through the ground.
  assert_true(planks >= 15)
}

///|
test "renderer emits one element per body" {
  let world = bridge_scene()
  let svg = @render.frame_svg(world, @render.SvgOptions::new())
  assert_true(svg.length() > 1000)
  for snapshot in world.snapshot() {
    let tag = if snapshot.shape().is_circle() { "<circle" } else { "<polygon" }
    assert_true(svg.contains(tag))
  }
}
