///|
test "distance joint holds its length" {
  let world = World::new()
  let pivot = world.add_body(Body::fixed(Shape::circle(0.1)).at(0.0, 5.0))
  let ball = world.add_body(Body::ball(0.3).at(2.0, 5.0))
  world.add_distance_joint(pivot, ball)
  world.run(240, FRAME)
  let distance = (world.body(pivot).position() - world.body(ball).position()).length()
  assert_true(approx(distance, 2.0, 0.05))
}

///|
test "distance joint swings under gravity" {
  let world = World::new()
  let pivot = world.add_body(Body::fixed(Shape::circle(0.1)).at(0.0, 5.0))
  let ball = world.add_body(Body::ball(0.3).at(2.0, 5.0))
  world.add_distance_joint(pivot, ball)
  world.run(60, FRAME)
  let position = world.body(ball).position()
  assert_true(position.y() < 4.9)
}

///|
test "revolve joint pins its anchor and lets the body swing" {
  let world = World::new()
  let mount = world.add_body(Body::fixed(Shape::circle(0.1)).at(0.0, 6.0))
  let bar = world.add_body(Body::box(1.0, 0.1).at(1.0, 5.0))
  world.add_revolve_joint_local(
    mount,
    bar,
    Vec2::new(0.0, -1.0),
    Vec2::new(-1.0, 0.0),
  )
  let mut max_swing = 0.0
  for _frame in 0..<240 {
    world.step(FRAME)
    max_swing = max_swing.max(world.body(bar).rotation().abs())
  }
  let anchor_world = world.body(bar).local_to_world(Vec2::new(-1.0, 0.0))
  assert_true((anchor_world - Vec2::new(0.0, 5.0)).length() < 0.05)
  assert_true(max_swing > 1.0)
}
