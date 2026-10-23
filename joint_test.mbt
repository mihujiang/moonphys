///|
test "distance joint holds its length" {
  let world = @moonphys.World::new()
  let pivot = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::circle(0.1)).at(0.0, 5.0),
  )
  let ball = world.add_body(@moonphys.Body::ball(0.3).at(2.0, 5.0))
  world.add_distance_joint(pivot, ball)
  world.run(240, FRAME)
  let distance = (world.body(pivot).position() - world.body(ball).position()).length()
  assert_true(approx(distance, 2.0, 0.05))
}

///|
test "distance joint swings under gravity" {
  let world = @moonphys.World::new()
  let pivot = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::circle(0.1)).at(0.0, 5.0),
  )
  let ball = world.add_body(@moonphys.Body::ball(0.3).at(2.0, 5.0))
  world.add_distance_joint(pivot, ball)
  world.run(60, FRAME)
  let position = world.body(ball).position()
  assert_true(position.y() < 4.9)
}

///|
test "revolve joint pins its anchor and lets the body swing" {
  let world = @moonphys.World::new()
  let mount = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::circle(0.1)).at(0.0, 6.0),
  )
  let bar = world.add_body(@moonphys.Body::box(1.0, 0.1).at(1.0, 5.0))
  world.add_revolve_joint_local(
    mount,
    bar,
    @moonphys.Vec2::new(0.0, -1.0),
    @moonphys.Vec2::new(-1.0, 0.0),
  )
  let mut max_swing = 0.0
  for _frame in 0..<240 {
    world.step(FRAME)
    max_swing = max_swing.max(world.body(bar).rotation().abs())
  }
  let anchor_world = world
    .body(bar)
    .local_to_world(@moonphys.Vec2::new(-1.0, 0.0))
  assert_true((anchor_world - @moonphys.Vec2::new(0.0, 5.0)).length() < 0.05)
  assert_true(max_swing > 1.0)
}

///|
test "prismatic joint slides only along its axis" {
  let world = @moonphys.World::with_gravity(0.0, 0.0)
  let rail = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.1, 0.1)).at(0.0, 10.0),
  )
  let cart = world.add_body(@moonphys.Body::box(0.3, 0.2).at(0.0, 3.0))
  ignore(
    world.add_prismatic_joint(
      rail,
      cart,
      @moonphys.Vec2::new(0.0, 3.0),
      @moonphys.Vec2::new(1.0, 0.0),
    ),
  )
  // Push sideways and up: the vertical part must be cancelled by the rail.
  world
  .body(cart)
  .apply_impulse(@moonphys.Vec2::new(0.05, 0.2), @moonphys.Vec2::new(0.0, 3.0))
  world.run(120, FRAME)
  let body = world.body(cart)
  assert_true(approx(body.position().y(), 3.0, 0.01))
  assert_true(approx(body.rotation(), 0.0, 0.01))
  assert_true(body.position().x() > 0.2)
}

///|
test "prismatic joint respects its travel limits" {
  let world = @moonphys.World::with_gravity(0.0, 0.0)
  let rail = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.1, 0.1)).at(0.0, 10.0),
  )
  let cart = world.add_body(@moonphys.Body::box(0.3, 0.2).at(0.0, 3.0))
  let slider = world.add_prismatic_joint(
    rail,
    cart,
    @moonphys.Vec2::new(0.0, 3.0),
    @moonphys.Vec2::new(1.0, 0.0),
  )
  ignore(slider.with_limits(-1.0, 1.0))
  world
  .body(cart)
  .apply_impulse(@moonphys.Vec2::new(0.3, 0.0), @moonphys.Vec2::new(0.0, 3.0))
  world.run(180, FRAME)
  let x = world.body(cart).position().x()
  assert_true(x > 0.9)
  assert_true(x < 1.05)
}

///|
test "motor joint drives the relative angular velocity" {
  let world = @moonphys.World::with_gravity(0.0, 0.0)
  let base = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.1, 0.1)).at(0.0, 10.0),
  )
  let arm = world.add_body(@moonphys.Body::box(1.0, 0.1).at(1.0, 3.0))
  world.add_revolve_joint(base, arm, @moonphys.Vec2::new(0.0, 3.0))
  ignore(world.add_motor_joint(base, arm, 2.0, 200.0))
  world.run(120, FRAME)
  assert_true(approx(world.body(arm).angular_velocity(), 2.0, 0.05))
}

///|
test "angle limit joint keeps the relative rotation bounded" {
  let world = @moonphys.World::with_gravity(0.0, 0.0)
  let base = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.1, 0.1)).at(0.0, 10.0),
  )
  let arm = world.add_body(@moonphys.Body::box(1.0, 0.1).at(1.0, 3.0))
  world.add_revolve_joint(base, arm, @moonphys.Vec2::new(0.0, 3.0))
  ignore(
    world.add_angle_limit_joint_at(
      base,
      arm,
      @moonphys.Vec2::new(0.0, 3.0),
      -0.2,
      0.2,
    ),
  )
  world.body(arm).apply_angular_impulse(0.3)
  world.run(180, FRAME)
  let rotation = world.body(arm).rotation()
  assert_true(rotation > 0.18)
  assert_true(rotation < 0.24)
}

///|
test "soft distance joint settles without oscillating" {
  let world = @moonphys.World::new()
  let pivot = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::circle(0.1)).at(0.0, 5.0),
  )
  let ball = world.add_body(@moonphys.Body::ball(0.3).at(2.0, 5.0))
  world.add_soft_distance_joint(pivot, ball, 4.0, 1.0)
  world.run(300, FRAME)
  let distance = (world.body(ball).position() - world.body(pivot).position()).length()
  assert_true(approx(distance, 2.0, 0.25))
  assert_true(world.body(ball).velocity().length() < 1.0)
}

///|
test "twenty link chain stays stable" {
  let world = @moonphys.World::new()
  let mount = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.2, 0.05)).at(-0.2, 9.0),
  )
  let links : Array[Int32] = []
  let mut previous = mount
  for i in 0..<20 {
    let link = world.add_body(
      @moonphys.Body::box(0.2, 0.05)
      .at(0.2 + i.to_double() * 0.4, 9.0)
      .with_friction(0.5),
    )
    world.add_revolve_joint_local(
      previous,
      link,
      @moonphys.Vec2::new(0.2, 0.0),
      @moonphys.Vec2::new(-0.2, 0.0),
    )
    links.push(link)
    previous = link
  }
  world.run(240, FRAME)
  for link in links {
    let body = world.body(link)
    assert_true(body.position().y() > -50.0)
    assert_true(body.position().y() < 50.0)
    assert_true(body.rotation().abs() < 50.0)
  }
}
