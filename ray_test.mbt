///|
fn ray_ground_world() -> @moonphys.World {
  let world = @moonphys.World::new()
  ignore(
    world.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::box(10.0, 0.1)).at(0.0, -0.1),
    ),
  )
  world
}

///|
test "ray hits the ground plane" {
  let world = ray_ground_world()
  let hit = world.ray_cast(
    @moonphys.Vec2::new(0.0, 5.0),
    @moonphys.Vec2::new(0.0, -1.0),
    100.0,
  )
  match hit {
    Some(hit) => {
      assert_true(approx(hit.distance(), 5.0, 1.0e-9))
      assert_eq(hit.body_index(), 0)
      assert_true(approx(hit.point().y(), 0.0, 1.0e-9))
      assert_true(approx(hit.normal().y(), 1.0, 1.0e-9))
    }
    None => assert_true(false)
  }
}

///|
test "ray misses when pointing away or out of range" {
  let world = ray_ground_world()
  // Pointing up: the ground is behind the origin.
  assert_true(
    world.ray_cast(
      @moonphys.Vec2::new(0.0, 5.0),
      @moonphys.Vec2::new(0.0, 1.0),
      100.0,
    )
    is None,
  )
  // Correct direction but the range stops before the ground.
  assert_true(
    world.ray_cast(
      @moonphys.Vec2::new(0.0, 5.0),
      @moonphys.Vec2::new(0.0, -1.0),
      3.0,
    )
    is None,
  )
}

///|
test "ray returns the closest @moonphys.Body" {
  let world = ray_ground_world()
  ignore(world.add_body(@moonphys.Body::ball(0.5).at(0.0, 3.0)))
  ignore(world.add_body(@moonphys.Body::ball(0.5).at(0.0, 1.0)))
  let hit = world.ray_cast(
    @moonphys.Vec2::new(0.0, 8.0),
    @moonphys.Vec2::new(0.0, -1.0),
    100.0,
  )
  match hit {
    Some(hit) => {
      // The upper ball (index 1) is hit first, at y = 3.5.
      assert_eq(hit.body_index(), 1)
      assert_true(approx(hit.distance(), 4.5, 1.0e-9))
      // And not the ground behind it.
      assert_true(hit.distance() < 8.0)
    }
    None => assert_true(false)
  }
}

///|
test "ray against a circle reports the entry point and normal" {
  let world = @moonphys.World::new()
  ignore(world.add_body(@moonphys.Body::ball(1.0).at(0.0, 0.0)))
  let hit = world.ray_cast(
    @moonphys.Vec2::new(-5.0, 0.0),
    @moonphys.Vec2::new(1.0, 0.0),
    100.0,
  )
  match hit {
    Some(hit) => {
      assert_true(approx(hit.distance(), 4.0, 1.0e-9))
      assert_true(approx(hit.normal().x(), -1.0, 1.0e-9))
    }
    None => assert_true(false)
  }
}

///|
test "ray starting inside a @moonphys.Shape misses" {
  let world = @moonphys.World::new()
  ignore(world.add_body(@moonphys.Body::ball(1.0).at(0.0, 0.0)))
  assert_true(
    world.ray_cast(
      @moonphys.Vec2::new(0.0, 0.0),
      @moonphys.Vec2::new(1.0, 0.0),
      100.0,
    )
    is None,
  )
}

///|
test "query_aabb filters by overlap" {
  let world = @moonphys.World::new()
  ignore(world.add_body(@moonphys.Body::box(0.5, 0.5).at(0.0, 0.0)))
  ignore(world.add_body(@moonphys.Body::box(0.5, 0.5).at(5.0, 0.0)))
  ignore(world.add_body(@moonphys.Body::box(0.5, 0.5).at(0.0, 5.0)))
  let region = @moonphys.Aabb::new(
    @moonphys.Vec2::new(-1.0, -1.0),
    @moonphys.Vec2::new(1.0, 1.0),
  )
  let found = world.query_aabb(region)
  assert_eq(found.length(), 1)
  assert_eq(found[0], 0)
}

///|
test "query_point picks the @moonphys.Body under the cursor" {
  let world = @moonphys.World::new()
  ignore(world.add_body(@moonphys.Body::box(0.5, 0.5).at(0.0, 0.0)))
  ignore(world.add_body(@moonphys.Body::box(0.5, 0.5).at(3.0, 0.0)))
  let found = world.query_point(@moonphys.Vec2::new(3.1, 0.1))
  assert_eq(found.length(), 1)
  assert_eq(found[0], 1)
}

///|
test "box rests on a high friction ramp" {
  let angle = 0.3490658503988659 // 20 degrees
  let world = @moonphys.World::new()
  let normal = @moonphys.Vec2::new(0.0, 1.0).rotate(angle)
  ignore(
    world.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::box(8.0, 0.2))
      .at(0.0, 0.0)
      .with_rotation(angle),
    ),
  )
  let start = normal.scale(0.2 + 0.15)
  let box_index = world.add_body(
    @moonphys.Body::box(0.4, 0.15)
    .at(start.x(), start.y())
    .with_rotation(angle)
    .with_friction(0.9),
  )
  // Friction is combined as sqrt(fa * fb), so the ramp default of 0.4 still
  // gives mu = 0.6 against the box, comfortably above tan(20 degrees).
  for _ in 0..<300 {
    world.step(FRAME)
  }
  let moved = (world.body(box_index).position() - start).length()
  assert_true(moved < 0.05)
}

///|
test "box slides down a low friction ramp" {
  let angle = 0.5235987755982988 // 30 degrees
  let world = @moonphys.World::new()
  let normal = @moonphys.Vec2::new(0.0, 1.0).rotate(angle)
  // cos/sin of 30 degrees, the downslope direction of a counter-clockwise
  // rotated ramp.
  let down = @moonphys.Vec2::new(-0.8660254037844387, -0.5)
  ignore(
    world.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::box(8.0, 0.2))
      .at(0.0, 0.0)
      .with_rotation(angle),
    ),
  )
  let start = normal.scale(0.2 + 0.15)
  let box_index = world.add_body(
    @moonphys.Body::box(0.4, 0.15)
    .at(start.x(), start.y())
    .with_rotation(angle)
    .with_friction(0.02),
  )

  for _ in 0..<60 {
    world.step(FRAME)
  }
  let body = world.body(box_index)
  let speed_along_ramp = body.velocity().dot(down)
  // Analytic: a = g (sin 30 - mu cos 30) ~= 4.7 m/s^2 after one second.
  assert_true(speed_along_ramp > 2.0)
  // It slides along the ramp, not into it.
  assert_true(body.velocity().dot(normal).abs() < 0.2)
}

///|
test "total momentum is conserved in a head-on collision" {
  let world = @moonphys.World::new()
  world.set_gravity(@moonphys.Vec2::zero())
  let a = @moonphys.Body::ball(0.5).at(-2.0, 0.0).with_velocity(1.0, 0.0)
  let b = @moonphys.Body::ball(0.5).at(2.0, 0.0).with_velocity(-1.0, 0.0)
  let mass_a = a.mass()
  let mass_b = b.mass()
  let index_a = world.add_body(a)
  let index_b = world.add_body(b)
  for _ in 0..<120 {
    world.step(FRAME)
  }
  let momentum = world.body(index_a).velocity().scale(mass_a) +
    world.body(index_b).velocity().scale(mass_b)
  assert_true(momentum.length() < 1.0e-9)
}

///|
test "deep overlap does not launch bodies" {
  let world = @moonphys.World::new()
  let index_a = world.add_body(@moonphys.Body::box(0.5, 0.5).at(0.0, 3.0))
  let index_b = world.add_body(@moonphys.Body::box(0.5, 0.5).at(0.1, 3.0))
  for _ in 0..<60 {
    world.step(FRAME)
  }
  assert_true(world.body(index_a).velocity().length() < 20.0)
  assert_true(world.body(index_b).velocity().length() < 20.0)
}

///|
test "duplicate and collinear vertices are cleaned" {
  let shape = @moonphys.Shape::convex([
    @moonphys.Vec2::new(0.0, 0.0),
    @moonphys.Vec2::new(1.0, 0.0),
    @moonphys.Vec2::new(1.0, 0.0), // duplicate
    @moonphys.Vec2::new(2.0, 0.0), // collinear with its neighbours
    @moonphys.Vec2::new(2.0, 1.0),
    @moonphys.Vec2::new(0.0, 1.0),
  ])
  assert_eq(shape.vertices().length(), 4)
  assert_true(approx(shape.area(), 2.0, 1.0e-9))
  let data = shape.mass_data(1.0)
  assert_true(approx(data.mass(), 2.0, 1.0e-9))
}
