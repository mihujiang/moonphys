///|
const FRAME : Double = 1.0 / 60.0

///|
fn ground(world : @moonphys.World) -> Int32 {
  world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(20.0, 0.5)).at(0.0, -0.5),
  )
}

///|
fn stack_scene() -> @moonphys.World {
  let world = @moonphys.World::new()
  ignore(ground(world))
  for i in 0..<6 {
    ignore(
      world.add_body(
        @moonphys.Body::box(0.5, 0.5)
        .at(0.0, 0.5 + i.to_double())
        .with_friction(0.6)
        .with_restitution(0.0),
      ),
    )
  }
  world
}

///|
test "free fall matches analytic solution" {
  let world = @moonphys.World::with_gravity(0.0, -10.0)
  let ball = world.add_body(@moonphys.Body::ball(0.1).at(0.0, 0.0))
  world.run(60, FRAME)
  let expected_velocity = -10.0
  let position = world.body(ball)
  assert_true(approx(position.velocity().y(), expected_velocity, 0.1))
  assert_true(approx(position.position().y(), -5.0, 0.2))
}

///|
test "ball comes to rest on the ground" {
  let world = @moonphys.World::new()
  ignore(ground(world))
  let ball = world.add_body(
    @moonphys.Body::ball(0.5).at(0.0, 3.0).with_restitution(0.0),
  )
  world.run(300, FRAME)
  let body = world.body(ball)
  assert_true(approx(body.position().y(), 0.5, 0.05))
  assert_true(body.velocity().length() < 0.1)
}

///|
test "box comes to rest on the ground without sinking" {
  let world = @moonphys.World::new()
  ignore(ground(world))
  let bin = world.add_body(
    @moonphys.Body::box(0.5, 0.5)
    .at(0.0, 4.0)
    .with_restitution(0.0)
    .with_friction(0.5),
  )
  world.run(300, FRAME)
  let body = world.body(bin)
  assert_true(approx(body.position().y(), 0.5, 0.05))
  assert_true(body.rotation().abs() < 0.05)
}

///|
test "stack of six boxes stays put" {
  let world = @moonphys.World::new()
  ignore(ground(world))
  let ids : Array[Int32] = []
  for i in 0..<6 {
    ids.push(
      world.add_body(
        @moonphys.Body::box(0.5, 0.5)
        .at(0.0, 0.5 + i.to_double())
        .with_friction(0.6)
        .with_restitution(0.0),
      ),
    )
  }
  world.run(300, FRAME)
  for i in 0..<6 {
    let body = world.body(ids[i])
    let expected = 0.5 + i.to_double()
    assert_true(approx(body.position().x(), 0.0, 0.05))
    assert_true(approx(body.position().y(), expected, 0.15))
    assert_true(body.rotation().abs() < 0.15)
  }
}

///|
test "restitution removes speed" {
  let world = @moonphys.World::with_gravity(0.0, 0.0)
  ignore(
    world.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::box(10.0, 0.5)).at(0.0, -0.5),
    ),
  )
  let ball = world.add_body(
    @moonphys.Body::ball(0.5)
    .at(0.0, 1.0)
    .with_velocity(0.0, -5.0)
    .with_restitution(0.5)
    .with_friction(0.0),
  )
  world.run(20, FRAME)
  let speed = world.body(ball).velocity().length()
  assert_true(speed < 5.0)
  assert_true(speed > 0.5)
}

///|
test "identical scenes replay bit for bit" {
  let a = stack_scene()
  let b = stack_scene()
  a.run(300, FRAME)
  b.run(300, FRAME)
  let sa = a.snapshot()
  let sb = b.snapshot()
  assert_eq(sa.length(), sb.length())
  for i in 0..<sa.length() {
    assert_true(sa[i].position() == sb[i].position())
    assert_true(sa[i].rotation() == sb[i].rotation())
  }
}

///|
/// The state digest folds every position, rotation and velocity in as raw bits,
/// so this assertion is a canary: it fails if a single ulp changes anywhere in
/// the solver, the broadphase order or the integration.
///
/// It is also the cross-target determinism check. `moon test --target js` and
/// `moon test --target wasm-gc` run the very same assertion, so a float path
/// that differs between backends shows up as a failure instead of a silent
/// drift. Regenerate the constant only when the physics is meant to change.
test "state digest is pinned and repeatable" {
  let world = stack_scene()
  world.run(300, FRAME)
  let digest = world.state_hash()
  let replay = stack_scene()
  replay.run(300, FRAME)
  assert_eq(replay.state_hash(), digest)
  assert_eq(digest.to_string(), "5172919354809653977")
}

///|
/// A resting pile has to fall asleep as a whole, and something landing on it has
/// to wake it again. Waking on "the neighbour is awake" alone deadlocks this: the
/// first body to sleep is woken by its still-awake neighbour every step, so the
/// pile never settles at all.
test "a stack falls asleep and wakes on impact" {
  let world = @moonphys.World::new()
  ignore(
    world.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::box(10.0, 0.5)).at(0.0, -0.5),
    ),
  )
  for i in 0..<3 {
    ignore(
      world.add_body(
        @moonphys.Body::box(0.45, 0.45).at(0.0, 0.45 + i.to_double() * 0.9),
      ),
    )
  }
  world.run(300, FRAME)
  let mut asleep = 0
  for i in 1..<world.body_count() {
    if !world.body(i).awake() {
      asleep = asleep + 1
    }
  }
  assert_eq(asleep, 3)
  // A sleeping body must not drift.
  let resting = world.body(2).position().y()
  world.run(60, FRAME)
  assert_eq(world.body(2).position().y(), resting)
  // Dropping a ball on the pile wakes every body it touches; the pile is
  // allowed to fall asleep again afterwards, so sample the peak.
  ignore(world.add_body(@moonphys.Body::ball(0.3).at(0.0, 6.0)))
  let mut peak = 0
  for _step in 0..<120 {
    world.step(FRAME)
    let mut awake = 0
    for i in 1..<world.body_count() {
      if world.body(i).awake() {
        awake = awake + 1
      }
    }
    if awake > peak {
      peak = awake
    }
  }
  assert_true(peak >= 3)
}

///|
test "sleeping bodies are reused until something moves" {
  let world = @moonphys.World::new()
  ignore(ground(world))
  let ball = world.add_body(
    @moonphys.Body::ball(0.5).at(0.0, 2.0).with_restitution(0.0),
  )
  world.run(300, FRAME)
  assert_true(world.body(ball).awake() == false)
  assert_true(world.contact_count() > 0)
}

///|
test "broadphase skips distant bodies" {
  let world = @moonphys.World::with_gravity(0.0, 0.0)
  ignore(world.add_body(@moonphys.Body::ball(0.5).at(-100.0, 0.0)))
  ignore(world.add_body(@moonphys.Body::ball(0.5).at(100.0, 0.0)))
  world.run(10, FRAME)
  assert_eq(world.arbiter_count(), 0)
}

///|
test "impulse changes velocity" {
  let world = @moonphys.World::with_gravity(0.0, 0.0)
  let ball = world.add_body(@moonphys.Body::ball(0.5).at(0.0, 0.0))
  world
  .body(ball)
  .apply_impulse(@moonphys.Vec2::new(2.0, 0.0), @moonphys.Vec2::zero())
  let mass = world.body(ball).mass()
  assert_true(approx(world.body(ball).velocity().x(), 2.0 / mass, 1.0e-9))
}
