///|
/// Performance harness for `moonphys`.
///
/// Three scenes cover the shapes real games throw at a 2D engine: a tall stack
/// (many resting contacts), a dense crowd (many broadphase pairs) and a chain
/// (long constraint chains). Each scene is stepped `STEPS` times at a fixed
/// `DT`, and the wall clock is reported per step together with the body count,
/// the arbiter count and a bit-exact state hash.
///
/// Run it with:
///
/// ```bash
/// moon run bench --target js
/// moon run bench --target wasm-gc
/// ```
///
/// The hash is the determinism check: the same scene must print the same hash
/// on every target, because the engine never touches randomness or hash order.
const DT : Double = 1.0 / 60.0

///|
const STEPS : Int32 = 600

///|
fn fixed_ground(world : @moonphys.World, half_width : Double) -> Unit {
  ignore(
    world.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::box(half_width, 0.5)).at(0.0, -0.5),
    ),
  )
}

///|
/// Ten layers of ten boxes: the worst case for resting contacts.
fn stack_scene() -> @moonphys.World {
  let world = @moonphys.World::new()
  fixed_ground(world, 40.0)
  for row in 0..<10 {
    for col in 0..<10 {
      ignore(
        world.add_body(
          @moonphys.Body::box(0.4, 0.4)
          .at(col.to_double() * 0.85 - 4.0, 0.45 + row.to_double() * 0.85)
          .with_friction(0.6),
        ),
      )
    }
  }
  world
}

///|
/// Five hundred circles packed shoulder to shoulder: the broadphase stress test.
fn crowd_scene() -> @moonphys.World {
  let world = @moonphys.World::new()
  fixed_ground(world, 60.0)
  for row in 0..<20 {
    for col in 0..<25 {
      ignore(
        world.add_body(
          @moonphys.Body::ball(0.35)
          .at(col.to_double() * 0.69 - 8.3, 0.4 + row.to_double() * 0.69)
          .with_friction(0.5),
        ),
      )
    }
  }
  world
}

///|
/// Twenty links hanging off a static mount: the constraint chain stress test.
fn chain_scene() -> @moonphys.World {
  let world = @moonphys.World::new()
  fixed_ground(world, 40.0)
  let mut previous = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.2, 0.05)).at(0.0, 12.0),
  )
  for i in 0..<20 {
    let link = world.add_body(
      @moonphys.Body::box(0.25, 0.05).at(0.25 + i.to_double() * 0.5, 12.0),
    )
    world.add_revolve_joint_local(
      previous,
      link,
      @moonphys.Vec2::new(0.25, 0.0),
      @moonphys.Vec2::new(-0.25, 0.0),
    )
    previous = link
  }
  world
}

///|
fn round3(value : Double) -> Double {
  (value * 1000.0).round() / 1000.0
}

///|
fn report(name : String, world : @moonphys.World) -> Unit {
  let start = @clock.monotonic_clock_start()
  world.run(STEPS, DT)
  let micros = @clock.monotonic_clock_end(start)
  let per_step = round3(micros / 1000.0 / STEPS.to_double())
  println(
    name +
    " | bodies=" +
    world.body_count().to_string() +
    " arbiters=" +
    world.arbiter_count().to_string() +
    " ms/step=" +
    per_step.to_string() +
    " hash=" +
    world.state_hash().to_string(),
  )
}

///|
fn main {
  println(
    "moonphys bench: dt=" + DT.to_string() + " steps=" + STEPS.to_string(),
  )
  report("stack(100 boxes)", stack_scene())
  report("crowd(500 circles)", crowd_scene())
  report("chain(20 links)", chain_scene())
}
