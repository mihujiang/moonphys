///|
fn snapshot_scene() -> World {
  let world = World::new()
  ignore(world.add_body(Body::fixed(Shape::box(20.0, 0.5)).at(0.0, -0.5)))
  ignore(world.add_body(Body::box(0.4, 0.4).at(0.0, 3.0)))
  ignore(world.add_body(Body::box(0.4, 0.6).at(0.35, 5.0)))
  world
}

///|
fn run_frames(world : World, frames : Int) -> Unit {
  let mut i = 0
  while i < frames {
    world.step(1.0 / 60.0)
    i = i + 1
  }
}

///|
test "load_state rebuilds a world that never ran" {
  let source = snapshot_scene()
  run_frames(source, 90)
  let hash_mark = source.state_hash()
  let mark = source.save_state()
  run_frames(source, 60)
  let hash_end = source.state_hash()
  // A fresh world that never stepped must reach the very same state, impulses
  // and sleep timers included.
  let fresh = snapshot_scene()
  assert_true(fresh.load_state(mark))
  assert_eq(fresh.state_hash(), hash_mark)
  run_frames(fresh, 60)
  assert_eq(fresh.state_hash(), hash_end)
}

///|
test "load_state rejects truncated or corrupted buffers" {
  let world = snapshot_scene()
  run_frames(world, 30)
  let bytes = world.save_state()
  let truncated = Bytes::makei(bytes.length() - 1, fn(i) { bytes[i] })
  assert_true(!snapshot_scene().load_state(truncated))
  let corrupted = Bytes::makei(
    bytes.length(),
    fn(i) {
      if i == 40 {
        (bytes[i].to_int() ^ 0xFF).to_byte()
      } else {
        bytes[i]
      }
    },
  )
  assert_true(!snapshot_scene().load_state(corrupted))
}

///|
test "load_state refuses worlds with joints" {
  let world = snapshot_scene()
  world.add_distance_joint(0, 1)
  let bytes = world.save_state()
  assert_true(!world.load_state(bytes))
}

///|
test "save_state is stable for an untouched world" {
  let world = snapshot_scene()
  let bytes = world.save_state()
  assert_eq(world.save_state(), bytes)
  assert_true(world.load_state(bytes))
  assert_eq(world.save_state(), bytes)
}
