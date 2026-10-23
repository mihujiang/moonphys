///|
/// All pairs whose AABBs overlap, found the slow, obviously-correct way.
fn brute_force_pairs(world : World) -> Array[(Int32, Int32)] {
  let out : Array[(Int32, Int32)] = []
  let n = world.body_count()
  for i in 0..<n {
    for j in 0..<n {
      if i >= j {
        continue
      }
      let body_i = world.body(i)
      let body_j = world.body(j)
      if !body_i.is_dynamic() && !body_j.is_dynamic() {
        continue
      }
      if body_i.aabb().overlaps(body_j.aabb()) {
        out.push((i, j))
      }
    }
  }
  out
}

///|
/// Copy a pair list and order it by `(lower index, higher index)`, so two
/// generated lists can be compared with plain equality.
fn sorted_pairs(pairs : Array[(Int32, Int32)]) -> Array[(Int32, Int32)] {
  let out : Array[(Int32, Int32)] = []
  for pair in pairs {
    out.push(pair)
  }
  for i in 1..<out.length() {
    let cur = out[i]
    let mut j = i
    while j > 0 {
      let prev = out[j - 1]
      if prev.0 < cur.0 || (prev.0 == cur.0 && prev.1 < cur.1) {
        break
      }
      out[j] = prev
      j = j - 1
    }
    out[j] = cur
  }
  out
}

///|
/// The broadphase keeps its sort order between steps; the result has to be
/// exactly the set of overlapping AABBs anyway. Boxes are dropped onto the
/// ground and slide into each other, so the cached order is genuinely disturbed
/// instead of staying sorted by construction.
test "incremental broadphase matches brute force" {
  let world = World::new()
  ignore(world.add_body(Body::fixed(Shape::box(20.0, 0.5)).at(0.0, -0.5)))
  for i in 0..<30 {
    let x = (i % 6).to_double() * 0.62 - 1.6
    let y = (i / 6).to_double() * 0.7 + 0.5
    ignore(
      world.add_body(
        Body::box(0.3, 0.3)
        .at(x, y)
        .with_rotation(x * 0.2)
        .with_velocity(y * 0.05, 0.0),
      ),
    )
  }
  for _step in 0..<60 {
    world.step(1.0 / 60.0)
    // The cached order must still cover every body exactly once.
    assert_eq(world.sorted.length(), world.body_count())
    let actual = sorted_pairs(world.compute_pairs())
    let expected = brute_force_pairs(world)
    assert_eq(actual.length(), expected.length())
    for i in 0..<expected.length() {
      assert_eq(actual[i], expected[i])
    }
  }
}

///|
/// Bodies added after the world has already stepped must still show up in the
/// cached order, and bodies never removed keep their slots.
test "broadphase picks up bodies added later" {
  let world = World::new()
  ignore(world.add_body(Body::box(0.3, 0.3).at(0.0, 2.0)))
  world.run(30, 1.0 / 60.0)
  let late = world.add_body(Body::box(0.3, 0.3).at(0.1, 2.15))
  assert_eq(late, 1)
  world.run(10, 1.0 / 60.0)
  assert_eq(world.sorted.length(), world.body_count())
  let actual = sorted_pairs(world.compute_pairs())
  let expected = brute_force_pairs(world)
  assert_eq(actual.length(), expected.length())
  for i in 0..<expected.length() {
    assert_eq(actual[i], expected[i])
  }
}
