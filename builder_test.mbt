///|
/// The builder is pure sugar: the same scene has to come out bit for bit
/// identical to the hand written one, otherwise it is hiding a behaviour change.
test "builder assembles the same world as manual calls" {
  let built = @moonphys.World::builder()
    .gravity(0.0, -3.0)
    .iterations(12)
    .ground(10.0)
    .ball(0.4, 0.0, 3.0)
    .box(0.5, 0.5, 1.0, 5.0)
    .body(@moonphys.Body::ball(0.25).at(-1.0, 6.0).with_friction(0.9))
    .build()
  let manual = @moonphys.World::with_gravity(0.0, -3.0)
  manual.set_velocity_iterations(12)
  ignore(
    manual.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::box(10.0, 0.5)).at(0.0, -0.5),
    ),
  )
  ignore(manual.add_body(@moonphys.Body::ball(0.4).at(0.0, 3.0)))
  ignore(manual.add_body(@moonphys.Body::box(0.5, 0.5).at(1.0, 5.0)))
  ignore(
    manual.add_body(@moonphys.Body::ball(0.25).at(-1.0, 6.0).with_friction(0.9)),
  )
  for _step in 0..<120 {
    built.step(FRAME)
    manual.step(FRAME)
  }
  assert_eq(built.body_count(), manual.body_count())
  assert_eq(built.state_hash(), manual.state_hash())
}

///|
test "builder reports the index of the body added last" {
  let builder = @moonphys.World::builder().ground(10.0).ball(0.3, 0.0, 2.0)
  assert_eq(builder.last_index(), 1)
  assert_eq(builder.box(0.2, 0.2, 1.0, 3.0).last_index(), 2)
}

///|
/// `snapshot_json` is what a front end or a replay recorder consumes, so the
/// payload has to stay valid JSON with one entry per body.
test "snapshot json covers every body" {
  let world = @moonphys.World::builder()
    .ground(10.0)
    .ball(0.4, 0.0, 2.0)
    .build()
  world.run(30, FRAME)
  let json = world.snapshot_json()
  assert_true(json.has_prefix("{\"time\":"))
  assert_true(json.has_suffix("]}"))
  assert_true(json.contains("\"shape\":\"polygon\",\"v\":["))
  assert_true(json.contains("\"shape\":\"circle\",\"r\":0.4"))
  assert_true(json.contains("\"asleep\":"))
  assert_eq(count_occurrences(json, "{\"i\":"), world.body_count())
  assert_eq(count_occurrences(json, "["), count_occurrences(json, "]"))
}

///|
fn count_occurrences(text : String, needle : String) -> Int32 {
  let mut count = 0
  let mut start = 0
  while true {
    let view = text.view(start_offset=start)
    let index = match view.find(needle) {
      Some(index) => index
      None => break
    }
    count = count + 1
    start = start + index + needle.length()
  }
  count
}
