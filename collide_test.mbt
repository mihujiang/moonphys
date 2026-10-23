///|
fn xf_at(x : Double, y : Double) -> @moonphys.Transform {
  @moonphys.Transform::new(@moonphys.Vec2::new(x, y), 0.0)
}

///|
test "circles overlap" {
  let points = @moonphys.collide(
    @moonphys.Shape::circle(1.0),
    xf_at(0.0, 0.0),
    @moonphys.Shape::circle(1.0),
    xf_at(1.0, 0.0),
  )
  assert_eq(points.length(), 1)
  assert_true(approx(points[0].separation(), -1.0, 1.0e-9))
  assert_true(approx(points[0].normal().x(), 1.0, 1.0e-9))
  assert_true(approx(points[0].position().x(), 1.0, 1.0e-9))
}

///|
test "circles apart" {
  let points = @moonphys.collide(
    @moonphys.Shape::circle(1.0),
    xf_at(0.0, 0.0),
    @moonphys.Shape::circle(1.0),
    xf_at(3.0, 0.0),
  )
  assert_eq(points.length(), 0)
}

///|
test "stacked boxes clamp to two contacts" {
  let points = @moonphys.collide(
    @moonphys.Shape::box(1.0, 1.0),
    xf_at(0.0, 0.0),
    @moonphys.Shape::box(1.0, 1.0),
    xf_at(0.0, 1.5),
  )
  assert_eq(points.length(), 2)
  for point in points {
    assert_true(approx(point.separation(), -0.5, 1.0e-6))
    assert_true(approx(point.normal().y(), 1.0, 1.0e-6))
  }
}

///|
test "boxes apart" {
  let points = @moonphys.collide(
    @moonphys.Shape::box(1.0, 1.0),
    xf_at(0.0, 0.0),
    @moonphys.Shape::box(1.0, 1.0),
    xf_at(0.0, 2.01),
  )
  assert_eq(points.length(), 0)
}

///|
test "circle resting on box" {
  let points = @moonphys.collide(
    @moonphys.Shape::box(2.0, 1.0),
    xf_at(0.0, 0.0),
    @moonphys.Shape::circle(0.5),
    xf_at(0.0, 1.4),
  )
  assert_eq(points.length(), 1)
  assert_true(approx(points[0].separation(), -0.1, 1.0e-9))
  assert_true(approx(points[0].normal().y(), 1.0, 1.0e-9))
}

///|
test "rotated box collides" {
  let rotated = @moonphys.Transform::new(
    @moonphys.Vec2::new(0.0, 1.3),
    @moonphys.K_PI / 4.0,
  )
  let points = @moonphys.collide(
    @moonphys.Shape::box(1.0, 1.0),
    xf_at(0.0, 0.0),
    @moonphys.Shape::box(1.0, 1.0),
    rotated,
  )
  assert_true(points.length() > 0)
  assert_true(points[0].normal().y() > 0.9)
}

///|
test "contact normal is flipped for circle first" {
  let points = @moonphys.collide(
    @moonphys.Shape::circle(0.5),
    xf_at(0.0, 1.4),
    @moonphys.Shape::box(2.0, 1.0),
    xf_at(0.0, 0.0),
  )
  assert_eq(points.length(), 1)
  assert_true(approx(points[0].normal().y(), -1.0, 1.0e-9))
}
