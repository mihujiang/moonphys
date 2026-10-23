///|
test "vec2 arithmetic" {
  let a = Vec2::new(3.0, 4.0)
  assert_eq(a.length(), 5.0)
  assert_eq(a.length_sq(), 25.0)
  assert_eq(a.dot(Vec2::new(1.0, 0.0)), 3.0)
  assert_eq(Vec2::new(1.0, 0.0).cross(Vec2::new(0.0, 1.0)), 1.0)
  assert_eq((a + a - a).x(), 3.0)
  assert_eq((-a).y(), -4.0)
}

///|
test "vec2 normalize is unit length" {
  let v = Vec2::new(-7.0, 2.5).normalize()
  assert_true(approx(v.length(), 1.0, 1.0e-12))
  assert_true(approx(Vec2::zero().normalize().length(), 0.0, 1.0e-12))
}

///|
test "vec2 rotate ninety degrees" {
  let v = Vec2::new(1.0, 0.0).rotate(K_PI / 2.0)
  assert_true(approx(v.x(), 0.0, 1.0e-12))
  assert_true(approx(v.y(), 1.0, 1.0e-12))
  let back = v.rotate_inv(K_PI / 2.0)
  assert_true(approx(back.x(), 1.0, 1.0e-12))
}

///|
test "cross scalar matches cross product" {
  let w = 2.0
  let r = Vec2::new(3.0, -1.0)
  let v = cross_scalar(w, r)
  assert_true(approx(v.x(), 2.0, 1.0e-12))
  assert_true(approx(v.y(), 6.0, 1.0e-12))
}

///|
test "box mass properties" {
  let data = Shape::box(1.0, 2.0).mass_data(1.0)
  assert_true(approx(data.mass(), 8.0, 1.0e-9))
  assert_true(approx(data.inertia(), 8.0 * (4.0 + 16.0) / 12.0, 1.0e-9))
  assert_true(approx(data.center().length(), 0.0, 1.0e-12))
}

///|
test "circle mass properties" {
  let data = Shape::circle(2.0).mass_data(1.0)
  assert_true(approx(data.mass(), K_PI * 4.0, 1.0e-9))
  assert_true(approx(data.inertia(), 0.5 * K_PI * 4.0 * 4.0, 1.0e-9))
}

///|
test "box aabb" {
  let box = Shape::box(1.0, 2.0).aabb(Transform::new(Vec2::new(3.0, 4.0), 0.0))
  assert_true(approx(box.min().x(), 2.0, 1.0e-12))
  assert_true(approx(box.max().y(), 6.0, 1.0e-12))
}
