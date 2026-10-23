// Whitebox tests: exercise the internal helpers directly (clipping, mass
// properties, degeneracy handling), which public API tests only reach
// indirectly.

///|
fn approx_wb(a : Double, b : Double, tol : Double) -> Bool {
  (a - b).abs() < tol
}

///|
test "signed_area follows the winding order" {
  let ccw : Array[Vec2] = [
    Vec2::new(0.0, 0.0),
    Vec2::new(1.0, 0.0),
    Vec2::new(1.0, 1.0),
    Vec2::new(0.0, 1.0),
  ]
  let cw : Array[Vec2] = [
    Vec2::new(0.0, 0.0),
    Vec2::new(0.0, 1.0),
    Vec2::new(1.0, 1.0),
    Vec2::new(1.0, 0.0),
  ]
  assert_true(approx_wb(signed_area(ccw), 1.0, 1.0e-9))
  assert_true(approx_wb(signed_area(cw), -1.0, 1.0e-9))
}

///|
test "Shape::convex normalizes clockwise input" {
  let cw : Array[Vec2] = [
    Vec2::new(0.0, 0.0),
    Vec2::new(0.0, 2.0),
    Vec2::new(2.0, 2.0),
    Vec2::new(2.0, 0.0),
  ]
  let shape = Shape::convex(cw)
  let vs = shape.vertices()
  assert_true(signed_area(vs) > 0.0)
  assert_true(approx_wb(shape.area(), 4.0, 1.0e-9))
}

///|
test "polygon mass data matches the analytic square" {
  let square : Array[Vec2] = [
    Vec2::new(-1.0, -1.0),
    Vec2::new(1.0, -1.0),
    Vec2::new(1.0, 1.0),
    Vec2::new(-1.0, 1.0),
  ]
  let data = polygon_mass_data(square, 1.0)
  assert_true(approx_wb(data.mass(), 4.0, 1.0e-9))
  assert_true(approx_wb(data.center().x(), 0.0, 1.0e-9))
  assert_true(approx_wb(data.center().y(), 0.0, 1.0e-9))
  // I = m * (w^2 + h^2) / 12 for a solid square.
  assert_true(approx_wb(data.inertia(), 4.0 * 8.0 / 12.0, 1.0e-9))
}

///|
test "mass data of an offset polygon is taken about its centroid" {
  // Triangle with vertices (0,0), (2,0), (0,2).
  let triangle : Array[Vec2] = [
    Vec2::new(0.0, 0.0),
    Vec2::new(2.0, 0.0),
    Vec2::new(0.0, 2.0),
  ]
  let data = polygon_mass_data(triangle, 2.0)
  assert_true(approx_wb(data.mass(), 4.0, 1.0e-9))
  assert_true(approx_wb(data.center().x(), 2.0 / 3.0, 1.0e-9))
  assert_true(approx_wb(data.center().y(), 2.0 / 3.0, 1.0e-9))
  // I about (0,0) is m * (a^2 + b^2) / 6 = 4 * 8 / 6, then shifted by
  // m * |c|^2 = 4 * 8 / 9 with the parallel axis theorem.
  let about_origin = 4.0 * 8.0 / 6.0
  let expected = about_origin - 4.0 * (8.0 / 9.0)
  assert_true(approx_wb(data.inertia(), expected, 1.0e-9))
}

///|
test "separating axis reports the deepest penetration" {
  let unit : Array[Vec2] = [
    Vec2::new(-0.5, -0.5),
    Vec2::new(0.5, -0.5),
    Vec2::new(0.5, 0.5),
    Vec2::new(-0.5, 0.5),
  ]
  let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0)
  let xf_b = Transform::new(Vec2::new(0.5, 0.0), 0.0)
  let (index, separation) = find_max_separation(unit, xf_a, unit, xf_b)
  assert_true(separation < 0.0)
  assert_true(approx_wb(separation, -0.5, 1.0e-9))
  // Either the +x or the -x face of A ties for deepest; both are valid.
  assert_true(index == 1 || index == 3)
}

///|
test "clip_segment_to_line keeps the inside part and the crossing point" {
  let segment : Array[ClipVertex] = [
    { v: Vec2::new(-1.0, 1.0), id: 7 },
    { v: Vec2::new(1.0, -1.0), id: 9 },
  ]
  let clipped = clip_segment_to_line(segment, Vec2::new(0.0, 1.0), 0.0, 3)
  assert_eq(clipped.length(), 2)
  // The retained vertex is the one on the negative side of y = 0.
  assert_true(approx_wb(clipped[0].v.y(), -1.0, 1.0e-9))
  // The second vertex is the crossing point, exactly on the clipping line.
  assert_true(approx_wb(clipped[1].v.x(), 0.0, 1.0e-9))
  assert_true(approx_wb(clipped[1].v.y(), 0.0, 1.0e-9))
}

///|
test "a segment fully inside the line is not clipped" {
  let segment : Array[ClipVertex] = [
    { v: Vec2::new(-1.0, -1.0), id: 1 },
    { v: Vec2::new(1.0, -2.0), id: 2 },
  ]
  let clipped = clip_segment_to_line(segment, Vec2::new(0.0, 1.0), 0.0, 0)
  assert_eq(clipped.length(), 2)
}

///|
test "incident edge picks the face most anti-parallel to the reference" {
  let unit : Array[Vec2] = [
    Vec2::new(-0.5, -0.5),
    Vec2::new(0.5, -0.5),
    Vec2::new(0.5, 0.5),
    Vec2::new(-0.5, 0.5),
  ]
  let xf = Transform::new(Vec2::new(0.0, 0.0), 0.0)
  let edge = find_incident_edge(unit, xf, Vec2::new(1.0, 0.0), 2)
  assert_eq(edge.length(), 2)
  // The incident edge is the left face of the box, vertical at x = -0.5.
  assert_true(approx_wb(edge[0].v.x(), -0.5, 1.0e-9))
  assert_true(approx_wb(edge[1].v.x(), -0.5, 1.0e-9))
  assert_true(edge[0].id != edge[1].id)
}
