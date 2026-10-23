///|
/// Skin width; contacts shallower than this are considered touching.
pub const LINEAR_SLOP : Double = 0.005

///|
/// One contact point of a manifold, in world space.
///
/// `normal` always points from the first shape toward the second one, and
/// `separation` is negative when the shapes overlap.
pub struct ContactPoint {
  position : Vec2
  normal : Vec2
  separation : Double
  feature_id : Int32
}

///|
pub fn ContactPoint::position(self : ContactPoint) -> Vec2 {
  self.position
}

///|
pub fn ContactPoint::normal(self : ContactPoint) -> Vec2 {
  self.normal
}

///|
pub fn ContactPoint::separation(self : ContactPoint) -> Double {
  self.separation
}

///|
pub fn ContactPoint::feature_id(self : ContactPoint) -> Int32 {
  self.feature_id
}

///|
/// A vertex produced by polygon clipping, carrying a stable feature id so that
/// impulses can be carried over between frames (warm starting).
priv struct ClipVertex {
  v : Vec2
  id : Int32
}

///|
fn world_vertices(vertices : Array[Vec2], xf : Transform) -> Array[Vec2] {
  let out : Array[Vec2] = []
  for v in vertices {
    out.push(xf.apply(v))
  }
  out
}

///|
fn polygon_edge_normal(vertices : Array[Vec2], i : Int32) -> Vec2 {
  let n = vertices.length()
  let edge = vertices[(i + 1) % n] - vertices[i]
  Vec2::new(edge.y, -edge.x).normalize()
}

///|
/// Collide two shapes and return 0..2 world-space contact points.
pub fn collide(
  shape_a : Shape,
  xf_a : Transform,
  shape_b : Shape,
  xf_b : Transform,
) -> Array[ContactPoint] {
  match (shape_a, shape_b) {
    (Shape::Circle(ra), Shape::Circle(rb)) =>
      collide_circles(ra, xf_a.position, rb, xf_b.position)
    (Shape::Circle(ra), Shape::Polygon(vb)) => {
      let points = collide_polygon_circle(vb, xf_b, ra, xf_a.position)
      flip_contacts(points)
    }
    (Shape::Polygon(va), Shape::Circle(rb)) =>
      collide_polygon_circle(va, xf_a, rb, xf_b.position)
    (Shape::Polygon(va), Shape::Polygon(vb)) =>
      collide_polygons(va, xf_a, vb, xf_b)
  }
}

///|
fn flip_contacts(points : Array[ContactPoint]) -> Array[ContactPoint] {
  let out : Array[ContactPoint] = []
  for p in points {
    out.push({
      position: p.position,
      normal: -p.normal,
      separation: p.separation,
      feature_id: p.feature_id,
    })
  }
  out
}

///|
fn collide_circles(
  ra : Double,
  ca : Vec2,
  rb : Double,
  cb : Vec2,
) -> Array[ContactPoint] {
  let out : Array[ContactPoint] = []
  let d = cb - ca
  let dist = d.length()
  let radius = ra + rb
  if dist > radius {
    return out
  }
  let normal = d.normalize_or(Vec2::new(1.0, 0.0))
  out.push({
    position: ca + normal.scale(ra),
    normal,
    separation: dist - radius,
    feature_id: 0,
  })
  out
}

///|
/// Circle (`radius` at `center`) against a convex polygon.
/// Normals point from the polygon toward the circle.
fn collide_polygon_circle(
  vertices : Array[Vec2],
  xf : Transform,
  radius : Double,
  center : Vec2,
) -> Array[ContactPoint] {
  let out : Array[ContactPoint] = []
  let n = vertices.length()
  let mut normal_index = 0
  let mut separation = -1.0e30
  for i in 0..<n {
    let v = xf.apply(vertices[i])
    let normal = polygon_edge_normal(vertices, i).rotate(xf.rotation)
    let s = normal.dot(center - v)
    if s > radius {
      return out
    }
    if s > separation {
      separation = s
      normal_index = i
    }
  }
  let v1 = xf.apply(vertices[normal_index])
  let v2 = xf.apply(vertices[(normal_index + 1) % n])
  let face_normal = polygon_edge_normal(vertices, normal_index).rotate(
    xf.rotation,
  )
  if separation < K_EPSILON {
    // Circle center is inside the polygon.
    out.push({
      position: center - face_normal.scale(radius),
      normal: face_normal,
      separation: separation - radius,
      feature_id: normal_index,
    })
    return out
  }
  let u1 = (center - v1).dot(v2 - v1)
  let u2 = (center - v2).dot(v1 - v2)
  if u1 <= 0.0 {
    let d = center - v1
    let dist = d.length()
    if dist > radius {
      return out
    }
    out.push({
      position: v1,
      normal: d.normalize_or(face_normal),
      separation: dist - radius,
      feature_id: normal_index,
    })
  } else if u2 <= 0.0 {
    let d = center - v2
    let dist = d.length()
    if dist > radius {
      return out
    }
    out.push({
      position: v2,
      normal: d.normalize_or(face_normal),
      separation: dist - radius,
      feature_id: (normal_index + 1) % n,
    })
  } else {
    out.push({
      position: center - face_normal.scale(radius),
      normal: face_normal,
      separation: separation - radius,
      feature_id: normal_index,
    })
  }
  out
}

///|
/// Deepest separation of `vb` from the edge normals of `va`.
fn find_max_separation(
  va : Array[Vec2],
  xf_a : Transform,
  vb : Array[Vec2],
  xf_b : Transform,
) -> (Int32, Double) {
  let world_b = world_vertices(vb, xf_b)
  let mut best_index = 0
  let mut max_separation = -1.0e30
  let n = va.length()
  for i in 0..<n {
    let normal = polygon_edge_normal(va, i).rotate(xf_a.rotation)
    let v1 = xf_a.apply(va[i])
    let mut si = 1.0e30
    for v2 in world_b {
      let s = normal.dot(v2 - v1)
      if s < si {
        si = s
      }
    }
    if si > max_separation {
      max_separation = si
      best_index = i
    }
  }
  (best_index, max_separation)
}

///|
/// Build the incident edge of `vb` for a reference normal given in world space.
fn find_incident_edge(
  vb : Array[Vec2],
  xf_b : Transform,
  reference_normal : Vec2,
  reference_edge : Int32,
) -> Array[ClipVertex] {
  let n = vb.length()
  let mut index = 0
  let mut min_dot = 1.0e30
  for i in 0..<n {
    let normal = polygon_edge_normal(vb, i).rotate(xf_b.rotation)
    let dot = reference_normal.dot(normal)
    if dot < min_dot {
      min_dot = dot
      index = i
    }
  }
  let v1 = xf_b.apply(vb[index])
  let v2 = xf_b.apply(vb[(index + 1) % n])
  [
    { v: v1, id: (reference_edge << 8) | (index % 256), },
    { v: v2, id: (reference_edge << 8) | ((index + 1) % n % 256), },
  ]
}

///|
fn clip_segment_to_line(
  v_in : Array[ClipVertex],
  normal : Vec2,
  offset : Double,
  id_base : Int32,
) -> Array[ClipVertex] {
  let out : Array[ClipVertex] = []
  let d0 = normal.dot(v_in[0].v) - offset
  let d1 = normal.dot(v_in[1].v) - offset
  if d0 <= 0.0 {
    out.push(v_in[0])
  }
  if d1 <= 0.0 {
    out.push(v_in[1])
  }
  if d0 * d1 < 0.0 {
    let t = d0 / (d0 - d1)
    out.push({
      v: v_in[0].v + (v_in[1].v - v_in[0].v).scale(t),
      id: (id_base << 16) | 0xFFFF,
    })
  }
  out
}

///|
/// Collide two convex polygons with the separating-axis + clipping algorithm.
fn collide_polygons(
  va : Array[Vec2],
  xf_a : Transform,
  vb : Array[Vec2],
  xf_b : Transform,
) -> Array[ContactPoint] {
  let out : Array[ContactPoint] = []
  let (edge_a, separation_a) = find_max_separation(va, xf_a, vb, xf_b)
  if separation_a > 0.0 {
    return out
  }
  let (edge_b, separation_b) = find_max_separation(vb, xf_b, va, xf_a)
  if separation_b > 0.0 {
    return out
  }
  let relative_tol = 0.98
  let absolute_tol = 0.001
  let flip = separation_b > relative_tol * separation_a + absolute_tol
  let (ref_vs, ref_xf, ref_edge, inc_vs, inc_xf) = if flip {
    (vb, xf_b, edge_b, va, xf_a)
  } else {
    (va, xf_a, edge_a, vb, xf_b)
  }
  let count = ref_vs.length()
  let iv1 = ref_edge
  let iv2 = (ref_edge + 1) % count
  let v11 = ref_xf.apply(ref_vs[iv1])
  let v12 = ref_xf.apply(ref_vs[iv2])
  let tangent = (v12 - v11).normalize_or(Vec2::new(1.0, 0.0))
  let normal = Vec2::new(tangent.y, -tangent.x)
  let front_offset = normal.dot(v11)
  let side_offset1 = -tangent.dot(v11)
  let side_offset2 = tangent.dot(v12)
  let incident = find_incident_edge(inc_vs, inc_xf, normal, ref_edge)
  let clipped1 = clip_segment_to_line(incident, -tangent, side_offset1, 0)
  if clipped1.length() < 2 {
    return out
  }
  let clipped2 = clip_segment_to_line(clipped1, tangent, side_offset2, 1)
  if clipped2.length() < 2 {
    return out
  }
  let world_normal = if flip { -normal } else { normal }
  for cp in clipped2 {
    let separation = normal.dot(cp.v) - front_offset
    if separation <= 0.0 {
      out.push({
        position: cp.v,
        normal: world_normal,
        separation,
        feature_id: cp.id,
      })
    }
  }
  out
}
