///|
/// Result of a ray cast against the world.
pub struct RayHit {
  body_index : Int32
  point : Vec2
  normal : Vec2
  distance : Double
}

///|
/// Index of the body that was hit.
pub fn RayHit::body_index(self : RayHit) -> Int32 {
  self.body_index
}

///|
/// World-space point where the ray entered the shape.
pub fn RayHit::point(self : RayHit) -> Vec2 {
  self.point
}

///|
/// Outward surface normal at the hit point.
pub fn RayHit::normal(self : RayHit) -> Vec2 {
  self.normal
}

///|
/// Distance from the ray origin to the hit point.
pub fn RayHit::distance(self : RayHit) -> Double {
  self.distance
}

///|
/// Cast a ray against one shape.
///
/// `direction` must be unit length (the `World` wrapper normalizes it).
/// Returns `(distance, normal)` along the ray, or `None` when the ray misses.
/// A ray starting inside the shape counts as a miss, which is what makes
/// `World::ray_cast` usable for line-of-sight checks.
pub fn ray_cast_shape(
  shape : Shape,
  xf : Transform,
  origin : Vec2,
  direction : Vec2,
  max_distance : Double,
) -> (Double, Vec2)? {
  match shape {
    Shape::Circle(radius) =>
      ray_circle(xf.position, radius, origin, direction, max_distance)
    Shape::Polygon(vertices) =>
      ray_polygon(vertices, xf, origin, direction, max_distance)
  }
}

///|
/// Ray against a circle, solved from the quadratic of the implicit equation.
fn ray_circle(
  center : Vec2,
  radius : Double,
  origin : Vec2,
  direction : Vec2,
  max_distance : Double,
) -> (Double, Vec2)? {
  let m = origin - center
  let b = m.dot(direction)
  let c = m.dot(m) - radius * radius
  // Outside and pointing away from the circle.
  if c > 0.0 && b > 0.0 {
    return None
  }
  let discriminant = b * b - c
  if discriminant < 0.0 {
    return None
  }
  let t = -b - discriminant.sqrt()
  if t < 0.0 || t > max_distance {
    return None
  }
  let point = origin + direction.scale(t)
  let normal = (point - center).normalize_or(Vec2::new(0.0, 1.0))
  Some((t, normal))
}

///|
/// Ray against a convex polygon by clipping the ray against every edge plane
/// (the slab method), which is O(n) and needs no allocations.
fn ray_polygon(
  vertices : Array[Vec2],
  xf : Transform,
  origin : Vec2,
  direction : Vec2,
  max_distance : Double,
) -> (Double, Vec2)? {
  let n = vertices.length()
  let mut lower = 0.0
  let mut upper = max_distance
  let mut normal = Vec2::zero()
  let mut entered = false
  for i in 0..<n {
    let v1 = xf.apply(vertices[i])
    let edge_normal = polygon_edge_normal(vertices, i).rotate(xf.rotation)
    let numerator = edge_normal.dot(v1 - origin)
    let denominator = edge_normal.dot(direction)
    if denominator == 0.0 {
      // Parallel to this edge: only a miss when the origin is outside.
      if numerator < 0.0 {
        return None
      }
    } else {
      let t = numerator / denominator
      if denominator < 0.0 {
        // Front facing edge: the ray is entering the polygon here.
        if t > lower {
          lower = t
          normal = edge_normal
          entered = true
        }
      } else if t < upper {
        upper = t
      }
      if lower > upper {
        return None
      }
    }
  }
  // `entered` is false when the origin already sits inside the polygon.
  if !entered || lower > max_distance {
    return None
  }
  Some((lower, normal))
}

///|
/// Cast a ray through the world and return the closest hit, or `None`.
///
/// `direction` is normalized internally, so `max_distance` is a plain world
/// distance. Static and moving bodies are both considered.
pub fn World::ray_cast(
  self : World,
  origin : Vec2,
  direction : Vec2,
  max_distance : Double,
) -> RayHit? {
  let dir = direction.normalize_or(Vec2::new(1.0, 0.0))
  let mut best : RayHit? = None
  let mut best_distance = max_distance
  let mut i = 0
  while i < self.bodies.length() {
    let body = self.bodies[i]
    let hit = ray_cast_shape(
      body.shape,
      body.transform(),
      origin,
      dir,
      best_distance,
    )
    match hit {
      Some(found) => {
        let (t, normal) = found
        if t < best_distance {
          best_distance = t
          best = Some({
            body_index: i,
            point: origin + dir.scale(t),
            normal,
            distance: t,
          })
        }
      }
      None => ()
    }
    i = i + 1
  }
  best
}

///|
/// All body indices whose AABB overlaps `region`, in ascending order.
pub fn World::query_aabb(self : World, region : Aabb) -> Array[Int32] {
  let out : Array[Int32] = []
  let mut i = 0
  while i < self.bodies.length() {
    if self.bodies[i].world_aabb.overlaps(region) {
      out.push(i)
    }
    i = i + 1
  }
  out
}

///|
/// Bodies whose shape contains the point, most useful for picking.
pub fn World::query_point(self : World, point : Vec2) -> Array[Int32] {
  let out : Array[Int32] = []
  let mut i = 0
  while i < self.bodies.length() {
    if self.bodies[i].contains_point(point) {
      out.push(i)
    }
    i = i + 1
  }
  out
}
