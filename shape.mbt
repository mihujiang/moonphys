///|
/// Axis-aligned bounding box.
pub struct Aabb {
  min : Vec2
  max : Vec2
}

///|
pub fn Aabb::new(min : Vec2, max : Vec2) -> Aabb {
  { min, max, }
}

///|
pub fn Aabb::overlaps(self : Aabb, other : Aabb) -> Bool {
  if other.min.x > self.max.x || other.max.x < self.min.x {
    return false
  }
  if other.min.y > self.max.y || other.max.y < self.min.y {
    return false
  }
  true
}

///|
pub fn Aabb::contains_point(self : Aabb, p : Vec2) -> Bool {
  p.x >= self.min.x &&
  p.x <= self.max.x &&
  p.y >= self.min.y &&
  p.y <= self.max.y
}

///|
pub fn Aabb::union(self : Aabb, other : Aabb) -> Aabb {
  { min: self.min.min(other.min), max: self.max.max(other.max), }
}

///|
/// Grow the box by `margin` on every side.
pub fn Aabb::expanded(self : Aabb, margin : Double) -> Aabb {
  { min: self.min - Vec2::splat(margin), max: self.max + Vec2::splat(margin), }
}

///|
pub fn Aabb::center(self : Aabb) -> Vec2 {
  (self.min + self.max).scale(0.5)
}

///|
pub fn Aabb::width(self : Aabb) -> Double {
  self.max.x - self.min.x
}

///|
pub fn Aabb::height(self : Aabb) -> Double {
  self.max.y - self.min.y
}

///|
pub fn Aabb::min(self : Aabb) -> Vec2 {
  self.min
}

///|
pub fn Aabb::max(self : Aabb) -> Vec2 {
  self.max
}

///|
/// Mass properties of a shape with unit density scaled by `density`.
pub struct MassData {
  mass : Double
  inertia : Double
  center : Vec2
}

///|
pub fn MassData::mass(self : MassData) -> Double {
  self.mass
}

///|
pub fn MassData::inertia(self : MassData) -> Double {
  self.inertia
}

///|
pub fn MassData::center(self : MassData) -> Vec2 {
  self.center
}

///|
/// Collision geometry in local space.
///
/// `Polygon` vertices must form a convex counter-clockwise hull; use
/// `Shape::convex` to build one safely.
pub enum Shape {
  Circle(Double)
  Polygon(Array[Vec2])
}

///|
pub fn Shape::circle(radius : Double) -> Shape {
  Circle(radius)
}

///|
/// Axis-aligned box centered at the local origin.
pub fn Shape::box(half_width : Double, half_height : Double) -> Shape {
  Polygon([
    Vec2::new(-half_width, -half_height),
    Vec2::new(half_width, -half_height),
    Vec2::new(half_width, half_height),
    Vec2::new(-half_width, half_height),
  ])
}

///|
/// Build a convex polygon, forcing counter-clockwise winding.
///
/// Duplicated and collinear vertices are dropped first, because a zero-length
/// edge makes the edge normal undefined and would poison the solver.
pub fn Shape::convex(vertices : Array[Vec2]) -> Shape {
  let cleaned = remove_degenerate_vertices(vertices)
  if cleaned.length() < 3 {
    abort("Shape::convex needs at least 3 distinct, non-collinear vertices")
  }
  let vs = if signed_area(cleaned) < 0.0 {
    reverse_vertices(cleaned)
  } else {
    cleaned
  }
  Polygon(vs)
}

///|
const VERTEX_EPSILON : Double = 1.0e-9

///|
/// Drop repeated vertices and vertices that lie on the straight line between
/// their neighbours. Convexity itself is the caller's responsibility.
fn remove_degenerate_vertices(vertices : Array[Vec2]) -> Array[Vec2] {
  let deduped : Array[Vec2] = []
  for v in vertices {
    let n = deduped.length()
    if n == 0 || (v - deduped[n - 1]).length() > VERTEX_EPSILON {
      deduped.push(v)
    }
  }
  // The wrap-around edge can be degenerate as well.
  let n = deduped.length()
  if n > 1 && (deduped[0] - deduped[n - 1]).length() <= VERTEX_EPSILON {
    ignore(deduped.pop())
  }
  let count = deduped.length()
  if count < 3 {
    return deduped
  }
  let kept : Array[Vec2] = []
  for i in 0..<count {
    let prev = deduped[(i + count - 1) % count]
    let cur = deduped[i]
    let next = deduped[(i + 1) % count]
    let twice_area = (cur - prev).cross(next - cur)
    if twice_area.abs() > VERTEX_EPSILON {
      kept.push(cur)
    }
  }
  kept
}

///|
fn reverse_vertices(vertices : Array[Vec2]) -> Array[Vec2] {
  let out : Array[Vec2] = []
  let mut i = vertices.length() - 1
  while i >= 0 {
    out.push(vertices[i])
    i = i - 1
  }
  out
}

///|
fn signed_area(vertices : Array[Vec2]) -> Double {
  let mut sum = 0.0
  let n = vertices.length()
  for i in 0..<n {
    sum = sum + vertices[i].cross(vertices[(i + 1) % n])
  }
  sum * 0.5
}

///|
/// Area of the shape (polygon area, or circle area).
pub fn Shape::area(self : Shape) -> Double {
  match self {
    Circle(r) => K_PI * r * r
    Polygon(vs) => signed_area(vs).abs()
  }
}

///|
/// Outward unit normal of polygon edge `i`.
pub fn Shape::edge_normal(self : Shape, i : Int32) -> Vec2 {
  match self {
    Circle(_) => Vec2::zero()
    Polygon(vs) => {
      let n = vs.length()
      let edge = vs[(i + 1) % n] - vs[i]
      Vec2::new(edge.y, -edge.x).normalize()
    }
  }
}

///|
/// Radius of a circle shape, or `0.0` for polygons.
pub fn Shape::radius(self : Shape) -> Double {
  match self {
    Circle(r) => r
    Polygon(_) => 0.0
  }
}

///|
/// Local-space vertices; empty for circles.
pub fn Shape::vertices(self : Shape) -> Array[Vec2] {
  match self {
    Circle(_) => []
    Polygon(vs) => vs
  }
}

///|
pub fn Shape::is_circle(self : Shape) -> Bool {
  match self {
    Circle(_) => true
    Polygon(_) => false
  }
}

///|
/// Compute mass properties for the given density.
pub fn Shape::mass_data(self : Shape, density : Double) -> MassData {
  match self {
    Circle(r) => {
      let mass = density * K_PI * r * r
      { mass, inertia: 0.5 * mass * r * r, center: Vec2::zero(), }
    }
    Polygon(vs) => polygon_mass_data(vs, density)
  }
}

///|
fn polygon_mass_data(vs : Array[Vec2], density : Double) -> MassData {
  let n = vs.length()
  let mut area_acc = 0.0
  let mut centroid_acc = Vec2::zero()
  let mut inertia_acc = 0.0
  for i in 0..<n {
    let p = vs[i]
    let q = vs[(i + 1) % n]
    let cross = p.cross(q)
    area_acc = area_acc + cross
    centroid_acc = centroid_acc + (p + q).scale(cross)
    inertia_acc = inertia_acc +
      cross * (p.length_sq() + p.dot(q) + q.length_sq())
  }
  let area = area_acc * 0.5
  if area.abs() < K_EPSILON {
    return { mass: 0.0, inertia: 0.0, center: Vec2::zero(), }
  }
  let center = centroid_acc.scale(1.0 / (6.0 * area))
  let mass = density * area.abs()
  let inertia_origin = density * inertia_acc / 12.0
  let inertia = inertia_origin - mass * center.length_sq()
  { mass, inertia: inertia.max(1.0e-12), center, }
}

///|
/// World-space AABB of the shape placed by `transform`.
pub fn Shape::aabb(self : Shape, transform : Transform) -> Aabb {
  match self {
    Circle(r) => {
      let c = transform.position
      Aabb::new(c - Vec2::splat(r), c + Vec2::splat(r))
    }
    Polygon(vs) => {
      let first = transform.apply(vs[0])
      let mut lo = first
      let mut hi = first
      let n = vs.length()
      for i in 1..<n {
        let p = transform.apply(vs[i])
        lo = lo.min(p)
        hi = hi.max(p)
      }
      Aabb::new(lo, hi)
    }
  }
}

///|
/// Whether a local-space point lies inside the shape.
pub fn Shape::contains_local_point(self : Shape, point : Vec2) -> Bool {
  match self {
    Circle(r) => point.length_sq() <= r * r
    Polygon(vs) => {
      let n = vs.length()
      for i in 0..<n {
        let normal = Vec2::new(
          (vs[(i + 1) % n] - vs[i]).y,
          -(vs[(i + 1) % n] - vs[i]).x,
        ).normalize()
        if (point - vs[i]).dot(normal) > K_EPSILON {
          return false
        }
      }
      true
    }
  }
}
