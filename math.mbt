///|
/// Numerical tolerance for length comparisons.
pub const K_EPSILON : Double = 1.0e-9

///|
/// Pi.
pub const K_PI : Double = 3.141592653589793

///|
/// Clamp `v` into `[lo, hi]`.
pub fn clamp(v : Double, lo : Double, hi : Double) -> Double {
  if v < lo {
    lo
  } else if v > hi {
    hi
  } else {
    v
  }
}

///|
/// A 2D vector of `Double`. Immutable: operations return fresh values, which
/// keeps constraint solving free of aliasing and keeps steps reproducible.
pub struct Vec2 {
  x : Double
  y : Double
}

///|
pub fn Vec2::new(x : Double, y : Double) -> Vec2 {
  { x, y }
}

///|
pub fn Vec2::zero() -> Vec2 {
  { x: 0.0, y: 0.0 }
}

///|
pub fn Vec2::splat(v : Double) -> Vec2 {
  { x: v, y: v }
}

///|
pub fn Vec2::x(self : Vec2) -> Double {
  self.x
}

///|
pub fn Vec2::y(self : Vec2) -> Double {
  self.y
}

///|
pub fn Vec2::dot(self : Vec2, other : Vec2) -> Double {
  self.x * other.x + self.y * other.y
}

///|
/// Scalar cross product `x1 * y2 - y1 * x2`.
pub fn Vec2::cross(self : Vec2, other : Vec2) -> Double {
  self.x * other.y - self.y * other.x
}

///|
/// `z x v = (-z * v.y, z * v.x)`.
pub fn Vec2::cross_scalar(self : Vec2, z : Double) -> Vec2 {
  Vec2::new(-z * self.y, z * self.x)
}

///|
/// `z x v = (-z * v.y, z * v.x)`, scalar on the left.
pub fn cross_scalar(z : Double, v : Vec2) -> Vec2 {
  Vec2::new(-z * v.y, z * v.x)
}

///|
/// Squared length.
pub fn Vec2::length_sq(self : Vec2) -> Double {
  self.x * self.x + self.y * self.y
}

///|
/// Length.
pub fn Vec2::length(self : Vec2) -> Double {
  self.length_sq().sqrt()
}

///|
/// Unit vector, or zero when the vector is degenerate.
pub fn Vec2::normalize(self : Vec2) -> Vec2 {
  let len = self.length()
  if len < K_EPSILON {
    Vec2::zero()
  } else {
    Vec2::new(self.x / len, self.y / len)
  }
}

///|
/// Unit vector, or `fallback` when degenerate.
pub fn Vec2::normalize_or(self : Vec2, fallback : Vec2) -> Vec2 {
  let len = self.length()
  if len < K_EPSILON {
    fallback
  } else {
    Vec2::new(self.x / len, self.y / len)
  }
}

///|
pub fn Vec2::scale(self : Vec2, s : Double) -> Vec2 {
  Vec2::new(self.x * s, self.y * s)
}

///|
/// Counter-clockwise perpendicular `(-y, x)`.
pub fn Vec2::perp(self : Vec2) -> Vec2 {
  Vec2::new(-self.y, self.x)
}

///|
/// Counter-clockwise rotation by `angle` radians.
pub fn Vec2::rotate(self : Vec2, angle : Double) -> Vec2 {
  let c = @math.cos(angle)
  let s = @math.sin(angle)
  Vec2::new(c * self.x - s * self.y, s * self.x + c * self.y)
}

///|
/// Roration by an angle given precomputed cosine/sine.
pub fn Vec2::rotate_by(self : Vec2, c : Double, s : Double) -> Vec2 {
  Vec2::new(c * self.x - s * self.y, s * self.x + c * self.y)
}

///|
/// Inverse rotation by `angle` radians.
pub fn Vec2::rotate_inv(self : Vec2, angle : Double) -> Vec2 {
  let c = @math.cos(angle)
  let s = @math.sin(angle)
  Vec2::new(c * self.x + s * self.y, -s * self.x + c * self.y)
}

///|
/// Component-wise absolute value.
pub fn Vec2::abs(self : Vec2) -> Vec2 {
  Vec2::new(self.x.abs(), self.y.abs())
}

///|
pub fn Vec2::min(self : Vec2, other : Vec2) -> Vec2 {
  Vec2::new(self.x.min(other.x), self.y.min(other.y))
}

///|
pub fn Vec2::max(self : Vec2, other : Vec2) -> Vec2 {
  Vec2::new(self.x.max(other.x), self.y.max(other.y))
}
