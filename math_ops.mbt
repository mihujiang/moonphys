///|
pub impl Add for Vec2 with fn add(self, other) {
  Vec2::new(self.x + other.x, self.y + other.y)
}

///|
pub impl Sub for Vec2 with fn sub(self, other) {
  Vec2::new(self.x - other.x, self.y - other.y)
}

///|
pub impl Neg for Vec2 with fn neg(self) {
  Vec2::new(-self.x, -self.y)
}

///|
pub impl Eq for Vec2 with fn equal(self, other) {
  self.x == other.x && self.y == other.y
}

///|
pub impl Show for Vec2 with fn to_string(self) {
  "(\{self.x},\{self.y})"
}

///|
/// A 2x2 matrix stored as two column vectors.
pub struct Mat22 {
  c1 : Vec2
  c2 : Vec2
}

///|
pub fn Mat22::from_angle(angle : Double) -> Mat22 {
  let c = @math.cos(angle)
  let s = @math.sin(angle)
  { c1: Vec2::new(c, s), c2: Vec2::new(-s, c) }
}

///|
pub fn Mat22::identity() -> Mat22 {
  { c1: Vec2::new(1.0, 0.0), c2: Vec2::new(0.0, 1.0) }
}

///|
pub fn Mat22::mul_vec(self : Mat22, v : Vec2) -> Vec2 {
  self.c1.scale(v.x) + self.c2.scale(v.y)
}

///|
pub fn Mat22::transpose(self : Mat22) -> Mat22 {
  { c1: Vec2::new(self.c1.x, self.c2.x), c2: Vec2::new(self.c1.y, self.c2.y) }
}

///|
pub fn Mat22::abs(self : Mat22) -> Mat22 {
  { c1: self.c1.abs(), c2: self.c2.abs() }
}

///|
/// Rotation of a body: world = rotation * local + position.
pub struct Transform {
  position : Vec2
  rotation : Double
}

///|
pub fn Transform::new(position : Vec2, rotation : Double) -> Transform {
  { position, rotation }
}

///|
/// Transform a point from local space to world space.
pub fn Transform::apply(self : Transform, point : Vec2) -> Vec2 {
  self.position + point.rotate(self.rotation)
}

///|
/// Rotate a vector from local space to world space.
pub fn Transform::apply_vector(self : Transform, direction : Vec2) -> Vec2 {
  direction.rotate(self.rotation)
}

///|
/// Rotate a vector from world space to local space.
pub fn Transform::apply_inv_vector(self : Transform, world : Vec2) -> Vec2 {
  world.rotate_inv(self.rotation)
}

///|
/// Transform a point from world space to local space.
pub fn Transform::apply_inv(self : Transform, world : Vec2) -> Vec2 {
  (world - self.position).rotate_inv(self.rotation)
}
