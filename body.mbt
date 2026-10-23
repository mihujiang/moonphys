///|
/// Static bodies never move; dynamic bodies are integrated and solved.
pub enum BodyType {
  Static
  Dynamic
}

///|
/// A rigid body: a shape plus its motion state and material parameters.
///
/// For polygons the vertices are re-centered on construction, so `position`
/// always denotes the center of mass. Circles are centered by definition.
pub struct Body {
  body_type : BodyType
  shape : Shape
  mut position : Vec2
  mut rotation : Double
  mut velocity : Vec2
  mut angular_velocity : Double
  mut force : Vec2
  mut torque : Double
  mut mass : Double
  mut inv_mass : Double
  mut inertia : Double
  mut inv_inertia : Double
  mut friction : Double
  mut restitution : Double
  mut linear_damping : Double
  mut angular_damping : Double
  mut allow_sleep : Bool
  mut awake : Bool
  mut sleep_time : Double
  mut user_data : Int
  mut world_aabb : Aabb
}

///|
pub fn Body::new(shape : Shape, body_type : BodyType) -> Body {
  let data = shape.mass_data(1.0)
  let mut local_shape = shape
  let mut offset = Vec2::zero()
  match shape {
    Shape::Polygon(_) =>
      if data.center.length() > K_EPSILON {
        offset = data.center
        local_shape = Shape::Polygon(shift_vertices(shape.vertices(), -offset))
      }
    Shape::Circle(_) => ()
  }
  let body : Body = {
    body_type,
    shape: local_shape,
    position: offset,
    rotation: 0.0,
    velocity: Vec2::zero(),
    angular_velocity: 0.0,
    force: Vec2::zero(),
    torque: 0.0,
    mass: 0.0,
    inv_mass: 0.0,
    inertia: 0.0,
    inv_inertia: 0.0,
    friction: 0.4,
    restitution: 0.1,
    linear_damping: 0.0,
    angular_damping: 0.0,
    allow_sleep: true,
    awake: true,
    sleep_time: 0.0,
    user_data: 0,
    world_aabb: Aabb::new(Vec2::zero(), Vec2::zero()),
  }
  body.set_mass_data(data)
  body.sync_aabb()
  body
}

///|
fn shift_vertices(vertices : Array[Vec2], offset : Vec2) -> Array[Vec2] {
  let out : Array[Vec2] = []
  for v in vertices {
    out.push(v + offset)
  }
  out
}

///|
/// A dynamic body, moved by the solver.
pub fn Body::dynamic(shape : Shape) -> Body {
  Body::new(shape, BodyType::Dynamic)
}

///|
/// An immovable body, e.g. the ground.
pub fn Body::fixed(shape : Shape) -> Body {
  Body::new(shape, BodyType::Static)
}

///|
/// A dynamic box centered at the current position.
pub fn Body::box(half_width : Double, half_height : Double) -> Body {
  Body::dynamic(Shape::box(half_width, half_height))
}

///|
/// A dynamic circle.
pub fn Body::ball(radius : Double) -> Body {
  Body::dynamic(Shape::circle(radius))
}

///|
pub fn Body::shape(self : Body) -> Shape {
  self.shape
}

///|
pub fn Body::position(self : Body) -> Vec2 {
  self.position
}

///|
pub fn Body::rotation(self : Body) -> Double {
  self.rotation
}

///|
pub fn Body::velocity(self : Body) -> Vec2 {
  self.velocity
}

///|
pub fn Body::angular_velocity(self : Body) -> Double {
  self.angular_velocity
}

///|
pub fn Body::mass(self : Body) -> Double {
  self.mass
}

///|
pub fn Body::inertia(self : Body) -> Double {
  self.inertia
}

///|
pub fn Body::friction(self : Body) -> Double {
  self.friction
}

///|
pub fn Body::restitution(self : Body) -> Double {
  self.restitution
}

///|
/// Free integer tag, handy for render colors and test bookkeeping.
pub fn Body::user_data(self : Body) -> Int {
  self.user_data
}

///|
pub fn Body::is_static(self : Body) -> Bool {
  match self.body_type {
    BodyType::Static => true
    BodyType::Dynamic => false
  }
}

///|
pub fn Body::is_dynamic(self : Body) -> Bool {
  !self.is_static()
}

///|
pub fn Body::transform(self : Body) -> Transform {
  Transform::new(self.position, self.rotation)
}

///|
pub fn Body::aabb(self : Body) -> Aabb {
  self.world_aabb
}

///|
/// Cached world AABB, refreshed by the world each step.
pub fn Body::sync_aabb(self : Body) -> Unit {
  self.world_aabb = self.shape.aabb(self.transform())
}

///|
pub fn Body::awake(self : Body) -> Bool {
  self.awake
}

///|
pub fn Body::wake(self : Body) -> Unit {
  if self.is_dynamic() {
    self.awake = true
    self.sleep_time = 0.0
  }
}

///|
pub fn Body::sleep(self : Body) -> Unit {
  if self.is_dynamic() {
    self.awake = false
    self.sleep_time = 0.0
    self.velocity = Vec2::zero()
    self.angular_velocity = 0.0
  }
}

///|
/// Recompute mass and inertia for `density`.
pub fn Body::set_density(self : Body, density : Double) -> Unit {
  self.set_mass_data(self.shape.mass_data(density))
}

///|
fn Body::set_mass_data(self : Body, data : MassData) -> Unit {
  match self.body_type {
    BodyType::Static => {
      self.mass = 0.0
      self.inv_mass = 0.0
      self.inertia = 0.0
      self.inv_inertia = 0.0
    }
    BodyType::Dynamic =>
      if data.mass() > K_EPSILON && data.inertia() > K_EPSILON {
        self.mass = data.mass()
        self.inv_mass = 1.0 / data.mass()
        self.inertia = data.inertia()
        self.inv_inertia = 1.0 / data.inertia()
      } else {
        self.mass = 0.0
        self.inv_mass = 0.0
        self.inertia = 0.0
        self.inv_inertia = 0.0
      }
  }
}

///|
/// Local point to world point.
pub fn Body::local_to_world(self : Body, point : Vec2) -> Vec2 {
  self.position + point.rotate(self.rotation)
}

///|
/// World point to local point.
pub fn Body::world_to_local(self : Body, point : Vec2) -> Vec2 {
  (point - self.position).rotate_inv(self.rotation)
}

///|
/// Whether a world-space point is inside the body.
pub fn Body::contains_point(self : Body, point : Vec2) -> Bool {
  self.shape.contains_local_point(self.world_to_local(point))
}

///|
pub fn Body::apply_force(self : Body, force : Vec2) -> Unit {
  if self.is_dynamic() {
    self.force = self.force + force
    self.wake()
  }
}

///|
pub fn Body::apply_torque(self : Body, torque : Double) -> Unit {
  if self.is_dynamic() {
    self.torque = self.torque + torque
    self.wake()
  }
}

///|
/// Apply an impulse at a world-space point.
pub fn Body::apply_impulse(self : Body, impulse : Vec2, point : Vec2) -> Unit {
  if self.is_dynamic() {
    self.velocity = self.velocity + impulse.scale(self.inv_mass)
    let r = point - self.position
    self.angular_velocity = self.angular_velocity +
      self.inv_inertia * r.cross(impulse)
    self.wake()
  }
}

///|
pub fn Body::apply_angular_impulse(self : Body, impulse : Double) -> Unit {
  if self.is_dynamic() {
    self.angular_velocity = self.angular_velocity + self.inv_inertia * impulse
    self.wake()
  }
}

///|
/// Velocity of a world-space point on the body.
pub fn Body::point_velocity(self : Body, point : Vec2) -> Vec2 {
  self.velocity + cross_scalar(self.angular_velocity, point - self.position)
}

///|
/// Translational plus rotational kinetic energy.
pub fn Body::kinetic_energy(self : Body) -> Double {
  0.5 *
  (
    self.mass * self.velocity.length_sq() +
    self.inertia * self.angular_velocity * self.angular_velocity
  )
}

///|
pub fn Body::clear_forces(self : Body) -> Unit {
  self.force = Vec2::zero()
  self.torque = 0.0
}

///|
/// Builder: move the body so its center of mass sits at `(x, y)`.
pub fn Body::at(self : Body, x : Double, y : Double) -> Body {
  self.position = Vec2::new(x, y)
  self.sync_aabb()
  self
}

///|
pub fn Body::with_rotation(self : Body, angle : Double) -> Body {
  self.rotation = angle
  self.sync_aabb()
  self
}

///|
pub fn Body::with_velocity(self : Body, vx : Double, vy : Double) -> Body {
  self.velocity = Vec2::new(vx, vy)
  self
}

///|
pub fn Body::with_angular_velocity(self : Body, w : Double) -> Body {
  self.angular_velocity = w
  self
}

///|
pub fn Body::with_friction(self : Body, friction : Double) -> Body {
  self.friction = friction
  self
}

///|
pub fn Body::with_restitution(self : Body, restitution : Double) -> Body {
  self.restitution = restitution
  self
}

///|
pub fn Body::with_density(self : Body, density : Double) -> Body {
  self.set_density(density)
  self
}

///|
pub fn Body::with_damping(
  self : Body,
  linear : Double,
  angular : Double,
) -> Body {
  self.linear_damping = linear
  self.angular_damping = angular
  self
}

///|
pub fn Body::with_allow_sleep(self : Body, allow : Bool) -> Body {
  self.allow_sleep = allow
  self
}

///|
pub fn Body::with_user_data(self : Body, data : Int) -> Body {
  self.user_data = data
  self
}
