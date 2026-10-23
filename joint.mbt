///|
/// Correction factor for joint position error.
const JOINT_BIAS_FACTOR : Double = 0.2

///|
/// Maximum positional error corrected per step, in world units.
const JOINT_MAX_CORRECTION : Double = 0.2

///|
/// A rigid rod that keeps two anchor points at a fixed distance.
pub struct DistanceJoint {
  body_a : Int32
  body_b : Int32
  local_anchor_a : Vec2
  local_anchor_b : Vec2
  length : Double
  mut impulse : Double
  mut mass : Double
  mut bias : Double
  mut gamma : Double
  mut frequency : Double
  mut damping_ratio : Double
}

///|
/// Pin the distance between two world anchors to length.
pub fn DistanceJoint::new(
  body_a : Int32,
  body_b : Int32,
  local_anchor_a : Vec2,
  local_anchor_b : Vec2,
  length : Double,
) -> DistanceJoint {
  {
    body_a,
    body_b,
    local_anchor_a,
    local_anchor_b,
    length,
    impulse: 0.0,
    mass: 0.0,
    bias: 0.0,
    gamma: 0.0,
    frequency: 0.0,
    damping_ratio: 1.0,
  }
}

///|
/// Join the centers of two bodies at their current distance.
pub fn DistanceJoint::between(
  world : World,
  body_a : Int32,
  body_b : Int32,
) -> DistanceJoint {
  let a = world.body(body_a)
  let b = world.body(body_b)
  DistanceJoint::new(
    body_a,
    body_b,
    Vec2::zero(),
    Vec2::zero(),
    (b.position() - a.position()).length(),
  )
}

///|
/// Target distance between the two anchors.
pub fn DistanceJoint::length(self : DistanceJoint) -> Double {
  self.length
}

///|
/// Turn the rigid rod into a spring.
///
/// `frequency` is in Hz and `damping_ratio` of 1 is critically damped. With
/// softness the joint trades exactness for stability to avoid the jitter a hard
/// constraint shows on long chains.
pub fn DistanceJoint::with_softness(
  self : DistanceJoint,
  frequency : Double,
  damping_ratio : Double,
) -> DistanceJoint {
  self.frequency = frequency.max(0.0)
  self.damping_ratio = damping_ratio.max(0.0)
  self
}

///|
/// World-space anchor points of the joint.
pub fn DistanceJoint::anchors(
  self : DistanceJoint,
  bodies : Array[Body],
) -> (Vec2, Vec2) {
  let a = bodies[self.body_a]
  let b = bodies[self.body_b]
  (
    a.position + self.local_anchor_a.rotate(a.rotation),
    b.position + self.local_anchor_b.rotate(b.rotation),
  )
}

///|
/// Compute the effective mass and the velocity bias for this step.
pub fn DistanceJoint::pre_step(
  self : DistanceJoint,
  bodies : Array[Body],
  inv_dt : Double,
) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let offset_a = self.local_anchor_a.rotate(body_a.rotation)
  let offset_b = self.local_anchor_b.rotate(body_b.rotation)
  let delta = body_b.position + offset_b - (body_a.position + offset_a)
  let current = delta.length()
  let axis = delta.normalize_or(Vec2::new(1.0, 0.0))
  let cr_a = offset_a.cross(axis)
  let cr_b = offset_b.cross(axis)
  let k = body_a.inv_mass +
    body_b.inv_mass +
    body_a.inv_inertia * cr_a * cr_a +
    body_b.inv_inertia * cr_b * cr_b
  let error = joint_error(current - self.length)
  if self.frequency > 0.0 && k > K_EPSILON {
    // Soft constraint: gamma and bias come from the spring-damper model, so
    // the solver sees a spring instead of a rigid rod.
    let dt = 1.0 / inv_dt
    let raw_mass = 1.0 / k
    let omega = 2.0 * K_PI * self.frequency
    let damping = 2.0 * raw_mass * self.damping_ratio * omega
    let stiffness = raw_mass * omega * omega
    let gamma = dt * (damping + dt * stiffness)
    self.gamma = if gamma > K_EPSILON { 1.0 / gamma } else { 0.0 }
    self.bias = error * dt * stiffness * self.gamma
    self.mass = 1.0 / (k + self.gamma)
  } else {
    self.gamma = 0.0
    self.mass = if k > K_EPSILON { 1.0 / k } else { 0.0 }
    self.bias = JOINT_BIAS_FACTOR * inv_dt * error
  }
  let impulse = axis.scale(self.impulse)
  body_a.velocity = body_a.velocity - impulse.scale(body_a.inv_mass)
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * offset_a.cross(impulse)
  body_b.velocity = body_b.velocity + impulse.scale(body_b.inv_mass)
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * offset_b.cross(impulse)
}

///|
/// Apply one iteration of the distance constraint.
pub fn DistanceJoint::solve_velocity(
  self : DistanceJoint,
  bodies : Array[Body],
) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let offset_a = self.local_anchor_a.rotate(body_a.rotation)
  let offset_b = self.local_anchor_b.rotate(body_b.rotation)
  let point_a = body_a.position + offset_a
  let point_b = body_b.position + offset_b
  let axis = (point_b - point_a).normalize_or(Vec2::new(1.0, 0.0))
  let dv = body_b.point_velocity(point_b) - body_a.point_velocity(point_a)
  let vn = dv.dot(axis)
  let d_impulse = -self.mass * (vn + self.bias + self.gamma * self.impulse)
  self.impulse = self.impulse + d_impulse
  let impulse = axis.scale(d_impulse)
  body_a.velocity = body_a.velocity - impulse.scale(body_a.inv_mass)
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * offset_a.cross(impulse)
  body_b.velocity = body_b.velocity + impulse.scale(body_b.inv_mass)
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * offset_b.cross(impulse)
}

///|
/// A hinge: two anchor points are kept coincident, both bodies may rotate.
pub struct RevolveJoint {
  body_a : Int32
  body_b : Int32
  local_anchor_a : Vec2
  local_anchor_b : Vec2
  mut impulse : Vec2
  mut k11 : Double
  mut k12 : Double
  mut k22 : Double
  mut bias : Vec2
}

///|
/// Pin two local anchors to the same world point, leaving rotation free.
pub fn RevolveJoint::new(
  body_a : Int32,
  body_b : Int32,
  local_anchor_a : Vec2,
  local_anchor_b : Vec2,
) -> RevolveJoint {
  {
    body_a,
    body_b,
    local_anchor_a,
    local_anchor_b,
    impulse: Vec2::zero(),
    k11: 0.0,
    k12: 0.0,
    k22: 0.0,
    bias: Vec2::zero(),
  }
}

///|
/// Revolve joint whose anchors are both at the given world point.
pub fn RevolveJoint::at(
  world : World,
  body_a : Int32,
  body_b : Int32,
  anchor : Vec2,
) -> RevolveJoint {
  RevolveJoint::new(
    body_a,
    body_b,
    world.body(body_a).world_to_local(anchor),
    world.body(body_b).world_to_local(anchor),
  )
}

///|
/// Inverse of the 2x2 effective mass matrix applied to `v`.
fn solve22(k11 : Double, k12 : Double, k22 : Double, v : Vec2) -> Vec2 {
  let det = k11 * k22 - k12 * k12
  if det.abs() < 1.0e-12 {
    return Vec2::zero()
  }
  let inv_det = 1.0 / det
  Vec2::new(
    inv_det * (k22 * v.x - k12 * v.y),
    inv_det * (k11 * v.y - k12 * v.x),
  )
}

///|
/// Compute the 2x2 effective mass and the position bias.
pub fn RevolveJoint::pre_step(
  self : RevolveJoint,
  bodies : Array[Body],
  inv_dt : Double,
) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let offset_a = self.local_anchor_a.rotate(body_a.rotation)
  let offset_b = self.local_anchor_b.rotate(body_b.rotation)
  let inv_mass = body_a.inv_mass + body_b.inv_mass
  let inv_inertia_a = body_a.inv_inertia
  let inv_inertia_b = body_b.inv_inertia
  self.k11 = inv_mass +
    inv_inertia_a * offset_a.y * offset_a.y +
    inv_inertia_b * offset_b.y * offset_b.y
  self.k12 = -inv_inertia_a * offset_a.x * offset_a.y -
    inv_inertia_b * offset_b.x * offset_b.y
  self.k22 = inv_mass +
    inv_inertia_a * offset_a.x * offset_a.x +
    inv_inertia_b * offset_b.x * offset_b.x
  let error = body_b.position + offset_b - (body_a.position + offset_a)
  let correction = Vec2::new(
    clamp(error.x, -JOINT_MAX_CORRECTION, JOINT_MAX_CORRECTION),
    clamp(error.y, -JOINT_MAX_CORRECTION, JOINT_MAX_CORRECTION),
  )
  self.bias = correction.scale(JOINT_BIAS_FACTOR * inv_dt)
  let impulse = self.impulse
  body_a.velocity = body_a.velocity - impulse.scale(body_a.inv_mass)
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * offset_a.cross(impulse)
  body_b.velocity = body_b.velocity + impulse.scale(body_b.inv_mass)
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * offset_b.cross(impulse)
}

///|
/// Apply one iteration of the hinge constraint.
pub fn RevolveJoint::solve_velocity(
  self : RevolveJoint,
  bodies : Array[Body],
) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let offset_a = self.local_anchor_a.rotate(body_a.rotation)
  let offset_b = self.local_anchor_b.rotate(body_b.rotation)
  let point = body_a.position + offset_a
  let dv = body_b.point_velocity(body_b.position + offset_b) -
    body_a.point_velocity(point)
  let rhs = dv + self.bias
  let d_impulse = solve22(self.k11, self.k12, self.k22, rhs).scale(-1.0)
  self.impulse = self.impulse + d_impulse
  body_a.velocity = body_a.velocity - d_impulse.scale(body_a.inv_mass)
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * offset_a.cross(d_impulse)
  body_b.velocity = body_b.velocity + d_impulse.scale(body_b.inv_mass)
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * offset_b.cross(d_impulse)
}

///|
/// Type-erased joint stored in a world.
pub enum Joint {
  Distance(DistanceJoint)
  Revolve(RevolveJoint)
  Prismatic(PrismaticJoint)
  AngleLimit(AngleLimitJoint)
  Motor(MotorJoint)
}

///|
/// Let the joint prepare its effective mass and bias for this step.
pub fn Joint::pre_step(
  self : Joint,
  bodies : Array[Body],
  inv_dt : Double,
) -> Unit {
  match self {
    Distance(joint) => joint.pre_step(bodies, inv_dt)
    Revolve(joint) => joint.pre_step(bodies, inv_dt)
    Prismatic(joint) => joint.pre_step(bodies, inv_dt)
    AngleLimit(joint) => joint.pre_step(bodies)
    Motor(joint) => joint.pre_step(bodies, inv_dt)
  }
}

///|
/// Let the joint apply one velocity iteration.
pub fn Joint::solve_velocity(self : Joint, bodies : Array[Body]) -> Unit {
  match self {
    Distance(joint) => joint.solve_velocity(bodies)
    Revolve(joint) => joint.solve_velocity(bodies)
    Prismatic(joint) => joint.solve_velocity(bodies)
    AngleLimit(joint) => joint.solve_velocity(bodies)
    Motor(joint) => joint.solve_velocity(bodies)
  }
}

///|
/// Post-integration positional pass. Hard stops are solved as pure velocity
/// constraints, so whatever overshoot is left is projected out here.
pub fn Joint::correct_positions(self : Joint, bodies : Array[Body]) -> Unit {
  match self {
    Prismatic(joint) => joint.correct_positions(bodies)
    AngleLimit(joint) => joint.correct_positions(bodies)
    Distance(_) | Revolve(_) | Motor(_) => ()
  }
}
