///|
/// Apply an impulse acting along `axis` at the two joint anchors.
///
/// `lever_a` / `lever_b` are the precomputed `r x axis` terms, which keep the
/// angular part in sync with the linear part without recomputing rotations.
fn apply_axial(
  body_a : Body,
  body_b : Body,
  axis : Vec2,
  lever_a : Double,
  lever_b : Double,
  impulse : Double,
) -> Unit {
  let p = axis.scale(impulse)
  body_a.velocity = body_a.velocity - p.scale(body_a.inv_mass)
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * lever_a * impulse
  body_b.velocity = body_b.velocity + p.scale(body_b.inv_mass)
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * lever_b * impulse
}

///|
/// Clamp a positional error to the amount the solver may correct in one step.
fn joint_error(raw : Double) -> Double {
  clamp(raw, -JOINT_MAX_CORRECTION, JOINT_MAX_CORRECTION)
}

///|
/// Fraction of the remaining violation a single step removes.
const JOINT_POSITION_FACTOR : Double = 0.2

///|
/// A slider: the two anchors are constrained to stay on one axis, relative
/// rotation is locked, and translation along the axis can be limited and driven
/// by a motor.
///
/// The axis is stored in body A's frame, so a slider attached to a static body
/// is a plain world-space rail.
pub struct PrismaticJoint {
  body_a : Int32
  body_b : Int32
  local_anchor_a : Vec2
  local_anchor_b : Vec2
  local_axis_a : Vec2
  mut has_limits : Bool
  mut lower : Double
  mut upper : Double
  mut motor_speed : Double
  mut max_motor_force : Double
  mut impulse : Vec2
  mut motor_impulse : Double
  mut limit_impulse : Double
  mut k11 : Double
  mut k12 : Double
  mut k22 : Double
  mut axis_mass : Double
  mut motor_cap : Double
  mut bias : Vec2
  mut active : Int32
}

///|
pub fn PrismaticJoint::new(
  body_a : Int32,
  body_b : Int32,
  local_anchor_a : Vec2,
  local_anchor_b : Vec2,
  local_axis_a : Vec2,
) -> PrismaticJoint {
  {
    body_a,
    body_b,
    local_anchor_a,
    local_anchor_b,
    local_axis_a: local_axis_a.normalize_or(Vec2::new(1.0, 0.0)),
    has_limits: false,
    lower: 0.0,
    upper: 0.0,
    motor_speed: 0.0,
    max_motor_force: 0.0,
    impulse: Vec2::zero(),
    motor_impulse: 0.0,
    limit_impulse: 0.0,
    k11: 0.0,
    k12: 0.0,
    k22: 0.0,
    axis_mass: 0.0,
    motor_cap: 0.0,
    bias: Vec2::zero(),
    active: 0,
  }
}

///|
/// Slider whose anchors are both at `anchor` and whose rail is `axis` in world
/// space at construction time.
pub fn PrismaticJoint::at(
  world : World,
  body_a : Int32,
  body_b : Int32,
  anchor : Vec2,
  axis : Vec2,
) -> PrismaticJoint {
  let a = world.body(body_a)
  PrismaticJoint::new(
    body_a,
    body_b,
    a.world_to_local(anchor),
    world.body(body_b).world_to_local(anchor),
    axis.rotate_inv(a.rotation),
  )
}

///|
/// Restrict the travel along the rail to `[lower, upper]`, in world units.
pub fn PrismaticJoint::with_limits(
  self : PrismaticJoint,
  lower : Double,
  upper : Double,
) -> PrismaticJoint {
  self.has_limits = true
  self.lower = lower.min(upper)
  self.upper = lower.max(upper)
  self
}

///|
/// Drive the slider along its rail at `speed` (units/second) with at most
/// `max_force` of force.
pub fn PrismaticJoint::with_motor(
  self : PrismaticJoint,
  speed : Double,
  max_force : Double,
) -> PrismaticJoint {
  self.motor_speed = speed
  self.max_motor_force = max_force.max(0.0)
  self
}

///|
pub fn PrismaticJoint::pre_step(
  self : PrismaticJoint,
  bodies : Array[Body],
  inv_dt : Double,
) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let offset_a = self.local_anchor_a.rotate(body_a.rotation)
  let offset_b = self.local_anchor_b.rotate(body_b.rotation)
  let axis = self.local_axis_a.rotate(body_a.rotation)
  let normal = axis.perp()
  let delta = body_b.position + offset_b - (body_a.position + offset_a)
  let arm_a = delta + offset_a
  let inv_mass = body_a.inv_mass + body_b.inv_mass
  let lever_a = arm_a.cross(normal)
  let lever_b = offset_b.cross(normal)
  self.k11 = inv_mass +
    body_a.inv_inertia * lever_a * lever_a +
    body_b.inv_inertia * lever_b * lever_b
  self.k12 = body_a.inv_inertia * lever_a + body_b.inv_inertia * lever_b
  let k22 = body_a.inv_inertia + body_b.inv_inertia
  self.k22 = if k22 > K_EPSILON { k22 } else { 1.0 }
  self.bias = Vec2::new(
    -JOINT_BIAS_FACTOR * inv_dt * joint_error(delta.dot(normal)),
    -JOINT_BIAS_FACTOR * inv_dt * joint_error(body_b.rotation - body_a.rotation),
  )
  let axis_lever_a = arm_a.cross(axis)
  let axis_lever_b = offset_b.cross(axis)
  let k_axis = inv_mass +
    body_a.inv_inertia * axis_lever_a * axis_lever_a +
    body_b.inv_inertia * axis_lever_b * axis_lever_b
  self.axis_mass = if k_axis > K_EPSILON { 1.0 / k_axis } else { 0.0 }
  self.motor_cap = self.max_motor_force / inv_dt
  let translation = delta.dot(axis)
  let mut side = 0
  // Stops are solved as pure velocity constraints; the residual overshoot is
  // removed afterwards by correct_positions(), which keeps the contact
  // inelastic instead of bouncing the body back into the free range.
  if self.has_limits {
    if translation < self.lower {
      side = -1
    } else if translation > self.upper {
      side = 1
    }
  }
  if side == 0 {
    // Keep the stop engaged while it still carries an impulse: the solver then
    // drives the approach speed to zero instead of letting the body bounce off.
    if self.active != 0 && self.limit_impulse != 0.0 {
      side = self.active
    } else {
      self.limit_impulse = 0.0
    }
  }
  self.active = side
  // Warm start: re-apply the impulses accumulated on the previous step.
  let p = normal.scale(self.impulse.x)
  body_a.velocity = body_a.velocity - p.scale(body_a.inv_mass)
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * (self.impulse.x * lever_a + self.impulse.y)
  body_b.velocity = body_b.velocity + p.scale(body_b.inv_mass)
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * (self.impulse.x * lever_b + self.impulse.y)
  apply_axial(
    body_a,
    body_b,
    axis,
    axis_lever_a,
    axis_lever_b,
    self.motor_impulse + self.limit_impulse,
  )
}

///|
pub fn PrismaticJoint::solve_velocity(
  self : PrismaticJoint,
  bodies : Array[Body],
) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let offset_a = self.local_anchor_a.rotate(body_a.rotation)
  let offset_b = self.local_anchor_b.rotate(body_b.rotation)
  let axis = self.local_axis_a.rotate(body_a.rotation)
  let normal = axis.perp()
  let point_a = body_a.position + offset_a
  let point_b = body_b.position + offset_b
  let arm_a = point_b - point_a + offset_a
  let lever_a = arm_a.cross(normal)
  let lever_b = offset_b.cross(normal)
  let dv = body_b.point_velocity(point_b) - body_a.point_velocity(point_a)
  let cdot = Vec2::new(
    dv.dot(normal),
    body_b.angular_velocity - body_a.angular_velocity,
  )
  let rhs = (cdot + self.bias).scale(-1.0)
  let d_impulse = solve22(self.k11, self.k12, self.k22, rhs)
  self.impulse = self.impulse + d_impulse
  let p = normal.scale(d_impulse.x)
  body_a.velocity = body_a.velocity - p.scale(body_a.inv_mass)
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * (d_impulse.x * lever_a + d_impulse.y)
  body_b.velocity = body_b.velocity + p.scale(body_b.inv_mass)
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * (d_impulse.x * lever_b + d_impulse.y)
  let axis_lever_a = arm_a.cross(axis)
  let axis_lever_b = offset_b.cross(axis)
  if self.active != 0 {
    let speed = (body_b.point_velocity(point_b) - body_a.point_velocity(point_a)).dot(
      axis,
    )
    let mut impulse = -self.axis_mass * speed
    let previous = self.limit_impulse
    self.limit_impulse = if self.active < 0 {
      (previous + impulse).max(0.0)
    } else {
      (previous + impulse).min(0.0)
    }
    impulse = self.limit_impulse - previous
    apply_axial(body_a, body_b, axis, axis_lever_a, axis_lever_b, impulse)
  }
  if self.motor_cap > 0.0 {
    let speed = (body_b.point_velocity(point_b) - body_a.point_velocity(point_a)).dot(
      axis,
    )
    let mut impulse = self.axis_mass * (self.motor_speed - speed)
    let previous = self.motor_impulse
    self.motor_impulse = (previous + impulse)
      .max(-self.motor_cap)
      .min(self.motor_cap)
    impulse = self.motor_impulse - previous
    apply_axial(body_a, body_b, axis, axis_lever_a, axis_lever_b, impulse)
  }
}

///|
/// Push the slider back inside its travel range without touching velocities.
pub fn PrismaticJoint::correct_positions(
  self : PrismaticJoint,
  bodies : Array[Body],
) -> Unit {
  if !self.has_limits {
    return
  }
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let offset_a = self.local_anchor_a.rotate(body_a.rotation)
  let offset_b = self.local_anchor_b.rotate(body_b.rotation)
  let axis = self.local_axis_a.rotate(body_a.rotation)
  let delta = body_b.position + offset_b - (body_a.position + offset_a)
  let translation = delta.dot(axis)
  let error = if translation > self.upper {
    translation - self.upper
  } else if translation < self.lower {
    translation - self.lower
  } else {
    return
  }
  let total = body_a.inv_mass + body_b.inv_mass
  if total <= K_EPSILON {
    return
  }
  let correction = -JOINT_POSITION_FACTOR * joint_error(error)
  let shift = axis.scale(correction / total)
  body_a.position = body_a.position - shift.scale(body_a.inv_mass)
  body_b.position = body_b.position + shift.scale(body_b.inv_mass)
}

///|
/// Keeps the relative rotation of two bodies inside `[lower, upper]`.
pub struct AngleLimitJoint {
  body_a : Int32
  body_b : Int32
  lower : Double
  upper : Double
  mut impulse : Double
  mut mass : Double
  mut active : Int32
  mut has_pivot : Bool
  mut local_pivot_b : Vec2
}

///|
pub fn AngleLimitJoint::new(
  body_a : Int32,
  body_b : Int32,
  lower : Double,
  upper : Double,
) -> AngleLimitJoint {
  {
    body_a,
    body_b,
    lower,
    upper,
    impulse: 0.0,
    mass: 0.0,
    active: 0,
    has_pivot: false,
    local_pivot_b: Vec2::zero(),
  }
}

///|
/// Angle stop that treats `anchor` as the pivot: the positional pass rotates the
/// bodies about that point, so a revolute joint sharing the same anchor is not
/// disturbed by the correction.
pub fn AngleLimitJoint::at(
  world : World,
  body_a : Int32,
  body_b : Int32,
  anchor : Vec2,
  lower : Double,
  upper : Double,
) -> AngleLimitJoint {
  let joint = AngleLimitJoint::new(body_a, body_b, lower, upper)
  joint.has_pivot = true
  joint.local_pivot_b = world.body(body_b).world_to_local(anchor)
  joint
}

///|
pub fn AngleLimitJoint::pre_step(
  self : AngleLimitJoint,
  bodies : Array[Body],
) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let k = body_a.inv_inertia + body_b.inv_inertia
  self.mass = if k > K_EPSILON { 1.0 / k } else { 0.0 }
  let angle = body_b.rotation - body_a.rotation
  let mut side = 0
  if angle < self.lower {
    side = -1
  } else if angle > self.upper {
    side = 1
  }
  if side == 0 {
    // Still touching the stop: hold the relative rotation instead of bouncing.
    if self.active != 0 && self.impulse != 0.0 {
      side = self.active
    } else {
      self.impulse = 0.0
    }
  }
  self.active = side
  if self.active != 0 {
    body_a.angular_velocity = body_a.angular_velocity -
      body_a.inv_inertia * self.impulse
    body_b.angular_velocity = body_b.angular_velocity +
      body_b.inv_inertia * self.impulse
  }
}

///|
pub fn AngleLimitJoint::solve_velocity(
  self : AngleLimitJoint,
  bodies : Array[Body],
) -> Unit {
  if self.active == 0 {
    return
  }
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let cdot = body_b.angular_velocity - body_a.angular_velocity
  let mut impulse = -self.mass * cdot
  let previous = self.impulse
  self.impulse = if self.active < 0 {
    (previous + impulse).max(0.0)
  } else {
    (previous + impulse).min(0.0)
  }
  impulse = self.impulse - previous
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * impulse
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * impulse
}

///|
/// Rotate the bodies back inside the allowed range without touching velocities.
pub fn AngleLimitJoint::correct_positions(
  self : AngleLimitJoint,
  bodies : Array[Body],
) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let angle = body_b.rotation - body_a.rotation
  let error = if angle > self.upper {
    angle - self.upper
  } else if angle < self.lower {
    angle - self.lower
  } else {
    return
  }
  let total = body_a.inv_inertia + body_b.inv_inertia
  if total <= K_EPSILON {
    return
  }
  let correction = -JOINT_POSITION_FACTOR * joint_error(error) / total
  let pivot = body_b.position + self.local_pivot_b.rotate(body_b.rotation)
  rotate_about(body_a, pivot, -correction * body_a.inv_inertia)
  rotate_about(body_b, pivot, correction * body_b.inv_inertia)
}

///|
/// Rotate a body by `angle` around a world point, keeping the point fixed.
fn rotate_about(body : Body, pivot : Vec2, angle : Double) -> Unit {
  body.rotation = body.rotation + angle
  let offset = body.position - pivot
  body.position = pivot + offset.rotate(angle)
}

///|
/// Angular motor: drives the relative angular velocity towards `target_speed`
/// with a bounded torque.
pub struct MotorJoint {
  body_a : Int32
  body_b : Int32
  target_speed : Double
  max_torque : Double
  mut impulse : Double
  mut mass : Double
  mut cap : Double
}

///|
pub fn MotorJoint::new(
  body_a : Int32,
  body_b : Int32,
  target_speed : Double,
  max_torque : Double,
) -> MotorJoint {
  {
    body_a,
    body_b,
    target_speed,
    max_torque,
    impulse: 0.0,
    mass: 0.0,
    cap: 0.0,
  }
}

///|
pub fn MotorJoint::pre_step(
  self : MotorJoint,
  bodies : Array[Body],
  inv_dt : Double,
) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let k = body_a.inv_inertia + body_b.inv_inertia
  self.mass = if k > K_EPSILON { 1.0 / k } else { 0.0 }
  self.cap = self.max_torque.max(0.0) / inv_dt
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * self.impulse
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * self.impulse
}

///|
pub fn MotorJoint::solve_velocity(
  self : MotorJoint,
  bodies : Array[Body],
) -> Unit {
  if self.cap <= 0.0 || self.mass <= 0.0 {
    return
  }
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let cdot = body_b.angular_velocity - body_a.angular_velocity
  let mut impulse = self.mass * (self.target_speed - cdot)
  let previous = self.impulse
  self.impulse = (previous + impulse).max(-self.cap).min(self.cap)
  impulse = self.impulse - previous
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * impulse
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * impulse
}
