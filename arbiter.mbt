///|
/// Baumgarte bias factor: how aggressively penetration is pushed out.
const BIAS_FACTOR : Double = 0.2

///|
/// Relative normal speed above which restitution is applied.
const RESTITUTION_THRESHOLD : Double = 1.0

///|
/// Farthest penetration the position bias tries to remove in one step, in
/// world units. Without this cap a deeply overlapping pair is launched apart.
const MAX_CORRECTION : Double = 0.2

///|
/// Persistent contact constraints between two bodies.
///
/// Impulses are accumulated and clamped, and are carried over to the next
/// frame by matching contact feature ids, which is what makes stacks stable.
pub struct Arbiter {
  body_a : Int32
  body_b : Int32
  mut contacts : Array[ContactPoint]
  mut normal_impulses : Array[Double]
  mut tangent_impulses : Array[Double]
  mut mass_normal : Array[Double]
  mut mass_tangent : Array[Double]
  mut bias : Array[Double]
  mut restitution_bias : Array[Double]
  mut friction : Double
  mut restitution : Double
}

///|
/// Create an arbiter for a body pair from freshly detected contact points.
pub fn Arbiter::new(
  body_a : Int32,
  body_b : Int32,
  contacts : Array[ContactPoint],
  friction : Double,
  restitution : Double,
) -> Arbiter {
  let n = contacts.length()
  {
    body_a,
    body_b,
    contacts,
    normal_impulses: Array::make(n, 0.0),
    tangent_impulses: Array::make(n, 0.0),
    mass_normal: Array::make(n, 0.0),
    mass_tangent: Array::make(n, 0.0),
    bias: Array::make(n, 0.0),
    restitution_bias: Array::make(n, 0.0),
    friction,
    restitution,
  }
}

///|
/// Index of the first body: the lower index of the pair.
pub fn Arbiter::body_a(self : Arbiter) -> Int32 {
  self.body_a
}

///|
/// Index of the second body: the higher index of the pair.
pub fn Arbiter::body_b(self : Arbiter) -> Int32 {
  self.body_b
}

///|
/// Contact points of this pair, each carrying its warm-started impulse.
pub fn Arbiter::contacts(self : Arbiter) -> Array[ContactPoint] {
  self.contacts
}

///|
/// Whether the last update produced at least one contact point.
pub fn Arbiter::is_touching(self : Arbiter) -> Bool {
  self.contacts.length() > 0
}

///|
/// Refresh the manifold, keeping impulses of surviving points.
pub fn Arbiter::update(
  self : Arbiter,
  contacts : Array[ContactPoint],
  friction : Double,
  restitution : Double,
) -> Unit {
  let n = contacts.length()
  let normal_impulses = Array::make(n, 0.0)
  let tangent_impulses = Array::make(n, 0.0)
  for i in 0..<n {
    let id = contacts[i].feature_id()
    let old = self.contacts.length()
    for j in 0..<old {
      if self.contacts[j].feature_id() == id {
        normal_impulses[i] = self.normal_impulses[j]
        tangent_impulses[i] = self.tangent_impulses[j]
        break
      }
    }
  }
  self.contacts = contacts
  self.normal_impulses = normal_impulses
  self.tangent_impulses = tangent_impulses
  self.mass_normal = Array::make(n, 0.0)
  self.mass_tangent = Array::make(n, 0.0)
  self.bias = Array::make(n, 0.0)
  self.restitution_bias = Array::make(n, 0.0)
  self.friction = friction
  self.restitution = restitution
}

///|
/// Apply an impulse pair to two bodies.
fn apply_impulse_pair(
  body_a : Body,
  body_b : Body,
  r_a : Vec2,
  r_b : Vec2,
  impulse : Vec2,
) -> Unit {
  body_a.velocity = body_a.velocity - impulse.scale(body_a.inv_mass)
  body_a.angular_velocity = body_a.angular_velocity -
    body_a.inv_inertia * r_a.cross(impulse)
  body_b.velocity = body_b.velocity + impulse.scale(body_b.inv_mass)
  body_b.angular_velocity = body_b.angular_velocity +
    body_b.inv_inertia * r_b.cross(impulse)
}

///|
/// Compute effective masses and bias terms, then warm start.
pub fn Arbiter::pre_step(
  self : Arbiter,
  bodies : Array[Body],
  inv_dt : Double,
) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let n = self.contacts.length()
  for i in 0..<n {
    let contact = self.contacts[i]
    let normal = contact.normal
    let r_a = contact.position - body_a.position
    let r_b = contact.position - body_b.position
    let rn_a = r_a.cross(normal)
    let rn_b = r_b.cross(normal)
    let k_normal = body_a.inv_mass +
      body_b.inv_mass +
      body_a.inv_inertia * rn_a * rn_a +
      body_b.inv_inertia * rn_b * rn_b
    self.mass_normal[i] = if k_normal > K_EPSILON {
      1.0 / k_normal
    } else {
      0.0
    }
    let tangent = normal.perp()
    let rt_a = r_a.cross(tangent)
    let rt_b = r_b.cross(tangent)
    let k_tangent = body_a.inv_mass +
      body_b.inv_mass +
      body_a.inv_inertia * rt_a * rt_a +
      body_b.inv_inertia * rt_b * rt_b
    self.mass_tangent[i] = if k_tangent > K_EPSILON {
      1.0 / k_tangent
    } else {
      0.0
    }
    let correction = (contact.separation + LINEAR_SLOP)
      .max(-MAX_CORRECTION)
      .min(0.0)
    self.bias[i] = -BIAS_FACTOR * inv_dt * correction
    let relative = body_b.point_velocity(contact.position) -
      body_a.point_velocity(contact.position)
    let vn = relative.dot(normal)
    self.restitution_bias[i] = if vn < -RESTITUTION_THRESHOLD {
      -self.restitution * vn
    } else {
      0.0
    }
  }
  for i in 0..<n {
    let contact = self.contacts[i]
    let r_a = contact.position - body_a.position
    let r_b = contact.position - body_b.position
    let impulse = contact.normal.scale(self.normal_impulses[i]) +
      contact.normal.perp().scale(self.tangent_impulses[i])
    apply_impulse_pair(body_a, body_b, r_a, r_b, impulse)
  }
}

///|
/// Run one sequential impulse iteration.
pub fn Arbiter::solve_velocity(self : Arbiter, bodies : Array[Body]) -> Unit {
  let body_a = bodies[self.body_a]
  let body_b = bodies[self.body_b]
  let n = self.contacts.length()
  for i in 0..<n {
    let contact = self.contacts[i]
    let normal = contact.normal
    let tangent = normal.perp()
    let r_a = contact.position - body_a.position
    let r_b = contact.position - body_b.position
    let dv = body_b.point_velocity(contact.position) -
      body_a.point_velocity(contact.position)
    let vn = dv.dot(normal)
    let mut dpn = self.mass_normal[i] *
      (-vn + self.bias[i] + self.restitution_bias[i])
    let old_normal = self.normal_impulses[i]
    let new_normal = (old_normal + dpn).max(0.0)
    dpn = new_normal - old_normal
    self.normal_impulses[i] = new_normal
    apply_impulse_pair(body_a, body_b, r_a, r_b, normal.scale(dpn))
    let dv2 = body_b.point_velocity(contact.position) -
      body_a.point_velocity(contact.position)
    let vt = dv2.dot(tangent)
    let mut dpt = self.mass_tangent[i] * -vt
    let max_friction = self.friction * new_normal
    let old_tangent = self.tangent_impulses[i]
    let new_tangent = clamp(old_tangent + dpt, -max_friction, max_friction)
    dpt = new_tangent - old_tangent
    self.tangent_impulses[i] = new_tangent
    apply_impulse_pair(body_a, body_b, r_a, r_b, tangent.scale(dpt))
  }
}

///|
/// Total normal impulse, useful to detect resting stability in tests.
pub fn Arbiter::total_normal_impulse(self : Arbiter) -> Double {
  let mut sum = 0.0
  for i in 0..<self.normal_impulses.length() {
    sum = sum + self.normal_impulses[i]
  }
  sum
}
