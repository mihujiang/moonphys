///|
/// Linear speed below which a body may fall asleep.
const SLEEP_LINEAR_TOLERANCE : Double = 0.05

///|
/// Angular speed below which a body may fall asleep.
const SLEEP_ANGULAR_TOLERANCE : Double = 0.05

///|
/// Seconds a body must stay slow before it sleeps.
const TIME_TO_SLEEP : Double = 0.5

///|
/// Upper bound on linear speed, to keep a bad tuning from exploding.
const MAX_LINEAR_VELOCITY : Double = 100.0

///|
/// Immutable view of a body, used for rendering and for regression tests.
pub struct BodySnapshot {
  position : Vec2
  rotation : Double
  shape : Shape
  asleep : Bool
  user_data : Int32
}

///|
pub fn BodySnapshot::position(self : BodySnapshot) -> Vec2 {
  self.position
}

///|
pub fn BodySnapshot::rotation(self : BodySnapshot) -> Double {
  self.rotation
}

///|
pub fn BodySnapshot::shape(self : BodySnapshot) -> Shape {
  self.shape
}

///|
pub fn BodySnapshot::asleep(self : BodySnapshot) -> Bool {
  self.asleep
}

///|
pub fn BodySnapshot::user_data(self : BodySnapshot) -> Int32 {
  self.user_data
}

///|
/// The simulation world: bodies, persistent contacts and joints.
pub struct World {
  bodies : Array[Body]
  mut arbiters : Array[Arbiter]
  joints : Array[Joint]
  mut gravity : Vec2
  mut velocity_iterations : Int32
  mut time : Double
  mut sleep_enabled : Bool
  /// Bodies sorted by `aabb.min.x`, carried over between steps so the
  /// broadphase only has to re-sort a nearly sorted array.
  sorted : Array[Int32]
  /// Reused pair buffer: one allocation for the lifetime of the world.
  pairs : Array[(Int32, Int32)]
  /// `arbiters` grouped by their lower body index, rebuilt every step so the
  /// persistent-contact lookup is O(1) per body instead of a linear scan.
  arbiter_buckets : Array[Array[Arbiter]]
}

///|
pub fn World::new() -> World {
  {
    bodies: [],
    arbiters: [],
    joints: [],
    gravity: Vec2::new(0.0, -9.81),
    velocity_iterations: 10,
    time: 0.0,
    sleep_enabled: true,
    sorted: [],
    pairs: [],
    arbiter_buckets: [],
  }
}

///|
pub fn World::with_gravity(gx : Double, gy : Double) -> World {
  let world = World::new()
  world.gravity = Vec2::new(gx, gy)
  world
}

///|
/// Add a body and return its index, which joints and arbiters refer to.
pub fn World::add_body(self : World, body : Body) -> Int32 {
  let index = self.bodies.length()
  body.sync_aabb()
  self.bodies.push(body)
  index
}

///|
pub fn World::add_joint(self : World, joint : Joint) -> Unit {
  self.joints.push(joint)
}

///|
/// Connect the centers of two bodies with a rigid distance joint.
pub fn World::add_distance_joint(
  self : World,
  body_a : Int32,
  body_b : Int32,
) -> Unit {
  self.add_joint(Joint::Distance(DistanceJoint::between(self, body_a, body_b)))
}

///|
/// Pin two bodies together at a world-space anchor (a hinge).
pub fn World::add_revolve_joint(
  self : World,
  body_a : Int32,
  body_b : Int32,
  anchor : Vec2,
) -> Unit {
  self.add_joint(Joint::Revolve(RevolveJoint::at(self, body_a, body_b, anchor)))
}

///|
/// Pin two bodies together, using one local anchor per body.
pub fn World::add_revolve_joint_local(
  self : World,
  body_a : Int32,
  body_b : Int32,
  local_anchor_a : Vec2,
  local_anchor_b : Vec2,
) -> Unit {
  self.add_joint(
    Joint::Revolve(
      RevolveJoint::new(body_a, body_b, local_anchor_a, local_anchor_b),
    ),
  )
}

///|
/// Attach a slider between two bodies. The returned joint is the live object
/// stored in the world, so limits and a motor can be configured in place:
///
/// ```moonbit nocheck
/// let slider = world.add_prismatic_joint(rail, cart, anchor, Vec2::new(1.0, 0.0))
/// slider.with_limits(-3.0, 3.0)
/// slider.with_motor(1.5, 40.0)
/// ```
pub fn World::add_prismatic_joint(
  self : World,
  body_a : Int32,
  body_b : Int32,
  anchor : Vec2,
  axis : Vec2,
) -> PrismaticJoint {
  let joint = PrismaticJoint::at(self, body_a, body_b, anchor, axis)
  self.add_joint(Joint::Prismatic(joint))
  joint
}

///|
/// Keep the relative rotation of two bodies between `lower` and `upper`.
pub fn World::add_angle_limit_joint(
  self : World,
  body_a : Int32,
  body_b : Int32,
  lower : Double,
  upper : Double,
) -> AngleLimitJoint {
  let joint = AngleLimitJoint::new(body_a, body_b, lower, upper)
  self.add_joint(Joint::AngleLimit(joint))
  joint
}

///|
/// Angle stop whose corrections pivot around `anchor`, which is what you want
/// when the same anchor also carries a revolute joint.
pub fn World::add_angle_limit_joint_at(
  self : World,
  body_a : Int32,
  body_b : Int32,
  anchor : Vec2,
  lower : Double,
  upper : Double,
) -> AngleLimitJoint {
  let joint = AngleLimitJoint::at(self, body_a, body_b, anchor, lower, upper)
  self.add_joint(Joint::AngleLimit(joint))
  joint
}

///|
/// Drive the relative angular velocity of two bodies towards `target_speed`
/// with at most `max_torque`.
pub fn World::add_motor_joint(
  self : World,
  body_a : Int32,
  body_b : Int32,
  target_speed : Double,
  max_torque : Double,
) -> MotorJoint {
  let joint = MotorJoint::new(body_a, body_b, target_speed, max_torque)
  self.add_joint(Joint::Motor(joint))
  joint
}

///|
/// A distance joint whose stiffness is finite, for ropes and springs.
pub fn World::add_soft_distance_joint(
  self : World,
  body_a : Int32,
  body_b : Int32,
  frequency : Double,
  damping_ratio : Double,
) -> Unit {
  let joint = DistanceJoint::between(self, body_a, body_b)
  ignore(joint.with_softness(frequency, damping_ratio))
  self.add_joint(Joint::Distance(joint))
}

///|
pub fn World::joint_count(self : World) -> Int32 {
  self.joints.length()
}

///|
pub fn World::body(self : World, index : Int32) -> Body {
  self.bodies[index]
}

///|
pub fn World::body_count(self : World) -> Int32 {
  self.bodies.length()
}

///|
pub fn World::arbiter_count(self : World) -> Int32 {
  self.arbiters.length()
}

///|
pub fn World::contact_count(self : World) -> Int32 {
  let mut total = 0
  for arbiter in self.arbiters {
    total = total + arbiter.contacts().length()
  }
  total
}

///|
/// Bit-exact digest of the simulated state: every position, rotation and
/// velocity is folded in as raw bits, so two runs that diverge by one ulp
/// produce different digests.
///
/// This is what makes determinism testable: the same scene has to print the same
/// digest on `native`, `js` and `wasm-gc`, because the engine uses no randomness
/// and no hash-ordered iteration.
pub fn World::state_hash(self : World) -> UInt64 {
  let mut hash : UInt64 = 0xcbf29ce484222325UL
  for body in self.bodies {
    hash = mix_state(hash, body.position.x)
    hash = mix_state(hash, body.position.y)
    hash = mix_state(hash, body.rotation)
    hash = mix_state(hash, body.velocity.x)
    hash = mix_state(hash, body.velocity.y)
    hash = mix_state(hash, body.angular_velocity)
  }
  hash
}

///|
/// FNV-1a style mixing: xor the raw double bits in, then multiply.
fn mix_state(hash : UInt64, value : Double) -> UInt64 {
  (hash ^ value.reinterpret_as_uint64()) * 0x100000001b3UL
}

///|
pub fn World::gravity(self : World) -> Vec2 {
  self.gravity
}

///|
pub fn World::set_gravity(self : World, gravity : Vec2) -> Unit {
  self.gravity = gravity
  for body in self.bodies {
    body.wake()
  }
}

///|
pub fn World::set_velocity_iterations(self : World, iterations : Int32) -> Unit {
  self.velocity_iterations = iterations
}

///|
pub fn World::set_sleep_enabled(self : World, enabled : Bool) -> Unit {
  self.sleep_enabled = enabled
}

///|
pub fn World::time(self : World) -> Double {
  self.time
}

///|
/// Bodies paired by the broadphase, in a deterministic order.
///
/// Sorting by the lower AABB bound and sweeping is a sort-and-sweep
/// broadphase: linear for sparse scenes, and stable across runs because ties
/// are broken by body index.
fn World::compute_pairs(self : World) -> Array[(Int32, Int32)] {
  let n = self.bodies.length()
  let order = self.sorted
  // Bodies are only ever appended, so the cached order needs at most a tail
  // extension; nothing has to be rebuilt from scratch.
  while order.length() > n {
    ignore(order.pop())
  }
  while order.length() < n {
    order.push(order.length())
  }
  // Incremental insertion sort. The array was sorted by the previous step and
  // bodies move by a fraction of their size per step, so almost every element
  // is already in place and the total work stays O(n).
  let count = order.length()
  for i in 1..<count {
    let cur = order[i]
    let cur_x = self.bodies[cur].world_aabb.min.x
    let mut j = i
    while j > 0 {
      let prev = order[j - 1]
      let prev_x = self.bodies[prev].world_aabb.min.x
      if cur_x > prev_x || (cur_x == prev_x && cur > prev) {
        break
      }
      order[j] = prev
      j = j - 1
    }
    order[j] = cur
  }
  let pairs = self.pairs
  pairs.clear()
  for a_index in 0..<count {
    let i = order[a_index]
    let box_a = self.bodies[i].world_aabb
    let mut b_index = a_index + 1
    while b_index < count {
      let j = order[b_index]
      let box_b = self.bodies[j].world_aabb
      if box_b.min.x > box_a.max.x {
        break
      }
      let (lo, hi) = if i < j { (i, j) } else { (j, i) }
      let body_lo = self.bodies[lo]
      let body_hi = self.bodies[hi]
      if body_lo.is_dynamic() || body_hi.is_dynamic() {
        if box_a.overlaps(box_b) {
          pairs.push((lo, hi))
        }
      }
      b_index = b_index + 1
    }
  }
  pairs
}

///|
/// Index the previous step's arbiters by their lower body index, so looking up
/// the arbiter of a pair is a short scan of one bucket instead of a scan over
/// every arbiter in the world.
fn World::index_arbiters(self : World) -> Unit {
  let buckets = self.arbiter_buckets
  while buckets.length() < self.bodies.length() {
    buckets.push([])
  }
  for bucket in buckets {
    bucket.clear()
  }
  for arbiter in self.arbiters {
    buckets[arbiter.body_a()].push(arbiter)
  }
}

///|
/// The arbiter that carried `(index_a, index_b)` on the previous step, if any.
fn World::find_arbiter(
  self : World,
  index_a : Int32,
  index_b : Int32,
) -> Arbiter? {
  for arbiter in self.arbiter_buckets[index_a] {
    if arbiter.body_b() == index_b {
      return Some(arbiter)
    }
  }
  None
}

///|
/// Advance the simulation by `dt` seconds.
pub fn World::step(self : World, dt : Double) -> Unit {
  if dt <= 0.0 {
    return
  }
  let inv_dt = 1.0 / dt
  for body in self.bodies {
    body.sync_aabb()
  }
  let pairs = self.compute_pairs()
  self.index_arbiters()
  let next_arbiters : Array[Arbiter] = []
  for pair in pairs {
    let (index_a, index_b) = pair
    let body_a = self.bodies[index_a]
    let body_b = self.bodies[index_b]
    let points = collide(
      body_a.shape,
      body_a.transform(),
      body_b.shape,
      body_b.transform(),
    )
    if points.length() > 0 {
      let friction = (body_a.friction * body_b.friction).sqrt()
      let restitution = body_a.restitution.max(body_b.restitution)
      match self.find_arbiter(index_a, index_b) {
        Some(arbiter) => {
          arbiter.update(points, friction, restitution)
          next_arbiters.push(arbiter)
        }
        None =>
          next_arbiters.push(
            Arbiter::new(index_a, index_b, points, friction, restitution),
          )
      }
      if body_a.is_dynamic() &&
        body_a.awake() &&
        body_b.is_dynamic() &&
        !body_b.awake() {
        body_b.wake()
      }
      if body_b.is_dynamic() &&
        body_b.awake() &&
        body_a.is_dynamic() &&
        !body_a.awake() {
        body_a.wake()
      }
    }
  }
  self.arbiters = next_arbiters
  for body in self.bodies {
    if body.is_dynamic() && body.awake() {
      let acceleration = self.gravity + body.force.scale(body.inv_mass)
      body.velocity = body.velocity + acceleration.scale(dt)
      body.angular_velocity = body.angular_velocity +
        body.inv_inertia * body.torque * dt
      if body.linear_damping > 0.0 {
        body.velocity = body.velocity.scale(
          1.0 / (1.0 + dt * body.linear_damping),
        )
      }
      if body.angular_damping > 0.0 {
        body.angular_velocity = body.angular_velocity /
          (1.0 + dt * body.angular_damping)
      }
      let speed = body.velocity.length()
      if speed > MAX_LINEAR_VELOCITY {
        body.velocity = body.velocity.scale(MAX_LINEAR_VELOCITY / speed)
      }
    }
    body.clear_forces()
  }
  for arbiter in self.arbiters {
    arbiter.pre_step(self.bodies, inv_dt)
  }
  for joint in self.joints {
    joint.pre_step(self.bodies, inv_dt)
  }
  for _iteration in 0..<self.velocity_iterations {
    for arbiter in self.arbiters {
      arbiter.solve_velocity(self.bodies)
    }
    for joint in self.joints {
      joint.solve_velocity(self.bodies)
    }
  }
  for body in self.bodies {
    if body.is_dynamic() && body.awake() {
      body.position = body.position + body.velocity.scale(dt)
      body.rotation = body.rotation + body.angular_velocity * dt
    }
  }
  for joint in self.joints {
    joint.correct_positions(self.bodies)
  }
  if self.sleep_enabled {
    for body in self.bodies {
      if body.is_dynamic() && body.awake() && body.allow_sleep {
        let slow = body.velocity.length() < SLEEP_LINEAR_TOLERANCE &&
          body.angular_velocity.abs() < SLEEP_ANGULAR_TOLERANCE
        if slow {
          body.sleep_time = body.sleep_time + dt
          if body.sleep_time > TIME_TO_SLEEP {
            body.sleep()
          }
        } else {
          body.sleep_time = 0.0
        }
      }
    }
  }
  for body in self.bodies {
    body.sync_aabb()
  }
  self.time = self.time + dt
}

///|
/// Advance by `steps` fixed-size steps.
pub fn World::run(self : World, steps : Int32, dt : Double) -> Unit {
  for _step in 0..<steps {
    self.step(dt)
  }
}

///|
pub fn World::snapshot(self : World) -> Array[BodySnapshot] {
  let out : Array[BodySnapshot] = []
  for body in self.bodies {
    out.push({
      position: body.position,
      rotation: body.rotation,
      shape: body.shape,
      asleep: !body.awake(),
      user_data: body.user_data,
    })
  }
  out
}

///|
/// Sum of the kinetic energy of all bodies; the solver must never increase it
/// in a purely dissipative scene.
pub fn World::total_kinetic_energy(self : World) -> Double {
  let mut total = 0.0
  for body in self.bodies {
    total = total + body.kinetic_energy()
  }
  total
}

///|
/// Wake every dynamic body.
pub fn World::wake_all(self : World) -> Unit {
  for body in self.bodies {
    body.wake()
  }
}
