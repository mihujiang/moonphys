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
  user_data : Int
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
pub fn BodySnapshot::user_data(self : BodySnapshot) -> Int {
  self.user_data
}

///|
/// The simulation world: bodies, persistent contacts and joints.
pub struct World {
  bodies : Array[Body]
  mut arbiters : Array[Arbiter]
  joints : Array[Joint]
  mut gravity : Vec2
  mut velocity_iterations : Int
  mut time : Double
  mut sleep_enabled : Bool
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
pub fn World::add_body(self : World, body : Body) -> Int {
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
  body_a : Int,
  body_b : Int,
) -> Unit {
  self.add_joint(Joint::Distance(DistanceJoint::between(self, body_a, body_b)))
}

///|
/// Pin two bodies together at a world-space anchor (a hinge).
pub fn World::add_revolve_joint(
  self : World,
  body_a : Int,
  body_b : Int,
  anchor : Vec2,
) -> Unit {
  self.add_joint(Joint::Revolve(RevolveJoint::at(self, body_a, body_b, anchor)))
}

///|
/// Pin two bodies together, using one local anchor per body.
pub fn World::add_revolve_joint_local(
  self : World,
  body_a : Int,
  body_b : Int,
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
pub fn World::body(self : World, index : Int) -> Body {
  self.bodies[index]
}

///|
pub fn World::body_count(self : World) -> Int {
  self.bodies.length()
}

///|
pub fn World::arbiter_count(self : World) -> Int {
  self.arbiters.length()
}

///|
pub fn World::contact_count(self : World) -> Int {
  let mut total = 0
  for arbiter in self.arbiters {
    total = total + arbiter.contacts().length()
  }
  total
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
pub fn World::set_velocity_iterations(self : World, iterations : Int) -> Unit {
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
fn World::compute_pairs(self : World) -> Array[(Int, Int)] {
  let n = self.bodies.length()
  let order : Array[Int] = []
  for i in 0..<n {
    order.push(i)
  }
  for i in 1..<n {
    let mut j = i
    while j > 0 {
      let cur = order[j]
      let prev = order[j - 1]
      let cur_x = self.bodies[cur].world_aabb.min.x
      let prev_x = self.bodies[prev].world_aabb.min.x
      if cur_x > prev_x || (cur_x == prev_x && cur > prev) {
        break
      }
      order[j] = prev
      order[j - 1] = cur
      j = j - 1
    }
  }
  let pairs : Array[(Int, Int)] = []
  for a_index in 0..<n {
    let i = order[a_index]
    let box_a = self.bodies[i].world_aabb
    let mut b_index = a_index + 1
    while b_index < n {
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
      let mut reused = false
      for arbiter in self.arbiters {
        if arbiter.body_a() == index_a && arbiter.body_b() == index_b {
          arbiter.update(points, friction, restitution)
          next_arbiters.push(arbiter)
          reused = true
          break
        }
      }
      if !reused {
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
pub fn World::run(self : World, steps : Int, dt : Double) -> Unit {
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
