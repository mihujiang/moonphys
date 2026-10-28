///|
/// Binary state snapshot, used by rollback netcode: every body's full motion
/// state, the persistent contact impulses (warm starting), and the world clock.
///
/// Layout (little endian):
///
/// ```text
/// magic   u32 = 0x4D504859 ("MPHY")
/// version u32 = 1
/// count   u32
/// time    f64
/// body*   { type u8, shape, position f64x2, rotation f64, velocity f64x2,
///           angular_velocity f64, force f64x2, torque f64, mass f64,
///           inv_mass f64, inertia f64, inv_inertia f64, friction f64,
///           restitution f64, linear_damping f64, angular_damping f64,
///           allow_sleep u8, awake u8, sleep_time f64, user_data i32 }
/// arb*    { body_a u32, body_b u32, contacts, impulses, bias... }
/// hash    u64  (FNV-1a over everything above)
/// ```
///
/// Joints are deliberately not serialized: they carry motor/solver state that is
/// not worth the format surface, so `load_state` refuses a world that has any.
const STATE_MAGIC : Int = 0x4D504859

///|
/// Snapshot layout version; bump on any change to the field order below.
const STATE_VERSION : Int = 1

///|
/// FNV-1a 64 offset basis, same constants as `state_hash`.
const STATE_FNV_BASIS : UInt64 = 0xcbf29ce484222325UL

///|
/// FNV-1a 64 prime.
const STATE_FNV_PRIME : UInt64 = 0x100000001b3UL

///|
/// Upper bound on bodies per snapshot; anything above is treated as corruption.
const STATE_MAX_BODIES : Int = 4096

///|
/// Upper bound on contact manifolds per snapshot.
const STATE_MAX_ARBITERS : Int = 8192

///|
/// Upper bound on contact points in one manifold.
const STATE_MAX_CONTACTS : Int = 16

///|
fn put_u8(out : Array[Byte], value : Int) -> Unit {
  out.push((value & 0xFF).to_byte())
}

///|
fn put_u32(out : Array[Byte], value : Int) -> Unit {
  put_u8(out, value)
  put_u8(out, value >> 8)
  put_u8(out, value >> 16)
  put_u8(out, value >> 24)
}

///|
fn put_u64(out : Array[Byte], value : UInt64) -> Unit {
  let mut i = 0
  while i < 8 {
    put_u8(out, ((value >> (i * 8)) & 0xFFUL).to_int())
    i = i + 1
  }
}

///|
fn put_f64(out : Array[Byte], value : Double) -> Unit {
  put_u64(out, value.reinterpret_as_uint64())
}

///|
fn put_doubles(out : Array[Byte], values : Array[Double]) -> Unit {
  put_u32(out, values.length())
  for value in values {
    put_f64(out, value)
  }
}

///|
fn put_vec2(out : Array[Byte], value : Vec2) -> Unit {
  put_f64(out, value.x)
  put_f64(out, value.y)
}

///|
fn state_hash_of(out : Array[Byte]) -> UInt64 {
  let mut hash = STATE_FNV_BASIS
  for byte in out {
    hash = (hash ^ byte.to_int().to_uint64()) * STATE_FNV_PRIME
  }
  hash
}

///|
/// Cursor over a snapshot. Reading past the end yields zeros and keeps counting,
/// so a truncated buffer fails the final length check instead of panicking.
priv struct StateReader {
  buf : Bytes
  mut pos : Int
}

///|
fn StateReader::new(buf : Bytes) -> StateReader {
  { buf, pos: 0 }
}

///|
fn StateReader::remaining(self : StateReader) -> Int {
  self.buf.length() - self.pos
}

///|
/// Mark the buffer as invalid; every later check fails.
fn StateReader::poison(self : StateReader) -> Unit {
  self.pos = self.buf.length() + 1
}

///|
fn StateReader::u8(self : StateReader) -> Int {
  let value = if self.pos >= 0 && self.pos < self.buf.length() {
    self.buf[self.pos].to_int() & 0xFF
  } else {
    0
  }
  self.pos = self.pos + 1
  value
}

///|
fn StateReader::u32(self : StateReader) -> Int {
  let b0 = self.u8()
  let b1 = self.u8()
  let b2 = self.u8()
  let b3 = self.u8()
  b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
}

///|
fn StateReader::u64(self : StateReader) -> UInt64 {
  let mut value = 0UL
  let mut i = 0
  while i < 8 {
    value = value | (self.u8().to_uint64() << (i * 8))
    i = i + 1
  }
  value
}

///|
fn StateReader::f64(self : StateReader) -> Double {
  self.u64().reinterpret_as_double()
}

///|
fn StateReader::vec2(self : StateReader) -> Vec2 {
  let x = self.f64()
  let y = self.f64()
  Vec2::new(x, y)
}

///|
fn StateReader::doubles(self : StateReader) -> Array[Double] {
  let count = self.u32()
  let out : Array[Double] = []
  if count < 0 || count > self.remaining() / 8 {
    self.poison()
    return out
  }
  let mut i = 0
  while i < count {
    out.push(self.f64())
    i = i + 1
  }
  out
}

///|
fn encode_state_body(out : Array[Byte], body : Body) -> Unit {
  match body.body_type {
    BodyType::Static => put_u8(out, 0)
    BodyType::Dynamic => put_u8(out, 1)
  }
  match body.shape {
    Shape::Circle(radius) => {
      put_u8(out, 0)
      put_f64(out, radius)
    }
    Shape::Polygon(vertices) => {
      put_u8(out, 1)
      put_u32(out, vertices.length())
      for vertex in vertices {
        put_vec2(out, vertex)
      }
    }
  }
  put_vec2(out, body.position)
  put_f64(out, body.rotation)
  put_vec2(out, body.velocity)
  put_f64(out, body.angular_velocity)
  put_vec2(out, body.force)
  put_f64(out, body.torque)
  put_f64(out, body.mass)
  put_f64(out, body.inv_mass)
  put_f64(out, body.inertia)
  put_f64(out, body.inv_inertia)
  put_f64(out, body.friction)
  put_f64(out, body.restitution)
  put_f64(out, body.linear_damping)
  put_f64(out, body.angular_damping)
  put_u8(out, if body.allow_sleep { 1 } else { 0 })
  put_u8(out, if body.awake { 1 } else { 0 })
  put_f64(out, body.sleep_time)
  put_u32(out, body.user_data)
}

///|
fn decode_state_body(reader : StateReader) -> Body {
  let body_type = if reader.u8() == 1 {
    BodyType::Dynamic
  } else {
    BodyType::Static
  }
  let shape = if reader.u8() == 0 {
    Shape::Circle(reader.f64())
  } else {
    let count = reader.u32()
    // A convex hull needs at least three vertices; refuse anything absurd.
    if count < 3 || count > 64 || count > reader.remaining() / 16 {
      reader.poison()
      Shape::Circle(0.5)
    } else {
      let vertices : Array[Vec2] = []
      let mut i = 0
      while i < count {
        vertices.push(reader.vec2())
        i = i + 1
      }
      Shape::Polygon(vertices)
    }
  }
  let body : Body = {
    body_type,
    shape,
    position: reader.vec2(),
    rotation: reader.f64(),
    velocity: reader.vec2(),
    angular_velocity: reader.f64(),
    force: reader.vec2(),
    torque: reader.f64(),
    mass: reader.f64(),
    inv_mass: reader.f64(),
    inertia: reader.f64(),
    inv_inertia: reader.f64(),
    friction: reader.f64(),
    restitution: reader.f64(),
    linear_damping: reader.f64(),
    angular_damping: reader.f64(),
    allow_sleep: reader.u8() == 1,
    awake: reader.u8() == 1,
    sleep_time: reader.f64(),
    user_data: reader.u32(),
    world_aabb: Aabb::new(Vec2::zero(), Vec2::zero()),
  }
  body.sync_aabb()
  body
}

///|
fn encode_state_arbiter(out : Array[Byte], arbiter : Arbiter) -> Unit {
  put_u32(out, arbiter.body_a)
  put_u32(out, arbiter.body_b)
  put_u32(out, arbiter.contacts.length())
  for contact in arbiter.contacts {
    put_vec2(out, contact.position)
    put_vec2(out, contact.normal)
    put_f64(out, contact.separation)
    put_u32(out, contact.feature_id)
  }
  put_doubles(out, arbiter.normal_impulses)
  put_doubles(out, arbiter.tangent_impulses)
  put_doubles(out, arbiter.mass_normal)
  put_doubles(out, arbiter.mass_tangent)
  put_doubles(out, arbiter.bias)
  put_doubles(out, arbiter.restitution_bias)
  put_f64(out, arbiter.friction)
  put_f64(out, arbiter.restitution)
}

///|
fn decode_state_arbiter(reader : StateReader) -> Arbiter {
  let body_a = reader.u32()
  let body_b = reader.u32()
  let count = reader.u32()
  let contacts : Array[ContactPoint] = []
  if count > STATE_MAX_CONTACTS || count > reader.remaining() / 36 {
    reader.poison()
  } else {
    let mut i = 0
    while i < count {
      contacts.push({
        position: reader.vec2(),
        normal: reader.vec2(),
        separation: reader.f64(),
        feature_id: reader.u32(),
      })
      i = i + 1
    }
  }
  {
    body_a,
    body_b,
    contacts,
    normal_impulses: reader.doubles(),
    tangent_impulses: reader.doubles(),
    mass_normal: reader.doubles(),
    mass_tangent: reader.doubles(),
    bias: reader.doubles(),
    restitution_bias: reader.doubles(),
    friction: reader.f64(),
    restitution: reader.f64(),
  }
}

///|
/// Serialize the full simulation state: bodies (including velocities, forces and
/// sleep timers), persistent contact impulses, and the world clock.
///
/// `hash` at the tail makes a corrupted or truncated buffer detectable.
pub fn World::save_state(self : World) -> Bytes {
  let out : Array[Byte] = []
  put_u32(out, STATE_MAGIC)
  put_u32(out, STATE_VERSION)
  put_u32(out, self.bodies.length())
  put_f64(out, self.time)
  for body in self.bodies {
    encode_state_body(out, body)
  }
  put_u32(out, self.arbiters.length())
  for arbiter in self.arbiters {
    encode_state_arbiter(out, arbiter)
  }
  put_u64(out, state_hash_of(out))
  Bytes::makei(out.length(), fn(i) { out[i] })
}

///|
/// Restore a snapshot produced by `save_state`.
///
/// Returns `false` and leaves the world untouched when the buffer is truncated,
/// corrupted, from another format version, or when the world has joints (joint
/// solver state is not part of the format).
pub fn World::load_state(self : World, bytes : Bytes) -> Bool {
  if self.joints.length() > 0 {
    return false
  }
  let total = bytes.length()
  if total < 20 {
    return false
  }
  let payload = total - 8
  let mut hash = STATE_FNV_BASIS
  let mut i = 0
  while i < payload {
    hash = (hash ^ bytes[i].to_int().to_uint64()) * STATE_FNV_PRIME
    i = i + 1
  }
  let mut stored = 0UL
  let mut k = 0
  while k < 8 {
    stored = stored | (bytes[payload + k].to_int().to_uint64() << (k * 8))
    k = k + 1
  }
  if hash != stored {
    return false
  }
  let reader = StateReader::new(bytes)
  if reader.u32() != STATE_MAGIC {
    return false
  }
  if reader.u32() != STATE_VERSION {
    return false
  }
  let body_count = reader.u32()
  if body_count < 0 || body_count > STATE_MAX_BODIES {
    return false
  }
  let time = reader.f64()
  let bodies : Array[Body] = []
  let mut b = 0
  while b < body_count && reader.pos <= total {
    bodies.push(decode_state_body(reader))
    b = b + 1
  }
  let arbiter_count = reader.u32()
  if arbiter_count < 0 || arbiter_count > STATE_MAX_ARBITERS {
    return false
  }
  let arbiters : Array[Arbiter] = []
  let mut a = 0
  while a < arbiter_count && reader.pos <= total {
    let arbiter = decode_state_arbiter(reader)
    // A body index out of range would panic in the next step.
    if arbiter.body_a >= body_count || arbiter.body_b >= body_count {
      return false
    }
    arbiters.push(arbiter)
    a = a + 1
  }
  if reader.pos != payload {
    return false
  }
  // Commit only after every field parsed cleanly.
  self.bodies.clear()
  for body in bodies {
    self.bodies.push(body)
  }
  self.arbiters = arbiters
  self.time = time
  true
}
