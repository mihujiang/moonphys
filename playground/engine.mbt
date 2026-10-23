///|
/// The engine behind `docs/playground`.
///
/// The ABI is deliberately numeric: every exported symbol takes and returns
/// `Int32` or `Double`, so the browser glue needs no memory views, no string codec
/// and no marshalling layer at all — `exports.pg_step(1 / 60)` is the interface,
/// and a polygon reaches the canvas through `pg_vx(i, k)` / `pg_vy(i, k)`.
///
/// The module is built as a `foreign_library`, which is what lets `#export_name`
/// put these functions in the `.wasm` export table.
///
/// ```bash
/// moon build --target wasm-gc --release
/// ```
///
/// Scene list (index → what it shows): 0 pyramid (falls asleep), 1 tower (click
/// it over), 2 ball crowd, 3 jointed chain + pendulum, 4 motor / prismatic /
/// angle-limit machine, 5 mixed shapes on a slope.
const SCENE_COUNT : Int32 = 6

///|
/// The single world the browser drives. Scenes are built by `pg_reset`.
let world_ref : Ref[@moonphys.World] = Ref(@moonphys.World::new())

///|
let scene_ref : Ref[Int32] = Ref(0)

///|
/// Index of the body under the mouse, or `-1` when nothing is being dragged.
let grab_ref : Ref[Int32] = Ref(-1)

///|
/// Where the mouse currently is, in world coordinates.
let target_ref : Ref[@moonphys.Vec2] = Ref(@moonphys.Vec2::zero())

///|
fn add_box(
  world : @moonphys.World,
  x : Double,
  y : Double,
  hw : Double,
  hh : Double,
) -> Int32 {
  world.add_body(@moonphys.Body::box(hw, hh).at(x, y).with_friction(0.6))
}

///|
fn add_ball(
  world : @moonphys.World,
  x : Double,
  y : Double,
  radius : Double,
) -> Int32 {
  world.add_body(@moonphys.Body::ball(radius).at(x, y).with_friction(0.4))
}

///|
fn add_poly(
  world : @moonphys.World,
  x : Double,
  y : Double,
  points : Array[@moonphys.Vec2],
) -> Int32 {
  world.add_body(
    @moonphys.Body::dynamic(@moonphys.Shape::convex(points))
    .at(x, y)
    .with_friction(0.6),
  )
}

///|
fn build_scene(world : @moonphys.World, scene : Int32) -> Unit {
  ignore(
    world.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::box(60.0, 0.5)).at(0.0, -0.5),
    ),
  )
  match scene {
    0 => build_pyramid(world)
    1 => build_tower(world)
    2 => build_crowd(world)
    3 => build_chain(world)
    4 => build_machine(world)
    _ => build_mixed(world)
  }
}

///|
fn build_pyramid(world : @moonphys.World) -> Unit {
  let layers = 5
  for layer in 0..<layers {
    let count = layers - layer
    for i in 0..<count {
      let x = (i.to_double() - (count - 1).to_double() / 2.0) * 0.86
      let y = 0.4 + layer.to_double() * 0.8
      ignore(add_box(world, x, y, 0.42, 0.4))
    }
  }
}

///|
fn build_tower(world : @moonphys.World) -> Unit {
  for i in 0..<9 {
    ignore(add_box(world, 0.0, 0.4 + i.to_double() * 0.8, 0.55, 0.4))
  }
  ignore(add_ball(world, -1.6, 9.0, 0.45))
  ignore(add_ball(world, 1.6, 9.8, 0.45))
}

///|
fn build_crowd(world : @moonphys.World) -> Unit {
  let cols = 20
  let rows = 10
  for row in 0..<rows {
    for col in 0..<cols {
      let x = (col.to_double() - (cols - 1).to_double() / 2.0) * 0.72
      let y = 0.4 + row.to_double() * 0.72
      ignore(add_ball(world, x, y, 0.35))
    }
  }
}

///|
fn build_chain(world : @moonphys.World) -> Unit {
  let anchor = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.12, 0.12)).at(0.0, 7.0),
  )
  let mut previous = anchor
  for i in 0..<4 {
    let y = 6.4 - i.to_double() * 1.0
    let segment = add_box(world, 0.0, y, 0.18, 0.5)
    let joint_y = if i == 0 { 6.9 } else { y + 0.5 }
    world.add_revolve_joint(
      previous,
      segment,
      @moonphys.Vec2::new(0.0, joint_y),
    )
    previous = segment
  }
  let hook = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.1, 0.1)).at(2.8, 7.0),
  )
  let bob = add_ball(world, 2.8, 5.4, 0.4)
  world.add_distance_joint(hook, bob)
}

///|
fn build_machine(world : @moonphys.World) -> Unit {
  // Spinning bar on a motor driven revolute joint.
  let hub = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.15, 0.15)).at(0.0, 5.6),
  )
  let rotor = add_box(world, 0.0, 5.6, 1.5, 0.16)
  world.add_revolve_joint(hub, rotor, @moonphys.Vec2::new(0.0, 5.6))
  ignore(world.add_motor_joint(hub, rotor, 1.6, 400.0))
  // Elevator on a vertical prismatic rail with travel limits.
  let rail = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.1, 0.1)).at(3.8, 8.8),
  )
  let cart = add_box(world, 3.8, 4.6, 0.85, 0.15)
  let slider = world.add_prismatic_joint(
    rail,
    cart,
    @moonphys.Vec2::new(3.8, 4.6),
    @moonphys.Vec2::new(0.0, 1.0),
  )
  ignore(slider.with_limits(-1.2, 1.2))
  ignore(add_box(world, 3.8, 5.1, 0.3, 0.25))
  ignore(add_box(world, 3.8, 5.65, 0.3, 0.25))
  // Arm held by an angle limit.
  let base = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.15, 0.15)).at(-3.8, 6.6),
  )
  let arm = add_box(world, -2.8, 6.6, 1.0, 0.14)
  world.add_revolve_joint(base, arm, @moonphys.Vec2::new(-3.8, 6.6))
  ignore(
    world.add_angle_limit_joint_at(
      base,
      arm,
      @moonphys.Vec2::new(-3.8, 6.6),
      -0.5,
      0.5,
    ),
  )
}

///|
fn build_mixed(world : @moonphys.World) -> Unit {
  ignore(
    world.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::box(3.6, 0.2))
      .at(0.0, 2.6)
      .with_rotation(-0.28),
    ),
  )
  ignore(add_ball(world, -2.4, 7.0, 0.4))
  ignore(add_box(world, -1.2, 8.0, 0.4, 0.4))
  ignore(add_box(world, 0.2, 8.6, 0.55, 0.25))
  ignore(
    add_poly(world, 1.6, 7.2, [
      @moonphys.Vec2::new(-0.45, -0.4),
      @moonphys.Vec2::new(0.45, -0.4),
      @moonphys.Vec2::new(0.0, 0.55),
    ]),
  )
  ignore(
    add_poly(world, 2.8, 8.2, [
      @moonphys.Vec2::new(0.0, 0.46),
      @moonphys.Vec2::new(-0.4374, 0.1421),
      @moonphys.Vec2::new(-0.2704, -0.3722),
      @moonphys.Vec2::new(0.2704, -0.3722),
      @moonphys.Vec2::new(0.4374, 0.1421),
    ]),
  )
}

///|
/// Number of scenes the playground can switch between.
#export_name("pg_scene_count")
pub fn pg_scene_count() -> Int32 {
  SCENE_COUNT
}

///|
/// Rebuild the world for `scene`; also the reset button.
#export_name("pg_reset")
pub fn pg_reset(scene : Int32) -> Unit {
  let world = @moonphys.World::new()
  build_scene(world, scene)
  world_ref.val = world
  scene_ref.val = scene
  grab_ref.val = -1
}

///|
/// Advance the simulation by `dt` seconds.
///
/// While a body is grabbed it is pulled towards the mouse with a critically
/// damped spring — force rather than teleporting, so it still collides with the
/// rest of the scene.
#export_name("pg_step")
pub fn pg_step(dt : Double) -> Unit {
  let world = world_ref.val
  let grabbed = grab_ref.val
  if grabbed >= 0 && grabbed < world.body_count() {
    let body = world.body(grabbed)
    if body.is_dynamic() {
      let mass = body.mass()
      let offset = target_ref.val - body.position()
      body.apply_force(
        offset.scale(90.0 * mass) - body.velocity().scale(14.0 * mass),
      )
      body.apply_angular_impulse(-body.angular_velocity() * mass * 0.05)
      body.wake()
    }
  }
  world.step(dt)
}

///|
#export_name("pg_count")
pub fn pg_count() -> Int32 {
  world_ref.val.body_count()
}

///|
#export_name("pg_contacts")
pub fn pg_contacts() -> Int32 {
  world_ref.val.arbiter_count()
}

///|
#export_name("pg_time")
pub fn pg_time() -> Double {
  world_ref.val.time()
}

///|
#export_name("pg_energy")
pub fn pg_energy() -> Double {
  world_ref.val.total_kinetic_energy()
}

///|
/// `0` dynamic polygon, `1` dynamic circle, `2` static polygon, `3` static
/// circle.
#export_name("pg_kind")
pub fn pg_kind(index : Int32) -> Int32 {
  let body = world_ref.val.body(index)
  let solid = !body.is_dynamic()
  if body.shape().is_circle() {
    if solid {
      3
    } else {
      1
    }
  } else if solid {
    2
  } else {
    0
  }
}

///|
#export_name("pg_x")
pub fn pg_x(index : Int32) -> Double {
  world_ref.val.body(index).position().x()
}

///|
#export_name("pg_y")
pub fn pg_y(index : Int32) -> Double {
  world_ref.val.body(index).position().y()
}

///|
#export_name("pg_rot")
pub fn pg_rot(index : Int32) -> Double {
  world_ref.val.body(index).rotation()
}

///|
#export_name("pg_radius")
pub fn pg_radius(index : Int32) -> Double {
  world_ref.val.body(index).shape().radius()
}

///|
#export_name("pg_vertex_count")
pub fn pg_vertex_count(index : Int32) -> Int32 {
  world_ref.val.body(index).shape().vertices().length()
}

///|
#export_name("pg_vx")
pub fn pg_vx(index : Int32, vertex : Int32) -> Double {
  world_ref.val.body(index).shape().vertices()[vertex].x()
}

///|
#export_name("pg_vy")
pub fn pg_vy(index : Int32, vertex : Int32) -> Double {
  world_ref.val.body(index).shape().vertices()[vertex].y()
}

///|
#export_name("pg_asleep")
pub fn pg_asleep(index : Int32) -> Int32 {
  if world_ref.val.body(index).awake() {
    0
  } else {
    1
  }
}

///|
#export_name("pg_gravity")
pub fn pg_gravity() -> Double {
  -world_ref.val.gravity().y()
}

///|
#export_name("pg_set_gravity")
pub fn pg_set_gravity(value : Double) -> Unit {
  world_ref.val.set_gravity(@moonphys.Vec2::new(0.0, -value))
}

///|
#export_name("pg_iterations")
pub fn pg_iterations() -> Int32 {
  world_ref.val.velocity_iterations
}

///|
#export_name("pg_set_iterations")
pub fn pg_set_iterations(value : Int32) -> Unit {
  world_ref.val.set_velocity_iterations(value)
}

///|
/// Pick the topmost dynamic body under the cursor. Returns its index, or `-1`.
#export_name("pg_grab_start")
pub fn pg_grab_start(x : Double, y : Double) -> Int32 {
  let world = world_ref.val
  let hits = world.query_point(@moonphys.Vec2::new(x, y))
  let mut picked = -1
  for hit in hits {
    if world.body(hit).is_dynamic() {
      picked = hit
    }
  }
  grab_ref.val = picked
  if picked >= 0 {
    target_ref.val = @moonphys.Vec2::new(x, y)
    world.body(picked).wake()
  }
  picked
}

///|
#export_name("pg_grab_move")
pub fn pg_grab_move(x : Double, y : Double) -> Unit {
  target_ref.val = @moonphys.Vec2::new(x, y)
}

///|
#export_name("pg_grab_end")
pub fn pg_grab_end() -> Unit {
  grab_ref.val = -1
}

///|
/// Radial impulse at `(x, y)`: every dynamic body within three metres gets
/// pushed away, scaled by how close it is. This is the "click to hit" verb.
#export_name("pg_blast")
pub fn pg_blast(x : Double, y : Double, power : Double) -> Unit {
  let world = world_ref.val
  let center = @moonphys.Vec2::new(x, y)
  let radius = 3.0
  for index in 0..<world.body_count() {
    let body = world.body(index)
    if body.is_dynamic() {
      let offset = body.position() - center
      let distance = offset.length()
      if distance < radius {
        let falloff = 1.0 - distance / radius
        let direction = if distance > 0.0001 {
          offset.normalize()
        } else {
          @moonphys.Vec2::new(0.0, 1.0)
        }
        body.apply_impulse(
          direction.scale(power * falloff * body.mass()),
          body.position(),
        )
        body.wake()
      }
    }
  }
}
