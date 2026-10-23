///|
/// Chainable scene setup.
///
/// ```moonbit nocheck
/// let world = @moonphys.World::builder()
///   .ground(20.0)
///   .ball(0.4, 0.0, 4.0)
///   .box(0.5, 0.5, 1.0, 6.0)
///   .build()
/// ```
///
/// Every method returns the builder itself, so the whole scene reads as one
/// expression. `build()` hands back the world; bodies are added in call order,
/// which means indices are known and joints can be attached afterwards with
/// `add_revolve_joint` and friends.
pub struct WorldBuilder {
  world : World
  mut last : Int32
}

///|
/// Start a builder on an empty world with default gravity.
pub fn World::builder() -> WorldBuilder {
  { world: World::new(), last: -1, }
}

///|
/// Set gravity, for example `(0.0, -9.81)` for Earth or `(0.0, 0.0)` for a
/// top-down scene.
pub fn WorldBuilder::gravity(
  self : WorldBuilder,
  gx : Double,
  gy : Double,
) -> WorldBuilder {
  self.world.set_gravity(Vec2::new(gx, gy))
  self
}

///|
/// Set how many solver iterations each step runs; more iterations mean stiffer
/// stacks and more time.
pub fn WorldBuilder::iterations(
  self : WorldBuilder,
  iterations : Int32,
) -> WorldBuilder {
  self.world.set_velocity_iterations(iterations)
  self
}

///|
/// Turn automatic sleeping on or off.
pub fn WorldBuilder::sleep_enabled(
  self : WorldBuilder,
  enabled : Bool,
) -> WorldBuilder {
  self.world.set_sleep_enabled(enabled)
  self
}

///|
/// Add a static floor of the given half width whose top surface is `y = 0`.
pub fn WorldBuilder::ground(
  self : WorldBuilder,
  half_width : Double,
) -> WorldBuilder {
  self.last = self.world.add_body(
    Body::fixed(Shape::box(half_width, 0.5)).at(0.0, -0.5),
  )
  self
}

///|
/// Add a dynamic box centred at `(x, y)`.
pub fn WorldBuilder::box(
  self : WorldBuilder,
  half_width : Double,
  half_height : Double,
  x : Double,
  y : Double,
) -> WorldBuilder {
  self.last = self.world.add_body(Body::box(half_width, half_height).at(x, y))
  self
}

///|
/// Add a dynamic circle of `radius` centred at `(x, y)`.
pub fn WorldBuilder::ball(
  self : WorldBuilder,
  radius : Double,
  x : Double,
  y : Double,
) -> WorldBuilder {
  self.last = self.world.add_body(Body::ball(radius).at(x, y))
  self
}

///|
/// Add a body that was configured ahead of time with the `Body::with_*`
/// builders, for example one with friction, restitution or a tag.
pub fn WorldBuilder::body(self : WorldBuilder, body : Body) -> WorldBuilder {
  self.last = self.world.add_body(body)
  self
}

///|
/// Index of the body added last, `-1` when nothing has been added yet.
pub fn WorldBuilder::last_index(self : WorldBuilder) -> Int32 {
  self.last
}

///|
/// Finish the scene and hand over the world.
pub fn WorldBuilder::build(self : WorldBuilder) -> World {
  self.world
}
