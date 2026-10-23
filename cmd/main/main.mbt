///|
/// `moonphys` demo generator.
///
/// Usage:
///   moon run cmd/main --target js > demo.html
///
/// The program simulates three scenes and prints a single self contained HTML
/// page that embeds every frame as inline SVG. The page plays the frames with
/// `requestAnimationFrame`, so the animation is completely deterministic: the
/// browser only replays what the solver produced.
const DT : Double = 1.0 / 60.0

///|
const FRAMES : Int = 240

///|
fn add_ground(world : @moonphys.World, half_width : Double) -> Unit {
  let ground = @moonphys.Body::fixed(@moonphys.Shape::box(half_width, 0.1)).at(
    0.0, -0.1,
  )
  ignore(world.add_body(ground.with_user_data(9)))
}

///|
/// A six row box pyramid, a ramp with a rolling ball and a few falling balls.
fn pyramid_scene() -> @moonphys.World {
  let world = @moonphys.World::new()
  add_ground(world, 14.0)
  // Static ramp on the left: a ball rolls down and shows friction.
  ignore(
    world.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::box(2.4, 0.1))
      .at(-5.6, 3.2)
      .with_rotation(-0.4)
      .with_user_data(4),
    ),
  )
  ignore(
    world.add_body(
      @moonphys.Body::ball(0.35)
      .at(-7.4, 4.6)
      .with_friction(0.5)
      .with_user_data(1),
    ),
  )
  // Pyramid on the right.
  let rows = 6
  for row in 0..<rows {
    let count = rows - row
    for column in 0..<count {
      let x = 2.0 + (column.to_double() - (count - 1).to_double() * 0.5) * 0.73
      let y = 0.36 + row.to_double() * 0.72
      ignore(
        world.add_body(
          @moonphys.Body::box(0.35, 0.35)
          .at(x, y)
          .with_user_data(row)
          .with_friction(0.6),
        ),
      )
    }
  }
  // Balls raining on the pyramid.
  for i in 0..<3 {
    ignore(
      world.add_body(
        @moonphys.Body::ball(0.32)
        .at(-1.0 + i.to_double() * 1.4, 8.6)
        .with_restitution(0.3)
        .with_user_data(i + 2),
      ),
    )
  }
  world
}

///|
/// A five link chain hanging from a static pin, plus a double pendulum.
fn pendulum_scene() -> @moonphys.World {
  let world = @moonphys.World::new()
  add_ground(world, 14.0)
  // Tilted chain: five links, pinned end to end.
  let anchor = @moonphys.Vec2::new(-3.4, 9.0)
  let tilt = 0.6
  // sin(0.6), cos(0.6)
  let ux = 0.5646
  let uy = -0.8253
  ignore(
    world.add_body(
      @moonphys.Body::fixed(@moonphys.Shape::circle(0.1))
      .at(anchor.x(), anchor.y())
      .with_user_data(9),
    ),
  )
  let mut previous = 0
  for i in 0..<5 {
    let off = 0.5 + i.to_double() * 1.0
    let center = @moonphys.Vec2::new(
      anchor.x() + ux * off,
      anchor.y() + uy * off,
    )
    let link = world.add_body(
      @moonphys.Body::box(0.08, 0.5)
      .at(center.x(), center.y())
      .with_rotation(tilt)
      .with_damping(0.25, 0.25)
      .with_user_data(i),
    )
    let joint = @moonphys.Vec2::new(
      anchor.x() + ux * i.to_double() * 1.0,
      anchor.y() + uy * i.to_double() * 1.0,
    )
    world.add_revolve_joint(previous, link, joint)
    previous = link
  }
  // Double pendulum on the right, released horizontally.
  let pivot = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::circle(0.1))
    .at(3.0, 8.6)
    .with_user_data(9),
  )
  let arm_a = world.add_body(
    @moonphys.Body::box(0.9, 0.07)
    .at(3.9, 8.6)
    .with_damping(0.05, 0.05)
    .with_user_data(2),
  )
  world.add_revolve_joint(pivot, arm_a, @moonphys.Vec2::new(3.0, 8.6))
  let arm_b = world.add_body(
    @moonphys.Body::box(0.9, 0.07)
    .at(5.7, 8.6)
    .with_damping(0.05, 0.05)
    .with_user_data(3),
  )
  world.add_revolve_joint(arm_a, arm_b, @moonphys.Vec2::new(4.8, 8.6))
  world
}

///|
/// A plank bridge between two static posts, with balls dropped on it.
fn bridge_scene() -> @moonphys.World {
  let world = @moonphys.World::new()
  add_ground(world, 14.0)
  let left = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.18, 0.4))
    .at(-7.0, 5.2)
    .with_user_data(9),
  )
  let right = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.18, 0.4))
    .at(7.0, 5.2)
    .with_user_data(9),
  )
  let mut previous = left
  let mut last = left
  for i in 0..<13 {
    let x = -6.0 + i.to_double() * 1.0
    let plank = world.add_body(
      @moonphys.Body::box(0.45, 0.06)
      .at(x, 5.0)
      .with_user_data(i % 5)
      .with_friction(0.7),
    )
    world.add_distance_joint(previous, plank)
    previous = plank
    last = plank
  }
  world.add_distance_joint(last, right)
  // Balls dropped onto the middle of the bridge.
  for i in 0..<3 {
    ignore(
      world.add_body(
        @moonphys.Body::ball(0.4)
        .at(-2.0 + i.to_double() * 2.0, 9.0)
        .with_restitution(0.35)
        .with_user_data(i + 1),
      ),
    )
  }
  world
}

///|
/// Print `FRAMES` frames of one scene as inline SVG inside a `<div>`.
fn emit_scene(world : @moonphys.World, options : @render.SvgOptions) -> Unit {
  println("<div class=\"scene\">")
  for _frame in 0..<FRAMES {
    println(@render.frame_svg(world, options))
    world.step(DT)
  }
  println("</div>")
}

///|
const PAGE_HEAD : String =
  #|<!DOCTYPE html>
  #|<html lang="en">
  #|<head>
  #|<meta charset="utf-8">
  #|<meta name="viewport" content="width=device-width, initial-scale=1">
  #|<title>moonphys — 2D rigid body physics in MoonBit</title>
  #|<style>
  #|:root { color-scheme: dark; }
  #|body { margin: 0; padding: 32px 24px 64px; background: #070b14; color: #e2e8f0;
  #|  font: 15px/1.6 ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif; }
  #|h1 { font-size: 26px; margin: 0 0 4px; letter-spacing: .3px; }
  #|p.lede { color: #94a3b8; margin: 0 0 28px; max-width: 720px; }
  #|h2 { font-size: 15px; font-weight: 600; letter-spacing: .4px; text-transform: uppercase;
  #|  color: #7dd3fc; margin: 28px 0 10px; }
  #|.scene { position: relative; width: 720px; max-width: 100%; aspect-ratio: 3 / 2;
  #|  border: 1px solid #1e293b; border-radius: 12px; overflow: hidden; background: #0b1020; }
  #|.scene svg { position: absolute; inset: 0; width: 100%; height: 100%; display: none; }
  #|.scene svg.on { display: block; }
  #|footer { margin-top: 40px; color: #64748b; font-size: 13px; }
  #|code { color: #fbbf24; }
  #|</style>
  #|</head>
  #|<body>
  #|<h1>moonphys</h1>
  #|<p class="lede">A deterministic 2D rigid body engine written in MoonBit:
  #|  SAT collision detection, impulse solver with warm starting, Coulomb friction,
  #|  revolute and distance joints, sleeping, and a sort-and-sweep broadphase.
  #|  Every frame below was produced by <code>moon run cmd/main --target js</code>.</p>

///|
const PAGE_SCRIPT : String =
  #|<script>
  #|const scenes = [...document.querySelectorAll('.scene')].map((el) => ({
  #|  frames: [...el.querySelectorAll('svg')],
  #|  index: 0,
  #|}));
  #|for (const scene of scenes) {
  #|  if (scene.frames.length > 0) scene.frames[0].classList.add('on');
  #|}
  #|let last = 0, accumulator = 0;
  #|const STEP = 1000 / 60;
  #|function tick(now) {
  #|  if (last === 0) last = now;
  #|  accumulator += now - last;
  #|  last = now;
  #|  if (accumulator > 500) accumulator = 500;
  #|  while (accumulator >= STEP) {
  #|    accumulator -= STEP;
  #|    for (const scene of scenes) {
  #|      const n = scene.frames.length;
  #|      if (n === 0) continue;
  #|      scene.frames[scene.index].classList.remove('on');
  #|      scene.index = (scene.index + 1) % n;
  #|      scene.frames[scene.index].classList.add('on');
  #|    }
  #|  }
  #|  requestAnimationFrame(tick);
  #|}
  #|requestAnimationFrame(tick);
  #|</script>
  #|<footer>moonphys · MIT / Apache-2.0 · ported concepts from Box2D-Lite and Planck.js</footer>
  #|</body>
  #|</html>

///|
fn main {
  let options = @render.SvgOptions::new()
    .with_size(720, 480)
    .with_scale(40.0)
    .with_center(0.0, 4.0)
  println(PAGE_HEAD)
  println("<h2>Pyramid stack · friction ramp · bouncing balls</h2>")
  emit_scene(pyramid_scene(), options)
  println("<h2>Five link chain · double pendulum</h2>")
  emit_scene(pendulum_scene(), options)
  println("<h2>Plank bridge · distance joints</h2>")
  emit_scene(bridge_scene(), options)
  println(PAGE_SCRIPT)
}
