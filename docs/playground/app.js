// moonphys playground: canvas front end for the wasm-gc engine.
//
// The engine exposes only numbers (see playground/engine.mbt), so this file is a
// renderer, not a physics playground of its own: every position below comes from
// `pg_x / pg_y / pg_vx / pg_vy`, every interaction goes back through
// `pg_grab_*` and `pg_blast`.

const FIXED_DT = 1 / 60;

const SCENES = [
  {
    name: "金字塔",
    note: "五层方块塔，动能衰减后会整体入睡",
    cx: 0,
    cy: 2.35,
    halfW: 4.5,
    halfH: 2.85,
  },
  {
    name: "塔与球",
    note: "九层塔，点击或拖拽把它推倒",
    cx: 0,
    cy: 5,
    halfW: 5.5,
    halfH: 5.5,
  },
  {
    name: "圆堆",
    note: "200 个圆，看宽相与求解器的吞吐",
    cx: 0,
    cy: 3.85,
    halfW: 7.6,
    halfH: 4.35,
  },
  {
    name: "关节链",
    note: "铰链串成的四节链 + 距离关节单摆",
    cx: 0,
    cy: 3.55,
    halfW: 4.5,
    halfH: 4.05,
  },
  {
    name: "机械",
    note: "马达、滑轨限位、角度限位同场",
    cx: 0,
    cy: 4.4,
    halfW: 6.5,
    halfH: 4.9,
  },
  {
    name: "混合形状",
    note: "三角、五边、方块、圆落在斜面上",
    cx: 0,
    cy: 4.4,
    halfW: 5,
    halfH: 4.9,
  },
];

const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d", { alpha: false });
const el = {
  scenes: document.getElementById("scenes"),
  toggle: document.getElementById("toggle"),
  step: document.getElementById("step"),
  reset: document.getElementById("reset"),
  gravity: document.getElementById("gravity"),
  gravityValue: document.getElementById("gravityValue"),
  iterations: document.getElementById("iterations"),
  iterationsValue: document.getElementById("iterationsValue"),
  fps: document.getElementById("fps"),
  bodies: document.getElementById("bodies"),
  contacts: document.getElementById("contacts"),
  asleep: document.getElementById("asleep"),
  error: document.getElementById("error"),
};

let engine = null;
let scene = 0;
let paused = false;
let zoom = 1;
const view = { cx: 0, cy: 0, halfW: 1, halfH: 1 };
const pointer = { x: 0, y: 0, down: false, dragging: false, moved: 0, downAt: 0 };

function applyScene(index) {
  scene = index;
  zoom = 1;
  const spec = SCENES[index];
  view.cx = spec.cx;
  view.cy = spec.cy;
  view.halfW = spec.halfW;
  view.halfH = spec.halfH;
  engine.pg_reset(index);
  for (const [i, node] of [...el.scenes.children].entries()) {
    node.classList.toggle("active", i === index);
  }
}

function resize() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(canvas.clientWidth * dpr);
  canvas.height = Math.round(canvas.clientHeight * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

// World to screen: uniform scale, so circles stay circles and a 1 m box is a
// square on any window shape.
function scale() {
  return Math.min(
    canvas.clientWidth / (2 * view.halfW * zoom),
    canvas.clientHeight / (2 * view.halfH * zoom),
  );
}

function toScreen(x, y) {
  const s = scale();
  return [canvas.clientWidth / 2 + (x - view.cx) * s, canvas.clientHeight / 2 - (y - view.cy) * s];
}

function toWorld(px, py) {
  const s = scale();
  return [view.cx + (px - canvas.clientWidth / 2) / s, view.cy - (py - canvas.clientHeight / 2) / s];
}

function draw() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  const s = scale();

  ctx.fillStyle = "#080d16";
  ctx.fillRect(0, 0, w, h);

  // 1 m grid, fading out as it gets dense.
  const left = view.cx - w / 2 / s;
  const right = view.cx + w / 2 / s;
  const bottom = view.cy - h / 2 / s;
  const top = view.cy + h / 2 / s;
  ctx.lineWidth = 1;
  ctx.strokeStyle = "rgba(125, 211, 252, 0.07)";
  ctx.beginPath();
  for (let x = Math.floor(left); x <= right; x += 1) {
    const [sx] = toScreen(x, 0);
    ctx.moveTo(sx, 0);
    ctx.lineTo(sx, h);
  }
  for (let y = Math.floor(bottom); y <= top; y += 1) {
    const [, sy] = toScreen(0, y);
    ctx.moveTo(0, sy);
    ctx.lineTo(w, sy);
  }
  ctx.stroke();

  const count = engine.pg_count();
  let asleep = 0;
  for (let i = 0; i < count; i++) {
    const kind = engine.pg_kind(i);
    const x = engine.pg_x(i);
    const y = engine.pg_y(i);
    const sleeping = engine.pg_asleep(i) === 1;
    if (sleeping) asleep += 1;
    const staticBody = kind >= 2;
    const [sx, sy] = toScreen(x, y);
    ctx.globalAlpha = sleeping && !staticBody ? 0.45 : 1;

    if (kind === 1 || kind === 3) {
      const r = engine.pg_radius(i) * s;
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, Math.PI * 2);
      if (staticBody) {
        ctx.fillStyle = "#131f31";
        ctx.strokeStyle = "#2b3d55";
      } else {
        ctx.fillStyle = `hsl(${(i * 29 + 165) % 360}, 72%, 62%)`;
        ctx.strokeStyle = "rgba(6, 12, 22, 0.55)";
      }
    } else {
      const vertexCount = engine.pg_vertex_count(i);
      if (vertexCount === 0) {
        ctx.globalAlpha = 1;
        continue;
      }
      const rot = engine.pg_rot(i);
      const cos = Math.cos(rot);
      const sin = Math.sin(rot);
      ctx.beginPath();
      for (let k = 0; k < vertexCount; k++) {
        const lx = engine.pg_vx(i, k);
        const ly = engine.pg_vy(i, k);
        const wx = x + lx * cos - ly * sin;
        const wy = y + lx * sin + ly * cos;
        const [px, py] = toScreen(wx, wy);
        if (k === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.closePath();
      if (staticBody) {
        ctx.fillStyle = "#131f31";
        ctx.strokeStyle = "#2b3d55";
      } else {
        ctx.fillStyle = `hsl(${(i * 47) % 360}, 66%, 60%)`;
        ctx.strokeStyle = "rgba(6, 12, 22, 0.55)";
      }
    }
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  if (pointer.dragging) {
    const [px, py] = toScreen(pointer.x, pointer.y);
    ctx.beginPath();
    ctx.arc(px, py, 6, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(125, 211, 252, 0.85)";
    ctx.fill();
    ctx.beginPath();
    ctx.arc(px, py, 14, 0, Math.PI * 2);
    ctx.strokeStyle = "rgba(125, 211, 252, 0.35)";
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  return asleep;
}

let last = performance.now();
let accumulator = 0;
let fps = 0;
// Refresh the numbers ten times a second; starting at the interval makes the
// very first frame paint real values instead of leaving `–` on screen.
const HUD_INTERVAL = 0.1;
let hudTimer = HUD_INTERVAL;

function updateHud(asleep) {
  hudTimer = 0;
  el.fps.textContent = fps > 0 ? Math.round(fps).toString() : "–";
  el.bodies.textContent = engine.pg_count().toString();
  el.contacts.textContent = engine.pg_contacts().toString();
  el.asleep.textContent = asleep.toString();
}

function frame(now) {
  const elapsed = Math.min((now - last) / 1000, 0.1);
  last = now;
  fps = fps === 0 ? 1 / Math.max(elapsed, 1e-3) : fps * 0.92 + (1 / Math.max(elapsed, 1e-3)) * 0.08;

  if (!paused) {
    accumulator += elapsed;
    let substeps = 0;
    while (accumulator >= FIXED_DT && substeps < 4) {
      engine.pg_step(FIXED_DT);
      accumulator -= FIXED_DT;
      substeps += 1;
    }
    if (substeps === 4) accumulator = 0;
  }

  const asleep = draw();

  hudTimer += elapsed;
  if (hudTimer >= HUD_INTERVAL) updateHud(asleep);
  requestAnimationFrame(frame);
}

function wire() {
  for (const [index, spec] of SCENES.entries()) {
    const node = document.createElement("button");
    node.className = "chip";
    node.textContent = spec.name;
    node.title = spec.note;
    node.addEventListener("click", () => applyScene(index));
    el.scenes.appendChild(node);
  }

  el.toggle.addEventListener("click", () => {
    paused = !paused;
    el.toggle.textContent = paused ? "继续" : "暂停";
    el.toggle.classList.toggle("primary", !paused);
  });
  el.step.addEventListener("click", () => {
    if (!paused) {
      paused = true;
      el.toggle.textContent = "继续";
      el.toggle.classList.remove("primary");
    }
    engine.pg_step(FIXED_DT);
  });
  el.reset.addEventListener("click", () => applyScene(scene));

  el.gravity.addEventListener("input", () => {
    const value = Number(el.gravity.value);
    engine.pg_set_gravity(value);
    el.gravityValue.textContent = value.toFixed(1);
  });
  el.iterations.addEventListener("input", () => {
    const value = Number(el.iterations.value);
    engine.pg_set_iterations(value);
    el.iterationsValue.textContent = value.toString();
  });

  canvas.addEventListener("pointerdown", (event) => {
    const rect = canvas.getBoundingClientRect();
    const [x, y] = toWorld(event.clientX - rect.left, event.clientY - rect.top);
    pointer.down = true;
    pointer.moved = 0;
    pointer.downAt = performance.now();
    pointer.x = x;
    pointer.y = y;
    const grabbed = engine.pg_grab_start(x, y);
    pointer.dragging = grabbed >= 0;
    if (pointer.dragging) canvas.classList.add("dragging");
    canvas.setPointerCapture(event.pointerId);
  });

  canvas.addEventListener("pointermove", (event) => {
    const rect = canvas.getBoundingClientRect();
    const [x, y] = toWorld(event.clientX - rect.left, event.clientY - rect.top);
    pointer.moved += Math.abs(x - pointer.x) + Math.abs(y - pointer.y);
    pointer.x = x;
    pointer.y = y;
    if (pointer.dragging) engine.pg_grab_move(x, y);
  });

  canvas.addEventListener("pointerup", (event) => {
    const rect = canvas.getBoundingClientRect();
    const [x, y] = toWorld(event.clientX - rect.left, event.clientY - rect.top);
    if (pointer.dragging) {
      engine.pg_grab_end();
      pointer.dragging = false;
      canvas.classList.remove("dragging");
    } else if (pointer.down && pointer.moved < 0.35) {
      engine.pg_blast(x, y, 16);
    }
    pointer.down = false;
    canvas.releasePointerCapture(event.pointerId);
  });

  canvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      zoom = Math.min(3, Math.max(0.35, zoom * Math.exp(event.deltaY * 0.0012)));
    },
    { passive: false },
  );

  window.addEventListener("keydown", (event) => {
    if (event.code === "Space") {
      event.preventDefault();
      el.toggle.click();
    } else if (event.key === "r" || event.key === "R") {
      applyScene(scene);
    } else if (event.key >= "1" && event.key <= String(SCENES.length)) {
      applyScene(Number(event.key) - 1);
    }
  });

  window.addEventListener("resize", resize);
}

async function boot() {
  wire();
  resize();
  try {
    if (location.protocol === "file:") {
      throw new Error("需要 HTTP 服务器：在仓库根目录运行 python -m http.server 后访问 /docs/playground/");
    }
    const response = await fetch("./engine.wasm");
    if (!response.ok) throw new Error(`engine.wasm 加载失败（HTTP ${response.status}）`);
    let instance;
    try {
      ({ instance } = await WebAssembly.instantiateStreaming(response, {}));
    } catch (error) {
      const buffer = await (await fetch("./engine.wasm")).arrayBuffer();
      ({ instance } = await WebAssembly.instantiate(buffer, {}));
    }
    engine = instance.exports;
    applyScene(0);
    // Paint once up front: a headless run may never deliver an animation frame,
    // and a blank canvas plus `–` in the HUD looks like a failed boot.
    updateHud(draw());
    if (new URLSearchParams(location.search).has("selftest")) {
      // Headless smoke check used by CI and by `node tools/`-less debugging:
      // drive every scene for three simulated seconds, then leave one rendered
      // frame behind so a screenshot is not blank. `document.title` carries the
      // numbers, which is what a `--dump-dom` run greps for.
      let bodies = 0;
      for (let s = 0; s < engine.pg_scene_count(); s++) {
        applyScene(s);
        bodies += engine.pg_count();
        for (let i = 0; i < 180; i++) engine.pg_step(FIXED_DT);
        engine.pg_blast(0, 2, 12);
        for (let i = 0; i < 30; i++) engine.pg_step(FIXED_DT);
      }
      applyScene(0);
      for (let i = 0; i < 120; i++) engine.pg_step(FIXED_DT);
      // Run the real loop once with a forced timestamp so the HUD path (and the
      // accumulator in `frame`) is exercised too, not just `draw`.
      frame(performance.now() + 400);
      const hud = [el.fps, el.bodies, el.contacts, el.asleep].map((node) => node.textContent).join("/");
      document.title = `selftest ok scenes=${engine.pg_scene_count()} bodies=${bodies} contacts=${engine.pg_contacts()} asleep=${[...Array(engine.pg_count()).keys()].reduce((sum, i) => sum + engine.pg_asleep(i), 0)} hud=${hud}`;
      return;
    }
    requestAnimationFrame(frame);
  } catch (error) {
    el.error.style.display = "grid";
    el.error.textContent = `启动失败：${error.message}`;
    throw error;
  }
}

boot();
