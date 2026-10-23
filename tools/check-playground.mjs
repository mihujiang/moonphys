// Smoke test for the wasm module behind docs/playground.
//
// The page talks to the engine through 24 numeric `pg_*` exports only, so a
// missing export (or a scene that blows up) is a rendering-agnostic failure we
// can catch without a browser. CI runs this after `moon build --target wasm-gc`,
// which also proves the committed `engine.wasm` is still in sync with a build.
//
//   node tools/check-playground.mjs            # check docs/playground/engine.wasm
//   node tools/check-playground.mjs path.wasm  # check an arbitrary build

import { readFile } from "node:fs/promises";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const wasmPath = process.argv[2] ? resolve(process.argv[2]) : join(here, "..", "docs", "playground", "engine.wasm");

const REQUIRED = [
  "pg_scene_count",
  "pg_reset",
  "pg_step",
  "pg_count",
  "pg_contacts",
  "pg_time",
  "pg_energy",
  "pg_kind",
  "pg_x",
  "pg_y",
  "pg_rot",
  "pg_radius",
  "pg_vertex_count",
  "pg_vx",
  "pg_vy",
  "pg_asleep",
  "pg_gravity",
  "pg_set_gravity",
  "pg_iterations",
  "pg_set_iterations",
  "pg_grab_start",
  "pg_grab_move",
  "pg_grab_end",
  "pg_blast",
];

function fail(message) {
  console.error(`check-playground: ${message}`);
  process.exit(1);
}

const module = new WebAssembly.Module(await readFile(wasmPath));
const exported = new Set(WebAssembly.Module.exports(module).map((entry) => entry.name));
const missing = REQUIRED.filter((name) => !exported.has(name));
if (missing.length > 0) fail(`missing exports: ${missing.join(", ")}`);

const engine = new WebAssembly.Instance(module, {}).exports;
const scenes = engine.pg_scene_count();
if (scenes < 1) fail("engine reports no scenes");

const dt = 1 / 60;
for (let scene = 0; scene < scenes; scene++) {
  engine.pg_reset(scene);
  const bodies = engine.pg_count();
  if (bodies < 2) fail(`scene ${scene} has ${bodies} bodies`);
  for (let step = 0; step < 180; step += 1) engine.pg_step(dt);
  for (let i = 0; i < bodies; i += 1) {
    const x = engine.pg_x(i);
    const y = engine.pg_y(i);
    if (!Number.isFinite(x) || !Number.isFinite(y)) fail(`scene ${scene} body ${i} escaped to (${x}, ${y})`);
    if (Math.abs(x) > 1e3 || Math.abs(y) > 1e3) fail(`scene ${scene} body ${i} drifted to (${x}, ${y})`);
  }
  const energy = engine.pg_energy();
  if (!Number.isFinite(energy) || energy < 0) fail(`scene ${scene} kinetic energy is ${energy}`);
  if (engine.pg_time() <= 0) fail(`scene ${scene} did not advance simulation time`);
}

// Interaction path: grabbing must pick a body, and a blast must add energy.
engine.pg_reset(1);
for (let step = 0; step < 60; step += 1) engine.pg_step(dt);
const dynamic = [...Array(engine.pg_count()).keys()].find((i) => engine.pg_kind(i) <= 1);
if (dynamic === undefined) fail("scene 1 has no dynamic body to grab");
const grabbed = engine.pg_grab_start(engine.pg_x(dynamic), engine.pg_y(dynamic));
if (grabbed < 0) fail("grab_start found no body at a dynamic body's centre");
engine.pg_grab_move(engine.pg_x(grabbed) + 0.5, engine.pg_y(grabbed) + 0.5);
engine.pg_grab_end();
for (let step = 0; step < 10; step += 1) engine.pg_step(dt);

const before = engine.pg_energy();
engine.pg_blast(0, 2, 12);
for (let step = 0; step < 5; step += 1) engine.pg_step(dt);
if (engine.pg_energy() <= before) fail("pg_blast did not add energy");

console.log(
  `check-playground: ok ${REQUIRED.length} exports, ${scenes} scenes, grabbed body ${grabbed}, ` +
    `energy ${before.toFixed(2)} -> ${engine.pg_energy().toFixed(2)}`,
);
