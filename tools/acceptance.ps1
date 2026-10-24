# End-to-end acceptance run: does the project work for someone who only has the
# repo or the published package?
#
#   powershell -File tools/acceptance.ps1
#   powershell -File tools/acceptance.ps1 -SkipClone -Keep     # reuse a checkout
#
# Path A clones the repository from GitHub into a temp dir and runs every command
# the README promises. Path B builds a separate consumer module, installs the
# package from mooncakes, and exercises the public API from its tests. Both paths
# print per-step pass/fail plus a summary; any failure exits non-zero.
#
# The script itself is ASCII-only on purpose: Windows PowerShell 5.1 decodes
# .ps1 files as ANSI unless they have a BOM, which corrupts non-ASCII literals.
#
# Needs `moon` and `node` on PATH (the same toolchain you build with).

param(
  [string]$RepoUrl = "https://github.com/mihujiang/moonphys.git",
  [string]$PackageName = "weopqrst/moonphys",
  [string]$Version = "",
  [string]$WorkDir = "",
  [switch]$SkipClone,
  [switch]$Keep
)

$ErrorActionPreference = "Stop"
if (-not $WorkDir) { $WorkDir = Join-Path $env:TEMP "moonphys-acceptance" }
$repoDir = Join-Path $WorkDir "repo"
$consumerDir = Join-Path $WorkDir "consumer"
$script:results = @()
$script:failed = 0

function Write-Head([string]$text) {
  Write-Host ""
  Write-Host "== $text" -ForegroundColor Cyan
}

function Invoke-Check([string]$name, [scriptblock]$body) {
  Write-Host "- $name ... " -NoNewline
  try {
    & $body
    Write-Host "ok" -ForegroundColor Green
    $script:results += [pscustomobject]@{ Step = $name; Ok = $true }
  } catch {
    Write-Host "FAIL" -ForegroundColor Red
    Write-Host "    $($_.Exception.Message)" -ForegroundColor Red
    $script:results += [pscustomobject]@{ Step = $name; Ok = $false }
    $script:failed += 1
  }
}

# Runs a command in $dir and throws with the tail of its output when it fails.
#
# Native tools write progress to stderr (git's "Cloning into...", moon's
# "Finished..."), and `$ErrorActionPreference = "Stop"` would turn those into
# terminating errors, so stderr is downgraded to text inside this function and
# the exit code alone decides success.
function Invoke-Native([string]$exe, [string[]]$arguments, [string]$dir) {
  $previous = $ErrorActionPreference
  $ErrorActionPreference = "Continue"
  Push-Location $dir
  try {
    $output = & $exe @arguments 2>&1 | Out-String
    $code = $LASTEXITCODE
  } finally {
    Pop-Location
    $ErrorActionPreference = $previous
  }
  if ($code -ne 0) {
    $tail = ($output -split "`r?`n" | Select-Object -Last 12) -join [Environment]::NewLine
    throw "'$exe $($arguments -join ' ')' exited $code`n$tail"
  }
  return $output
}

function Assert-Match([string]$text, [string]$pattern, [string]$what) {
  if ($text -notmatch $pattern) { throw "expected output missing ($what): $pattern" }
}

function Write-Utf8([string]$path, [string]$content) {
  $dir = Split-Path -Parent $path
  if ($dir -and -not (Test-Path $dir)) { New-Item -ItemType Directory -Force -Path $dir | Out-Null }
  [System.IO.File]::WriteAllText($path, $content, (New-Object System.Text.UTF8Encoding($false)))
}

function Remove-Dir([string]$path) {
  if (Test-Path $path) { [System.IO.Directory]::Delete($path, $true) }
}

Write-Host "moonphys acceptance run -> $WorkDir"

# ---------------------------------------------------------------- path A: repo
Write-Head "A. clone from GitHub and run every documented command"

$repoReady = $SkipClone -and (Test-Path (Join-Path $repoDir "moon.mod"))
if (-not $SkipClone) {
  Invoke-Check "clone repository" {
    Remove-Dir $repoDir
    New-Item -ItemType Directory -Force -Path $WorkDir | Out-Null
    Invoke-Native "git" @("-c", "core.autocrlf=false", "clone", "--depth", "1", $RepoUrl, $repoDir) $WorkDir | Out-Null
  }
  $repoReady = Test-Path (Join-Path $repoDir "moon.mod")
}

if ($repoReady -and -not $Version) {
  $modText = [System.IO.File]::ReadAllText((Join-Path $repoDir "moon.mod"), [System.Text.Encoding]::UTF8)
  if ($modText -match 'version\s*=\s*"([^"]+)"') { $Version = $matches[1] }
}
if (-not $Version) { throw "no checkout and no -Version: pass -Version <x.y.z> to test a published package only" }
Write-Host "   version: $Version"

if (-not $repoReady) {
  Write-Host "   checkout unavailable, skipping path A" -ForegroundColor Yellow
} else {

Invoke-Check "fmt --check" { Invoke-Native "moon" @("fmt", "--check") $repoDir | Out-Null }
Invoke-Check "check --deny-warn" { Invoke-Native "moon" @("check", "--deny-warn") $repoDir | Out-Null }
Invoke-Check "test" {
  $out = Invoke-Native "moon" @("test") $repoDir
  Assert-Match $out "passed: \d+, failed: 0" "moon test"
}
Invoke-Check "test --target js" {
  $out = Invoke-Native "moon" @("test", "--target", "js") $repoDir
  Assert-Match $out "passed: \d+, failed: 0" "moon test --target js"
}
Invoke-Check "examples/bounce" {
  $out = Invoke-Native "moon" @("run", "examples/bounce", "--target", "js") $repoDir
  Assert-Match $out "t=\d" "bounce output"
}
Invoke-Check "examples/pyramid" {
  $out = Invoke-Native "moon" @("run", "examples/pyramid", "--target", "js") $repoDir
  Assert-Match $out "asleep=\d+/\d+" "pyramid output"
}
Invoke-Check "examples/ragdoll" {
  $out = Invoke-Native "moon" @("run", "examples/ragdoll", "--target", "js") $repoDir
  Assert-Match $out "joints=\d+" "ragdoll output"
}
Invoke-Check "bench: both targets agree on state hashes" {
  $js = Invoke-Native "moon" @("run", "bench", "--target", "js") $repoDir
  $gc = Invoke-Native "moon" @("run", "bench", "--target", "wasm-gc") $repoDir
  Assert-Match $js "ms/step=" "js bench"
  $jsHashes = [regex]::Matches($js, "hash=(\d+)") | ForEach-Object { $_.Groups[1].Value }
  $gcHashes = [regex]::Matches($gc, "hash=(\d+)") | ForEach-Object { $_.Groups[1].Value }
  if ($jsHashes.Count -lt 3) { throw "bench printed fewer than 3 scenes" }
  if (($jsHashes -join ",") -ne ($gcHashes -join ",")) { throw "cross-target hash mismatch: $($jsHashes -join ',') vs $($gcHashes -join ',')" }
}
Invoke-Check "wasm build + playground smoke test" {
  Invoke-Native "moon" @("build", "--target", "wasm-gc", "--release") $repoDir | Out-Null
  $out = Invoke-Native "node" @("tools/check-playground.mjs") $repoDir
  Assert-Match $out "24 exports" "check-playground (committed wasm)"
  $out = Invoke-Native "node" @("tools/check-playground.mjs", "_build/wasm-gc/release/build/playground/playground.wasm") $repoDir
  Assert-Match $out "24 exports" "check-playground (fresh build)"
}
Invoke-Check "committed engine.wasm matches the source version" {
  $wasmPath = Join-Path $repoDir "docs/playground/engine.wasm"
  if (-not (Test-Path $wasmPath)) { throw "docs/playground/engine.wasm is missing" }
  $text = [System.Text.Encoding]::UTF8.GetString([System.IO.File]::ReadAllBytes($wasmPath))
  $needle = "weopqrst/moonphys/playground@" + $Version
  if (-not $text.Contains($needle)) {
    throw "engine.wasm embeds another version than $Version (rebuild it after bumping moon.mod)"
  }
}
Invoke-Check "docs/index.html is UTF-8 with 720 frames" {
  $htmlPath = Join-Path $repoDir "docs/index.html"
  $bytes = [System.IO.File]::ReadAllBytes($htmlPath)
  if ($bytes.Length -ge 2 -and (($bytes[0] -eq 0xFF -and $bytes[1] -eq 0xFE) -or ($bytes[0] -eq 0xFE -and $bytes[1] -eq 0xFF))) {
    throw "docs/index.html is UTF-16 (a PowerShell redirect artifact), expected UTF-8"
  }
  $html = [System.Text.Encoding]::UTF8.GetString($bytes)
  $frames = ([regex]::Matches($html, "<svg")).Count
  if ($frames -ne 720) { throw "demo page has $frames frames, expected 720 (3 scenes x 240)" }
}
Invoke-Check "package" {
  $out = Invoke-Native "moon" @("package") $repoDir
  Assert-Match $out "Check passed" "moon package"
}

} # end of path A

# ------------------------------------------------------------ path B: package
Write-Head "B. install $PackageName@$Version from mooncakes and run its own tests"

Remove-Dir $consumerDir
New-Item -ItemType Directory -Force -Path $WorkDir | Out-Null
Invoke-Native "moon" @("new", "consumer", "--user", "acceptance") $WorkDir | Out-Null
$moduleDir = Join-Path $WorkDir "consumer"
Invoke-Native "moon" @("add", "$PackageName@$Version") $moduleDir | Out-Null

# A separate package keeps the acceptance tests out of the generated main package.
Write-Utf8 (Join-Path $moduleDir "api/moon.pkg") @"
import {
  "$PackageName" @moonphys,
}
"@

Write-Utf8 (Join-Path $moduleDir "api/api.mbt") @'
///|
fn falling_scene() -> @moonphys.World {
  @moonphys.World::builder()
  .ground(10.0)
  .ball(0.5, 0.0, 4.0)
  .build()
}

///|
test "README builder snippet compiles and simulates" {
  let world = @moonphys.World::builder()
  .ground(20.0)
  .ball(0.4, 0.0, 6.0)
  .box(0.5, 0.5, 1.5, 8.0)
  .body(@moonphys.Body::ball(0.3).at(-2.0, 9.0).with_restitution(0.8))
  .build()
  world.run(240, 1.0 / 60.0)
  assert_true(world.body_count() == 4)
  assert_true(world.snapshot_json().length() > 100)
}

///|
test "ball settles on the ground" {
  let world = falling_scene()
  world.run(180, 1.0 / 60.0)
  let y = world.body(1).position().y()
  assert_true(y > 0.45 && y < 0.55)
  assert_true(world.total_kinetic_energy() < 1.0)
  assert_true(world.arbiter_count() >= 1)
}

///|
test "queries return hits" {
  let world = falling_scene()
  world.run(120, 1.0 / 60.0)
  let hit = world.ray_cast(
    @moonphys.Vec2::new(0.0, 5.0),
    @moonphys.Vec2::new(0.0, -1.0),
    20.0,
  )
  assert_true(hit is Some(_))
  let region = world.query_aabb(
    @moonphys.Aabb::new(
      @moonphys.Vec2::new(-5.0, -1.0),
      @moonphys.Vec2::new(5.0, 2.0),
    ),
  )
  assert_true(region.length() >= 1)
  let point = world.query_point(@moonphys.Vec2::new(0.0, 0.49))
  assert_true(point.length() >= 1)
}

///|
test "all five joint kinds stay finite" {
  let world = @moonphys.World::new()
  let anchor = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.2, 0.2)).at(0.0, 5.0),
  )
  let arm = world.add_body(@moonphys.Body::box(1.0, 0.1).at(1.0, 5.0))
  world.add_revolve_joint(anchor, arm, @moonphys.Vec2::new(0.0, 5.0))
  ignore(world.add_motor_joint(anchor, arm, 2.0, 200.0))
  let rail = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.1, 0.1)).at(3.0, 6.0),
  )
  let cart = world.add_body(@moonphys.Body::box(0.5, 0.2).at(3.0, 3.0))
  let slider = world.add_prismatic_joint(
    rail,
    cart,
    @moonphys.Vec2::new(3.0, 3.0),
    @moonphys.Vec2::new(0.0, 1.0),
  )
  ignore(slider.with_limits(-1.0, 1.0))
  let base = world.add_body(
    @moonphys.Body::fixed(@moonphys.Shape::box(0.1, 0.1)).at(-3.0, 5.0),
  )
  let lever = world.add_body(@moonphys.Body::box(0.8, 0.1).at(-2.2, 5.0))
  world.add_revolve_joint(base, lever, @moonphys.Vec2::new(-3.0, 5.0))
  ignore(
    world.add_angle_limit_joint_at(
      base,
      lever,
      @moonphys.Vec2::new(-3.0, 5.0),
      -0.5,
      0.5,
    ),
  )
  world.add_soft_distance_joint(anchor, cart, 3.0, 0.8)
  world.run(120, 1.0 / 60.0)
  for i in 0..<world.body_count() {
    let position = world.body(i).position()
    assert_true(position.x().abs() < 100.0 && position.y().abs() < 100.0)
  }
}

///|
test "state hash is reproducible" {
  let a = falling_scene()
  let b = falling_scene()
  // 30 steps: the ball is still in the air, so any divergence would show up.
  a.run(30, 1.0 / 60.0)
  b.run(30, 1.0 / 60.0)
  assert_true(a.state_hash() == b.state_hash())
  b.step(1.0 / 60.0)
  assert_true(a.state_hash() != b.state_hash())
}
'@

Invoke-Check "published package: moon test (wasm-gc)" {
  $out = Invoke-Native "moon" @("test", "--target", "wasm-gc") $moduleDir
  Assert-Match $out "passed: 5, failed: 0" "consumer tests"
}
Invoke-Check "published package: moon test (js)" {
  $out = Invoke-Native "moon" @("test", "--target", "js") $moduleDir
  Assert-Match $out "passed: 5, failed: 0" "consumer tests"
}

# -------------------------------------------------------------------- summary
Write-Head "summary"
$script:results | ForEach-Object {
  $mark = if ($_.Ok) { "PASS" } else { "FAIL" }
  Write-Host ("  [{0}] {1}" -f $mark, $_.Step)
}
$passed = ($script:results | Where-Object { $_.Ok }).Count
Write-Host ""
if ($script:failed -gt 0) {
  Write-Host "$script:failed of $($script:results.Count) checks failed" -ForegroundColor Red
  Write-Host "artifacts kept in $WorkDir"
  exit 1
}
Write-Host "all $passed checks passed ($PackageName@$Version)" -ForegroundColor Green
if (-not $Keep) {
  Remove-Dir $WorkDir
  Write-Host "cleaned $WorkDir"
} else {
  Write-Host "artifacts kept in $WorkDir"
}
