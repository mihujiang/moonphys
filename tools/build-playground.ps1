# Builds the wasm-gc engine behind docs/playground and drops it next to the page.
#
# The built module is committed so GitHub Pages can serve it without a MoonBit
# toolchain; rerun this script after touching playground/.
$ErrorActionPreference = "Stop"
Push-Location (Join-Path $PSScriptRoot "..")
try {
  moon build --target wasm-gc --release
  Copy-Item "_build/wasm-gc/release/build/playground/playground.wasm" "docs/playground/engine.wasm" -Force
  Write-Host "docs/playground/engine.wasm updated"
} finally {
  Pop-Location
}
