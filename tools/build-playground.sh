#!/usr/bin/env bash
# Builds the wasm-gc engine behind docs/playground and drops it next to the page.
#
# The built module is committed so GitHub Pages can serve it without a MoonBit
# toolchain; rerun this script after touching playground/.
set -euo pipefail
cd "$(dirname "$0")/.."
moon build --target wasm-gc --release
cp _build/wasm-gc/release/build/playground/playground.wasm docs/playground/engine.wasm
echo "docs/playground/engine.wasm updated"
