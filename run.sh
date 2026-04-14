#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

die() { echo "error: $*" >&2; exit 1; }

[[ -f third_party/urdf-loaders/javascript/src/URDFLoader.js ]] \
  || die "missing submodule — run: git submodule update --init --recursive"

[[ -f node_modules/three/build/three.module.js ]] \
  || die "missing three.js — run: npm install"

python -c "import numpy" 2>/dev/null \
  || die "missing python deps — run: pip install numpy pytest"

exec python main.py "$@"
