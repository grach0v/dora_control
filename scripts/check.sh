#!/usr/bin/env bash
# All repo checks in one go: lint + the unit suites + the e2e smoke (~40 s).
# Usage: ./scripts/check.sh [--fast]   (--fast skips the smoke)
set -euo pipefail
cd "$(dirname "$0")/.."

echo "== ruff =="
uv run ruff check .

echo "== unit: pinocchio =="
(cd nodes/pinocchio && uv run pytest -q)

echo "== unit: retarget =="
(cd nodes/retarget && uv run pytest -q)

if [[ "${1:-}" != "--fast" ]]; then
  echo "== e2e smoke (headless trossen_stationary_mujoco, ~40 s) =="
  (cd dataflows && uv run --project ../nodes/lerobot pytest tests -q)
fi

echo "ALL CHECKS PASSED"
