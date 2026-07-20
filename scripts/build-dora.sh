#!/usr/bin/env bash
# Build the dora 1.0 CLI + the record/replay node binaries from source into
# .venv/bin (no PyPI wheel exists past 0.5.0 and the 1.0 pip CLI is broken; the
# standalone binaries work, and nodes pin the matching dora-rs via [tool.uv.sources]).
# `dora record`/`dora replay` need their node binaries next to `dora`. Needs cargo +
# the project venv (run `uv sync` first; any Python 3.11+ works for the CLI build).
# Re-run after you recreate .venv or change DORA_REF.
set -euo pipefail

DORA_REF="8556977ed2c4f56624e08b90dfc7dc8f54897bd9"  # keep in sync with nodes' [tool.uv.sources]
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
src="$repo/.dora-build/dora-src"

if [[ ! -d "$src/.git" ]]; then
  git init -q "$src"
  git -C "$src" remote add origin https://github.com/dora-rs/dora
fi
git -C "$src" fetch --depth 1 origin "$DORA_REF"
git -C "$src" checkout -q FETCH_HEAD

# Build PyO3 against the project venv interpreter (created by `uv sync`; under
# `uv run` $VIRTUAL_ENV already points at it). Any 3.11+ works for the CLI build.
venv="${VIRTUAL_ENV:-$repo/.venv}"
[[ -x "$venv/bin/python" ]] || uv venv "$venv"

# dora-cli is the CLI; the record/replay node binaries are injected into a dataflow by
# `dora record` / `dora replay` and must sit next to the `dora` binary (the CLI searches
# alongside its own exe first), else those subcommands can't find them and fail.
PYO3_PYTHON="$venv/bin/python" \
  cargo build --release --manifest-path "$src/Cargo.toml" \
    -p dora-cli -p dora-record-node -p dora-replay-node

install -m 755 "$src/target/release/dora" "$venv/bin/dora"
install -m 755 "$src/target/release/dora-record-node" "$venv/bin/dora-record-node"
install -m 755 "$src/target/release/dora-replay-node" "$venv/bin/dora-replay-node"
"$venv/bin/dora" --version
