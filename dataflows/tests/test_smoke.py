"""E2E smoke test for the local Trossen sim dataflow (trossen_sim.yml).

(The Trossen flow is the smoke default because only the trossen MJCF defines sim
cameras — the ur5e_dual model has none yet, which is why ur5e_sim.yml ships with
cameras commented out.)

Builds the dataflow once, runs it headless for a bounded time, and asserts that
BOTH recording artifacts exist with real content:

  * the LeRobotDataset under out/lerobot_trossen_sim has meta/info.json with
    total_frames > 0 and at least one videos/*.mp4
  * the rerun .rrd under out/rerun_trossen_sim exists and is non-trivial in size

The shipped dataflow is meant for a human: the web-controller page (drive by
hand, press Record) and rerun VISUALIZE__SINK=spawn (live viewer window). CI has
no browser, so the test rewrites a headless copy — it swaps the web-controller
node for tests/scripted_driver.py (same node id + outputs, self-drives the arms
and emits one episode start) and sets VISUALIZE__SINK=memory so it runs windowless.

Run with:  cd dataflows && uv run --project nodes/lerobot pytest tests -q
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
from pathlib import Path

FLOW_DIR = Path(__file__).resolve().parent.parent   # dataflows/
SRC_FLOW = "trossen_sim.yml"
FLOW = "_smoke_headless.yml"                        # generated; gitignored
REPO_ROOT = (FLOW_DIR / "../out/lerobot_trossen_sim").resolve()
RRD_DIR = (FLOW_DIR / "../out/rerun_trossen_sim").resolve()
# The dora CLI is the pinned source build installed at the repo-root .venv (the
# 1.0 pip wheel is broken; see scripts/build-dora.sh). Run that binary directly.
ROOT_DORA = (FLOW_DIR / "../.venv/bin/dora").resolve()
# The scripted driver is a bare `.py` node; the 1.0 daemon spawns it with the
# venv named by VIRTUAL_ENV, so point that at the web-controller venv (it has
# dora + the pyarrow the driver imports), independent of how pytest was launched.
WEBCTL = (FLOW_DIR / "../nodes/web-controller").resolve()
STOP_AFTER = "25s"  # capture margin so a slow startup still records plenty of frames


def _clean() -> None:
    if REPO_ROOT.exists():
        shutil.rmtree(REPO_ROOT)
    if RRD_DIR.exists():
        shutil.rmtree(RRD_DIR)
    headless = FLOW_DIR / FLOW
    if headless.exists():
        headless.unlink()


# Headless stand-in for the web-controller node: same id + outputs, but runs the
# scripted driver instead of serving the browser page. robot_command is declared
# (the manager wires it) but never emitted — the run ends via --stop-after.
_DRIVER_BLOCK = """  - id: controller
    build: uv sync --project nodes/web-controller
    path: tests/scripted_driver.py
    inputs:
      tick: dora/timer/millis/33
      program_state: manager/program_state
    outputs:
      - command
      - episode_control
      - robot_command
"""


def _write_headless_flow() -> None:
    src = (FLOW_DIR / SRC_FLOW).read_text()
    src = src.replace('VISUALIZE__SINK: "spawn"', 'VISUALIZE__SINK: "memory"')
    # Replace the whole web-controller node block (browser-driven) with the
    # scripted driver, keeping every downstream wiring intact.
    src = re.sub(
        r"  - id: controller\n.*?(?=\n  - id: pinocchio)",
        _DRIVER_BLOCK.rstrip("\n"),
        src,
        flags=re.DOTALL,
    )
    (FLOW_DIR / FLOW).write_text(src)


def _run(cmd: list[str], timeout: int) -> subprocess.CompletedProcess:
    # Run the pinned root dora binary (swap the leading "dora" token for it), with
    # VIRTUAL_ENV=web-controller so the bare-.py scripted driver is spawned with a
    # python that has dora + pyarrow.
    full = [str(ROOT_DORA) if c == "dora" else c for c in cmd]
    env = {**os.environ, "VIRTUAL_ENV": str(WEBCTL / ".venv")}
    print("+", " ".join(full))
    proc = subprocess.run(
        full, cwd=FLOW_DIR, capture_output=True, text=True, timeout=timeout, env=env
    )
    print("STDOUT:\n", proc.stdout)
    print("STDERR:\n", proc.stderr)
    return proc


def test_smoke() -> None:
    _clean()
    _write_headless_flow()

    build = _run(["dora", "build", FLOW], timeout=600)
    assert build.returncode == 0, "dora build failed"

    run = _run(["dora", "run", FLOW, "--stop-after", STOP_AFTER], timeout=120)
    assert run.returncode == 0, "dora run failed"

    # --- LeRobot dataset assertions ---
    info_path = REPO_ROOT / "meta" / "info.json"
    assert info_path.exists(), f"missing {info_path}"
    info = json.loads(info_path.read_text())
    total_frames = info.get("total_frames", 0)
    assert total_frames > 0, f"expected total_frames > 0, got {total_frames}"

    videos = list((REPO_ROOT / "videos").rglob("*.mp4"))
    assert videos, f"no videos/*.mp4 under {REPO_ROOT / 'videos'}"
    assert any(v.stat().st_size > 0 for v in videos), "video file is empty"

    # --- rerun .rrd assertions (one per-episode file) ---
    rrds = list(RRD_DIR.glob("episode_*.rrd"))
    assert rrds, f"no episode_*.rrd under {RRD_DIR}"
    size = rrds[0].stat().st_size
    assert size > 10_000, f".rrd too small ({size} bytes), likely empty"

    print(f"OK: lerobot total_frames={total_frames}, "
          f"videos={[str(v) for v in videos]}, rrd={size} bytes")


if __name__ == "__main__":
    test_smoke()
