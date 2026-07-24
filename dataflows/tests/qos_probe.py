"""QoS probe node for remote-teleop testing (test-only, like scripted_driver.py).

Subscribes to any set of inputs and measures, per input id:
  * arrival rate (Hz)
  * payload size (bytes)
  * latency = arrival wall-clock - message `timestamp` metadata (valid when the
    producing and probing hosts are NTP-synced; cross-host skew adds a constant
    offset, so COMPARE deltas between configs rather than absolute values).

For inputs named in VALUES_INPUTS (CSV, e.g. "state") every sample's values are
also dumped, giving a joint-trace for smoothness analysis.

Env:
  OUT_PATH       where to write the JSONL report (one line per input per window,
                 plus per-sample lines for VALUES_INPUTS)
  WINDOW_S       aggregation window seconds (default 5)
  VALUES_INPUTS  CSV of input ids whose float values to dump per sample

Every line is a JSON object: {"kind": "agg"|"sample", ...}.
"""

import json
import os
import time

from dora import Node


def main() -> None:
    out_path = os.environ["OUT_PATH"]
    window_s = float(os.environ.get("WINDOW_S", "5"))
    values_inputs = set(filter(None, os.environ.get("VALUES_INPUTS", "").split(",")))

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    out = open(out_path, "w", buffering=1)

    stats: dict[str, dict] = {}
    window_start = time.time()

    node = Node()
    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT":
            continue
        now = time.time()
        input_id = event["id"]
        meta = event["metadata"] or {}
        ts = meta.get("timestamp")
        arr = event["value"]
        nbytes = arr.nbytes if hasattr(arr, "nbytes") else 0

        s = stats.setdefault(input_id, {"n": 0, "bytes": 0, "lat": []})
        s["n"] += 1
        s["bytes"] += nbytes
        if isinstance(ts, float):
            s["lat"].append(now - ts)

        if input_id in values_inputs:
            out.write(json.dumps({
                "kind": "sample", "input": input_id, "t": now, "ts": ts,
                "values": arr.to_pylist(),
            }) + "\n")

        if now - window_start >= window_s:
            for iid, st in stats.items():
                lat = sorted(st["lat"])
                line = {
                    "kind": "agg", "input": iid, "t": now, "window_s": now - window_start,
                    "hz": st["n"] / (now - window_start),
                    "kbytes_s": st["bytes"] / (now - window_start) / 1000,
                }
                if lat:
                    line["lat_ms_p50"] = lat[len(lat) // 2] * 1000
                    line["lat_ms_p90"] = lat[int(len(lat) * 0.9)] * 1000
                    line["lat_ms_max"] = lat[-1] * 1000
                out.write(json.dumps(line) + "\n")
                print(json.dumps(line), flush=True)  # visible via `dora logs <flow> <probe>`
            stats.clear()
            window_start = now

    out.close()


if __name__ == "__main__":
    main()
