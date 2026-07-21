# sync — reusable aggregator (completion-emit, skew-aware)

A generic node that **collects several event-driven inputs and emits one concatenated bundle
the moment every input has a fresh sample** (once per producer cycle). It's the missing
primitive for turning per-node streams into a synchronized whole-robot snapshot — e.g. two
leader arms → one `command`, two follower states → one `state`. (dora has no built-in
join/sync; this generalizes the timestamp logic the `lerobot` recorder already uses.)

Robot-agnostic: it concatenates the configured `INPUTS` **in order**; the consumer's scene
descriptor layout defines what the resulting vector means.

## Why emit-on-completion (no tick at all)

Emitting on a timer adds a timer-phase delay: producers and a tick sit on independent
clocks, so a tick-emitted bundle trails the freshest samples by up to a full frame.
Emitting the instant the LAST input of a cycle arrives removes that delay — the only
added latency is the few-ms spread between the producers' publishes. The bundle's
`timestamp` is the OLDEST component timestamp (honest for downstream alignment).

Misalignment is surfaced via `log.warning` (kept OUT of `node_state`, which stays a
simple value): at emit time, if the spread between the newest and oldest component
timestamps exceeds `MAX_STALE`, warn (a producer lagging its peers shows as the
spread). A producer that stops entirely stops the bundle stream — deliberately:
consumers must not act on a bundle that silently repeats a dead arm's last state,
and dora already logs node death. Before every input has been seen once, no partial
bundle is emitted.

## Mode: `collect`
- **Consumes:** the configured `INPUTS` (flat `float64` arrays), `program_state`.
- **Emits:** `<OUTPUT>` (concatenation of latest inputs, in order) + `node_state`.

## Config (env)
`MODE` (`collect`), **`INPUTS`** (CSV, ordered, required), `OUTPUT` (default `bundle`),
`MAX_STALE` (s, default `None` = no warning; set in the dataflow to ~`3/FPS`).
