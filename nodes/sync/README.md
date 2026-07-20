# sync — reusable aggregator (completion-emit, skew-aware)

A generic node that **collects several event-driven inputs and emits one concatenated bundle
the moment every input has a fresh sample** (once per producer cycle). It's the missing
primitive for turning per-node streams into a synchronized whole-robot snapshot — e.g. two
leader arms → one `command`, two follower states → one `state`. (dora has no built-in
join/sync; this generalizes the timestamp logic the `lerobot` recorder already uses.)

Robot-agnostic: it concatenates the configured `INPUTS` **in order**; the consumer's scene
descriptor layout defines what the resulting vector means.

## Why emit-on-completion (not emit-on-tick)

Emitting on the node's own `tick` adds a timer-phase delay: the producers and this node sit
on independent timers, so the emitted bundle trails the freshest samples by up to a full
frame. Emitting the instant the LAST input of a cycle arrives removes that delay — the only
added latency is the few-ms spread between the producers' publishes. The bundle's
`timestamp` is the OLDEST component timestamp (honest for downstream alignment). The `tick`
is kept purely as a watchdog, surfacing misalignment via `log.warning` (kept OUT of
`node_state`, which stays a simple value):
- **stale**: if any input's latest value is older than `MAX_STALE`, warn. A producer that
  stalls or falls behind the others shows up as *its* age being high (so this catches a
  phase skew too); all ages high = everything froze. A stalled producer also stops the
  bundle stream entirely (no completion) — deliberately: consumers must not act on a bundle
  that silently repeats a dead arm's last state.
- **missing**: before every input has been seen once, it does not emit a partial bundle — it
  reports what it's waiting for.

## Mode: `collect`
- **Consumes:** the configured `INPUTS` (flat `float64` arrays), `tick` (watchdog), `program_state`.
- **Emits:** `<OUTPUT>` (concatenation of latest inputs, in order) + `node_state`.

## Config (env)
`MODE` (`collect`), **`INPUTS`** (CSV, ordered, required), `OUTPUT` (default `bundle`),
`MAX_STALE` (s, default `None` = no warning; set in the dataflow to ~`3/FPS`). Debug port: 5688.
