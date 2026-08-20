# Experiments

Measurements and evaluations, one directory per experiment, named `YYYY-MM-DD-topic`.

Each contains the raw data, the ground truth, a runnable evaluator, and a write-up of the
findings — so results can be re-derived rather than taken on trust.

| Experiment | Question | Headline finding |
|---|---|---|
| [`2026-08-20-static-scene`](2026-08-20-static-scene/) | How repeatable is SmolVLM2-500M on one unchanging image? | pairwise agreement 0.12 on an identical image; latency stable to ±1.3 s; ground truth must come from the frame, not from memory |
