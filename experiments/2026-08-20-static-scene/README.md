# Experiment: repeatability of SmolVLM2-500M on a single static scene

**Date:** 2026-08-20 · **Board:** Arduino UNO Q · **Backend:** CPU (4 threads), the working
configuration from [`arduino_uno_q/`](../../arduino_uno_q/)

## Method

The webcam was pointed at an unchanging scene and `run_vlm.sh -interval 5s` was left running
for 20 consecutive inferences with an identical prompt. **The image never changed.** Any
variation in the output is therefore variation in the model, not the scene.

Ground truth, confirmed by the operator:

> **A stuffed toy cat sitting on a rug, with a red book on top of it.**

This design deliberately separates two questions that a single run cannot:

1. **Accuracy** — is the description right? (needs ground truth)
2. **Repeatability** — do repeated runs agree with *each other*? (needs no ground truth,
   because the input is constant)

Metric 2 is the cheap one, and it turned out to be the damning one.

## Reproducing

```bash
python3 evaluate.py raw_log.txt ground_truth.json
```

Stdlib only. `raw_log.txt` is the verbatim console output; `RESULTS.md` is the generated
report.

## Results

### Element recall

| Element | Detected | Recall |
|---|---|---|
| cat (the subject) | 19/20 | 95% |
| **is_toy** (stuffed, not alive) | **2/20** | **10%** |
| rug | 6/20 | 30% |
| **book** (the object on top) | **1/20** | **5%** |
| red | 11/20 | 55% |

**Runs describing the scene completely: 0/20.**

### Hallucination

**17/20 runs invented at least one object that was not there**, averaging 1.3 per run:

| Invented | Runs | | Invented | Runs |
|---|---|---|---|---|
| bottle | 7 | | scooter, shoe box, harness | 1 each |
| helmet | 4 | | spray paint, bowl | 1 each |
| skateboard | 3 | | cushion, chair | 1 each |
| towel | 3 | | hat | 2 |

One run described the cat "in the act of peeing on a towel." One repeated the same two
sentences verbatim (degenerate decoding).

### Repeatability

Pairwise word overlap (Jaccard, all 190 pairs) on an **identical image**:

- **mean 0.12**, min 0.03, max 0.28

A static image should produce near-identical descriptions. 0.12 means successive runs share
roughly an eighth of their content words — they are effectively **independent guesses**.

### Latency (the one stable thing)

- mean **57.2 s**, stdev **1.3 s**, min 54.1 s, max 58.3 s — a 7% spread

## Findings

**1. The model is fast-failing in a slow way.** Timing is rock solid (±1.3 s); content is not
(Jaccard 0.12). Everything unreliable here is in the output, not the runtime.

**2. It missed the two things that define the scene.** The red book — the most distinctive
object present — appeared in **1 of 20** runs. That it is a *stuffed toy* appeared in **2 of
20**; the other 18 describe a live animal, with "curious eyes", playfulness, and in one case
urination. A downstream robot acting on "there is a cat here" would be acting on a plush toy.

**3. Colour survives; identity does not.** "Red" persists in 55% of runs, but attaches to a
different invented object nearly every time — helmet, skateboard, scooter, hat, shoe box,
harness. The model reliably sees *a red thing* and unreliably guesses *what*. Low-level
features are being extracted; object identity is not.

**4. Fluency masks failure.** Every output is confident, detailed, grammatical prose. Nothing
in the text signals low confidence. This is the same failure mode as the black-frame trap
documented in the main README — **the model has no way to say "I am not sure"**, so the
description reads identically whether it is right or invented.

## Implications for the manipulator project

This is a 500M-parameter model at Q8_0 on 4 ARM cores — the finding is about *this
configuration*, not about VLMs generally. Within that scope:

- **Do not use free-form description as a perception input.** 0/20 complete descriptions and
  1.3 hallucinated objects per run is not a signal a controller can act on.
- **Ask closed questions instead.** "Is there a red object? yes/no" is far more likely to be
  reliable than "describe the scene", and it is trivially checkable. Worth measuring next.
- **Sample and vote.** With Jaccard 0.12, a single run is close to a coin flip on any given
  detail. Elements appearing across *n* runs are much more trustworthy — at 57 s each, that
  is expensive, which pushes back toward closed questions.
- **Repeatability is the cheapest test you have.** It needed no labels, only a fixed camera,
  and it exposed the problem faster than any accuracy metric.

## Next experiments

1. Closed-form questions ("is there a book? yes/no") vs. free description, same scene.
2. A larger model (SmolVLM2-2.2B) if it fits in 3.6 GB — is this a capacity limit or a
   quantization artifact?
3. Q8_0 vs Q4 at equal model size, to separate quantization damage from model capacity.
4. Better lighting / closer framing — how much is the model versus a poor 720p webcam image?

## Files

| File | What |
|---|---|
| `raw_log.txt` | Verbatim console output of all 20 runs |
| `ground_truth.json` | Scene description and match terms |
| `evaluate.py` | The evaluator (stdlib only) |
| `RESULTS.md` | Generated report |

**Caveats.** Keyword matching approximates understanding: a run saying "plush toy" scores
`is_toy` regardless of whether the rest is right. Word-boundary matching is used because
substring matching silently inflates counts ("hat" inside "that", "red" inside "covered") —
an early version of this script had exactly that bug. The hallucination list is conservative:
surfaces like table/desk/wall are *not* counted as invented, since the scene plausibly had
one. The frame itself was not saved, so image quality cannot be re-examined — future runs
should keep the JPEG alongside the text.
