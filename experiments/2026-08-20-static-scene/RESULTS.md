# Evaluation: 20 runs on one static scene

Ground truth: A stuffed toy cat behind a shaggy grey-and-white mop head, on a dark cluttered desk. A red-and-white object (book/box) sits above the cat; a desk lamp is upper right; a dark bottle stands at the left edge.

## 1. Element recall

| Element | Detected | Recall |
|---|---|---|
| `cat` (the subject) | 19/20 | 95% |
| `is_toy` (it is NOT a live animal) | 2/20 | 10% |
| `shaggy` (the shaggy mop head -- ANY of these is a fair reading) | 14/20 | 70% |
| `red` (the red object above the cat) | 11/20 | 55% |
| `bottle` (really is a bottle at the left edge) | 7/20 | 35% |
| `dark` (the frame is 64% near-black) | 12/20 | 60% |

Runs containing **every** ground-truth element: **0/20** (0%)

## 2. Hallucinated objects

| Invented object | Runs mentioning it |
|---|---|
| helmet | 4/20 |
| skateboard | 3/20 |
| scooter | 1/20 |
| shoe box | 1/20 |
| harness | 1/20 |
| spray paint | 1/20 |
| chair | 1/20 |
| bowl | 1/20 |

Runs with at least one invented object: **13/20**
Mean invented objects per run: **0.7**

## 3. Repeatability (same image, every run)

Pairwise word-overlap (Jaccard) across all 190 run pairs:
- mean **0.12**, min 0.03, max 0.28
- 1.00 would mean identical descriptions; a static image should score high.

Degenerate repetition (a sentence repeated verbatim within one output): **1/20 runs**, 2 repeated sentences total.

## 4. Latency

- mean **57.2 s**, stdev 1.3 s, min 54.1 s, max 58.3 s
- spread: 4.3 s (7% of the mean)

## 5. Per-run detail

| # | Time | cat | is_toy | shaggy | red | bottle | dark | Invented | s |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 16:36:46 | . | Y | . | Y | . | . | 0 | 58.1 |
| 2 | 16:37:52 | Y | . | Y | Y | Y | Y | 1 | 57.6 |
| 3 | 16:38:57 | Y | . | . | Y | . | Y | 1 | 54.3 |
| 4 | 16:39:59 | Y | . | Y | . | . | Y | 1 | 57.1 |
| 5 | 16:41:04 | Y | . | . | . | . | Y | 1 | 58.1 |
| 6 | 16:42:10 | Y | . | Y | Y | . | . | 1 | 54.2 |
| 7 | 16:43:12 | Y | . | Y | . | . | Y | 0 | 57.9 |
| 8 | 16:44:17 | Y | . | Y | Y | . | . | 1 | 58.0 |
| 9 | 16:45:23 | Y | . | Y | . | . | Y | 0 | 57.0 |
| 10 | 16:46:28 | Y | . | Y | . | . | . | 1 | 58.3 |
| 11 | 16:47:34 | Y | . | . | Y | . | . | 1 | 56.6 |
| 12 | 16:48:38 | Y | . | Y | Y | Y | . | 1 | 57.9 |
| 13 | 16:49:44 | Y | . | Y | Y | . | Y | 1 | 57.2 |
| 14 | 16:50:49 | Y | . | Y | . | . | Y | 0 | 58.2 |
| 15 | 16:51:55 | Y | . | . | . | Y | Y | 1 | 57.9 |
| 16 | 16:53:00 | Y | . | Y | Y | Y | Y | 0 | 57.4 |
| 17 | 16:54:06 | Y | . | Y | . | Y | Y | 0 | 57.8 |
| 18 | 16:55:11 | Y | . | Y | . | Y | . | 1 | 54.1 |
| 19 | 16:56:13 | Y | . | . | Y | . | Y | 0 | 57.7 |
| 20 | 16:57:18 | Y | Y | Y | Y | Y | . | 1 | 57.9 |
