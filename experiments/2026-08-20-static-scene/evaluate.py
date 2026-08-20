#!/usr/bin/env python3
"""Evaluate repeated VLM descriptions of a single static scene.

Three questions, three metric families:
  1. Accuracy    -- does the description contain the things that are actually there?
  2. Hallucination -- does it contain things that are not?
  3. Repeatability -- do repeated runs on an identical image agree with each other?

(3) needs no ground truth at all, which makes it the cheapest useful signal:
descriptions of one unchanging image should be near-identical, and the degree to
which they are not is a direct measure of how much the output can be trusted.

Stdlib only. Usage: ./evaluate.py [raw_log.txt] [ground_truth.json]
"""
import json
import re
import statistics
import sys
from itertools import combinations
from pathlib import Path

RUN_RE = re.compile(r"^\[(\d{2}:\d{2}:\d{2})\]\s+(.*?)\n\s+\(([\d.]+)s\)", re.M | re.S)

STOP = set("""a an the and or but of in on at to for with from is are was were be been being
it its it's this that these those there here as by into over under up down out off
which who whom whose what when where why how not no nor so than then too very can will
just also both each few more most other some such only own same s t don now image scene
picture appears appear seems seem suggests suggesting adding adds captured captures
overall sense moment its background foreground center centre""".split())


def load_runs(path):
    text = Path(path).read_text()
    runs = []
    for ts, body, secs in RUN_RE.findall(text):
        runs.append({"time": ts, "text": " ".join(body.split()), "seconds": float(secs)})
    return runs


def words(text):
    return [w for w in re.findall(r"[a-z']+", text.lower()) if w not in STOP and len(w) > 2]


def has_any(text, terms):
    """Word-boundary match. Substring matching is a trap here: "hat" hides inside
    "that"/"what", and "red" inside "covered" -- both silently inflate the counts."""
    low = text.lower()
    return any(re.search(r"\b" + re.escape(t) + r"s?\b", low) for t in terms)


def jaccard(a, b):
    A, B = set(a), set(b)
    return len(A & B) / len(A | B) if A | B else 0.0


def repeated_sentences(text):
    sents = [s.strip().lower() for s in re.split(r"(?<=[.!?])\s+", text) if len(s.strip()) > 25]
    seen, dupes = set(), 0
    for s in sents:
        if s in seen:
            dupes += 1
        seen.add(s)
    return dupes


def main():
    log = sys.argv[1] if len(sys.argv) > 1 else "raw_log.txt"
    gt_path = sys.argv[2] if len(sys.argv) > 2 else "ground_truth.json"
    gt = json.loads(Path(gt_path).read_text())
    runs = load_runs(log)
    if not runs:
        sys.exit("no runs parsed -- check the log format")

    n = len(runs)
    print(f"# Evaluation: {n} runs on one static scene\n")
    print(f"Ground truth: {gt['scene']}\n")

    # --- 1. accuracy ------------------------------------------------------
    print("## 1. Element recall\n")
    print("| Element | Detected | Recall |")
    print("|---|---|---|")
    per_run_hits = []
    for key, spec in gt["elements"].items():
        hits = [has_any(r["text"], spec["any_of"]) for r in runs]
        per_run_hits.append(hits)
        c = sum(hits)
        print(f"| `{key}` ({spec['note']}) | {c}/{n} | {100*c/n:.0f}% |")
    all_correct = sum(all(col) for col in zip(*per_run_hits))
    print(f"\nRuns containing **every** ground-truth element: **{all_correct}/{n}** "
          f"({100*all_correct/n:.0f}%)\n")

    # --- 2. hallucination -------------------------------------------------
    print("## 2. Hallucinated objects\n")
    markers = gt["hallucination_markers"]
    counts = {m: sum(has_any(r["text"], [m]) for r in runs) for m in markers}
    per_run = [sum(has_any(r["text"], [m]) for m in markers) for r in runs]
    print("| Invented object | Runs mentioning it |")
    print("|---|---|")
    for m, c in sorted(counts.items(), key=lambda kv: -kv[1]):
        if c:
            print(f"| {m} | {c}/{n} |")
    print(f"\nRuns with at least one invented object: **{sum(1 for p in per_run if p)}/{n}**")
    print(f"Mean invented objects per run: **{statistics.mean(per_run):.1f}**\n")

    # --- 3. repeatability -------------------------------------------------
    print("## 3. Repeatability (same image, every run)\n")
    toks = [words(r["text"]) for r in runs]
    sims = [jaccard(a, b) for a, b in combinations(toks, 2)]
    print(f"Pairwise word-overlap (Jaccard) across all {len(sims)} run pairs:")
    print(f"- mean **{statistics.mean(sims):.2f}**, "
          f"min {min(sims):.2f}, max {max(sims):.2f}")
    print("- 1.00 would mean identical descriptions; a static image should score high.\n")

    dupes = sum(repeated_sentences(r["text"]) for r in runs)
    degen = sum(1 for r in runs if repeated_sentences(r["text"]))
    print(f"Degenerate repetition (a sentence repeated verbatim within one output): "
          f"**{degen}/{n} runs**, {dupes} repeated sentences total.\n")

    # --- 4. latency -------------------------------------------------------
    secs = [r["seconds"] for r in runs]
    print("## 4. Latency\n")
    print(f"- mean **{statistics.mean(secs):.1f} s**, "
          f"stdev {statistics.pstdev(secs):.1f} s, "
          f"min {min(secs):.1f} s, max {max(secs):.1f} s")
    print(f"- spread: {max(secs)-min(secs):.1f} s "
          f"({100*(max(secs)-min(secs))/statistics.mean(secs):.0f}% of the mean)\n")

    # --- per-run table ----------------------------------------------------
    print("## 5. Per-run detail\n")
    keys = list(gt["elements"])
    print("| # | Time | " + " | ".join(keys) + " | Invented | s |")
    print("|---|---|" + "---|" * (len(keys) + 2))
    for i, r in enumerate(runs, 1):
        marks = " | ".join("Y" if has_any(r["text"], gt["elements"][k]["any_of"]) else "." for k in keys)
        inv = sum(has_any(r["text"], [m]) for m in markers)
        print(f"| {i} | {r['time']} | {marks} | {inv} | {r['seconds']:.1f} |")


if __name__ == "__main__":
    main()
