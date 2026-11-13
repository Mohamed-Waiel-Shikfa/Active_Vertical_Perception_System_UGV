#!/usr/bin/env python3
"""Score a candidate occupancy map against the truth map: table on stdout, optional JSON and heatmap."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from greenhouse.map_score import align, load_map, score, write_heatmap


def fmt(value, unit=""):
    return "-" if value is None else f"{value:.3f}{unit}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("truth", type=Path, help="truth map yaml (e.g. SETUP_DIR/greenhouse/nav-map/map.yaml)")
    parser.add_argument("candidate", type=Path, help="candidate map yaml, resampled onto the truth grid if needed")
    parser.add_argument("--heatmap", type=Path, help="write the difference heatmap PNG here")
    parser.add_argument("--json", type=Path, help="write the metrics JSON here")
    parser.add_argument("--tolerance-cells", type=int, default=0,
                        help="cells within which an occupied cell counts as correct (default 0; k=0 and k=1 are always reported)")
    args = parser.parse_args()
    if args.tolerance_cells < 0:
        parser.error("--tolerance-cells must be >= 0")
    try:
        truth, candidate = load_map(args.truth), load_map(args.candidate)
        results = [score(truth, candidate, k) for k in sorted({0, 1, args.tolerance_cells})]
        report = {"truth": str(args.truth), "candidate": str(args.candidate),
                  "resolution_m": truth.resolution, "grid_cells": list(truth.cells.shape),
                  "resampled": align(candidate, truth) is not candidate,
                  "coverage": results[0]["coverage"], "observed_fraction": results[0]["observed_fraction"],
                  "boundary_distance_m": results[0]["boundary_distance_m"],
                  "results": [{key: value for key, value in result.items()
                               if key not in ("coverage", "observed_fraction", "boundary_distance_m")}
                              for result in results]}
        print(f"grid {truth.cells.shape[0]}x{truth.cells.shape[1]} @ {truth.resolution} m"
              f"{' (candidate resampled)' if report['resampled'] else ''}")
        print(f"{'k':>3} {'precision':>9} {'recall':>7} {'f1':>6} {'iou':>6} | {'recall*':>7} {'iou*':>6}"
              f" {'tp':>7} {'fp':>7} {'fn':>7}")
        for result in report["results"]:
            counts, unknown = result["counts"], result["unknown_as_free"]
            print(f"{result['tolerance_cells']:>3} {fmt(result['precision']):>9} {fmt(result['recall']):>7}"
                  f" {fmt(result['f1']):>6} {fmt(result['iou']):>6} | {fmt(unknown['recall']):>7}"
                  f" {fmt(unknown['iou']):>6} {counts['true_positive']:>7} {counts['false_positive']:>7}"
                  f" {counts['false_negative']:>7}")
        print("* unknown counted as free")
        boundary = report["boundary_distance_m"]
        print(f"boundary m: cand->truth mean {fmt(boundary['candidate_to_truth_mean'])} p95 "
              f"{fmt(boundary['candidate_to_truth_p95'])}; truth->cand mean {fmt(boundary['truth_to_candidate_mean'])}"
              f" p95 {fmt(boundary['truth_to_candidate_p95'])}; symmetric mean {fmt(boundary['symmetric_mean'])}")
        print(f"coverage {fmt(report['coverage'])}  observed {fmt(report['observed_fraction'])}")
        if args.json:
            args.json.parent.mkdir(parents=True, exist_ok=True)
            args.json.write_text(json.dumps(report, indent=2) + "\n")
            print(f"json {args.json}")
        if args.heatmap:
            print(f"heatmap {write_heatmap(truth, candidate, args.heatmap, args.tolerance_cells)}"
                  f" (k={args.tolerance_cells})")
    except (OSError, ValueError, KeyError) as error:
        parser.exit(2, f"score_map: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
