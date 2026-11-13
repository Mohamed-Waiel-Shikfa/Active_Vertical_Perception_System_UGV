# SPDX-License-Identifier: GPL-3.0-only
"""Score a candidate occupancy map against the truth map; write a colour-blind-safe difference heatmap.

Grids follow `navigation_map.create_map` and ROS map_server: row 0 is the +y edge, column 0 the -x edge,
`origin` is the world (greenhouse frame) position of the lower-left corner of the lower-left cell, metres.
Cell states: FREE 0, OCCUPIED 1, UNKNOWN -1.
"""
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from scipy import ndimage

FREE, OCCUPIED, UNKNOWN = 0, 1, -1
# Heatmap classes, drawn in this order in the legend. Okabe-Ito colours (RGB), colour-blind safe.
CLASSES = (("true positive", (0, 114, 178)), ("false positive", (230, 159, 0)),
           ("false negative", (204, 121, 167)), ("observed free", (255, 255, 255)),
           ("unobserved", (189, 189, 189)))


@dataclass(frozen=True)
class Grid:
    """Occupancy grid: `cells` int8 (rows, cols) of FREE/OCCUPIED/UNKNOWN, `resolution` m/cell, `origin` (x, y) m."""
    cells: np.ndarray
    resolution: float
    origin: tuple[float, float]


def load_map(yaml_path: Path | str) -> Grid:
    """Read a map_server map (yaml + image) and classify cells exactly as nav2 map_io does.

    occupancy p = shade if negate else 1 - shade, shade = mean channel value / 255; p > occupied_thresh is
    OCCUPIED, p < free_thresh FREE, otherwise UNKNOWN. `scale` mode is classified like `trinary` (scoring
    needs states, not probabilities); a transparent pixel (alpha < 255) is UNKNOWN. `raw` mode reads values
    0..100 as percent occupancy and anything else as UNKNOWN. Raises ValueError for a rotated origin or an
    unreadable image.
    """
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    origin = [float(v) for v in meta["origin"]]
    if len(origin) > 2 and abs(origin[2]) > 1e-9:
        raise ValueError("Rotated map origins are not supported")
    image_path = Path(meta["image"])
    image = cv2.imread(str(image_path if image_path.is_absolute() else yaml_path.parent/image_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"Cannot read map image {image_path}")
    alpha = image[..., 3] if image.ndim == 3 and image.shape[2] == 4 else None
    values = (image[..., :3].mean(axis=2) if image.ndim == 3 else image).astype(np.float64)
    mode = meta.get("mode", "trinary")
    occupied_thresh, free_thresh = float(meta["occupied_thresh"]), float(meta["free_thresh"])
    cells = np.full(values.shape, UNKNOWN, dtype=np.int8)
    if mode == "raw":
        known = values <= 100
        cells[known & (values/100 > occupied_thresh)] = OCCUPIED
        cells[known & (values/100 < free_thresh)] = FREE
    elif mode in ("trinary", "scale"):
        shade = values/255
        p = shade if int(meta.get("negate", 0)) else 1 - shade
        cells[p > occupied_thresh] = OCCUPIED
        cells[p < free_thresh] = FREE
        if alpha is not None:
            cells[alpha < 255] = UNKNOWN
    else:
        raise ValueError(f"Unsupported map mode {mode}")
    return Grid(cells, float(meta["resolution"]), (origin[0], origin[1]))


def save_map(grid: Grid, yaml_path: Path | str, negate: bool = False) -> None:
    """Write `grid` as a trinary map_server map: yaml_path plus a .pgm beside it with the same stem.

    Pixels 254 free / 0 occupied / 205 unknown (inverted when `negate`); free_thresh 0.196 as in ROS
    map_saver so that 205 loads as UNKNOWN. Raises OSError if the image cannot be written.
    """
    yaml_path = Path(yaml_path)
    image = np.full(grid.cells.shape, 205, dtype=np.uint8)
    image[grid.cells == FREE], image[grid.cells == OCCUPIED] = 254, 0
    if negate:
        image = 255 - image
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(yaml_path.with_suffix(".pgm")), image):
        raise OSError("Cannot write occupancy map image")
    meta = {"image": yaml_path.with_suffix(".pgm").name, "mode": "trinary", "resolution": grid.resolution,
            "origin": [grid.origin[0], grid.origin[1], 0.0], "negate": int(negate),
            "occupied_thresh": 0.65, "free_thresh": 0.196}
    yaml_path.write_text(yaml.safe_dump(meta, sort_keys=False))


def align(candidate: Grid, truth: Grid) -> Grid:
    """Resample `candidate` onto the truth grid geometry (nearest cell in world coordinates).

    Each truth cell takes the state of the candidate cell containing its centre; centres outside the
    candidate are UNKNOWN. Returns `candidate` unchanged when geometry already matches.
    """
    if (candidate.cells.shape == truth.cells.shape and math.isclose(candidate.resolution, truth.resolution)
            and np.allclose(candidate.origin, truth.origin)):
        return candidate
    rows, cols = truth.cells.shape
    x = truth.origin[0] + (np.arange(cols) + .5)*truth.resolution
    y = truth.origin[1] + (rows - 1 - np.arange(rows) + .5)*truth.resolution
    c = np.floor((x - candidate.origin[0])/candidate.resolution).astype(np.int64)
    r = candidate.cells.shape[0] - 1 - np.floor((y - candidate.origin[1])/candidate.resolution).astype(np.int64)
    rr, cc = np.meshgrid(r, c, indexing="ij")
    inside = (rr >= 0) & (rr < candidate.cells.shape[0]) & (cc >= 0) & (cc < candidate.cells.shape[1])
    cells = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    cells[inside] = candidate.cells[rr[inside], cc[inside]]
    return Grid(cells, truth.resolution, truth.origin)


def _distance_to(mask: np.ndarray) -> np.ndarray:
    """Euclidean distance in cells from every cell to the nearest True cell of `mask` (inf if none)."""
    return ndimage.distance_transform_edt(~mask) if mask.any() else np.full(mask.shape, np.inf)


def _boundary(occupied: np.ndarray) -> np.ndarray:
    """Occupied cells with a 4-neighbour that is not occupied (the grid edge counts as not occupied)."""
    return occupied & ~ndimage.binary_erosion(occupied, border_value=0)


def _ratio(numerator: int, denominator: int) -> float | None:
    return numerator/denominator if denominator else None


def _f1(precision: float | None, recall: float | None) -> float | None:
    if precision is None or recall is None:
        return None
    return 2*precision*recall/(precision + recall) if precision + recall else 0.0


def _stats(distances: np.ndarray, resolution: float) -> tuple[float | None, float | None]:
    if not distances.size or not np.isfinite(distances).all():
        return None, None
    return float(distances.mean()*resolution), float(np.percentile(distances, 95)*resolution)


def classify(truth: Grid, candidate: Grid, tolerance_cells: int = 0) -> np.ndarray:
    """Per-cell heatmap class index into CLASSES on the truth grid (candidate is aligned first).

    true positive: candidate occupied within `tolerance_cells` of truth occupied; false positive: other
    candidate occupied; false negative: observed truth-occupied cell with no candidate occupied within the
    tolerance; observed free: every other observed cell; unobserved: candidate unknown or truth unknown.
    """
    cells = align(candidate, truth).cells
    known = truth.cells != UNKNOWN
    truth_occ, observed = truth.cells == OCCUPIED, known & (cells != UNKNOWN)
    cand_occ = observed & (cells == OCCUPIED)
    tp = cand_occ & (_distance_to(truth_occ) <= tolerance_cells)
    fn = observed & truth_occ & ~cand_occ & (_distance_to(cand_occ) > tolerance_cells)
    classes = np.full(cells.shape, 4, dtype=np.uint8)
    classes[observed] = 3
    classes[fn] = 2
    classes[cand_occ & ~tp] = 1
    classes[tp] = 0
    return classes


def score(truth: Grid, candidate: Grid, tolerance_cells: int = 0) -> dict[str, Any]:
    """Occupancy, boundary and coverage metrics of `candidate` against `truth` (fractions 0..1, metres).

    Cells where truth is UNKNOWN are ignored. Precision, recall, F1 and IoU use only cells the candidate
    observed (FREE or OCCUPIED); `unknown_as_free` repeats recall/F1/IoU counting candidate UNKNOWN as FREE.
    With tolerance k a candidate occupied cell is a true positive when a truth occupied cell lies within k
    cells (Euclidean), and a truth occupied cell is recalled when a candidate occupied cell lies within k;
    IoU = TP / (TP + FP + FN), the usual IoU at k = 0. Boundary distances run between boundary cells
    (occupied cells touching a non-occupied 4-neighbour); truth-to-candidate uses observed truth boundary
    cells only; symmetric_mean is the mean over both directions' samples pooled. Coverage is the fraction
    of truth FREE cells the candidate observed. A metric with an empty denominator is None.
    """
    resolution = truth.resolution
    cells = align(candidate, truth).cells
    known = truth.cells != UNKNOWN
    truth_occ, truth_free = truth.cells == OCCUPIED, truth.cells == FREE
    observed = known & (cells != UNKNOWN)
    cand_occ = observed & (cells == OCCUPIED)
    near_truth, near_cand = _distance_to(truth_occ) <= tolerance_cells, _distance_to(cand_occ) <= tolerance_cells
    tp, fp = int((cand_occ & near_truth).sum()), int((cand_occ & ~near_truth).sum())

    def occupancy(region: np.ndarray) -> dict[str, Any]:
        recalled, missed = int((region & truth_occ & near_cand).sum()), int((region & truth_occ & ~near_cand).sum())
        precision, recall = _ratio(tp, tp + fp), _ratio(recalled, recalled + missed)
        return {"precision": precision, "recall": recall, "f1": _f1(precision, recall),
                "iou": _ratio(tp, tp + fp + missed),
                "counts": {"true_positive": tp, "false_positive": fp, "false_negative": missed}}

    observed_metrics, all_metrics = occupancy(observed), occupancy(known)
    truth_edge, cand_edge = _boundary(truth_occ), _boundary(cand_occ)
    to_truth = _distance_to(truth_edge)[cand_edge]
    to_cand = _distance_to(cand_edge)[truth_edge & observed]
    c2t_mean, c2t_p95 = _stats(to_truth, resolution)
    t2c_mean, t2c_p95 = _stats(to_cand, resolution)
    pooled = np.concatenate((to_truth, to_cand))
    return {"tolerance_cells": tolerance_cells, **observed_metrics,
            "unknown_as_free": {key: all_metrics[key] for key in ("recall", "f1", "iou", "counts")},
            "boundary_distance_m": {"candidate_to_truth_mean": c2t_mean, "candidate_to_truth_p95": c2t_p95,
                                    "truth_to_candidate_mean": t2c_mean, "truth_to_candidate_p95": t2c_p95,
                                    "symmetric_mean": _stats(pooled, resolution)[0]},
            "coverage": _ratio(int((truth_free & observed).sum()), int(truth_free.sum())),
            "observed_fraction": _ratio(int(observed.sum()), int(known.sum()))}


def write_heatmap(truth: Grid, candidate: Grid, output: Path | str, tolerance_cells: int = 0,
                  min_size_px: int = 800) -> Path:
    """Write the `classify` heatmap as a PNG on the truth grid with a legend strip below the map.

    Cells are upscaled by an integer factor (nearest) until the longer side is at least `min_size_px`.
    Returns the path; raises OSError if the image cannot be written.
    """
    classes = classify(truth, candidate, tolerance_cells)
    palette = np.array([colour[::-1] for _, colour in CLASSES], dtype=np.uint8)  # BGR for OpenCV
    scale = max(1, math.ceil(min_size_px/max(classes.shape)))
    image = np.repeat(np.repeat(palette[classes], scale, axis=0), scale, axis=1)
    width, swatch, row_height = image.shape[1], 18, 28
    per_row = max(1, (width - 10)//190)
    legend = np.full((10 + row_height*math.ceil(len(CLASSES)/per_row), width, 3), 255, dtype=np.uint8)
    for index, (name, _) in enumerate(CLASSES):
        x, y = 10 + (index % per_row)*190, 10 + (index//per_row)*row_height
        cv2.rectangle(legend, (x, y), (x + swatch, y + swatch), tuple(int(v) for v in palette[index]), -1)
        cv2.rectangle(legend, (x, y), (x + swatch, y + swatch), (0, 0, 0), 1)
        cv2.putText(legend, name, (x + swatch + 6, y + swatch - 4), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1,
                    cv2.LINE_AA)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), np.vstack((image, legend))):
        raise OSError(f"Cannot write heatmap {output}")
    return output
