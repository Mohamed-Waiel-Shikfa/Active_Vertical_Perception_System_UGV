import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy import ndimage

from greenhouse.map_score import CLASSES, FREE, OCCUPIED, UNKNOWN, Grid, align, classify, load_map, save_map, score, write_heatmap

RES = .05


def greenhouse_like() -> Grid:
    """120 x 200 cells: a 2-cell perimeter wall and five 3-cell-thick rows along +x with headland gaps."""
    cells = np.zeros((120, 200), dtype=np.int8)
    cells[:2, :] = cells[-2:, :] = cells[:, :2] = cells[:, -2:] = OCCUPIED
    for row in range(20, 110, 20):
        cells[row:row + 3, 30:170] = OCCUPIED
    return Grid(cells, RES, (-5.0, -3.0))


def variant(grid: Grid, cells: np.ndarray) -> Grid:
    return Grid(cells.astype(np.int8), grid.resolution, grid.origin)


def shifted(grid: Grid, rows: int) -> Grid:
    cells = np.full(grid.cells.shape, FREE, dtype=np.int8)
    cells[rows:] = grid.cells[:-rows]
    return variant(grid, cells)


class MapScoreTests(unittest.TestCase):
    def setUp(self):
        self.truth = greenhouse_like()
        self.occ = self.truth.cells == OCCUPIED

    def test_identity_is_perfect(self):
        result = score(self.truth, self.truth)
        for key in ("precision", "recall", "f1", "iou", "coverage", "observed_fraction"):
            self.assertEqual(result[key], 1.0, key)
        self.assertEqual(set(result["boundary_distance_m"].values()), {0.0})
        self.assertEqual(result["counts"]["false_positive"] + result["counts"]["false_negative"], 0)
        self.assertEqual(result["unknown_as_free"]["iou"], 1.0)

    def test_shift_degrades_monotonically_and_tolerance_recovers(self):
        previous = score(self.truth, self.truth)
        for rows in (1, 2, 3):
            result = score(self.truth, shifted(self.truth, rows))
            self.assertLess(result["iou"], previous["iou"])
            self.assertLess(result["precision"], previous["precision"])
            self.assertGreater(result["boundary_distance_m"]["symmetric_mean"],
                               previous["boundary_distance_m"]["symmetric_mean"])
            self.assertLessEqual(result["boundary_distance_m"]["candidate_to_truth_p95"], rows*RES + 1e-9)
            self.assertEqual(score(self.truth, shifted(self.truth, rows), rows)["precision"], 1.0)
            previous = result

    def test_erode_keeps_precision_dilate_keeps_recall(self):
        eroded = score(self.truth, variant(self.truth, ndimage.binary_erosion(self.occ, border_value=1)))
        self.assertEqual(eroded["precision"], 1.0)
        self.assertLess(eroded["recall"], 1.0)
        dilated_cells = ndimage.binary_dilation(self.occ)
        dilated = score(self.truth, variant(self.truth, dilated_cells))
        self.assertEqual(dilated["recall"], 1.0)
        self.assertLess(dilated["precision"], 1.0)
        self.assertAlmostEqual(dilated["iou"], self.occ.sum()/dilated_cells.sum())
        self.assertGreater(dilated["boundary_distance_m"]["candidate_to_truth_mean"], 0.0)
        self.assertAlmostEqual(dilated["boundary_distance_m"]["candidate_to_truth_p95"], RES)
        self.assertEqual(score(self.truth, variant(self.truth, dilated_cells), 1)["precision"], 1.0)

    def test_salt_noise_counts_exact_false_positives(self):
        free = np.argwhere(~self.occ)
        picks = free[np.random.default_rng(7).choice(len(free), 150, replace=False)]
        cells = self.truth.cells.copy()
        cells[picks[:, 0], picks[:, 1]] = OCCUPIED
        result = score(self.truth, variant(self.truth, cells))
        self.assertEqual(result["recall"], 1.0)
        self.assertEqual(result["counts"]["false_positive"], 150)
        self.assertAlmostEqual(result["precision"], self.occ.sum()/(self.occ.sum() + 150))

    def test_unknown_block_drops_coverage_not_precision(self):
        cells = self.truth.cells.copy()
        cells[10:50, 40:90] = UNKNOWN
        result = score(self.truth, variant(self.truth, cells))
        block_free = int((self.truth.cells[10:50, 40:90] == FREE).sum())
        self.assertEqual(result["precision"], 1.0)
        self.assertEqual(result["recall"], 1.0)
        self.assertAlmostEqual(result["coverage"], 1 - block_free/int((~self.occ).sum()))
        hidden = int(self.occ[10:50, 40:90].sum())
        self.assertAlmostEqual(result["unknown_as_free"]["recall"], 1 - hidden/self.occ.sum())

    def test_missing_wall_drops_recall(self):
        cells = self.truth.cells.copy()
        cells[40:43, 30:170] = FREE
        result = score(self.truth, variant(self.truth, cells))
        self.assertEqual(result["precision"], 1.0)
        self.assertAlmostEqual(result["recall"], 1 - 3*140/self.occ.sum())
        self.assertAlmostEqual(result["iou"], result["recall"])
        self.assertGreater(result["boundary_distance_m"]["truth_to_candidate_p95"], 0.0)
        self.assertEqual(result["boundary_distance_m"]["candidate_to_truth_mean"], 0.0)

    def test_alignment_of_other_origin_and_resolution(self):
        fine = np.repeat(np.repeat(self.truth.cells, 2, axis=0), 2, axis=1)
        padded = np.full((fine.shape[0] + 14, fine.shape[1] + 10), UNKNOWN, dtype=np.int8)
        padded[:fine.shape[0], 10:] = fine  # 10 extra columns on -x, 14 extra rows on -y
        candidate = Grid(padded, RES/2, (self.truth.origin[0] - 10*RES/2, self.truth.origin[1] - 14*RES/2))
        np.testing.assert_array_equal(align(candidate, self.truth).cells, self.truth.cells)
        self.assertEqual(score(self.truth, candidate)["iou"], 1.0)
        partial = Grid(self.truth.cells[:, 100:], RES, (self.truth.origin[0] + 100*RES, self.truth.origin[1]))
        result = score(self.truth, partial)
        self.assertEqual(result["observed_fraction"], .5)
        self.assertEqual(result["precision"], 1.0)

    def test_round_trip_with_negate(self):
        cells = self.truth.cells.copy()
        cells[5:15, 5:15] = UNKNOWN
        grid = variant(self.truth, cells)
        with tempfile.TemporaryDirectory() as temp:
            for negate in (False, True):
                save_map(grid, Path(temp)/f"map{int(negate)}.yaml", negate)
                loaded = load_map(Path(temp)/f"map{int(negate)}.yaml")
                np.testing.assert_array_equal(loaded.cells, cells)
                self.assertEqual((loaded.resolution, loaded.origin), (RES, self.truth.origin))

    def test_thresholds_negate_and_modes(self):
        values = np.array([[0, 254, 205, 128, 255, 100]], dtype=np.uint8)
        with tempfile.TemporaryDirectory() as temp:
            cv2.imwrite(str(Path(temp)/"m.pgm"), values)

            def load(**meta):
                base = {"image": "m.pgm", "mode": "trinary", "resolution": .1, "origin": [1.0, 2.0, 0.0],
                        "negate": 0, "occupied_thresh": .65, "free_thresh": .25}
                (Path(temp)/"m.yaml").write_text(yaml.safe_dump({**base, **meta}))
                return load_map(Path(temp)/"m.yaml").cells[0].tolist()

            O, F, U = OCCUPIED, FREE, UNKNOWN
            self.assertEqual(load(), [O, F, F, U, F, U])  # nav2: 205 -> p 0.196 < 0.25 is free
            self.assertEqual(load(free_thresh=.196), [O, F, U, U, F, U])
            self.assertEqual(load(negate=1), [F, O, O, U, O, U])
            self.assertEqual(load(mode="scale"), load())
            self.assertEqual(load(mode="raw"), [F, U, U, U, U, O])
            with self.assertRaises(ValueError):
                load(origin=[0.0, 0.0, .5])

    def test_heatmap_classes_and_image(self):
        cells = self.truth.cells.copy()
        cells[40:43, 30:170] = FREE          # false negatives
        cells[10, 10:20] = OCCUPIED         # false positives
        cells[60:80, 60:80] = UNKNOWN       # unobserved
        candidate = variant(self.truth, cells)
        classes = classify(self.truth, candidate)
        self.assertEqual(int((classes == 2).sum()), 3*140)
        self.assertEqual(int((classes == 1).sum()), 10)
        self.assertEqual(int((classes == 4).sum()), 400)
        with tempfile.TemporaryDirectory() as temp:
            path = write_heatmap(self.truth, candidate, Path(temp)/"heat.png")
            image = cv2.imread(str(path))
            self.assertEqual(image.shape[1], 200*4)
            self.assertGreater(image.shape[0], 120*4)
            colours = {tuple(int(v) for v in pixel[::-1]) for pixel in image[:120*4].reshape(-1, 3)}
            self.assertEqual(colours, {colour for _, colour in CLASSES})

    def test_cli_writes_json(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp:
            save_map(self.truth, Path(temp)/"truth.yaml")
            save_map(shifted(self.truth, 1), Path(temp)/"cand.yaml")
            run = subprocess.run([sys.executable, str(root/"tools/score_map.py"), str(Path(temp)/"truth.yaml"),
                                  str(Path(temp)/"cand.yaml"), "--json", str(Path(temp)/"out.json"),
                                  "--heatmap", str(Path(temp)/"heat.png"), "--tolerance-cells", "2"],
                                 capture_output=True, text=True, check=True)
            self.assertIn("coverage", run.stdout)
            report = json.loads((Path(temp)/"out.json").read_text())
            self.assertEqual([r["tolerance_cells"] for r in report["results"]], [0, 1, 2])
            self.assertEqual(report["coverage"], 1.0)
            self.assertFalse(report["resampled"])
            self.assertTrue((Path(temp)/"heat.png").exists())


if __name__ == "__main__":
    unittest.main()
