from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/make_proxy_neardup_labels.py"
SPEC = importlib.util.spec_from_file_location("proxy_neardup_labels_tested", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
PROXY = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PROXY
SPEC.loader.exec_module(PROXY)


def test_proxy_rubric_has_explicit_uncertain_band() -> None:
    assert PROXY.classify(0.011999, 0.012, 0.015) == "duplicate"
    assert PROXY.classify(0.012, 0.012, 0.015) == "uncertain"
    assert PROXY.classify(0.014999, 0.012, 0.015) == "uncertain"
    assert PROXY.classify(0.015, 0.012, 0.015) == "not_duplicate"


def test_build_labels_is_sorted_and_declares_non_human_source() -> None:
    rows = [
        {"pair_id": "b", "dataset_key": "d", "selection_mode": "random", "chamfer_distance": 0.02},
        {"pair_id": "a", "dataset_key": "d", "selection_mode": "hard", "chamfer_distance": 0.01},
        {"pair_id": "c", "dataset_key": "d", "selection_mode": "hard", "chamfer_distance": 0.013},
    ]
    labels, receipt = PROXY.build_labels(rows, low=0.012, high=0.015)
    assert [row["pair_id"] for row in labels] == ["a", "b", "c"]
    assert [row["label"] for row in labels] == ["duplicate", "not_duplicate", "uncertain"]
    assert all(row["annotation_source"] == "synthetic_proxy_not_human" for row in labels)
    assert receipt["human_annotation_claim"] is False
    assert receipt["label_counts"] == {"duplicate": 1, "not_duplicate": 1, "uncertain": 1}
