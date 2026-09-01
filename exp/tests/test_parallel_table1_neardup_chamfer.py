from __future__ import annotations

import importlib.util
from pathlib import Path
import struct

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/parallel_table1_neardup_chamfer.py"
SPEC = importlib.util.spec_from_file_location("parallel_table1_neardup_tested", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)


def test_compute_chunk_matches_frozen_chamfer(tmp_path: Path) -> None:
    points = np.asarray(
        (
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        ),
        dtype=np.float64,
    )
    points_path = tmp_path / "points.npy"
    pairs_path = tmp_path / "pairs.bin"
    np.save(points_path, points)
    np.asarray([(0, 1, 0.0)], dtype=HELPER.PAIR_DTYPE).tofile(pairs_path)

    HELPER.init_worker(str(points_path), str(pairs_path), 1)
    start, payload = HELPER.compute_chunk((0, 1))

    assert start == 0
    left, right, distance = HELPER.CHAMFER_STRUCT.unpack(payload)
    assert (left, right) == (0, 1)
    assert distance == pytest.approx(HELPER.RUNNER.symmetric_chamfer(points[0], points[1]))


def test_validate_prefix_rejects_candidate_identity_mismatch(tmp_path: Path) -> None:
    pairs_path = tmp_path / "pairs.bin"
    temporary = tmp_path / "chamfer_pairs.bin.tmp"
    np.asarray([(0, 1, 0.0)], dtype=HELPER.PAIR_DTYPE).tofile(pairs_path)
    temporary.write_bytes(HELPER.CHAMFER_STRUCT.pack(0, 2, 0.1))

    with pytest.raises(ValueError, match="prefix right-ordinal mismatch"):
        HELPER.validate_prefix(temporary, pairs_path, 1, 1)
