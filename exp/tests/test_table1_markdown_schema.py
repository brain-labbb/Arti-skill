from __future__ import annotations

from pathlib import Path


DOCUMENT = Path(__file__).resolve().parents[1] / "URDF-Sim-Ready-Automatic-Evaluation.md"


def _table1_block() -> str:
    text = DOCUMENT.read_text(encoding="utf-8")
    start = text.index("## Table 1.")
    end = text.index("## Table 2.", start)
    return text[start:end]


def test_table1_uses_reduced_schema_and_mean_p90() -> None:
    block = _table1_block()
    header = next(
        line
        for line in block.splitlines()
        if line.startswith("| Dataset / Outputs | N | Links/Asset")
    )
    expected = (
        "| Dataset / Outputs | N | Links/Asset (mean / P90) | "
        "Movable Joints/Asset (mean / P90) | Var. Joints ↑ | "
        "Near-Duplicate Rate ↓ | Multi-joint Assets (%) ↑ |"
    )
    assert header == expected
    assert "N_release" not in header
    assert "N_eval" not in header
    assert "Observed Labels" not in header
    assert "Pooled Raw-tree Support" not in header
    assert "Exact Duplicate Rate" not in header
    assert "median" not in header.lower()


def test_table1_contains_proxy_diagnostic_receipt_and_all_rows() -> None:
    block = _table1_block()
    assert "make_proxy_neardup_labels.py" in block
    assert "tau = 0.011901784067423636" in block
    assert "all_pass=true" in block
    assert "candidate lower bound" in block
    for dataset in (
        "Ours-500K",
        "Ours / PV-A",
        "Articraft-10K",
        "LAM released outputs",
        "Artiverse",
        "PartNet-Mobility",
        "PhysX-Mobility",
        "SketchMobility",
        "Infinite Mobility",
        "Infinigen-Sim",
    ):
        assert dataset in block
