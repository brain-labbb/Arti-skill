from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts/audit_pva_table1_topologies.py"
)
SPEC = importlib.util.spec_from_file_location("audit_pva_table1_topologies_tested", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = AUDIT
SPEC.loader.exec_module(AUDIT)


def _row(
    category: str, topology: str | None, *, parsed: bool = True
) -> dict[str, object]:
    return {
        "raw_category": category,
        "parse_success": parsed,
        "valid_tree": topology is not None,
        "topology_hash": topology,
    }


def _fixture() -> tuple[list[dict[str, object]], dict[str, object]]:
    # Valid topology frequencies are A=[3,1,1] and B=[2,2,1].
    records = [
        _row("A", "h1"),
        _row("A", "h1"),
        _row("A", "h1"),
        _row("A", "h2"),
        _row("A", "h3"),
        _row("A", None, parsed=False),
        _row("B", "h1"),
        _row("B", "h1"),
        _row("B", "h2"),
        _row("B", "h2"),
        _row("B", "h4"),
    ]
    summary: dict[str, object] = {
        "n_eval": 11,
        "cohort": {"N_eval": 11, "N_parse": 10, "eval_raw_categories": 2},
        "unique_topologies": {
            "unique": 4,
            "denominator": 10,
            "rate": 0.4,
            "coverage_denominator": 11,
            "coverage_rate": 10 / 11,
        },
        "category_breakdown": {
            "A": {
                "N_eval": 6,
                "N_parse": 5,
                "unique_topologies": {"unique": 3, "denominator": 5, "rate": 0.6},
            },
            "B": {
                "N_eval": 5,
                "N_parse": 5,
                "unique_topologies": {"unique": 3, "denominator": 5, "rate": 0.6},
            },
        },
        "category_macro": {
            "category_count": 2,
            "unique_topologies_evaluable_categories": 2,
            "unique_topologies_rate": 0.6,
        },
    }
    return records, summary


def test_all_four_topology_views_and_exact_rarefaction() -> None:
    records, summary = _fixture()
    result = AUDIT.audit_topology_records(records, summary, k=2)

    assert result["pooled"] == {"unique": 4, "denominator": 10, "rate": 0.4}
    assert result["category_conditioned"] == {
        "unique": 6,
        "denominator": 10,
        "rate": 0.6,
    }
    assert result["category_macro"] == {"category_count": 2, "rate": 0.6}
    rarefaction = result["category_stratified_rarefaction"]
    assert rarefaction["k"] == 2
    assert rarefaction["category_count"] == 2
    assert rarefaction["expected_unique_macro"] == pytest.approx(1.75)
    assert rarefaction["rate"] == pytest.approx(0.875)
    assert rarefaction["exact_rate_numerator"] == "7"
    assert rarefaction["exact_rate_denominator"] == "8"
    assert result["validation"]["frozen_summary_consistent"] is True


def test_summary_drift_is_rejected_at_the_category_source() -> None:
    records, summary = _fixture()
    drifted = copy.deepcopy(summary)
    drifted["category_breakdown"]["A"]["unique_topologies"]["unique"] = 2

    with pytest.raises(
        AUDIT.TopologyAuditError,
        match=r"category_breakdown\.A\.unique_topologies\.unique mismatch",
    ):
        AUDIT.audit_topology_records(records, drifted, k=2)


def test_file_audit_checks_summary_self_hash_and_never_writes_inputs(
    tmp_path: Path,
) -> None:
    records, summary = _fixture()
    records_path = tmp_path / "asset_records.jsonl"
    summary_path = tmp_path / "summary.json"
    records_payload = "".join(
        json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n"
        for row in records
    )
    records_path.write_text(records_payload, encoding="utf-8")
    summary["summary_content_sha256"] = AUDIT.canonical_sha256(summary)
    summary_payload = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    summary_path.write_text(summary_payload, encoding="utf-8")

    result = AUDIT.audit_files(records_path, summary_path, k=2)

    assert result["category_stratified_rarefaction"]["rate"] == pytest.approx(0.875)
    assert result["validation"]["summary_content_sha256_verified"] is True
    assert records_path.read_text(encoding="utf-8") == records_payload
    assert summary_path.read_text(encoding="utf-8") == summary_payload

    tampered = json.loads(summary_payload)
    tampered["unique_topologies"]["unique"] = 3
    summary_path.write_text(json.dumps(tampered), encoding="utf-8")
    with pytest.raises(AUDIT.TopologyAuditError, match="summary_content_sha256 mismatch"):
        AUDIT.audit_files(records_path, summary_path, k=2)


def test_rarefaction_rejects_a_budget_larger_than_any_category() -> None:
    records, summary = _fixture()
    with pytest.raises(AUDIT.TopologyAuditError, match=r"k=6 exceeds.*2 categories"):
        AUDIT.audit_topology_records(records, summary, k=6)
