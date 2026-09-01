#!/usr/bin/env python3
"""Read-only acceptance checks for the Table 1/2/3 full-release receipts.

This command deliberately never imports or invokes a full-release runner.  It
rechecks the frozen rosters, the 24 published table artifacts, independent
reaggregation, and the three primary Markdown tables.  Use ``--pytest`` only
when the small contract suite is also desired; the default path does not
rerun any dataset evaluation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

try:
    import table123_full_release_common as common
    from verify_table123_full_release import (
        VerificationError,
        verify_dataset_receipts,
    )
except ImportError:  # pragma: no cover - package-style import
    from . import table123_full_release_common as common
    from .verify_table123_full_release import (
        VerificationError,
        verify_dataset_receipts,
    )


class AutomationError(ValueError):
    """Raised when a published full-release contract is not closed."""


DATASETS: tuple[dict[str, Any], ...] = (
    {"slug": "articraft", "display": "Articraft-10K", "n_eval": 9_996, "j_eval": 37_144},
    {"slug": "lam", "display": "LAM released outputs", "dataset_names": ("LAM", "LAM released outputs"), "n_eval": 3_217, "j_eval": 10_381},
    {"slug": "artiverse", "display": "Artiverse", "n_eval": 3_544, "j_eval": 16_332},
    {"slug": "partnet", "display": "PartNet-Mobility", "n_eval": 2_347, "j_eval": 11_971},
    {"slug": "physx", "display": "PhysX-Mobility", "n_eval": 2_024, "j_eval": 9_883},
    {"slug": "sketch", "display": "SketchMobility", "n_eval": 4_956, "j_eval": 11_009},
    {"slug": "infinite", "display": "Infinite Mobility", "n_eval": 720, "j_eval": 4_723},
    {"slug": "infinigen", "display": "Infinigen-Sim", "n_eval": 8_226, "j_eval": 31_975},
)

TABLES = ("table1", "table2", "table3")
TABLE2_METRICS = (
    "parse_rate",
    "resource_resolution",
    "finite_fields",
    "valid_tree",
    "valid_joint_spec",
    "collision_coverage",
    "inertial_coverage",
    "inertia_validity",
    "strict_urdf_pass",
)
TABLE3_PAIR_METRICS = (
    "valid_range",
    "joint_sweep_success",
    "non_degenerate_motion",
    "subtree_consistency",
    "joint_level_pass",
)
TABLE3_ALL_METRICS = TABLE3_PAIR_METRICS + (
    "fk_roundtrip_error",
    "strict_kinematic_pass",
)

# Frozen fingerprints of the six Ours/Brain rows in the primary Tables 1--3
# snapshot.  Comparison rows may change with a new full-release receipt, but
# these rows are explicitly outside this task's scope and must stay byte-stable.
OURS_PRIMARY_ROW_SHA256 = {
    "table1:Ours-500K": "037b6efd5b9ff46a8ac876cd32a1a958c9c6ec53dea8c0f5336fa044c6ae8a50",
    "table1:Ours per-class N=5 (supplementary)": "89fcab7b9c43847f9b635d4940585e1cb5dfb1bf94e0dab4c1577f98e5d41bdd",
    "table2:Ours-500K": "eaee8768ecc697b53913d4a941c9195e0a6f86d293eabcdc5120326d7950c4a2",
    "table2:Ours per-class N=5 (supplementary)": "8cc5e1f5d3b1d5b2b26cc7bc6f838ed7c36ec300fb03ddda25161d0df4939667",
    "table3:Ours-500K": "af1f31837c3a589dd8be527a5d4225c7de09f86a408038e1bbb3a85ed7b9ad4e",
    "table3:Ours per-class N=5 (supplementary)": "e61a5b7c4a921b2d53c52adb4695bbc2b41035aea8028a1a63f0e7028986229c",
}


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise AutomationError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise AutomationError(f"expected JSON object: {path}")
    return value


def _record_name(output: Path) -> Path:
    for name in ("records.jsonl", "asset_records.jsonl"):
        if (output / name).is_file():
            return output / name
    raise AutomationError(f"missing records JSONL: {output}")


def _status_counts(output: Path) -> dict[str, int]:
    counts: dict[str, int] = {}
    try:
        lines = _record_name(output).read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise AutomationError(f"cannot read table records: {output}: {error}") from error
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as error:
            raise AutomationError(f"invalid record JSON {output}:{line_number}: {error}") from error
        status = str(value.get("status", "unknown")) if isinstance(value, dict) else "unknown"
        counts[status] = counts.get(status, 0) + 1
    return dict(sorted(counts.items()))


def _table1_metrics(summary: Mapping[str, Any]) -> dict[str, Any]:
    names = (
        "links_per_asset",
        "movable_joints_per_asset",
        "multi_joint_assets",
        "unique_topologies",
        "exact_duplicate_rate",
    )
    return {name: summary.get(name, {}) for name in names}


def _int(value: Any, *, field: str) -> int:
    if isinstance(value, bool):
        raise AutomationError(f"{field} is not an integer: {value!r}")
    try:
        return int(str(value).replace(",", "").strip())
    except (TypeError, ValueError) as error:
        raise AutomationError(f"{field} is not an integer: {value!r}") from error


def _section(markdown: str, table_number: int) -> str:
    pattern = re.compile(rf"^## Table {table_number}\.\s.*$", re.MULTILINE)
    match = pattern.search(markdown)
    if match is None:
        raise AutomationError(f"Markdown is missing primary Table {table_number} heading")
    next_heading = re.search(r"^##\s+", markdown[match.end() :], re.MULTILINE)
    end = match.end() + next_heading.start() if next_heading else len(markdown)
    return markdown[match.start() : end]


def _table_rows(section: str) -> list[list[str]]:
    rows: list[list[str]] = []
    for line in section.splitlines():
        stripped = line.strip()
        if not stripped.startswith("|"):
            continue
        cells = [cell.strip() for cell in stripped.strip("|").split("|")]
        if not cells or all(re.fullmatch(r":?-{2,}:?", cell or "") for cell in cells):
            continue
        rows.append(cells)
    return rows


def _display_row(rows: Sequence[Sequence[str]], display: str) -> list[str]:
    for row in rows:
        if not row:
            continue
        first = row[0]
        if first == display or first.startswith(display + " ("):
            return list(row)
    raise AutomationError(f"Markdown primary table is missing comparison row: {display}")


def _pair(cell: str) -> tuple[int, int] | None:
    # Counts in the primary tables are always rendered as numerator / denominator.
    match = re.search(r"(?<![\w.])([0-9][0-9,]*)\s*/\s*([0-9][0-9,]*)", cell)
    if match is None:
        return None
    return _int(match.group(1), field="Markdown numerator"), _int(match.group(2), field="Markdown denominator")


def _assert_pair(cell: str, expected: Mapping[str, Any], *, label: str) -> bool:
    observed = _pair(cell)
    if observed is None:
        raise AutomationError(f"Markdown metric is not a numeric pair for {label}: {cell!r}")
    expected_passed = expected.get("passed", expected.get("pass_count"))
    expected_denominator = expected.get("denominator")
    if expected_passed is not None and observed[0] != int(expected_passed):
        raise AutomationError(f"Markdown metric mismatch for {label}: passed {observed[0]} != {expected_passed}")
    if expected_denominator is not None and observed[1] != int(expected_denominator):
        raise AutomationError(f"Markdown metric mismatch for {label}: denominator {observed[1]} != {expected_denominator}")
    return True


def _assert_fk(cell: str, expected: Mapping[str, Any], *, label: str) -> bool:
    match = re.search(r"\(([0-9][0-9,]*)\s*/\s*([0-9][0-9,]*)\s+measured", cell)
    if match is None:
        raise AutomationError(f"Markdown FK cell is not measurable for {label}: {cell!r}")
    observed = (_int(match.group(1), field="FK measured count"), _int(match.group(2), field="FK denominator"))
    measured = expected.get("measured_joint_count")
    denominator = expected.get("denominator")
    if measured is not None and observed[0] != int(measured):
        raise AutomationError(f"Markdown FK mismatch for {label}: measured {observed[0]} != {measured}")
    if denominator is not None and observed[1] != int(denominator):
        raise AutomationError(f"Markdown FK mismatch for {label}: denominator {observed[1]} != {denominator}")
    return True


def _assert_triplet(cell: str, expected: Mapping[str, Any], *, label: str) -> None:
    match = re.search(
        r"([0-9]+(?:\.[0-9]+)?)\s*/\s*([0-9]+(?:\.[0-9]+)?)\s*/\s*([0-9]+(?:\.[0-9]+)?)",
        cell,
    )
    if match is None:
        raise AutomationError(f"Markdown triplet is not numeric for {label}: {cell!r}")
    observed = tuple(float(match.group(index)) for index in (1, 2, 3))
    expected_values = (
        float(expected.get("mean")),
        float(expected.get("median")),
        float(expected.get("p90_nearest_rank")),
    )
    for name, actual, wanted in zip(("mean", "median", "p90"), observed, expected_values):
        if not math.isclose(actual, wanted, rel_tol=0.0, abs_tol=0.011):
            raise AutomationError(f"Markdown metric mismatch for {label} {name}: {actual} != {wanted}")
    denominator = re.search(r"\(\s*n\s*=\s*([0-9][0-9,]*)\s*\)", cell)
    if denominator is None or _int(denominator.group(1), field=f"{label} denominator") != int(expected.get("denominator")):
        raise AutomationError(f"Markdown metric mismatch for {label} denominator")


def _assert_percentage(cell: str, expected: Mapping[str, Any], *, label: str) -> None:
    match = re.search(r"([0-9]+(?:\.[0-9]+)?)\s*%.*?\(\s*n\s*=\s*([0-9][0-9,]*)\s*\)", cell)
    if match is None:
        raise AutomationError(f"Markdown percentage is not numeric for {label}: {cell!r}")
    observed = float(match.group(1))
    wanted = float(expected.get("rate")) * 100.0
    if not math.isclose(observed, wanted, rel_tol=0.0, abs_tol=0.011):
        raise AutomationError(f"Markdown metric mismatch for {label}: {observed} != {wanted}")
    denominator = _int(match.group(2), field=f"{label} denominator")
    if denominator != int(expected.get("denominator")):
        raise AutomationError(f"Markdown metric mismatch for {label} denominator: {denominator}")


def _assert_no_historical_n800(row: Sequence[str], display: str) -> None:
    text = " | ".join(row)
    if re.search(r"(?:n\s*=\s*800|\b800\s*/\s*800\b|\b800\s*/|/\s*800\b)", text, re.IGNORECASE):
        raise AutomationError(f"primary Markdown row still contains historical N=800 value: {display}")


def validate_primary_markdown(
    markdown_path: Path,
    results: Mapping[str, Mapping[str, Any]],
    *,
    enforce_ours_baseline: bool = True,
) -> dict[str, Any]:
    """Validate all comparison rows in the three primary Markdown tables.

    The function checks exact release denominators and, when a cell is numeric,
    its numerator/denominator against the live receipt summary.  Placeholder
    cells are tolerated only for tiny test fixtures, which lets the contract
    tests exercise row selection without embedding a full report.
    """

    try:
        markdown = Path(markdown_path).read_text(encoding="utf-8")
    except OSError as error:
        raise AutomationError(f"cannot read Markdown: {markdown_path}: {error}") from error
    sections = {number: _table_rows(_section(markdown, number)) for number in (1, 2, 3)}
    table1_count = table2_count = table3_count = 0
    for item in DATASETS:
        slug = item["slug"]
        result = results.get(slug)
        if not isinstance(result, Mapping):
            raise AutomationError(f"missing live result for {slug}")
        row1 = _display_row(sections[1], item["display"])
        _assert_no_historical_n800(row1, item["display"])
        if len(row1) < 3 or _int(row1[2], field="Table 1 N_eval") != item["n_eval"]:
            raise AutomationError(f"Table 1 N_release/N_eval mismatch for {item['display']}")
        # N_release is normally the same local count.  The receipt may use
        # null for release metadata, so the primary table still must carry N.
        if len(row1) < 2 or _int(row1[1], field="Table 1 N_release") != item["n_eval"]:
            raise AutomationError(f"Table 1 N_release/N_eval mismatch for {item['display']}")
        metrics1 = ((result.get("tables") or {}).get("table1") or {}).get("metrics") or {}
        for offset, metric in ((4, "links_per_asset"), (5, "movable_joints_per_asset")):
            expected = metrics1.get(metric)
            if not isinstance(expected, Mapping):
                raise AutomationError(f"missing Table 1 metric for {item['display']}: {metric}")
            _assert_triplet(row1[offset], expected, label=f"{item['display']} Table 1 {metric}")
        for offset, metric in ((6, "multi_joint_assets"), (7, "unique_topologies"), (8, "exact_duplicate_rate")):
            expected = metrics1.get(metric)
            if not isinstance(expected, Mapping):
                raise AutomationError(f"missing Table 1 metric for {item['display']}: {metric}")
            _assert_percentage(row1[offset], expected, label=f"{item['display']} Table 1 {metric}")
        table1_count += 1

        row2 = _display_row(sections[2], item["display"])
        _assert_no_historical_n800(row2, item["display"])
        if len(row2) < 1 + len(TABLE2_METRICS):
            raise AutomationError(f"Table 2 row has too few cells for {item['display']}")
        metrics2 = ((result.get("tables") or {}).get("table2") or {}).get("metrics") or {}
        for offset, metric in enumerate(TABLE2_METRICS, 1):
            expected = metrics2.get(metric, {})
            if isinstance(expected, Mapping):
                _assert_pair(row2[offset], expected, label=f"{item['display']} Table 2 {metric}")
            table2_count += 1

        row3 = _display_row(sections[3], item["display"])
        _assert_no_historical_n800(row3, item["display"])
        if len(row3) < 1 + len(TABLE3_ALL_METRICS):
            raise AutomationError(f"Table 3 row has too few cells for {item['display']}")
        metrics3 = ((result.get("tables") or {}).get("table3") or {}).get("metrics") or {}
        # The FK diagnostic sits between subtree consistency and joint-level
        # pass in the published column order.
        for offset, metric in enumerate(TABLE3_PAIR_METRICS[:4], 1):
            expected = metrics3.get(metric, {})
            if isinstance(expected, Mapping):
                _assert_pair(row3[offset], expected, label=f"{item['display']} Table 3 {metric}")
            table3_count += 1
        fk = metrics3.get("fk_roundtrip_error", {})
        if isinstance(fk, Mapping):
            _assert_fk(row3[5], fk, label=f"{item['display']} Table 3 FK")
        table3_count += 1
        joint_level = metrics3.get("joint_level_pass", {})
        if isinstance(joint_level, Mapping):
            _assert_pair(row3[6], joint_level, label=f"{item['display']} Table 3 joint_level_pass")
        table3_count += 1
        strict = metrics3.get("strict_kinematic_pass", {})
        if isinstance(strict, Mapping):
            _assert_pair(row3[-1], strict, label=f"{item['display']} Table 3 strict")
        table3_count += 1

    # Ours/Brain is intentionally outside the comparison roster.  Require the
    # rows to remain present and, for the live report, byte-stable against the
    # pre-existing primary-table snapshot.
    ours_rows: dict[str, str] = {}
    for number, rows in sections.items():
        for label in ("Ours-500K", "Ours per-class N=5 (supplementary)"):
            row = _display_row(rows, label)
            key = f"table{number}:{label}"
            rendered = " | ".join(row)
            digest = hashlib.sha256(rendered.encode("utf-8")).hexdigest()
            if enforce_ours_baseline and digest != OURS_PRIMARY_ROW_SHA256[key]:
                raise AutomationError(f"Ours/Brain primary row changed: {key}")
            ours_rows[key] = digest
    return {
        "table1_rows": table1_count,
        "table2_metrics": table2_count,
        "table3_metrics": table3_count,
        "ours_rows_present": len(ours_rows),
        "ours_row_sha256": ours_rows,
    }


def _live_dataset(root: Path, item: Mapping[str, Any], source_mode: bool | str) -> dict[str, Any]:
    dataset_root = root / str(item["slug"])
    manifest_path = dataset_root / "full_release_manifest.json"
    try:
        roster = common.load_roster(manifest_path, verify_sources=source_mode)
    except Exception as error:  # noqa: BLE001 - normalize CLI diagnostics
        raise AutomationError(f"{item['display']} roster verification failed: {error}") from error
    allowed_names = tuple(item.get("dataset_names", (item["display"],)))
    if roster.get("dataset") not in allowed_names:
        raise AutomationError(f"{item['display']} roster dataset mismatch: {roster.get('dataset')!r}")
    if roster.get("N_eval") != item["n_eval"] or roster.get("J_eval") != item["j_eval"]:
        raise AutomationError(f"{item['display']} roster N/J mismatch")
    tables: dict[str, Any] = {}
    for table in TABLES:
        output = dataset_root / table
        checkpoint = _json(output / "checkpoint.json")
        if checkpoint.get("state") != "complete":
            raise AutomationError(f"{item['display']} {table} checkpoint is not complete")
        try:
            common.verify_artifacts(output)
        except Exception as error:  # noqa: BLE001
            raise AutomationError(f"{item['display']} {table} artifact closure failed: {error}") from error
        summary = _json(output / "summary.json")
        if summary.get("n_eval", summary.get("N_eval")) != item["n_eval"]:
            raise AutomationError(f"{item['display']} {table} N_eval mismatch")
        if summary.get("j_eval", summary.get("J_eval")) != item["j_eval"]:
            raise AutomationError(f"{item['display']} {table} J_eval mismatch")
        status_counts = summary.get("status_counts") or _status_counts(output)
        if status_counts != _status_counts(output):
            raise AutomationError(f"{item['display']} {table} status count mismatch")
        tables[table] = {
            "summary": summary,
            "metrics": _table1_metrics(summary) if table == "table1" else summary.get("metrics", {}),
            "status_counts": status_counts,
            "checkpoint_state": checkpoint.get("state"),
        }
    try:
        independent = verify_dataset_receipts(dataset_root, verify_sources=source_mode)
    except (VerificationError, common.ManifestError, OSError, ValueError) as error:
        raise AutomationError(f"{item['display']} independent verification failed: {error}") from error
    if independent.get("n_eval") != item["n_eval"] or independent.get("j_eval") != item["j_eval"]:
        raise AutomationError(f"{item['display']} independent N/J mismatch")
    for table in TABLES:
        observed = independent["tables"][table]
        if observed.get("n_eval") != item["n_eval"] or observed.get("j_eval") != item["j_eval"]:
            raise AutomationError(f"{item['display']} independent {table} N/J mismatch")
        if observed.get("status_counts") != tables[table]["status_counts"]:
            raise AutomationError(f"{item['display']} independent {table} status mismatch")
    return {
        "dataset": item["display"],
        "n_eval": item["n_eval"],
        "j_eval": item["j_eval"],
        "roster_sha256": roster.get("roster_sha256"),
        "manifest_content_sha256": roster.get("manifest_content_sha256"),
        "tables": tables,
        "independent": independent,
    }


def _assert_equal_metrics(
    published: Mapping[str, Any],
    observed: Mapping[str, Any],
    *,
    label: str,
) -> None:
    """Compare receipt metric payloads without trusting display rounding."""

    if set(published) != set(observed):
        raise AutomationError(
            f"published receipt {label} metric set mismatch: "
            f"{sorted(published)} != {sorted(observed)}"
        )
    for name, expected in observed.items():
        actual = published.get(name)
        if actual != expected:
            raise AutomationError(f"published receipt {label} metric mismatch: {name}")


def _check_verification_index(
    root: Path,
    live: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    """Check the persisted independent-verification index against live rosters."""

    path = root / "full_release_verification_auto.json"
    verification = _json(path)
    expected_slugs = {str(item["slug"]) for item in DATASETS}
    if set(verification) != expected_slugs:
        raise AutomationError(
            "full_release_verification_auto.json dataset set is not exactly the eight releases"
        )
    for item in DATASETS:
        slug = str(item["slug"])
        value = verification.get(slug)
        if not isinstance(value, Mapping) or "error" in value:
            raise AutomationError(f"persisted independent verification failed for {item['display']}")
        current = live[slug]
        if value.get("roster_sha256") != current["roster_sha256"]:
            raise AutomationError(f"persisted verification roster hash mismatch for {item['display']}")
        if value.get("n_eval") != current["n_eval"] or value.get("j_eval") != current["j_eval"]:
            raise AutomationError(f"persisted verification N/J mismatch for {item['display']}")
        for table in TABLES:
            verified_table = (value.get("tables") or {}).get(table)
            if not isinstance(verified_table, Mapping):
                raise AutomationError(f"persisted verification missing {item['display']} {table}")
            live_table = current["tables"][table]
            if verified_table.get("status_counts") != live_table["status_counts"]:
                raise AutomationError(f"persisted verification status mismatch for {item['display']} {table}")
    return {
        "path": str(path),
        "sha256": common.sha256_file(path),
        "dataset_count": len(verification),
        "all_pass": True,
    }


def _check_published_receipt(root: Path, live: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    receipt = _json(root / "full_release_receipt.json")
    entries = receipt.get("datasets")
    if not isinstance(entries, list):
        raise AutomationError("full_release_receipt.json has no datasets list")
    by_slug = {str(entry.get("slug")): entry for entry in entries if isinstance(entry, dict)}
    expected_slugs = {str(item["slug"]) for item in DATASETS}
    if set(by_slug) != expected_slugs or len(entries) != len(expected_slugs):
        raise AutomationError("published receipt dataset set is not exactly the eight comparison releases")
    verification_result = _check_verification_index(root, live)
    source_flag = (receipt.get("source_aware_verification") or {}).get("all_pass")
    if source_flag is not True:
        raise AutomationError("published receipt does not attest source-aware verification")
    source_receipt = receipt.get("source_aware_verification") or {}
    if source_receipt.get("receipt_sha256") != verification_result["sha256"]:
        raise AutomationError("published receipt source-aware verification hash is stale")
    for item in DATASETS:
        entry = by_slug[item["slug"]]
        current = live[item["slug"]]
        for key in ("N_eval", "J_eval"):
            if entry.get(key) != current[key.lower()]:
                raise AutomationError(f"published receipt {item['display']} {key} mismatch")
        if entry.get("roster_sha256") != current["roster_sha256"]:
            raise AutomationError(f"published receipt {item['display']} roster hash mismatch")
        if entry.get("manifest_content_sha256") != current["manifest_content_sha256"]:
            raise AutomationError(f"published receipt {item['display']} manifest hash mismatch")
        tables = entry.get("tables") or {}
        for table in TABLES:
            published = tables.get(table) or {}
            observed = current["tables"][table]
            if published.get("checkpoint_state") != "complete":
                raise AutomationError(f"published receipt {item['display']} {table} is incomplete")
            if published.get("status_counts") != observed["status_counts"]:
                raise AutomationError(f"published receipt {item['display']} {table} statuses mismatch")
            if published.get("N_eval") != current["n_eval"] or published.get("J_eval") != current["j_eval"]:
                raise AutomationError(f"published receipt {item['display']} {table} N/J mismatch")
            _assert_equal_metrics(
                published.get("metrics") or {},
                observed.get("metrics") or {},
                label=f"{item['display']} {table}",
            )
            artifact_path = root / item["slug"] / table / "artifact_manifest.json"
            if published.get("artifact_manifest_sha256") != common.sha256_file(artifact_path):
                raise AutomationError(f"published receipt {item['display']} {table} artifact hash mismatch")
            run_manifest = _json(root / item["slug"] / table / "manifest.json")
            if published.get("run_manifest_sha256") != run_manifest.get("manifest_content_sha256"):
                raise AutomationError(f"published receipt {item['display']} {table} run manifest hash mismatch")
    return {
        "dataset_count": len(entries),
        "table_count": len(entries) * len(TABLES),
        "source_aware_flag": source_flag,
        "verification": verification_result,
    }


def _run_contract_tests(repo_root: Path) -> dict[str, Any]:
    test_files = [
        "exp/tests/test_table123_full_release_common.py",
        "exp/tests/test_table123_full_release_rosters.py",
        "exp/tests/test_infinigen_sim_full_release_extraction.py",
        "exp/tests/test_run_table1_full_release.py",
        "exp/tests/test_run_table2_full_release.py",
        "exp/tests/test_run_table3_full_release.py",
        "exp/tests/test_verify_table123_full_release.py",
        "exp/tests/test_table1_artiverse.py",
        "exp/tests/test_run_table1_lam.py",
        "exp/tests/test_table123_full_release_checks.py",
    ]
    python = os.environ.get("TABLE123_PYTHON", sys.executable)
    env = dict(os.environ)
    env["PYTHONPATH"] = str(repo_root / "exp" / "scripts") + os.pathsep + env.get("PYTHONPATH", "")
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env[name] = "1"
    command = [python, "-m", "pytest", "-q", *test_files]
    completed = subprocess.run(command, cwd=repo_root, env=env, text=True, capture_output=True)
    return {
        "command": command,
        "returncode": completed.returncode,
        "stdout_tail": completed.stdout[-4000:],
        "stderr_tail": completed.stderr[-4000:],
        "passed": completed.returncode == 0,
    }


def run_checks(
    root: Path,
    markdown: Path,
    *,
    source_mode: bool | str = "auto",
    run_pytest: bool = False,
    repo_root: Path | None = None,
) -> dict[str, Any]:
    root = Path(root).resolve()
    markdown = Path(markdown).resolve()
    live: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    expected_slugs = {str(item["slug"]) for item in DATASETS}
    try:
        actual_slugs = {
            child.name
            for child in root.iterdir()
            if child.is_dir() and (child / "full_release_manifest.json").is_file()
        }
    except OSError as error:
        actual_slugs = set()
        errors.append(f"cannot enumerate full-release root: {error}")
    if actual_slugs != expected_slugs:
        errors.append(
            f"full-release dataset directory set mismatch: "
            f"{sorted(actual_slugs)} != {sorted(expected_slugs)}"
        )
    for item in DATASETS:
        try:
            live[item["slug"]] = _live_dataset(root, item, source_mode)
        except AutomationError as error:
            errors.append(str(error))
    receipt_result: dict[str, Any] = {}
    markdown_result: dict[str, Any] = {}
    if not errors and len(live) == len(DATASETS):
        try:
            receipt_result = _check_published_receipt(root, live)
            markdown_result = validate_primary_markdown(markdown, live)
        except AutomationError as error:
            errors.append(str(error))
    pytest_result: dict[str, Any] | None = None
    if run_pytest:
        pytest_result = _run_contract_tests(repo_root or markdown.parents[1])
        if not pytest_result["passed"]:
            errors.append("focused contract pytest failed")
    report: dict[str, Any] = {
        "schema_version": "table123_full_release_automation_check_v1",
        "mode": "strict" if source_mode is True else "none" if source_mode is False else str(source_mode),
        "root": str(root),
        "markdown": str(markdown),
        "dataset_count": len(live),
        "table_count": sum(len(value.get("tables", {})) for value in live.values()),
        "datasets": {
            slug: {
                "dataset": value["dataset"],
                "n_eval": value["n_eval"],
                "j_eval": value["j_eval"],
                "roster_sha256": value["roster_sha256"],
                "tables": {
                    table: {
                        "checkpoint_state": info["checkpoint_state"],
                        "status_counts": info["status_counts"],
                    }
                    for table, info in value["tables"].items()
                },
            }
            for slug, value in live.items()
        },
        "published_receipt": receipt_result,
        "markdown_check": markdown_result,
        "pytest": pytest_result,
        "errors": errors,
        "all_pass": not errors and len(live) == len(DATASETS),
    }
    return report


def _source_mode(value: str) -> bool | str:
    if value == "strict":
        return True
    if value == "none":
        return False
    return value


def main(argv: Sequence[str] | None = None) -> int:
    script_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=script_root / "runtime" / "table123_full_release_20260825")
    parser.add_argument("--markdown", type=Path, default=script_root / "URDF-Sim-Ready-Automatic-Evaluation.md")
    parser.add_argument("--source-mode", choices=("none", "auto", "inventory", "strict"), default="auto")
    parser.add_argument("--pytest", action="store_true", dest="run_pytest", help="also run the focused contract suite")
    parser.add_argument("--json-out", type=Path, help="write the compact acceptance report")
    args = parser.parse_args(argv)
    report = run_checks(
        args.root,
        args.markdown,
        source_mode=_source_mode(args.source_mode),
        run_pytest=args.run_pytest,
        repo_root=script_root.parent,
    )
    if args.json_out:
        common._atomic_write_json(args.json_out, report)
    print(json.dumps(report, sort_keys=True, indent=2))
    return 0 if report["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "AutomationError",
    "DATASETS",
    "TABLE2_METRICS",
    "TABLE3_PAIR_METRICS",
    "validate_primary_markdown",
    "run_checks",
]
