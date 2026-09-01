#!/usr/bin/env python3
"""Independent verification for the Table 1/2/3 full-release receipts."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

import table123_full_release_common as common


class VerificationError(ValueError):
    """Raised when a published receipt is incomplete or internally inconsistent."""


def _load_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise VerificationError(f"cannot read {path.name}: {error}") from error


def _records_path(table_output: Path) -> Path:
    for name in ("records.jsonl", "asset_records.jsonl"):
        path = table_output / name
        if path.is_file():
            return path
    raise VerificationError("receipt records file is missing")


def _records(path: Path) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as error:
            raise VerificationError(f"invalid record JSON at line {line_number}: {error}") from error
        if not isinstance(value, dict):
            raise VerificationError(f"record at line {line_number} is not an object")
        result.append(value)
    return result


def _status_is_success(table: str, status: str) -> bool:
    """Apply the terminal status vocabulary owned by each table adapter."""

    # Runtime receipts commonly use collision-free names such as
    # ``table1_final`` or ``table1_preflight_partial``.  Classify by the
    # leading table token so suffixes cannot silently change failure counts.
    table_kind = table.split("_", 1)[0]
    if table_kind == "table1":
        # Table 1 reuses the legacy core's uppercase terminal status.
        return status == "EVALUATED"
    return status == "completed"


def _load_roster_copy(
    path: Path,
    *,
    verify_sources: bool | str = False,
) -> dict[str, Any]:
    """Load a table-local roster copy when its JSONL sidecar is not copied."""

    value = _load_json(path)
    if not isinstance(value, dict) or value.get("schema_version") != common.SCHEMA_VERSION:
        raise VerificationError("roster manifest schema mismatch")
    if value.get("roster_sha256") != common.canonical_sha256(value.get("rows")):
        raise VerificationError("roster content hash mismatch")
    unsigned = dict(value)
    declared = unsigned.pop("manifest_content_sha256", None)
    if declared != common.canonical_sha256(unsigned):
        raise VerificationError("roster manifest self-hash mismatch")
    rows = value.get("rows")
    if not isinstance(rows, list) or value.get("N_eval") != len(rows):
        raise VerificationError("roster denominator mismatch")
    if value.get("J_eval") != sum(int(row.get("joint_count", 0)) for row in rows):
        raise VerificationError("roster joint denominator mismatch")
    # Rebind primary files when the table receipt is detached from the
    # dataset-level manifest and its ordered JSONL sidecar.  The package
    # payload can be tens of gigabytes; default verification therefore trusts
    # the frozen package inventory and hashes only each evaluated URDF.  A
    # caller requesting strict source verification opts into the full package
    # attestation explicitly.
    for row in rows:
        try:
            rebound = common._bind_row(
                row,
                verify_package=bool(verify_sources is True),
                verify_primary=True,
            )
        except Exception as error:  # noqa: BLE001 - normalize contract failures
            raise VerificationError(f"source binding verification failed: {error}") from error
        for key in ("primary_urdf_sha256", "primary_urdf_size", "package_files", "package_binding_sha256"):
            if key in row and row.get(key) != rebound.get(key):
                raise VerificationError(f"source binding drift: {key}")
    return value


def _check_bindings(table_output: Path, roster: Mapping[str, Any], table: str) -> None:
    manifest_path = table_output / "manifest.json"
    if manifest_path.is_file():
        manifest = _load_json(manifest_path)
        if not isinstance(manifest, Mapping):
            raise VerificationError("run manifest is not an object")
        if manifest.get("roster_sha256") != roster.get("roster_sha256"):
            raise VerificationError("run manifest roster binding mismatch")
        if manifest.get("roster_manifest_content_sha256") not in (None, roster.get("manifest_content_sha256")):
            raise VerificationError("run manifest roster manifest binding mismatch")
        declared = manifest.get("manifest_content_sha256")
        unsigned = dict(manifest)
        unsigned.pop("manifest_content_sha256", None)
        if declared is not None and declared != common.canonical_sha256(unsigned):
            raise VerificationError("run manifest self-hash mismatch")
        for key, expected in (("N_eval", roster.get("N_eval")), ("J_eval", roster.get("J_eval"))):
            if key in manifest and manifest[key] != expected:
                raise VerificationError(f"run manifest {key} mismatch")

    protocol_path = table_output / "protocol_snapshot.json"
    if protocol_path.is_file():
        protocol = _load_json(protocol_path)
        if not isinstance(protocol, Mapping):
            raise VerificationError("protocol snapshot is not an object")
        protocol_bindings = {
            "roster_sha256": roster.get("roster_sha256"),
            "roster_manifest_content_sha256": roster.get("manifest_content_sha256"),
            "manifest_content_sha256": roster.get("manifest_content_sha256"),
        }
        for key, expected in protocol_bindings.items():
            if key in protocol and protocol[key] != expected:
                raise VerificationError(f"stale protocol snapshot: {key}")
        if "dataset" in protocol and protocol["dataset"] != roster.get("dataset"):
            raise VerificationError("stale protocol snapshot: dataset")
        if "table" in protocol and protocol["table"] != table:
            raise VerificationError("stale protocol snapshot: table")
        if "source_bindings" in protocol and protocol["source_bindings"] != roster.get("source_bindings", []):
            raise VerificationError("stale protocol snapshot: source bindings")


def _independent_summary(records: list[Mapping[str, Any]], roster: Mapping[str, Any], table: str) -> dict[str, Any]:
    rows = list(roster.get("rows", []))
    expected_ids = [str(row.get("asset_id")) for row in rows]
    actual_ids = [str(record.get("asset_id")) for record in records]
    if len(records) != len(rows):
        raise VerificationError(f"record count mismatch: {len(records)} != {len(rows)}")
    if actual_ids != expected_ids:
        raise VerificationError("record order or identity mismatch")
    for ordinal, (row, record) in enumerate(zip(rows, records)):
        if not isinstance(record.get("asset_id"), str) or "status" not in record:
            raise VerificationError(f"incomplete receipt record for {row['asset_id']}")
        if record.get("ordinal", ordinal) != ordinal:
            raise VerificationError(f"record ordinal mismatch for {row['asset_id']}")
        if row.get("primary_urdf_sha256") and record.get("primary_urdf_sha256") != row.get("primary_urdf_sha256"):
            raise VerificationError(f"record source hash mismatch for {row['asset_id']}")
        if "roster_joint_count" in record and int(record["roster_joint_count"]) != int(row.get("joint_count", 0)):
            raise VerificationError(f"record joint denominator mismatch for {row['asset_id']}")
        if "declared_joint_count" in record and int(record["declared_joint_count"]) != int(row.get("joint_count", 0)):
            raise VerificationError(f"record declared joint mismatch for {row['asset_id']}")

    statuses: dict[str, int] = {}
    for record in records:
        status = str(record.get("status", "unknown"))
        statuses[status] = statuses.get(status, 0) + 1
    joint_count = sum(int(row.get("joint_count", 0)) for row in rows)
    summary: dict[str, Any] = {
        "n_eval": len(rows),
        "j_eval": joint_count,
        "records_present": len(records),
        "status_counts": dict(sorted(statuses.items())),
        "asset_failure_count": sum(
            count for status, count in statuses.items()
            if not _status_is_success(table, status)
        ),
    }
    metric_names = sorted({name for record in records for name in (record.get("metrics") or {})})
    if metric_names:
        metrics: dict[str, Any] = {}
        for name in metric_names:
            passed = sum((record.get("metrics") or {}).get(name, {}).get("pass") is True for record in records)
            metrics[name] = {"passed": passed, "denominator": len(records), "rate": passed / len(records) if records else None}
        summary["metrics"] = metrics
        strict = metrics.get("strict_urdf_pass")
        if strict is not None:
            summary["strict_urdf_pass"] = strict
    if table in {"table1", "table2"}:
        summary["error_count"] = sum(count for status, count in statuses.items() if status in {"error", "timeout"})
    return summary


def reaggregate_table(
    table_output: Path,
    *,
    verify_sources: bool | str = False,
) -> dict[str, Any]:
    """Recompute receipt denominators and terminal status counts independently."""

    table_output = Path(table_output)
    # Dataset-level manifests normally have the ordered JSONL sidecar.  If a
    # stale convenience copy remains beside a receipt, select the candidate
    # whose hash matches the run manifest, then fall back to the table-local
    # copied roster.  This keeps detached receipts bound to their own freeze.
    run_manifest_path = table_output / "manifest.json"
    run_manifest = _load_json(run_manifest_path) if run_manifest_path.is_file() else {}
    declared_roster_hash = (
        run_manifest.get("roster_sha256")
        if isinstance(run_manifest, Mapping)
        else None
    )
    candidates = [
        table_output.parent / "full_release_manifest.json",
        table_output / "roster_manifest.json",
    ]
    roster = None
    last_error: Exception | None = None
    for roster_path in candidates:
        if not roster_path.is_file():
            continue
        try:
            raw = _load_json(roster_path)
            if (
                declared_roster_hash is not None
                and raw.get("roster_sha256") != declared_roster_hash
            ):
                continue
            try:
                candidate = common.load_roster(
                    roster_path, verify_sources=verify_sources
                )
            except common.ManifestError:
                candidate = _load_roster_copy(
                    roster_path,
                    verify_sources=verify_sources,
                )
            roster = candidate
            break
        except (VerificationError, common.ManifestError, OSError, json.JSONDecodeError) as error:
            last_error = error
    if roster is None:
        if last_error is not None:
            raise VerificationError(f"no roster matches table receipt: {last_error}") from last_error
        raise VerificationError("table receipt roster is missing")
    table = table_output.name
    _check_bindings(table_output, roster, table)
    records = _records(_records_path(table_output))
    return _independent_summary(records, roster, table)


def _compare_summary(table_output: Path, computed: Mapping[str, Any]) -> None:
    summary = _load_json(table_output / "summary.json")
    if not isinstance(summary, Mapping):
        raise VerificationError("summary is not an object")
    for key in ("n_eval", "j_eval", "records_present", "status_counts", "asset_failure_count"):
        if key in summary and summary[key] != computed.get(key):
            raise VerificationError(f"summary mismatch: {key}")
    for key in ("N_eval", "J_eval"):
        if key in summary and summary[key] != computed.get(key.lower()):
            raise VerificationError(f"summary mismatch: {key}")
    expected_metrics = computed.get("metrics", {})
    published_metrics = summary.get("metrics", {})
    if expected_metrics and not isinstance(published_metrics, Mapping):
        raise VerificationError("summary mismatch: metrics")
    for name, expected in expected_metrics.items():
        published = published_metrics.get(name)
        if not isinstance(published, Mapping):
            raise VerificationError(f"summary mismatch: metrics.{name}")
        if "passed" in published and published["passed"] != expected["passed"]:
            raise VerificationError(f"summary mismatch: metrics.{name}.passed")
        if "pass_count" in published and published["pass_count"] != expected["passed"]:
            raise VerificationError(f"summary mismatch: metrics.{name}.pass_count")
        if published.get("denominator") != expected["denominator"]:
            raise VerificationError(f"summary mismatch: metrics.{name}.denominator")
        rate = published.get("rate", published.get("pass_rate"))
        if rate is not None and rate != expected["rate"]:
            raise VerificationError(f"summary mismatch: metrics.{name}.rate")


def verify_dataset_receipts(
    dataset_output: Path,
    *,
    verify_sources: bool | str = "auto",
) -> dict[str, Any]:
    """Verify all three receipts and replay the runner source preflight.

    ``"auto"`` performs the bounded full-release check used by the runners;
    callers doing a one-time archival audit may pass ``True`` for payload
    hashes.  The default keeps verification practical for multi-gigabyte
    releases while still rejecting missing primary files and package inventory
    drift for small fixtures.
    """
    dataset_output = Path(dataset_output)
    roster = common.load_roster(
        dataset_output / "full_release_manifest.json",
        verify_sources=verify_sources,
    )
    tables: dict[str, Any] = {}
    for table in ("table1", "table2", "table3"):
        output = dataset_output / table
        if not output.is_dir():
            raise VerificationError(f"missing {table} receipt")
        common.verify_artifacts(output)
        checkpoint_path = output / "checkpoint.json"
        checkpoint = _load_json(checkpoint_path)
        if not isinstance(checkpoint, Mapping) or checkpoint.get("state") != "complete":
            raise VerificationError(f"incomplete {table} receipt: checkpoint is not complete")
        run_manifest = _load_json(output / "manifest.json")
        declared_roster_hash = (
            run_manifest.get("roster_sha256")
            if isinstance(run_manifest, Mapping)
            else None
        )
        if declared_roster_hash is not None and declared_roster_hash != roster.get("roster_sha256"):
            # The dataset-level convenience copy may lag a rebuilt release.
            # Rebind to the table-local roster that the run manifest names.
            local_path = output / "roster_manifest.json"
            if not local_path.is_file():
                raise VerificationError(f"{table} has no roster matching its run manifest")
            try:
                local = common.load_roster(local_path, verify_sources=verify_sources)
            except common.ManifestError:
                local = _load_roster_copy(local_path, verify_sources=verify_sources)
            if local.get("roster_sha256") != declared_roster_hash:
                raise VerificationError(f"{table} roster does not match its run manifest")
            roster = local
        computed = reaggregate_table(output, verify_sources=False)
        _compare_summary(output, computed)
        tables[table] = computed
    return {"dataset": roster["dataset"], "roster_sha256": roster["roster_sha256"], "n_eval": roster["N_eval"], "j_eval": roster["J_eval"], "tables": tables}


def render_full_release_rows(results: dict[str, Any]) -> str:
    dataset = results.get("dataset", "Unknown")
    n_eval = results.get("n_eval", "?")
    j_eval = results.get("j_eval", "?")
    lines = [f"| Dataset | Table | N/J | Statuses |", "|---|---|---:|---|"]
    for table, summary in sorted((results.get("tables") or {}).items()):
        statuses = ", ".join(f"{key}={value}" for key, value in sorted(summary.get("status_counts", {}).items()))
        lines.append(f"| {dataset} | {table} | N={summary.get('n_eval', n_eval)}, J={summary.get('j_eval', j_eval)} | {statuses} |")
    return "\n".join(lines)


__all__ = ["VerificationError", "verify_dataset_receipts", "reaggregate_table", "render_full_release_rows"]
