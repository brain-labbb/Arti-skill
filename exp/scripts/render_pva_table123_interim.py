#!/usr/bin/env python3
"""Render an auditable Table 1/2/2-supplementary/3 prefix snapshot.

The formal PV-A runner appends immutable result rows in ordinal order.  This
tool freezes the currently committed prefix without touching the writer or
claiming that the prefix is a full-release result.
"""

from __future__ import annotations

import argparse
from collections import Counter
import datetime as dt
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
from typing import Any, Mapping


SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import render_pva_table1234_full_release_results as final_renderer
import run_table1_full_release as table1
import run_table2_full_release as table2
import run_table2sup_full_release as table2sup
import run_table3_full_release as table3
import table123_full_release_common as common


SCHEMA_VERSION = "pva_table123_interim_prefix_v1"
CLASSIFICATION = "INTERIM_ORDERED_PREFIX_NOT_FULL_RELEASE"
RESULT_COLUMNS = {
    "table1": "table1_json",
    "table2": "table2_json",
    "table2_supplementary": "table2sup_json",
    "table3": "table3_json",
}


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _connect_read_only(database: Path) -> sqlite3.Connection:
    path = Path(database).resolve(strict=True)
    connection = sqlite3.connect(f"{path.as_uri()}?mode=ro", uri=True, timeout=120.0)
    connection.execute("PRAGMA query_only=ON")
    connection.execute("PRAGMA busy_timeout=120000")
    return connection


def _digest_row(digest: Any, *values: Any) -> None:
    payload = json.dumps(
        list(values),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    digest.update(len(payload).to_bytes(8, "big"))
    digest.update(payload)


def capture_prefix(database: Path) -> dict[str, Any]:
    """Freeze metadata and roster rows for the current contiguous prefix."""

    connection = _connect_read_only(database)
    try:
        result_count, minimum, maximum = connection.execute(
            "SELECT COUNT(*), MIN(ordinal), MAX(ordinal) FROM results"
        ).fetchone()
        n_snapshot = int(result_count)
        if n_snapshot <= 0:
            raise ValueError("the formal result database has no committed rows")
        if int(minimum) != 0 or int(maximum) != n_snapshot - 1:
            raise ValueError(
                "result rows are not a contiguous zero-based prefix: "
                f"count={n_snapshot}, min={minimum}, max={maximum}"
            )

        joined = connection.execute(
            """
            SELECT COUNT(*), COALESCE(SUM(a.joint_count), 0),
                   COUNT(DISTINCT a.category),
                   COALESCE(SUM(r.table4_state_count), 0)
            FROM results AS r
            JOIN assets AS a ON a.ordinal = r.ordinal AND a.asset_id = r.asset_id
            WHERE r.ordinal < ?
            """,
            (n_snapshot,),
        ).fetchone()
        if int(joined[0]) != n_snapshot:
            raise ValueError("result prefix is not identity-bound to the asset roster")

        status_counts = {
            str(status): int(count)
            for status, count in connection.execute(
                """
                SELECT worker_status, COUNT(*) FROM results
                WHERE ordinal < ? GROUP BY worker_status ORDER BY worker_status
                """,
                (n_snapshot,),
            )
        }
        rows: list[dict[str, Any]] = []
        roster_digest = hashlib.sha256()
        query = """
            SELECT ordinal, asset_id, category, joint_count, row_sha256, row_json
            FROM assets WHERE ordinal < ? ORDER BY ordinal
        """
        for expected, values in enumerate(connection.execute(query, (n_snapshot,))):
            ordinal, asset_id, category, joint_count, row_sha256, row_json = values
            if int(ordinal) != expected:
                raise ValueError(f"asset roster prefix is not contiguous at ordinal {expected}")
            source = json.loads(row_json)
            if not isinstance(source, dict):
                raise ValueError(f"asset roster row {expected} is not an object")
            if str(source.get("asset_id")) != str(asset_id):
                raise ValueError(f"asset roster identity mismatch at ordinal {expected}")
            row = dict(source)
            row.update(
                {
                    "ordinal": int(ordinal),
                    "asset_id": str(asset_id),
                    "category": str(source.get("category", category)),
                    "joint_count": int(joint_count),
                }
            )
            rows.append(row)
            _digest_row(roster_digest, int(ordinal), str(asset_id), str(row_sha256))
        if len(rows) != n_snapshot:
            raise ValueError("asset roster is shorter than the committed result prefix")
        return {
            "N_snapshot": n_snapshot,
            "ordinal_min": 0,
            "ordinal_max": n_snapshot - 1,
            "J_snapshot": int(joined[1]),
            "category_count_snapshot": int(joined[2]),
            "table4_state_records": int(joined[3]),
            "worker_status_counts": status_counts,
            "roster_prefix_sha256": roster_digest.hexdigest(),
            "rows": rows,
        }
    finally:
        connection.close()


def _load_records(
    database: Path,
    column: str,
    n_snapshot: int,
) -> tuple[list[dict[str, Any]], str]:
    if column not in RESULT_COLUMNS.values():
        raise ValueError(f"unsupported result column: {column}")
    connection = _connect_read_only(database)
    digest = hashlib.sha256()
    records: list[dict[str, Any]] = []
    try:
        query = f"SELECT ordinal, asset_id, {column} FROM results WHERE ordinal < ? ORDER BY ordinal"
        for expected, (ordinal, asset_id, payload) in enumerate(
            connection.execute(query, (n_snapshot,))
        ):
            if int(ordinal) != expected:
                raise ValueError(f"{column} result prefix is not contiguous at ordinal {expected}")
            value = json.loads(payload)
            if not isinstance(value, dict):
                raise ValueError(f"{column} result {expected} is not an object")
            record_asset_id = value.get("asset_id", value.get("asset_key"))
            if str(record_asset_id) != str(asset_id):
                raise ValueError(f"{column} result identity mismatch at ordinal {expected}")
            records.append(value)
            _digest_row(digest, int(ordinal), str(asset_id), str(payload))
        if len(records) != n_snapshot:
            raise ValueError(f"{column} result count changed during snapshot")
        return records, digest.hexdigest()
    finally:
        connection.close()


def _resolve_roster_manifest(root: Path, execution: Mapping[str, Any]) -> Path:
    value = execution.get("roster_manifest")
    if not isinstance(value, str) or not value:
        raise ValueError("execution manifest has no roster_manifest")
    path = Path(value)
    return (path if path.is_absolute() else root / path).resolve(strict=True)


def _write_summary(output: Path, name: str, summary: Mapping[str, Any]) -> Path:
    path = output / f"{name}_summary.json"
    common._atomic_write_json(path, dict(summary))
    return path


def _status_text(status_counts: Mapping[str, Any]) -> str:
    return ", ".join(f"{key}={int(value):,}" for key, value in sorted(status_counts.items()))


def render_interim(root: Path, output_dir: Path | None = None) -> Path:
    root = Path(root).resolve(strict=True)
    database = root / "results.sqlite3"
    execution_path = root / "manifest.json"
    execution = _json(execution_path)
    snapshot = capture_prefix(database)
    n_snapshot = int(snapshot["N_snapshot"])
    j_snapshot = int(snapshot["J_snapshot"])
    generated_at = utc_now()
    stamp = generated_at.replace("-", "").replace(":", "").replace(".", "").replace("Z", "Z")
    if output_dir is None:
        output_dir = root.parent / "interim" / f"table123_prefix_{n_snapshot:09d}_{stamp}"
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)

    roster_path = _resolve_roster_manifest(root, execution)
    roster_manifest = _json(roster_path)
    pseudo_roster = {
        "schema_version": common.SCHEMA_VERSION,
        "dataset": "Ours / PV-A",
        "N_eval": n_snapshot,
        "J_eval": j_snapshot,
        "rows": snapshot["rows"],
        "roster_sha256": roster_manifest["roster"]["sha256"],
        "manifest_content_sha256": roster_manifest["manifest_content_sha256"],
    }

    table_hashes: dict[str, str] = {}
    summaries: dict[str, dict[str, Any]] = {}

    records, table_hashes["table1"] = _load_records(
        database, RESULT_COLUMNS["table1"], n_snapshot
    )
    summary1 = table1.aggregate_full_release(records, pseudo_roster)
    summary1["cohort"].update(
        {
            "cohort_type": CLASSIFICATION,
            "N_release": int(roster_manifest["N_release"]),
            "release_raw_categories": int(roster_manifest["release_category_count"]),
        }
    )
    summaries["table1"] = summary1
    del records

    records, table_hashes["table2"] = _load_records(
        database, RESULT_COLUMNS["table2"], n_snapshot
    )
    summaries["table2"] = table2.aggregate_full_release(records, pseudo_roster)
    del records

    records, table_hashes["table2_supplementary"] = _load_records(
        database, RESULT_COLUMNS["table2_supplementary"], n_snapshot
    )
    summary2s = table2sup.aggregate_records(records, n_snapshot, j_snapshot)
    summary2s.update(
        {
            "schema_version": SCHEMA_VERSION,
            "dataset": "Ours / PV-A",
            "category_count": int(snapshot["category_count_snapshot"]),
        }
    )
    summaries["table2_supplementary"] = summary2s
    del records

    records, table_hashes["table3"] = _load_records(
        database, RESULT_COLUMNS["table3"], n_snapshot
    )
    summaries["table3"] = table3.aggregate_full_release(records, pseudo_roster)
    del records

    summary_paths = {
        name: _write_summary(output, name, summary) for name, summary in summaries.items()
    }
    status_counts = snapshot["worker_status_counts"]
    lines = [
        "# Ours / PV-A Table 1/2/3 Interim Results",
        "",
        f"Classification: **{CLASSIFICATION}**.",
        "",
        (
            f"Frozen committed prefix: **N={n_snapshot:,} assets**, **J={j_snapshot:,} movable joints**, "
            f"**{int(snapshot['category_count_snapshot']):,} / {int(roster_manifest['release_category_count']):,} "
            "generator classes observed**. "
            f"Formal intent remains N={int(execution['N_eval']):,}, J={int(execution['J_eval']):,}."
        ),
        "",
        f"Worker status at cutoff: `{_status_text(status_counts)}`. Cutoff UTC: `{generated_at}`.",
        "",
        (
            "This is not a full-release result and must not replace the final Ours / PV-A row. "
            "The roster is category-ordered, so this prefix is compositionally biased; all values "
            "will be replaced after the complete formal receipt passes verification."
        ),
        "",
    ]
    for section in (
        final_renderer._table1(summaries["table1"]),
        final_renderer._table2(summaries["table2"]),
        final_renderer._table2_supplementary(summaries["table2_supplementary"]),
        final_renderer._table3(summaries["table3"]),
    ):
        lines.extend(section)
        lines.extend(["", "---", ""])
    manifest_path = output / "interim_manifest.json"
    markdown_path = output / "pva_table123_interim_results.md"
    lines.extend(
        [
            "## Evidence",
            "",
            f"- Interim manifest: `{manifest_path}`",
            f"- Formal execution manifest: `{execution_path}`",
            f"- Frozen protocol snapshot: `{root / 'protocol_snapshot.md'}`",
            f"- Append-only result database: `{database}`",
            "",
        ]
    )
    common._atomic_write_bytes(markdown_path, "\n".join(lines).encode("utf-8"))

    source_files = {
        "interim_renderer": SCRIPT,
        "final_table_formatter": Path(final_renderer.__file__),
        "table1_aggregator": Path(table1.__file__),
        "table2_aggregator": Path(table2.__file__),
        "table2_supplementary_aggregator": Path(table2sup.__file__),
        "table3_aggregator": Path(table3.__file__),
    }
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "classification": CLASSIFICATION,
        "generated_at_utc": generated_at,
        "formal_run_status_at_cutoff": "RUNNING",
        "formal_run_root": str(root),
        "formal_execution_manifest": str(execution_path),
        "formal_execution_manifest_sha256": common.sha256_file(execution_path),
        "formal_protocol_snapshot_sha256": execution["protocol"]["snapshot_sha256"],
        "roster_manifest": str(roster_path),
        "roster_manifest_sha256": common.sha256_file(roster_path),
        "N_release": int(roster_manifest["N_release"]),
        "N_formal_intent": int(execution["N_eval"]),
        "J_formal_intent": int(execution["J_eval"]),
        "N_snapshot": n_snapshot,
        "J_snapshot": j_snapshot,
        "ordinal_min": int(snapshot["ordinal_min"]),
        "ordinal_max": int(snapshot["ordinal_max"]),
        "category_count_snapshot": int(snapshot["category_count_snapshot"]),
        "category_count_formal": int(execution["category_count"]),
        "table4_state_records_at_cutoff": int(snapshot["table4_state_records"]),
        "worker_status_counts": dict(status_counts),
        "roster_prefix_sha256": snapshot["roster_prefix_sha256"],
        "table_prefix_sha256": table_hashes,
        "source_hashes": {
            name: common.sha256_file(path) for name, path in sorted(source_files.items())
        },
        "artifacts": {
            **{
                f"{name}_summary": {
                    "path": str(path),
                    "sha256": common.sha256_file(path),
                }
                for name, path in sorted(summary_paths.items())
            },
            "markdown": {
                "path": str(markdown_path),
                "sha256": common.sha256_file(markdown_path),
            },
        },
        "warning": "Ordered-prefix interim result; compositionally biased and not publication-final.",
    }
    manifest["manifest_content_sha256"] = common.canonical_sha256(manifest)
    common._atomic_write_json(manifest_path, manifest)
    return markdown_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    try:
        output = render_interim(args.root, args.output_dir)
    except Exception as error:  # noqa: BLE001
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"output": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["capture_prefix", "render_interim"]
