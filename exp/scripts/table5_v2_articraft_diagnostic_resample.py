#!/usr/bin/env python3
"""Freeze a diagnostic Articraft-200 resample for Table 5 from the full export.

This is a DIAGNOSTIC protocol, separate from the frozen Table 5 cohort.  It
draws 200 Articraft records from the Articraft-10K-github universe using the
full-population quality census of the visual+collision (no-validate) export,
so the evaluation cohort deliberately spans the quality gradient:

  * stratum F: records whose export FAILED under the target policy
    (placeholder URDF);
  * stratum C: exported records with links missing collision geometry;
  * stratum I: records whose links ALL lack inertial data (the engine must
    synthesize masses), hash-sampled;
  * stratum R: uniform hash sample over the remaining eligible records.

Selection uses only pre-simulation information (roster scope filters and the
export census); simulator outcomes never affect membership.  Packages are
referenced in place from the full export; nothing is recompiled.
"""

from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

from table5_n200_manifest import canonical_sha256

SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
EXP_ROOT = REPO_ROOT / "exp"

DIAGNOSTIC_PROTOCOL_ID = "table5-v2-articraft-diagnostic-quality-stratified-v1"
DIAGNOSTIC_SEED = "arti-skill-table5-articraft-diagnostic-20260830"
SAMPLE_SIZE = 200
MIN_MOVABLE_JOINTS = 1
MAX_MOVABLE_JOINTS = 20
I_SIZE = 40

EXPORT_DIR = EXP_ROOT / "runtime/articraft_github_full10787_export_noverify_20260830"
MERGED_ROSTER = (
    EXP_ROOT
    / "runtime/articraft_github_merged_10787_20260827/rosters/merged/full_release_roster.jsonl"
)
SOURCE_ROOT = EXP_ROOT / "Articraft-10K-github"
FROZEN_COHORT = (
    EXP_ROOT
    / "runtime/table5_v2_core200_five_full_release_articraft10787_infinigen_paired_official/cohort_manifest.json"
)
DEFAULT_OUTPUT = EXP_ROOT / "runtime/table5_v2_articraft_diagnostic_resample_20260830"


class DiagnosticResampleError(RuntimeError):
    """Raised when the diagnostic cohort cannot be frozen as declared."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def diagnostic_rank(stratum: str, asset_id: str, universe_sha256: str) -> str:
    identity = "\0".join(
        (
            DIAGNOSTIC_PROTOCOL_ID,
            DIAGNOSTIC_SEED,
            stratum,
            universe_sha256,
            "articraft_10k",
            asset_id,
        )
    )
    return hashlib.sha256(identity.encode("utf-8")).hexdigest()


def movable_joint_count(row: Mapping[str, Any]) -> int | None:
    counts = row.get("xml_counts")
    if not isinstance(counts, Mapping):
        return None
    value = counts.get("movable_joints")
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def is_eligible(row: Mapping[str, Any]) -> bool:
    joint_count = movable_joint_count(row)
    return (
        isinstance(joint_count, int)
        and MIN_MOVABLE_JOINTS <= joint_count <= MAX_MOVABLE_JOINTS
    )


def load_inputs() -> tuple[dict[str, dict[str, Any]], dict[str, dict[str, Any]], str]:
    rows: dict[str, dict[str, Any]] = {}
    with MERGED_ROSTER.open("r", encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            rows[str(row["asset_id"])] = row
    if len(rows) != 10_787:
        raise DiagnosticResampleError("merged roster must contain 10,787 rows")
    roster_hash = sha256_file(MERGED_ROSTER)
    census_path = EXPORT_DIR / "quality_census.json"
    census = json.loads(census_path.read_text(encoding="utf-8"))
    if census.get("total") != 10_787:
        raise DiagnosticResampleError("quality census must cover 10,787 records")
    census_by_id = {str(record["asset_id"]): record for record in census["records"]}
    if len(census_by_id) != len(rows):
        raise DiagnosticResampleError("census/roster asset id mismatch")
    return rows, census_by_id, roster_hash


def select_strata(
    rows: Mapping[str, Mapping[str, Any]],
    census: Mapping[str, Mapping[str, Any]],
    roster_hash: str,
) -> tuple[list[str], list[str], list[str], list[str]]:
    failed_eligible = sorted(
        asset_id
        for asset_id, record in census.items()
        if record.get("export_status") != "success"
        and asset_id in rows
        and is_eligible(rows[asset_id])
    )
    collision_deficient = sorted(
        asset_id
        for asset_id, record in census.items()
        if record.get("export_status") == "success"
        and (record.get("counts") or {}).get("links_without_collision", 0) > 0
        and asset_id in rows
        and is_eligible(rows[asset_id])
    )
    no_inertial_pool = [
        asset_id
        for asset_id, record in census.items()
        if record.get("export_status") == "success"
        and (record.get("counts") or {}).get("links_without_inertial_fraction", 0.0)
        == 1.0
        and asset_id in rows
        and is_eligible(rows[asset_id])
        and asset_id not in set(collision_deficient)
    ]
    if len(no_inertial_pool) < I_SIZE:
        raise DiagnosticResampleError(
            f"only {len(no_inertial_pool)} eligible all-inertial-missing records; "
            f"{I_SIZE} required"
        )
    ranked_no_inertial = sorted(
        no_inertial_pool,
        key=lambda asset_id: diagnostic_rank("I", asset_id, roster_hash),
    )
    selected_i = ranked_no_inertial[:I_SIZE]
    excluded = set(failed_eligible) | set(collision_deficient) | set(no_inertial_pool)
    pool = [
        asset_id
        for asset_id, row in rows.items()
        if is_eligible(row) and asset_id not in excluded
    ]
    r_size = SAMPLE_SIZE - len(failed_eligible) - len(collision_deficient) - len(selected_i)
    if len(pool) < r_size:
        raise DiagnosticResampleError(f"random pool too small: {len(pool)}")
    ranked_pool = sorted(
        pool, key=lambda asset_id: diagnostic_rank("R", asset_id, roster_hash)
    )
    return sorted(failed_eligible), sorted(collision_deficient), selected_i, ranked_pool[:r_size]


def _parse_urdf_counts(package: Path) -> dict[str, int] | None:
    urdf = package / "model.urdf"
    try:
        root = ET.parse(urdf).getroot()
    except (OSError, ET.ParseError):
        return None
    links = root.findall("link")
    joints = root.findall("joint")
    movable = sum(
        1
        for joint in joints
        if joint.get("type") in ("revolute", "prismatic", "continuous")
    )
    return {
        "links": len(links),
        "joints": len(joints),
        "movable_joints": movable,
        "fixed_joints": sum(1 for joint in joints if joint.get("type") == "fixed"),
        "visual_elements": sum(len(link.findall("visual")) for link in links),
        "collision_elements": sum(len(link.findall("collision")) for link in links),
    }


def build_cohort(
    *,
    rows: Mapping[str, Mapping[str, Any]],
    census: Mapping[str, Mapping[str, Any]],
    selected: Sequence[str],
    stratum_of: Mapping[str, str],
    roster_hash: str,
    output: Path,
) -> Path:
    staging = EXPORT_DIR / "staging/data/cache/record_materialization"
    frozen = json.loads(FROZEN_COHORT.read_text(encoding="utf-8"))
    datasets = deepcopy(frozen["datasets"])
    art_index = next(
        i for i, d in enumerate(datasets) if d.get("dataset_slug") == "articraft_10k"
    )
    global_ranked = sorted(
        selected,
        key=lambda asset_id: diagnostic_rank(
            stratum_of[asset_id], asset_id, roster_hash
        ),
    )
    art_rows: list[dict[str, Any]] = []
    for order, asset_id in enumerate(global_ranked):
        source = rows[asset_id]
        package = (staging / asset_id).resolve(strict=True)
        urdf = package / "model.urdf"
        if not urdf.is_file():
            raise DiagnosticResampleError(f"exported package missing: {asset_id}")
        source_row_hash = canonical_sha256(source)
        rank = diagnostic_rank(stratum_of[asset_id], asset_id, roster_hash)
        counts = _parse_urdf_counts(package)
        if counts is None:
            counts = {"movable_joints": movable_joint_count(source)}
        census_record = census[asset_id]
        raw: dict[str, Any] = {
            "dataset_slug": "articraft_10k",
            "dataset_name": "Articraft-10K",
            "dataset_id": f"articraft_{order:04d}",
            "asset_id": asset_id,
            "category": str(source.get("category") or source.get("raw_category") or "N/E"),
            "package_root": str(package),
            "urdf_path": str(urdf),
            "urdf_sha256": sha256_file(urdf),
            "xml_counts": counts,
            "source_provenance": {
                "cohort_origin": source.get("cohort_origin"),
                "github_record_path": str(SOURCE_ROOT / "records" / asset_id),
                "export_dir": str(EXPORT_DIR),
                "universe_roster_sha256": roster_hash,
                "diagnostic_stratum": stratum_of[asset_id],
                "export_status": census_record.get("export_status"),
                "defect_score": census_record.get("defect_score"),
                "defect_counts": census_record.get("counts", {}),
            },
            "cohort_selection": {
                "protocol_id": DIAGNOSTIC_PROTOCOL_ID,
                "seed": DIAGNOSTIC_SEED,
                "selection_order": order,
                "rank_sha256": rank,
                "source_row_sha256": source_row_hash,
            },
        }
        raw["row_sha256"] = canonical_sha256(raw, exclude_fields=("row_sha256",))
        art_rows.append(raw)
    ordered_ids = [row["asset_id"] for row in art_rows]
    ordered_ranks = [row["cohort_selection"]["rank_sha256"] for row in art_rows]
    stratum_counts = Counter(stratum_of[asset_id] for asset_id in selected)
    datasets[art_index] = {
        "dataset_slug": "articraft_10k",
        "dataset_name": "Articraft-10K",
        "universe": {
            "kind": "diagnostic_quality_stratified_pool_from_full_noverify_export_v2",
            "path": str(MERGED_ROSTER),
            "sha256": roster_hash,
            "candidate_count": len(rows),
            "eligible_count": sum(1 for row in rows.values() if is_eligible(row)),
            "export_dir": str(EXPORT_DIR),
        },
        "selection": {
            "protocol_id": DIAGNOSTIC_PROTOCOL_ID,
            "seed": DIAGNOSTIC_SEED,
            "policy": (
                "quality-stratified diagnostic sample over the full-population "
                "no-validate visual+collision export census: F=export failures; "
                "C=links missing collision geometry; I=all links lack inertial "
                "data; R=uniform hash sample of the remaining eligible pool"
            ),
            "stratum_counts": dict(sorted(stratum_counts.items())),
            "selected_count": len(art_rows),
            "ordered_asset_ids_sha256": canonical_sha256(ordered_ids),
            "ordered_ranks_sha256": canonical_sha256(ordered_ranks),
        },
        "rows": art_rows,
    }
    cohort = deepcopy(frozen)
    cohort["datasets"] = datasets
    cohort["total_rows"] = sum(len(d["rows"]) for d in datasets)
    cohort["cohort_sha256"] = canonical_sha256(
        [
            {
                "dataset_slug": dataset["dataset_slug"],
                "universe_sha256": dataset["universe"]["sha256"],
                "ordered_asset_ids_sha256": dataset["selection"][
                    "ordered_asset_ids_sha256"
                ],
                "ordered_ranks_sha256": dataset["selection"]["ordered_ranks_sha256"],
            }
            for dataset in datasets
        ]
    )
    cohort.pop("manifest_sha256", None)
    cohort["manifest_sha256"] = canonical_sha256(
        cohort, exclude_fields=("manifest_sha256",)
    )
    output.mkdir(parents=True, exist_ok=True)
    cohort_path = output / "cohort_manifest.json"
    cohort_path.write_text(
        json.dumps(cohort, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    selection_dir = output / "selection"
    selection_dir.mkdir(parents=True, exist_ok=True)
    with (selection_dir / "selected_assets.jsonl").open("w", encoding="utf-8") as handle:
        for order, asset_id in enumerate(global_ranked):
            handle.write(
                json.dumps(
                    {
                        "asset_id": asset_id,
                        "stratum": stratum_of[asset_id],
                        "dataset_id": f"articraft_{order:04d}",
                        "export_status": census[asset_id].get("export_status"),
                        "defect_score": census[asset_id].get("defect_score"),
                    },
                    sort_keys=True,
                    ensure_ascii=True,
                )
                + "\n"
            )
    return cohort_path


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    output = arguments.out.resolve(strict=False)
    cohort_path = output / "cohort_manifest.json"
    if cohort_path.exists():
        raise SystemExit(f"cohort already frozen: {cohort_path}")
    try:
        rows, census, roster_hash = load_inputs()
        stratum_f, stratum_c, stratum_i, stratum_r = select_strata(
            rows, census, roster_hash
        )
        selected = [*stratum_f, *stratum_c, *stratum_i, *stratum_r]
        if len(set(selected)) != SAMPLE_SIZE:
            raise DiagnosticResampleError(
                f"expected {SAMPLE_SIZE} distinct selections; got {len(set(selected))}"
            )
        stratum_of = {
            **{asset_id: "F" for asset_id in stratum_f},
            **{asset_id: "C" for asset_id in stratum_c},
            **{asset_id: "I" for asset_id in stratum_i},
            **{asset_id: "R" for asset_id in stratum_r},
        }
        selection_dir = output / "selection"
        selection_dir.mkdir(parents=True, exist_ok=True)
        (selection_dir / "selection_summary.json").write_text(
            json.dumps(
                {
                    "protocol_id": DIAGNOSTIC_PROTOCOL_ID,
                    "seed": DIAGNOSTIC_SEED,
                    "universe": {
                        "path": str(MERGED_ROSTER),
                        "sha256": roster_hash,
                        "candidate_count": len(rows),
                        "eligible_count": sum(
                            1 for row in rows.values() if is_eligible(row)
                        ),
                    },
                    "export_dir": str(EXPORT_DIR),
                    "strata": {
                        "F": {"count": len(stratum_f), "asset_ids": stratum_f},
                        "C": {"count": len(stratum_c), "asset_ids": stratum_c},
                        "I": {"count": len(stratum_i), "asset_ids": stratum_i},
                        "R": {"count": len(stratum_r), "asset_ids": stratum_r},
                    },
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(
            json.dumps(
                {
                    "stage": "selected",
                    "F": len(stratum_f),
                    "C": len(stratum_c),
                    "I": len(stratum_i),
                    "R": len(stratum_r),
                },
                sort_keys=True,
            ),
            flush=True,
        )
        cohort_path = build_cohort(
            rows=rows,
            census=census,
            selected=selected,
            stratum_of=stratum_of,
            roster_hash=roster_hash,
            output=output,
        )
        print(
            json.dumps({"stage": "frozen", "cohort": str(cohort_path)}, sort_keys=True),
            flush=True,
        )
    except (DiagnosticResampleError, OSError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
