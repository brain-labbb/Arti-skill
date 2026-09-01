#!/usr/bin/env python3
"""Freeze genuinely unseen source pools for the formal T2 authoring experiment."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
TEMPLATE_ROOT = PROJECT_ROOT / "arti-template"
DATA_RECORDS = PROJECT_ROOT / "articraft_data" / "data" / "records"
DEFAULT_OUT = EXP_ROOT / "runtime" / "t2_formal_v1" / "preparation"
MAX_RECORDS_PER_TASK = 12
TOKEN_RE = re.compile(r"[a-z0-9]+")

sys.path.insert(0, str(TEMPLATE_ROOT))

from agent.source_maps import discover_source_pool, render_source_map_scaffold  # noqa: E402


TASKS: tuple[dict[str, str], ...] = (
    {"slug": "flip_phone", "complexity": "simple"},
    {"slug": "glove_compartment_door", "complexity": "simple"},
    {"slug": "flatbed_scanner_with_hinged_lid", "complexity": "simple"},
    {
        "slug": "clamp_meter_with_hinged_jaw_and_rotary_selector",
        "complexity": "simple",
    },
    {"slug": "garden_gate", "complexity": "medium"},
    {"slug": "bicycle_dropper_seatpost_assembly", "complexity": "medium"},
    {"slug": "air_purifier_with_filter_door", "complexity": "medium"},
    {"slug": "instrument_case_with_hinged_lid", "complexity": "medium"},
    {
        "slug": "adjustable_weight_bench_with_hinged_backrest",
        "complexity": "complex",
    },
    {"slug": "folding_kick_scooter", "complexity": "complex"},
    {"slug": "extension_ladder", "complexity": "complex"},
    {"slug": "dock_loading_ramp", "complexity": "complex"},
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def score(metadata: dict[str, Any]) -> int:
    values = [metadata.get("rating"), metadata.get("secondary_rating")]
    numeric = [int(value) for value in values if isinstance(value, (int, float))]
    return max(numeric, default=0)


def prompt_tokens(record_dir: Path, metadata: dict[str, Any]) -> set[str]:
    revision = str(metadata["active_revision_id"])
    prompt = record_dir / "revisions" / revision / "prompt.txt"
    text = prompt.read_text(encoding="utf-8", errors="replace") if prompt.is_file() else ""
    return set(TOKEN_RE.findall(text.casefold()))


def jaccard(left: set[str], right: set[str]) -> float:
    union = left | right
    return len(left & right) / len(union) if union else 1.0


def eligible_records(slug: str) -> list[dict[str, Any]]:
    prefix = f"rec_{slug}_"
    rows: list[dict[str, Any]] = []
    for record_dir in sorted(DATA_RECORDS.iterdir()):
        if not record_dir.is_dir() or not record_dir.name.startswith(prefix):
            continue
        metadata_path = record_dir / "record.json"
        if not metadata_path.is_file():
            continue
        metadata = load_json(metadata_path)
        if metadata.get("category_slug") != slug or score(metadata) < 4:
            continue
        revision = str(metadata.get("active_revision_id") or "")
        model = record_dir / "revisions" / revision / "model.py"
        if not revision or not model.is_file():
            continue
        rows.append(
            {
                "record_id": record_dir.name,
                "source_dir": record_dir,
                "metadata": metadata,
                "rating": score(metadata),
                "tokens": prompt_tokens(record_dir, metadata),
            }
        )
    return rows


def select_diverse(rows: list[dict[str, Any]], limit: int) -> list[dict[str, Any]]:
    if len(rows) <= limit:
        return sorted(rows, key=lambda row: row["record_id"])
    remaining = sorted(rows, key=lambda row: (-row["rating"], row["record_id"]))
    selected = [remaining.pop(0)]
    while remaining and len(selected) < limit:
        ranked = []
        for row in remaining:
            minimum_distance = min(
                1.0 - jaccard(row["tokens"], chosen["tokens"]) for chosen in selected
            )
            ranked.append((minimum_distance, row["rating"], row["record_id"], row))
        chosen = max(ranked, key=lambda item: (item[0], item[1], item[2]))[3]
        selected.append(chosen)
        remaining.remove(chosen)
    return sorted(selected, key=lambda row: row["record_id"])


def stage_record(row: dict[str, Any], slug: str, records_root: Path) -> dict[str, Any]:
    source = row["source_dir"]
    target = records_root / row["record_id"]
    revision = str(row["metadata"]["active_revision_id"])
    source_revision = source / "revisions" / revision
    target_revision = target / "revisions" / revision
    target_revision.mkdir(parents=True, exist_ok=True)
    for name in ("model.py", "prompt.txt", "provenance.json", "revision.json", "cost.json"):
        candidate = source_revision / name
        if candidate.is_file():
            shutil.copy2(candidate, target_revision / name)

    metadata = dict(row["metadata"])
    metadata["collections"] = ["workbench"]
    metadata["benchmark_source_record"] = str(source.relative_to(PROJECT_ROOT))
    dump_json(target / "record.json", metadata)
    dump_json(
        target / "picture.json",
        {
            "category": "T2Fresh",
            "subcategory": slug,
            "source": "generated_record_benchmark_staging",
            "path": None,
        },
    )
    dump_json(
        target / "collections" / "workbench.json",
        {
            "schema_version": 1,
            "record_id": row["record_id"],
            "archived": False,
            "benchmark_staging": True,
        },
    )
    return {
        "record_id": row["record_id"],
        "rating": row["rating"],
        "source_record": str(source.relative_to(PROJECT_ROOT)),
        "model_sha256": sha256_file(target_revision / "model.py"),
        "prompt_sha256": (
            sha256_file(target_revision / "prompt.txt")
            if (target_revision / "prompt.txt").is_file()
            else None
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    output = args.out.resolve()
    output.relative_to(EXP_ROOT.resolve())
    records_root = output / "records"
    source_maps = output / "source_maps"
    templates = {path.stem.casefold() for path in (TEMPLATE_ROOT / "agent/templates").glob("*.py")}

    task_rows: list[dict[str, Any]] = []
    for task in TASKS:
        slug = task["slug"]
        if slug.casefold() in templates:
            raise RuntimeError(f"fresh task already has a target template: {slug}")
        eligible = eligible_records(slug)
        selected = select_diverse(eligible, MAX_RECORDS_PER_TASK)
        if len(selected) < 8:
            raise RuntimeError(f"{slug}: only {len(selected)} eligible records")
        staged = [stage_record(row, slug, records_root) for row in selected]
        pool = discover_source_pool(
            records_root,
            picture_category="T2Fresh",
            picture_subcategory=slug,
        )
        if pool.problems or len(pool.records) != len(selected):
            raise RuntimeError(f"{slug}: staged source pool invalid: {pool.problems}")
        source_map_path = source_maps / f"{slug}.md"
        text = render_source_map_scaffold(
            slug=slug,
            picture_category="T2Fresh",
            picture_subcategory=slug,
            source_pool=pool,
        )
        if source_map_path.exists() and source_map_path.read_text(encoding="utf-8") != text:
            raise RuntimeError(f"existing SourceMap scaffold differs: {source_map_path}")
        if not source_map_path.exists():
            source_map_path.parent.mkdir(parents=True, exist_ok=True)
            source_map_path.write_text(text, encoding="utf-8")
        task_rows.append(
            {
                **task,
                "eligible_record_count": len(eligible),
                "selected_record_count": len(selected),
                "source_map_scaffold": str(source_map_path.relative_to(PROJECT_ROOT)),
                "source_map_scaffold_sha256": sha256_file(source_map_path),
                "records": staged,
            }
        )

    counts = {level: sum(row["complexity"] == level for row in task_rows) for level in (
        "simple", "medium", "complex"
    )}
    if counts != {"simple": 4, "medium": 4, "complex": 4}:
        raise RuntimeError(f"unbalanced task complexities: {counts}")
    manifest_path = output / "formal_source_manifest.json"
    payload = {
        "schema_version": 1,
        "experiment_id": "t2_formal_unseen_authoring_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "selection_policy": (
            "Exact unseen category slug; max(primary_rating, secondary_rating)>=4; active model.py; "
            "up to 12 records selected by deterministic greedy prompt-token Jaccard diversity."
        ),
        "source_map_status": "scaffold_pending_review",
        "task_count": len(task_rows),
        "complexity_counts": counts,
        "tasks": task_rows,
    }
    if manifest_path.exists():
        existing = load_json(manifest_path)
        existing.pop("created_at", None)
        comparable = dict(payload)
        comparable.pop("created_at", None)
        if existing != comparable:
            raise RuntimeError("existing formal source manifest differs from frozen selection")
    else:
        dump_json(manifest_path, payload)
    print(json.dumps({
        "task_count": len(task_rows),
        "selected_records": sum(row["selected_record_count"] for row in task_rows),
        "complexity_counts": counts,
        "manifest": str(manifest_path),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
