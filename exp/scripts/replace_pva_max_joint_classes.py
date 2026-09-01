#!/usr/bin/env python3
"""Replace fence and Ferris-wheel samples with maximum-joint PV-A assets."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import os
from collections import defaultdict
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Iterable, Mapping
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts.prepare_ours_pva_800 import (  # noqa: E402
    _archive_members,
    canonical_sha256,
    extract_selected_archive,
    package_binding,
    resolve_archive_name,
    sha256_file,
)
from exp.scripts.prepare_pva_per_class_n5 import (  # noqa: E402
    EXPECTED_PVA_MANIFEST_SHA256,
    PVA_ARCHIVES,
    PVA_MANIFEST,
)


SOURCE = REPO / "exp/PV-A-per-class-n5"
DEFAULT_OUTPUT = REPO / "exp/PV-A-per-class-n5-max-joints"
TARGET_CATEGORIES = (
    "Fence_Cascade_fences_MORE_THAN_1",
    "ferris_wheel",
)
PER_CLASS = 5
TIE_SEED = "arti-skill-pva-max-joints-tie-v1"
PROTOCOL_ID = "pva-per-class-n5-fence-ferris-max-movable-joints-v1"


def count_movable_joints(xml_bytes: bytes) -> int:
    root = ET.fromstring(xml_bytes)
    if root.tag != "robot":
        raise ValueError(f"expected robot root element, got {root.tag!r}")
    return sum(
        joint.get("type", "") != "fixed"
        for joint in root.findall("joint")
    )


def tie_rank(slug: str, asset_id: str, *, seed: str) -> str:
    identity = "\0".join((PROTOCOL_ID, seed, slug, asset_id))
    return hashlib.sha256(identity.encode("utf-8")).hexdigest()


def select_max_joint_rows(
    rows: Iterable[Mapping[str, str]],
    counts: Mapping[str, int],
    n: int,
    *,
    seed: str,
) -> list[dict[str, Any]]:
    if n < 1:
        raise ValueError("sample size must be positive")
    ranked: list[dict[str, Any]] = []
    seen: set[str] = set()
    slug: str | None = None
    for raw in rows:
        row: dict[str, Any] = dict(raw)
        asset_id = str(row.get("asset_id", ""))
        row_slug = str(row.get("slug", ""))
        if not asset_id or not row_slug or asset_id in seen:
            raise ValueError(f"invalid or duplicate candidate: {row_slug}/{asset_id}")
        if slug is None:
            slug = row_slug
        elif row_slug != slug:
            raise ValueError("selection candidates must belong to one class")
        if asset_id not in counts:
            raise ValueError(f"missing movable-joint count: {asset_id}")
        seen.add(asset_id)
        row["movable_joint_count"] = int(counts[asset_id])
        row["tie_rank_sha256"] = tie_rank(row_slug, asset_id, seed=seed)
        ranked.append(row)
    if len(ranked) < n:
        raise ValueError(f"requested {n} assets from only {len(ranked)} candidates")
    ranked.sort(
        key=lambda row: (
            -row["movable_joint_count"],
            row["tie_rank_sha256"],
            row["asset_id"],
        )
    )
    return ranked[:n]


def load_source_rows() -> list[dict[str, str]]:
    digest = sha256_file(PVA_MANIFEST)
    if digest != EXPECTED_PVA_MANIFEST_SHA256:
        raise ValueError(f"PV-A manifest SHA256 mismatch: {digest}")
    with PVA_MANIFEST.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    return rows


def extract_candidate_urdfs(
    archive: Path,
    asset_ids: list[str],
    destination: Path,
) -> None:
    members = set(_archive_members(archive))
    chosen = [f"{asset_id}/model.urdf" for asset_id in asset_ids]
    missing = [name for name in chosen if name not in members]
    if missing:
        raise ValueError(f"candidate URDFs missing from {archive.name}: {missing[:5]}")
    destination.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=destination.parent, delete=False
    ) as handle:
        list_path = Path(handle.name)
        handle.write("".join(f"{name}\n" for name in chosen))
    try:
        subprocess.run(
            [
                "tar",
                "--zstd",
                "-xf",
                str(archive),
                "-C",
                str(destination),
                "--no-same-owner",
                "--no-same-permissions",
                "--verbatim-files-from",
                "--files-from",
                str(list_path),
            ],
            check=True,
        )
    finally:
        list_path.unlink(missing_ok=True)


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def prepare(output: Path) -> Path:
    source = SOURCE.resolve(strict=True)
    source_manifest_path = source / "manifest.json"
    source_manifest = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    output = output.resolve(strict=False)
    if output.exists():
        raise FileExistsError(output)
    staging = output.with_name(f".{output.name}.work")
    if staging.exists():
        raise FileExistsError(staging)
    staging.mkdir(parents=True)

    all_rows = load_source_rows()
    rows_by_category = {
        category: [row for row in all_rows if row["slug"] == category]
        for category in TARGET_CATEGORIES
    }
    archive_names = {path.name for path in PVA_ARCHIVES.glob("*.tar.zst")}
    selected_by_category: dict[str, list[dict[str, Any]]] = {}
    archive_bindings: dict[str, dict[str, Any]] = {}

    candidate_root = staging / "candidate_urdfs"
    for category in TARGET_CATEGORIES:
        rows = rows_by_category[category]
        if not rows:
            raise ValueError(f"target category absent from release: {category}")
        grouped: dict[str, list[str]] = defaultdict(list)
        for row in rows:
            archive_name = resolve_archive_name(
                category, row["asset_id"], archive_names
            )
            row["archive_name"] = archive_name
            grouped[archive_name].append(row["asset_id"])
        category_candidates = candidate_root / category
        for archive_name, asset_ids in sorted(grouped.items()):
            extract_candidate_urdfs(
                PVA_ARCHIVES / archive_name,
                sorted(asset_ids),
                category_candidates,
            )
        counts = {
            row["asset_id"]: count_movable_joints(
                (category_candidates / row["asset_id"] / "model.urdf").read_bytes()
            )
            for row in rows
        }
        selected_by_category[category] = select_max_joint_rows(
            rows, counts, PER_CLASS, seed=TIE_SEED
        )

    (staging / "assets").mkdir()
    for category in TARGET_CATEGORIES:
        category_output = staging / "assets" / category
        grouped: dict[str, list[str]] = defaultdict(list)
        for row in selected_by_category[category]:
            grouped[row["archive_name"]].append(row["asset_id"])
        for archive_name, asset_ids in sorted(grouped.items()):
            binding = extract_selected_archive(
                PVA_ARCHIVES / archive_name,
                sorted(asset_ids),
                category_output,
            )
            archive_bindings[archive_name] = binding

    shutil.rmtree(candidate_root)
    replacement_iterators = {
        category: iter(selected_by_category[category])
        for category in TARGET_CATEGORIES
    }
    assets: list[dict[str, Any]] = []
    for original in source_manifest["assets"]:
        category = str(original["category"])
        if category not in replacement_iterators:
            assets.append(dict(original))
            continue
        selected = next(replacement_iterators[category])
        package = (staging / "assets" / category / selected["asset_id"]).resolve(
            strict=True
        )
        binding = package_binding(package)
        assets.append(
            {
                "selection_index": len(assets),
                "dataset_id": f"PV-A/{category}/{selected['asset_id']}",
                "category": category,
                "asset_id": selected["asset_id"],
                "seed": int(selected["seed"]),
                "stem": selected["stem"],
                "overrides_json": selected["overrides_json"],
                "selection_method": "maximum_declared_movable_joint_count",
                "movable_joint_count": selected["movable_joint_count"],
                "tie_rank_sha256": selected["tie_rank_sha256"],
                "archive_name": selected["archive_name"],
                "archive_sha256": archive_bindings[selected["archive_name"]]["sha256"],
                "package": str(output / "assets" / category / selected["asset_id"]),
                "primary_urdf_relative_path": "model.urdf",
                "urdf_sha256": sha256_file(package / "model.urdf"),
                "package_binding": binding,
            }
        )

    manifest = dict(source_manifest)
    manifest.pop("manifest_content_sha256", None)
    manifest.update(
        {
            "schema_version": "pva-per-class-extracted-cohort/v2",
            "protocol_id": PROTOCOL_ID,
            "created_at_utc": dt.datetime.now(dt.timezone.utc)
            .isoformat()
            .replace("+00:00", "Z"),
            "classification": "FROZEN_MIXED_STRATIFIED_SAMPLE",
            "source_cohort_manifest": str(source_manifest_path),
            "source_cohort_manifest_sha256": sha256_file(source_manifest_path),
            "selection_overrides": {
                "categories": list(TARGET_CATEGORIES),
                "metric": "declared non-fixed joints in model.urdf",
                "order": "movable_joint_count descending, then deterministic tie rank",
                "tie_seed": TIE_SEED,
                "per_class": PER_CLASS,
                "selected": selected_by_category,
            },
            "assets": assets,
        }
    )
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    write_json(staging / "manifest.json", manifest)
    (staging / "selection.jsonl").write_text(
        "".join(
            json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n"
            for row in assets
        ),
        encoding="utf-8",
    )
    os.replace(staging, output)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    result = prepare(args.output)
    print(json.dumps({"status": "COMPLETE", "output": str(result)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
