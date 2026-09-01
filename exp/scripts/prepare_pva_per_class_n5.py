#!/usr/bin/env python3
"""Freeze and extract a deterministic five-asset sample per PV-A class."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import os
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts.prepare_ours_pva_800 import (
    canonical_sha256,
    extract_selected_archive,
    package_binding,
    resolve_archive_name,
    sha256_file,
)


PVA_ROOT = Path("/mnt/zsn/data/particulate/datasets/PV-A")
PVA_MANIFEST = PVA_ROOT / "manifest.csv"
PVA_ARCHIVES = PVA_ROOT / "archives"
DEFAULT_OUTPUT = REPO / "exp/PV-A-per-class-n5"
PROTOCOL_ID = "pva-per-class-hash-sample-v1"
SAMPLE_SEED = "arti-skill-pva-per-class-n5-v1"
PER_CLASS = 5
EXTRACTION_WORKERS = 8
EXPECTED_PVA_MANIFEST_SHA256 = "11bbfa00067e5b8a4fe788db085f896a9754a6f2ec88818c16d9cee1c137c06a"


def rank_row(
    slug: str,
    asset_id: str,
    *,
    manifest_sha256: str,
    seed: str,
) -> str:
    identity = "\0".join((PROTOCOL_ID, manifest_sha256, seed, slug, asset_id))
    return hashlib.sha256(identity.encode("utf-8")).hexdigest()


def select_per_class(
    rows: Iterable[Mapping[str, str]],
    n: int,
    *,
    manifest_sha256: str,
    seed: str,
) -> list[dict[str, str]]:
    if n < 1:
        raise ValueError("per-class sample size must be positive")
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    seen: set[tuple[str, str]] = set()
    for raw in rows:
        row = dict(raw)
        slug = row.get("slug", "")
        asset_id = row.get("asset_id", "")
        identity = (slug, asset_id)
        if not slug or not asset_id or identity in seen:
            raise ValueError(f"invalid or duplicate PV-A identity: {identity!r}")
        seen.add(identity)
        row["rank_sha256"] = rank_row(
            slug,
            asset_id,
            manifest_sha256=manifest_sha256,
            seed=seed,
        )
        grouped[slug].append(row)
    if not grouped:
        raise ValueError("PV-A manifest contains no assets")
    selected: list[dict[str, str]] = []
    for slug in sorted(grouped):
        candidates = grouped[slug]
        if len(candidates) < n:
            raise ValueError(
                f"class {slug!r} has {len(candidates)} candidates; {n} required"
            )
        candidates.sort(key=lambda row: (row["rank_sha256"], row["asset_id"]))
        selected.extend(candidates[:n])
    return selected


def load_rows() -> tuple[list[dict[str, str]], str]:
    manifest_sha256 = sha256_file(PVA_MANIFEST)
    if manifest_sha256 != EXPECTED_PVA_MANIFEST_SHA256:
        raise ValueError(
            f"PV-A manifest SHA256 mismatch: {manifest_sha256}"
        )
    with PVA_MANIFEST.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    required = {"slug", "stem", "seed", "asset_id", "overrides_json"}
    if not rows or set(rows[0]) != required:
        raise ValueError("PV-A manifest schema mismatch")
    return rows, manifest_sha256


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def prepare(output: Path) -> Path:
    output = output.resolve(strict=False)
    if output.exists():
        raise FileExistsError(output)
    staging = output.with_name(f".{output.name}.work")
    if staging.exists():
        raise FileExistsError(staging)
    staging.mkdir(parents=True)

    rows, manifest_sha256 = load_rows()
    selected = select_per_class(
        rows,
        PER_CLASS,
        manifest_sha256=manifest_sha256,
        seed=SAMPLE_SEED,
    )
    archive_names = {path.name for path in PVA_ARCHIVES.glob("*.tar.zst")}
    by_archive_and_slug: dict[tuple[str, str], list[str]] = defaultdict(list)
    for row in selected:
        archive_name = resolve_archive_name(
            row["slug"], row["asset_id"], archive_names
        )
        row["archive_name"] = archive_name
        by_archive_and_slug[(archive_name, row["slug"])].append(row["asset_id"])

    selection_receipt = {
        "schema_version": "pva-per-class-selection/v1",
        "protocol_id": PROTOCOL_ID,
        "seed": SAMPLE_SEED,
        "per_class": PER_CLASS,
        "class_count": len({row["slug"] for row in selected}),
        "asset_count": len(selected),
        "source_manifest": str(PVA_MANIFEST),
        "source_manifest_sha256": manifest_sha256,
        "ordered_identities_sha256": canonical_sha256(
            [[row["slug"], row["asset_id"]] for row in selected]
        ),
        "assets": selected,
    }
    write_json(staging / "selection.json", selection_receipt)
    (staging / "selection.jsonl").write_text(
        "".join(
            json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n"
            for row in selected
        ),
        encoding="utf-8",
    )

    extraction_root = staging / "assets"
    archive_bindings: dict[str, dict[str, Any]] = {}
    extraction_jobs = sorted(by_archive_and_slug.items())
    with ThreadPoolExecutor(max_workers=EXTRACTION_WORKERS) as executor:
        futures = {
            executor.submit(
                extract_selected_archive,
                PVA_ARCHIVES / archive_name,
                sorted(asset_ids),
                extraction_root / slug,
            ): archive_name
            for (archive_name, slug), asset_ids in extraction_jobs
        }
        for future in as_completed(futures):
            archive_name = futures[future]
            binding = future.result()
            previous = archive_bindings.setdefault(archive_name, binding)
            if previous["sha256"] != binding["sha256"]:
                raise ValueError(
                    f"archive binding changed during extraction: {archive_name}"
                )

    assets: list[dict[str, Any]] = []
    for index, row in enumerate(selected):
        package = (extraction_root / row["slug"] / row["asset_id"]).resolve(
            strict=True
        )
        package.relative_to(extraction_root.resolve(strict=True))
        for required in ("model.urdf", "appearance.json", "physics.json"):
            if not (package / required).is_file():
                raise ValueError(
                    f"PV-A package missing {required}: {row['slug']}/{row['asset_id']}"
                )
        assets.append(
            {
                "selection_index": index,
                "dataset_id": f"PV-A/{row['slug']}/{row['asset_id']}",
                "category": row["slug"],
                "asset_id": row["asset_id"],
                "seed": int(row["seed"]),
                "stem": row["stem"],
                "overrides_json": row["overrides_json"],
                "rank_sha256": row["rank_sha256"],
                "archive_name": row["archive_name"],
                "archive_sha256": archive_bindings[row["archive_name"]]["sha256"],
                "package": str(output / "assets" / row["slug"] / row["asset_id"]),
                "primary_urdf_relative_path": "model.urdf",
                "urdf_sha256": sha256_file(package / "model.urdf"),
                "package_binding": package_binding(package),
            }
        )

    manifest = {
        "schema_version": "pva-per-class-extracted-cohort/v1",
        "protocol_id": PROTOCOL_ID,
        "created_at_utc": dt.datetime.now(dt.timezone.utc)
        .isoformat()
        .replace("+00:00", "Z"),
        "dataset": "PV-A-per-class-n5",
        "classification": "FROZEN_RANDOM_STRATIFIED_SAMPLE",
        "n_eval": len(assets),
        "class_count": selection_receipt["class_count"],
        "per_class": PER_CLASS,
        "selection": {
            key: selection_receipt[key]
            for key in (
                "protocol_id",
                "seed",
                "source_manifest",
                "source_manifest_sha256",
                "ordered_identities_sha256",
            )
        },
        "archive_bindings": archive_bindings,
        "assets": assets,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    write_json(staging / "manifest.json", manifest)
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
