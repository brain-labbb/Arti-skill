#!/usr/bin/env python3
"""Audit the pinned yuchen0187/partnet-mobility Hugging Face snapshot."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import pyarrow.parquet as pq


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
REPO_ID = "yuchen0187/partnet-mobility"
REVISION = "bf39e304f19a6c131b5244f128b79ec35000bb02"
SNAPSHOT = EXP_ROOT / "baselines/partnet-mobility-hf/snapshot" / REVISION
OUTPUT = EXP_ROOT / "baselines/partnet-mobility-hf"
EXPECTED = [
    ("train-00000-of-00006.parquet", 290601821, "22e75485ca8952f8a149bb7b7d03264e79556dfb8e3de793f78b7b499636bbf0"),
    ("train-00001-of-00006.parquet", 290660007, "1323c916c9c556412161ca5dc3fa54f7ac234be82f0bff85791349aff1826cd0"),
    ("train-00002-of-00006.parquet", 291740259, "6ee6a1804540ab112740733e1a53d4a32110ac3d9e98544be7ebd62d335c819b"),
    ("train-00003-of-00006.parquet", 291133220, "c96487e5d6be3187b9270aca285dad57174189e72622ad4c614e0b0723a8921f"),
    ("train-00004-of-00006.parquet", 290738883, "35b2c76151d108f6e723b6c78811c4bc40d33d6309eda71e3af7bf9c610daf7c"),
    ("train-00005-of-00006.parquet", 290933671, "17e799205c446dacaf5ed6f9fb606791f71324c74ebeabad461d5b72e546ab34"),
]


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    resolved.relative_to(WORKSPACE.resolve(strict=True))
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    snapshot = contained(SNAPSHOT)
    output = contained(OUTPUT)
    rows = []
    schemas = set()
    for filename, expected_size, expected_lfs_oid in EXPECTED:
        path = contained(snapshot / "data" / filename)
        actual_hash = sha256_file(path)
        parquet = pq.ParquetFile(path)
        schema = str(parquet.schema_arrow)
        schemas.add(schema)
        rows.append(
            {
                "path": str(path),
                "repo_relative_path": f"data/{filename}",
                "size_bytes": path.stat().st_size,
                "expected_size_bytes": expected_size,
                "sha256": actual_hash,
                "hf_lfs_oid_sha256": expected_lfs_oid,
                "size_matches": path.stat().st_size == expected_size,
                "sha256_matches_lfs_oid": actual_hash == expected_lfs_oid,
                "num_rows": parquet.metadata.num_rows,
                "num_row_groups": parquet.metadata.num_row_groups,
                "schema": schema,
                "schema_names": parquet.schema_arrow.names,
            }
        )

    readme = contained(snapshot / "README.md")
    attributes = contained(snapshot / ".gitattributes")
    manifest = {
        "repo_id": REPO_ID,
        "revision": REVISION,
        "snapshot_path": str(snapshot),
        "downloaded_file_count": len(rows) + 2,
        "parquet_shard_count": len(rows),
        "parquet_download_size_bytes": sum(row["size_bytes"] for row in rows),
        "num_rows": sum(row["num_rows"] for row in rows),
        "schema_consistent_across_shards": len(schemas) == 1,
        "schema_names": rows[0]["schema_names"],
        "schema": rows[0]["schema"],
        "shards": rows,
        "metadata_files": [
            {
                "path": str(readme),
                "size_bytes": readme.stat().st_size,
                "sha256": sha256_file(readme),
            },
            {
                "path": str(attributes),
                "size_bytes": attributes.stat().st_size,
                "sha256": sha256_file(attributes),
            },
        ],
        "declared_license": None,
        "license_status": "absent from dataset card metadata at pinned revision",
        "published_features": ["xyz", "rgb", "mask"],
        "hierarchy_usable": False,
        "hierarchy_blockers": [
            "no category field",
            "no object identity field",
            "no URDF field",
            "no semantic hierarchy field",
            "no mobility annotation field",
        ],
    }
    manifest_path = output / "download_manifest.json"
    write_json(manifest_path, manifest)

    checks = {
        "six_shards": len(rows) == 6,
        "all_sizes_match": all(row["size_matches"] for row in rows),
        "all_sha256_match_lfs_oid": all(row["sha256_matches_lfs_oid"] for row in rows),
        "row_count_is_2290": manifest["num_rows"] == 2290,
        "schema_consistent": manifest["schema_consistent_across_shards"],
        "top_level_schema_is_xyz_rgb_mask": manifest["schema_names"] == ["xyz", "rgb", "mask"],
        "license_absent": manifest["declared_license"] is None,
        "hierarchy_usable_false": manifest["hierarchy_usable"] is False,
    }
    verification = {
        "passed": all(checks.values()),
        "checks": checks,
        "download_manifest_sha256": sha256_file(manifest_path),
        "auditor_sha256": sha256_file(Path(__file__)),
    }
    write_json(output / "download_verification.json", verification)
    if not verification["passed"]:
        raise ValueError(f"download audit failed: {checks}")

    report = [
        "# yuchen0187/partnet-mobility download audit",
        "",
        f"- Revision: `{REVISION}`.",
        f"- Snapshot: `{snapshot}`.",
        f"- Parquet shards: 6; compressed bytes: {manifest['parquet_download_size_bytes']:,}.",
        f"- Rows: {manifest['num_rows']:,}; top-level schema: `xyz`, `rgb`, `mask`.",
        "- Every shard SHA-256 matches its pinned Hugging Face LFS OID.",
        "- Declared license: absent at the pinned revision.",
        "- Hierarchy usable: false. The release has no category, identity, URDF, hierarchy, or mobility fields.",
        "",
        "This downloaded mirror is retained as requested and audited, but it is not used as the Table 3 hierarchy reference.",
    ]
    (output / "download_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps({"manifest": str(manifest_path), "verification": verification}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
