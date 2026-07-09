"""Sync picture-expansion variant records into an arti-template checkout as 5-star template sources.

Copies each record directory (and its materialization cache, if present) into the
target checkout, then stamps ``rating`` on the target copy so the template-side
spec workflow (``articraft external examples --rating-min 5``) can find it.
Collections sidecars are copied verbatim, so workbench-only records stay
workbench-only in the target.

Usage:
    python scripts/sync_template_sources.py --target /path/to/arti-template \
        [--rating 5] [--rated-by picture_expansion_sync] [--source-map path.md] \
        rec_xxx rec_yyy ...
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sync_record(record_id: str, target_root: Path, rating: int, rated_by: str) -> None:
    src_record = ROOT / "data" / "records" / record_id
    if not src_record.is_dir():
        raise FileNotFoundError(f"record not found: {src_record}")

    dst_record = target_root / "data" / "records" / record_id
    if dst_record.exists():
        shutil.rmtree(dst_record)
    shutil.copytree(src_record, dst_record)

    src_cache = ROOT / "data" / "cache" / "record_materialization" / record_id
    if src_cache.is_dir():
        dst_cache = target_root / "data" / "cache" / "record_materialization" / record_id
        if dst_cache.exists():
            shutil.rmtree(dst_cache)
        dst_cache.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(src_cache, dst_cache)

    record_json = dst_record / "record.json"
    record = json.loads(record_json.read_text())
    record["rating"] = rating
    record["rated_by"] = rated_by
    record_json.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")

    collections = record.get("collections")
    print(f"synced {record_id}  rating={rating}  collections={collections}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", required=True, help="arti-template checkout root")
    parser.add_argument("--rating", type=int, default=5)
    parser.add_argument("--rated-by", default="picture_expansion_sync")
    parser.add_argument(
        "--source-map",
        help="optional source-map markdown to copy into the target's "
        "articraft_template_authoring/picture_source_maps/",
    )
    parser.add_argument("record_ids", nargs="+")
    args = parser.parse_args()

    target_root = Path(args.target).resolve()
    if not (target_root / "data" / "records").is_dir():
        print(f"target does not look like an articraft checkout: {target_root}")
        return 1

    for record_id in args.record_ids:
        sync_record(record_id, target_root, args.rating, args.rated_by)

    if args.source_map:
        src = Path(args.source_map)
        dst_dir = target_root / "articraft_template_authoring" / "picture_source_maps"
        dst_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst_dir / src.name)
        print(f"copied source map -> {dst_dir / src.name}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
