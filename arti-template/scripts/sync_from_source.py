#!/usr/bin/env python3
"""Sync 5-star sample records from a source repo (articraft_data) into this repo.

Optimization ① (upstream -> downstream sync). The per-subcategory 5-star sample
pool lives in articraft_data; the modular spec's `model.py:Lx-Ly` references and
`external examples --rating-min 5` only work once those records exist HERE. This
copies the record dirs + their materialization cache, stamps `rating=5` (so the
samples surface as 5-star sources), and rebuilds the search index.

Records can be named explicitly (`--records id1,id2`) or derived from a source
map (`--source-map <path>`): every `rec_..._<hash>` token in the map is resolved
against the source repo's records dir.

Defaults to a dry run; pass --execute to actually copy.

Usage:
    python scripts/sync_from_source.py \
        --source-repo /mnt/zsn/lyb/arti-skill/articraft_data \
        --source-map /mnt/zsn/lyb/arti-skill/articraft_data/picture_expansion/template_source_maps/Fence__Cascade_fences.md \
        --rating 5 --execute
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys


def _repo_root() -> str:
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _record_ids_from_source_map(source_map: str, src_records: str) -> list[str]:
    """Resolve record ids referenced by a source map, STRICTLY.

    A `rec_...` token is accepted only if it is (a) the exact name of an existing
    record dir, or (b) an abbreviated `rec_..._<hash>` whose trailing token is a
    real hex hash (>=6 chars) that resolves to EXACTLY ONE record dir. Short
    trailing words (e.g. `oval`, `pan`) and prose `rec_` fragments are ignored —
    this is what prevents the old `*{token}*` glob from over-matching the whole
    repo (1525 hits) and then crashing in _stamp_rating on an unrelated record.
    """
    text = open(source_map, encoding="utf-8").read()
    available = set(os.listdir(src_records)) if os.path.isdir(src_records) else set()
    out: list[str] = []
    seen: set[str] = set()

    def _add(rid: str) -> None:
        if rid not in seen:
            seen.add(rid)
            out.append(rid)

    for tok in re.findall(r"rec_[\w.\-]+", text):
        tok = tok.strip(".")
        if tok in available:  # (a) exact full record id
            _add(tok)
            continue
        h = tok.rsplit("_", 1)[-1].strip(".")  # (b) abbreviated rec_..._<hash>
        if re.fullmatch(r"[0-9a-fA-F]{6,}", h):
            hits = [d for d in available if d == h or d.endswith("_" + h)]
            if len(hits) == 1:
                _add(hits[0])
    return out


def _copy_tree(src: str, dst: str, *, execute: bool) -> str:
    if not os.path.isdir(src):
        return "missing-src"
    if os.path.exists(dst):
        return "skip-exists"
    if execute:
        shutil.copytree(src, dst)
    return "copied"


def _stamp_rating(record_json: str, rating: int, *, execute: bool) -> str:
    if not os.path.exists(record_json):
        return "no-record.json"
    try:
        with open(record_json, encoding="utf-8") as fh:
            data = json.load(fh)
    except (json.JSONDecodeError, OSError, UnicodeDecodeError):
        # empty / git-LFS-pointer / malformed record.json — skip, don't crash the run
        return "skip-unreadable-json"
    if data.get("rating") == rating:
        return "rating-ok"
    data["rating"] = rating
    if execute:
        json.dump(data, open(record_json, "w", encoding="utf-8"), indent=2)
    return f"rating->{rating}"


def main() -> int:
    ap = argparse.ArgumentParser(description="Sync 5-star sample records from a source repo.")
    ap.add_argument("--source-repo", required=True)
    ap.add_argument("--records", default="", help="Comma-separated full record ids.")
    ap.add_argument("--source-map", default="", help="Source map to derive record ids from.")
    ap.add_argument("--rating", type=int, default=5)
    ap.add_argument("--execute", action="store_true", help="Actually copy (default: dry run).")
    args = ap.parse_args()

    repo_root = _repo_root()
    src_records = os.path.join(args.source_repo, "data", "records")
    src_mat = os.path.join(args.source_repo, "data", "cache", "record_materialization")
    dst_records = os.path.join(repo_root, "data", "records")
    dst_mat = os.path.join(repo_root, "data", "cache", "record_materialization")

    ids: list[str] = [r.strip() for r in args.records.split(",") if r.strip()]
    if args.source_map:
        ids.extend(_record_ids_from_source_map(args.source_map, src_records))
    ids = list(dict.fromkeys(ids))  # de-dupe preserve order
    if not ids:
        print("No record ids resolved (give --records or --source-map).", file=sys.stderr)
        return 2

    mode = "EXECUTE" if args.execute else "DRY-RUN"
    print(f"[{mode}] syncing {len(ids)} record(s) from {args.source_repo}")
    n_copied = 0
    for rid in ids:
        rec_status = _copy_tree(
            os.path.join(src_records, rid), os.path.join(dst_records, rid), execute=args.execute
        )
        mat_status = _copy_tree(
            os.path.join(src_mat, rid), os.path.join(dst_mat, rid), execute=args.execute
        )
        rating_status = _stamp_rating(
            os.path.join(dst_records if args.execute else src_records, rid, "record.json"),
            args.rating,
            execute=args.execute,
        )
        if rec_status == "copied":
            n_copied += 1
        print(f"  {rid}\n    record={rec_status}  materialization={mat_status}  {rating_status}")

    if args.execute and n_copied:
        try:
            from storage.repo import StorageRepo
            from storage.search import SearchIndex

            stats = SearchIndex(StorageRepo(repo_root)).rebuild()
            print(
                f"Rebuilt search index: records={stats.record_count} "
                f"workbench_entries={stats.workbench_entry_count}"
            )
        except Exception as exc:  # noqa: BLE001
            print(f"WARN: search index rebuild failed ({exc}); run "
                  f"`uv run articraft workbench rebuild-search-index` manually.")
    elif not args.execute:
        print("Dry run complete. Re-run with --execute to copy + stamp rating + reindex.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
