#!/usr/bin/env python3
"""Freeze Brain-500 plus a deterministic PV-A-300 sample as Ours-800."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import tempfile
from typing import Any, Iterable, Mapping


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
PVA_ROOT = Path("/mnt/zsn/data/particulate/datasets/PV-A")
PVA_MANIFEST = PVA_ROOT / "manifest.csv"
PVA_ARCHIVES = PVA_ROOT / "archives"
BRAIN_TABLE4_MANIFEST = REPO / "exp/runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/frozen_manifest.json"
BRAIN_ROOT = REPO / "exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813"
DEFAULT_OUTPUT = REPO / "exp/runtime/ours_pva_800_cohort_v1"
PROTOCOL_ID = "ours-pva-global-sample-v1"
COHORT_PROTOCOL_ID = "ours-brain500-pva300-cohort-v1"
SAMPLE_SEED = "arti-skill-ours-pva-n300-v1"
PVA_N = 300
BRAIN_N = 500
EXPECTED_PVA_MANIFEST_SHA256 = "11bbfa00067e5b8a4fe788db085f896a9754a6f2ec88818c16d9cee1c137c06a"
EXPECTED_BRAIN_MANIFEST_SHA256 = "1b29d868112dcda326a08f8e3439d6b96c65833b99cc33af3bfcdb58fb4c2e24"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def rank_row(slug: str, asset_id: str, *, manifest_sha256: str, seed: str) -> str:
    identity = "\0".join((PROTOCOL_ID, manifest_sha256, seed, slug, asset_id))
    return hashlib.sha256(identity.encode("utf-8")).hexdigest()


def select_rows(rows: Iterable[Mapping[str, str]], n: int, *, manifest_sha256: str, seed: str) -> list[dict[str, str]]:
    if n < 1:
        raise ValueError("sample size must be positive")
    ranked: list[tuple[str, str, str, dict[str, str]]] = []
    seen: set[tuple[str, str]] = set()
    for raw in rows:
        row = dict(raw)
        slug, asset_id = row.get("slug", ""), row.get("asset_id", "")
        identity = (slug, asset_id)
        if not slug or not asset_id or identity in seen:
            raise ValueError(f"invalid or duplicate PV-A identity: {identity!r}")
        seen.add(identity)
        rank = rank_row(slug, asset_id, manifest_sha256=manifest_sha256, seed=seed)
        row["rank_sha256"] = rank
        ranked.append((rank, slug, asset_id, row))
    if len(ranked) < n:
        raise ValueError(f"requested {n} assets from only {len(ranked)} candidates")
    ranked.sort(key=lambda item: item[:3])
    return [item[3] for item in ranked[:n]]


def resolve_archive_name(slug: str, asset_id: str, available_names: set[str]) -> str:
    whole = f"{slug}.tar.zst"
    if whole in available_names:
        return whole
    match = re.fullmatch(r"seed_(\d{4})", asset_id)
    if match is None:
        raise ValueError(f"invalid PV-A asset_id: {asset_id!r}")
    index = int(match.group(1))
    sharded = f"{slug}_part{index // 400:02d}.tar.zst"
    if sharded in available_names:
        return sharded
    raise ValueError(f"archive unavailable for {slug}/{asset_id}: expected {whole!r} or {sharded!r}")


def merge_rows(brain: list[Mapping[str, Any]], pva: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    if len(brain) != BRAIN_N or len(pva) != PVA_N:
        raise ValueError(f"merged cohort requires exactly {BRAIN_N} Brain and {PVA_N} PV-A assets")
    merged: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, raw in enumerate([*brain, *pva]):
        row = dict(raw)
        identity = str(row.get("dataset_id", ""))
        if not identity or identity in seen:
            raise ValueError(f"duplicate dataset identity: {identity!r}")
        seen.add(identity)
        row["selection_index"] = index
        merged.append(row)
    return merged


def rebase_staging_packages(
    rows: Iterable[Mapping[str, Any]], staging: Path, output: Path
) -> list[dict[str, Any]]:
    staging = staging.resolve(strict=False)
    output = output.resolve(strict=False)
    rebased: list[dict[str, Any]] = []
    for raw in rows:
        row = dict(raw)
        package = Path(str(row["package"])).resolve(strict=False)
        try:
            relative = package.relative_to(staging)
        except ValueError:
            pass
        else:
            row["package"] = str(output / relative)
        rebased.append(row)
    return rebased


def package_binding(package: Path) -> dict[str, Any]:
    package = package.resolve(strict=True)
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            if (current / name).is_symlink():
                raise ValueError(f"package contains directory symlink: {package}: {name}")
        for name in file_names:
            path = current / name
            if path.is_symlink() or not path.is_file():
                raise ValueError(f"package contains non-regular file: {path}")
            relative = path.relative_to(package).as_posix()
            files.append({"path": relative, "bytes": path.stat().st_size, "sha256": sha256_file(path)})
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def _safe_member_name(name: str) -> bool:
    path = PurePosixPath(name)
    return bool(name) and not path.is_absolute() and ".." not in path.parts and "" not in path.parts


def _archive_members(archive: Path) -> list[str]:
    result = subprocess.run(
        ["tar", "--zstd", "-tf", str(archive)], check=True, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    members = result.stdout.splitlines()
    if not members or any(not _safe_member_name(name.rstrip("/")) for name in members):
        raise ValueError(f"archive contains unsafe or empty member names: {archive}")
    if len(members) != len(set(members)):
        raise ValueError(f"archive contains duplicate member names: {archive}")
    return members


def extract_selected_archive(archive: Path, selected_asset_ids: list[str], destination: Path) -> dict[str, Any]:
    members = _archive_members(archive)
    chosen: list[str] = []
    for asset_id in selected_asset_ids:
        prefix = f"{asset_id}/"
        matching = [name for name in members if name == asset_id or name == prefix or name.startswith(prefix)]
        if not matching or f"{asset_id}/model.urdf" not in matching:
            raise ValueError(f"selected asset is missing from archive: {archive.name}:{asset_id}")
        chosen.extend(matching)
    chosen = sorted(set(chosen))
    destination.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=destination.parent, delete=False) as handle:
        list_path = Path(handle.name)
        handle.write("".join(f"{name}\n" for name in chosen))
    try:
        subprocess.run(
            ["tar", "--zstd", "-xf", str(archive), "-C", str(destination),
             "--no-same-owner", "--no-same-permissions", "--no-recursion",
             "--verbatim-files-from", "--files-from", str(list_path)],
            check=True,
        )
    finally:
        list_path.unlink(missing_ok=True)
    return {"path": str(archive), "bytes": archive.stat().st_size, "sha256": sha256_file(archive),
            "selected_assets": len(selected_asset_ids), "selected_members": len(chosen)}


def _load_pva_rows() -> list[dict[str, str]]:
    if sha256_file(PVA_MANIFEST) != EXPECTED_PVA_MANIFEST_SHA256:
        raise ValueError("PV-A manifest SHA256 mismatch")
    with PVA_MANIFEST.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    required = {"slug", "stem", "seed", "asset_id", "overrides_json"}
    if not rows or set(rows[0]) != required:
        raise ValueError("PV-A manifest schema mismatch")
    return rows


def _brain_rows() -> list[dict[str, Any]]:
    if sha256_file(BRAIN_TABLE4_MANIFEST) != EXPECTED_BRAIN_MANIFEST_SHA256:
        raise ValueError("Brain frozen manifest SHA256 mismatch")
    manifest = json.loads(BRAIN_TABLE4_MANIFEST.read_text(encoding="utf-8"))
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != BRAIN_N:
        raise ValueError("Brain frozen cohort must contain 500 items")
    rows: list[dict[str, Any]] = []
    root = BRAIN_ROOT.resolve(strict=True)
    for index, item in enumerate(items):
        if item.get("order") != index:
            raise ValueError("Brain frozen order mismatch")
        package = (root / str(item["asset_root_relpath"])).resolve(strict=True)
        package.relative_to(root)
        rows.append({
            "dataset_id": str(item["dataset_id"]), "source": "Brain-500",
            "source_identity": str(item["dataset_id"]), "raw_category": str(item["category"]),
            "seed_name": str(item["seed_name"]), "package": str(package),
            "primary_urdf_relative_path": "model.urdf", "urdf_sha256": str(item["urdf_sha256"]),
            "package_binding": package_binding(package), "upstream_table4_item": item,
        })
    return rows


def _pva_rows(selected: list[dict[str, str]], extraction_root: Path, archive_bindings: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for row in selected:
        slug, asset_id = row["slug"], row["asset_id"]
        package = (extraction_root / slug / asset_id).resolve(strict=True)
        package.relative_to(extraction_root.resolve(strict=True))
        for required in ("model.urdf", "appearance.json", "physics.json"):
            if not (package / required).is_file():
                raise ValueError(f"PV-A package missing {required}: {slug}/{asset_id}")
        binding = package_binding(package)
        rows.append({
            "dataset_id": f"PV-A/{slug}/{asset_id}", "source": "PV-A-300",
            "source_identity": f"{slug}/{asset_id}", "raw_category": slug,
            "seed_name": asset_id, "seed": int(row["seed"]), "stem": row["stem"],
            "overrides_json": row["overrides_json"], "rank_sha256": row["rank_sha256"],
            "archive_name": row["archive_name"], "archive_sha256": archive_bindings[row["archive_name"]]["sha256"],
            "package": str(package), "primary_urdf_relative_path": "model.urdf",
            "urdf_sha256": sha256_file(package / "model.urdf"), "package_binding": binding,
        })
    return rows


def prepare(output: Path) -> Path:
    output = output.resolve(strict=False)
    if output.exists():
        raise FileExistsError(output)
    staging = output.with_name(f".{output.name}.work")
    if staging.exists():
        raise FileExistsError(staging)
    staging.mkdir(parents=True)
    try:
        pva_rows = _load_pva_rows()
        selected = select_rows(pva_rows, PVA_N, manifest_sha256=EXPECTED_PVA_MANIFEST_SHA256, seed=SAMPLE_SEED)
        archive_names = {path.name for path in PVA_ARCHIVES.glob("*.tar.zst")}
        grouped: dict[str, list[str]] = {}
        for row in selected:
            archive_name = resolve_archive_name(row["slug"], row["asset_id"], archive_names)
            row["archive_name"] = archive_name
            grouped.setdefault(archive_name, []).append(row["asset_id"])
        extraction_root = staging / "pva_assets"
        archive_bindings: dict[str, Any] = {}
        for archive_name in sorted(grouped):
            slug = next(row["slug"] for row in selected if row["archive_name"] == archive_name)
            archive_bindings[archive_name] = extract_selected_archive(
                PVA_ARCHIVES / archive_name, sorted(grouped[archive_name]), extraction_root / slug
            )
        pva = _pva_rows(selected, extraction_root, archive_bindings)
        brain = _brain_rows()
        merged = rebase_staging_packages(merge_rows(brain, pva), staging, output)
        fingerprints: dict[str, str] = {}
        for row in merged:
            fingerprint = str(row["package_binding"]["content_manifest_sha256"])
            if fingerprint in fingerprints:
                raise ValueError(f"duplicate package fingerprint: {fingerprints[fingerprint]} and {row['dataset_id']}")
            fingerprints[fingerprint] = str(row["dataset_id"])
        manifest = {
            "schema_version": "ours-pva-800-cohort/v1", "protocol_id": COHORT_PROTOCOL_ID,
            "created_at_utc": dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z"),
            "dataset": "Ours-800", "classification": "FORMAL", "n_eval": len(merged),
            "composition": {"Brain-500": BRAIN_N, "PV-A-300": PVA_N},
            "selection": {"protocol_id": PROTOCOL_ID, "seed": SAMPLE_SEED,
                          "pva_manifest_sha256": EXPECTED_PVA_MANIFEST_SHA256,
                          "selected_pva_identities_sha256": canonical_sha256([[r["slug"], r["asset_id"]] for r in selected]),
                          "ordered_dataset_ids_sha256": canonical_sha256([r["dataset_id"] for r in merged])},
            "sources": {"brain_table4_manifest": str(BRAIN_TABLE4_MANIFEST),
                        "brain_table4_manifest_sha256": EXPECTED_BRAIN_MANIFEST_SHA256,
                        "pva_manifest": str(PVA_MANIFEST), "pva_manifest_sha256": EXPECTED_PVA_MANIFEST_SHA256,
                        "archive_bindings": archive_bindings},
            "assets": merged,
        }
        manifest["manifest_content_sha256"] = canonical_sha256(manifest)
        (staging / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=True) + "\n", encoding="utf-8")
        (staging / "selected_pva.jsonl").write_text("".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in selected), encoding="utf-8")
        os.replace(staging, output)
    except BaseException:
        # Preserve staging evidence for diagnosis; a subsequent run must choose a new output or inspect it.
        raise
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
