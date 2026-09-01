#!/usr/bin/env python3
"""Build a render-only Artiverse snapshot after full release integrity checks."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import stat
from pathlib import Path
from typing import Any

from PIL import Image


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
HF_CACHE = Path("/root/.cache/huggingface").resolve()
REFERENCE = REPO / "exp/reference/table4_constraints_v2"
PROMPTS = REFERENCE / "prompts.jsonl"
PROTOCOL = REFERENCE / "protocol.json"
AMENDMENT = REFERENCE / "amendment_artiverse_clip_retrieval_v1.json"
ADDENDUM1 = REFERENCE / "amendment_artiverse_clip_retrieval_v1_addendum1.json"
ADDENDUM2 = REFERENCE / "amendment_artiverse_clip_retrieval_v1_addendum2.json"
ADDENDUM = REFERENCE / "amendment_artiverse_clip_retrieval_v1_addendum3.json"
ARTIVERSE = REPO / "exp/artiverse"
DATA = ARTIVERSE / "data"
DATASET_MANIFEST = ARTIVERSE / "dataset_chunks/manifest.json"
SELECTOR = REPO / "exp/scripts/run_artiverse_clip_retrieval_selection.py"
MATERIALIZER = REPO / "exp/scripts/run_artiverse_clip_retrieval_materialize.py"
MODEL_REVISION = "3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
MODEL_SNAPSHOT = HF_CACHE / "hub/models--openai--clip-vit-base-patch32/snapshots" / MODEL_REVISION
DEFAULT_OUTPUT = REPO / "exp/runtime/table4_constraints_v2/artiverse_clip_retrieval_v1_snapshot"

EXPECTED_AMENDMENT_SHA256 = "ded01ce6b663559ea64955d5a69100894d7baacc304579131692accb02042fce"
EXPECTED_ADDENDUM1_SHA256 = "0bcc779eabac90efbfa0f7df0926b650d349951081d531d561d3d807d184a46f"
EXPECTED_ADDENDUM2_SHA256 = "0e85e30b456b85ddd06df56099458e1fe4e65a2f0b6ef426bd9a60ca0717d76c"
EXPECTED_PROMPTS_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
EXPECTED_PROTOCOL_SHA256 = "6857194072ccb0ba3943d14a62f29c2364920ec01381a5475311535c1831031f"
EXPECTED_MANIFEST_SHA256 = "8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586"
EXPECTED_README_SHA256 = "2033582ef71f0b12bb15ca2a61d56edcf07414db5e7c3fe5437f6132ec4fea73"
EXPECTED_ARCHIVES = {
    "artiverse_data-00001-of-00002.tar.gz": {
        "bytes": 38163580631,
        "sha256": "695d2d602faafab922ce66359ea104d81505f5b0fdee8f461d8905f0ccb4ef3b",
    },
    "artiverse_data-00002-of-00002.tar.gz": {
        "bytes": 27170560473,
        "sha256": "56dffa50f1c8c20d3b1eef626046805a6c7cd997141e8ab5fac9ebdae8ffab81",
    },
}
EXPECTED_MODEL_FILES = {
    "config.json": "b575ef3c36f2a057fa19e221650105052d61cc9c1a972ec15019c6261ec98770",
    "merges.txt": "f526393189112391ce6f9795d4695f704121ce452c3aad1f5335cc41337eba85",
    "preprocessor_config.json": "910e70b3956ac9879ebc90b22fb3bc8a75b6a0677814500101a4c072bd7857bd",
    "pytorch_model.bin": "a63082132ba4f97a80bea76823f544493bffa8082296d62d71581a4feff1576f",
    "special_tokens_map.json": "f8c0d6c39aee3f8431078ef6646567b0aba7f2246e9c54b8b99d55c22b707cbf",
    "tokenizer.json": "b556ac8c99757ffb677208af34bc8c6721572114111a6e0aaf5fa69ff0b8d842",
    "tokenizer_config.json": "34b7336e4bee12e0a9730eaf5189f582ef3c3eea5027f65730e5717256755aad",
    "vocab.json": "5047b556ce86ccaf6aa22b3ffccfc52d391ea4accdab9c2f2407da5b742d4363",
}
REQUIRED_VIEWS = tuple(f"{index:03d}.png" for index in range(16))


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    roots = (WORKSPACE, HF_CACHE)
    if not any(resolved == root or root in resolved.parents for root in roots):
        raise RuntimeError(f"path outside authorized roots: {resolved}")
    return resolved


def sha256_file(path: Path, block_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with safe(path).open("rb") as stream:
        for block in iter(lambda: stream.read(block_size), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(safe(path).read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    for line_number, line in enumerate(safe(path).read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = json.loads(line)
        if not isinstance(row, dict):
            raise RuntimeError(f"JSONL line {line_number} is not an object: {path}")
        rows.append(row)
    return rows


def write_json(path: Path, value: Any) -> None:
    destination = safe(path, must_exist=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    destination = safe(path, must_exist=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def require_regular_file(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
        raise RuntimeError(f"required regular non-symlink file missing: {path}")
    return resolved


def require_regular_dir(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISDIR(path.lstat().st_mode):
        raise RuntimeError(f"required regular non-symlink directory missing: {path}")
    return resolved


def require_cache_file(path: Path) -> Path:
    resolved = safe(path)
    if not resolved.is_file() or HF_CACHE not in resolved.parents:
        raise RuntimeError(f"pinned model file does not resolve inside HF cache: {path}")
    return resolved


def expected_roots(manifest: dict[str, Any]) -> list[str]:
    roots = [str(root) for chunk in manifest["chunks"] for root in chunk["roots"]]
    if len(roots) != 3544 or len(set(roots)) != 3544:
        raise RuntimeError("dataset manifest must contain 3544 unique model roots")
    for identity in roots:
        parts = Path(identity).parts
        if len(parts) != 4 or parts[0] != "data" or ".." in parts:
            raise RuntimeError(f"unsafe model identity: {identity}")
    return sorted(roots)


def full_data_gate(expected: list[str], manifest: dict[str, Any]) -> dict[str, Any]:
    root = require_regular_dir(DATA)
    actual_roots = []
    files = 0
    total_bytes = 0
    symlinks = 0
    for directory, directory_names, file_names in os.walk(root, followlinks=False):
        current = safe(Path(directory))
        relative_parts = current.relative_to(ARTIVERSE).parts
        for name in list(directory_names):
            candidate = current / name
            if candidate.is_symlink():
                symlinks += 1
                directory_names.remove(name)
        directory_names.sort()
        if len(relative_parts) == 4:
            actual_roots.append(current.relative_to(ARTIVERSE).as_posix())
        for name in sorted(file_names):
            candidate = current / name
            if candidate.is_symlink():
                symlinks += 1
                continue
            resolved = safe(candidate)
            info = resolved.stat()
            if stat.S_ISREG(info.st_mode):
                files += 1
                total_bytes += info.st_size
    actual_roots.sort()
    payload = {
        "expected_model_roots": len(expected),
        "actual_model_roots": len(actual_roots),
        "root_set_exact": actual_roots == expected,
        "missing_roots": sorted(set(expected) - set(actual_roots)),
        "extra_roots": sorted(set(actual_roots) - set(expected)),
        "expected_file_count": manifest["file_count"],
        "actual_file_count": files,
        "expected_input_bytes": manifest["input_bytes"],
        "actual_input_bytes": total_bytes,
        "symlink_count": symlinks,
    }
    payload["passed"] = (
        payload["root_set_exact"]
        and files == 531937
        and total_bytes == 86992752890
        and symlinks == 0
    )
    if not payload["passed"]:
        raise RuntimeError(
            "full Artiverse data gate failed: "
            f"roots={len(actual_roots)}/3544 files={files}/531937 "
            f"bytes={total_bytes}/86992752890 symlinks={symlinks}"
        )
    return payload


def archive_gate(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    observed_entries = {entry["archive"]: entry for entry in manifest["chunks"]}
    if set(observed_entries) != set(EXPECTED_ARCHIVES):
        raise RuntimeError("dataset archive set drifted")
    records = []
    for name in sorted(EXPECTED_ARCHIVES):
        expected = EXPECTED_ARCHIVES[name]
        entry = observed_entries[name]
        if entry.get("archive_bytes") != expected["bytes"] or entry.get("sha256") != expected["sha256"]:
            raise RuntimeError(f"manifest archive record drifted: {name}")
        path = require_regular_file(ARTIVERSE / "dataset_chunks" / name)
        size = path.stat().st_size
        print(f"[snapshot-preflight] hashing {name} ({size} bytes)", flush=True)
        actual = sha256_file(path)
        record = {
            "archive": name,
            "bytes": size,
            "expected_bytes": expected["bytes"],
            "sha256": actual,
            "expected_sha256": expected["sha256"],
            "passed": size == expected["bytes"] and actual == expected["sha256"],
        }
        if not record["passed"]:
            raise RuntimeError(f"archive integrity failed: {name}")
        records.append(record)
    return records


def validate_code_and_addendum() -> dict[str, Any]:
    if sha256_file(require_regular_file(AMENDMENT)) != EXPECTED_AMENDMENT_SHA256:
        raise RuntimeError("original amendment hash drifted")
    if sha256_file(require_regular_file(ADDENDUM1)) != EXPECTED_ADDENDUM1_SHA256:
        raise RuntimeError("first addendum hash drifted")
    if sha256_file(require_regular_file(ADDENDUM2)) != EXPECTED_ADDENDUM2_SHA256:
        raise RuntimeError("second addendum hash drifted")
    addendum = read_json(require_regular_file(ADDENDUM))
    hashes = {
        "snapshot_preflight_script_sha256": sha256_file(require_regular_file(SCRIPT)),
        "selector_sha256": sha256_file(require_regular_file(SELECTOR)),
        "materializer_sha256": sha256_file(require_regular_file(MATERIALIZER)),
    }
    if addendum.get("original_amendment_sha256") != EXPECTED_AMENDMENT_SHA256:
        raise RuntimeError("addendum does not bind the original amendment")
    if addendum.get("previous_addendum_sha256") != EXPECTED_ADDENDUM2_SHA256:
        raise RuntimeError("effective addendum does not bind the second addendum")
    expected_prior_chain = [
        {
            "path": "exp/reference/table4_constraints_v2/amendment_artiverse_clip_retrieval_v1.json",
            "sha256": EXPECTED_AMENDMENT_SHA256,
        },
        {
            "path": "exp/reference/table4_constraints_v2/amendment_artiverse_clip_retrieval_v1_addendum1.json",
            "sha256": EXPECTED_ADDENDUM1_SHA256,
        },
        {
            "path": "exp/reference/table4_constraints_v2/amendment_artiverse_clip_retrieval_v1_addendum2.json",
            "sha256": EXPECTED_ADDENDUM2_SHA256,
        },
    ]
    if addendum.get("prior_chain") != expected_prior_chain:
        raise RuntimeError("effective addendum does not bind the complete prior chain")
    for field, actual in hashes.items():
        if addendum.get("implementation", {}).get(field) != actual:
            raise RuntimeError(f"addendum implementation hash mismatch: {field}")
    return {
        "original_amendment_sha256": EXPECTED_AMENDMENT_SHA256,
        "addendum1_sha256": EXPECTED_ADDENDUM1_SHA256,
        "addendum2_sha256": EXPECTED_ADDENDUM2_SHA256,
        "previous_addendum_sha256": EXPECTED_ADDENDUM2_SHA256,
        "addendum_sha256": sha256_file(ADDENDUM),
        **hashes,
    }


def inspect_candidate(identity: str, render_root: Path) -> dict[str, Any]:
    source_root = ARTIVERSE / identity
    identity_hash = sha256_text(identity)
    source_images = source_root / "imgs"
    reasons: list[str] = []
    views: dict[str, dict[str, Any]] = {}
    try:
        require_regular_dir(source_root)
        require_regular_dir(source_images)
        png_names = sorted(
            entry.name for entry in os.scandir(safe(source_images))
            if entry.name.lower().endswith(".png")
        )
        if png_names != list(REQUIRED_VIEWS):
            reasons.append(f"png_set_mismatch:{png_names}")
        for name in REQUIRED_VIEWS:
            source = require_regular_file(source_images / name)
            with Image.open(source) as image:
                rgb = image.convert("RGB")
                rgb.load()
                size = [rgb.width, rgb.height]
            views[name] = {
                "sha256": sha256_file(source),
                "bytes": source.stat().st_size,
                "size_px": size,
            }
    except Exception as exc:
        reasons.append(f"{type(exc).__name__}:{exc}")
    eligible = not reasons
    snapshot_relative = f"renders/{identity_hash}"
    if eligible:
        destination_dir = safe(render_root / identity_hash, must_exist=False)
        destination_dir.mkdir(parents=True, exist_ok=False)
        for name in REQUIRED_VIEWS:
            source = require_regular_file(source_images / name)
            destination = safe(destination_dir / name, must_exist=False)
            shutil.copyfile(source, destination)
            if sha256_file(destination) != views[name]["sha256"]:
                raise RuntimeError(f"snapshot copy hash mismatch: {identity}/{name}")
            destination.chmod(0o444)
    return {
        "identity": identity,
        "identity_sha256": identity_hash,
        "eligible": eligible,
        "ineligible_reasons": reasons,
        "snapshot_render_dir": snapshot_relative if eligible else None,
        "required_view_count": 16,
        "views": views,
    }


def copy_model(staging: Path) -> dict[str, str]:
    require_regular_dir(MODEL_SNAPSHOT)
    destination = safe(staging / "model", must_exist=False)
    destination.mkdir(parents=True, exist_ok=False)
    hashes = {}
    for name, expected in EXPECTED_MODEL_FILES.items():
        source = require_cache_file(MODEL_SNAPSHOT / name)
        if sha256_file(source) != expected:
            raise RuntimeError(f"pinned HF model hash mismatch: {name}")
        target = safe(destination / name, must_exist=False)
        shutil.copyfile(source, target)
        target.chmod(0o444)
        actual = sha256_file(target)
        if actual != expected:
            raise RuntimeError(f"snapshot model copy hash mismatch: {name}")
        hashes[name] = actual
    return hashes


def build_snapshot(output: Path) -> None:
    output = safe(output, must_exist=False)
    staging = safe(output.with_name(output.name + ".building"), must_exist=False)
    if output.exists() or staging.exists():
        raise RuntimeError(f"snapshot output or staging path already exists: {output}, {staging}")
    output.parent.mkdir(parents=True, exist_ok=True)
    fixed = {
        str(PROMPTS): sha256_file(require_regular_file(PROMPTS)),
        str(PROTOCOL): sha256_file(require_regular_file(PROTOCOL)),
        str(DATASET_MANIFEST): sha256_file(require_regular_file(DATASET_MANIFEST)),
        str(ARTIVERSE / "README.md"): sha256_file(require_regular_file(ARTIVERSE / "README.md")),
    }
    expected_fixed = {
        str(PROMPTS): EXPECTED_PROMPTS_SHA256,
        str(PROTOCOL): EXPECTED_PROTOCOL_SHA256,
        str(DATASET_MANIFEST): EXPECTED_MANIFEST_SHA256,
        str(ARTIVERSE / "README.md"): EXPECTED_README_SHA256,
    }
    if fixed != expected_fixed:
        raise RuntimeError(f"frozen source hash drift: {fixed}")
    code = validate_code_and_addendum()
    manifest = read_json(DATASET_MANIFEST)
    if manifest.get("model_count") != 3544 or manifest.get("file_count") != 531937:
        raise RuntimeError("dataset manifest count drifted")
    if manifest.get("input_bytes") != 86992752890:
        raise RuntimeError("dataset manifest byte count drifted")
    roots = expected_roots(manifest)
    archives = archive_gate(manifest)
    print("[snapshot-preflight] scanning the complete extracted tree", flush=True)
    data_gate = full_data_gate(roots, manifest)
    staging.mkdir(parents=False, exist_ok=False)
    renders = safe(staging / "renders", must_exist=False)
    renders.mkdir(parents=False, exist_ok=False)
    shutil.copyfile(PROMPTS, staging / "prompts.jsonl")
    (staging / "prompts.jsonl").chmod(0o444)
    inventory = []
    for index, identity in enumerate(roots, 1):
        inventory.append(inspect_candidate(identity, renders))
        if index % 100 == 0 or index == len(roots):
            print(
                f"[snapshot] {index}/3544 candidates; eligible="
                f"{sum(row['eligible'] for row in inventory)}",
                flush=True,
            )
    write_jsonl(staging / "candidate_inventory.jsonl", inventory)
    eligible = [row for row in inventory if row["eligible"]]
    summary = {
        "candidate_assets": len(inventory),
        "eligible_assets": len(eligible),
        "ineligible_assets": len(inventory) - len(eligible),
        "eligibility_rate": len(eligible) / len(inventory),
        "eligible_render_files": len(eligible) * 16,
        "eligibility_rule": "exactly regular non-symlink 000.png through 015.png and all RGB-decodable",
    }
    write_json(staging / "candidate_summary.json", summary)
    model_hashes = copy_model(staging)
    snapshot_files = {
        "prompts.jsonl": sha256_file(staging / "prompts.jsonl"),
        "candidate_inventory.jsonl": sha256_file(staging / "candidate_inventory.jsonl"),
        "candidate_summary.json": sha256_file(staging / "candidate_summary.json"),
    }
    lock = {
        "schema_version": 1,
        "status": "PASS",
        "snapshot_type": "render_only_identity_snapshot",
        "is_os_sandbox": False,
        "isolation_claim": "workspace-local read-only copies plus code-audited explicit inputs; not an OS sandbox",
        "full_source_gate": data_gate,
        "archives": archives,
        "dataset_manifest_sha256": EXPECTED_MANIFEST_SHA256,
        "prompt_manifest_sha256": EXPECTED_PROMPTS_SHA256,
        "candidate_assets": len(inventory),
        "eligible_assets": len(eligible),
        "ineligible_assets": len(inventory) - len(eligible),
        "eligible_render_files": len(eligible) * 16,
        "snapshot_files_sha256": snapshot_files,
        "model_revision": MODEL_REVISION,
        "model_files_sha256": model_hashes,
        "implementation": code,
        "repair_attempts": 0,
    }
    write_json(staging / "snapshot.lock.json", lock)
    for path in (staging / "candidate_inventory.jsonl", staging / "candidate_summary.json", staging / "snapshot.lock.json"):
        path.chmod(0o444)
    for directory, directory_names, _ in os.walk(staging, topdown=False):
        for name in directory_names:
            (Path(directory) / name).chmod(0o555)
    staging.chmod(0o555)
    staging.replace(output)
    print(json.dumps({
        "status": "SNAPSHOT_LOCKED",
        "candidate_assets": len(inventory),
        "eligible_assets": len(eligible),
        "snapshot": str(output),
        "snapshot_lock_sha256": sha256_file(output / "snapshot.lock.json"),
    }, indent=2), flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    build_snapshot(parse_args().output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
