#!/usr/bin/env python3
"""Create a selector-visible opaque snapshot from a mobility render build."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import stat
import tempfile
from pathlib import Path
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
HF_CACHE = Path("/root/.cache/huggingface").resolve()
DEFAULT_BUILD = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1_render_build"
DEFAULT_AMENDMENT = REPO / "exp/reference/table4_constraints_v2/amendment_partnet_mobility_clip_retrieval_v1.json"
DEFAULT_FORMAL_ROOT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1"
MODEL_REVISION = "3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
MODEL = HF_CACHE / "hub/models--openai--clip-vit-base-patch32/snapshots" / MODEL_REVISION
DEFAULT_OUTPUT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1_snapshot"
DEFAULT_CONTRACT = REPO / "exp/reference/table4_constraints_v2/selection_contract_partnet_mobility_clip_retrieval_v1.json"
DEFAULT_PROMPT_ONLY = REPO / "exp/reference/table4_constraints_v2/prompt_only_table4_constraints_v2.jsonl"
MODEL_FILES = ("config.json", "merges.txt", "preprocessor_config.json", "pytorch_model.bin", "special_tokens_map.json", "tokenizer.json", "tokenizer_config.json", "vocab.json")


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if not any(resolved == root or root in resolved.parents for root in (WORKSPACE, HF_CACHE)):
        raise RuntimeError(f"outside authorized roots: {resolved}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with safe(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(safe(path).read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in safe(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def write_json(path: Path, value: Any) -> None:
    destination = safe(path, must_exist=False)
    destination.write_text(json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n", encoding="utf-8")


def configured_path(value: str) -> Path:
    path = Path(value)
    return safe(path if path.is_absolute() else REPO / path, must_exist=False)


def lock(build: Path, amendment_path: Path, formal_root: Path, contract_path: Path, prompt_only_path: Path, output: Path, expected_assets: int) -> None:
    build = safe(build)
    amendment_path = safe(amendment_path)
    formal_root = safe(formal_root)
    contract_path = safe(contract_path)
    prompt_only_path = safe(prompt_only_path)
    output = safe(output, must_exist=False)
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    build_lock = read_json(build / "render_build.lock.json")
    amendment = read_json(amendment_path)
    shared = amendment.get("shared_pipeline", {})
    expected_configuration = {
        "render_build": build,
        "opaque_snapshot": output,
        "formal_root": formal_root,
    }
    for field, observed in expected_configuration.items():
        if configured_path(shared.get(field, "")) != observed:
            raise RuntimeError(f"amendment shared pipeline path drifted: {field}")
    if shared.get("expected_assets") != expected_assets or configured_path(shared.get("snapshot_locker", "")) != safe(SCRIPT):
        raise RuntimeError("amendment shared snapshot-locker configuration drifted")
    if configured_path(shared.get("selection_contract", "")) != contract_path or shared.get("selection_contract_sha256") != sha256_file(contract_path):
        raise RuntimeError("amendment selection-contract binding drifted")
    contract = read_json(contract_path)
    if contract.get("expected_assets") != expected_assets or contract.get("prompt_only_manifest_sha256") != sha256_file(prompt_only_path):
        raise RuntimeError("selection contract/prompt-only binding drifted")
    audit = read_json(formal_root / "protocol_audit_pre_result.json")
    if amendment.get("implementation", {}).get("snapshot_locker_sha256") != sha256_file(SCRIPT):
        raise RuntimeError("amendment snapshot-locker binding drifted")
    if audit.get("protocol_ready") is not True or audit.get("amendment_sha256") != sha256_file(amendment_path) or audit.get("snapshot_locker_sha256") != sha256_file(SCRIPT):
        raise RuntimeError("independent pre-result snapshot-locker audit binding drifted")
    inventory = read_jsonl(build / "candidate_inventory.jsonl")
    if build_lock.get("status") != "PASS" or build_lock.get("formal") is not True:
        raise RuntimeError("render build is not formal PASS")
    if len(inventory) != expected_assets or build_lock.get("candidate_inventory_sha256") != sha256_file(build / "candidate_inventory.jsonl"):
        raise RuntimeError("render inventory closure drifted")
    allowed_fields = {"candidate_key", "identity_sha256", "snapshot_render_dir", "eligible", "views", "render_sha256"}
    if any(set(row) != allowed_fields for row in inventory):
        raise RuntimeError("selector inventory contains forbidden fields")
    keys: set[str] = set()
    for row in inventory:
        key = row["candidate_key"]
        if row["identity_sha256"] != key or not isinstance(key, str) or len(key) != 64 or any(char not in "0123456789abcdef" for char in key):
            raise RuntimeError("invalid opaque candidate identity")
        if key in keys:
            raise RuntimeError(f"duplicate opaque candidate key: {key}")
        keys.add(key)
        if row["snapshot_render_dir"] != f"renders/{key}" or row["eligible"] is not True or row["views"] != 8 or set(row["render_sha256"]) != {f"{view:03d}.png" for view in range(8)}:
            raise RuntimeError(f"opaque candidate render schema drifted: {key}")
    run1 = safe(build / "run1")
    if run1.is_symlink() or not stat.S_ISDIR(run1.lstat().st_mode):
        raise RuntimeError("render source root is not a regular non-symlink directory")
    observed_render_dirs: set[str] = set()
    for entry in os.scandir(run1):
        path = Path(entry.path)
        if entry.is_symlink() or not entry.is_dir(follow_symlinks=False):
            raise RuntimeError(f"render source root contains non-directory: {path}")
        observed_render_dirs.add(entry.name)
    if observed_render_dirs != keys:
        raise RuntimeError(f"render directory exact closure mismatch missing={sorted(keys - observed_render_dirs)} extra={sorted(observed_render_dirs - keys)}")
    staging = Path(tempfile.mkdtemp(prefix=output.name + ".staging.", dir=str(output.parent)))
    try:
        renders = staging / "renders"
        renders.mkdir()
        for index, row in enumerate(inventory, 1):
            key = row["candidate_key"]
            source = safe(build / "run1" / key)
            destination = renders / key
            destination.mkdir()
            for view in range(8):
                name = f"{view:03d}.png"
                source_image = safe(source / name)
                if source_image.is_symlink() or not stat.S_ISREG(source_image.lstat().st_mode):
                    raise RuntimeError(f"source render is not regular: {source_image}")
                shutil.copyfile(source_image, destination / name)
                if sha256_file(destination / name) != row["render_sha256"][name]:
                    raise RuntimeError(f"snapshot render copy mismatch: {key}/{name}")
            if index % 100 == 0:
                print(f"[snapshot-lock] {index}/{len(inventory)}", flush=True)
        shutil.copyfile(build / "candidate_inventory.jsonl", staging / "candidate_inventory.jsonl")
        shutil.copyfile(contract_path, staging / "selection.execution_contract.json")
        shutil.copyfile(prompt_only_path, staging / "prompt_only.jsonl")
        model_dir = staging / "model"
        model_dir.mkdir()
        model_hashes = {}
        for name in MODEL_FILES:
            shutil.copyfile(safe(MODEL / name), model_dir / name)
            model_hashes[name] = sha256_file(model_dir / name)
        snapshot_lock = {
            "schema_version": 1,
            "status": "PASS",
            "phase": "selector_snapshot_locked_after_prompt_independent_render_build; contains exact prompt-only text but no full benchmark rows, specs, protocol contents, source metadata, or labels",
            "candidate_assets": len(inventory),
            "eligible_assets": len(inventory),
            "views_per_asset": 8,
            "candidate_inventory_sha256": sha256_file(staging / "candidate_inventory.jsonl"),
            "render_build_lock_sha256": sha256_file(build / "render_build.lock.json"),
            "private_source_audit_lock_sha256": build_lock["private_source_audit_lock_sha256"],
            "source_binding_sha256": build_lock["source_binding_sha256"],
            "render_worker_sha256": build_lock["render_worker_sha256"],
            "render_snapshot_builder_sha256": build_lock["snapshot_builder_sha256"],
            "snapshot_locker_sha256": sha256_file(SCRIPT),
            "fresh_render_runs": 2,
            "run_workers": build_lock["run_workers"],
            "cross_worker_full_render_replay_byte_identical": True,
            "runtime_fingerprint": build_lock["runtime_fingerprint"],
            "model_id": "openai/clip-vit-base-patch32",
            "model_revision": MODEL_REVISION,
            "model_file_sha256": model_hashes,
            "selection_execution_contract_sha256": sha256_file(staging / "selection.execution_contract.json"),
            "prompt_only_manifest_sha256": sha256_file(staging / "prompt_only.jsonl"),
            "selector_visible_closure": ["candidate_inventory.jsonl", "model", "prompt_only.jsonl", "renders", "selection.execution_contract.json", "snapshot.lock.json"],
            "forbidden_from_snapshot": ["source asset id/path", "URDF", "geometry", "meta.json", "semantics.txt", "category", "part names", "dimensions", "full benchmark rows", "protocol contents", "specs", "prior scores"],
        }
        write_json(staging / "snapshot.lock.json", snapshot_lock)
        staging.replace(output)
        print(json.dumps({"status": "OPAQUE_SNAPSHOT_LOCKED", "assets": len(inventory), "lock_sha256": sha256_file(output / "snapshot.lock.json")}, indent=2), flush=True)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render-build", type=Path, default=DEFAULT_BUILD)
    parser.add_argument("--amendment", type=Path, default=DEFAULT_AMENDMENT)
    parser.add_argument("--formal-root", type=Path, default=DEFAULT_FORMAL_ROOT)
    parser.add_argument("--selection-contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--prompt-only", type=Path, default=DEFAULT_PROMPT_ONLY)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--expected-assets", type=int, default=2347)
    args = parser.parse_args()
    if args.expected_assets <= 0:
        raise ValueError("expected assets must be positive")
    lock(args.render_build, args.amendment, args.formal_root, args.selection_contract, args.prompt_only, args.output_dir, args.expected_assets)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
