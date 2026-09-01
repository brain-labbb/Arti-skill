#!/usr/bin/env python3
"""Materialize and score a locked Artiverse CLIP retrieval selection."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
REFERENCE = REPO / "exp/reference/table4_constraints_v2"
PROMPTS = REFERENCE / "prompts.jsonl"
AMENDMENT = REFERENCE / "amendment_artiverse_clip_retrieval_v1.json"
ADDENDUM1 = REFERENCE / "amendment_artiverse_clip_retrieval_v1_addendum1.json"
ADDENDUM2 = REFERENCE / "amendment_artiverse_clip_retrieval_v1_addendum2.json"
ADDENDUM = REFERENCE / "amendment_artiverse_clip_retrieval_v1_addendum3.json"
ARTIVERSE = REPO / "exp/artiverse"
CANONICALIZER = REPO / "exp/scripts/canonicalize_table4_artifact.py"
SCORER = REPO / "exp/scripts/score_table4_constraints_v2.py"
INTEGRITY = REPO / "exp/scripts/verify_table4_constraints_v2_integrity.py"
SNAPSHOT_BUILDER = REPO / "exp/scripts/prepare_artiverse_clip_retrieval_snapshot.py"
DEFAULT_ROOT = REPO / "exp/runtime/table4_constraints_v2/artiverse_clip_retrieval_v1"
METHOD = "artiverse_clip_retrieval_v1"

EXPECTED_AMENDMENT_SHA256 = "ded01ce6b663559ea64955d5a69100894d7baacc304579131692accb02042fce"
EXPECTED_ADDENDUM1_SHA256 = "0bcc779eabac90efbfa0f7df0926b650d349951081d531d561d3d807d184a46f"
EXPECTED_ADDENDUM2_SHA256 = "0e85e30b456b85ddd06df56099458e1fe4e65a2f0b6ef426bd9a60ca0717d76c"
EXPECTED_PROMPTS_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
EXPECTED_DATASET_MANIFEST_SHA256 = "8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586"
EXPECTED_SELECTION_SCRIPT = REPO / "exp/scripts/run_artiverse_clip_retrieval_selection.py"
EXPECTED_SELECTION_SCRIPT_SHA256 = "5248cf927777e6bb6862713a5d7e854c9b4f193742963d870ccf384ab3448db5"
EXPECTED_SNAPSHOT_BUILDER_SHA256 = "a5bd9d8e531389eda6aa2e58d9ee4b9fd1775678cdd96aba1e9b20a3732815e9"
FORMAL_BATCH_ASSETS = 8
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


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
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


def atomic_text(path: Path, value: str) -> None:
    destination = safe(path, must_exist=False)
    safe(destination.parent).mkdir(parents=True, exist_ok=True)
    temporary = safe(destination.with_suffix(destination.suffix + ".tmp"), must_exist=False)
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(destination)


def write_json(path: Path, value: Any) -> None:
    atomic_text(path, json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n")


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    atomic_text(
        path,
        "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows),
    )


def require_regular_file(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
        raise RuntimeError(f"required regular non-symlink file missing: {path}")
    return resolved


def validate_selection(root: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    root = safe(root)
    if sha256_file(require_regular_file(AMENDMENT)) != EXPECTED_AMENDMENT_SHA256:
        raise RuntimeError("Artiverse retrieval amendment hash drifted")
    if sha256_file(require_regular_file(PROMPTS)) != EXPECTED_PROMPTS_SHA256:
        raise RuntimeError("frozen prompts hash drifted")
    lock_path = root / "selection.lock.json"
    lock = read_json(require_regular_file(lock_path))
    if lock.get("method") != METHOD or lock.get("phase") != "selection_locked_before_geometry_access":
        raise RuntimeError("invalid selection lock identity or phase")
    if lock.get("repair_attempts") != 0 or lock.get("task_count") != 18:
        raise RuntimeError("selection lock repair/task policy drifted")
    if lock.get("original_amendment_sha256") != EXPECTED_AMENDMENT_SHA256:
        raise RuntimeError("selection lock amendment hash mismatch")
    if lock.get("prompt_manifest_sha256") != EXPECTED_PROMPTS_SHA256:
        raise RuntimeError("selection lock prompt hash mismatch")
    actual_selector_hash = sha256_file(require_regular_file(EXPECTED_SELECTION_SCRIPT))
    if actual_selector_hash != EXPECTED_SELECTION_SCRIPT_SHA256:
        raise RuntimeError("selector source changed after independent protocol audit")
    if lock.get("selection_script_sha256") != EXPECTED_SELECTION_SCRIPT_SHA256:
        raise RuntimeError("selection runner changed after selection lock")
    if sha256_file(require_regular_file(SNAPSHOT_BUILDER)) != EXPECTED_SNAPSHOT_BUILDER_SHA256:
        raise RuntimeError("snapshot builder source changed after independent protocol audit")
    if lock.get("snapshot_preflight_script_sha256") != EXPECTED_SNAPSHOT_BUILDER_SHA256:
        raise RuntimeError("selection lock snapshot-builder binding mismatch")
    if lock.get("materializer_sha256") != sha256_file(SCRIPT):
        raise RuntimeError("selection lock materializer binding mismatch")
    if lock.get("dataset_manifest_sha256") != EXPECTED_DATASET_MANIFEST_SHA256:
        raise RuntimeError("selection lock dataset manifest binding mismatch")
    if lock.get("original_amendment_sha256") != EXPECTED_AMENDMENT_SHA256:
        raise RuntimeError("selection lock original amendment binding mismatch")
    if lock.get("formal_batch_assets") != FORMAL_BATCH_ASSETS:
        raise RuntimeError("selection lock formal batch size drifted")
    if lock.get("candidate_assets") != 3544 or lock.get("task_count") != 18:
        raise RuntimeError("selection lock candidate/task count drifted")
    if lock.get("full_embedding_replays") != 2 or lock.get("full_embedding_replay_byte_identical") is not True:
        raise RuntimeError("selection lock lacks two byte-identical full embedding runs")
    if lock.get("geometry_access_during_selection") is not False or lock.get("is_os_sandbox") is not False:
        raise RuntimeError("selection isolation claim drifted")
    for name, expected in lock.get("locked_file_sha256", {}).items():
        if Path(name).name != name or name in {"selection.lock.json", "materialization.lock.json"}:
            raise RuntimeError(f"unsafe locked filename: {name}")
        actual = sha256_file(require_regular_file(root / name))
        if actual != expected:
            raise RuntimeError(f"selection locked-file hash mismatch: {name}")
    required_locked = {
        "protocol_audit_pre_result.json",
        "report.md",
        "selection_preflight.json",
        "asset_embeddings_run1.npy",
        "asset_embeddings_run2.npy",
        "prompt_embeddings_run1.npy",
        "prompt_embeddings_run2.npy",
        "embedding_index.jsonl",
        "prompt_tokenization.jsonl",
        "selection.jsonl",
        "selection_replay.jsonl",
    }
    if set(lock.get("locked_file_sha256", {})) != required_locked:
        raise RuntimeError("selection locked-file set is incomplete or unexpected")
    preflight = read_json(root / "selection_preflight.json")
    if preflight.get("status") != "PASS" or preflight.get("candidate_assets") != 3544:
        raise RuntimeError("selection preflight was not a 3544-candidate PASS")
    if preflight.get("formal_batch_assets") != FORMAL_BATCH_ASSETS:
        raise RuntimeError("selection preflight batch size drifted")
    if preflight.get("full_embedding_replays") != 2 or preflight.get("full_embedding_replay_byte_identical") is not True:
        raise RuntimeError("selection preflight replay evidence drifted")
    if preflight.get("selection_script_sha256") != EXPECTED_SELECTION_SCRIPT_SHA256:
        raise RuntimeError("selection preflight selector binding drifted")
    if preflight.get("snapshot_preflight_script_sha256") != EXPECTED_SNAPSHOT_BUILDER_SHA256:
        raise RuntimeError("selection preflight snapshot-builder binding drifted")
    if preflight.get("materializer_sha256") != sha256_file(SCRIPT):
        raise RuntimeError("selection preflight materializer binding drifted")
    if preflight.get("original_amendment_sha256") != EXPECTED_AMENDMENT_SHA256:
        raise RuntimeError("selection preflight original amendment binding drifted")
    if preflight.get("addendum_sha256") != lock.get("addendum_sha256"):
        raise RuntimeError("selection preflight/addendum binding mismatch")
    if sha256_file(require_regular_file(ADDENDUM)) != lock.get("addendum_sha256"):
        raise RuntimeError("selection addendum hash mismatch")
    if sha256_file(require_regular_file(ADDENDUM1)) != EXPECTED_ADDENDUM1_SHA256:
        raise RuntimeError("first selection addendum hash mismatch")
    if sha256_file(require_regular_file(ADDENDUM2)) != EXPECTED_ADDENDUM2_SHA256:
        raise RuntimeError("second selection addendum hash mismatch")
    addendum = read_json(ADDENDUM)
    implementation = addendum.get("implementation", {})
    expected_implementation = {
        "snapshot_preflight_script_sha256": EXPECTED_SNAPSHOT_BUILDER_SHA256,
        "selector_sha256": EXPECTED_SELECTION_SCRIPT_SHA256,
        "materializer_sha256": sha256_file(SCRIPT),
    }
    for field, expected in expected_implementation.items():
        if implementation.get(field) != expected:
            raise RuntimeError(f"addendum implementation binding mismatch: {field}")
    if addendum.get("original_amendment_sha256") != EXPECTED_AMENDMENT_SHA256:
        raise RuntimeError("addendum original amendment binding mismatch")
    if addendum.get("previous_addendum_sha256") != EXPECTED_ADDENDUM2_SHA256:
        raise RuntimeError("addendum previous-addendum binding mismatch")
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
        raise RuntimeError("addendum complete prior-chain binding mismatch")
    if addendum.get("formal_selection", {}).get("batch_assets") != FORMAL_BATCH_ASSETS:
        raise RuntimeError("addendum formal batch size drifted")
    audit = read_json(root / "protocol_audit_pre_result.json")
    audit_verdict = str(audit.get("verdict") or audit.get("status") or "").upper()
    if audit_verdict != "PASS" or audit.get("protocol_ready") is not True:
        raise RuntimeError("pre-result independent protocol audit is not a ready PASS")
    audit_bindings = {
        "original_amendment_sha256": EXPECTED_AMENDMENT_SHA256,
        "addendum_sha256": lock.get("addendum_sha256"),
        "snapshot_preflight_script_sha256": EXPECTED_SNAPSHOT_BUILDER_SHA256,
        "selector_sha256": EXPECTED_SELECTION_SCRIPT_SHA256,
        "materializer_sha256": sha256_file(SCRIPT),
        "formal_batch_assets": FORMAL_BATCH_ASSETS,
        "report_sha256": sha256_file(root / "report.md"),
    }
    for field, expected in audit_bindings.items():
        if audit.get(field) != expected:
            raise RuntimeError(f"pre-result audit binding mismatch: {field}")
    snapshot_path = Path(preflight.get("snapshot") or "")
    snapshot_lock_path = safe(snapshot_path / "snapshot.lock.json")
    if sha256_file(snapshot_lock_path) != preflight.get("snapshot_lock_sha256"):
        raise RuntimeError("selection snapshot lock hash mismatch")
    if lock.get("snapshot_lock_sha256") != preflight.get("snapshot_lock_sha256"):
        raise RuntimeError("selection lock/preflight snapshot hash mismatch")
    snapshot_lock = read_json(snapshot_lock_path)
    if snapshot_lock.get("dataset_manifest_sha256") != EXPECTED_DATASET_MANIFEST_SHA256:
        raise RuntimeError("snapshot dataset manifest binding mismatch")
    if snapshot_lock.get("candidate_assets") != 3544:
        raise RuntimeError("snapshot candidate count drifted")
    if snapshot_lock.get("eligible_assets") != lock.get("eligible_assets"):
        raise RuntimeError("snapshot/selection eligible count mismatch")
    source_gate = snapshot_lock.get("full_source_gate", {})
    if not source_gate.get("passed") or source_gate.get("actual_model_roots") != 3544:
        raise RuntimeError("snapshot full-source gate is not a PASS")
    if source_gate.get("actual_file_count") != 531937 or source_gate.get("actual_input_bytes") != 86992752890:
        raise RuntimeError("snapshot full-source file/byte gate drifted")
    archives = snapshot_lock.get("archives")
    if not isinstance(archives, list) or len(archives) != 2 or not all(row.get("passed") for row in archives):
        raise RuntimeError("snapshot archive gate is incomplete or failed")
    archive_by_name = {row.get("archive"): row for row in archives}
    if set(archive_by_name) != set(EXPECTED_ARCHIVES):
        raise RuntimeError("snapshot archive set drifted")
    for name, expected in EXPECTED_ARCHIVES.items():
        row = archive_by_name[name]
        if (
            row.get("bytes") != expected["bytes"]
            or row.get("expected_bytes") != expected["bytes"]
            or row.get("sha256") != expected["sha256"]
            or row.get("expected_sha256") != expected["sha256"]
        ):
            raise RuntimeError(f"snapshot archive evidence drifted: {name}")
    snapshot_files = snapshot_lock.get("snapshot_files_sha256", {})
    expected_snapshot_files = {"prompts.jsonl", "candidate_inventory.jsonl", "candidate_summary.json"}
    if set(snapshot_files) != expected_snapshot_files:
        raise RuntimeError("snapshot locked-file set drifted")
    for name, expected in snapshot_files.items():
        if sha256_file(require_regular_file(snapshot_path / name)) != expected:
            raise RuntimeError(f"snapshot locked-file hash mismatch: {name}")
    snapshot_inventory = read_jsonl(snapshot_path / "candidate_inventory.jsonl")
    if len(snapshot_inventory) != 3544:
        raise RuntimeError("snapshot candidate inventory row count drifted")
    eligible_inventory = [row for row in snapshot_inventory if row.get("eligible") is True]
    if len(eligible_inventory) != snapshot_lock.get("eligible_assets"):
        raise RuntimeError("snapshot eligible inventory count drifted")
    index = read_jsonl(root / "embedding_index.jsonl")
    if any(row.get("row") != offset for offset, row in enumerate(index)):
        raise RuntimeError("embedding index rows are non-contiguous")
    if len(index) != lock.get("eligible_assets") or len(index) != preflight.get("eligible_assets"):
        raise RuntimeError("embedding index/eligible count mismatch")
    for row_number, (indexed, candidate) in enumerate(zip(index, eligible_inventory, strict=True)):
        if indexed.get("row") != row_number:
            raise RuntimeError("embedding index row order drifted")
        if indexed.get("identity") != candidate.get("identity"):
            raise RuntimeError("embedding index identity differs from snapshot eligible order")
        if indexed.get("identity_sha256") != candidate.get("identity_sha256"):
            raise RuntimeError("embedding index identity hash differs from snapshot inventory")
        if indexed.get("snapshot_render_dir") != candidate.get("snapshot_render_dir"):
            raise RuntimeError("embedding index render directory differs from snapshot inventory")
    asset_run1 = np.load(root / "asset_embeddings_run1.npy", allow_pickle=False)
    asset_run2 = np.load(root / "asset_embeddings_run2.npy", allow_pickle=False)
    prompt_run1 = np.load(root / "prompt_embeddings_run1.npy", allow_pickle=False)
    prompt_run2 = np.load(root / "prompt_embeddings_run2.npy", allow_pickle=False)
    if asset_run1.shape != (len(index), 512) or prompt_run1.shape != (18, 512):
        raise RuntimeError("locked embedding matrix shape mismatch")
    if not np.array_equal(asset_run1, asset_run2) or not np.array_equal(prompt_run1, prompt_run2):
        raise RuntimeError("locked full embedding replay differs")
    if not np.isfinite(asset_run1).all() or not np.isfinite(prompt_run1).all():
        raise RuntimeError("locked embeddings contain non-finite values")
    if not np.allclose(np.linalg.norm(asset_run1, axis=1), 1.0, atol=1e-12, rtol=1e-12):
        raise RuntimeError("locked asset embeddings are not unit normalized")
    if not np.allclose(np.linalg.norm(prompt_run1, axis=1), 1.0, atol=1e-12, rtol=1e-12):
        raise RuntimeError("locked prompt embeddings are not unit normalized")
    selection = read_jsonl(root / "selection.jsonl")
    selection_replay = read_jsonl(root / "selection_replay.jsonl")
    if selection_replay != selection:
        raise RuntimeError("selection replay differs from formal selection")
    prompts = read_jsonl(PROMPTS)
    prompt_by_task = {row["task_id"]: row for row in prompts}
    expected_tasks = {f"T4C{number:03d}" for number in range(1, 19)}
    if len(selection) != 18 or {row.get("task_id") for row in selection} != expected_tasks:
        raise RuntimeError("selection task set is incomplete or duplicated")
    selection_by_task = {row["task_id"]: row for row in selection}
    ordered_prompts = sorted(prompts, key=lambda row: row["task_id"])
    scores = prompt_run1 @ asset_run1.T
    for prompt_index, prompt_row in enumerate(ordered_prompts):
        task_id = prompt_row["task_id"]
        row = selection_by_task[task_id]
        task_id = row["task_id"]
        embedding_row = row.get("selected_embedding_row")
        if not isinstance(embedding_row, int) or not 0 <= embedding_row < len(index):
            raise RuntimeError(f"invalid selected embedding row: {task_id}")
        indexed = index[embedding_row]
        identity = row.get("selected_identity")
        if identity != indexed.get("identity"):
            raise RuntimeError(f"selected identity/index mismatch: {task_id}")
        if row.get("selected_identity_sha256") != sha256_text(identity):
            raise RuntimeError(f"selected identity hash mismatch: {task_id}")
        if indexed.get("identity_sha256") != row.get("selected_identity_sha256"):
            raise RuntimeError(f"embedding identity hash mismatch: {task_id}")
        expected_index = min(
            range(len(index)),
            key=lambda candidate_index: (
                -float(scores[prompt_index, candidate_index]),
                index[candidate_index]["identity_sha256"],
                index[candidate_index]["identity"],
            ),
        )
        if embedding_row != expected_index:
            raise RuntimeError(f"locked selection is not the deterministic global top-1: {task_id}")
        if row.get("clip_cosine_similarity") != float(scores[prompt_index, expected_index]):
            raise RuntimeError(f"locked selection score differs from recomputed score: {task_id}")
        prompt = prompt_by_task[task_id]["prompt"]
        if row.get("prompt_sha256") != sha256_text(prompt):
            raise RuntimeError(f"selection prompt hash mismatch: {task_id}")
        if row.get("fallback_allowed") is not False:
            raise RuntimeError(f"fallback policy drifted: {task_id}")
    return lock, sorted(selection, key=lambda row: row["task_id"])


def run_command(command: list[str], log_path: Path) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        command,
        cwd=safe(REPO),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        env={**os.environ, "PYTHONHASHSEED": "0"},
    )
    atomic_text(log_path, result.stdout)
    return result


def selected_glb(identity: str) -> Path:
    root = safe(ARTIVERSE / identity)
    candidates = []
    for candidate in root.glob("*.segmented.glb"):
        try:
            candidates.append(require_regular_file(candidate))
        except Exception:
            continue
    if len(candidates) != 1:
        raise RuntimeError(f"expected exactly one regular segmented GLB, found {len(candidates)}")
    return candidates[0]


def materialize(root: Path, selection: list[dict[str, Any]]) -> list[dict[str, Any]]:
    canonical_root = safe(root / "canonical", must_exist=False)
    if canonical_root.exists():
        raise RuntimeError(f"canonical output already exists; refusing overwrite: {canonical_root}")
    canonical_root.mkdir(parents=True, exist_ok=False)
    rows = []
    for record in selection:
        task_id = record["task_id"]
        destination = canonical_root / task_id
        row: dict[str, Any] = {
            "task_id": task_id,
            "method": METHOD,
            "status": "failure",
            "repair_attempts": 0,
            "selection_identity": record["selected_identity"],
            "selection_identity_sha256": record["selected_identity_sha256"],
            "clip_cosine_similarity": record["clip_cosine_similarity"],
            "fallback_used": False,
            "canonical_dir": str(destination),
        }
        try:
            source = selected_glb(record["selected_identity"])
            row["source_artifact"] = str(source)
            row["source_artifact_sha256"] = sha256_file(source)
            command = [
                sys.executable,
                str(require_regular_file(CANONICALIZER)),
                "--input",
                str(source),
                "--artifact-type",
                "glb",
                "--unit-scale-to-m",
                "1.0",
                "--output-dir",
                str(destination),
            ]
            result = run_command(command, canonical_root / f"{task_id}.canonicalize.log")
            if result.returncode != 0:
                raise RuntimeError(f"canonicalizer_exit_{result.returncode}")
            artifact = read_json(destination / "artifact.json")
            if artifact.get("source_sha256") != row["source_artifact_sha256"]:
                raise RuntimeError("canonical artifact source hash mismatch")
            if artifact.get("unit_scale_to_m") != 1.0:
                raise RuntimeError("canonical artifact unit scale drifted")
            row["status"] = "success"
            row["canonical_glb_sha256"] = artifact["canonical_glb_sha256"]
            row["semantic_node_count"] = artifact["semantic_node_count"]
            row["extents_m"] = artifact["extents_m"]
        except Exception as exc:
            row["failure_reason"] = f"{type(exc).__name__}:{exc}"
        rows.append(row)
        print(f"[materialize] {task_id}: {row['status']}", flush=True)
    write_jsonl(root / "artifact_manifest.jsonl", rows)
    return rows


def compare_score_runs(first: Path, second: Path) -> dict[str, str]:
    required = ("records.json", "summary.json", "report.md")
    hashes = {}
    for name in required:
        first_hash = sha256_file(require_regular_file(first / name))
        second_hash = sha256_file(require_regular_file(second / name))
        if first_hash != second_hash:
            raise RuntimeError(f"scorer replay mismatch: {first.name}/{name}")
        hashes[name] = first_hash
    return hashes


def score(root: Path, panel: str, label: str) -> tuple[dict[str, Any], dict[str, str]]:
    manifest = require_regular_file(root / "artifact_manifest.jsonl")
    directories = [root / f"score_{label}_run1", root / f"score_{label}_run2"]
    for directory in directories:
        if directory.exists():
            raise RuntimeError(f"score output already exists; refusing overwrite: {directory}")
        result = run_command(
            [
                sys.executable,
                str(require_regular_file(SCORER)),
                "--method",
                METHOD,
                "--panel",
                panel,
                "--artifact-manifest",
                str(manifest),
                "--output-dir",
                str(directory),
            ],
            root / f"score_{label}_{directory.name[-4:]}.log",
        )
        if result.returncode != 0:
            raise RuntimeError(f"{panel} scorer failed with exit {result.returncode}")
    hashes = compare_score_runs(directories[0], directories[1])
    return read_json(directories[0] / "summary.json"), hashes


def verify_integrity(root: Path) -> dict[str, Any]:
    output = root / "integrity.json"
    if output.exists():
        raise RuntimeError(f"integrity output already exists; refusing overwrite: {output}")
    result = run_command(
        [
            sys.executable,
            str(require_regular_file(INTEGRITY)),
            "--manifest",
            f"{METHOD}={root / 'artifact_manifest.jsonl'}",
            "--output",
            str(output),
        ],
        root / "integrity.log",
    )
    payload = read_json(output)
    if result.returncode != 0 or not payload.get("passed"):
        raise RuntimeError(f"artifact integrity verification failed with exit {result.returncode}")
    return payload


def report(root: Path, rows: list[dict[str, Any]], structured: dict[str, Any], numeric: dict[str, Any]) -> None:
    success = sum(row["status"] == "success" for row in rows)
    text = f"""# Artiverse prompt-only CLIP retrieval: Table 4 Constraints v2

Status: **COMPLETE**. This is a fixed-dataset retrieval/reference control, not
same-prompt generation and not an Artiverse generative method.

## Main supplementary result

| Candidate roots | Eligible rendered assets | Selected artifacts | Numeric pass |
|---:|---:|---:|---:|
| 3544 | {read_json(root / 'selection_preflight.json')['eligible_assets']} | {success}/18 | {numeric['numeric_pass']} |

The prompt-only selector used the exact 18 frozen prompts and global top-1
retrieval over eligible Artiverse assets. Selection used only identity paths and
the 16 reference renders. Geometry, category filters, part names, dimensions,
spec files, and scorer outputs were unavailable to selection. The selected GLB
was then evaluated without rescaling, repair, or rank fallback.

The structured score is `{structured['passed']}/{structured['constraints']}`
with count proxy `{structured['count_pass']}` and numeric
`{structured['numeric_pass']}`. The count value is only the frozen
name-matched renderable-node proxy; it is not semantic exact-count ground truth
for Artiverse node names and must not be a headline result.

Original selection amendment SHA-256: `{EXPECTED_AMENDMENT_SHA256}`.
Effective selection addendum SHA-256: `{lock_addendum_sha256(root)}`.
Selection lock SHA-256: `{sha256_file(root / 'selection.lock.json')}`.
Artifact manifest SHA-256: `{sha256_file(root / 'artifact_manifest.jsonl')}`.
"""
    atomic_text(root / "result_report.md", text)


def lock_addendum_sha256(root: Path) -> str:
    return str(read_json(root / "selection.lock.json")["addendum_sha256"])


def write_provenance(root: Path, lock: dict[str, Any]) -> dict[str, Any]:
    payload = {
        "schema_version": 1,
        "method": METHOD,
        "display_name": "Artiverse prompt-only CLIP retrieval (dataset reference; not generation)",
        "method_type": "fixed_dataset_prompt_only_retrieval_reference_control",
        "same_prompt_generation_method": False,
        "dataset": {
            "id": "3dlg-hcvc/artiverse",
            "revision": "8c4b120418e7cbdf9ac4c9580c5dbfdbf128a248",
            "huggingface": "https://huggingface.co/datasets/3dlg-hcvc/artiverse",
            "project": "https://3dlg-hcvc.github.io/artiverse/",
            "paper": "https://arxiv.org/abs/2605.24403",
            "release_state": "manual-gated pre-release subset",
            "license_field": "other",
            "license_note": "Use is subject to the corresponding licenses of the upstream source datasets; this experiment does not infer a single permissive license for all assets.",
            "local_manifest_sha256": lock["dataset_manifest_sha256"],
        },
        "retriever": {
            "model": "openai/clip-vit-base-patch32",
            "revision": "3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268",
            "selection_lock_sha256": sha256_file(root / "selection.lock.json"),
            "candidate_policy": "global top-1 over exactly-16-view eligible assets",
            "repair_attempts": 0,
            "rank_fallbacks": 0,
        },
        "benchmark": {
            "id": "table4_constraints_v2",
            "prompt_manifest_sha256": EXPECTED_PROMPTS_SHA256,
            "original_amendment_sha256": EXPECTED_AMENDMENT_SHA256,
            "effective_addendum_sha256": lock["addendum_sha256"],
        },
    }
    write_json(root / "provenance.json", payload)
    return payload


def run(root: Path) -> None:
    root = safe(root)
    lock, selection = validate_selection(root)
    rows = materialize(root, selection)
    structured, structured_hashes = score(root, "structured_main", "structured")
    numeric, numeric_hashes = score(root, "cad_numeric", "numeric")
    integrity = verify_integrity(root)
    write_provenance(root, lock)
    report(root, rows, structured, numeric)
    materialization_lock = {
        "schema_version": 1,
        "method": METHOD,
        "selection_lock_sha256": sha256_file(root / "selection.lock.json"),
        "selection_script_sha256": lock["selection_script_sha256"],
        "materialization_script_sha256": sha256_file(SCRIPT),
        "canonicalizer_sha256": sha256_file(CANONICALIZER),
        "scorer_sha256": sha256_file(SCORER),
        "integrity_verifier_sha256": sha256_file(INTEGRITY),
        "artifact_manifest_sha256": sha256_file(root / "artifact_manifest.jsonl"),
        "task_count": len(rows),
        "success_count": sum(row["status"] == "success" for row in rows),
        "failure_count": sum(row["status"] != "success" for row in rows),
        "repair_attempts": 0,
        "rank_fallbacks": 0,
        "structured_summary": structured,
        "numeric_summary": numeric,
        "structured_replay_sha256": structured_hashes,
        "numeric_replay_sha256": numeric_hashes,
        "integrity_passed": integrity["passed"],
        "provenance_sha256": sha256_file(root / "provenance.json"),
        "pre_result_audit_sha256": sha256_file(root / "protocol_audit_pre_result.json"),
        "pre_result_report_sha256": sha256_file(root / "report.md"),
        "result_report_sha256": sha256_file(root / "result_report.md"),
    }
    write_json(root / "materialization.lock.json", materialization_lock)
    print(json.dumps({
        "status": "COMPLETE",
        "artifacts": f"{materialization_lock['success_count']}/18",
        "numeric_pass": numeric["numeric_pass"],
        "count_proxy": structured["count_pass"],
        "integrity_passed": integrity["passed"],
        "materialization_lock_sha256": sha256_file(root / "materialization.lock.json"),
    }, indent=2), flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    return parser.parse_args()


def main() -> int:
    run(parse_args().root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
