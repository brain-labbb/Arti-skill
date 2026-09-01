#!/usr/bin/env python3
"""Validate a frozen mobility arm, then launch the metadata-blind selector."""

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


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
SELECTOR = REPO / "exp/scripts/run_partnet_mobility_clip_retrieval_selection.py"
DEFAULT_AMENDMENT = REPO / "exp/reference/table4_constraints_v2/amendment_partnet_mobility_clip_retrieval_v1.json"
DEFAULT_SNAPSHOT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1_snapshot"
DEFAULT_OUTPUT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1"
EXPECTED_PROMPT_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
EXPECTED_PROTOCOL_SHA256 = "6857194072ccb0ba3943d14a62f29c2364920ec01381a5475311535c1831031f"
PROMPTS = REPO / "exp/reference/table4_constraints_v2/prompts.jsonl"
PROTOCOL = REPO / "exp/reference/table4_constraints_v2/protocol.json"
PROMPT_ONLY_NAME = "prompt_only.jsonl"
EXPECTED_ENVIRONMENT = {
    "CUDA_VISIBLE_DEVICES": "1",
    "HF_HUB_OFFLINE": "1",
    "TRANSFORMERS_OFFLINE": "1",
    "TOKENIZERS_PARALLELISM": "false",
    "CUBLAS_WORKSPACE_CONFIG": ":4096:8",
    "CUDA_DEVICE_ORDER": "PCI_BUS_ID",
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
    "PYTHONHASHSEED": "0",
}
CONTRACT_FIELDS = {
    "schema_version", "contract_id", "expected_assets", "views", "batch_assets", "task_count",
    "source_prompt_manifest_sha256", "prompt_only_manifest_sha256", "protocol_sha256",
    "model_id", "model_revision", "implementation", "selector_runtime", "gpu", "thread_environment",
}


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"outside authorized workspace: {resolved}")
    return resolved


def regular_file(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
        raise RuntimeError(f"not a regular non-symlink file: {path}")
    return resolved


def regular_dir(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISDIR(path.lstat().st_mode):
        raise RuntimeError(f"not a regular non-symlink directory: {path}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with regular_file(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(regular_file(path).read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in regular_file(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def jsonl_bytes(rows: list[dict[str, Any]]) -> bytes:
    return "".join(json.dumps(row, sort_keys=True, ensure_ascii=True, separators=(",", ":")) + "\n" for row in rows).encode("utf-8")


def configured_path(value: str) -> Path:
    path = Path(value)
    return safe(path if path.is_absolute() else REPO / path, must_exist=False)


def implementation_hashes(amendment: dict[str, Any]) -> dict[str, str]:
    files = amendment.get("shared_pipeline", {}).get("implementation_files", {})
    if not isinstance(files, dict) or not files:
        raise RuntimeError("frozen implementation file map missing")
    return {field: sha256_file(configured_path(path)) for field, path in files.items()}


def validate(amendment_path: Path, snapshot: Path, output: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    amendment_path = regular_file(amendment_path)
    snapshot = regular_dir(snapshot)
    output = regular_dir(output)
    amendment = read_json(amendment_path)
    shared = amendment.get("shared_pipeline", {})
    if amendment.get("status") != "pre_result_frozen":
        raise RuntimeError("amendment is not pre-result frozen")
    required_paths = {
        "opaque_snapshot": snapshot,
        "formal_root": output,
        "selection_launcher": safe(SCRIPT),
        "selector": safe(SELECTOR),
    }
    for field, observed in required_paths.items():
        if configured_path(shared.get(field, "")) != observed:
            raise RuntimeError(f"frozen shared path drifted: {field}")
    hashes = implementation_hashes(amendment)
    if amendment.get("implementation") != hashes:
        raise RuntimeError("frozen implementation hashes drifted")
    if hashes.get("selection_launcher_sha256") != sha256_file(SCRIPT) or hashes.get("selector_sha256") != sha256_file(SELECTOR):
        raise RuntimeError("shared selection implementation binding drifted")
    contract_source = regular_file(configured_path(shared.get("selection_contract", "")))
    contract_sha256 = sha256_file(contract_source)
    if shared.get("selection_contract_sha256") != contract_sha256:
        raise RuntimeError("frozen selection execution contract drifted")
    contract = read_json(contract_source)
    if set(contract) != CONTRACT_FIELDS or contract.get("schema_version") != 1:
        raise RuntimeError("selection execution contract schema drifted")
    if set(contract["implementation"]) != {"selection_launcher_sha256", "selector_sha256"}:
        raise RuntimeError("selection execution contract implementation closure drifted")
    shared_selection_hashes = {field: hashes[field] for field in contract["implementation"]}
    if contract["implementation"] != shared_selection_hashes or contract.get("contract_id") != shared.get("contract_id"):
        raise RuntimeError("selection execution contract implementation/identity binding drifted")
    if contract.get("expected_assets") != shared.get("expected_assets") or contract.get("views") != 8 or contract.get("batch_assets") != 8 or contract.get("task_count") != 18:
        raise RuntimeError("selection execution contract cardinality drifted")
    if contract.get("source_prompt_manifest_sha256") != EXPECTED_PROMPT_SHA256 or contract.get("protocol_sha256") != EXPECTED_PROTOCOL_SHA256:
        raise RuntimeError("selection execution contract benchmark hashes drifted")
    if sha256_file(PROMPTS) != EXPECTED_PROMPT_SHA256 or sha256_file(PROTOCOL) != EXPECTED_PROTOCOL_SHA256:
        raise RuntimeError("frozen benchmark byte identity drifted")
    source_prompts = read_jsonl(PROMPTS)
    prompt_only = [{"task_id": row["task_id"], "prompt": row["prompt"]} for row in source_prompts]
    if len(source_prompts) != 18 or source_prompts != sorted(source_prompts, key=lambda row: row["task_id"]):
        raise RuntimeError("frozen source prompt order/cardinality drifted")
    if any(set(row) != {"task_id", "prompt"} for row in prompt_only):
        raise RuntimeError("prompt-only projection schema drifted")
    if hashlib.sha256(jsonl_bytes(prompt_only)).hexdigest() != contract.get("prompt_only_manifest_sha256"):
        raise RuntimeError("prompt-only manifest content drifted")
    if {entry.name for entry in os.scandir(output)} != {"protocol_audit_pre_result.json", "report.md"}:
        raise RuntimeError("formal output must contain exactly the independent pre-result audit and report")
    audit_path = regular_file(output / "protocol_audit_pre_result.json")
    report_path = regular_file(output / "report.md")
    audit = read_json(audit_path)
    if str(audit.get("verdict") or audit.get("status") or "").upper() != "PASS" or audit.get("protocol_ready") is not True:
        raise RuntimeError("independent pre-result audit is not ready PASS")
    required_audit = {"amendment_sha256": sha256_file(amendment_path), **hashes}
    for field, expected in required_audit.items():
        if audit.get(field) != expected:
            raise RuntimeError(f"independent pre-result audit binding drifted: {field}")
    if audit.get("report_sha256") != sha256_file(report_path):
        raise RuntimeError("independent pre-result audit report drifted")
    snapshot_lock = read_json(snapshot / "snapshot.lock.json")
    if snapshot_lock.get("status") != "PASS" or snapshot_lock.get("selection_execution_contract_sha256") != contract_sha256:
        raise RuntimeError("opaque snapshot selection-contract binding drifted")
    snapshot_contract = regular_file(snapshot / "selection.execution_contract.json")
    if sha256_file(snapshot_contract) != contract_sha256:
        raise RuntimeError("selector-visible execution contract drifted")
    snapshot_prompt_only = regular_file(snapshot / PROMPT_ONLY_NAME)
    if sha256_file(snapshot_prompt_only) != contract.get("prompt_only_manifest_sha256") or snapshot_prompt_only.read_bytes() != jsonl_bytes(prompt_only):
        raise RuntimeError("selector-visible prompt-only manifest drifted")
    return amendment, contract


def launch(amendment: Path, snapshot: Path, output: Path) -> None:
    _, contract = validate(amendment, snapshot, output)
    environment = dict(os.environ)
    for name, value in EXPECTED_ENVIRONMENT.items():
        if environment.get(name) not in (None, value):
            raise RuntimeError(f"selection launcher environment drifted: {name}={environment.get(name)!r}")
        environment[name] = value
    result = subprocess.run(
        [sys.executable, str(safe(SELECTOR)), "--snapshot-root", str(safe(snapshot)), "--output-dir", str(safe(output / "selection_bundle", must_exist=False))],
        cwd=str(safe(REPO)),
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    print(result.stdout, end="", flush=True)
    if result.returncode != 0:
        raise RuntimeError(f"metadata-blind selector failed with exit code {result.returncode}")
    selection_root = output / "selection_bundle"
    lock = read_json(selection_root / "selection.lock.json")
    if lock.get("status") != "PASS" or lock.get("contract_id") != contract["contract_id"]:
        raise RuntimeError("published selection lock identity/status drifted")
    if lock.get("selection_execution_contract_sha256") != sha256_file(snapshot / "selection.execution_contract.json"):
        raise RuntimeError("published selection lock contract binding drifted")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--amendment", type=Path, default=DEFAULT_AMENDMENT)
    parser.add_argument("--snapshot-root", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    launch(args.amendment, args.snapshot_root, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
