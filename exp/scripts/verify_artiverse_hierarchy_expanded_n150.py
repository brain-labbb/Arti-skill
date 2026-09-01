#!/usr/bin/env python3
"""Replay and byte-verify the expanded Artiverse real-data reference."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_REFERENCE = EXP_ROOT / "runtime/nano3d_hierarchy_expanded_n150/artiverse_reference"
DEFAULT_REPLAY = EXP_ROOT / "runtime/nano3d_hierarchy_expanded_n150/artiverse_reference_replay"
RUNNER = Path(__file__).with_name("run_artiverse_hierarchy_expanded_n150.py")


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


def selected_file_hashes(root: Path) -> dict[str, str]:
    selected = root / "selected_metadata"
    return {
        path.relative_to(root).as_posix(): sha256_file(path)
        for path in sorted(selected.glob("*/*"))
        if path.is_file()
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--artiverse-root", type=Path, default=EXP_ROOT / "artiverse")
    parser.add_argument("--protocol", type=Path, default=EXP_ROOT / "reference/partnet_hierarchy_correctness_v1.json")
    args = parser.parse_args()

    reference = contained(args.reference)
    replay = contained(args.replay, exists=False)
    artiverse = contained(args.artiverse_root)
    protocol = contained(args.protocol)
    selection = contained(reference / "frozen_selection.json")
    reference_protocol = contained(reference / "reference_protocol_snapshot.json")
    if replay.exists():
        raise FileExistsError(f"refusing to overwrite replay directory: {replay}")

    command = [
        sys.executable,
        str(RUNNER),
        "--artiverse-root",
        str(artiverse),
        "--output",
        str(replay),
        "--protocol",
        str(protocol),
        "--selection",
        str(selection),
        "--reference-protocol",
        str(reference_protocol),
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            f"replay failed with exit {completed.returncode}\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
        )

    comparable_files = (
        "frozen_selection.json",
        "reference_protocol_snapshot.json",
        "manifest.jsonl",
        "alias_sensitivity_manifest.jsonl",
        "primary_4class_matched_overlap_structure_records.jsonl",
        "table_alias_5class_sensitivity_structure_records.jsonl",
        "primary_4class_matched_overlap_urdf_name_only_alignment_records.jsonl",
        "table_alias_5class_sensitivity_urdf_name_only_alignment_records.jsonl",
        "articulation_metadata_records.jsonl",
        "category_feasibility_audit.json",
        "nested_original_verification.json",
        "manifest_root_verification.json",
        "hf_revision_verification.json",
        "provenance.json",
        "summary.json",
        "report.md",
        "verification.json",
    )
    file_comparisons: dict[str, dict[str, Any]] = {}
    for relative in comparable_files:
        original_path = reference / relative
        replay_path = replay / relative
        original_hash = sha256_file(original_path)
        replay_hash = sha256_file(replay_path)
        file_comparisons[relative] = {
            "reference_sha256": original_hash,
            "replay_sha256": replay_hash,
            "identical": original_hash == replay_hash,
        }
    reference_selected = selected_file_hashes(reference)
    replay_selected = selected_file_hashes(replay)
    checks = {
        "replay_exit_zero": completed.returncode == 0,
        "all_comparable_files_byte_identical": all(
            row["identical"] for row in file_comparisons.values()
        ),
        "selected_file_name_sets_identical": set(reference_selected) == set(replay_selected),
        "selected_files_byte_identical": reference_selected == replay_selected,
        "reference_verification_passed": json.loads(
            (reference / "verification.json").read_text(encoding="utf-8")
        )["passed"] is True,
        "replay_verification_passed": json.loads(
            (replay / "verification.json").read_text(encoding="utf-8")
        )["passed"] is True,
    }
    result = {
        "passed": all(checks.values()),
        "checks": checks,
        "reference": str(reference),
        "replay": str(replay),
        "selection_sha256": sha256_file(selection),
        "file_comparisons": file_comparisons,
        "selected_file_count": len(reference_selected),
        "runner_stdout": completed.stdout,
        "runner_stderr": completed.stderr,
    }
    destination = reference / "determinism_verification.json"
    destination.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"passed": result["passed"], "output": str(destination)}, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
