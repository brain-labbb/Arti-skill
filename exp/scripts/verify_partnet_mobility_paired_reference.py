#!/usr/bin/env python3
"""Replay and verify the PartNet side of the shared PhysX/PartNet cohort."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


EXP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = EXP_ROOT.parents[1]
DEFAULT_REFERENCE = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/physx_partnet_paired_partnet_reference_v2"
DEFAULT_REPLAY = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/physx_partnet_paired_partnet_reference_replay"
RUNNER = Path(__file__).with_name("run_partnet_mobility_paired_reference.py")


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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    args = parser.parse_args()
    reference = contained(args.reference)
    replay = contained(args.replay, exists=False)
    if replay.exists():
        raise FileExistsError(f"refusing to overwrite replay: {replay}")
    command = [
        sys.executable,
        str(RUNNER),
        "--output",
        str(replay),
        "--selection",
        str(reference / "paired_selection.json"),
        "--protocol",
        str(reference / "paired_protocol_snapshot.json"),
        "--ontology-protocol",
        str(reference / "ontology_protocol_snapshot.json"),
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"replay failed\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}")
    comparable = (
        "paired_selection.json",
        "paired_protocol_snapshot.json",
        "ontology_protocol_snapshot.json",
        "manifest.jsonl",
        "structure_records.jsonl",
        "urdf_name_only_records.jsonl",
        "package_semantics_assisted_records.jsonl",
        "paired_graph_records.jsonl",
        "summary.json",
        "verification.json",
    )
    comparisons = {}
    for relative in comparable:
        first = sha256_file(reference / relative)
        second = sha256_file(replay / relative)
        comparisons[relative] = {"reference_sha256": first, "replay_sha256": second, "identical": first == second}
    checks = {
        "replay_exit_zero": completed.returncode == 0,
        "all_comparable_files_byte_identical": all(row["identical"] for row in comparisons.values()),
        "reference_verification_passed": json.loads((reference / "verification.json").read_text(encoding="utf-8"))["passed"] is True,
        "replay_verification_passed": json.loads((replay / "verification.json").read_text(encoding="utf-8"))["passed"] is True,
    }
    result = {
        "passed": all(checks.values()),
        "checks": checks,
        "file_comparisons": comparisons,
        "selection_sha256": sha256_file(reference / "paired_selection.json"),
        "runner_stdout": completed.stdout,
        "runner_stderr": completed.stderr,
    }
    destination = reference / "determinism_verification.json"
    destination.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"passed": result["passed"], "output": str(destination)}, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
