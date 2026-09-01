#!/usr/bin/env python3
"""Replay and byte-verify the frozen PhysX-Mobility paired reference."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REFERENCE = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/physx_mobility_reference"
DEFAULT_REPLAY = EXP_ROOT / "runtime/nano3d_hierarchy_correctness/physx_mobility_reference_replay"
RUNNER = Path(__file__).with_name("run_physx_mobility_paired_reference.py")
CANONICAL_SELECTION_SHA256 = "a0a8eaf00c2970598f3d6191001361dc1e1be1df43ba3e8c394cb6ef988d581b"
AUTHORIZED_ROOTS = (Path("/mnt/zsn/lyb"), Path("/mnt/zsn/zsn_workspace"))


def contained(path: Path, *, exists: bool = True) -> Path:
    resolved = path.resolve(strict=exists)
    if not any(resolved == root or root in resolved.parents for root in AUTHORIZED_ROOTS):
        raise ValueError(f"path outside authorized roots: {resolved}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def tree_hashes(root: Path) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): sha256_file(path)
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    args = parser.parse_args()
    reference = contained(args.reference)
    replay = contained(args.replay, exists=False)
    if replay.exists():
        raise FileExistsError(f"refusing to overwrite replay directory: {replay}")

    command = [
        sys.executable,
        str(RUNNER),
        "--output",
        str(replay),
        "--selection",
        str(reference / "paired_selection.json"),
        "--reference-protocol",
        str(reference / "reference_protocol_snapshot.json"),
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            f"replay failed with exit {completed.returncode}\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
        )
    reference_hashes = tree_hashes(reference)
    replay_hashes = tree_hashes(replay)
    excluded = {"determinism_verification.json"}
    reference_comparable = {
        name: digest for name, digest in reference_hashes.items() if name not in excluded
    }
    replay_comparable = {
        name: digest for name, digest in replay_hashes.items() if name not in excluded
    }
    file_names_identical = set(reference_comparable) == set(replay_comparable)
    comparisons: dict[str, dict[str, Any]] = {}
    for name in sorted(set(reference_comparable) | set(replay_comparable)):
        left = reference_comparable.get(name)
        right = replay_comparable.get(name)
        comparisons[name] = {
            "reference_sha256": left,
            "replay_sha256": right,
            "identical": left is not None and left == right,
        }
    reference_verification = json.loads((reference / "verification.json").read_text(encoding="utf-8"))
    replay_verification = json.loads((replay / "verification.json").read_text(encoding="utf-8"))
    checks = {
        "replay_exit_zero": completed.returncode == 0,
        "file_name_sets_identical": file_names_identical,
        "all_files_byte_identical": all(row["identical"] for row in comparisons.values()),
        "reference_verification_passed": reference_verification["passed"] is True,
        "replay_verification_passed": replay_verification["passed"] is True,
        "reference_canonical_selection_matches": sha256_file(reference / "paired_selection.json")
        == CANONICAL_SELECTION_SHA256,
        "replay_canonical_selection_matches": sha256_file(replay / "paired_selection.json")
        == CANONICAL_SELECTION_SHA256,
    }
    result = {
        "passed": all(checks.values()),
        "checks": checks,
        "reference": str(reference),
        "replay": str(replay),
        "canonical_selection_sha256": CANONICAL_SELECTION_SHA256,
        "compared_file_count": len(comparisons),
        "selected_snapshot_file_count": sum(
            name.startswith("selected_metadata/") for name in reference_comparable
        ),
        "file_comparisons": comparisons,
        "runner_stdout": completed.stdout,
        "runner_stderr": completed.stderr,
    }
    destination = reference / "determinism_verification.json"
    destination.write_text(
        json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "passed": result["passed"],
                "compared_file_count": result["compared_file_count"],
                "selected_snapshot_file_count": result["selected_snapshot_file_count"],
                "output": str(destination),
            },
            indent=2,
        )
    )
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
