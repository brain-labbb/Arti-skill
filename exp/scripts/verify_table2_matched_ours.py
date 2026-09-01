#!/usr/bin/env python3
"""Re-evaluate and verify the frozen Ours Table 2 Naming cohort."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = PROJECT_ROOT.parent.resolve()
RUNNER = PROJECT_ROOT / "exp/scripts/run_table2_matched_ours.py"
PROTOCOL = PROJECT_ROOT / "exp/reference/table2_naming_matched_protocol_v1.json"
DEFAULT_RUNTIME = PROJECT_ROOT / "exp/runtime/table2_naming_matched_ours_v1"
CORE_FILES = (
    "manifest.json",
    "generation_records.json",
    "records.jsonl",
    "artifact_hash_manifest.json",
    "report.md",
    "self_check.json",
)


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def digest(path: Path) -> str:
    target = contained(path)
    if target.is_symlink() or not target.is_file():
        raise RuntimeError(f"not a regular workspace file: {target}")
    return hashlib.sha256(target.read_bytes()).hexdigest()


def object_at(path: Path) -> dict[str, Any]:
    value = json.loads(contained(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"expected JSON object: {path}")
    return value


def stable_summary(runtime: Path) -> dict[str, Any]:
    payload = object_at(runtime / "summary.json")
    payload.pop("generated_at_utc", None)
    return payload


def rerun(runtime: Path) -> None:
    result = subprocess.run(
        [sys.executable, str(RUNNER), "--out", str(runtime), "--evaluate-existing"],
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"evaluate-existing failed ({result.returncode}): {result.stderr[-4000:]}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime", type=Path, default=DEFAULT_RUNTIME)
    args = parser.parse_args()
    runtime = contained(args.runtime)

    rerun(runtime)
    first_hashes = {name: digest(runtime / name) for name in CORE_FILES}
    first_summary = stable_summary(runtime)
    rerun(runtime)
    second_hashes = {name: digest(runtime / name) for name in CORE_FILES}
    second_summary = stable_summary(runtime)

    summary = object_at(runtime / "summary.json")
    self_check = object_at(runtime / "self_check.json")
    records = [
        json.loads(line)
        for line in (runtime / "records.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    artifact_manifest = json.loads(
        (runtime / "artifact_hash_manifest.json").read_text(encoding="utf-8")
    )
    category_counts: dict[str, int] = {}
    actual_mesh_hashes = 0
    artifact_hashes_current = True
    for row in records:
        category_counts[row["category"]] = category_counts.get(row["category"], 0) + 1
        package = contained(WORKSPACE_ROOT / row["package"])
        artifact_hashes_current &= digest(package / "model.urdf") == row["urdf_sha256"]
        for relative, expected in row["mesh_reference_sha256"].items():
            artifact_hashes_current &= digest(package / relative) == expected
            actual_mesh_hashes += 1

    direct = summary["direct_metrics"]
    checks = {
        "two_existing_evaluations_byte_identical": first_hashes == second_hashes,
        "two_stable_summaries_identical": first_summary == second_summary,
        "runner_self_check_pass": self_check["status"] == "PASS",
        "protocol_hash_current": summary["protocol_sha256"] == digest(PROTOCOL),
        "five_by_seven_conserved": (
            len(records) == 35 and len(category_counts) == 5 and set(category_counts.values()) == {7}
        ),
        "artifact_manifest_current": (
            summary["artifact_hash_manifest_sha256"]
            == hashlib.sha256(
                json.dumps(
                    artifact_manifest,
                    sort_keys=True,
                    separators=(",", ":"),
                    ensure_ascii=False,
                ).encode("utf-8")
            ).hexdigest()
        ),
        "all_urdf_and_mesh_hashes_current": artifact_hashes_current,
        "artifact_tree_has_no_symlinks": not any(
            path.is_symlink() for path in (runtime / "artifacts").rglob("*")
        ),
        "part_count_conserved": (
            sum(row["renderable_part_count"] for row in records)
            == direct["total_renderable_parts"]
        ),
        "name_count_conserved": (
            sum(row["named_renderable_part_count"] for row in records)
            == direct["total_named_renderable_parts"]
        ),
        "geometry_count_conserved": (
            sum(row["valid_visual_geometry_count"] for row in records)
            == direct["valid_visual_geometry_count"]
            and sum(row["invalid_visual_geometry_count"] for row in records)
            == direct["invalid_visual_geometry_count"]
        ),
        "mesh_hash_count_conserved": actual_mesh_hashes
        == direct["hashed_mesh_reference_count"],
        "semantic_metrics_fail_closed": all(
            value is None
            for key, value in summary["semantic_metrics"].items()
            if key != "reason"
        ),
    }
    payload = {
        "protocol_id": "nano3d_table2_naming_matched_ours_repro_v1",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "category_counts": dict(sorted(category_counts.items())),
        "conservation": {
            "assets": len(records),
            "renderable_parts": direct["total_renderable_parts"],
            "named_parts": direct["total_named_renderable_parts"],
            "valid_visual_geometries": direct["valid_visual_geometry_count"],
            "invalid_visual_geometries": direct["invalid_visual_geometry_count"],
            "hashed_mesh_references": actual_mesh_hashes,
        },
        "core_file_hashes": second_hashes,
        "stable_summary_sha256": summary["stable_summary_sha256"],
        "verifier_sha256": digest(Path(__file__)),
    }
    output = runtime / "reproducibility_check.json"
    temporary = runtime / ".reproducibility_check.json.tmp"
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(output)
    print(json.dumps({"status": payload["status"], "checks": len(checks), "output": str(output)}))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
