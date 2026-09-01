#!/usr/bin/env python3
"""Re-execute frozen official LAM Three.js parents and compile mesh + URDF.

No model provider is used. The released source, articulation JSON, and hierarchy
JSON are loaded from the official parquet and hash-checked against the manifest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
EXP_ROOT = REPO_ROOT / "exp"
LAM_ROOT = REPO_ROOT / ".cache/table6_sources/lam"
LAM_CODE = LAM_ROOT / "code"
PARQUET = LAM_ROOT / "dataset/articulated_code.parquet"
DEFAULT_ROOT = EXP_ROOT / "runtime/nano3d_editability_v2/lam_shared_editor"
DEFAULT_MANIFEST = DEFAULT_ROOT / "frozen_parent_manifest.json"
DEFAULT_OUT = DEFAULT_ROOT / "offline_parent_smoke"


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path outside authorized workspace: {resolved}")
    return resolved


def sha_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def sha_file(path: Path) -> str:
    return hashlib.sha256(contained(path, strict=True).read_bytes()).hexdigest()


def dump(path: Path, value: Any) -> None:
    path = contained(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")


def parse_optional_json(raw: str) -> Any:
    if raw.strip().lower() in {"", "nan", "none", "null"}:
        return None
    return json.loads(raw)


def localize_three_import(source: str) -> str:
    module = contained(LAM_CODE / "node_modules/three/build/three.module.js", strict=True)
    replacement = f"import * as THREE from '{module.as_posix()}';"
    localized, count = re.subn(
        r"import\s+\*\s+as\s+THREE\s+from\s+['\"](?:https://esm\.sh/three(?:@[^'\"]+)?|three)['\"]\s*;",
        replacement,
        source,
        count=1,
    )
    if count != 1:
        raise ValueError(f"expected exactly one supported THREE import, replaced {count}")
    return localized


def parse_graph(urdf: Path) -> dict[str, Any]:
    root = ET.parse(contained(urdf, strict=True)).getroot()
    links = sorted(node.attrib.get("name", "") for node in root.findall("link"))
    edges = sorted(
        (
            joint.find("parent").attrib.get("link", ""),
            joint.find("child").attrib.get("link", ""),
            joint.attrib.get("type", ""),
        )
        for joint in root.findall("joint")
        if joint.find("parent") is not None and joint.find("child") is not None
    )
    refs = []
    missing = []
    for mesh in root.findall(".//mesh"):
        raw = mesh.attrib.get("filename", "")
        refs.append(raw)
        if not contained(urdf.parent / raw).is_file():
            missing.append(raw)
    return {"links": links, "edges": edges, "mesh_references": refs, "missing_mesh_references": missing}


def execute(item: dict[str, Any], source_row: pd.Series, out: Path, timeout: int) -> dict[str, Any]:
    task_id = item["task_id"]
    task_out = contained(out / task_id)
    task_out.mkdir(parents=True, exist_ok=True)
    raw_source = str(source_row["threejs_code"])
    inline_urdf = str(source_row["urdf"])
    articulation_raw = str(source_row["articulation_json"])
    hierarchy_raw = str(source_row["links_hierarchy_json"])
    expected = {
        "threejs_code_sha256": sha_text(raw_source),
        "inline_urdf_sha256": sha_text(inline_urdf),
        "articulation_json_sha256": sha_text(articulation_raw),
        "links_hierarchy_json_sha256": sha_text(hierarchy_raw),
    }
    hash_match = all(item.get(key) == value for key, value in expected.items())
    if not hash_match:
        raise ValueError(f"{task_id}: release field hash differs from frozen manifest")

    raw_path = task_out / "parent_source.release.js"
    executable_path = task_out / "parent_source.localized.js"
    inline_path = task_out / "parent_inline_reference.urdf"
    configs = task_out / "configs"
    configs.mkdir(exist_ok=True)
    raw_path.write_text(raw_source, encoding="utf-8")
    executable_path.write_text(localize_three_import(raw_source), encoding="utf-8")
    inline_path.write_text(inline_urdf, encoding="utf-8")
    (configs / "articulation.json").write_text(
        json.dumps(json.loads(articulation_raw), indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (configs / "links_hierarchy.json").write_text(
        json.dumps(parse_optional_json(hierarchy_raw), indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    part_meshes = task_out / "part_meshes"
    part_meshes.mkdir(exist_ok=True)
    log_path = task_out / "node_export.log"
    command = [
        "node",
        str(contained(LAM_CODE / "utils/threejs_to_mesh.js", strict=True)),
        str(executable_path),
        str(part_meshes),
    ]
    completed = subprocess.run(
        command,
        cwd=contained(LAM_CODE, strict=True),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )
    log_path.write_text(completed.stdout, encoding="utf-8")
    generated = task_out / "generated.urdf"
    urdf_error = None
    if completed.returncode == 0:
        sys.path.insert(0, str(contained(LAM_CODE, strict=True)))
        try:
            from utils.generate_urdf import generate_urdf

            generate_urdf(str(task_out / "links"), str(task_out), str(generated))
        except Exception as exc:  # noqa: BLE001
            urdf_error = f"{type(exc).__name__}: {exc}"
    graph = parse_graph(generated) if generated.is_file() else None
    inline_graph = parse_graph(inline_path)
    record = {
        "task_id": task_id,
        "object_release_id": item["object_release_id"],
        "status": (
            "PASS"
            if completed.returncode == 0 and urdf_error is None and graph and not graph["missing_mesh_references"]
            else "FAIL"
        ),
        "api_called": False,
        "manifest_field_hashes_match": hash_match,
        "node_command": command,
        "node_returncode": completed.returncode,
        "node_log": str(log_path),
        "urdf_error": urdf_error,
        "raw_source": str(raw_path),
        "raw_source_sha256": sha_file(raw_path),
        "localized_source": str(executable_path),
        "localized_source_sha256": sha_file(executable_path),
        "generated_urdf": str(generated) if generated.is_file() else None,
        "generated_urdf_sha256": sha_file(generated) if generated.is_file() else None,
        "part_obj_count": len(list(part_meshes.rglob("*.obj"))),
        "link_obj_count": len(list((task_out / "links").glob("*.obj"))),
        "generated_graph": graph,
        "inline_graph": inline_graph,
        "topology_matches_inline": bool(graph and graph["links"] == inline_graph["links"] and graph["edges"] == inline_graph["edges"]),
    }
    dump(task_out / "record.json", record)
    return record


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--timeout", type=int, default=300)
    args = parser.parse_args()
    manifest_path = contained(args.manifest, strict=True)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    items = manifest.get("items", [])
    task_ids = [item.get("task_id") for item in items]
    allowed_task_ids = {f"{prefix}{index:02d}" for prefix in ("A", "M", "S") for index in range(1, 7)}
    if not items or len(task_ids) != len(set(task_ids)) or not set(task_ids).issubset(allowed_task_ids):
        raise ValueError("offline executor requires a non-empty unique subset of shared A01-A06/M01-M06/S01-S06 tasks")
    if manifest.get("main_table_eligible") is not False or manifest.get("api_called") is not False:
        raise ValueError("manifest must identify an offline external-editor probe excluded from the main table")
    shared_manifest = contained(WORKSPACE_ROOT / manifest["shared_task_manifest"], strict=True)
    if sha_file(shared_manifest) != manifest.get("shared_task_manifest_sha256"):
        raise ValueError("shared task manifest SHA differs from frozen cohort provenance")
    frame = pd.read_parquet(contained(PARQUET, strict=True))
    frame = frame[frame["tier"] == "viable"]
    records = []
    for item in items:
        matches = frame[frame["object_release_id"] == item["object_release_id"]]
        if len(matches) != 1:
            raise ValueError(f"missing unique release row: {item['object_release_id']}")
        records.append(execute(item, matches.iloc[0], contained(args.out), args.timeout))
        print(item["task_id"], records[-1]["status"], flush=True)
    summary = {
        "protocol_id": "nano3d_table5_lam_released_source_offline_smoke_v3",
        "status": "PASS" if all(row["status"] == "PASS" for row in records) else "FAIL",
        "api_called": False,
        "generated_code_executed": True,
        "main_table_eligible": False,
        "shared_task_manifest_sha256": manifest["shared_task_manifest_sha256"],
        "manifest": str(manifest_path.relative_to(WORKSPACE_ROOT)),
        "manifest_sha256": sha_file(manifest_path),
        "case_count": len(records),
        "pass_count": sum(row["status"] == "PASS" for row in records),
        "topology_match_count": sum(row["topology_matches_inline"] for row in records),
        "records": records,
    }
    dump(contained(args.out) / "summary.json", summary)
    print(json.dumps({key: summary[key] for key in ("status", "case_count", "pass_count", "topology_match_count", "api_called")}, indent=2))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
