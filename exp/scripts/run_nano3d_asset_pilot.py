#!/usr/bin/env python3
"""Run the directly measurable Nano3D existing-export pilot.

This harness is intentionally limited to measurements that do not require a
hidden semantic gold spec, an LLM judge, edit tasks, or a full physics sweep.
It reads the 33 asset paths from Nano3dasset.md, never edits source assets, and
writes all generated artifacts under an output directory.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
import xml.etree.ElementTree as ET
from collections import Counter, deque
from pathlib import Path
from typing import Any


WORKSPACE = Path("/mnt/zsn/lyb/arti-skill").resolve()
TEMPLATE_ROOT = WORKSPACE / "arti-template"
ASSET_DOC = WORKSPACE / "exp/Nano3dasset.md"
DEFAULT_OUTPUT = WORKSPACE / "exp/runtime/nano3d_asset_pilot"
ASSET_LINK_RE = re.compile(r"\]\((/mnt/zsn/lyb/arti-skill/(?:seed_exports|seed_exports_physics_10)/[^)]+)\)")
PLACEHOLDER_RE = re.compile(
    r"^(?:link|part|mesh|geometry|object)(?:[_-]?(?:\d+|new|object))?$",
    re.IGNORECASE,
)
MESH_SUFFIXES = {".obj", ".stl", ".ply", ".glb", ".gltf", ".off", ".dae"}


def sha256_file(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_asset_paths() -> list[Path]:
    text = ASSET_DOC.read_text(encoding="utf-8")
    paths: list[Path] = []
    seen: set[str] = set()
    for raw in ASSET_LINK_RE.findall(text):
        path = Path(raw).resolve()
        if str(path) not in seen:
            seen.add(str(path))
            paths.append(path)
    if len(paths) != 33:
        raise RuntimeError(f"expected 33 asset links in {ASSET_DOC}, found {len(paths)}")
    return paths


def asset_identity(path: Path) -> dict[str, Any]:
    slug = path.parent.name
    seed_name = path.name
    seed = int(seed_name.removeprefix("seed_"))
    source = "seed_exports_physics_10" if "seed_exports_physics_10" in path.parts else "seed_exports"
    return {
        "asset_id": f"{slug}__{seed_name}",
        "slug": slug,
        "seed": seed,
        "source": source,
        "asset_path": str(path),
    }


def mesh_reference_path(asset_dir: Path, filename: str) -> Path | None:
    raw = str(filename).strip()
    if not raw or raw.startswith("package://"):
        if raw.startswith("package://"):
            raw = raw.removeprefix("package://")
        else:
            return None
    if raw.startswith("file://"):
        raw = raw.removeprefix("file://")
    candidate = Path(raw)
    if candidate.is_absolute():
        return candidate
    return (asset_dir / candidate).resolve()


def parse_urdf(asset_dir: Path, urdf_path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {
        "valid_urdf": False,
        "links": 0,
        "joints": 0,
        "roots": 0,
        "acyclic": False,
        "reachable": False,
        "valid_tree": False,
        "max_depth": 0,
        "raw_name_coverage": None,
        "joint_metadata_rate": None,
        "movable_joints": 0,
        "joint_types": {},
        "mesh_references": 0,
        "missing_mesh_references": [],
        "absolute_path_references": 0,
        "parent_child_edges": 0,
    }
    try:
        root = ET.parse(urdf_path).getroot()
    except Exception as exc:  # noqa: BLE001
        result["urdf_error"] = f"{type(exc).__name__}: {exc}"
        return result

    links = [node.attrib.get("name", "") for node in root.findall("link")]
    link_set = {name for name in links if name}
    joints = root.findall("joint")
    result["valid_urdf"] = bool(link_set)
    result["links"] = len(links)
    result["joints"] = len(joints)

    names = [name for name in links if name]
    names.extend(node.attrib.get("name", "") for node in joints if node.attrib.get("name"))
    if names:
        semantic = [name for name in names if not PLACEHOLDER_RE.fullmatch(name)]
        result["raw_name_coverage"] = len(semantic) / len(names)

    adjacency: dict[str, list[str]] = {name: [] for name in link_set}
    children: set[str] = set()
    metadata_ok = 0
    joint_types: Counter[str] = Counter()
    for joint in joints:
        joint_type = joint.attrib.get("type", "")
        joint_types[joint_type] += 1
        if joint_type != "fixed":
            result["movable_joints"] += 1
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        parent = parent_node.attrib.get("link", "") if parent_node is not None else ""
        child = child_node.attrib.get("link", "") if child_node is not None else ""
        if parent in link_set and child in link_set:
            adjacency[parent].append(child)
            children.add(child)
            result["parent_child_edges"] += 1
        complete = bool(joint_type and parent in link_set and child in link_set)
        if joint_type != "fixed":
            complete = complete and joint.find("axis") is not None
            if joint_type in {"revolute", "prismatic"}:
                complete = complete and joint.find("limit") is not None
        if complete:
            metadata_ok += 1

    result["joint_types"] = dict(sorted(joint_types.items()))
    result["joint_metadata_rate"] = metadata_ok / len(joints) if joints else 1.0
    roots = sorted(link_set - children)
    result["roots"] = len(roots)

    visited: set[str] = set()
    active: set[str] = set()
    cycle = False

    def visit(node: str) -> None:
        nonlocal cycle
        if node in active:
            cycle = True
            return
        if node in visited:
            return
        active.add(node)
        for child in adjacency.get(node, []):
            visit(child)
        active.remove(node)
        visited.add(node)

    for node in link_set:
        visit(node)
    result["acyclic"] = not cycle

    reachable: set[str] = set()
    queue: deque[tuple[str, int]] = deque((root_name, 0) for root_name in roots)
    while queue:
        node, depth = queue.popleft()
        if node in reachable:
            continue
        reachable.add(node)
        result["max_depth"] = max(result["max_depth"], depth)
        queue.extend((child, depth + 1) for child in adjacency.get(node, []))
    result["reachable"] = reachable == link_set
    result["valid_tree"] = (
        len(roots) == 1
        and result["acyclic"]
        and result["reachable"]
        and result["parent_child_edges"] == max(0, len(link_set) - 1)
    )

    missing_meshes: list[str] = []
    for mesh_node in root.findall(".//mesh"):
        filename = mesh_node.attrib.get("filename", "")
        result["mesh_references"] += 1
        raw_filename = str(filename).strip()
        is_absolute_reference = raw_filename.startswith("/") or raw_filename.startswith("file:///")
        ref_path = mesh_reference_path(asset_dir, filename)
        if is_absolute_reference:
            result["absolute_path_references"] += 1
        if ref_path is not None and not ref_path.exists():
            missing_meshes.append(filename)
    result["missing_mesh_references"] = sorted(set(missing_meshes))
    return result


def mesh_stats(asset_dir: Path) -> dict[str, Any]:
    mesh_paths = sorted(
        path for path in (asset_dir / "assets").rglob("*") if path.is_file() and path.suffix.lower() in MESH_SUFFIXES
    )
    payload: dict[str, Any] = {
        "mesh_files": len(mesh_paths),
        "readable_mesh_files": 0,
        "watertight_mesh_files": 0,
        "winding_consistent_mesh_files": 0,
        "open_edges": 0,
        "degenerate_faces": 0,
        "mesh_errors": [],
    }
    if not mesh_paths:
        return payload
    try:
        import trimesh
    except Exception as exc:  # noqa: BLE001
        payload["mesh_errors"].append(f"trimesh_unavailable: {exc}")
        return payload
    for path in mesh_paths:
        try:
            loaded = trimesh.load(path, force="scene", process=False)
            geometries = list(loaded.geometry.values()) if hasattr(loaded, "geometry") else [loaded]
            geometries = [mesh for mesh in geometries if hasattr(mesh, "faces") and hasattr(mesh, "vertices")]
            if not geometries:
                raise ValueError("no mesh geometry")
            for mesh in geometries:
                payload["readable_mesh_files"] += 1
                if bool(getattr(mesh, "is_watertight", False)):
                    payload["watertight_mesh_files"] += 1
                if bool(getattr(mesh, "is_winding_consistent", False)):
                    payload["winding_consistent_mesh_files"] += 1
                try:
                    edges = mesh.edges_sorted
                    _, counts = __import__("numpy").unique(edges, axis=0, return_counts=True)
                    payload["open_edges"] += int((counts == 1).sum())
                except Exception:
                    pass
                try:
                    payload["degenerate_faces"] += int((mesh.area_faces <= 1e-12).sum())
                except Exception:
                    pass
        except Exception as exc:  # noqa: BLE001
            payload["mesh_errors"].append(f"{path.name}: {type(exc).__name__}: {exc}")
    return payload


def static_record(asset_dir: Path) -> dict[str, Any]:
    identity = asset_identity(asset_dir)
    model_py = asset_dir / "model.py"
    model_urdf = asset_dir / "model.urdf"
    compile_report_path = asset_dir / "compile_report.json"
    required = [model_py, model_urdf, asset_dir / "assets", compile_report_path]
    required_complete = all(path.exists() for path in required)
    compile_status = None
    try:
        compile_status = json.loads(compile_report_path.read_text(encoding="utf-8")).get("status")
    except Exception:
        pass
    urdf = parse_urdf(asset_dir, model_urdf) if model_urdf.exists() else {"valid_urdf": False}
    mesh = mesh_stats(asset_dir) if (asset_dir / "assets").exists() else {}
    asset_files = [path for path in (asset_dir / "assets").rglob("*") if path.is_file()] if (asset_dir / "assets").exists() else []
    source_bytes = model_py.stat().st_size if model_py.exists() else 0
    urdf_bytes = model_urdf.stat().st_size if model_urdf.exists() else 0
    mesh_bytes = sum(path.stat().st_size for path in asset_files)
    validation_path = asset_dir / "validation_report.json"
    physics_status = "not_applicable"
    if "seed_exports_physics_10" in asset_dir.parts:
        if validation_path.exists():
            try:
                validation = json.loads(validation_path.read_text(encoding="utf-8"))
                physics_status = "validated" if validation.get("success") and validation.get("dataset_ready") else "validation_failed"
            except Exception:
                physics_status = "invalid_validation_report"
        else:
            physics_status = "missing_validation_report"
    package_complete = (
        required_complete
        and bool(asset_files)
        and not urdf.get("missing_mesh_references")
        and urdf.get("absolute_path_references", 0) == 0
    )
    links = int(urdf.get("links", 0) or 0)
    difficulty = "L1" if links <= 3 else "L2" if links <= 8 else "L3"
    record = {
        **identity,
        "difficulty": difficulty,
        "required_complete": required_complete,
        "compile_status": compile_status,
        "package_complete": package_complete,
        "asset_file_count": len(asset_files),
        "source_bytes": source_bytes,
        "source_loc": len(model_py.read_text(encoding="utf-8", errors="replace").splitlines()) if model_py.exists() else 0,
        "urdf_bytes": urdf_bytes,
        "mesh_bytes": mesh_bytes,
        "physics_validation": physics_status,
        **urdf,
        **mesh,
    }
    return record


COMPILE_CODE = r'''
import json, sys, traceback
from pathlib import Path
from agent.compiler import compile_urdf_report
path = Path(sys.argv[1]).resolve()
try:
    report = compile_urdf_report(path, sdk_package="sdk", run_checks=True, target="full", rewrite_visual_glb=False, motion_qc=False)
    bundle = getattr(report, "signal_bundle", None)
    signals = bundle.to_dict() if bundle is not None and hasattr(bundle, "to_dict") else None
    print(json.dumps({"status": "success", "urdf_xml": report.urdf_xml, "warnings": list(report.warnings), "signals": signals}))
except BaseException as exc:
    print(json.dumps({"status": "failure", "error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()}))
    raise
'''


def reexecute_one(asset_dir: Path, output_root: Path, timeout_s: float) -> dict[str, Any]:
    identity = asset_identity(asset_dir)
    asset_id = identity["asset_id"]
    started = time.monotonic()
    work_root = Path(tempfile.mkdtemp(prefix=f"nano3d_{asset_id}_", dir=output_root / "work"))
    copied = work_root / asset_dir.name
    try:
        shutil.copytree(asset_dir, copied)
        proc = subprocess.run(
            [sys.executable, "-c", COMPILE_CODE, str(copied / "model.py")],
            cwd=TEMPLATE_ROOT,
            env={**os.environ, "PYTHONHASHSEED": "0", "PYTHONPATH": str(TEMPLATE_ROOT)},
            text=True,
            capture_output=True,
            timeout=timeout_s,
        )
        stdout_lines = [line for line in proc.stdout.splitlines() if line.strip()]
        payload: dict[str, Any] = {}
        if stdout_lines:
            try:
                payload = json.loads(stdout_lines[-1])
            except json.JSONDecodeError:
                payload = {"status": "failure", "error": "non-json compiler output"}
        status = str(payload.get("status", "failure")) if proc.returncode == 0 else "failure"
        result: dict[str, Any] = {
            **identity,
            "status": status,
            "elapsed_s": time.monotonic() - started,
            "returncode": proc.returncode,
            "stdout_tail": proc.stdout[-4000:],
            "stderr_tail": proc.stderr[-4000:],
            "error": payload.get("error"),
            "warnings": payload.get("warnings", []),
        }
        if status == "success" and isinstance(payload.get("urdf_xml"), str):
            output_dir = output_root / "reexecuted" / asset_id
            output_dir.mkdir(parents=True, exist_ok=True)
            generated_urdf = output_dir / "model.urdf"
            generated_urdf.write_text(payload["urdf_xml"], encoding="utf-8")
            report_payload = {key: value for key, value in payload.items() if key != "urdf_xml"}
            report_payload["generated_urdf"] = str(generated_urdf)
            (output_dir / "reexecute_report.json").write_text(
                json.dumps(report_payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
            )
            result["artifact_saved"] = generated_urdf.exists()
            result["generated_urdf"] = str(generated_urdf)
            result["generated_urdf_bytes"] = generated_urdf.stat().st_size
        else:
            result["artifact_saved"] = False
        return result
    except subprocess.TimeoutExpired as exc:
        return {
            **identity,
            "status": "timeout",
            "elapsed_s": time.monotonic() - started,
            "returncode": None,
            "artifact_saved": False,
            "error": f"timeout after {timeout_s}s",
            "stdout_tail": (exc.stdout or "")[-4000:] if isinstance(exc.stdout, str) else "",
            "stderr_tail": (exc.stderr or "")[-4000:] if isinstance(exc.stderr, str) else "",
        }
    except Exception as exc:  # noqa: BLE001
        return {
            **identity,
            "status": "harness_error",
            "elapsed_s": time.monotonic() - started,
            "returncode": None,
            "artifact_saved": False,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }
    finally:
        shutil.rmtree(work_root, ignore_errors=True)


def aggregate(records: list[dict[str, Any]], reexec: list[dict[str, Any]]) -> dict[str, Any]:
    def mean(key: str, rows: list[dict[str, Any]] | None = None) -> float | None:
        values = [row[key] for row in (rows or records) if isinstance(row.get(key), (int, float))]
        return sum(values) / len(values) if values else None

    static_valid = sum(bool(row.get("valid_urdf")) for row in records)
    valid_tree = sum(bool(row.get("valid_tree")) for row in records)
    package = sum(bool(row.get("package_complete")) for row in records)
    reexec_success = sum(row.get("status") == "success" for row in reexec)
    artifact_saved = sum(bool(row.get("artifact_saved")) for row in reexec)
    mesh_readable = sum(int(row.get("readable_mesh_files", 0)) for row in records)
    mesh_watertight = sum(int(row.get("watertight_mesh_files", 0)) for row in records)
    mesh_winding = sum(int(row.get("winding_consistent_mesh_files", 0)) for row in records)
    mesh_total = sum(int(row.get("readable_mesh_files", 0)) for row in records)
    physics_validated = sum(row.get("physics_validation") == "validated" for row in records)
    return {
        "asset_count": len(records),
        "source_counts": dict(Counter(row.get("source") for row in records)),
        "difficulty_counts": dict(Counter(row.get("difficulty") for row in records)),
        "preexisting_compile_success": sum(row.get("compile_status") == "success" for row in records),
        "static_valid_urdf": static_valid,
        "valid_tree": valid_tree,
        "package_complete": package,
        "raw_name_coverage_mean": mean("raw_name_coverage"),
        "joint_metadata_rate_mean": mean("joint_metadata_rate"),
        "links_mean": mean("links"),
        "joints_mean": mean("joints"),
        "movable_joints_mean": mean("movable_joints"),
        "source_loc_total": sum(int(row.get("source_loc", 0)) for row in records),
        "source_bytes_total": sum(int(row.get("source_bytes", 0)) for row in records),
        "urdf_bytes_total": sum(int(row.get("urdf_bytes", 0)) for row in records),
        "mesh_bytes_total": sum(int(row.get("mesh_bytes", 0)) for row in records),
        "mesh_readable_files": mesh_readable,
        "mesh_watertight_rate": mesh_watertight / mesh_total if mesh_total else None,
        "mesh_winding_consistent_rate": mesh_winding / mesh_total if mesh_total else None,
        "open_edges_total": sum(int(row.get("open_edges", 0)) for row in records),
        "degenerate_faces_total": sum(int(row.get("degenerate_faces", 0)) for row in records),
        "reexec_success": reexec_success,
        "reexec_artifact_saved": artifact_saved,
        "reexec_wall_time_s": sum(float(row.get("elapsed_s", 0.0)) for row in reexec),
        "physics_validated": physics_validated,
        "physics_records": sum(row.get("source") == "seed_exports_physics_10" for row in records),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--no-reexecute", action="store_true")
    args = parser.parse_args()
    output_root = args.output.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "work").mkdir(parents=True, exist_ok=True)
    paths = read_asset_paths()
    records = [static_record(path) for path in paths]
    manifest = []
    for record in records:
        asset_dir = Path(record["asset_path"])
        manifest.append(
            {
                **{key: record.get(key) for key in ("asset_id", "slug", "seed", "source", "difficulty", "asset_path")},
                "model_py": str(asset_dir / "model.py"),
                "model_urdf": str(asset_dir / "model.urdf"),
                "assets_dir": str(asset_dir / "assets"),
                "compile_report": str(asset_dir / "compile_report.json"),
                "model_py_sha256": sha256_file(asset_dir / "model.py") if (asset_dir / "model.py").exists() else None,
                "model_urdf_sha256": sha256_file(asset_dir / "model.urdf") if (asset_dir / "model.urdf").exists() else None,
                "compile_report_sha256": sha256_file(asset_dir / "compile_report.json") if (asset_dir / "compile_report.json").exists() else None,
            }
        )
    (output_root / "asset_manifest.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n" for row in manifest), encoding="utf-8"
    )
    (output_root / "static_records.json").write_text(
        json.dumps(records, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )
    reexec: list[dict[str, Any]] = []
    if not args.no_reexecute:
        for index, path in enumerate(paths, 1):
            print(f"[{index}/{len(paths)}] reexecute {path}", flush=True)
            reexec.append(reexecute_one(path, output_root, args.timeout))
    (output_root / "reexecution_records.json").write_text(
        json.dumps(reexec, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )
    summary = aggregate(records, reexec)
    summary["protocol"] = "nano3d_existing_export_pilot_v1"
    summary["source_document"] = str(ASSET_DOC)
    summary["reexecution_performed"] = not args.no_reexecute
    (output_root / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True))
    print(f"outputs={output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
