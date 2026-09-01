#!/usr/bin/env python3
"""Run and audit the frozen Infinite Mobility 20-factory seed cohort."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import os
import shutil
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PROTOCOL = REPO_ROOT / "exp/reference/infinite_mobility_protocol_v1.json"
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/infinite_mobility_v1"
DEFAULT_BASELINE = REPO_ROOT / ".cache/Infinite-Mobility"
DEFAULT_BLENDER = REPO_ROOT / ".cache/blender-3.6.0-linux-x64/blender"
DEFAULT_PARTS = DEFAULT_OUTPUT / "inputs/parts"
WORKER = REPO_ROOT / "exp/scripts/infinite_mobility_blender_worker.py"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def source_tree_sha256(root: Path) -> str:
    digest = hashlib.sha256()
    files = sorted(
        path for path in root.rglob("*.py") if "__pycache__" not in path.parts
    )
    for path in files:
        relative = path.relative_to(root).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def resolve_mesh_path(urdf_path: Path, filename: str) -> Path:
    candidate = Path(filename)
    if candidate.is_absolute():
        return candidate
    urdf_relative = urdf_path.parent / candidate
    if urdf_relative.exists():
        return urdf_relative
    return REPO_ROOT / candidate


def validate_package(case_dir: Path, process_exit_code: int | None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "process_exit_code": process_exit_code,
        "process_exit_zero": process_exit_code == 0,
        "urdf_count": 0,
        "valid_urdf": False,
        "valid_tree": False,
        "mesh_references_valid": False,
        "portable_relative_paths": False,
        "link_count": 0,
        "joint_count": 0,
        "movable_joint_count": 0,
        "visual_count": 0,
        "collision_count": 0,
        "mesh_reference_count": 0,
        "errors": [],
    }
    urdfs = sorted(case_dir.rglob("scene.urdf")) if case_dir.exists() else []
    result["urdf_count"] = len(urdfs)
    if len(urdfs) != 1:
        result["errors"].append(f"expected one scene.urdf, found {len(urdfs)}")
        return result

    urdf_path = urdfs[0]
    result["urdf_path"] = str(urdf_path.relative_to(case_dir))
    try:
        robot = ET.parse(urdf_path).getroot()
    except (ET.ParseError, OSError) as exc:
        result["errors"].append(f"URDF parse failed: {exc}")
        return result
    if robot.tag != "robot":
        result["errors"].append(f"root tag is {robot.tag!r}, expected 'robot'")
        return result

    links = [node.get("name", "") for node in robot.findall("link")]
    joints = robot.findall("joint")
    result["link_count"] = len(links)
    result["joint_count"] = len(joints)
    result["movable_joint_count"] = sum(
        node.get("type") not in {None, "fixed"} for node in joints
    )
    result["visual_count"] = len(robot.findall(".//visual"))
    result["collision_count"] = len(robot.findall(".//collision"))
    if not links or any(not name for name in links) or len(set(links)) != len(links):
        result["errors"].append("links are empty, unnamed, or duplicated")
    else:
        result["valid_urdf"] = True

    parents: dict[str, list[str]] = {name: [] for name in links}
    indegree: Counter[str] = Counter()
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = parent.get("link", "") if parent is not None else ""
        child_name = child.get("link", "") if child is not None else ""
        if parent_name not in parents or child_name not in parents:
            result["errors"].append(
                f"joint {joint.get('name', '')!r} has an unknown endpoint"
            )
            continue
        parents[parent_name].append(child_name)
        indegree[child_name] += 1

    roots = [name for name in links if indegree[name] == 0]
    if len(roots) == 1 and all(indegree[name] <= 1 for name in links):
        visited: set[str] = set()
        active: set[str] = set()

        def walk(node: str) -> bool:
            if node in active:
                return False
            if node in visited:
                return True
            active.add(node)
            for child_name in parents[node]:
                if not walk(child_name):
                    return False
            active.remove(node)
            visited.add(node)
            return True

        result["valid_tree"] = walk(roots[0]) and len(visited) == len(links)
    if not result["valid_tree"]:
        result["errors"].append(
            f"link graph is not a single-root connected tree (roots={len(roots)})"
        )

    mesh_nodes = robot.findall(".//mesh")
    result["mesh_reference_count"] = len(mesh_nodes)
    mesh_errors: list[str] = []
    relative_only = True
    for node in mesh_nodes:
        filename = node.get("filename", "")
        relative_only &= bool(filename) and not Path(filename).is_absolute()
        mesh_path = resolve_mesh_path(urdf_path, filename) if filename else urdf_path
        try:
            mesh_path.resolve().relative_to(case_dir.resolve())
        except ValueError:
            mesh_errors.append(f"mesh escapes case directory: {filename}")
            continue
        if not mesh_path.is_file() or mesh_path.stat().st_size == 0:
            mesh_errors.append(f"missing or empty mesh: {filename}")
    result["mesh_references_valid"] = bool(mesh_nodes) and not mesh_errors
    result["portable_relative_paths"] = relative_only and bool(mesh_nodes)
    result["errors"].extend(mesh_errors[:20])
    result["strict_pass"] = all(
        result[key]
        for key in (
            "process_exit_zero",
            "valid_urdf",
            "valid_tree",
            "mesh_references_valid",
        )
    )
    return result


def package_sha256(case_dir: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in case_dir.rglob("*") if item.is_file()):
        if path.name in {"stdout.log", "stderr.log", "record.json"}:
            continue
        relative = path.relative_to(case_dir).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(bytes.fromhex(sha256_file(path)))
    return digest.hexdigest()


def make_command(
    args: argparse.Namespace,
    factory: str,
    seed: int,
    staging_relative: Path,
) -> list[str]:
    return [
        str(args.blender),
        "--background",
        "--factory-startup",
        "--python",
        str(WORKER),
        "--",
        "--baseline-repo",
        str(args.baseline_repo),
        "--parts-root",
        str(args.parts_root),
        "--output-dir",
        str(staging_relative),
        "--factory",
        factory,
        "--seed",
        str(seed),
        "--texture-resolution",
        str(args.texture_resolution),
    ]


def archive_existing_attempt(case_dir: Path) -> None:
    existing = [
        case_dir / name
        for name in ("package", "stdout.log", "stderr.log", "record.json")
        if (case_dir / name).exists()
    ]
    if not existing:
        return
    attempts = case_dir / "attempts"
    attempts.mkdir(parents=True, exist_ok=True)
    attempt_index = 1
    while (attempts / f"attempt_{attempt_index:03d}").exists():
        attempt_index += 1
    archive = attempts / f"attempt_{attempt_index:03d}"
    archive.mkdir()
    for path in existing:
        shutil.move(str(path), str(archive / path.name))


def normalize_urdf_mesh_paths(package_dir: Path) -> None:
    urdfs = sorted(package_dir.rglob("scene.urdf"))
    if len(urdfs) != 1:
        return
    urdf_path = urdfs[0]
    tree = ET.parse(urdf_path)
    changed = False
    source_obj_roots: set[Path] = set()
    for node in tree.getroot().findall(".//mesh"):
        filename = node.get("filename", "")
        parts = Path(filename).parts
        if "objs" not in parts:
            continue
        obj_index = parts.index("objs")
        if not Path(filename).is_absolute():
            source_obj_roots.add(urdf_path.parent / Path(*parts[: obj_index + 1]))
        normalized = Path(*parts[obj_index:]).as_posix()
        if filename != normalized:
            node.set("filename", normalized)
            changed = True
    target_objs = urdf_path.parent / "objs"
    for source_objs in sorted(source_obj_roots):
        if source_objs == target_objs or not source_objs.is_dir():
            continue
        target_objs.mkdir(parents=True, exist_ok=True)
        for child in sorted(source_objs.iterdir()):
            destination = target_objs / child.name
            if destination.exists():
                raise FileExistsError(
                    f"object-root collision for {urdf_path}: {child.name}"
                )
            shutil.move(str(child), str(destination))
    if changed:
        ET.indent(tree, space="  ")
        tree.write(urdf_path, encoding="utf-8", xml_declaration=True)


def run_case(args: argparse.Namespace, factory: str, seed: int) -> dict[str, Any]:
    case_dir = args.output_root / "cases" / factory / f"seed_{seed:03d}"
    case_dir.mkdir(parents=True, exist_ok=True)
    record_path = case_dir / "record.json"
    if args.resume and record_path.is_file():
        previous = json.loads(record_path.read_text())
        validation = validate_package(case_dir / "package", 0)
        if previous.get("status") == "PASS" and validation.get("strict_pass"):
            previous["resumed"] = True
            return previous
        terminal_failure = (
            previous.get("status") in {"FAIL", "TIMEOUT"}
            and previous.get("protocol_terminal") is True
        )
        legacy_full_timeout = (
            previous.get("status") == "TIMEOUT"
            and previous.get("timed_out") is True
            and float(previous.get("elapsed_seconds", 0.0)) >= args.timeout * 0.95
        )
        if terminal_failure or legacy_full_timeout:
            previous["resumed"] = True
            return previous
    archive_existing_attempt(case_dir)
    staging_relative = Path(".nano3d_runs") / factory / f"seed_{seed:03d}"
    staging_dir = args.baseline_repo / staging_relative
    if staging_dir.exists():
        orphan_dir = case_dir / "attempts" / f"orphan_staging_{int(time.time())}"
        orphan_dir.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(staging_dir), str(orphan_dir))

    env = os.environ.copy()
    env.update(
        {
            "PYTHONPATH": str(args.baseline_repo),
            "BLENDER_USER_CONFIG": str(args.blender_user_root / "config"),
            "BLENDER_USER_SCRIPTS": str(args.blender_user_root / "scripts"),
            "BLENDER_USER_DATAFILES": str(args.blender_user_root / "datafiles"),
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
            "CUDA_VISIBLE_DEVICES": "",
        }
    )
    started_at = utc_now()
    start = time.monotonic()
    exit_code: int | None = None
    timed_out = False
    stdout = ""
    stderr = ""
    try:
        completed = subprocess.run(
            make_command(args, factory, seed, staging_relative),
            cwd=args.baseline_repo,
            env=env,
            text=True,
            capture_output=True,
            timeout=args.timeout,
            check=False,
        )
        exit_code = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", errors="replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", errors="replace")
    elapsed = time.monotonic() - start
    if staging_dir.exists():
        shutil.move(str(staging_dir), str(case_dir / "package"))
        normalize_urdf_mesh_paths(case_dir / "package")
    (case_dir / "stdout.log").write_text(stdout, encoding="utf-8")
    (case_dir / "stderr.log").write_text(stderr, encoding="utf-8")

    validation = validate_package(case_dir / "package", exit_code)
    status = "TIMEOUT" if timed_out else "PASS" if validation.get("strict_pass") else "FAIL"
    record: dict[str, Any] = {
        "factory": factory,
        "seed": seed,
        "status": status,
        "started_at": started_at,
        "finished_at": utc_now(),
        "elapsed_seconds": elapsed,
        "timed_out": timed_out,
        "protocol_terminal": True,
        "worker_concurrency": args.workers,
        "texture_resolution": args.texture_resolution,
        "validation": validation,
    }
    if validation.get("strict_pass"):
        record["package_sha256"] = package_sha256(case_dir / "package")
    record_path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    return record


def write_summary(args: argparse.Namespace, protocol: dict[str, Any], records: list[dict[str, Any]]) -> dict[str, Any]:
    statuses = Counter(record["status"] for record in records)
    factories = sorted({record["factory"] for record in records})
    seeds = sorted({record["seed"] for record in records})
    strict_passes = sum(record["status"] == "PASS" for record in records)
    all_seed_pass = sum(
        all(
            any(
                item["factory"] == factory
                and item["seed"] == seed
                and item["status"] == "PASS"
                for item in records
            )
            for seed in seeds
        )
        for factory in factories
    )
    validation_records = [record["validation"] for record in records]
    resumed_case_count = sum(record.get("resumed") is True for record in records)
    case_elapsed_sum = sum(record["elapsed_seconds"] for record in records)
    summary = {
        "protocol_id": protocol["protocol_id"],
        "generated_at": utc_now(),
        "case_count": len(records),
        "factory_count": len(factories),
        "seeds": seeds,
        "status_counts": dict(sorted(statuses.items())),
        "strict_pass_count": strict_passes,
        "strict_pass_rate": strict_passes / len(records) if records else None,
        "all_seed_pass_factories": all_seed_pass,
        "all_seed_pass_rate": all_seed_pass / len(factories) if factories else None,
        "movable_joint_total": sum(item["movable_joint_count"] for item in validation_records),
        "collision_total": sum(item["collision_count"] for item in validation_records),
        "portable_relative_path_count": sum(item["portable_relative_paths"] for item in validation_records),
        "mean_elapsed_seconds": (
            case_elapsed_sum / len(records)
            if records
            else None
        ),
        "case_elapsed_sum_seconds": case_elapsed_sum,
        "resumed_case_count": resumed_case_count,
        "fresh_case_count": len(records) - resumed_case_count,
        "invocation_wall_seconds": args.wall_seconds,
        "wall_time_scope": "current_resume_invocation_only",
    }
    (args.output_root / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    report = [
        "# Infinite Mobility baseline report",
        "",
        f"- Protocol: `{summary['protocol_id']}`",
        f"- Cases: {strict_passes}/{len(records)} strict package QC pass",
        f"- Factories: {all_seed_pass}/{len(factories)} pass every requested seed",
        f"- Statuses: `{summary['status_counts']}`",
        f"- Movable joints: {summary['movable_joint_total']}",
        f"- Collision elements: {summary['collision_total']}",
        f"- Relative-path portable packages: {summary['portable_relative_path_count']}/{len(records)}",
        f"- Sum of per-case elapsed time: {summary['case_elapsed_sum_seconds']:.2f} s",
        (
            "- Current resume invocation wall time: "
            f"{summary['invocation_wall_seconds']:.2f} s "
            f"({summary['resumed_case_count']} reused; {summary['fresh_case_count']} executed)"
        ),
        "",
        "This is the frozen public-factory supplementary cohort. It is not a common-category matched result.",
    ]
    (args.output_root / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--baseline-repo", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument("--parts-root", type=Path, default=DEFAULT_PARTS)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--timeout", type=int, default=None)
    parser.add_argument("--texture-resolution", type=int, default=None)
    parser.add_argument("--factory", action="append", default=[])
    parser.add_argument("--seed", action="append", type=int, default=[])
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--resume", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    for path in (args.protocol, args.blender, WORKER):
        if not path.is_file():
            raise FileNotFoundError(path)
    for path in (args.baseline_repo, args.parts_root):
        if not path.is_dir():
            raise FileNotFoundError(path)
    if args.workers < 1:
        raise ValueError("--workers must be positive")

    protocol = json.loads(args.protocol.read_text())
    factories = args.factory or protocol["factories"]
    seeds = args.seed or protocol["seeds"]
    if args.smoke:
        factories, seeds = ["LiteDoorFactory"], [0]
    args.timeout = args.timeout or protocol["case_timeout_seconds"]
    args.texture_resolution = (
        args.texture_resolution or protocol["adapter"]["texture_resolution"]
    )
    args.output_root.mkdir(parents=True, exist_ok=True)
    args.blender_user_root = args.output_root / ".blender_user"
    for subdir in ("config", "scripts", "datafiles"):
        (args.blender_user_root / subdir).mkdir(parents=True, exist_ok=True)

    manifest = {
        "protocol": protocol,
        "run_started_at": utc_now(),
        "factories": factories,
        "seeds": seeds,
        "workers": args.workers,
        "timeout_seconds": args.timeout,
        "texture_resolution": args.texture_resolution,
        "baseline_source_tree_sha256": source_tree_sha256(args.baseline_repo),
        "blender_sha256": sha256_file(args.blender),
        "parts_file_count": sum(1 for item in args.parts_root.rglob("*") if item.is_file()),
    }
    (args.output_root / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    tasks = [(factory, seed) for factory in factories for seed in seeds]
    records: list[dict[str, Any]] = []
    records_lock = threading.Lock()
    wall_start = time.monotonic()
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {
            pool.submit(run_case, args, factory, seed): (factory, seed)
            for factory, seed in tasks
        }
        for future in concurrent.futures.as_completed(futures):
            factory, seed = futures[future]
            try:
                record = future.result()
            except Exception as exc:
                record = {
                    "factory": factory,
                    "seed": seed,
                    "status": "HARNESS_ERROR",
                    "elapsed_seconds": 0.0,
                    "validation": {
                        "movable_joint_count": 0,
                        "collision_count": 0,
                        "portable_relative_paths": False,
                        "errors": [repr(exc)],
                    },
                }
            with records_lock:
                records.append(record)
                ordered = sorted(records, key=lambda item: (item["factory"], item["seed"]))
                (args.output_root / "records.json").write_text(
                    json.dumps(ordered, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
            print(f"[{len(records)}/{len(tasks)}] {factory} seed={seed}: {record['status']}", flush=True)

    args.wall_seconds = time.monotonic() - wall_start
    records.sort(key=lambda item: (item["factory"], item["seed"]))
    summary = write_summary(args, protocol, records)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["strict_pass_count"] == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
