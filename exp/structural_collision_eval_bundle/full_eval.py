#!/usr/bin/env python3
"""Parallel, resumable structural-integrity and collision evaluation."""

from __future__ import annotations

import argparse
from collections import defaultdict
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

try:
    import structural_integrity_eval as structural
except ModuleNotFoundError:
    repository_script_dir = Path(__file__).resolve().parents[1] / "scripts"
    sys.path.insert(0, str(repository_script_dir))
    import structural_integrity_eval as structural


SCHEMA_VERSION = "articulated_integrity_asset_record_v1"
PROTOCOL_ID = "articulated-structural-collision-fk-v1"


def _canonical_hash(value: Any) -> str:
    payload = json.dumps(
        value, ensure_ascii=True, allow_nan=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=True, allow_nan=False, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _read_manifest(path: Path) -> list[dict[str, Any]]:
    rows = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    seen: set[tuple[str, str]] = set()
    for row in rows:
        identity = (str(row.get("dataset_slug", "")), str(row.get("asset_id", "")))
        if not all(identity) or not row.get("urdf_path"):
            raise ValueError(f"invalid manifest row: {row}")
        if identity in seen:
            raise ValueError(f"duplicate dataset_slug/asset_id: {identity}")
        seen.add(identity)
    return rows


def _resolve_path(raw: str, manifest_dir: Path) -> Path:
    path = Path(raw).expanduser()
    return (path if path.is_absolute() else manifest_dir / path).resolve()


def _result_path(out: Path, row: Mapping[str, Any]) -> Path:
    identity = {
        "dataset_slug": row["dataset_slug"],
        "asset_id": row["asset_id"],
        "urdf_path": row["urdf_path"],
    }
    digest = _canonical_hash(identity)[:24]
    safe_slug = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(row["dataset_slug"]))
    return out / "records" / safe_slug / f"{digest}.json"


def _compact_structural(result: dict[str, Any]) -> dict[str, Any]:
    pose_swept = result.get("pose_swept")
    if not isinstance(pose_swept, dict):
        return result
    compact_rows: list[dict[str, Any]] = []
    for row in pose_swept.get("rows", []):
        compact_rows.append(
            {
                key: row.get(key)
                for key in (
                    "joint",
                    "pose_index",
                    "q",
                    "passed",
                    "support_pass",
                    "gap_pass",
                    "support_gap_passed",
                    "axis_support_pass",
                    "status",
                    "reason",
                )
                if row.get(key) is not None
            }
        )
    pose_swept["rows"] = compact_rows
    evaluated = [row for row in compact_rows if row.get("status") != "error"]
    pose_swept["axis_pose_rate"] = (
        sum(bool(row.get("axis_support_pass")) for row in evaluated) / len(evaluated)
        if evaluated
        else None
    )
    by_joint: dict[str, list[bool]] = defaultdict(list)
    for row in evaluated:
        by_joint[str(row.get("joint"))].append(bool(row.get("axis_support_pass")))
    joint_rates = [sum(values) / len(values) for values in by_joint.values() if values]
    pose_swept["axis_pose_joint_macro_rate"] = (
        sum(joint_rates) / len(joint_rates) if joint_rates else None
    )
    return result


def _evaluate_structural(
    urdf_path: Path,
    package_root: Path,
    settings: Mapping[str, Any],
    asset_seed: int,
) -> dict[str, Any]:
    config = structural.Config(
        geometry_role=str(settings.get("geometry_role", "visual")),
        roi_fraction=float(settings.get("roi_fraction", 0.02)),
        gap_fraction=float(settings.get("gap_fraction", 0.002)),
        support_threshold=float(settings.get("support_threshold", 0.10)),
        pose_samples=int(settings.get("pose_samples", 9)),
        surface_samples=int(settings.get("surface_samples", 128)),
        skip_penetration=True,
    )
    asset = structural._parse_asset(urdf_path, package_root, config.geometry_role)
    return _compact_structural(structural._evaluate_asset(asset, config, asset_seed))


def _pair(left: str, right: str) -> tuple[str, str]:
    return tuple(sorted((left, right)))


def _fixed_clusters(asset: structural.AssetModel) -> dict[str, str]:
    parent = {name: name for name in asset.links}

    def find(name: str) -> str:
        while parent[name] != name:
            parent[name] = parent[parent[name]]
            name = parent[name]
        return name

    def union(left: str, right: str) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    for joint in asset.joints:
        if joint.joint_type == "fixed":
            union(joint.parent, joint.child)
    return {name: find(name) for name in asset.links}


def _descendants(asset: structural.AssetModel, root: str) -> set[str]:
    children: dict[str, list[str]] = defaultdict(list)
    for joint in asset.joints:
        children[joint.parent].append(joint.child)
    result: set[str] = set()
    stack = [root]
    while stack:
        name = stack.pop()
        if name in result:
            continue
        result.add(name)
        stack.extend(children.get(name, []))
    return result


def _eligible_collision_joints(asset: structural.AssetModel) -> list[structural.Joint]:
    return [
        joint
        for joint in asset.joints
        if joint.joint_type in {"revolute", "prismatic"}
        and structural._finite(joint.lower)
        and structural._finite(joint.upper)
        and float(joint.lower) < float(joint.upper)
    ]


def _crossing_pairs(
    asset: structural.AssetModel,
    joint: structural.Joint,
    clusters: Mapping[str, str],
) -> list[tuple[str, str]]:
    moving = _descendants(asset, joint.child)
    stationary = set(asset.links) - moving
    return sorted(
        {
            _pair(left, right)
            for left in moving
            for right in stationary
            if clusters[left] != clusters[right]
        }
    )


def _fcl_objects(asset: structural.AssetModel) -> dict[str, Any]:
    import fcl

    objects: dict[str, Any] = {}
    for name, geometry in asset.links.items():
        mesh = geometry.mesh
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        faces = np.asarray(mesh.faces, dtype=np.int32)
        model = fcl.BVHModel()
        model.beginModel(len(vertices), len(faces))
        model.addSubModel(vertices, faces)
        model.endModel()
        objects[name] = fcl.CollisionObject(model)
    return objects


def _set_fcl_transforms(objects: Mapping[str, Any], transforms: Mapping[str, np.ndarray]) -> None:
    import fcl

    for name, obj in objects.items():
        transform = transforms[name]
        obj.setTransform(fcl.Transform(transform[:3, :3], transform[:3, 3]))


def _local_bound_corners(asset: structural.AssetModel) -> dict[str, np.ndarray]:
    corners: dict[str, np.ndarray] = {}
    for name, geometry in asset.links.items():
        low, high = np.asarray(geometry.mesh.bounds, dtype=np.float64)
        corners[name] = np.asarray(
            [[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])],
            dtype=np.float64,
        )
    return corners


def _world_bounds(
    corners: Mapping[str, np.ndarray], transforms: Mapping[str, np.ndarray]
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    result: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for name, points in corners.items():
        transform = transforms[name]
        world = points @ transform[:3, :3].T + transform[:3, 3]
        result[name] = (world.min(axis=0), world.max(axis=0))
    return result


def _bounds_overlap(
    left: tuple[np.ndarray, np.ndarray], right: tuple[np.ndarray, np.ndarray]
) -> bool:
    return bool(np.all(left[1] >= right[0]) and np.all(right[1] >= left[0]))


def _contacts(left: Any, right: Any, max_contacts: int) -> list[dict[str, Any]]:
    import fcl

    request = fcl.CollisionRequest(num_max_contacts=max_contacts, enable_contact=True)
    result = fcl.CollisionResult()
    fcl.collide(left, right, request, result)
    return [
        {
            "position": np.asarray(contact.pos, dtype=np.float64),
            "depth_m": max(0.0, float(contact.penetration_depth)),
        }
        for contact in result.contacts
    ]


def _joint_axis_world(
    joint: structural.Joint, transforms: Mapping[str, np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    frame = transforms[joint.parent] @ joint.origin
    origin = frame[:3, 3]
    axis = frame[:3, :3] @ joint.axis
    return origin, axis / np.linalg.norm(axis)


def _axis_half_length(
    asset: structural.AssetModel,
    joint: structural.Joint,
    transforms: Mapping[str, np.ndarray],
    origin: np.ndarray,
    axis: np.ndarray,
) -> float:
    maximum = 0.0
    for name in (joint.parent, joint.child):
        mesh = asset.links[name].mesh
        transform = transforms[name]
        world = np.asarray(mesh.vertices, dtype=np.float64) @ transform[:3, :3].T + transform[:3, 3]
        projections = (world - origin) @ axis
        maximum = max(maximum, float(np.max(np.abs(projections))))
    return max(maximum, 1e-9)


def _outside_joint_roi_depth(
    contacts: Sequence[Mapping[str, Any]],
    *,
    asset: structural.AssetModel,
    joint: structural.Joint,
    pair: tuple[str, str],
    transforms: Mapping[str, np.ndarray],
    radius_m: float,
) -> tuple[float, int, int]:
    direct = pair == _pair(joint.parent, joint.child)
    if not direct:
        return max((float(contact["depth_m"]) for contact in contacts), default=0.0), 0, len(contacts)
    origin, axis = _joint_axis_world(joint, transforms)
    half_length = _axis_half_length(asset, joint, transforms, origin, axis)
    retained: list[float] = []
    masked = 0
    for contact in contacts:
        relative = np.asarray(contact["position"], dtype=np.float64) - origin
        axial = float(relative @ axis)
        radial = float(np.linalg.norm(relative - axial * axis))
        if radial <= radius_m and abs(axial) <= half_length:
            masked += 1
        else:
            retained.append(float(contact["depth_m"]))
    return max(retained, default=0.0), masked, len(retained)


def _asset_diagonal(
    corners: Mapping[str, np.ndarray], transforms: Mapping[str, np.ndarray]
) -> float:
    bounds = _world_bounds(corners, transforms)
    low = np.min(np.vstack([value[0] for value in bounds.values()]), axis=0)
    high = np.max(np.vstack([value[1] for value in bounds.values()]), axis=0)
    diagonal = float(np.linalg.norm(high - low))
    if not math.isfinite(diagonal) or diagonal <= 0:
        raise ValueError("asset q=0 diagonal must be positive and finite")
    return diagonal


def _evaluate_collision(
    urdf_path: Path,
    package_root: Path,
    settings: Mapping[str, Any],
) -> dict[str, Any]:
    geometry_role = str(settings.get("geometry_role", "collision"))
    samples = int(settings.get("pose_samples", 9))
    if samples < 3:
        raise ValueError("collision pose_samples must be at least 3")
    absolute_fraction = float(settings.get("absolute_fraction", 0.002))
    growth_fraction = float(settings.get("growth_fraction", 0.002))
    roi_fraction = float(settings.get("joint_contact_roi_radius_fraction", 0.02))
    max_contacts = int(settings.get("max_contacts", 128))
    asset = structural._parse_asset(urdf_path, package_root, geometry_role)
    if any(geometry.mesh is None for geometry in asset.links.values()):
        raise ValueError("one or more links have no collision/visual-fallback geometry")
    q0 = structural._fk(asset, {})
    corners = _local_bound_corners(asset)
    diagonal = _asset_diagonal(corners, q0)
    absolute_m = absolute_fraction * diagonal
    growth_m = growth_fraction * diagonal
    roi_m = roi_fraction * diagonal
    objects = _fcl_objects(asset)
    clusters = _fixed_clusters(asset)
    eligible = _eligible_collision_joints(asset)
    declared_movable = sum(joint.joint_type != "fixed" for joint in asset.joints)
    joint_rows: list[dict[str, Any]] = []
    for joint in eligible:
        pairs = _crossing_pairs(asset, joint, clusters)
        _set_fcl_transforms(objects, q0)
        q0_bounds = _world_bounds(corners, q0)
        baseline: dict[tuple[str, str], float] = {}
        for left, right in pairs:
            if not _bounds_overlap(q0_bounds[left], q0_bounds[right]):
                baseline[(left, right)] = 0.0
                continue
            contact_rows = _contacts(objects[left], objects[right], max_contacts)
            depth, _, _ = _outside_joint_roi_depth(
                contact_rows,
                asset=asset,
                joint=joint,
                pair=(left, right),
                transforms=q0,
                radius_m=roi_m,
            )
            baseline[(left, right)] = depth
        pose_rows: list[dict[str, Any]] = []
        values = np.linspace(float(joint.lower), float(joint.upper), samples).tolist()
        for pose_index, q_value in enumerate(values):
            transforms = structural._fk(asset, {joint.name: float(q_value)})
            _set_fcl_transforms(objects, transforms)
            bounds = _world_bounds(corners, transforms)
            events: list[dict[str, Any]] = []
            masked_contacts = 0
            retained_contacts = 0
            max_growth = 0.0
            for left, right in pairs:
                if not _bounds_overlap(bounds[left], bounds[right]):
                    continue
                contact_rows = _contacts(objects[left], objects[right], max_contacts)
                depth, masked, retained = _outside_joint_roi_depth(
                    contact_rows,
                    asset=asset,
                    joint=joint,
                    pair=(left, right),
                    transforms=transforms,
                    radius_m=roi_m,
                )
                masked_contacts += masked
                retained_contacts += retained
                base = baseline[(left, right)]
                growth = max(0.0, depth - base)
                max_growth = max(max_growth, growth)
                if depth > absolute_m and growth > growth_m:
                    events.append(
                        {
                            "left": left,
                            "right": right,
                            "direct_parent_child": (left, right) == _pair(joint.parent, joint.child),
                            "depth_fraction": depth / diagonal,
                            "baseline_depth_fraction": base / diagonal,
                            "growth_fraction": growth / diagonal,
                        }
                    )
            pose_rows.append(
                {
                    "pose_index": pose_index,
                    "q": float(q_value),
                    "normalized_q": pose_index / (samples - 1),
                    "collision_free": not events,
                    "max_growth_fraction": max_growth / diagonal,
                    "masked_joint_roi_contacts": masked_contacts,
                    "retained_contacts": retained_contacts,
                    "events": events,
                }
            )
        free = [bool(row["collision_free"]) for row in pose_rows]
        valid_intervals = sum(free[index] and free[index + 1] for index in range(samples - 1))
        motion_range_rate = valid_intervals / (samples - 1)
        premature_free = all(free[1:-1])
        joint_rows.append(
            {
                "joint": joint.name,
                "joint_type": joint.joint_type,
                "lower": joint.lower,
                "upper": joint.upper,
                "crossing_pair_count": len(pairs),
                "collision_free_motion_range_rate": motion_range_rate,
                "premature_collision_free": premature_free,
                "max_penetration_growth_fraction": max(
                    (float(row["max_growth_fraction"]) for row in pose_rows), default=0.0
                ),
                "pose_rows": pose_rows,
            }
        )
    ranges = [float(row["collision_free_motion_range_rate"]) for row in joint_rows]
    premature = [bool(row["premature_collision_free"]) for row in joint_rows]
    growths = [float(row["max_penetration_growth_fraction"]) for row in joint_rows]
    return {
        "status": "complete" if joint_rows else "not_evaluable",
        "reason": None if joint_rows else "no_bounded_revolute_or_prismatic_joint",
        "geometry_role": geometry_role,
        "asset_diagonal_m": diagonal,
        "declared_movable_joints": declared_movable,
        "eligible_bounded_joints": len(joint_rows),
        "eligible_joint_coverage": len(joint_rows) / declared_movable if declared_movable else None,
        "collision_free_joint_motion_range_rate": sum(ranges) / len(ranges) if ranges else None,
        "premature_collision_free_joint_rate": sum(premature) / len(premature) if premature else None,
        "penetration_growth_p95_fraction": float(np.percentile(growths, 95)) if growths else None,
        "joint_rows": joint_rows,
    }


def _worker(job: Mapping[str, Any]) -> dict[str, Any]:
    row = dict(job["row"])
    output = Path(job["output"])
    signature = str(job["signature"])
    task = str(job["task"])
    started = time.time()
    record: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "run_signature": signature,
        "identity": {
            key: row.get(key)
            for key in ("dataset_slug", "dataset_name", "dataset_id", "asset_id", "category", "split")
            if row.get(key) is not None
        },
        "source": {
            "urdf_path": row["urdf_path"],
            "package_root": row["package_root"],
        },
        "requested_task": task,
    }
    errors: dict[str, str] = {}
    urdf_path = Path(row["urdf_path"])
    package_root = Path(row["package_root"])
    asset_seed = int(
        hashlib.sha256(
            f"{PROTOCOL_ID}|{row['dataset_slug']}|{row['asset_id']}".encode("utf-8")
        ).hexdigest()[:16],
        16,
    )
    if task in {"structural", "both"}:
        try:
            record["structural"] = _evaluate_structural(
                urdf_path, package_root, job["protocol"]["structural"], asset_seed
            )
        except Exception as exc:
            errors["structural"] = f"{type(exc).__name__}: {exc}"
    if task in {"collision", "both"}:
        try:
            record["collision"] = _evaluate_collision(
                urdf_path, package_root, job["protocol"]["collision"]
            )
        except Exception as exc:
            errors["collision"] = f"{type(exc).__name__}: {exc}"
    record["errors"] = errors
    record["status"] = "complete" if not errors else "partial" if len(errors) == 1 and task == "both" else "error"
    record["elapsed_seconds"] = time.time() - started
    _atomic_json(output, record)
    return {
        "status": record["status"],
        "dataset_slug": row["dataset_slug"],
        "asset_id": row["asset_id"],
        "elapsed_seconds": record["elapsed_seconds"],
    }


def _existing_matches(path: Path, signature: str) -> bool:
    if not path.is_file():
        return False
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return False
    return record.get("run_signature") == signature and record.get("status") in {
        "complete",
        "partial",
        "error",
    }


def _run_bounded(jobs: Sequence[Mapping[str, Any]], workers: int) -> Iterable[dict[str, Any]]:
    with ProcessPoolExecutor(max_workers=workers) as executor:
        iterator = iter(jobs)
        pending = set()
        for _ in range(min(len(jobs), max(workers * 3, 1))):
            try:
                pending.add(executor.submit(_worker, next(iterator)))
            except StopIteration:
                break
        while pending:
            done, pending = wait(pending, return_when=FIRST_COMPLETED)
            for future in done:
                yield future.result()
                try:
                    pending.add(executor.submit(_worker, next(iterator)))
                except StopIteration:
                    pass


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--task", choices=("structural", "collision", "both"), default="both")
    parser.add_argument("--datasets", default="all", help="comma-separated slugs or all")
    parser.add_argument("--workers", type=int, default=max(1, min(32, os.cpu_count() or 1)))
    parser.add_argument("--shard-count", type=int, default=1)
    parser.add_argument("--shard-index", type=int, default=0)
    parser.add_argument("--max-assets", type=int, default=None)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    if args.workers < 1 or args.shard_count < 1 or not 0 <= args.shard_index < args.shard_count:
        parser.error("workers/shard-count must be positive and 0 <= shard-index < shard-count")
    manifest_path = args.manifest.resolve()
    manifest_dir = manifest_path.parent
    protocol = json.loads(args.protocol.read_text(encoding="utf-8"))
    selected = None if args.datasets == "all" else {x.strip() for x in args.datasets.split(",") if x.strip()}
    source_rows = _read_manifest(manifest_path)
    rows: list[dict[str, Any]] = []
    for original_index, source in enumerate(source_rows):
        if selected is not None and source["dataset_slug"] not in selected:
            continue
        identity_hash = int(_canonical_hash([source["dataset_slug"], source["asset_id"]])[:16], 16)
        if identity_hash % args.shard_count != args.shard_index:
            continue
        row = dict(source)
        row["urdf_path"] = str(_resolve_path(str(row["urdf_path"]), manifest_dir))
        row["package_root"] = str(_resolve_path(str(row.get("package_root") or Path(row["urdf_path"]).parent), manifest_dir))
        row["manifest_index"] = original_index
        rows.append(row)
    if args.max_assets is not None:
        rows = rows[: args.max_assets]
    signature_payload = {
        "protocol_id": PROTOCOL_ID,
        "protocol": protocol,
        "task": args.task,
        "manifest_sha256": _sha256_file(manifest_path),
        "shard_count": args.shard_count,
    }
    signature = _canonical_hash(signature_payload)
    jobs: list[dict[str, Any]] = []
    skipped = 0
    for row in rows:
        output = _result_path(args.out, row)
        if output.is_file() and not args.overwrite:
            if _existing_matches(output, signature):
                skipped += 1
                continue
            raise RuntimeError(
                f"existing record has a different run signature: {output}; "
                "use a new --out directory or explicitly pass --overwrite"
            )
        jobs.append(
            {
                "row": row,
                "output": str(output),
                "signature": signature,
                "task": args.task,
                "protocol": protocol,
            }
        )
    run_manifest = {
        "schema_version": "articulated_integrity_run_manifest_v1",
        "protocol_id": PROTOCOL_ID,
        "run_signature": signature,
        "task": args.task,
        "manifest_path": str(manifest_path),
        "manifest_sha256": _sha256_file(manifest_path),
        "protocol_path": str(args.protocol.resolve()),
        "protocol_sha256": _sha256_file(args.protocol),
        "selected_rows": len(rows),
        "scheduled_rows": len(jobs),
        "resume_skipped_rows": skipped,
        "workers": args.workers,
        "shard_count": args.shard_count,
        "shard_index": args.shard_index,
    }
    _atomic_json(args.out / f"run_manifest.shard_{args.shard_index:04d}.json", run_manifest)
    counts: dict[str, int] = defaultdict(int)
    start = time.time()
    for completed, result in enumerate(_run_bounded(jobs, args.workers), start=1):
        counts[str(result["status"])] += 1
        if completed <= 10 or completed % 100 == 0 or completed == len(jobs):
            rate = completed / max(time.time() - start, 1e-9)
            print(
                f"completed={completed}/{len(jobs)} rate={rate:.2f}/s "
                f"status={dict(sorted(counts.items()))} last={result['dataset_slug']}/{result['asset_id']}",
                flush=True,
            )
    completion = {
        **run_manifest,
        "status_counts": dict(sorted(counts.items())),
        "elapsed_seconds": time.time() - start,
        "finished": True,
    }
    _atomic_json(args.out / f"completion.shard_{args.shard_index:04d}.json", completion)
    print(json.dumps(completion, ensure_ascii=True, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
