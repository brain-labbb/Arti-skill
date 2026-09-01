#!/usr/bin/env python3
"""Evaluate dependency-aware functional articulation motion in Genesis."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable, Mapping, Sequence


EARLY_CPU_AFFINITY_ENV = "LAM_GENESIS_CPU_AFFINITY"


def _bind_requested_affinity_early() -> None:
    raw = os.environ.get(EARLY_CPU_AFFINITY_ENV)
    if raw is None:
        return
    try:
        requested = {int(token) for token in raw.split(",") if token.strip()}
    except ValueError as error:
        raise RuntimeError(f"invalid {EARLY_CPU_AFFINITY_ENV}: {raw!r}") from error
    if not requested:
        raise RuntimeError(f"empty {EARLY_CPU_AFFINITY_ENV}")
    os.sched_setaffinity(0, requested)
    observed = set(os.sched_getaffinity(0))
    if observed != requested:
        raise RuntimeError(
            f"early CPU affinity mismatch: {sorted(observed)} != {sorted(requested)}"
        )


_bind_requested_affinity_early()


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
REPO = SCRIPT_PATH.parents[2]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import run_urdf_lam_supplementary_v1 as genesis_collision  # noqa: E402
import table5_stable_v2_aggregate as table5_base  # noqa: E402


SCHEMA_VERSION = "table5_dependency_aware_functional_motion_v1"
PROTOCOL_ID = "table5-dependency-aware-functional-motion-genesis-20260830"
RECORD_SCHEMA = "table5_dependency_aware_functional_motion_record_v1"
SUMMARY_SCHEMA = "table5_dependency_aware_functional_motion_summary_v1"
LOW_FRACTION = 0.10
HIGH_FRACTION = 0.90
SEARCH_APPROACH_SAMPLES = 3
SEARCH_SWEEP_SAMPLES = 5
VERIFY_APPROACH_SAMPLES = 5
VERIFY_SWEEP_SAMPLES = 11
MAX_SEARCH_NODES = 192
MAX_COLLISION_QUERIES = 6000
DEFAULT_WORKERS = 8
DEFAULT_TIMEOUT_SECONDS = 1200
SIMULATOR = "genesis"


class FunctionalMotionError(RuntimeError):
    """The frozen functional-motion protocol cannot be evaluated."""


class Table5GenesisCollisionAdapter(genesis_collision.GenesisTable4aAdapter):
    """Table 4 collision oracle with Table 5's version-and-diff provenance gate."""

    def build(self) -> None:
        import genesis as gs

        if str(gs.__version__) != genesis_collision.GENESIS_VERSION:
            raise FunctionalMotionError(
                f"Genesis version mismatch: {gs.__version__}"
            )
        if bool(getattr(gs, "_initialized", False)):
            raise FunctionalMotionError(
                "Genesis already initialized; use one asset per process"
            )
        started = time.monotonic()
        gs.init(
            backend=gs.cpu,
            precision=genesis_collision.GENESIS_PRECISION,
            logging_level=logging.ERROR,
            seed=20260813,
        )
        self.gs = gs
        options = gs.options.RigidOptions(
            gravity=(0.0, 0.0, 0.0),
            enable_collision=True,
            enable_self_collision=True,
            enable_neutral_collision=True,
            enable_adjacent_collision=True,
            max_collision_pairs=genesis_collision.MAX_COLLISION_PAIRS,
            max_contacts=genesis_collision.MAX_CONTACTS,
            contact_pruning_tolerance=genesis_collision.CONTACT_PRUNING_TOLERANCE_M,
        )
        self.scene = gs.Scene(rigid_options=options, show_viewer=False)
        morph = gs.morphs.URDF(
            file=str(self.urdf_path),
            fixed=True,
            visualization=False,
            collision=True,
            merge_fixed_links=False,
            convexify=False,
            decimate=False,
            watertighten=0,
            recompute_inertia=False,
            align=False,
        )
        self.entity = self.scene.add_entity(morph)
        self.scene.build()
        self.load_time_seconds = time.monotonic() - started
        self._map_dofs()
        self._map_pairs()


def _genesis_runtime_record() -> dict[str, Any]:
    cpu_affinity = genesis_collision.bind_cpu_affinity()
    thread_environment = genesis_collision.bind_thread_environment()
    import torch

    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    import genesis as gs
    import rtree
    import trimesh

    if str(gs.__version__) != genesis_collision.GENESIS_VERSION:
        raise FunctionalMotionError(f"Genesis version mismatch: {gs.__version__}")
    if str(rtree.__version__) != genesis_collision.RTREE_VERSION:
        raise FunctionalMotionError(f"rtree version mismatch: {rtree.__version__}")
    if str(trimesh.__version__) != genesis_collision.TRIMESH_VERSION:
        raise FunctionalMotionError(f"trimesh version mismatch: {trimesh.__version__}")
    source_root = genesis_collision.GENESIS_SOURCE_ROOT.resolve(strict=True)

    def git(*arguments: str) -> str:
        completed = subprocess.run(
            ["git", "-C", str(source_root), *arguments],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        if completed.returncode != 0:
            raise FunctionalMotionError(
                f"Genesis git provenance failed: {completed.stderr[-1000:]}"
            )
        return completed.stdout

    commit = git("rev-parse", "HEAD").strip()
    if commit != genesis_collision.EXPECTED_GENESIS_COMMIT:
        raise FunctionalMotionError(f"Genesis commit mismatch: {commit}")
    status = git("status", "--porcelain=v1")
    diff = git("diff", "--binary", "HEAD")
    return {
        "engine": "genesis",
        "version": str(gs.__version__),
        "backend": "cpu",
        "precision": genesis_collision.GENESIS_PRECISION,
        "launcher": str(Path(sys.executable).resolve(strict=True)),
        "genesis_source_root": str(source_root),
        "genesis_commit": commit,
        "genesis_dirty": bool(status),
        "genesis_status_sha256": hashlib.sha256(status.encode("utf-8")).hexdigest(),
        "genesis_tracked_diff_sha256": hashlib.sha256(diff.encode("utf-8")).hexdigest(),
        "genesis_status": status.splitlines(),
        "trimesh_version": str(trimesh.__version__),
        "rtree_version": str(rtree.__version__),
        "torch_version": str(torch.__version__),
        "torch_threads": {
            "intra_op": int(torch.get_num_threads()),
            "inter_op": int(torch.get_num_interop_threads()),
        },
        "cache_path": os.environ.get("GS_CACHE_FILE_PATH"),
        "cpu_affinity": cpu_affinity,
        "thread_environment": thread_environment,
    }


def _canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    )


def _canonical_sha256(value: Any, *, exclude: Sequence[str] = ()) -> str:
    if exclude and isinstance(value, Mapping):
        value = {key: item for key, item in value.items() if key not in exclude}
    return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise FunctionalMotionError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise FunctionalMotionError(f"JSON root is not an object: {path}")
    return value


def _atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(
                value,
                stream,
                indent=2,
                ensure_ascii=True,
                allow_nan=False,
                sort_keys=True,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _protocol() -> dict[str, Any]:
    value: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "engine_protocol_id": genesis_collision.ENGINE_PROTOCOL_ID,
        "engine": {
            "name": "Genesis",
            "version": genesis_collision.GENESIS_VERSION,
            "backend": "cpu",
            "precision": genesis_collision.GENESIS_PRECISION,
            "gravity_m_per_s2": [0.0, 0.0, 0.0],
            "base": "fixed",
            "execution": "direct kinematic configuration replay",
            "torch_threads": {"intra_op": 1, "inter_op": 1},
        },
        "motion_scope": {
            "eligible_joint_types": ["revolute", "prismatic"],
            "requires_finite_strict_bounds": True,
            "range_fractions": [LOW_FRACTION, HIGH_FRACTION],
            "neutral_policy": "zero_clamped_to_declared_joint_range",
            "dependency_policy": (
                "deterministic depth-first search over joint order and endpoint "
                "direction; completed joints remain at the reached endpoint"
            ),
            "functional_claim": (
                "collision-constrained dependency-aware motion proxy without "
                "independent semantic ground truth"
            ),
        },
        "collision": {
            "oracle": "Genesis detect_collision plus get_contacts",
            "illegal_penetration_m": genesis_collision.PENETRATION_THRESHOLD_M,
            "eligible_pairs": (
                "distinct source links excluding direct URDF parent-child pairs"
            ),
            "contact_pruning_tolerance_m": None,
            "fail_closed_on_unmapped_pair_or_capacity": True,
        },
        "search": {
            "approach_samples": SEARCH_APPROACH_SAMPLES,
            "sweep_samples": SEARCH_SWEEP_SAMPLES,
            "maximum_nodes": MAX_SEARCH_NODES,
            "maximum_collision_queries": MAX_COLLISION_QUERIES,
        },
        "verification": {
            "approach_samples": VERIFY_APPROACH_SAMPLES,
            "sweep_samples": VERIFY_SWEEP_SAMPLES,
            "requires_exact_joint_readback": True,
            "requires_complete_plan_for_every_eligible_joint": True,
        },
        "implementation": {
            "runner": str(SCRIPT_PATH),
            "runner_sha256": _sha256_file(SCRIPT_PATH),
            "genesis_collision_adapter": str(
                Path(genesis_collision.__file__).resolve()
            ),
            "genesis_collision_adapter_sha256": _sha256_file(
                Path(genesis_collision.__file__).resolve()
            ),
        },
    }
    value["protocol_sha256"] = _canonical_sha256(value)
    return value


def _finite(value: Any) -> bool:
    return bool(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _eligible_specs(row: Mapping[str, Any]) -> list[dict[str, Any]]:
    specs: list[dict[str, Any]] = []
    seen: set[str] = set()
    for joint in row.get("scalar_joints", []):
        if not isinstance(joint, Mapping):
            continue
        name = joint.get("name")
        kind = joint.get("type")
        lower = joint.get("lower")
        upper = joint.get("upper")
        if (
            kind not in {"revolute", "prismatic"}
            or not isinstance(name, str)
            or not name
            or not _finite(lower)
            or not _finite(upper)
            or float(lower) >= float(upper)
        ):
            continue
        if name in seen:
            raise FunctionalMotionError(f"duplicate eligible joint name: {name}")
        seen.add(name)
        lower_value = float(lower)
        upper_value = float(upper)
        neutral = min(max(0.0, lower_value), upper_value)
        specs.append(
            {
                "name": name,
                "type": str(kind),
                "parent": str(joint.get("parent") or ""),
                "child": str(joint.get("child") or ""),
                "lower": lower_value,
                "upper": upper_value,
                "neutral": neutral,
                "low": lower_value + LOW_FRACTION * (upper_value - lower_value),
                "high": lower_value + HIGH_FRACTION * (upper_value - lower_value),
            }
        )
    depth = _joint_depths(row)
    for spec in specs:
        spec["depth"] = depth.get(spec["name"], 10**6)
    return sorted(specs, key=lambda item: (int(item["depth"]), str(item["name"])))


def _joint_depths(row: Mapping[str, Any]) -> dict[str, int]:
    tree = row.get("joint_tree")
    if not isinstance(tree, Mapping):
        return {}
    roots = tree.get("root_links")
    joints = tree.get("joints")
    if not isinstance(roots, list) or not isinstance(joints, list):
        return {}
    depth_by_link = {str(name): 0 for name in roots if isinstance(name, str)}
    pending = [joint for joint in joints if isinstance(joint, Mapping)]
    depth_by_joint: dict[str, int] = {}
    for _ in range(len(pending) + 1):
        retained: list[Mapping[str, Any]] = []
        changed = False
        for joint in pending:
            parent = str(joint.get("parent") or "")
            child = str(joint.get("child") or "")
            name = str(joint.get("name") or "")
            if parent not in depth_by_link:
                retained.append(joint)
                continue
            joint_depth = depth_by_link[parent]
            depth_by_joint[name] = joint_depth
            depth_by_link[child] = min(
                depth_by_link.get(child, joint_depth + 1), joint_depth + 1
            )
            changed = True
        pending = retained
        if not pending or not changed:
            break
    return depth_by_joint


def _interpolate(
    start: Sequence[float], end: Sequence[float], samples: int
) -> list[tuple[float, ...]]:
    if samples < 2 or len(start) != len(end):
        raise FunctionalMotionError("invalid interpolation request")
    return [
        tuple(
            float(left) + fraction * (float(right) - float(left))
            for left, right in zip(start, end, strict=True)
        )
        for fraction in (index / (samples - 1) for index in range(1, samples))
    ]


def _state_key(values: Sequence[float]) -> tuple[float, ...]:
    return tuple(round(float(value), 12) for value in values)


def plan_dependency_aware_motion(
    specs: Sequence[Mapping[str, Any]],
    neutral: Sequence[float],
    index_by_name: Mapping[str, int],
    observe: Callable[[tuple[float, ...]], Mapping[str, Any]],
    *,
    maximum_nodes: int = MAX_SEARCH_NODES,
    maximum_queries: int = MAX_COLLISION_QUERIES,
) -> dict[str, Any]:
    """Find a deterministic collision-free order for all bounded joints."""

    ordered = [dict(spec) for spec in specs]
    names = tuple(str(spec["name"]) for spec in ordered)
    if any(name not in index_by_name for name in names):
        missing = sorted(name for name in names if name not in index_by_name)
        return {
            "passed": False,
            "reason": "eligible_joint_unmapped",
            "missing_joint_names": missing,
            "eligible_joint_count": len(names),
            "completed_joint_count": 0,
            "plan": [],
            "search_nodes": 0,
            "collision_queries": 0,
        }

    cache: dict[tuple[float, ...], dict[str, Any]] = {}
    query_count = 0
    search_nodes = 0
    best_plan: list[dict[str, Any]] = []
    best_values = tuple(float(value) for value in neutral)
    exhausted = False

    def checked(values: tuple[float, ...]) -> dict[str, Any]:
        nonlocal query_count, exhausted
        key = _state_key(values)
        if key in cache:
            return cache[key]
        if query_count >= maximum_queries:
            exhausted = True
            return {"valid": False, "reason": "collision_query_budget_exhausted"}
        query_count += 1
        raw = dict(observe(values))
        valid = raw.get("valid") is True
        raw["valid"] = valid
        cache[key] = raw
        return raw

    initial = tuple(float(value) for value in neutral)
    initial_observation = checked(initial)
    if not initial_observation["valid"]:
        return {
            "passed": False,
            "reason": str(initial_observation.get("reason") or "neutral_state_invalid"),
            "eligible_joint_count": len(names),
            "completed_joint_count": 0,
            "plan": [],
            "search_nodes": 0,
            "collision_queries": query_count,
            "neutral_observation": initial_observation,
        }

    visited: set[tuple[tuple[str, ...], tuple[float, ...]]] = set()

    def transition(
        current: tuple[float, ...], spec: Mapping[str, Any], direction: str
    ) -> tuple[tuple[float, ...], dict[str, Any]] | None:
        index = int(index_by_name[str(spec["name"])])
        start_value, end_value = (
            (float(spec["low"]), float(spec["high"]))
            if direction == "low_to_high"
            else (float(spec["high"]), float(spec["low"]))
        )
        start = list(current)
        start[index] = start_value
        end = list(start)
        end[index] = end_value
        samples = [
            *_interpolate(current, start, SEARCH_APPROACH_SAMPLES),
            *_interpolate(start, end, SEARCH_SWEEP_SAMPLES),
        ]
        maximum_penetration = 0.0
        for sample in samples:
            observation = checked(sample)
            penetration = observation.get("max_penetration_m")
            if _finite(penetration):
                maximum_penetration = max(maximum_penetration, float(penetration))
            if not observation["valid"]:
                return None
        return tuple(end), {
            "joint_name": str(spec["name"]),
            "joint_type": str(spec["type"]),
            "direction": direction,
            "range_fraction_covered": HIGH_FRACTION - LOW_FRACTION,
            "search_state_count": len(samples),
            "search_max_penetration_m": maximum_penetration,
        }

    def search(
        current: tuple[float, ...], remaining: tuple[str, ...], plan: list[dict[str, Any]]
    ) -> list[dict[str, Any]] | None:
        nonlocal search_nodes, best_plan, best_values, exhausted
        if len(plan) > len(best_plan):
            best_plan = deepcopy(plan)
            best_values = current
        if not remaining:
            return deepcopy(plan)
        if search_nodes >= maximum_nodes or exhausted:
            exhausted = True
            return None
        key = (remaining, _state_key(current))
        if key in visited:
            return None
        visited.add(key)
        search_nodes += 1
        remaining_set = set(remaining)
        candidates = [spec for spec in ordered if spec["name"] in remaining_set]
        for spec in candidates:
            name = str(spec["name"])
            next_remaining = tuple(item for item in remaining if item != name)
            neutral_value = float(spec["neutral"])
            directions = (
                ("low_to_high", "high_to_low")
                if abs(neutral_value - float(spec["low"]))
                <= abs(neutral_value - float(spec["high"]))
                else ("high_to_low", "low_to_high")
            )
            for direction in directions:
                result = transition(current, spec, direction)
                if result is None:
                    if exhausted:
                        return None
                    continue
                next_values, step = result
                found = search(next_values, next_remaining, [*plan, step])
                if found is not None:
                    return found
        return None

    found_plan = search(initial, names, [])
    return {
        "passed": found_plan is not None,
        "reason": (
            None
            if found_plan is not None
            else "search_budget_exhausted"
            if exhausted
            else "no_complete_dependency_order"
        ),
        "eligible_joint_count": len(names),
        "completed_joint_count": len(found_plan or best_plan),
        "completed_joint_names": [
            str(step["joint_name"]) for step in (found_plan or best_plan)
        ],
        "plan": found_plan or best_plan,
        "best_terminal_values": list(best_values),
        "search_nodes": search_nodes,
        "collision_queries": query_count,
        "cached_configuration_count": len(cache),
        "search_exhausted": exhausted,
        "neutral_observation": initial_observation,
    }


def _simulator_source(row: Mapping[str, Any]) -> Mapping[str, Any]:
    sources = row.get("simulator_sources")
    source = sources.get(SIMULATOR) if isinstance(sources, Mapping) else None
    if not isinstance(source, Mapping):
        raise FunctionalMotionError("Genesis simulator source is unavailable")
    path = source.get("path")
    sha256 = source.get("sha256")
    if not isinstance(path, str) or not isinstance(sha256, str):
        raise FunctionalMotionError("Genesis simulator source binding is malformed")
    resolved = Path(path).resolve(strict=True)
    if _sha256_file(resolved) != sha256:
        raise FunctionalMotionError("Genesis simulator source hash mismatch")
    return {**source, "path": str(resolved)}


def _runtime_mapping(
    adapter: Any, row: Mapping[str, Any]
) -> tuple[dict[str, int], list[float]]:
    order = adapter.dof_order
    if not isinstance(order, list):
        raise FunctionalMotionError("Genesis DoF order is unavailable")
    index_by_name: dict[str, int] = {}
    for item in order:
        name = item.get("joint_name") if isinstance(item, Mapping) else None
        index = item.get("dof_index") if isinstance(item, Mapping) else None
        if (
            not isinstance(name, str)
            or not isinstance(index, int)
            or isinstance(index, bool)
            or name in index_by_name
        ):
            raise FunctionalMotionError("Genesis DoF order is malformed")
        index_by_name[name] = index
    neutral = [0.0] * len(order)
    for spec in _eligible_specs(row):
        if spec["name"] in index_by_name:
            neutral[index_by_name[str(spec["name"])]] = float(spec["neutral"])
    return index_by_name, neutral


def _verify_plan(
    adapter: Any,
    specs: Sequence[Mapping[str, Any]],
    neutral: Sequence[float],
    index_by_name: Mapping[str, int],
    plan: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    by_name = {str(spec["name"]): spec for spec in specs}
    current = tuple(float(value) for value in neutral)
    state_count = 0
    maximum_penetration = 0.0
    steps: list[dict[str, Any]] = []
    for raw_step in plan:
        name = str(raw_step["joint_name"])
        direction = str(raw_step["direction"])
        spec = by_name[name]
        index = int(index_by_name[name])
        start_value, end_value = (
            (float(spec["low"]), float(spec["high"]))
            if direction == "low_to_high"
            else (float(spec["high"]), float(spec["low"]))
        )
        start = list(current)
        start[index] = start_value
        end = list(start)
        end[index] = end_value
        samples = [
            *_interpolate(current, start, VERIFY_APPROACH_SAMPLES),
            *_interpolate(start, end, VERIFY_SWEEP_SAMPLES),
        ]
        step_maximum = 0.0
        for sample in samples:
            _, q_error, penetration, contact = adapter._observe_configuration(sample)
            state_count += 1
            step_maximum = max(step_maximum, float(penetration))
            maximum_penetration = max(maximum_penetration, float(penetration))
            if q_error > genesis_collision.READBACK_TOLERANCE:
                return {
                    "passed": False,
                    "reason": "verification_readback_error",
                    "state_count": state_count,
                    "maximum_penetration_m": maximum_penetration,
                    "contact": contact,
                    "steps": steps,
                }
            if penetration > genesis_collision.PENETRATION_THRESHOLD_M:
                return {
                    "passed": False,
                    "reason": "verification_illegal_penetration",
                    "state_count": state_count,
                    "maximum_penetration_m": maximum_penetration,
                    "contact": contact,
                    "steps": steps,
                }
        current = tuple(end)
        steps.append(
            {
                "joint_name": name,
                "direction": direction,
                "state_count": len(samples),
                "maximum_penetration_m": step_maximum,
            }
        )
    return {
        "passed": True,
        "reason": None,
        "state_count": state_count,
        "maximum_penetration_m": maximum_penetration,
        "steps": steps,
        "final_values_sha256": _canonical_sha256(list(current)),
    }


def evaluate_row(row: Mapping[str, Any], protocol: Mapping[str, Any]) -> dict[str, Any]:
    started = time.monotonic()
    dataset_slug = str(row.get("dataset_slug") or "")
    dataset_id = str(row.get("dataset_id") or "")
    record: dict[str, Any] = {
        "schema_version": RECORD_SCHEMA,
        "protocol": dict(protocol),
        "identity": {
            "dataset_slug": dataset_slug,
            "dataset_id": dataset_id,
            "row_sha256": row.get("row_sha256"),
        },
        "state": "FAIL",
        "reason": None,
        "source": None,
        "eligible_joint_count": 0,
        "planner": None,
        "verification": None,
    }
    adapter: Any | None = None
    try:
        if protocol != _protocol():
            raise FunctionalMotionError("worker protocol differs from implementation")
        source = _simulator_source(row)
        record["source"] = source
        specs = _eligible_specs(row)
        record["eligible_joint_count"] = len(specs)
        record["eligible_joint_names"] = [str(spec["name"]) for spec in specs]
        if not specs:
            record["reason"] = "no_bounded_revolute_or_prismatic_joint"
            return record

        runtime = _genesis_runtime_record()
        record["runtime"] = runtime
        adapter = Table5GenesisCollisionAdapter(
            Path(str(source["path"])), runtime
        )
        adapter.build()
        index_by_name, neutral = _runtime_mapping(adapter, row)

        def observe(values: tuple[float, ...]) -> dict[str, Any]:
            _, q_error, penetration, contact = adapter._observe_configuration(values)
            return {
                "valid": bool(
                    q_error <= genesis_collision.READBACK_TOLERANCE
                    and penetration <= genesis_collision.PENETRATION_THRESHOLD_M
                ),
                "reason": (
                    None
                    if penetration <= genesis_collision.PENETRATION_THRESHOLD_M
                    else "illegal_penetration"
                ),
                "q_readback_max_abs_error": q_error,
                "max_penetration_m": penetration,
                "eligible_contact_count": contact["eligible_contact_count"],
            }

        planner = plan_dependency_aware_motion(specs, neutral, index_by_name, observe)
        record["planner"] = planner
        if not planner["passed"]:
            record["reason"] = planner["reason"]
            return record
        verification = _verify_plan(
            adapter, specs, neutral, index_by_name, planner["plan"]
        )
        record["verification"] = verification
        if not verification["passed"]:
            record["reason"] = verification["reason"]
            return record
        record["state"] = "PASS"
        record["reason"] = None
        return record
    except BaseException as error:
        record["reason"] = f"{type(error).__name__}: {error}"[-2000:]
        return record
    finally:
        if adapter is not None:
            try:
                adapter.close()
            except BaseException as error:
                record["close_error"] = f"{type(error).__name__}: {error}"[-2000:]
                record["state"] = "FAIL"
                record["reason"] = "adapter_close_failed"
        record["elapsed_seconds"] = time.monotonic() - started
        record["record_sha256"] = _canonical_sha256(
            record, exclude=("record_sha256",)
        )


def _worker(request_path: Path, response_path: Path) -> int:
    request = _read_json(request_path)
    row = request.get("row")
    protocol = request.get("protocol")
    if not isinstance(row, Mapping) or not isinstance(protocol, Mapping):
        raise FunctionalMotionError("worker request is malformed")
    if request.get("request_sha256") != _canonical_sha256(
        request, exclude=("request_sha256",)
    ):
        raise FunctionalMotionError("worker request hash mismatch")
    response = evaluate_row(row, protocol)
    _atomic_json(response_path, response)
    return 0


def _fixed_datasets(
    formal_prepared: Path, articraft_prepared: Path
) -> list[dict[str, Any]]:
    return table5_base._fixed_datasets(
        table5_base._read_json(formal_prepared),
        table5_base._read_json(articraft_prepared),
    )


def _record_path(root: Path, slug: str, dataset_id: str) -> Path:
    return root / "records" / slug / f"{dataset_id}.json"


def _valid_record(
    path: Path, row: Mapping[str, Any], protocol: Mapping[str, Any]
) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        record = _read_json(path)
    except FunctionalMotionError:
        return None
    identity = record.get("identity")
    if (
        record.get("schema_version") != RECORD_SCHEMA
        or record.get("protocol") != protocol
        or not isinstance(identity, Mapping)
        or identity.get("dataset_slug") != row.get("dataset_slug")
        or identity.get("dataset_id") != row.get("dataset_id")
        or identity.get("row_sha256") != row.get("row_sha256")
        or record.get("record_sha256")
        != _canonical_sha256(record, exclude=("record_sha256",))
    ):
        return None
    return record


def _run_one(
    row: Mapping[str, Any],
    root: Path,
    protocol: Mapping[str, Any],
    timeout: int,
    cpu_affinity: Sequence[int] | None = None,
) -> tuple[str, str, str]:
    slug = str(row["dataset_slug"])
    dataset_id = str(row["dataset_id"])
    response_path = _record_path(root, slug, dataset_id)
    if _valid_record(response_path, row, protocol) is not None:
        return slug, dataset_id, "cached"
    request_path = root / "requests" / slug / f"{dataset_id}.json"
    request = {"row": row, "protocol": protocol}
    request["request_sha256"] = _canonical_sha256(request)
    _atomic_json(request_path, request)
    response_path.parent.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.update(genesis_collision.THREAD_ENV_VALUES)
    environment["GS_CACHE_FILE_PATH"] = str((root / "genesis_cache.pkl").resolve())
    if cpu_affinity:
        environment[genesis_collision.CPU_AFFINITY_ENV] = ",".join(
            str(cpu) for cpu in cpu_affinity
        )
    command = [
        str(genesis_collision.DEFAULT_GENESIS_PYTHON),
        str(SCRIPT_PATH),
        "--worker",
        str(request_path),
        str(response_path),
    ]
    try:
        completed = subprocess.run(
            command,
            cwd=REPO,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        failure = {
            "schema_version": RECORD_SCHEMA,
            "protocol": protocol,
            "identity": {
                "dataset_slug": slug,
                "dataset_id": dataset_id,
                "row_sha256": row.get("row_sha256"),
            },
            "state": "FAIL",
            "reason": "worker_timeout",
            "eligible_joint_count": len(_eligible_specs(row)),
            "elapsed_seconds": float(timeout),
        }
        failure["record_sha256"] = _canonical_sha256(failure)
        _atomic_json(response_path, failure)
        return slug, dataset_id, "timeout"
    if completed.returncode != 0 or not response_path.is_file():
        failure = {
            "schema_version": RECORD_SCHEMA,
            "protocol": protocol,
            "identity": {
                "dataset_slug": slug,
                "dataset_id": dataset_id,
                "row_sha256": row.get("row_sha256"),
            },
            "state": "FAIL",
            "reason": f"worker_exit_{completed.returncode}",
            "stderr_tail": completed.stderr[-4000:],
            "stdout_tail": completed.stdout[-4000:],
            "eligible_joint_count": len(_eligible_specs(row)),
            "elapsed_seconds": None,
        }
        failure["record_sha256"] = _canonical_sha256(failure)
        _atomic_json(response_path, failure)
        return slug, dataset_id, "worker_failure"
    if _valid_record(response_path, row, protocol) is None:
        return slug, dataset_id, "invalid_record"
    return slug, dataset_id, "completed"


def _rate(passed: int, denominator: int) -> dict[str, Any]:
    return {
        "passed": passed,
        "denominator": denominator,
        "percentage": 0.0 if denominator == 0 else 100.0 * passed / denominator,
    }


def aggregate_records(
    datasets: Sequence[Mapping[str, Any]], root: Path, protocol: Mapping[str, Any]
) -> dict[str, Any]:
    output: list[dict[str, Any]] = []
    complete = True
    for dataset in datasets:
        rows = dataset["rows"]
        records: list[dict[str, Any]] = []
        for row in rows:
            record = _valid_record(
                _record_path(root, str(dataset["dataset_slug"]), str(row["dataset_id"])),
                row,
                protocol,
            )
            if record is None:
                complete = False
                continue
            records.append(record)
        asset_passed = sum(record.get("state") == "PASS" for record in records)
        eligible_joints = sum(int(record.get("eligible_joint_count", 0)) for record in records)
        completed_joints = sum(
            int((record.get("planner") or {}).get("completed_joint_count", 0))
            for record in records
            if isinstance(record.get("planner"), Mapping)
        )
        evaluable_assets = sum(int(record.get("eligible_joint_count", 0)) > 0 for record in records)
        reasons: dict[str, int] = {}
        for record in records:
            reason = str(record.get("reason") or "PASS")
            reasons[reason] = reasons.get(reason, 0) + 1
        output.append(
            {
                "dataset_slug": dataset["dataset_slug"],
                "dataset_name": dataset["dataset_name"],
                "n": len(rows),
                "record_count": len(records),
                "asset_success": _rate(asset_passed, len(rows)),
                "evaluable_asset_coverage": _rate(evaluable_assets, len(rows)),
                "joint_completion": _rate(completed_joints, eligible_joints),
                "eligible_joint_count": eligible_joints,
                "completed_joint_count": completed_joints,
                "failure_reason_counts": reasons,
            }
        )
    summary: dict[str, Any] = {
        "schema_version": SUMMARY_SCHEMA,
        "classification": "COMPLETE" if complete else "PARTIAL",
        "protocol": protocol,
        "datasets": output,
    }
    summary["summary_sha256"] = _canonical_sha256(summary)
    return summary


def _run_parent(args: argparse.Namespace) -> int:
    protocol = _protocol()
    datasets = _fixed_datasets(args.formal_prepared, args.articraft_prepared)
    if args.dataset:
        requested = set(args.dataset)
        datasets = [
            dataset for dataset in datasets if dataset["dataset_slug"] in requested
        ]
        missing = requested - {str(dataset["dataset_slug"]) for dataset in datasets}
        if missing:
            raise FunctionalMotionError(f"unknown dataset slugs: {sorted(missing)}")
    rows = [row for dataset in datasets for row in dataset["rows"]]
    if args.asset:
        requested_assets = set(args.asset)
        rows = [row for row in rows if row["dataset_id"] in requested_assets]
        missing_assets = requested_assets - {str(row["dataset_id"]) for row in rows}
        if missing_assets:
            raise FunctionalMotionError(
                f"unknown dataset IDs in selected datasets: {sorted(missing_assets)}"
            )
    if args.limit is not None:
        rows = rows[: args.limit]
    args.out.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": SUMMARY_SCHEMA,
        "protocol": protocol,
        "formal_prepared": str(args.formal_prepared.resolve()),
        "formal_prepared_sha256": _sha256_file(args.formal_prepared),
        "articraft_prepared": str(args.articraft_prepared.resolve()),
        "articraft_prepared_sha256": _sha256_file(args.articraft_prepared),
        "dataset_slugs": [dataset["dataset_slug"] for dataset in datasets],
        "requested_row_count": len(rows),
    }
    manifest["manifest_sha256"] = _canonical_sha256(manifest)
    manifest_path = args.out / "manifest.json"
    if manifest_path.exists() and _read_json(manifest_path) != manifest:
        raise FunctionalMotionError("existing output manifest differs from this run")
    if not manifest_path.exists():
        _atomic_json(manifest_path, manifest)

    pending = [
        row
        for row in rows
        if _valid_record(
            _record_path(args.out, str(row["dataset_slug"]), str(row["dataset_id"])),
            row,
            protocol,
        )
        is None
    ]
    available_cpus = sorted(int(cpu) for cpu in os.sched_getaffinity(0))
    required_cpus = args.workers * genesis_collision.CPU_AFFINITY_WIDTH
    if len(available_cpus) < required_cpus:
        raise FunctionalMotionError(
            f"{args.workers} workers require {required_cpus} available CPUs, "
            f"observed {len(available_cpus)}"
        )
    selected_cpus = available_cpus[-required_cpus:]
    affinity_slots = [
        selected_cpus[index : index + genesis_collision.CPU_AFFINITY_WIDTH]
        for index in range(0, len(selected_cpus), genesis_collision.CPU_AFFINITY_WIDTH)
    ]
    affinity_pool: queue.Queue[list[int]] = queue.Queue()
    for slot in affinity_slots:
        affinity_pool.put(slot)

    def run_with_affinity(row: Mapping[str, Any]) -> tuple[str, str, str]:
        slot = affinity_pool.get()
        try:
            return _run_one(row, args.out, protocol, args.timeout, slot)
        finally:
            affinity_pool.put(slot)

    if pending:
        first = pending.pop(0)
        print(json.dumps({"warmup": first["dataset_id"], "remaining": len(pending)}))
        run_with_affinity(first)
    completed = 1 if rows else 0
    if pending:
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            futures = {
                executor.submit(run_with_affinity, row): row
                for row in pending
            }
            for future in as_completed(futures):
                completed += 1
                slug, dataset_id, state = future.result()
                if completed % 10 == 0 or state not in {"completed", "cached"}:
                    print(
                        json.dumps(
                            {
                                "completed": completed,
                                "total": len(rows),
                                "dataset_slug": slug,
                                "dataset_id": dataset_id,
                                "state": state,
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
    selected_by_slug: dict[str, list[Mapping[str, Any]]] = {}
    for row in rows:
        selected_by_slug.setdefault(str(row["dataset_slug"]), []).append(row)
    aggregate_datasets = [
        {**dataset, "rows": selected_by_slug.get(str(dataset["dataset_slug"]), [])}
        for dataset in datasets
        if selected_by_slug.get(str(dataset["dataset_slug"]), [])
    ]
    summary = aggregate_records(aggregate_datasets, args.out, protocol)
    _atomic_json(args.out / "summary.json", summary)
    print(
        json.dumps(
            {
                "classification": summary["classification"],
                "out": str(args.out.resolve()),
                "summary_sha256": summary["summary_sha256"],
            },
            sort_keys=True,
        )
    )
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", nargs=2, metavar=("REQUEST", "RESPONSE"))
    parser.add_argument("--formal-prepared", type=Path)
    parser.add_argument("--articraft-prepared", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--dataset", action="append")
    parser.add_argument("--asset", action="append")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT_SECONDS)
    args = parser.parse_args(argv)
    if args.worker:
        return _worker(Path(args.worker[0]), Path(args.worker[1]))
    if not args.formal_prepared or not args.articraft_prepared or not args.out:
        parser.error("parent mode requires --formal-prepared, --articraft-prepared, --out")
    if args.workers <= 0 or args.timeout <= 0 or (args.limit is not None and args.limit <= 0):
        parser.error("workers, timeout, and limit must be positive")
    return _run_parent(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except FunctionalMotionError as error:
        print(f"table5_functional_motion: {error}", file=sys.stderr)
        raise SystemExit(2)
