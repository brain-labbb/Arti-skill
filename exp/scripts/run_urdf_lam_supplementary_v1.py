#!/usr/bin/env python3
"""Fail-closed LAM supplementary runner using Genesis only.

The historical PyBullet Table-4 runner is intentionally not imported.  This
module freezes the given Table-3 cohort, computes the static/Table-4b atoms,
and exposes a one-asset Genesis Table-4a adapter plus an append-only qualified
bulk driver.  It never silently falls back to a different simulator.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import logging
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_geometry as geometry  # noqa: E402
from exp.scripts import lam_supplementary_static as static  # noqa: E402
from exp.scripts import verify_urdf_lam_supplementary_v1 as verifier  # noqa: E402

DEFAULT_DATASET_ROOT = REPO / "exp/Articulated-Object-Code"
DEFAULT_SOURCE_RECORDS = REPO / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl"
DEFAULT_SOURCE_MANIFEST = DEFAULT_SOURCE_RECORDS.with_name("manifest.json")
DEFAULT_OUTPUT_ROOT = REPO / "exp/runtime/urdf_lam_supplementary_n800_20260817_v2"
DEFAULT_GENESIS_PYTHON = Path("/mnt/zsn/miniconda3/envs/genesis-main/bin/python")
GENESIS_SOURCE_ROOT = Path("/mnt/zsn/zsn_workspace/PhysAI/simulation/genesis-world")
EXPECTED_GENESIS_COMMIT = "b1ddc20e102e010ca0a967c88bfc21715c1bc597"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"

PROTOCOL_ID = "urdf_lam_supplementary_n800_genesis_v1"
ENGINE_PROTOCOL_ID = "genesis_contact_penetration_v1"
SAMPLE_SIZE = 800
JOINT_COUNT = 2395
STATE_SAMPLES_PER_JOINT = 21
SOBOL_STATE_COUNT_PER_ASSET = 64
SOBOL_SEED = 20260813
SOBOL_SCRAMBLE = True
EXPECTED_MOVABLE_ASSET_COUNT = 774
SOBOL_STATE_COUNT_TOTAL = SOBOL_STATE_COUNT_PER_ASSET * EXPECTED_MOVABLE_ASSET_COUNT
STRICT_STATE_COUNT_TOTAL = SAMPLE_SIZE + SOBOL_STATE_COUNT_TOTAL
PENETRATION_THRESHOLD_M = 1e-6
GENESIS_VERSION = "1.3.1"
TRIMESH_VERSION = "5.0.0"
RTREE_VERSION = "1.4.1"
GENESIS_PRECISION = "64"
GEOMETRY_SAMPLES_PER_DIRECTION = 32768
GEOMETRY_WELD_REL_TOL = 1e-9
MAX_COLLISION_PAIRS = 16384
MAX_CONTACTS = 16384
# Contact pruning is intentionally disabled for the formal protocol.  Genesis
# may report its own capacity/overflow state, but the evaluator must not hide
# contacts by configuring a positive pruning tolerance.
CONTACT_PRUNING_TOLERANCE_M: float | None = None
READBACK_TOLERANCE = 1e-9
STATE_RECORD_POLICY = "all_intended"
CHILD_TIMEOUT_SECONDS = 900
THREAD_ENV_KEYS = (
    "OMP_NUM_THREADS",
    "MKL_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "BLIS_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
    "QD_NUM_THREADS",
    "GS_PARA_LEVEL",
)
THREAD_ENV_VALUES = {
    **{key: "1" for key in THREAD_ENV_KEYS if key != "GS_PARA_LEVEL"},
    "GS_PARA_LEVEL": "0",
}
CPU_AFFINITY_ENV = "LAM_GENESIS_CPU_AFFINITY"
CPU_AFFINITY_WIDTH = 4
INPUT_IDENTITY_FIELDS = (
    "asset_key", "selection_rank", "selection_hash", "tier", "category",
    "object_release_id", "rel_path", "urdf_sha256", "package_relpath",
    "package_content_manifest_sha256", "expected_movable_joint_count",
)


class GenesisAdapterError(RuntimeError):
    """A state or frozen input cannot be certified by the Genesis adapter."""


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False)
            stream.write("\n")
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def atomic_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            for row in rows:
                stream.write(canonical_json(dict(row)) + "\n")
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _regular_file(path: Path) -> Path:
    if path.is_symlink() or not path.is_file():
        raise GenesisAdapterError(f"unsafe or missing regular file: {path}")
    return path.resolve(strict=True)


def _regular_dir(path: Path) -> Path:
    if path.is_symlink() or not path.is_dir():
        raise GenesisAdapterError(f"unsafe or missing regular directory: {path}")
    return path.resolve(strict=True)


def _package(dataset_root: Path, rel_path: str) -> Path:
    if not isinstance(rel_path, str) or not rel_path or "\\" in rel_path:
        raise GenesisAdapterError(f"unsafe release path: {rel_path!r}")
    relative = Path(rel_path)
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
        raise GenesisAdapterError(f"unsafe release path: {rel_path!r}")
    root = _regular_dir(dataset_root)
    release = _regular_dir(root / "released_outputs")
    candidate = root / "released_outputs" / relative
    current = root
    for part in ("released_outputs", *relative.parts):
        current /= part
        if current.is_symlink():
            raise GenesisAdapterError(f"symlink in release path: {current}")
    resolved = candidate.resolve(strict=True)
    try:
        resolved.relative_to(release)
    except ValueError as exc:
        raise GenesisAdapterError("release path escaped root") from exc
    return _regular_dir(resolved)


def package_binding(package: Path) -> dict[str, Any]:
    """Return a deterministic, symlink-free recursive package fingerprint."""

    package = _regular_dir(package)
    files: list[dict[str, Any]] = []
    for raw_current, dirs, names in os.walk(package, followlinks=False):
        current = Path(raw_current)
        dirs.sort()
        names.sort()
        if any((current / name).is_symlink() for name in dirs):
            raise GenesisAdapterError(f"directory symlink in package: {current}")
        for name in names:
            path = current / name
            if path.is_symlink() or not path.is_file():
                raise GenesisAdapterError(f"unsafe package file: {path}")
            resolved = path.resolve(strict=True)
            resolved.relative_to(package)
            files.append({
                "path": resolved.relative_to(package).as_posix(),
                "bytes": resolved.stat().st_size,
                "sha256": sha256_file(resolved),
            })
    return {
        "file_count": len(files),
        "total_bytes": sum(int(row["bytes"]) for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def load_source_cohort(
    source_records: Path = DEFAULT_SOURCE_RECORDS,
    source_manifest: Path = DEFAULT_SOURCE_MANIFEST,
) -> verifier.SourceCohort:
    config = verifier.VerifierConfig(source_records=source_records, source_manifest=source_manifest)
    return verifier._build_source_cohort(config)  # type: ignore[attr-defined]


def source_joints(source: verifier.SourceCohort, asset_key: str) -> list[dict[str, Any]]:
    rows = [dict(row) for (key, _), row in source.joints_by_key.items() if key == asset_key]
    return sorted(rows, key=lambda row: source.joint_order[(asset_key, str(row["joint_name"]))])


def bind_genesis_cache(output_root: Path) -> Path:
    """Bind Genesis cache below this run root before Genesis is imported."""

    expected = (output_root.resolve(strict=False) / "genesis-cache").resolve(strict=False)
    observed_raw = os.environ.get("GS_CACHE_FILE_PATH")
    if observed_raw is None:
        os.environ["GS_CACHE_FILE_PATH"] = str(expected)
    else:
        observed = Path(observed_raw)
        if not observed.is_absolute() or observed.resolve(strict=False) != expected:
            raise GenesisAdapterError(
                f"GS_CACHE_FILE_PATH must equal {expected}, observed {observed_raw!r}"
            )
    return expected


def bind_cpu_affinity() -> list[int]:
    """Pin each Genesis process to a small, explicitly recorded CPU set."""

    try:
        available = sorted(int(cpu) for cpu in os.sched_getaffinity(0))
    except (AttributeError, OSError) as exc:
        raise GenesisAdapterError(f"CPU affinity is unavailable: {type(exc).__name__}: {exc}") from exc
    raw = os.environ.get(CPU_AFFINITY_ENV)
    if raw is None:
        expected = available[: min(CPU_AFFINITY_WIDTH, len(available))]
    else:
        try:
            expected = sorted({int(token) for token in raw.split(",") if token.strip()})
        except ValueError as exc:
            raise GenesisAdapterError(f"invalid {CPU_AFFINITY_ENV}: {raw!r}") from exc
    if not expected or any(cpu not in available for cpu in expected):
        raise GenesisAdapterError(
            f"requested CPU affinity is not available: requested={expected}, available={available}"
        )
    try:
        os.sched_setaffinity(0, set(expected))
        observed = sorted(int(cpu) for cpu in os.sched_getaffinity(0))
    except (AttributeError, OSError) as exc:
        raise GenesisAdapterError(f"cannot set CPU affinity: {type(exc).__name__}: {exc}") from exc
    if observed != expected:
        raise GenesisAdapterError(f"CPU affinity readback mismatch: {observed} != {expected}")
    os.environ[CPU_AFFINITY_ENV] = ",".join(str(cpu) for cpu in expected)
    return observed


def bind_thread_environment() -> dict[str, str]:
    """Freeze native and Genesis-specific thread fan-out for this protocol."""

    for key, value in THREAD_ENV_VALUES.items():
        os.environ[key] = value
    return {key: os.environ[key] for key in THREAD_ENV_KEYS}


def _git_output(root: Path, *arguments: str) -> str:
    try:
        completed = subprocess.run(
            ["git", "-C", str(root), *arguments],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise GenesisAdapterError(
            f"cannot bind Genesis source git identity: {type(exc).__name__}: {exc}"
        ) from exc
    return completed.stdout.strip()


def genesis_source_identity(module_file: Path) -> dict[str, Any]:
    root = _regular_dir(GENESIS_SOURCE_ROOT)
    module = _regular_file(module_file)
    try:
        module.relative_to(root)
    except ValueError as exc:
        raise GenesisAdapterError(f"Genesis module is outside frozen source root: {module}") from exc
    commit = _git_output(root, "rev-parse", "HEAD")
    if commit != EXPECTED_GENESIS_COMMIT:
        raise GenesisAdapterError(f"Genesis source commit mismatch: {commit}")
    tree = _git_output(root, "rev-parse", "HEAD^{tree}")
    status = _git_output(root, "status", "--short", "--untracked-files=no")
    if status:
        raise GenesisAdapterError("Genesis tracked source tree is dirty")
    return {
        "module_path": str(module),
        "source_root": str(root),
        "git_commit": commit,
        "git_tree": tree,
        "tracked_tree_clean": True,
        "package_tree_identity": f"git-tree:{tree}",
    }


def current_code_identity() -> dict[str, dict[str, str]]:
    components = {
        "runner": SCRIPT,
        "static": Path(static.__file__),
        "geometry": Path(geometry.__file__),
        "verifier": Path(verifier.__file__),
    }
    return {
        name: {"path": str(_regular_file(path)), "sha256": sha256_file(_regular_file(path))}
        for name, path in components.items()
    }


def genesis_runtime_binding(
    *, require_current_interpreter: bool = True,
    expected_cache_path: Path | None = None,
) -> dict[str, Any]:
    """Check the frozen Genesis environment; this function never probes PyBullet."""

    cpu_affinity = bind_cpu_affinity()
    thread_environment = bind_thread_environment()
    expected = DEFAULT_GENESIS_PYTHON.resolve(strict=True)
    observed = Path(sys.executable).resolve(strict=True)
    if require_current_interpreter and observed != expected:
        raise GenesisAdapterError(f"wrong Genesis launcher: {observed} != {expected}")
    try:
        import genesis as gs
        import rtree
        import scipy
        import trimesh
    except Exception as exc:  # noqa: BLE001
        raise GenesisAdapterError(f"Genesis import failed: {type(exc).__name__}: {exc}") from exc
    cache_raw = os.environ.get("GS_CACHE_FILE_PATH")
    if cache_raw is None or not Path(cache_raw).is_absolute():
        raise GenesisAdapterError("GS_CACHE_FILE_PATH must be an explicit absolute path")
    cache_path = Path(cache_raw).resolve(strict=False)
    if expected_cache_path is not None and cache_path != expected_cache_path.resolve(strict=False):
        raise GenesisAdapterError("Genesis cache path does not match the frozen output root")
    result = {
        # These names and values are part of the verifier-facing binding.  In
        # particular, do not change ``engine`` to a display-name variant.
        "engine": "genesis",
        "genesis_version": str(gs.__version__),
        "version": str(gs.__version__),
        "trimesh_version": str(trimesh.__version__),
        "rtree_version": str(rtree.__version__),
        "scipy_version": str(scipy.__version__),
        "launcher": str(expected),
        "launcher_sha256": sha256_file(expected),
        "backend": "cpu",
        "device": "cpu",
        "precision": GENESIS_PRECISION,
        "collision_detection": "contact_penetration",
        "penetration_threshold_m": PENETRATION_THRESHOLD_M,
        "q_readback_tolerance": READBACK_TOLERANCE,
        "engine_protocol_id": ENGINE_PROTOCOL_ID,
        "python": {
            "executable": str(observed),
            "executable_sha256": sha256_file(observed),
            "version": platform.python_version(),
            "implementation": platform.python_implementation(),
        },
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
        },
        "cpu_affinity": cpu_affinity,
        "cpu_count": os.cpu_count(),
        "thread_environment": thread_environment,
        "genesis_source": genesis_source_identity(Path(gs.__file__)),
        "cache": {
            "mode": "GS_CACHE_FILE_PATH",
            "path": str(cache_path),
        },
        "morph": {
            "collision": True,
            "visualization": False,
            "fixed": True,
            "merge_fixed_links": False,
            "convexify": False,
            "decimate": False,
            "watertighten": 0,
            "merge_submeshes_for_collision": False,
            "recompute_inertia": False,
            "align": False,
        },
        "rigid_options": {
            "gravity": [0.0, 0.0, 0.0],
            "enable_collision": True,
            "enable_self_collision": True,
            "enable_neutral_collision": True,
            "enable_adjacent_collision": True,
            "max_collision_pairs": MAX_COLLISION_PAIRS,
            "max_contacts": MAX_CONTACTS,
            "contact_pruning_tolerance": None,
        },
    }
    if result["genesis_version"] != GENESIS_VERSION:
        raise GenesisAdapterError(f"Genesis version mismatch: {result['genesis_version']}")
    if result["trimesh_version"] != TRIMESH_VERSION or result["rtree_version"] != RTREE_VERSION:
        raise GenesisAdapterError("Genesis geometry dependency version mismatch")
    return result


def build_frozen_manifest(
    *, dataset_root: Path = DEFAULT_DATASET_ROOT,
    source_records: Path = DEFAULT_SOURCE_RECORDS,
    source_manifest: Path = DEFAULT_SOURCE_MANIFEST,
    qualification_smoke: bool = False,
    expected_cache_path: Path | None = None,
) -> dict[str, Any]:
    """Freeze membership by Table-3 selection rank, never JSONL physical order."""

    source = load_source_cohort(source_records, source_manifest)
    if len(source.records_by_rank) != SAMPLE_SIZE or len(source.joints_by_key) != JOINT_COUNT:
        raise GenesisAdapterError("frozen Table-3 denominator mismatch")
    movable_assets = {asset_key for asset_key, _ in source.joints_by_key}
    if len(movable_assets) != EXPECTED_MOVABLE_ASSET_COUNT:
        raise GenesisAdapterError("frozen movable-asset denominator mismatch")
    runtime = genesis_runtime_binding(expected_cache_path=expected_cache_path)
    dataset_root = _regular_dir(dataset_root)
    items: list[dict[str, Any]] = []
    for rank in range(1, SAMPLE_SIZE + 1):
        row = source.records_by_rank[rank]
        asset_key = str(row["asset_key"])
        package = _package(dataset_root, str(row["rel_path"]))
        urdf = _regular_file(package / "generated.urdf")
        if sha256_file(urdf) != row.get("urdf_sha256"):
            raise GenesisAdapterError(f"URDF drift at selection_rank={rank}")
        binding = package_binding(package)
        joint_count = len(source_joints(source, asset_key))
        item = {key: row.get(key) for key in (
            "asset_key", "selection_rank", "selection_hash", "tier", "category",
            "object_release_id", "rel_path", "urdf_sha256",
        )}
        item.update({
            "package_relpath": f"released_outputs/{row['rel_path']}",
            "primary_urdf_relpath": "generated.urdf",
            "package_binding": binding,
            "package_content_manifest_sha256": binding["content_manifest_sha256"],
            "expected_movable_joint_count": joint_count,
            "source_record_sha256": canonical_sha256(row),
            "source_manifest_record_sha256": canonical_sha256(source.manifest_records_by_rank[rank]),
        })
        item["input_identity_sha256"] = canonical_sha256({field: item.get(field) for field in INPUT_IDENTITY_FIELDS})
        items.append(item)
    ordered_hash = canonical_sha256([item["asset_key"] for item in items])
    if ordered_hash != source.ordered_keys_sha256:
        raise GenesisAdapterError("selection-rank order hash mismatch")
    protocol = _regular_file(PROTOCOL_DOCUMENT)
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "protocol_id": PROTOCOL_ID,
        "status": "FROZEN",
        "qualification_smoke": bool(qualification_smoke),
        "sample_size": SAMPLE_SIZE,
        "joint_count": JOINT_COUNT,
        "state_record_policy": STATE_RECORD_POLICY,
        "strict_state_record_policy": "all_intended",
        "strict_rest_state_count_per_asset": 1,
        "sobol_state_count_per_movable_asset": SOBOL_STATE_COUNT_PER_ASSET,
        "sobol_movable_asset_count": EXPECTED_MOVABLE_ASSET_COUNT,
        "sobol_state_expected": SOBOL_STATE_COUNT_TOTAL,
        "strict_state_expected": STRICT_STATE_COUNT_TOTAL,
        "sobol_protocol": {
            "generator": "scipy.stats.qmc.Sobol.random_base2",
            "scramble": SOBOL_SCRAMBLE,
            "seed": SOBOL_SEED,
            "base2_m": 6,
            "joint_interval_source": "frozen_table3_range_lower_upper",
        },
        "dataset_root": str(dataset_root),
        "source": {
            "table3_asset_records_path": str(_regular_file(source_records)),
            "table3_asset_records_sha256": source.records_sha256,
            "table3_manifest_path": str(_regular_file(source_manifest)),
            "table3_manifest_sha256": source.manifest_sha256,
            "table3_manifest_content_sha256": source.manifest_content_sha256,
            "ordered_selected_asset_keys_sha256": source.ordered_keys_sha256,
            "protocol_snapshot_name": "protocol_snapshot.md",
            "protocol_snapshot_sha256": sha256_file(protocol),
        },
        "input_identity_fields": list(INPUT_IDENTITY_FIELDS),
        "ordered_selected_asset_keys_sha256": ordered_hash,
        "items": items,
        "items_sha256": canonical_sha256(items),
        "engine_protocol_id": ENGINE_PROTOCOL_ID,
        "runtime_binding": runtime,
        "code_identity": current_code_identity(),
        # Keep a compact protocol block for human readers while the two
        # top-level fields above remain the verifier authority.
        "engine_protocol": {
            "engine_protocol_id": ENGINE_PROTOCOL_ID,
            "engine": "genesis", "version": GENESIS_VERSION, "backend": "cpu", "precision": GENESIS_PRECISION,
            "gravity": [0.0, 0.0, 0.0], "q_neutral": 0.0,
            "collision_detection": "contact_penetration",
            "penetration_threshold_m": PENETRATION_THRESHOLD_M,
            "direct_parent_child_policy": "exclude_only_direct_xml_parent_child",
            "merge_fixed_links": False, "convexify": False, "decimate": False, "watertighten": 0,
            "enable_neutral_collision": True, "enable_adjacent_collision": True,
            "contact_pruning_tolerance_m": None,
            "max_collision_pairs": MAX_COLLISION_PAIRS, "max_contacts": MAX_CONTACTS,
            "unmapped_or_overflow_policy": "unexecuted_fail_closed",
        },
        "table4b_protocol": {
            "d_visual": "q0_loadable_visual_union_aabb_diagonal_v1",
            "samples_per_direction": GEOMETRY_SAMPLES_PER_DIRECTION,
            "weld_relative_tolerance": GEOMETRY_WELD_REL_TOL,
            "exact_surface_backend": geometry.EXACT_BACKEND,
        },
        "registries": {
            "release_mechanical_receipt_registry": [],
            "method_specific_allowance_registry": [],
            "placeholder_mass_registry": [],
        },
        "runtime": runtime,
    }
    payload = dict(manifest)
    manifest["manifest_content_sha256"] = canonical_sha256(payload)
    return manifest


def write_frozen_manifest(output_root: Path, manifest: Mapping[str, Any]) -> None:
    if output_root.exists():
        raise GenesisAdapterError(f"refusing to overwrite output root: {output_root}")
    output_root.mkdir(parents=True, exist_ok=False)
    snapshot = _regular_file(PROTOCOL_DOCUMENT)
    if sha256_file(snapshot) != manifest.get("source", {}).get("protocol_snapshot_sha256"):
        raise GenesisAdapterError("protocol changed during freeze")
    atomic_json(output_root / "frozen_manifest.json", dict(manifest))
    (output_root / "protocol_snapshot.md").write_bytes(snapshot.read_bytes())


def _finite_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _tensor_flat(value: Any) -> list[Any]:
    if hasattr(value, "detach"):
        value = value.detach().cpu().reshape(-1).tolist()
    elif hasattr(value, "reshape") and hasattr(value, "tolist"):
        value = value.reshape(-1).tolist()
    elif hasattr(value, "tolist"):
        value = value.tolist()
    if not isinstance(value, list):
        raise GenesisAdapterError("Genesis readback is not a sequence")
    result: list[Any] = []
    for item in value:
        result.extend(_tensor_flat(item) if isinstance(item, list) else [item])
    return result


def _xml_direct_pairs(urdf_path: Path) -> set[tuple[str, str]]:
    root = ET.parse(urdf_path).getroot()
    links = {node.get("name", "").strip() for node in root.findall("link")}
    if not links or "" in links:
        raise GenesisAdapterError("invalid URDF link names")
    direct: set[tuple[str, str]] = set()
    for joint in root.findall("joint"):
        parents, children = joint.findall("parent"), joint.findall("child")
        if len(parents) != 1 or len(children) != 1:
            raise GenesisAdapterError("invalid joint parent/child cardinality")
        pair = tuple(sorted((parents[0].get("link", "").strip(), children[0].get("link", "").strip())))
        if pair[0] not in links or pair[1] not in links or pair[0] == pair[1]:
            raise GenesisAdapterError("invalid joint parent/child link")
        direct.add(pair)
    return direct


def _xml_collision_inventory(urdf_path: Path) -> dict[str, Any]:
    root = ET.parse(urdf_path).getroot()
    links: set[str] = set()
    element_count = 0
    for link in root.findall("link"):
        name = str(link.get("name", "")).strip()
        collisions = link.findall("collision")
        if collisions:
            if not name:
                raise GenesisAdapterError("collision-bearing URDF link has no name")
            links.add(name)
            element_count += len(collisions)
    return {
        "source_collision_element_count": element_count,
        "source_collision_link_count": len(links),
        "source_collision_links": sorted(links),
    }


class GenesisTable4aAdapter:
    """Version-pinned, one-process Genesis collision/readback adapter."""

    def __init__(self, urdf_path: Path, runtime: Mapping[str, Any]) -> None:
        self.urdf_path = _regular_file(urdf_path)
        self.runtime = dict(runtime)
        self.gs: Any | None = None
        self.scene: Any | None = None
        self.entity: Any | None = None
        self.dof_order: list[dict[str, Any]] = []
        self.geom_to_link: dict[int, str] = {}
        self.direct_pairs: set[tuple[str, str]] = set()
        self.eligible_pairs: set[tuple[int, int]] = set()
        self.valid_pairs: set[tuple[int, int]] = set()
        self.mapping: dict[str, Any] = {}
        self.load_time_seconds: float | None = None

    @staticmethod
    def _pair(a: int, b: int) -> tuple[int, int]:
        return (a, b) if a < b else (b, a)

    def build(self) -> None:
        if genesis_runtime_binding() != self.runtime:
            raise GenesisAdapterError("Genesis runtime identity changed after freeze")
        import genesis as gs
        if bool(getattr(gs, "_initialized", False)):
            raise GenesisAdapterError("Genesis already initialized; use one asset per process")
        started = time.monotonic()
        gs.init(backend=gs.cpu, precision=GENESIS_PRECISION, logging_level=logging.ERROR, seed=20260813)
        self.gs = gs
        options = gs.options.RigidOptions(
            gravity=(0.0, 0.0, 0.0), enable_collision=True, enable_self_collision=True,
            enable_neutral_collision=True, enable_adjacent_collision=True,
            max_collision_pairs=MAX_COLLISION_PAIRS, max_contacts=MAX_CONTACTS,
            contact_pruning_tolerance=CONTACT_PRUNING_TOLERANCE_M,
        )
        self.scene = gs.Scene(rigid_options=options, show_viewer=False)
        morph = gs.morphs.URDF(
            file=str(self.urdf_path), fixed=True, visualization=False, collision=True,
            merge_fixed_links=False, convexify=False, decimate=False, watertighten=0,
            recompute_inertia=False, align=False,
        )
        self.entity = self.scene.add_entity(morph)
        self.scene.build()
        self.load_time_seconds = time.monotonic() - started
        self._map_dofs()
        self._map_pairs()

    def close(self) -> None:
        if self.scene is not None:
            self.scene.destroy()
        if self.gs is not None:
            self.gs.destroy()
        self.scene = None
        self.entity = None

    def _map_pairs(self) -> None:
        if self.entity is None:
            raise GenesisAdapterError("Genesis entity is not built")
        direct = _xml_direct_pairs(self.urdf_path)
        inventory = _xml_collision_inventory(self.urdf_path)
        self.geom_to_link = {int(geom.idx): str(geom.link.name) for geom in self.entity.geoms}
        if len(self.geom_to_link) != len(self.entity.geoms) or any(not value for value in self.geom_to_link.values()):
            raise GenesisAdapterError("Genesis geom-to-link mapping is incomplete")
        genesis_links = set(self.geom_to_link.values())
        source_links = set(inventory["source_collision_links"])
        if genesis_links != source_links:
            missing = sorted(source_links - genesis_links)
            extra = sorted(genesis_links - source_links)
            raise GenesisAdapterError(
                f"Genesis collision-link inventory mismatch: missing={missing}, extra={extra}"
            )
        self.direct_pairs = direct
        for a, b in itertools.combinations(sorted(self.geom_to_link), 2):
            links = tuple(sorted((self.geom_to_link[a], self.geom_to_link[b])))
            if links[0] != links[1] and links not in direct:
                self.eligible_pairs.add(self._pair(a, b))
        raw_valid = getattr(getattr(self.entity.solver, "collider", None), "_valid_collision_pairs", None)
        if raw_valid is None:
            raise GenesisAdapterError("Genesis valid-pair table is unavailable")
        for row in raw_valid:
            if len(row) != 2:
                raise GenesisAdapterError("Genesis valid-pair table is malformed")
            self.valid_pairs.add(self._pair(int(row[0]), int(row[1])))
        missing = self.eligible_pairs - self.valid_pairs
        mapped = len(self.eligible_pairs & self.valid_pairs)
        self.mapping = {
            "status": "COMPLETE" if not missing else "FAILED",
            # Keep explicit counts rather than inferring them from booleans;
            # the verifier independently checks these closure fields.
            "eligible_pair_count": len(self.eligible_pairs),
            "mapped_pair_count": mapped,
            "unmapped_pair_count": len(missing),
            "overflow_count": 0,
            "pruned_pair_count": 0,
            # Readable aliases are retained for diagnostic consumers.
            "eligible": len(self.eligible_pairs),
            "mapped": mapped,
            "unmapped": len(missing),
            "overflow": False,
            "pruned": False,
            **inventory,
            "genesis_geom_count": len(self.geom_to_link),
            "genesis_collision_link_count": len(genesis_links),
        }
        if missing:
            self.mapping["reason"] = "Genesis filtered eligible non-adjacent collision geometry"
            raise GenesisAdapterError(self.mapping["reason"])

    def _map_dofs(self) -> None:
        if self.entity is None:
            raise GenesisAdapterError("Genesis entity is not built")
        names: list[str | None] = [None] * int(self.entity.n_dofs)
        for joint in self.entity.joints:
            for raw_index in joint.dofs_idx_local:
                index = int(raw_index)
                if index < 0 or index >= len(names) or names[index] is not None:
                    raise GenesisAdapterError("Genesis joint-to-DoF mapping is malformed")
                names[index] = str(joint.name)
        if any(name is None or not name for name in names):
            raise GenesisAdapterError("Genesis joint-to-DoF mapping is incomplete")
        self.dof_order = [
            {"dof_index": index, "joint_name": str(name)}
            for index, name in enumerate(names)
        ]

    def _contacts(self) -> tuple[float, dict[str, Any]]:
        if self.entity is None:
            raise GenesisAdapterError("Genesis entity is not built")
        pairs = self.entity.detect_collision()
        self.entity.solver.check_errno()
        contacts = self.entity.get_contacts()
        if not isinstance(contacts, Mapping):
            raise GenesisAdapterError("Genesis contact readback is not a mapping")
        geom_a = _tensor_flat(contacts.get("geom_a"))
        geom_b = _tensor_flat(contacts.get("geom_b"))
        penetration = _tensor_flat(contacts.get("penetration"))
        if not len(geom_a) == len(geom_b) == len(penetration):
            raise GenesisAdapterError("Genesis contact arrays have inconsistent lengths")
        detected = {self._pair(int(row[0]), int(row[1])) for row in pairs}
        reported = {self._pair(int(a), int(b)) for a, b in zip(geom_a, geom_b)}
        if detected != reported:
            raise GenesisAdapterError("detect_collision/contact_data pair mismatch")
        maximum = 0.0
        eligible_contact_count = 0
        excluded_direct_parent_child_contact_count = 0
        for a, b, raw in zip(geom_a, geom_b, penetration):
            pair = self._pair(int(a), int(b))
            if int(a) not in self.geom_to_link or int(b) not in self.geom_to_link:
                raise GenesisAdapterError("contact references unknown geometry")
            value = _finite_float(raw)
            if value is None or value < 0.0:
                raise GenesisAdapterError("contact penetration is non-finite or negative")
            if pair in self.eligible_pairs:
                eligible_contact_count += 1
                maximum = max(maximum, value)
            else:
                links = tuple(sorted((self.geom_to_link[int(a)], self.geom_to_link[int(b)])))
                if links in self.direct_pairs:
                    excluded_direct_parent_child_contact_count += 1
        if len(detected) >= MAX_COLLISION_PAIRS or len(geom_a) >= MAX_CONTACTS:
            raise GenesisAdapterError("contact buffer reached frozen capacity")
        return maximum, {
            "status": "COMPLETE", "success": True, "finite": True,
            "overflow": False, "pruning": False,
            "overflow_count": 0, "pruning_count": 0,
            "detected_geom_pair_count": len(detected), "contact_point_count": len(geom_a),
            "raw_contact_count": len(geom_a),
            "eligible_contact_count": eligible_contact_count,
            "excluded_direct_parent_child_contact_count": excluded_direct_parent_child_contact_count,
        }

    def _observe_configuration(
        self, intended_values: Sequence[float],
    ) -> tuple[list[float], float, float, dict[str, Any]]:
        if self.entity is None or self.gs is None:
            raise GenesisAdapterError("Genesis adapter is not built")
        intended = [_finite_float(value) for value in intended_values]
        if len(intended) != int(self.entity.n_dofs) or any(value is None for value in intended):
            raise GenesisAdapterError("intended full DoF vector is incomplete or non-finite")
        intended_floats = [float(value) for value in intended]
        self.entity.set_dofs_position(intended_floats, zero_velocity=True)
        readback = [_finite_float(x) for x in _tensor_flat(self.entity.get_dofs_position())]
        if len(readback) != len(intended_floats) or any(value is None for value in readback):
            raise GenesisAdapterError("Genesis full DoF readback is incomplete")
        q_values = [float(value) for value in readback]
        q_error = max(
            (abs(observed - expected) for observed, expected in zip(q_values, intended_floats)),
            default=0.0,
        )
        if q_error > READBACK_TOLERANCE:
            raise GenesisAdapterError("Genesis full DoF readback failed")
        maximum, contact = self._contacts()
        return q_values, q_error, maximum, contact

    def _observation_fields(
        self, *, intended: Sequence[float], q_values: Sequence[float],
        q_error: float, maximum: float, contact: Mapping[str, Any],
        target_dof_index: int | None,
    ) -> dict[str, Any]:
        source_mapping = sorted((int(geom), link) for geom, link in self.geom_to_link.items())
        intended_values = [float(value) for value in intended]
        readback_values = [float(value) for value in q_values]
        intended_hash = canonical_sha256(intended_values)
        readback_hash = canonical_sha256(readback_values)
        return {
            "executed": True,
            "illegal_collision": maximum > PENETRATION_THRESHOLD_M,
            "clearance_normalized": None,
            "max_eligible_penetration_m": maximum,
            "runtime_binding": dict(self.runtime),
            "mapping": dict(self.mapping),
            "contact_readback": dict(contact),
            "readback": {"status": "COMPLETE", "success": True, "finite": True, "max_abs_error": q_error},
            "q_intended_values": intended_values,
            "q_readback_values": readback_values,
            "q_intended_values_sha256": intended_hash,
            "q_readback_values_sha256": readback_hash,
            "q_values_sha256": readback_hash,
            "target_dof_index": target_dof_index,
            "joint_dof_order": list(self.dof_order),
            "joint_dof_order_sha256": canonical_sha256(self.dof_order),
            "source_link_mapping_sha256": canonical_sha256(source_mapping),
            "contact_readout_source": "detect_collision_then_get_contacts",
            "q_readback_max_abs_error": q_error,
            "observation_status": "COMPLETE",
            "overflow_or_pruning_detected": False,
            "clearance_status": "N/E",
            "clearance_reason": "Genesis contact penetration has no signed clearance; clearance is N/E.",
            "raw_contact_count": int(contact["raw_contact_count"]),
            "eligible_contact_count": int(contact["eligible_contact_count"]),
            "excluded_direct_parent_child_contact_count": int(contact["excluded_direct_parent_child_contact_count"]),
        }

    def state(self, *, item: Mapping[str, Any], joint_name: str, sample_index: int, value: float) -> dict[str, Any]:
        if self.entity is None:
            raise GenesisAdapterError("Genesis adapter is not built")
        joint = self.entity.get_joint(name=joint_name)
        if int(joint.n_dofs) != 1:
            raise GenesisAdapterError(f"joint {joint_name!r} has {joint.n_dofs} Genesis DOFs")
        index = int(joint.dofs_idx_local[0])
        intended = [0.0] * int(self.entity.n_dofs)
        if index < 0 or index >= len(intended):
            raise GenesisAdapterError("Genesis target DoF index is outside the full q vector")
        intended[index] = value
        q_values, q_error, maximum, contact = self._observe_configuration(intended)
        return {
            "state_key": f"{item['asset_key']}::{joint_name}::{sample_index}",
            "protocol_id": PROTOCOL_ID, "engine_protocol_id": ENGINE_PROTOCOL_ID,
            "asset_key": item["asset_key"], "selection_rank": item["selection_rank"],
            "input_identity_sha256": item["input_identity_sha256"], "joint_name": joint_name,
            "phase": "joint_full_range", "sample_index": sample_index, "joint_value": value,
            **self._observation_fields(
                intended=intended, q_values=q_values, q_error=q_error,
                maximum=maximum, contact=contact, target_dof_index=index,
            ),
            "terminal": True, "status": "completed",
        }

    def strict_state(
        self, *, item: Mapping[str, Any], phase: str, sample_index: int,
        intended: Sequence[float],
    ) -> dict[str, Any]:
        if phase not in {"rest", "multi_joint_sobol"}:
            raise GenesisAdapterError(f"invalid strict phase: {phase}")
        if (phase == "rest" and sample_index != 0) or (
            phase == "multi_joint_sobol" and not 0 <= sample_index < SOBOL_STATE_COUNT_PER_ASSET
        ):
            raise GenesisAdapterError("invalid strict sample index")
        q_values, q_error, maximum, contact = self._observe_configuration(intended)
        return {
            "strict_state_key": f"{item['asset_key']}::strict::{phase}::{sample_index}",
            "protocol_id": PROTOCOL_ID,
            "engine_protocol_id": ENGINE_PROTOCOL_ID,
            "asset_key": item["asset_key"],
            "selection_rank": item["selection_rank"],
            "input_identity_sha256": item["input_identity_sha256"],
            "phase": phase,
            "sample_index": sample_index,
            "sobol_seed": SOBOL_SEED,
            "sobol_scramble": SOBOL_SCRAMBLE,
            "sobol_dimension": len(intended),
            **self._observation_fields(
                intended=intended, q_values=q_values, q_error=q_error,
                maximum=maximum, contact=contact, target_dof_index=None,
            ),
            "terminal": True,
            "status": "completed",
        }


def unexecuted_state(item: Mapping[str, Any], joint_name: str, index: int, value: float | None, runtime: Mapping[str, Any], reason: str) -> dict[str, Any]:
    return {
        "state_key": f"{item['asset_key']}::{joint_name}::{index}",
        "protocol_id": PROTOCOL_ID, "engine_protocol_id": ENGINE_PROTOCOL_ID,
        "asset_key": item["asset_key"], "selection_rank": item["selection_rank"],
        "input_identity_sha256": item["input_identity_sha256"], "joint_name": joint_name,
        "phase": "joint_full_range", "sample_index": index, "joint_value": value,
        "executed": False, "illegal_collision": None, "clearance_normalized": None,
        "max_eligible_penetration_m": None, "runtime_binding": dict(runtime),
        "mapping": {
            "status": "N/E", "eligible_pair_count": 0, "mapped_pair_count": 0,
            "unmapped_pair_count": 0, "overflow_count": 0, "pruned_pair_count": 0,
            "eligible": 0, "mapped": 0, "unmapped": 0, "overflow": False,
            "pruned": False, "reason": reason,
        },
        "contact_readback": {
            "status": "N/E", "success": False, "finite": False,
            "overflow": False, "pruning": False, "overflow_count": 0,
            "pruning_count": 0, "reason": reason,
        },
        "readback": {"status": "N/E", "success": False, "finite": False, "reason": reason},
        "q_intended_values": [],
        "q_readback_values": [],
        "q_intended_values_sha256": canonical_sha256([]),
        "q_readback_values_sha256": canonical_sha256([]),
        "q_values_sha256": canonical_sha256([]),
        "target_dof_index": None,
        "joint_dof_order": [],
        "joint_dof_order_sha256": canonical_sha256([]),
        "source_link_mapping_sha256": canonical_sha256([]),
        "contact_readout_source": "detect_collision_then_get_contacts",
        "q_readback_max_abs_error": None,
        "observation_status": "N/E",
        "overflow_or_pruning_detected": False,
        "clearance_status": "N/E",
        "clearance_reason": f"Genesis observation unavailable: {reason}",
        "raw_contact_count": 0,
        "eligible_contact_count": 0,
        "excluded_direct_parent_child_contact_count": 0,
        "terminal": True, "status": "not_executed",
    }


def unexecuted_strict_state(
    item: Mapping[str, Any], phase: str, index: int,
    runtime: Mapping[str, Any], reason: str, *, dimension: int,
    joint_dof_order: Sequence[Mapping[str, Any]] = (),
) -> dict[str, Any]:
    order = [dict(row) for row in joint_dof_order]
    return {
        "strict_state_key": f"{item['asset_key']}::strict::{phase}::{index}",
        "protocol_id": PROTOCOL_ID,
        "engine_protocol_id": ENGINE_PROTOCOL_ID,
        "asset_key": item["asset_key"],
        "selection_rank": item["selection_rank"],
        "input_identity_sha256": item["input_identity_sha256"],
        "phase": phase,
        "sample_index": index,
        "sobol_seed": SOBOL_SEED,
        "sobol_scramble": SOBOL_SCRAMBLE,
        "sobol_dimension": dimension,
        "executed": False,
        "illegal_collision": None,
        "clearance_normalized": None,
        "max_eligible_penetration_m": None,
        "runtime_binding": dict(runtime),
        "mapping": {
            "status": "N/E", "eligible_pair_count": 0, "mapped_pair_count": 0,
            "unmapped_pair_count": 0, "overflow_count": 0, "pruned_pair_count": 0,
            "eligible": 0, "mapped": 0, "unmapped": 0, "overflow": False,
            "pruned": False, "reason": reason,
        },
        "contact_readback": {
            "status": "N/E", "success": False, "finite": False,
            "overflow": False, "pruning": False, "overflow_count": 0,
            "pruning_count": 0, "reason": reason,
        },
        "readback": {"status": "N/E", "success": False, "finite": False, "reason": reason},
        "q_intended_values": [],
        "q_readback_values": [],
        "q_intended_values_sha256": canonical_sha256([]),
        "q_readback_values_sha256": canonical_sha256([]),
        "q_values_sha256": canonical_sha256([]),
        "target_dof_index": None,
        "joint_dof_order": order,
        "joint_dof_order_sha256": canonical_sha256(order),
        "source_link_mapping_sha256": canonical_sha256([]),
        "contact_readout_source": "detect_collision_then_get_contacts",
        "q_readback_max_abs_error": None,
        "observation_status": "N/E",
        "overflow_or_pruning_detected": False,
        "clearance_status": "N/E",
        "clearance_reason": f"Genesis observation unavailable: {reason}",
        "raw_contact_count": 0,
        "eligible_contact_count": 0,
        "excluded_direct_parent_child_contact_count": 0,
        "terminal": True,
        "status": "not_executed",
    }


def sobol_intended_vectors(
    adapter: GenesisTable4aAdapter, joints: Sequence[Mapping[str, Any]],
) -> list[list[float]]:
    if adapter.entity is None:
        raise GenesisAdapterError("Genesis adapter is not built")
    dimension = len(joints)
    if dimension == 0:
        return []
    if dimension != int(adapter.entity.n_dofs):
        raise GenesisAdapterError("frozen joint count does not equal Genesis DoF count")
    dof_by_name = {str(row["joint_name"]): int(row["dof_index"]) for row in adapter.dof_order}
    source_names = [str(row["joint_name"]) for row in joints]
    if len(dof_by_name) != dimension or set(dof_by_name) != set(source_names):
        raise GenesisAdapterError("frozen joint names do not close against Genesis DoF order")
    intervals: list[tuple[float, float]] = []
    for row in joints:
        lower = _finite_float(row.get("range_lower"))
        upper = _finite_float(row.get("range_upper"))
        values = source_values(row)
        if lower is None or upper is None or upper <= lower or values is None:
            raise GenesisAdapterError(f"frozen Sobol range is invalid for {row.get('joint_name')!r}")
        if not math.isclose(values[0], lower, rel_tol=0.0, abs_tol=1e-12) or not math.isclose(
            values[-1], upper, rel_tol=0.0, abs_tol=1e-12
        ):
            raise GenesisAdapterError(f"frozen sweep endpoints drifted for {row.get('joint_name')!r}")
        intervals.append((lower, upper))
    try:
        from scipy.stats import qmc
    except Exception as exc:  # noqa: BLE001
        raise GenesisAdapterError(f"SciPy Sobol import failed: {type(exc).__name__}: {exc}") from exc
    unit = qmc.Sobol(d=dimension, scramble=SOBOL_SCRAMBLE, seed=SOBOL_SEED).random_base2(m=6)
    if len(unit) != SOBOL_STATE_COUNT_PER_ASSET:
        raise GenesisAdapterError("Sobol generator did not return 64 states")
    vectors: list[list[float]] = []
    for unit_vector in unit:
        intended = [0.0] * dimension
        for scalar, interval, name in zip(unit_vector, intervals, source_names):
            lower, upper = interval
            intended[dof_by_name[name]] = float(lower + float(scalar) * (upper - lower))
        vectors.append(intended)
    if len({canonical_sha256(vector) for vector in vectors}) != SOBOL_STATE_COUNT_PER_ASSET:
        raise GenesisAdapterError("Sobol intended q vectors are not unique")
    return vectors


def source_values(row: Mapping[str, Any]) -> list[float] | None:
    values = row.get("sample_values")
    if not isinstance(values, list) or len(values) != STATE_SAMPLES_PER_JOINT:
        return None
    parsed = [_finite_float(value) for value in values]
    return [float(value) for value in parsed] if all(value is not None for value in parsed) else None


def joint_record(item: Mapping[str, Any], source_joint: Mapping[str, Any], states: Sequence[Mapping[str, Any]], portable: bool, dynamics: bool) -> dict[str, Any]:
    executed = [row for row in states if row.get("executed") is True]
    full_range = len(executed) == STATE_SAMPLES_PER_JOINT and all(row.get("illegal_collision") is False for row in executed)
    bounded = source_joint.get("joint_type") != "continuous"
    endpoints = {int(row["sample_index"]): row for row in executed}
    reachable = all(endpoints.get(i, {}).get("illegal_collision") is False for i in (0, STATE_SAMPLES_PER_JOINT - 1)) if bounded else None
    return {
        "protocol_id": PROTOCOL_ID, "asset_key": item["asset_key"], "selection_rank": item["selection_rank"],
        "input_identity_sha256": item["input_identity_sha256"], "joint_name": source_joint["joint_name"],
        "joint_type": source_joint.get("joint_type"), "table3_joint_pass": bool(source_joint.get("joint_level_pass")),
        "intended_state_count": STATE_SAMPLES_PER_JOINT, "executed_state_count": len(executed),
        "full_range_cf_pass": full_range, "joint_limit_portable": portable, "dynamics_present": dynamics,
        "bounded": bounded, "limit_reachable": reachable, "terminal": True,
        "status": "completed" if len(executed) == STATE_SAMPLES_PER_JOINT else "partial",
    }


def _state_gate_reason(
    static_record: Mapping[str, Any], geometry_record: Mapping[str, Any],
    source_record: Mapping[str, Any], joint_count: int,
) -> str | None:
    issues: list[str] = []
    table2 = static_record.get("table2_supplementary")
    table2 = table2 if isinstance(table2, Mapping) else {}
    visual = table2.get("visual_bearing_collision_coverage")
    visual = visual if isinstance(visual, Mapping) else {}
    limits = table2.get("joint_limit_portability")
    limits = limits if isinstance(limits, Mapping) else {}
    resource = static_record.get("resource_closure")
    resource = resource if isinstance(resource, Mapping) else {}
    if source_record.get("parse_success") is not True or source_record.get("tree_valid") is not True:
        issues.append("frozen Table-3 tree/parse closure failed")
    if static_record.get("status") != "completed" or static_record.get("parse", {}).get("success") is not True:
        issues.append("static URDF parse did not complete")
    if visual.get("asset_pass") is not True or visual.get("link_extraction_complete") is not True:
        issues.append("visual-bearing collision coverage is incomplete")
    if resource.get("complete") is not True or resource.get("status") != "COMPLETE":
        issues.append("resource closure is incomplete")
    if limits.get("extraction_complete") is not True or limits.get("joints_extracted") != joint_count:
        issues.append("movable-joint extraction is incomplete")
    declared = geometry_record.get("declared_collision_element_count")
    loadable = geometry_record.get("loadable_collision_element_count")
    if geometry_record.get("tree_valid") is not True:
        issues.append("geometry tree/q0 extraction is incomplete")
    if not isinstance(declared, int) or isinstance(declared, bool) or declared <= 0:
        issues.append("no positive declared collision-element denominator")
    elif loadable != declared:
        issues.append(f"loadable collision elements do not close ({loadable}/{declared})")
    return "; ".join(issues) if issues else None


def evaluate_item(*, item: Mapping[str, Any], source: verifier.SourceCohort, dataset_root: Path) -> dict[str, Any]:
    """Produce static, geometry, Genesis joint and atomic state records for one item."""

    runtime = genesis_runtime_binding()
    package = _package(dataset_root, str(item["rel_path"]))
    if package_binding(package) != item.get("package_binding"):
        raise GenesisAdapterError("package binding drift after freeze")
    urdf = _regular_file(package / str(item.get("primary_urdf_relpath", "generated.urdf")))
    if sha256_file(urdf) != item.get("urdf_sha256"):
        raise GenesisAdapterError("URDF hash drift after freeze")
    key = str(item["asset_key"])
    joints = source_joints(source, key)
    source_record = source.records_by_rank[int(item["selection_rank"])]
    static_record = static.audit_lam_package(
        package, asset_id=key, expected_movable_joints=len(joints), placeholder_registry=[]
    )
    geometry_record = geometry.evaluate_table4b_geometry(
        urdf, key, PROTOCOL_ID, GEOMETRY_SAMPLES_PER_DIRECTION, GEOMETRY_WELD_REL_TOL
    )
    timing_record = geometry.measure_collision_load_time_in_asset_child(
        urdf, weld_rel_tol=GEOMETRY_WELD_REL_TOL
    )
    triangle_measurement = geometry.collision_triangle_validation_measurement(geometry_record)
    redundancy_measurement = geometry.collision_redundancy_measurement(geometry_record)
    table2 = static_record.get("table2_supplementary", {})
    portable_map: dict[str, bool] = {}
    dynamics_map: dict[str, bool] = {}
    if isinstance(table2, Mapping):
        limits = table2.get("joint_limit_portability", {})
        dynamics = table2.get("joint_dynamics_coverage", {})
        if isinstance(limits, Mapping):
            portable_map = {
                str(row["joint_name"]): bool(row.get("limit_portability_pass"))
                for row in limits.get("joint_records", [])
                if isinstance(row, Mapping) and isinstance(row.get("joint_name"), str)
            }
        if isinstance(dynamics, Mapping):
            dynamics_map = {
                str(row["joint_name"]): bool(row.get("covered"))
                for row in dynamics.get("joint_records", [])
                if isinstance(row, Mapping) and isinstance(row.get("joint_name"), str)
            }

    state_map: dict[str, list[dict[str, Any]]] = {}
    strict_rows: list[dict[str, Any]] = []
    state_issues: list[str] = []
    gate_reason = _state_gate_reason(static_record, geometry_record, source_record, len(joints))
    adapter: GenesisTable4aAdapter | None = None
    if gate_reason is None:
        try:
            adapter = GenesisTable4aAdapter(urdf, runtime)
            adapter.build()
            try:
                strict_rows.append(
                    adapter.strict_state(
                        item=item, phase="rest", sample_index=0,
                        intended=[0.0] * len(adapter.dof_order),
                    )
                )
            except Exception as exc:  # noqa: BLE001
                reason = f"{type(exc).__name__}: {exc}"
                state_issues.append(f"rest: {reason}")
                strict_rows.append(
                    unexecuted_strict_state(
                        item, "rest", 0, runtime, reason, dimension=len(joints),
                        joint_dof_order=adapter.dof_order,
                    )
                )
            if joints:
                try:
                    sobol_vectors = sobol_intended_vectors(adapter, joints)
                except Exception as exc:  # noqa: BLE001
                    reason = f"{type(exc).__name__}: {exc}"
                    state_issues.append(f"sobol_generation: {reason}")
                    sobol_vectors = []
                    strict_rows.extend(
                        unexecuted_strict_state(
                            item, "multi_joint_sobol", index, runtime, reason,
                            dimension=len(joints), joint_dof_order=adapter.dof_order,
                        )
                        for index in range(SOBOL_STATE_COUNT_PER_ASSET)
                    )
                for index, intended in enumerate(sobol_vectors):
                    try:
                        strict_rows.append(
                            adapter.strict_state(
                                item=item, phase="multi_joint_sobol",
                                sample_index=index, intended=intended,
                            )
                        )
                    except Exception as exc:  # noqa: BLE001
                        reason = f"{type(exc).__name__}: {exc}"
                        state_issues.append(f"sobol_{index}: {reason}")
                        strict_rows.append(
                            unexecuted_strict_state(
                                item, "multi_joint_sobol", index, runtime, reason,
                                dimension=len(joints), joint_dof_order=adapter.dof_order,
                            )
                        )
            for row in joints:
                name = str(row["joint_name"])
                values = source_values(row)
                if values is None:
                    raise GenesisAdapterError(f"invalid frozen sample_values for {name}")
                state_map[name] = []
                for index, value in enumerate(values):
                    try:
                        state_map[name].append(
                            adapter.state(item=item, joint_name=name, sample_index=index, value=value)
                        )
                    except Exception as exc:  # retain every intended state
                        reason = f"{type(exc).__name__}: {exc}"
                        state_issues.append(f"{name}_{index}: {reason}")
                        state_map[name].append(
                            unexecuted_state(item, name, index, value, runtime, reason)
                        )
        except Exception as exc:  # all states remain fail-closed
            gate_reason = f"{type(exc).__name__}: {exc}"
        finally:
            if adapter is not None:
                adapter.close()
    if gate_reason is not None:
        state_issues.append(gate_reason)
        for row in joints:
            name = str(row["joint_name"])
            values = source_values(row)
            state_map[name] = [
                unexecuted_state(item, name, index, values[index] if values else None, runtime, gate_reason)
                for index in range(STATE_SAMPLES_PER_JOINT)
            ]
        strict_rows = [
            unexecuted_strict_state(
                item, "rest", 0, runtime, gate_reason, dimension=len(joints)
            )
        ]
        if joints:
            strict_rows.extend(
                unexecuted_strict_state(
                    item, "multi_joint_sobol", index, runtime, gate_reason,
                    dimension=len(joints),
                )
                for index in range(SOBOL_STATE_COUNT_PER_ASSET)
            )

    joint_rows = [
        joint_record(
            item, row, state_map[str(row["joint_name"])],
            portable_map.get(str(row["joint_name"]), False),
            dynamics_map.get(str(row["joint_name"]), False),
        )
        for row in joints
    ]
    states = [state for row in joints for state in state_map[str(row["joint_name"])]]
    single_safe = bool(joints) and all(
        state.get("executed") is True and state.get("illegal_collision") is False
        for state in states
    )
    strict_phase_safe = all(
        row.get("executed") is True and row.get("illegal_collision") is False
        for row in strict_rows
    )
    strict_pass = bool(single_safe and strict_phase_safe)
    measurement_complete = bool(
        all(state.get("executed") is True for state in states)
        and all(row.get("executed") is True for row in strict_rows)
    )

    visual = table2.get("visual_bearing_collision_coverage", {}) if isinstance(table2, Mapping) else {}
    placeholder = table2.get("placeholder_mass_incidence", {}) if isinstance(table2, Mapping) else {}
    s1 = static_record.get("s1_evidence", {})
    allowance = s1.get("allowance", {}) if isinstance(s1, Mapping) else {}
    resource = static_record.get("resource_closure", {})
    visual_count = int(visual.get("visual_bearing_links_declared", 0)) if isinstance(visual, Mapping) else 0
    covered_count = int(visual.get("covered_visual_bearing_links", 0)) if isinstance(visual, Mapping) else 0
    declared_collision = geometry_record.get("declared_collision_element_count")
    loadable_collision = geometry_record.get("loadable_collision_element_count")
    if (
        geometry_record.get("tree_valid") is True
        and isinstance(resource, Mapping) and resource.get("complete") is True
        and isinstance(loadable_collision, int) and not isinstance(loadable_collision, bool)
        and loadable_collision > 0 and loadable_collision == declared_collision
        and visual_count > 0
    ):
        shapes_measurement = {
            "status": "COMPLETE", "value": loadable_collision / visual_count,
            "reason": None,
        }
    else:
        shapes_measurement = _measurement_ne(
            "tree/resource/collision extraction or visual-bearing denominator is incomplete"
        )
    asset = {
        "protocol_id": PROTOCOL_ID, "asset_key": key, "selection_rank": item["selection_rank"],
        "input_identity_sha256": item["input_identity_sha256"], "source_record_sha256": item["source_record_sha256"],
        "evaluation_success": measurement_complete, "visual_bearing_link_count": visual_count,
        "collision_covered_visual_bearing_link_count": covered_count,
        "visual_bearing_collision_coverage_asset_pass": bool(visual.get("asset_pass", False)) if isinstance(visual, Mapping) else False,
        "visual_bearing_collision_coverage_complete": visual_count > 0 and covered_count == visual_count,
        "mass_evaluable_link_count": 0, "placeholder_mass_link_count": 0,
        "complete_inertial_link_count_unclassified": int(placeholder.get("complete_inertial_links", 0)) if isinstance(placeholder, Mapping) else 0,
        "release_receipt_bound": False, "release_receipt_replay_pass": False,
        "deterministic_rebuild_eligible": False, "deterministic_rebuild_match": False,
        "eligible_non_adjacent_pair_count": int(allowance.get("eligible_nonadjacent_pair_count") or 0) if isinstance(allowance, Mapping) else 0,
        "registered_method_allowance_pair_count": 0,
        "strict_collision_pass_no_method_allowance": strict_pass,
        "strict_collision_pass_registered_allowance": strict_pass,
        "static_record": static_record, "geometry_record": geometry_record,
        "engine_failure": "; ".join(state_issues) if state_issues else None,
        "terminal": True, "status": "completed" if measurement_complete else "partial",
        "collision_load_time_seconds": timing_record,
        "shapes_per_visual_bearing_link": shapes_measurement,
        "collision_mesh_triangles_per_asset": triangle_measurement,
        "intra_link_redundancy": redundancy_measurement,
    }
    directions = (geometry_record.get("visual_to_collision", {}), geometry_record.get("collision_to_visual", {}))
    for name, record in zip(("visual_to_collision_p95_normalized", "collision_to_visual_p95_normalized"), directions):
        value = record.get("normalized_p95") if isinstance(record, Mapping) and record.get("status") == "COMPLETE" else None
        asset[name] = {"status": "COMPLETE", "value": float(value)} if _finite_float(value) is not None else {"status": "N/E", "value": None, "reason": str(record.get("reason", "exact surface measurement unavailable")) if isinstance(record, Mapping) else "geometry unavailable"}
    asset["intra_link_shape_volume_m3"] = float(redundancy_measurement.get("shape_volume_m3") or 0.0)
    asset["intra_link_redundant_volume_m3"] = float(redundancy_measurement.get("redundant_volume_m3") or 0.0)
    asset["loadable_collision_element_count"] = int(loadable_collision or 0)
    asset["analytic_collision_element_count"] = int(geometry_record.get("analytic_collision_element_count", 0))
    asset["collision_shape_count"] = asset["loadable_collision_element_count"]
    triangle_value = triangle_measurement.get("value") if triangle_measurement.get("status") == "COMPLETE" else 0
    asset["collision_mesh_triangle_count"] = int(triangle_value or 0)
    return {
        "asset_record": asset,
        "joint_records": joint_rows,
        "state_records": states,
        "strict_state_records": strict_rows,
    }


def _manifest_hash(manifest: Mapping[str, Any]) -> str:
    payload = dict(manifest)
    payload.pop("manifest_content_sha256", None)
    return canonical_sha256(payload)


def write_child_receipt(output_root: Path, rank: int, result: Mapping[str, Any]) -> Path:
    path = Path(output_root) / "children" / f"rank_{rank:04d}.json"
    if path.exists():
        raise GenesisAdapterError(f"refusing to overwrite child receipt: {path}")
    atomic_json(path, result)
    return path


def write_child_attempt(output_root: Path, rank: int, result: Mapping[str, Any]) -> Path:
    """Write a diagnostic child result that only the parent may promote."""

    path = Path(output_root) / "child_attempts" / f"rank_{rank:04d}.json"
    if path.exists():
        raise GenesisAdapterError(f"refusing to overwrite child attempt: {path}")
    atomic_json(path, result)
    return path


def _measurement_ne(reason: str) -> dict[str, Any]:
    return {"status": "N/E", "value": None, "reason": reason}


def fail_closed_result(
    *, item: Mapping[str, Any], source: verifier.SourceCohort,
    runtime: Mapping[str, Any], reason: str,
) -> dict[str, Any]:
    """Materialize every intended row when a rank is deliberately unexecuted.

    Pilot runs use this for the 800-rank denominator outside the selected
    pilot ranks.  The rows are terminal and explicit so aggregation cannot
    mistake omitted work for a pass.
    """

    key = str(item["asset_key"])
    joints = source_joints(source, key)
    states_by_name: dict[str, list[dict[str, Any]]] = {}
    for source_joint in joints:
        name = str(source_joint["joint_name"])
        values = source_values(source_joint)
        states_by_name[name] = [
            unexecuted_state(
                item, name, index,
                values[index] if values is not None else None,
                runtime, reason,
            )
            for index in range(STATE_SAMPLES_PER_JOINT)
        ]
    joint_rows = [
        joint_record(item, source_joint, states_by_name[str(source_joint["joint_name"])], False, False)
        for source_joint in joints
    ]
    asset = {
        "protocol_id": PROTOCOL_ID,
        "asset_key": key,
        "selection_rank": item["selection_rank"],
        "input_identity_sha256": item["input_identity_sha256"],
        "source_record_sha256": item["source_record_sha256"],
        "evaluation_success": False,
        "visual_bearing_link_count": 0,
        "collision_covered_visual_bearing_link_count": 0,
        "visual_bearing_collision_coverage_asset_pass": False,
        "visual_bearing_collision_coverage_complete": False,
        "mass_evaluable_link_count": 0,
        "placeholder_mass_link_count": 0,
        "complete_inertial_link_count_unclassified": 0,
        "release_receipt_bound": False,
        "release_receipt_replay_pass": False,
        "deterministic_rebuild_eligible": False,
        "deterministic_rebuild_match": False,
        "eligible_non_adjacent_pair_count": 0,
        "registered_method_allowance_pair_count": 0,
        "strict_collision_pass_no_method_allowance": False,
        "strict_collision_pass_registered_allowance": False,
        "visual_to_collision_p95_normalized": _measurement_ne(reason),
        "collision_to_visual_p95_normalized": _measurement_ne(reason),
        "collision_load_time_seconds": _measurement_ne(reason),
        "intra_link_shape_volume_m3": 0.0,
        "intra_link_redundant_volume_m3": 0.0,
        "loadable_collision_element_count": 0,
        "analytic_collision_element_count": 0,
        "collision_shape_count": 0,
        "collision_mesh_triangle_count": 0,
        "shapes_per_visual_bearing_link": _measurement_ne(reason),
        "collision_mesh_triangles_per_asset": _measurement_ne(reason),
        "intra_link_redundancy": {
            "status": "N/E", "value": None, "redundant_volume_m3": None,
            "shape_volume_m3": None, "measured_links": 0, "intended_links": 0,
            "reason": reason,
        },
        "static_record": {"status": "N/E", "reason": reason},
        "geometry_record": {"status": "N/E", "reason": reason},
        "engine_failure": reason,
        "terminal": True,
        "status": "fail_closed",
    }
    strict_rows = [
        unexecuted_strict_state(item, "rest", 0, runtime, reason, dimension=len(joints))
    ]
    if joints:
        strict_rows.extend(
            unexecuted_strict_state(
                item, "multi_joint_sobol", index, runtime, reason, dimension=len(joints)
            )
            for index in range(SOBOL_STATE_COUNT_PER_ASSET)
        )
    return {
        "asset_record": asset,
        "joint_records": joint_rows,
        "state_records": [
            state
            for source_joint in joints
            for state in states_by_name[str(source_joint["joint_name"])]
        ],
        "strict_state_records": strict_rows,
    }


def _result_matches_item(
    result: Mapping[str, Any], item: Mapping[str, Any], source: verifier.SourceCohort,
) -> bool:
    asset = result.get("asset_record")
    if not isinstance(asset, Mapping):
        return False
    if (
        asset.get("selection_rank") != item.get("selection_rank")
        or asset.get("asset_key") != item.get("asset_key")
        or asset.get("input_identity_sha256") != item.get("input_identity_sha256")
    ):
        return False
    joints = source_joints(source, str(item["asset_key"]))
    joint_rows = result.get("joint_records")
    state_rows = result.get("state_records")
    strict_rows = result.get("strict_state_records")
    if not isinstance(joint_rows, list) or len(joint_rows) != len(joints):
        return False
    if not isinstance(state_rows, list) or len(state_rows) != len(joints) * STATE_SAMPLES_PER_JOINT:
        return False
    expected_strict_count = 1 + (SOBOL_STATE_COUNT_PER_ASSET if joints else 0)
    if not isinstance(strict_rows, list) or len(strict_rows) != expected_strict_count:
        return False
    expected_names = [str(row["joint_name"]) for row in joints]
    observed_names = [str(row.get("joint_name")) for row in joint_rows if isinstance(row, Mapping)]
    if observed_names != expected_names:
        return False
    expected_strict_keys = [f"{item['asset_key']}::strict::rest::0"]
    if joints:
        expected_strict_keys.extend(
            f"{item['asset_key']}::strict::multi_joint_sobol::{index}"
            for index in range(SOBOL_STATE_COUNT_PER_ASSET)
        )
    return [row.get("strict_state_key") for row in strict_rows] == expected_strict_keys


def _read_child_receipt(
    output_root: Path, rank: int, item: Mapping[str, Any], source: verifier.SourceCohort,
) -> dict[str, Any] | None:
    path = Path(output_root) / "children" / f"rank_{rank:04d}.json"
    if not path.exists():
        return None
    data = json.loads(_regular_file(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not _result_matches_item(data, item, source):
        raise GenesisAdapterError(f"existing child receipt does not bind to rank {rank}")
    return data


def _read_child_attempt(
    output_root: Path, rank: int, item: Mapping[str, Any], source: verifier.SourceCohort,
) -> dict[str, Any] | None:
    path = Path(output_root) / "child_attempts" / f"rank_{rank:04d}.json"
    if not path.exists():
        return None
    data = json.loads(_regular_file(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not _result_matches_item(data, item, source):
        raise GenesisAdapterError(f"child attempt does not bind to rank {rank}")
    return data


def _parse_rank_spec(spec: str, *, maximum: int = SAMPLE_SIZE) -> list[int]:
    ranks: set[int] = set()
    for token in (part.strip() for part in spec.split(",")):
        if not token:
            raise GenesisAdapterError("--pilot-ranks contains an empty token")
        if "-" in token:
            pieces = token.split("-")
            if len(pieces) != 2 or not all(piece.strip().isdigit() for piece in pieces):
                raise GenesisAdapterError(f"invalid pilot rank range: {token!r}")
            start, end = (int(piece) for piece in pieces)
            if start > end:
                raise GenesisAdapterError(f"descending pilot rank range: {token!r}")
            ranks.update(range(start, end + 1))
        elif token.isdigit():
            ranks.add(int(token))
        else:
            raise GenesisAdapterError(f"invalid pilot rank: {token!r}")
    if not ranks or min(ranks) < 1 or max(ranks) > maximum:
        raise GenesisAdapterError(f"pilot ranks must lie in [1,{maximum}]")
    if len(ranks) > 32:
        raise GenesisAdapterError("pilot mode is capped at 32 ranks; use --run-all for explicit full execution")
    return sorted(ranks)


def _load_scope_manifest(
    *, output_root: Path, dataset_root: Path, source_records: Path,
    source_manifest: Path, pilot: bool,
) -> dict[str, Any]:
    manifest_path = output_root / "frozen_manifest.json"
    if not manifest_path.exists():
        if not pilot:
            raise GenesisAdapterError("formal execution requires a prepared frozen manifest; run --prepare first")
        if output_root.exists():
            raise GenesisAdapterError("pilot output root exists without a manifest; refusing to populate it")
        manifest = build_frozen_manifest(
            dataset_root=dataset_root,
            source_records=source_records,
            source_manifest=source_manifest,
            qualification_smoke=True,
            expected_cache_path=output_root / "genesis-cache",
        )
        write_frozen_manifest(output_root, manifest)
        return manifest
    manifest = json.loads(_regular_file(manifest_path).read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or _manifest_hash(manifest) != manifest.get("manifest_content_sha256"):
        raise GenesisAdapterError("frozen manifest self-hash mismatch")
    if bool(manifest.get("qualification_smoke")) != pilot:
        raise GenesisAdapterError("pilot/formal scope does not match frozen manifest qualification_smoke")
    source = load_source_cohort(source_records, source_manifest)
    config = verifier.VerifierConfig(source_records=source_records, source_manifest=source_manifest)
    # Reuse the independent manifest validator on a hash-equivalent formal
    # view; its only intentional rejection for pilots is qualification_smoke.
    validation_view = dict(manifest)
    validation_view["qualification_smoke"] = False
    validation_view["manifest_content_sha256"] = _manifest_hash(validation_view)
    verifier._validate_manifest(output_root, validation_view, source, config)  # type: ignore[attr-defined]
    return manifest


def _validate_execution_binding(manifest: Mapping[str, Any], output_root: Path) -> dict[str, Any]:
    """Recompute the frozen runtime/code identity before every execution."""

    expected_cache = (output_root.resolve(strict=False) / "genesis-cache").resolve(strict=False)
    observed_runtime = genesis_runtime_binding(expected_cache_path=expected_cache)
    frozen_runtime = manifest.get("runtime_binding")
    if not isinstance(frozen_runtime, Mapping) or dict(frozen_runtime) != observed_runtime:
        raise GenesisAdapterError("current Genesis runtime does not equal frozen runtime_binding")
    frozen_code = manifest.get("code_identity")
    if not isinstance(frozen_code, Mapping) or dict(frozen_code) != current_code_identity():
        raise GenesisAdapterError("runner/static/geometry/verifier code identity drifted from freeze")
    return observed_runtime


def _write_once_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    if path.exists():
        raise GenesisAdapterError(f"refusing to overwrite final artifact: {path}")
    atomic_jsonl(path, rows)


def _write_once_json(path: Path, value: Mapping[str, Any]) -> None:
    if path.exists():
        raise GenesisAdapterError(f"refusing to overwrite final artifact: {path}")
    atomic_json(path, value)


def _write_once_text(path: Path, text: str) -> None:
    if path.exists():
        raise GenesisAdapterError(f"refusing to overwrite final artifact: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _child_environment(output_root: Path, runtime: Mapping[str, Any] | None = None) -> dict[str, str]:
    env = os.environ.copy()
    env["GS_CACHE_FILE_PATH"] = str(
        (output_root.resolve(strict=False) / "genesis-cache").resolve(strict=False)
    )
    frozen_threads = runtime.get("thread_environment") if isinstance(runtime, Mapping) else None
    if not isinstance(frozen_threads, Mapping):
        frozen_threads = THREAD_ENV_VALUES
    for key in THREAD_ENV_KEYS:
        value = frozen_threads.get(key)
        if not isinstance(value, str):
            raise GenesisAdapterError(f"frozen child thread environment lacks {key}")
        env[key] = value
    frozen_affinity = runtime.get("cpu_affinity") if isinstance(runtime, Mapping) else None
    if not isinstance(frozen_affinity, list) or not frozen_affinity:
        raise GenesisAdapterError("frozen child runtime lacks cpu_affinity")
    env[CPU_AFFINITY_ENV] = ",".join(str(int(cpu)) for cpu in frozen_affinity)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    return env


def _capture_text(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _run_child_rank_subprocess(
    *, output_root: Path, dataset_root: Path, source_records: Path,
    source_manifest: Path, rank: int, item: Mapping[str, Any],
    source: verifier.SourceCohort, runtime: Mapping[str, Any],
) -> dict[str, Any]:
    """Run one rank in a fresh Genesis process and preserve a terminal receipt."""

    child_dir = output_root / "children"
    attempt_dir = output_root / "child_attempts"
    log_dir = output_root / "child_logs"
    child_dir.mkdir(parents=True, exist_ok=True)
    attempt_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    receipt_path = child_dir / f"rank_{rank:04d}.json"
    attempt_path = attempt_dir / f"rank_{rank:04d}.json"
    log_path = log_dir / f"rank_{rank:04d}.log"
    if receipt_path.exists():
        cached = _read_child_receipt(output_root, rank, item, source)
        if cached is None:
            raise GenesisAdapterError(f"child receipt unexpectedly disappeared for rank {rank}")
        return cached
    if log_path.exists() or attempt_path.exists():
        result = fail_closed_result(
            item=item, source=source, runtime=runtime,
            reason="prior child log/attempt exists without a formal receipt; retry is forbidden",
        )
        write_child_receipt(output_root, rank, result)
        return result
    command = [
        str(DEFAULT_GENESIS_PYTHON.resolve(strict=True)), str(SCRIPT),
        "--output-root", str(output_root.resolve(strict=True)),
        "--dataset-root", str(_regular_dir(dataset_root)),
        "--source-asset-records", str(_regular_file(source_records)),
        "--source-manifest", str(_regular_file(source_manifest)),
        "--child-rank", str(rank),
    ]
    completed: subprocess.CompletedProcess[str] | None = None
    timeout_error: subprocess.TimeoutExpired | None = None
    launch_error: Exception | None = None
    try:
        completed = subprocess.run(
            command,
            cwd=str(REPO),
            env=_child_environment(output_root, runtime),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=CHILD_TIMEOUT_SECONDS,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        timeout_error = exc
    except Exception as exc:  # noqa: BLE001 - parent must materialize fail-closed output
        launch_error = exc
    return_code = None if completed is None else completed.returncode
    output = _capture_text(timeout_error.stdout if timeout_error is not None else (completed.stdout if completed else None))
    log_lines = [
        f"command={canonical_json(command)}",
        f"returncode={return_code!r}",
        f"timeout={timeout_error is not None}",
    ]
    if launch_error is not None:
        log_lines.append(f"launch_error={type(launch_error).__name__}: {launch_error}")
    if timeout_error is not None:
        log_lines.append(f"timeout_error={type(timeout_error).__name__}: child exceeded {CHILD_TIMEOUT_SECONDS}s")
    if output:
        log_lines.extend(("child_output:", output.rstrip("\n")))
    log_lines.append(f"attempt_present={attempt_path.exists()}")
    _write_once_text(log_path, "\n".join(log_lines) + "\n")
    if receipt_path.exists():
        raise GenesisAdapterError(f"rank {rank} child improperly published a formal receipt")
    successful_exit = bool(
        completed is not None and completed.returncode == 0
        and timeout_error is None and launch_error is None
    )
    attempt_error: Exception | None = None
    if successful_exit:
        try:
            attempted = _read_child_attempt(output_root, rank, item, source)
        except Exception as exc:  # noqa: BLE001 - invalid attempt remains diagnostic-only
            attempt_error = exc
        else:
            if attempted is not None:
                write_child_receipt(output_root, rank, attempted)
                return attempted
    if timeout_error is not None:
        reason = f"child timeout after {CHILD_TIMEOUT_SECONDS}s"
    elif launch_error is not None or not successful_exit:
        reason = f"child launch/exit failure (returncode={return_code!r})"
    else:
        reason = "child exited 0 without a valid append-only attempt"
    if launch_error is not None:
        reason += f": {type(launch_error).__name__}: {launch_error}"
    if attempt_error is not None:
        reason += f": {type(attempt_error).__name__}: {attempt_error}"
    result = fail_closed_result(item=item, source=source, runtime=runtime, reason=reason)
    write_child_receipt(output_root, rank, result)
    return result


def run_scope(
    *, output_root: Path, dataset_root: Path, source_records: Path,
    source_manifest: Path, selected_ranks: Sequence[int], pilot: bool,
) -> dict[str, Any]:
    """Run selected ranks in isolated Genesis children and publish append-only aggregates."""

    manifest = _load_scope_manifest(
        output_root=output_root, dataset_root=dataset_root,
        source_records=source_records, source_manifest=source_manifest, pilot=pilot,
    )
    source = load_source_cohort(source_records, source_manifest)
    items = {
        int(item["selection_rank"]): item
        for item in manifest.get("items", [])
        if isinstance(item, Mapping)
    }
    if set(items) != set(range(1, SAMPLE_SIZE + 1)):
        raise GenesisAdapterError("frozen manifest does not cover ranks 1..800")
    runtime = _validate_execution_binding(manifest, output_root)
    selected = set(int(rank) for rank in selected_ranks)
    if selected - set(items):
        raise GenesisAdapterError("selected rank is outside the frozen cohort")
    (output_root / "children").mkdir(parents=True, exist_ok=True)

    results: dict[int, dict[str, Any]] = {}
    for rank in range(1, SAMPLE_SIZE + 1):
        item = items[rank]
        cached = _read_child_receipt(output_root, rank, item, source)
        if cached is not None:
            result = cached
        elif rank in selected:
            result = _run_child_rank_subprocess(
                output_root=output_root, dataset_root=dataset_root,
                source_records=source_records, source_manifest=source_manifest,
                rank=rank, item=item, source=source, runtime=runtime,
            )
        else:
            result = fail_closed_result(
                item=item, source=source, runtime=runtime,
                reason="rank not selected by pilot scope",
            )
            write_child_receipt(output_root, rank, result)
        if not _result_matches_item(result, item, source):
            raise GenesisAdapterError(f"rank {rank} result does not close against frozen item")
        results[rank] = result

    asset_rows = [results[rank]["asset_record"] for rank in range(1, SAMPLE_SIZE + 1)]
    joint_rows = [
        joint for rank in range(1, SAMPLE_SIZE + 1) for joint in results[rank]["joint_records"]
    ]
    joint_rows.sort(key=lambda row: source.joint_order[(str(row["asset_key"]), str(row["joint_name"]))])
    state_rows = [
        state for rank in range(1, SAMPLE_SIZE + 1) for state in results[rank]["state_records"]
    ]
    state_rows.sort(
        key=lambda row: (
            source.joint_order[(str(row["asset_key"]), str(row["joint_name"]))],
            int(row["sample_index"]),
        )
    )
    strict_rows = [
        state
        for rank in range(1, SAMPLE_SIZE + 1)
        for state in results[rank]["strict_state_records"]
    ]
    expected_strict_count = SAMPLE_SIZE + sum(
        SOBOL_STATE_COUNT_PER_ASSET for rank in range(1, SAMPLE_SIZE + 1)
        if source_joints(source, str(items[rank]["asset_key"]))
    )
    if len(strict_rows) != expected_strict_count:
        raise GenesisAdapterError(
            f"strict state aggregate has {len(strict_rows)} rows, expected {expected_strict_count}"
        )

    config = verifier.VerifierConfig(source_records=source_records, source_manifest=source_manifest)
    assets_by_rank = {int(row["selection_rank"]): dict(row) for row in asset_rows}
    joints_by_key = {
        (str(row["asset_key"]), str(row["joint_name"])): dict(row) for row in joint_rows
    }
    states_by_joint: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for row in state_rows:
        key = (str(row["asset_key"]), str(row["joint_name"]))
        states_by_joint.setdefault(key, []).append(dict(row))
    _, derived_strict_pass = verifier._validate_strict_state_records(
        [dict(row) for row in strict_rows], source, items, assets_by_rank,
        states_by_joint, str(manifest["protocol_id"]), runtime, config,
    )
    aggregates = verifier.aggregate_records(
        assets_by_rank, joints_by_key, states_by_joint, derived_strict_pass, source, config
    )
    evaluated_assets = sum(bool(row.get("evaluation_success")) for row in asset_rows)
    failed_assets = SAMPLE_SIZE - evaluated_assets
    all_states_executed = all(row.get("executed") is True for row in state_rows)
    all_strict_executed = all(row.get("executed") is True for row in strict_rows)
    complete = bool(not pilot and failed_assets == 0 and all_states_executed and all_strict_executed)
    status = "COMPLETE" if complete else "PARTIAL"
    mode = "pilot qualification smoke" if pilot else "formal Genesis run"
    aggregate_hash = canonical_sha256(aggregates)
    summary = {
        "schema_version": 1,
        "protocol_id": manifest["protocol_id"],
        "status": status,
        "cohort": {
            "selected": SAMPLE_SIZE, "assets": SAMPLE_SIZE, "joints": JOINT_COUNT,
            "pilot_selected_ranks": sorted(selected),
            "evaluated_asset_count": evaluated_assets,
            "fail_closed_asset_count": failed_assets,
            "strict_state_record_count": len(strict_rows),
            "strict_state_expected": expected_strict_count,
        },
        "input_binding": {
            "table3_asset_records_sha256": source.records_sha256,
            "table3_manifest_sha256": source.manifest_sha256,
            "table3_manifest_content_sha256": source.manifest_content_sha256,
            "ordered_selected_asset_keys_sha256": source.ordered_keys_sha256,
        },
        "scope": {
            "mode": mode, "engine_protocol_id": ENGINE_PROTOCOL_ID,
            "selected_rank_count": len(selected), "state_record_policy": STATE_RECORD_POLICY,
            "strict_state_record_policy": "all_intended", "formal_claim": complete,
        },
        "runtime_binding": dict(runtime),
        "code_identity": current_code_identity(),
        "verification_aggregates_sha256": aggregate_hash,
        "verification_aggregates": aggregates,
    }
    report_lines = [
        "# LAM Genesis supplementary evaluation", "",
        f"Protocol: `{manifest['protocol_id']}` (`{ENGINE_PROTOCOL_ID}`).", "",
        "Frozen cohort: N=800 assets, J=2395 movable joints, K=21 intended states per joint.",
        f"Scope: {mode}; selected ranks={len(selected)}; terminal fail-closed assets={failed_assets}.",
        f"Verification aggregates SHA256: `{aggregate_hash}`.",
        "Strict state records: 50336 intended raw rows (800 rest + 49536 Sobol).",
        "",
        "Table-4a uses Genesis contact penetration with a strict illegal threshold of 1e-6 m; signed clearance is N/E because this adapter does not invent a separated-pair signed distance.",
        "Table-2, Table-4b, and Supplementary S1 records remain explicit in the atomic asset rows; empty LAM receipt/allowance registries are preserved.",
    ]
    if pilot:
        report_lines.extend([
            "", "This is a qualification smoke output only. Non-selected ranks are represented by terminal N/E records and must not be cited as a formal result.",
        ])
    report = "\n".join(report_lines) + "\n"
    final_paths = [
        output_root / "asset_records.jsonl", output_root / "joint_records.jsonl",
        output_root / "state_records.jsonl", output_root / "strict_state_records.jsonl",
        output_root / "summary.json", output_root / "report.md",
    ]
    if any(path.exists() for path in final_paths):
        raise GenesisAdapterError("one or more final artifacts already exist; refusing overwrite")
    _write_once_jsonl(final_paths[0], asset_rows)
    _write_once_jsonl(final_paths[1], joint_rows)
    _write_once_jsonl(final_paths[2], state_rows)
    _write_once_jsonl(final_paths[3], strict_rows)
    _write_once_json(final_paths[4], summary)
    _write_once_text(final_paths[5], report)
    return summary


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--source-asset-records", type=Path, default=DEFAULT_SOURCE_RECORDS)
    parser.add_argument("--source-manifest", type=Path, default=DEFAULT_SOURCE_MANIFEST)
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--child-rank", type=int)
    parser.add_argument(
        "--pilot-ranks",
        help="qualification-only rank list/ranges (for example 1,3-5; maximum 32)",
    )
    parser.add_argument(
        "--run-all",
        action="store_true",
        help="explicitly execute all 800 frozen ranks and aggregate formal artifacts",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    modes = sum((
        bool(args.prepare),
        args.child_rank is not None,
        args.pilot_ranks is not None,
        bool(args.run_all),
    ))
    if modes != 1:
        raise SystemExit("choose exactly one of --prepare, --child-rank, --pilot-ranks, or --run-all")
    cache_path = bind_genesis_cache(args.output_root)
    if args.prepare:
        manifest = build_frozen_manifest(
            dataset_root=args.dataset_root,
            source_records=args.source_asset_records,
            source_manifest=args.source_manifest,
            expected_cache_path=cache_path,
        )
        write_frozen_manifest(args.output_root, manifest)
        print(args.output_root)
        return 0
    if args.pilot_ranks is not None:
        summary = run_scope(
            output_root=args.output_root,
            dataset_root=args.dataset_root,
            source_records=args.source_asset_records,
            source_manifest=args.source_manifest,
            selected_ranks=_parse_rank_spec(args.pilot_ranks),
            pilot=True,
        )
        print(canonical_json({"status": summary["status"], "output_root": str(args.output_root)}))
        return 0
    if args.run_all:
        summary = run_scope(
            output_root=args.output_root,
            dataset_root=args.dataset_root,
            source_records=args.source_asset_records,
            source_manifest=args.source_manifest,
            selected_ranks=range(1, SAMPLE_SIZE + 1),
            pilot=False,
        )
        print(canonical_json({"status": summary["status"], "output_root": str(args.output_root)}))
        return 0
    rank = int(args.child_rank)
    if not 1 <= rank <= SAMPLE_SIZE:
        raise SystemExit(f"child rank must be in [1,{SAMPLE_SIZE}]")
    raw_manifest = json.loads(_regular_file(args.output_root / "frozen_manifest.json").read_text(encoding="utf-8"))
    if not isinstance(raw_manifest, dict) or _manifest_hash(raw_manifest) != raw_manifest.get("manifest_content_sha256"):
        raise GenesisAdapterError("frozen manifest self-hash mismatch")
    manifest = _load_scope_manifest(
        output_root=args.output_root, dataset_root=args.dataset_root,
        source_records=args.source_asset_records, source_manifest=args.source_manifest,
        pilot=bool(raw_manifest.get("qualification_smoke")),
    )
    _validate_execution_binding(manifest, args.output_root)
    item = next(
        (row for row in manifest.get("items", [])
         if isinstance(row, Mapping) and int(row.get("selection_rank", -1)) == rank),
        None,
    )
    if not isinstance(item, Mapping):
        raise GenesisAdapterError(f"frozen manifest lacks selection rank {rank}")
    source = load_source_cohort(args.source_asset_records, args.source_manifest)
    result = evaluate_item(item=item, source=source, dataset_root=args.dataset_root)
    print(write_child_attempt(args.output_root, rank, result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
