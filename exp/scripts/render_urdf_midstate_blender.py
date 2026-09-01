#!/usr/bin/env python3
"""Render one URDF package with every movable joint at a midpoint pose.

This worker reuses the audited studio implementation but replaces the rest
pose link transform with URDF axis/type/limit-aware midpoint transforms. It
is intended for diagnostic stills, not for dynamics simulation.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path, PureWindowsPath
from typing import Any


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
BASE_RENDERER = REPO_ROOT / "exp/scripts/render_articraft10k_asset_blender.py"
PARTNET_RENDERER = REPO_ROOT / "exp/scripts/render_partnet_mobility_asset_blender.py"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _load(path: Path, name: str) -> Any:
    path = path.expanduser().resolve(strict=True)
    spec = importlib.util.spec_from_file_location(f"_{name}", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import renderer module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _float_triplet(value: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if not value:
        return default
    values = tuple(float(item) for item in value.split())
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise ValueError(f"invalid triplet: {value!r}")
    return values  # type: ignore[return-value]


def _safe_resolve(asset_dir: Path, urdf_path: Path, value: object, field: str) -> Path:
    text = str(value or "").strip()
    relative = Path(text)
    windows = PureWindowsPath(text)
    if (
        not text
        or "\\" in text
        or "://" in text
        or relative.is_absolute()
        or windows.is_absolute()
        or bool(windows.drive)
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise ValueError(f"{field} must be a contained relative path")
    candidate = (urdf_path.parent / relative).resolve(strict=True)
    try:
        candidate.relative_to(asset_dir.resolve(strict=True))
    except ValueError as exc:
        raise ValueError(f"{field} escapes asset root: {candidate}") from exc
    if not candidate.is_file():
        raise ValueError(f"{field} is not a file: {candidate}")
    return candidate


def _load_artiverse_package(partnet: Any, base: Any, asset_dir: Path, urdf_path: Path) -> Any:
    """Use the PartNet parser while resolving all paths beside Artiverse URDF."""

    original = base._contained_file

    def resolve(root: Path, value: object, *, field: str) -> Path:
        if field == "mobility.urdf":
            return urdf_path
        return _safe_resolve(asset_dir, urdf_path, value, field)

    base._contained_file = resolve
    try:
        return partnet.load_partnet_package(asset_dir, base)
    finally:
        base._contained_file = original


def _joint_specs(urdf_path: Path) -> dict[str, dict[str, Any]]:
    root = ET.parse(urdf_path).getroot()
    specs: dict[str, dict[str, Any]] = {}
    for element in root.findall("joint"):
        name = str(element.get("name") or "").strip()
        if not name:
            continue
        kind = str(element.get("type") or "fixed").strip().lower()
        axis = _float_triplet(
            None if element.find("axis") is None else element.find("axis").get("xyz"),
            (0.0, 0.0, 1.0),
        )
        norm = math.sqrt(sum(value * value for value in axis))
        if norm <= 1e-12:
            axis = (0.0, 0.0, 1.0)
        else:
            axis = tuple(value / norm for value in axis)  # type: ignore[assignment]
        lower = upper = None
        limit = element.find("limit")
        if limit is not None:
            try:
                lower = float(limit.get("lower")) if limit.get("lower") is not None else None
                upper = float(limit.get("upper")) if limit.get("upper") is not None else None
            except (TypeError, ValueError):
                lower = upper = None
        if kind == "continuous":
            value = math.pi / 2.0
            fraction = 0.5
        elif kind in {"revolute", "prismatic"} and lower is not None and upper is not None and upper > lower:
            value = (lower + upper) / 2.0
            fraction = 0.5
        else:
            value = 0.0
            fraction = None
        specs[name] = {
            "type": kind,
            "axis": axis,
            "lower": lower,
            "upper": upper,
            "value": value,
            "fraction": fraction,
        }
    return specs


def _motion_matrix(base: Any, spec: dict[str, Any]) -> Any:
    import mathutils

    kind = spec["type"]
    value = float(spec["value"])
    axis = mathutils.Vector(spec["axis"])
    if kind == "prismatic":
        matrix = mathutils.Matrix.Identity(4)
        matrix.translation = axis * value
        return matrix
    if kind in {"revolute", "continuous"}:
        return mathutils.Matrix.Rotation(value, 4, axis)
    return mathutils.Matrix.Identity(4)


def _midpoint_matrices(base: Any, package: Any, specs: dict[str, dict[str, Any]]) -> dict[str, Any]:
    identity = (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    matrices: dict[str, Any] = {package.root_link: identity}
    pending = list(package.joints)
    while pending:
        progress = False
        remaining = []
        for joint in pending:
            parent = matrices.get(joint.parent)
            if parent is None:
                remaining.append(joint)
                continue
            spec = specs.get(joint.name, {"type": "fixed", "axis": (0.0, 0.0, 1.0), "value": 0.0})
            origin = base._origin_matrix(joint.origin)
            motion = _motion_matrix(base, spec)
            matrices[joint.child] = base._matmul(base._matmul(parent, origin), tuple(tuple(float(motion[row][col]) for col in range(4)) for row in range(4)))
            progress = True
        if not progress:
            unresolved = ", ".join(joint.name for joint in remaining)
            raise RuntimeError(f"joint graph is cyclic or disconnected: {unresolved}")
        pending = remaining
    return matrices


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", choices=("articraft", "artiverse"), required=True)
    parser.add_argument("--asset-dir", type=Path, required=True)
    parser.add_argument("--urdf-path", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else sys.argv[1:]
    args = parser.parse_args(raw)

    base = _load(BASE_RENDERER, "midstate_articraft_base")
    partnet = _load(PARTNET_RENDERER, "midstate_partnet_renderer")
    asset_dir = args.asset_dir.expanduser().resolve(strict=True)
    urdf_path = args.urdf_path.expanduser().resolve(strict=True)
    urdf_path.relative_to(asset_dir)
    specs = _joint_specs(urdf_path)
    if args.dataset == "articraft":
        package = base.load_asset_package(asset_dir)
    else:
        package = _load_artiverse_package(partnet, base, asset_dir, urdf_path)
    original_rest = base.rest_link_matrices
    base.rest_link_matrices = lambda loaded: _midpoint_matrices(base, loaded, specs)
    try:
        result = partnet.render_asset(
            package,
            args.output,
            resolution=args.resolution,
            samples=args.samples,
            shared_renderer=base.DEFAULT_SHARED_RENDERER,
            shared_renderer_sha256=None,
            base=base,
            base_receipt={"path": str(BASE_RENDERER)},
        )
    finally:
        base.rest_link_matrices = original_rest
    output = args.output.expanduser().resolve()
    if not output.is_file() or output.read_bytes()[: len(PNG_SIGNATURE)] != PNG_SIGNATURE:
        raise RuntimeError(f"invalid PNG output: {output}")
    result["dataset"] = args.dataset
    result["urdf_path"] = str(urdf_path)
    result["joint_midpoints"] = [
        {"name": name, **spec} for name, spec in sorted(specs.items())
    ]
    print(json.dumps(result, sort_keys=True, ensure_ascii=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
