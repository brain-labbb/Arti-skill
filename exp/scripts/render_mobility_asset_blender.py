#!/usr/bin/env python3
"""Render one Infinigen-Sim or PhysX-Mobility URDF package.

The parser and studio implementation are intentionally shared with the
audited PartNet-Mobility worker. ``--asset-dir`` is the package boundary and
``--urdf-path`` may point anywhere inside it; mesh filenames are resolved
relative to the URDF while remaining confined to that boundary.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import sys
from pathlib import Path, PureWindowsPath
from typing import Any, Sequence


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
PARTNET_WORKER = REPO_ROOT / "exp/scripts/render_partnet_mobility_asset_blender.py"
DEFAULT_BASE_RENDERER = REPO_ROOT / "exp/scripts/render_articraft10k_asset_blender.py"
DEFAULT_SHARED_RENDERER = REPO_ROOT / "arti-template/scripts/render_exported_asset_blender.py"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load(path: Path, name: str, expected_sha256: str | None = None) -> Any:
    path = path.expanduser().resolve(strict=True)
    actual_sha256 = _sha256(path)
    if expected_sha256 is not None and actual_sha256 != expected_sha256:
        raise RuntimeError(
            f"support renderer SHA-256 mismatch: expected {expected_sha256}, found {actual_sha256}"
        )
    spec = importlib.util.spec_from_file_location(f"_{name}_{actual_sha256}", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import renderer support: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, required=True)
    parser.add_argument("--urdf-path", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--support-renderer", type=Path, default=PARTNET_WORKER)
    parser.add_argument("--support-renderer-sha256", required=True)
    parser.add_argument("--base-renderer", type=Path, default=DEFAULT_BASE_RENDERER)
    parser.add_argument("--base-renderer-sha256", required=True)
    parser.add_argument("--shared-renderer", type=Path, default=DEFAULT_SHARED_RENDERER)
    parser.add_argument("--shared-renderer-sha256", required=True)
    return parser


def _load_package(support: Any, base: Any, asset_dir: Path, urdf_path: Path) -> Any:
    """Adapt the audited PartNet parser to an explicitly named URDF."""
    asset_dir = asset_dir.expanduser().resolve(strict=True)
    urdf_path = urdf_path.expanduser().resolve(strict=True)
    try:
        urdf_path.relative_to(asset_dir)
    except ValueError as exc:
        raise RuntimeError(f"URDF escapes declared asset root: {urdf_path}") from exc
    original = base._contained_file

    def resolve(root: Path, value: object, *, field: str) -> Path:
        if field == "mobility.urdf":
            return urdf_path
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
        ):
            raise RuntimeError(f"{field} must be a contained relative path")
        try:
            candidate = (urdf_path.parent / relative).resolve(strict=True)
            candidate.relative_to(asset_dir)
        except (OSError, ValueError) as exc:
            raise RuntimeError(f"{field} must resolve inside the asset root") from exc
        if not candidate.is_file():
            raise RuntimeError(f"{field} does not resolve to a regular file")
        return candidate

    base._contained_file = resolve
    try:
        return support.load_partnet_package(asset_dir, base)
    finally:
        base._contained_file = original


def main(argv: Sequence[str] | None = None) -> int:
    raw = sys.argv[sys.argv.index("--") + 1 :] if argv is None and "--" in sys.argv else argv
    args = _parser().parse_args(raw)
    try:
        support_path = args.support_renderer.expanduser().resolve(strict=True)
        support = _load(
            support_path, "mobility_partnet_support", args.support_renderer_sha256
        )
        base_path = args.base_renderer.expanduser().resolve(strict=True)
        base_sha = _sha256(base_path)
        if base_sha != args.base_renderer_sha256:
            raise RuntimeError(
                f"base renderer SHA-256 mismatch: expected {args.base_renderer_sha256}, found {base_sha}"
            )
        base = support._load_module(base_path, args.base_renderer_sha256, "mobility_articraft_support")
        asset_dir = args.asset_dir.expanduser().resolve(strict=True)
        urdf_path = args.urdf_path.expanduser().resolve(strict=True)
        package = _load_package(support, base, asset_dir, urdf_path)
        result = support.render_asset(
            package,
            args.output,
            resolution=args.resolution,
            samples=args.samples,
            shared_renderer=args.shared_renderer,
            shared_renderer_sha256=args.shared_renderer_sha256,
            base=base,
            base_receipt={"path": str(base_path), "sha256": args.base_renderer_sha256},
        )
        # Keep the source receipt useful to runners even when the package root
        # is a shared dataset directory (as in PhysX-Mobility).
        result["asset_dir"] = str(asset_dir)
        result["urdf_path"] = str(urdf_path)
        result["worker"] = {"path": str(SCRIPT), "sha256": _sha256(SCRIPT)}
        result["support_renderer"] = {
            "path": str(support_path), "sha256": args.support_renderer_sha256
        }
    except Exception as exc:
        print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        return 1
    print(json.dumps(result, sort_keys=True, ensure_ascii=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
