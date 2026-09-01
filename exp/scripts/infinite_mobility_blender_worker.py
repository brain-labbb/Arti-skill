#!/usr/bin/env python3
"""Run one Infinite Mobility factory/seed inside Blender 3.6."""

from __future__ import annotations

import argparse
import os
import sys
import traceback
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-repo", type=Path, required=True)
    parser.add_argument("--parts-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--factory", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--texture-resolution", type=int, default=64)
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else None
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    for path in (args.baseline_repo, args.parts_root):
        if not path.is_dir():
            raise FileNotFoundError(path)

    sys.path.insert(0, str(args.baseline_repo))

    from infinigen.assets.utils import auxiliary_parts
    from infinigen.assets.utils import object as object_utils
    from infinigen.core.util.math import FixedSeed
    from infinigen_examples import generate_individual_assets as generator
    import trimesh
    import urdfpy

    auxiliary_parts.AUXILIARY_PATH = str(args.parts_root)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    original_mesh_init = urdfpy.Mesh.__init__

    def compatible_mesh_init(self, filename, scale=None, meshes=None):
        original_mesh_init(self, filename, scale=scale, meshes=meshes)
        if len(self.meshes) > 1:
            self.meshes = [trimesh.util.concatenate(self.meshes)]

    urdfpy.Mesh.__init__ = compatible_mesh_init

    original_export = object_utils.export_curr_scene

    def structural_export(*export_args, **export_kwargs):
        export_kwargs["image_res"] = args.texture_resolution
        return original_export(*export_args, **export_kwargs)

    object_utils.export_curr_scene = structural_export

    original_argv = sys.argv
    sys.argv = [
        "generate_individual_assets.py",
        "--output_folder",
        str(args.output_dir),
        "--factories",
        args.factory,
        "--n_images",
        "1",
        "--seed",
        str(args.seed),
        "--render",
        "none",
        "--n_workers",
        "1",
    ]
    try:
        generator_args = generator.make_args()
        generator_args.no_mod = generator_args.no_mod or generator_args.fire
        generator_args.film_transparent = (
            generator_args.film_transparent and not generator_args.hdri
        )
        try:
            with FixedSeed(1):
                generator.main(generator_args)
        except SystemExit as exc:
            if exc.code not in (None, 0):
                raise
    finally:
        sys.argv = original_argv

    print(f"NANO3D_CASE_COMPLETE factory={args.factory} seed={args.seed}")
    return 0


if __name__ == "__main__":
    try:
        exit_code = main()
    except BaseException:
        traceback.print_exc()
        exit_code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)
