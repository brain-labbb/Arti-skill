#!/usr/bin/env python3

from __future__ import annotations

import unittest
from pathlib import Path
import subprocess
import sys

from exp.scripts.prepare_pva_per_class_n5 import (
    resolve_archive_name,
    select_per_class,
)


class SelectPerClassTests(unittest.TestCase):
    def test_selects_exactly_n_rows_from_each_slug(self) -> None:
        rows = [
            {
                "slug": slug,
                "asset_id": f"seed_{seed:04d}",
                "seed": str(seed),
                "stem": slug.lower(),
                "overrides_json": "{}",
            }
            for slug in ("Alpha", "Beta")
            for seed in range(7)
        ]

        selected = select_per_class(
            rows,
            5,
            manifest_sha256="a" * 64,
            seed="fixed-seed",
        )

        self.assertEqual(len(selected), 10)
        self.assertEqual(
            {slug: sum(row["slug"] == slug for row in selected) for slug in ("Alpha", "Beta")},
            {"Alpha": 5, "Beta": 5},
        )
        self.assertEqual(selected, sorted(selected, key=lambda row: (row["slug"], row["rank_sha256"], row["asset_id"])))

    def test_same_seed_reproduces_the_same_selection(self) -> None:
        rows = [
            {"slug": "Alpha", "asset_id": f"seed_{seed:04d}"}
            for seed in range(12)
        ]

        first = select_per_class(rows, 5, manifest_sha256="b" * 64, seed="repeatable")
        second = select_per_class(reversed(rows), 5, manifest_sha256="b" * 64, seed="repeatable")

        self.assertEqual(first, second)

    def test_rejects_a_slug_with_fewer_than_n_candidates(self) -> None:
        rows = [
            {"slug": "Alpha", "asset_id": f"seed_{seed:04d}"}
            for seed in range(4)
        ]

        with self.assertRaisesRegex(ValueError, "Alpha.*4.*5"):
            select_per_class(rows, 5, manifest_sha256="c" * 64, seed="fixed-seed")


class ResolveArchiveNameTests(unittest.TestCase):
    def test_prefers_whole_archive_when_present(self) -> None:
        names = {"Alpha.tar.zst", "Alpha_part01.tar.zst"}

        self.assertEqual(resolve_archive_name("Alpha", "seed_0401", names), "Alpha.tar.zst")

    def test_maps_seed_to_400_asset_shard(self) -> None:
        names = {"Alpha_part00.tar.zst", "Alpha_part01.tar.zst"}

        self.assertEqual(resolve_archive_name("Alpha", "seed_0401", names), "Alpha_part01.tar.zst")


class CommandLineTests(unittest.TestCase):
    def test_script_file_entrypoint_can_load_repository_imports(self) -> None:
        repo = Path(__file__).resolve().parents[2]
        result = subprocess.run(
            [sys.executable, "exp/scripts/prepare_pva_per_class_n5.py", "--help"],
            cwd=repo,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--output", result.stdout)


if __name__ == "__main__":
    unittest.main()
