#!/usr/bin/env python3
"""Render closed/open PV-A pairs for the categories used in the comparison page."""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from build_articraft_lam_comparison_html import PVA_MATCHES


ROOT = Path(__file__).resolve().parents[2]
ROSTER = ROOT / "exp/runtime/pva531_uniform_tsne/generator_roster_resolved.csv"
ASSET_ROOT = Path("/mnt/zsn/data/particulate/datasets/PV-A/extracted")
OUTPUT_ROOT = ROOT / "failure_case_renders/pva"
BLENDER = Path("/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender")
RENDERER = ROOT / "arti-template/scripts/render_exported_asset_blender.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--samples", type=int, default=2)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--seeds", nargs="+", default=["seed_0000", "seed_0001", "seed_0002"])
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def load_items(seeds: list[str]) -> list[dict[str, str]]:
    with ROSTER.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    by_name = {row["generator_name"].strip(): row for row in rows}
    names = sorted({name for name, _ in PVA_MATCHES.values()})
    missing = sorted(set(names) - set(by_name))
    if missing:
        raise FileNotFoundError(f"PV-A roster missing categories: {missing}")
    items = []
    for name in names:
        row = by_name[name]
        index = row["generator_index"].strip()
        for seed in seeds:
            asset_dir = ASSET_ROOT / name / seed
            if not (asset_dir / "model.urdf").is_file() or not (asset_dir / "appearance.json").is_file():
                raise FileNotFoundError(f"PV-A asset package missing: {asset_dir}")
            pair_dir = OUTPUT_ROOT / f"{index}__{name}__{seed}"
            items.append({"generator_index": index, "generator_name": name, "seed": seed, "asset_dir": str(asset_dir), "pair_dir": str(pair_dir)})
    return items


def render_one(item: dict[str, str], args: argparse.Namespace) -> dict[str, object]:
    pair_dir = Path(item["pair_dir"])
    pair_dir.mkdir(parents=True, exist_ok=True)
    log_dir = OUTPUT_ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    result: dict[str, object] = {**item, "rest": False, "midstate": False}
    for label, fraction in (("rest", "0"), ("midstate", "0.5")):
        output = pair_dir / f"{label}.png"
        if output.is_file() and output.stat().st_size > 1000 and not args.force:
            result[label] = True
            continue
        command = [
            str(BLENDER), "-b", "--factory-startup", "-noaudio", "-P", str(RENDERER), "--",
            "--asset-dir", item["asset_dir"], "--output", str(output),
            "--resolution", str(args.resolution), "--samples", str(args.samples),
            "--state-fraction", fraction,
        ]
        env = os.environ.copy()
        env.setdefault("CUDA_VISIBLE_DEVICES", "0")
        completed = subprocess.run(command, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        (log_dir / f"{item['generator_index']}_{label}.log").write_text(completed.stdout, encoding="utf-8")
        result[label] = completed.returncode == 0 and output.is_file() and output.stat().st_size > 1000
        if not result[label]:
            result[f"{label}_error"] = completed.stdout[-2000:]
    return result


def main() -> int:
    args = parse_args()
    if args.workers < 1 or args.samples < 1 or args.resolution < 64:
        raise SystemExit("workers/samples must be positive and resolution >= 64")
    if not BLENDER.is_file() or not RENDERER.is_file():
        raise FileNotFoundError("Blender or renderer is missing")
    items = load_items(args.seeds)
    results: list[dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(render_one, item, args) for item in items]
        for index, future in enumerate(as_completed(futures), start=1):
            result = future.result()
            results.append(result)
            print(f"[{index}/{len(items)}] {result['generator_name']} rest={result['rest']} midstate={result['midstate']}", flush=True)
    results.sort(key=lambda row: str(row["generator_index"]))
    import json
    (OUTPUT_ROOT / "render_manifest.json").write_text(json.dumps({"count": len(results), "rows": results}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    failed = [row for row in results if not row["rest"] or not row["midstate"]]
    print(json.dumps({"count": len(results), "pairs": len(results) - len(failed), "failed": failed}, ensure_ascii=False))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
