#!/usr/bin/env python3
"""Render rest/midstate LAM representatives for the failure-case matches.

The worker uses the same AOC renderer as the existing ``lam_n5`` studio
renders.  Each selected category gets one deterministic representative and
two 256x256 PNGs: ``rest.png`` (state 0) and ``midstate.png`` (joint midpoint).
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
MATCHES = REPO / "failure_case_renders/lam_overlap_rating_le3/matches.json"
MANIFEST = REPO / "exp/remote-audit/lam_manifest.csv"
SOURCE_ROOT = REPO / "exp/Articulated-Object-Code/released_outputs"
DEFAULT_OUTPUT = REPO / "failure_case_renders/lam_similarity_pairs"
WORKER = REPO / "exp/scripts/render_articulated_object_code_asset_blender.py"
BLENDER = Path("/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender")


def _representatives(include_related: bool, include_broken: bool) -> list[dict[str, str]]:
    matches = json.loads(MATCHES.read_text(encoding="utf-8"))
    names: set[str] = set()
    sections = ["same_or_near_object_class_matches"]
    if include_related:
        sections.append("related_family_or_mechanism_matches")
    for section in sections:
        for row in matches[section]:
            for lam in row.get("lam_categories", []):
                category = str(lam.get("category") or "").strip()
                if category:
                    names.add(category)

    by_category: dict[str, list[dict[str, str]]] = {}
    with MANIFEST.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            category = str(row.get("category") or "").strip()
            tier = str(row.get("tier") or "").strip()
            asset_id = str(row.get("object_release_id") or "").strip()
            rel_path = str(row.get("rel_path") or "").strip()
            accepted_tiers = {"viable", "loads_only"}
            if include_broken:
                accepted_tiers.add("broken")
            if category in names and tier in accepted_tiers and asset_id and rel_path:
                by_category.setdefault(category, []).append(
                    {
                        "category": category,
                        "asset_id": asset_id,
                        "rel_path": rel_path,
                        "tier": tier,
                        "audit_reasons": str(row.get("reasons") or "").strip(),
                    }
                )

    chosen = []
    for category in sorted(names):
        options = sorted(
            by_category.get(category, []),
            key=lambda row: (
                {"viable": 0, "loads_only": 1, "broken": 2}.get(row["tier"], 3),
                row["asset_id"],
            ),
        )
        if options:
            chosen.append(options[0])
    return chosen


def _run_one(item: dict[str, str], output_root: Path, blender: Path, samples: int) -> dict[str, object]:
    category = item["category"]
    asset_id = item["asset_id"]
    asset_dir = SOURCE_ROOT / item["rel_path"]
    pair_dir = output_root / category / asset_id
    pair_dir.mkdir(parents=True, exist_ok=True)
    render_mode = "strict"
    if item["tier"] == "broken":
        render_mode = "tolerant_articulated"
        try:
            robot = ET.parse(asset_dir / "generated.urdf").getroot()
            mesh_refs = [
                str(mesh.get("filename") or "").strip()
                for mesh in robot.findall(".//visual/geometry/mesh")
            ]
            if mesh_refs and not any((asset_dir / ref).is_file() for ref in mesh_refs):
                render_mode = "combined_assembly_static_fallback"
        except (OSError, ET.ParseError):
            pass
    result: dict[str, object] = {
        **item,
        "asset_dir": str(asset_dir),
        "render_mode": render_mode,
        "rest": False,
        "midstate": False,
    }
    for label, fraction in (("rest", "0"), ("midstate", "0.5")):
        out = pair_dir / f"{label}.png"
        if out.is_file() and out.stat().st_size > 1000:
            result[label] = True
            continue
        cmd = [
            str(blender), "-b", "--factory-startup", "-P", str(WORKER), "--",
            "--asset-dir", str(asset_dir), "--output", str(out),
            "--resolution", "256", "--samples", str(samples), "--state-fraction", fraction,
        ]
        if item["tier"] == "broken":
            cmd.append("--tolerant-broken")
        env = os.environ.copy()
        env.setdefault("CUDA_VISIBLE_DEVICES", "0")
        proc = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        result[label] = proc.returncode == 0 and out.is_file() and out.stat().st_size > 1000
        if not result[label]:
            result[f"{label}_error"] = proc.stdout[-2000:]
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--samples", type=int, default=2)
    parser.add_argument("--same-only", action="store_true")
    parser.add_argument("--include-broken", action="store_true")
    parser.add_argument("--blender", type=Path, default=BLENDER)
    args = parser.parse_args()
    if not args.blender.is_file():
        raise SystemExit(f"Blender not found: {args.blender}")
    items = _representatives(not args.same_only, args.include_broken)
    args.output_root.mkdir(parents=True, exist_ok=True)
    print(f"[PLAN] categories={len(items)} output={args.output_root}", flush=True)
    rows: list[dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = [pool.submit(_run_one, item, args.output_root, args.blender, args.samples) for item in items]
        for index, future in enumerate(as_completed(futures), 1):
            row = future.result()
            rows.append(row)
            print(f"[{index}/{len(items)}] {row['category']}/{row['asset_id']} rest={row['rest']} midstate={row['midstate']}", flush=True)
    rows.sort(key=lambda row: (str(row["category"]), str(row["asset_id"])))
    manifest = args.output_root / "render_manifest.json"
    manifest.write_text(json.dumps({"count": len(rows), "rows": rows}, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
    ok = sum(bool(row["rest"] and row["midstate"]) for row in rows)
    print(f"[DONE] complete={ok}/{len(rows)} manifest={manifest}", flush=True)
    return 0 if ok == len(rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
