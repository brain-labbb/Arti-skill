#!/usr/bin/env python3
"""Render all available Articraft records with rating <= 3.

Each record is written to its own directory so the static ``rest.png`` and
the corresponding midpoint-pose image can be reviewed as a pair.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
ARTICRAFT_MANIFEST = REPO_ROOT / "exp/Articraft-10K-github/records_manifest.jsonl"
ARTICRAFT_PACKAGES = REPO_ROOT / "exp/runtime/articraft_github_merged_10787_20260827/packages"
ARTICRAFT_RENDERER = REPO_ROOT / "exp/scripts/render_articraft10k_asset_blender.py"
BLENDER_DEFAULT = Path(
    "/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender"
)

CATEGORY_MAP = {
    "blender": "blender",
    "ceiling_fan": "ceiling_fan",
    "desktop_stapler_with_hinged_top_arm": "stapler",
    "electric_kettle_with_hinged_lid": "electric_kettle",
    "faucet_with_side_handle": "faucet",
    "floor_lamp": "floor_lamp",
    "frontload_washing_machine": "washing_machine",
    "laptop_clamshell": "laptop",
    "microwave_oven": "microwave",
    "monitor_mount": "monitor_arm",
    "remote": "remote",
    "skateboard": "skateboard",
    "top_load_washing_machine_with_hinged_lid": "washing_machine",
    "trigger_spray_bottle": "spray_bottle",
}

SEMANTIC_STAPLER_ASSET = "stapler/3dw/3ea85c1c8977484c8def06fff01a9692"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _safe_name(value: str) -> str:
    return value.replace("/", "__").replace("\\", "__")


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _articraft_items() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with ARTICRAFT_MANIFEST.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            category = str(row.get("category_slug") or "uncategorized")
            rating = row.get("rating")
            if not isinstance(rating, int) or rating > 3:
                continue
            record_id = str(row["record_id"])
            package = ARTICRAFT_PACKAGES / record_id
            urdf = package / "model.urdf"
            category_dir = _safe_name(category)
            asset_dir = _safe_name(record_id)
            rows.append(
                {
                    "source": "Articraft",
                    "category": category,
                    "source_category": category,
                    "asset_id": record_id,
                    "signal": "articraft_rating_le_3",
                    "rating": rating,
                    "input": str(package),
                    "input_file": str(urdf),
                    "output": str(
                        Path("articraft") / category_dir / asset_dir / "rest.png"
                    ),
                }
            )
    return sorted(rows, key=lambda row: (row["category"], row["asset_id"]))


def _run_one(
    item: dict[str, Any],
    output_root: Path,
    blender: Path,
    resolution: int,
    samples: int,
    blender_threads: int,
) -> dict[str, Any]:
    output = output_root / str(item["output"])
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.is_file() and output.read_bytes()[: len(PNG_SIGNATURE)] == PNG_SIGNATURE:
        result = dict(item)
        result.update(
            {
                "status": "rendered",
                "output_exists": True,
                "skipped": True,
                "started_at": _utc_now(),
                "finished_at": _utc_now(),
            }
        )
        return result
    log_stem = _safe_name(f"{item['source']}_{item['asset_id']}")
    stdout_path = output_root / "logs" / f"{log_stem}.stdout.txt"
    stderr_path = output_root / "logs" / f"{log_stem}.stderr.txt"
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    worker = ARTICRAFT_RENDERER
    args = ["--asset-dir", str(item["input"]), "--output", str(output)]
    command = [
        str(blender),
        "-b",
        "--threads",
        str(blender_threads),
        "--factory-startup",
        "-P",
        str(worker),
        "--",
        *args,
        "--resolution",
        str(resolution),
        "--samples",
        str(samples),
    ]
    result: dict[str, Any] = dict(item)
    result["command"] = command
    result["started_at"] = _utc_now()
    try:
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            env={**os.environ, "BLENDER_USER_CONFIG": str(output_root / "blender_config")},
            capture_output=True,
            text=True,
            timeout=900,
            check=False,
        )
        stdout_path.write_text(completed.stdout, encoding="utf-8")
        stderr_path.write_text(completed.stderr, encoding="utf-8")
        result["returncode"] = completed.returncode
        result["stdout_log"] = str(stdout_path)
        result["stderr_log"] = str(stderr_path)
        result["status"] = "rendered" if completed.returncode == 0 else "failed"
        result["output_exists"] = output.is_file() and output.read_bytes()[: len(PNG_SIGNATURE)] == PNG_SIGNATURE
        if result["status"] == "rendered" and not result["output_exists"]:
            result["status"] = "invalid_output"
    except Exception as exc:  # pragma: no cover - operational failure path
        result["status"] = "exception"
        result["error"] = repr(exc)
    result["finished_at"] = _utc_now()
    return result


def _write_contact_sheet(rows: list[dict[str, Any]], output_root: Path, resolution: int) -> None:
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return
    good = [row for row in rows if row.get("status") == "rendered" and row.get("output_exists")]
    if not good:
        return
    cell_w, cell_h = resolution, resolution + 34
    columns = 5
    sheet = Image.new("RGB", (columns * cell_w, ((len(good) + columns - 1) // columns) * cell_h), "white")
    draw = ImageDraw.Draw(sheet)
    for index, row in enumerate(good):
        image = Image.open(output_root / str(row["output"])).convert("RGB")
        x = (index % columns) * cell_w
        y = (index // columns) * cell_h
        sheet.paste(image, (x, y))
        label = f"{row['source']} | {row['category']}\n{row['asset_id'].split('/')[-1][:24]}"
        draw.multiline_text((x + 3, y + resolution + 2), label, fill="black", spacing=1)
    sheet.save(output_root / "contact_sheet.png")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=REPO_ROOT / "failure_case_renders")
    parser.add_argument("--blender", type=Path, default=BLENDER_DEFAULT)
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--blender-threads", type=int, default=1)
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    output_root = args.output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    items = _articraft_items()
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "created_at": _utc_now(),
        "resolution": args.resolution,
        "samples": args.samples,
        "articraft_count": sum(item["source"] == "Articraft" for item in items),
        "rating_policy": "all Articraft records with integer rating <= 3",
        "items": [],
    }
    print(f"rendering {len(items)} items to {output_root}", flush=True)
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = {
            pool.submit(
                _run_one,
                item,
                output_root,
                args.blender,
                args.resolution,
                args.samples,
                args.blender_threads,
            ): item
            for item in items
        }
        for index, future in enumerate(as_completed(futures), start=1):
            result = future.result()
            manifest["items"].append(result)
            print(
                f"[{index}/{len(items)}] {result['status']} {result['source']} {result['asset_id']}",
                flush=True,
            )
    manifest["items"].sort(key=lambda row: (row["source"], row["category"], row["asset_id"], row["signal"]))
    manifest["rendered_count"] = sum(row.get("status") == "rendered" for row in manifest["items"])
    manifest["failed_count"] = len(manifest["items"]) - manifest["rendered_count"]
    (output_root / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=True) + "\n", encoding="utf-8"
    )
    _write_contact_sheet(manifest["items"], output_root, args.resolution)
    print(json.dumps({k: manifest[k] for k in ("articraft_count", "rendered_count", "failed_count")}), flush=True)
    return 0 if manifest["failed_count"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
