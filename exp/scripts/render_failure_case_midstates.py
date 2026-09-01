#!/usr/bin/env python3
"""Render midpoint joint poses for an Articraft static-render bundle."""

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
WORKER = REPO_ROOT / "exp/scripts/render_urdf_midstate_blender.py"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _safe_name(value: str) -> str:
    return value.replace("/", "__").replace("\\", "__")


def _urdf_for_item(item: dict[str, Any]) -> tuple[Path, Path]:
    source = item["source"]
    if source == "Articraft":
        asset_dir = Path(item["input"]).resolve(strict=True)
        urdf_path = asset_dir / "model.urdf"
    else:
        glb_path = Path(item["input"]).resolve(strict=True)
        asset_dir = glb_path.parent
        candidates = sorted((asset_dir / "urdf_w_collider").glob("*.urdf"))
        if not candidates:
            raise FileNotFoundError(f"no Artiverse URDF under {asset_dir}")
        urdf_path = candidates[0]
    if not urdf_path.is_file():
        raise FileNotFoundError(f"missing URDF: {urdf_path}")
    return asset_dir, urdf_path


def _run_one(
    item: dict[str, Any],
    output_root: Path,
    blender: Path,
    resolution: int,
    samples: int,
    blender_threads: int,
) -> dict[str, Any]:
    asset_dir, urdf_path = _urdf_for_item(item)
    original_output = Path(item["output"])
    output = output_root / original_output.parent / "midstate.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    log_stem = _safe_name(f"{item['source']}_{item['asset_id']}_midstate")
    stdout_path = output_root / "midstate_logs" / f"{log_stem}.stdout.txt"
    stderr_path = output_root / "midstate_logs" / f"{log_stem}.stderr.txt"
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(blender),
        "-b",
        "--threads",
        str(blender_threads),
        "--factory-startup",
        "-P",
        str(WORKER),
        "--",
        "--dataset",
        "articraft" if item["source"] == "Articraft" else "artiverse",
        "--asset-dir",
        str(asset_dir),
        "--urdf-path",
        str(urdf_path),
        "--output",
        str(output),
        "--resolution",
        str(resolution),
        "--samples",
        str(samples),
    ]
    result = dict(item)
    result.update(
        {
            "pose": "joint_midpoint",
            "midstate_input_asset_dir": str(asset_dir),
            "midstate_urdf_path": str(urdf_path),
        "midstate_output": str(output.relative_to(output_root)),
            "command": command,
            "started_at": _utc_now(),
        }
    )
    try:
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            env={**os.environ, "BLENDER_USER_CONFIG": str(output_root / "blender_config_midstate")},
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
        if completed.stdout.strip():
            try:
                payload = json.loads(completed.stdout.strip().splitlines()[-1])
                result["joint_midpoints"] = payload.get("joint_midpoints", [])
                result["joint_count"] = len(result["joint_midpoints"])
                result["movable_joint_count"] = sum(
                    row.get("type") in {"revolute", "continuous", "prismatic"}
                    for row in result["joint_midpoints"]
                )
            except json.JSONDecodeError:
                result["worker_json_parse"] = "failed"
    except Exception as exc:  # pragma: no cover - operational failure path
        result["status"] = "exception"
        result["error"] = repr(exc)
    result["finished_at"] = _utc_now()
    return result


def _open(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_sheets(rows: list[dict[str, Any]], output_root: Path, resolution: int) -> None:
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return
    good = [
        row
        for row in rows
        if row.get("status") == "rendered"
        and row.get("output_exists")
        and (output_root / str(row["output"])).is_file()
    ]
    if not good:
        return
    from collections import defaultdict

    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in good:
        if (output_root / str(row["midstate_output"])).is_file():
            groups[str(row["category"])].append(row)

    pair_root = output_root / "pair_sheets"
    pair_root.mkdir(parents=True, exist_ok=True)
    index_lines = [
        "<!doctype html>",
        "<meta charset='utf-8'>",
        "<title>Articraft rating &lt;= 3 rest-midstate pairs</title>",
        "<h1>Articraft rating &lt;= 3</h1>",
        "<p>Each tile is rest (left) and midpoint joint pose (right).</p>",
        "<ul>",
    ]
    columns = 3
    pair_w, pair_h = resolution * 2, resolution + 38
    sheet_paths: list[str] = []
    for category in sorted(groups):
        category_rows = sorted(groups[category], key=lambda row: row["asset_id"])
        rows_needed = (len(category_rows) + columns - 1) // columns
        sheet = Image.new("RGB", (columns * pair_w, rows_needed * pair_h), "white")
        draw = ImageDraw.Draw(sheet)
        for index, row in enumerate(category_rows):
            static_image = Image.open(output_root / str(row["output"])).convert("RGB")
            mid_image = Image.open(output_root / str(row["midstate_output"])).convert("RGB")
            x, y = (index % columns) * pair_w, (index // columns) * pair_h
            sheet.paste(static_image, (x, y))
            sheet.paste(mid_image, (x + resolution, y))
            label = f"{row['asset_id'][:44]} | rating={row.get('rating')}"
            draw.text((x + 3, y + resolution + 3), label, fill="black")
        safe_category = category.replace("/", "__").replace("\\", "__")
        sheet_path = pair_root / f"{safe_category}.png"
        sheet.save(sheet_path)
        sheet_paths.append(str(sheet_path.relative_to(output_root)))
        index_lines.append(
            f"<li><a href='{sheet_path.name}'>{category}</a> ({len(category_rows)} assets)</li>"
        )
    index_lines.extend(["</ul>", ""])
    (pair_root / "index.html").write_text("\n".join(index_lines), encoding="utf-8")
    (output_root / "pair_sheet_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "resolution": resolution,
                "category_count": len(sheet_paths),
                "asset_count": sum(len(values) for values in groups.values()),
                "sheets": sheet_paths,
            },
            indent=2,
            ensure_ascii=True,
        )
        + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle-root", type=Path, default=REPO_ROOT / "failure_case_renders")
    parser.add_argument("--blender", type=Path, default=Path("/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender"))
    parser.add_argument("--resolution", type=int, default=256)
    parser.add_argument("--samples", type=int, default=4)
    parser.add_argument("--blender-threads", type=int, default=1)
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    bundle_root = args.bundle_root.expanduser().resolve(strict=True)
    source_manifest = _open(bundle_root / "manifest.json")
    items = list(source_manifest["items"])
    output_manifest: dict[str, Any] = {
        "schema_version": 1,
        "created_at": _utc_now(),
        "pose_policy": "URDF movable joints at midpoint; continuous joints use pi/2",
        "resolution": args.resolution,
        "samples": args.samples,
        "source_manifest": str(bundle_root / "manifest.json"),
        "items": [],
    }
    print(f"rendering midpoint pose for {len(items)} items", flush=True)
    with ThreadPoolExecutor(max_workers=max(1, args.workers)) as pool:
        futures = {
            pool.submit(
                _run_one,
                item,
                bundle_root,
                args.blender,
                args.resolution,
                args.samples,
                args.blender_threads,
            ): item
            for item in items
        }
        for index, future in enumerate(as_completed(futures), start=1):
            result = future.result()
            output_manifest["items"].append(result)
            print(f"[{index}/{len(items)}] {result['status']} {result['source']} {result['asset_id']}", flush=True)
    output_manifest["items"].sort(key=lambda row: (row["category"], row["asset_id"]))
    output_manifest["rendered_count"] = sum(row.get("status") == "rendered" for row in output_manifest["items"])
    output_manifest["failed_count"] = len(items) - output_manifest["rendered_count"]
    (bundle_root / "midstate_manifest.json").write_text(
        json.dumps(output_manifest, indent=2, ensure_ascii=True) + "\n", encoding="utf-8"
    )
    _write_sheets(output_manifest["items"], bundle_root, args.resolution)
    print(json.dumps({k: output_manifest[k] for k in ("rendered_count", "failed_count")}), flush=True)
    return 0 if output_manifest["failed_count"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
