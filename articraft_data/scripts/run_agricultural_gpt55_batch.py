"""Run the Agricultural picture folder through the Articraft OpenAI pipeline.

Each reference image under picture/Agricultural/<subcategory>/ is generated as a
workbench record with a deterministic picture.json binding. The script keeps
per-image logs/results so interrupted batches can be resumed without guessing.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import subprocess
import traceback
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from agent.providers.factory import validate_provider_credentials
from articraft.config import load_repo_env
from storage.picture_binding import binding_from_picture_path, write_binding
from storage.repo import StorageRepo


MODEL_ID = "gpt-5.5"
PROVIDER = "openai"
THINKING_LEVEL = "high"
IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".webp"}

QUALITY_PROMPT = """Use the attached reference image as the primary visual source.
Create one high-quality articulated 3D Articraft asset for picture/Agricultural/{subcategory}/{image_name}.
It must remain exactly in the small class "{subcategory}" under Agricultural, not a neighboring object.

Quality bar:
- Model the full visible object, not a symbolic proxy: recognizable proportions, silhouettes, handles, brackets, fasteners, seams, bevels, panels, wheels/tines/vents/arms, and material separation.
- Build real connected subassemblies with plausible supports, pivots, axles, rails, hinges, linkages, stops, and mounting hardware where the reference implies them.
- Include at least one meaningful non-fixed joint matching the object mechanism, with realistic axis, limits, and visible moving geometry.
- Use realistic agricultural materials and colors: painted steel, rubber tires/grips, galvanized or aluminum sheet, plastic hoppers/cans, glass/polycarbonate panels, soil-contact worn metal, and small dark hardware where appropriate.
- Avoid floating parts, paper-thin placeholder boxes, accidental intersections, and color-only detail. Add geometry for the specific details you claim.
- Add prompt-specific run_tests() checks for the named small class, visible mechanism, key subassemblies, and at least one non-fixed joint.

Object-specific mechanism and detail requirements:
{specific_requirements}
"""

SPECIFIC_REQUIREMENTS = {
    "Greenhouse vent roof": (
        "Make a greenhouse roof vent assembly with transparent/polycarbonate roof panels, "
        "aluminum/galvanized frame rails, hinge knuckles along the ridge or upper edge, "
        "an opening vent sash, weather seals, latch/handle hardware, and a visible actuator "
        "or stay arm. The primary articulation should be the vent roof panel opening on "
        "a revolute hinge with believable limits."
    ),
    "Hand cultivator": (
        "Make a handheld cultivator/tiller tool with a long handle or twin handles, "
        "soil-working tines/claws or star wheels, collars, bolts, worn metal tips, and "
        "grippy hand contact areas. The primary articulation should be a rotating tine "
        "wheel or adjustable cultivating head, with clear axle/pivot hardware."
    ),
    "Harvester vehicle (arm)": (
        "Make an agricultural harvester vehicle with cabin/chassis, large tires or tracks, "
        "panels, steps, lights, intake/header detail, hydraulic cylinders/hoses, and a "
        "multi-part harvesting arm/boom. The primary articulation should be the arm with "
        "revolute boom joints and visible hydraulic supports, plus optional rotating wheel "
        "joints if useful."
    ),
    "Seed spreader": (
        "Make the specific tow-behind seed/fertilizer spreader in the reference: an open-top, "
        "hollow olive/black plastic hopper with thick rolled rim, visible inner walls, and no "
        "fabric/tarp cover. The hopper must not be a solid block: model a real interior cavity "
        "and a clear bottom throat/opening where seed can pass downward. Under that opening add "
        "a sliding flow gate, linkage brackets, control lever/cable, short chute, and diffuser/"
        "spinner plate so the seed path from hollow hopper through the bottom aperture to the "
        "spinner is readable. Match the red tubular tow frame, front hitch tongue, upright "
        "height/flow handle, cross braces, axle, large treaded pneumatic tires with silver hubs, "
        "bolts, washers, brackets, seams, bevels, and small dark hardware. The primary "
        "articulations should include rotating wheels or spinner plate plus a movable gate/lever "
        "that visibly opens and closes the bottom seed outlet."
    ),
    "Single-Wheelbarrow": (
        "Make a single-wheel wheelbarrow with deep tray, front wheel, axle brackets, "
        "support legs, handles/grips, tray lip, braces, bolts, and rubber tire detail. "
        "The primary articulation should be the front wheel rotating on its axle, with "
        "the axle and bracket geometry visibly supporting it."
    ),
    "Tractor": (
        "Make a detailed tractor with hood, cab/seat/roll bar as appropriate to the "
        "reference, large rear tires, front wheels, fenders, hitch, axle housings, lights, "
        "exhaust, grille, steering column, and panel seams. The primary articulation should "
        "include wheel rotation and a believable steering/front axle or hitch linkage where visible."
    ),
    "Watering can": (
        "Make a watering can with hollow body, fill opening, spout, sprinkler rose/nozzle "
        "perforations, top/side handles, seams, and molded/metal material detail. The primary "
        "articulation should be a pivoting bail handle, hinged fill cap, or rotating/removable "
        "sprinkler rose, with visible hinge/pivot geometry."
    ),
}


@dataclass(frozen=True)
class BatchItem:
    index: int
    category: str
    subcategory: str
    image_path: Path
    image_rel: str
    record_hint: str
    result_path: Path
    log_path: Path


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def slugify(text: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")
    return slug or "asset"


def record_id_for(category: str, subcategory: str, image_path: Path) -> str:
    return f"rec_agricultural_gpt55_{slugify(subcategory)}_{image_path.stem}"


def iter_images(repo_root: Path, category: str) -> list[tuple[str, Path]]:
    root = repo_root / "picture" / category
    if not root.is_dir():
        raise FileNotFoundError(f"missing picture category folder: {root}")
    out: list[tuple[str, Path]] = []
    for subdir in sorted(path for path in root.iterdir() if path.is_dir()):
        for image_path in sorted(path for path in subdir.iterdir() if path.is_file()):
            if image_path.suffix.lower() in IMAGE_EXTENSIONS:
                out.append((subdir.name, image_path))
    return out


def build_prompt(subcategory: str, image_name: str) -> str:
    specific = SPECIFIC_REQUIREMENTS.get(
        subcategory,
        (
            "Capture the reference object's agricultural structure faithfully. Include "
            "visible working parts, hardware, supports, and one mechanically meaningful "
            "joint that a real object of this small class would have."
        ),
    )
    return QUALITY_PROMPT.format(
        subcategory=subcategory,
        image_name=image_name,
        specific_requirements=specific,
    ).strip()


def load_json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def make_items(
    repo_root: Path,
    category: str,
    log_root: Path,
    *,
    force: bool,
    from_index: int | None = None,
    to_index: int | None = None,
) -> list[BatchItem]:
    items: list[BatchItem] = []
    for index, (subcategory, image_path) in enumerate(iter_images(repo_root, category), start=1):
        if from_index is not None and index < from_index:
            continue
        if to_index is not None and index > to_index:
            continue
        image_rel = image_path.relative_to(repo_root).as_posix()
        record_hint = record_id_for(category, subcategory, image_path)
        result_path = log_root / f"{index:03d}_{slugify(subcategory)}_{image_path.stem}.json"
        log_path = log_root / f"{index:03d}_{slugify(subcategory)}_{image_path.stem}.log"
        existing = load_json(result_path) or {}
        existing_record_dir = repo_root / str(existing.get("record_dir") or "")
        if not force and existing.get("status") == "success" and existing_record_dir.is_dir():
            continue
        items.append(
            BatchItem(
                index=index,
                category=category,
                subcategory=subcategory,
                image_path=image_path,
                image_rel=image_rel,
                record_hint=record_hint,
                result_path=result_path,
                log_path=log_path,
            )
        )
    return items


def parse_record_dir_from_log(log_path: Path, repo_root: Path) -> tuple[str, str] | None:
    text = log_path.read_text(encoding="utf-8", errors="replace") if log_path.exists() else ""
    matches = re.findall(r"Wrote record to\s+([^\n\r]+data/records/(rec_[^\s]+))", text)
    if matches:
        full_path, record_id = matches[-1]
        try:
            record_dir = Path(full_path).resolve().relative_to(repo_root).as_posix()
        except ValueError:
            record_dir = Path(full_path).resolve().as_posix()
        return record_id, record_dir
    matches = re.findall(r"(data/records/(rec_[^\s]+))", text)
    if matches:
        record_dir, record_id = matches[-1]
        return record_id, record_dir
    return None


async def run_command_to_log(command: list[str], *, cwd: Path, log_path: Path) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        log.write(f"[{utc_now()}] command={' '.join(command)}\n\n")
        log.flush()
        process = await asyncio.create_subprocess_exec(
            *command,
            cwd=str(cwd),
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        return await process.wait()


async def run_one(
    item: BatchItem,
    *,
    repo_root: Path,
    repo: StorageRepo,
    max_cost_usd: float | None,
) -> dict[str, Any]:
    prompt = build_prompt(item.subcategory, item.image_path.name)
    payload: dict[str, Any] = {
        "started_at": utc_now(),
        "status": "running",
        "item": asdict(item) | {
            "image_path": str(item.image_path),
            "result_path": str(item.result_path),
            "log_path": str(item.log_path),
        },
        "provider": PROVIDER,
        "model_id": MODEL_ID,
        "thinking_level": THINKING_LEVEL,
        "prompt": prompt,
    }
    write_json(item.result_path, payload)

    try:
        command = [
            "uv",
            "run",
            "python",
            "-c",
            "from agent.runner_cli import main; raise SystemExit(main())",
            "--repo-root",
            str(repo_root),
            "--prompt",
            prompt,
            "--image",
            str(item.image_path),
            "--provider",
            PROVIDER,
            "--model",
            MODEL_ID,
            "--thinking",
            THINKING_LEVEL,
            "--collection",
            "workbench",
            "--label",
            f"Agricultural / {item.subcategory} / {item.image_path.name}",
            "--tag",
            "Agricultural",
            "--tag",
            item.subcategory,
            "--tag",
            "gpt55",
            "--tag",
            "picture-seed",
        ]
        if max_cost_usd is not None:
            command.extend(["--max-cost-usd", str(max_cost_usd)])

        print(
            f"[{utc_now()}] start {item.index:03d} "
            f"{item.subcategory}/{item.image_path.name}",
            flush=True,
        )
        exit_code = await run_command_to_log(command, cwd=repo_root, log_path=item.log_path)
        print(
            f"[{utc_now()}] done {item.index:03d} "
            f"{item.subcategory}/{item.image_path.name} exit_code={exit_code}",
            flush=True,
        )
        if exit_code != 0:
            raise RuntimeError(f"Articraft generation failed with exit_code={exit_code}")

        parsed = parse_record_dir_from_log(item.log_path, repo_root)
        if parsed is None:
            raise RuntimeError(f"could not parse record id from log: {item.log_path}")
        record_id, record_dir = parsed
        binding = binding_from_picture_path(item.image_rel, source="explicit")
        if binding is None:
            raise RuntimeError(f"could not build picture binding for {item.image_rel}")
        write_binding(repo, record_id, binding)
        payload.update(
            {
                "finished_at": utc_now(),
                "status": "success",
                "record_id": record_id,
                "record_dir": record_dir,
                "picture_binding": binding.to_dict(),
            }
        )
    except Exception as exc:  # noqa: BLE001
        payload.update(
            {
                "finished_at": utc_now(),
                "status": "failed",
                "error": str(exc),
                "traceback": traceback.format_exc(),
            }
        )
    write_json(item.result_path, payload)
    return payload


async def run_batch(args: argparse.Namespace) -> int:
    repo_root = args.repo_root.resolve()
    log_root = (repo_root / args.log_dir).resolve()
    log_root.mkdir(parents=True, exist_ok=True)
    repo = StorageRepo(repo_root)

    load_repo_env()
    validate_provider_credentials(PROVIDER)

    max_cost_usd = args.max_cost_usd
    if max_cost_usd is None and os.environ.get("ARTICRAFT_MAX_COST_USD"):
        max_cost_usd = float(os.environ["ARTICRAFT_MAX_COST_USD"])

    items = make_items(
        repo_root,
        args.category,
        log_root,
        force=args.force,
        from_index=args.from_index,
        to_index=args.to_index,
    )
    print(
        f"batch category={args.category} items={len(items)} concurrency={args.concurrency} "
        f"model={MODEL_ID} log_dir={log_root}",
        flush=True,
    )
    if args.dry_run:
        for item in items:
            print(f"{item.index:03d} {item.image_rel} -> {item.record_hint}")
        return 0
    if not items:
        print("nothing to do")
        return 0

    semaphore = asyncio.Semaphore(args.concurrency)

    async def guarded(item: BatchItem) -> dict[str, Any]:
        async with semaphore:
            return await run_one(item, repo_root=repo_root, repo=repo, max_cost_usd=max_cost_usd)

    results = await asyncio.gather(*(guarded(item) for item in items))
    success = sum(1 for row in results if row.get("status") == "success")
    failed = len(results) - success
    summary = {
        "finished_at": utc_now(),
        "category": args.category,
        "model_id": MODEL_ID,
        "thinking_level": THINKING_LEVEL,
        "concurrency": args.concurrency,
        "success": success,
        "failed": failed,
        "results": [
            {
                "status": row.get("status"),
                "record_id": row.get("record_id"),
                "record_hint": row.get("item", {}).get("record_hint"),
                "image_rel": row.get("item", {}).get("image_rel"),
                "error": row.get("error"),
            }
            for row in results
        ],
    }
    write_json(log_root / "summary.json", summary)
    print(f"done success={success} failed={failed} summary={log_root / 'summary.json'}", flush=True)
    return 0 if failed == 0 else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--category", default="Agricultural")
    parser.add_argument("--concurrency", type=int, default=4)
    parser.add_argument("--from-index", type=int, default=None)
    parser.add_argument("--to-index", type=int, default=None)
    parser.add_argument("--max-cost-usd", type=float, default=None)
    parser.add_argument("--log-dir", default="logs/agricultural_gpt55_batch")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return asyncio.run(run_batch(args))


if __name__ == "__main__":
    raise SystemExit(main())
