"""Run selected picture subcategories through the Articraft OpenAI pipeline."""

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


MODEL_ID = os.environ.get("PICTURE_BATCH_MODEL_ID", "gpt-5.5")
PROVIDER = os.environ.get("PICTURE_BATCH_PROVIDER", "openai")
THINKING_LEVEL = os.environ.get("PICTURE_BATCH_THINKING_LEVEL", "high")
ATTACH_IMAGE = os.environ.get("PICTURE_BATCH_ATTACH_IMAGE", "1").strip().lower() not in {
    "0",
    "false",
    "no",
    "off",
}
IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".webp"}

QUALITY_PROMPT = """Use the attached reference image as the primary visual source.
Create one high-quality articulated 3D Articraft asset for picture/{category}/{subcategory}/{image_name}.
It must remain exactly in the small class "{subcategory}" under {category}, not a neighboring object.

Quality bar:
- Model the full visible object, not a symbolic proxy: recognizable proportions, real hard-surface construction, panel seams, fasteners, brackets, handles, molded plastic, metal hardware, labels, terminals, and cable/conduit detail where appropriate.
- Build real connected subassemblies with plausible supports, pivots, axles, hinges, latches, clamps, detents, bus bars, terminal screws, covers, and mounting hardware where the reference implies them.
- Include at least one meaningful non-fixed joint matching the object mechanism, with realistic axis, limits, and visible moving geometry.
- Use realistic electrical materials and colors: molded black/gray plastic, painted sheet metal, copper/brass terminals, galvanized conduit, rubber cable jackets, warning labels, transparent inspection windows, and small dark screws.
- Avoid floating parts, placeholder boxes, accidental intersections, and color-only detail. Add geometry for the specific details you claim.
- Add prompt-specific run_tests() checks for the named small class, visible mechanism, key subassemblies, electrical-specific details, and at least one non-fixed joint.

Object-specific mechanism and detail requirements:
{specific_requirements}
"""

SPECIFIC_REQUIREMENTS = {
    "Cable reel": (
        "Make an electrical cable reel with two flanged side drums or spool cheeks, a central "
        "rotating axle, wound cable layers, plug/socket or outlet block, crank or carrying handle, "
        "support frame/feet, strain reliefs, molded rubber cable jacket, screw heads, labels, and "
        "a visible hub. The primary articulation should be the reel rotating on its axle, with "
        "optional hinged carry handle or crank handle as a second mechanism."
    ),
    "Circuit breaker": (
        "Make a molded-case or DIN-rail circuit breaker module with front toggle lever, terminal "
        "screw clamps, wire entry ports, DIN rail latch/clip, molded ribs, rating label recesses, "
        "arc-chute/vent slots, side seams, and small fasteners. The primary articulation should "
        "be the breaker toggle lever rotating between off/on positions, with a visible pivot and "
        "detent stop geometry."
    ),
    "Conduit bender": (
        "Make a manual electrical conduit bender with curved bender shoe/head, degree markings, "
        "hook lip, conduit channel/groove, long tubular handle, foot pedal, reinforced socket, "
        "cast-metal ribs, and a sample conduit segment seated in the channel. The primary "
        "articulation should show the handle/head bending action or a conduit segment rotating "
        "around the shoe with a believable bend axis and limits."
    ),
    "Distribution board panel": (
        "Make an electrical distribution board panel with sheet-metal enclosure, hinged front "
        "door or transparent inspection cover, breaker rows, bus bar or neutral/earth bars, "
        "terminal screws, cable knockouts/glands, wiring channels, labels, latch/lock, mounting "
        "holes, and panel seams. The primary articulation should be the front door opening on "
        "side hinges, with optional small breaker toggles inside."
    ),
    "Junction box": (
        "Make an electrical junction box with a molded or metal enclosure, removable hinged or "
        "screw-fastened cover, gasket lip, cable glands or conduit knockouts on multiple sides, "
        "internal terminal strip or wire nuts, ground lug, strain reliefs, mounting ears, screw "
        "bosses, small labels, and believable bundled wire ends. The primary articulation should "
        "be the cover opening on a hinge or a latch/cover assembly moving with clear limits; "
        "include visible hinge barrels or retained screws so the motion is mechanically grounded."
    ),
    "Surge protector switch": (
        "Make a plug-in surge protector switch or switched protection module with a molded "
        "plastic housing, rocker/toggle switch, indicator light lens, outlet or terminal features, "
        "reset/test details where appropriate, screw seams, rating label plate, brass/copper "
        "contacts visible in recesses, strain reliefs, and protective ribs. The primary "
        "articulation should be the rocker or toggle switch rotating about a real pivot with "
        "on/off detent stops, optionally with a small hinged dust cover or reset button."
    ),
    "Wire stripper": (
        "Make a hand wire stripper tool with two scissor-like handles, pivot screw/bolt, serrated "
        "gripping jaws, multiple gauge stripping holes, cutting notch, crimping slots, return "
        "spring or stop screw, rubberized handle grips, stamped size marks, exposed steel jaws, "
        "and layered handle plates. The primary articulation should be the two handles/jaws "
        "rotating around the central pivot with realistic limits; align the cutting and stripping "
        "features so the jaws meet plausibly."
    ),
}

TEXT_REFERENCE_DESCRIPTIONS = {
    ("Wire stripper", "001.png"): (
        "Reference 001 is a red-and-black automatic wire stripper/cutter shown open on a white "
        "background. It has two long curved handles with red rubber grip panels over a black "
        "metal/plastic body, a large circular central pivot cap, several shiny round rivets, black "
        "layered upper jaw plates, a brass/gold adjustment knob near the head, a black return "
        "spring stretched between the handles near the jaws, a red sliding stop/guide block across "
        "the upper jaw area, small screw heads, and serrated/cutting jaw details. One handle has "
        "white printed gauge markings for wire cutter, insulated crimper, non-insulated crimper, "
        "and AWG/mm ranges. The silhouette should read as a compound automatic stripper with "
        "overlapping link plates, not simple pliers."
    ),
    ("Wire stripper", "002.png"): (
        "Reference 002 is a yellow-and-black multi-function wire stripper/plier shown open from the "
        "side. It has bright yellow ergonomic handles with black rubber grip inserts, a visible "
        "black coil return spring between the handles, a ribbed black thumb wheel/adjuster near the "
        "upper handle, and exposed brushed-steel jaws with black textured jaw carriers. The head "
        "has pointed plier tips, serrated gripping teeth, scalloped stripping/crimping notches, "
        "several circular gauge holes labelled 1.0, 1.5, 2.5, 4.0 MM, a large round brushed metal "
        "pivot disk with a vertical logo/marking, and alternating black/silver layered plates. The "
        "asset should emphasize the open spring-loaded jaw mechanism and the yellow/black handle "
        "molding."
    ),
}


@dataclass(frozen=True)
class BatchItem:
    index: int
    attempt: int
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


def record_id_for(category: str, subcategory: str, image_path: Path, attempt: int = 1) -> str:
    suffix = f"_try{attempt}" if attempt > 1 else ""
    return f"rec_{slugify(category)}_gpt55_{slugify(subcategory)}_{image_path.stem}{suffix}"


def selected_subcategories(raw: str | None) -> set[str] | None:
    if not raw:
        return None
    values = {item.strip() for item in raw.split(",") if item.strip()}
    return values or None


def iter_images(
    repo_root: Path,
    category: str,
    subcategories: set[str] | None,
) -> list[tuple[str, Path]]:
    root = repo_root / "picture" / category
    if not root.is_dir():
        raise FileNotFoundError(f"missing picture category folder: {root}")
    out: list[tuple[str, Path]] = []
    for subdir in sorted(path for path in root.iterdir() if path.is_dir()):
        if subcategories is not None and subdir.name not in subcategories:
            continue
        for image_path in sorted(path for path in subdir.iterdir() if path.is_file()):
            if image_path.suffix.lower() in IMAGE_EXTENSIONS:
                out.append((subdir.name, image_path))
    return out


def build_prompt(category: str, subcategory: str, image_name: str) -> str:
    specific = SPECIFIC_REQUIREMENTS.get(
        subcategory,
        (
            "Capture the reference object faithfully with electrical-grade hard-surface detail, "
            "visible mounting hardware, wiring/cable features, and one mechanically meaningful "
            "joint that a real object of this small class would have."
        ),
    )
    prompt = QUALITY_PROMPT.format(
        category=category,
        subcategory=subcategory,
        image_name=image_name,
        specific_requirements=specific,
    ).strip()
    if not ATTACH_IMAGE:
        prompt = prompt.replace(
            "Use the attached reference image as the primary visual source.",
            (
                "No image is attached because this run uses a text-only Qwen API. "
                "Use the textual reference description below as the primary visual source."
            ),
        )
        description = TEXT_REFERENCE_DESCRIPTIONS.get((subcategory, image_name))
        if description:
            prompt = f"{prompt}\n\nTextual reference description:\n{description}"
    return prompt


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
    subcategories: set[str] | None,
    log_root: Path,
    *,
    force: bool,
    attempts: int = 1,
    from_index: int | None = None,
    to_index: int | None = None,
) -> list[BatchItem]:
    items: list[BatchItem] = []
    for index, (subcategory, image_path) in enumerate(
        iter_images(repo_root, category, subcategories), start=1
    ):
        if from_index is not None and index < from_index:
            continue
        if to_index is not None and index > to_index:
            continue
        image_rel = image_path.relative_to(repo_root).as_posix()
        for attempt in range(1, attempts + 1):
            attempt_suffix = f"_try{attempt}" if attempts > 1 else ""
            record_hint = record_id_for(category, subcategory, image_path, attempt)
            result_path = (
                log_root / f"{index:03d}_{slugify(subcategory)}_{image_path.stem}{attempt_suffix}.json"
            )
            log_path = (
                log_root / f"{index:03d}_{slugify(subcategory)}_{image_path.stem}{attempt_suffix}.log"
            )
            existing = load_json(result_path) or {}
            existing_record_dir = repo_root / str(existing.get("record_dir") or "")
            if not force and existing.get("status") == "success" and existing_record_dir.is_dir():
                continue
            items.append(
                BatchItem(
                    index=index,
                    attempt=attempt,
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
    prompt = build_prompt(item.category, item.subcategory, item.image_path.name)
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
            "--provider",
            PROVIDER,
            "--model",
            MODEL_ID,
            "--thinking",
            THINKING_LEVEL,
            "--collection",
            "workbench",
            "--label",
            f"{item.category} / {item.subcategory} / {item.image_path.name} / try{item.attempt}",
            "--tag",
            item.category,
            "--tag",
            item.subcategory,
            "--tag",
            f"provider-{slugify(PROVIDER)}",
            "--tag",
            f"model-{slugify(MODEL_ID)}",
            "--tag",
            "picture-seed",
            "--tag",
            f"try{item.attempt}",
        ]
        if ATTACH_IMAGE:
            command[command.index("--provider") : command.index("--provider")] = [
                "--image",
                str(item.image_path),
            ]
        if max_cost_usd is not None:
            command.extend(["--max-cost-usd", str(max_cost_usd)])

        print(f"[{utc_now()}] start {item.index:03d}.try{item.attempt} {item.image_rel}", flush=True)
        exit_code = await run_command_to_log(command, cwd=repo_root, log_path=item.log_path)
        print(
            f"[{utc_now()}] done {item.index:03d}.try{item.attempt} "
            f"{item.image_rel} exit_code={exit_code}",
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


def write_combined_summary(log_root: Path, *, category: str) -> dict[str, Any]:
    results = []
    for path in sorted(log_root.glob("[0-9][0-9][0-9]_*.json")):
        data = load_json(path) or {}
        item = data.get("item") if isinstance(data.get("item"), dict) else {}
        results.append(
            {
                "result_file": path.name,
                "status": data.get("status"),
                "image_rel": item.get("image_rel"),
                "attempt": item.get("attempt"),
                "record_id": data.get("record_id"),
                "record_dir": data.get("record_dir"),
                "model_id": data.get("model_id"),
                "thinking_level": data.get("thinking_level"),
                "picture_binding": data.get("picture_binding"),
                "error": data.get("error"),
            }
        )
    summary = {
        "status": "success" if all(row["status"] == "success" for row in results) else "partial",
        "category": category,
        "model_id": MODEL_ID,
        "thinking_level": THINKING_LEVEL,
        "total": len(results),
        "success": sum(row["status"] == "success" for row in results),
        "failed": sum(row["status"] == "failed" for row in results),
        "running": sum(row["status"] == "running" for row in results),
        "results": results,
    }
    write_json(log_root / "combined_summary.json", summary)
    return summary


async def run_batch(args: argparse.Namespace) -> int:
    repo_root = args.repo_root.resolve()
    subcategories = selected_subcategories(args.subcategories)
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
        subcategories,
        log_root,
        force=args.force,
        attempts=args.attempts,
        from_index=args.from_index,
        to_index=args.to_index,
    )
    print(
        f"batch category={args.category} items={len(items)} attempts={args.attempts} "
        f"concurrency={args.concurrency} "
        f"model={MODEL_ID} thinking={THINKING_LEVEL} log_dir={log_root}",
        flush=True,
    )
    if args.dry_run:
        for item in items:
            print(f"{item.index:03d} {item.image_rel} -> {item.record_hint}")
        return 0
    if not items:
        print("nothing to do")
        write_combined_summary(log_root, category=args.category)
        return 0

    semaphore = asyncio.Semaphore(args.concurrency)

    async def guarded(item: BatchItem) -> dict[str, Any]:
        async with semaphore:
            return await run_one(item, repo_root=repo_root, repo=repo, max_cost_usd=max_cost_usd)

    results = await asyncio.gather(*(guarded(item) for item in items))
    success = sum(1 for row in results if row.get("status") == "success")
    failed = len(results) - success
    summary = write_combined_summary(log_root, category=args.category)
    print(
        f"done success={success} failed={failed} "
        f"combined={log_root / 'combined_summary.json'} total={summary['total']}",
        flush=True,
    )
    return 0 if failed == 0 else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--category", required=True)
    parser.add_argument(
        "--subcategories",
        default=None,
        help="Comma-separated subcategory folder names. Defaults to every subcategory.",
    )
    parser.add_argument("--concurrency", type=int, default=4)
    parser.add_argument("--attempts", type=int, default=1)
    parser.add_argument("--from-index", type=int, default=None)
    parser.add_argument("--to-index", type=int, default=None)
    parser.add_argument("--max-cost-usd", type=float, default=None)
    parser.add_argument("--log-dir", default="logs/picture_gpt55_batch")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return asyncio.run(run_batch(args))


if __name__ == "__main__":
    raise SystemExit(main())
