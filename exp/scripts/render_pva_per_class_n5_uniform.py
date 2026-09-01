#!/usr/bin/env python3
"""Render the frozen five-assets-per-class PV-A cohort with the baseline studio."""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import re
import sys
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from exp.scripts import render_pva531_uniform as baseline


DEFAULT_COHORT_MANIFEST = REPO_ROOT / "exp" / "PV-A-per-class-n5" / "manifest.json"
DEFAULT_INDEX_CSV = REPO_ROOT / "template_maps" / "generator_picture_index.csv"
DEFAULT_BASELINE_CONFIG = Path(
    "/mnt/zsn/data/particulate/datasets/PV-A/renders/"
    "uniform531_studio_256_v1/render_config.json"
)
DEFAULT_OUTPUT_ROOT = Path(
    "/mnt/zsn/data/particulate/datasets/PV-A/renders/"
    "uniform531_n5_studio_256_v1"
)
EXPECTED_COHORT_MANIFEST_SHA256 = (
    "6c1a76ffcee8f439f8d850f2574038906f4af6315968bd86bb2def6dd92d2227"
)
EXPECTED_COHORT_CONTENT_SHA256 = (
    "4bb13ac3ed1a11b1b32876141e8123c7d926910bf71263478afd53f39e86bb60"
)
EXPECTED_INDEX_SHA256 = (
    "30f719dfcb2d5db2c5e1f753e2ecf345f791f5e38802dd860c73a6175414c69d"
)
EXPECTED_BASELINE_CONFIG_SHA256 = (
    "b97cb912147db3908de0183cf2736f98477a5a17963b6c29c17d98142ed4545e"
)
EXPECTED_CLASS_COUNT = 531
EXPECTED_PER_CLASS = 5
EXPECTED_ASSET_COUNT = EXPECTED_CLASS_COUNT * EXPECTED_PER_CLASS


@dataclass(frozen=True, slots=True)
class FrozenRenderItem:
    ordinal: int
    generator_index: str
    generator_name: str
    sample_index: int
    source_type: str
    picture_category: str
    asset_id: str
    seed: int
    rank_sha256: str
    asset_dir: Path
    output_path: Path
    urdf_sha256: str
    package_content_sha256: str

    @property
    def render_key(self) -> str:
        return (
            f"{self.generator_index}__S{self.sample_index:02d}__{self.asset_id}"
        )

    def baseline_item(self) -> baseline.RenderItem:
        return baseline.RenderItem(
            ordinal=self.ordinal,
            generator_index=self.render_key,
            generator_name=self.generator_name,
            source_type=self.source_type,
            picture_category=self.picture_category,
            asset_dir=self.asset_dir,
            output_path=self.output_path,
        )


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _read_index(index_csv: Path) -> list[dict[str, str]]:
    with index_csv.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = {
        "generator_index",
        "generator_name",
        "source_type",
        "picture_category",
    }
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"generator index schema mismatch: {index_csv}")
    names: set[str] = set()
    indices: set[str] = set()
    for row in rows:
        generator_index = row["generator_index"].strip()
        generator_name = row["generator_name"].strip()
        source_type = row["source_type"].strip()
        if not re.fullmatch(r"G\d{4}", generator_index):
            raise ValueError(f"invalid generator index: {generator_index!r}")
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", generator_name):
            raise ValueError(f"unsafe generator name: {generator_name!r}")
        if source_type not in {
            "picture_backed",
            "articraft_builtin_dataset_no_picture",
        }:
            raise ValueError(f"unsupported source type: {source_type!r}")
        if generator_index in indices or generator_name in names:
            raise ValueError(
                f"duplicate generator identity: {generator_index} / {generator_name}"
            )
        indices.add(generator_index)
        names.add(generator_name)
    return rows


def load_frozen_items(
    cohort_manifest: Path,
    *,
    index_csv: Path,
    output_root: Path,
    strict_release: bool = True,
) -> tuple[tuple[FrozenRenderItem, ...], dict[str, Any]]:
    cohort_manifest = cohort_manifest.expanduser().resolve(strict=True)
    index_csv = index_csv.expanduser().resolve(strict=True)
    output_root = output_root.expanduser().resolve()
    if strict_release:
        if baseline._sha256(cohort_manifest) != EXPECTED_COHORT_MANIFEST_SHA256:
            raise ValueError(f"frozen cohort manifest SHA mismatch: {cohort_manifest}")
        if baseline._sha256(index_csv) != EXPECTED_INDEX_SHA256:
            raise ValueError(f"generator index SHA mismatch: {index_csv}")

    manifest = json.loads(cohort_manifest.read_text(encoding="utf-8"))
    declared_content_sha = manifest.get("manifest_content_sha256")
    content_payload = dict(manifest)
    content_payload.pop("manifest_content_sha256", None)
    actual_content_sha = _canonical_sha256(content_payload)
    if declared_content_sha != actual_content_sha:
        raise ValueError(
            f"cohort content SHA mismatch: {actual_content_sha} != {declared_content_sha}"
        )
    if strict_release and actual_content_sha != EXPECTED_COHORT_CONTENT_SHA256:
        raise ValueError(f"unexpected frozen cohort content SHA: {actual_content_sha}")

    index_rows = _read_index(index_csv)
    assets = manifest.get("assets")
    if not isinstance(assets, list):
        raise ValueError("cohort manifest assets must be a list")
    if strict_release:
        expected_header = {
            "n_eval": EXPECTED_ASSET_COUNT,
            "class_count": EXPECTED_CLASS_COUNT,
            "per_class": EXPECTED_PER_CLASS,
        }
        for field, expected in expected_header.items():
            if manifest.get(field) != expected:
                raise ValueError(
                    f"cohort {field} mismatch: {manifest.get(field)!r} != {expected}"
                )

    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    identities: set[tuple[str, str]] = set()
    declared_paths: set[Path] = set()
    for asset in assets:
        if not isinstance(asset, dict):
            raise ValueError("cohort asset is not an object")
        category = str(asset.get("category", ""))
        asset_id = str(asset.get("asset_id", ""))
        identity = (category, asset_id)
        if (
            not re.fullmatch(r"[A-Za-z0-9_.-]+", category)
            or not re.fullmatch(r"seed_\d{4,}", asset_id)
            or identity in identities
        ):
            raise ValueError(f"invalid or duplicate cohort identity: {identity!r}")
        identities.add(identity)
        grouped[category].append(asset)

    index_names = {row["generator_name"].strip() for row in index_rows}
    if set(grouped) != index_names:
        missing = sorted(index_names - set(grouped))
        extra = sorted(set(grouped) - index_names)
        raise ValueError(f"cohort/index category mismatch; missing={missing}, extra={extra}")
    if strict_release and (
        len(index_rows) != EXPECTED_CLASS_COUNT
        or len(assets) != EXPECTED_ASSET_COUNT
        or set(map(len, grouped.values())) != {EXPECTED_PER_CLASS}
    ):
        raise ValueError("frozen cohort is not exactly 531 classes x 5 assets")

    asset_root = (cohort_manifest.parent / "assets").resolve(strict=True)
    items: list[FrozenRenderItem] = []
    for row in index_rows:
        generator_index = row["generator_index"].strip()
        generator_name = row["generator_name"].strip()
        source_type = row["source_type"].strip()
        if source_type == "articraft_builtin_dataset_no_picture":
            source_type = "builtin_no_picture"
        category_assets = sorted(
            grouped[generator_name],
            key=lambda asset: (str(asset.get("rank_sha256", "")), str(asset["asset_id"])),
        )
        for sample_index, asset in enumerate(category_assets, start=1):
            asset_id = str(asset["asset_id"])
            expected_dir = asset_root / generator_name / asset_id
            asset_dir = baseline._inside(asset_root, Path(str(asset.get("package", ""))))
            if asset_dir != expected_dir.resolve(strict=True) or asset_dir in declared_paths:
                raise ValueError(f"unexpected or duplicate cohort package: {asset_dir}")
            declared_paths.add(asset_dir)
            urdf_path = asset_dir / "model.urdf"
            appearance_path = asset_dir / "appearance.json"
            if not urdf_path.is_file() or not appearance_path.is_file():
                raise FileNotFoundError(f"incomplete cohort package: {asset_dir}")
            urdf_sha256 = str(asset.get("urdf_sha256", ""))
            if baseline._sha256(urdf_path) != urdf_sha256:
                raise ValueError(f"URDF SHA mismatch: {urdf_path}")
            package_binding = asset.get("package_binding")
            package_content_sha256 = (
                str(package_binding.get("content_manifest_sha256", ""))
                if isinstance(package_binding, dict)
                else ""
            )
            if not re.fullmatch(r"[0-9a-f]{64}", package_content_sha256):
                raise ValueError(f"invalid package content receipt: {asset_dir}")
            render_key = f"{generator_index}__S{sample_index:02d}__{asset_id}"
            output_path = output_root / f"{render_key}__{generator_name}.png"
            items.append(
                FrozenRenderItem(
                    ordinal=len(items) + 1,
                    generator_index=generator_index,
                    generator_name=generator_name,
                    sample_index=sample_index,
                    source_type=source_type,
                    picture_category=row["picture_category"].strip(),
                    asset_id=asset_id,
                    seed=int(asset["seed"]),
                    rank_sha256=str(asset["rank_sha256"]),
                    asset_dir=asset_dir,
                    output_path=output_path,
                    urdf_sha256=urdf_sha256,
                    package_content_sha256=package_content_sha256,
                )
            )
    if len({item.render_key for item in items}) != len(items):
        raise ValueError("duplicate render keys in frozen cohort")
    if len({item.output_path for item in items}) != len(items):
        raise ValueError("duplicate output paths in frozen cohort")
    return tuple(items), manifest


def _load_baseline_contract(path: Path, *, strict_release: bool) -> dict[str, Any]:
    path = path.expanduser().resolve(strict=True)
    if strict_release and baseline._sha256(path) != EXPECTED_BASELINE_CONFIG_SHA256:
        raise ValueError(f"baseline render config SHA mismatch: {path}")
    config = json.loads(path.read_text(encoding="utf-8"))
    required = {"renderer", "library_root", "resolution", "samples", "studio"}
    if not required.issubset(config):
        raise ValueError(f"baseline render config is incomplete: {path}")
    return config


def _build_run_config(
    args: argparse.Namespace,
    *,
    items: Sequence[FrozenRenderItem],
    cohort: dict[str, Any],
    baseline_config: dict[str, Any],
    renderer: Path,
    blender_path: Path,
    library_root: Path,
    gpus: Sequence[str],
    roster_path: Path,
) -> dict[str, Any]:
    baseline_items = [item.baseline_item() for item in items]
    source_counts = Counter(item.source_type for item in items)
    per_class_counts = Counter(item.generator_name for item in items)
    count_values = sorted(set(per_class_counts.values()))
    return {
        "schema_version": 1,
        "render_contract": "pva531_n5_uniform_studio_v1",
        "class_count": len({item.generator_name for item in items}),
        "per_class": count_values[0] if len(count_values) == 1 else None,
        "per_class_count_values": count_values,
        "asset_count": len(items),
        "source_type_counts": dict(sorted(source_counts.items())),
        "cohort_manifest": str(args.cohort_manifest.expanduser().resolve(strict=True)),
        "cohort_manifest_sha256": baseline._sha256(
            args.cohort_manifest.expanduser().resolve(strict=True)
        ),
        "cohort_manifest_content_sha256": cohort["manifest_content_sha256"],
        "selection": copy.deepcopy(cohort.get("selection")),
        "index_csv": str(args.index_csv.expanduser().resolve(strict=True)),
        "index_csv_sha256": baseline._sha256(args.index_csv.expanduser().resolve(strict=True)),
        "baseline_config": str(args.baseline_config.expanduser().resolve(strict=True)),
        "baseline_config_sha256": baseline._sha256(
            args.baseline_config.expanduser().resolve(strict=True)
        ),
        "driver": str(SCRIPT),
        "driver_sha256": baseline._sha256(SCRIPT),
        "render_roster": str(roster_path),
        "render_roster_sha256": baseline._sha256(roster_path),
        "output_root": str(args.output_root),
        "renderer": str(renderer),
        "renderer_sha256": baseline._sha256(renderer),
        "blender": str(blender_path),
        "blender_version": baseline._blender_version(blender_path),
        "library_root": str(library_root),
        "input_receipt": baseline._asset_input_receipt(
            baseline_items, library_root=library_root
        ),
        "resolution": int(baseline_config["resolution"]),
        "samples": int(baseline_config["samples"]),
        "studio": copy.deepcopy(baseline_config["studio"]),
        "gpu_visibility": list(gpus),
        "workers_per_gpu": args.workers_per_gpu,
        "timeout_seconds": args.timeout_seconds,
    }


def _stable_config(config: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in config.items()
        if key not in {"gpu_visibility", "workers_per_gpu", "timeout_seconds"}
    }


def _enrich_result(
    raw: dict[str, Any], item: FrozenRenderItem, *, gpu: str
) -> dict[str, Any]:
    result = dict(raw)
    result["render_key"] = result.pop("generator_index")
    result.update(
        {
            "generator_index": item.generator_index,
            "sample_index": item.sample_index,
            "asset_id": item.asset_id,
            "seed": item.seed,
            "rank_sha256": item.rank_sha256,
            "urdf_sha256": item.urdf_sha256,
            "package_content_sha256": item.package_content_sha256,
            "gpu": gpu,
        }
    )
    return result


def _write_manifest(path: Path, results: Sequence[dict[str, Any]]) -> None:
    fields = [
        "ordinal",
        "render_key",
        "generator_index",
        "generator_name",
        "sample_index",
        "source_type",
        "picture_category",
        "asset_id",
        "seed",
        "rank_sha256",
        "asset_dir",
        "urdf_sha256",
        "package_content_sha256",
        "output_path",
        "gpu",
        "status",
        "elapsed_seconds",
        "png_bytes",
        "png_sha256",
        "started_at",
        "finished_at",
        "error",
        "renderer_result",
    ]
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in sorted(results, key=lambda row: int(row["ordinal"])):
            row = dict(result)
            row["renderer_result"] = json.dumps(
                row.get("renderer_result"), sort_keys=True, ensure_ascii=True
            )
            writer.writerow({field: row.get(field, "") for field in fields})
    temporary.replace(path)


def _write_roster(path: Path, items: Sequence[FrozenRenderItem]) -> None:
    fields = [
        "ordinal",
        "render_key",
        "generator_index",
        "generator_name",
        "sample_index",
        "source_type",
        "picture_category",
        "asset_id",
        "seed",
        "rank_sha256",
        "asset_dir",
        "urdf_sha256",
        "package_content_sha256",
        "output_path",
    ]
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in items:
            writer.writerow(
                {
                    "ordinal": item.ordinal,
                    "render_key": item.render_key,
                    "generator_index": item.generator_index,
                    "generator_name": item.generator_name,
                    "sample_index": item.sample_index,
                    "source_type": item.source_type,
                    "picture_category": item.picture_category,
                    "asset_id": item.asset_id,
                    "seed": item.seed,
                    "rank_sha256": item.rank_sha256,
                    "asset_dir": str(item.asset_dir),
                    "urdf_sha256": item.urdf_sha256,
                    "package_content_sha256": item.package_content_sha256,
                    "output_path": str(item.output_path),
                }
            )
    temporary.replace(path)


def _parse_gpus(values: Sequence[str]) -> tuple[str, ...]:
    gpus: list[str] = []
    for value in values:
        gpus.extend(part.strip() for part in value.split(",") if part.strip())
    if not gpus or any(not re.fullmatch(r"\d+", gpu) for gpu in gpus):
        raise ValueError("--gpus requires one or more numeric GPU indices")
    if len(set(gpus)) != len(gpus):
        raise ValueError("--gpus contains duplicate indices")
    return tuple(gpus)


def run(args: argparse.Namespace) -> dict[str, Any]:
    if args.workers_per_gpu < 1 or args.timeout_seconds <= 0:
        raise ValueError("workers per GPU and timeout must be positive")
    args.output_root = args.output_root.expanduser().resolve()
    gpus = _parse_gpus(args.gpus)
    items, cohort = load_frozen_items(
        args.cohort_manifest,
        index_csv=args.index_csv,
        output_root=args.output_root,
        strict_release=not args.allow_release_drift,
    )
    if args.names:
        requested = set(args.names)
        items = tuple(item for item in items if item.generator_name in requested)
        missing = sorted(requested - {item.generator_name for item in items})
        if missing:
            raise ValueError(f"unknown --names: {', '.join(missing)}")
    if args.limit is not None:
        if args.limit < 1:
            raise ValueError("--limit must be positive")
        items = items[: args.limit]
    if not items:
        raise ValueError("selection contains no render items")

    baseline_config = _load_baseline_contract(
        args.baseline_config, strict_release=not args.allow_release_drift
    )
    renderer = Path(str(baseline_config["renderer"])).resolve(strict=True)
    library_root = Path(str(baseline_config["library_root"])).resolve(strict=True)
    blender_path = Path(str(baseline_config["blender"])).resolve(strict=True)
    if baseline._sha256(renderer) != baseline_config.get("renderer_sha256"):
        raise ValueError("baseline renderer no longer matches its content receipt")

    args.output_root.mkdir(parents=True, exist_ok=True)
    (args.output_root / "logs").mkdir(exist_ok=True)
    roster_path = args.output_root / "render_roster.csv"
    _write_roster(roster_path, items)
    config = _build_run_config(
        args,
        items=items,
        cohort=cohort,
        baseline_config=baseline_config,
        renderer=renderer,
        blender_path=blender_path,
        library_root=library_root,
        gpus=gpus,
        roster_path=roster_path,
    )
    config_path = args.output_root / "render_config.json"
    if config_path.is_file():
        previous = json.loads(config_path.read_text(encoding="utf-8"))
        if _stable_config(previous) != _stable_config(config):
            raise ValueError(
                f"output root contains a different render contract: {config_path}"
            )
    else:
        baseline._write_json(config_path, config)
    if args.dry_run:
        return {"config": config, "selected": len(items), "status": "dry_run"}

    print(
        f"[render] {len(items)} frozen PV-A assets -> {args.output_root} "
        f"({config['resolution']}px, {config['samples']} samples, "
        f"GPUs={','.join(gpus)}, workers/GPU={args.workers_per_gpu})",
        flush=True,
    )
    manifest_path = args.output_root / "render_manifest.csv"
    reuse_receipts: dict[str, dict[str, str]] = {}
    if manifest_path.is_file():
        with manifest_path.open("r", encoding="utf-8", newline="") as stream:
            previous_rows = list(csv.DictReader(stream))
        keys = [row.get("render_key", "") for row in previous_rows]
        if not all(keys) or len(keys) != len(set(keys)):
            raise ValueError(f"invalid render keys in prior manifest: {manifest_path}")
        reuse_receipts = {row["render_key"]: row for row in previous_rows}

    started_at = baseline._utc_now()
    results: list[dict[str, Any]] = []
    state_path = args.output_root / "render_state.jsonl"
    executors = {
        gpu: ThreadPoolExecutor(max_workers=args.workers_per_gpu) for gpu in gpus
    }
    futures: dict[Any, tuple[FrozenRenderItem, str]] = {}
    try:
        for item in items:
            gpu = gpus[(item.ordinal - 1) % len(gpus)]
            worker_args = argparse.Namespace(**vars(args))
            worker_args.gpu = gpu
            worker_args.resolution = config["resolution"]
            worker_args.samples = config["samples"]
            future = executors[gpu].submit(
                baseline._render_one,
                item.baseline_item(),
                args=worker_args,
                blender=blender_path,
                renderer=renderer,
                library_root=library_root,
                reuse_receipt=reuse_receipts.get(item.render_key),
            )
            futures[future] = (item, gpu)
        with state_path.open("a", encoding="utf-8") as state_stream:
            for completed, future in enumerate(as_completed(futures), start=1):
                item, gpu = futures[future]
                result = _enrich_result(future.result(), item, gpu=gpu)
                results.append(result)
                state_stream.write(
                    json.dumps(result, sort_keys=True, ensure_ascii=True) + "\n"
                )
                state_stream.flush()
                _write_manifest(manifest_path, results)
                print(
                    f"[render] {completed}/{len(items)} {item.render_key} "
                    f"{result['status']} ({result['elapsed_seconds']:.1f}s, GPU {gpu})",
                    flush=True,
                )
    finally:
        for executor in executors.values():
            executor.shutdown(wait=True, cancel_futures=False)

    success_statuses = {"rendered", "reused_valid"}
    failures = [row for row in results if row["status"] not in success_statuses]
    valid_png_count = sum(
        baseline._valid_png(item.output_path, config["resolution"]) for item in items
    )
    per_category_valid = Counter(
        item.generator_name
        for item in items
        if baseline._valid_png(item.output_path, config["resolution"])
    )
    summary = {
        "schema_version": 1,
        "started_at": started_at,
        "finished_at": baseline._utc_now(),
        "class_count": len({item.generator_name for item in items}),
        "per_class_target": EXPECTED_PER_CLASS,
        "selected_count": len(items),
        "rendered_count": sum(row["status"] == "rendered" for row in results),
        "reused_valid_count": sum(row["status"] == "reused_valid" for row in results),
        "failure_count": len(failures),
        "valid_png_count": valid_png_count,
        "categories_with_five_valid": sum(
            count == EXPECTED_PER_CLASS for count in per_category_valid.values()
        ),
        "manifest": str(manifest_path),
        "config": str(config_path),
        "failure_keys": [row["render_key"] for row in failures],
    }
    baseline._write_json(args.output_root / "render_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    if failures:
        raise RuntimeError(f"{len(failures)} render(s) failed; rerun to resume")
    return summary


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cohort-manifest", type=Path, default=DEFAULT_COHORT_MANIFEST)
    parser.add_argument("--index-csv", type=Path, default=DEFAULT_INDEX_CSV)
    parser.add_argument("--baseline-config", type=Path, default=DEFAULT_BASELINE_CONFIG)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--gpus", nargs="+", default=["4", "6"])
    parser.add_argument("--workers-per-gpu", type=int, default=4)
    parser.add_argument("--timeout-seconds", type=float, default=900.0)
    parser.add_argument("--names", nargs="+")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-release-drift", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        run(build_argument_parser().parse_args(argv))
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
