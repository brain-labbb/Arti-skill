#!/usr/bin/env python3
"""Bind the external source receipts to completed Table 4 runs.

The evaluator works from the immutable full-release rosters.  This utility
adds an auditable link from those materialized packages to the user-supplied
``parts.zip`` and ``Infinigen-Sim`` archive release without rerunning any
asset.  It updates only provenance fields and the dependent self-hashes.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


SCRIPT = Path(__file__).resolve()
EXP_ROOT = SCRIPT.parents[1]
DEFAULT_OUTPUT_ROOT = EXP_ROOT / "runtime" / "table4_full_release_20260826"
DEFAULT_PARTS_ZIP = EXP_ROOT / "parts.zip"
DEFAULT_INFINIGEN_SOURCE = EXP_ROOT / "Infinigen-Sim"
DEFAULT_ARCHIVE_RECEIPT = (
    EXP_ROOT
    / "runtime"
    / "table123_full_release_20260825"
    / "infinigen_archive_validation_receipt.json"
)


def _runner() -> Any:
    path = SCRIPT.with_name("run_table4_full_release.py")
    spec = importlib.util.spec_from_file_location("table4_runner_for_source_binding", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import runner: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, sort_keys=True, indent=2, ensure_ascii=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _binding(name: str, path: Path, *, digest: bool = False, **extra: Any) -> dict[str, Any]:
    path = Path(path).resolve(strict=True)
    result: dict[str, Any] = {"name": name, "path": str(path)}
    if digest:
        if not path.is_file():
            raise ValueError(f"hashed source binding is not a file: {path}")
        result["sha256"] = sha256_file(path)
        result["bytes"] = path.stat().st_size
    result.update(extra)
    return result


def _dedupe(bindings: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    seen: set[tuple[str, str]] = set()
    for value in bindings:
        if not isinstance(value, Mapping):
            continue
        item = dict(value)
        key = (str(item.get("name", "")), str(item.get("path", "")))
        if key in seen:
            continue
        seen.add(key)
        result.append(item)
    return result


def _infinigen_archive_binding(receipt_path: Path) -> dict[str, Any]:
    receipt = _load(receipt_path)
    if not isinstance(receipt, Mapping):
        raise ValueError("Infinigen archive validation receipt is not an object")
    archive_root = Path(str(receipt.get("archive_root", ""))).resolve(strict=True)
    extracted_root = Path(str(receipt.get("extracted_root", ""))).resolve(strict=True)
    count = int(receipt.get("extracted_urdf_count", 0))
    if count <= 0:
        raise ValueError("archive validation receipt has no extracted URDF count")
    return _binding(
        "infinigen_archive_validation_receipt",
        receipt_path,
        digest=True,
        receipt_content_sha256=receipt.get("receipt_sha256"),
        archive_root=str(archive_root),
        extracted_root=str(extracted_root),
        extracted_urdf_count=count,
        validation_mode=receipt.get("validation_mode"),
    )


def source_bindings_for_dataset(
    dataset: str,
    manifest: Mapping[str, Any],
    *,
    parts_zip: Path = DEFAULT_PARTS_ZIP,
    infinigen_source: Path = DEFAULT_INFINIGEN_SOURCE,
    archive_receipt: Path = DEFAULT_ARCHIVE_RECEIPT,
) -> list[dict[str, Any]]:
    """Return canonical source bindings while preserving roster provenance."""

    bindings = _dedupe(manifest.get("source_bindings", []))
    # The run manifest may only contain explicit command-line bindings.  The
    # frozen roster is the authoritative source map for the release itself,
    # so carry those bindings into the run-level receipt as well.
    roster_path = manifest.get("roster") or manifest.get("roster_path")
    if roster_path:
        try:
            roster = _load(Path(str(roster_path)).resolve(strict=True))
        except (OSError, json.JSONDecodeError, ValueError):
            roster = {}
        if isinstance(roster, Mapping):
            bindings = _dedupe([*bindings, *(roster.get("source_bindings", []) or [])])
    if dataset == "infinite":
        # The generated cohort is the evaluated input; parts.zip is retained
        # as the immutable upstream source requested by the user.
        bindings = [item for item in bindings if item.get("name") != "parts_zip"]
        bindings.append(_binding("parts_zip", parts_zip, digest=True))
        cohort = EXP_ROOT / "runtime" / "infinite_mobility_urdf_table123_cohort" / "manifest.json"
        if cohort.is_file():
            bindings.append(_binding("infinite_cohort_manifest", cohort, digest=True))
    elif dataset == "infinigen":
        bindings = [
            item
            for item in bindings
            if item.get("name") not in {"source_root", "Infinigen-Sim", "infinigen_archive_validation_receipt", "extracted_root"}
        ]
        receipt_binding = _infinigen_archive_binding(archive_receipt)
        bindings.append(_binding("source_root", infinigen_source))
        bindings.append(
            _binding(
                "extracted_root",
                Path(receipt_binding["extracted_root"]),
                extracted_urdf_count=receipt_binding["extracted_urdf_count"],
                materialization="validated_archive_extraction",
            )
        )
        bindings.append(receipt_binding)
    return _dedupe(bindings)


def bind_dataset(
    output: Path,
    *,
    parts_zip: Path = DEFAULT_PARTS_ZIP,
    infinigen_source: Path = DEFAULT_INFINIGEN_SOURCE,
    archive_receipt: Path = DEFAULT_ARCHIVE_RECEIPT,
) -> dict[str, Any]:
    """Patch one completed output directory and rebuild dependent hashes."""

    runner = _runner()
    output = Path(output).resolve(strict=True)
    manifest_path = output / "manifest.json"
    summary_path = output / "summary.json"
    checkpoint_path = output / "checkpoint.json"
    manifest = _load(manifest_path)
    if not isinstance(manifest, dict):
        raise ValueError(f"run manifest is not an object: {manifest_path}")
    if str(manifest.get("schema_version", "")) not in {
        "table4_full_release_run_v1",
        "table4_full_release_run_v2",
    }:
        raise ValueError(f"unsupported Table 4 manifest: {manifest_path}")
    if str(_load(checkpoint_path).get("state", "")).lower() not in {"complete", "completed"}:
        raise ValueError(f"run is not complete: {output}")
    dataset = str(manifest.get("dataset_slug") or output.name)
    bindings = source_bindings_for_dataset(
        dataset,
        manifest,
        parts_zip=parts_zip,
        infinigen_source=infinigen_source,
        archive_receipt=archive_receipt,
    )
    existing_summary = _load(summary_path)
    if (
        isinstance(existing_summary, Mapping)
        and manifest.get("source_bindings") == bindings
        and existing_summary.get("source_bindings") == bindings
        and manifest.get("source_binding_schema") == "table4_source_binding_v1"
    ):
        return {"dataset": dataset, "output": str(output), "source_bindings": bindings, "unchanged": True}
    manifest["source_bindings"] = bindings
    manifest["source_binding_schema"] = "table4_source_binding_v1"
    manifest["manifest_content_sha256"] = runner._self_hash(manifest, "manifest_content_sha256")
    runner.atomic_json(manifest_path, manifest)

    summary = existing_summary
    if not isinstance(summary, dict):
        raise ValueError(f"summary is not an object: {summary_path}")
    summary["source_bindings"] = bindings
    summary["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    summary["summary_content_sha256"] = runner._self_hash(summary, "summary_content_sha256")
    runner.atomic_json(summary_path, summary)

    checkpoint = _load(checkpoint_path)
    checkpoint["manifest_content_sha256"] = manifest["manifest_content_sha256"]
    checkpoint["summary_sha256"] = sha256_file(summary_path)
    checkpoint["checkpoint_content_sha256"] = runner._self_hash(checkpoint, "checkpoint_content_sha256")
    runner.atomic_json(checkpoint_path, checkpoint)
    runner.atomic_json(output / "artifact_manifest.json", runner._artifact_manifest(output))
    return {"dataset": dataset, "output": str(output), "source_bindings": bindings}


def bind_all(
    output_root: Path = DEFAULT_OUTPUT_ROOT,
    *,
    datasets: Sequence[str] | None = None,
    parts_zip: Path = DEFAULT_PARTS_ZIP,
    infinigen_source: Path = DEFAULT_INFINIGEN_SOURCE,
    archive_receipt: Path = DEFAULT_ARCHIVE_RECEIPT,
) -> list[dict[str, Any]]:
    root = Path(output_root).resolve(strict=True)
    names = list(datasets or ("articraft", "lam", "artiverse", "partnet", "physx", "sketch", "infinite", "infinigen"))
    return [
        bind_dataset(
            root / dataset,
            parts_zip=parts_zip,
            infinigen_source=infinigen_source,
            archive_receipt=archive_receipt,
        )
        for dataset in names
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--dataset", action="append")
    parser.add_argument("--parts-zip", type=Path, default=DEFAULT_PARTS_ZIP)
    parser.add_argument("--infinigen-source", type=Path, default=DEFAULT_INFINIGEN_SOURCE)
    parser.add_argument("--archive-receipt", type=Path, default=DEFAULT_ARCHIVE_RECEIPT)
    args = parser.parse_args(argv)
    result = bind_all(
        args.output_root,
        datasets=args.dataset,
        parts_zip=args.parts_zip,
        infinigen_source=args.infinigen_source,
        archive_receipt=args.archive_receipt,
    )
    print(json.dumps({"datasets": result}, ensure_ascii=True, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["bind_all", "bind_dataset", "source_bindings_for_dataset"]
