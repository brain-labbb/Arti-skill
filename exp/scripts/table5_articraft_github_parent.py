#!/usr/bin/env python3
"""Freeze a Table 5 parent roster from the Articraft-10K GitHub dataset."""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[2]
EXP_ROOT = REPO_ROOT / "exp"

DEFAULT_SOURCE = EXP_ROOT / "Articraft-10K-github/records_manifest.jsonl"
DEFAULT_MATERIALIZED_ROOT = EXP_ROOT / "Articraft-10K/released_urdf"
DEFAULT_SAMPLE_SIZE = 200
DEFAULT_SALT = "arti-skill-table5-articraft-github-materialized-n200-v1"
SCHEMA_VERSION = "table5_articraft_github_parent_v1"
SAFE_COMPONENT = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")
SHA256 = re.compile(r"[0-9a-f]{64}")
GIT_OBJECT_ID = re.compile(r"(?:[0-9a-f]{40}|[0-9a-f]{64})")


class ParentManifestError(ValueError):
    """Raised when a source or frozen parent binding is not trustworthy."""


def canonical_json_bytes(value: Any, *, exclude_fields: Sequence[str] = ()) -> bytes:
    excluded = set(exclude_fields)

    def filtered(item: Any) -> Any:
        if isinstance(item, dict):
            return {
                key: filtered(child)
                for key, child in item.items()
                if key not in excluded
            }
        if isinstance(item, list):
            return [filtered(child) for child in item]
        return item

    return json.dumps(
        filtered(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def canonical_sha256(value: Any, *, exclude_fields: Sequence[str] = ()) -> str:
    return hashlib.sha256(
        canonical_json_bytes(value, exclude_fields=exclude_fields)
    ).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _safe_component(value: Any, label: str) -> str:
    if not isinstance(value, str) or SAFE_COMPONENT.fullmatch(value) is None:
        raise ParentManifestError(f"{label} is not a safe path component: {value!r}")
    return value


def _read_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ParentManifestError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise ParentManifestError(f"{label} must be a JSON object: {path}")
    return value


def _read_source_rows(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    identities: dict[str, int] = {}
    try:
        handle = path.open("rb")
    except OSError as error:
        raise ParentManifestError(
            f"cannot open source JSONL {path}: {error}"
        ) from error
    with handle:
        for line_number, raw_line in enumerate(handle, 1):
            payload = raw_line.rstrip(b"\r\n")
            if not payload.strip():
                continue
            try:
                value = json.loads(payload.decode("utf-8"))
            except (UnicodeError, json.JSONDecodeError) as error:
                raise ParentManifestError(
                    f"invalid source JSONL at {path}:{line_number}: {error}"
                ) from error
            if not isinstance(value, dict):
                raise ParentManifestError(
                    f"source JSONL row is not an object at {path}:{line_number}"
                )
            record_id = _safe_component(value.get("record_id"), "record_id")
            if record_id in identities:
                raise ParentManifestError(
                    f"duplicate record_id {record_id!r} at source lines "
                    f"{identities[record_id]} and {line_number}"
                )
            identities[record_id] = line_number
            rows.append(
                {
                    "record_id": record_id,
                    "line_number": line_number,
                    "line_sha256": hashlib.sha256(payload).hexdigest(),
                    "row_sha256": canonical_sha256(value),
                    "row": value,
                }
            )
    if not rows:
        raise ParentManifestError(f"source JSONL is empty: {path}")
    return rows, {row["record_id"]: row for row in rows}


def _git_binding(source_path: Path, source_repo: Path | None) -> dict[str, str]:
    search_root = (source_repo or source_path.parent).resolve(strict=True)

    def git(*arguments: str) -> str:
        try:
            completed = subprocess.run(
                ["git", "-C", str(search_root), *arguments],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=30,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise ParentManifestError(
                f"cannot inspect source Git repository: {error}"
            ) from error
        if completed.returncode != 0 or not completed.stdout.strip():
            detail = completed.stderr.strip()[-2000:]
            raise ParentManifestError(
                f"source is not bound to a Git repository: {search_root}: {detail}"
            )
        return completed.stdout.strip()

    root = Path(git("rev-parse", "--show-toplevel")).resolve(strict=True)
    try:
        source_path.relative_to(root)
    except ValueError as error:
        raise ParentManifestError(
            f"source manifest is outside the bound Git repository: {root}"
        ) from error
    commit = git("rev-parse", "HEAD")
    if GIT_OBJECT_ID.fullmatch(commit) is None:
        raise ParentManifestError(f"Git HEAD is not a SHA-1/256 object id: {commit!r}")
    return {"root": str(root), "commit": commit}


def _materialized_directory_ids(root: Path) -> list[str]:
    try:
        return sorted(entry.name for entry in root.iterdir() if entry.is_dir())
    except OSError as error:
        raise ParentManifestError(
            f"cannot enumerate materialized root {root}: {error}"
        ) from error


def selection_rank(salt: str, source_manifest_sha256: str, record_id: str) -> str:
    payload = (
        salt.encode("utf-8")
        + b"\0"
        + source_manifest_sha256.encode("ascii")
        + b"\0"
        + record_id.encode("utf-8")
    )
    return hashlib.sha256(payload).hexdigest()


def _selected_source_binding(
    source_entry: Mapping[str, Any],
    *,
    source_path: Path,
    source_sha256: str,
    records_root: Path,
    git_binding: Mapping[str, str],
) -> tuple[str, dict[str, Any]]:
    record_id = str(source_entry["record_id"])
    record_path = records_root / record_id / "record.json"
    record = _read_json_object(record_path, "record.json")
    if record.get("record_id") != record_id:
        raise ParentManifestError(f"record.json identity mismatch for {record_id}")
    active_revision = _safe_component(
        record.get("active_revision_id"), f"{record_id}.active_revision_id"
    )
    declared_revision = source_entry["row"].get("active_revision_id")
    if declared_revision is not None and declared_revision != active_revision:
        raise ParentManifestError(
            f"source manifest active revision mismatch for {record_id}"
        )
    model_path = records_root / record_id / "revisions" / active_revision / "model.py"
    if model_path.is_symlink() or not model_path.is_file():
        raise ParentManifestError(
            f"active model.py is missing or symlinked: {model_path}"
        )
    model_hash = sha256_file(model_path)
    hashes = record.get("hashes")
    declared_model_hash = (
        hashes.get("model_py_sha256") if isinstance(hashes, Mapping) else None
    )
    if declared_model_hash != model_hash:
        raise ParentManifestError(
            f"record.json model_py_sha256 mismatch for {record_id}"
        )
    category = record.get("category_slug")
    if not isinstance(category, str) or not category:
        raise ParentManifestError(
            f"record.json category_slug is invalid for {record_id}"
        )
    declared_category = source_entry["row"].get("category_slug")
    if declared_category is not None and declared_category != category:
        raise ParentManifestError(f"source manifest category mismatch for {record_id}")
    provenance = {
        "git_root": git_binding["root"],
        "git_commit": git_binding["commit"],
        "source_manifest_path": str(source_path),
        "source_manifest_sha256": source_sha256,
        "source_manifest_line_number": source_entry["line_number"],
        "source_manifest_line_sha256": source_entry["line_sha256"],
        "source_manifest_row_sha256": source_entry["row_sha256"],
        "source_manifest_row": deepcopy(source_entry["row"]),
        "record_json_path": str(record_path.resolve(strict=True)),
        "record_json_sha256": sha256_file(record_path.resolve(strict=True)),
        "active_revision_id": active_revision,
        "model_py_path": str(model_path.resolve(strict=True)),
        "model_py_sha256": model_hash,
        "declared_model_py_sha256": declared_model_hash,
    }
    return category, provenance


def _materialized_binding(package: Path) -> tuple[str | None, dict[str, Any]]:
    urdf_path = package / "model.urdf"
    if urdf_path.is_symlink():
        urdf_hash = None
        status = "unsafe_model_urdf_symlink"
    elif urdf_path.is_file():
        urdf_hash = sha256_file(urdf_path)
        status = "available"
    else:
        urdf_hash = None
        status = "model_urdf_missing"
    compile_report = package / "compile_report.json"
    compile_report_hash = (
        sha256_file(compile_report)
        if compile_report.is_file() and not compile_report.is_symlink()
        else None
    )
    return urdf_hash, {
        "status": status,
        "package": str(package),
        "model_urdf_path": str(urdf_path),
        "model_urdf_sha256": urdf_hash,
        "compile_report_path": str(compile_report),
        "compile_report_sha256": compile_report_hash,
    }


def build_parent_manifest(
    *,
    source: Path | str = DEFAULT_SOURCE,
    materialized_root: Path | str = DEFAULT_MATERIALIZED_ROOT,
    sample_size: int = DEFAULT_SAMPLE_SIZE,
    salt: str = DEFAULT_SALT,
    source_repo: Path | str | None = None,
) -> dict[str, Any]:
    """Build the deterministic source/materialization-intersection parent."""

    if (
        isinstance(sample_size, bool)
        or not isinstance(sample_size, int)
        or sample_size <= 0
    ):
        raise ParentManifestError("sample_size must be a positive integer")
    if not isinstance(salt, str) or not salt:
        raise ParentManifestError("salt must be a non-empty string")
    source_path = Path(source).resolve(strict=True)
    materialized = Path(materialized_root).resolve(strict=True)
    source_rows, source_by_id = _read_source_rows(source_path)
    source_hash = sha256_file(source_path)
    git = _git_binding(
        source_path, Path(source_repo) if source_repo is not None else None
    )

    # Eligibility is frozen only from source membership and first-level directory
    # names. No URDF path is inspected until after this selection is complete.
    materialized_ids = _materialized_directory_ids(materialized)
    eligible_ids = sorted(set(source_by_id).intersection(materialized_ids))
    if len(eligible_ids) < sample_size:
        raise ParentManifestError(
            f"eligible intersection has {len(eligible_ids)} assets, fewer than "
            f"sample_size={sample_size}"
        )
    ranked = sorted(
        (
            selection_rank(salt, source_hash, record_id),
            record_id,
        )
        for record_id in eligible_ids
    )
    selected = ranked[:sample_size]

    records_root = source_path.parent / "records"
    records: list[dict[str, Any]] = []
    for selection_index, (rank, record_id) in enumerate(selected):
        category, provenance = _selected_source_binding(
            source_by_id[record_id],
            source_path=source_path,
            source_sha256=source_hash,
            records_root=records_root,
            git_binding=git,
        )
        package = materialized / record_id
        urdf_hash, materialization = _materialized_binding(package)
        row: dict[str, Any] = {
            "asset_id": record_id,
            "package": str(package),
            "model_urdf_sha256": urdf_hash,
            "selection_index": selection_index,
            "selection_rank": rank,
            "category_slug": category,
            "source_provenance": provenance,
            "materialization": materialization,
        }
        row["row_sha256"] = canonical_sha256(row, exclude_fields=("row_sha256",))
        records.append(row)

    ordered_asset_ids = [row["asset_id"] for row in records]
    ordered_rank_bindings = [
        {
            "selection_index": row["selection_index"],
            "selection_rank": row["selection_rank"],
            "asset_id": row["asset_id"],
        }
        for row in records
    ]
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "dataset": "Articraft-10K",
        "source": {
            "records_manifest_path": str(source_path),
            "records_manifest_sha256": source_hash,
            "records_manifest_row_count": len(source_rows),
            "ordered_record_ids_sha256": canonical_sha256(
                [row["record_id"] for row in source_rows]
            ),
            "records_root": str(records_root.resolve(strict=True)),
            "git_root": git["root"],
            "git_commit": git["commit"],
        },
        "materialized_roster": {
            "root": str(materialized),
            "first_level_directory_count": len(materialized_ids),
            "first_level_directory_ids_sha256": canonical_sha256(materialized_ids),
            "source_intersection_count": len(eligible_ids),
            "source_intersection_ids_sha256": canonical_sha256(eligible_ids),
        },
        "selection": {
            "algorithm": (
                "sort eligible source/materialized directory-name intersection by "
                "(sha256(salt + NUL + source_manifest_sha256 + NUL + record_id), "
                "record_id), then take the first sample_size"
            ),
            "salt": salt,
            "sample_size": sample_size,
            "n_eval": sample_size,
            "eligible_count": len(eligible_ids),
            "ordered_asset_ids_sha256": canonical_sha256(ordered_asset_ids),
            "ordered_rank_bindings_sha256": canonical_sha256(ordered_rank_bindings),
            "replacement": False,
            "outcome_based_reselection": False,
            "urdf_preflight_before_selection": False,
        },
        "records": records,
    }
    manifest["manifest_content_sha256"] = canonical_sha256(
        manifest, exclude_fields=("manifest_content_sha256",)
    )
    validate_parent_manifest(manifest, verify_inputs=False)
    return manifest


def validate_parent_manifest(
    manifest: Mapping[str, Any], *, verify_inputs: bool = False
) -> None:
    if not isinstance(manifest, Mapping):
        raise ParentManifestError("parent manifest must be an object")
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ParentManifestError("parent manifest schema_version mismatch")
    expected_self_hash = canonical_sha256(
        manifest, exclude_fields=("manifest_content_sha256",)
    )
    if manifest.get("manifest_content_sha256") != expected_self_hash:
        raise ParentManifestError("parent manifest self-hash mismatch")
    source = manifest.get("source")
    materialized = manifest.get("materialized_roster")
    selection = manifest.get("selection")
    records = manifest.get("records")
    if not all(isinstance(item, Mapping) for item in (source, materialized, selection)):
        raise ParentManifestError(
            "parent source/materialized/selection binding is missing"
        )
    if not isinstance(records, list):
        raise ParentManifestError("parent records must be a list")
    sample_size = selection.get("sample_size")
    if (
        not isinstance(sample_size, int)
        or isinstance(sample_size, bool)
        or sample_size <= 0
        or len(records) != sample_size
        or selection.get("n_eval") != sample_size
    ):
        raise ParentManifestError("parent sample size binding is invalid")
    source_hash = source.get("records_manifest_sha256")
    salt = selection.get("salt")
    if (
        not isinstance(source_hash, str)
        or SHA256.fullmatch(source_hash) is None
        or not isinstance(salt, str)
        or not salt
    ):
        raise ParentManifestError("parent rank inputs are invalid")
    observed_ids: list[str] = []
    rank_bindings: list[dict[str, Any]] = []
    for index, row in enumerate(records):
        if not isinstance(row, Mapping):
            raise ParentManifestError(f"parent record {index} is not an object")
        record_id = _safe_component(row.get("asset_id"), f"records[{index}].asset_id")
        if row.get("selection_index") != index:
            raise ParentManifestError(f"selection_index mismatch for {record_id}")
        expected_rank = selection_rank(salt, source_hash, record_id)
        if row.get("selection_rank") != expected_rank:
            raise ParentManifestError(f"selection rank mismatch for {record_id}")
        if row.get("row_sha256") != canonical_sha256(
            row, exclude_fields=("row_sha256",)
        ):
            raise ParentManifestError(f"row self-hash mismatch for {record_id}")
        provenance = row.get("source_provenance")
        if (
            not isinstance(provenance, Mapping)
            or provenance.get("source_manifest_sha256") != source_hash
            or provenance.get("source_manifest_row", {}).get("record_id") != record_id
        ):
            raise ParentManifestError(f"source provenance mismatch for {record_id}")
        urdf_hash = row.get("model_urdf_sha256")
        if urdf_hash is not None and (
            not isinstance(urdf_hash, str) or SHA256.fullmatch(urdf_hash) is None
        ):
            raise ParentManifestError(f"invalid model_urdf_sha256 for {record_id}")
        observed_ids.append(record_id)
        rank_bindings.append(
            {
                "selection_index": index,
                "selection_rank": expected_rank,
                "asset_id": record_id,
            }
        )
    if len(observed_ids) != len(set(observed_ids)):
        raise ParentManifestError("selected asset IDs are duplicated")
    if observed_ids != [
        item[1]
        for item in sorted((row["selection_rank"], row["asset_id"]) for row in records)
    ]:
        raise ParentManifestError("selected records are not in frozen rank order")
    if selection.get("ordered_asset_ids_sha256") != canonical_sha256(observed_ids):
        raise ParentManifestError("ordered selected asset hash mismatch")
    if selection.get("ordered_rank_bindings_sha256") != canonical_sha256(rank_bindings):
        raise ParentManifestError("ordered rank binding hash mismatch")
    if verify_inputs:
        source_path = Path(str(source.get("records_manifest_path", ""))).resolve(
            strict=True
        )
        if sha256_file(source_path) != source_hash:
            raise ParentManifestError("source records manifest file hash mismatch")
        materialized_root = Path(str(materialized.get("root", ""))).resolve(strict=True)
        materialized_ids = _materialized_directory_ids(materialized_root)
        if len(materialized_ids) != materialized.get(
            "first_level_directory_count"
        ) or canonical_sha256(materialized_ids) != materialized.get(
            "first_level_directory_ids_sha256"
        ):
            raise ParentManifestError("materialized directory roster mismatch")


def atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
    destination = path.resolve(strict=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = (
        json.dumps(
            value,
            sort_keys=True,
            indent=2,
            ensure_ascii=True,
            allow_nan=False,
        )
        + "\n"
    )
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, destination)
    finally:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument(
        "--materialized-root", type=Path, default=DEFAULT_MATERIALIZED_ROOT
    )
    parser.add_argument("--sample-size", type=int, default=DEFAULT_SAMPLE_SIZE)
    parser.add_argument("--salt", default=DEFAULT_SALT)
    parser.add_argument("--source-repo", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    manifest = build_parent_manifest(
        source=args.source,
        materialized_root=args.materialized_root,
        sample_size=args.sample_size,
        salt=args.salt,
        source_repo=args.source_repo,
    )
    validate_parent_manifest(manifest, verify_inputs=True)
    atomic_write_json(args.out, manifest)
    print(
        json.dumps(
            {
                "out": str(args.out.resolve(strict=False)),
                "sample_size": args.sample_size,
                "eligible_count": manifest["selection"]["eligible_count"],
                "manifest_content_sha256": manifest["manifest_content_sha256"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ParentManifestError as error:
        print(f"error: {error}", file=os.sys.stderr)
        raise SystemExit(2)
