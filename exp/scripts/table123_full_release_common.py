"""Shared immutable roster and artifact contracts for the Table 1/2/3 release."""

from __future__ import annotations

import hashlib
import json
import os
import stat
import tempfile
from pathlib import Path
from typing import Any, Iterable, Mapping


SCHEMA_VERSION = "table123_full_release_manifest_v1"
ARTIFACT_SCHEMA_VERSION = "table123_artifact_manifest_v1"


class ManifestError(ValueError):
    """Raised when a roster, source binding, or artifact is not trustworthy."""


def _canonical_bytes(value: Any) -> bytes:
    try:
        encoded = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        )
    except (TypeError, ValueError) as error:
        raise ManifestError(f"value is not canonical JSON: {error}") from error
    return encoded.encode("utf-8")


def canonical_sha256(value: Any) -> str:
    """Hash a value using the release's stable compact JSON representation."""

    return hashlib.sha256(_canonical_bytes(value)).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as handle:
            for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise ManifestError(f"cannot hash file {path}: {error}") from error
    return digest.hexdigest()


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.is_symlink():
        raise ManifestError(f"refusing to overwrite symlink: {path}")
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _atomic_write_json(path: Path, value: Any) -> None:
    _atomic_write_bytes(
        Path(path),
        _canonical_bytes(value) + b"\n",
    )


def _without_hash(value: Mapping[str, Any], field: str) -> dict[str, Any]:
    result = dict(value)
    result.pop(field, None)
    return result


def _assert_no_symlink(path: Path) -> Path:
    """Reject symlinks in every existing component of a source-bound path."""

    path = Path(path)
    if not path.is_absolute():
        raise ManifestError(f"source path must be absolute: {path}")
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        try:
            info = current.lstat()
        except FileNotFoundError as error:
            raise ManifestError(f"source path does not exist: {path}") from error
        if stat.S_ISLNK(info.st_mode):
            raise ManifestError(f"source path contains symlink: {path}")
    return path


def _validate_relative(value: Any, field: str) -> None:
    if not isinstance(value, str) or not value:
        raise ManifestError(f"{field} must be a non-empty relative path")
    candidate = Path(value)
    if candidate.is_absolute() or ".." in candidate.parts:
        raise ManifestError(f"relative path escapes its source root: {value}")


def _path_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _validate_source_bindings(manifest: Mapping[str, Any], *, verify_sources: bool | str) -> None:
    """Validate manifest-level source receipts when strict loading is requested.

    Dataset roots are intentionally path-only bindings because recursively
    hashing those shared trees is handled by row package bindings.  A binding
    that supplies an explicit ``sha256`` is a small immutable receipt (for
    example the Infinigen archive validation receipt) and must be checked at
    runner/verifier preflight time.
    """

    if not verify_sources:
        return
    bindings = manifest.get("source_bindings", [])
    if not isinstance(bindings, list):
        raise ManifestError("source_bindings must be a list")
    for binding in bindings:
        if not isinstance(binding, Mapping):
            raise ManifestError("source binding must be an object")
        path_value = binding.get("path")
        declared = binding.get("sha256")
        if path_value is None:
            if declared is not None:
                raise ManifestError("hashed source binding is missing path")
            continue
        try:
            path = _assert_no_symlink(Path(path_value)).resolve(strict=False)
        except (TypeError, ValueError) as error:
            raise ManifestError(f"invalid source binding path: {path_value}") from error
        if not path.exists():
            raise ManifestError(f"source binding path does not exist: {path}")
        if declared is None:
            continue
        if not path.is_file():
            raise ManifestError(f"hashed source binding is not a file: {path}")
        if not isinstance(declared, str) or len(declared) != 64:
            raise ManifestError(f"source binding SHA-256 is malformed: {path}")
        try:
            observed = sha256_file(path)
        except ManifestError as error:
            raise ManifestError(f"source binding cannot be hashed: {path}") from error
        if observed != declared:
            name = binding.get("name", path)
            raise ManifestError(f"source binding drift: {name}")


def _primary_path(row: Mapping[str, Any]) -> Path | None:
    for key in ("primary_urdf_path", "urdf_path", "primary_urdf"):
        value = row.get(key)
        if value is not None:
            return Path(value)
    return None


def _joint_count(row: Mapping[str, Any]) -> int:
    for key in ("non_fixed_joints", "movable_joints", "declared_non_fixed_joints"):
        value = row.get(key)
        if value is not None:
            if not isinstance(value, list):
                raise ManifestError(f"{key} must be a list")
            return len(value)
    return 0


def _package_binding(source: Path) -> tuple[list[dict[str, Any]], str]:
    files: list[dict[str, Any]] = []
    for candidate in sorted(source.rglob("*"), key=lambda item: item.as_posix()):
        info = candidate.lstat()
        if stat.S_ISLNK(info.st_mode):
            raise ManifestError(f"package contains symlink: {candidate}")
        if not stat.S_ISREG(info.st_mode):
            continue
        relative = candidate.relative_to(source).as_posix()
        files.append(
            {
                "path": relative,
                "size": info.st_size,
                "sha256": sha256_file(candidate),
            }
        )
    return files, canonical_sha256(files)


def _normalize_package_files(value: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    normalized: list[dict[str, Any]] = []
    for item in value:
        if not isinstance(item, Mapping) or not isinstance(item.get("path"), str):
            raise ManifestError("package file binding entry is invalid")
        size = item.get("size", item.get("bytes"))
        digest = item.get("sha256")
        if not isinstance(size, int) or size < 0 or not isinstance(digest, str) or len(digest) != 64:
            raise ManifestError("package file binding requires size and SHA-256")
        normalized.append({"path": item["path"], "size": size, "sha256": digest})
    return sorted(normalized, key=lambda item: item["path"])


def _bind_row(
    row: Mapping[str, Any],
    *,
    verify_package: bool = True,
    verify_primary: bool = True,
) -> dict[str, Any]:
    if not isinstance(row, Mapping):
        raise ManifestError("roster row must be an object")
    bound = dict(row)
    asset_id = bound.get("asset_id")
    if not isinstance(asset_id, str) or not asset_id:
        raise ManifestError("roster row requires asset_id")

    source_value = bound.get("source_path") or bound.get("package_root")
    if not source_value:
        raise ManifestError(f"roster row {asset_id} requires source_path")

    for key, value in list(bound.items()):
        if key.endswith("_relative_path") or key in {"source_relative_path", "portable_path"}:
            _validate_relative(value, key)

    if source_value is not None:
        source = _assert_no_symlink(Path(source_value))
        source = source.resolve(strict=False)
        if not source.exists() or not source.is_dir():
            raise ManifestError(f"source path must be an existing directory: {source}")
        bound["source_path"] = str(source)
        source_root = bound.get("source_root")
        if source_root is not None:
            root = _assert_no_symlink(Path(source_root)).resolve(strict=False)
            if not _path_within(source, root):
                raise ManifestError(f"source path escapes source_root: {source}")

    primary = _primary_path(bound)
    if primary is None:
        raise ManifestError(f"roster row {asset_id} requires primary_urdf_path")
    if primary is not None:
        if not primary.is_absolute() and source_value is not None:
            primary = Path(source_value) / primary
        primary = _assert_no_symlink(primary)
        primary = primary.resolve(strict=False)
        if not primary.exists() or not primary.is_file():
            raise ManifestError(f"primary URDF must be an existing file: {primary}")
        if source_value is not None and not _path_within(primary, Path(source_value).resolve()):
            raise ManifestError(f"primary URDF escapes source package: {primary}")
        bound["primary_urdf_path"] = str(primary)
        bound["primary_urdf_size"] = primary.stat().st_size
        if verify_primary:
            bound["primary_urdf_sha256"] = sha256_file(primary)
        elif not isinstance(bound.get("primary_urdf_sha256"), str):
            raise ManifestError(f"roster row {asset_id} is missing primary URDF hash")

    package_files = bound.get("package_files")
    if package_files is not None and not isinstance(package_files, list):
        raise ManifestError("package_files must be a list")
    if source_value is not None:
        supplied_files = bound.get("package_files")
        supplied_hash = bound.get("package_binding_sha256")
        if bound.get("package_binding_deferred"):
            # Some releases expose one shared container directory for all
            # primary URDFs (notably PhysX-Mobility).  Re-scanning that same
            # directory per asset is both redundant and prohibitively large;
            # the source-specific adapter records the authoritative binding
            # hash and the Table 2 resource audit remains fail-closed.
            bound["package_files"] = list(supplied_files or [])
            bound["package_binding_sha256"] = str(supplied_hash or "")
        elif verify_package or not supplied_files or not supplied_hash:
            package_files, package_hash = _package_binding(source)
            bound["package_files"] = package_files
            bound["package_binding_sha256"] = package_hash
        else:
            bound["package_files"] = _normalize_package_files(supplied_files)
            bound["package_binding_sha256"] = str(supplied_hash)
    bound["joint_count"] = _joint_count(bound)
    return bound


def _validate_rows(manifest: Mapping[str, Any], *, verify_sources: bool | str = False) -> None:
    rows = manifest.get("rows")
    if not isinstance(rows, list):
        raise ManifestError("manifest rows must be a list")
    ids: list[str] = []
    # ``auto`` keeps the full content attestation for small fixture/package
    # rows, while trusting the frozen package binding for very large releases.
    # The evaluators still hash each primary URDF immediately before and after
    # scoring; the independent verifier can request the expensive deep package
    # attestation once per frozen receipt.  ``inventory`` is an optional
    # metadata-only recursive check for callers that want it without payload
    # reads.
    # Keep the normal runner preflight bounded.  Large release packages are
    # checked by their frozen path/size inventory; a caller that wants the
    # payload-level package attestation uses ``verify_sources=True`` once.
    deep_package_limit = 4 * 1024 * 1024
    deep_file_limit = 64
    large_release = len(rows) > 128
    inventory_cache: dict[str, tuple[set[str], dict[str, int]]] = {}
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping):
            raise ManifestError("manifest row must be an object")
        if row.get("ordinal") != index:
            raise ManifestError("roster ordinals must be contiguous and deterministic")
        asset_id = row.get("asset_id")
        if not isinstance(asset_id, str) or not asset_id:
            raise ManifestError("roster row requires asset_id")
        ids.append(asset_id)
        expected = row.get("primary_urdf_sha256")
        primary = _primary_path(row)
        if primary is not None:
            declared_files = row.get("package_files") or []
            declared_bytes = sum(
                int(item.get("size", item.get("bytes", 0)))
                for item in declared_files
                if isinstance(item, Mapping)
            )
            deep = verify_sources is True or (
                verify_sources == "auto"
                and not large_release
                and len(declared_files) <= deep_file_limit
                and declared_bytes <= deep_package_limit
            )
            bound = _bind_row(
                row,
                verify_package=deep,
                # Small fixtures/releases get the original deep check.  For
                # large ``auto`` runs the evaluator hashes each primary URDF
                # in its own isolated worker, avoiding a duplicate preflight
                # read of the same payload.
                verify_primary=deep,
            )
            if not deep and verify_sources == "inventory":
                # Validate the frozen package inventory without reading every
                # payload.  This catches missing/replaced files and symlinks;
                # payload hashes remain available to the strict verifier.
                source = Path(str(row.get("source_path") or row.get("package_root"))).resolve()
                cache_key = str(source)
                if cache_key not in inventory_cache:
                    actual: set[str] = set()
                    sizes: dict[str, int] = {}
                    for candidate in sorted(source.rglob("*"), key=lambda item: item.as_posix()):
                        info = candidate.lstat()
                        if stat.S_ISLNK(info.st_mode):
                            raise ManifestError(f"package contains symlink: {candidate}")
                        if stat.S_ISREG(info.st_mode):
                            relative = candidate.relative_to(source).as_posix()
                            actual.add(relative)
                            sizes[relative] = info.st_size
                    inventory_cache[cache_key] = (actual, sizes)
                actual, sizes = inventory_cache[cache_key]
                declared_paths = set()
                for item in declared_files:
                    if not isinstance(item, Mapping):
                        raise ManifestError(f"package file binding entry is invalid for {asset_id}")
                    relative = item.get("path")
                    _validate_relative(relative, "package file path")
                    declared_paths.add(str(relative))
                    if str(relative) not in actual:
                        raise ManifestError(f"package file missing for {asset_id}: {relative}")
                    expected_size = int(item.get("size", item.get("bytes", -1)))
                    if sizes[str(relative)] != expected_size:
                        raise ManifestError(f"package file size drift for {asset_id}: {relative}")
                if actual != declared_paths:
                    extra = sorted(actual - declared_paths)
                    missing = sorted(declared_paths - actual)
                    detail = extra[0] if extra else missing[0]
                    raise ManifestError(f"package file inventory drift for {asset_id}: {detail}")
            if expected != bound.get("primary_urdf_sha256"):
                raise ManifestError(f"primary URDF hash drift for {asset_id}")
            if row.get("primary_urdf_size") != bound["primary_urdf_size"]:
                raise ManifestError(f"primary URDF size drift for {asset_id}")
            if row.get("package_files") != bound.get("package_files"):
                raise ManifestError(f"package file binding drift for {asset_id}")
            if row.get("package_binding_sha256") != bound.get("package_binding_sha256"):
                raise ManifestError(f"package binding hash drift for {asset_id}")
    if len(ids) != len(set(ids)):
        raise ManifestError("duplicate asset_id in roster")
    if manifest.get("N_eval") != len(rows):
        raise ManifestError("N_eval does not match roster row count")
    joint_count = sum(int(row.get("joint_count", _joint_count(row))) for row in rows)
    if manifest.get("J_eval") != joint_count:
        raise ManifestError("J_eval does not match declared movable joints")


def freeze_roster(
    rows: Iterable[dict[str, Any]],
    output: Path,
    *,
    dataset: str,
    source_bindings: list[dict[str, str]],
) -> dict[str, Any]:
    """Freeze source-bound rows into a deterministic, self-hashed manifest."""

    if not isinstance(dataset, str) or not dataset:
        raise ManifestError("dataset must be a non-empty string")
    raw = list(rows)
    # Builders may already have computed the complete package binding (often
    # from an authoritative release inventory).  Reuse it during publication;
    # verify_roster(..., verify_sources=True) remains the strict drift check.
    bound = [_bind_row(row, verify_package=False) for row in raw]
    ids = [row["asset_id"] for row in bound]
    if len(ids) != len(set(ids)):
        raise ManifestError("duplicate asset_id in roster")
    bound.sort(key=lambda row: (row["asset_id"], str(row.get("source_relative_path", ""))))
    for ordinal, row in enumerate(bound):
        row["ordinal"] = ordinal
    bindings = sorted((dict(binding) for binding in source_bindings), key=lambda item: canonical_sha256(item))
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "dataset": dataset,
        "rows": bound,
        "N_eval": len(bound),
        "J_eval": sum(row["joint_count"] for row in bound),
        "source_bindings": bindings,
        "roster_sha256": canonical_sha256(bound),
    }
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)

    output = Path(output)
    _atomic_write_json(output, manifest)
    roster_path = output.with_name("full_release_roster.jsonl")
    lines = b"".join(_canonical_bytes(row) + b"\n" for row in bound)
    _atomic_write_bytes(roster_path, lines)
    manifest["roster_jsonl_sha256"] = sha256_file(roster_path)
    manifest["manifest_content_sha256"] = canonical_sha256(
        _without_hash(manifest, "manifest_content_sha256")
    )
    _atomic_write_json(output, manifest)
    return manifest


def load_roster(
    path: Path,
    *,
    expected_dataset: str | None = None,
    verify_sources: bool | str = False,
) -> dict[str, Any]:
    path = Path(path)
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ManifestError(f"invalid roster manifest: {error}") from error
    if not isinstance(manifest, dict) or manifest.get("schema_version") != SCHEMA_VERSION:
        raise ManifestError("roster manifest schema mismatch")
    if expected_dataset is not None and manifest.get("dataset") != expected_dataset:
        raise ManifestError("roster dataset mismatch")
    declared_hash = manifest.get("manifest_content_sha256")
    if declared_hash != canonical_sha256(_without_hash(manifest, "manifest_content_sha256")):
        raise ManifestError("manifest self-hash mismatch")
    if manifest.get("roster_sha256") != canonical_sha256(manifest.get("rows")):
        raise ManifestError("roster content hash mismatch")
    _validate_source_bindings(manifest, verify_sources=verify_sources)
    _validate_rows(manifest, verify_sources=verify_sources)
    roster_path = path.with_name("full_release_roster.jsonl")
    try:
        roster_bytes = roster_path.read_bytes()
    except OSError as error:
        raise ManifestError(f"missing ordered roster JSONL: {error}") from error
    if manifest.get("roster_jsonl_sha256") != sha256_file(roster_path):
        raise ManifestError("ordered roster JSONL hash mismatch")
    parsed_rows: list[dict[str, Any]] = []
    for line in roster_bytes.splitlines():
        try:
            item = json.loads(line)
        except json.JSONDecodeError as error:
            raise ManifestError(f"invalid ordered roster JSONL: {error}") from error
        if not isinstance(item, dict):
            raise ManifestError("ordered roster JSONL row must be an object")
        parsed_rows.append(item)
    if parsed_rows != manifest["rows"]:
        raise ManifestError("ordered roster JSONL differs from manifest rows")
    return manifest


def verify_roster(path: Path) -> dict[str, Any]:
    return load_roster(path, verify_sources=True)


def write_checkpoint(path: Path, payload: dict[str, Any]) -> None:
    if not isinstance(payload, dict):
        raise ManifestError("checkpoint payload must be an object")
    checkpoint = dict(payload)
    checkpoint.pop("checkpoint_content_sha256", None)
    checkpoint["checkpoint_content_sha256"] = canonical_sha256(checkpoint)
    _atomic_write_json(Path(path), checkpoint)


def verify_artifacts(output: Path) -> None:
    """Verify every entry in an output directory's artifact manifest."""

    output = Path(output)
    manifest_path = output if output.is_file() else output / "artifact_manifest.json"
    try:
        artifact = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ManifestError(f"invalid artifact manifest: {error}") from error
    if artifact.get("schema_version") != ARTIFACT_SCHEMA_VERSION:
        raise ManifestError("artifact manifest schema mismatch")
    declared = artifact.get("artifact_manifest_content_sha256")
    if declared != canonical_sha256(_without_hash(artifact, "artifact_manifest_content_sha256")):
        raise ManifestError("artifact manifest self-hash mismatch")
    artifacts = artifact.get("artifacts")
    if not isinstance(artifacts, list):
        raise ManifestError("artifact manifest artifacts must be a list")
    root = manifest_path.parent.resolve()
    for entry in artifacts:
        if not isinstance(entry, Mapping) or not isinstance(entry.get("path"), str):
            raise ManifestError("invalid artifact entry")
        _validate_relative(entry["path"], "artifact path")
        candidate = root / entry["path"]
        if not candidate.exists():
            raise ManifestError(f"artifact is missing: {entry['path']}")
        _assert_no_symlink(candidate)
        target = candidate.resolve(strict=False)
        if not _path_within(target, root):
            raise ManifestError(f"artifact path escapes output: {entry['path']}")
        if not target.is_file():
            raise ManifestError(f"artifact is missing: {entry['path']}")
        if entry.get("size") != target.stat().st_size:
            raise ManifestError(f"artifact size mismatch: {entry['path']}")
        if entry.get("sha256") != sha256_file(target):
            raise ManifestError(f"artifact hash mismatch: {entry['path']}")
