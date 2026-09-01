#!/usr/bin/env python3
"""Freeze and verify the supplementary full generated Infinite Mobility cohort."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePath
import shutil
import stat
import sys
import tempfile
from typing import Any
import uuid
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
DEFAULT_PRIMARY = REPO_ROOT / "exp/runtime/infinite_mobility_v1"
DEFAULT_RECOVERY = REPO_ROOT / "exp/runtime/infinite_mobility_timeout_recovery_v1"
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/infinite_mobility_urdf_table123_cohort"
PROTOCOL_PATH = REPO_ROOT / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
APPROVED_FACTORIES = (
    "OfficeChairFactory", "BarChairFactory", "BeverageFridgeFactory", "DishwasherFactory",
    "MicrowaveFactory", "OvenFactory", "TVFactory", "TapFactory", "ToiletFactory",
    "LiteDoorFactory", "LampFactory", "PlateOnRackBaseFactory", "KitchenCabinetFactory",
    "VaseFactory", "BottleFactory", "TableCocktailFactory", "TableDiningFactory",
    "PotFactory", "PanFactory", "WindowFactory",
)
FORMAL_SEEDS = tuple(range(36))
FORMAL_COUNT = len(APPROVED_FACTORIES) * len(FORMAL_SEEDS)
APPROVED_RECOVERY_IDENTITIES = (
    "KitchenCabinetFactory/seed_008", "KitchenCabinetFactory/seed_019",
    "KitchenCabinetFactory/seed_020", "KitchenCabinetFactory/seed_023",
    "KitchenCabinetFactory/seed_031", "OfficeChairFactory/seed_026",
    "WindowFactory/seed_029",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()).hexdigest()


def manifest_self_hash(manifest: dict[str, Any]) -> str:
    value = dict(manifest)
    value.pop("manifest_content_sha256", None)
    return canonical_sha256(value)


def _relative_parts(relative: str | Path) -> tuple[str, ...]:
    path = PurePath(relative)
    if path.is_absolute() or not path.parts or any(part in {"", ".", ".."} for part in path.parts):
        raise ValueError(f"invalid relative path: {relative}")
    return tuple(path.parts)


def _no_follow(root: Path, relative: str | Path, *, directory: bool, label: str) -> Path:
    """Resolve an expected child without allowing symlink components or escape."""

    root_raw = root.absolute()
    if root_raw.is_symlink() or not root_raw.is_dir():
        raise ValueError(f"trusted {label} root is not a real directory: {root_raw}")
    current = root_raw
    for part in _relative_parts(relative):
        current = current / part
        try:
            mode = os.lstat(current).st_mode
        except OSError as error:
            raise ValueError(f"missing {label}: {current}") from error
        if stat.S_ISLNK(mode):
            raise ValueError(f"symlink in {label} path: {current}")
    resolved_root = root_raw.resolve(strict=True)
    resolved = current.resolve(strict=True)
    try:
        resolved.relative_to(resolved_root)
    except ValueError as error:
        raise ValueError(f"{label} escapes trusted root: {current}") from error
    if directory:
        if not resolved.is_dir():
            raise ValueError(f"{label} is not a directory: {current}")
    elif not resolved.is_file():
        raise ValueError(f"{label} is not a regular file: {current}")
    return resolved


def regular_file(path: Path, *, label: str) -> Path:
    """Open-time no-follow guard for an absolute artifact/source path."""

    raw = path.absolute()
    anchor = Path(raw.anchor)
    current = anchor
    for part in raw.parts[1:]:
        current = current / part
        try:
            mode = os.lstat(current).st_mode
        except OSError as error:
            raise ValueError(f"missing {label}: {raw}") from error
        if stat.S_ISLNK(mode):
            raise ValueError(f"symlink in {label} path: {current}")
    if not raw.is_file():
        raise ValueError(f"{label} is not a regular file: {raw}")
    return raw


def _read_json(path: Path) -> Any:
    checked = regular_file(path, label="JSON source")
    try:
        return json.loads(checked.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON: {checked}") from error


def scan_package(package: Path) -> dict[str, Any]:
    """One no-follow traversal deriving Table 2 binding and legacy package digest."""

    package = package.absolute()
    if package.is_symlink() or not package.is_dir():
        raise ValueError(f"package is not a real directory: {package}")
    files: list[dict[str, Any]] = []
    for raw_current, directories, names in os.walk(package, followlinks=False):
        current = Path(raw_current)
        directories.sort()
        names.sort()
        for name in directories:
            child = current / name
            if child.is_symlink():
                raise ValueError(f"package contains directory symlink: {child.relative_to(package)}")
        for name in names:
            child = current / name
            if child.is_symlink():
                raise ValueError(f"package contains file symlink: {child.relative_to(package)}")
            if not child.is_file():
                raise ValueError(f"package entry is not a regular file: {child.relative_to(package)}")
            files.append({"path": child.relative_to(package).as_posix(), "bytes": child.stat().st_size, "sha256": sha256_file(child)})
    legacy = hashlib.sha256()
    for item in sorted(files, key=lambda entry: entry["path"]):
        if Path(item["path"]).name in {"stdout.log", "stderr.log", "record.json"}:
            continue
        encoded = item["path"].encode("utf-8")
        legacy.update(len(encoded).to_bytes(8, "big"))
        legacy.update(encoded)
        legacy.update(bytes.fromhex(item["sha256"]))
    return {
        "files": files,
        "files_by_path": {item["path"]: item for item in files},
        "package_binding": {
            "file_count": len(files), "total_bytes": sum(item["bytes"] for item in files),
            "files": files, "content_manifest_sha256": canonical_sha256(files),
        },
        "baseline_package_sha256": legacy.hexdigest(),
    }


def package_binding(package: Path) -> dict[str, Any]:
    return scan_package(package)["package_binding"]


def baseline_package_sha256(package: Path) -> str:
    return scan_package(package)["baseline_package_sha256"]


def _record_index(records: Any, label: str) -> dict[tuple[str, int], dict[str, Any]]:
    if not isinstance(records, list):
        raise ValueError(f"{label} must be an array")
    indexed: dict[tuple[str, int], dict[str, Any]] = {}
    for record in records:
        if not isinstance(record, dict):
            raise ValueError(f"{label} contains a non-object")
        try:
            key = (str(record["factory"]), int(record["seed"]))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"{label} lacks factory/seed") from error
        if key in indexed:
            raise ValueError(f"duplicate {label[:-1]} record: {key[0]}/{key[1]}")
        indexed[key] = record
    return indexed


def _source_binding(path: Path, label: str) -> dict[str, str]:
    checked = regular_file(path, label=label)
    return {"label": label, "path": str(checked), "sha256": sha256_file(checked)}


def _recovery_index(repo_root: Path, recovery_root: Path) -> tuple[dict[tuple[str, int], dict[str, Any]], list[dict[str, str]]]:
    manifest_path = recovery_root / "recovery_manifest.json"
    summary_path = recovery_root / "recovery_records.json"
    manifest = _read_json(manifest_path)
    summary = _read_json(summary_path)
    cases = manifest.get("cases") if isinstance(manifest, dict) else None
    if not isinstance(cases, list) or not isinstance(summary, list):
        raise ValueError("recovery manifest and summary must contain cases")
    if manifest.get("expected_recovery_case_count") != len(cases) or canonical_sha256(cases) != canonical_sha256(summary):
        raise ValueError("recovery manifest/summary mismatch")
    indexed: dict[tuple[str, int], dict[str, Any]] = {}
    for case in cases:
        if not isinstance(case, dict):
            raise ValueError("recovery case is not an object")
        key = (str(case.get("factory")), int(case.get("seed", -1)))
        expected = recovery_root / "cases" / key[0] / f"seed_{key[1]:03d}" / "record.json"
        supplied = repo_root / str(case.get("recovery_record", ""))
        try:
            if supplied.absolute() != expected.absolute():
                raise ValueError(f"expected recovery record path: {expected}")
            record_path = _no_follow(recovery_root, expected.relative_to(recovery_root), directory=False, label="recovery record")
        except ValueError:
            raise
        expected_hash = str(case.get("recovery_record_sha256", ""))
        if sha256_file(record_path) != expected_hash:
            raise ValueError(f"recovery record hash drift: {record_path}")
        record = _read_json(record_path)
        if key in indexed or (str(record.get("factory")), record.get("seed")) != key:
            raise ValueError(f"invalid recovery identity: {key}")
        if case.get("original_status") != "TIMEOUT" or case.get("recovery_status") != "PASS" or record.get("status") != "PASS":
            raise ValueError(f"invalid recovery status: {key}")
        indexed[key] = {"case": case, "record": record, "record_path": record_path}
    return indexed, [_source_binding(manifest_path, "recovery_manifest"), _source_binding(summary_path, "recovery_records")]


def _selected_package(source_root: Path, factory: str, seed: int, record: dict[str, Any]) -> tuple[Path, str]:
    relative_package = Path("cases") / factory / f"seed_{seed:03d}" / "package"
    package = _no_follow(source_root, relative_package, directory=True, label="package")
    validation = record.get("validation")
    urdf_relpath = validation.get("urdf_path") if isinstance(validation, dict) else None
    if not isinstance(urdf_relpath, str):
        raise ValueError(f"missing validation.urdf_path for {factory}/seed_{seed:03d}")
    try:
        _no_follow(package, urdf_relpath, directory=False, label="URDF")
    except ValueError as error:
        if "invalid relative path" in str(error):
            raise ValueError(f"URDF escapes package: {urdf_relpath}") from error
        raise
    return package, urdf_relpath


def _validate_formal_rows(rows: list[dict[str, Any]], factories: list[str], seeds: list[int]) -> None:
    if tuple(factories) != APPROVED_FACTORIES or tuple(seeds) != FORMAL_SEEDS or len(rows) != FORMAL_COUNT:
        raise ValueError("formal cohort must use the approved 20 factory x 36 seed matrix")
    expected = {f"{factory}/seed_{seed:03d}" for factory in APPROVED_FACTORIES for seed in FORMAL_SEEDS}
    actual = {str(row.get("asset_id")) for row in rows}
    if actual != expected:
        raise ValueError("formal cohort identity matrix mismatch")
    timeouts = [row for row in rows if row.get("original_status") == "TIMEOUT"]
    if len(timeouts) != 7 or {row["asset_id"] for row in timeouts} != set(APPROVED_RECOVERY_IDENTITIES):
        raise ValueError("formal cohort recovery provenance mismatch")
    if any(row.get("recovery_used") is not True or not isinstance(row.get("recovery_provenance"), dict) for row in timeouts):
        raise ValueError("formal cohort has incomplete recovery provenance")
    if sum(row.get("original_status") == "PASS" for row in rows) != 713:
        raise ValueError("formal cohort must retain 713 primary PASS rows")
    for index, row in enumerate(rows, start=1):
        expected_id = f"{row.get('factory')}/seed_{int(row.get('seed', -1)):03d}"
        if row.get("asset_id") != expected_id or row.get("raw_category") != row.get("factory") or row.get("selection_index") != index:
            raise ValueError("formal cohort identity fields are inconsistent")


def build_cohort_rows(repo_root: Path, primary_root: Path, recovery_root: Path) -> list[dict[str, Any]]:
    repo_root = repo_root.resolve(strict=True)
    primary_root = primary_root.absolute()
    recovery_root = recovery_root.absolute()
    primary_manifest_path = primary_root / "manifest.json"
    primary_records_path = primary_root / "records.json"
    source_manifest = _read_json(primary_manifest_path)
    factories = source_manifest.get("factories") if isinstance(source_manifest, dict) else None
    seeds = source_manifest.get("protocol", {}).get("seeds") if isinstance(source_manifest, dict) else None
    if not isinstance(factories, list) or not isinstance(seeds, list):
        raise ValueError("primary manifest lacks factory/seed order")
    factories = [str(item) for item in factories]
    seeds = [int(item) for item in seeds]
    if len(set(factories)) != len(factories) or len(set(seeds)) != len(seeds):
        raise ValueError("primary manifest contains duplicate identities")
    primary = _record_index(_read_json(primary_records_path), "primary records")
    recovery, _ = _recovery_index(repo_root, recovery_root)
    expected = {(factory, seed) for factory in factories for seed in seeds}
    if set(primary) != expected or not set(recovery).issubset(expected):
        raise ValueError("source records do not match the declared identity matrix")
    rows: list[dict[str, Any]] = []
    for factory in factories:
        for seed in seeds:
            original = primary[(factory, seed)]
            original_hash = canonical_sha256(original)
            status = original.get("status")
            if status == "PASS":
                source_root, selected, recovery_used, recovery_provenance = primary_root, original, False, None
            elif status == "TIMEOUT":
                recovered = recovery.get((factory, seed))
                if recovered is None:
                    raise ValueError(f"missing recovery for TIMEOUT {factory}/seed_{seed:03d}")
                source_root, selected, recovery_used = recovery_root, recovered["record"], True
                recovery_provenance = {
                    **recovered["case"], "original_record_path": str(regular_file(primary_records_path, label="primary records")),
                    "original_record_sha256": original_hash, "recovery_record_path": str(recovered["record_path"]),
                    "recovery_record_sha256": sha256_file(recovered["record_path"]),
                }
            else:
                raise ValueError(f"unexpected original status for {factory}/seed_{seed:03d}: {status!r}")
            package, urdf_relpath = _selected_package(source_root, factory, seed, selected)
            scanned = scan_package(package)
            urdf_item = scanned["files_by_path"].get(urdf_relpath)
            if urdf_item is None:
                raise ValueError(f"URDF vanished during package scan: {factory}/seed_{seed:03d}")
            recorded_hash = selected.get("package_sha256")
            if recorded_hash is not None and recorded_hash != scanned["baseline_package_sha256"]:
                raise ValueError(f"baseline package hash drift for {factory}/seed_{seed:03d}")
            try:
                root = ET.parse(package / urdf_relpath).getroot()
                declared_joint_count_hint = sum(
                    joint.get("type", "").strip().lower() != "fixed" for joint in root.findall("joint")
                )
            except (ET.ParseError, OSError) as error:
                raise ValueError(f"cannot count declared joints for {factory}/seed_{seed:03d}") from error
            rows.append({
                "asset_id": f"{factory}/seed_{seed:03d}", "factory": factory, "raw_category": factory, "seed": seed,
                "original_status": status, "recovery_used": recovery_used, "recovery_provenance": recovery_provenance,
                "package_path": str(package), "urdf_relpath": urdf_relpath, "primary_urdf_sha256": urdf_item["sha256"],
                "baseline_package_sha256": scanned["baseline_package_sha256"], "package_binding": scanned["package_binding"],
                "selection_index": len(rows) + 1, "source": "recovery" if recovery_used else "primary",
                "declared_joint_count_hint": declared_joint_count_hint,
            })
    return rows


def cohort_manifest(rows: list[dict[str, Any]], *, factory_order: list[str], seeds: list[int], source_bindings: list[dict[str, str]] | None = None) -> dict[str, Any]:
    return {
        "schema_version": 2, "dataset": "Infinite Mobility", "release_status": "SUPPLEMENTARY_FULL_GENERATED_COHORT",
        "cohort_type": "SUPPLEMENTARY_FULL_GENERATED_COHORT_NOT_OFFICIAL_FINITE_RELEASE",
        "description": "All actually obtained generated packages; not an official finite release.",
        "N_release": len(rows), "N_eval": len(rows), "factory_order": factory_order, "seeds": seeds,
        "source_selection": {"PASS": "runtime/infinite_mobility_v1/cases/.../package", "TIMEOUT": "runtime/infinite_mobility_timeout_recovery_v1/cases/.../package", "identity_policy": "identity preservation with pre-freeze recovery overlay; no post-freeze reselection"},
        "source_bindings": source_bindings or [], "assets": rows,
    }


def verify_source_bindings(manifest: dict[str, Any]) -> None:
    bindings = manifest.get("source_bindings", [])
    if not isinstance(bindings, list):
        raise ValueError("source_bindings must be an array")
    for binding in bindings:
        if not isinstance(binding, dict):
            raise ValueError("invalid source binding")
        path = regular_file(Path(str(binding.get("path", ""))), label=str(binding.get("label", "source")))
        if sha256_file(path) != binding.get("sha256"):
            raise ValueError(f"source binding drift: {binding.get('label')}")


def verify_evaluation_bindings(manifest: dict[str, Any]) -> None:
    evaluation = manifest.get("evaluation")
    if evaluation is None:
        return
    if not isinstance(evaluation, dict):
        raise ValueError("invalid evaluation binding")
    pairs = (
        ("freezer_path", "freezer_sha256"), ("preparer_path", "preparer_sha256"),
        ("table1_runner_path", "table1_runner_sha256"),
        ("table1_evaluator_path", "table1_evaluator_sha256"),
        ("protocol_path", "protocol_sha256"),
    )
    for path_key, hash_key in pairs:
        path = regular_file(Path(str(evaluation.get(path_key, ""))), label=f"evaluation {path_key}")
        if sha256_file(path) != evaluation.get(hash_key):
            raise ValueError(f"evaluation binding drift: {path_key}")


def verify_cohort_manifest(path: Path, *, formal: bool = False) -> dict[str, Any]:
    if path.is_dir() and not path.is_symlink():
        path = path / "manifest.json"
    manifest = _read_json(path)
    if not isinstance(manifest, dict) or manifest.get("manifest_content_sha256") != manifest_self_hash(manifest):
        raise ValueError("cohort manifest self-hash drift")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or len({row.get("asset_id") for row in assets if isinstance(row, dict)}) != len(assets):
        raise ValueError("cohort assets are missing or non-unique")
    verify_source_bindings(manifest)
    verify_evaluation_bindings(manifest)
    if formal:
        _validate_formal_rows(assets, list(manifest.get("factory_order", [])), list(manifest.get("seeds", [])))
        if manifest.get("N_release") != FORMAL_COUNT or manifest.get("N_eval") != FORMAL_COUNT:
            raise ValueError("formal cohort declarations must be 720")
    return manifest


def _write_json(path: Path, value: Any) -> None:
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    candidate = Path(temporary)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True, ensure_ascii=True)
            stream.write("\n"); stream.flush(); os.fsync(stream.fileno())
        candidate.replace(path)
    except BaseException:
        candidate.unlink(missing_ok=True); raise


def verify_artifacts(root: Path) -> None:
    manifest = _read_json(root / "artifact_manifest.json")
    files = manifest.get("files") if isinstance(manifest, dict) else None
    if not isinstance(files, dict) or not files:
        raise ValueError("artifact manifest has no files")
    for name, expected in files.items():
        path = regular_file(root / str(name), label="artifact")
        if path.stat().st_size != expected.get("bytes") or sha256_file(path) != expected.get("sha256"):
            raise ValueError(f"artifact drift: {name}")
    manifest_path = root / "manifest.json"
    if manifest_path.is_file():
        value = _read_json(manifest_path)
        if not isinstance(value, dict) or value.get("manifest_content_sha256") != manifest_self_hash(value):
            raise ValueError("artifact manifest self-hash drift")
        if "assets" in value:
            verify_cohort_manifest(manifest_path, formal=False)


@contextmanager
def output_lock(output: Path):
    output = output.absolute(); output.parent.mkdir(parents=True, exist_ok=True)
    lock = output.parent / f".{output.name}.lock"
    with lock.open("a+", encoding="utf-8") as handle:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError(f"output is locked: {output}") from error
        try:
            if output.exists() or output.is_symlink():
                raise RuntimeError(f"refusing to overwrite existing output: {output}")
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def publish_staged(staging: Path, output: Path) -> None:
    """Publish a verified real directory without replacing any existing path."""

    verify_artifacts(staging)
    rename_noreplace(staging, output)


def reserve_output(target: Path) -> tuple[int, int]:
    """Reserve a destination with CPFS-safe mkdir semantics and return its inode."""

    target = target.absolute()
    try:
        target.mkdir()
    except FileExistsError as error:
        raise RuntimeError(f"refusing to overwrite existing output: {target}") from error
    try:
        stat_result = target.stat()
    except OSError:
        shutil.rmtree(target, ignore_errors=True)
        raise
    return stat_result.st_dev, stat_result.st_ino


def _guard_reservation(target: Path, reservation: tuple[int, int]) -> None:
    try:
        current = target.stat()
    except OSError as error:
        raise RuntimeError(f"output reservation disappeared: {target}") from error
    if (current.st_dev, current.st_ino) != reservation:
        raise RuntimeError(f"output reservation changed: {target}")


def rename_noreplace(source: Path, target: Path) -> None:
    """CPFS-compatible no-overwrite directory publication with inode guards."""

    source = source.absolute()
    target = target.absolute()
    if not source.is_dir() or source.is_symlink():
        raise ValueError(f"staging source is not a real directory: {source}")
    reservation = reserve_output(target)
    try:
        for child in sorted(source.iterdir(), key=lambda item: item.name):
            _guard_reservation(target, reservation)
            destination = target / child.name
            if destination.exists() or destination.is_symlink():
                raise RuntimeError(f"refusing to overwrite staged artifact: {destination}")
            child.rename(destination)
        _guard_reservation(target, reservation)
        source.rmdir()
    except BaseException as error:
        try:
            _guard_reservation(target, reservation)
        except Exception as guard_error:
            raise RuntimeError(f"output reservation changed: {target}") from guard_error
        shutil.rmtree(target, ignore_errors=True)
        raise


def publish_cohort(*, repo_root: Path, primary_root: Path, recovery_root: Path, output: Path, formal: bool) -> dict[str, Any]:
    with output_lock(output):
        source_manifest = _read_json(primary_root / "manifest.json")
        factories = [str(item) for item in source_manifest["factories"]]
        seeds = [int(item) for item in source_manifest["protocol"]["seeds"]]
        rows = build_cohort_rows(repo_root, primary_root, recovery_root)
        if formal:
            _validate_formal_rows(rows, factories, seeds)
        bindings = [
            _source_binding(primary_root / "manifest.json", "primary_manifest"),
            _source_binding(primary_root / "records.json", "primary_records"),
            _source_binding(recovery_root / "recovery_manifest.json", "recovery_manifest"),
            _source_binding(recovery_root / "recovery_records.json", "recovery_records"),
            _source_binding(SCRIPT, "cohort_freezer"),
            _source_binding(SCRIPT.parent / "prepare_infinite_mobility_table123_cohort.py", "cohort_preparer"),
        ]
        evaluation = {
            "freezer_path": str(SCRIPT), "freezer_sha256": sha256_file(SCRIPT),
            "preparer_path": str(SCRIPT.parent / "prepare_infinite_mobility_table123_cohort.py"),
            "preparer_sha256": sha256_file(SCRIPT.parent / "prepare_infinite_mobility_table123_cohort.py"),
            "table1_runner_path": str(SCRIPT.parent / "run_table1_infinite_mobility.py"),
            "table1_runner_sha256": sha256_file(SCRIPT.parent / "run_table1_infinite_mobility.py"),
            "table1_evaluator_path": str(SCRIPT.parent / "run_table1_artiverse.py"),
            "table1_evaluator_sha256": sha256_file(SCRIPT.parent / "run_table1_artiverse.py"),
            "protocol_path": str(PROTOCOL_PATH), "protocol_sha256": sha256_file(regular_file(PROTOCOL_PATH, label="evaluation protocol")),
        }
        manifest = cohort_manifest(rows, factory_order=factories, seeds=seeds, source_bindings=bindings)
        manifest["source"] = {"bindings": bindings}
        manifest["evaluation"] = evaluation
        manifest["manifest_content_sha256"] = manifest_self_hash(manifest)
        staging = output.parent / f".{output.name}.staging.{uuid.uuid4().hex}"; staging.mkdir()
        try:
            protocol = {"primary_protocol": source_manifest.get("protocol"), "factory_order": factories, "seeds": seeds, "source_bindings": bindings, "evaluation": evaluation, "identity_policy": manifest["source_selection"]["identity_policy"]}
            selection = manifest["source_selection"]
            _write_json(staging / "manifest.json", manifest)
            _write_json(staging / "cohort_protocol_snapshot.json", protocol)
            _write_json(staging / "source_selection.json", selection)
            names = ("manifest.json", "cohort_protocol_snapshot.json", "source_selection.json")
            _write_json(staging / "artifact_manifest.json", {"schema_version": 1, "files": {name: {"bytes": (staging / name).stat().st_size, "sha256": sha256_file(staging / name)} for name in names}})
            publish_staged(staging, output)
        except BaseException:
            shutil.rmtree(staging, ignore_errors=True); raise
    return manifest


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primary-root", type=Path, default=DEFAULT_PRIMARY); parser.add_argument("--recovery-root", type=Path, default=DEFAULT_RECOVERY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT); parser.add_argument("--formal", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    manifest = publish_cohort(repo_root=REPO_ROOT, primary_root=args.primary_root, recovery_root=args.recovery_root, output=args.output, formal=args.formal)
    print(json.dumps({"state": "COMPLETE", "N_eval": manifest["N_eval"], "output": str(args.output.absolute())}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
