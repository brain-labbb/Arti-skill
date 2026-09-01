#!/usr/bin/env python3
"""Status-aware, read-only reaggregation for the sealed PV-A Table 4 v3 run.

The v3 evaluator deliberately keeps one strict, full-roster denominator.  That
number is useful for a release claim, but it hides whether a failure came from
the asset, the collision representation, or the execution backend.  This
module reads the sealed SQLite result database and publishes a *derived*
diagnostic report with those dimensions separated.  It never edits the v3
directory and never treats a missing measurement as a successful measurement.

Only the database is used as the source of metric values.  The manifest,
receipt, checkpoint, and (small) summary/artifact bindings are checked before
any rows are consumed.  Large JSONL artifact hashes can be enabled with
``--verify-large-artifacts`` when a complete byte-level audit is desired.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import stat
import sys
from typing import Any, Iterable, Mapping
import zlib


SCRIPT = Path(__file__).resolve()

RUN_SCHEMA = "pva_table4_kinematic_aware_full_release_run_v3"
RECEIPT_SCHEMA = "pva_table4_kinematic_aware_full_release_receipt_v1"
CHECKPOINT_SCHEMA = "pva_table4_kinematic_aware_checkpoint_v1"
ARTIFACT_SCHEMA = "pva_table4_kinematic_aware_artifacts_v1"
RESULT_DB_SCHEMA = "pva_table4_kinematic_aware_results_db_v1"
PROTOCOL_ID = "urdf_sim_ready_table4_pva_full_release_v3"
SINGLE_SAMPLES = 21

PHASES = ("rest", "single", "sobol")
PASS_KEYS = (
    "rest_all_pair_cf",
    "rest_non_adjacent_cf",
    "single_joint_sweep_cf",
    "multi_joint_sobol_cf",
    "strict_collision_pass",
)
DOF_BINS = ("0", "1", "2-3", "4-7", ">=8")
DECLARED_DOF_BINS = (
    "0",
    "1",
    "2-3",
    "4-7",
    "8-15",
    "16-31",
    "32-63",
    "64-127",
    ">=128",
)
CAPACITY_BINS = ("<128_links", ">=128_links", "unknown_links")

# The first band includes the evaluator's strict threshold.  Subsequent bands
# are deliberately expressed in metres so that the report remains machine
# readable; the labels expose the corresponding millimetre scale to readers.
SEVERITY_BANDS = (
    ("<=0.001mm", None, 1.0e-6),
    ("0.001-0.1mm", 1.0e-6, 1.0e-4),
    ("0.1-1mm", 1.0e-4, 1.0e-3),
    ("1-10mm", 1.0e-3, 1.0e-2),
    (">10mm", 1.0e-2, None),
)


class ReaggregateError(ValueError):
    """Raised when a sealed v3 result cannot be independently reaggregated."""


def _canonical_text(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def _canonical_sha256(value: Any) -> str:
    return hashlib.sha256(_canonical_text(value).encode("utf-8")).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise ReaggregateError(f"cannot hash {path}: {exc}") from exc
    return digest.hexdigest()


def _require_regular(path: Path, label: str) -> Path:
    try:
        info = path.lstat()
    except OSError as exc:
        raise ReaggregateError(f"{label} is unavailable: {path}") from exc
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise ReaggregateError(f"{label} is not a regular non-symlink file: {path}")
    return path


def _load_object(path: Path, label: str) -> dict[str, Any]:
    _require_regular(path, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReaggregateError(f"cannot read {label} {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ReaggregateError(f"{label} is not a JSON object: {path}")
    return value


def _self_hash(value: Mapping[str, Any], field: str) -> str:
    payload = dict(value)
    payload.pop(field, None)
    return _canonical_sha256(payload)


def _check_self_hash(value: Mapping[str, Any], field: str, label: str) -> None:
    observed = value.get(field)
    if observed != _self_hash(value, field):
        raise ReaggregateError(f"{label} self-hash mismatch")


def _hex_digest(value: Any, label: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ReaggregateError(f"{label} is not a lowercase SHA-256 digest")
    return value


def _relative_artifact(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ReaggregateError(f"{label} path is missing")
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise ReaggregateError(f"{label} path is unsafe: {value}")
    path = root / relative
    _require_regular(path, label)
    return path


def _sidecar_snapshot(database: Path) -> dict[str, tuple[int, int, int, int] | None]:
    snapshot: dict[str, tuple[int, int, int, int] | None] = {}
    for suffix in ("-wal", "-journal", "-shm"):
        sidecar = Path(f"{database}{suffix}")
        try:
            info = sidecar.lstat()
        except FileNotFoundError:
            snapshot[suffix] = None
            continue
        except OSError as exc:
            raise ReaggregateError(f"cannot inspect SQLite sidecar {sidecar}") from exc
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
            raise ReaggregateError(f"unsafe SQLite sidecar: {sidecar}")
        if suffix in ("-wal", "-journal") and info.st_size != 0:
            raise ReaggregateError(
                f"sealed SQLite {suffix[1:]} is non-empty: {sidecar}"
            )
        snapshot[suffix] = (
            int(info.st_dev),
            int(info.st_ino),
            int(info.st_size),
            int(info.st_mtime_ns),
        )
    return snapshot


def _file_identity(path: Path) -> tuple[int, int, int, int]:
    info = path.lstat()
    return (
        int(info.st_dev),
        int(info.st_ino),
        int(info.st_size),
        int(info.st_mtime_ns),
    )


def _fraction(numerator: int, denominator: int) -> dict[str, Any]:
    numerator = int(numerator)
    denominator = int(denominator)
    return {
        "passed": numerator,
        "denominator": denominator,
        "rate": (numerator / denominator if denominator else None),
    }


def _state_layer(
    expected: Mapping[str, int],
    executed: Mapping[str, int],
    free: Mapping[str, int],
) -> dict[str, Any]:
    """Summarize one explicit state-coverage layer.

    ``free_intent`` uses every planned state (fail-closed publication), while
    ``free_executed`` conditions on states that reached the oracle. Keeping
    both values prevents backend omissions from being confused with collision
    outcomes.
    """

    phases: dict[str, dict[str, Any]] = {}
    for phase in PHASES:
        planned = int(expected.get(phase, 0))
        ran = int(executed.get(phase, 0))
        clear = int(free.get(phase, 0))
        if planned < 0 or ran < 0 or clear < 0 or ran > planned or clear > ran:
            raise ReaggregateError(
                f"invalid state layer accounting for {phase}: "
                f"expected={planned}, executed={ran}, free={clear}"
            )
        phases[phase] = {
            "expected": planned,
            "executed": ran,
            "free": clear,
            "unexecuted": planned - ran,
            "free_intent": _fraction(clear, planned),
            "free_executed": _fraction(clear, ran),
            "collision_state_intent": _fraction(planned - clear, planned),
            "collision_state_executed": _fraction(ran - clear, ran),
            "coverage": _fraction(ran, planned),
        }

    def combine(names: Iterable[str]) -> dict[str, Any]:
        phase_names = tuple(names)
        planned = sum(phases[name]["expected"] for name in phase_names)
        ran = sum(phases[name]["executed"] for name in phase_names)
        clear = sum(phases[name]["free"] for name in phase_names)
        return {
            "expected": planned,
            "executed": ran,
            "free": clear,
            "unexecuted": planned - ran,
            "free_intent": _fraction(clear, planned),
            "free_executed": _fraction(clear, ran),
            "collision_state_intent": _fraction(planned - clear, planned),
            "collision_state_executed": _fraction(ran - clear, ran),
            "coverage": _fraction(ran, planned),
        }

    return {
        "expected": {phase: phases[phase]["expected"] for phase in PHASES},
        "executed": {phase: phases[phase]["executed"] for phase in PHASES},
        "free": {phase: phases[phase]["free"] for phase in PHASES},
        "unexecuted": {phase: phases[phase]["unexecuted"] for phase in PHASES},
        "phases": phases,
        "all_states": combine(PHASES),
        "all_motion": combine(("single", "sobol")),
    }


def _motion_layer_view(layer: Mapping[str, Any]) -> dict[str, Any]:
    """Expose motion phases with human-readable names plus an all-motion row."""

    return {
        "single_joint_sweep": layer["phases"]["single"],
        "joint_space_sobol": layer["phases"]["sobol"],
        "all_motion": layer["all_motion"],
    }


def _triplet(
    numerator: int,
    *,
    release_denominator: int,
    observed_denominator: int,
    measured_denominator: int,
    state_observed_denominator: int | None = None,
) -> dict[str, Any]:
    """Report one numerator against explicit release/coverage layers.

    ``observed`` is retained as the complete-state layer for compatibility;
    ``state_observed`` is the weaker ``state_count > 0`` layer and is useful
    for exposing partial assets without allowing them to pass a metric.
    """

    result = {
        "release": _fraction(numerator, release_denominator),
        "observed": _fraction(numerator, observed_denominator),
        "collision_measured": _fraction(numerator, measured_denominator),
    }
    if state_observed_denominator is not None:
        result["state_observed"] = _fraction(numerator, state_observed_denominator)
    return result


def _dof_bin(value: Any) -> str:
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise ReaggregateError(f"invalid independent DoF count: {value!r}") from exc
    if number < 0:
        raise ReaggregateError(f"negative independent DoF count: {number}")
    if number == 0:
        return "0"
    if number == 1:
        return "1"
    if number <= 3:
        return "2-3"
    if number <= 7:
        return "4-7"
    return ">=8"


def _capacity_bin(value: Any, ordinal: int) -> str:
    """Classify the raw-link-count tail implicated by the PyBullet API cap."""

    if value is None:
        return "unknown_links"

    try:
        links = int(value)
    except (TypeError, ValueError) as exc:
        raise ReaggregateError(
            f"asset {ordinal} link_count is not an integer: {value!r}"
        ) from exc
    if links < 0:
        raise ReaggregateError(f"asset {ordinal} link_count is negative: {links}")
    return ">=128_links" if links >= 128 else "<128_links"


def _declared_dof_bin(value: Any, ordinal: int) -> str:
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise ReaggregateError(
            f"asset {ordinal} declared movable joint count is not an integer: {value!r}"
        ) from exc
    if number < 0:
        raise ReaggregateError(
            f"asset {ordinal} declared movable joint count is negative: {number}"
        )
    if number == 0:
        return "0"
    if number == 1:
        return "1"
    if number <= 3:
        return "2-3"
    if number <= 7:
        return "4-7"
    if number <= 15:
        return "8-15"
    if number <= 31:
        return "16-31"
    if number <= 63:
        return "32-63"
    if number <= 127:
        return "64-127"
    return ">=128"


def _as_nonnegative_int(record: Mapping[str, Any], field: str, ordinal: int) -> int:
    value = record.get(field, 0)
    if isinstance(value, bool):
        raise ReaggregateError(f"asset {ordinal} {field} is boolean")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ReaggregateError(f"asset {ordinal} {field} is not an integer") from exc
    if parsed < 0:
        raise ReaggregateError(f"asset {ordinal} {field} is negative")
    return parsed


def _as_nonnegative_float(value: Any, label: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ReaggregateError(f"{label} is not numeric") from exc
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ReaggregateError(f"{label} is not finite/non-negative")
    return parsed


def _state_nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ReaggregateError(f"{label} is not a non-negative integer")
    return int(value)


def _bool_value(record: Mapping[str, Any], field: str, ordinal: int) -> bool:
    value = record.get(field)
    if not isinstance(value, bool):
        raise ReaggregateError(f"asset {ordinal} {field} is not boolean")
    return value


def _phase_key(phase: Any, ordinal: int) -> str:
    if phase == "rest":
        return "rest"
    if phase == "single_joint_sweep":
        return "single"
    if phase == "multi_joint_sobol":
        return "sobol"
    raise ReaggregateError(f"asset {ordinal} has unknown state phase: {phase!r}")


def _severity_band(value_m: float) -> str:
    for label, lower, upper in SEVERITY_BANDS:
        if lower is not None and value_m <= lower:
            continue
        if upper is None or value_m <= upper:
            return label
    # The first band has no lower bound and catches zero exactly.
    return SEVERITY_BANDS[0][0]


def _quantile(values: list[float], probability: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    weight = position - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def _quantiles(values: list[float]) -> dict[str, float | None]:
    return {
        name: _quantile(values, probability)
        for name, probability in (
            ("p50", 0.50),
            ("p90", 0.90),
            ("p95", 0.95),
            ("p99", 0.99),
            ("p99_9", 0.999),
            ("max", 1.0),
        )
    }


def _asset_rate_quantiles(values: list[float]) -> dict[str, float | None]:
    """Expose the lower tail that distinguishes mostly collision-free assets."""

    return {
        name: _quantile(values, probability)
        for name, probability in (
            ("min", 0.0),
            ("p1", 0.01),
            ("p5", 0.05),
            ("p10", 0.10),
            ("p25", 0.25),
            ("p50", 0.50),
            ("p75", 0.75),
            ("p90", 0.90),
            ("p95", 0.95),
            ("p99", 0.99),
            ("p99_9", 0.999),
            ("max", 1.0),
        )
    }


class _StreamingSeverity:
    """Bounded-memory state severity accumulator.

    Asset maxima are only ~302k values and are retained exactly for quantiles.
    State rows can exceed 48 million, so their bands are counted online and a
    deterministic reservoir is retained only for the optional raw-vs-
    calibrated diagnostic quantiles.
    """

    _RESERVOIR_LIMIT = 100_000

    def __init__(self, *, bands: bool = True) -> None:
        self.count = 0
        self.band_counts: Counter[str] = Counter()
        self.values: list[float] = []
        self.bands = bands
        self._rng = 0x9E3779B9
        self.positive = 0
        self.negative = 0
        self.zero = 0

    def add(self, value: float, *, band_value: float | None = None) -> None:
        value = float(value)
        self.count += 1
        if self.bands and band_value is not None:
            self.band_counts[_severity_band(float(band_value))] += 1
        if value > 0.0:
            self.positive += 1
        elif value < 0.0:
            self.negative += 1
        else:
            self.zero += 1
        if len(self.values) < self._RESERVOIR_LIMIT:
            self.values.append(value)
        else:
            # Deterministic reservoir sampling keeps the report reproducible.
            self._rng = (1664525 * self._rng + 1013904223) & 0xFFFFFFFF
            slot = self._rng % self.count
            if slot < self._RESERVOIR_LIMIT:
                self.values[slot] = value

    def band_report(self) -> dict[str, dict[str, Any]]:
        result: dict[str, dict[str, Any]] = {}
        for label, lower, upper in SEVERITY_BANDS:
            count = int(self.band_counts[label])
            result[label] = {
                "lower_exclusive_m": lower,
                "upper_inclusive_m": upper,
                "count": count,
                "denominator": int(self.count),
                "rate": count / self.count if self.count else None,
            }
        return result

    def merge(self, other: "_StreamingSeverity") -> None:
        """Merge counters and sampled values without retaining all states."""

        self.count += other.count
        self.positive += other.positive
        self.negative += other.negative
        self.zero += other.zero
        self.band_counts.update(other.band_counts)
        for value in other.values:
            self.add(value)


def _band_counts(values: Iterable[float]) -> dict[str, dict[str, Any]]:
    counts = Counter(_severity_band(float(value)) for value in values)
    total = sum(counts.values())
    result: dict[str, dict[str, Any]] = {}
    for label, lower, upper in SEVERITY_BANDS:
        result[label] = {
            "lower_exclusive_m": lower,
            "upper_inclusive_m": upper,
            "count": int(counts[label]),
            "denominator": int(total),
            "rate": (counts[label] / total if total else None),
        }
    return result


def _classify_error(
    record: Mapping[str, Any],
    worker_status: str,
    ordinal: int,
    *,
    derived_complete: bool | None = None,
) -> str | None:
    status = str(record.get("status", ""))
    issues = record.get("issues")
    if not isinstance(issues, list):
        issues = []
    text = " ".join(str(item) for item in issues).lower()
    complete = (
        bool(record.get("measurement_complete"))
        if derived_complete is None
        else bool(derived_complete)
    )
    if status == "completed" and complete:
        return None
    if "package binding drift" in text or "package_error" in worker_status.lower():
        return "package_binding_drift"
    if "reset/readback error" in text or "reset readback error" in text:
        return "reset_readback_partial"
    if any(
        token in text
        for token in (
            "getbasepositionandorientation",
            "getjointstate",
            "reset readback",
            "backend capacity",
            "capacity",
        )
    ):
        return "backend_capacity"
    if "source_integrity" in text or "source hash" in text or "source drift" in text:
        return "source_integrity"
    if "timeout" in text or status == "timeout" or worker_status == "timeout":
        return "timeout"
    if any(
        token in text
        for token in (
            "sampling",
            "mimic",
            "joint plan",
            "range_evaluable",
        )
    ):
        return "sampling_plan"
    if status == "error" or worker_status not in {"completed", ""}:
        return "other_error"
    return "measurement_incomplete"


def _validate_artifact_manifest(
    root: Path,
    artifact: Mapping[str, Any],
    *,
    verify_large: bool,
) -> dict[str, Any]:
    if artifact.get("schema_version") != ARTIFACT_SCHEMA:
        raise ReaggregateError("artifact manifest schema mismatch")
    _check_self_hash(artifact, "artifact_manifest_content_sha256", "artifact manifest")
    rows = artifact.get("artifacts")
    if not isinstance(rows, list):
        raise ReaggregateError("artifact manifest has no artifacts list")
    checked = 0
    skipped_large = 0
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping):
            raise ReaggregateError(f"artifact manifest row {index} is invalid")
        path = _relative_artifact(root, row.get("path"), f"artifact row {index}")
        expected_bytes = row.get("bytes", row.get("size"))
        if isinstance(expected_bytes, bool) or not isinstance(expected_bytes, int):
            raise ReaggregateError(f"artifact row {index} byte count is invalid")
        if path.stat().st_size != expected_bytes:
            raise ReaggregateError(f"artifact row {index} byte count mismatch")
        expected_hash = _hex_digest(row.get("sha256"), f"artifact row {index} hash")
        if verify_large or path.stat().st_size <= 16 * 1024 * 1024:
            if _sha256_file(path) != expected_hash:
                raise ReaggregateError(f"artifact row {index} hash mismatch")
            checked += 1
        else:
            skipped_large += 1
    return {
        "rows": len(rows),
        "hashes_checked": checked,
        "large_hashes_skipped": skipped_large,
    }


def _validate_sealed_input(
    output: Path,
    *,
    verify_large_artifacts: bool,
) -> tuple[
    dict[str, Any], dict[str, Any], dict[str, Any], sqlite3.Connection, dict[str, Any]
]:
    try:
        output = output.resolve(strict=True)
    except OSError as exc:
        raise ReaggregateError(
            f"sealed output directory is unavailable: {output}"
        ) from exc
    if not output.is_dir():
        raise ReaggregateError(f"sealed output is not a directory: {output}")

    manifest_path = output / "manifest.json"
    receipt_path = output / "full_release_receipt.json"
    checkpoint_path = output / "checkpoint.json"
    summary_path = output / "summary.json"
    artifact_path = output / "artifact_manifest.json"
    manifest = _load_object(manifest_path, "manifest")
    receipt = _load_object(receipt_path, "receipt")
    checkpoint = _load_object(checkpoint_path, "checkpoint")
    _check_self_hash(manifest, "manifest_content_sha256", "manifest")
    _check_self_hash(receipt, "receipt_content_sha256", "receipt")
    _check_self_hash(checkpoint, "checkpoint_content_sha256", "checkpoint")
    if manifest.get("schema_version") != RUN_SCHEMA:
        raise ReaggregateError("manifest schema mismatch")
    if receipt.get("schema_version") != RECEIPT_SCHEMA:
        raise ReaggregateError("receipt schema mismatch")
    if checkpoint.get("schema_version") != CHECKPOINT_SCHEMA:
        raise ReaggregateError("checkpoint schema mismatch")
    for label, value in (
        ("manifest", manifest),
        ("receipt", receipt),
        ("checkpoint", checkpoint),
    ):
        if value.get("protocol_id") != PROTOCOL_ID:
            raise ReaggregateError(f"{label} protocol mismatch")

    n_eval = int(manifest.get("N_eval", -1))
    j_eval = int(manifest.get("J_eval", -1))
    if n_eval <= 0 or j_eval < 0:
        raise ReaggregateError("manifest N_eval/J_eval are invalid")
    if (
        int(receipt.get("N_eval", -1)) != n_eval
        or int(receipt.get("J_eval", -1)) != j_eval
    ):
        raise ReaggregateError("receipt N_eval/J_eval mismatch")
    if receipt.get("manifest") != "manifest.json":
        raise ReaggregateError("receipt manifest path is not canonical")
    if receipt.get("manifest_sha256") != _sha256_file(manifest_path):
        raise ReaggregateError("receipt manifest hash mismatch")
    if checkpoint.get("manifest_content_sha256") != manifest.get(
        "manifest_content_sha256"
    ):
        raise ReaggregateError("checkpoint manifest binding mismatch")
    if (
        checkpoint.get("state") != "complete"
        or int(checkpoint.get("records", -1)) != n_eval
    ):
        raise ReaggregateError("checkpoint is not complete for the sealed roster")

    # Summary is small and is always checked when present.  The large JSONL
    # artifacts are checked only under the explicit opt-in flag.
    if summary_path.exists():
        summary = _load_object(summary_path, "summary")
        _check_self_hash(summary, "summary_content_sha256", "summary")
        if receipt.get("summary") != "summary.json":
            raise ReaggregateError("receipt summary path is not canonical")
        if receipt.get("summary_sha256") != _sha256_file(summary_path):
            raise ReaggregateError("receipt summary hash mismatch")
    else:
        raise ReaggregateError("sealed summary.json is missing")

    if not artifact_path.exists():
        raise ReaggregateError("sealed artifact_manifest.json is missing")
    artifact = _load_object(artifact_path, "artifact manifest")
    artifact_audit = _validate_artifact_manifest(
        output, artifact, verify_large=verify_large_artifacts
    )
    if receipt.get("artifact_manifest") != "artifact_manifest.json":
        raise ReaggregateError("receipt artifact manifest path is not canonical")
    if receipt.get("artifact_manifest_sha256") != _sha256_file(artifact_path):
        raise ReaggregateError("receipt artifact manifest hash mismatch")

    database_path = _relative_artifact(
        output, receipt.get("result_database"), "result database"
    )
    expected_db_hash = _hex_digest(
        receipt.get("result_database_sha256"), "receipt database hash"
    )
    before_identity = _file_identity(database_path)
    before_sidecars = _sidecar_snapshot(database_path)
    if _sha256_file(database_path) != expected_db_hash:
        raise ReaggregateError("sealed result database SHA-256 mismatch")
    connection = sqlite3.connect(
        f"{database_path.as_uri()}?mode=ro&immutable=1",
        uri=True,
    )
    try:
        connection.execute("PRAGMA query_only=ON")
        if int(connection.execute("PRAGMA query_only").fetchone()[0]) != 1:
            raise ReaggregateError("SQLite query_only could not be enabled")
        meta = {
            str(key): json.loads(value)
            for key, value in connection.execute("SELECT key,value FROM meta")
        }
        if meta.get("schema_version") != RESULT_DB_SCHEMA:
            raise ReaggregateError("result database schema mismatch")
        if meta.get("protocol_id") != PROTOCOL_ID:
            raise ReaggregateError("result database protocol mismatch")
        if (
            int(meta.get("N_eval", -1)) != n_eval
            or int(meta.get("J_eval", -1)) != j_eval
        ):
            raise ReaggregateError("result database N_eval/J_eval mismatch")
        if meta.get("manifest_content_sha256") != manifest.get(
            "manifest_content_sha256"
        ):
            raise ReaggregateError("result database manifest binding mismatch")
    except BaseException:
        connection.close()
        raise
    after_identity = _file_identity(database_path)
    after_sidecars = _sidecar_snapshot(database_path)
    # SQLite may create/update the shared-memory sidecar while an immutable
    # read-only connection is opened.  A non-empty WAL/journal is still fatal;
    # the database inode/size/mtime and those two sidecars must remain fixed.
    if before_identity != after_identity or any(
        before_sidecars[suffix] != after_sidecars[suffix]
        for suffix in ("-wal", "-journal")
    ):
        connection.close()
        raise ReaggregateError("sealed database changed during validation")
    bindings = {
        "manifest_sha256": _sha256_file(manifest_path),
        "manifest_content_sha256": manifest["manifest_content_sha256"],
        "receipt_sha256": _sha256_file(receipt_path),
        "checkpoint_sha256": _sha256_file(checkpoint_path),
        "database_sha256": expected_db_hash,
        "artifact_manifest_sha256": _sha256_file(artifact_path),
        "artifact_audit": artifact_audit,
        "summary": summary,
        "n_eval": n_eval,
        "j_eval": j_eval,
    }
    return manifest, receipt, checkpoint, connection, bindings


def _empty_bin() -> dict[str, Any]:
    return {
        "assets": 0,
        "state_observed_assets": 0,
        "observed_assets": 0,
        "input_bound_assets": 0,
        "load_success_assets": 0,
        "geometry_present_assets": 0,
        "collision_geometry_assets": 0,
        "collision_measured_assets": 0,
        "rest_all_pair_pass": 0,
        "strict_collision_pass": 0,
        "rest_non_adjacent_pass": 0,
        "single_joint_sweep_pass": 0,
        "multi_joint_sobol_pass": 0,
        "rest_adjacent_only": 0,
        "rest_pass_motion_fail": 0,
        "independent_dofs": 0,
        "range_evaluable_independent_dofs": 0,
        "state_observed_independent_dofs": 0,
        "observed_independent_dofs": 0,
        "collision_measured_independent_dofs": 0,
        "safe_dof_passed": 0,
        "safe_dof_observed": 0,
        "expected_states": Counter(),
        "executed_states": Counter(),
        "free_states": Counter(),
    }


def _new_category() -> dict[str, Any]:
    return {
        "assets": 0,
        "state_observed_assets": 0,
        "observed_assets": 0,
        "input_bound_assets": 0,
        "load_success_assets": 0,
        "geometry_present_assets": 0,
        "collision_geometry_assets": 0,
        "collision_measured_assets": 0,
        "strict_collision_pass": 0,
        "rest_all_pair_pass": 0,
        "rest_non_adjacent_pass": 0,
        "single_joint_sweep_pass": 0,
        "multi_joint_sobol_pass": 0,
        "rest_adjacent_only": 0,
        "rest_pass_motion_fail": 0,
    }


def _record_state_metrics(
    record: Mapping[str, Any],
    blob: bytes,
    state_count: int,
    ordinal: int,
    *,
    state_severity: _StreamingSeverity | None,
    state_raw_delta: _StreamingSeverity | None,
) -> dict[str, Any]:
    """Parse one compressed state group and return recomputed phase facts."""

    try:
        payload = zlib.decompress(blob)
    except zlib.error as exc:
        raise ReaggregateError(f"asset {ordinal} state decompression failed") from exc
    if payload and not payload.endswith(b"\n"):
        raise ReaggregateError(f"asset {ordinal} state payload lacks final newline")
    lines = payload.splitlines()
    if len(lines) != state_count:
        raise ReaggregateError(
            f"asset {ordinal} state count mismatch: {len(lines)} != {state_count}"
        )
    declared_state_count = record.get("state_records_count")
    if declared_state_count is not None:
        if (
            isinstance(declared_state_count, bool)
            or int(declared_state_count) != state_count
        ):
            raise ReaggregateError(
                f"asset {ordinal} record/state_records_count mismatch: "
                f"{declared_state_count!r} != {state_count}"
            )
    state_digest = hashlib.sha256()
    state_digest.update(b"[")
    for index, line in enumerate(lines):
        if index:
            state_digest.update(b",")
        state_digest.update(line)
    state_digest.update(b"]")
    declared_state_hash = record.get("state_records_sha256")
    if declared_state_hash != state_digest.hexdigest():
        raise ReaggregateError(f"asset {ordinal} state-record group hash mismatch")
    phases: dict[str, int] = Counter()
    free: dict[str, int] = Counter()
    all_pair_free = True
    all_pair_seen = False
    single_groups: dict[str, dict[int, bool]] = defaultdict(dict)
    sobol_samples: set[int] = set()
    max_metric: float | None = None
    for state_index, line in enumerate(lines):
        try:
            state = json.loads(line)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ReaggregateError(
                f"asset {ordinal} state {state_index} is invalid JSON"
            ) from exc
        if not isinstance(state, dict):
            raise ReaggregateError(
                f"asset {ordinal} state {state_index} is not an object"
            )
        if state.get("protocol_id") != PROTOCOL_ID or state.get(
            "dataset_id"
        ) != record.get("dataset_id"):
            raise ReaggregateError(
                f"asset {ordinal} state {state_index} identity mismatch"
            )
        if state.get("schema_version") != "table4_state_v3":
            raise ReaggregateError(
                f"asset {ordinal} state {state_index} schema mismatch"
            )
        if state.get("joint_sampling_plan_sha256") != record.get(
            "joint_sampling_plan_sha256"
        ):
            raise ReaggregateError(
                f"asset {ordinal} state {state_index} sampling-plan mismatch"
            )
        if int(state.get("order", -1)) != ordinal:
            raise ReaggregateError(
                f"asset {ordinal} state {state_index} order mismatch"
            )
        phase = _phase_key(state.get("phase"), ordinal)
        phases[phase] += 1
        all_illegal = _state_nonnegative_int(
            state.get("all_pair_illegal_penetration_count"),
            f"asset {ordinal} state {state_index} all-pair illegal",
        )
        all_contacts = _state_nonnegative_int(
            state.get("all_pair_contact_count"),
            f"asset {ordinal} state {state_index} all-pair contacts",
        )
        non_illegal = _state_nonnegative_int(
            state.get("non_adjacent_illegal_penetration_count"),
            f"asset {ordinal} state {state_index} non-adjacent illegal",
        )
        non_contacts = _state_nonnegative_int(
            state.get("non_adjacent_contact_count"),
            f"asset {ordinal} state {state_index} non-adjacent contacts",
        )
        if all_illegal > all_contacts or non_illegal > non_contacts:
            raise ReaggregateError(
                f"asset {ordinal} state {state_index} contact counters invalid"
            )
        metric = _as_nonnegative_float(
            state.get("metric_max_penetration_m"),
            f"asset {ordinal} state {state_index} metric penetration",
        )
        expected_metric = (
            _as_nonnegative_float(
                state.get("all_pair_max_penetration_m"),
                f"asset {ordinal} state {state_index} all-pair maximum",
            )
            if phase == "rest"
            else _as_nonnegative_float(
                state.get("non_adjacent_max_penetration_m"),
                f"asset {ordinal} state {state_index} non-adjacent maximum",
            )
        )
        if not math.isclose(metric, expected_metric, rel_tol=0.0, abs_tol=1e-15):
            raise ReaggregateError(
                f"asset {ordinal} state {state_index} metric policy mismatch"
            )
        raw_metric = (
            _as_nonnegative_float(
                state.get("raw_all_pair_max_penetration_m"),
                f"asset {ordinal} state {state_index} raw all-pair maximum",
            )
            if phase == "rest"
            else _as_nonnegative_float(
                state.get("raw_non_adjacent_max_penetration_m"),
                f"asset {ordinal} state {state_index} raw non-adjacent maximum",
            )
        )
        if state_severity is not None:
            state_severity.add(metric, band_value=metric)
        if state_raw_delta is not None:
            state_raw_delta.add(raw_metric - metric)
        free[phase] += int(non_illegal == 0)
        if phase == "rest":
            all_pair_seen = True
            all_pair_free = all_pair_free and all_illegal == 0
            if state.get("sample_index") != 0 or state.get("joint_name") not in (
                None,
                "",
            ):
                raise ReaggregateError(
                    f"asset {ordinal} state {state_index} rest coverage invalid"
                )
        elif phase == "single":
            name = state.get("joint_name")
            if not isinstance(name, str) or not name:
                raise ReaggregateError(
                    f"asset {ordinal} state {state_index} single joint name missing"
                )
            sample = state.get("sample_index")
            if (
                isinstance(sample, bool)
                or not isinstance(sample, int)
                or not 0 <= sample < SINGLE_SAMPLES
            ):
                raise ReaggregateError(
                    f"asset {ordinal} state {state_index} single sample invalid"
                )
            if sample in single_groups[name]:
                raise ReaggregateError(
                    f"asset {ordinal} duplicate single state {name}:{sample}"
                )
            single_groups[name][sample] = non_illegal == 0
        else:
            sample = state.get("sample_index")
            if (
                isinstance(sample, bool)
                or not isinstance(sample, int)
                or not 0 <= sample < 64
            ):
                raise ReaggregateError(
                    f"asset {ordinal} state {state_index} Sobol sample invalid"
                )
            if sample in sobol_samples or state.get("joint_name") not in (None, ""):
                raise ReaggregateError(f"asset {ordinal} duplicate/invalid Sobol state")
            sobol_samples.add(sample)
        if max_metric is None or metric > max_metric:
            max_metric = metric

    expected = {
        "rest": _as_nonnegative_int(record, "rest_state_expected", ordinal),
        "single": _as_nonnegative_int(record, "single_state_expected", ordinal),
        "sobol": _as_nonnegative_int(record, "sobol_state_expected", ordinal),
    }
    for phase in PHASES:
        if phases[phase] != _as_nonnegative_int(
            record, f"{phase}_state_executed", ordinal
        ):
            raise ReaggregateError(f"asset {ordinal} {phase} executed-state mismatch")
    range_dof = _as_nonnegative_int(
        record, "range_evaluable_independent_dof_count", ordinal
    )
    independent = _as_nonnegative_int(record, "independent_dof_count", ordinal)
    safe_dof_observed = 0
    safe_dof_passed = 0
    for samples in single_groups.values():
        if set(samples) == set(range(SINGLE_SAMPLES)):
            safe_dof_observed += 1
            safe_dof_passed += int(all(samples.values()))
    if safe_dof_observed > range_dof:
        raise ReaggregateError(f"asset {ordinal} observed more safe DoFs than declared")
    rest_all = all_pair_seen and phases["rest"] == expected["rest"] and all_pair_free
    rest_non = phases["rest"] == expected["rest"] and free["rest"] == expected["rest"]
    single_groups_complete = len(single_groups) == range_dof and all(
        set(samples) == set(range(SINGLE_SAMPLES)) for samples in single_groups.values()
    )
    single = (
        expected["single"] == 0
        or phases["single"] == expected["single"]
        and free["single"] == expected["single"]
        and single_groups_complete
    )
    sobol = (
        independent > 0
        and range_dof == independent
        and expected["sobol"] > 0
        and phases["sobol"] == expected["sobol"]
        and free["sobol"] == expected["sobol"]
    )
    complete = range_dof == independent and all(
        phases[phase] == expected[phase] for phase in PHASES
    )
    strict = bool(independent > 0 and complete and rest_non and single and sobol)
    phase_counts = {phase: int(phases.get(phase, 0)) for phase in PHASES}
    free_counts = {phase: int(free.get(phase, 0)) for phase in PHASES}
    return {
        "phases": phase_counts,
        "expected": expected,
        "free": free_counts,
        "rest_all_pair_pass": bool(rest_all),
        "rest_non_adjacent_pass": bool(rest_non),
        "single_joint_sweep_pass": bool(single),
        "multi_joint_sobol_pass": bool(sobol),
        "strict_collision_pass": strict,
        "measurement_complete": bool(complete),
        "single_groups": single_groups,
        "safe_dof_observed": safe_dof_observed,
        "safe_dof_passed": safe_dof_passed,
        "max_metric_m": max_metric,
    }


def _macro_rate(
    rows: list[dict[str, Any]], numerator_key: str, denominator_key: str
) -> dict[str, Any]:
    eligible = [row for row in rows if int(row[denominator_key]) > 0]
    values = [int(row[numerator_key]) / int(row[denominator_key]) for row in eligible]
    return {
        "mean_rate": (sum(values) / len(values) if values else None),
        "eligible_categories": len(eligible),
        "category_denominator": len(rows),
    }


def _aggregate(
    connection: sqlite3.Connection,
    *,
    n_eval: int,
    j_eval: int,
    progress_every: int = 0,
) -> dict[str, Any]:
    row_count = int(connection.execute("SELECT COUNT(*) FROM results").fetchone()[0])
    if row_count != n_eval:
        raise ReaggregateError(f"result row count mismatch: {row_count} != {n_eval}")

    status_counts: Counter[str] = Counter()
    worker_status_counts: Counter[str] = Counter()
    error_counts: Counter[str] = Counter()
    error_examples: dict[str, dict[str, Any]] = {}
    categories: dict[str, dict[str, Any]] = defaultdict(_new_category)
    bins: dict[str, dict[str, Any]] = {name: _empty_bin() for name in DOF_BINS}
    declared_bins: dict[str, dict[str, Any]] = {
        name: _empty_bin() for name in DECLARED_DOF_BINS
    }
    capacity: dict[str, dict[str, Any]] = {name: _empty_bin() for name in CAPACITY_BINS}
    pass_counts: Counter[str] = Counter()
    transition_counts: Counter[str] = Counter()
    coverage = {
        "release_assets": n_eval,
        "state_observed_assets": 0,
        "observed_assets": 0,
        "input_bound_assets": 0,
        "input_evaluable_assets": 0,
        "load_success_assets": 0,
        "runtime_identity_assets": 0,
        "capacity_limited_assets": 0,
        "capacity_unknown_assets": 0,
        "geometry_present_assets": 0,
        "collision_geometry_assets": 0,
        "collision_measured_assets": 0,
        "release_dofs": j_eval,
        "state_observed_independent_dofs": 0,
        "state_observed_range_evaluable_independent_dofs": 0,
        "observed_independent_dofs": 0,
        "observed_range_evaluable_independent_dofs": 0,
        "collision_measured_independent_dofs": 0,
        "collision_measured_range_evaluable_independent_dofs": 0,
        "expected_states": Counter(),
        "executed_states": Counter(),
        "free_states": Counter(),
        "state_observed_expected_states": Counter(),
        "state_observed_executed_states": Counter(),
        "state_observed_free_states": Counter(),
        "observed_expected_states": Counter(),
        "observed_executed_states": Counter(),
        "observed_free_states": Counter(),
    }
    kinematic = {
        "independent_dofs": 0,
        "range_evaluable_independent_dofs": 0,
        "unknown_independent_dofs": 0,
        "mimic_followers": 0,
        "native_mimic_followers": 0,
        "external_constraint_followers": 0,
        "fixed_root_joints": 0,
        "assets_with_mimic": 0,
        "assets_with_native_mimic": 0,
        "assets_with_external_constraints": 0,
        "assets_with_fixed_roots": 0,
    }
    asset_max_m: list[float] = []
    asset_max_normalized: list[float] = []
    asset_all_state_free_rates: list[float] = []
    asset_motion_state_free_rates: list[float] = []
    state_metric = _StreamingSeverity()
    state_raw_delta = _StreamingSeverity(bands=False)
    measured_state_expected: Counter[str] = Counter()
    measured_state_executed: Counter[str] = Counter()
    measured_state_free: Counter[str] = Counter()
    adjacent_only = 0
    rest_observed = 0
    rest_state_observed = 0
    rest_motion_eligible = 0

    cursor = connection.execute(
        "SELECT ordinal,asset_id,record_json,states_zlib,state_count,worker_status "
        "FROM results ORDER BY ordinal"
    )
    for expected_ordinal in range(n_eval):
        row = cursor.fetchone()
        if row is None:
            raise ReaggregateError(
                f"result database ended before ordinal {expected_ordinal}"
            )
        ordinal, asset_id, record_json, blob, state_count, worker_status = row
        if int(ordinal) != expected_ordinal:
            raise ReaggregateError(
                f"non-contiguous result ordinal: {ordinal} != {expected_ordinal}"
            )
        try:
            record = json.loads(record_json)
        except (TypeError, json.JSONDecodeError) as exc:
            raise ReaggregateError(f"asset {ordinal} record JSON is invalid") from exc
        if not isinstance(record, dict):
            raise ReaggregateError(f"asset {ordinal} record is not an object")
        if record.get("protocol_id") != PROTOCOL_ID:
            raise ReaggregateError(f"asset {ordinal} record protocol mismatch")
        if int(record.get("order", -1)) != expected_ordinal:
            raise ReaggregateError(f"asset {ordinal} record order mismatch")
        if str(record.get("dataset_id")) != str(asset_id):
            raise ReaggregateError(f"asset {ordinal} record/database identity mismatch")
        if isinstance(state_count, bool) or int(state_count) < 0:
            raise ReaggregateError(f"asset {ordinal} state_count is invalid")
        state_count = int(state_count)
        category = str(record.get("category") or "__UNSPECIFIED__")
        status = str(record.get("status") or "error")
        status_counts[status] += 1
        worker_status_counts[str(worker_status)] += 1
        independent = _as_nonnegative_int(record, "independent_dof_count", ordinal)
        range_dof = _as_nonnegative_int(
            record, "range_evaluable_independent_dof_count", ordinal
        )
        if range_dof > independent:
            raise ReaggregateError(
                f"asset {ordinal} range-evaluable DoFs exceed independent DoFs"
            )
        native_mimic = _as_nonnegative_int(record, "native_mimic_joint_count", ordinal)
        mimic = _as_nonnegative_int(record, "mimic_joint_count", ordinal)
        external_constraints = _as_nonnegative_int(
            record, "external_joint_constraint_count", ordinal
        )
        fixed_roots = _as_nonnegative_int(record, "fixed_root_joint_count", ordinal)
        kinematic["independent_dofs"] += independent
        kinematic["range_evaluable_independent_dofs"] += range_dof
        kinematic["unknown_independent_dofs"] += independent - range_dof
        kinematic["mimic_followers"] += mimic
        kinematic["native_mimic_followers"] += native_mimic
        kinematic["external_constraint_followers"] += external_constraints
        kinematic["fixed_root_joints"] += fixed_roots
        kinematic["assets_with_mimic"] += int(mimic > 0)
        kinematic["assets_with_native_mimic"] += int(native_mimic > 0)
        kinematic["assets_with_external_constraints"] += int(external_constraints > 0)
        kinematic["assets_with_fixed_roots"] += int(fixed_roots > 0)
        bin_name = _dof_bin(independent)
        declared_count = _as_nonnegative_int(record, "expected_movable_joints", ordinal)
        declared_name = _declared_dof_bin(declared_count, ordinal)
        link_count = record.get("link_count")
        capacity_name = _capacity_bin(link_count, ordinal)
        category_row = categories[category]
        bin_row = bins[bin_name]
        declared_row = declared_bins[declared_name]
        capacity_row = capacity[capacity_name]
        for target in (category_row, bin_row, declared_row, capacity_row):
            target["assets"] += 1
        bin_row["independent_dofs"] += independent
        bin_row["range_evaluable_independent_dofs"] += range_dof
        capacity_row["independent_dofs"] += independent
        capacity_row["range_evaluable_independent_dofs"] += range_dof
        declared_row["independent_dofs"] += independent
        declared_row["range_evaluable_independent_dofs"] += range_dof
        input_bound = bool(record.get("package_binding_verified") is True)
        load_success = bool(record.get("load_success") is True)
        runtime_identity = bool(
            load_success
            and isinstance(record.get("runtime_identity"), Mapping)
            and record.get("runtime_identity")
        )
        coverage["input_bound_assets"] += int(input_bound)
        coverage["input_evaluable_assets"] += int(input_bound and load_success)
        coverage["load_success_assets"] += int(load_success)
        coverage["runtime_identity_assets"] += int(runtime_identity)
        coverage["capacity_limited_assets"] += int(capacity_name == ">=128_links")
        coverage["capacity_unknown_assets"] += int(capacity_name == "unknown_links")
        for target in (category_row, bin_row, declared_row, capacity_row):
            target["input_bound_assets"] += int(input_bound)
            target["load_success_assets"] += int(load_success)

        geometry_present = (
            _as_nonnegative_int(record, "native_collision_elements", ordinal) > 0
        )
        geometry_available = geometry_present and str(
            record.get("collision_metric_status", "")
        ).upper() not in {
            "N/E",
            "NE",
            "BLOCKED",
            "NO_NATIVE_COLLISION",
            "NO_COLLISION_GEOMETRY",
        }
        if geometry_present:
            coverage["geometry_present_assets"] += 1
            category_row["geometry_present_assets"] += 1
            bin_row["geometry_present_assets"] += 1
            declared_row["geometry_present_assets"] += 1
            capacity_row["geometry_present_assets"] += 1
        if geometry_available:
            coverage["collision_geometry_assets"] += 1
            category_row["collision_geometry_assets"] += 1
            bin_row["collision_geometry_assets"] += 1
            declared_row["collision_geometry_assets"] += 1
            capacity_row["collision_geometry_assets"] += 1

        # We need the complete-state decision before streaming severity into
        # the global accumulator.  The cheap record-level gate is rechecked
        # against every state by ``_record_state_metrics`` below.
        record_complete_candidate = status == "completed" and range_dof == independent
        if record_complete_candidate:
            try:
                record_complete_candidate = all(
                    _as_nonnegative_int(record, f"{phase}_state_executed", ordinal)
                    == _as_nonnegative_int(record, f"{phase}_state_expected", ordinal)
                    for phase in PHASES
                )
            except ReaggregateError:
                record_complete_candidate = False
        state_accumulator = (
            state_metric if record_complete_candidate and geometry_available else None
        )
        delta_accumulator = (
            state_raw_delta
            if record_complete_candidate and geometry_available
            else None
        )
        facts = _record_state_metrics(
            record,
            bytes(blob),
            state_count,
            ordinal,
            state_severity=state_accumulator,
            state_raw_delta=delta_accumulator,
        )
        # Recompute the pass flags from the state payload.  A stored flag is
        # provenance only; if present it must agree with the independently
        # derived value so a stale summary cannot silently influence metrics.
        recomputed_flags = {
            "rest_all_pair_cf": facts["rest_all_pair_pass"],
            "rest_non_adjacent_cf": facts["rest_non_adjacent_pass"],
            "single_joint_sweep_cf": facts["single_joint_sweep_pass"],
            "multi_joint_sobol_cf": facts["multi_joint_sobol_pass"],
            "strict_collision_pass": facts["strict_collision_pass"],
        }
        for flag, derived in recomputed_flags.items():
            stored = record.get(flag)
            if stored is not None and not isinstance(stored, bool):
                raise ReaggregateError(f"asset {ordinal} stored {flag} is not boolean")
            # Incomplete child receipts may retain a phase-level flag from
            # the point at which the child failed.  The derived value is the
            # source of truth; such stale flags are deliberately ignored.
            if (
                status == "completed"
                and facts["measurement_complete"]
                and stored is not None
                and stored != derived
            ):
                raise ReaggregateError(
                    f"asset {ordinal} stored {flag} disagrees with states"
                )
        stored_complete = record.get("measurement_complete")
        if not isinstance(stored_complete, bool):
            raise ReaggregateError(
                f"asset {ordinal} measurement_complete is not boolean"
            )
        if stored_complete != facts["measurement_complete"]:
            raise ReaggregateError(
                f"asset {ordinal} measurement_complete disagrees with states"
            )
        for field, phase in (
            ("rest_non_adjacent_free", "rest"),
            ("single_non_adjacent_free", "single"),
            ("sobol_non_adjacent_free", "sobol"),
        ):
            if (
                field in record
                and _as_nonnegative_int(record, field, ordinal) != facts["free"][phase]
            ):
                raise ReaggregateError(
                    f"asset {ordinal} stored {field} disagrees with states"
                )
        observed = status == "completed" and facts["measurement_complete"]
        measured = observed and geometry_available
        if observed:
            coverage["observed_assets"] += 1
            coverage["observed_independent_dofs"] += independent
            coverage["observed_range_evaluable_independent_dofs"] += range_dof
            category_row["observed_assets"] += 1
            bin_row["observed_assets"] += 1
            declared_row["observed_assets"] += 1
            capacity_row["observed_assets"] += 1
            bin_row["observed_independent_dofs"] += independent
            declared_row["observed_independent_dofs"] += independent
            capacity_row["observed_independent_dofs"] += independent
        if state_count > 0:
            coverage["state_observed_assets"] += 1
            coverage["state_observed_independent_dofs"] += independent
            coverage["state_observed_range_evaluable_independent_dofs"] += range_dof
            category_row["state_observed_assets"] += 1
            bin_row["state_observed_assets"] += 1
            declared_row["state_observed_assets"] += 1
            capacity_row["state_observed_assets"] += 1
            bin_row["state_observed_independent_dofs"] += independent
            declared_row["state_observed_independent_dofs"] += independent
            capacity_row["state_observed_independent_dofs"] += independent
            for phase in PHASES:
                coverage["state_observed_expected_states"][phase] += facts["expected"][
                    phase
                ]
                coverage["state_observed_executed_states"][phase] += facts["phases"][
                    phase
                ]
                coverage["state_observed_free_states"][phase] += facts["free"][phase]
        if observed:
            for phase in PHASES:
                coverage["observed_expected_states"][phase] += facts["expected"][phase]
                coverage["observed_executed_states"][phase] += facts["phases"][phase]
                coverage["observed_free_states"][phase] += facts["free"][phase]
        if measured:
            coverage["collision_measured_assets"] += 1
            coverage["collision_measured_independent_dofs"] += independent
            coverage["collision_measured_range_evaluable_independent_dofs"] += range_dof
            category_row["collision_measured_assets"] += 1
            bin_row["collision_measured_assets"] += 1
            declared_row["collision_measured_assets"] += 1
            capacity_row["collision_measured_assets"] += 1
            bin_row["collision_measured_independent_dofs"] += independent
            declared_row["collision_measured_independent_dofs"] += independent
            capacity_row["collision_measured_independent_dofs"] += independent
            for phase in PHASES:
                measured_state_expected[phase] += facts["expected"][phase]
                measured_state_executed[phase] += facts["phases"][phase]
                measured_state_free[phase] += facts["free"][phase]
        if measured:
            metric_max = facts["max_metric_m"]
            if metric_max is not None:
                asset_max_m.append(float(metric_max))
            normalized = record.get("max_penetration_normalized")
            if normalized is not None:
                normalized_value = _as_nonnegative_float(
                    normalized, f"asset {ordinal} normalized maximum"
                )
                asset_max_normalized.append(normalized_value)
                diagonal = record.get("object_bbox_diagonal_m")
                if diagonal is not None:
                    diagonal_value = _as_nonnegative_float(
                        diagonal, f"asset {ordinal} bbox diagonal"
                    )
                    if diagonal_value <= 0.0:
                        raise ReaggregateError(
                            f"asset {ordinal} bbox diagonal is not positive"
                        )
                    expected_normalized = metric_max / diagonal_value
                    if not math.isclose(
                        normalized_value,
                        expected_normalized,
                        rel_tol=1e-9,
                        abs_tol=1e-12,
                    ):
                        raise ReaggregateError(
                            f"asset {ordinal} normalized maximum mismatch"
                        )
            asset_expected = sum(facts["expected"].values())
            asset_free = sum(facts["free"].values())
            if asset_expected:
                asset_all_state_free_rates.append(asset_free / asset_expected)
            motion_expected = facts["expected"]["single"] + facts["expected"]["sobol"]
            motion_free = facts["free"]["single"] + facts["free"]["sobol"]
            if motion_expected:
                asset_motion_state_free_rates.append(motion_free / motion_expected)

        for phase in PHASES:
            coverage["expected_states"][phase] += facts["expected"][phase]
            coverage["executed_states"][phase] += facts["phases"][phase]
            coverage["free_states"][phase] += facts["free"][phase]
            bin_row["expected_states"][phase] += facts["expected"][phase]
            bin_row["executed_states"][phase] += facts["phases"][phase]
            bin_row["free_states"][phase] += facts["free"][phase]
            declared_row["expected_states"][phase] += facts["expected"][phase]
            declared_row["executed_states"][phase] += facts["phases"][phase]
            declared_row["free_states"][phase] += facts["free"][phase]
            capacity_row["expected_states"][phase] += facts["expected"][phase]
            capacity_row["executed_states"][phase] += facts["phases"][phase]
            capacity_row["free_states"][phase] += facts["free"][phase]

        metric_map = {
            "rest_all_pair_cf": facts["rest_all_pair_pass"],
            "rest_non_adjacent_cf": facts["rest_non_adjacent_pass"],
            "single_joint_sweep_cf": facts["single_joint_sweep_pass"],
            "multi_joint_sobol_cf": facts["multi_joint_sobol_pass"],
            "strict_collision_pass": facts["strict_collision_pass"],
        }
        if measured:
            for key, value in metric_map.items():
                pass_counts[key] += int(value)
                category_row_key = {
                    "rest_all_pair_cf": "rest_all_pair_pass",
                    "rest_non_adjacent_cf": "rest_non_adjacent_pass",
                    "single_joint_sweep_cf": "single_joint_sweep_pass",
                    "multi_joint_sobol_cf": "multi_joint_sobol_pass",
                    "strict_collision_pass": "strict_collision_pass",
                }[key]
                category_row[category_row_key] += int(value)
                bin_row[category_row_key] += int(value)
                declared_row[category_row_key] += int(value)
                capacity_row[category_row_key] += int(value)
        if facts["phases"]["rest"] > 0:
            rest_observed += int(observed)
            rest_state_observed += 1
            adjacent = (
                facts["rest_all_pair_pass"] is False
                and facts["rest_non_adjacent_pass"] is True
            )
            adjacent_only += int(adjacent and measured)
            category_row["rest_adjacent_only"] += int(adjacent and measured)
            capacity_row["rest_adjacent_only"] += int(adjacent and measured)
        if measured and facts["measurement_complete"]:
            rest_motion_eligible += 1
            if facts["rest_non_adjacent_pass"]:
                if facts["single_joint_sweep_pass"] and facts["multi_joint_sobol_pass"]:
                    transition_counts["rest_pass_motion_pass"] += 1
                else:
                    transition_counts["rest_pass_motion_fail"] += 1
                    category_row["rest_pass_motion_fail"] += 1
                    capacity_row["rest_pass_motion_fail"] += 1
                    if not facts["single_joint_sweep_pass"]:
                        transition_counts["rest_pass_single_fail"] += 1
                    if not facts["multi_joint_sobol_pass"]:
                        transition_counts["rest_pass_sobol_fail"] += 1
            elif facts["single_joint_sweep_pass"] and facts["multi_joint_sobol_pass"]:
                transition_counts["rest_fail_motion_pass"] += 1
            else:
                transition_counts["rest_fail_motion_fail"] += 1

        bin_row["safe_dof_observed"] += facts["safe_dof_observed"] if observed else 0
        bin_row["safe_dof_passed"] += facts["safe_dof_passed"] if measured else 0
        declared_row["safe_dof_observed"] += (
            facts["safe_dof_observed"] if observed else 0
        )
        declared_row["safe_dof_passed"] += facts["safe_dof_passed"] if measured else 0
        capacity_row["safe_dof_observed"] += (
            facts["safe_dof_observed"] if observed else 0
        )
        capacity_row["safe_dof_passed"] += facts["safe_dof_passed"] if measured else 0

        error_kind = _classify_error(
            record,
            str(worker_status),
            ordinal,
            derived_complete=observed,
        )
        if error_kind is not None:
            error_counts[error_kind] += 1
            error_examples.setdefault(
                error_kind,
                {
                    "ordinal": ordinal,
                    "dataset_id": str(asset_id),
                    "category": category,
                    "issue": (record.get("issues") or [""])[0],
                },
            )
        if progress_every and (expected_ordinal + 1) % progress_every == 0:
            print(
                f"reaggregate: assets={expected_ordinal + 1}/{n_eval}",
                file=sys.stderr,
                flush=True,
            )

    if cursor.fetchone() is not None:
        raise ReaggregateError("result database contains rows beyond N_eval")
    if sum(status_counts.values()) != n_eval:
        raise ReaggregateError("status accounting does not cover the release roster")

    # The declared DoF decomposition is an accounting identity for this
    # protocol: independent roots plus follower joints plus fixed roots equals
    # the frozen declared movable-J-plus-fixed joint total.
    kinematic["declared_joint_total"] = int(j_eval)
    kinematic["independent_dof_reduction"] = int(j_eval - kinematic["independent_dofs"])
    kinematic["independent_dof_retention_rate"] = (
        kinematic["independent_dofs"] / j_eval if j_eval else None
    )
    kinematic["range_evaluable_rate_of_independent"] = (
        kinematic["range_evaluable_independent_dofs"] / kinematic["independent_dofs"]
        if kinematic["independent_dofs"]
        else None
    )
    kinematic["decomposition_sum"] = int(
        kinematic["independent_dofs"]
        + kinematic["mimic_followers"]
        + kinematic["fixed_root_joints"]
    )
    kinematic["decomposition_matches_declared"] = (
        kinematic["decomposition_sum"] == j_eval
    )
    kinematic["runtime_follower_residual"] = {
        "status": "N/E",
        "reason": "state records retain joint-value hashes, not follower readback values",
    }

    release_state_layer = _state_layer(
        coverage["expected_states"],
        coverage["executed_states"],
        coverage["free_states"],
    )
    state_observed_layer = _state_layer(
        coverage["state_observed_expected_states"],
        coverage["state_observed_executed_states"],
        coverage["state_observed_free_states"],
    )
    complete_state_layer = _state_layer(
        coverage["observed_expected_states"],
        coverage["observed_executed_states"],
        coverage["observed_free_states"],
    )
    collision_measured_state_layer = _state_layer(
        measured_state_expected,
        measured_state_executed,
        measured_state_free,
    )

    # Build category rows and equal-weight macro summaries.  Categories with
    # no observed/measured assets are retained in release macro denominators;
    # conditional macro rates explicitly report their eligible category count.
    category_rows: list[dict[str, Any]] = []
    for category in sorted(categories):
        row = {"category": category, **categories[category]}
        row["release_strict_rate"] = (
            row["strict_collision_pass"] / row["assets"] if row["assets"] else None
        )
        row["observed_strict_rate"] = (
            row["strict_collision_pass"] / row["observed_assets"]
            if row["observed_assets"]
            else None
        )
        row["measured_strict_rate"] = (
            row["strict_collision_pass"] / row["collision_measured_assets"]
            if row["collision_measured_assets"]
            else None
        )
        category_rows.append(row)

    macro_metrics: dict[str, Any] = {}
    for key in (
        "strict_collision_pass",
        "rest_all_pair_pass",
        "rest_non_adjacent_pass",
        "single_joint_sweep_pass",
        "multi_joint_sobol_pass",
        "rest_adjacent_only",
        "rest_pass_motion_fail",
    ):
        macro_metrics[key] = {
            "release": _macro_rate(category_rows, key, "assets"),
            "state_observed": _macro_rate(category_rows, key, "state_observed_assets"),
            "observed": _macro_rate(category_rows, key, "observed_assets"),
            "collision_measured": _macro_rate(
                category_rows, key, "collision_measured_assets"
            ),
        }

    def bin_report(row: Mapping[str, Any]) -> dict[str, Any]:
        result = {
            "assets": int(row["assets"]),
            "state_observed_assets": int(row.get("state_observed_assets", 0)),
            "observed_assets": int(row["observed_assets"]),
            "input_bound_assets": int(row.get("input_bound_assets", 0)),
            "load_success_assets": int(row.get("load_success_assets", 0)),
            "geometry_present_assets": int(row.get("geometry_present_assets", 0)),
            "collision_geometry_assets": int(row["collision_geometry_assets"]),
            "collision_measured_assets": int(row["collision_measured_assets"]),
            "independent_dofs": int(row["independent_dofs"]),
            "range_evaluable_independent_dofs": int(
                row["range_evaluable_independent_dofs"]
            ),
            "state_observed_independent_dofs": int(
                row.get("state_observed_independent_dofs", 0)
            ),
            "complete_independent_dofs": int(row.get("observed_independent_dofs", 0)),
            "collision_measured_independent_dofs": int(
                row.get("collision_measured_independent_dofs", 0)
            ),
            "asset_metrics": {},
            "safe_dof_retention": {
                "passed": int(row["safe_dof_passed"]),
                "complete_sweeps": int(row["safe_dof_observed"]),
                # Unknown/non-evaluable independent roots remain in the
                # release denominator.  They are not silently removed from
                # the retention metric when a package/backend failed.
                "denominator": int(row["independent_dofs"]),
                "range_unknown_dofs": int(
                    row["independent_dofs"] - row["range_evaluable_independent_dofs"]
                ),
                "not_complete_safe_sweep_dofs": int(
                    row["independent_dofs"] - row["safe_dof_observed"]
                ),
                "release_rate": (
                    row["safe_dof_passed"] / row["independent_dofs"]
                    if row["independent_dofs"]
                    else None
                ),
                "sweep_conditional_rate": (
                    row["safe_dof_passed"] / row["safe_dof_observed"]
                    if row["safe_dof_observed"]
                    else None
                ),
            },
            "state_accounting": _state_layer(
                row["expected_states"],
                row["executed_states"],
                row["free_states"],
            ),
        }
        for key, field in (
            ("rest_all_pair_cf", "rest_all_pair_pass"),
            ("rest_non_adjacent_cf", "rest_non_adjacent_pass"),
            ("single_joint_sweep_cf", "single_joint_sweep_pass"),
            ("multi_joint_sobol_cf", "multi_joint_sobol_pass"),
            ("strict_collision_pass", "strict_collision_pass"),
        ):
            result["asset_metrics"][key] = _triplet(
                int(row[field]),
                release_denominator=int(row["assets"]),
                observed_denominator=int(row["observed_assets"]),
                measured_denominator=int(row["collision_measured_assets"]),
                state_observed_denominator=int(row.get("state_observed_assets", 0)),
            )
        return result

    metrics = {
        "asset_pass": {
            key: _triplet(
                int(pass_counts[key]),
                release_denominator=n_eval,
                observed_denominator=coverage["observed_assets"],
                measured_denominator=coverage["collision_measured_assets"],
                state_observed_denominator=coverage["state_observed_assets"],
            )
            for key in PASS_KEYS
        },
        "rest_adjacent_only": _triplet(
            adjacent_only,
            release_denominator=n_eval,
            observed_denominator=rest_observed,
            measured_denominator=coverage["collision_measured_assets"],
            state_observed_denominator=rest_state_observed,
        ),
        "motion_transition": {
            "eligible_assets": _fraction(
                rest_motion_eligible, coverage["collision_measured_assets"]
            ),
            "categories": {
                key: _fraction(value, rest_motion_eligible)
                for key, value in sorted(transition_counts.items())
            },
            "rest_pass_to_motion_fail_rate": _fraction(
                transition_counts["rest_pass_motion_fail"],
                transition_counts["rest_pass_motion_fail"]
                + transition_counts["rest_pass_motion_pass"],
            ),
            "rest_pass_to_single_fail_rate": _fraction(
                transition_counts["rest_pass_single_fail"],
                transition_counts["rest_pass_motion_fail"]
                + transition_counts["rest_pass_motion_pass"],
            ),
            "rest_pass_to_sobol_fail_rate": _fraction(
                transition_counts["rest_pass_sobol_fail"],
                transition_counts["rest_pass_motion_fail"]
                + transition_counts["rest_pass_motion_pass"],
            ),
        },
        "state_micro": {
            "release": release_state_layer,
            "state_observed": state_observed_layer,
            "complete": complete_state_layer,
            "collision_measured": collision_measured_state_layer,
        },
        "motion_state_rates": {
            "release": _motion_layer_view(release_state_layer),
            "state_observed": _motion_layer_view(state_observed_layer),
            "complete": _motion_layer_view(complete_state_layer),
            "collision_measured": _motion_layer_view(collision_measured_state_layer),
        },
        "asset_equal_state_free_rate": {
            "all_states": {
                "assets": len(asset_all_state_free_rates),
                "mean_rate": (
                    sum(asset_all_state_free_rates) / len(asset_all_state_free_rates)
                    if asset_all_state_free_rates
                    else None
                ),
                "quantiles": _asset_rate_quantiles(asset_all_state_free_rates),
            },
            "motion_states": {
                "assets": len(asset_motion_state_free_rates),
                "mean_rate": (
                    sum(asset_motion_state_free_rates)
                    / len(asset_motion_state_free_rates)
                    if asset_motion_state_free_rates
                    else None
                ),
                "quantiles": _asset_rate_quantiles(asset_motion_state_free_rates),
            },
        },
        "joint_level_single_sweep": {
            "release": _fraction(
                sum(int(row["safe_dof_passed"]) for row in bins.values()),
                kinematic["independent_dofs"],
            ),
            "sweep_conditional": _fraction(
                sum(int(row["safe_dof_passed"]) for row in bins.values()),
                sum(int(row["safe_dof_observed"]) for row in bins.values()),
            ),
            "range_evaluable_independent_dofs": int(
                kinematic["range_evaluable_independent_dofs"]
            ),
            "unknown_independent_dofs": int(kinematic["unknown_independent_dofs"]),
        },
        "severity": {
            "asset_max_penetration_m": {
                "observed_assets": len(asset_max_m),
                "quantiles": _quantiles(asset_max_m),
                "bands": _band_counts(asset_max_m),
            },
            "asset_max_penetration_mm": {
                "observed_assets": len(asset_max_m),
                "quantiles": {
                    key: (None if value is None else value * 1000.0)
                    for key, value in _quantiles(asset_max_m).items()
                },
            },
            "asset_max_penetration_normalized": {
                "observed_assets": len(asset_max_normalized),
                "quantiles": _quantiles(asset_max_normalized),
            },
            "state_metric_penetration_m": {
                "observed_states": int(state_metric.count),
                "bands": state_metric.band_report(),
            },
            "raw_minus_calibrated_m": {
                "observed_states": int(state_raw_delta.count),
                "quantiles": _quantiles(state_raw_delta.values),
                "quantile_sample_size": len(state_raw_delta.values),
                "quantile_sampling": (
                    "exact"
                    if state_raw_delta.count <= state_raw_delta._RESERVOIR_LIMIT
                    else "deterministic_reservoir"
                ),
                "positive_count": int(state_raw_delta.positive),
                "negative_count": int(state_raw_delta.negative),
                "zero_count": int(state_raw_delta.zero),
            },
        },
        "dof_bins": {name: bin_report(bins[name]) for name in DOF_BINS},
        "declared_dof_bins": {
            name: bin_report(declared_bins[name]) for name in DECLARED_DOF_BINS
        },
        "capacity_bins": {name: bin_report(capacity[name]) for name in CAPACITY_BINS},
        "local_contact_adjusted": {
            "status": "N/E",
            "registered_pair_count": 0,
            "reason": (
                "no asset/URDF-hash/link-pair/local-region/depth-bound review "
                "registry was frozen for v3; the no-allowlist oracle remains authoritative"
            ),
        },
        "category_macro": {
            "category_count": len(category_rows),
            "metrics": macro_metrics,
            "rows": category_rows,
        },
    }
    return {
        "coverage": {
            "release": {
                "assets": n_eval,
                "declared_joints": j_eval,
                "independent_dofs": int(kinematic["independent_dofs"]),
                "states": release_state_layer,
            },
            "state_observed": {
                "assets": coverage["state_observed_assets"],
                "rate_of_release": (
                    coverage["state_observed_assets"] / n_eval if n_eval else None
                ),
                "independent_dofs": coverage["state_observed_independent_dofs"],
                "range_evaluable_independent_dofs": coverage[
                    "state_observed_range_evaluable_independent_dofs"
                ],
                "states": state_observed_layer,
            },
            "complete": {
                "assets": coverage["observed_assets"],
                "rate_of_release": (
                    coverage["observed_assets"] / n_eval if n_eval else None
                ),
                "independent_dofs": coverage["observed_independent_dofs"],
                "range_evaluable_independent_dofs": coverage[
                    "observed_range_evaluable_independent_dofs"
                ],
                "states": complete_state_layer,
            },
            # Compatibility alias: in this report "observed" always means
            # complete state accounting, never merely state_count > 0.
            "observed": {
                "assets": coverage["observed_assets"],
                "complete_assets": coverage["observed_assets"],
                "state_observed_assets": coverage["state_observed_assets"],
                "input_bound_assets": coverage["input_bound_assets"],
                "input_evaluable_assets": coverage["input_evaluable_assets"],
                "load_success_assets": coverage["load_success_assets"],
                "runtime_identity_assets": coverage["runtime_identity_assets"],
                "independent_dofs": coverage["observed_independent_dofs"],
                "range_evaluable_independent_dofs": coverage[
                    "observed_range_evaluable_independent_dofs"
                ],
                "states": complete_state_layer,
            },
            "collision_geometry": {
                "present_assets": coverage["geometry_present_assets"],
                "present_rate_of_release": (
                    coverage["geometry_present_assets"] / n_eval if n_eval else None
                ),
                "oracle_initialized_assets": coverage["collision_geometry_assets"],
                "oracle_initialized_rate_of_release": (
                    coverage["collision_geometry_assets"] / n_eval if n_eval else None
                ),
            },
            "input": {
                "bound_assets": coverage["input_bound_assets"],
                "evaluable_assets": coverage["input_evaluable_assets"],
                "load_success_assets": coverage["load_success_assets"],
                "runtime_identity_assets": coverage["runtime_identity_assets"],
                "capacity_limited_assets": coverage["capacity_limited_assets"],
                "capacity_unknown_assets": coverage["capacity_unknown_assets"],
                "rates_of_release": {
                    "bound": (
                        coverage["input_bound_assets"] / n_eval if n_eval else None
                    ),
                    "evaluable": (
                        coverage["input_evaluable_assets"] / n_eval if n_eval else None
                    ),
                    "load_success": (
                        coverage["load_success_assets"] / n_eval if n_eval else None
                    ),
                },
            },
            "collision_measured": {
                "assets": coverage["collision_measured_assets"],
                "rate_of_release": (
                    coverage["collision_measured_assets"] / n_eval if n_eval else None
                ),
                "oracle_coverage_of_geometry_present": _fraction(
                    coverage["collision_measured_assets"],
                    coverage["geometry_present_assets"],
                ),
                "independent_dofs": coverage["collision_measured_independent_dofs"],
                "range_evaluable_independent_dofs": coverage[
                    "collision_measured_range_evaluable_independent_dofs"
                ],
                "states": collision_measured_state_layer,
            },
            "state_accounting": release_state_layer,
        },
        "status_counts": dict(sorted(status_counts.items())),
        "worker_status_counts": dict(sorted(worker_status_counts.items())),
        "kinematic_decomposition": kinematic,
        "error_taxonomy": {
            "counts": dict(sorted(error_counts.items())),
            "total_error_or_incomplete": int(sum(error_counts.values())),
            "rates_of_release": {
                key: _fraction(value, n_eval)
                for key, value in sorted(error_counts.items())
            },
            "backend_capacity_rate_of_high_link_tail": _fraction(
                error_counts["backend_capacity"],
                capacity[">=128_links"]["assets"],
            ),
            "package_binding_drift_rate_of_release": _fraction(
                error_counts["package_binding_drift"], n_eval
            ),
            "examples": {key: error_examples[key] for key in sorted(error_examples)},
        },
        "metrics": metrics,
    }


def reaggregate(
    output: Path,
    *,
    verify_large_artifacts: bool = False,
    progress_every: int = 0,
) -> dict[str, Any]:
    """Validate and reaggregate a sealed v3 output directory."""

    reaggregator_sha256 = _sha256_file(SCRIPT)
    manifest, receipt, checkpoint, connection, bindings = _validate_sealed_input(
        output,
        verify_large_artifacts=verify_large_artifacts,
    )
    try:
        body = _aggregate(
            connection,
            n_eval=bindings["n_eval"],
            j_eval=bindings["j_eval"],
            progress_every=progress_every,
        )
    finally:
        connection.close()
    if _sha256_file(SCRIPT) != reaggregator_sha256:
        raise ReaggregateError("reaggregator source changed during execution")
    sealed_summary = bindings["summary"]
    derived_metrics = body["metrics"]
    derived_coverage = body["coverage"]
    parity_checks = {
        "status_counts": body["status_counts"] == sealed_summary.get("status_counts"),
        "declared_joint_total": body["kinematic_decomposition"]["declared_joint_total"]
        == sealed_summary.get("declared_dof_count"),
        "independent_dofs": body["kinematic_decomposition"]["independent_dofs"]
        == sealed_summary.get("independent_dof_count"),
        "range_evaluable_independent_dofs": body["kinematic_decomposition"][
            "range_evaluable_independent_dofs"
        ]
        == sealed_summary.get("range_evaluable_independent_dof_count"),
        "mimic_followers": body["kinematic_decomposition"]["mimic_followers"]
        == sealed_summary.get("mimic_joint_count"),
        "external_constraints": body["kinematic_decomposition"][
            "external_constraint_followers"
        ]
        == sealed_summary.get("external_joint_constraint_count"),
        "fixed_roots": body["kinematic_decomposition"]["fixed_root_joints"]
        == sealed_summary.get("fixed_root_joint_count"),
        "expected_states": derived_coverage["state_accounting"]["expected"]
        == sealed_summary.get("expected_states"),
        "executed_states": derived_coverage["state_accounting"]["executed"]
        == sealed_summary.get("executed_states"),
        "collision_geometry_assets": derived_coverage["collision_geometry"][
            "oracle_initialized_assets"
        ]
        == sealed_summary.get("collision_geometry_assets"),
        "asset_pass_counts": all(
            derived_metrics["asset_pass"][key]["release"]["passed"]
            == sealed_summary.get("metrics", {}).get(key, {}).get("passed")
            for key in PASS_KEYS
        ),
        "single_state_free": derived_coverage["state_accounting"]["free"]["single"]
        == sealed_summary.get("metrics", {})
        .get("collision_free_range", {})
        .get("passed_states"),
        "collision_state_count": derived_coverage["state_accounting"]["all_states"][
            "collision_state_intent"
        ]["passed"]
        == sealed_summary.get("metrics", {})
        .get("collision_state_rate", {})
        .get("collision_states"),
        "maximum_normalized_penetration": math.isclose(
            derived_metrics["severity"]["asset_max_penetration_normalized"][
                "quantiles"
            ]["max"],
            sealed_summary.get("metrics", {})
            .get("max_penetration", {})
            .get("maximum_observed_normalized"),
            rel_tol=0.0,
            abs_tol=1e-15,
        ),
    }
    failed_parity = [name for name, passed in parity_checks.items() if not passed]
    if failed_parity:
        raise ReaggregateError(
            "derived metrics disagree with sealed summary: " + ", ".join(failed_parity)
        )
    report: dict[str, Any] = {
        "schema_version": "pva_table4_status_aware_metrics_v1",
        "protocol_id": PROTOCOL_ID,
        "source": {
            "output": str(Path(output).resolve()),
            "manifest_sha256": bindings["manifest_sha256"],
            "manifest_content_sha256": bindings["manifest_content_sha256"],
            "receipt_sha256": bindings["receipt_sha256"],
            "checkpoint_sha256": bindings["checkpoint_sha256"],
            "database_sha256": bindings["database_sha256"],
            "artifact_manifest_sha256": bindings["artifact_manifest_sha256"],
            "artifact_audit": bindings["artifact_audit"],
            "reaggregator_sha256": reaggregator_sha256,
            "summary_content_sha256": sealed_summary["summary_content_sha256"],
            "N_eval": bindings["n_eval"],
            "J_eval": bindings["j_eval"],
        },
        "definitions": {
            "release_denominator": "all manifest N_eval assets/declared J_eval joints; missing execution remains fail-closed",
            "state_observed_denominator": "asset with state_count > 0 (partial assets retained for coverage only)",
            "observed_denominator": "compatibility alias for status=completed with complete rest/single/joint-space-Sobol state accounting",
            "complete_denominator": "status=completed with every planned discrete state executed and every independent root range-evaluable",
            "collision_measured_denominator": "complete asset with native collision geometry and an initialized collision oracle",
            "state_free_intent": "free states divided by planned states; unexecuted states are not free",
            "state_free_executed": "free states divided only by states that reached the oracle; execution coverage is reported beside it",
            "state_micro": "state-weighted aggregation; assets with more independent DoFs contribute more sweep states",
            "asset_equal_state_free_rate": "per-asset free-state fraction summarized with equal asset weight over collision-measured assets",
            "input_bound": "package_binding_verified=true",
            "input_evaluable": "package binding verified and load_success=true",
            "collision_geometry_present": "native_collision_elements > 0, independent of whether the backend completed",
            "oracle_initialized": "native geometry present and collision_metric_status is neither N/E nor blocked; partial execution remains separate from complete measurement",
            "capacity_bin": "raw source link_count <128, >=128, or unknown; >=128 is a backend-stress diagnostic, not a geometry-failure label",
            "independent_dof_bins": list(DOF_BINS),
            "declared_dof_bins": list(DECLARED_DOF_BINS),
            "kinematic_denominator": "independent roots are sampled; native mimic and registered external followers are derived, not independent Sobol dimensions",
            "single_joint_samples": SINGLE_SAMPLES,
            "joint_space_sobol": "64 discrete joint-space samples for every asset with independent DoF > 0, including one-DoF assets; this is not a continuous-space guarantee",
            "rest_adjacent_only": "rest all-pair fails while rest non-adjacent passes",
            "motion_transition": "complete measured asset transition from rest non-adjacent result to single/Sobol results",
            "category_macro": "equal-weight mean of per-category asset pass rates; state_micro is reported separately",
            "local_contact_adjustment": "supplementary only and N/E because v3 froze an empty reviewed registry; no category/link-wide or post-hoc allowlist is applied",
            "severity_bands_m": [
                {"label": label, "lower_exclusive_m": lower, "upper_inclusive_m": upper}
                for label, lower, upper in SEVERITY_BANDS
            ],
        },
        "sealed_summary_parity": {
            "status": "PASS",
            "checks": parity_checks,
        },
        **body,
    }
    report["status"] = "COMPLETE"
    report["report_content_sha256"] = _self_hash(report, "report_content_sha256")
    return report


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def _summary_markdown(report: Mapping[str, Any]) -> str:
    """Create a compact human-readable receipt without changing the JSON source."""

    coverage = report["coverage"]
    metrics = report["metrics"]

    def rate(path: tuple[str, ...], layer: str = "release") -> str:
        value: Any = metrics
        for key in path:
            value = value[key]
        fraction = value.get(layer, value)
        if not isinstance(fraction, Mapping) or fraction.get("rate") is None:
            return "N/E"
        return f"{fraction['passed']} / {fraction['denominator']} ({fraction['rate'] * 100:.4f}%)"

    state = coverage["state_accounting"]
    expected_total = sum(int(value) for value in state["expected"].values())
    executed_total = sum(int(value) for value in state["executed"].values())
    lines = [
        "# PV-A Table 4 status-aware metrics",
        "",
        "This is a deterministic, read-only reaggregation of the sealed v3 SQLite result.",
        "It is not a second physics run; the source receipt and hashes are in `metrics.json`.",
        "",
        f"- Protocol: `{report['protocol_id']}`",
        f"- Release assets: `{coverage['release']['assets']}`",
        f"- Input-bound assets: `{coverage['input']['bound_assets']}`",
        f"- State-observed assets: `{coverage['state_observed']['assets']}`",
        f"- Complete assets: `{coverage['complete']['assets']}`",
        f"- Fully measured assets: `{coverage['collision_measured']['assets']}`",
        f"- State coverage: `{executed_total} / {expected_total}` "
        f"({executed_total / expected_total * 100:.4f}%)",
        "",
        "| Asset metric | Release (fail-closed) | Collision-measured conditional |",
        "|---|---:|---:|",
        f"| Rest non-adjacent CF | {rate(('asset_pass', 'rest_non_adjacent_cf'))} | {rate(('asset_pass', 'rest_non_adjacent_cf'), 'collision_measured')} |",
        f"| Single-joint sweep CF | {rate(('asset_pass', 'single_joint_sweep_cf'))} | {rate(('asset_pass', 'single_joint_sweep_cf'), 'collision_measured')} |",
        f"| Joint-space Sobol CF | {rate(('asset_pass', 'multi_joint_sobol_cf'))} | {rate(('asset_pass', 'multi_joint_sobol_cf'), 'collision_measured')} |",
        f"| Strict collision pass | {rate(('asset_pass', 'strict_collision_pass'))} | {rate(('asset_pass', 'strict_collision_pass'), 'collision_measured')} |",
        f"| Rest adjacent-only diagnostic | {rate(('rest_adjacent_only',))} | {rate(('rest_adjacent_only',), 'collision_measured')} |",
        "",
        "`complete` means complete discrete state accounting; `state_observed` also includes partial assets.",
        "State-micro, asset-equal, category-macro, DoF-bin, severity, and error-taxonomy views are in `metrics.json`.",
        "Unexecuted states remain fail-closed in release rates and are never relabeled as geometry failures.",
        "",
    ]
    return "\n".join(lines)


def write_bundle(report: Mapping[str, Any], output_dir: Path) -> dict[str, Any]:
    """Write a small, hash-bound derived publication bundle."""

    output_dir = Path(output_dir)
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    if output_dir.exists():
        if not output_dir.is_dir():
            raise ReaggregateError(f"derived output is not a directory: {output_dir}")
        if any(output_dir.iterdir()):
            raise ReaggregateError(
                f"derived output directory is not empty: {output_dir}"
            )
    else:
        output_dir.mkdir()

    metrics_path = output_dir / "metrics.json"
    category_path = output_dir / "category_metrics.json"
    verification_path = output_dir / "verification.json"
    summary_path = output_dir / "summary.md"
    _write_json(metrics_path, dict(report))
    category_payload: dict[str, Any] = {
        "schema_version": "pva_table4_status_aware_category_metrics_v1",
        "protocol_id": report["protocol_id"],
        "source": report["source"],
        "category_macro": report["metrics"]["category_macro"],
    }
    category_payload["content_sha256"] = _self_hash(category_payload, "content_sha256")
    _write_json(category_path, category_payload)
    verification_checks = {
        "report_content_hash_valid": report.get("report_content_sha256")
        == _self_hash(report, "report_content_sha256"),
        "release_asset_count": report["coverage"]["release"]["assets"]
        == report["source"]["N_eval"],
        "kinematic_decomposition": report["kinematic_decomposition"][
            "decomposition_matches_declared"
        ],
        "sealed_summary_parity": report["sealed_summary_parity"]["status"] == "PASS"
        and all(report["sealed_summary_parity"]["checks"].values()),
        "category_count": report["metrics"]["category_macro"]["category_count"] == 531,
        "state_accounting_present": "state_accounting" in report["coverage"],
    }
    verification: dict[str, Any] = {
        "schema_version": "pva_table4_status_aware_verification_v1",
        "protocol_id": report["protocol_id"],
        "status": (
            "PASS"
            if report.get("status") == "COMPLETE" and all(verification_checks.values())
            else "FAIL"
        ),
        "source": report["source"],
        "checks": verification_checks,
    }
    verification["content_sha256"] = _self_hash(verification, "content_sha256")
    _write_json(verification_path, verification)
    summary_path.write_text(_summary_markdown(report), encoding="utf-8")

    artifact_paths = (metrics_path, category_path, verification_path, summary_path)
    manifest: dict[str, Any] = {
        "schema_version": "pva_table4_status_aware_metrics_manifest_v1",
        "protocol_id": report["protocol_id"],
        "source": report["source"],
        "report_content_sha256": report["report_content_sha256"],
        "artifacts": [
            {
                "path": path.name,
                "bytes": path.stat().st_size,
                "sha256": _sha256_file(path),
            }
            for path in artifact_paths
        ],
    }
    manifest["manifest_content_sha256"] = _self_hash(
        manifest, "manifest_content_sha256"
    )
    manifest_path = output_dir / "manifest.json"
    _write_json(manifest_path, manifest)
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", type=Path, required=True, help="sealed v3 output directory"
    )
    parser.add_argument("--output", type=Path, help="write derived report JSON here")
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="write a hash-bound derived publication bundle (mutually exclusive with --output)",
    )
    parser.add_argument(
        "--verify-large-artifacts",
        action="store_true",
        help="also hash the large records/state JSONL files listed by artifact_manifest.json",
    )
    parser.add_argument(
        "--progress-every",
        type=int,
        default=10000,
        help="write progress to stderr every N assets (0 disables it)",
    )
    args = parser.parse_args(argv)
    if args.output is not None and args.output_dir is not None:
        parser.error("--output and --output-dir are mutually exclusive")
    try:
        report = reaggregate(
            args.input,
            verify_large_artifacts=args.verify_large_artifacts,
            progress_every=max(0, args.progress_every),
        )
    except Exception as exc:  # noqa: BLE001
        payload = {
            "schema_version": "pva_table4_status_aware_metrics_v1",
            "status": "ERROR",
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=True))
        return 1
    text = json.dumps(report, indent=2, sort_keys=True, ensure_ascii=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(f"derived_report={args.output.resolve()}", file=sys.stderr, flush=True)
    if args.output_dir:
        manifest = write_bundle(report, args.output_dir)
        print(
            f"derived_bundle={args.output_dir.resolve()} "
            f"manifest_content_sha256={manifest['manifest_content_sha256']}",
            file=sys.stderr,
            flush=True,
        )
    if args.output is None and args.output_dir is None:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["ReaggregateError", "reaggregate", "main"]
