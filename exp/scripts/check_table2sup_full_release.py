#!/usr/bin/env python3
"""Read-only acceptance checks for the full-release Table 2 supplementary run.

The checker deliberately does not import or execute the evaluator. It reads
the frozen roster and published records, recomputes the four diagnostic atoms,
and verifies the hashes that bind every output to its inputs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Mapping, Sequence

try:
    import table123_full_release_common as common
except ImportError:  # pragma: no cover
    from . import table123_full_release_common as common


class AutomationError(ValueError):
    """Raised when a supplementary full-release contract is not closed."""


DATASETS = (
    {"slug": "articraft", "display": "Articraft-10K", "n_eval": 9996, "j_eval": 37144},
    {"slug": "lam", "display": "LAM released outputs", "dataset_names": ("LAM", "LAM released outputs"), "n_eval": 3217, "j_eval": 10381},
    {"slug": "artiverse", "display": "Artiverse", "n_eval": 3544, "j_eval": 16332},
    {"slug": "partnet", "display": "PartNet-Mobility", "n_eval": 2347, "j_eval": 11971},
    {"slug": "physx", "display": "PhysX-Mobility", "n_eval": 2024, "j_eval": 9883},
    {"slug": "sketch", "display": "SketchMobility", "n_eval": 4956, "j_eval": 11009},
    {"slug": "infinite", "display": "Infinite Mobility", "n_eval": 720, "j_eval": 4723},
    {"slug": "infinigen", "display": "Infinigen-Sim", "n_eval": 8226, "j_eval": 31975},
)
METRICS = (
    "visual_bearing_collision_coverage",
    "joint_limit_portability",
    "joint_dynamics_coverage",
    "placeholder_mass_incidence",
)
DATASET_BY_SLUG = {item["slug"]: item for item in DATASETS}
# The historical combined receipt labels LAM as ``LAM`` while the publication
# table uses ``LAM released outputs``.  Both labels identify the same frozen
# cohort; all other display names are canonical.
DISPLAY_ALIASES = {
    "lam": {"LAM", "LAM released outputs"},
}


def _json(path: Path) -> Any:
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise AutomationError(f"cannot read JSON {path}: {exc}") from exc


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as handle:
            for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise AutomationError(f"cannot hash {path}: {exc}") from exc
    return digest.hexdigest()


def _without(value: Mapping[str, Any], field: str) -> dict[str, Any]:
    result = dict(value)
    result.pop(field, None)
    return result


def _check_self_hash(
    value: Mapping[str, Any],
    label: str,
    *,
    field: str | None = None,
    required: bool = False,
) -> str | None:
    """Validate the self-hash for one receipt object.

    Several receipt objects deliberately carry *other* receipt hashes (for
    example a summary carries ``manifest_content_sha256``).  Selecting the
    first hash-looking key therefore validates the wrong payload.  Callers
    pass the schema-specific field explicitly; the fallback keeps this helper
    useful for legacy callers while still requiring a matching declaration.
    """
    fields = (
        "manifest_content_sha256",
        "checkpoint_content_sha256",
        "artifact_manifest_content_sha256",
        "summary_content_sha256",
        "receipt_content_sha256",
    )
    if field is None:
        # Prefer a field whose name matches the object label.  If no such field
        # is present, retain the historical first-present fallback.
        label_lower = label.lower()
        preferred = (
            "receipt_content_sha256" if "receipt" in label_lower else None,
            "artifact_manifest_content_sha256" if "artifact" in label_lower else None,
            "summary_content_sha256" if "summary" in label_lower else None,
            "checkpoint_content_sha256" if "checkpoint" in label_lower else None,
            "manifest_content_sha256" if "manifest" in label_lower else None,
        )
        field = next((name for name in preferred if name and name in value), None)
        field = field or next((name for name in fields if name in value), None)
    elif field not in fields:
        raise AutomationError(f"{label} has unsupported self-hash field: {field}")
    if field is None:
        if required:
            raise AutomationError(f"{label} has no self-hash")
        return None
    declared = value.get(field)
    observed = common.canonical_sha256(_without(value, field))
    if declared != observed:
        raise AutomationError(f"{label} self-hash mismatch")
    return str(declared)


def _records(path: Path) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise AutomationError(f"cannot read records {path}: {exc}") from exc
    for number, line in enumerate(lines, 1):
        if not line.strip():
            raise AutomationError(f"blank JSONL row: {path}:{number}")
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise AutomationError(f"invalid record JSON {path}:{number}: {exc}") from exc
        if not isinstance(value, dict):
            raise AutomationError(f"record is not an object: {path}:{number}")
        result.append(value)
    return result


def _metric_obj(record: Mapping[str, Any]) -> Mapping[str, Any]:
    value = record.get("table2_supplementary")
    if value is None:
        value = record.get("metrics")
    return value if isinstance(value, Mapping) else {}


def _metric(record: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = _metric_obj(record).get(name, {})
    return value if isinstance(value, Mapping) else {}


def _number(value: Any, *, label: str) -> int:
    if isinstance(value, bool):
        raise AutomationError(f"{label} is not an integer")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise AutomationError(f"{label} is not an integer") from exc
    if parsed < 0:
        raise AutomationError(f"{label} is negative")
    return parsed


def _aggregate(records: Sequence[Mapping[str, Any]], n_eval: int, j_eval: int) -> dict[str, Any]:
    if len(records) != n_eval:
        raise AutomationError(f"record count mismatch: {len(records)} != {n_eval}")
    visual_passed = visual_declared = visual_covered = 0
    extraction_complete = zero_visual = 0
    port_passed = port_extracted = port_intended = 0
    dyn_covered = dyn_extracted = dyn_intended = 0
    complete_inertial = dynamic_links = 0
    statuses: dict[str, int] = {}
    for index, record in enumerate(records):
        status = str(record.get("status", ""))
        if status not in {"completed", "error", "timeout"}:
            raise AutomationError(f"record {index} has unknown status: {status!r}")
        statuses[status] = statuses.get(status, 0) + 1
        visual = _metric(record, "visual_bearing_collision_coverage")
        port = _metric(record, "joint_limit_portability")
        dyn = _metric(record, "joint_dynamics_coverage")
        inertial = _metric(record, "placeholder_mass_incidence")
        visual_passed += int(bool(visual.get("asset_pass", visual.get("asset_passed", False))))
        visual_declared += _number(visual.get("visual_bearing_links_declared", 0), label=f"record {index} visual links")
        visual_covered += _number(visual.get("covered_visual_bearing_links", 0), label=f"record {index} covered links")
        extraction_complete += int(bool(visual.get("link_extraction_complete", False)))
        if status == "completed" and visual.get("visual_bearing_links_declared", 0) == 0:
            zero_visual += 1
        intended = _number(port.get("joints_intended", port.get("denominator", 0)), label=f"record {index} portability denominator")
        extracted = _number(port.get("joints_extracted", 0), label=f"record {index} portability extracted")
        passed = _number(port.get("joints_passed", port.get("numerator", 0)), label=f"record {index} portability numerator")
        if passed > intended or passed > extracted:
            raise AutomationError(
                f"record {index} portability counts exceed intended/extracted joints"
            )
        port_intended += intended
        port_extracted += extracted
        port_passed += passed
        intended = _number(dyn.get("joints_intended", dyn.get("denominator", 0)), label=f"record {index} dynamics denominator")
        extracted = _number(dyn.get("joints_extracted", 0), label=f"record {index} dynamics extracted")
        covered = _number(dyn.get("joints_covered", dyn.get("numerator", 0)), label=f"record {index} dynamics numerator")
        if covered > intended or covered > extracted:
            raise AutomationError(
                f"record {index} dynamics counts exceed intended/extracted joints"
            )
        dyn_intended += intended
        dyn_extracted += extracted
        dyn_covered += covered
        complete_inertial += _number(inertial.get("complete_inertial_links", inertial.get("complete_inertial", 0)), label=f"record {index} complete inertial")
        dynamic_links += _number(inertial.get("dynamic_links", inertial.get("denominator", 0)), label=f"record {index} dynamic links")
    if port_intended != j_eval:
        raise AutomationError(f"joint portability denominator mismatch: {port_intended} != {j_eval}")
    if dyn_intended != j_eval:
        raise AutomationError(f"joint dynamics denominator mismatch: {dyn_intended} != {j_eval}")
    return {
        "n_eval": n_eval,
        "j_eval": j_eval,
        "status_counts": dict(sorted(statuses.items())),
        "metrics": {
            "visual_bearing_collision_coverage": {
                "passed": visual_passed,
                "denominator": n_eval,
                "link_micro": {"numerator": visual_covered, "denominator": visual_declared},
                "link_extraction_complete_assets": extraction_complete,
                "zero_visual_bearing_assets_completed": zero_visual,
            },
            "joint_limit_portability": {"passed": port_passed, "denominator": j_eval, "joints_extracted": port_extracted},
            "joint_dynamics_coverage": {"passed": dyn_covered, "denominator": j_eval, "joints_extracted": dyn_extracted},
            "placeholder_mass_incidence": {
                "status": "N/E",
                "passed": None,
                "denominator": None,
                "complete_inertial_links": complete_inertial,
                "dynamic_links": dynamic_links,
            },
        },
    }


def _find_records(output: Path) -> Path:
    for name in ("records.jsonl", "asset_records.jsonl"):
        path = output / name
        if path.is_file():
            return path
    raise AutomationError(f"missing records JSONL: {output}")


def _load_roster(manifest: Mapping[str, Any], run_root: Path) -> tuple[dict[str, Any], Path]:
    value = manifest.get("roster") or manifest.get("roster_path")
    if not isinstance(value, str) or not value:
        raise AutomationError("run manifest has no frozen roster path")
    path = Path(value)
    if not path.is_absolute():
        path = run_root / path
    path = path.resolve(strict=True)
    roster = _json(path)
    if not isinstance(roster, dict) or roster.get("schema_version") != "table123_full_release_manifest_v1":
        raise AutomationError(f"roster schema mismatch: {path}")
    _check_self_hash(
        roster,
        "roster manifest",
        field="manifest_content_sha256",
        required=True,
    )
    rows = roster.get("rows")
    if not isinstance(rows, list):
        raise AutomationError("roster has no rows")
    if manifest.get("roster_sha256") and _sha(path) != manifest["roster_sha256"]:
        raise AutomationError("run manifest roster file hash mismatch")
    ordered_jsonl = path.with_name("full_release_roster.jsonl")
    if roster.get("roster_jsonl_sha256"):
        if not ordered_jsonl.is_file() or _sha(ordered_jsonl) != roster["roster_jsonl_sha256"]:
            raise AutomationError("ordered roster JSONL hash mismatch")
    return roster, path


def _verify_source_bindings(manifest: Mapping[str, Any], *, strict: bool) -> list[dict[str, Any]]:
    raw = manifest.get("source_bindings", [])
    if not isinstance(raw, list):
        raise AutomationError("source_bindings must be a list")
    checked: list[dict[str, Any]] = []
    for binding in raw:
        if not isinstance(binding, Mapping) or not binding.get("path"):
            continue
        path = Path(str(binding["path"]))
        if not path.is_absolute():
            path = Path.cwd() / path
        path = path.resolve(strict=True)
        if binding.get("sha256") and _sha(path) != str(binding["sha256"]):
            raise AutomationError(f"source binding hash mismatch: {path}")
        if path.name == "parts.zip" and path == Path("/mnt/zsn/lyb/arti-skill/exp/parts.zip").resolve():
            expected = "9d501098ca5516fdb347c95e1c18170900563cd133e489bc7f0020515e643e16"
            if _sha(path) != expected:
                raise AutomationError("canonical parts.zip hash mismatch")
        checked.append({"name": binding.get("name"), "path": str(path), "sha256": binding.get("sha256")})
    if strict and not checked:
        raise AutomationError("manifest has no source bindings")
    return checked


def _verify_children(output: Path, records: Sequence[Mapping[str, Any]], jobs: Sequence[Mapping[str, Any]]) -> None:
    children = output / "children"
    if not children.is_dir():
        raise AutomationError("missing children directory")
    for job, record in zip(jobs, records, strict=True):
        index = int(job["selection_index"])
        path = children / f"{index:06d}.json"
        if not path.is_file():
            raise AutomationError(f"missing child receipt: {path.name}")
        child = _json(path)
        if not isinstance(child, Mapping):
            raise AutomationError(f"child receipt is not an object: {path.name}")
        if child.get("asset_id") != record.get("asset_id") or int(child.get("selection_index", -1)) != index:
            raise AutomationError(f"child binding mismatch: {path.name}")
        if child.get("expected_primary_urdf_sha256") != job.get("expected_primary_urdf_sha256"):
            raise AutomationError(f"child URDF binding mismatch: {path.name}")
        # The parent record is the value that is aggregated.  Bind its
        # evaluator payload to the child receipt so a stale or substituted
        # child cannot silently pass artifact checks.
        if child.get("status") != record.get("status"):
            raise AutomationError(f"child status binding mismatch: {path.name}")
        if common.canonical_sha256(child.get("table2_supplementary")) != common.canonical_sha256(record.get("table2_supplementary")):
            raise AutomationError(f"child table2_supplementary binding mismatch: {path.name}")
        if _runtime_path(child.get("package")) != _runtime_path(record.get("package")):
            raise AutomationError(f"child package binding mismatch: {path.name}")
        if _relative_runtime_path(child.get("urdf_relative_path")) != _relative_runtime_path(record.get("urdf_relative_path")):
            raise AutomationError(f"child URDF relative path binding mismatch: {path.name}")
        # Failure records may intentionally omit these optional fields.  When
        # the parent publishes one, however, the child must carry the exact
        # same frozen expectation.
        for field in ("expected_primary_urdf_sha256", "expected_movable_joints"):
            if field in record and child.get(field) != record.get(field):
                raise AutomationError(f"child {field} binding mismatch: {path.name}")
        if "run_manifest_content_sha256" in record and child.get("run_manifest_content_sha256") != record.get("run_manifest_content_sha256"):
            raise AutomationError(f"child run manifest binding mismatch: {path.name}")


def _runtime_path(value: Any) -> str | None:
    """Normalize a package path for parent/child binding comparisons."""

    if value is None:
        return None
    if not isinstance(value, str):
        return str(value)
    return Path(value).resolve(strict=False).as_posix()


def _relative_runtime_path(value: Any) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        return str(value)
    return value.replace("\\", "/")


def _live_dataset(root: Path, item: Mapping[str, Any], source_mode: bool | str, entry: Mapping[str, Any] | None = None) -> dict[str, Any]:
    # Resolve once up front.  Without this, a direct caller passing a relative
    # root and an absent output_root would join ``root`` twice.
    root = Path(root).resolve()
    entry = entry or {}
    output_value = entry.get("output_root") or entry.get("output")
    output = Path(str(output_value)) if output_value else Path(str(item["slug"]))
    if not output.is_absolute():
        output = root / output
    output = output.resolve()
    if not output.is_dir():
        raise AutomationError(f"missing output directory: {item['display']}")
    manifest_path = output / "manifest.json"
    if not manifest_path.is_file():
        raise AutomationError(f"missing run manifest: {item['display']}")
    manifest = _json(manifest_path)
    if not isinstance(manifest, Mapping):
        raise AutomationError(f"run manifest is not an object: {item['display']}")
    _check_self_hash(
        manifest,
        f"{item['display']} run manifest",
        field="manifest_content_sha256",
        required=True,
    )
    dataset = manifest.get("dataset", item["display"])
    if dataset not in tuple(item.get("dataset_names", (item["display"],))):
        raise AutomationError(f"dataset identity mismatch: {dataset!r}")
    n_eval = _number(manifest.get("N_eval"), label=f"{item['display']} N_eval")
    j_eval = _number(manifest.get("J_eval"), label=f"{item['display']} J_eval")
    if (n_eval, j_eval) != (item["n_eval"], item["j_eval"]):
        raise AutomationError(f"{item['display']} manifest N/J mismatch")
    strict_sources = source_mode is True or source_mode == "strict"
    bindings = _verify_source_bindings(manifest, strict=strict_sources)
    if item["slug"] == "infinite" and strict_sources and not any(str(x.get("name", "")).lower() == "parts_zip" for x in bindings):
        raise AutomationError("Infinite Mobility manifest is missing parts.zip binding")
    if item["slug"] == "infinigen" and strict_sources and not any("archive_validation_receipt" in str(x.get("name", "")) for x in bindings):
        raise AutomationError("Infinigen-Sim manifest is missing archive validation receipt binding")
    roster, roster_path = _load_roster(manifest, root)
    rows = roster["rows"]
    if len(rows) != n_eval or sum(_number(row.get("joint_count", 0), label="roster joint_count") for row in rows) != j_eval:
        raise AutomationError(f"{item['display']} frozen roster N/J mismatch")
    jobs: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping) or not isinstance(row.get("asset_id"), str):
            raise AutomationError(f"{item['display']} invalid roster row {index}")
        jobs.append({
            "selection_index": index,
            "asset_id": str(row["asset_id"]),
            "expected_primary_urdf_sha256": row.get("primary_urdf_sha256"),
            "expected_movable_joints": _number(row.get("joint_count", 0), label="roster joint_count"),
        })
    records_path = _find_records(output)
    records = _records(records_path)
    if len(records) != n_eval:
        raise AutomationError(f"{item['display']} record count mismatch")
    seen: set[str] = set()
    for job, record in zip(jobs, records, strict=True):
        asset_id = str(record.get("asset_id", ""))
        if asset_id in seen or asset_id != job["asset_id"]:
            raise AutomationError(f"{item['display']} record identity/order mismatch at {job['selection_index']}")
        seen.add(asset_id)
        if int(record.get("selection_index", -1)) != job["selection_index"]:
            raise AutomationError(f"{item['display']} record selection index mismatch")
        if record.get("expected_primary_urdf_sha256") not in {None, job["expected_primary_urdf_sha256"]}:
            raise AutomationError(f"{item['display']} record URDF hash binding mismatch")
        if record.get("expected_movable_joints") not in {None, job["expected_movable_joints"]}:
            raise AutomationError(f"{item['display']} record joint binding mismatch")
    _verify_children(output, records, jobs)
    aggregate = _aggregate(records, n_eval, j_eval)
    summary_path = output / "summary.json"
    if not summary_path.is_file():
        raise AutomationError(f"missing summary: {item['display']}")
    summary = _json(summary_path)
    if not isinstance(summary, Mapping):
        raise AutomationError(f"summary is not an object: {item['display']}")
    _check_self_hash(
        summary,
        f"{item['display']} summary",
        field="summary_content_sha256",
        required=True,
    )
    if summary.get("records_sha256") and _sha(records_path) != summary["records_sha256"]:
        raise AutomationError(f"{item['display']} summary records hash mismatch")
    if summary.get("manifest_content_sha256") != manifest.get("manifest_content_sha256"):
        raise AutomationError(f"{item['display']} summary manifest binding mismatch")
    if summary.get("status_counts") != aggregate["status_counts"]:
        raise AutomationError(f"{item['display']} summary status_counts mismatch")
    published = summary.get("metrics", {})
    if not isinstance(published, Mapping):
        raise AutomationError(f"{item['display']} summary metrics missing")
    for name, expected in aggregate["metrics"].items():
        observed = published.get(name)
        if not isinstance(observed, Mapping):
            raise AutomationError(f"{item['display']} summary metric missing: {name}")
        if name == "visual_bearing_collision_coverage":
            observed = observed.get("asset", observed)
        if expected.get("passed") is not None and _number(observed.get("passed", observed.get("numerator")), label=name) != expected["passed"]:
            raise AutomationError(f"{item['display']} summary metric mismatch: {name}")
        if expected.get("denominator") is not None and _number(observed.get("denominator"), label=name) != expected["denominator"]:
            raise AutomationError(f"{item['display']} summary metric denominator mismatch: {name}")
    checkpoint_path = output / "checkpoint.json"
    if not checkpoint_path.is_file():
        raise AutomationError(f"missing checkpoint: {item['display']}")
    checkpoint = _json(checkpoint_path)
    _check_self_hash(
        checkpoint,
        f"{item['display']} checkpoint",
        field="checkpoint_content_sha256",
        required=True,
    )
    if checkpoint.get("state") != "complete" or _number(checkpoint.get("records"), label="checkpoint records") != n_eval:
        raise AutomationError(f"{item['display']} checkpoint is not complete")
    if checkpoint.get("records_sha256") != _sha(records_path):
        raise AutomationError(f"{item['display']} checkpoint records hash mismatch")
    if checkpoint.get("manifest_content_sha256") != manifest.get("manifest_content_sha256"):
        raise AutomationError(f"{item['display']} checkpoint manifest binding mismatch")
    if checkpoint.get("summary_sha256") and _sha(summary_path) != checkpoint["summary_sha256"]:
        raise AutomationError(f"{item['display']} checkpoint summary hash mismatch")
    _verify_receipt_entry(
        root,
        item,
        entry,
        dataset=dataset,
        n_eval=n_eval,
        j_eval=j_eval,
        summary=summary,
        summary_path=summary_path,
        checkpoint=checkpoint,
    )
    artifact_path = output / "artifact_manifest.json"
    if not artifact_path.is_file():
        raise AutomationError(f"missing artifact manifest: {item['display']}")
    artifact = _json(artifact_path)
    _check_self_hash(
        artifact,
        f"{item['display']} artifact manifest",
        field="artifact_manifest_content_sha256",
        required=True,
    )
    try:
        common.verify_artifacts(output)
    except Exception as exc:  # noqa: BLE001
        raise AutomationError(f"{item['display']} artifact closure failed: {exc}") from exc
    return {
        "dataset": dataset,
        "n_eval": n_eval,
        "j_eval": j_eval,
        "output": str(output),
        "manifest": str(manifest_path),
        "roster": str(roster_path),
        "aggregate": aggregate,
        "source_bindings": bindings,
    }


def _verify_receipt_entry(
    root: Path,
    item: Mapping[str, Any],
    entry: Mapping[str, Any],
    *,
    dataset: Any,
    n_eval: int,
    j_eval: int,
    summary: Mapping[str, Any],
    summary_path: Path,
    checkpoint: Mapping[str, Any],
) -> None:
    """Bind one combined-receipt row to the live publication artifacts."""

    if not entry:
        raise AutomationError(f"combined receipt has no entry: {item['display']}")
    entry_n = _number(
        entry.get("N_eval", entry.get("n_eval")),
        label=f"{item['display']} receipt N_eval",
    )
    entry_j = _number(
        entry.get("J_eval", entry.get("j_eval")),
        label=f"{item['display']} receipt J_eval",
    )
    if (entry_n, entry_j) != (n_eval, j_eval):
        raise AutomationError(f"{item['display']} combined receipt N/J mismatch")

    entry_dataset = entry.get("dataset")
    if not isinstance(entry_dataset, str) or entry_dataset != dataset:
        raise AutomationError(f"{item['display']} combined receipt dataset mismatch")
    entry_display = entry.get("display") or entry.get("display_name")
    allowed_display = DISPLAY_ALIASES.get(item["slug"], {str(item["display"])})
    if not isinstance(entry_display, str) or entry_display not in allowed_display:
        raise AutomationError(f"{item['display']} combined receipt display mismatch")

    if entry.get("status") != "complete" or checkpoint.get("state") != "complete":
        raise AutomationError(f"{item['display']} combined receipt status mismatch")

    entry_metrics = entry.get("metrics")
    live_metrics = summary.get("metrics")
    if not isinstance(entry_metrics, Mapping) or not isinstance(live_metrics, Mapping):
        raise AutomationError(f"{item['display']} combined receipt metrics missing")
    if common.canonical_sha256(entry_metrics) != common.canonical_sha256(live_metrics):
        raise AutomationError(f"{item['display']} combined receipt metrics mismatch")

    evidence = entry.get("evidence")
    summary_ref = evidence.get("summary") if isinstance(evidence, Mapping) else None
    if not isinstance(summary_ref, str) or not summary_ref:
        raise AutomationError(f"{item['display']} combined receipt evidence.summary missing")
    candidate = Path(summary_ref)
    if not candidate.is_absolute():
        candidate = root / candidate
    try:
        candidate = candidate.resolve(strict=True)
    except OSError as exc:
        raise AutomationError(f"{item['display']} combined receipt evidence path is missing") from exc
    if candidate != summary_path.resolve():
        raise AutomationError(f"{item['display']} combined receipt evidence path mismatch")


def _section(markdown: str) -> str:
    match = re.search(r"^#{2,3}\s+Table 2 supplementary\..*$", markdown, re.MULTILINE | re.IGNORECASE)
    if not match:
        raise AutomationError("Markdown is missing Table 2 supplementary heading")
    tail = markdown[match.end():]
    end = re.search(r"^#{2,4}\s+", tail, re.MULTILINE)
    return markdown[match.start(): match.end() + (end.start() if end else len(tail))]


def _table_rows(section: str) -> dict[str, list[str]]:
    rows: dict[str, list[str]] = {}
    for line in section.splitlines():
        stripped = line.strip()
        if not stripped.startswith("|") or re.match(r"\|\s*:?-{2,}", stripped):
            continue
        cells = [cell.strip() for cell in stripped.strip("|").split("|")]
        if cells and cells[0] and not cells[0].lower().startswith("dataset"):
            rows[cells[0]] = cells
    return rows


def _denominator_in_cell(cell: str, denominator: int) -> bool:
    return re.search(rf"/\s*{denominator:,}(?:\b|\s|\))", cell) is not None or re.search(rf"/\s*{denominator}(?:\b|\s|\))", cell) is not None


def _fraction_in_cell(cell: str) -> tuple[int, int] | None:
    """Extract a human-readable ``numerator / denominator`` pair."""

    match = re.search(
        r"(?<![\w=])([0-9][0-9,]*)\s*/\s*([0-9][0-9,]*)(?![\w])",
        cell,
    )
    if not match:
        return None
    return int(match.group(1).replace(",", "")), int(match.group(2).replace(",", ""))


def _percent_in_cell(cell: str) -> float | None:
    match = re.search(r"(?<![\w.])([0-9]+(?:\.[0-9]+)?)\s*%", cell)
    if not match:
        return None
    return float(match.group(1))


def _live_metric_map(result: Mapping[str, Any]) -> Mapping[str, Any]:
    aggregate = result.get("aggregate") if isinstance(result, Mapping) else None
    metrics = aggregate.get("metrics") if isinstance(aggregate, Mapping) else None
    return metrics if isinstance(metrics, Mapping) else {}


def validate_supplementary_markdown(markdown_path: Path, results: Mapping[str, Mapping[str, Any]], *, enforce_ours_baseline: bool = True) -> dict[str, Any]:
    text = Path(markdown_path).read_text(encoding="utf-8")
    section = _section(text)
    rows = _table_rows(section)
    missing_results = [item for item in DATASETS if item["slug"] not in results]
    if missing_results:
        target = next((item for item in missing_results if item["slug"] == "infinigen"), missing_results[0])
        raise AutomationError(f"Markdown supplementary table is missing comparison row: {target['display']}")
    for item in DATASETS:
        row = rows.get(item["display"])
        if row is None:
            raise AutomationError(f"Markdown supplementary table is missing comparison row: {item['display']}")
        joined = " | ".join(row)
        if re.search(r"(?:\bn\s*=\s*800\b|\bj\s*=\s*800\b|/\s*800\b)", joined, re.IGNORECASE):
            raise AutomationError(f"comparison row contains historical N=800: {item['display']}")
        if len(row) < 5:
            raise AutomationError(f"supplementary row has too few cells: {item['display']}")
        metrics = _live_metric_map(results[item["slug"]])
        visual = metrics.get("visual_bearing_collision_coverage", {})
        if isinstance(visual, Mapping):
            visual = visual.get("asset", visual)
        portability = metrics.get("joint_limit_portability", {})
        dynamics = metrics.get("joint_dynamics_coverage", {})
        expected_fractions = [
            (
                visual.get("passed", visual.get("numerator")),
                visual.get("denominator"),
            )
            if isinstance(visual, Mapping)
            else (None, None),
            (
                portability.get("passed", portability.get("numerator")),
                portability.get("denominator"),
            )
            if isinstance(portability, Mapping)
            else (None, None),
            (
                dynamics.get("passed", dynamics.get("numerator")),
                dynamics.get("denominator"),
            )
            if isinstance(dynamics, Mapping)
            else (None, None),
        ]
        for cell, (numerator, denominator) in zip(row[1:4], expected_fractions, strict=True):
            if numerator is None or denominator is None:
                raise AutomationError(f"live metric numerator/denominator missing: {item['display']}")
            expected_pair = (_number(numerator, label="live metric numerator"), _number(denominator, label="live metric denominator"))
            observed_pair = _fraction_in_cell(cell)
            if observed_pair is None:
                raise AutomationError(f"Markdown metric fraction missing: {item['display']}")
            if observed_pair != expected_pair:
                raise AutomationError(
                    f"Markdown metric numerator/denominator mismatch: {item['display']} "
                    f"({observed_pair[0]}/{observed_pair[1]} != {expected_pair[0]}/{expected_pair[1]})"
                )
            observed_percent = _percent_in_cell(cell)
            if observed_percent is None:
                raise AutomationError(f"Markdown metric percentage missing: {item['display']}")
            expected_percent = round(100.0 * expected_pair[0] / expected_pair[1], 2) if expected_pair[1] else 0.0
            if abs(observed_percent - expected_percent) > 0.005:
                raise AutomationError(
                    f"Markdown metric percentage mismatch: {item['display']} "
                    f"({observed_percent:.2f}% != {expected_percent:.2f}%)"
                )
        if not re.search(r"\bN\s*/\s*E\b|\bN/E\b", row[4], re.IGNORECASE):
            raise AutomationError(f"Markdown placeholder incidence must be N/E: {item['display']}")
    if enforce_ours_baseline:
        for label in ("Ours-500K", "Ours per-class N=5 (supplementary)"):
            if label not in rows:
                raise AutomationError(f"Markdown supplementary table is missing preserved row: {label}")
    return {"comparison_rows": len(DATASETS), "ours_rows_present": sum(label in rows for label in ("Ours-500K", "Ours per-class N=5 (supplementary)"))}


def _receipt_entries(root: Path) -> dict[str, Mapping[str, Any]]:
    path = root / "full_release_receipt.json"
    if not path.is_file():
        raise AutomationError("missing combined full_release_receipt.json")
    receipt = _json(path)
    if not isinstance(receipt, Mapping):
        raise AutomationError("combined receipt is not an object")
    _check_self_hash(
        receipt,
        "combined receipt",
        field="receipt_content_sha256",
        required=True,
    )
    declared_root = receipt.get("root")
    if not isinstance(declared_root, str) or Path(declared_root).resolve() != root.resolve():
        raise AutomationError("combined receipt root mismatch")
    raw = receipt.get("methods", receipt.get("datasets"))
    if isinstance(raw, Mapping):
        raw = list(raw.values())
    if not isinstance(raw, list):
        raise AutomationError("combined receipt has no methods/datasets list")
    entries: dict[str, Mapping[str, Any]] = {}
    for value in raw:
        if not isinstance(value, Mapping):
            raise AutomationError("combined receipt entry is not an object")
        slug = str(value.get("slug") or value.get("dataset_id") or "").lower()
        if slug in entries or slug not in DATASET_BY_SLUG:
            raise AutomationError(f"invalid or duplicate combined receipt slug: {slug}")
        entries[slug] = value
    if set(entries) != set(DATASET_BY_SLUG):
        raise AutomationError("combined receipt does not contain exactly eight datasets")
    return entries


def run_checks(root: Path, markdown: Path, *, source_mode: bool | str = "auto", run_pytest: bool = False, repo_root: Path | None = None) -> dict[str, Any]:
    root, markdown = Path(root).resolve(), Path(markdown).resolve()
    errors: list[str] = []
    live: dict[str, dict[str, Any]] = {}
    entries: dict[str, Mapping[str, Any]] = {}
    try:
        entries = _receipt_entries(root)
    except AutomationError as exc:
        errors.append(str(exc))
    for item in DATASETS:
        try:
            live[item["slug"]] = _live_dataset(root, item, source_mode, entries.get(item["slug"]))
        except AutomationError as exc:
            errors.append(str(exc))
    markdown_result: dict[str, Any] = {}
    if not errors:
        try:
            markdown_result = validate_supplementary_markdown(markdown, live)
        except (AutomationError, OSError) as exc:
            errors.append(str(exc))
    pytest_result = None
    if run_pytest:
        repo = repo_root or Path(__file__).resolve().parents[2]
        env = dict(os.environ)
        env["PYTHONPATH"] = str(repo / "exp" / "scripts") + os.pathsep + env.get("PYTHONPATH", "")
        command = [sys.executable, "-m", "pytest", "-q", "exp/tests/test_table2sup_full_release.py", "exp/tests/test_render_table2sup_full_release_results.py", "exp/tests/test_run_table2sup_full_release_smoke.py"]
        proc = subprocess.run(command, cwd=repo, text=True, capture_output=True, env=env, check=False)
        pytest_result = {"returncode": proc.returncode, "passed": proc.returncode == 0, "stdout_tail": proc.stdout[-4000:], "stderr_tail": proc.stderr[-4000:]}
        if proc.returncode:
            errors.append("focused contract pytest failed")
    return {
        "schema_version": "table2sup_full_release_automation_check_v2",
        "root": str(root),
        "markdown_path": str(markdown),
        "dataset_count": len(live),
        "datasets": live,
        "markdown": markdown_result,
        "pytest": pytest_result,
        "errors": errors,
        "all_pass": not errors and len(live) == len(DATASETS),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    parser.add_argument("--source-mode", choices=("auto", "strict", "none"), default="auto")
    parser.add_argument("--pytest", action="store_true")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args(argv)
    mode: bool | str = {"strict": True, "none": False}.get(args.source_mode, "auto")
    report = run_checks(args.root, args.markdown, source_mode=mode, run_pytest=args.pytest)
    payload = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if report["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
