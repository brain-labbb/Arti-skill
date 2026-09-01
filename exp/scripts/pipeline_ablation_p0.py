#!/usr/bin/env python3
"""Fail-closed P0 preparation for the pipeline ablation.

This tool performs no model calls.  It has two independent jobs:

* classify candidate tasks from local development evidence without ever
  declaring a task fresh; and
* project a strict SourceMap and TemplateDesign into isolated S/D factors and
  four author-visible packets after dependency and leakage checks.

Hashes and validation diagnostics belong to the private audit.  They are not
included in author-visible packets because a withheld-factor hash is itself a
side channel.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib
import json
import os
import re
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable, Iterator


SCHEMA_VERSION = 1
CONTRACT_PATH = Path(__file__).resolve().parents[1] / "reference" / "pipeline_ablation_p0_contract_v1.json"
FRESHNESS_STATUSES = (
    "development_only",
    "no_local_conflict_found",
    "indeterminate",
)
ARM_FACTORS = {
    "A00": (),
    "A10": ("S_factor",),
    "A01": ("D_factor",),
    "A11": ("S_factor", "D_factor"),
}
TEXT_SUFFIXES = {
    ".json",
    ".jsonl",
    ".log",
    ".md",
    ".py",
    ".rst",
    ".toml",
    ".txt",
    ".yaml",
    ".yml",
}
SKIP_DIRECTORY_NAMES = {
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".venv",
    "__pycache__",
    "node_modules",
    "pbr_material_library",
}
CONTENT_SKIP_DIRECTORY_NAMES = {
    "artifacts",
    "assets",
    "frames",
    "meshes",
    "renders",
    "revisions",
}
RAW_SOURCE_DIRECTORY_NAMES = {"records"}
SLUG_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_]*\Z")
REFERENCE_PATTERN = re.compile(
    r"(?:rec_[A-Za-z0-9_.-]+|rev_\d+|model\.py:L\d+\s*-\s*L\d+|"
    r"(?:^|[/\\])records(?:[/\\]|$))",
    re.IGNORECASE,
)
D_FORBIDDEN_KEYS = {
    "evidence",
    "implementation_function",
    "input_paths",
    "model_path",
    "record_id",
    "revision",
    "row_index",
    "source_map_path",
    "source_map_sha256",
    "source_ref",
    "source_spans",
}
S_FORBIDDEN_KEYS = {
    "assembly_notes",
    "bindings",
    "category_anchors",
    "interfaces",
    "multiplicities",
    "parameters",
    "slots",
}
COMMON_FORBIDDEN_KEYS = {
    "diagnostics",
    "factor_hashes",
    "findings",
    "gold",
    "hidden",
    "input_hashes",
    "source_map",
    "source_map_sha256",
    "template_design",
}


class ContractError(ValueError):
    """A packet cannot be released under the P0 contract."""


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode(
        "utf-8"
    )


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    return sha256_bytes(canonical_bytes(value))


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def display_path(path: Path, project_root: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(project_root.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def contract_info() -> dict[str, str]:
    return {
        "path": CONTRACT_PATH.as_posix(),
        "sha256": sha256_file(CONTRACT_PATH),
    }


def _task_rows(payload: Any) -> list[dict[str, Any]]:
    if isinstance(payload, list):
        return [row for row in payload if isinstance(row, dict)]
    if not isinstance(payload, dict):
        return []
    rows = payload.get("tasks")
    if isinstance(rows, list):
        return [row for row in rows if isinstance(row, dict)]
    if isinstance(payload.get("selected_task_slugs"), list):
        return [{"slug": item} for item in payload["selected_task_slugs"]]
    if isinstance(payload.get("slug"), str):
        return [payload]
    return []


def load_candidates(manifests: Iterable[Path], slugs: Iterable[str] = ()) -> tuple[list[dict[str, Any]], list[Path]]:
    by_slug: dict[str, dict[str, Any]] = {}
    paths: list[Path] = []
    for path in manifests:
        path = path.resolve()
        payload = read_json(path)
        paths.append(path)
        for row in _task_rows(payload):
            slug = str(row.get("slug") or "").strip()
            entry = by_slug.setdefault(slug, {"slug": slug, "identifiers": set(), "declared_in": set()})
            entry["declared_in"].add(path.as_posix())
            for record in row.get("records") or ():
                if isinstance(record, dict) and isinstance(record.get("record_id"), str):
                    entry["identifiers"].add(record["record_id"])
            for key in ("identifiers", "aliases"):
                for value in row.get(key) or ():
                    if isinstance(value, str) and value.strip():
                        entry["identifiers"].add(value.strip())
    for slug in slugs:
        slug = slug.strip()
        by_slug.setdefault(slug, {"slug": slug, "identifiers": set(), "declared_in": set()})
    result: list[dict[str, Any]] = []
    for slug in sorted(by_slug):
        row = by_slug[slug]
        result.append(
            {
                "slug": slug,
                "identifiers": sorted(row["identifiers"]),
                "declared_in": sorted(row["declared_in"]),
            }
        )
    return result, sorted(set(paths), key=lambda item: item.as_posix())


def default_scan_scopes(project_root: Path) -> list[tuple[str, Path]]:
    candidates = [
        ("legacy_authoring_pilot", project_root / "exp/t2_authoring_pilot"),
        ("production_templates", project_root / "arti-template/agent/templates"),
        ("authoring_metadata", project_root / "arti-template/articraft_template_authoring"),
        ("root_template_maps", project_root / "template_maps"),
        ("root_template_source_maps", project_root / "template_source_maps"),
        ("experiment_reference", project_root / "exp/reference"),
        ("experiment_scripts", project_root / "exp/scripts"),
        ("experiment_runtime", project_root / "exp/runtime"),
    ]
    exp_root = project_root / "exp"
    if exp_root.is_dir():
        candidates.extend(
            (f"experiment_document:{path.name}", path)
            for path in sorted(exp_root.iterdir())
            if path.is_file() and path.suffix.casefold() in TEXT_SUFFIXES
        )
    return [(name, path) for name, path in candidates if path.exists()]


def _classify_evidence(path: Path) -> str:
    lowered = [part.casefold() for part in path.parts]
    name = path.name.casefold()
    if "agent" in lowered and "templates" in lowered and path.suffix == ".py":
        return "target_template"
    if "source_maps" in lowered or "template_source_maps" in lowered or "picture_source_maps" in lowered:
        return "source_map"
    if "designs" in lowered or "design_scaffolds" in lowered or "template_maps" in lowered:
        return "template_design"
    if any(token in name for token in ("first_shot", "repair_", "run_result", "packet")) or "output" in lowered:
        return "authoring_output_or_repair"
    if any(token in lowered for token in ("llm_logs", "design_logs", "traces", "preparation")):
        return "preparation_or_runtime_log"
    if any(token in name for token in ("manifest", "protocol", "report", "summary", "records")):
        return "protocol_or_report"
    return "prior_reference"


def _walk_scope(root: Path) -> Iterator[Path]:
    if root.is_file():
        yield root
        return
    for current, directories, files in os.walk(root, followlinks=False):
        directories[:] = sorted(
            name
            for name in directories
            if name not in SKIP_DIRECTORY_NAMES
            and name not in CONTENT_SKIP_DIRECTORY_NAMES
            and name not in RAW_SOURCE_DIRECTORY_NAMES
        )
        base = Path(current)
        for name in sorted(files):
            yield base / name


def _token_regex(tokens: Iterable[str]) -> re.Pattern[str] | None:
    values = sorted({token for token in tokens if token}, key=lambda item: (-len(item), item.casefold()))
    if not values:
        return None
    return re.compile(
        r"(?<![A-Za-z0-9_])(?:"
        + "|".join(re.escape(item) for item in values)
        + r")(?=__|[^A-Za-z0-9_]|$)",
        re.IGNORECASE,
    )


def scan_freshness(
    *,
    project_root: Path,
    candidates: list[dict[str, Any]],
    manifest_paths: Iterable[Path] = (),
    scopes: list[tuple[str, Path]] | None = None,
    max_evidence_per_candidate: int = 100,
) -> dict[str, Any]:
    """Scan local development evidence; never return a `fresh` label."""

    project_root = project_root.resolve()
    scopes = scopes if scopes is not None else default_scan_scopes(project_root)
    excluded = {path.resolve() for path in manifest_paths}
    errors: list[str] = []
    by_slug: dict[str, list[dict[str, Any]]] = {row["slug"]: [] for row in candidates}
    truncated: dict[str, bool] = {row["slug"]: False for row in candidates}
    token_owner: dict[str, set[str]] = defaultdict(set)
    invalid_slugs: set[str] = set()
    for row in candidates:
        slug = row["slug"]
        if not SLUG_PATTERN.fullmatch(slug):
            invalid_slugs.add(slug)
            continue
        token_owner[slug.casefold()].add(slug)
        for identifier in row.get("identifiers") or ():
            token_owner[str(identifier).casefold()].add(slug)
    matcher = _token_regex(token_owner)
    evidence_seen: set[tuple[str, str, str, int | None, str]] = set()
    scanned_files = 0
    scanned_text_files = 0
    scanned_scopes: list[dict[str, Any]] = []

    def add_evidence(slug: str, path: Path, match_kind: str, line: int | None, token: str) -> None:
        rel = display_path(path, project_root)
        kind = _classify_evidence(path)
        key = (slug, rel, match_kind, line, kind)
        if key in evidence_seen:
            return
        evidence_seen.add(key)
        if len(by_slug[slug]) >= max_evidence_per_candidate:
            truncated[slug] = True
            return
        by_slug[slug].append(
            {
                "evidence_kind": kind,
                "match_kind": match_kind,
                "path": rel,
                "line": line,
                "matched_identifier": token,
            }
        )

    for scope_name, raw_root in scopes:
        root = raw_root.resolve()
        if not root.exists():
            errors.append(f"scope_missing:{scope_name}:{root.as_posix()}")
            continue
        scope_file_count = 0
        for path in _walk_scope(root):
            resolved = path.resolve()
            if resolved in excluded:
                continue
            scanned_files += 1
            scope_file_count += 1
            path_matcher = matcher.search(display_path(path, project_root)) if matcher else None
            if path_matcher:
                token = path_matcher.group(0)
                for slug in sorted(token_owner[token.casefold()]):
                    add_evidence(slug, path, "path", None, token)
            if path.suffix.casefold() not in TEXT_SUFFIXES:
                continue
            if CONTENT_SKIP_DIRECTORY_NAMES.intersection(part.casefold() for part in path.parts):
                continue
            try:
                with path.open("r", encoding="utf-8", errors="replace") as handle:
                    scanned_text_files += 1
                    for line_number, line_text in enumerate(handle, 1):
                        if not matcher:
                            continue
                        for match in matcher.finditer(line_text):
                            token = match.group(0)
                            for slug in sorted(token_owner[token.casefold()]):
                                add_evidence(slug, path, "content", line_number, token)
            except OSError as exc:
                errors.append(f"read_error:{display_path(path, project_root)}:{type(exc).__name__}")
        scanned_scopes.append(
            {"name": scope_name, "path": display_path(root, project_root), "file_count": scope_file_count}
        )

    rows: list[dict[str, Any]] = []
    for candidate in sorted(candidates, key=lambda item: item["slug"]):
        slug = candidate["slug"]
        evidence = sorted(
            by_slug.get(slug, []),
            key=lambda item: (
                item["path"],
                item["line"] is None,
                item["line"] or 0,
                item["match_kind"],
                item["evidence_kind"],
            ),
        )
        if slug in invalid_slugs:
            status = "indeterminate"
            reasons = ["invalid_slug"]
        elif evidence:
            status = "development_only"
            reasons = sorted({item["evidence_kind"] for item in evidence})
        elif errors:
            status = "indeterminate"
            reasons = ["scan_incomplete"]
        else:
            status = "no_local_conflict_found"
            reasons = ["no_match_in_scanned_local_scopes"]
        rows.append(
            {
                "slug": slug,
                "status": status,
                "reasons": reasons,
                "evidence_count_retained": len(evidence),
                "evidence_truncated": truncated.get(slug, False),
                "evidence": evidence,
                "claim_boundary": (
                    "Absence of a local match is not proof of freshness and does not authorize cohort selection."
                ),
            }
        )

    policy = {
        "schema_version": SCHEMA_VERSION,
        "statuses": list(FRESHNESS_STATUSES),
        "raw_source_pool_is_not_scanned_as_prior_authoring_evidence": True,
        "candidate_manifests_are_excluded_from_self-matching": True,
        "no_status_named_fresh": True,
        "text_suffixes": sorted(TEXT_SUFFIXES),
        "skip_directory_names": sorted(SKIP_DIRECTORY_NAMES),
        "content_skip_directory_names": sorted(CONTENT_SKIP_DIRECTORY_NAMES),
        "raw_source_directory_names_excluded_from_prior_evidence": sorted(
            RAW_SOURCE_DIRECTORY_NAMES
        ),
        "max_evidence_per_candidate": max_evidence_per_candidate,
    }
    counts = {status: sum(row["status"] == status for row in rows) for status in FRESHNESS_STATUSES}
    return {
        "schema_version": SCHEMA_VERSION,
        "report_kind": "pipeline_ablation_local_freshness_audit",
        "contract": contract_info(),
        "scanner_policy": policy,
        "scanner_policy_sha256": canonical_sha256(policy),
        "project_root": project_root.as_posix(),
        "excluded_candidate_manifests": [display_path(path, project_root) for path in sorted(excluded)],
        "scanned_scopes": scanned_scopes,
        "scanned_file_count": scanned_files,
        "scanned_text_file_count": scanned_text_files,
        "scan_errors": sorted(set(errors)),
        "status_counts": counts,
        "tasks": rows,
        "selection_authorized": False,
        "claim_boundary": "This audit can disqualify development tasks; it cannot certify a task as fresh.",
    }


def _load_authoring_modules(project_root: Path) -> tuple[Any, Any]:
    template_root = (project_root / "arti-template").resolve()
    if not template_root.is_dir():
        raise ContractError(f"arti-template repository is unavailable: {template_root}")
    text = template_root.as_posix()
    if text not in sys.path:
        sys.path.insert(0, text)
    source_maps = importlib.import_module("agent.source_maps")
    template_design = importlib.import_module("agent.template_design")
    return source_maps, template_design


def _copy_keys(payload: dict[str, Any], keys: Iterable[str]) -> dict[str, Any]:
    return {key: copy.deepcopy(payload[key]) for key in keys if key in payload}


def build_s_factor(resolution: Any, raw_design: dict[str, Any], project_root: Path) -> dict[str, Any]:
    pool = {record.record_id: record for record in resolution.source_pool}
    reviews: list[dict[str, Any]] = []
    for review in resolution.reviews:
        record = pool.get(review.record_id)
        reviews.append(
            {
                "record_id": review.record_id,
                "revision": review.revision,
                "source_ref": review.source_ref,
                "review_status": review.review_status,
                "decision": review.decision,
                "note": review.note,
                "model_path": display_path(record.model_path, project_root) if record else None,
                "input_paths": [display_path(path, project_root) for path in (record.input_paths if record else ())],
            }
        )
    candidates: list[dict[str, Any]] = []
    for candidate in resolution.candidates:
        record = pool.get(candidate.record_id)
        candidates.append(
            {
                "slot": candidate.slot,
                "candidate": candidate.candidate,
                "component_type": candidate.component_type,
                "record_id": candidate.record_id,
                "revision": candidate.revision,
                "source_ref": (
                    f"{candidate.record_id}/{candidate.revision}"
                    if candidate.record_id and candidate.revision
                    else None
                ),
                "source_spans": [span.code_ref for span in candidate.spans],
                "distinction": candidate.distinction,
                "evidence": candidate.evidence,
                "model_path": display_path(record.model_path, project_root) if record else None,
            }
        )
    design_candidates = _candidate_map_from_design(raw_design)
    ownership: list[dict[str, Any]] = []
    for candidate in resolution.candidates:
        native = design_candidates.get((candidate.slot, candidate.candidate), {})
        ownership.append(
            {
                "slot": candidate.slot,
                "candidate": candidate.candidate,
                "record_id": candidate.record_id,
                "revision": candidate.revision,
                "source_spans": [span.code_ref for span in candidate.spans],
                "implementation_function": native.get("implementation_function"),
            }
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "factor_kind": "S_factor",
        "source_map": {
            "path": display_path(resolution.path, project_root),
            "export_category": resolution.export_category,
            "picture_category": resolution.picture_category,
            "picture_subcategory": resolution.picture_subcategory,
            "category_scope": resolution.category_scope,
            "reviews": reviews,
            "candidates": candidates,
            "source_to_owner_functions": ownership,
        },
    }


def build_d_factor(raw_design: dict[str, Any]) -> dict[str, Any]:
    slots: list[dict[str, Any]] = []
    for slot in raw_design.get("slots") or ():
        candidates: list[dict[str, Any]] = []
        for candidate in slot.get("candidates") or ():
            candidates.append(_copy_keys(candidate, ("name", "parameters", "interfaces", "notes")))
        projected = _copy_keys(slot, ("name", "component_type", "required"))
        projected["candidates"] = candidates
        slots.append(projected)
    return {
        "schema_version": SCHEMA_VERSION,
        "factor_kind": "D_factor",
        "slug": raw_design.get("slug"),
        "slots": slots,
        "multiplicities": copy.deepcopy(raw_design.get("multiplicities") or []),
        "bindings": copy.deepcopy(raw_design.get("bindings") or []),
        "category_anchors": copy.deepcopy(raw_design.get("category_anchors") or []),
        "assembly_notes": raw_design.get("assembly_notes") or "",
    }


def walk_json(value: Any, pointer: str = "") -> Iterator[tuple[str, str | None, Any]]:
    if isinstance(value, dict):
        for key in sorted(value):
            child = f"{pointer}/{key}"
            yield child, key, value[key]
            yield from walk_json(value[key], child)
    elif isinstance(value, list):
        for index, item in enumerate(value):
            yield from walk_json(item, f"{pointer}/{index}")


def _forbidden_key_findings(value: Any, forbidden: set[str], label: str) -> list[str]:
    return [
        f"{label}_forbidden_key:{pointer}"
        for pointer, key, _ in walk_json(value)
        if key is not None and key.casefold() in forbidden
    ]


def _sensitive_tokens(s_factor: dict[str, Any]) -> list[str]:
    sensitive_keys = {
        "input_paths",
        "implementation_function",
        "model_path",
        "path",
        "record_id",
        "revision",
        "source_ref",
        "source_spans",
    }
    tokens: set[str] = set()
    for _, key, value in walk_json(s_factor):
        if key not in sensitive_keys:
            continue
        values = value if isinstance(value, list) else [value]
        for item in values:
            if isinstance(item, str) and item:
                tokens.add(item)
    return sorted(tokens, key=lambda item: (-len(item), item))


def leakage_findings(s_factor: dict[str, Any], d_factor: dict[str, Any]) -> list[str]:
    findings = _forbidden_key_findings(s_factor, S_FORBIDDEN_KEYS, "S")
    findings.extend(_forbidden_key_findings(d_factor, D_FORBIDDEN_KEYS, "D"))
    tokens = _sensitive_tokens(s_factor)
    for pointer, _, value in walk_json(d_factor):
        if not isinstance(value, str):
            continue
        for token in tokens:
            if token.casefold() in value.casefold():
                findings.append(f"D_sensitive_token:{pointer}:{token}")
                break
        if REFERENCE_PATTERN.search(value):
            findings.append(f"D_source_reference_pattern:{pointer}")
    return sorted(set(findings))


def schema_findings(definition: str, value: Any) -> list[str]:
    schema = read_json(CONTRACT_PATH)
    definitions = schema["$defs"]
    root = definitions.get(definition)
    if not isinstance(root, dict):
        return [f"schema:{definition}:/:definition_missing"]

    def resolve(node: dict[str, Any]) -> dict[str, Any]:
        reference = node.get("$ref")
        if not isinstance(reference, str):
            return node
        prefix = "#/$defs/"
        if not reference.startswith(prefix) or reference[len(prefix) :] not in definitions:
            return {"__invalid_reference__": reference}
        return definitions[reference[len(prefix) :]]

    def type_ok(expected: str, item: Any) -> bool:
        checks = {
            "array": lambda: isinstance(item, list),
            "boolean": lambda: isinstance(item, bool),
            "integer": lambda: isinstance(item, int) and not isinstance(item, bool),
            "null": lambda: item is None,
            "number": lambda: isinstance(item, (int, float)) and not isinstance(item, bool),
            "object": lambda: isinstance(item, dict),
            "string": lambda: isinstance(item, str),
        }
        return expected in checks and checks[expected]()

    findings: list[str] = []

    def validate(node: dict[str, Any], item: Any, pointer: str) -> None:
        node = resolve(node)
        if "__invalid_reference__" in node:
            findings.append(f"schema:{definition}:{pointer}:invalid_ref:{node['__invalid_reference__']}")
            return
        if "oneOf" in node:
            alternatives = node["oneOf"]
            successes = 0
            for alternative in alternatives:
                before = len(findings)
                validate(alternative, item, pointer)
                if len(findings) == before:
                    successes += 1
                else:
                    del findings[before:]
            if successes != 1:
                findings.append(f"schema:{definition}:{pointer}:oneOf")
            return
        expected = node.get("type")
        if isinstance(expected, list):
            if not any(type_ok(name, item) for name in expected):
                findings.append(f"schema:{definition}:{pointer}:type")
                return
        elif isinstance(expected, str) and not type_ok(expected, item):
            findings.append(f"schema:{definition}:{pointer}:type")
            return
        if "const" in node and item != node["const"]:
            findings.append(f"schema:{definition}:{pointer}:const")
        if "enum" in node and item not in node["enum"]:
            findings.append(f"schema:{definition}:{pointer}:enum")
        if isinstance(item, str):
            if len(item) < int(node.get("minLength", 0)):
                findings.append(f"schema:{definition}:{pointer}:minLength")
            if "pattern" in node and re.search(node["pattern"], item) is None:
                findings.append(f"schema:{definition}:{pointer}:pattern")
        if isinstance(item, (int, float)) and not isinstance(item, bool):
            if "minimum" in node and item < node["minimum"]:
                findings.append(f"schema:{definition}:{pointer}:minimum")
        if isinstance(item, list):
            if len(item) < int(node.get("minItems", 0)):
                findings.append(f"schema:{definition}:{pointer}:minItems")
            if "maxItems" in node and len(item) > int(node["maxItems"]):
                findings.append(f"schema:{definition}:{pointer}:maxItems")
            if node.get("uniqueItems") and len({canonical_bytes(entry) for entry in item}) != len(item):
                findings.append(f"schema:{definition}:{pointer}:uniqueItems")
            child_schema = node.get("items")
            if isinstance(child_schema, dict):
                for index, child in enumerate(item):
                    validate(child_schema, child, f"{pointer}/{index}")
        if isinstance(item, dict):
            required = set(node.get("required") or ())
            for missing in sorted(required - set(item)):
                findings.append(f"schema:{definition}:{pointer}/{missing}:required")
            properties = node.get("properties") or {}
            if node.get("additionalProperties") is False:
                for extra in sorted(set(item) - set(properties)):
                    findings.append(f"schema:{definition}:{pointer}/{extra}:additionalProperties")
            for key in sorted(set(item) & set(properties)):
                validate(properties[key], item[key], f"{pointer}/{key}")

    validate(root, value, "")
    return sorted(set(findings))


def common_packet_findings(common: dict[str, Any], slug: str) -> list[str]:
    findings = schema_findings("common_authoring_packet", common)
    if ((common.get("task") or {}).get("slug")) != slug:
        findings.append("common_task_slug_mismatch")
    for pointer, key, _ in walk_json(common):
        if key is None:
            continue
        lowered = key.casefold()
        if lowered in COMMON_FORBIDDEN_KEYS or "sha256" in lowered or lowered.endswith("_hash"):
            findings.append(f"common_forbidden_key:{pointer}")
    return sorted(set(findings))


def _candidate_map_from_source(resolution: Any) -> dict[tuple[str, str], Any]:
    return {(candidate.slot, candidate.candidate): candidate for candidate in resolution.candidates}


def _candidate_map_from_design(raw: dict[str, Any]) -> dict[tuple[str, str], dict[str, Any]]:
    return {
        (str(slot.get("name") or ""), str(candidate.get("name") or "")): candidate
        for slot in raw.get("slots") or ()
        for candidate in slot.get("candidates") or ()
    }


def factor_alignment_findings(resolution: Any, raw_design: dict[str, Any], source_map: Path) -> list[str]:
    findings: list[str] = []
    if raw_design.get("slug") != resolution.export_category:
        findings.append("slug_export_category_mismatch")
    if raw_design.get("source_map_sha256") != resolution.sha256:
        findings.append("design_source_map_sha256_mismatch")
    raw_path = str(raw_design.get("source_map_path") or "")
    if not raw_path:
        findings.append("design_source_map_path_missing")
    else:
        candidate_path = Path(raw_path)
        if not candidate_path.is_absolute():
            candidate_path = source_map.parent / candidate_path
        if candidate_path.resolve() != source_map.resolve():
            findings.append("design_source_map_path_mismatch")
    source = _candidate_map_from_source(resolution)
    design = _candidate_map_from_design(raw_design)
    for key in sorted(source.keys() - design.keys()):
        findings.append(f"design_candidate_missing:{key[0]}.{key[1]}")
    for key in sorted(design.keys() - source.keys()):
        findings.append(f"design_candidate_outside_source_map:{key[0]}.{key[1]}")
    for key in sorted(source.keys() & design.keys()):
        source_candidate = source[key]
        design_candidate = design[key]
        expected = {
            "record_id": source_candidate.record_id,
            "revision": source_candidate.revision,
            "source_spans": [span.code_ref for span in source_candidate.spans],
            "evidence": source_candidate.evidence,
        }
        for field, value in expected.items():
            actual = design_candidate.get(field)
            if field == "source_spans":
                actual = list(actual or ())
            if actual != value:
                findings.append(f"candidate_provenance_mismatch:{key[0]}.{key[1]}:{field}")
    return findings


def normalized_native_author_information(raw_design: dict[str, Any]) -> dict[str, Any]:
    """Normalize native Design into the information assigned to S or D.

    Integrity-only top-level SourceMap path/hash are deliberately excluded.
    Candidate provenance and implementation ownership are assigned to S; all
    semantic design fields are assigned to D.
    """

    source_candidates: list[dict[str, Any]] = []
    ownership: list[dict[str, Any]] = []
    for slot in raw_design.get("slots") or ():
        for candidate in slot.get("candidates") or ():
            source_candidates.append(
                {
                    "slot": slot.get("name"),
                    "candidate": candidate.get("name"),
                    "record_id": candidate.get("record_id"),
                    "revision": candidate.get("revision"),
                    "source_spans": list(candidate.get("source_spans") or ()),
                    "evidence": candidate.get("evidence") or "",
                }
            )
            ownership.append(
                {
                    "slot": slot.get("name"),
                    "candidate": candidate.get("name"),
                    "record_id": candidate.get("record_id"),
                    "revision": candidate.get("revision"),
                    "source_spans": list(candidate.get("source_spans") or ()),
                    "implementation_function": candidate.get("implementation_function"),
                }
            )
    return {
        "source_candidate_projection": source_candidates,
        "source_to_owner_functions": ownership,
        "D_factor": build_d_factor(raw_design),
    }


def reconstructed_author_information(s_factor: dict[str, Any], d_factor: dict[str, Any]) -> dict[str, Any]:
    source = s_factor["source_map"]
    return {
        "source_candidate_projection": [
            {
                "slot": row["slot"],
                "candidate": row["candidate"],
                "record_id": row["record_id"],
                "revision": row["revision"],
                "source_spans": row["source_spans"],
                "evidence": row["evidence"],
            }
            for row in source["candidates"]
        ],
        "source_to_owner_functions": copy.deepcopy(source["source_to_owner_functions"]),
        "D_factor": copy.deepcopy(d_factor),
    }


def _split_endpoint(value: str) -> tuple[str, str] | None:
    parts = value.split(".")
    return (parts[0], parts[1]) if len(parts) == 2 and all(parts) else None


def _cycle(nodes: set[str], dependencies: dict[str, set[str]]) -> bool:
    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(node: str) -> bool:
        if node in visiting:
            return True
        if node in visited:
            return False
        visiting.add(node)
        if any(visit(child) for child in dependencies.get(node, set()) if child in nodes):
            return True
        visiting.remove(node)
        visited.add(node)
        return False

    return any(visit(node) for node in sorted(nodes))


def dependency_findings(d_factor: dict[str, Any]) -> list[str]:
    findings: list[str] = []
    slots: dict[str, dict[str, Any]] = {}
    for slot in d_factor.get("slots") or ():
        name = str(slot.get("name") or "")
        if not name or name in slots:
            findings.append(f"slot_name_invalid_or_duplicate:{name}")
            continue
        slots[name] = slot
        for candidate in slot.get("candidates") or ():
            owner = f"{name}.{candidate.get('name')}"
            parameters = candidate.get("parameters") or []
            parameter_names = {str(item.get("name") or "") for item in parameters}
            graph: dict[str, set[str]] = {}
            for parameter in parameters:
                parameter_name = str(parameter.get("name") or "")
                dependencies = {str(item) for item in parameter.get("depends_on") or ()}
                graph[parameter_name] = {item for item in dependencies if "." not in item}
                for dependency in sorted(dependencies):
                    if "." in dependency or dependency not in parameter_names:
                        findings.append(f"candidate_dependency_unknown:{owner}.{parameter_name}:{dependency}")
            if _cycle(parameter_names, graph):
                findings.append(f"candidate_dependency_cycle:{owner}")

    interface_index: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for slot_name, slot in slots.items():
        candidates = slot.get("candidates") or []
        for candidate in candidates:
            for interface in candidate.get("interfaces") or ():
                interface_index[(slot_name, str(interface.get("name") or ""))].append(interface)

    all_binding_derived = {
        str(parameter.get("name") or "")
        for binding in d_factor.get("bindings") or ()
        for parameter in binding.get("derived") or ()
    }
    for binding in d_factor.get("bindings") or ():
        binding_id = str(binding.get("binding_id") or "")
        endpoints: dict[str, tuple[str, str]] = {}
        for endpoint_kind in ("provider", "consumer"):
            raw_endpoint = str(binding.get(endpoint_kind) or "")
            endpoint = _split_endpoint(raw_endpoint)
            if endpoint is None or endpoint[0] not in slots:
                findings.append(f"binding_endpoint_invalid:{binding_id}:{endpoint_kind}:{raw_endpoint}")
                continue
            endpoints[endpoint_kind] = endpoint
            interfaces = interface_index.get(endpoint, [])
            candidate_count = len(slots[endpoint[0]].get("candidates") or [])
            if len(interfaces) != candidate_count:
                findings.append(f"binding_endpoint_not_total:{binding_id}:{endpoint_kind}:{raw_endpoint}")
                continue
            expected_roles = {endpoint_kind, "bidirectional"}
            if any(interface.get("role") not in expected_roles for interface in interfaces):
                findings.append(f"binding_endpoint_role_mismatch:{binding_id}:{endpoint_kind}:{raw_endpoint}")
        if len(endpoints) == 2:
            provider_kinds = {item.get("kind") for item in interface_index[endpoints["provider"]]}
            consumer_kinds = {item.get("kind") for item in interface_index[endpoints["consumer"]]}
            if len(provider_kinds) != 1 or provider_kinds != consumer_kinds:
                findings.append(f"binding_endpoint_kind_mismatch:{binding_id}")

        derived = binding.get("derived") or []
        local_names = {str(item.get("name") or "") for item in derived}
        graph: dict[str, set[str]] = {}
        for parameter in derived:
            name = str(parameter.get("name") or "")
            dependencies = {str(item) for item in parameter.get("depends_on") or ()}
            graph[name] = {item for item in dependencies if "." not in item}
            for dependency in sorted(dependencies):
                parts = dependency.split(".")
                if len(parts) == 1:
                    if dependency not in local_names:
                        findings.append(f"binding_dependency_unknown:{binding_id}.{name}:{dependency}")
                    continue
                if len(parts) == 2 and parts[0] == "resolved":
                    if parts[1] not in all_binding_derived:
                        findings.append(f"binding_resolved_dependency_unknown:{binding_id}.{name}:{dependency}")
                    continue
                if len(parts) == 2 and parts[0] in {"provider", "consumer"}:
                    endpoint = endpoints.get(parts[0])
                    if endpoint is None or any(
                        parts[1] not in (interface.get("dimensions") or {})
                        for interface in interface_index.get(endpoint, [])
                    ):
                        findings.append(f"binding_endpoint_dimension_unknown:{binding_id}.{name}:{dependency}")
                    continue
                if len(parts) == 3 and parts[0] in slots:
                    endpoint = (parts[0], parts[1])
                    interfaces = interface_index.get(endpoint, [])
                    candidate_count = len(slots[parts[0]].get("candidates") or [])
                    if len(interfaces) != candidate_count or any(
                        parts[2] not in (interface.get("dimensions") or {}) for interface in interfaces
                    ):
                        findings.append(f"binding_slot_dimension_unknown:{binding_id}.{name}:{dependency}")
                    continue
                findings.append(f"binding_dependency_invalid:{binding_id}.{name}:{dependency}")
        if _cycle(local_names, graph):
            findings.append(f"binding_dependency_cycle:{binding_id}")
    return sorted(set(findings))


def validate_author_packet(packet: dict[str, Any]) -> list[str]:
    findings: list[str] = []
    required = {
        "schema_version",
        "packet_kind",
        "arm_id",
        "slug",
        "common_authoring_packet",
        "exposed_factors",
        "factors",
    }
    if set(packet) != required:
        findings.append("packet_top_level_schema_mismatch")
    arm = packet.get("arm_id")
    if arm not in ARM_FACTORS:
        findings.append("packet_arm_invalid")
        return findings
    expected = list(ARM_FACTORS[arm])
    if packet.get("schema_version") != SCHEMA_VERSION or packet.get("packet_kind") != "pipeline_ablation_author_packet":
        findings.append("packet_identity_invalid")
    if packet.get("exposed_factors") != expected:
        findings.append("packet_exposed_factors_mismatch")
    factors = packet.get("factors")
    if not isinstance(factors, dict) or list(factors) != expected:
        findings.append("packet_factor_keys_mismatch")
    findings.extend(schema_findings("author_packet", packet))
    return findings


def make_author_packets(
    slug: str,
    common_authoring_packet: dict[str, Any],
    s_factor: dict[str, Any],
    d_factor: dict[str, Any],
) -> dict[str, dict[str, Any]]:
    factors = {"S_factor": s_factor, "D_factor": d_factor}
    packets: dict[str, dict[str, Any]] = {}
    for arm, exposed in ARM_FACTORS.items():
        packet = {
            "schema_version": SCHEMA_VERSION,
            "packet_kind": "pipeline_ablation_author_packet",
            "arm_id": arm,
            "slug": slug,
            "common_authoring_packet": copy.deepcopy(common_authoring_packet),
            "exposed_factors": list(exposed),
            "factors": {name: copy.deepcopy(factors[name]) for name in exposed},
        }
        findings = validate_author_packet(packet)
        if findings:
            raise ContractError("; ".join(findings))
        packets[arm] = packet
    common_hashes = {
        canonical_sha256(packet["common_authoring_packet"]) for packet in packets.values()
    }
    if len(common_hashes) != 1:
        raise ContractError("common_authoring_packet differs across arms")
    return packets


def prepare_factor_bundle(
    *,
    project_root: Path,
    source_map: Path,
    design_path: Path,
    records_root: Path,
    common_authoring_packet: dict[str, Any],
) -> dict[str, Any]:
    project_root = project_root.resolve()
    source_map = source_map.resolve()
    design_path = design_path.resolve()
    records_root = records_root.resolve()
    problems: list[str] = []
    warnings: list[str] = []
    source_maps, template_design = _load_authoring_modules(project_root)
    resolution = source_maps.parse_source_map(source_map, records_root)
    problems.extend(f"source_map:{item}" for item in resolution.problems)
    raw_design = read_json(design_path)
    if not isinstance(raw_design, dict):
        raise ContractError("TemplateDesign root must be an object")
    try:
        native_design = template_design.load_template_design(design_path)
    except Exception as exc:
        native_design = None
        problems.append(f"template_design:{type(exc).__name__}:{exc}")
    if native_design is not None:
        problems.extend(f"alignment:{item}" for item in factor_alignment_findings(resolution, raw_design, source_map))
    s_factor = build_s_factor(resolution, raw_design, project_root)
    d_factor = build_d_factor(raw_design)
    problems.extend(common_packet_findings(common_authoring_packet, str(raw_design.get("slug") or "")))
    problems.extend(schema_findings("S_factor", s_factor))
    problems.extend(schema_findings("D_factor", d_factor))
    problems.extend(f"dependency:{item}" for item in dependency_findings(d_factor))
    problems.extend(f"leakage:{item}" for item in leakage_findings(s_factor, d_factor))
    native_normalized = normalized_native_author_information(raw_design)
    reconstructed = reconstructed_author_information(s_factor, d_factor)
    if canonical_bytes(native_normalized) != canonical_bytes(reconstructed):
        problems.append("normalization:S_union_D_does_not_reconstruct_native_author_information")
    if raw_design.get("slug") != resolution.export_category:
        warnings.append("slug derives from TemplateDesign while SourceMap export_category differs")
    problems = sorted(set(problems))
    packets: dict[str, dict[str, Any]] = {}
    if not problems:
        packets = make_author_packets(
            str(raw_design["slug"]), common_authoring_packet, s_factor, d_factor
        )
    audit = {
        "schema_version": SCHEMA_VERSION,
        "report_kind": "pipeline_ablation_factor_private_audit",
        "contract": contract_info(),
        "verdict": "pass" if not problems else "fail",
        "author_packets_released": not problems,
        "inputs": {
            "source_map": {"path": display_path(source_map, project_root), "sha256": sha256_file(source_map)},
            "template_design": {"path": display_path(design_path, project_root), "sha256": sha256_file(design_path)},
            "records_root": display_path(records_root, project_root),
        },
        "factor_hashes": {
            "S_factor": canonical_sha256(s_factor),
            "D_factor": canonical_sha256(d_factor),
        },
        "normalization_reconstruction": {
            "native_projection_sha256": canonical_sha256(native_normalized),
            "reconstructed_S_union_D_sha256": canonical_sha256(reconstructed),
            "byte_identical": canonical_bytes(native_normalized) == canonical_bytes(reconstructed),
            "top_level_integrity_fields_excluded": ["source_map_path", "source_map_sha256"],
        },
        "common_authoring_packet_sha256": canonical_sha256(common_authoring_packet),
        "common_authoring_packet_byte_identical_across_arms": (
            len({canonical_bytes(packet["common_authoring_packet"]) for packet in packets.values()}) == 1
            if packets
            else False
        ),
        "author_packet_hashes": {arm: canonical_sha256(packet) for arm, packet in sorted(packets.items())},
        "private_only_information": [
            "private input hashes",
            "private factor hashes",
            "validation diagnostics",
            "withheld-factor metadata",
        ],
        "factor_assignment": {
            "TemplateDesign.candidate.implementation_function": "S_factor.source_map.source_to_owner_functions"
        },
        "problems": problems,
        "warnings": sorted(set(warnings)),
        "leakage_scope": (
            "Checks forbidden keys, exact source-navigation tokens, source-reference patterns, and factor closure; "
            "it does not prove absence of semantic paraphrase leakage."
        ),
    }
    return {"private_audit": audit, "author_packets": packets}


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2).encode("utf-8") + b"\n")


def _scope_args(project_root: Path, values: list[str] | None) -> list[tuple[str, Path]] | None:
    if not values:
        return None
    result = []
    for index, value in enumerate(values):
        path = Path(value)
        if not path.is_absolute():
            path = project_root / path
        result.append((f"custom_{index:02d}", path))
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[2])
    subparsers = parser.add_subparsers(dest="command", required=True)

    freshness = subparsers.add_parser("freshness", help="scan candidates for local development evidence")
    freshness.add_argument("--manifest", type=Path, action="append", default=[])
    freshness.add_argument("--slug", action="append", default=[])
    freshness.add_argument("--scope", action="append")
    freshness.add_argument("--max-evidence", type=int, default=100)
    freshness.add_argument("--output", type=Path)

    factors = subparsers.add_parser("factors", help="build redacted S/D author packets")
    factors.add_argument("--source-map", type=Path, required=True)
    factors.add_argument("--design", type=Path, required=True)
    factors.add_argument("--records-root", type=Path, required=True)
    factors.add_argument("--common-authoring-packet", type=Path, required=True)
    factors.add_argument("--output-dir", type=Path)
    return parser.parse_args(argv)


def _resolve_from_root(path: Path, project_root: Path) -> Path:
    return path if path.is_absolute() else project_root / path


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    project_root = args.project_root.resolve()
    if args.command == "freshness":
        manifests = [_resolve_from_root(path, project_root) for path in args.manifest]
        candidates, manifest_paths = load_candidates(manifests, args.slug)
        if not candidates:
            raise ContractError("at least one --manifest task or --slug is required")
        report = scan_freshness(
            project_root=project_root,
            candidates=candidates,
            manifest_paths=manifest_paths,
            scopes=_scope_args(project_root, args.scope),
            max_evidence_per_candidate=args.max_evidence,
        )
        if args.output:
            write_json(_resolve_from_root(args.output, project_root), report)
        else:
            sys.stdout.buffer.write(json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2).encode("utf-8") + b"\n")
        return 0 if report["status_counts"]["indeterminate"] == 0 else 2

    bundle = prepare_factor_bundle(
        project_root=project_root,
        source_map=_resolve_from_root(args.source_map, project_root),
        design_path=_resolve_from_root(args.design, project_root),
        records_root=_resolve_from_root(args.records_root, project_root),
        common_authoring_packet=read_json(
            _resolve_from_root(args.common_authoring_packet, project_root)
        ),
    )
    if args.output_dir:
        output_dir = _resolve_from_root(args.output_dir, project_root)
        write_json(output_dir / "private_audit.json", bundle["private_audit"])
        if bundle["private_audit"]["verdict"] == "pass":
            for arm, packet in bundle["author_packets"].items():
                write_json(output_dir / "author_packets" / f"{arm}.json", packet)
    else:
        sys.stdout.buffer.write(json.dumps(bundle, ensure_ascii=False, sort_keys=True, indent=2).encode("utf-8") + b"\n")
    return 0 if bundle["private_audit"]["verdict"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
