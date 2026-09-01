#!/usr/bin/env python3
"""Behavior tests for the independent PhysX-Mobility S1 verifier."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import zipfile

import pytest


REPO = Path(__file__).resolve().parents[2]
VERIFIER_PATH = REPO / "exp/scripts/verify_s1_physx_mobility.py"
SPEC = importlib.util.spec_from_file_location("verify_s1_physx_mobility_test_target", VERIFIER_PATH)
assert SPEC is not None and SPEC.loader is not None
verifier = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = verifier
SPEC.loader.exec_module(verifier)


def _record(index: int, *, strict: bool) -> dict:
    return {
        "selection_index": index,
        "dataset_id": str(index + 1),
        "status": "completed",
        "binding_verified": True,
        "declared_collision_elements": 0,
        "receipt_bound": False,
        "receipt_replay_pass": False,
        "rebuild_eligible": False,
        "rebuild_match": None,
        "allowance_complete": True,
        "registered_allowance_pairs": 0,
        "eligible_nonadjacent_pairs": 0,
        "strict_pass_no_method_allowance": strict,
        "registered_allowance_strict_pass": strict,
    }


def _summary() -> dict:
    return {
        "n_eval": 2,
        "status_counts": {"completed": 2},
        "claim_boundary": {
            "declared_collision_element_total": 0,
            "strict_collision_outcome_is_vacuous": True,
        },
        "metrics": {
            "receipt_bound_assets": {"passed": 0, "denominator": 2, "rate": 0.0, "percentage": 0.0},
            "receipt_replay_pass": {"passed": 0, "denominator": 2, "rate": 0.0, "percentage": 0.0},
            "deterministic_rebuild_match": {
                "status": "N/E",
                "passed": None,
                "denominator": 0,
                "rate": None,
                "percentage": None,
                "eligible_assets": 0,
                "asset_denominator": 2,
            },
            "allowance_density": {
                "status": "N/E",
                "registered_pairs": 0,
                "eligible_pairs": 0,
                "rate": None,
                "percentage": None,
                "measured_assets": 2,
                "intended_assets": 2,
                "reason": "no eligible non-adjacent collision-bearing source-link pairs",
            },
            "strict_pass_no_method_allowance": {"passed": 1, "denominator": 2, "rate": 0.5, "percentage": 50.0},
            "registered_allowance_gain_pp": {
                "status": "COMPLETE",
                "value": 0.0,
                "registered_passed": 1,
                "no_allowance_passed": 1,
                "denominator": 2,
            },
        },
    }


def test_independent_aggregate_accepts_zero_pair_denominator_as_not_evaluable() -> None:
    records = [_record(0, strict=True), _record(1, strict=False)]

    verifier.verify_aggregates(records, _summary())


def test_independent_aggregate_rejects_zero_pair_denominator_as_zero_percent() -> None:
    records = [_record(0, strict=True), _record(1, strict=False)]
    tampered = deepcopy(_summary())
    tampered["metrics"]["allowance_density"].update(
        {"status": "COMPLETE", "rate": 0.0, "percentage": 0.0}
    )

    with pytest.raises(verifier.VerificationError, match="aggregate mismatch"):
        verifier.verify_aggregates(records, tampered)


def test_archive_binding_rejects_missing_selected_member(tmp_path: Path) -> None:
    archive = tmp_path / "release.zip"
    urdf = b"<robot name='x'><link name='base'/></robot>"
    metadata = json.dumps({"category": "Test"}).encode()
    with zipfile.ZipFile(archive, "w") as stream:
        stream.writestr("PhysX_mobility/urdf/1.urdf", urdf)
        stream.writestr("PhysX_mobility/finaljson/1.json", metadata)
    receipt = {
        "archive_path": str(archive),
        "archive_bytes": archive.stat().st_size,
        "archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        "archive_member_prefix": "PhysX_mobility",
    }
    row = {
        "dataset_id": 1,
        "urdf_relative_path": "urdf/1.urdf",
        "urdf_sha256": hashlib.sha256(urdf).hexdigest(),
        "finaljson_relative_path": "finaljson/1.json",
        "finaljson_sha256": hashlib.sha256(metadata).hexdigest(),
        "resources": [
            {
                "relative_path": "partseg/1/objs/part.obj",
                "sha256": hashlib.sha256(b"mesh").hexdigest(),
            }
        ],
    }

    with pytest.raises(verifier.VerificationError, match="archive member missing"):
        verifier.verify_archive_binding(receipt, [row], archive_path=archive)


def test_output_manifest_rejects_missing_artifact_binding(tmp_path: Path) -> None:
    for name in verifier.REQUIRED_MANIFEST_ARTIFACTS:
        (tmp_path / name).write_text(name, encoding="utf-8")
    frozen = {
        "protocol_id": verifier.PROTOCOL_ID,
        "classification": "SMOKE",
        "dataset": verifier.DATASET,
        "cohort": {"n_eval": 2},
        "table4_source": {"denominator": 2},
        "code_identity": {"runner_sha256": "runner"},
    }
    summary = {
        "protocol_id": verifier.PROTOCOL_ID,
        "classification": "SMOKE",
        "dataset": verifier.DATASET,
        "n_eval": 2,
    }
    artifacts = {
        name: {
            "bytes": (tmp_path / name).stat().st_size,
            "sha256": verifier.sha256_file(tmp_path / name),
        }
        for name in verifier.REQUIRED_MANIFEST_ARTIFACTS
    }
    artifacts.pop("summary.md")
    manifest = {
        "protocol_id": verifier.PROTOCOL_ID,
        "classification": "SMOKE",
        "dataset": verifier.DATASET,
        "n_eval": 2,
        "cohort": frozen["cohort"],
        "table4_source": frozen["table4_source"],
        "code_identity": frozen["code_identity"],
        "artifacts": artifacts,
        "post_manifest_receipts": {"verification.json": "explicitly excluded"},
    }
    manifest["manifest_content_sha256"] = verifier.canonical_sha256(manifest)

    with pytest.raises(verifier.VerificationError, match="artifact key set"):
        verifier.verify_output_manifest(tmp_path, manifest, frozen, summary)
