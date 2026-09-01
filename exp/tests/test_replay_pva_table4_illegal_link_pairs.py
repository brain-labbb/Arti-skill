from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/replay_pva_table4_illegal_link_pairs.py"
SPEC = importlib.util.spec_from_file_location("link_pair_replay_tested", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_replay_classification_boundaries() -> None:
    tolerance = MODULE.DEPTH_EQUIVALENCE_TOLERANCE_M
    assert MODULE.classify_replay(2, 2, tolerance) == "exact"
    assert MODULE.classify_replay(2, 3, tolerance) == "manifold_count_only"
    assert MODULE.classify_replay(2, 2, tolerance * 1.01) == "partial_non_equivalent"


def test_artifact_self_hash() -> None:
    artifact = {"schema_version": "fixture", "value": 1}
    artifact["artifact_content_sha256"] = MODULE.hashlib.sha256(
        MODULE.canonical(artifact).encode()
    ).hexdigest()
    assert MODULE.self_hash_valid(artifact)
    artifact["value"] = 2
    assert not MODULE.self_hash_valid(artifact)


def test_provenance_hashes_are_content_sensitive() -> None:
    first = MODULE.hashlib.sha256(b"record-a").hexdigest()
    second = MODULE.hashlib.sha256(b"record-b").hexdigest()
    assert first != second


def test_runtime_contract_rejects_missing_runner() -> None:
    expected = {"python_version": "x", "runner_sha256": "abc"}
    with pytest.raises(RuntimeError, match="field-set mismatch.*runner_sha256"):
        MODULE.verify_runtime_contract(expected, {"python_version": "x"})


def test_manifest_self_hash_rejects_tamper() -> None:
    manifest = {"schema_version": "fixture", "value": 1}
    manifest["manifest_content_sha256"] = MODULE.hashlib.sha256(
        MODULE.canonical(manifest).encode()
    ).hexdigest()
    assert MODULE.verify_manifest_self_hash(manifest)["match"] is True
    manifest["value"] = 2
    with pytest.raises(RuntimeError, match="manifest self-hash mismatch"):
        MODULE.verify_manifest_self_hash(manifest)
