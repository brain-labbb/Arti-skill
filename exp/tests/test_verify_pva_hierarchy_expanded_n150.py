from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/verify_pva_hierarchy_expanded_n150.py"


def load_verifier():
    spec = importlib.util.spec_from_file_location("verify_pva_expanded_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_normalize_core_replay_payload_removes_only_declared_path_and_time_fields() -> None:
    verifier = load_verifier()
    original = "/tmp/reference/pva"
    replay = "/tmp/replay/pva_replay"
    left = {
        "created_at_utc": "first",
        "sample_id": "table/seed_0",
        "urdf_path": f"{original}/packages/table/seed_0/model.urdf",
        "nested": {"updated_at_utc": "first", "metric": 0.5},
    }
    right = {
        "created_at_utc": "second",
        "sample_id": "table/seed_0",
        "urdf_path": f"{replay}/packages/table/seed_0/model.urdf",
        "nested": {"updated_at_utc": "second", "metric": 0.5},
    }
    assert verifier.normalize_replay_payload(left, original, replay) == verifier.normalize_replay_payload(
        right, replay, replay
    )


def test_core_replay_payload_detects_metric_drift() -> None:
    verifier = load_verifier()
    left = {"metric": 0.5, "created_at_utc": "first"}
    right = {"metric": 0.6, "created_at_utc": "second"}
    assert verifier.normalize_replay_payload(left, "/tmp/a", "/tmp/b") != verifier.normalize_replay_payload(
        right, "/tmp/b", "/tmp/b"
    )


def test_normalize_core_replay_payload_omits_derived_file_hash_fields() -> None:
    verifier = load_verifier()
    left = {
        "records_sha256": "hash-from-reference-path-bearing-jsonl",
        "selection_manifest_sha256": "hash-from-reference-manifest",
        "metric": 0.5,
    }
    right = {
        "records_sha256": "hash-from-replay-path-bearing-jsonl",
        "selection_manifest_sha256": "hash-from-replay-manifest",
        "metric": 0.5,
    }
    assert verifier.normalize_replay_payload(left, "/tmp/a", "/tmp/b") == verifier.normalize_replay_payload(
        right, "/tmp/b", "/tmp/b"
    )


def test_rewrite_terminal_for_replay_rebases_available_paths() -> None:
    verifier = load_verifier()
    source = Path("/tmp/reference/pva")
    replay = Path("/tmp/replay/pva_replay")
    row = {
        "category": "table",
        "seed": 7,
        "artifact_dir": str(source / "packages/table/seed_7"),
        "model_urdf": str(source / "packages/table/seed_7/model.urdf"),
        "parseable_final_urdf": True,
    }
    rewritten = verifier.rewrite_terminal_for_replay(row, source, replay)
    assert rewritten["artifact_dir"] == str(replay / "packages/table/seed_7")
    assert rewritten["model_urdf"] == str(replay / "packages/table/seed_7/model.urdf")


def test_rewrite_terminal_for_replay_leaves_failure_without_paths_unchanged() -> None:
    verifier = load_verifier()
    row = {
        "category": "microwave",
        "seed": 0,
        "artifact_dir": None,
        "parseable_final_urdf": False,
        "failure_type": "fail_if_isolated_parts()",
    }
    assert verifier.rewrite_terminal_for_replay(row, Path("/tmp/a"), Path("/tmp/b")) == row


def test_prepare_replay_rejects_existing_directory_unless_explicit_reuse() -> None:
    verifier = load_verifier()
    assert verifier.replay_preparation_mode(Path("/tmp/existing"), exists=True, reuse=False) == "reject"
    assert verifier.replay_preparation_mode(Path("/tmp/existing"), exists=True, reuse=True) == "reuse"
    assert verifier.replay_preparation_mode(Path("/tmp/new"), exists=False, reuse=False) == "create"
