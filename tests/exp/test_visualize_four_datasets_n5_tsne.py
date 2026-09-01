from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "exp/scripts/visualize_four_datasets_n5_tsne.py"
)


def _module():
    name = "_test_visualize_four_datasets_n5_tsne"
    spec = importlib.util.spec_from_file_location(name, SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _fixture_root(tmp_path: Path) -> tuple[Path, list[dict[str, object]]]:
    root = tmp_path / "renders"
    root.mkdir()
    rows: list[dict[str, object]] = []
    ordinal = 0
    for class_number in range(2):
        for sample_index in range(1, 6):
            ordinal += 1
            image_path = root / f"C{class_number}_S{sample_index}.png"
            Image.new(
                "RGB", (16, 16), (30 + class_number * 80, 20 + sample_index, 90)
            ).save(image_path)
            payload = image_path.read_bytes()
            rows.append(
                {
                    "ordinal": ordinal,
                    "render_key": f"C{class_number}:S{sample_index}",
                    "generator_index": f"C{class_number}",
                    "generator_name": f"class_{class_number}",
                    "sample_index": sample_index,
                    "source_type": "test",
                    "asset_id": f"asset_{class_number}_{sample_index}",
                    "output_path": str(image_path),
                    "png_bytes": len(payload),
                    "png_sha256": hashlib.sha256(payload).hexdigest(),
                }
            )
    _write_csv(root / "render_roster.csv", rows)
    _write_csv(
        root / "render_manifest.csv",
        [{**row, "status": "rendered"} for row in reversed(rows)],
    )
    config = {
        "resolution": 16,
        "samples": 4,
        "studio": {"camera_direction": [1.25, -1.35, 0.85]},
        "renderer_sha256": "a" * 64,
        "blender_version": "Blender test",
    }
    (root / "render_config.json").write_text(json.dumps(config), encoding="utf-8")
    (root / "render_summary.json").write_text(
        json.dumps({"failure_count": 0, "selected_count": 10}), encoding="utf-8"
    )
    return root, rows


def test_load_cohort_receipt_checks_and_manifest_reordering(tmp_path: Path) -> None:
    module = _module()
    root, _ = _fixture_root(tmp_path)
    spec = module.DatasetSpec("test", "Test", root, 2, 10, 2)
    cohort = module.load_cohort(spec)
    assert [sample.ordinal for sample in cohort.samples] == list(range(1, 11))
    assert cohort.strict_class_ids == ("C0", "C1")
    assert len(cohort.strict_samples) == 10


def test_load_cohort_rejects_png_receipt_drift(tmp_path: Path) -> None:
    module = _module()
    root, rows = _fixture_root(tmp_path)
    rows[0]["png_sha256"] = "0" * 64
    _write_csv(root / "render_roster.csv", rows)
    spec = module.DatasetSpec("test", "Test", root, 2, 10, 2)
    with pytest.raises(ValueError, match="roster/manifest|SHA-256"):
        module.load_cohort(spec)


def test_load_cohort_accepts_manifest_only_png_receipts(tmp_path: Path) -> None:
    module = _module()
    root, rows = _fixture_root(tmp_path)
    roster_rows = [
        {key: value for key, value in row.items() if key not in {"png_bytes", "png_sha256"}}
        for row in rows
    ]
    _write_csv(root / "render_roster.csv", roster_rows)
    spec = module.DatasetSpec("test", "Test", root, 2, 10, 2)
    cohort = module.load_cohort(spec)
    assert len(cohort.samples) == 10
    assert all(len(sample.png_sha256) == 64 for sample in cohort.samples)


def test_separation_metrics_excludes_self_and_recovers_clusters() -> None:
    module = _module()
    angles = np.asarray([-0.04, -0.02, 0.0, 0.02, 0.04], dtype=np.float32)
    class_a = np.column_stack((np.cos(angles), np.sin(angles)))
    class_b = np.column_stack((np.cos(angles + np.pi / 2), np.sin(angles + np.pi / 2)))
    features = np.concatenate((class_a, class_b), axis=0).astype(np.float32)
    labels = ["a"] * 5 + ["b"] * 5
    metrics = module.separation_metrics(features, labels)
    assert metrics["top1_same_class_rate"] == pytest.approx(1.0)
    assert metrics["top4_same_class_fraction"] == pytest.approx(1.0)
    assert metrics["mean_nearest_positive_minus_negative_margin"] > 0.9
    assert metrics["cosine_silhouette"] > 0.99


def test_top1_metric_is_rank1_after_self_exclusion() -> None:
    module = _module()
    features = np.random.default_rng(0).normal(size=(10, 6)).astype(np.float32)
    features /= np.linalg.norm(features, axis=1, keepdims=True)
    labels = np.asarray(["a"] * 5 + ["b"] * 5)
    similarity = features @ features.T
    np.fill_diagonal(similarity, -np.inf)
    order = np.argsort(-similarity, axis=1)
    expected_top1 = float(np.mean(labels[order[:, 0]] == labels))
    rank2_rate = float(np.mean(labels[order[:, 1]] == labels))
    assert expected_top1 == pytest.approx(0.4)
    assert rank2_rate == pytest.approx(0.2)
    metrics = module.separation_metrics(features, labels.tolist())
    assert metrics["top1_same_class_rate"] == pytest.approx(expected_top1)


def test_separation_metrics_requires_exact_n5() -> None:
    module = _module()
    features = np.eye(9, dtype=np.float32)
    with pytest.raises(ValueError, match="exactly five"):
        module.separation_metrics(features, ["a"] * 5 + ["b"] * 4)


def test_studio_contract_prefers_shared_renderer() -> None:
    module = _module()
    contract = module._studio_contract(
        {
            "resolution": 256,
            "samples": 4,
            "studio": {"mode": "opaque_studio"},
            "renderer_sha256": "a" * 64,
            "shared_renderer_sha256": "b" * 64,
            "blender_version": "Blender 4.2.19 LTS",
        }
    )
    assert contract["effective_shared_renderer_sha256"] == "b" * 64


def test_dependency_preflight_reports_missing_package(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _module()
    real_find_spec = module.importlib.util.find_spec

    def fake_find_spec(name: str):
        return None if name == "sklearn" else real_find_spec(name)

    monkeypatch.setattr(module.importlib.util, "find_spec", fake_find_spec)
    with pytest.raises(RuntimeError, match="sklearn"):
        module.dependency_preflight()
