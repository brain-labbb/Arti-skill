from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SUBJECT = ROOT / "exp/scripts/compare_pva_artiverse_articraft_partnet_uniform.py"
RENDER_DRIVER = ROOT / "exp/scripts/render_partnet_mobility_uniform.py"


def _load(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def subject() -> Any:
    return _load("compare_four_source_partnet_test_subject", SUBJECT)


@pytest.fixture(scope="module")
def render_driver() -> Any:
    return _load("compare_four_source_partnet_render_driver", RENDER_DRIVER)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _make_release_manifest(render_driver: Any, tmp_path: Path) -> tuple[Path, Path]:
    source_root = tmp_path / "source"
    source_root.mkdir()
    rows: list[dict[str, Any]] = []
    for ordinal, (asset_id, category) in enumerate((("100", "alpha"), ("101", "beta"))):
        package = source_root / asset_id
        package.mkdir()
        urdf = package / "mobility.urdf"
        urdf.write_text(f"<robot name='{asset_id}'><link name='base'/></robot>\n", encoding="utf-8")
        meta = package / "meta.json"
        meta.write_text(json.dumps({"model_cat": category}) + "\n", encoding="utf-8")
        package_files = [
            {"path": path.name, "sha256": _sha(path), "size": path.stat().st_size}
            for path in sorted((meta, urdf), key=lambda value: value.name)
        ]
        nested_files = [
            {"bytes": row["size"], "path": row["path"], "sha256": row["sha256"]}
            for row in package_files
        ]
        binding = {
            "content_manifest_sha256": render_driver._canonical_sha256(nested_files),
            "file_count": len(package_files),
            "files": nested_files,
            "total_bytes": sum(row["size"] for row in package_files),
        }
        rows.append(
            {
                "asset_id": asset_id,
                "category": category,
                "ordinal": ordinal,
                "package_binding": binding,
                "package_binding_sha256": render_driver._canonical_sha256(package_files),
                "package_files": package_files,
                "parse_status": "valid",
                "primary_urdf_bytes": urdf.stat().st_size,
                "primary_urdf_path": str(urdf.resolve()),
                "primary_urdf_relative_path": f"{asset_id}/mobility.urdf",
                "primary_urdf_sha256": _sha(urdf),
                "primary_urdf_size": urdf.stat().st_size,
                "raw_category": category,
                "source_path": str(package.resolve()),
                "source_relative_path": asset_id,
                "xml_parse_status": "valid",
            }
        )
    manifest: dict[str, Any] = {
        "J_eval": 0,
        "N_eval": len(rows),
        "dataset": render_driver.EXPECTED_DATASET,
        "roster_sha256": render_driver._canonical_sha256(rows),
        "rows": rows,
        "schema_version": render_driver.EXPECTED_SCHEMA,
        "source_bindings": [
            {"name": render_driver.EXPECTED_SOURCE_NAME, "path": str(source_root.resolve())}
        ],
    }
    manifest["manifest_content_sha256"] = render_driver._canonical_sha256(manifest)
    path = tmp_path / "full_release_manifest.json"
    path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    return path, source_root


def _make_uniform_render(
    subject: Any,
    render_driver: Any,
    tmp_path: Path,
) -> tuple[Path, dict[str, Any]]:
    dataset_manifest, source_root = _make_release_manifest(render_driver, tmp_path)
    root = tmp_path / "renders"
    root.mkdir()
    items = render_driver.load_render_items(
        dataset_manifest,
        output_root=root,
        strict_counts=False,
        validate_inputs=True,
    )
    winners = tuple(item for item in items if item.category_one_shot)
    support = {
        "driver": RENDER_DRIVER,
        "renderer": ROOT / "exp/scripts/render_partnet_mobility_asset_blender.py",
        "base_renderer": ROOT / "exp/scripts/render_articraft10k_asset_blender.py",
        "shared_renderer": ROOT / "arti-template/scripts/render_exported_asset_blender.py",
    }
    studio = {"camera": "fixture", "background": [1.0, 1.0, 1.0, 1.0]}
    config = {
        "schema_version": 1,
        "render_contract": "partnet_mobility_uniform_studio_v1",
        "dataset": "PartNet-Mobility",
        "model_count": len(items),
        "category_count": len({item.category for item in items}),
        "selected_count": len(winners),
        "selected_category_count": len(winners),
        "dataset_manifest": str(dataset_manifest.resolve()),
        "dataset_manifest_sha256": _sha(dataset_manifest),
        "dataset_manifest_content_sha256": json.loads(dataset_manifest.read_text())["manifest_content_sha256"],
        "dataset_roster_sha256": json.loads(dataset_manifest.read_text())["roster_sha256"],
        "source_root": str(source_root.resolve()),
        "output_root": str(root.resolve()),
        "blender_version": "Blender fixture",
        "resolution": 256,
        "samples": 4,
        "studio": studio,
        "pose_policy": "URDF rest pose",
        "material_policy": render_driver.MATERIAL_POLICY,
        "selection": {
            "one_shot_only": True,
            "selected_receipt": render_driver._selection_receipt(winners),
        },
    }
    for key, path in support.items():
        config[key] = str(path.resolve())
        config[f"{key}_sha256"] = _sha(path)
    (root / "render_config.json").write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    results = []
    for item in winners:
        item.output_path.parent.mkdir(parents=True)
        Image.new("RGB", (256, 256), (120, 150, 180)).save(item.output_path)
        results.append(
            {
                **render_driver._item_row(item),
                "status": "rendered",
                "elapsed_seconds": 1.0,
                "png_bytes": item.output_path.stat().st_size,
                "png_sha256": _sha(item.output_path),
                "started_at": "fixture",
                "finished_at": "fixture",
                "error": "",
                "renderer_result": {
                    "asset_dir": str(item.source_path.resolve()),
                    "output": str(item.output_path.resolve()),
                    "material_policy": render_driver.MATERIAL_POLICY,
                    "base_renderer": {
                        "path": str(support["base_renderer"].resolve()),
                        "sha256": _sha(support["base_renderer"]),
                    },
                    "shared_renderer": {
                        "path": str(support["shared_renderer"].resolve()),
                        "sha256": _sha(support["shared_renderer"]),
                    },
                },
            }
        )
    render_driver._write_manifest(root / "render_manifest.csv", results)
    render_driver._write_one_shot_roster(root / "category_one_shot_roster.csv", winners)
    pva_config = {
        "resolution": 256,
        "samples": 4,
        "studio": studio,
        "blender_version": "Blender fixture",
        # The PVA contract names its shared studio implementation `renderer`.
        "renderer_sha256": config["shared_renderer_sha256"],
    }
    return root, pva_config


def test_loader_binds_full_manifest_packages_and_pva_renderer_key(
    subject: Any,
    render_driver: Any,
    tmp_path: Path,
) -> None:
    root, pva_config = _make_uniform_render(subject, render_driver, tmp_path)

    bundle = subject.load_partnet_uniform(
        root,
        pva_render_config=pva_config,
        strict_counts=False,
    )

    assert len(bundle.records) == 2
    assert bundle.categories == ("alpha", "beta")
    assert bundle.receipts["release"]["asset_count"] == 2
    assert bundle.receipts["dataset_manifest"]["sha256"] == bundle.config[
        "dataset_manifest_sha256"
    ]
    assert all(record.package_content_manifest_sha256 for record in bundle.records)
    assert all(record.package_binding_sha256 for record in bundle.records)


def test_loader_keeps_package_content_and_binding_receipts_distinct(
    subject: Any,
    render_driver: Any,
    tmp_path: Path,
) -> None:
    root, pva_config = _make_uniform_render(subject, render_driver, tmp_path)
    manifest_path = root / "render_manifest.csv"
    with manifest_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
        fieldnames = list(rows[0])
    rows[0]["package_content_manifest_sha256"] = rows[0]["package_binding_sha256"]
    with manifest_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    with pytest.raises(ValueError, match="package_content_manifest_sha256"):
        subject.load_partnet_uniform(root, pva_render_config=pva_config, strict_counts=False)


def test_loader_rejects_renderer_result_base_receipt_drift(
    subject: Any,
    render_driver: Any,
    tmp_path: Path,
) -> None:
    root, pva_config = _make_uniform_render(subject, render_driver, tmp_path)
    manifest_path = root / "render_manifest.csv"
    with manifest_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
        fieldnames = list(rows[0])
    result = json.loads(rows[0]["renderer_result"])
    result["base_renderer"]["sha256"] = "0" * 64
    rows[0]["renderer_result"] = json.dumps(result, sort_keys=True)
    with manifest_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    with pytest.raises(ValueError, match="base renderer receipt mismatch"):
        subject.load_partnet_uniform(root, pva_render_config=pva_config, strict_counts=False)


def test_partnet_feature_cache_keys_amp_and_helper_sha(
    subject: Any,
    render_driver: Any,
    tmp_path: Path,
) -> None:
    root, pva_config = _make_uniform_render(subject, render_driver, tmp_path)
    bundle = subject.load_partnet_uniform(root, pva_render_config=pva_config, strict_counts=False)
    helper_path = tmp_path / "feature_helper.py"
    helper_path.write_text("VERSION = 1\n", encoding="utf-8")
    model_dirs = {encoder: tmp_path / f"model-{encoder}" for encoder in subject.ENCODERS}
    for path in model_dirs.values():
        path.mkdir()
    calls: list[tuple[str, bool]] = []

    class Helper:
        def extract_image_features(
            self,
            image_paths: Any,
            *,
            model_path: Path,
            batch_size: int,
            device: str,
            num_workers: int,
            use_amp: bool,
        ) -> tuple[np.ndarray, dict[str, Any]]:
            encoder = model_path.name.removeprefix("model-")
            dimension = 3 if encoder == "dinov2" else 2
            calls.append((encoder, use_amp))
            matrix = np.tile(np.arange(1, dimension + 1, dtype=np.float32), (len(image_paths), 1))
            return matrix, {
                "model_type": encoder,
                "feature_dim": dimension,
                "device": "cpu",
                "amp": False,
                "batch_size": batch_size,
                "num_workers": num_workers,
            }

    def normalize(matrix: np.ndarray) -> np.ndarray:
        matrix = np.asarray(matrix, dtype=np.float32)
        return matrix / np.linalg.norm(matrix, axis=1, keepdims=True)

    fake_base = SimpleNamespace(PVA_HELPER_SCRIPT=helper_path, _pva_helper=lambda: Helper())
    subject._THREE = SimpleNamespace(_base=lambda: fake_base, l2_normalize=normalize)
    pva = SimpleNamespace(
        feature_manifests={
            encoder: {
                "model_path": str(model_dirs[encoder]),
                "model_fingerprint": f"fingerprint-{encoder}",
            }
            for encoder in subject.ENCODERS
        },
        features={
            "dinov2": np.ones((1, 3), dtype=np.float32),
            "clip": np.ones((1, 2), dtype=np.float32),
        },
    )
    for use_amp in (False, True, True):
        subject._extract_partnet(
            bundle,
            pva=pva,
            output_dir=tmp_path / "features",
            batch_size=2,
            device="cpu",
            num_workers=1,
            use_amp=use_amp,
            force_extract=False,
        )
    assert calls == [
        ("dinov2", False),
        ("clip", False),
        ("dinov2", True),
        ("clip", True),
    ]

    helper_path.write_text("VERSION = 2\n", encoding="utf-8")
    subject._extract_partnet(
        bundle,
        pva=pva,
        output_dir=tmp_path / "features",
        batch_size=2,
        device="cpu",
        num_workers=1,
        use_amp=True,
        force_extract=False,
    )
    assert len(calls) == 6


def test_metrics_cover_all_six_pairs_and_four_sources(subject: Any) -> None:
    fake_base = SimpleNamespace(
        encoder_agreement_metrics=lambda *_args, **_kwargs: {"agreement": 1.0},
        projection_trustworthiness=lambda *_args, **_kwargs: {"trustworthiness": 1.0},
    )
    subject._THREE = SimpleNamespace(
        _base=lambda: fake_base,
        named_cross_dataset_metrics=lambda *_args, **kwargs: {
            "pair": [kwargs["first_name"], kwargs["second_name"]]
        },
        multi_source_metrics=lambda sources, order, **_kwargs: {
            "counts": {name: len(sources[name]) for name in order}
        },
    )
    counts = dict(zip(subject.SOURCE_ORDER, (2, 3, 4, 5), strict=True))
    source_features = {
        source: {
            encoder: np.ones((counts[source], 3), dtype=np.float32)
            for encoder in subject.ENCODERS
        }
        for source in subject.SOURCE_ORDER
    }
    coordinates = {
        encoder: np.zeros((sum(counts.values()), 2), dtype=np.float32)
        for encoder in subject.ENCODERS
    }

    metrics = subject.compute_metrics(source_features, coordinates, neighbor_fraction=0.1)

    expected_pairs = {
        "pva_vs_artiverse",
        "pva_vs_articraft10k",
        "pva_vs_partnet_mobility",
        "artiverse_vs_articraft10k",
        "artiverse_vs_partnet_mobility",
        "articraft10k_vs_partnet_mobility",
    }
    for encoder in subject.ENCODERS:
        assert set(metrics["per_encoder"][encoder]["pairwise"]) == expected_pairs
        assert metrics["per_encoder"][encoder]["four_source"]["counts"] == counts


def test_joint_coordinate_csv_has_four_contiguous_source_blocks(
    subject: Any,
    tmp_path: Path,
) -> None:
    records = {
        "pva_records": [SimpleNamespace(generator_index=1, generator_name="g", source_type="picture")],
        "artiverse_records": [SimpleNamespace(category="a", one_shot_source="s", one_shot_manifest_root="r")],
        "articraft_records": [SimpleNamespace(category="b", cohort_origin="o", asset_id="x")],
        "partnet_records": [SimpleNamespace(category="c", dataset_id="y")],
    }
    path = tmp_path / "joint.csv"
    subject.write_joint_coordinates(
        path,
        np.arange(8, dtype=np.float32).reshape(4, 2),
        **records,
    )
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert [row["dataset"] for row in rows] == list(subject.SOURCE_ORDER)
    assert [int(row["joint_index"]) for row in rows] == [0, 1, 2, 3]
