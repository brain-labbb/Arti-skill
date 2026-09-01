from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Mapping, Sequence

import numpy as np
import pytest
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[2]
SUBJECT = ROOT / "exp/scripts/compare_uniform_seven_sources.py"


def _load(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def subject() -> Any:
    return _load("compare_uniform_seven_sources_test_subject", SUBJECT)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    assert rows
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _make_uniform_root(tmp_path: Path) -> tuple[Path, dict[str, Any]]:
    root = tmp_path / "uniform"
    root.mkdir()
    support_root = tmp_path / "support"
    support_root.mkdir()
    support: dict[str, Path] = {}
    for name in ("driver", "renderer", "base_renderer", "shared_renderer"):
        path = support_root / f"{name}.py"
        path.write_text(f"ROLE = {name!r}\n", encoding="utf-8")
        support[name] = path

    studio = {
        "background": [0.8, 0.84, 0.9, 1.0],
        "camera": [1.25, -1.35, 0.85],
    }
    config: dict[str, Any] = {
        "schema_version": 1,
        "render_contract": "infinigen_sim_uniform_studio_v1",
        "dataset": "Infinigen-Sim",
        "official_model_count": 20,
        "official_category_count": 2,
        "selected_count": 2,
        "selected_category_count": 2,
        "output_root": str(root.resolve()),
        "blender_version": "Blender fixture",
        "resolution": 256,
        "samples": 4,
        "studio": studio,
    }
    for name, path in support.items():
        config[name] = str(path.resolve())
        config[f"{name}_sha256"] = _sha(path)
    (root / "render_config.json").write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    manifest_rows: list[dict[str, Any]] = []
    roster_rows: list[dict[str, Any]] = []
    for ordinal, (dataset_id, category) in enumerate((("1253", "box"), ("1240", "cabinet"))):
        source_path = tmp_path / "source" / category / dataset_id
        source_path.mkdir(parents=True)
        urdf_path = source_path / f"{category}.urdf"
        urdf_path.write_text(
            f"<robot name='{dataset_id}'><link name='base'/></robot>\n", encoding="utf-8"
        )
        image_path = root / "images" / f"{ordinal:03d}_{category}.png"
        image_path.parent.mkdir(exist_ok=True)
        image = Image.new("RGB", (256, 256), (80, 110, 140))
        ImageDraw.Draw(image).rectangle((64, 48, 210, 220), fill=(220, 180, 90))
        image.save(image_path)
        renderer_result = {
            "asset_dir": str(source_path.resolve()),
            "output": str(image_path.resolve()),
            "base_renderer": {
                "path": str(support["base_renderer"].resolve()),
                "sha256": config["base_renderer_sha256"],
            },
            "shared_renderer": {
                "path": str(support["shared_renderer"].resolve()),
                "sha256": config["shared_renderer_sha256"],
            },
        }
        manifest_rows.append(
            {
                "ordinal": ordinal,
                "dataset_id": dataset_id,
                "category": category,
                "source_path": str(source_path.resolve()),
                "urdf_path": str(urdf_path.resolve()),
                "image_path": str(image_path.resolve()),
                "image_bytes": image_path.stat().st_size,
                "image_sha256": _sha(image_path),
                "renderer_result": json.dumps(renderer_result, sort_keys=True),
            }
        )
        roster_rows.append(
            {"ordinal": ordinal, "dataset_id": dataset_id, "category": category}
        )
    _write_csv(root / "render_manifest.csv", manifest_rows)
    _write_csv(root / "category_one_shot_roster.csv", roster_rows)
    pva_config = {
        "resolution": 256,
        "samples": 4,
        "studio": studio,
        "blender_version": "Blender fixture",
        "renderer_sha256": config["shared_renderer_sha256"],
    }
    return root, pva_config


def test_load_uniform_bundle_accepts_valid_receipted_render(
    subject: Any,
    tmp_path: Path,
) -> None:
    root, pva_config = _make_uniform_root(tmp_path)

    bundle = subject.load_uniform_bundle(
        root,
        name="Infinigen-Sim",
        key="infinigen_sim",
        pva_config=pva_config,
        expected_count=2,
        expected_categories=2,
    )

    assert bundle.categories == ("box", "cabinet")
    assert [record.dataset_id for record in bundle.records] == ["1253", "1240"]
    assert bundle.official_model_count == 20
    assert bundle.receipts["render_manifest"]["sha256"] == _sha(
        root / "render_manifest.csv"
    )


def test_load_uniform_bundle_rejects_tampered_image_receipt(
    subject: Any,
    tmp_path: Path,
) -> None:
    root, pva_config = _make_uniform_root(tmp_path)
    manifest_path = root / "render_manifest.csv"
    with manifest_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
        fieldnames = list(rows[0])
    rows[0]["image_sha256"] = "0" * 64
    with manifest_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    with pytest.raises(ValueError, match="image receipt mismatch"):
        subject.load_uniform_bundle(
            root,
            name="Infinigen-Sim",
            key="infinigen_sim",
            pva_config=pva_config,
            expected_count=2,
            expected_categories=2,
        )


def test_metrics_cover_all_21_pairs_for_both_encoders(subject: Any) -> None:
    fake_base = SimpleNamespace(
        encoder_agreement_metrics=lambda *_args, **_kwargs: {"agreement": 1.0},
        projection_trustworthiness=lambda *_args, **_kwargs: {"trustworthiness": 1.0},
    )
    fake_three = SimpleNamespace(
        _base=lambda: fake_base,
        named_cross_dataset_metrics=lambda *_args, **kwargs: {
            "pair": [kwargs["first_name"], kwargs["second_name"]]
        },
        multi_source_metrics=lambda sources, order, **_kwargs: {
            "order": list(order),
            "counts": {name: len(sources[name]) for name in order},
        },
    )
    subject._FOUR = SimpleNamespace(_three=lambda: fake_three)
    counts = dict(zip(subject.SOURCE_ORDER, range(1, 8), strict=True))
    features = {
        source: {
            encoder: np.ones((counts[source], 3), dtype=np.float32)
            for encoder in subject.ENCODERS
        }
        for source in subject.SOURCE_ORDER
    }
    total = sum(counts.values())
    coords = {
        encoder: np.zeros((total, 2), dtype=np.float32) for encoder in subject.ENCODERS
    }

    metrics = subject.compute_metrics(features, coords, neighbor_fraction=0.1)

    expected_pairs = {
        f"{subject.SOURCE_KEYS[first]}_vs_{subject.SOURCE_KEYS[second]}"
        for first_index, first in enumerate(subject.SOURCE_ORDER)
        for second in subject.SOURCE_ORDER[first_index + 1 :]
    }
    assert len(expected_pairs) == 21
    assert set(metrics["encoder_agreement"]) == set(subject.SOURCE_KEYS.values())
    for encoder in subject.ENCODERS:
        assert set(metrics["per_encoder"][encoder]["pairwise"]) == expected_pairs
        assert metrics["per_encoder"][encoder]["seven_source"]["counts"] == counts
        assert metrics["per_encoder"][encoder]["seven_source"]["order"] == list(
            subject.SOURCE_ORDER
        )


def _audit_inputs(subject: Any, tmp_path: Path) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    bundles: dict[str, Any] = {}
    features: dict[str, Any] = {}
    records_per_source = 2
    for source in subject.SOURCE_ORDER:
        key = subject.SOURCE_KEYS[source]
        records = tuple(
            subject.GenericRecord(
                ordinal=index,
                dataset_id=f"{key}-{index}",
                category=f"{key}-category-{index}",
                image_path=tmp_path / f"{key}-{index}.png",
                image_bytes=1,
                image_sha256="0" * 64,
                source_path=tmp_path,
                urdf_path=None,
            )
            for index in range(records_per_source)
        )
        bundles[source] = subject.GenericBundle(
            name=source,
            key=key,
            root=tmp_path,
            records=records,
            categories=tuple(record.category for record in records),
            config={},
            receipts={},
            official_model_count=records_per_source,
            official_category_count=records_per_source,
        )
        vector = np.asarray([1.0, 2.0, 3.0], dtype=np.float32)
        vector /= np.linalg.norm(vector)
        features[source] = {
            encoder: np.tile(vector, (records_per_source, 1))
            for encoder in subject.ENCODERS
        }
    total = records_per_source * len(subject.SOURCE_ORDER)
    coords = {
        encoder: np.arange(total * 2, dtype=np.float32).reshape(total, 2)
        for encoder in subject.ENCODERS
    }
    return bundles, features, coords


def test_audit_recognizes_joint_seven_source_coordinate_filename(
    subject: Any,
    tmp_path: Path,
) -> None:
    bundles, features, coords = _audit_inputs(subject, tmp_path)
    output_dir = tmp_path / "output"
    path = output_dir / "dinov2" / "joint_seven_source_tsne_coordinates.csv"
    rows: list[dict[str, Any]] = []
    joint_index = 0
    for source in subject.SOURCE_ORDER:
        for local_index in range(len(bundles[source].records)):
            rows.append(
                {
                    "joint_index": joint_index,
                    "dataset": source,
                    "class_name": bundles[source].records[local_index].category,
                    "tsne_x": float(coords["dinov2"][joint_index, 0]),
                    "tsne_y": float(coords["dinov2"][joint_index, 1]),
                }
            )
            joint_index += 1
    _write_csv(path, rows)

    audit = subject.audit_outputs(
        output_dir,
        bundles=bundles,
        features=features,
        coords=coords,
        artifact_paths=[path],
    )

    rel = path.relative_to(output_dir).as_posix()
    assert audit["checks"][f"rows:{rel}"] is True
    assert audit["checks"][f"source_counts:{rel}"] is True
    assert audit["pass"] is True


def test_audit_derives_independent_coordinate_key_from_filename(
    subject: Any,
    tmp_path: Path,
) -> None:
    bundles, features, coords = _audit_inputs(subject, tmp_path)
    output_dir = tmp_path / "output"
    source = "PV-A"
    bundle = bundles[source]
    path = output_dir / "clip" / f"{bundle.key}_one_shot_tsne_coordinates.csv"
    _write_csv(
        path,
        [
            {
                "category": record.category,
                "dataset_id": record.dataset_id,
                "tsne_x": index,
                "tsne_y": index + 1,
            }
            for index, record in enumerate(bundle.records)
        ],
    )

    audit = subject.audit_outputs(
        output_dir,
        bundles=bundles,
        features=features,
        coords=coords,
        artifact_paths=[path],
    )

    rel = path.relative_to(output_dir).as_posix()
    assert audit["checks"][f"rows:{rel}"] is True
    assert audit["pass"] is True
