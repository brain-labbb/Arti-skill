from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import pytest
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SUBJECT = ROOT / "exp/scripts/visualize_pva531_n5_tsne.py"


def _load_subject() -> Any:
    spec = importlib.util.spec_from_file_location("visualize_pva531_n5_tsne", SUBJECT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture()
def subject() -> Any:
    return _load_subject()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_csv(
    path: Path,
    rows: Sequence[Mapping[str, Any]],
    *,
    fieldnames: Sequence[str] | None = None,
) -> None:
    assert rows
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames or list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _make_n5_fixture(tmp_path: Path) -> tuple[Path, Path, Path]:
    render_root = tmp_path / "uniform_n5"
    render_root.mkdir()
    index_csv = tmp_path / "generator_picture_index.csv"
    index_rows = [
        {
            "generator_index": "G0001",
            "generator_name": "picture_generator",
            "source_type": "picture_backed",
            "picture_category": "Tools",
            "picture_label": "Tool",
            "picture_source_path": "unused",
        },
        {
            "generator_index": "G0002",
            "generator_name": "builtin_generator",
            "source_type": "articraft_builtin_dataset_no_picture",
            "picture_category": "",
            "picture_label": "",
            "picture_source_path": "",
        },
    ]
    _write_csv(index_csv, index_rows)

    manifest_rows: list[dict[str, Any]] = []
    roster_rows: list[dict[str, Any]] = []
    ordinal = 0
    for class_offset, index_row in enumerate(index_rows):
        for sample_index in range(1, 6):
            ordinal += 1
            generator_index = index_row["generator_index"]
            generator_name = index_row["generator_name"]
            asset_id = f"seed_{class_offset * 10 + sample_index:04d}"
            render_key = f"{generator_index}__S{sample_index:02d}__{asset_id}"
            image_path = render_root / f"{render_key}__{generator_name}.png"
            Image.new(
                "RGB",
                (8, 8),
                (20 + ordinal, 40 + class_offset * 30, 60 + sample_index),
            ).save(image_path)
            common = {
                "ordinal": ordinal,
                "render_key": render_key,
                "generator_index": generator_index,
                "generator_name": generator_name,
                "sample_index": sample_index,
                "source_type": index_row["source_type"],
                "picture_category": index_row["picture_category"],
                "asset_id": asset_id,
                "seed": class_offset * 10 + sample_index,
                "rank_sha256": f"{ordinal:064x}",
                "asset_dir": str((tmp_path / "assets" / generator_name / asset_id).resolve()),
                "urdf_sha256": "a" * 64,
                "package_content_sha256": "b" * 64,
                "output_path": str(image_path.resolve()),
            }
            roster_rows.append(common)
            manifest_rows.append(
                {
                    **common,
                    "gpu": "0",
                    "status": "rendered",
                    "elapsed_seconds": "0.1",
                    "png_bytes": image_path.stat().st_size,
                    "png_sha256": _sha256(image_path),
                    "started_at": "2026-08-31T00:00:00Z",
                    "finished_at": "2026-08-31T00:00:01Z",
                    "error": "",
                    "renderer_result": "{}",
                }
            )
    roster_path = render_root / "render_roster.csv"
    _write_csv(roster_path, roster_rows)
    _write_csv(render_root / "render_manifest.csv", manifest_rows)
    config = {
        "schema_version": 1,
        "render_contract": "pva531_n5_uniform_studio_v1",
        "class_count": 2,
        "asset_count": 10,
        "per_class": 5,
        "per_class_count_values": [5],
        "index_csv": str(index_csv.resolve()),
        "index_csv_sha256": _sha256(index_csv),
        "render_roster": str(roster_path.resolve()),
        "render_roster_sha256": _sha256(roster_path),
        "input_receipt": {
            "asset_and_dependency_sha256": "c" * 64,
            "file_count": 10,
            "total_bytes": 100,
        },
        "resolution": 8,
        "samples": 1,
    }
    (render_root / "render_config.json").write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    color_index = tmp_path / "generator_class_color_index.csv"
    _write_csv(
        color_index,
        [
            {
                "generator_index": "G0001",
                "generator_name": "picture_generator",
                "color_hex": "#123456",
            },
            {
                "generator_index": "G0002",
                "generator_name": "builtin_generator",
                "color_hex": "#abcdef",
            },
        ],
    )
    return index_csv, render_root, color_index


def _rewrite_roster_and_refresh_config(
    render_root: Path,
    rows: Sequence[Mapping[str, Any]],
) -> None:
    roster_path = render_root / "render_roster.csv"
    _write_csv(roster_path, rows)
    config_path = render_root / "render_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config["render_roster_sha256"] = _sha256(roster_path)
    config_path.write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def test_n5_discovery_keeps_exactly_five_ordered_samples_per_class(
    subject: Any,
    tmp_path: Path,
) -> None:
    index_csv, render_root, _ = _make_n5_fixture(tmp_path)

    bundle = subject.discover_n5_render_records(
        index_csv,
        render_root=render_root,
        strict_counts=False,
        expected_per_class=5,
    )

    assert len(bundle.records) == 2
    assert len(bundle.samples) == 10
    assert len(bundle.raw_image_paths) == 10
    assert [record.generator_index for record in bundle.records] == ["G0001", "G0002"]
    for record in bundle.records:
        assert len(record.image_paths) == 5
        class_samples = [
            sample for sample in bundle.samples
            if sample.generator_index == record.generator_index
        ]
        assert [sample.sample_index for sample in class_samples] == [1, 2, 3, 4, 5]
        assert [sample.image_path for sample in class_samples] == list(record.image_paths)
    assert bundle.summary["class_count"] == 2
    assert bundle.summary["raw_unique_image_count"] == 10
    assert bundle.summary["per_class"] == 5


def test_n5_discovery_rejects_missing_or_duplicate_sample_index(
    subject: Any,
    tmp_path: Path,
) -> None:
    index_csv, render_root, _ = _make_n5_fixture(tmp_path)
    manifest_path = render_root / "render_manifest.csv"
    with manifest_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
        fieldnames = list(rows[0])
    rows[4]["sample_index"] = "4"
    _write_csv(manifest_path, rows, fieldnames=fieldnames)

    with pytest.raises(ValueError, match=r"sample(_index)?.*(1.*5|duplicate|exact)"):
        subject.discover_n5_render_records(
            index_csv,
            render_root=render_root,
            strict_counts=False,
            expected_per_class=5,
        )


def test_n5_discovery_rejects_duplicate_asset_id_within_a_class(
    subject: Any,
    tmp_path: Path,
) -> None:
    index_csv, render_root, _ = _make_n5_fixture(tmp_path)
    roster_path = render_root / "render_roster.csv"
    manifest_path = render_root / "render_manifest.csv"
    with roster_path.open("r", encoding="utf-8", newline="") as stream:
        roster_rows = list(csv.DictReader(stream))
    with manifest_path.open("r", encoding="utf-8", newline="") as stream:
        manifest_rows = list(csv.DictReader(stream))
    assert roster_rows[0]["generator_index"] == roster_rows[1]["generator_index"]
    roster_rows[1]["asset_id"] = roster_rows[0]["asset_id"]
    manifest_rows[1]["asset_id"] = manifest_rows[0]["asset_id"]
    _rewrite_roster_and_refresh_config(render_root, roster_rows)
    _write_csv(manifest_path, manifest_rows)

    with pytest.raises(
        ValueError,
        match=r"duplicate.*asset_id|asset_id.*duplicate|asset_id.*unique",
    ):
        subject.discover_n5_render_records(
            index_csv,
            render_root=render_root,
            strict_counts=False,
            expected_per_class=5,
        )


def test_n5_discovery_rejects_duplicate_png_sha_within_a_class(
    subject: Any,
    tmp_path: Path,
) -> None:
    index_csv, render_root, _ = _make_n5_fixture(tmp_path)
    manifest_path = render_root / "render_manifest.csv"
    with manifest_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["generator_index"] == rows[1]["generator_index"]
    first_image = Path(rows[0]["output_path"])
    second_image = Path(rows[1]["output_path"])
    second_image.write_bytes(first_image.read_bytes())
    rows[1]["png_bytes"] = str(second_image.stat().st_size)
    rows[1]["png_sha256"] = _sha256(second_image)
    _write_csv(manifest_path, rows)

    with pytest.raises(ValueError, match=r"duplicate.*PNG.*SHA|PNG.*SHA.*duplicate"):
        subject.discover_n5_render_records(
            index_csv,
            render_root=render_root,
            strict_counts=False,
            expected_per_class=5,
        )


def test_strict_release_still_requires_531_classes(
    subject: Any,
    tmp_path: Path,
) -> None:
    index_csv, render_root, _ = _make_n5_fixture(tmp_path)

    with pytest.raises(ValueError, match="531"):
        subject.discover_n5_render_records(
            index_csv,
            render_root=render_root,
            strict_counts=True,
            expected_per_class=5,
        )


def test_frozen_color_map_is_unique_and_shared_by_all_five_class_samples(
    subject: Any,
    tmp_path: Path,
) -> None:
    index_csv, render_root, color_index = _make_n5_fixture(tmp_path)
    bundle = subject.discover_n5_render_records(
        index_csv,
        render_root=render_root,
        strict_counts=False,
        expected_per_class=5,
    )

    color_map = subject.build_class_color_map(
        bundle.records,
        color_index_csv=color_index,
    )

    assert color_map == {"G0001": "#123456", "G0002": "#abcdef"}
    sample_colors = [color_map[sample.generator_index] for sample in bundle.samples]
    assert sample_colors[:5] == ["#123456"] * 5
    assert sample_colors[5:] == ["#abcdef"] * 5

    bad_rows = [
        {
            "generator_index": "G0001",
            "generator_name": "wrong_name",
            "color_hex": "#123456",
        },
        {
            "generator_index": "G0002",
            "generator_name": "builtin_generator",
            "color_hex": "#abcdef",
        },
    ]
    _write_csv(color_index, bad_rows)
    with pytest.raises(ValueError, match="generator.*(name|identity)|align"):
        subject.build_class_color_map(bundle.records, color_index_csv=color_index)


def test_output_audit_accepts_exact_finite_coordinates_and_rejects_drift(
    subject: Any,
    tmp_path: Path,
) -> None:
    index_csv, render_root, color_index = _make_n5_fixture(tmp_path)
    bundle = subject.discover_n5_render_records(
        index_csv,
        render_root=render_root,
        strict_counts=False,
        expected_per_class=5,
    )
    color_map = subject.build_class_color_map(
        bundle.records,
        color_index_csv=color_index,
    )
    coordinates = {
        "dinov2": np.arange(20, dtype=np.float32).reshape(10, 2),
        "clip": np.arange(20, 40, dtype=np.float32).reshape(10, 2),
    }

    audit = subject.audit_tsne_outputs(bundle, color_map, coordinates)

    assert audit["pass"] is True
    assert all(audit["checks"].values())
    assert audit["counts"]["sample_count"] == 10
    assert audit["counts"]["class_count"] == 2
    assert audit["counts"]["color_count"] == 2
    assert audit["counts"]["samples_per_class"] == [5]

    with pytest.raises(ValueError, match="coordinate.*count|sample.*count"):
        subject.audit_tsne_outputs(
            bundle,
            color_map,
            {**coordinates, "clip": coordinates["clip"][:-1]},
        )
    nonfinite = coordinates["dinov2"].copy()
    nonfinite[0, 0] = np.nan
    with pytest.raises(ValueError, match="finite|NaN|Inf"):
        subject.audit_tsne_outputs(
            bundle,
            color_map,
            {**coordinates, "dinov2": nonfinite},
        )
    with pytest.raises(ValueError, match="color.*(count|coverage|mapping)"):
        subject.audit_tsne_outputs(
            bundle,
            {"G0001": "#123456"},
            coordinates,
        )


def test_strict_feature_cache_requires_matching_contract_and_raw_feature_receipt(
    subject: Any,
    tmp_path: Path,
) -> None:
    model_dir = tmp_path / "dinov2"
    model_dir.mkdir()
    raw_feature_path = model_dir / "raw_image_features.npy"
    np.save(
        raw_feature_path,
        np.arange(40, dtype=np.float32).reshape(10, 4),
        allow_pickle=False,
    )
    requested_contract = {
        "schema_version": 1,
        "encoder": "dinov2",
        "model_fingerprint": "a" * 64,
        "ordered_input_receipt_sha256": "b" * 64,
        "sample_count": 10,
    }
    feature_manifest = {
        "schema_version": 2,
        "strict_cache_contract": requested_contract,
        "raw_feature_receipt": {
            "path": "raw_image_features.npy",
            "size_bytes": raw_feature_path.stat().st_size,
            "sha256": _sha256(raw_feature_path),
        },
    }
    (model_dir / "feature_manifest.json").write_text(
        json.dumps(feature_manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    assert subject._strict_feature_cache_valid(model_dir, requested_contract) is True

    # Keep the same array shape and dtype so only the content receipt detects drift.
    np.save(
        raw_feature_path,
        np.arange(40, dtype=np.float32).reshape(10, 4) + 1,
        allow_pickle=False,
    )
    assert raw_feature_path.stat().st_size == feature_manifest["raw_feature_receipt"]["size_bytes"]
    assert subject._strict_feature_cache_valid(model_dir, requested_contract) is False

    np.save(
        raw_feature_path,
        np.arange(40, dtype=np.float32).reshape(10, 4),
        allow_pickle=False,
    )
    changed_contract = {**requested_contract, "sample_count": 11}
    assert subject._strict_feature_cache_valid(model_dir, changed_contract) is False
