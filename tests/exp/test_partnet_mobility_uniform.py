from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import sys
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
RENDER_DRIVER = ROOT / "exp/scripts/render_partnet_mobility_uniform.py"


def load_subject(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def render_driver() -> Any:
    return load_subject("render_partnet_mobility_uniform_test_subject", RENDER_DRIVER)


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture_manifest(
    render_driver: Any,
    tmp_path: Path,
    rows_spec: tuple[tuple[str, str], ...] = (("asset-a", "alpha"), ("asset-b", "beta")),
) -> tuple[Path, Path]:
    source_root = tmp_path / "source"
    source_root.mkdir()
    rows = []
    for ordinal, (asset_id, category) in enumerate(rows_spec):
        package = source_root / asset_id
        package.mkdir()
        urdf = package / "mobility.urdf"
        urdf.write_text(
            f"<robot name='{asset_id}'><link name='base'/></robot>\n", encoding="utf-8"
        )
        meta = package / "meta.json"
        meta.write_text(
            json.dumps({"model_cat": category, "anno_id": asset_id}, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        package_files = [
            {"path": path.name, "sha256": _file_sha256(path), "size": path.stat().st_size}
            for path in sorted((meta, urdf), key=lambda path: path.name)
        ]
        nested_files = [
            {"bytes": entry["size"], "path": entry["path"], "sha256": entry["sha256"]}
            for entry in package_files
        ]
        package_binding = {
            "content_manifest_sha256": render_driver._canonical_sha256(nested_files),
            "file_count": len(package_files),
            "files": nested_files,
            "total_bytes": sum(entry["size"] for entry in package_files),
        }
        rows.append(
            {
                "asset_id": asset_id,
                "category": category,
                "ordinal": ordinal,
                "package_binding": package_binding,
                "package_binding_sha256": render_driver._canonical_sha256(package_files),
                "package_files": package_files,
                "parse_status": "valid",
                "primary_urdf_bytes": urdf.stat().st_size,
                "primary_urdf_path": str(urdf.resolve()),
                "primary_urdf_relative_path": f"{asset_id}/mobility.urdf",
                "primary_urdf_sha256": _file_sha256(urdf),
                "primary_urdf_size": urdf.stat().st_size,
                "raw_category": category,
                "source_path": str(package.resolve()),
                "source_relative_path": asset_id,
                "xml_parse_status": "valid",
            }
        )
    manifest = {
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


def test_frozen_full_release_selects_one_hash_winner_per_46_categories(
    render_driver: Any,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def reject_file_hash(*_args: Any, **_kwargs: Any) -> str:
        raise AssertionError("metadata-only roster loading must not hash package payloads")

    monkeypatch.setattr(render_driver, "_sha256", reject_file_hash)
    items = render_driver.load_render_items(
        render_driver.DEFAULT_DATASET_MANIFEST,
        output_root=tmp_path / "renders",
        strict_counts=True,
        validate_inputs=False,
    )

    winners = [item for item in items if item.category_one_shot]
    assert len(items) == 2_347
    assert len({item.asset_id for item in items}) == 2_347
    assert len({item.category for item in items}) == 46
    assert len(winners) == 46
    assert set(Counter(item.category for item in winners).values()) == {1}
    for category in {item.category for item in items}:
        candidates = [item for item in items if item.category == category]
        expected = min(candidates, key=lambda item: (item.identity_sha256, item.asset_id))
        selected = next(item for item in candidates if item.category_one_shot)
        assert selected.asset_id == expected.asset_id


def test_default_universe_is_full_release_not_sub16_split(render_driver: Any) -> None:
    assert render_driver.DEFAULT_DATASET_MANIFEST.name == "full_release_manifest.json"
    assert "table123_full_release_20260825/rosters/partnet" in str(
        render_driver.DEFAULT_DATASET_MANIFEST
    )
    assert "data-split.json" not in str(render_driver.DEFAULT_DATASET_MANIFEST)
    assert render_driver.EXPECTED_MODEL_COUNT == 2_347
    assert render_driver.EXPECTED_CATEGORY_COUNT == 46


def test_count_drift_accepts_self_consistent_manifest_but_strict_mode_rejects_it(
    render_driver: Any,
    tmp_path: Path,
) -> None:
    manifest, _source = _fixture_manifest(render_driver, tmp_path)
    with pytest.raises(ValueError, match="expected exactly"):
        render_driver.load_render_items(
            manifest, output_root=tmp_path / "strict", strict_counts=True
        )

    items = render_driver.load_render_items(
        manifest,
        output_root=tmp_path / "drift",
        strict_counts=False,
        validate_inputs=True,
    )

    assert len(items) == 2
    assert {item.category for item in items} == {"alpha", "beta"}
    assert all(item.category_one_shot for item in items)


def test_complete_package_closure_rejects_unbound_extra_file(
    render_driver: Any,
    tmp_path: Path,
) -> None:
    manifest_path, source_root = _fixture_manifest(
        render_driver, tmp_path, (("asset-a", "alpha"),)
    )
    _manifest, rows = render_driver._load_manifest(manifest_path, strict_counts=False)
    package = source_root / "asset-a"
    assert render_driver._validate_package_files(rows[0], package) == rows[0][
        "package_binding_sha256"
    ]

    (package / "unbound.txt").write_text("not in the frozen package receipt\n", encoding="utf-8")
    with pytest.raises(ValueError, match="package closure drift"):
        render_driver._validate_package_files(rows[0], package)


def test_package_binding_rejects_manifest_encoding_disagreement(
    render_driver: Any,
    tmp_path: Path,
) -> None:
    manifest_path, source_root = _fixture_manifest(
        render_driver, tmp_path, (("asset-a", "alpha"),)
    )
    _manifest, rows = render_driver._load_manifest(manifest_path, strict_counts=False)
    rows[0]["package_binding"]["files"][0]["bytes"] += 1

    with pytest.raises(ValueError, match="content manifest drift"):
        render_driver._validate_package_files(rows[0], source_root / "asset-a")


def test_package_binding_accepts_equivalent_legacy_ordering(
    render_driver: Any,
    tmp_path: Path,
) -> None:
    manifest_path, source_root = _fixture_manifest(
        render_driver, tmp_path, (("asset-a", "alpha"),)
    )
    _manifest, rows = render_driver._load_manifest(manifest_path, strict_counts=False)
    nested = rows[0]["package_binding"]
    nested["files"] = list(reversed(nested["files"]))
    nested["content_manifest_sha256"] = render_driver._canonical_sha256(nested["files"])

    assert render_driver._validate_package_files(
        rows[0], source_root / "asset-a"
    ) == rows[0]["package_binding_sha256"]


def test_config_binds_both_support_renderers_and_frozen_studio(
    render_driver: Any,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest_path, _source = _fixture_manifest(render_driver, tmp_path)
    items = render_driver.load_render_items(
        manifest_path, output_root=tmp_path / "renders", strict_counts=False
    )
    monkeypatch.setattr(render_driver, "_sha256", lambda _path: "f" * 64)
    monkeypatch.setattr(render_driver, "_blender_version", lambda _path: "Blender fixture")
    args = SimpleNamespace(
        dataset_manifest=manifest_path,
        output_root=tmp_path / "renders",
        allow_count_drift=True,
        one_shot_only=True,
        resolution=256,
        samples=4,
        gpu="0",
        workers=1,
        timeout_seconds=10.0,
    )

    config = render_driver.build_run_config(
        args=args,
        all_items=items,
        selected=items,
        renderer=render_driver.DEFAULT_RENDERER,
        base_renderer=render_driver.DEFAULT_BASE_RENDERER,
        shared_renderer=render_driver.DEFAULT_SHARED_RENDERER,
        blender=Path("/fixture/blender"),
    )

    assert config["base_renderer"] == str(render_driver.DEFAULT_BASE_RENDERER)
    assert config["base_renderer_sha256"] == "f" * 64
    assert config["shared_renderer"] == str(render_driver.DEFAULT_SHARED_RENDERER)
    assert config["shared_renderer_sha256"] == "f" * 64
    assert config["material_policy"] == render_driver.MATERIAL_POLICY
    assert config["selection"]["universe"].endswith("sub16 data-split.json is excluded")
    assert config["studio"] == {
        "mode": "opaque_studio",
        "cycles_denoising": True,
        "view_transform": "AgX",
        "look": "AgX - Medium High Contrast",
        "world_rgba": [0.8, 0.84, 0.9, 1.0],
        "world_strength": 0.55,
        "ground_rgba": [0.32, 0.35, 0.4, 1.0],
        "ground_roughness": 0.82,
        "camera_vertical_fov_degrees": 42.0,
        "camera_direction": [1.25, -1.35, 0.85],
        "camera_distance_policy": "bounding_sphere_auto_frame_1.18",
        "lights": [
            {"direction": [0.4, -0.8, 1.5], "gain": 42.0, "size_ratio": 1.5},
            {"direction": [-1.2, -0.3, 0.6], "gain": 15.0, "size_ratio": 1.8},
            {"direction": [0.2, 1.0, 1.2], "gain": 24.0, "size_ratio": 1.2},
        ],
    }


def test_render_command_passes_and_validates_both_support_receipts(
    render_driver: Any,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest_path, _source = _fixture_manifest(
        render_driver, tmp_path, (("asset-a", "alpha"),)
    )
    item = render_driver.load_render_items(
        manifest_path, output_root=tmp_path / "renders", strict_counts=False
    )[0]
    blender = tmp_path / "blender"
    worker = tmp_path / "worker.py"
    base_renderer = tmp_path / "base.py"
    shared_renderer = tmp_path / "shared.py"
    for path in (blender, worker, base_renderer, shared_renderer):
        path.write_text("fixture\n", encoding="utf-8")
    base_sha = _file_sha256(base_renderer)
    shared_sha = _file_sha256(shared_renderer)
    observed: dict[str, Any] = {}

    def fake_run(command: list[str], **kwargs: Any) -> Any:
        observed["command"] = command
        observed["environment"] = kwargs["env"]
        output = Path(command[command.index("--output") + 1])
        Image.new("RGBA", (256, 256), (80, 100, 120, 255)).save(output)
        receipt = {
            "asset_dir": str(item.source_path.resolve()),
            "output": str(output),
            "material_policy": render_driver.MATERIAL_POLICY,
            "base_renderer": {"path": str(base_renderer.resolve()), "sha256": base_sha},
            "shared_renderer": {"path": str(shared_renderer.resolve()), "sha256": shared_sha},
        }
        return SimpleNamespace(
            returncode=0, stdout=json.dumps(receipt, sort_keys=True) + "\n", stderr=""
        )

    monkeypatch.setattr(render_driver.subprocess, "run", fake_run)
    args = SimpleNamespace(
        force=True,
        resolution=256,
        samples=4,
        gpu="3",
        timeout_seconds=10.0,
        output_root=tmp_path / "renders",
    )
    result = render_driver._render_one(
        item,
        args=args,
        blender=blender.resolve(),
        renderer=worker.resolve(),
        base_renderer=base_renderer.resolve(),
        base_renderer_sha256=base_sha,
        shared_renderer=shared_renderer.resolve(),
        shared_renderer_sha256=shared_sha,
        reuse_receipt=None,
    )

    command = observed["command"]
    assert command[command.index("--base-renderer") + 1] == str(base_renderer.resolve())
    assert command[command.index("--base-renderer-sha256") + 1] == base_sha
    assert command[command.index("--shared-renderer") + 1] == str(shared_renderer.resolve())
    assert command[command.index("--shared-renderer-sha256") + 1] == shared_sha
    assert observed["environment"]["CUDA_VISIBLE_DEVICES"] == "3"
    assert result["status"] == "rendered"
    assert item.output_path.is_file()
    assert result["renderer_result"]["output"] == str(item.output_path)


def test_renderer_receipt_mismatch_is_fail_closed(
    render_driver: Any,
    tmp_path: Path,
) -> None:
    base = tmp_path / "base.py"
    shared = tmp_path / "shared.py"
    base.write_text("base\n", encoding="utf-8")
    shared.write_text("shared\n", encoding="utf-8")
    error = render_driver._validate_renderer_result(
        {
            "asset_dir": str(tmp_path),
            "output": str(shared),
            "material_policy": render_driver.MATERIAL_POLICY,
            "base_renderer": {"path": str(base), "sha256": "0" * 64},
            "shared_renderer": {"path": str(shared), "sha256": _file_sha256(shared)},
        },
        base_renderer=base.resolve(),
        base_renderer_sha256=_file_sha256(base),
        shared_renderer=shared.resolve(),
        shared_renderer_sha256=_file_sha256(shared),
        expected_asset_dir=tmp_path,
        expected_output=shared,
    )
    assert error == "renderer receipt base_renderer mismatch"


def test_reuse_requires_and_preserves_support_renderer_receipts(
    render_driver: Any,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest_path, _source = _fixture_manifest(
        render_driver, tmp_path, (("asset-a", "alpha"),)
    )
    item = render_driver.load_render_items(
        manifest_path, output_root=tmp_path / "renders", strict_counts=False
    )[0]
    item.output_path.parent.mkdir(parents=True)
    Image.new("RGBA", (256, 256), (80, 100, 120, 255)).save(item.output_path)
    base_renderer = tmp_path / "base.py"
    shared_renderer = tmp_path / "shared.py"
    base_renderer.write_text("base\n", encoding="utf-8")
    shared_renderer.write_text("shared\n", encoding="utf-8")
    base_sha = _file_sha256(base_renderer)
    shared_sha = _file_sha256(shared_renderer)
    renderer_result = {
        "asset_dir": str(item.source_path.resolve()),
        "output": str(item.output_path),
        "material_policy": render_driver.MATERIAL_POLICY,
        "base_renderer": {"path": str(base_renderer.resolve()), "sha256": base_sha},
        "shared_renderer": {"path": str(shared_renderer.resolve()), "sha256": shared_sha},
    }
    receipt = {
        **render_driver._item_row(item),
        "ordinal": str(item.ordinal),
        "status": "rendered",
        "png_bytes": str(item.output_path.stat().st_size),
        "png_sha256": _file_sha256(item.output_path),
        "renderer_result": json.dumps(renderer_result, sort_keys=True),
    }
    assert render_driver._receipt_allows_reuse(
        item,
        receipt,
        resolution=256,
        base_renderer=base_renderer.resolve(),
        base_renderer_sha256=base_sha,
        shared_renderer=shared_renderer.resolve(),
        shared_renderer_sha256=shared_sha,
    )
    monkeypatch.setattr(
        render_driver.subprocess,
        "run",
        lambda *_args, **_kwargs: pytest.fail("valid receipt must not launch Blender"),
    )
    result = render_driver._render_one(
        item,
        args=SimpleNamespace(
            force=False,
            resolution=256,
            samples=4,
            gpu="0",
            timeout_seconds=10.0,
            output_root=tmp_path / "renders",
        ),
        blender=tmp_path / "blender",
        renderer=tmp_path / "worker.py",
        base_renderer=base_renderer.resolve(),
        base_renderer_sha256=base_sha,
        shared_renderer=shared_renderer.resolve(),
        shared_renderer_sha256=shared_sha,
        reuse_receipt=receipt,
    )
    assert result["status"] == "reused_valid"
    assert result["renderer_result"] == renderer_result


def test_dry_run_writes_nothing(
    render_driver: Any,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest_path, _source = _fixture_manifest(
        render_driver, tmp_path, (("asset-a", "alpha"),)
    )
    support_paths = []
    for name in ("blender", "worker.py", "base.py", "shared.py"):
        path = tmp_path / name
        path.write_text("fixture\n", encoding="utf-8")
        support_paths.append(path)
    blender, worker, base_renderer, shared_renderer = support_paths
    output_root = tmp_path / "dry-run-output"
    monkeypatch.setattr(render_driver, "_blender_version", lambda _path: "Blender fixture")
    args = render_driver.build_argument_parser().parse_args(
        [
            "--dataset-manifest",
            str(manifest_path),
            "--output-root",
            str(output_root),
            "--renderer",
            str(worker),
            "--base-renderer",
            str(base_renderer),
            "--shared-renderer",
            str(shared_renderer),
            "--blender",
            str(blender),
            "--allow-count-drift",
            "--dry-run",
        ]
    )

    result = render_driver.run(args)

    assert result["status"] == "dry_run"
    assert result["selection"]["selected_count"] == 1
    assert not output_root.exists()


def test_recovery_journal_latest_row_overrides_checkpoint(
    render_driver: Any,
    tmp_path: Path,
) -> None:
    manifest = tmp_path / "render_manifest.csv"
    state = tmp_path / "render_state.jsonl"
    manifest.write_text(
        "asset_id,ordinal,status,error\nasset-a,7,rendered,\n", encoding="utf-8"
    )
    state.write_text(
        "\n".join(
            json.dumps(row, sort_keys=True)
            for row in (
                {"asset_id": "asset-a", "ordinal": 7, "status": "failed"},
                {"asset_id": "asset-a", "ordinal": 7, "status": "reused_valid"},
            )
        )
        + "\n",
        encoding="utf-8",
    )

    receipts = render_driver._read_recovery_receipts(
        manifest_path=manifest,
        state_path=state,
        roster={"asset-a": SimpleNamespace(ordinal=7)},
    )
    assert receipts["asset-a"]["status"] == "reused_valid"


def test_one_shot_roster_is_category_sorted(
    render_driver: Any,
    tmp_path: Path,
) -> None:
    manifest_path, _source = _fixture_manifest(
        render_driver,
        tmp_path,
        (("asset-z", "zeta"), ("asset-a", "alpha")),
    )
    items = render_driver.load_render_items(
        manifest_path, output_root=tmp_path / "renders", strict_counts=False
    )
    roster = tmp_path / "category_one_shot_roster.csv"
    render_driver._write_one_shot_roster(roster, items)
    with roster.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert [row["category"] for row in rows] == ["alpha", "zeta"]
    assert all(row["category_one_shot"] == "True" for row in rows)
