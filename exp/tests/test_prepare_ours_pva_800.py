from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest


REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / "exp/scripts/prepare_ours_pva_800.py"


def load_module():
    spec = importlib.util.spec_from_file_location("prepare_ours_pva_800", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_rank_uses_slug_so_repeated_seed_ids_remain_distinct() -> None:
    module = load_module()

    first = module.rank_row(
        "alpha", "seed_0001", manifest_sha256="a" * 64,
        seed="arti-skill-ours-pva-n300-v1",
    )
    second = module.rank_row(
        "beta", "seed_0001", manifest_sha256="a" * 64,
        seed="arti-skill-ours-pva-n300-v1",
    )

    assert first == "7fb9263cb174c24be9ccf6464c9bc63240ff201a426944c235fd258a281048e8"
    assert second == "f04733976db94ec47edc8daa8e0539c805a5dd5dff84425743116cbd58dbc8bd"


def test_select_rows_is_deterministic_and_does_not_mutate_manifest_order() -> None:
    module = load_module()
    rows = [
        {"slug": "alpha", "asset_id": "seed_0001", "seed": "1"},
        {"slug": "beta", "asset_id": "seed_0001", "seed": "1"},
        {"slug": "alpha", "asset_id": "seed_0401", "seed": "401"},
        {"slug": "gamma", "asset_id": "seed_0002", "seed": "2"},
    ]

    selected = module.select_rows(
        rows, 2, manifest_sha256="a" * 64,
        seed="arti-skill-ours-pva-n300-v1",
    )

    assert [(row["slug"], row["asset_id"]) for row in selected] == [
        ("alpha", "seed_0401"),
        ("alpha", "seed_0001"),
    ]
    assert [row["slug"] for row in rows] == ["alpha", "beta", "alpha", "gamma"]


@pytest.mark.parametrize(
    ("slug", "asset_id", "names", "expected"),
    [
        ("small", "seed_0007", {"small.tar.zst"}, "small.tar.zst"),
        ("large", "seed_0000", {"large_part00.tar.zst", "large_part01.tar.zst"}, "large_part00.tar.zst"),
        ("large", "seed_0401", {"large_part00.tar.zst", "large_part01.tar.zst"}, "large_part01.tar.zst"),
    ],
)
def test_archive_resolution_handles_whole_and_400_asset_shards(
    slug: str, asset_id: str, names: set[str], expected: str
) -> None:
    module = load_module()

    assert module.resolve_archive_name(slug, asset_id, names) == expected


def test_archive_resolution_rejects_missing_expected_shard() -> None:
    module = load_module()

    with pytest.raises(ValueError, match="archive unavailable"):
        module.resolve_archive_name("large", "seed_0401", {"large_part00.tar.zst"})


def test_archive_resolution_rejects_noncanonical_asset_id() -> None:
    module = load_module()

    with pytest.raises(ValueError, match="invalid PV-A asset_id"):
        module.resolve_archive_name("large", "seed-401", {"large_part01.tar.zst"})


def test_merge_cohort_requires_500_brain_and_300_unique_pva_assets() -> None:
    module = load_module()
    brain = [{"dataset_id": f"brain/{index}"} for index in range(500)]
    pva = [{"dataset_id": f"pva/slug/seed_{index:04d}"} for index in range(300)]

    merged = module.merge_rows(brain, pva)

    assert len(merged) == 800
    assert [row["selection_index"] for row in merged] == list(range(800))
    with pytest.raises(ValueError, match="duplicate dataset identity"):
        module.merge_rows(brain, pva[:-1] + [pva[0]])


def test_rebase_staging_packages_targets_final_cohort_without_touching_brain(
    tmp_path: Path,
) -> None:
    module = load_module()
    staging = tmp_path / ".cohort.work"
    output = tmp_path / "cohort"
    brain_package = tmp_path / "brain" / "seed_0"
    rows = [
        {"dataset_id": "brain/seed_0", "package": str(brain_package)},
        {
            "dataset_id": "PV-A/alpha/seed_0001",
            "package": str(staging / "pva_assets" / "alpha" / "seed_0001"),
        },
    ]

    rebased = module.rebase_staging_packages(rows, staging, output)

    assert rebased[0]["package"] == str(brain_package)
    assert rebased[1]["package"] == str(
        output / "pva_assets" / "alpha" / "seed_0001"
    )
    assert rows[1]["package"].startswith(str(staging))


def test_selective_extraction_handles_directory_and_explicit_child_members(
    tmp_path: Path,
) -> None:
    module = load_module()
    source = tmp_path / "source"
    package = source / "seed_0012"
    (package / "assets" / "meshes").mkdir(parents=True)
    (package / "model.urdf").write_text("<robot name='fixture'/>\n", encoding="utf-8")
    (package / "appearance.json").write_text("{}\n", encoding="utf-8")
    (package / "physics.json").write_text("{}\n", encoding="utf-8")
    (package / "assets" / "meshes" / "part.obj").write_text("v 0 0 0\n", encoding="utf-8")
    archive = tmp_path / "fixture.tar.zst"
    subprocess.run(
        ["tar", "--zstd", "-cf", str(archive), "-C", str(source), "seed_0012"],
        check=True,
    )
    destination = tmp_path / "extracted"

    module.extract_selected_archive(archive, ["seed_0012"], destination)

    assert (destination / "seed_0012" / "model.urdf").is_file()
    assert (destination / "seed_0012" / "assets" / "meshes" / "part.obj").is_file()
