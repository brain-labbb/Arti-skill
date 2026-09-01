from __future__ import annotations

import csv
import importlib.util
import sys
from collections import Counter
from pathlib import Path

import pytest
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp/scripts/render_lam_n5_uniform.py"


@pytest.fixture(scope="module")
def subject():
    spec = importlib.util.spec_from_file_location("render_lam_n5_uniform_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_frozen_candidate_and_selection_counts(subject, tmp_path: Path) -> None:
    metadata, candidates = subject.load_candidates(subject.DEFAULT_DATASET_MANIFEST)
    assert metadata["row_count"] == 3217
    assert len(candidates) == 2832
    assert len({item.category for item in candidates}) == 660
    selected, stats = subject.select_samples(
        candidates,
        output_root=tmp_path / "renders",
        source_root=subject.DEFAULT_SOURCE_ROOT,
    )
    counts = Counter(item.candidate.category for item in selected)
    assert len(selected) == 1279
    assert len(counts) == 660
    assert sorted(set(counts.values())) == [1, 2, 3, 4, 5]
    assert stats["per_class_count_values"] == [1, 2, 3, 4, 5]
    assert stats["balanced_n5_eligible"] == 93
    assert stats["short_category_count"] == 567
    assert len({item.candidate.asset_id for item in selected}) == len(selected)


def test_sample_one_matches_existing_one_shot_winner(subject, tmp_path: Path) -> None:
    _metadata, candidates = subject.load_candidates(subject.DEFAULT_DATASET_MANIFEST)
    selected, _stats = subject.select_samples(
        candidates,
        output_root=tmp_path / "renders",
        source_root=subject.DEFAULT_SOURCE_ROOT,
    )
    first = {item.candidate.category: item for item in selected if item.sample_index == 1}
    with subject.DEFAULT_REUSE_ROOT.joinpath("render_manifest.csv").open(
        "r", encoding="utf-8", newline=""
    ) as stream:
        old = {(row["category"], row["asset_id"]): row for row in csv.DictReader(stream)}
    assert len(first) == 660
    for category, sample in first.items():
        matches = [row for (cat, _asset), row in old.items() if cat == category]
        assert len(matches) == 1
        assert sample.candidate.asset_id == matches[0]["asset_id"]


def test_balanced_selection_is_exactly_five(subject, tmp_path: Path) -> None:
    _metadata, candidates = subject.load_candidates(subject.DEFAULT_DATASET_MANIFEST)
    selected, stats = subject.select_samples(
        candidates,
        output_root=tmp_path / "renders",
        source_root=subject.DEFAULT_SOURCE_ROOT,
        balanced_only=True,
    )
    counts = Counter(item.candidate.category for item in selected)
    assert len(counts) == 93
    assert len(selected) == 465
    assert set(counts.values()) == {5}
    assert stats["class_count"] == 93


def test_roster_row_contains_shared_loader_fields(subject, tmp_path: Path) -> None:
    candidate = subject.Candidate(
        category="alpha",
        asset_id="alpha_000",
        tier="viable",
        rel_path="objects/alpha/alpha_000",
        identity="viable:objects/alpha/alpha_000",
        identity_sha256="a" * 64,
        selection_sha256="b" * 64,
    )
    selected, _ = subject.select_samples(
        [candidate], output_root=tmp_path / "renders", source_root=tmp_path
    )
    row = subject._item_row(selected[0])
    required = {
        "render_key",
        "generator_index",
        "generator_name",
        "sample_index",
        "source_type",
        "asset_id",
        "output_path",
        "png_bytes",
        "png_sha256",
    }
    assert required.issubset(row)
    assert row["source_type"] == "lam"


def test_reuse_validates_png_receipt(subject, tmp_path: Path) -> None:
    candidate = subject.Candidate(
        category="alpha",
        asset_id="alpha_000",
        tier="viable",
        rel_path="objects/alpha/alpha_000",
        identity="viable:objects/alpha/alpha_000",
        identity_sha256="a" * 64,
        selection_sha256="b" * 64,
    )
    selected, _ = subject.select_samples(
        [candidate], output_root=tmp_path / "new", source_root=tmp_path
    )
    sample = selected[0]
    old = tmp_path / "old.png"
    Image.new("RGB", (256, 256), (30, 40, 50)).save(old)
    manifest = tmp_path / "render_manifest.csv"
    manifest.write_text(
        "category,asset_id,status,output_path,png_bytes,png_sha256,identity_sha256,package_binding_sha256\n"
        f"alpha,alpha_000,rendered,{old},{old.stat().st_size},{subject._sha256(old)},"
        f"{'a' * 64},\n",
        encoding="utf-8",
    )
    path, baseline = subject._read_reuse_manifest(tmp_path)
    assert path == manifest
    result = subject._external_reuse(
        sample,
        reuse_root=tmp_path,
        reuse_manifest=path,
        baseline=baseline,
        resolution=256,
        force=False,
    )
    assert result is not None and result["status"] == "reused_valid"
    assert sample.item.output_path.is_file()
    assert subject._sha256(sample.item.output_path) == subject._sha256(old)
