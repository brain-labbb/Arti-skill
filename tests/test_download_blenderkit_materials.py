from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import sys
import threading
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / "download_blenderkit_materials.py"
SPEC = importlib.util.spec_from_file_location(
    "download_blenderkit_materials", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
downloader = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = downloader
SPEC.loader.exec_module(downloader)


class _Session:
    def close(self) -> None:
        pass


class _Response:
    def __init__(self, payload: object) -> None:
        self._payload = payload

    def json(self) -> object:
        return self._payload


def _material_tree() -> dict[str, object]:
    return {
        "name": "Material",
        "slug": "material",
        "asset_count": 0,
        "asset_count_cumulative": 20,
        "children": [
            {
                "name": "Glass",
                "slug": "glass",
                "asset_count": 3,
                "asset_count_cumulative": 8,
                "children": [
                    {
                        "name": "Patterned Glass",
                        "slug": "patterned-glass",
                        "asset_count": 2,
                        "asset_count_cumulative": 2,
                        "children": [],
                    }
                ],
            },
            {
                "name": "Metal",
                "slug": "metal",
                "asset_count": 5,
                "asset_count_cumulative": 12,
                "children": [],
            },
        ],
    }


def _write_taxonomy(path: Path) -> None:
    tree = _material_tree()
    metadata = downloader._taxonomy_metadata(tree)
    payload = {
        "schema_version": 2,
        "source": "blenderkit",
        "material_tree": tree,
        **metadata,
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def _write_profiles(path: Path, *, category_slug: str = "glass") -> None:
    payload = {
        "schema_version": 2,
        "defaults": {
            "license_policy": "record_only",
            "accepted_licenses": ["cc_zero", "royalty_free"],
            "file_types": ["resolution_1K", "resolution_0_5K"],
            "max_file_bytes": 1024 * 1024,
        },
        "profiles": [
            {
                "id": "canary_glass",
                "category_slug": category_slug,
                "queries": ["architectural"],
                "finish_hints": ["frosted"],
                "quota": 1,
                "shader_profile": "optical_principled",
                "require_pure_pbr": True,
                "require_nonprocedural": True,
                "realistic_only": True,
                "max_per_author": 1,
            }
        ],
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def _asset(
    *,
    license_name: str = "royalty_free",
    asset_base_id: str = "base-1",
) -> dict[str, object]:
    return {
        "id": f"version-{asset_base_id}",
        "assetBaseId": asset_base_id,
        "versionNumber": 1,
        "name": "Frosted Architectural Glass",
        "displayName": "Frosted Architectural Glass",
        "assetType": "material",
        "category": "patterned-glass",
        "isFree": True,
        "access": "free",
        "license": license_name,
        "canDownload": True,
        "verificationStatus": "validated",
        "tags": ["glass", "frosted"],
        "description": "A frosted glass material",
        "score": 50,
        "ratingsCount": {"bookmarks": 10},
        "author": {"id": 7, "fullName": "Material Author"},
        "thumbnailMiddleUrlNonsquaredWebp": "https://cdn.invalid/preview.webp",
        "dictParameters": {
            "purePbr": True,
            "procedural": False,
            "materialStyle": "realistic",
            "pbrType": "metallic",
            "textureSizeMeters": 1.0,
        },
        "files": [
            {
                "id": 12,
                "fileType": "resolution_0_5K",
                "fileUploadSize": 2048,
                "downloadUrl": "https://api.invalid/file/12",
                "filename": "small.blend",
            },
            {
                "id": 11,
                "fileType": "resolution_1K",
                "fileUploadSize": 4096,
                "downloadUrl": "https://api.invalid/file/11",
                "filename": "one-k.blend",
            },
            {
                "id": 13,
                "fileType": "blend",
                "fileUploadSize": 8192,
                "downloadUrl": "https://api.invalid/file/13",
                "filename": "native.blend",
            },
        ],
    }


def _build_plan(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    taxonomy_path = tmp_path / "taxonomy.json"
    profiles_path = tmp_path / "profiles.json"
    plan_path = tmp_path / "plan.json"
    _write_taxonomy(taxonomy_path)
    _write_profiles(profiles_path)
    seen_queries: list[str] = []

    def fake_search_assets(
        session: object,
        query: str,
        *,
        limit: int,
        page_size: int,
        retries: int,
    ) -> list[dict[str, object]]:
        del session, limit, retries
        assert page_size == 80
        seen_queries.append(query)
        return [_asset()]

    monkeypatch.setattr(downloader, "_new_session", lambda **_kwargs: _Session())
    monkeypatch.setattr(downloader, "_search_assets", fake_search_assets)
    args = argparse.Namespace(
        taxonomy=taxonomy_path,
        profiles=profiles_path,
        accepted_licenses=None,
        file_types=None,
        max_file_bytes=None,
        pool_multiplier=2,
        page_size=80,
        retries=1,
        budget_gib=0.01,
        strict=True,
        execute=True,
        output=plan_path,
    )
    assert downloader.build_plan(args) == 0
    assert seen_queries
    assert all("asset_type:material" in query for query in seen_queries)
    assert all("is_free:true" in query for query in seen_queries)
    assert all("category_subtree:glass" in query for query in seen_queries)
    assert all("license:" not in query for query in seen_queries)
    return plan_path


def test_cli_registers_category_first_pipeline() -> None:
    parser = downloader.build_parser()
    for command in ("snapshot-categories", "build-plan", "sync-plan"):
        args = parser.parse_args([command])
        assert callable(args.handler)
    args = parser.parse_args(
        [
            "build-plan",
            "--exclude-catalog",
            "raw.jsonl",
            "--exclude-catalog",
            "normalized.jsonl",
        ]
    )
    assert args.exclude_catalog == [Path("raw.jsonl"), Path("normalized.jsonl")]
    sync_args = parser.parse_args(["sync-plan", "--workers", "8", "--resume-existing"])
    assert sync_args.workers == 8
    assert sync_args.resume_existing is True
    with pytest.raises(SystemExit):
        parser.parse_args(["sync-plan", "--overwrite", "--resume-existing"])


def test_recursive_taxonomy_hash_and_top_level_validation(tmp_path: Path) -> None:
    taxonomy_path = tmp_path / "taxonomy.json"
    _write_taxonomy(taxonomy_path)
    payload, top_level = downloader._load_taxonomy(taxonomy_path)
    assert top_level == {"glass", "metal"}
    patterned = next(
        row for row in payload["slug_index"] if row["slug"] == "patterned-glass"
    )
    assert patterned["depth"] == 2
    assert patterned["top_level_slug"] == "glass"

    payload["material_tree"]["children"][0]["children"][0]["name"] = "Changed"
    taxonomy_path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="integrity"):
        downloader._load_taxonomy(taxonomy_path)


def test_snapshot_categories_recurses_without_network(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    raw_tree = {
        "name": "Material",
        "slug": "material",
        "assetCount": 0,
        "assetCountCumulative": 20,
        "children": [
            {
                "name": "Glass",
                "slug": "glass",
                "assetCount": 3,
                "assetCountCumulative": 8,
                "children": [
                    {
                        "name": "Patterned Glass",
                        "slug": "patterned-glass",
                        "assetCount": 2,
                        "assetCountCumulative": 2,
                        "children": [],
                    }
                ],
            }
        ],
    }
    monkeypatch.setattr(downloader, "_new_session", lambda **_kwargs: _Session())
    monkeypatch.setattr(
        downloader,
        "_request",
        lambda *_args, **_kwargs: _Response({"results": [raw_tree]}),
    )
    seen_count_queries: list[str] = []

    def fake_count(query: str, *, retries: int) -> int:
        del retries
        seen_count_queries.append(query)
        return 7

    monkeypatch.setattr(downloader, "_search_count", fake_count)
    output = tmp_path / "taxonomy.json"
    args = argparse.Namespace(
        retries=1,
        workers=1,
        licenses=["cc_zero", "royalty_free"],
        execute=True,
        output=output,
    )
    assert downloader.snapshot_categories(args) == 0
    payload, top_level = downloader._load_taxonomy(output)
    assert top_level == {"glass"}
    assert payload["material_tree"]["children"][0]["children"][0]["slug"] == (
        "patterned-glass"
    )
    assert len(seen_count_queries) == 3
    assert all("is_free:true" in query for query in seen_count_queries)


def test_profiles_use_one_official_category_and_explicit_file_policy(
    tmp_path: Path,
) -> None:
    profiles_path = tmp_path / "profiles.json"
    _write_profiles(profiles_path)
    _defaults, profiles = downloader._load_profiles(
        profiles_path,
        official_top_level_slugs={"glass", "metal"},
    )
    assert profiles[0].category_slug == "glass"
    assert profiles[0].file_types == ("resolution_1K", "resolution_0_5K")
    assert profiles[0].require_validated is True
    assert profiles[0].require_compatible_pbr_type is True

    _write_profiles(profiles_path, category_slug="patterned-glass")
    with pytest.raises(ValueError, match="top-level"):
        downloader._load_profiles(
            profiles_path,
            official_top_level_slugs={"glass", "metal"},
        )


def test_null_max_per_author_disables_author_limit(tmp_path: Path) -> None:
    profiles_path = tmp_path / "profiles.json"
    _write_profiles(profiles_path)
    payload = json.loads(profiles_path.read_text(encoding="utf-8"))
    payload["profiles"][0]["max_per_author"] = None
    profiles_path.write_text(json.dumps(payload), encoding="utf-8")

    _defaults, profiles = downloader._load_profiles(
        profiles_path,
        official_top_level_slugs={"glass"},
    )
    assert profiles[0].max_per_author is None

    candidates = []
    for index in range(3):
        asset = _asset(asset_base_id=f"same-author-{index}")
        candidates.append(
            {
                "asset": asset,
                "_tokens": {f"texture-{index}"},
                "_base_score": float(index),
            }
        )
    selected = downloader._select_diverse(
        candidates,
        quota=3,
        max_per_author=None,
        excluded_asset_base_ids=set(),
    )
    assert len(selected) == 3


def test_inventory_profile_can_disable_verification_and_pbr_type_filters(
    tmp_path: Path,
) -> None:
    profiles_path = tmp_path / "profiles.json"
    _write_profiles(profiles_path)
    payload = json.loads(profiles_path.read_text(encoding="utf-8"))
    payload["profiles"][0]["require_validated"] = False
    payload["profiles"][0]["require_compatible_pbr_type"] = False
    profiles_path.write_text(json.dumps(payload), encoding="utf-8")

    _defaults, profiles = downloader._load_profiles(
        profiles_path,
        official_top_level_slugs={"glass"},
    )
    profile = profiles[0]
    asset = _asset()
    asset["verificationStatus"] = "uploaded"
    asset["dictParameters"]["pbrType"] = "dielectric"

    reason, selected_file = downloader._candidate_rejection(
        asset,
        profile=profile,
        licenses={"royalty_free"},
        filter_licenses=False,
        file_types=profile.file_types,
        max_file_bytes=1024 * 1024,
    )

    assert reason is None
    assert selected_file is not None

    asset["license"] = "future_free_license"
    reason, selected_file = downloader._candidate_rejection(
        asset,
        profile=profile,
        licenses={"royalty_free"},
        filter_licenses=False,
        file_types=profile.file_types,
        max_file_bytes=1024 * 1024,
    )
    assert reason is None
    assert selected_file is not None


def test_fine_class_query_gate_rejects_provider_false_positive() -> None:
    cork = _asset()
    cork["name"] = "Natural Cork Board"
    cork["displayName"] = "Natural Cork Board"
    cork["tags"] = ["cork", "bottle stopper"]
    off_topic = _asset()
    off_topic["name"] = "Pigeon Poop"
    off_topic["displayName"] = "Pigeon Poop"
    off_topic["tags"] = ["natural", "dirt", "urban"]

    query = downloader._text_tokens("natural cork")
    assert downloader._candidate_matches_query(cork, query, category_slug="organic")
    assert not downloader._candidate_matches_query(
        off_topic,
        query,
        category_slug="organic",
    )


def test_build_plan_is_free_record_only_and_does_not_partition_license(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan_path = _build_plan(tmp_path, monkeypatch)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    assert plan["license_policy"] == "record_only"
    assert plan["accepted_licenses"] == ["cc_zero", "royalty_free"]
    material = plan["materials"][0]
    assert material["category_slug"] == "glass"
    assert material["source"]["license"] == "royalty_free"
    assert material["selected_file"]["file_type"] == "resolution_1K"
    assert "/royalty_free/" not in material["relative_path"]
    assert material["relative_path"].startswith("raw/blenderkit/material/glass/")
    assert plan["plan_sha256"] == downloader._plan_content_sha256(plan)


def test_record_only_plan_preserves_unfiltered_license(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan_path = _build_plan(tmp_path, monkeypatch)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    plan["filter_licenses"] = False
    plan["materials"][0]["source"]["license"] = "future_free_license"
    plan["plan_sha256"] = downloader._plan_content_sha256(plan)
    plan_path.write_text(json.dumps(plan), encoding="utf-8")

    loaded = downloader._load_locked_plan(plan_path)
    assert loaded["materials"][0]["source"]["license"] == "future_free_license"


def test_build_plan_excludes_raw_and_normalized_catalog_sources(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    taxonomy_path = tmp_path / "taxonomy.json"
    profiles_path = tmp_path / "profiles.json"
    plan_path = tmp_path / "plan.json"
    raw_catalog = tmp_path / "raw.jsonl"
    normalized_catalog = tmp_path / "normalized.jsonl"
    _write_taxonomy(taxonomy_path)
    _write_profiles(profiles_path)
    raw_catalog.write_text(
        json.dumps(
            {
                "material_id": "raw-existing",
                "source": {"asset_base_id": "base-1"},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    normalized_catalog.write_text(
        "".join(
            [
                json.dumps(
                    {
                        "material_id": "normalized-existing",
                        "source": {"assetBaseId": "base-2"},
                    }
                )
                + "\n",
                json.dumps(
                    {
                        "material_id": "normalized-duplicate",
                        "asset_base_id": "base-1",
                    }
                )
                + "\n",
            ]
        ),
        encoding="utf-8",
    )

    monkeypatch.setattr(downloader, "_new_session", lambda **_kwargs: _Session())
    monkeypatch.setattr(
        downloader,
        "_search_assets",
        lambda *_args, **_kwargs: [
            _asset(asset_base_id="base-1"),
            _asset(asset_base_id="base-2"),
            _asset(asset_base_id="base-3"),
        ],
    )
    args = argparse.Namespace(
        taxonomy=taxonomy_path,
        profiles=profiles_path,
        accepted_licenses=None,
        file_types=None,
        max_file_bytes=None,
        pool_multiplier=2,
        exclude_catalog=[raw_catalog, normalized_catalog],
        page_size=80,
        retries=1,
        budget_gib=0.01,
        strict=True,
        execute=True,
        output=plan_path,
    )

    assert downloader.build_plan(args) == 0
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    assert plan["excluded_asset_base_id_count"] == 2
    assert [row["sha256"] for row in plan["exclude_catalogs"]] == [
        downloader._sha256(raw_catalog),
        downloader._sha256(normalized_catalog),
    ]
    assert [row["asset_base_id_count"] for row in plan["exclude_catalogs"]] == [1, 2]
    assert plan["profiles"][0]["rejected"]["existing_catalog_asset"] == 2
    assert [row["source"]["asset_base_id"] for row in plan["materials"]] == ["base-3"]
    assert plan["materials"][0]["source"]["is_free"] is True
    assert plan["materials"][0]["category_slug"] == "glass"
    assert plan["materials"][0]["selected_file"]["file_type"] == "resolution_1K"
    assert plan["plan_sha256"] == downloader._plan_content_sha256(plan)


def test_page_size_above_api_limit_is_rejected() -> None:
    with pytest.raises(ValueError, match="between 1 and 80"):
        downloader._validated_page_size(81)


def test_sync_plan_is_non_mutating_without_execute(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan_path = _build_plan(tmp_path, monkeypatch)
    library = tmp_path / "library"
    monkeypatch.setattr(
        downloader,
        "_download_file",
        lambda **_kwargs: pytest.fail("dry-run attempted a download"),
    )
    args = argparse.Namespace(
        plan=plan_path,
        pbr_root=library,
        catalog=None,
        provenance=None,
        budget_gib=None,
        max_file_bytes=None,
        include_previews=False,
        preview_budget_mib=1.0,
        max_preview_bytes=1024,
        retries=1,
        overwrite=False,
        execute=False,
    )
    assert downloader.sync_plan(args) == 0
    assert not library.exists()


def test_sync_plan_writes_verified_catalog_and_provenance_atomically(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan_path = _build_plan(tmp_path, monkeypatch)
    library = tmp_path / "library"
    provenance_directory = tmp_path / "explicit-provenance"
    provenance_directory.mkdir()

    def fake_download_file(**kwargs: object) -> tuple[int, str]:
        destination = kwargs["destination"]
        assert isinstance(destination, Path)
        expected_size = int(kwargs["expected_size_bytes"])
        content = b"x" * expected_size
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
        return len(content), hashlib.sha256(content).hexdigest()

    monkeypatch.setattr(downloader, "_download_file", fake_download_file)
    monkeypatch.setattr(
        downloader,
        "_download_preview",
        lambda **_kwargs: pytest.fail("previews are opt-in"),
    )
    args = argparse.Namespace(
        plan=plan_path,
        pbr_root=library,
        catalog=None,
        provenance=provenance_directory,
        budget_gib=0.01,
        max_file_bytes=8192,
        include_previews=False,
        preview_budget_mib=1.0,
        max_preview_bytes=1024,
        retries=1,
        overwrite=False,
        execute=True,
    )
    assert downloader.sync_plan(args) == 0
    catalog_path = library / "catalog" / "materials.jsonl"
    rows = [json.loads(line) for line in catalog_path.read_text().splitlines()]
    assert len(rows) == 1
    assert rows[0]["license"] == "royalty_free"
    assert rows[0]["validation"]["status"] == "downloaded_verified"
    assert len(rows[0]["selected_file"]["sha256"]) == 64
    provenance_files = list(provenance_directory.glob("sync_*.json"))
    assert len(provenance_files) == 1
    provenance = json.loads(provenance_files[0].read_text())
    assert provenance["downloaded_count"] == 1
    assert provenance["preview_count"] == 0


def test_sync_plan_downloads_concurrently_but_writes_deterministic_order(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan_path = _build_plan(tmp_path, monkeypatch)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    source_row = plan["materials"][0]
    materials = []
    for material_id in ("material-z", "material-a", "material-m"):
        row = json.loads(json.dumps(source_row))
        row["material_id"] = material_id
        row["relative_path"] = f"raw/{material_id}.blend"
        row["preview_relative_path"] = f"previews/{material_id}.webp"
        materials.append(row)
    plan["materials"] = materials
    plan["selected_count"] = len(materials)
    plan["expected_download_bytes"] = sum(
        int(row["selected_file"]["expected_size_bytes"]) for row in materials
    )
    plan["plan_sha256"] = downloader._plan_content_sha256(plan)
    plan_path.write_text(json.dumps(plan), encoding="utf-8")

    barrier = threading.Barrier(3)

    def fake_download_file(**kwargs: object) -> tuple[int, str]:
        barrier.wait(timeout=2)
        destination = kwargs["destination"]
        assert isinstance(destination, Path)
        expected_size = int(kwargs["expected_size_bytes"])
        content = destination.name.encode()[:1] * expected_size
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
        return len(content), hashlib.sha256(content).hexdigest()

    monkeypatch.setattr(downloader, "_download_file", fake_download_file)
    provenance_path = tmp_path / "provenance.json"
    args = argparse.Namespace(
        plan=plan_path,
        pbr_root=tmp_path / "library",
        catalog=None,
        provenance=provenance_path,
        budget_gib=0.01,
        max_file_bytes=8192,
        include_previews=False,
        preview_budget_mib=1.0,
        max_preview_bytes=1024,
        retries=1,
        workers=3,
        overwrite=False,
        resume_existing=False,
        execute=True,
    )
    assert downloader.sync_plan(args) == 0
    catalog = downloader._read_catalog_rows(
        tmp_path / "library" / "catalog" / "materials.jsonl"
    )
    assert [row["material_id"] for row in catalog] == [
        "material-a",
        "material-m",
        "material-z",
    ]
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    assert [row["material_id"] for row in provenance["materials"]] == [
        "material-a",
        "material-m",
        "material-z",
    ]
    assert provenance["downloaded_count"] == 3
    assert provenance["reused_existing_count"] == 0


def test_sync_plan_resume_existing_hashes_and_reuses_exact_size_file(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan_path = _build_plan(tmp_path, monkeypatch)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    row = plan["materials"][0]
    library = tmp_path / "library"
    destination = library / row["relative_path"]
    destination.parent.mkdir(parents=True)
    content = b"r" * int(row["selected_file"]["expected_size_bytes"])
    destination.write_bytes(content)
    monkeypatch.setattr(
        downloader,
        "_download_file",
        lambda **_kwargs: pytest.fail("resume attempted a network download"),
    )
    provenance_path = tmp_path / "resume-provenance.json"
    args = argparse.Namespace(
        plan=plan_path,
        pbr_root=library,
        catalog=None,
        provenance=provenance_path,
        budget_gib=0.01,
        max_file_bytes=8192,
        include_previews=False,
        preview_budget_mib=1.0,
        max_preview_bytes=1024,
        retries=1,
        workers=8,
        overwrite=False,
        resume_existing=True,
        execute=True,
    )
    assert downloader.sync_plan(args) == 0
    catalog = downloader._read_catalog_rows(library / "catalog" / "materials.jsonl")
    assert catalog[0]["selected_file"]["sha256"] == hashlib.sha256(content).hexdigest()
    assert catalog[0]["provenance"]["acquisition"] == "resumed_existing"
    provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
    assert provenance["downloaded_count"] == 0
    assert provenance["reused_existing_count"] == 1


def test_sync_plan_resume_existing_rejects_wrong_size_before_download(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan_path = _build_plan(tmp_path, monkeypatch)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    row = plan["materials"][0]
    library = tmp_path / "library"
    destination = library / row["relative_path"]
    destination.parent.mkdir(parents=True)
    destination.write_bytes(b"wrong-size")
    monkeypatch.setattr(
        downloader,
        "_download_file",
        lambda **_kwargs: pytest.fail("invalid resume attempted a network download"),
    )
    args = argparse.Namespace(
        plan=plan_path,
        pbr_root=library,
        catalog=None,
        provenance=None,
        budget_gib=0.01,
        max_file_bytes=8192,
        include_previews=False,
        preview_budget_mib=1.0,
        max_preview_bytes=1024,
        retries=1,
        workers=8,
        overwrite=False,
        resume_existing=True,
        execute=True,
    )
    with pytest.raises(FileExistsError, match="cannot be resumed"):
        downloader.sync_plan(args)
    assert not (library / "catalog" / "materials.jsonl").exists()


def test_sync_provenance_path_does_not_guess_missing_suffixless_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(downloader, "_utc_stamp", lambda: "20260810T120000Z")
    plan_sha256 = "a" * 64
    expected_name = "sync_aaaaaaaaaaaaaaaa_20260810T120000Z.json"
    existing_directory = tmp_path / "existing"
    existing_directory.mkdir()

    assert (
        downloader._resolve_sync_provenance_path(
            existing_directory,
            pbr_root=tmp_path / "library",
            plan_sha256=plan_sha256,
        )
        == existing_directory / expected_name
    )
    missing_suffixless_path = tmp_path / "future-provenance"
    assert (
        downloader._resolve_sync_provenance_path(
            missing_suffixless_path,
            pbr_root=tmp_path / "library",
            plan_sha256=plan_sha256,
        )
        == missing_suffixless_path.resolve()
    )
    assert (
        downloader._resolve_sync_provenance_path(
            None,
            pbr_root=tmp_path / "library",
            plan_sha256=plan_sha256,
        )
        == tmp_path / "library" / "provenance" / expected_name
    )
