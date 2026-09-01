#!/usr/bin/env python3
"""Package renderable source assets for the comparison selector.

The standalone HTML keeps the rendered PNGs embedded. This bundle contains
the source files needed for a Blender rerender, while raster textures remain
outside the archive and are listed in the metadata index.
"""

from __future__ import annotations

import json
import shutil
import zipfile
from pathlib import Path
from tempfile import TemporaryDirectory


ROOT = Path(__file__).resolve().parents[2]
RENDER_ROOT = ROOT / "failure_case_renders"
HTML_PATH = RENDER_ROOT / "articraft_lam_category_comparison_standalone.html"
OUTPUT_ZIP = RENDER_ROOT / "articraft_lam_category_comparison_assets.zip"

TEXTURE_SUFFIXES = {
    ".avif", ".bmp", ".exr", ".gif", ".hdr", ".jpeg", ".jpg", ".png",
    ".svg", ".tga", ".tif", ".tiff", ".webp",
}
RENDERABLE_SUFFIXES = {
    ".blend", ".dae", ".fbx", ".glb", ".gltf", ".json", ".mtl", ".obj",
    ".off", ".ply", ".stl", ".toml", ".urdf", ".xml", ".yaml", ".yml",
}
DERIVED_RENDER_DIRS = {"pipeline_logs", "render", "renders"}


def read_viewer_data() -> dict:
    html = HTML_PATH.read_text(encoding="utf-8")
    marker = '<script id="viewer-data"'
    start = html.index(marker)
    payload_start = html.index(">", start) + 1
    payload_end = html.index("</script>", payload_start)
    return json.loads(html[payload_start:payload_end])


def copy_source_files(
    source: Path, destination: Path, *, include_textures: bool
) -> tuple[list[str], int, int, int, int]:
    """Copy renderable files and optionally retain source material textures."""
    if not source.is_dir():
        raise FileNotFoundError(f"Missing source asset directory: {source}")
    packaged: list[str] = []
    packaged_textures = 0
    skipped_textures = 0
    skipped_render_artifacts = 0
    packaged_bytes = 0
    for path in sorted(source.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(source)
        if (
            any(part.lower() in DERIVED_RENDER_DIRS for part in relative.parts[:-1])
            or path.name in {"rest.png", "midstate.png"}
        ):
            skipped_render_artifacts += 1
            continue
        suffix = path.suffix.lower()
        if suffix in TEXTURE_SUFFIXES:
            if not include_textures or path.name in {"rest.png", "midstate.png"}:
                skipped_textures += 1
                continue
            packaged_textures += 1
        elif suffix not in RENDERABLE_SUFFIXES:
            continue
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        packaged.append(relative.as_posix())
        packaged_bytes += path.stat().st_size
    return packaged, packaged_textures, skipped_textures, skipped_render_artifacts, packaged_bytes


def source_record(
    source_name: str,
    asset: dict,
    bundle: Path,
    copied: dict[tuple[str, str], dict],
) -> dict:
    """Copy one source package once and return its archive/index metadata."""
    asset_dir = str(asset.get("asset_dir") or "")
    if not asset_dir:
        raise FileNotFoundError(f"{source_name} asset has no asset_dir: {asset}")
    key = (source_name, asset_dir)
    if key in copied:
        return copied[key]

    package_path = str(asset.get("package_path") or "")
    if not package_path:
        raise ValueError(f"{source_name} asset has no package_path: {asset}")
    source = Path(asset_dir)
    destination = bundle / package_path
    include_textures = source_name == "LAM"
    packaged_files, packaged_textures, skipped_textures, skipped_render_artifacts, packaged_bytes = copy_source_files(
        source, destination, include_textures=include_textures
    )
    urdf_path = str(asset.get("urdf_path") or "")
    try:
        urdf_relative = Path(urdf_path).resolve().relative_to(source.resolve()).as_posix()
    except ValueError:
        urdf_relative = Path(urdf_path).name if urdf_path else ""
    record = {
        "source": source_name,
        "asset_dir": asset_dir,
        "urdf_path": urdf_path,
        "package_path": package_path,
        "urdf_package_path": f"{package_path}/{urdf_relative}" if urdf_relative else None,
        "packaged_files": packaged_files,
        "indexed_textures": asset.get("texture_index", []),
        "packaged_texture_files": packaged_textures,
        "omitted_texture_files": skipped_textures,
        "skipped_render_artifacts": skipped_render_artifacts,
        "packaged_bytes": packaged_bytes,
        "texture_policy": "packaged" if include_textures else ("none" if source_name == "Articraft" else "indexed_only"),
    }
    copied[key] = record
    return record


def main() -> int:
    data = read_viewer_data()
    with TemporaryDirectory(prefix="comparison-assets-") as temporary:
        bundle = Path(temporary) / "articraft_lam_renderable_assets"
        (bundle / "metadata").mkdir(parents=True)
        copied: dict[tuple[str, str], dict] = {}
        selected_groups = []
        pva_keys: set[tuple[str, str]] = set()
        art_keys: set[tuple[str, str]] = set()
        lam_keys: set[tuple[str, str]] = set()

        for group in data["groups"]:
            pva_records = []
            for pva in group["pva_assets"]:
                base = source_record("PV-A", pva, bundle, copied)
                pva_records.append(
                    {
                        **base,
                        "generator_index": pva["generator_index"],
                        "generator_name": pva["generator_name"],
                        "seed": pva["seed"],
                        "relation": pva["relation"],
                    }
                )
                pva_keys.add((pva["generator_name"], pva["seed"]))

            art_records = []
            for asset in group["art_assets"]:
                base = source_record("Articraft", asset, bundle, copied)
                art_records.append(
                    {**base, "asset_id": asset["asset_id"], "rating": asset["rating"]}
                )
                art_keys.add((str(asset["asset_id"]), str(asset["asset_dir"])))

            lam_records = []
            for asset in group["lam_assets"]:
                if not asset.get("asset_id"):
                    raise FileNotFoundError(
                        f"LAM asset is missing for {group['category']}: {asset['category']}"
                    )
                base = source_record("LAM", asset, bundle, copied)
                lam_records.append(
                    {
                        **base,
                        "category": asset["category"],
                        "asset_id": asset["asset_id"],
                        "tier": asset["tier"],
                        "render_mode": asset["render_mode"],
                    }
                )
                lam_keys.add((str(asset["asset_id"]), str(asset["asset_dir"])))

            selected_groups.append(
                {
                    "category": group["category"],
                    "level": group["level"],
                    "pva": pva_records,
                    "articraft": art_records,
                    "lam": lam_records,
                }
            )

        packaged_files = sum(len(record["packaged_files"]) for record in copied.values())
        packaged_bytes = sum(record["packaged_bytes"] for record in copied.values())
        omitted_texture_files = sum(
            record["omitted_texture_files"] for record in copied.values()
        )
        packaged_texture_files = sum(
            record["packaged_texture_files"] for record in copied.values()
        )
        skipped_render_artifacts = sum(
            record["skipped_render_artifacts"] for record in copied.values()
        )
        manifest = {
            "schema_version": 2,
            "package_kind": "renderable_source_assets",
            "source_html": HTML_PATH.name,
            "comparison_groups": len(selected_groups),
            "pva_assets": len(pva_keys),
            "pva_unique_categories": len({name for name, _ in pva_keys}),
            "articraft_assets": len(art_keys),
            "lam_assets": len(lam_keys),
            "total_assets": len(pva_keys) + len(art_keys) + len(lam_keys),
            "rendered_images": 0,
            "packaged_source_directories": len(copied),
            "packaged_files": packaged_files,
            "packaged_bytes": packaged_bytes,
            "packaged_texture_files": packaged_texture_files,
            "omitted_texture_files": omitted_texture_files,
            "skipped_render_artifacts": skipped_render_artifacts,
            "raw_asset_policy": "PV-A textures are indexed only; Articraft has no detected textures; LAM source material textures are packaged. Source pipeline/render directories are excluded.",
            "groups": selected_groups,
        }
        manifest_text = json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"
        (bundle / "metadata" / "selected_assets.json").write_text(
            manifest_text, encoding="utf-8"
        )
        (bundle / "metadata" / "asset_index.json").write_text(
            manifest_text, encoding="utf-8"
        )
        readme = (
            "PV-A / Articraft / LAM renderable source asset bundle\n"
            "=====================================================\n\n"
            f"Comparison groups: {len(selected_groups)}\n"
            f"PV-A candidate assets: {len(pva_keys)} ({len({name for name, _ in pva_keys})} categories, 3 per category)\n"
            f"Articraft assets: {len(art_keys)}\n"
            f"LAM assets: {len(lam_keys)}\n"
            f"Packaged source directories: {len(copied)}\n"
            f"Packaged files: {packaged_files}\n"
            f"Packaged LAM texture files: {packaged_texture_files}\n"
            f"Omitted texture files: {omitted_texture_files}\n\n"
            f"Skipped source render artifacts: {skipped_render_artifacts}\n\n"
            "The standalone HTML is delivered separately and embeds all rest/midstate PNGs for offline viewing.\n"
            "This ZIP intentionally contains no rendered rest.png or midstate.png files.\n"
            "PV-A texture bytes are omitted and indexed only; Articraft has no detected textures; LAM source material textures are retained.\n"
            "Use metadata/asset_index.json (or selected_assets.json) to map each category/asset to package_path\n"
            "and urdf_package_path. indexed_textures records texture references without copying the material library.\n"
        )
        (bundle / "README.txt").write_text(readme, encoding="utf-8")
        OUTPUT_ZIP.unlink(missing_ok=True)
        with zipfile.ZipFile(
            OUTPUT_ZIP, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6
        ) as archive:
            for path in sorted(bundle.rglob("*")):
                if path.is_file():
                    archive.write(path, path.relative_to(Path(temporary)).as_posix())
    print(
        json.dumps(
            {
                "output": str(OUTPUT_ZIP),
                **{
                    key: manifest[key]
                    for key in (
                        "comparison_groups",
                        "pva_assets",
                        "pva_unique_categories",
                        "articraft_assets",
                        "lam_assets",
                        "total_assets",
                        "rendered_images",
                        "packaged_source_directories",
                        "packaged_files",
                        "packaged_texture_files",
                        "omitted_texture_files",
                        "skipped_render_artifacts",
                    )
                },
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
