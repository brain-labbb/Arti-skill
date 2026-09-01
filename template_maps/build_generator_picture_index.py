#!/usr/bin/env python3
"""Build the current generator-to-picture index and its picture-pool audit."""

from __future__ import annotations

import csv
import re
import unicodedata
from collections import defaultdict
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
TEMPLATE_DIR = REPO_ROOT / "arti-template" / "agent" / "templates"
PICTURE_ROOT = REPO_ROOT / "articraft_data" / "picture"
LOCAL_PICTURE_ROOT = REPO_ROOT / "arti-template" / "picture"
MAP_ROOT = REPO_ROOT / "template_maps"

BUILTIN_MAP = MAP_ROOT / "articraft_builtin_100_map.csv"
KNOWN_PICTURE_MAPS = (
    MAP_ROOT / "non_articraft_190_map.csv",
    MAP_ROOT / "extra_requested_gallery_map.csv",
)
OUTPUT_INDEX = MAP_ROOT / "generator_picture_index.csv"
OUTPUT_UNMATCHED_PICTURES = MAP_ROOT / "picture_without_generator.csv"


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def normalized(value: str) -> str:
    ascii_value = (
        unicodedata.normalize("NFKD", value)
        .encode("ascii", "ignore")
        .decode("ascii")
        .lower()
    )
    return re.sub(r"[^a-z0-9]+", "", ascii_value)


def relative_picture_dirs(root: Path) -> set[str]:
    return {
        path.relative_to(root).as_posix()
        for path in root.glob("*/*")
        if path.is_dir()
    }


def png_count(path: Path) -> int:
    return sum(1 for item in path.iterdir() if item.is_file() and item.suffix.lower() == ".png")


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    generators = {
        path.stem: path
        for path in TEMPLATE_DIR.glob("*.py")
        if not path.name.startswith("_")
    }
    builtin_rows = read_rows(BUILTIN_MAP)
    builtin_by_name = {row["template_name"]: row for row in builtin_rows}
    live_builtin_names = set(generators) & set(builtin_by_name)

    picture_dirs = relative_picture_dirs(PICTURE_ROOT)
    local_picture_dirs = relative_picture_dirs(LOCAL_PICTURE_ROOT)
    if not local_picture_dirs <= picture_dirs:
        raise RuntimeError(
            "arti-template/picture contains paths absent from articraft_data/picture: "
            f"{sorted(local_picture_dirs - picture_dirs)}"
        )

    picture_by_full_key: dict[str, list[str]] = defaultdict(list)
    picture_0611_by_label_key: dict[str, list[str]] = defaultdict(list)
    for picture_dir in sorted(picture_dirs):
        picture_by_full_key[normalized(picture_dir)].append(picture_dir)
        category, label = picture_dir.split("/", 1)
        if category == "0611":
            picture_0611_by_label_key[normalized(label)].append(picture_dir)

    picture_mapping: dict[str, tuple[str, str]] = {}
    for map_path in KNOWN_PICTURE_MAPS:
        for row in read_rows(map_path):
            name = row["template_name"]
            if name not in generators or name in live_builtin_names or not row.get("template_path"):
                continue
            picture_dir = row["picture_path"].removeprefix("articraft_data/picture/")
            if picture_dir not in picture_dirs:
                raise RuntimeError(f"Missing mapped picture directory for {name}: {picture_dir}")
            previous = picture_mapping.get(name)
            if previous is not None and previous[0] != picture_dir:
                raise RuntimeError(
                    f"Conflicting picture mappings for {name}: {previous[0]} vs {picture_dir}"
                )
            picture_mapping[name] = (picture_dir, f"existing_map:{map_path.name}")

    picture_backed_names = set(generators) - live_builtin_names
    for name in sorted(picture_backed_names - set(picture_mapping)):
        candidates = picture_by_full_key[normalized(name)]
        if len(candidates) == 1:
            picture_mapping[name] = (candidates[0], "normalized_full_picture_path")

    for name in sorted(picture_backed_names - set(picture_mapping)):
        label = name.removeprefix("pictureX_0611_")
        candidates = picture_0611_by_label_key[normalized(label)]
        if len(candidates) == 1:
            picture_mapping[name] = (candidates[0], "normalized_0611_picture_label")

    unresolved = sorted(picture_backed_names - set(picture_mapping))
    if unresolved:
        raise RuntimeError(f"Unresolved picture-backed generators: {unresolved}")

    picture_index = {
        picture_dir: f"P{index:04d}"
        for index, picture_dir in enumerate(sorted(picture_dirs), start=1)
    }
    rows: list[dict[str, object]] = []
    for generator_index, name in enumerate(sorted(generators, key=str.casefold), start=1):
        generator_path = generators[name].relative_to(REPO_ROOT).as_posix()
        if name in live_builtin_names:
            builtin = builtin_by_name[name]
            rows.append(
                {
                    "generator_index": f"G{generator_index:04d}",
                    "generator_name": name,
                    "generator_path": generator_path,
                    "source_type": "articraft_builtin_dataset_no_picture",
                    "picture_index": "",
                    "picture_category": "",
                    "picture_label": "",
                    "picture_source_path": "",
                    "arti_template_picture_path": "",
                    "picture_storage": "none",
                    "picture_png_count": 0,
                    "mapping_source": BUILTIN_MAP.name,
                    "catalog_image_path": builtin.get("catalog_image_path", ""),
                }
            )
            continue

        picture_dir, mapping_source = picture_mapping[name]
        category, label = picture_dir.split("/", 1)
        local_copy = picture_dir in local_picture_dirs
        rows.append(
            {
                "generator_index": f"G{generator_index:04d}",
                "generator_name": name,
                "generator_path": generator_path,
                "source_type": "picture_backed",
                "picture_index": picture_index[picture_dir],
                "picture_category": category,
                "picture_label": label,
                "picture_source_path": f"articraft_data/picture/{picture_dir}",
                "arti_template_picture_path": (
                    f"arti-template/picture/{picture_dir}" if local_copy else ""
                ),
                "picture_storage": (
                    "articraft_data_and_arti_template" if local_copy else "articraft_data_only"
                ),
                "picture_png_count": png_count(PICTURE_ROOT / picture_dir),
                "mapping_source": mapping_source,
                "catalog_image_path": "",
            }
        )

    used_picture_dirs = {picture_mapping[name][0] for name in picture_backed_names}
    unmatched_picture_rows = []
    for picture_dir in sorted(picture_dirs - used_picture_dirs):
        category, label = picture_dir.split("/", 1)
        local_copy = picture_dir in local_picture_dirs
        unmatched_picture_rows.append(
            {
                "picture_index": picture_index[picture_dir],
                "picture_category": category,
                "picture_label": label,
                "picture_source_path": f"articraft_data/picture/{picture_dir}",
                "arti_template_picture_path": (
                    f"arti-template/picture/{picture_dir}" if local_copy else ""
                ),
                "picture_png_count": png_count(PICTURE_ROOT / picture_dir),
                "status": "no_generator",
            }
        )

    write_csv(
        OUTPUT_INDEX,
        [
            "generator_index",
            "generator_name",
            "generator_path",
            "source_type",
            "picture_index",
            "picture_category",
            "picture_label",
            "picture_source_path",
            "arti_template_picture_path",
            "picture_storage",
            "picture_png_count",
            "mapping_source",
            "catalog_image_path",
        ],
        rows,
    )
    write_csv(
        OUTPUT_UNMATCHED_PICTURES,
        [
            "picture_index",
            "picture_category",
            "picture_label",
            "picture_source_path",
            "arti_template_picture_path",
            "picture_png_count",
            "status",
        ],
        unmatched_picture_rows,
    )

    duplicate_picture_dirs = {
        picture_dir: sorted(
            name for name, (mapped_dir, _) in picture_mapping.items() if mapped_dir == picture_dir
        )
        for picture_dir in used_picture_dirs
        if sum(mapped_dir == picture_dir for mapped_dir, _ in picture_mapping.values()) > 1
    }
    missing_builtin_generators = sorted(set(builtin_by_name) - set(generators))
    local_picture_dirs_without_generator = sorted(local_picture_dirs - used_picture_dirs)
    print(f"generators={len(generators)}")
    print(f"builtin_dataset_no_picture={len(live_builtin_names)}")
    print(f"picture_backed_generators={len(picture_backed_names)}")
    print(f"unique_mapped_picture_dirs={len(used_picture_dirs)}")
    print(f"all_picture_dirs={len(picture_dirs)}")
    print(f"arti_template_picture_dirs={len(local_picture_dirs)}")
    print(f"picture_dirs_without_generator={len(unmatched_picture_rows)}")
    print(f"arti_template_picture_dirs_without_generator={local_picture_dirs_without_generator}")
    print(f"duplicate_picture_mappings={duplicate_picture_dirs}")
    print(f"builtin_map_rows_without_generator={missing_builtin_generators}")


if __name__ == "__main__":
    main()
