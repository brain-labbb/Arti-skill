#!/usr/bin/env python3
"""Build a self-contained Articraft/LAM category comparison viewer."""

from __future__ import annotations

import argparse
import base64
import csv
import json
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = (
    ROOT
    / "failure_case_renders"
    / "articraft_lam_category_comparison_standalone.html"
)
PVA_RENDER_ROOT = ROOT / "failure_case_renders" / "pva"
PVA_ASSET_ROOT = Path("/mnt/zsn/data/particulate/datasets/PV-A/extracted")
PVA_ROSTER = ROOT / "exp/runtime/pva531_uniform_tsne/generator_roster_resolved.csv"
TEXTURE_SUFFIXES = {
    ".bmp",
    ".gif",
    ".jpeg",
    ".jpg",
    ".png",
    ".tif",
    ".tiff",
    ".webp",
}
DERIVED_RENDER_DIRS = {"pipeline_logs", "render", "renders"}

# Curated semantic alignment from the 75 Articraft failure categories to the
# closest PV-A generator class. Exact names are preferred; the remaining
# entries are object-, mechanism-, or use-family matches.
PVA_MATCHES = {
    "all_in_one_printer_with_scanner_lid_and_paper_tray": ("Technology_Printer", "对象近似"),
    "air_pump_with_t_handle": ("Container_Pump", "同类泵"),
    "bench_vise_with_prismatic_jaw": ("Industrial_Industrial_vice", "同类夹持工具"),
    "bicycle_crankset_and_pedal_assembly": ("bicycle_crankset_and_pedal_assembly", "完全同名"),
    "binoculars": ("Others_Binocular", "同类光学器件"),
    "blender": ("blender_countertop", "同类厨房电器"),
    "cash_register": ("cash_register", "完全同名"),
    "ceiling_fan": ("ceiling_fan", "完全同名"),
    "desktop_hole_punch_with_articulated_components": ("hole_punch", "同类办公工具"),
    "desktop_pencil_sharpener_with_hand_crank_and_drawer": ("Stationary_Pencil_sharpener", "同类办公工具"),
    "desktop_stapler_with_hinged_top_arm": ("Handtools_Stapler", "同类办公工具"),
    "dishwasher_with_dropdown_door_and_sliding_racks": ("dishwasher_with_dropdown_door_and_sliding_racks", "完全同名"),
    "espresso_machine_with_articulated_components": ("Kitchen_Coffee_machine", "同类厨房电器"),
    "faucet_with_side_handle": ("Other_single_hole_basin_faucet", "同类水龙头"),
    "folding_chair": ("Chair_Folding_chair", "同类座椅"),
    "frontload_washing_machine": ("Bathroom_washmachine", "同类洗衣机"),
    "top_load_washing_machine_with_hinged_lid": ("Bathroom_washmachine", "同类洗衣机"),
    "globe": ("globe", "完全同名"),
    "hand_truck_dolly": ("platform_cart", "同类搬运车"),
    "laptop_clamshell": ("Technology_Laptop", "同类笔记本电脑"),
    "microwave_oven": ("Kitchen_Microwave", "同类厨房电器"),
    "monitor_mount": ("monitor_mount", "完全同名"),
    "observation_wheel": ("ferris_wheel", "同类观景轮"),
    "remote": ("Technology_Remote_Control", "同类遥控器"),
    "skateboard": ("Sports_Skateboard", "同类运动器材"),
    "simple_ironing_board": ("ironing_board", "同类家居用品"),
    "stand_mixer": ("stand_mixer", "完全同名"),
    "toaster_oven": ("Kitchen_Toaster", "同类厨房电器"),
    "turnstile_gates": ("turnstile_gates", "完全同名"),
    "tv_wall_mount": ("tv_wall_mount", "完全同名"),
    "waffle_maker": ("waffle_maker", "完全同名"),
    "zippo_lighter": ("zippo_lighter", "完全同名"),
    "padlock_with_shackle": ("Equipment_Lock", "同类锁具"),
    "revolving_door": ("revolving_door", "完全同名"),
    "articulated_task_lamp": ("articulated_task_lamp", "完全同名"),
    "floor_lamp": ("studio_lamp", "同类灯具"),
    "ring_light_on_stand": ("studio_lamp", "同类灯具"),
    "studio_spotlight_on_yoke": ("studio_spotlight_on_yoke", "完全同名"),
    "camera_flash": ("camera_flash", "完全同名"),
    "camcorder_with_flipout_screen": ("camcorder_with_flipout_screen", "完全同名"),
    "camera_lens": ("camera_lens", "完全同名"),
    "bicycle_fork_and_handlebar_assembly": ("Sports_Bike", "同类自行车部件"),
    "box_fan_with_control_knob": ("box_fan_with_control_knob", "完全同名"),
    "tilting_fan": ("ceiling_fan", "同类风扇"),
    "drillpress_tilt_table": ("Industrial_Drill_press_table", "同类钻床机构"),
    "clamp_meter_with_hinged_jaw_and_rotary_selector": ("Handtools_Clamp", "同类夹持工具"),
    "digital_multimeter_with_tilt_stand_and_rotary_selector": ("Equipment_Control_panel", "同类控制面板"),
    "screwcap_bottle": ("screwcap_bottle", "完全同名"),
    "pump_bottle": ("Container_Pump", "同类泵瓶"),
    "trigger_spray_bottle": ("Container_Paint_spray", "同类喷雾容器"),
    "rice_cooker": ("Kitchen_Air_fryer", "同类厨房电器"),
    "turntable": ("turntable", "完全同名"),
    "stove_top": ("Other_stove", "同类炉具"),
    "toy_car": ("Sports_Toy_car", "完全同名"),
    "car_axles": ("car_axles", "完全同名"),
    "desktop_pc_tower": ("desktop_pc_tower", "完全同名"),
    "glove_compartment_door": ("Door_Door", "同类门体"),
    "hinged_window_or_hatch": ("Window_Window", "同类窗体"),
    "sliding_window": ("Window_Sliding_window", "同类窗体"),
    "hingeddoor_cabinet": ("Other_Cabinet", "同类柜体"),
    "desk_with_drawer": ("desk_with_drawer", "完全同名"),
    "overbed_table": ("drafting_table", "同类桌面机构"),
    "adjustable_weight_bench_with_hinged_backrest": ("piano_bench", "同类长凳"),
    "lounge_chair_with_independent_backrest_and_footrest": ("Other_armchair", "同类座椅"),
    "swivel_bar_stool": ("Chair_Chair", "同类座椅"),
    "rolling_toolbox_with_telescoping_handle": ("rolling_toolbox_with_telescoping_handle", "完全同名"),
    "miter_saw_arm_assembly": ("miter_saw_arm_assembly", "完全同名"),
    "telescoping_boom": ("telescoping_boom", "完全同名"),
    "serial_elbow_arm": ("serial_elbow_arm", "完全同名"),
    "shoulderelbowwrist_arm": ("shoulderelbowwrist_arm", "完全同名"),
    "cantilever_articulated_arm": ("cantilever_articulated_arm", "完全同名"),
    "multisegment_foldout_arm": ("multisegment_foldout_arm", "完全同名"),
    "opposed_twinslide_gripper": ("soft_pneumatic_gripper", "同类夹爪"),
    "robotic_arms": ("robotic_arms", "完全同名"),
    "robotic_leg": ("robotic_leg", "完全同名"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def read_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def image_data_url(path: Path) -> str:
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:image/png;base64,{encoded}"


def package_component(value: object) -> str:
    """Keep source-package paths portable and free of directory traversal."""
    return str(value).replace("/", "_").replace("\\", "_").strip() or "unnamed"


def source_package_path(source: str, *parts: object) -> str:
    return "/".join(
        ["source_assets", package_component(source)]
        + [package_component(part) for part in parts]
    )


def texture_index(asset_dir: str, metadata_paths: list[str]) -> list[dict]:
    """Index texture references while leaving the texture bytes outside the bundle."""
    root = Path(asset_dir)
    references: dict[str, dict] = {}

    def add(reference: str, origin: str) -> None:
        value = str(reference).strip().strip('"').strip("'")
        if not value:
            return
        suffix = Path(value.split("?")[0]).suffix.lower()
        if suffix not in TEXTURE_SUFFIXES and "texture" not in value.lower():
            return
        key = value.replace("\\", "/")
        candidate = Path(value)
        if not candidate.is_absolute():
            candidate = root / value
        references.setdefault(
            key,
            {
                "path": key,
                "origin": origin,
                "exists_in_source": candidate.is_file(),
            },
        )

    def is_derived_render(path: Path) -> bool:
        try:
            relative_parts = path.relative_to(root).parts
        except ValueError:
            relative_parts = path.parts
        return any(part.lower() in DERIVED_RENDER_DIRS for part in relative_parts[:-1])

    def walk(value: object, origin: str) -> None:
        if isinstance(value, dict):
            for key, child in value.items():
                if isinstance(child, str) and (
                    key.lower() in {"path", "texture", "filename", "file"}
                    or "texture" in key.lower()
                ):
                    add(child, origin)
                walk(child, origin)
        elif isinstance(value, list):
            for child in value:
                walk(child, origin)

    for metadata_path in metadata_paths:
        path = Path(metadata_path)
        if not path.is_file() or path.suffix.lower() != ".json":
            continue
        try:
            walk(read_json(path), path.name)
        except (OSError, json.JSONDecodeError):
            continue

    if root.is_dir():
        for path in root.rglob("*"):
            if (
                path.is_file()
                and path.suffix.lower() in TEXTURE_SUFFIXES
                and path.name not in {"rest.png", "midstate.png"}
                and not is_derived_render(path)
            ):
                add(path.relative_to(root).as_posix(), "asset_dir_scan")
        for path in root.rglob("*.mtl"):
            if is_derived_render(path):
                continue
            try:
                for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
                    stripped = line.strip()
                    if not stripped or stripped.startswith("#"):
                        continue
                    key, _, value = stripped.partition(" ")
                    if key.lower().startswith(("map_", "bump", "norm", "disp", "decal")):
                        add(value.strip().split()[-1], path.relative_to(root).as_posix())
            except OSError:
                continue

    return [references[key] for key in sorted(references)]


def build_data(render_root: Path) -> tuple[dict, dict[str, str]]:
    art_manifest = read_json(render_root / "manifest.json")
    matches = read_json(render_root / "lam_overlap_rating_le3" / "matches.json")
    lam_manifest = read_json(render_root / "lam_similarity_pairs" / "render_manifest.json")
    pva_by_name: dict[str, dict[str, str]] = {}
    with PVA_ROSTER.open("r", encoding="utf-8", newline="") as stream:
        pva_rows = list(csv.DictReader(stream))
        pva_roster_count = len(pva_rows)
        for row in pva_rows:
            name = str(row.get("generator_name") or "").strip()
            index = str(row.get("generator_index") or "").strip()
            pair_roots = sorted(PVA_RENDER_ROOT.glob(f"{index}__{name}__seed_*"))
            candidates = []
            for pair_root in pair_roots:
                rest = pair_root / "rest.png"
                midstate = pair_root / "midstate.png"
                if rest.is_file() and midstate.is_file():
                    candidates.append({
                        "seed": pair_root.name.rsplit("__", 1)[-1],
                        "rest_path": str(rest),
                        "midstate_path": str(midstate),
                        "asset_dir": str(PVA_ASSET_ROOT / name / pair_root.name.rsplit("__", 1)[-1]),
                    })
            if name and len(candidates) >= 3:
                pva_by_name[name] = {
                    "generator_index": index,
                    "generator_name": name,
                    "source_type": str(row.get("source_type") or ""),
                    "assets": candidates[:3],
                }

    exact = set(matches["exact_text_matches"])
    match_rows: list[tuple[str, dict]] = []
    for row in matches["same_or_near_object_class_matches"]:
        level = "exact" if row["articraft_category"] in exact else "near"
        match_rows.append((level, row))
    match_rows.extend(
        ("related", row) for row in matches["related_family_or_mechanism_matches"]
    )

    matched_categories = {row["articraft_category"] for _, row in match_rows}
    art_by_category: dict[str, list[dict]] = defaultdict(list)
    for item in art_manifest["items"]:
        category = item["category"]
        if category not in matched_categories:
            continue
        rest = render_root / item["output"]
        midstate = render_root / "articraft" / category / item["asset_id"] / "midstate.png"
        art_by_category[category].append(
            {
                "asset_id": item["asset_id"],
                "rating": int(item["rating"]),
                "asset_dir": str(item.get("input") or ""),
                "urdf_path": str(item.get("input_file") or ""),
                "rest_path": rest if rest.is_file() else None,
                "midstate_path": midstate if midstate.is_file() else None,
            }
        )

    lam_by_category = {row["category"]: row for row in lam_manifest["rows"]}
    groups = []
    image_paths: dict[Path, str] = {}
    texture_index_cache: dict[str, list[dict]] = {}

    def indexed_textures(asset_dir: str, metadata_paths: list[str]) -> list[dict]:
        if not asset_dir:
            return []
        if asset_dir not in texture_index_cache:
            texture_index_cache[asset_dir] = texture_index(asset_dir, metadata_paths)
        return texture_index_cache[asset_dir]

    def register_image(path: Path) -> str:
        resolved = path.resolve()
        if resolved not in image_paths:
            image_paths[resolved] = f"img_{len(image_paths):04d}"
        return image_paths[resolved]

    missing_pva = sorted(set(PVA_MATCHES) - matched_categories)
    if missing_pva:
        raise KeyError(f"PV-A mapping has categories absent from Articraft matches: {missing_pva}")
    missing_pva_names = sorted(
        {
            generator
            for generator, _ in PVA_MATCHES.values()
            if generator not in pva_by_name
        }
    )
    if missing_pva_names:
        raise FileNotFoundError(f"PV-A representative image missing: {missing_pva_names}")

    level_order = {"exact": 0, "near": 1, "related": 2}
    for level, row in sorted(
        match_rows,
        key=lambda pair: (level_order[pair[0]], pair[1]["articraft_category"]),
    ):
        category = row["articraft_category"]
        pva_name, pva_relation = PVA_MATCHES[category]
        pva_record = pva_by_name[pva_name]
        pva_assets = [
            {
                "generator_index": pva_record["generator_index"],
                "generator_name": pva_name,
                "source_type": pva_record["source_type"],
                "relation": pva_relation,
                "seed": asset["seed"],
                "asset_dir": asset["asset_dir"],
                "urdf_path": str(Path(asset["asset_dir"]) / "model.urdf"),
                "appearance_path": str(Path(asset["asset_dir"]) / "appearance.json"),
                "physics_path": str(Path(asset["asset_dir"]) / "physics.json"),
                "package_path": source_package_path("PV-A", pva_name, asset["seed"]),
                "texture_index": indexed_textures(
                    asset["asset_dir"],
                    [str(Path(asset["asset_dir"]) / "appearance.json")],
                ),
                "rest": register_image(Path(asset["rest_path"])),
                "midstate": register_image(Path(asset["midstate_path"])),
            }
            for asset in pva_record["assets"]
        ]
        art_assets = []
        for asset in sorted(art_by_category[category], key=lambda value: value["asset_id"]):
            art_assets.append(
                {
                    "asset_id": asset["asset_id"],
                    "rating": asset["rating"],
                    "asset_dir": asset["asset_dir"],
                    "urdf_path": asset["urdf_path"],
                    "package_path": source_package_path("Articraft", category, asset["asset_id"]),
                    "texture_index": indexed_textures(asset["asset_dir"], []),
                    "rest": (
                        register_image(asset["rest_path"])
                        if asset["rest_path"]
                        else None
                    ),
                    "midstate": (
                        register_image(asset["midstate_path"])
                        if asset["midstate_path"]
                        else None
                    ),
                }
            )

        lam_assets = []
        for match in row["lam_categories"]:
            lam_row = lam_by_category.get(match["category"])
            if not lam_row:
                lam_assets.append(
                    {
                        "category": match["category"],
                        "asset_id": None,
                        "tier": "未渲染",
                        "audit_reasons": "",
                        "render_mode": "missing",
                        "metadata_asset_count": int(match["metadata_asset_count"]),
                        "in_frozen_100": bool(match["in_frozen_100"]),
                        "package_path": None,
                        "texture_index": [],
                        "rest": None,
                        "midstate": None,
                    }
                )
                continue
            # The manifest points at source assets; rendered images live in the output tree.
            rendered_dir = render_root / "lam_similarity_pairs" / match["category"] / lam_row["asset_id"]
            rest = rendered_dir / "rest.png"
            midstate = rendered_dir / "midstate.png"
            if not rest.is_file() or not midstate.is_file():
                raise FileNotFoundError(f"Missing LAM render pair: {rest}, {midstate}")
            lam_assets.append(
                {
                    "category": match["category"],
                    "asset_id": lam_row["asset_id"],
                    "tier": lam_row.get("tier", "unknown"),
                    "audit_reasons": lam_row.get("audit_reasons", ""),
                    "render_mode": lam_row.get("render_mode", "strict"),
                    "asset_dir": str(lam_row.get("asset_dir") or ""),
                    "urdf_path": str(Path(str(lam_row.get("asset_dir") or "")) / "generated.urdf"),
                    "package_path": source_package_path(
                        "LAM", match["category"], lam_row["asset_id"]
                    ),
                    "texture_index": indexed_textures(
                        str(lam_row.get("asset_dir") or ""), []
                    ),
                    "metadata_asset_count": int(match["metadata_asset_count"]),
                    "in_frozen_100": bool(match["in_frozen_100"]),
                    "rest": register_image(rest),
                    "midstate": register_image(midstate),
                }
            )
        groups.append(
            {
                "category": category,
                "level": level,
                "pva_assets": pva_assets,
                "art_assets": art_assets,
                "lam_assets": lam_assets,
            }
        )

    images = {image_id: image_data_url(path) for path, image_id in image_paths.items()}
    payload = {
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "groups": groups,
        "stats": {
            "groups": len(groups),
            "pva_roster_categories": pva_roster_count,
            "pva_categories": len({asset["generator_name"] for group in groups for asset in group["pva_assets"]}),
            "pva_assets": sum(len(group["pva_assets"]) for group in groups),
            "pva_unique_assets": len({(asset["generator_name"], asset["seed"]) for group in groups for asset in group["pva_assets"]}),
            "pva_exact_matches": sum(group["pva_assets"][0]["relation"] == "完全同名" for group in groups),
            "art_assets": sum(len(group["art_assets"]) for group in groups),
            "lam_references": sum(len(group["lam_assets"]) for group in groups),
            "lam_unique_assets": len(
                {
                    asset["asset_id"]
                    for group in groups
                    for asset in group["lam_assets"]
                    if asset["asset_id"]
                }
            ),
            "missing_lam_categories": len(
                {
                    asset["category"]
                    for group in groups
                    for asset in group["lam_assets"]
                    if not asset["asset_id"]
                }
            ),
            "lam_broken_assets": len(
                {
                    asset["asset_id"]
                    for group in groups
                    for asset in group["lam_assets"]
                    if asset["asset_id"] and asset["tier"] == "broken"
                }
            ),
            "lam_static_fallbacks": len(
                {
                    asset["asset_id"]
                    for group in groups
                    for asset in group["lam_assets"]
                    if asset["asset_id"]
                    and asset["render_mode"] == "combined_assembly_static_fallback"
                }
            ),
            "embedded_images": len(images),
            "incomplete_art_assets": sum(
                1
                for group in groups
                for asset in group["art_assets"]
                if not asset["rest"] or not asset["midstate"]
            ),
            "missing_art_images": sum(
                int(not asset[pose])
                for group in groups
                for asset in group["art_assets"]
                for pose in ("rest", "midstate")
            ),
        },
    }
    return payload, images


HTML_TEMPLATE = r'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PV-A / Articraft / LAM 三方类别对照</title>
<style>
:root{color-scheme:light;--ink:#202421;--muted:#666d68;--line:#d9ddd9;--paper:#f5f6f3;--surface:#fff;--green:#26734d;--green-soft:#e8f3ed;--amber:#9a5b00;--amber-soft:#fff1d8;--blue:#315f88;--blue-soft:#e8f1f8;--danger:#a33b32}
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:var(--paper);color:var(--ink);font:14px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif;letter-spacing:0}
button,input,select{font:inherit}.top{background:#1f2923;color:#fff;padding:24px max(20px,calc((100vw - 1540px)/2)) 20px}.top h1{font-size:25px;line-height:1.2;margin:0 0 7px}.top p{color:#cfd8d2;margin:0;max-width:900px}.summary{display:flex;gap:20px;flex-wrap:wrap;margin-top:17px}.summary strong{font-size:19px;display:block}.summary span{color:#bfcac3;font-size:12px}
.toolbar{position:sticky;top:0;z-index:20;background:rgba(255,255,255,.96);border-bottom:1px solid var(--line);box-shadow:0 2px 8px #1f292312}.controls{max-width:1540px;margin:auto;padding:10px 20px;display:grid;grid-template-columns:minmax(220px,1.8fr) repeat(3,minmax(145px,.7fr)) auto auto;gap:9px;align-items:end}.control label{display:block;color:var(--muted);font-size:11px;margin:0 0 3px}.control input[type=search],.control select{width:100%;height:36px;border:1px solid #bbc2bc;border-radius:5px;background:#fff;color:var(--ink);padding:0 10px;outline:none}.control input:focus,.control select:focus{border-color:var(--green);box-shadow:0 0 0 2px #26734d22}.check{height:36px;display:flex;align-items:center;gap:7px;white-space:nowrap}.actions{display:flex;gap:6px}.button{height:36px;border:1px solid #bbc2bc;border-radius:5px;background:#fff;color:var(--ink);padding:0 11px;cursor:pointer}.button:hover{border-color:#788079;background:#f7f8f6}
.status{max-width:1540px;margin:14px auto 8px;padding:0 20px;color:var(--muted)}main{max-width:1540px;margin:auto;padding:0 20px 50px}.empty{padding:60px 20px;text-align:center;color:var(--muted);background:#fff;border:1px solid var(--line)}
.group{background:var(--surface);border:1px solid var(--line);border-radius:7px;margin:10px 0;overflow:hidden}.group>summary{position:relative;list-style:none;cursor:pointer;padding:13px 15px 13px 49px;display:grid;grid-template-columns:minmax(280px,1fr) auto auto;gap:14px;align-items:center}.group>summary::-webkit-details-marker{display:none}.group>summary:before{content:"+";position:absolute;left:15px;top:50%;transform:translateY(-50%);width:22px;height:22px;border:1px solid #abb2ac;border-radius:4px;display:grid;place-items:center;font-weight:600}.group[open]>summary:before{content:"−"}.group-name{font-weight:650;overflow-wrap:anywhere}.group-counts{color:var(--muted);font-size:12px;white-space:nowrap}.badge{display:inline-flex;align-items:center;height:23px;padding:0 8px;border-radius:999px;font-size:11px;font-weight:650;white-space:nowrap}.badge.exact{color:var(--green);background:var(--green-soft)}.badge.near{color:var(--blue);background:var(--blue-soft)}.badge.related{color:var(--amber);background:var(--amber-soft)}
.group-body{border-top:1px solid var(--line);padding:0 15px 18px}.source{padding-top:15px}.source+.source{margin-top:18px;border-top:1px dashed #cfd4cf}.source-head{display:flex;gap:10px;align-items:baseline;margin-bottom:10px}.source-head h2{font-size:15px;margin:0}.source-head span{font-size:12px;color:var(--muted)}.asset-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:10px}.pva-grid{grid-template-columns:minmax(240px,360px)}.asset{border:1px solid var(--line);border-radius:6px;overflow:hidden;background:#fafbf9}.asset-meta{padding:8px 9px;border-bottom:1px solid var(--line);min-height:55px}.asset-title{font-size:12px;font-weight:650;line-height:1.3;overflow-wrap:anywhere}.asset-sub{display:flex;gap:8px;flex-wrap:wrap;margin-top:4px;color:var(--muted);font-size:11px}.asset-reason{margin-top:4px;color:#815c23;font-size:11px;overflow-wrap:anywhere}.rating,.broken{color:var(--danger);font-weight:650}.pva-mark{color:var(--green);font-weight:650}.frozen{color:var(--green);font-weight:650}.fallback{color:var(--amber);font-weight:650}.pva-single{background:var(--line)}.pair{display:grid;grid-template-columns:1fr 1fr;gap:1px;background:var(--line)}.shot{position:relative;aspect-ratio:1;background:#eef0ed;border:0;padding:0;cursor:zoom-in;overflow:hidden}.shot img{width:100%;height:100%;display:block;object-fit:contain}.shot span{position:absolute;left:5px;bottom:5px;padding:2px 6px;background:#182019cf;color:#fff;border-radius:3px;font-size:10px}.placeholder{width:100%;height:100%;display:grid;place-items:center;color:#89908b;font-size:11px}.shot.missing{display:grid;place-items:center;color:#8b514a;cursor:default;font-size:12px}
dialog{border:0;padding:0;background:#161a17;color:#fff;max-width:none;max-height:none;width:100vw;height:100vh}dialog::backdrop{background:#000}.lightbox{height:100%;display:grid;grid-template-rows:auto 1fr}.lightbox-head{display:flex;justify-content:space-between;align-items:center;padding:10px 14px;background:#202622}.lightbox-title{overflow-wrap:anywhere}.close{width:38px;height:34px;border:1px solid #657069;border-radius:5px;background:transparent;color:#fff;font-size:21px;cursor:pointer}.lightbox-stage{min-height:0;display:grid;place-items:center;padding:12px}.lightbox-stage img{max-width:100%;max-height:100%;object-fit:contain}.note{max-width:1540px;margin:0 auto;padding:14px 20px;color:#c4cdc7;font-size:12px}
@media(max-width:900px){.controls{grid-template-columns:1fr 1fr}.search{grid-column:1/-1}.actions{grid-column:1/-1}.group>summary{grid-template-columns:1fr auto}.group-counts{grid-column:1/-1}.asset-grid{grid-template-columns:1fr}}@media(max-width:520px){.controls{grid-template-columns:1fr}.search,.actions{grid-column:auto}.group>summary{grid-template-columns:1fr}.badge{width:max-content}.top h1{font-size:21px}}
</style>
</head>
<body>
<header class="top">
  <h1>PV-A / Articraft / LAM 三方类别对照</h1>
  <p>从 531 个 PV-A 模板中选取与 Articraft 低评分类别相同或相近的高质量代表，和 Articraft、LAM 的闭合与打开（midstate）渲染并排展示。</p>
  <div class="summary" id="summary"></div>
</header>
<div class="toolbar">
  <div class="controls">
    <div class="control search"><label for="search">搜索类别或资产 ID</label><input id="search" type="search" placeholder="例如 folding_chair"></div>
    <div class="control"><label for="level">匹配级别</label><select id="level"><option value="all">全部</option><option value="exact">完全同名</option><option value="near">同类 / 近同类</option><option value="related">相关家族 / 机构</option></select></div>
    <div class="control"><label for="rating">Articraft 评分</label><select id="rating"><option value="3">≤ 3</option><option value="2">≤ 2</option><option value="1">≤ 1</option></select></div>
    <div class="control"><label for="lam-tier">LAM 质量</label><select id="lam-tier"><option value="all">全部</option><option value="broken">仅看 broken</option><option value="normal">仅看 viable / loads_only</option></select></div>
    <label class="check"><input id="frozen" type="checkbox">仅看 Frozen-100 LAM</label>
    <div class="actions"><button class="button" id="expand" type="button">展开全部</button><button class="button" id="collapse" type="button">收起全部</button></div>
  </div>
</div>
<div class="status" id="status"></div>
<main id="groups"></main>
<footer class="top note">PV-A 每类提供 3 个高质量候选资产，均使用统一 studio、相同相机和灯光渲染闭合/打开两张图；Articraft 只保留评分 ≤3 的失败资产，LAM 展示匹配类别的代表资产。完全同名 = 类别字符串一致；其余为对象、机构或使用方式相近。PV-A 的打开图按 URDF 关节限位中点生成。broken 资产采用容错关节加载；“静态合并网格”表示原 URDF 的全部 visual mesh 引用失效，闭合和打开图均由 combined_assembly.obj 保底生成，因此可能完全相同。图片已嵌入本文件，可离线查看。</footer>
<dialog id="lightbox"><div class="lightbox"><div class="lightbox-head"><span class="lightbox-title" id="lightbox-title"></span><button class="close" id="lightbox-close" aria-label="关闭">×</button></div><div class="lightbox-stage"><img id="lightbox-image" alt="放大图"></div></div></dialog>
<script id="viewer-data" type="application/json">__VIEWER_DATA__</script>
<script id="image-data" type="application/json">__IMAGE_DATA__</script>
<script>
const DATA=JSON.parse(document.getElementById('viewer-data').textContent);
const IMAGES=JSON.parse(document.getElementById('image-data').textContent);
const $=id=>document.getElementById(id);
const labels={exact:'完全同名',near:'同类 / 近同类',related:'相关家族 / 机构'};
const state={search:'',level:'all',rating:3,lamTier:'all',frozen:false};
let observer;

function esc(value){return String(value).replace(/[&<>"']/g,char=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[char]));}
function imageButton(imageId,label,title){if(!imageId)return `<div class="shot missing"><span>${label}</span>缺少渲染</div>`;return `<button class="shot" data-image="${imageId}" data-title="${esc(title+' · '+label)}"><span>${label}</span><div class="placeholder">展开后加载</div></button>`;}
function pair(asset,title){return `<div class="pair">${imageButton(asset.rest,'闭合',title)}${imageButton(asset.midstate,'打开（midstate）',title)}</div>`;}
function artCard(asset){const title=asset.asset_id;return `<article class="asset"><div class="asset-meta"><div class="asset-title">${esc(title)}</div><div class="asset-sub"><span class="rating">评分 ${asset.rating}</span></div></div>${pair(asset,title)}</article>`;}
function pvaCard(asset){const title=asset.generator_name+' · '+asset.seed;return `<article class="asset"><div class="asset-meta"><div class="asset-title">${esc(asset.generator_name)}</div><div class="asset-sub"><span class="pva-mark">${esc(asset.generator_index)} · PV-A · ${esc(asset.seed)}</span><span>${esc(asset.relation)}</span></div></div>${pair(asset,title+' · PV-A')}</article>`;}
function lamCard(asset){const title=asset.category+(asset.asset_id?' / '+asset.asset_id:'');const fallback=asset.render_mode==='combined_assembly_static_fallback';return `<article class="asset"><div class="asset-meta"><div class="asset-title">${esc(asset.category)}</div><div class="asset-sub">${asset.asset_id?`<span>${esc(asset.asset_id)}</span>`:''}<span class="${asset.tier==='broken'?'broken':''}">${esc(asset.tier)}</span><span>元数据 ${asset.metadata_asset_count}</span>${fallback?'<span class="fallback">静态合并网格</span>':''}${asset.in_frozen_100?'<span class="frozen">Frozen-100</span>':''}</div>${asset.audit_reasons?`<div class="asset-reason">审计：${esc(asset.audit_reasons)}</div>`:''}</div>${pair(asset,title)}</article>`;}
function searchable(group){return [group.category,...group.pva_assets.map(x=>x.generator_name+' '+x.seed),...group.art_assets.map(x=>x.asset_id),...group.lam_assets.flatMap(x=>[x.category,x.asset_id,x.audit_reasons])].join(' ').toLowerCase();}
function visibleGroups(){const query=state.search.trim().toLowerCase();return DATA.groups.map(group=>{const art=group.art_assets.filter(x=>x.rating<=state.rating);const lam=group.lam_assets.filter(x=>(!state.frozen||x.in_frozen_100)&&(state.lamTier==='all'||(state.lamTier==='broken'&&x.tier==='broken')||(state.lamTier==='normal'&&x.tier!=='broken')));return {...group,art_assets:art,lam_assets:lam};}).filter(group=>(state.level==='all'||group.level===state.level)&&group.art_assets.length&&group.lam_assets.length&&(!query||searchable(group).includes(query)));}
function hydrate(scope=document){if(observer)observer.disconnect();observer=new IntersectionObserver(entries=>{for(const entry of entries){if(!entry.isIntersecting)continue;const button=entry.target;const image=document.createElement('img');image.alt=button.dataset.title;image.loading='lazy';image.src=IMAGES[button.dataset.image];image.addEventListener('error',()=>{button.querySelector('.placeholder').textContent='图片加载失败';});button.querySelector('.placeholder').replaceWith(image);observer.unobserve(button);}},{rootMargin:'300px'});scope.querySelectorAll('.shot[data-image]:not(:has(img))').forEach(node=>observer.observe(node));}
function render(){const groups=visibleGroups();let pvaCount=0,artCount=0,lamCount=0;const html=groups.map((group,index)=>{pvaCount+=group.pva_assets.length;artCount+=group.art_assets.length;lamCount+=group.lam_assets.length;return `<details class="group" data-index="${index}"><summary><div class="group-name">${esc(group.category)}</div><span class="badge ${group.level}">${labels[group.level]}</span><div class="group-counts">PV-A ${group.pva_assets.length} 个 · Articraft ${group.art_assets.length} 个 · LAM ${group.lam_assets.length} 类</div></summary><div class="group-body"><section class="source"><div class="source-head"><h2>PV-A（高质量）</h2><span>每类 3 个候选 · 531 类统一 studio</span></div><div class="asset-grid pva-grid">${group.pva_assets.map(pvaCard).join('')}</div></section><section class="source"><div class="source-head"><h2>Articraft（低评分）</h2><span>评分 ≤ ${state.rating}</span></div><div class="asset-grid">${group.art_assets.map(artCard).join('')}</div></section><section class="source"><div class="source-head"><h2>LAM</h2><span>对应类别代表资产</span></div><div class="asset-grid">${group.lam_assets.map(lamCard).join('')}</div></section></div></details>`;}).join('');
  $('groups').innerHTML=html||'<div class="empty">没有符合当前筛选条件的类别。</div>';
  $('status').textContent=`当前显示 ${groups.length} 个类别 · ${pvaCount} 个 PV-A 候选资产 · ${artCount} 个 Articraft 资产 · ${lamCount} 个 LAM 对应项`;
  document.querySelectorAll('.group').forEach(details=>details.addEventListener('toggle',()=>{if(details.open)hydrate(details);}));
  document.querySelectorAll('.shot[data-image]').forEach(button=>button.addEventListener('click',()=>openImage(button)));
}
function openImage(button){$('lightbox-title').textContent=button.dataset.title;$('lightbox-image').src=IMAGES[button.dataset.image];$('lightbox').showModal();}
function summary(){const s=DATA.stats;$('summary').innerHTML=`<div><strong>${s.groups}</strong><span>匹配类别组</span></div><div><strong>${s.pva_unique_assets}</strong><span>PV-A 唯一候选资产 · ${s.pva_categories} 类 · ${s.pva_assets} 个组内引用 · roster ${s.pva_roster_categories} 类 · ${s.pva_exact_matches} 个完全同名组</span></div><div><strong>${s.art_assets}</strong><span>Articraft 低评分资产</span></div><div><strong>${s.lam_unique_assets}</strong><span>LAM 资产 · ${s.lam_broken_assets} 个 broken</span></div><div><strong>${s.embedded_images}</strong><span>嵌入渲染图 · ${s.lam_static_fallbacks} 个静态保底${s.missing_art_images?' · '+s.missing_art_images+' 张 Articraft 缺失':''}</span></div>`;}
$('search').addEventListener('input',event=>{state.search=event.target.value;render();});
$('level').addEventListener('change',event=>{state.level=event.target.value;render();});
$('rating').addEventListener('change',event=>{state.rating=Number(event.target.value);render();});
$('lam-tier').addEventListener('change',event=>{state.lamTier=event.target.value;render();});
$('frozen').addEventListener('change',event=>{state.frozen=event.target.checked;render();});
$('expand').addEventListener('click',()=>{document.querySelectorAll('.group').forEach(x=>x.open=true);hydrate();});
$('collapse').addEventListener('click',()=>document.querySelectorAll('.group').forEach(x=>x.open=false));
$('lightbox-close').addEventListener('click',()=>$('lightbox').close());
$('lightbox').addEventListener('click',event=>{if(event.target===$('lightbox'))$('lightbox').close();});
document.addEventListener('keydown',event=>{if(event.key==='Escape'&&$('lightbox').open)$('lightbox').close();});
summary();render();
</script>
</body>
</html>
'''


def main() -> None:
    args = parse_args()
    render_root = ROOT / "failure_case_renders"
    payload, images = build_data(render_root)
    html = HTML_TEMPLATE.replace(
        "__VIEWER_DATA__", json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    ).replace(
        "__IMAGE_DATA__", json.dumps(images, ensure_ascii=True, separators=(",", ":"))
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(html, encoding="utf-8")
    print(json.dumps({"output": str(args.output), **payload["stats"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
