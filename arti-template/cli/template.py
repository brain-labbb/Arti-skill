from __future__ import annotations

import argparse
import sys
from datetime import datetime, timezone
from pathlib import Path

from agent.runner import create_workbench_draft_record
from agent.template_sweep import (
    DEFAULT_COMPILE_TIMEOUT_S,
    DEFAULT_PASS_THRESHOLD,
    DEFAULT_SEED_COUNT,
    parse_seed_spec,
    report_to_json,
    run_sweep,
    stderr_progress_reporter,
    write_report,
)
from agent.template_sweep_pipeline import (
    PipelineStageResult,
    pipeline_report_to_json,
    run_sweep_pipeline,
    write_pipeline_report,
)
from cli.common import add_data_root_argument, warn_if_post_commit_hook_missing
from cli.external import _compile_record, _refresh_external_record, _validate_external_record
from storage.collections import CollectionStore
from storage.dataset_workflow import promote_record_workflow
from storage.datasets import DatasetStore
from storage.queries import StorageQueries
from storage.repo import StorageRepo
from storage.revisions import active_model_path
from storage.search import SearchIndex

ALLOWED_EXTERNAL_AGENTS = ("codex", "claude-code")
DEFAULT_PROVIDER_BY_AGENT = {
    "codex": "openai",
    "claude-code": "anthropic",
}

# Registry of procedural templates available for batch generation.
# Maps the template slug (module name under agent/templates/) to the function stem used
# inside that module: the template MUST export `build_<stem>`, `config_from_seed`, and
# `run_<stem>_tests`.
TEMPLATE_REGISTRY: dict[str, str] = {
    # --- 2026-07-05 batch: Technology 小类 (11) ---
    "Technology_Audio_Device": "audio_device",
    "Technology_Flashlight": "flashlight",
    "Technology_Graphics_Card": "graphics_card",
    "Technology_Keyboard": "keyboard",
    "Technology_Laptop": "laptop",
    "Technology_Mobile_Phone": "mobile_phone",
    "Technology_Monitor": "monitor",
    "Technology_Mouse": "mouse",
    "Technology_Printer": "printer",
    "Technology_Remote_Control": "remote_control",
    "Technology_Telescope": "telescope",
    # --- 2026-07-05 batch: Agricultural 小类 ---
    "harvester": "harvester",
    "watering_can": "watering_can",
    "tractor": "tractor",
    "greenhouse_vent_roof": "greenhouse_vent_roof",
    "hand_cultivator": "hand_cultivator",
    "single_wheelbarrow": "single_wheelbarrow",
    # --- 2026-07-05 batch: Electrical / Wiring 小类 (7) ---
    "Electrical_Wiring_Circuit_breaker": "circuit_breaker",
    "Electrical_Wiring_Junction_box": "junction_box",
    "Electrical_Wiring_Conduit_bender": "conduit_bender",
    "Electrical_Wiring_Wire_stripper": "wire_stripper",
    "Electrical_Wiring_Cable_reel": "cable_reel",
    "Electrical_Wiring_Surge_protector_switch": "surge_protector_switch",
    "Electrical_Wiring_Distribution_board_panel": "distribution_board_panel",
    # --- 2026-07-05 batch: Healthcare 小类 (6) ---
    "Healthcare_Adjustable_hospital_bed": "hospital_bed",
    "Healthcare_Wheelchair": "wheelchair",
    "Healthcare_Pill_bottle_box": "pill_box",
    "Healthcare_First_aid_box": "first_aid_box",
    "Healthcare_Prosthetic_limb": "prosthetic_leg",
    "Healthcare_Crutches": "walking_cane",
    # --- 2026-06-28 batch: Faucet 小类 (3, from qwen37v copy pool) ---
    "Other_single_hole_basin_faucet": "single_hole_basin_faucet",
    "Other_widespread_two_handle_faucet": "widespread_two_handle_faucet",
    "Other_high_arc_gooseneck_faucet": "high_arc_gooseneck_faucet",
    # --- 2026-06-25 batch: Sports subcategories (14 小类) ---
    "Sports_Baby_cycle": "baby_cycle",
    "Sports_Bike": "bicycle",
    "Sports_Carabiner": "carabiner",
    "Sports_Dumble": "dumbbell",
    "Sports_Exercise_bike": "exercise_bike",
    "Sports_Fidget_Spinner": "fidget_spinner",
    "Sports_Karting": "go_kart",
    "Sports_Roller_scates": "roller_skates",
    "Sports_Scooter_Racer": "kick_scooter",
    "Sports_Skateboard": "skateboard",
    "Sports_Skincare_Roller": "skincare_roller",
    "Sports_Table_football": "foosball_table",
    "Sports_Toy_car": "toy_car",
    "Sports_game_console": "handheld_game_console",
    # --- Military/Science/Powertools/Bench batch (21 小类) ---
    "Military_Aircraft": "military_aircraft",
    "Military_Granade": "grenade",
    "Military_Gun": "handgun",
    "Military_Military_knife": "military_knife",
    "Military_Radio": "field_radio",
    "Military_Rifle": "rifle",
    "Military_Tank": "armored_vehicle",
    "Military_Turret": "sentry_turret",
    "Military_knife": "katana",
    "Military_sword": "sword",
    "Science_Dental_setup": "dental_setup",
    "Science_Surgical_bed": "surgical_bed",
    "Science_Surgical_chair": "surgical_chair",
    "Science_microscope": "microscope",
    "Powertools_Lawn_mower": "lawn_mower",
    "Powertools_angle_grinder": "angle_grinder",
    "Powertools_drill": "cordless_drill",
    "Bench_Wood_Swing": "wood_swing",
    # --- 2026-06-25 batch: Playground subcategories ---
    "Playground_playground_chair_swing_carousel": "playground_chair_swing_carousel",
    "Playground_playground_merry_go_round": "playground_merry_go_round",
    "Playground_swing": "playground_swing_set",
    "Playground_swing_circular": "circular_ring_swing",
    # --- 2026-06-21 batch: 9 categories / 11 小类 ---
    "Bathroom_Hair_dryer": "hair_dryer",
    "Bathroom_washmachine": "washing_machine",
    "Equipment_Lock": "padlock",
    "Accessories_glasses": "glasses",
    "Bar_Piano": "piano",
    "Curtain_blind": "window_blind",
    "Facade_Element_Air_conditioner_outdoor_unit": "ac_outdoor_unit",
    "Facade_Element_Gutter_downchain": "rain_chain",
    "Facade_Element_Lamp1": "wall_lantern",
    "Fountain_Drick_fountain": "drinking_fountain",
    "Headwear_Racing_helmet": "racing_helmet",
    "Vehicle_Sports_car": "sports_car",
    "Door_wooden_plank_door_with_a_ring_pull": "plank_ring_door",
    "Others_Binocular": "binocular",
    "Technology_Conference_Phone": "conference_phone",
    "Other_Air_conditioner": "air_conditioner",
    "Other_Built_in_oven": "built_in_oven",
    "Other_stove": "stove",
    "Stationary_Calculater": "calculator",
    "Stationary_Clip": "clip",
    "Stationary_Clipboard": "clipboard",
    "Stationary_Folder": "folder",
    "Stationary_Pen": "pen",
    "Handtools_Pen": "pen",
    "Stationary_Pencil_sharpener": "pencil_sharpener",
    "Stationary_Scissors": "scissors",
    "Other_pliers": "pliers",
    "Handtools_Clamp": "clamp",
    "Other_Lighter": "lighter",
    "Others_Matchbox": "matchbox",
    "Others_Safe": "freestanding_security_safe",
    "Container_Bottle": "container_bottle",
    "Container_Jar": "container_jar",
    "Container_Barrel": "container_barrel",
    "Container_Basket": "container_basket",
    "Container_Bottle_serum": "container_bottle_serum",
    "Container_Box": "container_box",
    "Container_Can": "container_can",
    "Container_Cosmetic": "container_cosmetic",
    "Container_Cup": "container_cup",
    "Container_Dispenser": "container_dispenser",
    "Container_Gas_cylinder": "container_gas_cylinder",
    "Container_Glass_bottle": "container_glass_bottle",
    "Container_Kettle": "container_kettle",
    "Container_Lipstick": "container_lipstick",
    "Container_Locker": "container_locker",
    "Container_Paint_spray": "container_paint_spray",
    "Container_Plastic_can": "container_plastic_can",
    "Container_Primer_bottle": "container_primer_bottle",
    "Container_Pump": "container_pump",
    "Container_Shipping_container": "container_shipping_container",
    "Container_Tube": "container_tube",
    "Container_laundry_detergent_bottle": "container_laundry_detergent_bottle",
    "Accessories_Cushion": "cushion",
    "Urban_Environment_Fire_Extinguisher": "fire_extinguisher",
    "Urban_Environment_bucket2": "bucket2",
    "Urban_Environment_Fire_Hydrant": "fire_hydrant",
    "Sign_sign": "sign",
    "Door_Trap_door": "trap_door",
    "Door_Door": "door",
    "Door_Other": "door_other",
    "Door_Double_Door": "double_door",
    "Door_Gate": "gate",
    "Handtools_caulking_gun": "caulking_gun",
    "Other_armchair": "armchair",
    "Chair_Chair": "chair",
    "Other_Bedside": "bedside",
    "Other_cauldron": "cauldron",
    "Urban_Environment_Trashcan1": "trashcan1",
    "Urban_Environment_Trashcan2": "trashcan2",
    "Other_Coffin": "coffin",
    "Playground_Seesaw": "seesaw",
    "articulated_task_lamp": "articulated_task_lamp",
    "Military_helicopter": "tandem_rotor_helicopter",
    "Bathroom_toilet": "toilet",
    "Urban_Environment_Public_toilet": "public_toilet",
    "Urban_Environment_Public_toilet1": "furnished_public_toilet",
    "branching_tree_with_three_independent_rotary_branches": (
        "branching_tree_with_three_independent_rotary_branches"
    ),
    "branching_tree_with_two_independent_rotary_branches": (
        "branching_tree_with_two_independent_rotary_branches"
    ),
    "barrier_gate_boom": "barrier_gate",
    "barrier_gate_leaf_gate": "barrier_gate",
    "bell_tower_with_swinging_bell": "bell_tower_with_swinging_bell",
    "bicycle_crankset_and_pedal_assembly": "bicycle_crankset",
    "blender_countertop": "blender",
    "immersion_blender": "blender",
    "box_fan_with_control_knob": "box_fan",
    "camcorder_with_flipout_screen": "camcorder",
    "camera_flash": "camera_flash",
    "camera_lens": "camera_lens",
    "Science_Capsule": "capsule",
    "cannon": "cannon",
    "candy_vending_machine": "candy_vending_machine",
    "Other_Water_dispenser": "water_dispenser",
    "casino_machine": "casino_machine",
    "cantilever_articulated_arm": "cantilever_arm",
    "cctv_mast_with_pantilt_camera_head": "cctv_mast_camera",
    "Technology_Security_Camera": "security_camera",
    "Urban_Environment_Roof_antena": "roof_antenna",
    "ceiling_fan": "ceiling_fan",
    "ceiling_light_fixture_adjustable": "ceiling_light",
    "chest_freezer_with_hinged_lid": "chest_freezer",
    "coaxial_rotary_stack": "coaxial_rotary_stack",
    "crane_tower": "crane_tower",
    "desk_with_drawer": "desk_with_drawer",
    "desk_with_drawer_card_catalog": "desk_with_drawer",
    "desktop_monitor_with_tilt_swivel_stand": "desktop_monitor",
    "Handtools_dial_caliper": "dial_caliper",
    "Handtools_Wrench": "wrench",
    "Handtools_Hand_plane": "hand_plane",
    "Handtools_Knife": "knife",
    "Handtools_Paint_roller": "paint_roller",
    "Handtools_Stapler": "stapler",
    "Handtools_Tool_cart": "tool_cart",
    "Urban_Environment_Caster_Trolley": "caster_trolley",
    "Urban_Environment_Caster_Trolley2": "shopping_cart",
    "Handtools_clothes_peg": "clothes_peg",
    "Handtools_wooden_tongs": "wooden_tongs",
    "Others_blacksmith_tongs": "blacksmith_tongs",
    "Kitchen_Air_fryer": "air_fryer",
    "Kitchen_Coffee_machine": "coffee_machine",
    "Kitchen_Corkscrew": "corkscrew",
    "Kitchen_Dish_washer": "dish_washer",
    "Kitchen_Hood": "hood",
    "Kitchen_Knife_set": "knife_set",
    "Urban_Environment_Manhole_cover": "manhole_cover",
    "Kitchen_Microwave": "microwave",
    "Kitchen_Toaster": "toaster",
    "monitor_mount": "monitor_mount",
    "multisegment_foldout_arm": "multisegment_foldout_arm",
    "n_joint_revolute_chain": "n_joint_revolute_chain",
    "paper_cutter_guillotine": "paper_cutter_guillotine",
    "parabolic_dish_on_azimuth_elevation_mount": "parabolic_dish",
    "Urban_Environment_sci_fi_satellite_dish": "satellite_dish",
    "playground_swing": "playground_swing",
    "desktop_pc_tower": "desktop_pc_tower",
    "display_freezer_with_sliding_glass_lids": "display_freezer",
    "dj_equipment": "dj_equipment",
    "drone": "drone",
    "Structure_Elevator": "elevator",
    "elevator_shaft": "elevator_shaft",
    "graphics_card_with_cooling_fans": "graphics_card",
    "globe": "globe",
    "louvered_shutter_assembly": "louvered_shutter",
    "miter_saw_arm_assembly": "miter_saw_arm_assembly",
    "retractable_utility_knife": "retractable_utility_knife",
    "screwcap_bottle": "screwcap_bottle",
    "screwin_light_bulb_with_socket": "screwin_light_bulb_with_socket",
    "serial_elbow_arm": "serial_elbow_arm",
    "ferris_wheel": "ferris_wheel",
    "Fence_Cascade_fences_MORE_THAN_1": "fence_cascade",
    "folding_arm_chain": "folding_arm_chain",
    "Window_Sliding_window": "sliding_window",
    "sliding_window_classic": "sliding_window_classic",
    "Window_Window": "window",
    "Urban_Environment_Phone_box": "phone_box",
    "Door_Sliding_Door": "sliding_door",
    "sluice_gate_with_vertical_lift_panel": "sluice_gate_with_vertical_lift_panel",
    "studio_spotlight_on_yoke": "studio_spotlight_on_yoke",
    "tackle_box_with_simple_hinged_lid": "tackle_box",
    "telescoping_boom": "telescoping_boom",
    "turnstile_gates": "turnstile_gates",
    "Other_Tripod_Turnstile": "tripod_turnstile",
    "turntable": "turntable",
    "vane_array_with_independent_pivots": "vane_array_with_independent_pivots",
    "wall_safe_with_hinged_door_and_dial": "wall_safe_with_hinged_door_and_dial",
    "windshield_wiper_assembly": "windshield_wiper_assembly",
    "wind_turbine": "wind_turbine",
    "standing_desk_with_synchronous_telescoping_legs_and_articulated_controls": "standing_desk",
    "platform_cart": "platform_cart",
    "rolling_toolbox_with_telescoping_handle": "rolling_toolbox",
    "refrigerator_with_hinged_doors": "refrigerator",
    "revolving_door": "revolving_door",
    "robotic_arms": "robotic_arms",
    "robotic_leg": "robotic_leg",
    "wheelbarrow": "wheelbarrow",
    "seed_spreader": "seed_spreader",
    "Urban_Environment_Tipping_Barrow": "tipping_barrow",
    "Urban_Environment_Draft_Wagon": "draft_wagon",
    "Urban_Environment_Large_Trashcan": "large_trashcan",
    "satellite_with_articulated_solar_panels": "satellite_with_articulated_solar_panels",
    "simple_aframe_step_ladder": "simple_aframe_step_ladder",
    "stand_mixer": "stand_mixer",
    "searchlight_tower": "searchlight_tower",
    "shoulderelbowwrist_arm": "shoulderelbowwrist_arm",
    "astronomical_telescope_on_tripod": "astronomical_telescope_on_tripod",
    "clock_tower_with_rotating_hour_and_minute_hands": "clock_tower_with_rotating_hour_and_minute_hands",
    "missile_launcher": "missile_launcher",
    "remote_weapon_station": "remote_weapon_station",
    "rotating_observatory_dome": "observatory_dome",
    "rotary_table_with_tilting_trunnion": "rotary_table_with_tilting_trunnion",
    "jet_engine": "jet_engine",
    "lever_chain": "lever_chain",
    "pushpull_plunger_chain": "pushpull_plunger_chain",
    "drawer_cabinet_with_sliding_drawers": "drawer_cabinet_with_sliding_drawers",
    "Other_Cabinet": "cabinet",
    "Urban_Environment_utility_box": "utility_box",
    "Science_First_aid_cabinet": "first_aid_cabinet",
    "dual_independent_finger_chains": "dual_independent_finger_chains",
    "fingerlike_phalanx_chain": "fingerlike_phalanx_chain",
    "car_axles": "car_axles",
    "gear_assemblies": "gear_assemblies",
    "dishwasher_with_dropdown_door_and_sliding_racks": (
        "dishwasher_with_dropdown_door_and_sliding_racks"
    ),
    "simple_drying_rack": "simple_drying_rack",
    "Bag_Suitcase_Box": "bag_suitcase_box",
    "Bag_Suitcase_Luggage_bag": "luggage_bag",
    "Chair_Folding_chair": "folding_chair",
    "Bag_Suitcase_Shopping_bucket": "shopping_bucket",
    "Urban_Environment_bucket1": "bucket1",
    "Bag_Suitcase_Suitcase": "suitcase",
    "Bag_Suitcase_Treasure_chest": "treasure_chest",
    "Other_Folding_screen": "folding_screen",
    "Door_folding_door": "folding_door",
    "Door_Folding_gate": "folding_gate",
    "Other_Scale": "scale",
    "Other_Switch": "switch",
    "Other_TV": "tv",
    "Other_Metal_drain": "metal_drain",
    "Other_Vent": "vent",
    "Door_Garage_shutter": "garage_shutter",
    "Door_Scifi_Gate": "scifi_gate",
    "Stairs_Escalator": "escalator",
    "Science_Syringe": "syringe",
    "Urban_Environment_Garbage_bin": "garbage_bin",
    # --- 2026-06 picture-subcategory batch (9 小类) ---
    "Equipment_Pipeline": "pipeline",
    "Equipment_Game_console": "arcade_cabinet",
    "Equipment_Control_panel": "control_panel",
    "Equipment_LED_work_light": "led_work_light",
    "Light_Latern": "hurricane_lantern",
    "Machinery_Watermill": "watermill_waterwheel",
    "Parts_quick_release_clamp": "quick_release_clamp",
    "Equipment_Power_switch": "power_switch",
    "Equipment_Pump2": "hand_pump",
    "Urban_Environment_Mailbox": "mailbox",
    "Urban_Environment_Well_lid": "well_lid",
    "Urban_Environment_Fire_cabinet": "fire_cabinet",
    # --- 2026-06-27 batch: Music subcategories (6 小类) ---
    "Music_Amplifier": "guitar_amplifier",
    "Music_CD_case": "cd_jewel_case",
    "Music_Headphone": "over_ear_headphones",
    "Music_Violin_case": "violin_case",
    "Music_Vocal_mic": "vocal_microphone",
    "Music_keyboard": "music_keyboard",
    # --- previously unregistered built-in templates ---
    "lighthouse_with_rotating_beacon_assembly": "lighthouse_with_rotating_beacon_assembly",
    "metronome": "metronome",
    "overshot_waterwheel": "overshot_waterwheel",
    "single_revolute_hinge": "single_revolute_hinge",
    "single_rotor_helicopter": "single_rotor_helicopter",
    "singleleaf_drawbridge": "singleleaf_drawbridge",
    "threestage_telescoping_slide": "threestage_telescoping_slide",
    "traditional_windmill": "traditional_windmill",
    "twojoint_prismatic_chain": "twojoint_prismatic_chain",
    "twojoint_revolute_chain": "twojoint_revolute_chain",
    "usb_drive_with_swivel_cover": "usb_drive_with_swivel_cover",
    "wheelie_bin_with_hinged_lid": "wheelie_bin_with_hinged_lid",
    "zippo_lighter": "zippo_lighter",
}

GENERIC_MODEL_TEMPLATE = """from __future__ import annotations

from agent.templates.{slug} import (
    build_{stem},
    config_from_seed,
    run_{stem}_tests,
)
from sdk import AssetContext

SEED = {seed}
CONFIG = config_from_seed(SEED)
ASSETS = AssetContext.from_script(__file__)


def build_object_model():
    return build_{stem}(CONFIG, assets=ASSETS)


def run_tests():
    return run_{stem}_tests(object_model, CONFIG)


object_model = build_object_model()
"""


def _write_template_model(model_path: Path, *, slug: str, stem: str, seed: int) -> None:
    model_path.parent.mkdir(parents=True, exist_ok=True)
    body = GENERIC_MODEL_TEMPLATE.format(slug=slug, stem=stem, seed=seed)
    model_path.write_text(body, encoding="utf-8")


def batch_template(
    repo_root: Path,
    *,
    slug: str,
    stem: str,
    seeds: list[int],
    agent: str,
    category_slug: str | None,
    dry_run: bool,
) -> int:
    if dry_run:
        for seed in seeds:
            prompt = f"seeded {slug} {seed}"
            print(f"[seed={seed}] dry-run: would create record for prompt={prompt!r}")
        print(f"batch dry-run completed for {len(seeds)} seed(s)")
        return 0

    repo = StorageRepo(repo_root)
    repo.ensure_layout()
    provider = DEFAULT_PROVIDER_BY_AGENT[agent]
    failures: list[str] = []

    # Gate every requested seed through the SAME machinery the sweep uses
    # (full baseline + motion QC) BEFORE any record is materialized. Batch is
    # the promotion step into the dataset; the sweep only certifies seeds
    # 0-49 + corners at a 0.95 threshold, so an arbitrary --seeds value (or
    # one of the tolerated sweep failures) must not ship unchecked.
    from agent.template_sweep import run_seed_outcomes

    print(f"pre-validating {len(seeds)} seed(s) with the sweep gate (motion QC on)...")
    gate_outcomes = run_seed_outcomes(
        slug=slug,
        stem=stem,
        seeds=list(seeds),
        max_workers=min(4, max(1, len(seeds))),
        repo_root=repo_root,
        compile_timeout_s=180.0,
        motion_qc=True,
    )
    failing_gate = {o.seed: o for o in gate_outcomes if o.verdict != "pass"}
    for seed in sorted(failing_gate):
        outcome = failing_gate[seed]
        failures.append(
            f"seed={seed}: rejected by the batch gate ({outcome.failure_type}): "
            f"{(outcome.failure_details or '')[:300]}"
        )
        print(f"[seed={seed}] REJECTED by batch gate: {outcome.failure_type}")
    seeds = [seed for seed in seeds if seed not in failing_gate]

    for seed in seeds:
        prompt = f"seeded {slug} {seed}"
        print(f"[seed={seed}] starting")
        try:
            record_dir = create_workbench_draft_record(
                repo_root=repo_root,
                prompt_text=prompt,
                provider=provider,
                model_id=None,
                thinking_level=None,
                sdk_package="sdk",
                label=f"{stem}_seed_{seed}",
                tags=["template_batch", slug],
                record_id=None,
                external_agent=agent,
            )
        except ValueError as exc:
            failures.append(f"seed={seed}: init failed: {exc}")
            continue

        record_id = record_dir.name
        model_path = active_model_path(repo, record_id)
        _write_template_model(model_path, slug=slug, stem=stem, seed=seed)

        status = _compile_record(repo_root, record_dir, target="visual", validate=True)
        if status != 0:
            failures.append(f"seed={seed}: compile/check failed for {record_id}")
            continue

        errors = _validate_external_record(repo, record_id)
        if errors:
            failures.append(f"seed={seed}: validation failed for {record_id}: {'; '.join(errors)}")
            continue

        if category_slug:
            _refresh_external_record(
                repo,
                record_id,
                final_status="external_finalized",
            )
            try:
                entry, category, _, _stats = promote_record_workflow(
                    repo,
                    DatasetStore(repo),
                    StorageQueries(repo),
                    record_id=record_id,
                    category_title=None,
                    category_slug=category_slug,
                    dataset_id=None,
                    promoted_at=datetime.now(timezone.utc)
                    .replace(microsecond=0)
                    .isoformat()
                    .replace("+00:00", "Z"),
                )
            except ValueError as exc:
                failures.append(f"seed={seed}: promote failed for {record_id}: {exc}")
                continue
            CollectionStore(repo).ensure_workbench()
            print(
                f"[seed={seed}] finalized into category={category.get('slug') or category_slug}: "
                f"{record_id} dataset_id={entry.get('dataset_id')}"
            )
        else:
            _refresh_external_record(repo, record_id, final_status="external_ready")
            print(f"[seed={seed}] ready: {record_id}")

    stats = SearchIndex(repo).rebuild()
    print(
        f"search_index={stats.path} records={stats.record_count} "
        f"categories={stats.category_count} workbench_entries={stats.workbench_entry_count}"
    )

    if failures:
        print("batch completed with failures:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print(f"batch completed successfully for {len(seeds)} seed(s)")
    return 0


def batch_ferris_wheel(
    repo_root: Path,
    *,
    seeds: list[int],
    agent: str,
    category_slug: str | None,
    dry_run: bool,
) -> int:
    return batch_template(
        repo_root,
        slug="ferris_wheel",
        stem=TEMPLATE_REGISTRY["ferris_wheel"],
        seeds=seeds,
        agent=agent,
        category_slug=category_slug,
        dry_run=dry_run,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="articraft template")
    add_data_root_argument(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    batch = subparsers.add_parser(
        "batch", help="Batch-generate records from a procedural template."
    )
    batch_sub = batch.add_subparsers(dest="template_name", required=True)

    for slug in TEMPLATE_REGISTRY:
        sp = batch_sub.add_parser(slug, help=f"Batch-generate seeded {slug} records.")
        sp.add_argument(
            "--seeds",
            required=True,
            help="Seed list/ranges, e.g. '1-20' or '1,3,5-8'.",
        )
        sp.add_argument("--agent", default="codex", choices=ALLOWED_EXTERNAL_AGENTS)
        sp.add_argument(
            "--category-slug",
            default=None,
            help="Optional dataset category slug. When set, records are finalized and promoted.",
        )
        sp.add_argument(
            "--dry-run",
            action="store_true",
            help="Print the planned batch without creating records.",
        )

    sweep = subparsers.add_parser(
        "compile-sweep",
        help="Run multi-seed full-baseline compile sweep for a procedural template.",
    )
    sweep.add_argument("slug", choices=sorted(TEMPLATE_REGISTRY.keys()))
    sweep.add_argument(
        "--seeds",
        default=f"0-{DEFAULT_SEED_COUNT - 1}",
        help=(f"Seed list/ranges; defaults to '0-{DEFAULT_SEED_COUNT - 1}' (DEFAULT_SEED_COUNT)."),
    )
    sweep.add_argument(
        "--pass-threshold",
        type=float,
        default=DEFAULT_PASS_THRESHOLD,
        help=(f"Minimum pass_rate required for verdict=pass (default {DEFAULT_PASS_THRESHOLD})."),
    )
    sweep.add_argument(
        "--max-workers",
        type=int,
        default=None,
        help=(
            "ProcessPoolExecutor worker count. Defaults to min(len(seeds), 4); "
            "use 1 to run sequentially in the current process."
        ),
    )
    sweep.add_argument(
        "--sdk-package", default="sdk", help="SDK package to load (defaults to 'sdk')."
    )
    sweep.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Optional file path to write the JSON report to (in addition to stdout).",
    )
    sweep.add_argument(
        "--state-dir",
        type=Path,
        default=None,
        help=(
            "Directory to persist per-slug streak state across sweeps. "
            "Defaults to <repo_root>/.articraft/template_sweep_state when omitted. "
            "Pass an empty string to disable streak tracking."
        ),
    )
    sweep.add_argument(
        "--compile-timeout",
        type=float,
        default=DEFAULT_COMPILE_TIMEOUT_S,
        help=(
            "Per-seed wall-time budget in seconds. Each seed compile runs in a "
            "fresh subprocess that is SIGKILL'd on timeout; the seed is marked "
            "compile_timeout in the JSON. Set to 0 to disable timeouts ("
            f"in-process ProcessPool path). Default {DEFAULT_COMPILE_TIMEOUT_S:.0f}s."
        ),
    )
    sweep.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress per-seed stderr progress lines.",
    )
    sweep.add_argument(
        "--no-motion-qc",
        action="store_true",
        help=(
            "Disable the harness-owned sampled-pose overlap gate "
            "(harness_motion_qc). By default every seed compile also checks "
            "part overlaps at joint-limit poses."
        ),
    )

    pipeline = subparsers.add_parser(
        "sweep-pipeline",
        help="Run incremental seed0/fast/medium/final/corner compile sweep pipeline.",
    )
    pipeline.add_argument("slug", choices=sorted(TEMPLATE_REGISTRY.keys()))
    pipeline.add_argument(
        "--pass-threshold",
        type=float,
        default=DEFAULT_PASS_THRESHOLD,
        help=(
            f"Minimum pass_rate required for each stage verdict=pass (default {DEFAULT_PASS_THRESHOLD})."
        ),
    )
    pipeline.add_argument(
        "--max-workers",
        type=int,
        default=None,
        help=(
            "ProcessPoolExecutor worker count. Defaults to min(len(stage seeds), 4); "
            "use 1 to run sequentially in the current process."
        ),
    )
    pipeline.add_argument(
        "--sdk-package", default="sdk", help="SDK package to load (defaults to 'sdk')."
    )
    pipeline.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Optional file path to write the JSON report to (in addition to stdout).",
    )
    pipeline.add_argument(
        "--state-dir",
        type=Path,
        default=None,
        help=(
            "Directory to persist per-slug streak state across pipeline runs. "
            "Defaults to <repo_root>/.articraft/template_sweep_state when omitted. "
            "Pass an empty string to disable streak tracking."
        ),
    )
    pipeline.add_argument(
        "--compile-timeout",
        type=float,
        default=DEFAULT_COMPILE_TIMEOUT_S,
        help=(
            "Per-seed wall-time budget in seconds. Each seed compile runs in a "
            "fresh subprocess that is SIGKILL'd on timeout; the seed is marked "
            "compile_timeout in the JSON. Set to 0 to disable timeouts ("
            f"in-process ProcessPool path). Default {DEFAULT_COMPILE_TIMEOUT_S:.0f}s."
        ),
    )
    pipeline.add_argument(
        "--quiet",
        action="store_true",
        help="Suppress per-seed and per-stage progress lines.",
    )
    pipeline.add_argument(
        "--no-motion-qc",
        action="store_true",
        help=(
            "Disable the harness-owned sampled-pose overlap gate "
            "(harness_motion_qc). By default every seed compile also checks "
            "part overlaps at joint-limit poses."
        ),
    )
    pipeline.add_argument(
        "--no-corner-seeds",
        action="store_true",
        help=(
            "Disable the corner-seed stage. By default the pipeline appends "
            "deterministically selected extra seeds that hit per-field numeric "
            "extremes and slot combos the 0-49 sweep never realized."
        ),
    )

    return parser


def _resolve_state_dir(repo_root: Path, override: Path | None) -> Path | None:
    """Resolve the per-slug streak-state directory.

    Passing an empty string on the CLI maps to override=Path('') which we treat
    as 'disable streak tracking' (returns None). Otherwise default to
    `<repo_root>/.articraft/template_sweep_state`.
    """
    if override is None:
        return Path(repo_root) / ".articraft" / "template_sweep_state"
    text = str(override).strip()
    if not text or text == ".":
        return None
    return Path(text)


def compile_sweep(
    *,
    slug: str,
    stem: str,
    seeds: list[int],
    pass_threshold: float,
    max_workers: int | None,
    sdk_package: str,
    out_path: Path | None,
    state_dir: Path | None,
    compile_timeout_s: float,
    quiet: bool,
    motion_qc: bool = True,
) -> int:
    progress = None if quiet else stderr_progress_reporter(total=len(seeds))
    try:
        report = run_sweep(
            slug=slug,
            stem=stem,
            seeds=seeds,
            sdk_package=sdk_package,
            pass_threshold=pass_threshold,
            max_workers=max_workers,
            progress=progress,
            state_dir=state_dir,
            compile_timeout_s=compile_timeout_s,
            motion_qc=motion_qc,
        )
    except (FileNotFoundError, AttributeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    payload = report_to_json(report)
    print(payload)
    if out_path is not None:
        write_report(report, out_path=out_path)
    return 0 if report.verdict == "pass" else 1


def _pipeline_stage_progress(event: str, stage: PipelineStageResult) -> None:
    if event == "start":
        print(
            f"[stage={stage.name}] starting added={stage.added_seeds} "
            f"cumulative={stage.cumulative_seeds[0]}-{stage.cumulative_seeds[-1]}",
            file=sys.stderr,
            flush=True,
        )
        return
    if stage.status == "skipped":
        print(f"[stage={stage.name}] skipped", file=sys.stderr, flush=True)
        return
    report = stage.report
    elapsed = "" if report is None else f" ({report.elapsed_s:.2f}s)"
    print(
        f"[stage={stage.name}] {stage.status.upper()}{elapsed}",
        file=sys.stderr,
        flush=True,
    )


def sweep_pipeline(
    *,
    slug: str,
    stem: str,
    pass_threshold: float,
    max_workers: int | None,
    sdk_package: str,
    out_path: Path | None,
    state_dir: Path | None,
    compile_timeout_s: float,
    quiet: bool,
    motion_qc: bool = True,
    corner_seeds: bool = True,
) -> int:
    progress = None if quiet else stderr_progress_reporter(total=DEFAULT_SEED_COUNT)
    stage_progress = None if quiet else _pipeline_stage_progress
    try:
        report = run_sweep_pipeline(
            slug=slug,
            stem=stem,
            sdk_package=sdk_package,
            pass_threshold=pass_threshold,
            max_workers=max_workers,
            progress=progress,
            stage_progress=stage_progress,
            state_dir=state_dir,
            compile_timeout_s=compile_timeout_s,
            motion_qc=motion_qc,
            corner_seeds=corner_seeds,
        )
    except (FileNotFoundError, AttributeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    payload = pipeline_report_to_json(report)
    print(payload)
    if out_path is not None:
        write_pipeline_report(report, out_path=out_path)
    total_seeds = len(report.stages[-1].cumulative_seeds) if report.stages else DEFAULT_SEED_COUNT
    print(
        f"sweep-pipeline {slug} {report.verdict.upper()} "
        f"stages={len(report.stages)} seeds={total_seeds} elapsed={report.elapsed_s:.2f}s",
        file=sys.stderr,
        flush=True,
    )
    return 0 if report.verdict == "pass" else 1


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.command == "compile-sweep":
        try:
            seeds = parse_seed_spec(args.seeds)
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 2
        state_dir = _resolve_state_dir(args.repo_root, args.state_dir)
        return compile_sweep(
            slug=args.slug,
            stem=TEMPLATE_REGISTRY[args.slug],
            seeds=seeds,
            pass_threshold=float(args.pass_threshold),
            max_workers=(None if args.max_workers is None else int(args.max_workers)),
            sdk_package=str(args.sdk_package),
            out_path=args.out,
            state_dir=state_dir,
            compile_timeout_s=float(args.compile_timeout),
            quiet=bool(args.quiet),
            motion_qc=not bool(args.no_motion_qc),
        )

    if args.command == "sweep-pipeline":
        state_dir = _resolve_state_dir(args.repo_root, args.state_dir)
        return sweep_pipeline(
            slug=args.slug,
            stem=TEMPLATE_REGISTRY[args.slug],
            pass_threshold=float(args.pass_threshold),
            max_workers=(None if args.max_workers is None else int(args.max_workers)),
            sdk_package=str(args.sdk_package),
            out_path=args.out,
            state_dir=state_dir,
            compile_timeout_s=float(args.compile_timeout),
            quiet=bool(args.quiet),
            motion_qc=not bool(args.no_motion_qc),
            corner_seeds=not bool(args.no_corner_seeds),
        )

    if args.command != "batch" or args.template_name not in TEMPLATE_REGISTRY:
        parser.error("Unsupported template command")

    try:
        seeds = parse_seed_spec(args.seeds)
    except ValueError as exc:
        print(str(exc))
        return 1

    if not args.dry_run:
        warn_if_post_commit_hook_missing(args.repo_root)

    return batch_template(
        args.repo_root,
        slug=args.template_name,
        stem=TEMPLATE_REGISTRY[args.template_name],
        seeds=seeds,
        agent=args.agent,
        category_slug=str(args.category_slug or "").strip() or None,
        dry_run=bool(args.dry_run),
    )
