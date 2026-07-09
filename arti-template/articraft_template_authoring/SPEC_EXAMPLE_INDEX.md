# Spec Example Index

本文件定义 `articraft_template_authoring` 中 spec 示例的读取边界，避免 agent 把旧格式 spec 当成新模板格式来源。

## Canonical Modular V1 Examples

`specs_modular_v1/` 是新 spec 的唯一示例来源。新增类别 spec 必须以 `SPEC_TEMPLATE.md` 为准，并且只能从本目录学习 schema、章节顺序、slot/module/interface 写法和 procedural-first sampling / sweep plan 表达。

当前文件：

- `articulated_task_lamp.md`
- `astronomical_telescope_on_tripod.md`
- `bell_tower_with_swinging_bell.md`
- `clock_tower_with_rotating_hour_and_minute_hands.md`
- `candy_vending_machine.md`
- `casino_machine.md`
- `globe.md`
- `lighthouse_with_rotating_beacon_assembly.md`
- `missile_launcher.md`
- `metronome.md`
- `overshot_waterwheel.md`
- `paper_cutter_guillotine.md`
- `parabolic_dish_on_azimuth_elevation_mount.md`
- `playground_swing.md`
- `remote_weapon_station.md`
- `rotary_table_with_tilting_trunnion.md`
- `rotating_observatory_dome.md`
- `searchlight_tower.md`
- `single_rotor_helicopter.md`
- `singleleaf_drawbridge.md`
- `traditional_windmill.md`
- `turnstile_gates.md`
- `turntable.md`
- `dual_independent_finger_chains.md`
- `gear_assemblies.md`
- `car_axles.md`
- `drawer_cabinet_with_sliding_drawers.md`
- `branching_tree_with_two_independent_rotary_branches.md`
- `dishwasher_with_dropdown_door_and_sliding_racks.md`
- `simple_drying_rack.md`

## Transitional Modular References

`specs_modular_transitional/` 中的文件有 slot/module 思路和有价值的结构分析，但不保证完全符合当前 `SPEC_TEMPLATE.md` 的强制字段。可以参考结构识别、slot graph、validator 和 source adaptation 思路；禁止作为新 spec 的 schema 来源。

这些文件可能保留旧的 `seed=0 anchor`、Stage 1/Stage 2、coverage seed 术语；这些术语是历史迁移材料，不适用于新 spec。

当前文件：

- `robotic_arms.md`
- `robotic_leg.md`
- `single_revolute_hinge.md`
- `threestage_telescoping_slide.md`
- `twojoint_prismatic_chain.md`
- `twojoint_revolute_chain.md`
- `usb_drive_with_swivel_cover.md`
- `wheelbarrow.md`
- `wheelie_bin_with_hinged_lid.md`
- `wind_turbine.md`
- `zippo_Other_Lighter.md`

## Legacy Reference Only

`specs_legacy_reference_only/` 中的文件是旧 parts/joints spec、baseline style reference，或含 `primary_anchor` 的旧路线材料。它们只能用于理解历史类别边界、validator 风格或迁移需求；禁止作为 modular spec schema、source contract、seed-domain contract 或新模板 authoring 路线来源。

这些文件中的 anchor / baseline / parts-only schema 术语不得带入新 modular spec。

当前文件：

- `barrier_Door_Gate.md`
- `bicycle_crankset_and_pedal_assembly.md`
- `blender.md`
- `box_fan_with_control_knob.md`
- `camcorder_with_flipout_screen.md`
- `camera_flash.md`
- `camera_lens.md`
- `cannon.md`
- `cantilever_articulated_arm.md`
- `car_sunroof_cassette.md`
- `cctv_mast_with_pantilt_camera_head.md`
- `ceiling_fan.md`
- `ceiling_light_fixture.md`
- `chest_freezer_with_hinged_lid.md`
- `coaxial_rotary_stack.md`
- `crane_tower.md`
- `desk_with_drawer.md`
- `desktop_monitor_with_tilt_swivel_stand.md`
- `desktop_pc_tower.md`
- `display_freezer_with_sliding_glass_lids.md`
- `dj_equipment.md`
- `drone.md`
- `ferris_wheel.md`
- `graphics_card_with_cooling_fans.md`
- `louvered_shutter_assembly.md`
- `monitor_mount.md`
- `platform_cart.md`
- `refrigerator_with_hinged_doors.md`
- `retractable_utility_Handtools_Knife.md`
- `revolving_Door_Door.md`
- `rolling_toolbox_with_telescoping_handle.md`
- `screwcap_bottle.md`
- `screwin_light_bulb_with_socket.md`
- `serial_elbow_arm.md`
- `simple_aframe_step_ladder.md`
- `Window_Sliding_Window_Window.md`
- `stand_mixer.md`
- `standing_desk_with_synchronous_telescoping_legs_and_articulated_controls.md`
- `tackle_box_with_simple_hinged_lid.md`
- `telescoping_boom.md`
