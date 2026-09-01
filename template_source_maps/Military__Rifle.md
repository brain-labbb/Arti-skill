# Military / Rifle — template source map

pattern: mixed

parents: rec_model-an-m4-style-military-carbine-rifle-all-mat_20260610_081537_124088_497c83bd ← picture/Military/Rifle/001.png (single母资产, covers all four slots' base candidate: collapsible buttstock / quad-Picatinny handguard / red-dot reflex sight / birdcage muzzle; spine parts = receiver + barrel + handguard + magazine + trigger + safety_selector; 5 nonfixed joints: stock_slide·charging_handle_slide·magazine_release PRISMATIC + trigger_pull·safety_selector_rotate REVOLUTE)

## Slot 候选覆盖

The fixed structural spine — `receiver` (with `upper_receiver`/`rail_base`/`rail_ribs`/`lower_receiver`/`magwell`/`guard_bar`/`buffer_tube`/`grip_body`/`delta_ring`), `barrel` (`barrel_blank`), `magazine` (`mag_top`/`mag_body`, joint `magazine_release` prismatic on canted axis), `trigger` (`trigger_blade`, joint `trigger_pull` revolute +Y), `charging_handle` (`handle_shaft`, joint `charging_handle_slide` prismatic -X), `safety_selector` (`selector_lever`, joint `safety_selector_rotate` revolute -Y) — is invariant across all variants. Only the four slots below swap.

### Slot A:buttstock (rear shoulder support, mounts off receiver buffer-tube axis)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| collapsible | rec_model-an-m4-style-military-carbine-rifle-all-mat_20260610_081537_124088_497c83bd | part `buttstock`(`stock_body`) / joint `stock_slide` PRISMATIC axis(1,0,0) | six-position polymer collar riding the full `buffer_tube`; slides forward to collapse, 0.09 m travel | converged (parent) |
| fixed_A2 | rec_rifle_var_fixedstock | part `buttstock`(`stock_body`/`butt_pad`/`sling_swivel`) / joint `receiver_to_stock` FIXED | solid one-piece A2 shell fully enclosing buffer tube + castle nut; rubber butt pad at rear, no travel (4 nonfixed joints) | converged (workbench, rating pending sync) |
| side_folding | rec_rifle_var_foldstock | parts `receiver`(adds `buffer_tube_stub`/`hinge_bracket`/`hinge_pin`) + `buttstock`(`stock_knuckle`/`stock_body`) / joint `stock_fold` REVOLUTE axis(0,0,-1) ~175° | skeletonized arm on left-wall hinge bracket; folds flat to +Y against receiver; buffer tube shortened to stub | converged (workbench, rating pending sync) |
| pdw_wire | rec_rifle_var_pdwstock | part `buttstock`(`stock_collar`/`stock_pad`/`stock_rail_0`/`stock_rail_1`) / joint `stock_slide` PRISMATIC axis(1,0,0); helper `_pdw_wire_rail()` | compact skeletonized twin-wire stock with shoulder pad; clearance-bore collar slides on buffer tube, 0.09 m travel | converged (workbench, rating pending sync) |

### Slot B:handguard-surface (forearm attachment surface, part `handguard`, joint `receiver_to_handguard` FIXED)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| quad_picatinny | rec_model-an-m4-style-military-carbine-rifle-all-mat_20260610_081537_124088_497c83bd | `handguard_tube` + `hg_rail_top`/`hg_rail_bottom`/`hg_rail_left`/`hg_rail_right` (hg_xs loop len 11) | four full Picatinny rib rails on all four faces; bottom rail skips foregrip clamp zone `if not (0.178<x<0.222)` | converged (parent) |
| smooth_tube | rec_rifle_var_smoothtube | `handguard_tube` only (no rail elements; hg_xs loop removed) | featureless smooth round free-float tube, 0 slots / 0 rails | converged (workbench, rating pending sync) |
| mlok | rec_rifle_var_mlokrail | `handguard_tube` + `mlok_slot_0..11` + retained `hg_rail_top` (hg_rib_xs loop len 11, top rail only) | round tube with 12 M-LOK oval slots (4/row × 3 rows at 90/-90/180°); only top Picatinny rail survives | converged (workbench, rating pending sync) |
| keymod | rec_rifle_var_keymod | `handguard_tube` + `keymod_slot_0..18` + `hg_rail_top_base`/`hg_rail_top_ribs` (hg_rib_xs loop len 11); helper `_keymod_slot()` | rounded-rect tube with 19 KeyMod keyhole slots (7/row × 3 rows, bottom row skips foregrip zone); only top rail survives | converged (workbench, rating pending sync) |

### Slot C:optic (sighting system, mounts off receiver flat-top rail / handguard)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| red_dot | rec_model-an-m4-style-military-carbine-rifle-all-mat_20260610_081537_124088_497c83bd | part `reflex_sight`(`optic_mount`/`optic_body`/`optic_hood`/`optic_rear_housing`/`optic_knob`) / joint `receiver_to_reflex_sight` FIXED | compact red-dot reflex sight clamped on the receiver top rail | converged (parent) |
| scope | rec_rifle_var_scope | part `scope`(`scope_body`/`ring_mount_0`/`ring_mount_1`/`elevation_turret`/`windage_turret`) / joint `receiver_to_scope` FIXED | long magnified telescope: fused tube + objective bell + eyepiece carried in twin ring mounts, knurled elevation/windage KnobGeometry turrets | converged (workbench, rating pending sync) |
| iron_sights | rec_rifle_var_ironsights | parts `rear_sight`(`rear_sight_frame`/`rear_grip_0..1`) + `front_sight`(`front_sight_body`/`front_knob_0..1`) / joints `rear_sight_flip` REVOLUTE axis(0,-1,0) on receiver + `front_sight_flip` REVOLUTE axis(0,-1,0) on handguard (both ~90°) | flip-up BUIS pair: rear aperture peep + front post; no reflex_sight; adds 2 revolute flip joints (7 nonfixed total) | converged (workbench, rating pending sync) |

### Slot D:muzzle (muzzle device, added as visual on part `barrel`, joint `receiver_to_barrel` FIXED)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| birdcage | rec_model-an-m4-style-military-carbine-rifle-all-mat_20260610_081537_124088_497c83bd | barrel visual `flash_hider` | slotted birdcage flash hider on the barrel muzzle | converged (parent) |
| suppressor | rec_rifle_var_suppressor | barrel visual `suppressor` (replaces flash_hider); helper `_suppressor_ring()` | long ~0.19 m sound-suppressor can over the muzzle: collar + smooth tube + endcap + internal baffles; no new joint | converged (workbench, rating pending sync) |
| brake | rec_rifle_var_brake | barrel visual `muzzle_brake` (replaces flash_hider); helper `_muzzle_brake_body()` | ~0.054 m ported compensator brake: through-bore body + thread collar + 6 side ports/baffle cuts; no new joint | converged (workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: top-rail Picatinny **slot count** (the `hg_xs`/`hg_rib_xs` rib loop length N on the handguard top rail)
- N 样本已覆盖: {7, 11, 18} → rec_rifle_var_rails7 / rec_model-an-m4-style-military-carbine-rifle-all-mat_20260610_081537_124088_497c83bd (parent, N=11) / rec_rifle_var_rails18
- 模板建议 N_range: [6, 30]
- copied object / naming / placement / joint policy:
  - **copied object**: one Picatinny rail rib (a `_box_compound` box, top/bottom `(0.0075,0.034,0.0075)` left/right `(0.0075,0.0075,0.034)`; rails18 shrinks ribs to `(0.005,0.034,0.006)` for visual distinctness)
  - **naming**: ribs are bundled into one compound mesh per face (`hg_rail_top`/`hg_rail_bottom`/`hg_rail_left`/`hg_rail_right`), NOT one named element per rib — the loop only varies the `hg_xs` x-centers list `[0.112 + i*(0.178/(N-1)) for i in range(N)]`
  - **placement**: equidistant along bore +X, fixed start x=0.112 and fixed first-to-last span 0.178 (step = 0.178/(N-1)); bottom-rail copies skip the foregrip clamp gap `if not (0.178<x<0.222)`
  - **joint policy**: the whole `handguard` part (and thus all rib copies) is rigidly FIXED to the receiver via `receiver_to_handguard`; ribs themselves carry no joints

## 排除项(未来 compatibility matrix 素材)
- magazine straight↔curve geometry (mag_body cant segments) is a continuous parameter, NOT a discrete slot/multiplicity candidate — do not enumerate as variants
- bullpup layout (magazine/action behind the trigger, reversed receiver topology) — out of `rifle` 小类 identity, breaks the receiver/buffer-tube/magwell spine
- belt-fed / box-feed LMG layouts (feed tray + belt instead of magwell magazine) — out of category
