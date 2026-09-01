# Military / knife — template source map

> NOTE: 小类 is labeled `knife`, but the parent asset and every variant are a **KATANA DISPLAY SET** (slug `katana`) — a multi-sword display rack, not a single knife. The articulated unit is a sheathed katana (saya + blade assembly) that draws out of its scabbard; the rack is the structural spine. Treat the spec slots below as "katana display set" slots.

pattern: mixed
parents: rec_model-a-decorative-japanese-katana-display-set-a_20260610_080556_400208_ac7923b8 ← picture/Military/knife/001.png (3-sword display set; covers Slot A stand-style=tiered_crescent_rack, Slot B tsuba=round_disc, Slot C tsuka-wrap=pink_diamond_ito; per-sword FIXED saya mount + PRISMATIC blade draw; sword multiplicity {3} via a hand-tuned 3-key SWORD_MOUNTS dict, NOT a clean loop)

## Slot 候选覆盖

### Slot A:stand_style(承架 / 母体根件)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| tiered_crescent_rack | rec_model-a-decorative-japanese-katana-display-set-a_20260610_080556_400208_ac7923b8 | part `display_stand`; elems `base_plinth`/`base_body`/`base_cap`, `pillar_0`/`pillar_1` (mesh `_pillar_shape`), `plaque_panel`+`frame_*`+`kanji_stroke_*`; joints `{prefix}_saya_mount`(FIXED) | base box + two black-lacquer pillar planks with twin forward crescent cradle notches (two tiers) + 3rd sword on box top; gold-framed kanji plaque on -Y face | converged (parent) |
| vertical_post | rec_katana_var_vpost | part `display_stand`; elems `post_{i}` (mesh `_post_shape`, U-channel: back panel + side walls + foot); joint `saya_mount_{i}`(FIXED) with `rpy=(0,PITCH,0)` 2° lean | three upright U-channel posts on the base box hold near-vertical sheathed katanas (PITCH=-(π/2-LEAN)); MOUTH_Z=0.87 mount | converged (workbench, rating pending sync) |
| wall_mount | rec_katana_var_wallmount | part `wall_panel` (root, NO base box); elems `panel_body`, `hook_{0..5}` (mesh `_hook_shape`, L-bracket + cradle notch + rib), `frame_*`/`plaque_*`; joint `saya_mount_{i}`(FIXED) | flat vertical wall board, 6 L-bracket hooks in 3 tiers, swords float above ground (panel bottom z≈0.08) | converged (workbench, rating pending sync) |

### Slot B:tsuba(刀镡 / 护手盘,blade-assembly 上)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_disc | rec_model-a-decorative-japanese-katana-display-set-a_20260610_080556_400208_ac7923b8 | `tsuba_disc` (Cylinder r=0.041, rotated π/2 about Y) + `tsuba_rim` (mesh `_ring_shape`) in `_build_blade_assembly` | plain round red disc guard with gold annular rim, diameter ~0.082 m > 2.2·SAYA_R | converged (parent) |
| square_mokko | rec_katana_var_tsubasq | `tsuba_plate` (mesh `_mokko_tsuba_shape`) + `tsuba_rim` (mesh `_mokko_tsuba_rim_shape`) | squared mokkō-gata (four-lobe rounded-square) guard plate + matching square rim, replacing the round disc | converged (workbench, rating pending sync) |
| pierced_sukashi | rec_katana_var_tsubapierced | `tsuba_disc` (mesh `_tsuba_sukashi_shape`) + `tsuba_rim` | openwork sukashi guard: round plate boolean-cut with central nakago-ana tang slot, four 45° radial petal slots, four cardinal circular holes | converged (workbench, rating pending sync) |

### Slot C:tsuka_wrap(柄卷 / 握把缠绕,blade-assembly 上)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pink_diamond_ito | rec_model-a-decorative-japanese-katana-display-set-a_20260610_080556_400208_ac7923b8 | `tsuka_grip` (pink Cylinder) + `wrap_diamond_top_{0..2}` + `wrap_diamond_front_{0..1}` (rotated dark Box accents) | pink grip with dark diamond-wrap (ito) lozenge accents on top and front faces | converged (parent) |
| smooth_samegawa | rec_katana_var_gripsmooth | `tsuka_grip` (pink Cylinder); NO wrap_diamond_* elements | bare smooth samegawa-style grip cylinder, all diamond-wrap accents removed | converged (workbench, rating pending sync) |
| cord_ring_bands | rec_katana_var_gripcord | `tsuka_grip` + `grip_band_{i}` (mesh `grip_band_mesh`, count `GRIP_BAND_COUNT`, evenly spaced along grip span) | pink grip wrapped with evenly spaced dark cord ring bands instead of diamonds | converged (workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: sword count (parent: implicit 3-key `SWORD_MOUNTS` dict keyed `top`/`middle`/`bottom`; clean variants: `N_SWORDS` int with indexed `TIER_X`/`TIER_Y`/`TIER_Z` arrays)
- N 样本已覆盖: {1, 2, 3} → rec_katana_var_n1 {1} / rec_katana_var_n2 {2} / rec_katana_var_n3loop {3, loop form} ; N=3 also covered by the parent {3, hand-tuned-dict form}
- 模板建议 N_range: [1, 5]
- copied object / naming / placement / joint policy:
  - copied object: one full sheathed katana = `{prefix|sword_i}_saya` part (built by `_build_saya`) + `{prefix|sword_i}_blade_assembly` part (built by `_build_blade_assembly`); saya/blade meshes are shared (built once, reused).
  - naming: parent uses hand-tuned string prefixes (`top_saya`, `middle_saya`, `bottom_saya` + `*_blade_assembly`); clean variants use index naming `sword_{i}_saya` / `sword_{i}_blade_assembly`.
  - placement: parent reads literal per-key tuples from the hand-tuned 3-key `SWORD_MOUNTS` dict (`top`/`middle` in pillar cradles, `bottom` on box top — irregular, not a stride). var_n3loop **rewrites that dict into the loop form**: index arrays `TIER_X=(0.17,0.28,0.28)`, `TIER_Y=(-0.06,CRADLE_Y,CRADLE_Y)`, `TIER_Z=(box-top seat, lower cradle, upper cradle)` read by helper `_sword_mount_xyz(i)`, emitted by a single `for i in range(N_SWORDS)` loop calling helper `_mount_sword(...)`.
  - joint policy (uniform per sword): `{...}_saya_mount` = **FIXED** (stand/panel → saya, at the mount xyz); `{...}_blade_draw` = **PRISMATIC** (saya → blade_assembly, axis (1,0,0), origin (0,0,0), MotionLimits lower=0 upper=TRAVEL=0.70, effort=25, vel=0.6). Each sword is fully independent (its own FIXED mount + own PRISMATIC draw).

## 排除项(未来 compatibility matrix 素材)
- **blade-drawn vs sheathed = pose, NOT an axis.** Drawn (`blade_draw` at q=0.35/0.70) vs fully sheathed (q=0) is a joint-pose state space exercised by tests, not a template slot — do not enumerate as a candidate.
- **color/material = not an axis.** Saya/grip/tsuba colorways (pink ito, deep-pink bands, red tsuba, gold fittings) are reskins; not a structural slot in this map.
- **slot combinations = combos sampler's job.** This map covers each slot's candidates independently (one varied slot per variant); cross-products (e.g. wall_mount × pierced_sukashi × cord_ring_bands) are left to the downstream combinations sampler, not pre-enumerated here.
