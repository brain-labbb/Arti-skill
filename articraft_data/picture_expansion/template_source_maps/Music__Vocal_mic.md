# Music / Vocal_mic — template source map

pattern: mixed
parents:
  - rec_blue-usb-condenser-vocal-microphone-yeti-style-w_20260605_161854_747260_91210c09 ← picture/Music/Vocal_mic/002.png (covers: USB-condenser body, ball head, shock cradle, control-knob multiplicity)
  - rec_vintage-silver-desktop-vocal-microphone-with-a-r_20260605_161846_030934_4ebe60a8 ← picture/Music/Vocal_mic/001.png (covers: vintage desktop body, cyl head, tripod stand)

## Slot 候选覆盖

### Slot A: head_form
The mic capsule/windscreen form. Two families: a separate spherical/ball grille seated on a tapered body (parent A reskin), and the in-yoke capsule head whose own grille shape varies (parent B reskin).

| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| grille_band_dome (A baseline) | rec_blue-usb-condenser-vocal-microphone-yeti-style-w_20260605_161854_747260_91210c09 | part `body`; visuals `body_shell`/`grille_band`(`_mesh_grille_band`)/`dome_cap`(`_dome_cap`); joint `base_to_body` | [A] vertical ribbed mesh-grille band on the upper body tube + domed cap; head is integral to the tilting `body` (no separate head part) | converged(已同步) |
| ball_grille_head | rec_vocal_microphone_var_head_ball | part `body`; visuals `body_shell`(`LatheGeometry` tapered)/`head_inner`(`_sphere_grille_inner`)/`head_ribs`(`_sphere_grille_ribs`, `_meridian_path`/`_parallel_path`/`_rib_tube`)/`head_collar`(`_head_collar`); joint `base_to_body` | [A] round spherical wire-cage windscreen (dark inner sphere + meridian/parallel chrome ribs + equator ring) seated on tapered neck via collar; merged into `body` part | converged(已同步) |
| oval_ribbed_capsule (B baseline) | rec_vintage-silver-desktop-vocal-microphone-with-a-r_20260605_161846_030934_4ebe60a8 | part `capsule_head`; visuals `capsule_shell`(`_capsule_mesh`/`_loft_yz`)/`grille_interior`(`_grille_interior_mesh`)/`badge`/`tilt_pin`; joint `yoke_to_capsule` | [B] flat oval/teardrop Shure-55 grille head; horizontal slat slots cut through +X face; dark inner block; pinned in yoke (separate `capsule_head` part) | converged(已同步) |
| cyl_basket_head | rec_vocal_microphone_var_head_cyl_on_vintage | part `capsule_head`; visuals `basket_shell`(`_basket_shell_mesh`/`_basket_rib`)/`basket_dome`(`_basket_dome_mesh`)/`basket_interior`(`_basket_interior_mesh`)/`tilt_pin`; joint `yoke_to_capsule` | [B] tall upright cylindrical mesh basket (16 vertical ribs + top/bottom retaining bands + domed cap + dark interior cylinder); pinned in yoke | converged(已同步) |

### Slot B: mount_stand
The body+stand family AND its real articulation. Parent A = round weighted base + Y-fork yoke that tilts the body (REVOLUTE about +Y); parent B = round weighted base + vertical swivel post carrying a U-yoke that tilts the capsule (CONTINUOUS swivel + REVOLUTE tilt). Variants swap in a shock-mount cradle (A family) and a folding tripod (B family).

| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| y_fork_yoke (A baseline) | rec_blue-usb-condenser-vocal-microphone-yeti-style-w_20260605_161854_747260_91210c09 | parts `base`(`base_disc`+`fork_arm_pos`/`fork_arm_neg` via `_fork_arm_mesh`+`_base_disc`), `body`(`pivot_boss_pos`/`pivot_boss_neg` via `_pivot_boss`); joint `base_to_body` (REVOLUTE +Y, ±45°) | [A] round weighted disc + two curved Y-fork arms with pivot knuckles capturing body trunnion bosses; body tilts fore/aft about +Y | converged(已同步) |
| swivel_post_uyoke (B baseline) | rec_vintage-silver-desktop-vocal-microphone-with-a-r_20260605_161846_030934_4ebe60a8 | parts `base_disc`(`_base_mesh`), `swivel_post`(`post_shell`/`yoke_shell` via `_post_mesh`/`_yoke_mesh`), `cable`(`cable_shell`/`xlr_tip`); joints `base_to_post`(CONTINUOUS +Z), `yoke_to_capsule`(REVOLUTE +Y ±45°), `base_to_cable`(FIXED) | [B] round weighted disc + tapered swivel post + U-yoke bridge/arms; vertical swivel under capsule tilt; drooping XLR cable + plug fixed to base | converged(已同步) |
| shock_mount_cradle | rec_vocal_microphone_var_mount_shockcradle | parts `base`(`base_disc`/`support_arm`(`_support_arm_mesh`)/`outer_ring`(`_outer_ring_mesh`)/`cradle_ring`(`_cradle_ring_mesh`)/`band_0..7`(`_elastic_band`, `N_BANDS=8`)/`pivot_axle`(`_pivot_axle_mesh`)), `body`(`pivot_hub`(`_pivot_hub_mesh`)); joint `base_to_body` (REVOLUTE +Y, ±45°) | [A] **STATIC** shock-mount built on the base: rear curved support arm → outer support ring; inner cradle ring suspended by 8 elastic bands running outer→cradle (one connected static assembly, all at cradle height z=RING_Z); central pivot rod across the cradle; the mic body hangs INSIDE the static cradle and tilts about +Y on the pivot rod (captured pin via body `pivot_hub`). Corrected from the prior broken fork whose cradle+bands were welded to the tilting body and collapsed to z≈0. | converged(已同步) |
| folding_tripod | rec_vocal_microphone_var_mount_tripod | parts `tripod_hub`(`hub_shell`+`leg_0..2`(`_leg_strut_mesh`)+`foot_0..2`(`_foot_mesh`), `N_LEGS=3`), `swivel_post`(`post_shell`/`yoke_shell`), `capsule_head`, `cable`; joints `hub_to_post`(CONTINUOUS +Z), `yoke_to_capsule`(REVOLUTE +Y ±45°), `hub_to_cable`(FIXED) | [B] base disc replaced by central hub + 3 splayed tapered legs with rubber feet (120° spacing); post swivels on hub, capsule tilts in yoke; cable fixed to hub | converged(已同步) |

## Multiplicity / Copy Logic
- count_param: control_knob_count ;
- N 样本已覆盖: {2, 3, 4} → parent A(=2: `volume_knob`+`gain_knob`, separate front/side axes) / rec_vocal_microphone_var_controls_n3 (`knob_0`/`knob_1`/`knob_2`, `KNOB_NAMES`, `N_FRONT_KNOBS=3`) / rec_vocal_microphone_var_controls_n4 (`front_knob_0..3`, `KNOB_COUNT=4`, `KNOB_ZS`)
- 模板建议 N_range: [2, 4]
- copied object: one front rotary knob = part + 2 visuals (`_front_knob_mesh` knob body + `_front_knob_marker` off-center pointer marker) + 1 CONTINUOUS joint
- naming: parent A uses semantic names (`volume_knob`/`gain_knob` + `*_marker`); n3 uses `knob_i`/`marker_i`; n4 uses `front_knob_i`/`front_marker_i` — template should pick ONE scheme (e.g. `front_knob_{i}`/`front_marker_{i}`) for all N
- placement: vertical column on the +X front face at `x=BODY_R-0.001`, evenly spaced Z centers (`KNOB_Z_GROUND`/`KNOB_ZS`) in the lower body below the grille band; N=4 lengthens the body tube (`BODY_TUBE_H` 0.110→0.160) to fit the taller column
- joint policy: each knob `body_to_<name>` CONTINUOUS about +X (axis (1,0,0)), effort 0.3 / velocity 8.0; knob built along local +Z (`center=False`) and rotated `rpy=(0, π/2, 0)` so its face points +X; allow_overlap knob_i↔body_shell (seated against front wall). Note A's side `gain_knob` is the lone +Y exception — collapse to all-front-column for the multiplicity slot.

## 排除项
- Cross-parent combos not sampled: ball/cyl head on the OTHER body family (e.g. ball head on vintage swivel post, or oval/cyl head on the USB Y-fork body); each head form was only forked from its own parent.
- shock-mount cradle only sampled on parent A; folding tripod only on parent B. No cradle-on-vintage or tripod-on-USB-body.
- control_knob_count multiplicity only sampled on parent A (USB-condenser front face). Parent B's vintage/tripod families carry no front knob column (controls = single front `badge` dot only), so N is gated to the A body family.
