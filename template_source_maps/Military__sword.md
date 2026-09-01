# Military / sword — template source map

pattern: mixed
parents: rec_model-a-roman-gladius-style-short-sword-sheathed_20260610_080621_578517_42984874 ← picture/Military/sword/001.png (gladius + ornate scabbard; fills Slot A=gladius_leaf, Slot B=box_guard+bead_pommel, Multiplicity N=2 hand-paired front/rear suspension rings; structural core = `sword_draw` PRISMATIC + `front_ring_pivot`/`rear_ring_pivot` REVOLUTE = 3 nonfixed joints)

## Slot 候选覆盖

### Slot A:blade-profile
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| gladius_leaf | rec_model-a-roman-gladius-style-short-sword-sheathed_20260610_080621_578517_42984874 | `sword` part / `_blade_solid` / elem `blade` + `blade_spine` | leaf-shaped double-edged blade, polyline tapers from `BLADE_HALF_W_BASE`=0.023 through `BLADE_HALF_W_SHOULDER`=0.015 to point; flat extrude | converged (parent) |
| straight_double_edge | rec_sword_var_straightblade | `_blade_solid` / elem `blade` | arming-sword: parallel edges of uniform `BLADE_HALF_W`=0.020, taper only past `BLADE_TAPER_X`=−0.39 to tip | converged (workbench, rating pending sync) |
| curved_saber | rec_sword_var_saber | `_curved_blade_solid` / `_bow`+`_sword_bow`+`_scabbard_bow` / `_offset_loft` / `_curved_sections` / elem `blade` | single-edged parabolic-bowed blade (`BOW_MAX`), edge on +Y / spine on −Y; scabbard cavity bowed by `_scabbard_bow` to match | converged (workbench, rating pending sync) |
| broad_triangular | rec_sword_var_broadsword | `_blade_solid` / `_blade_spine_solid` / elem `blade` + `blade_spine` | broad triangular: full width ~0.076 at guard, single straight taper to point; raised central triangular ridge spine | converged (workbench, rating pending sync) |

### Slot B:hilt (guard + pommel)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| box_guard + bead_pommel | rec_model-a-roman-gladius-style-short-sword-sheathed_20260610_080621_578517_42984874 | elem `guard` + `guard_relief_{i}` / `grip_core`/`grip_lower`/`grip_upper`/`grip_collar_bead_{i}` / `_pommel_solid` elem `pommel` / `finial_stem`+`finial_ball` | rectangular box guard with relief panels; oblate amber ellipsoid pommel; gold spiral collar beads; brass ball finial | converged (parent) |
| cruciform + disc_pommel | rec_sword_var_crossguard_disc | `_crossguard_solid` elem `guard` + `guard_groove_{i}`+`guard_tip_{i}` / `pommel_collar` / `_disc_pommel_solid` elem `pommel` | wide straight cruciform crossguard (bar + center block, quillon tips); flat round wheel/disc pommel with central boss + brass peening collar | converged (workbench, rating pending sync) |
| knuckle_guard + scent_stopper | rec_sword_var_basket_scentstop | elem `crossguard`+`crossguard_relief_{i}` / `tube_from_spline_points(KNUCKLE_POINTS)` elem `knuckle_guard` / `_scent_stopper_pommel` elem `pommel` | box crossguard plus swept brass knuckle-guard tube arcing over the grip; tall faceted octagonal scent-stopper pommel + finial | converged (workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: suspension band/ring count (parent uses hand-paired `front`/`rear`; loop variants use `N_BANDS` / `NUM_RINGS`)
- N 样本已覆盖: {2, 4, 6} → rec_model-a-roman-gladius-style-short-sword-sheathed_20260610_080621_578517_42984874 (N=2) / rec_sword_var_bands4 (N=4) / rec_sword_var_bands6 (N=6)
- 模板建议 N_range: [1, 8]
- copied object / naming / placement / joint policy: copied unit = one suspension-ring station = lug (flange + pin + retaining head, optional band wrap) on the scabbard + a torus `ring` child part. Parent hand-codes a front/rear 2-tuple (`{tag}_ring_flange/pin/head`, parts `front_suspension_ring`/`rear_suspension_ring`, joints `front_ring_pivot`/`rear_ring_pivot`). bands4 rewrites this into a `for i in range(N_BANDS)` loop over `BAND_XS` with `_lug_positions` helper → parts `band_{i}_ring`, lugs `band_{i}_flange/pin/head` + `band_{i}_wrap`, joints `band_{i}_pivot`; bands6 uses `_ring_positions()` (alternating +Y/−Y sides) → parts `ring_{i}`, joints `ring_{i}_pivot`. Placement = regular x-spacing along the scabbard body (clear of chape and throat band). Joint policy = uniform: each ring is REVOLUTE about +Y (`SWING_AXIS_Y`=(0,1,0)), `±RING_LIMIT`=60°, hung `RING_HANG` below the pin so the off-axis swing is verifiable. `sword_draw` PRISMATIC along +X (lower=0, upper=0.50) is retained in ALL parent + variant models (verified: every model carries exactly one `prismatic`).

## 排除项(未来 compatibility matrix 素材)
- curved_saber (Slot A) × bowed scabbard cavity: WATCH — curved blade + curved cavity raises blade↔cavity interference risk along the bow; saber keeps `_scabbard_bow` matched to `_sword_bow`, but blade-profile × scabbard-cavity is the key compatibility-matrix axis to validate when combining Slot A choices with the hollow-scabbard nesting tests.
- color / scale are not articulation axes (material rgba and overall length held fixed across variants; not enumerated as slots).
