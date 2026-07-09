# Urban_Environment / Roof antena — template source map

slug: roof_antenna
shard: Roof_antena
picdir: picture/Urban Environment/Roof antena
pattern: mixed (axis slots: antenna_type + element_count_N + mast_mount + boom_config)

parents:
- rec_rooftop-yagi-tv-antenna-on-a-tall-vertical-mast-_20260608_164527_444902_495f9ccb ← picture/Urban Environment/Roof antena/001.png (weathered rooftop Yagi TV antenna; parts: mast + antenna_head + yagi_boom; 2 REVOLUTE = azimuth about +Z primary + elevation tilt about Y secondary; helper _rod via tube_from_spline_points; **single straight boom + flat-base mast + ~9-element director array + rear reflector grid baseline**)

## Identity (must persist in every variant)
Rooftop antenna on a tall weathered vertical mast standing on the roof at z=0, a head/array assembly at the mast top, and a base mount. Mast azimuth swivel (REVOLUTE about +Z) is the real primary joint; head/boom elevation tilt (REVOLUTE about Y) is the secondary joint. Variants stay rooftop antennas; never leave the category.

## 组合数预审 (HARD GATE)
antenna_type 4 (yagi/dish/dipole_whip/panel) × mast_mount 4 (flat_base/tripod/chimney_strap/wall_bracket) × boom_config 2 (single/X_dual) × distinct-N 3 (~5 / ~9 / ~14) = **96 ≥ 10 ✓**
(non-N candidate product alone 4 × 4 × 2 = 32 ≥ 10 ✓)

## Slot 候选覆盖
### Slot A: antenna_type (head array form — primary structure; azimuth+elevation REVOLUTE kept)
| 候选 | record_id | 关键 part/joint | 状态 |
|---|---|---|---|
| yagi_director_array (基线) | P_roof_antenna | boom + element_{i} rods + reflector grid | parent |
| dish_reflector | rec_roof_antenna_var_type_dish | parabolic dish (lathe/mesh) + feed horn on strut tripod | converged |
| dipole_whip | rec_roof_antenna_var_type_dipole_whip | short dipole crossbar + tall vertical whip rod (loop) | converged |
| panel | rec_roof_antenna_var_type_panel | flat upright radome panel + patch bump grid loop | converged |

### Slot B: element_count_N (director rods along the boom — multiplicity)
| 候选 | record_id | 关键 part/joint | 状态 |
|---|---|---|---|
| ~9 (基线) | P_roof_antenna | hand-written element_specs list (9 rods) | parent |
| ~5 (few) | rec_roof_antenna_var_elements_few | element_{i} loop, N≈5, computed taper | converged |
| ~14 (many) | rec_roof_antenna_var_elements_many | element_{i} loop, N≈14, computed taper | converged |

### Slot C: mast_mount (base mount)
| 候选 | record_id | 关键 part | 状态 |
|---|---|---|---|
| flat_base_mast (基线) | P_roof_antenna | round foot plate + 2 standoff brackets | parent |
| tripod_feet | rec_roof_antenna_var_mount_tripod | 3 splayed legs leg_{i} loop to pad feet | converged |
| chimney_strap | rec_roof_antenna_var_mount_chimney_strap | brick chimney stub + 2 hose straps strap_{i} loop | converged |
| wall_bracket | rec_roof_antenna_var_mount_wall_bracket | wall plate + 2 standoff arms arm_{i} loop | converged |

### Slot D: boom_config
| 候选 | record_id | 关键 part | 状态 |
|---|---|---|---|
| single_boom (基线) | P_roof_antenna | one central boom spine | parent |
| X_dual_boom | rec_roof_antenna_var_boom_xdual | two crossed booms, each w/ element_{i} loop | converged |

## Multiplicity / Copy Logic (loop notes)
- count_param: element_count N (director/dipole rods crossing the boom).
- **Parent loop status:** elements ARE loop-emitted (`for idx,(ex,elen) in enumerate(element_specs)` → `element_rod_{idx:02d}`), BUT the count is fixed by a HAND-WRITTEN `element_specs` list of 9 explicit (x,len) tuples (model.py lines 160-171). The N-multiplicity variants (Slot B) MUST request rewriting `element_specs` to be generated programmatically from a single N param via `for i in range(N)` with computed even spacing + front-tapering length, named `element_{i}`.
- Already-parametric loops in parent: reflector grid (`for g in range(n_grid)`, n_grid=17 → `reflector_grid_{g:02d}`), reflector stiles (2-loop → `reflector_stile_{s}`), standoff brackets/pads (`for i,bz` → `standoff_bracket_{i}`).
- Template suggested N_range: director elements [5, 14] (parent ~9); reflector grid rods fixed ~17; tripod legs fixed 3; straps/arms fixed 2.
- copied object: single transverse rod element; naming element_{i}; placement evenly along boom +X with tapering length; joint policy FIXED riding the boom/head (Rule1 inline as boom visuals).

## 排除项 (dropped axes)
- color/material/pure-scale — never a structural axis (suffix forbids; cosmetic only).
- reflector-grid density as its OWN axis — dropped to avoid near-duplicate of element_count_N (both are "row of parallel rods"); folded into the type/baseline geometry instead.
- guy-wire / cable accessory axis — dropped (would create disconnected thin-cable islands; low structural value).

## Variant count
9 NEW variants planned (3 type + 2 element_count + 3 mount + 1 boom). All workbench-only, single-axis diff, ≥1 non-fixed joint, azimuth+elevation REVOLUTE preserved.

## Status legend
planned = PHASE 0 axis-plan only (no fork run yet).
