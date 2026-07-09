# Door / Garage shutter — template source map

pattern: single-parent fan-out (multiplicity + named slots)
slug: garage_shutter
parents:
- rec_door_garage_shutter ← picture/Door/Garage shutter/001.png (residential sectional telescoping garage door: black steel surround frame [jamb_0/jamb_1, header, threshold, guide_track_0/1], stack of 6 grey embossed horizontal panel leaves slat_0..slat_5, depth-staggered face-to-face nesting, slat_0 FIXED to frame via frame_to_slat_0, each lower leaf PRISMATIC up one pitch via slat_{i-1}_to_slat_{i}, latch_handle on bottom leaf). Covers Slot N=6, Slot TYPE=sectional_telescoping, Slot WINDOW=no_windows, Slot SURFACE=flat_embossed_pillow, Slot FRAME=slim_surround.

Garage shutter family with a horizontal-panel curtain. Variants isolate panel multiplicity N, the
curtain lift TYPE/kinematics, the presence of a top window row, the per-panel surface treatment, and
the surround/guide-rail FRAME style, while retaining at least one real non-fixed joint (telescoping
prismatic chain or tilt-up revolute pivot). Color/material never counts as the change.

Loop-emission status: PARENT ALREADY LOOP-EMITTED. The leaves are built in `for i in range(N_SLATS)`
with `slat_{i}` naming, a shared per-leaf visual block (panel_field, panel_pillow, seam_groove),
regular vertical band stacking via `_band_center_z(i)`/`_leaf_center_y(i)`, and a uniform prismatic
joint policy in `for i in range(1, N_SLATS)`. Multiplicity variants only retune `N_SLATS` (and pitch);
no hand-written-leaf rewrite is required.

## Slot 候选覆盖

### Slot N: panel/slat multiplicity
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| six_panels | rec_door_garage_shutter (parent) | slat_0..slat_5; frame_to_slat_0; slat_{i-1}_to_slat_{i} | parent 6-leaf telescoping stack | converged(parent) |
| three_panels | rec_garage_shutter_var_n3 | slat_0..slat_2; slat_{i-1}_to_slat_{i} | 3 taller leaves (SLAT_PITCH 0.71), same opening | converged(已同步) |
| ten_slats | rec_garage_shutter_var_n10 | slat_0..slat_9; slat_{i-1}_to_slat_{i} | 10 narrower roller-style slats (SLAT_PITCH 0.213), same opening | converged(已同步) |
| single_slab (tilt-up) | rec_garage_shutter_var_tiltup_slab | door_leaf; panel_pillow_{i}/seam_groove_{i} (N_ROWS=6) | one monolithic leaf, embossing as 6 inlaid rows (N collapses to 1 part) | converged(已同步, also TYPE slot) |

### Slot TYPE: curtain lift kinematics
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| sectional_telescoping | rec_door_garage_shutter (parent) | slat_{i-1}_to_slat_{i} (+Z PRISMATIC chain) | rigid leaves telescope/nest behind fixed top leaf | converged(parent) |
| single_tilt_up | rec_garage_shutter_var_tiltup_slab | door_leaf; frame_to_leaf (REVOLUTE, axis -X, hinge at header) | one rigid canopy slab tilting up/out on a single top pivot | converged(已同步) |

### Slot WINDOW: top window row
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| no_windows | rec_door_garage_shutter (parent) | solid panel_field per leaf | all leaves solid embossed | converged(parent) |
| divided_lite_top_row | rec_garage_shutter_var_window_row | top leaf slat_0: window_glass_{j}, mullion_{j} (N_WINDOWS=5), top_rail/bottom_rail | top leaf glazed into 5 divided-lite recessed windows with mullions + rails | converged(已同步) |

### Slot SURFACE: panel face treatment
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_embossed_pillow | rec_door_garage_shutter (parent) | panel_pillow per leaf | single raised flat pillow per leaf | converged(parent) |
| vertical_ribbed (horizontal corrugation) | rec_garage_shutter_var_ribbed | rib_{r} loop per leaf (n_ribs=5) | several thin parallel horizontal corrugation ribs proud of each leaf face | converged(已同步) |
| raised_panel_field | rec_garage_shutter_var_sectional_panels | section_{i}: panel_body + raised_field (recessed border / proud center) | classic raised-panel embossing: recessed stile/rail border around a proud central field | converged(已同步) |
| perforated_grille | rec_garage_shutter_var_grille | per-leaf grille_panel (ExtrudeWithHolesGeometry grid of square through-cuts + solid border, mesh_from_geometry) + full-depth edge_rail_0/edge_rail_1 | each leaf is a thin open security-grille sheet: regular grid of square through-cut holes inside a solid border, flanked by structural edge rails; retains parent N=6 sectional_telescoping/no_windows/slim_surround | converged(已同步) |

### Slot FRAME: surround / guide-rail style
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| slim_surround | rec_door_garage_shutter (parent) | jamb_0/jamb_1, header, threshold, guide_track_0/1 (flat boxes) | slim flat black surround with flush galvanized guide tracks | converged(parent) |
| bold_square_tube_rails | rec_garage_shutter_var_box_rails | guide_rail_{i} (ExtrudeWithHolesGeometry hollow square tube, mesh_from_geometry), header beam, sill | bold proud 90 mm hollow square-tube guide rails with visible track channels | converged(已同步) |

## Multiplicity / Copy Logic
- count_param: `N_SLATS` (panels/slats). Secondary inner-loop counts: `N_WINDOWS` (top-leaf divided lites), `n_ribs` (horizontal corrugation ribs per leaf), `N_ROWS` (tilt-up slab embossing rows), and for the grille SURFACE the per-leaf hole-grid `n_cols`/`n_rows` (derived from GRILLE_PITCH).
- N 样本已覆盖: N_SLATS {1 (tiltup_slab single leaf), 3 → rec_garage_shutter_var_n3, 6 (parent + grille), 10 → rec_garage_shutter_var_n10}; window lites {5}; ribs {5}.
- 模板建议 N_range: [3, 14] for the horizontal-panel stack (near-square opening, pitch = OPENING_H / N); the single_tilt_up TYPE collapses N to one slab.
- copied object / naming / placement / joint policy: leaves loop-emitted as `slat_{i}` (top FIXED, rest uniform +Z PRISMATIC one-pitch-per-link); the sectional_panels variant renames to `section_{i}` with the same telescoping policy; the grille variant keeps `slat_{i}` but swaps each leaf's face for a perforated `grille_panel` mesh plus `edge_rail_0/1` rails (same telescoping policy, same N=6); tilt-up emits embossing as inlaid `panel_pillow_{i}`/`seam_groove_{i}` on one `door_leaf` driven by one REVOLUTE pivot; window lites `window_glass_{j}`/`mullion_{j}` and ribs `rib_{r}` are inlaid parent visuals (no per-decoration FIXED joints); box_rails emits hollow square-tube `guide_rail_{i}` via ExtrudeWithHolesGeometry + mesh_from_geometry.

## 组合数预审 (GATE P1)
Counted from ON-DISK CONVERGED variants only (every id in the converged set is mapped).
Slot TYPE(2) × Slot WINDOW(2) × Slot SURFACE(4) × Slot FRAME(2) × distinct N(4) = 128 ≥ 10 ✓.
Minimal independent read: SURFACE(4) × distinct-N(4) = 16 ≥ 10 — already met by existing variants; the TYPE/WINDOW/FRAME axes push further. distinct N covered = {1, 3, 6, 10} (4 distinct, satisfying the 2–3 distinct-N requirement).
Per-slot candidate counts: N=4, TYPE=2, WINDOW=2, SURFACE=4, FRAME=2 — every slot ≥2 candidates ✓.

GATE P1 met: YES. No single-candidate slot remains; no gap fork required.

## 排除项(未来 compatibility matrix 素材)
- single_tilt_up curtain TYPE replaces the per-leaf telescoping prismatic chain; the parent telescoping nesting policy does not co-apply (different kinematics), but it keeps its own real non-fixed joint (tilt-up revolute pivot).
- divided_lite_top_row consumes the top FIXED leaf (slat_0) as a glazed panel; on the single_tilt_up TYPE the window row would inlay into the slab's top band instead of a separate top leaf — reframed/compatible at best.
- vertical_ribbed, raised_panel_field, and perforated_grille surfaces all replace the flat pillow on the panel-stack TYPEs; on single_tilt_up they reframe onto the one slab face. The perforated_grille SURFACE additionally introduces full-depth structural edge rails (edge_rail_0/1) as the face-to-face telescoping contact chain in place of solid panel laps.
- bold_square_tube_rails (FRAME slot) is orthogonal to TYPE/SURFACE/WINDOW: it swaps the surround and guide geometry only and co-applies with any panel-stack TYPE.
- roller_coiling TYPE (continuous slats coiling onto a top barrel drum on one driving REVOLUTE) was scoped as a possible TYPE enrichment but is NOT on disk / NOT converged, so it is excluded from the candidate counts above. The TYPE slot already holds ≥2 converged candidates and the gate is met without it; it is recorded here only as future compatibility material.
- No blocked cells; all mapped variants converged-by-design from the loop-emitted parent. No duplicate cells.
