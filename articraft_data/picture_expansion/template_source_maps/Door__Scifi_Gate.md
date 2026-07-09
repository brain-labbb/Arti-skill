# Door / Scifi Gate — template source map

pattern: parallel_children (named slots: opening_mechanism 主机构 + multiplicity N + leaf/seam/surface motif + frame_surround)
slug: scifi_gate · shard: Scifi_Gate · picdir: picture/Door/Scifi Gate (001.png, 002.png)

Wall-mounted sci-fi blast/security gate, true through-opening (no back wall), standing on z=0.
The doorway is cleared by a powered mechanism (lateral telescope / single-leaf pocket / vertical
lift / radial iris) over a fixed structural surround. Variants isolate the opening MECHANISM, the
moving-segment multiplicity N, the leaf seam/surface MOTIF, and the frame SURROUND style, while
always keeping at least one real non-fixed joint and a true clear through-opening. Color/material
never counts as the change; fixed greeble swaps (pistons/beacons/keypad/clamps) are NOT a counted
structural axis.

parents (fork sources):
- **rec_door_scifi_hex** ← picture/Door/Scifi Gate/002.png — hexagonal beveled armored slab.
  `_frame_mesh` cuts a chamfered-hex through-opening in a flat grey slab; bi-parting 2-stage
  horizontal telescope: inner hazard leaves `door_0`/`door_1` on a wavy complementary COSINE seam
  (`_seam_points`, `_inner_leaf_mesh`, `_hazard_stripe_mesh`) + outer grey leaves
  `door_0_outer`/`door_1_outer`, four mimic-coupled PRISMATIC slides off `door_0_slide` (the other
  three mimic at 1.0×/0.5×); twin cyan jamb strips (`light_strip_*`/`strip_backing_*`); mid-height
  clamp blocks w/ red lamps (`clamp_*`/`clamp_lamp_*`); top recess; chamfer trim; bolts.
  Baseline cell = mech:biparting_lateral_telescope × N:2_per_side × motif:hazard_cosine_seam ×
  surround:hex_beveled_slab. **Closest parent for hex-surround / iris / radial / multiplicity forks.**
- **rec_door_scifi_zigzag** ← picture/Door/Scifi Gate/001.png — lintel+sill+side-housing surround.
  Fixed `frame` (lintel_header, sill, top_guide/bottom_guide rails) + fixed side housings
  `left_housing`/`right_housing` (cowl_front/cowl_plate/rear_rail/pocket_end_wall as retract
  pockets) + wall `keypad` (keypad_box+screen+3×3 `key_r_c`+green status_lamp) + base-corner
  `hazard_left`/`hazard_right` chevron blocks; bi-parting 2-stage horizontal telescope: inner panels
  `door_0`/`door_1` on a sharp ZIGZAG dovetail seam (`_seam_points` angular knots, `_make_inner_panel`,
  `_make_inner_accent`) + outer panels `door_0_outer`/`door_1_outer`, four mimic-coupled PRISMATIC
  slides off `door_0_slide`; amber lintel warning_lamp_l/r.
  Baseline cell = mech:biparting_lateral_telescope × N:2_per_side × motif:zigzag_dovetail_seam ×
  surround:lintel_sill_side_housing. **Closest parent for housing-surround / vertical-lift / pocket
  / bulkhead forks.**

Both parents share the SAME root mechanism (bi-parting 2-stage lateral telescope, 4 mimic-coupled
PRISMATIC off `door_0_slide`); they differ only on seam MOTIF (cosine-wave vs zigzag-dovetail) and
SURROUND (flat hex slab vs lintel+sill+side-housing). The 9 converged variants below fan out the
real mechanism / multiplicity / motif / surround cells.

## Loop-emission status (source audit)

`grep -nE "for .* in (range|enumerate)"`:
- **Parents + pocket + piston_greeble + round_bulkhead: the moving leaves are HAND-WRITTEN, not
  count-loop-emitted.** They declare 4 (or, in pocket, 2) fixed named telescoping leaves
  (`door_0`/`door_1`/`door_0_outer`/`door_1_outer`) driven by explicit `model.articulation(...)`
  calls + 2-element `for part,side,tag` pair tuples. Their `for` loops are DECORATION/fixed-mount
  only (`_seam_points` sample pts, `_hazard_stripe_mesh`/`_make_hazard_stripes` bars, keypad 3×3
  grid, rivet/bolt rows, the fixed-furniture FIXED-joint loop).
- **blastslabs4 / blastslabs6 / iris6 / iris8 / leaves4 / leaves8: ALREADY LOOP-EMITTED on a real
  segment COUNT.** Each builds its moving segments in `for i in range(N_SLABS|N_PETALS|N_LEAVES)`
  with `slab_{i}` / `petal_{i}` / `leaf_{i}` naming, a shared geometry helper, regular linear/angular
  placement, and a uniform joint policy (independent staggered +Z, ±Z bi-parting, mimic REVOLUTE
  swing, or mimic radial PRISMATIC). These are the loop-rewrites the parents lack.
- **GOTCHA for any future multiplicity-retune template**: drive N off the loop-emitted variants
  (slab/petal/leaf `for i in range(N)`), NOT the parents — a multiplicity template forked from a
  parent must first delete the 4 hand-written leaves and rewrite to `segment_{i}` from one shared
  helper, exactly as the converged loop variants already do.

## 组合数预审 (GATE P1)

Slot & candidate counts (on-disk converged sources only):
- **Slot A opening_mechanism**: {biparting_lateral_telescope, single_leaf_sliding_pocket,
  vertical_lift_overhead/biparting, iris_revolute_swing, iris_radial_slide} = **5**
- **Slot B multiplicity N (distinct)**: {1, 2, 4, 6, 8} = **5 distinct N**
- **Slot C leaf/seam/surface motif**: {hazard_cosine_seam, zigzag_dovetail_seam,
  horizontal_louver_slab, radial_petal_wedge, flat_armor_plate} = **5**
- **Slot D frame_surround**: {hex_beveled_slab, lintel_sill_side_housing, circular_bulkhead_ring} = **3**

Π(Slot A × Slot C × Slot D) × distinct-N = (5 × 5 × 3) × 5 = 75 × 5 = **375 ≥ 10 ✓**.
Every slot ≥ 2 candidates ✓. distinct N = 5 (≥ 2–3 required) ✓.
Minimal independent read: Slot A(5) × distinct-N(5) = 25 ≥ 10 — already met by mechanism × N alone.
Joint topology also diversifies: lateral ±X PRISMATIC telescope vs ±Z PRISMATIC lift vs radial-spoke
PRISMATIC iris vs +Y/about-Y REVOLUTE petal swing.

**GATE P1 met by existing on-disk variants: YES.** No slot has < 2 candidates → **0 gap forks needed.**

## Slot 候选覆盖

### Slot A: opening_mechanism (主机构 — how the gate clears the doorway)
| 候选 (future module) | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| biparting_lateral_telescope (基线) | rec_door_scifi_hex / rec_door_scifi_zigzag / rec_scifi_gate_var_piston_greeble / rec_scifi_gate_var_round_bulkhead | door_0/door_1(+_outer); door_0_slide + 3 mimic PRISMATIC ±X | bi-parting 2-stage lateral telescope, captured leaves nest into side pockets | converged-parent (hex/zigzag) / converged |
| single_leaf_sliding_pocket | rec_scifi_gate_var_pocket | door_0 + door_0_outer; door_0_slide PRISMATIC +X, door_0_outer_slide mimic 0.5× | one full-width leaf slides right into an enlarged right pocket (2-stage telescope) | converged |
| vertical_lift (overhead / bi-parting) | rec_scifi_gate_var_blastslabs4 / rec_scifi_gate_var_blastslabs6 | slab_{i}; slab_{i}_slide PRISMATIC ±Z | horizontal slabs lift up into lintel (blastslabs4) or part vertically up+down into lintel/sill pockets (blastslabs6) | converged |
| iris_revolute_swing | rec_scifi_gate_var_iris6 / rec_scifi_gate_var_iris8 | petal_{i}; petal_0_joint/hinge REVOLUTE + mimic | N petals on revolute hinges swing about the rim (+Y dilate / about-+Y CCW) to clear a round aperture | converged |
| iris_radial_slide | rec_scifi_gate_var_leaves4 / rec_scifi_gate_var_leaves8 | leaf_{i}; leaf_{i}_slide PRISMATIC radial | N leaves slide straight outward along radial axes to dilate the aperture | converged |

### Slot B: multiplicity N (leaves / slabs / petals)
| 候选 | record_id | N | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|---|
| single_leaf | rec_scifi_gate_var_pocket | 1 | door_0 (+door_0_outer rear stage) | one full-width sliding leaf, 2 telescoping stages | converged |
| 2_per_side (基线) | parents + rec_scifi_gate_var_piston_greeble + rec_scifi_gate_var_round_bulkhead | 2 | door_0/door_1 + _outer pair | inner+outer telescoping pair per side | converged-parent / converged |
| 4_segment | rec_scifi_gate_var_blastslabs4 / rec_scifi_gate_var_leaves4 | 4 | slab_0..slab_3 / leaf_0..leaf_3 | 4 loop-emitted overhead louver slabs / 4 radial cross slide leaves | converged |
| 6_segment | rec_scifi_gate_var_blastslabs6 / rec_scifi_gate_var_iris6 | 6 | slab_0..slab_5 / petal_0..petal_5 | 6 vertical-parting slabs / 6 iris petals @ 60° | converged |
| 8_segment | rec_scifi_gate_var_iris8 / rec_scifi_gate_var_leaves8 | 8 | petal_0..petal_7 / leaf_0..leaf_7 | 8 iris blades @ 45° / 8 radial pie-wedge leaves @ 45° | converged |

### Slot C: leaf / seam / surface motif (face treatment + seam on the moving leaves)
| 候选 | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| hazard_cosine_seam (基线) | rec_door_scifi_hex / rec_scifi_gate_var_piston_greeble | hazard_0/hazard_1 decals + `_seam_points` cosine | yellow/black diagonal chevron decal islands + smooth cosine central seam | converged-parent / converged |
| zigzag_dovetail_seam (基线) | rec_door_scifi_zigzag / rec_scifi_gate_var_round_bulkhead | leaf_seam_accent + `_seam_points` angular knots | sharp dovetail tooth-into-notch central seam + yellow seam ribbon | converged-parent / converged |
| horizontal_louver_slab | rec_scifi_gate_var_blastslabs4 / rec_scifi_gate_var_blastslabs6 | slab_body/slab_panel + accent_line / edge_strip_top/bottom + center_rib + meeting_accent | flat gunmetal horizontal bars w/ yellow safety accent stripe (4) or edge strips + central rib + meeting accent (6) | converged |
| radial_petal_wedge | rec_scifi_gate_var_iris6 / rec_scifi_gate_var_iris8 / rec_scifi_gate_var_leaves8 | plate/blade + spine/ridge/rib + stripe (hazard arc on leaves8) | triangular/curved/pie-wedge armored petals w/ reinforcing spine/ridge; leaves8 adds hazard-yellow arc band | converged |
| flat_armor_plate | rec_scifi_gate_var_pocket / rec_scifi_gate_var_leaves4 | leaf_panel + leaf_edge_accent + groove_*/kick_plate (pocket); plate_{i} + accent_{i} + groove slots/outer rib (leaves4) | flat chamfered armor plate w/ leading-edge yellow accent + horizontal grooves / wedge plate w/ transverse groove slots + outer rib | converged |

### Slot D: frame / surround style
| 候选 | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| hex_beveled_slab (基线) | rec_door_scifi_hex / rec_scifi_gate_var_piston_greeble / rec_scifi_gate_var_iris6 / rec_scifi_gate_var_iris8 / rec_scifi_gate_var_leaves4 / rec_scifi_gate_var_leaves8 | frame_slab (`_frame_mesh` hex cut) + top_track/bottom_track + chamfer trim + bolts | flat rect slab, chamfered-hex through-opening, dark guide bars, chamfer trim, bolt studs | converged-parent / converged |
| lintel_sill_side_housing (基线) | rec_door_scifi_zigzag / rec_scifi_gate_var_pocket / rec_scifi_gate_var_blastslabs4 / rec_scifi_gate_var_blastslabs6 | lintel_header + sill + top_guide/bottom_guide + left_housing/right_housing + keypad + hazard_left/right | header beam + floor sill + side-cowl retract pockets + wall keypad + base hazard chevrons (pocket enlarges right housing; blastslabs deepen sill/lintel pockets) | converged-parent / converged |
| circular_bulkhead_ring | rec_scifi_gate_var_round_bulkhead | bulkhead_ring (`_make_bulkhead_ring` revolved disk + rect bore, deck-trimmed) + ring_bolt_{i} + header_trim arc | round armoured airlock bulkhead ring w/ rect doorway bore, rim lip, inner door-stop frame, perimeter bolt circle | converged |

### (non-structural) Slot E: fixed root furniture / greebles — exercised, NOT a counted axis
| 候选 | record_id | 关键 part | 结构特征 | 状态 |
|---|---|---|---|---|
| jamb_strips + clamp_blocks (基线) | rec_door_scifi_hex | light_strip_*, clamp_*, clamp_lamp_* | cyan jamb glow strips + mid-height clamp/latch blocks w/ red lamps | converged-parent |
| keypad + hazard_chevrons (基线) | rec_door_scifi_zigzag | keypad_box/key_*, hazard_left/right | wall keypad (screen+3×3 keys+green lamp) + base-corner yellow/black hazard chevrons | converged-parent |
| piston_rams + warning_beacons | rec_scifi_gate_var_piston_greeble | piston_0/1, beacon_0/1 | inlaid hydraulic actuator rams flanking + rotating amber warning beacons (replaces strips+clamps) | converged |

Fixed-furniture is the layer the piston_greeble variant moved; it is NOT a counted structural axis
and does NOT count toward GATE P1. No further furniture-only forks.

## Multiplicity / Copy Logic
- count_param: per-mechanism — `N_SLABS` (vertical lift), `N_PETALS` (iris swing), `N_LEAVES` (iris
  radial slide). Lateral telescope / pocket use a fixed leaf set (2 per side / 1), not a swept count.
  These are NOT a single shared orthogonal N across mechanisms (iris N ≠ slab N ≠ telescope N).
- N 样本已覆盖: {1 (pocket), 2 (parents/piston/bulkhead, per side), 4 (blastslabs4, leaves4),
  6 (blastslabs6, iris6), 8 (iris8, leaves8)} — 5 distinct N spanning the natural range.
- 模板建议 N_range: vertical-lift slabs `[3, 8]`; iris petals/blades `[5, 10]` (round-aperture
  evidence, 360/N angular); radial-slide leaves `[3, 8]`; lateral telescope `[1, 2]` leaves per side.
- copied object / naming / placement / joint policy: every multiplicity mechanism is loop-emitted
  from ONE shared geometry helper — `slab_{i}` (uniform ±Z PRISMATIC, staggered or even-pitch),
  `petal_{i}` (regular 360/N angular placement, REVOLUTE driver + mimics off petal_0), `leaf_{i}`
  (radial axis per index, PRISMATIC driver + mimics). Decoration (hazard decals, grooves, rivet/bolt
  rows, keypad grid, ring bolt circle) is inlaid as parent visuals with no per-decoration FIXED joint.

## 排除项 / dropped axes (future compatibility-matrix material)
- **Color / material only**: dropped (olive vs gunmetal vs white/grey plating) — never the change;
  allowed only as free top-dressing.
- **Pure scale**: dropped (opening width/height/depth) — continuous, owned by the template scaler.
- **Fixed furniture / greebles** (piston rams, beacons, keypad, clamp blocks, jamb strips): exercised
  by piston_greeble, NOT a counted axis, no further furniture-only forks.
- **Cross-mechanism N coupling**: iris N, slab N, and leaf N are NOT a shared count param; each
  mechanism keeps its own local count (Slot B lists them per-mechanism, not as one orthogonal axis).
- Compatibility notes for the future matrix: iris/radial-slide pair naturally with a round aperture
  and would pair best with the circular_bulkhead_ring surround (the converged iris/leaves sources
  currently host the iris on the hex slab — straight cyan jamb strips do not co-apply to a round
  aperture and were dropped on those variants). vertical-lift and lateral-telescope pair with either
  flat surround (hex slab or lintel-housing). Single-leaf pocket reuses the lintel-housing surround
  with an enlarged side pocket. zigzag-on-bulkhead (round_bulkhead) and hazard-cosine-on-hex (hex
  parent) are the two seam×surround corners that bridge the parent baselines.
- No blocked cells this batch; all 9 variants are distinct on at least one structural axis and none
  repeats the piston_greeble furniture-only cell.
