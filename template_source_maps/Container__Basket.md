# Container / Basket — template source map

pattern: mixed (linear_chain body→lid with one prismatic/revolute closure joint; multiplicity only on the optional side-handle layer)

parents (5; body_footprint axis is essentially fully covered by these, so NO variant re-creates a footprint):
- rec_woven-rattan-basket-with-a-fitted-lift-off-woven_20260618_090109_567794_66ae2d28 ← picture/Container/Basket/001.png — round/bulbous body, shallow flush woven lift-off lid, bare lid (no grip)
- rec_hexagonal-woven-rattan-basket-with-lift-off-lid_20260618_123018_a27c9e51 ← picture/Container/Basket/002.png — soft-hexagonal body, thick overhanging cushion lid, bare lid (color-band decorated)
- rec_oval-woven-rattan-basket-with-raised-lid-handle_20260618_121200_003basket ← picture/Container/Basket/003.png — oval/superellipse body, full domed lid, raised woven oval knob
- rec_rectangular-woven-rattan-basket-with-fitted-lift_20260618_130217_161905_99719568 ← picture/Container/Basket/004.png — rounded-rectangle body, tented lid, raised woven rectangular handle
- rec_cylindrical-woven-rattan-basket-with-flat-lid_20260618_131521_basket005 ← picture/Container/Basket/005.png — straight cylindrical body, flat lid, black upright ring handle

Shared functional layers across all 5 parents: woven bottom/floor (crossed canes + foot ring) → vertical stakes (for-j loop) → horizontal weave rows (for-i loop) → braided body mouth rim → woven lid face (crossed strips) + braided lid rim → `body_to_lid` PRISMATIC joint, axis (0,0,1). Parts are uniformly named `basket_body` / `basket_lid`; joint `body_to_lid`. Readability contract is honored in every parent (stakes/rows/floor/lid strips all emitted via for-i loops with shared helpers).

## Slot 候选覆盖

### Slot A:lid_closure_joint (how the lid attaches and opens)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| prismatic_liftoff | (all 5 parents) | basket_lid / body_to_lid (PRISMATIC, axis z) | lid lifts straight off vertically | converged(parent) |
| hinged_flip_lid | rec_container_basket_var_hinged_flip_lid | basket_lid / body_to_lid (REVOLUTE, axis along rear rim chord) | lid swings up/back on a rear hinge instead of lifting off; fork of cylinder parent | converged |
| bail_swing_handle | rec_container_basket_var_bail_swing_handle | carry_bail / body_to_bail (REVOLUTE, horizontal axis) | fitted lid + overhead carry bail that swings up/down on opposite rim lugs (bail is the mover); fork of round parent | converged |

### Slot B:lid_grip (the grasp/handle feature on the lid)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| bare | rec_woven-rattan-basket-with-a-fitted-lift-off-woven_20260618_090109_567794_66ae2d28 / rec_hexagonal-woven-rattan-basket-with-lift-off-lid_20260618_123018_a27c9e51 | basket_lid (no grip part) | flush/decorated lid, no grasp feature | converged(parent) |
| raised_oval_knob | rec_oval-woven-rattan-basket-with-raised-lid-handle_20260618_121200_003basket | raised_handle_base_braid / raised_handle_vertical_weave | low woven oval knob centered on lid | converged(parent) |
| raised_rect_handle | rec_rectangular-woven-rattan-basket-with-fitted-lift_20260618_130217_161905_99719568 | raised_rect_handle_rim / handle_vertical_stake / handle_top_weave | small woven rectangular box handle | converged(parent) |
| black_ring_handle | rec_cylindrical-woven-rattan-basket-with-flat-lid_20260618_131521_basket005 | black_ring_handle_loop / black_ring_handle_mount | small upright black ring loop | converged(parent) |
| arched_carry_bail | rec_container_basket_var_arched_carry_bail | carry_bail_arch / bail_post_i (for-i posts) | tall fixed woven overhead grab arch (picnic-style) on lid; fork of cylinder parent | converged |
| twist_turn_knob_latch | rec_container_basket_var_twist_turn_knob_latch | turn_knob / lid_to_knob (REVOLUTE, axis z) | central quarter-turn lock knob (active revolute) replacing passive knob; fork of oval parent | converged |

### Slot C:wall_weave_structure (the side-wall weave pattern)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| dense_over_under | (all 5 parents) | horizontal_*_weave_band_i / vertical_*_stake_j | tight plain over-under solid wall | converged(parent) |
| open_lattice_weave | rec_container_basket_var_open_lattice_weave | lattice_stake_i / lattice_cross_i (shared lattice helper) | widely spaced openwork lattice with large see-through diamond gaps; fork of rect parent | converged |
| diagonal_twill_weave | rec_container_basket_var_diagonal_twill_weave | twill_strand_i (diagonal-bias shared helper) | slanted over-two/under-two herringbone twill ribs; fork of round parent | converged |
| dense_checker_weave | rec_container_basket_var_dense_checker_weave | checker_band_i / checker_stake_j (alternating over-under helper) | denser checkerboard-style weave with compact square cells | converged |
| banded_wave_weave | rec_container_basket_var_banded_wave_weave | wave_band_i / sinusoidal weave-row helper | horizontal wave bands wrapping the wall, distinct from diagonal twill and open lattice | converged |

## Multiplicity / Copy Logic
- count_param: `side_handle_count` — the optional pair of integral woven side carry grips on the body wall (only the side_carry_handles variant uses copy logic; core body→lid structure is fixed named slots).
- N 样本已覆盖: {2} → rec_container_basket_var_side_carry_handles (a symmetric pair, one per opposing side face)
- 模板建议 N_range: [0, 2] (real baskets carry 0 side grips on a lidded basket, or a single opposing pair = 2; odd counts are not realistic, so the template should sample {0, 2})
- copied object / naming / placement / joint policy: copied object = one woven loop grip from a shared handle-loop helper; naming = side_carry_handle_i; placement = evenly on opposing body side faces at a fixed grip height; joint policy = inlined as body visuals (non-moving woven grips, no per-grip articulation), consistent with the parent's inline-decoration rule.

组合数预审: Π(Slot A=3 × Slot B=6 × Slot C=5) = 90 ; × side-handle multiplicity (N samples {0,2}=2) = 180 ≥ 10 ✓
(9 empty cells actually planned as variants: 2 on Slot A, 2 on Slot B, 4 on Slot C, 1 multiplicity. body_footprint cells are all filled FREE by the 5 parents and are not re-planned.)

## 排除项(未来 compatibility matrix 素材)
- body_footprint axis (round / hexagonal / oval / rectangular / cylindrical): all 5 cells already occupied by the 5 parents — deliberately NOT planned as variants (would only re-create a parent footprint).
- twist_turn_knob_latch × open_lattice_weave / diagonal_twill_weave: a turn-lock latch needs a solid lid + rim seat to bear against; pairing it with openwork side walls is plausible but the latch lugs must still grab a solid mouth rim — flagged as a likely non-converging cross until the rim/lug contact is proven.
- side_carry_handles on the round/bulbous parent: a recessed grip needs a near-vertical side face; on the strongly bulbous round body the grip would float off the curved belly, so the pair is anchored on the hex parent's flatter side faces instead (excluded the round host for this axis).
- color/material-only and pure-scale changes: not axes (excluded per FORK_VARIANTS §2).
