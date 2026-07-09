# Door / Other — template source map

slug: door_other  ·  shard: Other  ·  picdir: picture/Door/Other

pattern: mixed (parallel named slots: split_mechanism + head_profile + leaf_infill/hardware) × multiplicity (plank / lite / slat count)

> **HETEROGENEITY FLAG (catch-all subcategory).** Door/Other is a heterogeneous "other single door" bucket. Its two parents are two genuinely DIFFERENT objects: a Dutch (stable) door (002.png) and a rustic arched plank door (001.png). They differ in split mechanism, head profile, leaf infill, and hardware all at once — they are not minor variants of one object. The on-disk variant pool also splits into two shells (Dutch two-leaf shell vs arched single-leaf-in-stone shell) that share almost no code.
>
> **Downstream recommendation for the spec author:** organize the template around the **horizontal-split leaf mechanism (the Dutch identity) as the primary slot (Slot A)**, because it carries the most distinctive articulation: a leaf split into an independently-swinging top half and bottom half on two separate vertical revolute joints. Treat **head profile** (flat / full-semicircle / barn-segmental / shallow-segmental / flat-top-in-arched-surround) and **leaf infill/hardware** (glazed lite, raised panel, louvered vent, plank+ring-pull, porthole) as orthogonal slots that the two shells fill from opposite corners. **Heads-up:** the two shells are currently structurally disjoint (Dutch shell = `door_frame`/`upper_leaf`/`lower_leaf`; arched shell = `stone_frame`/`door`/`ring_pull`). Slot A's `single_solid_leaf` candidate today lives only on the arched stone-surround shell, not on the Dutch wood-casing shell — see 排除项. The spec author must decide whether to unify the frame context as a separate slot or keep two parent corners.

parents:
- **rec_door_dutch** ← picture/Door/Other/002.png (Dutch / stable door; leaf split HORIZONTALLY into an upper glazed-lite leaf and a lower raised-panel leaf, each on its own vertical Z revolute hinge on the SAME jamb, opening independently; wood `door_frame` root = `hinge_jamb`/`latch_jamb`/`head_jamb`/`threshold`/`casing_*`; visible barrel hinges `upper_hinge_{i}`/`lower_hinge_{i}`; latch-edge `door_knob`). **PRIMARY identity + base shell for the Dutch-leaf variants.** Fills Slot A=`dutch_two_leaf_horizontal_split`, Slot B=`flat_square_head`, Slot C=`glazed_lite_over_raised_panel + knob`, N=4 lites (2×2).
- **rec_door_other_arched** ← picture/Door/Other/001.png (rustic arched single-plank leaf; one solid full-height `door` leaf on a vertical Z revolute `frame_to_door` hinge inside a carved stone arch `stone_frame`; vertical plank board-and-batten face fused via a `for i in range(1, PLANK_COUNT)` groove loop; black wrought-iron `strap_hinges` + `iron_studs` dome studs + hanging `ring_pull` on its own revolute `door_to_ring_pull` pivot; fixed `jamb_pintles` on the stone). **SECONDARY shell.** Fills Slot A=`single_solid_leaf`, Slot B=`full_semicircular_arch`, Slot C=`vertical_plank_strap + ring_pull`, N=6 planks.

Frame conventions: both shells use X=door width (hinge edge at local x=0, latch edge at large X), Y=thickness (room/front side at +Y), Z=height (floor at z=0). Both use a VERTICAL (Z) revolute hinge at the hinge jamb; the Dutch parent just has two of them at different heights, the arched parent one. The ring-pull adds a second, horizontal (door-local X) revolute pivot on the arched shell.

## Readability / loop-emission audit (parents + variants)

- **rec_door_dutch**: hinges loop-emitted (`for i, hz in enumerate(...)` → `upper_hinge_{i}` / `lower_hinge_{i}`). The **divided lites (2×2 muntin grid) and the single raised panel are HAND-WRITTEN** (single `v_bar`/`h_bar` in `_window_frame_leaf_cq`; one `panel` in `_raised_panel_leaf_cq`). These are the repeated parts that a future `lite_count` / `panel_count` template must convert to a `for-i-in-range` loop with a shared helper.
- **rec_door_other_arched**: plank seams loop-emitted as a FUSED groove cut (`for i in range(1, PLANK_COUNT)`); batten + strap loops over `(0.45, 1.30)`. Dome studs hand-written as fused decoration (fine as a parent visual, not jointed).
- **rec_door_other_var_plankcount**: BEST multiplicity sample — planks promoted to **individual loop-emitted visuals** `plank_{i}` (`for i in range(PLANK_COUNT)` + shared `_build_single_plank` helper + `_plank_pitch`), each board its own named part. This is the clean copy-logic source.
- **rec_door_other_var_plankcount_three**: planks fused, but seams loop-emitted via `_plank_pitch` + `for i in range(seam_count)`.
- **rec_door_other_var_louvered**: louver slats loop-emitted as `slat_{i}` (`for i in range(N_SLATS)`, shared `Box` geometry, even pitch, uniform FIXED-into-leaf policy) — clean copy-logic source for the slat multiplicity.

## Slot 候选覆盖

### Slot A: leaf split mechanism (PRIMARY axis — the Dutch identity)
| 候选 (future module) | record_id | 关键 part / joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| dutch_two_leaf_horizontal_split (baseline) | rec_door_dutch (parent); also rec_door_other_var_levered_panel, rec_door_other_var_louvered | upper_leaf / lower_leaf; frame_to_upper + frame_to_lower (2 independent Z revolute) | two halves split at a mid-rail, each on its own vertical hinge, opening independently | converged (parent + 2 variants) |
| single_solid_leaf | rec_door_other_arched (parent); also plankcount / plankcount_three / porthole / roundtop / segmental | door; frame_to_door (1 Z revolute) | one continuous full-height leaf on one hinge | converged (parent + 5 variants) |
| center_pivot | rec_door_other_var_centerpivot | center-pivot leaf on a single vertical pivot through the leaf centerline | leaf rotates about its own vertical centerline rather than a jamb edge | converged (gap-fill; compile success, 2 joints, workbench) |

### Slot B: head / top profile
| 候选 (future module) | record_id | 关键 part / joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_square_head (baseline) | rec_door_dutch (parent); also levered_panel, louvered | head_jamb / casing_head (Box) | flat square top, rectangular wood casing | converged |
| full_semicircular_arch | rec_door_other_arched (parent); also plankcount, plankcount_three | `_arched_profile_face(width, spring, top)` with radius=W/2 (semicircle), `stone_arch` keystone | tall church-style full semicircle springing at LEAF_SPRING | converged |
| broad_barn_segmental_arch | rec_door_other_var_roundtop | `_arched_profile_face(width, spring, rise)` with ARCH_RISE=0.20, LEAF_W=1.20 wide | broad shallow barn-door curved head on a wide leaf | converged |
| shallow_segmental_arch | rec_door_other_var_segmental | `_arched_profile_face` with LEAF_RISE=0.12 (very shallow) | gently cambered low-rise segmental top | converged |
| flat_top_rect_in_arched_surround | rec_door_other_var_porthole | flat-top rectangular leaf (LEAF_TOP=2.00, no leaf arch) inside a tympanum stone arch | rectangular leaf head; the arch lives only in the stone surround above | converged |

> NOTE on id naming: `rec_door_other_var_roundtop` is NOT a round/semicircular top — it is the **broad barn segmental** arch (rise 0.20 on a 1.20 m leaf). The id is misleading; use the structural feature, not the slug. The true full semicircle is the arched PARENT + plankcount variants.

### Slot C: leaf infill / latch hardware
| 候选 (future module) | record_id | 关键 part / joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| glazed_lite_over_raised_panel + knob (baseline) | rec_door_dutch (parent) | window_glass + raised panel (`_window_frame_leaf_cq` muntins) + door_knob | frosted 2×2 lite over moulded raised panel, latch knob | converged |
| solid_raised_panel_both_leaves + lever | rec_door_other_var_levered_panel | upper_body + lower_body (both `_raised_panel_leaf_cq`) + lever_handle (`_lever_handle_mesh`) | both leaves solid raised-panel, horizontal lever on backplate | converged |
| louvered_slat_vent + knob | rec_door_other_var_louvered | slat_{i} (loop) over `_louver_frame_leaf_cq` opening + door_knob | upper leaf is a tilted-slat louver vent (N_SLATS), lower raised panel | converged |
| vertical_plank_strap + ring_pull | rec_door_other_arched (parent); also plankcount, plankcount_three, roundtop, segmental | door_planks (groove loop) / plank_{i} (plankcount); strap_hinges; ring_pull (revolute door_to_ring_pull) | board-and-batten planks, forged strap hinges, hanging iron ring pull | converged |
| porthole_glazed_round_light | rec_door_other_var_porthole | porthole_glass + muntin_rings (annular) cut into door_planks; ring_pull | circular through-cut porthole light with continuous iron muntin ring + glass pane | converged |

## Multiplicity / Copy Logic
- count_param: `plank_count` (vertical plank boards on the single-leaf shell) — PRIMARY multiplicity, best-sampled. Secondary axes: `lite_count` (divided panes in the Dutch glazed leaf, currently hand-written), `louver_slat_count` (`N_SLATS` tilted slats).
- N 样本已覆盖:
  - planks: {3 → rec_door_other_var_plankcount_three (fused wide boards), 6 → rec_door_other_arched / porthole / segmental, 8 → rec_door_other_var_roundtop, 9 → rec_door_other_var_plankcount (loop-emitted `plank_{i}`)} = **4 distinct N** (satisfies 2–3 distinct N).
  - lites: {4 (2×2, Dutch parent)} — single sample, hand-written; flag for loop conversion.
  - louver slats: {13 (rec_door_other_var_louvered, loop-emitted `slat_{i}`)} — single sample, clean copy logic.
- 模板建议 N_range: planks [3, 12] (weighted toward 4–8; cleanest copy source = `_plank_pitch` + `_build_single_plank` + `for i in range(PLANK_COUNT)` in plankcount); lites [2, 9] (2×2..3×3 grid, weighted 4); louver slats [6, 18]. Wider ranges need extra reference evidence.
- copied object / naming / placement / joint policy:
  - **planks** → `plank_{i}` (or fused seam `for i in range(seam_count)`), shared arched/rect profile helper, uniform pitch = LEAF_W / PLANK_COUNT, all rigidly part of the leaf (no per-plank joint — they move with the leaf hinge).
  - **lites** → should become `pane_{i}` / `muntin_{i}` over a row×col grid, regular pitch, uniform FIXED-into-leaf transparent glazing.
  - **louver slats** → `slat_{i}`, shared `Box` geometry, even vertical pitch `(i+0.5)/N_SLATS`, uniform tilt angle, FIXED into the leaf.

## 组合数预审 (HARD GATE — P1)
- Slot A candidates (≥1 on-disk sample): **2** (dutch_two_leaf, single_solid_leaf). `center_pivot` is planned-forking, not counted.
- Slot B candidates: **5** (flat_square, full_semicircle, broad_barn_segmental, shallow_segmental, flat_top_rect).
- Slot C candidates: **5** (glazed_lite, solid_raised_panel, louvered, plank_strap+ring, porthole).
- distinct N (plank multiplicity): **4** (3, 6, 8, 9).
- **combo = A(2) × B(5) × C(5) × N(4) = 200 ≥ 10 ✓ GATE P1 MET.**

## 格子覆盖 (this batch — on-disk; parents are the two baseline corners)
- 7 converged variants on disk: levered_panel, louvered (Dutch-shell forks); plankcount, plankcount_three, porthole, roundtop, segmental (arched-shell forks).
- Slot A: 2 candidates filled (both parents); center_pivot planned-forking (would make 3).
- Slot B: 5 candidates filled (flat from Dutch, semicircle from arched parent + plankcount, barn-segmental from roundtop, shallow-segmental from segmental, flat-top-rect from porthole).
- Slot C: 5 candidates filled (glazed-lite Dutch, solid-panel+lever levered_panel, louvered, plank+ring arched, porthole).
- Multiplicity: 4 distinct plank N (plankcount_three=3, parent/porthole/segmental=6, roundtop=8, plankcount=9) + lite N=4 + slat N=13.

## 排除项 (future compatibility matrix material / dropped axes)
- **DROPPED color/material/pure-scale axes (forbidden):** pine vs oak vs painted tone, glass tint, leaf width/height scaling (note roundtop's LEAF_W=1.20 vs 0.90 is a continuous proportion param, NOT a separate slot — only its barn-arch *profile* counts as the Slot B candidate).
- **DROPPED frame-context axis (stone arch surround vs wood casing/jamb):** this rides along with the parent shell choice (arched variants carry `stone_frame`+`jamb_pintles`; Dutch variants carry `door_frame`+casing). It is NOT an independent structural slot here — fold it into the parent-corner / head-profile choice rather than forking it separately. The spec author may promote it to a slot if the two shells are unified, but the fork batch does not bank it.
- **Shell-incompatibility heads-up:** Slot A `single_solid_leaf` currently exists ONLY on the arched stone-surround shell; `dutch_two_leaf` only on the Dutch wood-casing shell. The Dutch×plank-strap, Dutch×porthole, and single-leaf-on-Dutch-casing cells are unfilled by design (would need shell unification) — mark them in the compatibility matrix rather than forcing forks.
- **Mutually exclusive picks:** the five Slot B head profiles are one-at-a-time; the five Slot C infills are one-at-a-time. `single_solid_leaf` (j=1, one leaf) is incompatible with any Dutch-only infill that needs two leaves (glazed-lite-over-panel, louvered-upper-over-panel-lower).
- No non-converging axis values were recorded this batch (all 7 variants compiled + passed run_tests with ≥2 non-fixed joints).
