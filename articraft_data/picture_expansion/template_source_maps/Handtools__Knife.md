# Handtools / Knife — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-knif_20260609_163936_067357_211319cd ← picture/Handtools/Knife/001.png (yellow 18 mm snap-off box cutter: tapered yellow shell, gray top channel, segmented silver snap-off blade, black thumb-grip, rear lanyard hole, sliding thumb button)

Slide-out utility knife / box cutter. The defining motion is the blade-deployment
**mechanism**: in the parent a `blade_carrier` rides the handle's top channel on a
PRISMATIC joint (`handle_to_carrier`, axis +X) that pushes the blade out the nose.
The handle root is `handle` (a `_handle_body_shape` loft, with `top_channel` rail,
`thumb_grip`, lanyard-hole cut, and a rear `end_cap` part on a FIXED `handle_to_cap`).
The three independent structural slots are: how the blade deploys (mechanism), the
handle/grip form, and the exposed blade profile. NOTE: an old non-modular
`retractable_utility_knife.py` exists downstream — it is NOT a source here; this map
is for a fresh modular template covering the whole Knife pool.

## Slot 候选覆盖

### Slot A:blade deployment mechanism (the non-fixed joint + the moving member)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| snap_off_slide (baseline) | rec_..._knif_..._211319cd (parent) | `blade_carrier` part, `handle_to_carrier` (PRISMATIC axis +X); carrier visuals `blade_steel`/`blade_tip`/`blade_spine`/`thumb_button`; `handle_to_cap` (FIXED) | blade carrier slides along the channel +X; only ~12 mm exposed at rest, pushes out the nose; 1 non-fixed joint (prismatic) | converged |
| retract_full (trapezoidal) | rec_knife_var_mech_retract | `blade_carrier` part, `handle_to_carrier` (PRISMATIC axis +X); carrier = `blade_body`+`blade_clamp`+`post_0/post_1`(loop)+`thumb_button`; end_cap inlined as handle visual | same prismatic slide but blade FULLY retracts inside the body at q=0; trapezoidal blade clamped by a plate + 2 mounting posts; cleanest fully-retracting copy-logic sample | converged |
| fold_pivot | rec_knife_var_mech_fold | `blade` part, `handle_to_blade` (REVOLUTE axis Y, origin near nose `PIVOT_X/PIVOT_Z`, limits 0..π); handle `pivot_pin`(brass)+`front_bolster` visuals; `handle_to_cap` (FIXED) | prismatic slide replaced by a folding hinge; blade stows inside the handle (closed) and swings out the front-top groove (open); 1 non-fixed joint (revolute) | converged |
| flipup_guard (fixed blade) | rec_knife_var_mech_guard | `safety_guard` part, `handle_to_guard` (REVOLUTE axis -Y, origin `HINGE_X/HINGE_Z`, limits 0..π); blade INLINED on handle (`blade_steel`/`blade_tip`); handle `hinge_bracket` ears; `handle_to_cap` (FIXED) | blade is permanently fixed (no deployment); the moving member is instead a flip-up orange safety guard (barrel+plate+skirts+front_lip+ribs) that pivots off the cutting edge; 1 non-fixed joint (revolute) | converged |

> Joint topologies present: PRISMATIC (snap_off_slide, retract_full) and 2 distinct
> REVOLUTE placements (fold = blade pivot near nose, guard = guard pivot above edge).
> snap_off_slide vs retract_full share the prismatic joint but differ in moving-member
> topology (carrier+clamp+posts vs carrier+spine) and retract depth — both kept as
> distinct candidates so the template author has the simple slider AND the fully-
> retracting clamped slider to choose from.

### Slot B:handle / grip form (`handle` root silhouette + surface)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| tapered_molded (baseline) | rec_..._knif_..._211319cd (parent) | `_handle_body_shape` rounded-rect loft (`handle_shell` yellow), `top_channel`, `thumb_grip`, `_lanyard_hole_cut` | classic tapered molded plastic shell, tall blocky rear → pointed nose | converged |
| ergo_contoured | rec_knife_var_grip_ergo | `_handle_body_shape` w/ palm-swell profile + `_finger_groove_cuts`; `finger_groove_{i}` rubber inserts (loop over `_groove_positions`, 4×) | contoured ergonomic grip: mid-handle palm swell + 4 molded concave finger grooves with dark rubber inserts | converged |
| overmold_barrel | rec_knife_var_grip_overmold | `_build_barrel_body` circular loft (`barrel_shell` rubber), `tpr_rib_{i}` revolved ring ribs (loop N_RIBS=8) | round rubber-overmolded barrel grip with N raised TPR rib rings revolved around the barrel axis | converged |
| flat_metal_bar | rec_knife_var_grip_flat | `_handle_body_shape` = uniform squared `box` bar (`handle_shell` brushed steel), `_blade_exit_slot`, side-face `thumb_grip` via `_knurl_bumps_side` | slim flat squared-section metal bar (uniform, no taper); side-mounted knurled grip pad; shared knurl helpers | converged |

### Slot C:exposed blade profile (`blade_steel` outline + `blade_tip` + edge detail)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| snap_off_segmented (baseline) | rec_..._knif_..._211319cd (parent) | `_build_blade_shape` parallelogram + `_build_blade_score_lines` (loop 5, rotated diagonal cuts) + `_build_blade_tip_visual` | straight segmented snap-off blade with diagonal score lines + dark worn tip | converged |
| hawkbill | rec_knife_var_blade_hawk | `_build_blade_shape` spline (concave edge + hooked tip), `_hawkbill_edge_z` helper sizes score lines, `_build_blade_tip_visual` covers hook | hawkbill: concave cutting edge curving to a downward hook below the spine line | converged |
| drop_point | rec_knife_var_blade_drop | `_build_blade_shape` drop-point polyline (sloping spine + belly + centered point), `_build_blade_grind_line` bevel groove, `_build_blade_tip_visual` | drop-point trimming blade: gently sloping spine, curved belly, centered tip; bevel grind line | converged |
| serrated_sheepsfoot | rec_knife_var_blade_serr | `_sheepsfoot_outline_pts` (curved-down spine, blunt rounded tip, 14 triangular teeth via loop), `_build_blade_shape`, `_build_blade_tip_visual` | serrated sheepsfoot: toothed cutting edge + blunt rounded tip (no sharp point) | converged |

## Multiplicity / Copy Logic
- count_param: 无 dedicated count axis for the core knife. The defining structure is
  fixed named slots (mechanism / grip / blade). Copy loops are local detail textures,
  NOT a knife-level multiplicity axis:
  - `tpr_rib_{i}` rib rings on overmold_barrel grip (N_RIBS, here 8) — circumferential rings.
  - `finger_groove_{i}` finger grooves on ergo_contoured grip (here 4) — paired cut+insert.
  - `post_{i}` blade-mount posts on retract_full carrier (here 2).
  - `_build_blade_score_lines` (5) / sheepsfoot teeth (14) — blade-edge detail loops.
- N 样本: no multiplicity slot sweep — N is incidental texture, not a topology axis.
- 模板建议 N_range: if the template author promotes barrel TPR ribs to a parameter,
  suggest tpr_rib_count ∈ [4, 16] (sample 8); finger_groove_count ∈ [2, 5] (sample 4).
- copied object / naming / placement / joint policy: all copy loops emit `name_{i}` via
  `for i in range(n)` with a shared geometry helper, regular placement (equal X pitch for
  grooves/ribs/teeth, revolve angle for rings), and a uniform joint policy = FIXED/inline
  parent visuals (none of the copied detail elements articulate).

## 组合数预审
Slot A(4 mechanism candidates) × Slot B(4 grip) × Slot C(4 blade) = 64 ≥ 10 ✓.
Even the conservative count (3 distinct joint topologies × 4 grips × 4 blades = 48) far
pattern = parallel_children; no multiplicity axis needed to clear the gate.

## 排除项(未来 compatibility matrix 素材)
- flipup_guard uses a FIXED (permanently exposed) blade; it is the only mechanism
  candidate incompatible with the "blade retracts into body" claim. When composed with
  the fold/slide mechanisms the guard module should be dropped (a deploying blade and a
  blade-covering guard are mutually exclusive real-world configs) — flag for the spec
  compatibility matrix.
- fold_pivot needs the blade to STOW inside the handle when closed; pairing it with the
  flat_metal_bar grip (slim HANDLE_H≈0.014) leaves little internal volume for the blade —
  combo untested, possible containment/clearance interference; let the sampler test or
  gate via compatibility matrix.
- Pure dimensional variation (handle length/width, blade exposure length, channel depth,
  grip diameter) is NOT a candidate — it is template controlled local parameterization.
- Cross-slot combos (e.g. overmold_barrel × hawkbill, ergo × serrated) were intentionally
  NOT fork-sampled; combinations are the template sampler's output, not the source pool's job.
