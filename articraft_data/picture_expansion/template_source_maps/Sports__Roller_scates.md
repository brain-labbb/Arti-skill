# Sports / Roller scates — template source map

pattern: multiplicity (wheels per skate are the copied sub-part; secondary structural slots are fixed named layers)
parents: rec_a-pair-of-inline-roller-skates-with-hard-boots-a_20260611_163854_397477_26f6e0cf ← picture/Sports/Roller scates/001.png
  (single parent; fills the inline-4 / hard-shell / lace cell. Each variant forks from this one parent — smallest single-axis diff.)

Object: a pair of inline roller skates. Per-skate local frame: +X toe, +Y lateral, +Z up, ground at z=0. The right skate is the same construction with mirrored lateral coordinates (side multiplier, never negative mesh scaling), FIXED-mounted to the left boot. Both skates share one mesh-construction routine.

## Slot 候选覆盖

### Slot A:wheel_arrangement (chassis + wheel multiplicity — primary multiplicity axis)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| inline_4 (parent) | rec_a-pair-of-inline-roller-skates-with-hard-boots-a_...26f6e0cf | {prefix}_frame (rail_lateral/rail_medial + deck_plate) / {prefix}_wheel_{i} ×4 / {prefix}_wheel_{i}_spin (CONTINUOUS, axis +Y) | single-file twin-rail rockered frame, 4 evenly spaced wheels, axle bosses at AXLE_XS, all coplanar on ground | converged (parent baseline) |
| inline_3 | rec_roller_skates_var_inline3 | frame rail (3 rocker arches/bosses) / {prefix}_wheel_{i} ×3 / {prefix}_wheel_{i}_spin ×3 | same single-file frame, wheel-count constant = 3 larger wheels, rail length + boss stations re-derived from N | built ✓ |
| inline_5 | rec_roller_skates_var_inline5 | frame rail (5 rocker arches/bosses) / {prefix}_wheel_{i} ×5 / {prefix}_wheel_{i}_spin ×5 | same single-file frame, wheel-count constant = 5 smaller speed wheels, rail length + boss stations re-derived from N | built ✓ |
| quad_2x2 | rec_roller_skates_var_quad | {prefix}_front_truck / {prefix}_rear_truck (baseplate + hanger) / {prefix}_wheel_{i} ×4 (2 per truck, L+R) / {prefix}_wheel_{i}_spin ×4 | two short transverse truck plates replace the inline rail; 2x2 wheel pattern on two cross axles | built ✓ |

> Note: inline_3 / inline_4 / inline_5 are the pure multiplicity (N) samples on a single inline frame; quad_2x2 is a distinct arrangement candidate (different chassis topology + placement law), still emitted by the same for-i loop over corner positions. Template should keep wheel_count + arrangement as one slot with N driving the inline branch.

### Slot B:boot_form
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| hard_shell (parent) | rec_a-pair-of-inline-roller-skates-with-hard-boots-a_...26f6e0cf | {prefix}_boot visuals: shell / shaft / liner_collar / heel_panel / toe_bumper ; {prefix}_cuff (collar/upper_collar) / {prefix}_cuff_flex (REVOLUTE +Y, ankle limits) | rigid molded superellipse side-loft shell + tall stiff ankle shaft + rigid cuff collar | converged (parent baseline) |
| soft_boot | rec_roller_skates_var_softboot | {prefix}_boot soft upper visuals / {prefix}_cuff (padded collar) / {prefix}_cuff_flex | soft lace-up high-top padded fabric upper, quilted ankle collar, deeper lace throat; soft padded cuff | built ✓ |
| low_cut | rec_roller_skates_var_lowcut | {prefix}_boot low-cut shell visuals / short {prefix}_cuff band / {prefix}_cuff_flex | aggressive/speed boot cut down near ankle bone, tall shaft + high collar dropped to short flexing cuff band | built ✓ |

### Slot C:closure
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| laces (parent) | rec_a-pair-of-inline-roller-skates-with-hard-boots-a_...26f6e0cf | boot visuals: tongue / eyelet_stay_lateral / eyelet_stay_medial / lace_{i} ×5 (loop, _lace_centers helper) ; cuff strap / strap_buckle | tongue + eyelet stays + 5 lace rungs across the tongue + cuff strap-buckle | converged (parent baseline) |
| buckle_ratchet | rec_roller_skates_var_buckle | {prefix}_boot instep_strap_{i} ×N (loop, shared strap+cam-buckle helper) / cuff power_strap + ratchet | molded ladder straps in cam buckles across instep + wide ratcheting power strap over cuff | built ✓ |
| velcro_strap | rec_roller_skates_var_velcro | {prefix}_boot velcro_strap_{i} ×N (loop, shared flat-strap helper) / cuff velcro_strap | wide flat hook-and-loop fabric straps across instep + wider velcro strap around cuff | built ✓ |

## Multiplicity / Copy Logic
- count_param: wheel_count (per-skate wheel multiplicity; parent currently iterates over AXLE_XS tuple — template/variant must read N from a single integer constant and derive axle stations from it)
- N 样本已覆盖: {3, 4, 5} → rec_roller_skates_var_inline3 / parent(inline_4) / rec_roller_skates_var_inline5 ; arrangement variant {quad 2x2} → rec_roller_skates_var_quad
- 模板建议 N_range: inline branch [3, 5] (real inline skates run 3–5 wheels; below 3 or above 5 leaves the category). quad branch is fixed at 4 wheels (2 trucks × 2). Secondary closure-strap multiplicity (instep straps) suggested 2–3 per boot.
- copied object: one wheel = tire mesh (TireGeometry) + hub mesh (WheelGeometry) + steel axle Cylinder. naming: {prefix}_wheel_{i} with per-skate prefix (left/right). placement: inline → evenly spaced axle stations along frame X derived from wheel_count and wheelbase; quad → 4 truck-corner positions (front/rear × left/right). joint policy: one CONTINUOUS spin joint per wheel about lateral +Y axle, origin at the axle boss contact (xyz=(station, 0, AXLE_Z)), uniform effort/velocity limits — every wheel identical and independently spinning.

## 排除项(未来 compatibility matrix 素材)
- (all variants forked, compiled & workbench-bound. Interface risks watched during fork, recorded here:)
- quad_2x2 × soft_boot / low_cut: untested chassis↔boot mount combo; quad trucks mount under a flat sole, low-cut/soft sole footprint may differ — leave to template compatibility matrix, not forked here (single-axis rule: each variant changes one slot only).
- inline_5 wheel packing: shrinking wheel radius to pack 5 wheels into the wheelbase risks tire-tire overlap; if wheels touch at radius needed for coplanar ground contact, this N may need a longer rail (note for template N_range upper bound).
- segment-count gotcha (from memory arti-roller-skate-hollow-throat-carve): boolean ops / side-loft on this boot are degenerate at segments=64 ("Profile area must be non-zero"); use segments <= 56 in any soft_boot / low_cut shell rebuild.

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
