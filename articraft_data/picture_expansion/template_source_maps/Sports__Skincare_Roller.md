# Sports / Skincare Roller — template source map

pattern: multiplicity (one roller-head + fork unit copied per end; both ends spin on independent revolute axles)
parents: rec_jade-facial-massage-roller-with-a-metal-handle-a_20260605_165942_156361_d964a399 ← picture/Sports/Skincare Roller/001.png (double-ended jade roller; fills Slot A=smooth_oval, Slot B=straight_stone_bar, Slot C=u_wire_fork, N=2)

Real object: a handheld jade facial massage tool. A static stone handle carries a polished metal collar at each end; a metal fork rises from each collar and captures a stone roller head on a cross axle; each head spins freely. Functional layers: handle body (static grip), collars (static decoration on the handle), forks/yokes (static, hold the axle), roller heads (the moving parts, one CONTINUOUS spin joint each).

## Slot 候选覆盖

### Slot A: roller_head_form  (the stone that spins in each fork)
| 候选 (future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| smooth_oval | rec_jade-facial-massage-roller-with-a-metal-handle-a_20260605_165942_156361_d964a399 | small_roller / large_roller stone via _oval_roller helper; small_roller_spin + large_roller_spin (continuous, axis X) | parent: oblate ellipsoid jade ovals, glossy smooth, tiny off-axis marker nub | converged (parent) |
| faceted_gem | rec_skincare_roller_var_faceted_gem | <roller>_stone faceted via shared lathe/CadQuery facet helper; per-end spin joints | multi-facet cut-gem barrel (rose/briolette), discrete planar facets instead of smooth surface | built ✓ |
| spiky_germanium | rec_skincare_roller_var_spiky_ball | <roller>_stone core ball + nub_{i} field (nested loop); per-end spin joints | near-spherical massage ball studded with a regular field of short rounded nubs/spikes | built ✓ |
| textured_ridged | rec_skincare_roller_var_ridged | <roller>_stone ridged via lathe profile with wavy outline; per-end spin joints | barrel with regular circumferential grooves/ridges around the spin axis (revolved ribs) | built ✓ |

### Slot B: handle_form  (the static stone grip body)
| 候选 (future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| straight_stone_bar | rec_jade-facial-massage-roller-with-a-metal-handle-a_20260605_165942_156361_d964a399 | body.handle (CylinderGeometry + faint barrel bulge merge) | parent: slim constant-radius round shaft, faint mid barrel bulge | converged (parent) |
| contoured_waisted | rec_skincare_roller_var_waisted | body.handle as lathe/CadQuery revolved profile dipping at waist | ergonomic hourglass-leaning grip: narrows at mid-waist, swells toward both collar ends; same length + collar seats | built ✓ |
| flat_paddle_bar | rec_skincare_roller_var_paddle | body.handle as lofted/extruded flat oval cross-section | flat lozenge cross-section grip (wide one way, thin the other) like a paddle; same length + collar seats | built ✓ |

### Slot C: fork_yoke_form  (the metal yoke holding the roller axle)
| 候选 (future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| u_wire_fork | rec_jade-facial-massage-roller-with-a-metal-handle-a_20260605_165942_156361_d964a399 | body.top_fork / bottom_fork via _u_fork helper (tube_from_spline_points) | parent: thin round bent-wire U-fork, two arms to +/-X axle tips | converged (parent) |
| flat_blade_yoke | rec_skincare_roller_var_blade_yoke | body.<top/bot>_yoke as flat extruded/lofted plate, two blade arms with axle bores | stamped flat sheet-metal yoke; two flat blade arms instead of round wire | built ✓ |
| single_cantilever_arm | rec_skincare_roller_var_cantilever | body.<top/bot>_arm bent-tube/lofted single arm; axle = cantilever stub | one-sided arm holds the axle as a cantilever stub (roller supported from one side only) | built ✓ |

## Multiplicity / Copy Logic
- count_param: roller_count (number of roller-head + fork units mounted on the handle)
- copied object: one roller-head + its fork/yoke + collar + axle + CONTINUOUS spin joint, emitted per end via a for-i-in-range(roller_count) loop with name_{i} naming (e.g. roller_{i}, fork_{i}, roller_{i}_spin)
- N 样本已覆盖: {1, 2} → rec_skincare_roller_var_single_ended (N=1, single-ended; one head, plain rounded stone tip + collar at the bare end) / parent (N=2, double-ended, small head one end + large head the other)
- 模板建议 N_range: [1, 2] (a handheld facial roller physically supports only one or two roller ends on a single straight handle; N is a true topology axis but its real-world domain is small)
- placement: roller units sit at the +Z and -Z ends of the handle axis (top/bottom collar seats); for N=1 only the larger -Z end unit is emitted
- joint policy: each roller unit gets its own independent CONTINUOUS revolute joint about its axle (axis X), parent=body, child=roller_{i}; uniform across all units, no chaining
- note: at N=2 the two heads differ in size (small top, large bottom) — this is a controlled per-unit size parameter (continuous), NOT a separate structural candidate; the copy helper is shared and scales each unit.

## 排除项 (future compatibility matrix material)
- single_cantilever_arm (Slot C) pairs most naturally with N=1 (single-ended); a one-sided cantilever on a double-ended tool is buildable but mechanically odd. Flag as a Slot C × N compatibility note: cantilever yoke is primarily a single-ended-friendly module. Not excluded, just gated for the matrix.
- spiky_germanium head (Slot A) on a flat_blade_yoke or single_cantilever (Slot C) has tighter axle-clearance near the nub field; watch nub-vs-arm penetration when those modules combine (interface risk note for the compatibility matrix).
- No axis was dropped to a single candidate; all three slots carry 3-4 structurally distinct candidates plus a 2-value multiplicity axis.

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
