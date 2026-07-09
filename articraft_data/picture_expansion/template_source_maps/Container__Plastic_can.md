# Container / Plastic can — template source map

pattern: parallel_children
parents:
- rec_white-plastic-gallon-jug-with-a-screw-cap-and-an_20260612_114111_535190_0f3f18ac ← picture/Container/Plastic can/001.png (tall rounded-rect HDPE gallon jug, tapered shoulder to small deck, integrated raised loop handle, offset screw cap) — fills Slot A `tall_rounded_rect` · Slot B `integrated_loop` · Slot C `screw_cap`
- rec_gold-plastic-engine-oil-jug-with-a-black-screw-c_20260612_114121_711041_47fb268a ← picture/Container/Plastic can/002.png (tall rounded-rect 4L oil jug, full-width shoulder slab, flush oval D-grip cut through depth, corner screw cap) — fills Slot A `tall_rounded_rect` (dup) · Slot B `flush_dgrip` · Slot C `screw_cap`
- rec_black-plastic-square-jerrycan-with-a-round-screw_20260612_114133_356061_1078fc9e ← picture/Container/Plastic can/003.png (near-cubic square HDPE pail, chunky rounded edges, recessed top deck with raised bridge/strap grip, offset screw cap) — fills Slot A `square_cubic` · Slot B `bridge_strap` · Slot C `screw_cap`
- rec_black-plastic-jerrycan-with-a-screw-cap-and-a-bu_20260612_114145_800343_ab019c96 ← picture/Container/Plastic can/004.png (tall rectangular jerrycan, sloped/peaked shoulder, integrated front-to-back through-slot grip in the high plateau, screw cap) — fills Slot A `tall_rectangular_sloped` · Slot B `recessed_top_slot` · Slot C `screw_cap`

All four parents share the same defining kinematics: a `body` (root) carries a massless `cap_carrier`
on a CONTINUOUS `cap_rotate` (axis +Z at the neck) and the `cap` on a PRISMATIC `cap_slide` (axis +Z),
giving a decoupled screw cap present in every parent. The structural vocabulary therefore lives in
three independent slots: overall body footprint/shape family (A), handle/grip mechanism (B), and the
cap/closure mechanism (C). The four parents already fill all of Slot A and four of six Slot B cells;
variants fill the two remaining handle candidates and the two non-screw closure candidates.

## Slot 候选覆盖

### Slot A:body footprint / shape family
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| tall_rounded_rect | rec_white-plastic-gallon-jug-with-a-screw-cap-and-an_20260612_114111_535190_0f3f18ac (parent 001) | `body`/`jug_shell`, helper `_rrect_loft` (stacked rounded-rect loft → tapered shoulder + small deck) | tall blow-molded rounded-rect jug, drawn-in shoulder | converged(parent) |
| tall_rounded_rect (dup) | rec_gold-plastic-engine-oil-jug-with-a-black-screw-c_20260612_114121_711041_47fb268a (parent 002) | `body`/`jug_shell`, helper `_loft_rrects` (full-width top slab, corner neck) | tall rounded-rect oil jug, square shoulder slab | converged(parent) |
| square_cubic | rec_black-plastic-square-jerrycan-with-a-round-screw_20260612_114133_356061_1078fc9e (parent 003) | `body`/`body_shell`, filleted `cq.box` + inset `shoulder` slab | near-cubic chunky square pail, rounded vertical edges | converged(parent) |
| tall_rectangular_sloped | rec_black-plastic-jerrycan-with-a-screw-cap-and-a-bu_20260612_114145_800343_ab019c96 (parent 004) | `body`/`jug_body`, helper `_left_wedge_cutter` (sloped shoulder to high plateau) | tall rectangular can, peaked/sloped shoulder | converged(parent) |

### Slot B:handle / grip mechanism
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| integrated_loop | rec_white-plastic-gallon-jug-with-a-screw-cap-and-an_20260612_114111_535190_0f3f18ac (parent 001) | helper `_handle_solid` (raised rounded loop + finger-hole cut), fused into `jug_shell` | separate raised top loop, real finger hole, FIXED into body | converged(parent) |
| flush_dgrip | rec_gold-plastic-engine-oil-jug-with-a-black-screw-c_20260612_114121_711041_47fb268a (parent 002) | `grip_hole` ellipse cut through full Y depth of the shoulder slab (`_bottle_body`) | flush oval finger hole, grip bar flush in silhouette | converged(parent) |
| bridge_strap | rec_black-plastic-square-jerrycan-with-a-round-screw_20260612_114133_356061_1078fc9e (parent 003) | `bar` strap + `tunnel` finger gap over recessed deck (`_body_solid`) | top-mounted arched strap with open finger gap beneath | converged(parent) |
| recessed_top_slot | rec_black-plastic-jerrycan-with-a-screw-cap-and-a-bu_20260612_114145_800343_ab019c96 (parent 004) | helper `_rounded_slot_cutter` (front-to-back through-slot leaving a grip bar in the high plateau) | molded carry slot cut in the high shoulder plateau | converged(parent) |
| swing_bail | rec_container_plastic_can_var_swing_bail | planned: `bail`/`bail_handle` part + side `mount_lug_i` (`for i in range(2)`) on shoulder, `bail_hinge` REVOLUTE | separate U-bar carry handle that swings up/folds flat (moving handle) | converged |
| recessed_grip_pocket | rec_container_plastic_can_var_recessed_grip_pocket | planned: concave `grip_pocket` scoop boolean-cut into upper side wall of `jug_shell` (no through-hole, no loop) | sunken molded side hand-hold pocket | converged |

### Slot C:cap / closure mechanism
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| screw_cap | all 4 parents | `cap_carrier` + `cap`, `cap_rotate` (CONTINUOUS +Z) + `cap_slide` (PRISMATIC +Z), `_cap_mesh` knurled cap | round threaded screw cap, spins then lifts off | converged(parent) |
| flip_top_spout | rec_container_plastic_can_var_flip_top_spout | planned: fixed `cap_base` collar + raised spout, hinged `flip_lid` on `lid_hinge` REVOLUTE | snap flip-top lid swings open on a hinge, captive | converged |
| hinged_tethered_lid | rec_container_plastic_can_var_hinged_tethered_lid | planned: `neck_collar` + captive `tether_cap` joined by living-hinge strap, `tether_hinge` REVOLUTE | one-piece tethered cap swings open about the strap | converged |

## Multiplicity / Copy Logic
- count_param: 无小类级 multiplicity 轴 — 核心结构为固定 named slots (body / handle / closure).
- 部件内复制(非小类轴,不作独立 slot): every parent emits cap knurl ribs via `for i in range(n)`
  (`_cap_mesh`); the planned `swing_bail` variant emits its two `mount_lug_i` via a `for i in range(2)`
  loop with a shared lug helper and a uniform revolute policy. These rib/lug counts are module-internal
  parameters, not a 小类 N axis.
- N 样本已覆盖: 无
- 模板建议 N_range: 无(无 multiplicity 轴)
- copied object / naming / placement / joint policy: 无 (no per-N copied sub-part at the 小类 level)

## 组合数预审
组合数预审: Π(Slot A 4 × Slot B 6 × Slot C 3) × N(无) = 72 ≥ 10 ✓
(Slot A 计 4 候选 with `tall_rounded_rect` doubly sampled; conservatively counting 3 distinct A shape
families still gives 3 × 6 × 3 = 54 ≥ 10 ✓. Every slot has ≥2 candidates.) pattern = parallel_children.

## 排除项(未来 compatibility matrix 素材)
- 无 multiplicity 轴: this 小类 has no "N identical sub-parts" copy logic in the real object (a can has
  one body, one handle, one closure); the count-axis is honestly excluded rather than padded with a fake N.
- Slot A `tall_rounded_rect` is sampled by two parents (001, 002) which differ mainly in shoulder/handle,
  not body family — no extra A variant is forked (would be a duplicate cell or a scale-only diff).
- Pure size/proportion (taller/wider/flatter), wall thickness, and color/material (black vs gold vs white)
  are model continuous parameters / cosmetics, not slots; no variants spent on them.
- Potential cross-axis interface risk to revisit downstream: `bridge_strap` / `recessed_top_slot` handles
  occupy the top deck where the closure also sits — pairing either with a tall hinged `flip_top_spout` may
  contend for deck clearance (future compatibility-matrix check); not blocking, no combo-probe variant forked.
