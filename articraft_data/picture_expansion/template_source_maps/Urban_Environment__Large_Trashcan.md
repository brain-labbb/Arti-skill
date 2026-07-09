# Urban Environment / Large Trashcan — template source map

slug: `large_trashcan`  ·  shard: `Large_Trashcan`  ·  picdir: `picture/Urban Environment/Large Trashcan`

identity: a large wheeled curbside / commercial trash bin — hollow body + one-or-more rear-hinged flip lid(s) (REVOLUTE) + ground wheels/casters (CONTINUOUS) + a lift interface (top grab handle / side grab handles / front DIN lift-comb trunnion bar). Variants stay wheeled trash bins.

pattern: mixed (parallel_children wheels/casters + lid-count multiplicity, each lid a real REVOLUTE; body shell + lift interface as inlined parent visuals)

## Parents (2 — both loop-clean)

- **P1 wheelie** `rec_gray-two-wheel-curbside-wheelie-trash-bin-lavex-_20260608_164516_642420_a2aacece` ← `picture/Urban Environment/Large Trashcan/001.png`
  - gray ~240 L two-wheel curbside wheelie cart; tapered hollow plastic shell; thick top rim; vertical front reinforcing ribs; recessed top grab lip on lid; rear molded axle housing + fixed steel axle bar.
  - parts/joints: `body` (root) + `axle` visual; `lid` (single, REVOLUTE `lid_hinge`, axis −Y, slightly domed flip lid w/ front grab lip); `left_wheel`/`right_wheel` (CONTINUOUS `*_spin`, axis +Y, WheelGeometry+TireGeometry).
  - fills SlotA `two_rear_wheels`, SlotB `single_domed_lid`, SlotC `top_grab_handle`, SlotD `tapered_plastic`. converged (parent)
- **P2 1100L** `rec_large-commercial-waste-container-1100-liter-four_20260608_170111_720968_f6b001c4` ← `picture/Urban Environment/Large Trashcan/002.png`
  - 1100 L four-caster steel commercial container; tall boxy ribbed shell (horizontal corrugations); thick top rim; front lifting comb (DIN trunnion bar on gussets); side grab handles; four swivel-style casters on boss+pillar+fork.
  - parts/joints: `body` (root) + 4×`axle_*` visuals; `lid_left`/`lid_right` (split twin, each REVOLUTE `lid_*_hinge`, axis −Y); `caster_fl/fr/rl/rr` (CONTINUOUS `*_spin`, axis +Y).
  - fills SlotA `four_casters`, SlotB `split_twin_lid`, SlotC `front_lift_comb` (+ `side_grab_handles`), SlotD `boxy_steel`. converged (parent)

## Slot 候选覆盖

### Slot A: ground / wheel count N (滚动接地)
| 候选 (future module) | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| two_rear_wheels | P1 wheelie | left_wheel / right_wheel (CONTINUOUS *_spin) | 后轴两轮 + 固定钢轴 + 模制轴座 | converged (parent) |
| four_casters | P2 1100L | caster_fl/fr/rl/rr (CONTINUOUS *_spin) | 底四角短转向柱 boss+pillar+fork 上的脚轮 | converged (parent) |
| four_caster_base | rec_large_trashcan_var_four_caster_base | caster_* (CONTINUOUS, 4-pos loop) | wheelie 车身改坐四脚轮(2→4),位置列表驱动数量 | converged (workbench) — fills four_casters on wheelie body |
| six_caster_long | rec_large_trashcan_var_six_caster_long | caster_* (CONTINUOUS, 6-pos loop) | 1100L 加中部一对脚轮(4→6),长箱体中轴 | converged (workbench) — EMPTY cell `six_casters` |

### Slot B: lid type / lid count multiplicity (后铰翻盖)
| 候选 (future module) | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| single_domed_lid | P1 wheelie | lid (REVOLUTE lid_hinge) | 单片微拱翻盖 + 前抓边 | converged (parent) |
| split_twin_lid | P2 1100L | lid_left / lid_right (REVOLUTE) | 左右对开双半盖,各自 hinge | converged (parent) |
| split_twin_lid (on wheelie) | rec_large_trashcan_var_split_twin_lid | lid_left/lid_right (REVOLUTE, L/R sign loop) | wheelie 单盖→中线对开双半盖(1→2 lid) | converged (workbench) |
| triple_split_lid | rec_large_trashcan_var_triple_split_lid | lid_{0,1,2} (REVOLUTE, 3-slot loop) | 1100L 顶面三片横向分盖(2→3 lid) | converged (workbench) — EMPTY cell `triple_split` |
| single_flat_lid | rec_large_trashcan_var_single_flat_lid | lid (REVOLUTE, single full-width) | 1100L 双盖合并为一整片全宽平盖(2→1 lid) | converged (workbench) — EMPTY cell `single_full_width` |

### Slot C: lift interface (起吊/抓取接口)
| 候选 (future module) | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| top_grab_handle | P1 wheelie | lid front grab lip / recessed top grip | 盖前抓边 + 顶部凹握 | converged (parent) |
| front_lift_comb | P2 1100L | comb trunnion bar + gussets (parent visuals) | 前面横向 DIN 起吊横杆 + 连接板 | converged (parent) |
| side_grab_handles | P2 1100L | side handle slabs (parent visuals) | 两侧面外凸抓手 | converged (parent, co-present) |
| front_lift_comb (on wheelie) | rec_large_trashcan_var_front_lift_comb | comb bar + gussets (inlined parent visuals) | wheelie 顶握→前面起吊横杆接口 | converged (workbench) |
| side_grab_handles (on wheelie) | rec_large_trashcan_var_side_grab_handles | handle_{l,r} (L/R loop, parent visuals) | wheelie 顶握→两侧对称抓手 | converged (workbench) |

### Slot D: body profile (车身轮廓)
| 候选 (future module) | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| tapered_plastic | P1 wheelie | body shell (loft taper + vertical front ribs) | 上宽下窄锥形塑壳,竖向前肋 | converged (parent) |
| boxy_steel | P2 1100L | body shell (near-vertical + horizontal corrugations) | 近直壁方箱,横向波纹肋 | converged (parent) |
| boxy_body_profile (on wheelie) | rec_large_trashcan_var_boxy_body_profile | body shell + rib_{i} (height-list loop) | wheelie 锥形→方箱横波纹轮廓 | converged (workbench) |

## Multiplicity / Copy Logic
- count_param:
  - `wheel_count` / `caster_count` — parents cover N ∈ {2 (P1 axle wheels), 4 (P2 casters)}; variants add {4 (four_caster_base), 6 (six_caster_long)} → distinct-N {2,4,6}. Both parents emit via position-list / sign loop (`for name,sign`, `for sx in ±1: for sy in ±1`).
  - `lid_count` — parents cover N ∈ {1 (P1), 2 (P2)}; variants add {2 (split_twin on wheelie), 3 (triple_split), 1 (single_flat on 1100L)} → distinct-N {1,2,3}. P2 emits lids via L/R sign loop; triple_split moves to 3-slot list loop.
  - `rib_count` — P2 horizontal corrugations `for zc in (...)`; P1 vertical front ribs `for yy in (...)`; boxy_body_profile re-emits horizontal ribs via height list (count = sampler's job, not a fork).
- copied object / naming / placement / joint policy:
  - copied object: wheels/casters (`caster_{tag}` / `left_wheel`/`right_wheel`), lid panels (`lid_left`/`lid_right`/`lid_{i}`), corrugation ribs, side handles.
  - naming: position-list / sign loop + `f"{name}_{tag}"` (caster_fl…) or `f"lid_{i}"`; ribs/handles in `for ... in (...)`.
  - placement: casters at bottom corners (+ mid pair for 6); lids split equally across width at shared rear hinge X/Z; ribs equally spaced up the face; handles mirrored across ±Y.
  - joint policy: each wheel/caster a CONTINUOUS spin (axis +Y); each lid a REVOLUTE (axis −Y, opens up/rearward); lift comb / side handles / top grip are inlined parent visuals (NO fixed-joint decoration parts).

## COMBO PRE-AUDIT (HARD GATE)
- Structural candidate slots: A (ground count) ×3, B (lid type/count) ×3, C (lift interface) ×3 [+ D body profile ×2].
- product(candidates over A,B,C) = 3 × 3 × 3 = **27**.
- distinct-N: wheel count {2,4,6} (3) and lid count {1,2,3} (3).
- **27 ≥ 10 → PASS.**

## 排除项 (compatibility-matrix material; not forked)
- color / material / pure-scale — NOT structural axes (free to layer on top).
- rib / corrugation count N — not forked: both parents loop-emit multiple ribs; template seed sweep covers the count axis (sampler's job).
- the swivel kingpin (caster swivel about its vertical stem) is modeled as fixed-stem geometry in P2, not a live joint — keep that idiom; do NOT expand into a real swivel DOF.
- cross-slot combos (e.g. boxy body × six casters × triple lid) left to the template sampler, not hand-forked.
- dropped axis: standalone `single_flat_lid` on the wheelie (P1 already IS single-lid) — the single-lid candidate is instead exercised on the 1100L parent (`single_flat_lid` = 2→1 merge) to add real coverage; no wheelie single-lid no-op variant.

---
## 8 variant 填格情况 (planned, workbench-only, sweep pending)
- var_four_caster_base   → Slot A `four_casters` on wheelie body (2→4, position-list loop)
- var_six_caster_long    → Slot A `six_casters` (EMPTY cell; 1100L 4→6, mid pair)
- var_split_twin_lid     → Slot B `split_twin_lid` on wheelie (1→2 lids, L/R sign loop)
- var_triple_split_lid   → Slot B `triple_split` (EMPTY cell; 1100L 2→3 lids, 3-slot loop)
- var_single_flat_lid    → Slot B `single_full_width` (EMPTY cell; 1100L 2→1 merged full-width lid)
- var_front_lift_comb    → Slot C `front_lift_comb` on wheelie (top grip → DIN comb trunnion bar)
- var_side_grab_handles  → Slot C `side_grab_handles` on wheelie (top grip → two side handles, L/R loop)
- var_boxy_body_profile  → Slot D `boxy_body_profile` on wheelie (tapered → boxy horizontal-corrugation shell)
