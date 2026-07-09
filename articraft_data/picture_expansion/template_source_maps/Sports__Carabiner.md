# Sports / Carabiner — template source map

pattern: parallel_children

parents:
- rec_stainless-steel-spring-snap-hook-carabiner-with-_20260605_165816_513107_550acf8b ← picture/Sports/Carabiner/001.png — fills Slot A = pear/teardrop body AND Slot B = straight solid gate. Single parent; every variant forks from it (smallest clean diff, one axis each).

The object has two real structural axes: the bent-bar BODY frame outline, and the GATE closure mechanism (the swinging revolute member). There is no multiplicity sub-part — a carabiner is one body + one gate (the screw-lock variant adds one extra sliding sleeve, not a copied N). Π = 4 (body) × 4 (gate) = 16 ≥ 10; no N axis needed.

## Slot 候选覆盖

### Slot A: body frame form (the bent round-bar open-hook outline)
| 候选 (future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pear_teardrop (HMS) | rec_stainless-steel-spring-snap-hook-carabiner-with-_20260605_165816_513107_550acf8b | body / body_loop (tube_from_spline_points), nose_lug | asymmetric pear: broad rounded top bend, narrow bottom eye; parent baseline | converged (parent) |
| oval_symmetric | rec_carabiner_var_oval | body / body_loop | left-right symmetric egg/oval; equal top & bottom bend radius; taller than wide | built ✓ |
| d_shape | rec_carabiner_var_dshape | body / body_loop | climbing-D: flat gate-side back + bowed spine, quarter-circle end corners | built ✓ |
| offset_d | rec_carabiner_var_offsetd | body / body_loop | asymmetric/offset-D: broad top bearing bend tapering to pinched bottom eye, slanted gate-side back | built ✓ |

Body outline is authored as the centerline point list fed to `tube_from_spline_points` (helper `_body_hook_mesh`); a module swaps the point list + bend radii, keeping the same open-hook topology (nose free-end + hinge free-end, straight gate gap on -X). NOT a scale change — the outline family changes.

### Slot B: gate closure mechanism (the swinging revolute member + latch interface)
| 候选 (future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| straight_solid_gate | rec_stainless-steel-spring-snap-hook-carabiner-with-_20260605_165816_513107_550acf8b | gate / gate_bar (CadQuery capsule), gate_latch, hinge_pin; joint gate_hinge (REVOLUTE, axis Y); body nose_lug | solid round bar flush on -X line, latch notch top seats on nose lug; parent baseline | converged (parent) |
| wire_gate | rec_carabiner_var_wiregate | gate / gate_wire_loop (thin bent-wire U), hinge_pin; joint gate_hinge (REVOLUTE, axis Y) | slim spring-wire hairpin loop instead of fat bar; same hinge + nose seating; lighter/thinner | built ✓ |
| bent_gate | rec_carabiner_var_bentgate | gate / gate_bar (bent near tip) + gate_tongue; body nose_slot; joint gate_hinge (REVOLUTE, axis Y) | solid bar with a bend near the tip dropping a notched tongue into a mating slot cut in the nose (positive hook-and-slot) | built ✓ |
| screw_lock_sleeve | rec_carabiner_var_screwlock | gate / gate_bar + lock_sleeve (knurled hollow barrel); joints gate_hinge (REVOLUTE, axis Y) + lock_slide (PRISMATIC along gate axis) | adds a second moving part: sleeve slides on the gate bar to bridge/clear the gate-nose seam = lock. TWO non-fixed joints | built ✓ |

Gate module owns the `gate` part, the `gate_hinge` revolute joint (axis = loop normal Y, origin at the bottom hinge rivet contact on the body bar), the latch/nose mating interface, and any extra locking joint. The hinge origin and the nose-seat contact face are the cross-slot interface points (body provides the hinge-rivet boss face + nose lug/slot; gate consumes them). screw_lock_sleeve is the candidate that proves a viable SECOND joint exists in this family.

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots (one body + one gate). 没有 N-identical 子件。
- N 样本已覆盖: 无 (no multiplicity axis)。
- 模板建议 N_range: 不适用。如未来要表达 wire_gate 的双平行 wire legs,用一个固定 2-leg 的 `for i in range(2)` 局部 helper,而不是把它当一根 N 轴。
- copied object / naming / placement / joint policy: 仅 wire_gate 内部的两根 wire legs 走 `for i in range(2)` + `gate_wire_leg_{i}` 共享 helper、镜像 placement、同一 gate part 上无独立 joint (legs 是同一 gate visual);整体 carabiner 无跨件复制逻辑。

## 排除项 (future compatibility matrix material)
- 暂无确认的不收敛组合 (P0 planning only — none forked yet)。
- 预期注意点 (P0 forecast, 待 fork 验证):
  - screw_lock_sleeve × wire_gate: 螺锁套需要一根连续实心 gate 杆来承托滑动,wire 细环上挂滑套会穿模 → 该跨轴组合预期不兼容,锁套候选默认绑 straight_solid_gate body baseline;若 fork 失败记此处。
  - bent_gate 的 nose_slot 是对 body nose_lug 的一次接口改动 (lug → lug+slot),属同一 gate-closure 接口的配套面,不算第二根轴;若与非-pear body 的 nose 位置不匹配再记排除。

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
