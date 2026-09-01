# Handtools / Stapler — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-stap_20260609_163946_295867_3c83c7d2 ← picture/Handtools/Stapler/001.png (charcoal/dark-gray half-strip desktop stapler; rounded hump cover, exposed rear knuckle-and-pin hinge). Covers baseline cell of all three slots.

Desktop stapler. Core kinematics shared by every candidate: a `base` (root) lower
body/magazine tray and a `top_arm` magazine-cover/handle that swings up over it.
The single non-fixed joint is `base_to_arm`, REVOLUTE about the rear transverse
pivot (`origin=(HINGE_X,0,HINGE_Z)`, `axis=(0,-1,0)`, `lower=0`, `upper≈0.52`):
positive q lifts the front driver blade upward to open the stapler. Every variant
preserves this part tree (`base` / `top_arm`), this joint name, and these shared
visuals: `base_body`, `anvil_plate`, `carrier_rail`, `driver_blade`. The body
silhouette, the rear hinge hardware, and the top cover shape are the three
independent structural slots below.

## Slot 候选覆盖

### Slot A:body type / lower-body silhouette (`base` root geometry)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| half_strip_tray | parent (..._stap_..._3c83c7d2) | `base_body` (lofted XZ side profile, rounded toe), `base_feet` | classic desk tray ~0.158 m, low deck + rear heel | converged |
| compact_stubby | rec_stapler_var_compactbody | `base_body` (short profile), `foot_pad_{i}` loop, `hinge_knuckle_{i}` loop | mini stapler ~0.092 m, blunt toe, tall wall/length ratio | converged |
| full_strip_tray | rec_stapler_var_fullstrip | `base_body` (elongated flat deck), `tray_liner`, `foot_{i}` loop (4 feet) | ~0.280 m long flat office tray, full-strip channel liner | converged |
| plier_grip | rec_stapler_var_plierbody | `base_grip` (swept elliptical section on XZ spline path), `grip_ridge_{i}` loop, anvil platform | curved ergonomic hand-squeeze grip dropping below pivot (replaces flat tray) | converged |

### Slot B:hinge mechanism (rear pivot hardware on `base` + `top_arm` rear)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| exposed_knuckle_pin | parent (..._stap_..._3c83c7d2) | `hinge_knuckles` (base, bored ring + skirt), `hinge_pin` (arm), arm rear cheeks | visible interleaved knuckle ring + transverse pin at the rear | converged |
| boxed_housing | rec_stapler_var_boxedhinge | `rear_housing` (base, U-channel enclosure), arm central rear tongue; NO `hinge_knuckles`/`hinge_pin` | tall boxed plastic housing hides the pivot; arm tongue pivots inside cavity | converged |
| torsion_spring | rec_stapler_var_springhinge | `hinge_knuckles` + `hinge_pin` + `torsion_spring` (tube_from_spline_points helix, base/arm tangs) | exposed pin/knuckle plus a coiled return spring wrapped on the pivot | converged |

### Slot C:top cap / cover shape (`arm_shell` on `top_arm`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rounded_hump | parent (..._stap_..._3c83c7d2) | `arm_shell` (curved XZ silhouette, crown fillets, hollowed, rear cheeks) | soft pebble/dome magazine cover | converged |
| slab_cap | rec_stapler_var_slabcap | `arm_shell` (near-rectangular profile, `|X and >Z` chamfers, flat top) | crisp flat-topped rectangular slab cap with chamfered side edges | converged |
| ribbed_cap | rec_stapler_var_ribbedcap | `arm_shell` (hump) + `grip_rib_{i}` loop (5 transverse ridges, shared helper) | rounded hump crown carrying evenly-spaced molded finger-grip ribs | converged |

## Multiplicity / Copy Logic
- count_param: 无独立的小类级 multiplicity 轴;核心结构是固定 named slots(body / hinge / cap)。
  仅 cap 的 `ribbed_cap` 候选内部带一个局部复制逻辑(grip ribs),body 的若干候选带 feet 复制。
- N 样本: 无主 multiplicity 轴。局部复制示例(下游 module 内部参数,不是顶层 slot):
  - grip ribs: `grip_rib_{i}` × 5 (rec_stapler_var_ribbedcap) → `GRIP_RIB_COUNT`,等距沿 crown 排布。
  - feet: `foot_{i}` × 4 (fullstrip) / `foot_pad_{i}` × 2 (compactbody, boxedhinge),装饰性。
  - 装饰示例: `grip_ridge_{i}` × 5 (plierbody), `staple_{i}` × 12 (fullstrip,弹仓内可见钉条)。
- 模板建议 N_range: rib_count [3, 8] / staple_count [0, 24] / foot_count {2, 4}(均为 module 内部连续/局部参数,不顶替 slot 结构差异)。
- copied object / naming / placement / joint policy: 所有复制件均 `for i in range(n)` + `f"<name>_{i}"`
  + 共享几何 helper(`_single_grip_rib` / `_rubber_foot` / `_foot_pad_shape` / `_knuckle_shape` /
  `_staple_crown`)+ 等距/对称 placement;全部 FIXED 随其宿主 part 动(ribs/staples 随 arm,feet 随 base),无独立关节。

## 组合数预审
Slot A(4) × Slot B(3) × Slot C(3) = 36 ≥ 10 ✓。每个 slot ≥2 候选(A=4, B=3, C=3)。
pattern = parallel_children(每个 slot 是 base/arm 的独立结构层,共享同一个 base_to_arm 铰链),无顶层 multiplicity 轴。

## 排除项(未来 compatibility matrix 素材)
- 无连续不收敛取值:全部 7 个变体 + parent 均收敛(compile + run_tests 通过,1 个非 fixed joint base_to_arm)。
- 接口风险待裁(组合抽检候选,组合本身由模板采样器生成):
  - plier_grip(Slot A)假定 `anvil_platform` 在负 Z 区承托 anvil_plate(`ANVIL_SEAT_Z<0`),与默认 tray 的 anvil 座高(`DECK_FRONT_Z+...`)不同;plier body × 任一 cap/hinge 时 anvil/blade 的 Z 寄存高度需按 body 选择重算(body slot 决定 anvil 座面)。
  - boxed_housing(Slot B)用单中央 tongue + 宽于 arm 的外壳让 arm 跨骑外壳,删除了 `hinge_knuckles`/`hinge_pin`;arm 侧的 rear-cheek vs rear-tongue 由 hinge slot 决定,故 hinge slot 同时拥有 base 侧 hardware 与 arm 侧下伸结构(跨 base/arm 的接口,非纯 base-only)。
  - torsion_spring(Slot B)的 arm-tang 与 arm_shell rear cheek 有意接触(allow_overlap);若与 boxed_housing 互斥(spring 依赖暴露的 knuckle/cheek 几何),应在 spec 中标 incompatible(spring 仅兼容 exposed_knuckle_pin 的 cheek 形态)。

## 同步备注
- 全部变体为 workbench-only,fork 自 parent 继承 collection;同步进 arti-template 时脚本批量写 rating=5。两边均不进 dataset。
