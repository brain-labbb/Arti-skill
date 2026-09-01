# Handtools / clothes peg — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-clot_20260609_164008_024673_add41790 ← picture/Handtools/clothes peg/001.png (classic wooden spring clothespin: two mirror wooden halves, coiled steel torsion spring at the pivot, flat-notched gripping jaws, flared finger pads)

Spring clothespin. Core kinematics shared by all candidates: a `lower_half` (root, rests
flat on the ground) and an `upper_half` that PIVOTs against it about the spring-barrel axis
via a single REVOLUTE joint (`pivot`, axis (0,-1,0) in the joint frame = world vertical;
limits [0, PIVOT_MAX]). A one-piece spring part is rigidly mounted to the root via the
`lower_to_spring` FIXED joint and its arms/legs ride the relieved inner tail faces, pressing
the jaws closed at rest. Both wooden halves are built from a single shared `_wood_half()`
CadQuery solid (one geometry helper, placed twice). The three independent structural slots —
the spring/pivot mechanism, the gripping-jaw shape, and the tail/grip-end shape — are below.

Two-arm emission note: the two wooden legs are the two `parallel_children` of the pivot and
carry DIFFERENT joint roles (root vs. revolute child), so they are correctly authored as two
named parts (`lower_half`/`upper_half`) sharing one `_wood_half()` solid — not a copy loop.
Where a candidate has genuinely symmetric repeated sub-parts it DOES loop: `rounded_jaw`
emits the halves as `half_{i}` via `for i in range(2)`; `leaf_spring` emits its two spring
arms as `arm_{i}` via `for i in range(2)` + shared `_leaf_spring_arm(sign)` helper. The
torsion-spring candidates emit the coil as a single one-piece swept tube (no loop needed).

## Slot 候选覆盖

### Slot A:spring / pivot mechanism (the part fixed to the root via `lower_to_spring`, biasing the `pivot` joint)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| torsion_coil | rec_build-...-clot_...add41790 (parent) | `spring` part; `_spring_mesh()` (`tube_from_spline_points`, ~2-turn coil on pivot axis + 2 straight legs); round `SEAT_R` barrel seat carved by `_wood_half()` | coiled steel torsion spring, one-piece swept wire tube wound around the pivot axis; legs ride the relieved tail faces | converged |
| leaf_spring | rec_clothes_peg_var_leaf_spring | `leaf_spring` part with `arm_0`/`arm_1`/`bend` visuals; helpers `_leaf_spring_arm(sign)` + `_leaf_spring_bend()`; rectangular `SEAT_L×SEAT_W×SEAT_D` box seat | flat bent steel strip: two tilted box arms + a short connecting bend instead of a coil; arms emitted via `for i in range(2)` | converged |

### Slot B:jaw / gripping-tip shape (front end of `_wood_half()`, ahead of the pivot)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_notched | rec_build-...-clot_...add41790 (parent) | `_wood_half()` flat parting face (pts to `NOSE_X`) + half-round `groove` cut at `NOSE_X-0.009` | flat parallel jaw faces with one carved clothesline groove behind the tip | converged |
| rounded_barrel | rec_clothes_peg_var_rounded_jaw | `_wood_half()` `BARREL_R` arc (`arc_pts` loop) forming a half-round nose; no groove cut | smooth half-round barrel-head jaw; two halves close into a full cylindrical gripping head | converged |
| toothed_serrated | rec_clothes_peg_var_toothed_jaw | `_wood_half()` + `_cut_serrated_teeth()` (`for i in range(_N_TEETH)` V-groove prisms); constants `TOOTH_PITCH`/`TOOTH_DEPTH`/`TEETH_START_X`/`TEETH_END_X` | row of shallow V-groove serrations across the parting face ahead of the pivot; raised teeth bite the line | converged |

### Slot C:tail / grip-end shape (back/finger end of `_wood_half()`, behind the pivot)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flared_pad | rec_build-...-clot_...add41790 (parent) | `_wood_half()` back profile pts ending at flared `z_back=0.0095` finger pad | rounded outward-flared flat finger pad at the tail | converged |
| dished_thumb | rec_clothes_peg_var_dished_tail | `_wood_half()` raised pressing pad (`PAD_BULGE`) + spherical `dish_sphere` cut (`DISH_R`/`DISH_DEPTH`); constants `Z_BACK`/`PAD_BULGE`/`DISH_R`/`DISH_DEPTH` | raised pad with a concave spherical thumb depression carved into the outer face | converged |
| square_stub | rec_clothes_peg_var_square_tail | `_wood_half()` back profile ending at blunt flat-top `z_back=0.0075` stub (no flare/pad) | plain blunt square finger stub, no flare or pad | converged |

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots(spring / jaw / tail,外加 root+moving 两块木腿)。无 "× N 同构子件" 复制逻辑。
- N 样本: 无 multiplicity 轴。
- copied object / naming / placement / joint policy:
  - 两块木腿共用一个 `_wood_half()` solid,但因角色不同(`lower_half` root / `upper_half` revolute child)是两份具名 part,**不是**复制循环 —— 正确。
  - 真正的对称重复子件用循环:`rounded_jaw` 的 `half_{i}`(`for i in range(2)`)、`leaf_spring` 的 `arm_{i}`(`for i in range(2)` + 共享 `_leaf_spring_arm(sign)` helper、对称 placement、统一 visual-on-part policy)。
  - torsion_coil 弹簧是一根 `tube_from_spline_points` 一体扫掠管(coil + 两条腿),无需循环。

## 组合数预审
Slot A(2) × Slot B(3) × Slot C(3) = 18 ≥ 10 ✓。每个 slot ≥2 候选。pattern = parallel_children,无 multiplicity。
注意:Slot A(spring/pivot 机构)只有 2 个候选(parent 的 torsion_coil + leaf_spring)——满足每槽 ≥2 的底线,但为本批最薄的一槽;真实世界 clothes peg 弹簧词汇表本身有限(扭簧 / 板簧为主流,其余多为尺寸/材质连续参数,不入 slot),组合数已由 B×C 撑到 18。若下游模板要加厚 Slot A,可考虑塑料一体活铰(living-hinge,无独立弹簧件)作为第三候选,但其拓扑会改动 `pivot`/`lower_to_spring` 关节结构,需单独 fork 验证后再入池。

## 排除项(未来 compatibility matrix 素材)
- 无连续不收敛取值;5 个变体全部 compile + run_tests 通过,各含恰好 1 个非 fixed joint(`pivot` REVOLUTE),`lower_to_spring` 为 FIXED。
- 跨轴组合(如 leaf_spring × toothed_serrated × dished_thumb)未抽检 —— 组合由模板采样器生成,非样本池义务。三轴在几何上彼此正交(前端 jaw / 后端 tail / 中部 spring seat 互不干涉),无已知接口冲突。
- 纯尺寸(腿长 LEG_LEN、宽 HALF_W、关节上限 PIVOT_MAX、弹簧线径/半径)不作为候选 —— 属模板连续参数(controlled local parameterization),不入 slot。
