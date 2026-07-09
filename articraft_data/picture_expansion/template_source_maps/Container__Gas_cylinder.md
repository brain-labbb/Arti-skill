# Container / Gas cylinder — template source map

pattern: mixed（固定 named slots:top_guard(主装饰/保护) + valve_closure(主机构) + base；**外加** cage_bar_count 多重性轴，归属 top_guard 的 vented_cage_shroud 候选）

parents:
- P_lpg rec_red-lpg-gas-cylinder-with-a-top-valve-handwheel-_20260606_074732_769479_5d5b07e2 ← picture/Container/Gas cylinder/001.png（红色 LPG 钢瓶：lathe 钢瓶身 + 域顶，brass 阀体含侧出气嘴，star handwheel REVOLUTE 含 off-axis lug，bare-steel 环形护圈 ring-on-4-struts，dark foot ring 底座）。占基线格子：Slot A=ring_on_struts，Slot B=handwheel_valve，Slot C=foot_ring。

母资产代码形态审查（§4 可读性契约）：
- 分层命名良好：part `body` / `foot_ring` / `valve` / `handwheel`，具名 visual `body_shell` / `collar_guard` / `hazard_label` / `valve_body` / `foot_ring`。直接映射到 slot 名。
- 复制逻辑已用循环：collar 四根 struts (`_collar_mesh` 中 `for i in range(4)`) 与 handwheel 四根 spokes (`_handwheel_mesh` 中 `for i in range(4)`) 均循环发射。**但它们 merge 进单一 mesh、非独立 part/joint**——作为 visual 内部复制可读，但小类级 multiplicity 轴需要在 vented_cage_shroud 候选里把 bar 提为可循环计数的实体（fork prompt 已显式要求 `for i in range(n)` + `name_i` + 共享 helper + 等角 placement）。
- primitive 保真：身体用 LatheGeometry，阀体用 CadQuery 布尔并集，护圈/手轮用 Torus+Cylinder geometry merge——曲面/复合形未降级为 Box。
- 装饰内联：hazard_label 与 collar_guard 都是 `body.visual(...)`，无 FIXED-joint 装饰 part。✓
- 活动锚定：handwheel REVOLUTE 锚在 valve 阀杆顶（真实承托面），expect_contact 校验。✓
- 唯一活动关节 = valve_to_handwheel (REVOLUTE，竖直阀轴)。所有非 valve-closure 轴的变体必须保留它；valve_closure 轴的变体改的就是这个机构本身（换 lever / screw-cap，仍 ≥1 非 fixed joint）。

批次（计划，未 fork）：container_gas_cylinder（建议 dashscope qwen3.7-max / medium，与同批小类统一）。

## Slot 候选覆盖

### Slot A:top_guard（阀顶护圈/护罩；含 multiplicity 候选）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| ring_on_struts | P_lpg (parent) | `body.collar_guard` visual / `_collar_mesh` (torus ring + `for i in range(4)` struts) | 环形护圈 + 4 短支柱，开放式 | converged(parent) |
| solid_neck_collar | rec_container_gas_cylinder_var_solid_neck_collar | `collar_guard`(实心 lathe 杯形领圈) | 一体冲压焊接钢领圈，连续壁包裹阀 | converged |
| carry_handle_arch | rec_container_gas_cylinder_var_carry_handle_arch | `carry_handle`(倒 U 弯杆) / 两 anchor pad | 高拱提手弓形护罩，可提可护 | converged |
| vented_cage_shroud | rec_container_gas_cylinder_var_vented_cage_shroud | `cage_bar_{i}` (`for i in range(n)`, 共享 bar helper, 等角) + `cage_top_ring` | N=4 竖条+顶环的开槽护笼（multiplicity 默认 N） | converged |

### Slot B:valve_closure（**主机构槽**——被操作的开关件，承载非 fixed joint）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| handwheel_valve | P_lpg (parent) | `handwheel` part / `valve_to_handwheel` REVOLUTE(竖直阀轴) / `_handwheel_mesh`(hub+torus rim+`for i in range(4)` spokes+off-axis lug) | 星形手轮绕竖直阀轴旋转，off-axis lug 使旋转可检测 | converged(parent) |
| lever_clip_valve | rec_container_gas_cylinder_var_lever_clip_valve | `valve_lever` part / `valve_to_lever` REVOLUTE(水平轴) | 自闭式扳手阀，水平杆绕水平轴下压开/弹回闭 | converged |
| screw_bonnet_cap | rec_container_gas_cylinder_var_screw_bonnet_cap | `bonnet_cap` part / `valve_to_cap` REVOLUTE(竖直阀轴，旋拧下降) | 螺纹防护螺帽罩，旋拧绕竖直轴密封阀 | converged |

### Slot C:base（底座/支脚）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| foot_ring | P_lpg (parent) | `foot_ring` part / `body_to_foot` FIXED / `_foot_mesh`(lathe 裙环) | 深色钢制裙状脚环，套在瓶底边 | converged(parent) |
| concave_recessed_base | rec_container_gas_cylinder_var_concave_recessed_base | `body_shell` lathe profile 内凹底 + 外缘支承环（无独立脚环 part） | 内凹碟形底，靠一体外缘 rim 站立 | converged |
| n_feet_base | rec_container_gas_cylinder_var_n_feet_base | `base_foot_{i}` (`for i in range(n)`, 共享 foot helper, 等角) / `body_to_foot_{i}` FIXED | 模塑底 + N=3 短脚墩等角分布 | converged |

## Multiplicity / Copy Logic
- count_param: **`cage_bar_count`**（vented_cage_shroud 护笼竖条数；归属 Slot A 该候选，非 footprint/closure 轴）
  - N 样本已覆盖: {4, 6, 8} → vented_cage_shroud(N=4 默认) / rec_container_gas_cylinder_var_cage_n6 (N=6) / rec_container_gas_cylinder_var_cage_n8 (N=8)
  - 模板建议 N_range: [3, 12]（采样域可远大于样本覆盖；护笼竖条数现实区间约 3–12）
  - copied object: 单根竖直 `cage_bar`（共享 bar geometry helper）；naming `cage_bar_{i}`；placement 绕阀轴等角分布在固定半径环上，顶端接 `cage_top_ring`；joint policy 全部 FIXED 到 body（随 body 动，非独立活动）。
- 次要副本逻辑（非小类级 multiplicity 轴，仅候选内部）：
  - n_feet_base 候选内有 `foot_count`（脚墩数），fork 样本 N=3；建议 foot N_range [3, 4]，等角 FIXED 到 body 底缘。
  - parent 的 collar struts (4) 与 handwheel spokes (4) 是 visual 内部循环复制，不提为小类 multiplicity 轴。

## 组合数预审
组合数预审: Slot A(4) × Slot B(3) × Slot C(3) × cage_bar_count N{4,6,8}=3 = 108 ≥ 10 ✓
（即便剔除 multiplicity，基础 4×3×3 = 36 ≥ 10 已稳。每个 slot ≥2 候选。主机构槽 = valve_closure 含 REVOLUTE 手轮/扳手/螺帽三机构。pattern = mixed。）

## 待填格子推导（§2 批次规模）
parent 免费占每槽基线格 + 默认 N=4。空格：
- Slot A: 3（solid_neck_collar / carry_handle_arch / vented_cage_shroud）
- Slot B: 2（lever_clip_valve / screw_bonnet_cap）
- Slot C: 2（concave_recessed_base / n_feet_base）
- multiplicity cage_bar_count: 2（N=6 / N=8；N=4 由 vented_cage_shroud 本体覆盖）
cells = 3+2+2+2 = 9 → 计划 9 个变体（一格一个，无组合枚举，无尺寸/配色凑数）。

## 排除项（未来 compatibility matrix 素材）
- footprint/截面形状家族未作独立轴：真实燃气钢瓶几乎都是圆柱旋转体（方形 LPG 瓶罕见且易出类目），故不设 round↔square footprint 轴；瓶身高矮/胖瘦是模板连续参数（controlled local parameterization），不入 slot。
- 颜色/材质（红/蓝/灰钢瓶、brass↔chrome 阀）鼓励在结构变化之上自由叠加，永不算结构轴。
- 侧出气嘴 spigot 朝向/有无属 valve 候选内部细节，未单列为轴（信息量不足以撑一个 slot）。
- 仅有单图单母资产：所有候选均从同一 parent fork，基线一致、diff 干净；若后续补入更多 Gas cylinder 参考图，可在 Slot A/C 追加候选并按本节回填格子。
