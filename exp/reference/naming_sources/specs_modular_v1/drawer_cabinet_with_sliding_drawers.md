# Drawer Cabinet with Sliding Drawers Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `drawer_cabinet_with_sliding_drawers` |
| template path | `agent/templates/drawer_cabinet_with_sliding_drawers.py` |
| test path (optional) | — |
| stage | `TEMPLATE_AFTER_REVIEW` |
| status | `approved` |
| __modular__ | `True` |
| pattern | `mixed` |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 73 |
| read_count | 73 |
| read_scope | 类别内全部 73 个 5 星样本已汇总阅读 |
| source_index_policy | 仅索引下方采纳 module source |

结构族分布：

| 结构族 | 样本数 | 说明 |
|---|---:|---|
| 柜体 + N 抽屉 PRISMATIC 滑出 | 58 | 1–5 抽屉，+X 或 +Y 滑动 |
| 可选顶盖 REVOLUTE | 31 | hinged lid / compartment |
| bedside / uniform stack / plinth | 45 | 柜体外形 slot |
| 木质 / 烤漆 / 工业灰 palette | 73 | 材质 slot |

被采纳样本：
- `rec_drawer_cabinet_with_sliding_drawers_69cfec3f23ba4b5ab534557732594e9a` — adopted：多抽屉 tray slide（本模板简化为 +X）。
- `rec_drawer_cabinet_with_sliding_drawers_cca8a138c3d94ec7991f48f8ed3d5a53` — adopted：uniform stack 柜体。
- `rec_drawer_cabinet_with_sliding_drawers_722e43d31182424586901bd24f35143a` — adopted：plinth base。
- `rec_drawer_cabinet_with_sliding_drawers_ed911543`（锚点）— adopted：bedside 3 抽屉 + 顶盖 compartment；seed=0 复现。

## 核心身份

**落地柜体 body** 承载 **N 个 PRISMATIC 抽屉**（沿 **+X** 滑出），可选 **REVOLUTE 顶盖 lid**。抽屉数 N 为 **joint-bearing multiplicity**（每个抽屉一对 body→drawer_i 关节）。

边界：
- 不是 `desk_with_drawer` 单抽屉书桌。
- 不是 `threestage_telescoping_slide` 多级伸缩套筒。
- 不是 `chest_freezer_with_hinged_lid` 卧式冷冻柜（无多抽屉 PRISMATIC 栈）。

## 槽位 + 候选模块表

### Slot A：cabinet_body

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `bedside_compartment` | anchor | template | eligible | 上格 compartment + shelf |
| `uniform_stack` | S2 | `L80-L200` | eligible | 均匀抽屉栈 + 内 shelf |
| `plinth_base` | S3 | `L80-L180` | eligible | 踢脚 plinth |
| `wide_low` | S1 | `L80-L200` | eligible | 宽矮柜 + top molding |

### Slot B：drawer_multiplicity（joint-bearing）

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `1_drawers` … `5_drawers` | 58 samples | drawer loop | eligible | N∈[1,5]，N 个 PRISMATIC 关节 |

### Slot C：top_lid

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `hinged_lid` | anchor | `L300-L320` | eligible if has_top_lid | body→lid REVOLUTE |
| `none` | — | — | eligible if ¬has_top_lid | 无 lid part |

### Slot D：material_palette

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `warm_oak` | anchor | palette | eligible | seed=0 |
| `painted_white` | S2 | palette | eligible | — |
| `walnut_stain` | S3 | palette | eligible | — |
| `industrial_grey` | S1 | palette | eligible | — |

## 槽位图（slot graph）

pattern = `mixed`（multiplicity + optional lid）

```
[body] (grounded)
    |-- PRISMATIC +X --> [drawer_0]
    |-- PRISMATIC +X --> [drawer_1]
    |-- ... --> [drawer_{N-1}]
    |-- REVOLUTE (optional) --> [lid]
```

- 关节原点：柜体前开口平面（+X 外露面）；抽屉 child 原点 = 前板外侧面。
- 滑动轴固定 (1,0,0)。

## 参数范围汇总
| 参数 | 类型 | 取值范围 | 默认 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| `drawer_count` | int | 1–5 | 3 | Slot B | anchor |
| `cabinet_style` | enum | 4 modules | `bedside_compartment` | Slot A | anchor |
| `has_top_lid` | bool | ~50% | true | Slot C | anchor |
| `cabinet_width` | float | 0.34–0.80 | 0.52 | 派生抽屉宽 | anchor |
| `drawer_travel` | float | 0.10–0.36 | 0.24 | PRISMATIC upper | anchor |
| `material_style` | enum | 4 palettes | `warm_oak` | Slot D | anchor |

## Multiplicity / Copy Logic

| 项 | 值 |
|---|---|
| M1 `drawer_multiplicity` | **joint-bearing**；`drawer_count` ∈ [1,5]；导出 `"{N}_drawers"` |
| naming | `drawer_{i}`, `body_to_drawer_{i}` |
| placement | `drawer_center_z[i]` 垂直栈；anchor 三抽屉用固定 Z 表 |
| joint policy | PRISMATIC +X；lower=0，upper=drawer_travel |
| lid | 可选单 REVOLUTE，轴 −Y，parent=body |

## 拓扑多样性审计

总组合数：4 × 5 × 2 × 4 = **160**


seed_domain_policy：procedural_first

| item | policy |
|---|---|
| overlap policy | body↔drawer、相邻 drawer 前板共面：whole-part `allow_overlap` |
| wide_low | top_molding 须与侧板连通 |
| anchor | seed=0 复现 ed911543 比例（非特殊采样） |
| random sweep | 0–49 |

## Validator

- `drawer_count` 与 drawer part / joint 数一致
- 全部 `body_to_drawer_i` 轴 = +X
- `fail_if_articulation_origin_far_from_geometry`（前立柱 + slide rail）
- lid 仅在 `has_top_lid` 时存在

## Reject cases

- 抽屉滑动轴非 +X
- 关节原点不在柜体前面开口几何上
- N 抽屉但关节数不匹配
- 关闭姿态 drawer 与 body 非 intentional overlap 未声明
- wide_low top_molding 浮岛

## 与相邻类别的边界

- `desk_with_drawer`：单抽屉 + 桌面
- `display_freezer_with_sliding_glass_lids`：玻璃盖滑动，非木柜抽屉
- `wall_safe_with_hinged_door_and_dial`：保险箱门，非抽屉栈

## 模板实现备注（可选）

- 模板已先行实现并通过 sweep 50/50（41 distinct）。
- 前 stile + per-drawer slide rail 锚定 parent 关节原点。
- 抽屉前板朝 −X（柜内），joint 在 +X 外露面。
- element-scoped / whole-part `allow_overlap` 覆盖 cavity 共面。

## Module Source Index

| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| anchor | `rec_drawer_cabinet_with_sliding_drawers_ed911543` | template anchor | seed=0 bedside 3-drawer + lid |
| S1 | `rec_drawer_cabinet_with_sliding_drawers_69cfec3f23ba4b5ab534557732594e9a` | `L250-L400` | 多抽屉 slide |
| S2 | `rec_drawer_cabinet_with_sliding_drawers_cca8a138c3d94ec7991f48f8ed3d5a53` | `L80-L250` | uniform stack |
| S3 | `rec_drawer_cabinet_with_sliding_drawers_722e43d31182424586901bd24f35143a` | `L80-L220` | plinth |

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | **approved** |
| reviewer notes | 用户确认 spec 通过（2026-06-07）。模板已对齐实现并通过 sweep-pipeline 50/50（41 distinct topology）。drawer_count 1–5、+X PRISMATIC、可选顶盖、四柜体风格已签收。 |
