# Urban_Environment / Caster Trolley — template source map

object identity: wheeled **service / utility / stock cart** family (industrial platform stock cart, flat platform utility truck, tall wire-mesh roll-cage trolley, two-tier galvanized service/bus cart). Every member is a horizontal load surface on a wheeled steel chassis riding on swivel casters, pushed by a tubular handle. Variants must stay inside this warehouse/service-cart family. **NOTE (2026-07-02 re-scope):** the chrome wire-basket **supermarket shopping trolley** (rec_chrome-wire-basket-…58ed850d) was moved OUT to its own 小类 `Caster Trolley2` — it is NO LONGER a member. The supermarket-only features it contributed (tapered wire-lattice basket, low basket push-bar, fold-down child-seat flap, chrome_supermarket palette) are therefore DROPPED from this template's design space so the seed set stays coherent and realistic.

pattern: mixed — parallel_children (named structural slots: load_surface 主机构 + handle/upright 机构) + multiplicity (swivel casters × N; shelf tiers × N).

## parents（4 张图，4 个原始母资产；全部 4 casters，REVOLUTE/CONTINUOUS 双关节，casters 均 for-loop 发射）

- rec_industrial-platform-stock-cart-a-flat-wooden-dec_20260608_164504_879812_974df157 ← picture/Urban Environment/Caster Trolley/001.png（工业平板备料车：钢底盘 + 木甲板 deck，单端管式 push_handle，端部 wire-mesh 护栏板；4 swivel caster `caster_yoke_{i}`/`caster_wheel_{i}`，`deck_to_caster_yoke_{i}` REVOLUTE-Z + `caster_yoke_{i}_to_wheel` CONTINUOUS-Y）
- rec_flat-platform-utility-cart-with-a-tall-tubular-p_20260608_164421_000197_0233d588 ← .../002.png（平板工具车：灰钢 deck slab + 卷边 lip，单端 tall `push_handle`（inverted-U + 2 cross-rail）+ 另一端短 `end_guard`；4 swivel caster，`deck_to_caster_yoke_{i}`/`caster_yoke_{i}_to_wheel`）**Slot B / caster-count 变体的基线 parent**
- rec_tall-wire-mesh-shelf-roll-cage-trolley-with-a-bl_20260608_164442_835516_ef2c8c79 ← .../003.png（高 wire-mesh roll-cage：黑管 `frame` root + 网侧/网背板 + `SHELF_HEIGHTS` 4-tuple 驱动 `shelf_{si}`（`frame_to_shelf_{si}` FIXED）+ 装载 `box_{bi}`；4 swivel caster）**tier-count / cage 变体的基线 parent**
- rec_two-tier-galvanized-steel-service-cart-with-rais_20260608_164458_476803_7e448273 ← .../004.png（双层镀锌服务车：`lower_tray` root + `upper_tray`（`lower_to_upper` FIXED）+ 两端 `end_frame_{tag}` 立腿弯成 handle；4 swivel caster）

（已移出：rec_chrome-wire-basket-shopping-cart-…58ed850d → 现属 `Urban Environment / Caster Trolley2`，不再参与本模板。）

## 组合数预审（HARD GATE）

Slot A load_surface **4 候选**（flat_platform_deck / stacked_open_trays / wire_mesh_shelf_stack / utility_wire_bin）× Slot B handle/upright **3 候选**（tall_inverted_U_one_end / handle_both_ends / full_cage_uprights）× Slot C 多重性 distinct-N 3（casters N∈{4,6,3}；tiers N∈{4,3} 复用同一 multiplicity 轴）= **36 ≥ 10** ✓。
即便单独 Slot A(4) × distinct-N(3) = 12 ≥ 10 ✓，Slot A(4) × Slot B(3) = 12 ≥ 10 ✓。门槛安全通过。所有候选均由现存 4 母资产 + 9 变体（无 supermarket seed）真源支撑。

## Slot 候选覆盖

### Slot A：load_surface（主机构槽——车的承载面拓扑）
| 候选(未来 module) | variant | 关键部件 / 结构 | 状态 |
|---|---|---|---|
| flat_platform_deck（基线） | parent 001 / 002 | 单块 `deck` slab + 卷边 lip | converged (parent) |
| stacked_open_trays（基线） | parent 004 | `lower_tray`/`upper_tray` 浅盘 + lip，corner legs 撑开 | converged (parent) |
| wire_mesh_shelf_stack（基线） | parent 003 | `SHELF_HEIGHTS` 驱动的 `shelf_{si}` 多层网架 | converged (parent) |
| utility_wire_bin | rec_caster_trolley_var_deck_to_basket | **矩形** wire-mesh 载物笼/料箱（lattice for-loop，直壁不锥形）——仓储 utility bin，非超市购物篮 | converged (forked) |
| deck→two_tray | rec_caster_trolley_var_deck_to_two_tray | 平板 deck 改双层托盘 + corner legs（shared tray helper） | converged (forked) |
| two_tray→deck | rec_caster_trolley_var_tray_to_deck | 双层托盘改单块平板 deck（去 upper tray + legs） | converged (forked) |

### Slot B：handle_upright（推手 / 立柱机构槽）
| 候选 | variant | 关键 joint / 结构 | 状态 |
|---|---|---|---|
| tall_inverted_U_one_end（基线） | parent 001 / 002 | 单端 inverted-U `push_handle` + cross-rail（FIXED） | converged (parent) |
| handle_both_ends（基线） | parent 004 | 两端 `end_frame_{tag}` 立腿弯成 handle | converged (parent) |
| full_cage_uprights（基线） | parent 003 | 满高 wire-mesh `frame` 立柱 + 网板 | converged (parent) |
| single→double_handle | rec_caster_trolley_var_single_to_double_handle | 短 end_guard 改第二根 tall handle（shared handle helper，两端对称） | converged (forked) |
| fixed→folding_handle | rec_caster_trolley_var_fixed_to_folding_handle | tall handle 改底部 REVOLUTE 折叠铰（新增非 fixed joint，铰origin在 deck 顶面） | converged (forked) |
| cage→open_post_handle | rec_caster_trolley_var_cage_to_open_post_handle | 拆网笼改 4 角立柱 + 单端 tall handle，保留 shelves | converged (forked) |

（已移除候选：`low_basket_handle`（超市篮口低把横杆）+ `child_seat_flap`（折叠童座 REVOLUTE）——仅由已移出的 005 支撑，随其一并删除。）

### Slot C：caster_count / tier_count（MULTIPLICITY 槽）
| 候选 | variant | N / copy logic | 状态 |
|---|---|---|---|
| casters_N=4（基线） | parent（全部 4 个） | `for i in range(4)` over 4-corner 列表；每件 REVOLUTE-Z swivel + CONTINUOUS-Y roll，uniform joint policy | converged (parent) |
| casters_N=6 | rec_caster_trolley_var_casters_six | 6 caster（加中段轴对），count = len(位置列表) 驱动的单 for-loop | converged (forked) |
| casters_N=3 | rec_caster_trolley_var_casters_three | 3 caster tricycle（一端双 + 一端单中），count = len(位置列表) | converged (forked) |
| tiers_N=4（基线） | parent 003 | `SHELF_HEIGHTS` 4-tuple 驱动 `shelf_{si}` | converged (parent) |
| tiers_N=3 | rec_caster_trolley_var_shelves_three | 3 层 shelf，count = len(shelf-height 列表)，shared shelf helper | converged (forked) |

## Multiplicity / Copy Logic

- **caster ring**（primary count axis）：所有 parent 已是 `for i in range(4)` over 角点列表 + shared yoke/wheel helper + uniform joint policy（每件 `deck_to_caster_yoke_{i}` REVOLUTE/CONTINUOUS Z-swivel + `caster_yoke_{i}_to_wheel` CONTINUOUS-Y roll，各 caster 独立活动）。**注意现有 parent 把数量硬编码为 4**（`range(4)` / 4-元角点列表）——caster-count 变体的 prompt 已显式要求"count 由位置列表长度驱动"，把硬编码 4 重写成 `len(positions)` 驱动的循环，供模板侧做 N_range。模板建议 N_range ∈ [3, 8]（样本只覆盖 {3,4,6} 展示 copy logic，其余由 sweep 放大）。
- **shelf tier ring**（secondary count axis，仅 roll-cage / shelf 系）：parent 003 已是 `for si, h in enumerate(SHELF_HEIGHTS)` + shared shelf helper + 每层 `frame_to_shelf_{si}` FIXED（全部 FIXED 在 frame 上）。tier-count 变体把 N 改为列表长度驱动。模板建议 tier N_range ∈ [2, 6]，样本覆盖 {3,4}。
- 这两个 multiplicity 轴不需要互相组合穷举；fork 批只各给干净的 copy-logic 样本。

## Readability / loop-emission notes

- 全部 4 个 parent 的 casters 都是 for-loop 发射（`for i in range(4)` / `enumerate(corners)`），shared yoke+wheel helper，uniform 双关节策略 — caster multiplicity 变体可直接继承，只需把字面 4 换成 `len(positions)`。
- 003 的 shelves（`SHELF_HEIGHTS` 驱动）、004 的双 tray（shared `_tray_visuals` helper）、deck_to_basket 变体的 bin lattice（`for i in range(n_*)` 网格）都已循环化 — Slot A 变体复用各自的 helper。
- 无手写重复需要回头重写：所有同构子件均已循环发射，无 left/right 手写对偶病。

## 格子覆盖 / 计划

4 parent 预占 3 个 Slot-A 格（deck / trays / shelf_stack）、3 个 Slot-B 格（inverted_U / both_ends / full_cage）、casters N=4 + tiers N=4 两个 multiplicity 基线格。9 个变体填其余：
- Slot A 空格（utility_wire_bin + deck↔tray 互换路径，从最近 parent fork）
- Slot B 空格 3 个（double / folding / open-post handle）
- Slot C 空格 3 个（casters N=6, N=3；tiers N=3）

variant count = 9（cap 8–10 内）。dropped axis：supermarket-only load/handle/flap（随 005 移出到 Caster Trolley2）。纯尺寸/颜色/材质未计入候选（满足"不把连续尺寸当候选"复核）。
