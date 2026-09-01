# Source Map — Electrical_Wiring / Distribution board panel

一 slug（`distribution_board_panel`）：核心脊柱 = enclosure(fixed) → [door_hinge revolute] → door；内含
breaker 场（DIN 轨/立柱上 loop 排布的 MCB 模块，其中若干带真实 toggle revolute）+ 母排/主开关。
enclosure+门数与 breaker 场拓扑是主轴。

## Origins（全量对账，2/2 上格）

| id | pic | 建成形态 | 网格角色 | 承载 fork |
|---|---|---|---|---|
| S1 `rec_use-the-attached-reference-image-as-the-primary-_20260625_164631_232036_c548199a` | 002 | 灰 load-center，单铰链门+deadfront，两立柱×12 行=24 breaker（2 真 toggle），铜母排+中性/地排，警示标 | ③single_door / topo=two_columns / N=24 / mains=bus_only | eight_way · mcb_only_subboard · eighteen_way |
| S2 `rec_use-the-attached-reference-image-as-the-primary-_20260625_164631_229630_4cb00767` | 001 | 大双 bay 板，center_divider，左 power bay(2 主 MCCB+4 相母排+aux+表) 右 3 DIN 轨×14=42 MCB，双窗门 | ③two_door / topo=3_din_rails / N=42 / mains=full_bay | open_backplate · single_din_rail |

## Slots

- **A ③ enclosure+door（主轴）**：single_door(S1) / two_door(S2) / open_backplate_no_door(fork@S2) —— 可外推 single_wide_door
- **B breaker_topology**：two_vertical_columns(S1) / stacked_din_rails(S2) / single_din_rail(fork@S2)
- **C breaker_count N（multiplicity）**：8(fork@S1) / 12/col(S1) / 14/rail(S2) / 18(fork@S1)——模板可外推，articulated_slots 选哪些动
- **D mains_assembly**：full_main_bay(S2，主 MCCB+相母排+aux+表) / bus_bars_only(S1) / mcb_only_subboard(fork@S1，去主开关母排)
- **门样式**：solid_metal(S1) / transparent_window(S2)——折入 A（open 值即无门）

## 交叉矩阵（door × topology；接口=门 hinge + breaker toggle，多数外推）

| door × topo | two_columns | din_rails | single_rail |
|---|---|---|---|
| single_door | 源S1 | 外推 | 外推(小板) |
| two_door | 外推 | 源S2 | gate(双门配单轨不真实) |
| no_door(open) | 外推 | fork open_backplate@S2 | fork single_din_rail@S2 |

## Multiplicity / Copy Logic

- 2D 网格：S1 bank(2)×row(12)；S2 rail(3)×count(14)——`_add_breaker_row(count=N)` loop，`articulated_slots` 选真 toggle
- 门统一用 S2 的**关门 rest 位**约定（hinge 0→开）；S1 baked-open 是 grandfather 反模式，新门变体勿沿用
- screw/standoff/knockout/conduit/wire 均 loop；无复制粘贴块
- Box 面板 / Cyl 螺丝glands pins / mesh 线，primitive 保真

## Forks（5，全部 EXIT=0 + compile success + ≥1 非fixed joint（保留 breaker toggle revolute；open_backplate 去门后仍 4 rev）+ workbench-only + run_tests 断言主轴 + 绑定已核对）

open_backplate@S2(③无门) · single_din_rail@S2(拓扑) · eight_way@S1(N=8) ·
mcb_only_subboard@S1(去主开关) · eighteen_way@S1(N=18)

## 排除项

- 无 origin 排除（2/2 上格）
- 颜色（灰浅灰）= ⑥ palette，不 fork
- 门是否透明窗 = 折入 ③ door 轴，不单列
