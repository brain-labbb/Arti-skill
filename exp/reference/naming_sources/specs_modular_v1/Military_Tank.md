# Modular Spec — Military / Tank (wheeled APC)

## 元信息
| 项 | 值 |
|---|---|
| slug | `armored_vehicle` |
| template path | `agent/templates/Military_Tank.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed`（3 个结构 slot：running-gear × weapon-station × hull-armor；外加 1 根 multiplicity 轴 `axle_pairs` = 每侧轮对数） |

> NOTE：小类标 "Tank"，但 parent + 全部变体是 **轮式装甲运兵车（APC）** 家族；身份是 wheeled/tracked APC，**不是主战坦克（MBT）**。

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 8（1 parent + 7 variants） |
| read_count | 8 |

**共性骨架**：root `hull`（`_build_hull_solid` ~L76）→ 8× `{side}_wheel_{i}`（CONTINUOUS，`_tire_geom`~L345 / `_wheel_geom`~L357 loop over AXLE_X 4-tuple）+ `turret`（CONTINUOUS yaw）→ `autocannon`（REVOLUTE elevation）。10 非 fixed 关节。CadQuery hull/turret loft + TireGeometry/WheelGeometry。轮已 loop-emit。

## 核心身份
轮式（或履带）装甲运兵车（APC）。识别 = **装甲车体 + 武器站（炮塔/RWS/pintle）+ 行走机构**（≥1 非 fixed joint：轮自转/炮塔/俯仰）。不该混入：主战坦克（低矮厚甲 + 大口径主炮，`tracked` 变体须仍读作 APC 而非 MBT —— 见兼容矩阵 flag）、卡车、装甲轿车。

## 槽位 + 候选模块表

### Slot A：running-gear（行走机构）
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| wheeled_8x8 | parent | `_tire_geom` L345-L356 / `_wheel_geom` L357-L407（AXLE_X 4-tuple loop） | eligible | 8 轮（每侧 4 对），各 CONTINUOUS 横轴 |
| wheeled_6x6 | rec_armored_vehicle_var_6x6 | 同 loop，AXLE_X 3-tuple L345-L407 | eligible | 6 轮（每侧 3 对） |
| tracked | rec_armored_vehicle_var_tracked | `_build_track_band` L125-L156 + `_build_sprocket` L157-L178 + `_build_idler` L179-L193 + `_road_wheel_geom` L483-L497 | eligible（category-risk，见 §reject） | 两条履带 + bogie loop + 主动/诱导轮 |

### Slot B：weapon-station（武器站）
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| autocannon_turret | parent | `_turret_shell` L145-L172 + autocannon | eligible | 有人小炮塔 + 机炮（yaw + elevation） |
| remote_weapon_station | rec_armored_vehicle_var_rws | `_rws_pedestal` L137-L146 + `_rws_pod_shell` L147-L174 | eligible | 顶部遥控武器站（pod yaw + gun elevation，无大炮塔） |
| open_pintle | rec_armored_vehicle_var_pintle | `_ring_race` L161-L170 + `_rotating_ring` L171-L180 + `_pintle_post` L191-L200 + `_mg_receiver_body` L211-L230 | eligible | 开放环座枪架（yaw），开顶载员甲板 |

### Slot C：hull-armor（车体/装甲）
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| sloped_welded | parent | `_build_hull_solid` L76-L88 | eligible | 倾斜焊接车体 |
| slab_mrap | rec_armored_vehicle_var_slab | `_build_hull_solid` L74-L109 | eligible | 高直立 MRAP 板状车体 |
| applique_panels | rec_armored_vehicle_var_applique | `build` L185 + `panel_{i}` loop（standoff bosses） | eligible | 在 sloped 车体外加挂 `panel_{i}` 附加装甲阵列（loop） |

> **single-candidate slot degrade**：无（每槽 3）。

## 槽位图（slot graph）
```
 hull (root, [Slot C hull-armor])
   ├─ [Slot A running-gear] {side}_wheel_{i} (CONTINUOUS, AXLE_X N对) | tracked bogies
   └─ [Slot B weapon-station] turret(yaw CONT)─►autocannon(elev REV) | RWS(yaw+elev) | pintle(yaw)
```

## 每槽位 Module Emits / Interfaces
- **Slot A**：轮组经 AXLE_X tuple loop `{side}_wheel_{i}` + 轮拱/轴桩/挡泥随 tuple；tracked emits 履带带 + sprocket/idler + bogie loop。
- **Slot B**：autocannon_turret（yaw CONT + elev REV）；rws（pod yaw + gun elev）；pintle（ring yaw）。
- **Slot C**：hull 实体（sloped/slab）；applique emits `panel_{i}` standoff 阵列（FIXED 装饰）。
- **Multiplicity**：road-wheel 对数经 AXLE_X tuple loop，每轮 CONTINUOUS 横轴。

## 参数范围汇总
| 参数 | 类型 | 取值 | 约束 | 来源 |
|---|---|---|---|---|
| running_gear | enum | wheeled_8x8 / wheeled_6x6 / tracked | choice | Slot A |
| weapon_station | enum | autocannon_turret / remote_weapon_station / open_pintle | choice | Slot B |
| hull_armor | enum | sloped_welded / slab_mrap / applique_panels | choice | Slot C |
| palette_style | enum | nato_green / desert_tan / woodland_camo / urban_grey（≥3） | palette only | S 材质 |
| axle_pairs | int | {3,4,5}，N_range [3,5]（仅 wheeled；>5 出 APC 词汇表） | multiplicity（AXLE_X loop） | 6x6/10x10 |
| hull_len_scale | float | [0.92,1.12] | independent clamp | parent |

## Multiplicity / Copy Logic
- **count_param**：`axle_pairs`（每侧轮对数，AXLE_X tuple 长度）。
- **N_range**：**[3, 5]** 仅 wheeled（重型轮式 APC 真实词汇；超 5 对离开类别）。采样 {3,4,5}（parent=4）。
- **copied object**：`{side}_wheel_{i}`（tire+rim），沿 AXLE_X 前后等距。
- **joint policy**：每轮独立 CONTINUOUS 横轴；轮拱切、轴桩、挡泥长度随 tuple 同步。
- **conditional**：axle_pairs 仅 wheeled 适用；tracked 的 bogie 数是其自身内部 copy-logic，不参与该 N 扫。

## 拓扑多样性审计
- A(3) × B(3) × C(3) = **27** 纯 slot；× axle N{3,4,5}（仅 wheeled）= **45+** distinct。
- procedural_first：采 A/B/C → 若 A=wheeled 采 axle_pairs → palette → 连续 scale。
- 兼容矩阵：**tracked × sloped/slab OK，但须保持 APC 车体 + 低炮塔**（不读作 MBT —— flag）；axle_pairs 仅 wheeled；applique 挂在 sloped/slab 上。
- Topology target ~45；远超 10。（统一口径：富类别建议 ≥300；低于 300 记录真实组合空间或兼容约束原因；report-only，不设门。）

## Validator
- 轮 CONTINUOUS / 炮塔 yaw / 机炮 elev 关节恒在（≥1）。
- axle_pairs ∈ {3,4,5}，仅 wheeled；轮拱/轴桩/挡泥随 tuple。
- tracked 读作 APC（boxy hull + 低 autocannon 炮塔），非 MBT。
- applique `panel_{i}` loop FIXED standoff。
- 镜像 L/R 家具（门/储物/扶手）非 multiplicity；element-scoped allow_overlap。

## Reject cases
- `tracked` 读成主战坦克（厚甲 glacis + 大主炮）→ 出类目；保持 APC 车体 + 低炮塔；2-3 次不收敛记 blocked。
- axle_pairs > 5 → 离开轮式 APC 词汇。
- 把 axle_pairs 用于 tracked → 非法（bogie 是其内部 copy-logic）。
- 炮塔/轮关节降为 FIXED → 0 关键关节。
- palette/scale 当 candidate → 非结构差异。

## 与相邻类别的边界
- 主战坦克：履带 + 厚甲 + 大口径主炮，低矮车体（本类是 APC 高车体 + 小机炮/RWS）。
- 卡车/装甲卡车：无武器站 + 非装甲车体语义。
- 装甲轿车：非军用运兵语义。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |

## Module Source Index
| source_id | slot | module | record_id |
|---|---|---|---|
| S1 | A/B/C | wheeled_8x8 / autocannon_turret / sloped_welded (parent) | rec_model-an-eight-wheeled-...-carrier_..._407e2520 |
| S2 | A | wheeled_6x6 | rec_armored_vehicle_var_6x6 |
| S3 | A | tracked | rec_armored_vehicle_var_tracked |
| S4 | multiplicity | axle_pairs N=5 (10x10) | rec_armored_vehicle_var_10x10 |
| S5 | B | remote_weapon_station | rec_armored_vehicle_var_rws |
| S6 | B | open_pintle | rec_armored_vehicle_var_pintle |
| S7 | C | slab_mrap | rec_armored_vehicle_var_slab |
| S8 | C | applique_panels | rec_armored_vehicle_var_applique |
