# Source Map — Agricultural / Hand cultivator

slug `cultivator` · pattern **mixed** (parallel_children root `frame`→`tine_wheel` on one
`wheel_axle` continuous spin, wrapping multiplicity copy-logic chains: `spring_claw_{i}`,
`wheel_spoke_{i}`). A walk-behind **wheel-hoe**; the defining slot is the interchangeable
**working head**.

## Origins（全量对账，1/1 上格）
| id | pic | 建成形态 | 网格角色 |
|---|---|---|---|
| A `rec_use-the-attached-reference-image-as-the-primary-_20260625_155418_882758_a6707f8e` | 001 | 单大铁辐轮(8 辐)+ 双木柄 + 5 根弯簧齿头，唯一活动关节 `wheel_axle` continuous | head=spring_tine / wheel=spoked_iron / handle=double_straight |

## Slots
- **A working_head（主轴 ①/③）**：spring_tine_claws(A) / rigid_tines(fork) / stirrup_hoe(fork) / sweep(fork) / ridger(fork) — 模板可外推 plow_share / rake_bar / rotary_star
- **B ground_wheel**：spoked_iron(A) / solid_disc(fork) / pneumatic(fork) — 可外推 dual_narrow
- **C handle_config**：double_straight(A) / single_central(fork) — 可外推 T/loop grip
- **N**：tines ×N {3,5(A),7}；wheel spokes ×N（A=8，loop 发射）

## Slot 候选覆盖
### Slot A：working_head
| 候选 | source_type | record_id | 关键 part/joint | 状态 |
|---|---|---|---|---|
| spring_tine_claws | forked_anchor(origin) | A | `spring_claw_{i}`/`worn_claw_tip_{i}` on `rake_crossbar` | converged |
| rigid_tines | forked_anchor | rec_cultivator_var_rigid_tines | `rigid_tine_{i}` straight drop | converged |
| stirrup_hoe | forked_anchor | rec_cultivator_var_stirrup_hoe | stirrup U-blade on `rake_neck` | converged |
| sweep | forked_anchor | rec_cultivator_var_sweep | wide V duckfoot blade | converged |
| ridger | forked_anchor | rec_cultivator_var_ridger | moldboard wing share | converged |

### Slot B：ground_wheel
| 候选 | source_type | record_id | 关键 part | 状态 |
|---|---|---|---|---|
| spoked_iron | forked_anchor(origin) | A | `wheel_spoke_{i}`+`inner_iron_rim` | converged |
| solid_disc | forked_anchor | rec_cultivator_var_disc_wheel | solid iron disc, hub keeps spin | converged |
| pneumatic | forked_anchor | rec_cultivator_var_pneumatic_wheel | rubber tire torus + steel hub | converged |

### Slot C：handle_config
| 候选 | source_type | record_id | 状态 |
|---|---|---|---|
| double_straight | forked_anchor(origin) | A | converged |
| single_central | forked_anchor | rec_cultivator_var_single_handle | converged |

## Multiplicity / Copy Logic
- count_param: `n_tines` — copied object=弯簧齿 `spring_claw_{i}`+`worn_claw_tip_{i}`；placement=沿 `rake_crossbar` 等距；joint policy=FIXED 视觉(不单独铰接，仅 `wheel_axle` 旋转)
- N 样本已覆盖: {3,5,7} → rec_cultivator_var_tines3 / A / rec_cultivator_var_tines7
- 次级: wheel spokes `wheel_spoke_{i}` N（A=8，loop）；模板建议 N_range tines [3,9]、spokes [6,16]

## 视觉多样性 6 轴考察
| 轴 | 处理 | 本小类取值 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor | head 5 形态 + wheel 3 + handle 2；tines N∈{3,5,7} |
| ② 关节类型 | forked_anchor | `wheel_axle` continuous(全轴)；无第二活动轴(depth-pivot 留模板，本批不做) |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | working_head 形态(Volumetric Envelope)；可外推 plow/rake/rotary |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | `bolt_head_{i}` 铆钉、rust、worn bright edge、maker plate |
| ⑤ 尺寸/行程 | record_only | wheel dia ±30%、handle length/spread、tine depth；`wheel_axle` continuous |
| ⑥ 涂装 | record_only | rusted iron / painted green|red / galvanized / blued / polished steel / Planet-Jr green+yellow |

## 排除项
- dual_narrow_wheels / T-grip handle / spokes N=12 — 结构真实但留模板外推(预算)，非失败；均可后续补格
