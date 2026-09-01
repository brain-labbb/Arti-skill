# Source Map — Agricultural / Harvester vehicle (arm)

slug `harvester` · pattern **mixed**（linear_chain: carrier→[turret]→boom→stick→head→grapple +
wheels/rollers/fingers multiplicity + parallel_children）。3 origin 占满 undercarriage 三型、boom 两型、
head 两型；fork 只填空格。

## Origins（全量对账，3/3 上格）
| id | pic | 建成形态 | 网格角色 | 非固定关节 |
|---|---|---|---|---|
| P1 `rec_use-the-attached-reference-image-as-the-primary-_20260625_155418_883282_26d122a9` | 001 | 孤立臂:pedestal king-post 底 + 倒 V 平衡臂 + 4 指伐木抓头(rotator+`claw_finger_{i}`) | under=pedestal / boom=inverted_V / head=felling_grapple | 8 |
| P2 `rec_use-the-attached-reference-image-as-the-primary-_20260625_155418_893521_f77bf836` | 002 | JD 绿轮式:6 轮 bogie carrier + cab + 2 段 knuckle boom + processing head(feed roller/saw/knife) | under=wheeled_bogie / boom=knuckle_2sec / head=processing | 10 |
| P3 `rec_use-the-attached-reference-image-as-the-primary-_20260625_155130_522179_e451456d` | 003 | Komatsu 红履带:crawler + turret slew + box boom + processing head | under=tracked_crawler / boom=knuckle_2sec / head=processing | 4 |

## Slots
- **A undercarriage（① skeleton）**：wheeled_bogie(P2) / tracked_crawler(P3) / pedestal_static(P1) — articulated-frame(forwarder) 仅 gate 不 fork(类目风险)
- **B boom_type**：knuckle_2section(P2,P3) / inverted_V(P1) / telescopic(fork@P3)
- **C head（"(arm)" 核心 slot，最丰富）**：processing_head(P2,P3) / felling_grapple_4finger(P1) / feller_saw(fork@P3) / mulcher(fork@P2,gate) / log_grapple(fork@P2)
- **N**：carrier wheels/axles ×N {3axle/6(P2),4axle/8}；track rollers ×N {4/side(P3),6/side}；boom lift cyl {1(P2),2(P3)}(双源锚，不 fork)；grapple fingers ×N(4,P1)

## Slot 候选覆盖
### Slot A：undercarriage
| wheeled_bogie(P2) / tracked_crawler(P3) / pedestal_static(P1) | 全 origin | converged(0 fork) |
### Slot B：boom_type
| knuckle_2section(P2,P3) / inverted_V(P1) | origin | converged |
| telescopic(`stick` 内伸 prismatic) | rec_harvester_var_boom_telescopic | converged |
### Slot C：head
| processing_head(P2,P3) | origin | converged |
| felling_grapple_4finger(P1) | origin | converged |
| feller_saw(`saw_disc`+`grab_arm_{0,1}` revolute) | rec_harvester_var_head_feller_saw | converged |
| mulcher(`mulcher_drum` continuous+`tooth_{i}`) | rec_harvester_var_head_mulcher | converged(gate:类目忠实) |
| log_grapple(`jaw_{0,1}` revolute) | rec_harvester_var_head_log_grapple | converged |

## Multiplicity / Copy Logic
- wheels: count_param `num_axles` — P2 手写 `wheel_specs`(6)/axle(3)/fender(6) → fork 改计算 loop 4-axle/8-wheel；每轮独立 continuous → rec_harvester_var_wheels_8；模板 N_range 轴 [2,5]
- track rollers: `*_roller_{i}` 4→6/side，FIXED 视觉 → rec_harvester_var_track_rollers_6；模板 [3,8]
- boom lift cylinders 1↔2 双 origin 锚(P2/P3)，记录不 fork；grapple fingers `claw_finger_{i}`=4(P1 loop)

## 视觉多样性 6 轴考察
| 轴 | 处理 | 取值 |
|---|---|---|
| ① 骨架图 | forked_anchor | under 3 + boom 3 + head 5；wheels N{6,8}、rollers N{4,6} |
| ② 关节类型 | forked_anchor | boom/wrist revolute、turret slew(±1.45,P3)、head rotator/mulcher continuous、grapple/finger/jaw revolute、telescopic prismatic、wheel continuous |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | head/undercarriage 型；可外推 boom 分段/head 变体 |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | 液压软管(`*_hose`)、decal(`capacity_black_mark_{i}`)、guards/lights/grousers |
| ⑤ 尺寸/行程 | record_only | reach(main~2.55+stick~2.0)；lift ±0.4-0.55、elbow ±0.6-0.8、wrist ±0.65-1.2、slew ±1.45、grapple 0→1.15 |
| ⑥ 涂装 | record_only | JD green+yellow / Komatsu red / black arm / Ponsse yellow / Tigercat orange / grey |

## 排除项
- articulated-frame forwarder undercarriage — 类目风险(forwarder≠harvester)，gate 不 fork
- mulcher head — gated：reviewer 需确认仍读作"带臂 harvester"而非独立 mulcher 类
