# Source Map — Agricultural / Tractor

slug `tractor` · pattern **mixed**（单 `chassis` 根挂独立铰接 child: front axle + 4-6 wheels +
hitch/loader + 转向盘 + 可选 trailer；grille-slat / wheel loop multiplicity）。

## Origins（全量对账，2/2 上格）
| id | pic | 建成形态 | 网格角色 |
|---|---|---|---|
| A `rec_use-the-attached-reference-image-as-the-primary-_20260625_155130_521981_4133cc32` | 001 | 现代封闭 cab (Belarus):玻璃驾驶室 + 大后/小前轮 + 长 hood + 立排气 + 后挂 trailer；steering revolute, trailer yaw, 6 wheel spins | station=cab / front=wide / impl=drawbar+trailer / hood=long_flat |
| B `rec_use-the-attached-reference-image-as-the-primary-_20260625_155130_516579_482a6ef6` | 002 | 复古开放 (John Deere):敞开座椅 + 独立旋转 `steering_wheel` + 大后/窄前轮 + 3-point hitch lift；4 spins + steer + hitch revolute | station=open / front=wide / impl=3point_hitch / hood=long_flat |

## Slots
- **A operator_station**：enclosed_cab(A) / open_bare(B) / open_ROPS(fork@B)
- **B front_axle**：wide_standard(A,B) / narrow_tricycle(fork@B) / single_front(fork@B)
- **C implement**：plain_drawbar(A,B) / 3point_hitch(B) / front_loader(fork@A) / towed_trailer(A)
- **D hood_form（③ Volumetric Envelope）**：long_flat(A,B) / rounded_vintage(fork@B) — 可外推 stepped_modern
- **N**：grille slats ×N {6,10(B)}；wheel lug bolts=BoltPattern `count=` 参数(非 fork)

## Slot 候选覆盖
### Slot A：operator_station
| enclosed_cab(origin) | A | converged |
| open_bare(origin) | B | converged |
| open_ROPS(roll-bar arch) | rec_tractor_var_rops | converged |
### Slot B：front_axle
| wide_standard(origin) | A,B | converged |
| narrow_tricycle | rec_tractor_var_tricycle | converged |
| single_front(yoke steer) | rec_tractor_var_singlefront | converged |
### Slot C：implement
| plain_drawbar(origin) | A,B | converged |
| 3point_hitch(origin, `chassis_to_hitch` revolute) | B | converged |
| front_loader(tower+boom, lift+curl revolute) | rec_tractor_var_loader | converged |
| towed_trailer(origin, `chassis_to_trailer` yaw) | A | converged |
### Slot D：hood_form
| long_flat(origin) | A,B | converged |
| rounded_vintage | rec_tractor_var_roundhood | converged |

## Multiplicity / Copy Logic
- count_param: `n_grille_slats` — `grille_slat_{idx}` 等距，FIXED 装饰于 `front_grille_panel`；N {6,10} → rec_tractor_var_grille6 / B；模板 N_range [4,16]
- wheels: 每轮独立 continuous(`*_wheel_*_spin`)，前对随 `front_axle`/`steering_axle` revolute 转向；tricycle/single-front 改前轮数(Slot B，非 N-sweep)

## 视觉多样性 6 轴考察
| 轴 | 处理 | 取值 |
|---|---|---|
| ① 骨架图 | forked_anchor | station 3 + front_axle 3 + implement 4 + hood 2 |
| ② 关节类型 | forked_anchor | wheel continuous、steering/front-axle revolute、hitch lift revolute、loader lift+curl revolute、trailer yaw |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | hood envelope(long/round)；可外推 stepped_modern |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | `john_deere_letter_{i}`/badge decal、grille pattern(slat vs vent-mesh)、exhaust cap；可外推 stripe/rivet |
| ⑤ 尺寸/行程 | record_only | rear/front tire Ø ~1.5-2.0；steering ±0.45、hitch ±0.3-0.42；wheel continuous |
| ⑥ 涂装 | record_only | JD green+yellow / Belarus blue / Massey red / Kubota orange / NH blue / vintage grey |

## 排除项
- grille N=14 — 取消(N{6,10} 已示 copy logic)；articulated-frame(铰接机)留模板/gate
