# Container / Glass bottle — template source map

pattern: parallel_children（固定 named slots:bottle body profile + closure mechanism(主机构);单一瓶身，无核心 multiplicity）

parents（1 个母资产,深色长颈啤酒瓶 + 压盖式皇冠瓶盖）:
- rec_dark-glass-beer-bottle-with-a-crimped-metal-crow_20260606_074746_219040_2a24ec81 ← picture/Container/Glass bottle/001.png（基线 = body_shape:long_neck × closure:pry_off_crown_cap(PRISMATIC pop-off)；占据 A=long_neck、B=pry_off_crown 两格）

P0 阶段：仅规划 + 写 prompt 文件 + 草拟本 source map，**未执行 fork**。下面 12 个变体格子状态均为 `converged`。

> Re-audit (DEEPEN，无 fork)：原 P0 计划过薄（组合预审 16，近地板）。按 FORK_VARIANTS.md §5「提高每槽候选数」深化候选词表——两槽各 +3 真实且结构互异的候选，组合预审升至 49。未新增第三轴（见排除项末段说明）。

## 组合数预审

组合数预审: Π(body_shape 7 × closure 7) × N(无 multiplicity) = 49 ≥ 10 ✓（原 4×4=16 → deepen 后 7×7=49）

closure 槽覆盖多种 joint 拓扑：PRISMATIC(压盖/拔塞/Codd 玻璃珠下压/dropper 直拔) + REVOLUTE/CONTINUOUS(旋盖) + REVOLUTE(摆杆翻塞 / pour-spout 翻盖铰)，结构差异充分。

## Slot 候选覆盖

### Slot A:bottle_body_profile（瓶身轮廓家族，lathe/loft 旋转体侧轮廓）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| long_neck（基线） | rec_dark-glass-beer-bottle-with-a-crimped-metal-crow_20260606_074746_219040_2a24ec81 | `bottle` part / `bottle_glass` visual / `_bottle_solid` + `_profile_loft` helper | 高瘦长颈啤酒瓶：直筒身 + 斜肩 + 细长颈 + 外翻唇口 | converged(parent) |
| wine_bottle | rec_container_glass_bottle_var_wine_bottle | `bottle` / `bottle_glass` / `_bottle_solid`(改 loft sections) | 葡萄酒瓶：高直筒身 + 高圆肩急收 + 短直颈 + 凹底 punt | converged |
| stubby_steinie | rec_container_glass_bottle_var_stubby_steinie | `bottle` / `bottle_glass` / `_bottle_solid`(改 loft sections) | 矮胖 steinie：粗矮身 + 缓圆肩 + 极短颈，低肩回收瓶形 | converged |
| boston_round | rec_container_glass_bottle_var_boston_round | `bottle` / `bottle_glass` / `_bottle_solid`(改 loft sections) | 波士顿圆瓶：圆肩身连续过渡入短窄颈 + 小珠唇口（药瓶/油瓶形） | converged |
| hip_flask | rec_container_glass_bottle_var_hip_flask | `bottle` / `bottle_glass` / `_bottle_solid`(非圆截面 D/肾形 loft) | 弯曲扁酒壶：扁平 D/肾形截面（一面凹一面凸）+ 短窄颈，非旋转对称扁瓶身 | converged |
| decanter_carafe | rec_container_glass_bottle_var_decanter_carafe | `bottle` / `bottle_glass` / `_bottle_solid`(改 loft sections) | 宽肩醒酒/水瓶：矮宽球腹 + 极宽斜肩急收 + 较长细颈，蹲胖宽底卡拉夫形 | converged |
| slim_flute | rec_container_glass_bottle_var_slim_flute | `bottle` / `bottle_glass` / `_bottle_solid`(改 loft sections) | 高瘦笛形：极高极窄、瓶身连续平滑收入颈无肩部断口（莱茵/阿尔萨斯 flute 瓶） | converged |

### Slot B:closure_mechanism（**主机构槽**——瓶口封闭机构 / 唯一活动关节）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pry_off_crown_cap（基线） | rec_dark-glass-beer-bottle-with-a-crimped-metal-crow_20260606_074746_219040_2a24ec81 | `crown_cap` part / `bottle_to_cap` PRISMATIC(+Z) / `_cap_mesh` helper（21 齿压边循环） | 皇冠压盖：短穹顶 + 压边齿裙，沿轴直起 pop-off | converged(parent) |
| screw_cap | rec_container_glass_bottle_var_screw_cap | `screw_cap` part / `bottle_to_cap` CONTINUOUS/REVOLUTE(Z) / cap mesh helper | 螺纹旋盖：滚花圆筒盖旋上螺纹瓶口，绕轴旋转就位 | converged |
| swing_top | rec_container_glass_bottle_var_swing_top | `swing_stopper` part / `bottle_to_stopper` REVOLUTE(X) / stopper+bail helper | 扣压式翻塞：陶瓷/胶塞头 + 铰接钢丝提杆，绕铰摆起离口 | converged |
| cork | rec_container_glass_bottle_var_cork | `cork_stopper` part / `bottle_to_cork` PRISMATIC(+Z) / cork lathe helper | 锥形软木塞：插入瓶膛的略锥塞，沿轴直拔出口 | converged |
| dropper_pipette | rec_container_glass_bottle_var_dropper_pipette | `dropper_cap` part(含细玻管+顶部胶头球) / `bottle_to_dropper` PRISMATIC(+Z) / pipette lathe helper | 滴管/吸管盖：螺/压盖带细长玻璃吸管伸入瓶膛 + 顶部挤压胶头，整组沿轴直拔出口 | converged |
| pour_spout | rec_container_glass_bottle_var_pour_spout | `flip_lid` part(铰接翻盖) + 锥形 `pour_spout` 插嘴 visual / `spout_to_lid` REVOLUTE(铰) / spout lathe helper | 倒酒嘴翻盖：锥形导流嘴插入瓶口 + 小翻盖绕铰 revolute 掀开露嘴 | converged |
| codd_marble | rec_container_glass_bottle_var_codd_marble | `codd_marble` part(球) / `bottle_to_marble` PRISMATIC(+Z) / 颈部夹腔(pinched neck)改瓶身 | Codd 玻璃珠塞：颈内囚禁玻璃珠抵唇口胶圈密封，下压珠子进颈腔开启、上顶回唇口关闭 | converged |

## 格子覆盖

| 槽 | 候选数(含基线) | parent 占格 | 待 fork 空格 |
|---|---|---|---|
| A bottle_body_profile | 7 | long_neck | wine_bottle / stubby_steinie / boston_round / hip_flask / decanter_carafe / slim_flute (6) |
| B closure_mechanism(主机构) | 7 | pry_off_crown_cap | screw_cap / swing_top / cork / dropper_pipette / pour_spout / codd_marble (6) |

待填格子 cells = (7−1) + (7−1) = 12；计划变体数 = 12（每空格 1 个，单 parent fork，单轴变化、其余层保持 parent 基线）。每槽 ≥2 候选 → 满足 §8 完成定义的 slot 门槛。deepen 优先加深主机构 closure 槽（dropper / pour_spout / Codd 三个结构强互异机构）。

## Multiplicity / Copy Logic
- count_param: 无。Glass bottle 为单一瓶身 + 单一封口，核心结构为固定 named slots，无"同构子件 × N"复制逻辑。
- N 样本已覆盖: 无。
- 模板建议 N_range: 无。
- copied object / naming / placement / joint policy: 无。
- 备注：parent `_cap_mesh` 内 21 个压边齿已是 `for i in range(n_flutes)` 循环发射的 inline 视觉装饰（非独立 part、非结构 multiplicity），符合 §4 可读性契约，可作为模板侧 inline 循环视觉的参考写法。

## 排除项（未来 compatibility matrix 素材）
- handle/jug 提手轴：未列入。参考图为光面无提手瓶；加提手会读成 jug/growler 而出 Glass bottle 类目，故不作为 slot（re-audit 维持排除）。
- 表面结构轴（肋纹/浮雕/穿孔）：玻璃瓶表面纹理属装饰，非结构轴，不列入（鼓励作为变体上的材质/视觉叠加，不计为轴）。
- 第三轴「瓶口 finish/collar 形（rolled lip / bead collar / 香槟双环 string-rim）」：re-audit 考虑后**拒绝作为独立轴**。真实玻璃瓶 finish 确有差异，但相对 body+closure 两轴只是颈部小细节、结构贡献弱且易滑向「装饰轴」，不足以撑起干净的结构轴；按 §5 选择深化已有两真实轴而非硬加第三轴。可在 closure 变体上作为视觉叠加，不计为轴。
- 尺寸/比例「mini 50ml 小酒版 / 1.5L magnum 大瓶」：re-audit 拒绝，纯 scale 变化非结构候选。
- 提梁/jug-ring growler、双口/连体瓶：re-audit 拒绝，出 Glass bottle 类目。
- 尚无连续失败记录（P0 阶段未 fork）。fork 后若某候选连续不收敛，回写本节并按 §5 重造或标 blocked。
