# Urban Environment / Mailbox — template source map

slug: mailbox　shard: Mailbox　picdir: picture/Urban Environment/Mailbox
identity: mail collection box — hollow body + openable access door/flap (real REVOLUTE) + mount (post / legs / pedestal / wall); optional side semaphore signal flag (REVOLUTE). Must keep reading as a mailbox; workbench-only fork variants; each variant changes exactly ONE structural axis.

pattern: mixed（固定 named slots: body_form + mount + door_mechanism + signal_flag）

parents (pre-fill cells):
- P_TUNNEL = rec_classic-curbside-us-tunnel-shaped-residential-ma_20260612_132256_365232_809bed2b ← picture/Urban Environment/Mailbox/003.png（arched half-cylinder "tunnel" residential box; **front pull-down flap door REVOLUTE + side signal flag REVOLUTE 基线**; two-leg scrollwork post; helpers `_arch_skin`/`_arch_mouth_ring`/`_door_plate`; hollow outer+inner skin）
- P_PILLAR = rec_vintage-blue-cast-iron-us-mail-letter-collection_20260612_132251_186263_ed61e0ab ← picture/Urban Environment/Mailbox/002.png（boxy upright cast-iron LETTERS pillar box; rounded tubular cap; **pull-down hopper flap REVOLUTE 基线**; stands directly on ground / base skirt; helper `_half_cyl_top`; no post, no flag）
- P_SLANT = rec_curbside-blue-street-collection-box-on-a-single-_20260612_132234_578811_4565f031 ← picture/Urban Environment/Mailbox/001.png（slanted-top boxy street collection cabinet; **pull-down hopper door REVOLUTE 基线**; single square steel post + ground base plate + collar; no flag）

## 组合数预审（HARD GATE）
body_form 4 × mount 4 × signal_flag 2 = 32 distinct configs ≥ 10 ✓
（door_mechanism is a 4th real axis: pull_down_flap 基线 / side_hinge_swing / 等；including it only raises the product. Even the 3-slot product 4×4×2 = 32 gives broad coverage before counting door_mechanism or distinct-N.）

## Slot 候选覆盖

### Slot A: body_form（主体轮廓 —— 主结构轴）
| 候选 | record_id | 关键 part/helper | 状态 |
|---|---|---|---|
| tunnel_arched（基线） | P_TUNNEL | 半圆拱壳 `_arch_skin` outer+inner hollow | parent |
| boxy_pillar（基线） | P_PILLAR | 直立矩形柜 + 圆管顶盖 `_half_cyl_top` | parent |
| slanted_cabinet（基线） | P_SLANT | 斜顶矩形柜 wedge lid | parent |
| rounded_streetbox | rec_mailbox_var_streetbox | 圆肩连续曲面前+顶壳（lathe/曲面 hollow） | converged |

### Slot B: mount（支撑/安装）
| 候选 | record_id | 关键 part | 状态 |
|---|---|---|---|
| single_post（基线） | P_SLANT | 单方钢柱 + 地脚板 + collar | parent |
| two_legs_scroll（基线） | P_TUNNEL | 两腿 + 卷铁花饰 stand | parent |
| ground_pedestal | rec_mailbox_var_pedestal | 宽地座/裙边底座直立落地 | converged |
| two_legs_plain | rec_mailbox_var_twolegs | 双腿支架（for-loop leg helper + 脚垫 + 横梁） | converged |
| single_post（apply to tunnel body） | rec_mailbox_var_singlepost | 拱体改单钢柱（替换卷铁双腿） | converged |
| wall_bracket | rec_mailbox_var_wall | 平背墙挂板 root + 螺栓凸台（无落地） | converged |

### Slot C: door_mechanism（投递口机构 —— 真关节）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| pull_down_flap（基线） | P_TUNNEL / P_PILLAR / P_SLANT | 底铰下翻 hopper flap REVOLUTE (axis Y) | parent |
| side_hinge_swing | rec_mailbox_var_sidedoor | 侧立铰 cupboard 摆门 REVOLUTE (axis Z) | converged |
| (front_pull_door / drawer 可扩展) | — | — | reserved |

### Slot D: signal_flag（信号旗 —— 真关节, present/absent）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| absent（基线） | P_PILLAR / P_SLANT | 无信号旗 | parent |
| present（基线 on tunnel） | P_TUNNEL | 侧 semaphore 旗 REVOLUTE (axis Y, raise/lower) | parent |
| add_flag（to flagless body） | rec_mailbox_var_flag | 给无旗体加 L 形侧 semaphore 旗 REVOLUTE | converged |

## Multiplicity / Copy Logic
- count_param: leg_count（two_legs/scroll N=2, for-loop leg helper）; scroll_ring_count（卷铁 n_scroll）; decal_stripe_count（旗帜贴纸条 for-loop）; back_rib_count（铸铁背肋）
- copied object: 单腿+脚垫 / 单卷铁环 / 单贴纸条; naming leg_{i}/scroll_{k}_{s}/decal_stripe_{i}; placement 对称/规则栈; joint policy FIXED 随 body（Rule1 inline 装饰为 parent visual, 结构腿为 post part）
- 模板建议 N_range: legs 固定 2; scroll rings [2,4]; decal stripes [3,5]

## Loop / 可读性 notes
- P_TUNNEL: 良好 for-loop 发射 — legs `for i,xx in enumerate((...))`; scroll `for k in range(n_scroll)` × `for s,sgn`; decal `for i in range(n_stripe)`; mesh helpers 参数化。无明显手抄重复。
- P_PILLAR: side walls/jambs `for sgn` 对偶; back ribs `for i,yy in enumerate((...))`. OK。
- P_SLANT: side jambs / gable posts `for sgn`. 较少重复（单柱单门），无手抄腿堆叠问题。
- 通用 gotcha（继承 MEMORY）: Cylinder origin 是中心; 关节 origin 落在真实可见父面接触处, 不要发明毫米级 anchor pad; 拱壳 inner skin + mouth ring + back wall 的 allow_overlap 要在变体里一并复制; door/flap 闭合 expect_gap 容差沿用父值。

## Variant 计数
planned NEW variants = 8（streetbox, slanted, singlepost, pedestal, wall, flag, sidedoor, twolegs）; 3 parents pre-fill remaining cells. cap ~8-10 ✓

## 排除项 / dropped axes
- color / material / pure-scale —— 明确排除（suffix 规定不计为结构变更）。
- cap_profile（圆管顶 vs 平顶盖）—— 视为 body_form 子项, 不单列, 避免与 body_form 轴重叠产生伪轴。
- drawer / front_pull door 第三 door 候选 —— reserved, 本批未发射（side_hinge_swing 已足够使 door_mechanism ≥2 候选; 留待后续若需更多 cells 再开）。
- lock / coin slot 等小五金 —— inline 装饰, 非结构轴, 不发射为变体。
