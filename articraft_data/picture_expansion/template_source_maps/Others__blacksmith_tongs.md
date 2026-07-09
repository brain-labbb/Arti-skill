# Others / blacksmith tongs — template source map

pattern: parallel_children
parents:
- rec_model-a-pair-of-blacksmith-bow-ring-jaw-tongs-in_20260610_085153_097608_12053487 ← picture/Others/blacksmith tongs/001.png (bow/ring-jaw tongs: each arm ends in a half-ring so the two closed jaws form a round bow loop; occupies Slot A=`ring_bow_jaw`, with its own rein + flat-boss/round-rivet baseline)
- rec_model-a-pair-of-short-stout-blacksmith-pincers-n_20260610_085202_612713_9dcb992b ← picture/Others/blacksmith tongs/002.png (short stout nipper-style pincers: curved claw jaws meeting at a hardened bite edge + short stout handles; occupies Slot A=`pincer_claw_jaw` and the Slot B short-stout-handle form)
- rec_model-a-pair-of-long-slim-wolf-jaw-blacksmith-to_20260610_085212_387606_af04e104 ← picture/Others/blacksmith tongs/003.png (long slim wolf-jaw tongs: short flat wolf bits with a V-notch + transverse grooves, very long tapering reins; occupies Slot A=`wolf_flat_jaw` + Slot B=`long_straight_rein` baseline + Slot C=`flat_boss_domed_rivet` baseline. THIS is the fork baseline — cleanest module helpers `_jaw_solid`/`_boss_solid`/`_rein_solid`/`_rivet_solid` + `_place(flipped=)` two-arm emission + named visuals jaw/boss/rein/rivet)

Core kinematics shared by all three: two forged arms (`fixed_arm`/`moving_arm`, or `rear_arm`/`front_arm`)
cross at an elliptical boss and are joined by a single round-head **rivet** → one **REVOLUTE** joint
(`rivet_pivot` / `boss_rivet_pivot`, axis perpendicular to the tool plane). Squeezing the reins/handles
closes the jaws. Each arm = **jaw** (working end) + **boss** (lapped pivot plate) + **rein/handle**
(long end) + **rivet**. The three independent structural slots are: the jaw/working-end type, the
rein/handle form, and the boss/rivet pivot detail.

## Slot 候选覆盖

### Slot A:jaw / working end (the defining axis — what the closed jaws grip; `jaw` visual + `_jaw_solid` family)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| wolf_flat_jaw (baseline) | rec_model-...wolf-jaw...af04e104 (parent) | `jaw` visual ← `_jaw_solid()` flat polyline `JAW_PTS` converging to a V-notch tip + transverse `GROOVES` | short flat wolf bits, inner faces meet in a narrow V to grip bar; shallow grip grooves | converged |
| ring_bow_jaw | rec_model-...bow-ring-jaw...12053487 (parent) | `ring_jaw`+`jaw_tip` visuals ← `_ring_half()`/`_jaw_tip()` | each arm ends in a half-ring; two closed jaws form a round bow loop (caliper-like) | converged |
| pincer_claw_jaw | rec_model-...pincers...9dcb992b (parent) | `jaw`+`bite_edge` visuals ← `_jaw()`/`_bite_edge()` | short curved claw jaws meeting along a hardened bite edge (nipper/end-cutter form) | converged |
| flat_box_jaw | rec_blacksmith_tongs_var_flat_box_jaw | `jaw` visual ← rewritten `_jaw_solid()` (flat squared box bit, parallel flat gripping plates) | flat rectangular box/pickup bits that clamp flat plate; flat face, no V-notch | converged |
| vgroove_jaw | rec_blacksmith_tongs_var_vgroove_jaw | `jaw` visual ← `_jaw_solid()` + `_vgroove_cutter()` | deep longitudinal V-groove in each bit → diamond/V socket gripping round bar along its axis | converged |
| hollow_bit_jaw | rec_blacksmith_tongs_var_hollow_bit_jaw | `jaw` visual ← `_bit_jaw_solid()` + `_bore_cutter()` | opposed concave half-cylinder bits forming a round bore that cradles rod/pipe | converged |

### Slot B:rein / handle (the long end held in the hand; `rein` visual + `_rein_solid` family)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| long_straight_rein (baseline) | rec_model-...wolf-jaw...af04e104 (parent) | `rein` visual ← `_rein_solid()` lofted `REIN_SECTIONS` square→round taper, gently splayed, rounded tip | very long straight reins, square near the pivot blending to round at the tip | converged |
| scrolled_rein_ends | rec_blacksmith_tongs_var_scrolled_reins | `rein` visual ← `_rein_solid()` + `_scroll_solid()` | rein tip curls back into a flat forged scroll/volute (decorative fireplace-tong form) | converged |
| looped_eye_reins | rec_blacksmith_tongs_var_looped_eye_reins | `rein` visual ← `_rein_solid()` + `_eye_loop_solid()` | rein end bends round into a closed eye/ring with a real bore (hangs on a hook) | converged |
| square_bar_reins | rec_blacksmith_tongs_var_square_bar_reins | `rein` visual ← `_rein_solid()` with constant square sections (no round-tip taper) | chunky constant square-section forged bars full length, crisp edges | converged |
| (short_stout_handle) | rec_model-...pincers...9dcb992b (parent) | `handle`+`handle_tip` visuals ← `_handle()`/`_handle_tip()` | short stout handles instead of long reins (from the pincer parent) — secondary B form, available to the template if a short-rein module is wanted | converged (cross-parent) |

### Slot C:boss / rivet pivot detail (how the two arms are lapped + pinned; `boss`+`rivet` visuals)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_boss_domed_rivet (baseline) | rec_model-...wolf-jaw...af04e104 (parent) | `boss`←`_boss_solid()` half-lapped ellipse + `LAP_R` lap cut; `rivet`←`_rivet_solid()` shaft + two domed sphere heads | flat lapped elliptical boss + round-head rivet proud on both faces | converged |
| countersunk_flush_rivet | rec_blacksmith_tongs_var_countersunk_rivet | `rivet`←`_rivet_solid()` + `_countersink_cutter()` | rivet ends peened flush into conical countersinks; pivot flush with boss faces | converged |
| raised_boss_collar | rec_blacksmith_tongs_var_raised_boss_collar | `boss`←`_boss_solid()` with an inline raised cylindrical collar/hub around the rivet | turned cylindrical collar/washer ring lifts the rivet head on a stepped boss (inlined visual) | converged |

## Multiplicity / Copy Logic
- count_param: 无核心 N 复制轴。主结构恒为 **2 个对称 arm + 1 个 rivet pivot**(命名 slot,不是 N 复制)。
- 对称双臂的复制逻辑(模板要继承的 copy logic):moving arm = fixed arm 的同一锻件翻转 180°——
  `_place(solid, flipped=bool)` 镜像 + `for i,(arm_name,flipped) in enumerate(arms): model.part(arm_name)`
  循环发射两臂(hollow_bit / raised_boss_collar 变体已把它写成显式 enumerate 循环,是最干净的范例;
  其余变体沿用 parent 的 `_place(flipped=False/True)` 两次调用——下游模板统一折成 enumerate 循环)。
- 唯一的真循环发射子件:`_rein_solid()` 的 `REIN_SECTIONS` loft 站点 `for i in range(REIN_SECTION_PTS)`
  (截面多边形点),以及 jaw 上的 `GROOVES`/`DIMPLES` 列表循环——都是装饰/几何细分层,不是产品 N 轴。
- N 样本: 无 multiplicity 轴(arm 数恒为 2)。
- 模板建议 N_range: 无。jaw 的 groove 数、rein 的 loft 段数是连续/装饰参数(controlled local parameterization),不入 slot,不作 N 轴。
- copied object / naming / placement / joint policy: copied = arm(jaw+boss+rein+rivet 的整条锻件);
  naming = `fixed_arm`/`moving_arm`;placement = 同件翻转 180°(jaw/rein 换边,lap 面相对);
  joint policy = fixed_arm 为 root,moving_arm 经单个 `rivet_pivot` REVOLUTE 铰接(axis ⟂ 工具平面)。

## 组合数预审
- Slot A(jaw): 6 候选(wolf_flat / ring_bow / pincer_claw / flat_box / vgroove / hollow_bit)。
- Slot B(rein): 4 候选(long_straight / scrolled / looped_eye / square_bar;+ 跨 parent 的 short_stout_handle 备选)。
- Slot C(boss/rivet): 3 候选(flat_boss_domed / countersunk_flush / raised_boss_collar)。
- **组合积 = 6 × 4 × 3 = 72 ≥ 10 ✓**,且每个 slot ≥ 2 候选(无单候选 slot)。无 multiplicity 轴,组合数全部由候选乘积撑起——jaw(主机构轴)满配 6 个。

## 批次构成(cells 推导)
- 目标格子 = Slot A 6 + Slot B 4(核心)+ Slot C 3 = 13 候选 cell。
- parent 免费占格:wolf-jaw(A1+B1+C1)、bow-ring(A1)、pincers(A1 + short-handle B 备选)= 覆盖 A 的 3 格 + B/C 各 1 基线格。
- 待填 cells = A 缺 3(flat_box/vgroove/hollow_bit)+ B 缺 3(scrolled/looped_eye/square_bar)+ C 缺 2(countersunk/raised_collar)= **8 个变体**。
- 实际 fork = 8,与 cells 推导一致;全部从 wolf-jaw parent(最简洁基线)单轴 fork,compile-success、≥1 非 fixed joint、workbench-only、单轴 diff、picture-bound(`data/index/subcat/Others__blacksmith_tongs.jsonl` 11 条:8 变体 parent_record_id 已设 + 3 母资产)。

## 排除项(未来 compatibility matrix 素材)
- 无不收敛取值(8/8 变体一次 fork 即收敛)。
- 潜在跨槽干涉(留给 spec compatibility matrix 抽检,不预防性造变体):
  - `ring_bow_jaw`(闭合成环)× `hollow_bit_jaw` 互斥——同属 Slot A 不同候选,天然单选,不构成组合。
  - `raised_boss_collar`(Slot C 抬高 boss)× `pincer_claw_jaw`(短颈靠近 boss):collar 抬高量与短 neck 的贴合留隙需在模板侧重算,接口风险由 compatibility matrix 处置。
- 纯尺寸(rein 更长/更短、jaw 更宽、splay 角、boss 椭圆比例)不作候选——属模板连续参数,不入 slot。

## 同步备注
- 同步进 arti-template 时,parent ×3 + 变体 ×8 = 11 条全部 `rating=5`、workbench-only(两边都不进 dataset)。
- 来源定位用上表 part/joint/helper 名;`model.py:Lx-Ly` 行号在 arti-template 侧写正式 spec 时按当时文件填。
