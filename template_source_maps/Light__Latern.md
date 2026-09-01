# Light / Latern — template source map

pattern: mixed(type/cap/carry 三个固定 named slots + side-guard 链式 multiplicity)

slug: hurricane_lantern_with_swinging_bail_handle

parent(基线,三槽 baseline 同源):
- rec_model-a-vintage-hurricane-kerosene-lantern-about_20260610_081109_101773_5da4cb46 ← picture/Light/Latern/001.png(vintage hurricane kerosene lantern;stepped fount `_fount_solid` + barrel glass globe `_globe_solid` + 两根 curved side air tube `_side_tube_solid`(**手镜像**:`tube_r` 与 `tube_l = tube_r.mirror("YZ")`)+ domed chimney `_top_assembly_solid` + brass wire bail `_bail_solid`;两条非 fixed joint:`body_to_bail_handle` REVOLUTE / `fount_to_wick_knob` CONTINUOUS。同时填 Slot A=hurricane_side_tube、Slot B=domed_vent_cap、Slot C=swinging_bail、guard_count N=2 baseline)

所有 12 个变体均 fork 自 5da4cb46,workbench-only(collections=['workbench'],未 promote)。

## 组合数预审

4(type)× 4(cap)× 4(carry)× 3(N 样本)≫ 10 ✓(本批每格恰一收敛样本,不为组合覆盖补造)

## Slot 候选覆盖

### Slot A:type / 光腔与挡风层主形(被 cap/carry 承托的主体)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| hurricane_side_tube(基线) | rec_...5da4cb46(parent) | `_side_tube_solid`(×2 手镜像)/ `_globe_solid` | barrel glass globe + 两根 curved 侧进气管 | parent(现成) |
| railroad_cage | rec_lantern_var_typeA_railroad_cage | `_guard_wire_solid` / `_guard_ring_solid` / `_guard_rings_solid`(`_green_body_solid` 内 union) | 圆柱形直立钢丝护笼罩住玻璃罩,取代侧管 | converged |
| candle_panel | rec_lantern_var_typeA_candle_panel | `_rounded_box_solid` / `_corner_post_solid` / `_frame_rail_solid` / `_glass_pane_solid` | 方形烛笼光腔,四面平玻璃由角柱+横档夹持 | converged |
| tubular_coldblast | rec_lantern_var_typeA_tubular_coldblast | `_draft_tube_solid`(×2 粗管)、BODY_TOP_Z=0.286 | 一对粗外冷风管喂更高烟囱、更大玻璃罩 | converged |

### Slot B:cap / 顶部排烟冠
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| domed_vent_cap(基线) | rec_...5da4cb46(parent) | `_top_assembly_solid` + `vent_slot_{i}`(12 槽循环) | 穹顶烟囱,穿孔通风带 + 外翻顶盘 | parent(现成) |
| flat_pierced_crown | rec_lantern_var_topB_flat_pierced_crown | `_top_assembly_solid`(重写),BODY_TOP_Z=0.260 | 低矮平顶穿孔冠 + 短筒领 + 顶环 | converged |
| conical_louver | rec_lantern_var_topB_conical_louver | `_top_assembly_solid`(高锥 funnel profile,open chimney throat) | 高瘦锥形百叶烟囱、敞口喉 | converged |
| peaked_roof | rec_lantern_var_topB_peaked_roof | `_tent_roof_solid`(polygon→polygon loft,TENT_ROOF_SIDES/TENT_EAVE_R/TENT_PEAK_R)+ `_top_finial_solid` | 多面尖顶/塔式宝顶 + 顶尖饰 | converged |

### Slot C:carry / 提携件(本槽决定第二条非 fixed joint)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| swinging_bail(基线) | rec_...5da4cb46(parent) | part `bail_handle` / `_bail_solid`;joint `body_to_bail_handle` REVOLUTE,axis=X(1,0,0),±100° | 顶部跨弧提梁,绕侧管 ear boss 摆动 | parent(现成) |
| top_swivel_ring | rec_lantern_var_carryC_top_ring | part `carry_ring` / `_carry_ring_wire_geometry`;joint `body_to_carry_ring` REVOLUTE,axis=Z(0,0,1),±180°(CARRY_LIMIT=π) | 固定顶部旋转吊环,绕竖轴自由打转 | converged |
| folding_side_strap | rec_lantern_var_carryC_folding_strap | part `side_strap_handle` / `_side_strap_hinge/barrel/loop/tab_solid`;joint `body_to_side_strap` REVOLUTE,axis=−Y(0,−1,0),0..112°(STRAP_LIMIT) | 侧面折叠提带,绕铰链向上翻折 | converged |
| swivel_hook_hanger | rec_lantern_var_carryC_hook_hanger | part `s_hook_hanger` / `_s_hook_geometry`;joint `body_to_s_hook` REVOLUTE,axis=X(1,0,0),±85°(HOOK_SWING_LIMIT) | 顶部旋转 S 钩吊挂件,前后摆动 | converged |

注:`fount_to_wick_knob`(CONTINUOUS,axis=+Y,`wick_knob`)在全部变体中保留为不变的第二/第三 joint,不进 slot,属固定附件。

## Multiplicity / Copy Logic(side guard 护笼/侧管)

- count_param: `guard_count`(N=10 变体已抽成模块常量 `GUARD_COUNT`;N=6 变体用 GUARD_R/GUARD_BOTTOM_Z/GUARD_TOP_Z/GUARD_WIRE_R 几何常量)
- copied object: 单根竖直 guard member / cage wire(`_guard_member_solid(angle)` 或 `_guard_wire_solid()`),两端 bottom_lug/top_lug 略插入下/上绿领,作真实固定支撑
- naming: `guard_member_{i}`(n2 loop)/ `guard_wire_{i}`(n6、n10),`for i in range(N)` 循环发射
- placement: 绕 Z 等角分布在玻璃罩外,`ang = 2.0*math.pi*i/N`,通过 `Origin(rpy=(0,0,ang))` 或 solid `.rotate(...,angle)` 摆位
- joint policy: **无 joint** —— guard 是焊在 `lantern_body` 根件上的固定结构 visual(夹在 `_lower_collar_solid` 与 `_top_assembly_solid`/上领之间),multiplicity 轴只增减固定支撑数,不产生新自由度
- N 样本已覆盖: {2 → rec_lantern_var_guardN_2_loop, 6 → rec_lantern_var_guardN_6, 10 → rec_lantern_var_guardN_10}
- 模板建议 N_range: **[2, 16]**(模板采样域;真实风灯护笼竖丝常 2–12,留余量到 16)
- **注意:parent 基线 N=2 是手写镜像(`tube_r` + `tube_r.mirror("YZ")`),未循环化。guardN_2_loop 才把它重写成 `guard_member_{i}` 循环,guardN_6/10 进一步参数化(`GUARD_COUNT`)。模板 multiplicity 的 copy-logic 源码应取自这三个 loop 变体,而非 parent。**

## 跨层接口(未来 InterfaceSpec 预填)

- guard ↔ body:每根 guard wire 的 bottom_lug 坐入 `_lower_collar_solid`(globe-seat 领,z≈0.102),top_lug 坐入上领/`_top_assembly_solid`(z≈0.206);mating face = 上下绿领环带,anchor = 半径 GUARD_R 处绕 Z 等角点,consumer = 固定 weld(无 joint)。
- carry ↔ body:bail/ring/strap/hook 均挂在顶部;bail/hook 经侧管/cage 上 `_ear_solid` ear boss 提供枢轴(world (0,0,PIVOT_Z)),ring 贴 chimney 顶领,strap 贴侧壁铰链座;joint origin 贴各自承托面,consumer joint = REVOLUTE(轴见 Slot C 表)。
- cap ↔ body:`_top_assembly_solid`/`_tent_roof_solid` 底缘坐在上领顶环(z≈0.205);mating ring = 上领顶 rim,anchor = Z 轴中心。
- wick_knob ↔ fount:`_knob_solid` stem 穿 fount 壁(scoped allow_overlap),CONTINUOUS,axis=+Y,origin=(0,KNOB_BASE_Y,KNOB_Z),全变体不变。

## 排除项(未来 compatibility matrix 素材)

- 暂无不收敛取值(本批 12 变体全部 converged)。
- 已主动排除(出类目/冲突风险,未列为候选):把 side guard 整成可动护笼(护笼应为固定支撑,加 joint 会读作非灯具机构);把玻璃罩换成不透明实体(失去"灯"语义,Slot A 不再读作 lantern);carry 槽同时挂 bail+ring 双提携件(语义冲突,留模板 compatibility matrix 裁决,不在 fork 批造)。Slot A=candle_panel × side-guard multiplicity 的跨格组合(方腔无圆周护笼位)亦留给 matrix。

---
## Post-fork verification (SEGMENT 1 complete)

All 12 planned variants forked from 5da4cb46 and verified on-disk: each `revisions/rev_000001/model.py` compiles, retains ≥1 non-fixed joint(carry REVOLUTE + wick CONTINUOUS;type/cap 变体保留 parent 的 bail+knob),collections=['workbench'](workbench-only,未 promote,category_slug 继承 `hurricane_lantern_with_swinging_bail_handle`),picture 绑入 `Light__Latern` 子类分片。Slot A/B/C 状态格 converged,guard multiplicity 轴覆盖 N∈{2,6,10}。Ready for SEGMENT 2 (spec authoring)。
