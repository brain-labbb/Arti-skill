# Others / Safe — template source map

pattern: mixed
parents: rec_model-a-compact-freestanding-security-safe-about_20260610_085143_253768_86c0d4c8 ← picture/Others/Safe/001.png

母资产是 freestanding(落地)保险箱:hollow thick-walled box body(后/左/右/顶/底壁 + 一层内部 shelf)向前开口;一块厚门通过右侧 vertical-axis REVOLUTE `door_hinge`(0→~120°)向外开;门的自由边由 `bolt_carriage`(PRISMATIC `bolt_slide`)上的 N 根 locking bolt 锁入 latch 壁的 strike pockets;门面并联两个旋转控件——`combination_dial`(CONTINUOUS `dial_spin`)与 four-spoke `handle_wheel`(REVOLUTE `handle_spin`)。核心身份 = 落地金属箱 + 右铰厚门 + 门面 dial/handle + 门边 locking bolts;非 wall-recessed 保险箱、非普通柜门、非无锁机的收纳箱。

下游模板 slug 建议:`freestanding_security_safe`。

## Slot 候选覆盖

### Slot A:turn_handle(门面转动锁控机构 / 主机构槽)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| four_spoke_wheel | rec_model-a-compact-freestanding-security-safe-about_20260610_085143_253768_86c0d4c8 | part `handle_wheel`(hub + hub_cap + `spoke_{i}`×4 + `spoke_knob_{i}`×4 循环)/ `handle_spin`(REVOLUTE, door-normal, ±90°) | 四辐手轮 + 球端,中央 hub 坐 `handle_boss` | converged(parent) |
| tbar_handle | rec_safe_var_tbar_handle | part `tbar_handle`(hub + 单根水平 cross bar + 端 knob)/ `handle_spin`(REVOLUTE) | 单根水平 T 形/十字杠杆把手 | converged |
| lever_handle | rec_safe_var_lever_handle | part `handle_lever`(hub + 单根 lever arm + grip 端)/ `handle_spin`(REVOLUTE, 0→90°) | 单根下摆 lever 把手 | converged |

### Slot B:lock_entry(组合/密码输入界面)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rotary_dial | rec_model-a-compact-freestanding-security-safe-about_20260610_085143_253768_86c0d4c8 | part `combination_dial`(`dial_ring` + KnobGeometry `dial_knob` + `dial_tick_{i}`×12)/ `dial_spin`(CONTINUOUS) + door `dial_boss`/`dial_index_mark` | 单个旋转组合密码盘 | converged(parent) |
| electronic_keypad | rec_safe_var_keypad | door visuals `keypad_housing`/`keypad_display` + `_button_geometry()` helper + `button_{i}`×12 (3×4 网格,各自 PRISMATIC 按压) | 电子键盘替换 dial(数字保险箱);dial 部件移除 | converged |
| dual_rotary_dial | rec_safe_var_dual_dial | `_build_dial(model, dial_index)` helper → `combination_dial_{i}`×2(各 `dial_ring`/`dial_knob`/ticks)/ `dial_spin_{i}`×2(CONTINUOUS),for-range(2) 发射 | 上下叠放双密码盘 | converged |

### Slot C:interior_layout(内胆/分层结构)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_shelf | rec_model-a-compact-freestanding-security-safe-about_20260610_085143_253768_86c0d4c8 | safe_body `shelf` visual(嵌入两侧壁)+ floor/shelf `gold_ingot_{i}` 循环 props | 一层水平隔板 → 上下两舱 | converged(parent) |
| no_shelf | rec_safe_var_no_shelf | safe_body 去 shelf;单层 `gold_ingot_{i}` 循环堆于箱底 | 单一开放内腔 | converged |
| pull_out_drawer | rec_safe_var_inner_drawer | part `cash_drawer`(front/base/side/back 壁)/ `drawer_slide`(PRISMATIC, +X door-normal),坐于 body 内 runner 面 | 前拉式现金抽屉(额外 prismatic) | converged |

### Slot D:base_stance(底座/支撑形式)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flush_bottom | rec_model-a-compact-freestanding-security-safe-about_20260610_085143_253768_86c0d4c8 | safe_body `bottom_wall` 直接落地(无独立底座件) | 箱体平底直接落地 | converged(parent) |
| leveling_feet | rec_safe_var_leveling_feet | safe_body 内 `foot_{i}`×4 循环 visual(四角短圆柱,inline 非独立 part) | 四角调平脚,箱体抬起 | converged |
| plinth_base | rec_safe_var_plinth_base | safe_body 内 plinth/skirt visual(内缩矩形基座 / 四面裙板,inline) | 抬高的 plinth 踢脚座 | converged |

## Multiplicity / Copy Logic
- count_param: `bolt_count`(locking bolt 数;`BOLT_ZS` 列表长度驱动 `bolt_carriage` 内 `lock_bolt_{i}` 与 latch 壁 strike pocket 的双向循环)
- N 样本已覆盖: {2, 3, 5} → rec_safe_var_two_bolts / parent / rec_safe_var_five_bolts(five_bolts 自带 `lock_bolt_` 计数 run_tests 断言)
- 模板建议 N_range: [2, 6](门高有限,>6 栓位互相挤压;采样域可取 [2,6],样本只示范 copy logic)
- copied object: 单根 lock bolt(共享 bolt 几何 helper)+ 对应 latch 壁 strike pocket
- naming: `lock_bolt_{i}` / pocket 经 `for zr in BOLT_ZS` 循环
- placement: 沿门高在 `BOLT_ZS` 等距分布(门-normal 方向同一 carriage 上)
- joint policy: 全部 N 根 bolt 随单一 `bolt_carriage`(一个 PRISMATIC `bolt_slide`)整体平移,bolt 自身不单独活动

## 跨槽接口(未来 InterfaceSpec 素材)
- door 面是 Slot A(handle_boss)+ Slot B(dial_boss / keypad_housing)的共享 mating face;两控件并联挂在 door 上,joint 原点贴 boss 接触面,door-normal 为公共轴。
- door 自由边 ↔ latch 壁:bolt 行程跨 jamb gap 进 strike pocket;Slot 多/少 bolt 时 pocket 数须同步(已在 two/five_bolts 验证)。
- body 底面 ↔ Slot D base:floor 接触面;feet/plinth 为 inline body visuals(无 FIXED 装饰 part)。

## 排除项(未来 compatibility matrix 素材)
- 无:全部 10 个变体首轮 compile + run_tests 收敛(failures=0),无 blocked 格子。
- 注意(组合约束,非排除):Slot B=electronic_keypad 时无旋转 dial,模板侧 `dial_spin` 拓扑应缺省;Slot C=pull_out_drawer 引入第二个 PRISMATIC,与 Slot A/B 无干涉但 module_topology 计为不同拓扑。

## 批次说明
- 实际变体数 10 = cells(Slot A 2 + Slot B 2 + Slot C 2 + Slot D 2 + multiplicity N 2),与 §2 推导一致,无补造/无 blocked。
- 全部 workbench-only(collections=['workbench'], category_slug=None),未 promote;均 picture-bound 于 Others/Safe(data/index/subcat/Others__Safe.jsonl)。
