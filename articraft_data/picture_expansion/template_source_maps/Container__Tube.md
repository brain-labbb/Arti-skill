# Container / Tube — template source map

pattern: parallel_children（固定 named slots: body_footprint + closure_mechanism；闭合件单子 part 挂在 body 颈口上）
parents:
- rec_blue-sunscreen-squeeze-tube-with-a-hinged-flip-t_20260606_075018_228582_60b00467 ← picture/Container/Tube/001.png（蓝色防晒挤压管;body=rounded-rectangle superellipse slab + open neck/bore;closure=白色垂直 PRISMATIC 升降盖 cap_lift;label 标记用 for-i-range 循环发射;占 Slot A=slab_rect × Slot B=lift_cap 格）
- rec_cosmetic-cream-squeeze-tube-with-a-white-screw-c_20260606_075026_811431_7c11d416 ← picture/Container/Tube/002.png（淡黄化妆膏挤压管;body=round-to-flat 底缝→近圆肩部 loft;closure=白螺旋盖 双解耦 CONTINUOUS 旋转(cap_rotate)+PRISMATIC 滑出(cap_slide);nozzle 螺纹环/cap 滚花筋用 for 循环发射;占 Slot A=round_to_flat × Slot B=screw_cap 格）

## 组合数预审
body_footprint 5 × closure_mechanism 8 = 40 ≥ 10 ✓
（深化:闭合/分配机构主槽由 4 → 8,管身截面槽由 4 → 5;旧 4×4=16 已淘汰）

## Slot 候选覆盖

### Slot A: body_footprint（管身截面/轮廓家族 —— 形状家族更换,非缩放）
| 候选(未来 module) | record_id | 关键 part/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| slab_rect（基线） | rec_blue-sunscreen-squeeze-tube-with-a-hinged-flip-t_20260606_075018_228582_60b00467 | body / tube_body；helper _oval_loop + _loop_xy（超椭圆 p=3.4 loft） | 圆角矩形扁板,Y 宽 X 薄,broad 正面 | converged(parent) |
| round_to_flat（基线） | rec_cosmetic-cream-squeeze-tube-with-a-white-screw-c_20260606_075026_811431_7c11d416 | tube_body / tube_body；helper _body_solid + _rounded_rect_pts（底扁缝→近圆肩 loft + shell 中空） | 底部扁压缝,上行收圆,经典圆-扁挤压管 | converged(parent) |
| cylindrical | rec_container_tube_var_cylindrical | body / tube_body；目标:近圆等径直筒 loft helper | 直筒等径圆截面 barrel,无扁面 | converged |
| oval_lozenge | rec_container_tube_var_oval_lozenge | body / tube_body；目标:平滑椭圆/lozenge loop helper | 软扁椭圆截面,无 broad 平面,纯圆顺 | converged |
| tapered_cone | rec_container_tube_var_tapered_cone | tube_body / tube_body；目标:扁缝底→窄肩单调收锥 cone-taper loft helper | 宽扁底压缝→上行单调收窄至细圆肩,连续锥/漏斗 taper(非 barrel 非挤压鼓肚) | converged |

### Slot B: closure_mechanism（顶部闭合/分配机构 —— 主机构轴,决定 joint 类型）
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| lift_cap（基线） | rec_blue-sunscreen-squeeze-tube-with-a-hinged-flip-t_20260606_075018_228582_60b00467 | lift_cap / cap_lift(PRISMATIC +Z)；helper _cap + _open_neck | 白盖沿 +Z 垂直升降罩住开口颈 | converged(parent) |
| screw_cap（基线） | rec_cosmetic-cream-squeeze-tube-with-a-white-screw-c_20260606_075026_811431_7c11d416 | cap + cap_carrier / cap_rotate(CONTINUOUS +Z) + cap_slide(PRISMATIC +Z)；helper _cap_mesh + _nozzle_mesh | 双解耦螺旋盖:绕 +Z 旋转 + 沿 +Z 滑出螺纹 nozzle | converged(parent) |
| flip_top | rec_container_tube_var_flip_top | 目标:flip_lid / lid_hinge(REVOLUTE 水平轴)；铰接 tab 锚在颈口 collar 面 | 卡扣翻盖绕水平活铰摆开/合,贴颈口承托面 | converged |
| pull_cone | rec_container_tube_var_pull_cone | 目标:pull_cap / cap_pull(PRISMATIC +Z)；锥形 nozzle helper | 尖锥分配嘴 + 尖头拉拔盖沿 +Z 直拔脱出 | converged |
| standup_flip_cap | rec_container_tube_var_standup_cap | 目标:base_cap(宽扁八角站立盘)+flip_lid / lid_hinge(REVOLUTE 水平轴)；盘锚在颈口/肩面 | 宽扁八角站立底盘盖(管倒立站在盖上)+ 小卡扣翻盖绕水平活铰摆开 | converged |
| slant_applicator | rec_container_tube_var_slant_applicator | 目标:slant_tip(斜切分配嘴)+ slant_cap / cap_pull(PRISMATIC 沿嘴轴)；斜口 nozzle helper | 斜切对角出料涂抹嘴(精华/眼霜杆)+ 配套斜口卡盖沿嘴轴直拔 | converged |
| roller_ball | rec_container_tube_var_roller_ball | 目标:ball_socket 座 + roller_ball / ball_spin(CONTINUOUS/球关节)+ overcap / cap_pull(PRISMATIC +Z)；穹顶球座 helper | 穹顶球座内自由旋转的滚珠涂抹头(真实滚动关节)+ 可拔 overcap | converged |
| twist_up_stick | rec_container_tube_var_twist_up | 目标:lift_platform / platform_rise(PRISMATIC +Z) + twist_base / base_twist(CONTINUOUS +Z)；升降平台 helper | 旋底升膏:底部滚花旋环驱动管口内平台沿 +Z 升降(膏/止汗棒升降机构) | converged |

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots（body_footprint + closure_mechanism）。
- 现有复制逻辑均为装饰/握持细节,非身份级 N 轴:label 标记（vertical_brand_mark_{i}/small_front_label_{i}）、nozzle 螺纹环（k 循环）、cap 滚花筋（rib，i 循环）—— 这些是 for 循环发射的细节带,模板侧可作连续/装饰参数,但不构成结构 multiplicity 轴。
- N 样本已覆盖: 无（不以 N 作轴）。
- 模板建议 N_range: 无身份级复制件;装饰带细节数（螺纹环 / 滚花筋 / 标记）可作连续装饰参数,非 module 轴。
- copied object / naming / placement / joint policy: 无（无需 copy-logic module）。

## 深化记录（re-audit）
- 主机构槽(Slot B closure/dispenser)由 4 → 8:在原 lift_cap/screw_cap/flip_top/pull_cone 基础上,新增 4 个真实且结构互异的分配机构 —— standup_flip_cap(宽扁站立底盘盖,管倒立站立)、slant_applicator(斜切涂抹嘴+斜口拔盖)、roller_ball(穹顶球座滚珠涂抹头,真实滚动关节)、twist_up_stick(旋底升膏平台机构,膏/止汗棒)。均为软膏/精华/护理管常见真实分配头,joint 类型互不相同(REVOLUTE/PRISMATIC/CONTINUOUS+球关节/双解耦升降),非缩放非配色。
- 管身槽(Slot A body)由 4 → 5:新增 tapered_cone(扁底单调收锥至窄肩的连续锥/漏斗截面),区别于 round_to_flat 的挤压鼓肚与 cylindrical 的等径直筒。
- 每个新候选 = 一个新空格 = 一个单轴变体,其余功能层保持 parent 基线;新闭合候选均自最近 parent fork:standup/slant/roller 均挂带 nozzle/shoulder 的 P2(7c11d416)以承接出料嘴几何,twist_up 挂 P1(60b00467,slab 身)以承接平面口升降平台;tapered_cone(身槽)挂 P2(round-to-flat 身最近)。

## 排除项（未来 compatibility matrix 素材）
- 无 multiplicity 轴:本小类是单口分配容器,身份在"管身截面 × 顶部闭合/分配机构"两根轴,无"× N 同构子件"复制逻辑,故 multiplicity = 无,组合数靠 5×8=40 候选堆厚达标。
- 跨轴组合(如 cylindrical × screw_cap、round_to_flat × flip_top、tapered_cone × roller_ball)不专门造变体:由模板采样器自由产出;两参考图已分别占据对角两格(slab_rect×lift_cap、round_to_flat×screw_cap),变体均以最近 parent 做单轴干净 diff 填补各槽余格。
- 已考虑但**剔除**(非结构/越类):①泵头压泵(pump dispenser)—— 真实但属于"瓶/泵 Bottle"身份,装在硬瓶上而非软挤压管,出本小类故不收;②金属软膏管尾部夹封折叠(crimp fold)细节 —— 仅尾封工艺细节,非顶部分配机构,可作连续/装饰参数不单列;③不同颜色/容量/标签文字 —— 纯配色/缩放,不计候选。
- 暂无连续不收敛取值需记录;若 flip_top/standup 活铰锚定、roller_ball 球座穿插、twist_up 平台升降穿模反复失败,fork 阶段回写此处。
