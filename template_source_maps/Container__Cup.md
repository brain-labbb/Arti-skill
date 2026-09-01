# Container / Cup — template source map

pattern: mixed (固定 named slots:body + lid/closure 主机构 + handle/grip; 叠加一个 wall-rib multiplicity 轴 N)

parents:
- rec_paper-coffee-cup-with-a-white-snap-on-sip-lid-th_20260606_074712_535833_2eedb76c ← picture/Container/Cup/001.png（kraft-paper 锥形纸杯:tapered ribbed hollow shell + 56 道竖肋 + 白色 snap-on 圆顶 sip 盖 + 翻盖 sip flap;基线 = body:tapered_cone × lid:snap_lift_dome × handle:ribbed_sleeve(无把手) × N_ribs≈56;**全批 fork 基线**）

单 parent 占四个轴各一格(body=tapered_cone / lid=snap_lift_dome / handle=none-ribbed_sleeve / N=56)。变体只填空格,每个变体从该 parent fork、只改一根目标轴。

部件树(parent):`cup`(root: `cup_shell` + `cup_ribs`)→ `lid`(`lid_shell`, PRISMATIC `cup_to_lid` +Z 抬升)→ `sip_flap`(`flap_knuckle` + `flap_tab`, REVOLUTE `lid_to_sip_flap`, lid 的 child)。helper: `_cup_shell()` / `_cup_ribs()`(已用 `for i in range(N_RIBS)` 循环 + 共享几何,符合 §4 可读性契约) / `_lid_solid()` / `_flap_solid()`。

## Slot 候选覆盖

### Slot A:body_form（杯体轮廓/footprint 形态家族;连续尺寸由模板缩放,这里只列结构不同的形态）
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| tapered_cone（基线） | rec_paper-coffee-cup-with-a-white-snap-on-sip-lid-th_20260606_074712_535833_2eedb76c | `cup` / `_cup_shell()`(三站 loft 锥壳) | 窄底宽口锥形 hollow shell | converged(parent) |
| straight_cylinder | rec_container_cup_var_straight_cylinder | `cup` / `_cup_shell()`→ 等半径圆柱壳 | 直壁圆柱马克杯轮廓 | converged |
| waisted_tumbler | rec_container_cup_var_waisted_tumbler | `cup` / `_cup_shell()`→ lathe/多站 loft 收腰 | 中段内收 contour travel tumbler | converged |
| stemmed_foot | rec_container_cup_var_stemmed_foot | `cup` + `cup_stem` + `cup_foot`(lathe revolve) | 杯碗 + 细柄 + 喇叭底足 pedestal | converged |

### Slot B:lid_closure（盖/封口机构——主机构槽,含 joint 拓扑多样:PRISMATIC ↔ REVOLUTE-twist ↔ REVOLUTE-flap）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| snap_lift_dome（基线） | rec_paper-coffee-cup-with-a-white-snap-on-sip-lid-th_20260606_074712_535833_2eedb76c | `lid` / `cup_to_lid`(PRISMATIC +Z) + `sip_flap` / `lid_to_sip_flap`(REVOLUTE) | snap-on 圆顶盖直抬 + 小翻盖 | converged(parent) |
| screw_twist_lid | rec_container_cup_var_screw_twist_lid | `lid` / `cup_to_lid`(REVOLUTE/continuous,绕 Z 旋拧) + 螺纹 rim collar | 螺纹旋拧密封盖(旋转封口) | converged |
| flip_top_lid | rec_container_cup_var_flip_top_lid | `lid`(FIXED 在 rim) + `spout_flap` / `lid_to_spout`(REVOLUTE 侧铰) | 固定盖 + 大翻嘴 flip-top(无直抬) | converged |
| slide_close_ring | rec_container_cup_var_slide_close_ring | `lid` + `slide_ring` / `lid_to_ring`(REVOLUTE 绕 Z) | 顶面旋转环对齐/遮挡 drink hole | converged |

### Slot C:handle_grip（提手 / 握持结构）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| ribbed_sleeve_none（基线） | rec_paper-coffee-cup-with-a-white-snap-on-sip-lid-th_20260606_074712_535833_2eedb76c | `cup_ribs`(parent visual,无 joint) | 竖肋防滑壁,无独立把手 | converged(parent) |
| loop_handle | rec_container_cup_var_loop_handle | `cup_handle`(C 形耳,swept/lathe,两端贴壁,FIXED 形态)+ 主机构盖仍带活动 joint | 侧 C 形马克杯耳把 | converged |
| snap_sleeve | rec_container_cup_var_snap_sleeve | `cup_sleeve` / `cup_to_sleeve`(PRISMATIC 沿 Z 滑配) | 瓦楞防烫套筒,上下滑配 | converged |
| fold_clip_handle | rec_container_cup_var_fold_clip_handle | `handle_bracket`(贴壁支架) + `fold_handle` / `bracket_to_handle`(REVOLUTE 水平铰,折叠) | 可折叠摆动提手 | converged |

## Multiplicity / Copy Logic
- count_param: **N_ribs**（杯壁竖肋/棱面复制数;parent 已用 `_cup_ribs()` 的 `for i in range(N_RIBS)` + 共享 segment helper + 等角 placement + 统一 FIXED 到 `cup` part 的 joint policy 实现,可机械读出）。
- N 样本已覆盖: {56(parent), 12, 8} → rec_paper-coffee-cup-with-a-white-snap-on-sip-lid-th_20260606_074712_535833_2eedb76c / rec_container_cup_var_n12_bold_flutes / rec_container_cup_var_n8_facet_panels（除 N（及 n8 的 facet 形态）外其余层全保持 parent 基线,copy logic 在样本间完全隔离）。
- 模板建议 N_range: [6, 80]（采样域远大于样本覆盖值正常;6 = 粗壮少棱面，80 = 细密肋）。
- copied object / naming / placement / joint policy: 复制对象 = 单根竖肋 segment（`_cup_ribs()` 内的 lofted rib）；命名 `rib_i` / 经 helper 发射；placement = 绕杯轴等角 `ang = 2*pi*i/N`、沿 taper 跟随半径；joint policy = 全部作为 `cup` part 的 visual（无独立 joint，随 root 走）。

## 组合数预审

## 格子覆盖（parent 占 4 格,变体填 11 空格）
- Slot A 3 空格: straight_cylinder / waisted_tumbler / stemmed_foot
- Slot B 3 空格: screw_twist_lid / flip_top_lid / slide_close_ring
- Slot C 3 空格: loop_handle / snap_sleeve / fold_clip_handle
- Multiplicity 2 空格: n12_bold_flutes / n8_facet_panels
计划 11 个变体(全部 converged;fork 在主循环执行)。

## 排除项（未来 compatibility matrix 素材）
- 纯尺寸(杯高/口径/容量)是模板连续参数 controlled local parameterization,不入 slot。
- 颜色/材质(kraft vs 白 vs 彩印)鼓励在结构变化上自由叠加,但永不算结构轴。
- 跨轴组合(如 stemmed_foot × screw_twist_lid)不专门造变体——组合由模板采样器免费生成;无已知特殊接口/干涉风险需组合抽检。
- 潜在收敛风险记录(待 fork 后回写):stemmed_foot 的细 stem 与 lift 盖测试、snap_sleeve 的 PRISMATIC 滑套不要与 lid 抬升 joint 冲突、flip_top_lid 需保证移除 PRISMATIC 后仍 ≥1 非 fixed joint(spout REVOLUTE 兜底)。
