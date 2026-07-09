# Container / Lipstick — template source map

pattern: parallel_children

parents:
- rec_lipstick-tube-with-a-pull-off-cap-and-a-twist-up_20260606_074836_191857_d2d2bc8b ← picture/Container/Lipstick/001.png — round white slim twist-up lipstick. Occupies: Slot A `round_cylinder`, Slot B `twist_up_bullet`, Slot C `friction_pull_off`.
- rec_lip-gloss-tube-with-a-pull-out-cap-and-applicato_20260606_074848_098160_86a4438e ← picture/Container/Lipstick/002.png — octagonal gold lip gloss tube with doe-foot applicator wand. Occupies: Slot A `octagon_faceted`, Slot B `doe_foot_applicator`, Slot C `friction_pull_off`.

Both parents satisfy the §4 readability contract for their own structure (clear functional-layer parts: tube_base/cap/bullet_carrier/bullet for B; tube/cap_applicator for A; non-moving center_band and twist_marker are inlined as parent visuals, not FIXED parts). Neither parent has any "N identical sub-parts" — a lipstick has a single bullet/applicator and a single cap, so there is no for-i loop to audit; this is a parallel_children object, not a multiplicity object.

## Slot 候选覆盖

### Slot A:body cross-section family (整体截面/轮廓家族)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_cylinder | rec_lipstick-tube-with-a-pull-off-cap-and-a-twist-up_20260606_074836_191857_d2d2bc8b | tube_base / _tube_base_solid (circle.extrude) | 直筒圆柱体，恒定半径，cq circle 拉伸 | converged(parent) |
| octagon_faceted | rec_lip-gloss-tube-with-a-pull-out-cap-and-applicato_20260606_074848_098160_86a4438e | tube / _octa_prism / _tube_body_mesh | 正八边形棱柱体（apothem→circumradius polyline），刻面 | converged(parent) |
| square_rounded | rec_container_lipstick_var_square_rounded | tube_base / square-rounded prism helper | 圆角方形(squircle)截面棱柱，base+cap 同截面 | converged |
| tapered_waisted | rec_container_lipstick_var_tapered_waisted | tube_base / lathe waisted profile helper | 收腰沙漏轮廓，中段内收、底部外扩，lathe 扫掠侧壁 | converged |

### Slot B:dispense / applicator mechanism (出膏/涂抹机构)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| twist_up_bullet | rec_lipstick-tube-with-a-pull-off-cap-and-a-twist-up_20260606_074836_191857_d2d2bc8b | bullet_carrier(continuous bullet_twist z) + bullet(prismatic bullet_rise z) / _bullet_cup_solid / _bullet_red_solid | 旋转 carrier + 上升 bullet 螺旋链；斜切红膏头 | converged(parent) |
| doe_foot_applicator | rec_lip-gloss-tube-with-a-pull-out-cap-and-applicato_20260606_074848_098160_86a4438e | cap_applicator(prismatic tube_to_cap z) / _wand_mesh / _tip_mesh / _gloss_mesh | 帽身一体细杆 + doe-foot 软头，浸入 gloss 蓄液，抽出 | converged(parent) |
| push_up_swivel_pot | rec_container_lipstick_var_push_up_swivel_pot | bullet(single prismatic push_up z) / push slider helper | 单一上推滑块（无独立旋转 carrier），拇轮直推膏体 | converged |

### Slot C:cap retention joint type (顶盖固定/开启关节)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| friction_pull_off | rec_lipstick-tube-with-a-pull-off-cap-and-a-twist-up_20260606_074836_191857_d2d2bc8b (+ rec_lip-gloss-tube-with-a-pull-out-cap-and-applicato_20260606_074848_098160_86a4438e) | cap / cap_pull(prismatic z) [parent A: cap_applicator / tube_to_cap(prismatic z)] | 直拔摩擦盖，沿 +Z 平移脱出 | converged(parent) |
| screw_thread_twist_off | rec_container_lipstick_var_screw_thread_twist_off | cap / cap_unscrew(continuous/revolute z) / thread ridge helper | 旋拧盖，绕 +Z 旋转脱出，颈部+盖内可见螺纹 | converged |
| hinged_flip_cap | rec_container_lipstick_var_hinged_flip_cap | cap / cap_flip(revolute lateral) / hinge_lug 承托 visual | 翻盖，侧向 revolute 铰链翻起，顶缘可见铰耳 | converged |

## Multiplicity / Copy Logic
- count_param: 无。核心结构为固定 named slots(单 bullet/单 applicator + 单 cap)，无同构子件 × N 的复制逻辑。
- N 样本已覆盖: 无(non-multiplicity 小类)。
- 模板建议 N_range: 无。
- copied object / naming / placement / joint policy: 无。

组合数预审: Π(Slot A 4 × Slot B 3 × Slot C 3) × N(无) = 36 ≥ 10 ✓

## 批次规模推导
- 全部目标格子 = 4(A) + 3(B) + 3(C) 候选，按"一格一变体、单轴控制变量"逐槽数空格。
- parent 免费占格:B 占 A.round_cylinder / B.twist_up_bullet / C.friction_pull_off;A 占 A.octagon_faceted / B.doe_foot_applicator / C.friction_pull_off(与 B 同格，重复不增格)。
- 待填空格 cells = A:{square_rounded, tapered_waisted}(2) + B:{push_up_swivel_pot}(1) + C:{screw_thread_twist_off, hinged_flip_cap}(2) = 5。
- 计划变体数 = 5(简单类，落在 §2 的 5–7 区间)。每个变体只改 1 槽，从最近 parent fork(全部从 round 直筒的 parent B fork，base 形态最简、diff 最干净;A 只作为 octagon/doe-foot 两格的已占样本)。

## 排除项(未来 compatibility matrix 素材)
- 暂无连续不收敛取值(规划阶段尚未 fork)。
- 候选词汇表上限说明:口红的真实结构词汇较窄——body 截面再扩(如三角/心形)易出类目或失真,出膏机构超出 twist-up / doe-foot / push-up 三种亦无真实形态,故 Slot B 停在 3 候选;Slot C 的 bayonet/卡扣 与 friction_pull_off 在外观与 joint 语义上区分度低，未单列为候选(若后续需要可作为 friction_pull_off 的连续参数变体，不算 module)。
- 颜色/材质(金/银/白/红/磨砂等)与纯比例(更高更细/更粗短)按规则不作为轴,留给模板连续参数与采样器。
