# Bag_Suitcase / Shopping bucket — template source map

pattern: mixed（固定 named slots:body + wall + handle + lid/closure 主机构 + interior;**外加** basket_count 嵌套堆叠 multiplicity 轴）

parents（3 个母资产,均矩形手提购物篮,基线 = body:rectangular × wall:slotted/perforated × handle:carry × lid:none × interior:plain）:
- rec_a-rectangular-red-plastic-shopping-basket-with-r_20260608_160213_314844_c383d977 ← picture/Bag_Suitcase/Shopping bucket/002.png（红色光面,圆角,竖向开槽,中央拱把;**solid-wall / 嵌套 N 轴的 fork 基线**）
- rec_a-rectangular-blue-plastic-hand-held-shopping-ba_20260608_160205_213315_254643d5 ← picture/Bag_Suitcase/Shopping bucket/001.png（蓝色,开槽/冲孔侧壁,单中央拱把）
- rec_plastic-shopping-basket-with-perforated-slotted-_20260605_133637_322465_b050efd5 ←（冲孔开槽壁 + 两折叠提梁;dual-folding 把手基线)

变体来源混合:① 20 个早期手动 fork(2026-06-09,API fork,各小类轴单改);② 3 个 qwen3.7-max/med 补造(2026-06-16):solid-wall 硬空格 + N=3 / N=5 嵌套堆叠。全部 compile rc=0、均有 URDF。
**登记缺口:** 这批未进 `picture_expansion/generated_assets.jsonl`(原 20 个是手动 fork,3 个 qwen 补造跑了 `--skip-search-index`)。此处直接列全 record_id;后续可回填 generated_assets.jsonl。

## 组合数预审


## Slot 候选覆盖

### Slot A:body_form（体形/足迹;连续比例由模板缩放,这里列结构不同的足迹形态)
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| rectangular（基线） | parents ×3 | 矩形箱体足迹 | parent |
| rounded_oval_deep | rec_change-the-overall-shape-to-a-rounded-oval-deep-_20260609_052813_481554_2ec27abe | 圆角椭圆深篮 | converged |
| hexagonal_footprint | rec_restructure-into-a-hexagonal-footprint-shopping-_20260609_054804_556262_50948264 | 六边形足迹 | converged |
| round_cylindrical_bucket | rec_change-the-footprint-to-a-round-cylindrical-buck_20260609_054825_387710_e78fd94a | 圆筒桶式 | converged |
| tapered_stackable | rec_redesign-as-a-tapered-stackable-nesting-shopping_20260609_054812_490053_6a8aa2c9 | 强内收锥度(可叠) | converged |
| shallow_wide_tray | rec_change-the-proportions-to-a-shallow-wide-tray-li_20260609_052818_608677_b5cd2054 | 浅宽托盘式 | converged |

### Slot B:wall_style（壁面/材质)
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| slotted_perforated_plastic（基线） | parents | 开槽/冲孔塑料壁 | parent |
| steel_wire_mesh | rec_change-the-body-material-to-a-stainless-steel-wi_20260609_051950_402636_1d416a15 | 不锈钢网篮 | converged |
| solid_smooth_plastic | rec_keep-the-same-rectangular-hand-held-shopping-bas_20260616_152752_435107_77eca573 | 全封闭光面 tub(`_tub_mesh`)| converged（2026-06-16 补） |

### Slot C:handle（提手类型)
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| fixed_central_arched（基线） | P2/P3 | 中央固定拱把 | parent |
| dual_folding_bail | rec_add-two-folding-bail-handles-to-make-the-tray-ar_20260609_062932_854922_35468439 | 两侧折叠提梁 REVOLUTE | converged |
| single_swing_bail | rec_add-a-single-central-swing-bail-handle-to-make-t_20260609_062931_487782_287b16ac | 中央半圆摆动提梁 REVOLUTE | converged |
| single_telescoping_pull_up | rec_replace-the-two-folding-carry-handles-with-a-sin_20260609_065759_301946_09d0f958 | 中央 U 形伸缩拉杆 | converged |
| （把手结构改造:单→双独立 见下) | rec_change-the-handle-structure-replace-the-single-f_20260609_052816_880253_08aedfc5 | 双独立把 | converged(作 dual 变体样本) |

### Slot D:lid_closure（**主机构槽**——篮子的开合动作)
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| open_no_lid（基线） | parents | 敞口无盖 | parent |
| hinged_top_lid | rec_add-a-hinged-top-lid-to-make-the-basket-articula_20260609_062924_147603_ab5a00e2 | 顶翻盖 REVOLUTE | converged |
| two_leaf_clamshell | rec_add-a-two-leaf-clamshell-lid-two-top-flaps-hinge_20260609_135304_181832_2822118b | 双叶蛤壳盖 REVOLUTE ×2 | converged |
| drop_front_gate | rec_add-a-hinged-drop-front-gate-to-the-wire-mesh-ba_20260609_054819_398495_4598924c | 前壁掉头门 REVOLUTE | converged |
| tilt_down_front_panel | rec_make-the-front-wall-a-low-hinged-panel-that-tilt_20260609_135307_236890_85ab5e66 | 前壁下翻倒料板 REVOLUTE | converged |

### Slot E:interior（内部机构)
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| plain（基线） | parents | 无内部机构 | parent |
| two_compartment_divider | rec_make-it-a-two-compartment-divided-basket-add-a-f_20260609_054821_407943_31a4dbd5 | 固定中央隔断双格 | converged |
| removable_inner_caddy | rec_add-a-removable-inner-caddy-tray-that-seats-on-t_20260609_135308_824047_5030ab7f | 可取内托盘 PRISMATIC 提起 | converged |

## Multiplicity / Copy Logic

- count_param: **`basket_count`**(嵌套堆叠购物篮:店门口那种可叠一摞)
- N 样本已覆盖: {1（单篮 = 各 parent / 各单体变体）, 3, 5}
  - N=3 → rec_restructure-the-parent-into-a-vertical-nesting-s_20260616_153521_561026_465484ba（runtime: 3 `basket_i` 链 + 3 `handle_joint_i` REVOLUTE + 2 FIXED 嵌套链）
  - N=5 → rec_restructure-the-parent-into-a-vertical-nesting-s_20260616_155259_758160_27719f51（runtime: 5 `basket_i` + 5 `basket_i_to_handle_i` REVOLUTE + 4 FIXED 嵌套链）
- 模板建议 N_range: **[1, 6]**（一摞嵌套篮的现实上限较小;sweep 小 N 高频)
- copied object: 整只 tapered 篮体 + 自带摆动 bail 把手,由共享 helper(`_basket_mesh` / `_handle_mesh`)发射
- naming: `basket_{i}` / `handle_{i}`,`for i in range(N)`(N=3/N=5 变体已用此结构,可直接作 module 源码)
- placement: 沿 +Z 等距递增(嵌套深度),`basket_0` 为根
- joint policy: 每个复制件**独立活动**——`basket_{i}_to_handle_{i}` REVOLUTE(每篮一把),篮体间 `basket_{i}_to_basket_{i+1}` FIXED 嵌套链

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A body_form | 6 | oval / hex / round / tapered / tray(5) |
| B wall_style | 3 | wire_mesh / **solid(补)**(2) |
| C handle | 4(+1 样本) | dual / swing / telescoping(+结构改造)(3–4) |
| D lid_closure(主机构) | 5 | hinged / clamshell / drop_front / tilt_down(4) |
| E interior | 3 | divider / caddy(2) |
| multiplicity basket_count | N∈{1,3,5} | N=3 / N=5(补) |

每槽均 ≥2(目标 3–6)converged 候选 + multiplicity ≥2 个 N → 满足 §8 完成定义。**Shopping bucket 小类样本池就绪。**

## 排除项（出"手提购物篮"范围,记作 compatibility matrix 素材,不进核心模板)

这 4 个已飘向手推车/拉杆/多体,故移出核心模板槽位:
- wheeled_pull_along（带轮拖行 + 拉杆)→ rec_convert-into-a-wheeled-pull-along-shopping-baske_20260609_133943_516464_43ad0ea4
- collapsible_four_hinged_walls（四壁折叠收平)→ rec_make-the-four-side-walls-hinge-so-the-basket-col_20260609_135259_845833_651978c4
- two_tier_stacked（双层叠放共享框)→ rec_make-it-a-two-tier-stacked-carry-basket-on-a-sha_20260609_135302_065520_332f5cbc
- under_basket_slide_drawer（底部抽屉)→ rec_add-a-slide-out-under-basket-drawer-tray-at-the-_20260609_065801_743480_52ba7599

（如后续想纳入,可各作一条薄可选轴,每条再补 1 个第二候选;当前按 §8.1 折出。)
