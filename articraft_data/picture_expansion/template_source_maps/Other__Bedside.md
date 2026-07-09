# Other / Bedside — template source map

pattern: mixed（固定 named slots:base_support + storage_mechanism(主机构) + handle;**外加** drawer_count 多重性轴）

parents（1 个母资产,现代双抽屉床头柜）:
- rec_model-a-modern-two-drawer-bedside-nightstand-app_20260610_083553_223535_c9162b4f ← picture/Other/Bedside/001.png（基线 = base:inset_floating_plinth × storage:two_prismatic_drawers × handle:chrome_bar × drawer_count:2）

批次：other_bedside_qwen37max_20260617（dashscope qwen3.7-max / medium）。10 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审


## Slot 候选覆盖

### Slot A:base_support（底座/支撑形式）
| 候选 | record_id | 关键 part/visual | 状态 |
|---|---|---|---|
| inset_floating_plinth（基线） | parent | `plinth`（四面内收悬浮）| parent |
| four_splayed_legs | rec_variant-base-support-four-splayed-legs-replace-t_20260617_091347_452678_65751baf | 四锥形外撇腿 | converged(2) |
| solid_toe_kick_box | rec_variant-base-support-solid-toe-kick-box-replace-_20260617_091347_458924_6dddcda5 | 落地踢脚箱基座 | converged(2) |
| hairpin_metal_legs | rec_variant-base-support-hairpin-metal-legs-replace-_20260617_091347_462104_dcdcb159 | 弯钢发夹腿(cylinder/mesh) | converged(2) |

### Slot B:storage_mechanism（**主机构槽**——储物开合动作）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| two_prismatic_drawers（基线） | parent | `carcass_to_{tag}_drawer` PRISMATIC ×2（+X）| parent |
| hinged_cabinet_door | rec_variant-storage-mechanism-hinged-cabinet-door-re_20260617_091347_463604_02643395 | 侧铰柜门 REVOLUTE（Z）| converged(1) |
| open_niche_over_drawer | rec_variant-storage-mechanism-open-niche-over-drawer_20260617_091347_458998_bd3941ca | 上部固定开放格 + 下单抽屉 PRISMATIC | converged(1) |
| drop_down_flap_door | rec_variant-storage-mechanism-drop-down-flap-door-re_20260617_091751_253009_2bf8cef6 | 底铰下翻门 REVOLUTE（Y）| converged(1) |

### Slot C:handle（提手/握持机构）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| chrome_bar_on_posts（基线） | parent | 两柱铬条把手 | parent |
| recessed_finger_pull | rec_variant-handle-recessed-finger-pull-replace-the-_20260617_091816_691920_968aa0b6 | 抠手槽(嵌入式)| converged(2) |
| round_knobs | rec_variant-handle-round-knobs-replace-the-chrome-ba_20260617_092217_210713_0504431c | 每抽屉两圆旋钮(循环)| converged(2) |

## Multiplicity / Copy Logic

- count_param: **`drawer_count`**（堆叠抽屉数；基线 2）
- N 样本已覆盖: {1, 2, 3}
  - N=1 → rec_variant-drawer-count-1-make-it-a-single-tall-dra_20260617_092542_364274_b8d0c6c4（1 PRISMATIC）
  - N=2 → parent（2 PRISMATIC）
  - N=3 → rec_variant-drawer-count-3-make-it-a-three-stacked-d_20260617_092726_313367_50c223fe（3 PRISMATIC）
- 模板建议 N_range: **[1, 4]**（床头柜抽屉数现实上限小）
- copied object: 整只 hollow open-top tray + slab front + 把手,由 `_build_drawer` helper 发射
- naming: 变体已要求改写为 `drawer_{i}` / `for i in range(n)`（母资产为 named top/bottom + helper,N 变体已循环化,可作 module 源码）
- placement: 沿 +Z 等距堆叠
- joint policy: 每抽屉**独立** PRISMATIC（+X,0..0.36 m）

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A base_support | 4 | legs / toe_kick / hairpin(3) |
| B storage(主机构) | 4 | cabinet_door / niche+drawer / dropdown_flap(3) |
| C handle | 3 | finger_pull / knobs(2) |
| multiplicity drawer_count | N∈{1,2,3} | N=1 / N=3(2) |

每槽 ≥2 + multiplicity 3 个 N → 满足 §8。**Bedside 小类样本池就绪。**

## 排除项
- 无（10 变体全收敛）。
