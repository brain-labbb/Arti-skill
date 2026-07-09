# Other / TV — template source map

pattern: mixed（固定 named slots:cabinet_form + screen_tube + base_support;**外加** knob_count 多重性轴）

parents（1 个母资产,1980s 三星木纹 CRT 电视）:
- rec_model-a-vintage-1980s-samsung-crt-television-in-_20260610_084618_066506_f8f3f341 ← picture/Other/TV/001.png（基线 = cabinet:boxy_wood × screen:bulged_crt_glass × controls:channel_dial(CONTINUOUS) + 3 knob(REVOLUTE) × base:tabletop）

批次：other_tv_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

cabinet 3 × screen 3 × knob_count{2,3,5} × base 3 ≫ 10。controls 含 CONTINUOUS(dial)+REVOLUTE(knobs);base 含 swivel CONTINUOUS。

## Slot 候选覆盖

### Slot A:cabinet_form（机壳形态）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| boxy_wood（基线） | parent | 方木壳 | parent |
| rounded_space_age_pod | rec_variant-cabinet-form-rounded-space-age-pod-resha_20260617_124820_945331_1a475b57 | 圆角太空舱(lathe)| converged(4) |
| portable_with_handle | rec_variant-cabinet-form-portable-with-handle-make-i_20260617_124820_956157_cb7ecf95 | 便携带提梁(inline)| converged(4) |

### Slot B:screen_tube（屏幕/显像管）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| bulged_crt_glass（基线） | parent | 凸面 CRT(loft)| parent |
| flat_face_crt | rec_variant-screen-tube-flat-face-crt-replace-the-bu_20260617_124820_954860_d3e64ba3 | 平面 CRT | converged(4) |
| porthole_round | rec_variant-screen-tube-porthole-round-replace-the-r_20260617_124820_945784_fa608269 | 圆窗圆屏(lathe)| converged(4) |

### Slot C:base_support（支撑）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| tabletop（基线） | parent | 落地(无)| parent |
| splayed_console_legs | rec_variant-base-support-splayed-console-legs-lift-t_20260617_125155_795859_f3c0f936 | 四撇 console 腿 | converged(4) |
| swivel_base | rec_variant-base-support-swivel-base-mount-the-cabin_20260617_125257_605474_397aff58 | 旋转底座 CONTINUOUS(Z)| converged(5) |

## Multiplicity / Copy Logic
- count_param: **`knob_count`**（前面板控制旋钮数;基线 3）
  - N 样本: {2, 3, 5}
  - N=2 → rec_variant-knob-count-2-give-it-two-control-knobs-i_20260617_124820_953009_927cbb3a
  - N=5 → rec_variant-knob-count-5-give-it-five-control-knobs-_20260617_125152_442327_4c615bef
  - 模板建议 N_range: [1, 8]；copied object: 单 knob(KnobGeometry);naming `knob_{i}`/`for i in range(n)`;placement 沿 Z 等距;joint policy 各独立 REVOLUTE(+X)
  - 另:grille_slat 为循环固定件(slat_count,连续多重性,无关节)。

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A cabinet_form | 3 | rounded_pod / portable(2) |
| B screen_tube | 3 | flat_face / porthole(2) |
| C base_support | 3 | console_legs / swivel(2) |
| multiplicity knob_count | N∈{2,3,5} | N=2 / N=5(2) |

每槽 ≥2 + multiplicity 3 个 N → 满足 §8。**TV 小类样本池就绪。**

## 排除项
- 无（8 变体全收敛）。
