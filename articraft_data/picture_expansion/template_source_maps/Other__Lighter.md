# Other / Lighter — template source map

pattern: parallel_children（固定 named slots:body_shape + ignition_mechanism(主机构) + cap + flame_adjust;无核心 multiplicity）

parents（1 个母资产,BIC 式一次性燧石口袋打火机）:
- rec_model-a-classic-disposable-flint-pocket-lighter-_20260610_084427_650680_c45ce5f7 ← picture/Other/Lighter/001.png（基线 = body:stadium_oval × ignition:flint_spark_wheel(CONTINUOUS wheel + REVOLUTE lever) × cap:open_hood × flame_adjust:none）

批次：other_lighter_qwen37max_20260617（dashscope qwen3.7-max / medium）。6 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

body 3 × ignition 2 × cap 3 × flame_adjust 2 = 36 ≫ 10。ignition 含 CONTINUOUS+REVOLUTE 与 PRISMATIC 两种拓扑;cap 含 REVOLUTE(翻盖)与 PRISMATIC(滑盖);flame_adjust 含 CONTINUOUS。

## Slot 候选覆盖

### Slot A:body_shape（壳体截面家族）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| stadium_oval（基线） | parent | 椭圆 stadium 储液仓 | parent |
| round_cylindrical | rec_variant-body-shape-round-cylindrical-change-the-_20260617_111044_357663_099fd912 | 圆筒(lathe) | converged(2) |
| rectangular_slab | rec_variant-body-shape-rectangular-slab-change-the-r_20260617_111044_359049_d28014ae | 矩形扁板 | converged(2) |

### Slot B:ignition_mechanism（**主机构槽**——点火机构）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| flint_spark_wheel（基线） | parent | `spark_wheel` CONTINUOUS(X) + `fuel_lever` REVOLUTE | parent |
| piezo_push_button | rec_variant-ignition-mechanism-piezo-push-button-rep_20260617_111044_357162_62f5cdf2 | 顶部按钮 PRISMATIC(-Z) | converged(1) |
> 注:一次性打火机点火现实词汇表仅燧石轮 vs 电子压电两类,本槽 2 候选已近真实上限(§5 允许 ≥2)。

### Slot C:cap（喷嘴盖/罩）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| open_hood（基线） | parent | 无盖(固定 hood band) | parent |
| flip_top_cap | rec_variant-cap-flip-top-cap-add-a-hinged-flip-top-c_20260617_111044_356651_47195100 | 翻盖 REVOLUTE(X) | converged(3) |
| sliding_nozzle_cover | rec_variant-cap-sliding-nozzle-cover-add-a-sliding-n_20260617_111044_358289_b701f9ec | 滑盖 PRISMATIC(+Y) | converged(3) |

### Slot D:flame_adjust（火焰高度调节）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| none（基线） | parent | 无 | parent |
| flame_height_thumb_wheel | rec_variant-flame-adjust-flame-height-thumb-wheel-ad_20260617_112200_053612_fe4840b8 | 喷嘴基部调节轮 CONTINUOUS(Z) | converged(3) |

## Multiplicity / Copy Logic
- 无核心复制逻辑(打火机为固定 named slots)。hood vent 穿孔 / wheel 滚花齿可作 inline 循环视觉,非结构 multiplicity。

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A body_shape | 3 | round / rectangular(2) |
| B ignition(主机构) | 2 | piezo(1)（近真实上限） |
| C cap | 3 | flip_top / sliding(2) |
| D flame_adjust | 2 | thumb_wheel(1) |

每槽 ≥2 → 满足 §8(主机构 2 候选已注明近真实上限,模板侧可将 cap 作机构多样性补充)。**Lighter 小类样本池就绪。**

## 排除项
- 无（6 变体全收敛）。
