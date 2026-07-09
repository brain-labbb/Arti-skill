# Other / Water dispenser — template source map

pattern: mixed（固定 named slots:tap_type(主机构) + body_form + base + reservoir;**外加** faucet_count 多重性轴,核心,由多母资产预填）

parents（4 个可 fork 母资产 + 1 个 map-only 缺失）:
- P_cooler rec_model-a-countertop-bottled-water-cooler-the-base_20260610_084813_230850_6fffaa2f ← .../001.png（瓶装冷水机:hot/cold paddle REVOLUTE ×2 + drip_tray PRISMATIC + bottle FIXED）
- P1 single rec_model-a-single-faucet-countertop-beverage-dispen_20260610_084832_965957_b6696ffc ← .../002.png（**fork 基线**:矩形饮料箱 + 1 push tap）
- P3 triple rec_model-a-triple-faucet-countertop-beverage-dispen_20260610_084909_908684_acfaa2cf ← .../004.png（3 push tap）
- P4 four_tap rec_model-a-four-tap-t-style-countertop-beverage-dis_20260610_084923_661531_3f3a9da3 ← .../005.png（T 型 4 tap,`faucet_{i}`/`tap_handle_{i}_pivot` REVOLUTE 循环）
- （dual .../003.png 在 map 中但 record 目录缺失,faucet_count=2 空格由 fork 补 [faucet-count-2]）

批次：other_water_dispenser_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

tap_type 3 × body_form 3 × base 3 × faucet_count{1,2,3,4,6} ≫ 10。tap_type 含 REVOLUTE(杆/旋塞)/PRISMATIC(按钮);base 含 PRISMATIC(抽拉滴水盘)。

## Slot 候选覆盖

### Slot A:tap_type（**主机构槽**——出水阀）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| push_lever（基线 P1/P3/P4） | parents | tap_handle REVOLUTE | parent |
| push_button | rec_variant-tap-type-push-button-replace-the-push-le_20260617_141101_313670_1a8f4bb6 | 按钮 PRISMATIC | converged(1) |
| twist_spigot | rec_variant-tap-type-twist-spigot-replace-the-push-l_20260617_141101_314016_ffb293c3 | 旋塞 REVOLUTE(spout 轴) | converged(1) |
| paddle（P_cooler）| P_cooler | hot/cold paddle REVOLUTE | parent |

### Slot B:body_form
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| rectangular_box（基线） | P1/P3/P4 | 矩形饮料箱 | parent |
| bottled_cooler_cabinet（P_cooler）| P_cooler | 瓶装冷水柜 | parent |
| cylindrical_urn | rec_variant-body-form-cylindrical-urn-reshape-the-re_20260617_141101_313532_b7793c5c | 圆筒饮料桶(lathe)| converged(1) |

### Slot C:base / reservoir
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| countertop（基线） | parents | 台面 | parent |
| floor_standing_pedestal | rec_variant-base-floor-standing-pedestal-mount-the-b_20260617_141514_076723_9a1aa34a | 落地柜座 | converged(3) |
| removable_drip_tray | rec_variant-base-removable-drip-tray-add-a-removable_20260617_142358_226577_7e820f42 | 抽拉滴水盘 PRISMATIC | converged(4) |
| inverted_top_bottle（reservoir）| rec_variant-reservoir-inverted-top-bottle-add-an-inv_20260617_142452_723369_870bb3bc | 顶置倒装水瓶 FIXED | converged(1) |

## Multiplicity / Copy Logic（核心轴）
- count_param: **`faucet_count`**（出水龙头数）
  - N 样本: {1(P1), 2, 3(P3), 4(P4), 6} —— 母资产已覆盖 1/3/4,fork 补 2 与 6
  - N=2 → rec_variant-faucet-count-2-... / N=6 → rec_variant-faucet-count-6-...
  - 模板建议 N_range: **[1, 8]**；copied object: 单 faucet + tap_handle;naming `faucet_{i}`/`tap_handle_{i}`(P4 已 `for i in range(4)`);placement 沿 X 等距;joint policy 各 tap 独立 REVOLUTE

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A tap_type(主机构) | 4 | push_button / twist_spigot(2)（+paddle 基线） |
| B body_form | 3 | cylindrical_urn(1)（+2 基线形态） |
| C base/reservoir | 4 | floor_pedestal / drip_tray / top_bottle(3) |
| multiplicity faucet_count | N∈{1,2,3,4,6} | N=2 / N=6(2;1/3/4 母资产) |

每槽 ≥2 + multiplicity 5 个 N → 满足 §8。**Water dispenser 小类样本池就绪(faucet_count 多重性由 4 母资产 + 2 fork 充分覆盖)。**

## 排除项
- dual-faucet（.../003.png）record 目录缺失,以 fork [faucet-count-2] 补该格。
