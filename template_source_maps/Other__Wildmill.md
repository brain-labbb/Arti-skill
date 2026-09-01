# Other / Wildmill — template source map

pattern: mixed（固定 named slots:tower_form + roof + tail_vane;主机构 = rotor spin（CONTINUOUS）+ door（REVOLUTE）保持;**外加** sail_count 多重性轴,核心）

parents（1 个母资产,装饰性美式花园风车塔）:
- rec_model-a-decorative-american-style-garden-windmil_20260610_084933_134455_40ee4efc ← picture/Other/Wildmill/001.png（基线 = tower:tapered_four_sided × sails:5_lattice × roof:hip+cupola × tail:none；rotor CONTINUOUS + door REVOLUTE）

批次：other_wildmill_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

tower 4 × sail_count{4,5,12} × roof 3 × tail{none,vane} ≫ 10。rotor CONTINUOUS + door REVOLUTE 保持;tail 增 REVOLUTE yaw。

## Slot 候选覆盖

### Slot A:tower_form（塔身形态）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| tapered_four_sided（基线） | parent | 收锥四面板墙 | parent |
| round_cylindrical | rec_variant-tower-form-round-cylindrical-replace-the_20260617_152544_483906_113ad1f7 | 圆筒塔(lathe)| converged(2) |
| octagonal | rec_variant-tower-form-octagonal-replace-the-four-si_20260617_152544_475942_17358d7f | 八面塔 | converged(2) |
| open_lattice_truss | rec_variant-tower-form-open-lattice-truss-replace-th_20260617_152544_476199_61cab713 | 开放格构钢架(循环斜撑)| converged(2) |

### Slot B:roof（顶盖）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| hip_with_cupola（基线） | parent | 四坡顶 + 小亭 | parent |
| conical_round | rec_variant-roof-conical-round-replace-the-pyramidal_20260617_152739_557133_9d95796a | 圆锥顶(lathe)| converged(2) |
| open_observation_platform | rec_variant-roof-open-observation-platform-replace-t_20260617_153357_802419_c94f7047 | 开放观景平台 | converged(2) |

### Slot C:tail_vane（尾舵)
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| none（基线） | parent | 无 | parent |
| yawing_directional_vane | rec_variant-tail-vane-yawing-directional-vane-add-a-_20260617_153603_656000_e4472071 | 尾舵 REVOLUTE yaw(Z)| converged(3) |

## Multiplicity / Copy Logic（核心轴）
- count_param: **`sail_count`**（转子叶/帆数；基线 5）
  - N 样本: {4, 5, 12}
  - N=4 → rec_variant-sail-count-4-... / N=12 → rec_variant-sail-count-12-...（多叶美式抽水风车扇）
  - 模板建议 N_range: **[4, 18]**（装饰 lattice 4–8;多叶 fan 12–18）
  - copied object: 单 sail/blade;naming `sail_{k}`/`for k in range(SAIL_COUNT)`(母资产已循环);placement 等角;随 rotor CONTINUOUS 旋转

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A tower_form | 4 | round / octagonal / lattice_truss(3) |
| B roof | 3 | conical / open_platform(2) |
| C tail_vane | 2 | yawing_vane(1) |
| multiplicity sail_count | N∈{4,5,12} | N=4 / N=12(2) |

每槽 ≥2 + multiplicity 3 个 N → 满足 §8。**Wildmill 小类样本池就绪。**

## 排除项
- 无（8 变体全收敛）。
