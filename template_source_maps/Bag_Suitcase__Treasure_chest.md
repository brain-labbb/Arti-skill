# Bag_Suitcase / Treasure chest — template source map

pattern: mixed（固定 named slots: lid_profile + lock_mechanism + banding）
parents:
- rec_medieval-treasure-chest-with-a-domed-curved-lid-_20260605_133633_502277_6febc2df ← picture/Bag_Suitcase/Treasure chest/001.png（中世纪木+铁箍宝箱;chest_body + chest_lid + lock_hasp;2 REVOLUTE(盖+搭扣);_half_disk_profile 圆顶 + _barrel_band;**圆顶/前搭扣/铁箍带 基线**）

## 组合数预审
lid_profile 3 × lock_mechanism 2 × banding 2 = 12 ≥ 10 ✓

## Slot 候选覆盖
### Slot A: lid_profile（盖型 —— 主结构轴;盖 REVOLUTE 保持）
| 候选 | record_id | 关键 part/helper | 状态 |
|---|---|---|---|
| domed_barrel（基线） | P_chest | _half_disk_profile 圆顶弧盖 | parent |
| flat_plank | rec_chest_var_flatlid | 平板盖 | converged |
| gabled_peaked | rec_chest_var_gabled | 双坡尖顶盖 | converged |

### Slot B: lock_mechanism（锁扣）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| front_hasp（基线） | P_chest | lock_hasp 前翻搭扣 REVOLUTE | parent |
| padlock_loop | rec_chest_var_padlock | 锁鼻环 + 挂锁体 REVOLUTE 锁梁 | converged |

### Slot C: banding（箍带/包边）
| 候选 | record_id | 关键 part | 状态 |
|---|---|---|---|
| iron_straps（基线） | P_chest | _barrel_band for-loop 铁箍带 | parent |
| corner_brackets | rec_chest_var_corners | 角铁包边（for-loop 8 角） | converged |

## Multiplicity / Copy Logic
- count_param: band_count（_barrel_band 横箍带 for-loop 发射）
- 模板建议 N_range: [2,6]
- copied object: 单条铁箍带 / 单角铁;naming band_{i}/bracket_{i};placement 等距横箍 / 八角;joint policy FIXED 随 body/lid（Rule1）

## 排除项
- 无,本批 4/4 变体全部 compile=success、≥1非fixed joint、workbench-only、单轴 diff、绑定门禁通过;无出类目、无排除项。
