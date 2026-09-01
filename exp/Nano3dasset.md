# Nano3D 实验评测资产清单

本文档记录依据 [`Nano3d.md`](/mnt/zsn/lyb/arti-skill/exp/Nano3d.md) 选出的 33 个可进行实验评测的资产。

## 1. 筛选标准

每个入选资产均满足：

- 资产目录存在；
- 存在 `model.py`；
- 存在 `model.urdf`；
- 存在 `assets/` 目录；
- 存在 `compile_report.json`，且其中 `status=success`；
- 不依赖 `seed_exports/_rejected/` 中被拒绝的主导出结果。

难度按照 URDF 中的 `<link>` 节点数划分，与实验设计一致：

| 难度 | 判定标准 | 数量 |
|---|---:|---:|
| L1 | ≤3 links | 11 |
| L2 | 4–8 links | 11 |
| L3 | >8 links | 11 |

本次共选择 33 个资产：

- `seed_exports`：23 个；
- `seed_exports_physics_10`：10 个；
- 所有资产的编译状态均为 `success`。

表中的 `assets 文件数` 是递归统计对应资产目录下 `assets/` 中的文件总数，不等同于三角面数。

## 2. 资产总表

| # | 难度 | 资产 | 来源 | seed | URDF links | URDF joints | assets 文件数 | 编译状态 | physics 验证 |
|---:|:---:|---|---|---:|---:|---:|---:|---|---|
| 1 | L1 | Astronomy Antenna dish | `seed_exports` | 343 | 3 | 2 | 17 | success | N/A |
| 2 | L1 | Door Double Door | `seed_exports` | 222 | 3 | 2 | 114 | success | N/A |
| 3 | L1 | Electrical Wiring Wire stripper | `seed_exports` | 5943 | 3 | 2 | 34 | success | N/A |
| 4 | L1 | Bag / Suitcase / Shopping bucket | `seed_exports_physics_10` | 148 | 2 | 1 | 4 | success | 未生成 `validation_report.json` |
| 5 | L1 | Door Other | `seed_exports_physics_10` | 431 | 3 | 2 | 18 | success | 未生成 `validation_report.json` |
| 6 | L1 | Door Trap door | `seed_exports_physics_10` | 1495 | 3 | 2 | 6 | success | 未生成 `validation_report.json` |
| 7 | L1 | Handtools clothes peg | `seed_exports_physics_10` | 453 | 3 | 2 | 5 | success | **通过**，`success=true`，`dataset_ready=true` |
| 8 | L1 | Urban Environment bucket1 | `seed_exports_physics_10` | 1009 | 2 | 1 | 5 | success | 未生成 `validation_report.json` |
| 9 | L1 | Urban Environment bucket2 | `seed_exports_physics_10` | 6101 | 3 | 2 | 6 | success | 未生成 `validation_report.json` |
| 10 | L1 | Garlic press | `seed_exports_physics_10` | 2 | 3 | 2 | 14 | success | 未生成 `validation_report.json` |
| 11 | L1 | Guitar tuning peg mechanism | `seed_exports_physics_10` | 202 | 3 | 2 | 10 | success | 未生成 `validation_report.json` |
| 12 | L2 | Industrial Mine cart | `seed_exports` | 13 | 7 | 6 | 16 | success | N/A |
| 13 | L2 | C-shaped sofa side table | `seed_exports` | 0 | 4 | 3 | 14 | success | N/A |
| 14 | L2 | Dressing table | `seed_exports` | 0 | 6 | 5 | 20 | success | N/A |
| 15 | L2 | Butter maker | `seed_exports` | 0 | 8 | 7 | 20 | success | N/A |
| 16 | L2 | Garden pruner | `seed_exports` | 0 | 6 | 5 | 5 | success | N/A |
| 17 | L2 | Ice cream machine | `seed_exports` | 0 | 5 | 4 | 16 | success | N/A |
| 18 | L2 | Juicer press with handle | `seed_exports` | 0 | 7 | 6 | 25 | success | N/A |
| 19 | L2 | Bi-fold closet door system | `seed_exports` | 0 | 7 | 6 | 16 | success | N/A |
| 20 | L2 | Riot shield | `seed_exports` | 3141 | 4 | 3 | 9 | success | N/A |
| 21 | L2 | Stationary Pencil sharpener | `seed_exports_physics_10` | 6 | 6 | 5 | 13 | success | 未生成 `validation_report.json` |
| 22 | L2 | Sailboat winch with pawl and handle | `seed_exports_physics_10` | 11 | 5 | 4 | 10 | success | 未生成 `validation_report.json` |
| 23 | L3 | Astronomy Space shuttle | `seed_exports` | 0 | 20 | 19 | 48 | success | N/A |
| 24 | L3 | Laundry clothes drying rack | `seed_exports` | 1341 | 14 | 13 | 13 | success | N/A |
| 25 | L3 | Vehicle Sports car | `seed_exports` | 202 | 10 | 9 | 20 | success | N/A |
| 26 | L3 | Industrial rolling work table | `seed_exports` | 0 | 17 | 16 | 12 | success | N/A |
| 27 | L3 | Hole punch | `seed_exports` | 0 | 12 | 11 | 20 | success | N/A |
| 28 | L3 | Drawing compass with adjustable legs | `seed_exports` | 0 | 11 | 10 | 14 | success | N/A |
| 29 | L3 | Ergonomic clamp with adjustable components | `seed_exports` | 0 | 14 | 13 | 16 | success | N/A |
| 30 | L3 | Hand-crank clothes wringer | `seed_exports` | 0 | 10 | 9 | 27 | success | N/A |
| 31 | L3 | Ergonomic clamp with adjustable | `seed_exports` | 0 | 11 | 10 | 16 | success | N/A |
| 32 | L3 | Astronomy Pressurised module door | `seed_exports` | 1020 | 9 | 8 | 26 | success | N/A |
| 33 | L3 | Folding table 5 | `seed_exports` | 2 | 17 | 16 | 3 | success | N/A |

## 3. 资产绝对路径清单

### L1：≤3 links

1. [`seed_exports/Astronomy_Antenna_dish/seed_343`](/mnt/zsn/lyb/arti-skill/seed_exports/Astronomy_Antenna_dish/seed_343)
2. [`seed_exports/Door_Double_Door/seed_222`](/mnt/zsn/lyb/arti-skill/seed_exports/Door_Double_Door/seed_222)
3. [`seed_exports/Electrical_Wiring_Wire_stripper/seed_5943`](/mnt/zsn/lyb/arti-skill/seed_exports/Electrical_Wiring_Wire_stripper/seed_5943)
4. [`seed_exports_physics_10/Bag_Suitcase_Shopping_bucket/seed_148`](/mnt/zsn/lyb/arti-skill/seed_exports_physics_10/Bag_Suitcase_Shopping_bucket/seed_148)
5. [`seed_exports_physics_10/Door_Other/seed_431`](/mnt/zsn/lyb/arti-skill/seed_exports_physics_10/Door_Other/seed_431)
6. [`seed_exports_physics_10/Door_Trap_door/seed_1495`](/mnt/zsn/lyb/arti-skill/seed_exports_physics_10/Door_Trap_door/seed_1495)
7. [`seed_exports_physics_10/Handtools_clothes_peg/seed_453`](/mnt/zsn/lyb/arti-skill/seed_exports_physics_10/Handtools_clothes_peg/seed_453)
8. [`seed_exports_physics_10/Urban_Environment_bucket1/seed_1009`](/mnt/zsn/lyb/arti-skill/seed_exports_physics_10/Urban_Environment_bucket1/seed_1009)
9. [`seed_exports_physics_10/Urban_Environment_bucket2/seed_6101`](/mnt/zsn/lyb/arti-skill/seed_exports_physics_10/Urban_Environment_bucket2/seed_6101)
10. [`seed_exports_physics_10/pictureX_0611_garlic_press/seed_2`](/mnt/zsn/lyb/arti-skill/seed_exports_physics_10/pictureX_0611_garlic_press/seed_2)
11. [`seed_exports_physics_10/pictureX_0611_guitar_tuning_peg_mechanism/seed_202`](/mnt/zsn/lyb/arti-skill/seed_exports_physics_10/pictureX_0611_guitar_tuning_peg_mechanism/seed_202)

### L2：4–8 links

12. [`seed_exports/Industrial_Mine_cart/seed_13`](/mnt/zsn/lyb/arti-skill/seed_exports/Industrial_Mine_cart/seed_13)
13. [`seed_exports/pictureX_0611_C_shaped_sofa_side_table/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_C_shaped_sofa_side_table/seed_0)
14. [`seed_exports/pictureX_0611_Dressing_table/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_Dressing_table/seed_0)
15. [`seed_exports/pictureX_0611_Butter_maker/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_Butter_maker/seed_0)
16. [`seed_exports/pictureX_0611_Garden_pruner/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_Garden_pruner/seed_0)
17. [`seed_exports/pictureX_0611_Ice_crream_machine/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_Ice_crream_machine/seed_0)
18. [`seed_exports/pictureX_0611_juicer_press_with_handle/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_juicer_press_with_handle/seed_0)
19. [`seed_exports/pictureX_0611_bi_fold_closet_door_system/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_bi_fold_closet_door_system/seed_0)
20. [`seed_exports/riot_shield/seed_3141`](/mnt/zsn/lyb/arti-skill/seed_exports/riot_shield/seed_3141)
21. [`seed_exports_physics_10/Stationary_Pencil_sharpener/seed_6`](/mnt/zsn/lyb/arti-skill/seed_exports_physics_10/Stationary_Pencil_sharpener/seed_6)
22. [`seed_exports_physics_10/sailboat_winch_with_pawl_and_handle/seed_11`](/mnt/zsn/lyb/arti-skill/seed_exports_physics_10/sailboat_winch_with_pawl_and_handle/seed_11)

### L3：>8 links

23. [`seed_exports/Astronomy_Space_shuttle/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/Astronomy_Space_shuttle/seed_0)
24. [`seed_exports/Household_Laundry_Clothes_drying_rack_Laundry_drying_rack/seed_1341`](/mnt/zsn/lyb/arti-skill/seed_exports/Household_Laundry_Clothes_drying_rack_Laundry_drying_rack/seed_1341)
25. [`seed_exports/Vehicle_Sports_car/seed_202`](/mnt/zsn/lyb/arti-skill/seed_exports/Vehicle_Sports_car/seed_202)
26. [`seed_exports/pictureX_0611_Industrial_rolling_work_table/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_Industrial_rolling_work_table/seed_0)
27. [`seed_exports/pictureX_0611_Hole_punch/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_Hole_punch/seed_0)
28. [`seed_exports/pictureX_0611_drawing_compass_with_adjustable_legs/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_drawing_compass_with_adjustable_legs/seed_0)
29. [`seed_exports/pictureX_0611_ergonomic_clamp_with_adjustable_components/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_ergonomic_clamp_with_adjustable_components/seed_0)
30. [`seed_exports/pictureX_0611_Hand_crank_clothes_wringer/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_Hand_crank_clothes_wringer/seed_0)
31. [`seed_exports/pictureX_0611_ergonomic_clamp_with_adjustable/seed_0`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_ergonomic_clamp_with_adjustable/seed_0)
32. [`seed_exports/Astronomy_Pressurised_module_door/seed_1020`](/mnt/zsn/lyb/arti-skill/seed_exports/Astronomy_Pressurised_module_door/seed_1020)
33. [`seed_exports/pictureX_0611_Folding_table5/seed_2`](/mnt/zsn/lyb/arti-skill/seed_exports/pictureX_0611_Folding_table5/seed_2)

## 4. 每个资产的标准文件

主导出目录中的 23 个资产均应使用以下文件作为实验输入：

```text
<asset_dir>/model.py
<asset_dir>/model.urdf
<asset_dir>/assets/
<asset_dir>/compile_report.json
```

`seed_exports_physics_10` 中的 10 个资产至少包含上述编译输入；其中 clothes peg 还包含：

```text
<asset_dir>/model_physics.urdf
<asset_dir>/articulated_physics.json
<asset_dir>/validation_report.json
<asset_dir>/vlm_physics_priors.json
```

## 5. 按实验轴的使用建议

| 实验轴 | 推荐使用方式 |
|---|---|
| Reliability | 使用全部 33 个资产，检查源代码执行、URDF 导出、mesh 导出和 artifact 保存。 |
| Naming | 使用全部 33 个资产，统计部件命名覆盖率、语义 precision/recall 和跨 seed 一致性。 |
| Hierarchy | 使用全部 33 个资产，测量树结构、父子边、语义深度和 pivot。 |
| Constraints | 优先使用 L2/L3 资产，并从最终 URDF/mesh 重新测量数量、尺寸、接口和运动约束。 |
| Editability | 优先使用 Dressing table、bi-fold closet door、juicer、garden pruner、clamp、wringer、space shuttle 等具有明确局部功能部件的资产。 |
| Articulation | 优先使用包含 revolute、prismatic 或 continuous joint 的资产；physics_10 资产可用于关节类型、轴和 joint limit 检查。 |
| Production Readiness | 使用全部 33 个资产，检查 mesh 完整性、URDF 元数据、visual/collision、包结构和源代码紧凑性。 |

## 6. Physics 验证状态说明

`seed_exports_physics_10` 的 10 个资产的 `compile_report.json` 均为 `success`，因此可以进行代码/URDF 层面的实验。但是，当前只有：

- `Handtools_clothes_peg/seed_453` 具有 `validation_report.json`；
- 该报告中 `success=true`、`dataset_ready=true`；
- 其物理验证包含 3 个 links、2 个 joints，并保留了 visual/collision 和惯性信息。

其余 9 个 physics_10 资产的日志显示物理标注阶段未生成验证报告，主要原因是 VLM 调用超时。因此它们可以纳入主实验和 articulation 结构实验，但在报告中应将完整 physics validation 标为“未完成”，不能作为 physics dataset-ready 样本统计。

## 7. 复核结果

截至本清单生成时：

- 资产数量：33；
- `seed_exports`：23；
- `seed_exports_physics_10`：10；
- L1/L2/L3：11/11/11；
- `model.py` 完整率：33/33；
- `model.urdf` 完整率：33/33；
- `assets/` 完整率：33/33；
- `compile_report.json` 成功率：33/33；
- physics `dataset_ready=true`：1/10（仅 clothes peg）。
