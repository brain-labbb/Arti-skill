# Nano3D Table 2 Naming 评测报告（paper-aligned v2）

流程按论文拆成 Parts → Named → Richness 三个直接/候选 gate，再单独处理 semantic judges。源文档角色只用于 count-aware recall；未命中 core-role 的命名节点进入 extra-real-part judge queue，不再自动算作 precision 假阳性。

## 汇总

| Metric | Result | Status |
|---|---:|---|
| Parts（mesh-bearing URDF links/asset） | 7.242 [5.667, 8.970] | direct GLB-node proxy; N=33 |
| Named / Nameability | 1.000 | direct; mesh-bearing links |
| Paper-aligned Naming Richness | 1.482 [1.279, 1.709] | candidate proxy; named mesh links/spec instances; N=32 |
| Naming Richness micro | 1.564 | supplementary; pooled 233/149 |
| Source-role Recall | 0.994 [0.981, 1.000] | source-derived, count-aware macro; micro=0.987 |
| Strong-match sensitivity | 0.805 | conservative; 120/149 without single-token aliases |
| Functional Core Coverage | 0.993 | source-derived macro; micro=0.984 |
| Instance Discriminability | 0.950 | 38/40 instances across 18 groups |
| Semantic Precision | N/A | pending three independent judges |
| Semantic Judge Recall | N/A | pending three independent judges |
| Cross-Seed raw name Jaccard | pair-micro 0.580; cohort-macro 0.542 | supplementary; 23 cohorts |
| Cross-Seed role-count Jaccard | pair-micro 0.948; cohort-macro 0.952 | source-derived supplementary proxy |
| Over-Segmentation Rate | N/A | missing part-to-role decomposition gold; not a paper Naming metric |

Semantic source subset excludes `Stationary_Pencil_sharpener__seed_6` because its v1 role list was output-derived fallback rather than copied independent source evidence.
The judge queue contains 147 assigned required-role links and 86 additional named-part candidates. Additional candidates are not false positives until judges decide that they are invalid or hallucinated.

## 资产级明细

| Asset | Mesh Parts | Named | Richness candidate* | Source Recall* | Strong sensitivity* | Functional Coverage* | Instance* | Extra judge candidates | Eligible |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Astronomy_Antenna_dish__seed_343 | 3 | 3 | 1.000 | 1.000 | 1.000 | 1.000 | N/A | 0 | yes |
| Door_Double_Door__seed_222 | 3 | 3 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 0 | yes |
| Electrical_Wiring_Wire_stripper__seed_5943 | 3 | 3 | 1.000 | 1.000 | 1.000 | 1.000 | N/A | 0 | yes |
| Bag_Suitcase_Shopping_bucket__seed_148 | 2 | 2 | 1.000 | 1.000 | 1.000 | 1.000 | N/A | 0 | yes |
| Door_Other__seed_431 | 3 | 3 | 1.000 | 1.000 | 1.000 | 1.000 | N/A | 0 | yes |
| Door_Trap_door__seed_1495 | 3 | 3 | 1.000 | 1.000 | 1.000 | 1.000 | N/A | 0 | yes |
| Handtools_clothes_peg__seed_453 | 3 | 3 | 1.000 | 1.000 | 1.000 | 1.000 | N/A | 0 | yes |
| Urban_Environment_bucket1__seed_1009 | 2 | 2 | 1.000 | 1.000 | 1.000 | 1.000 | N/A | 0 | yes |
| Urban_Environment_bucket2__seed_6101 | 3 | 3 | 1.000 | 1.000 | 0.667 | 1.000 | N/A | 0 | yes |
| pictureX_0611_garlic_press__seed_2 | 3 | 3 | 1.000 | 1.000 | 0.667 | 1.000 | N/A | 0 | yes |
| pictureX_0611_guitar_tuning_peg_mechanism__seed_202 | 3 | 3 | 1.000 | 1.000 | 1.000 | 1.000 | N/A | 0 | yes |
| Industrial_Mine_cart__seed_13 | 7 | 7 | 1.167 | 1.000 | 1.000 | 1.000 | 1.000 | 1 | yes |
| pictureX_0611_C_shaped_sofa_side_table__seed_0 | 4 | 4 | 1.333 | 1.000 | 0.667 | 1.000 | N/A | 1 | yes |
| pictureX_0611_Dressing_table__seed_0 | 6 | 6 | 2.000 | 1.000 | 1.000 | 1.000 | N/A | 3 | yes |
| pictureX_0611_Butter_maker__seed_0 | 8 | 8 | 1.600 | 1.000 | 0.600 | 1.000 | N/A | 3 | yes |
| pictureX_0611_Garden_pruner__seed_0 | 6 | 6 | 1.200 | 1.000 | 1.000 | 1.000 | N/A | 1 | yes |
| pictureX_0611_Ice_crream_machine__seed_0 | 5 | 5 | 1.000 | 1.000 | 1.000 | 1.000 | N/A | 0 | yes |
| pictureX_0611_juicer_press_with_handle__seed_0 | 7 | 7 | 1.000 | 1.000 | 0.143 | 1.000 | N/A | 0 | yes |
| pictureX_0611_bi_fold_closet_door_system__seed_0 | 7 | 7 | 1.400 | 1.000 | 1.000 | 1.000 | 1.000 | 2 | yes |
| riot_shield__seed_3141 | 4 | 4 | 1.333 | 1.000 | 0.667 | 1.000 | N/A | 1 | yes |
| Stationary_Pencil_sharpener__seed_6 | 6 | 6 | 1.200 | 1.000 | 1.000 | 1.000 | N/A | 1 | no |
| sailboat_winch_with_pawl_and_handle__seed_11 | 5 | 5 | 1.000 | 1.000 | 0.000 | 1.000 | N/A | 0 | yes |
| Astronomy_Space_shuttle__seed_0 | 20 | 20 | 2.000 | 1.000 | 0.800 | 1.000 | 1.000 | 10 | yes |
| Household_Laundry_Clothes_drying_rack_Laundry_drying_rack__seed_1341 | 14 | 14 | 2.800 | 1.000 | 0.800 | 1.000 | 1.000 | 9 | yes |
| Vehicle_Sports_car__seed_202 | 8 | 8 | 0.800 | 0.800 | 0.800 | 0.778 | 0.750 | 0 | yes |
| pictureX_0611_Industrial_rolling_work_table__seed_0 | 17 | 17 | 2.429 | 1.000 | 1.000 | 1.000 | 1.000 | 10 | yes |
| pictureX_0611_Hole_punch__seed_0 | 12 | 12 | 2.400 | 1.000 | 0.800 | 1.000 | N/A | 7 | yes |
| pictureX_0611_drawing_compass_with_adjustable_legs__seed_0 | 11 | 11 | 1.833 | 1.000 | 1.000 | 1.000 | 1.000 | 5 | yes |
| pictureX_0611_ergonomic_clamp_with_adjustable_components__seed_0 | 14 | 14 | 2.800 | 1.000 | 0.600 | 1.000 | N/A | 9 | yes |
| pictureX_0611_Hand_crank_clothes_wringer__seed_0 | 10 | 10 | 1.667 | 1.000 | 1.000 | 1.000 | 1.000 | 4 | yes |
| pictureX_0611_ergonomic_clamp_with_adjustable__seed_0 | 11 | 11 | 1.571 | 1.000 | 0.571 | 1.000 | N/A | 4 | yes |
| Astronomy_Pressurised_module_door__seed_1020 | 9 | 9 | 2.250 | 1.000 | 1.000 | 1.000 | N/A | 5 | yes |
| pictureX_0611_Folding_table5__seed_2 | 17 | 17 | 2.833 | 1.000 | 0.833 | 1.000 | 1.000 | 11 | yes |

`*` 表示 source-derived/candidate proxy，不是论文三 judge 的独立 semantic 结果。

## 关键口径修正

- 原 `matched core-role links/asset = 6.818` 不再作为 Part Exists；论文的 Parts gate 统计产出的部件节点，因此本地改为 mesh-bearing URDF link/asset。
- 原 `matched core-role links / all links = 0.934` 不再称为 Semantic Precision；core-role list 不覆盖额外真实部件，未命中不等于 hallucination。
- Recall 展开 `min_count` 后做一对一最大匹配，不再只按 role 是否出现计分。
- Richness 使用论文方向的 `named parts / spec part instances`，但在 judge 完成前明确标为 candidate proxy。
- Instance 指标按 required instance 加权，不再使用每个 role group 等权的 15/18 布尔通过率。
- 另报 strong-match sensitivity：只接受 canonical role token 或至少两 token 的已冻结 pattern，用于显示单 token aliases 对 1.000 Recall 的影响。
- 跨 seed 同时报 pair-micro 与 cohort-macro，避免 seed 较多的 cohort 在唯一汇总值中占更高权重。
- 10,000 次 bootstrap 以 asset 为重采样单元，随机种子冻结在 protocol 文件中。

## 仍不能证明

- 未完成三独立 judge，因此不能报告论文同口径 Semantic Precision/Recall 或 judge-validated Richness。
- 没有 point/mesh-level semantic masks，因此不是 segmentation IoU，也不能测严格 Over-Segmentation。
- URDF link 与论文 GLB mesh node 仅为表示层代理，不应直接横向排名。
