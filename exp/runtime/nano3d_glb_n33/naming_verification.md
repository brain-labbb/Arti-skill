# Nano3D Naming direct GLB verification

资产逐字段一致：33/33。
汇总字段一致：PASS。

直接从转换后 GLB 读取全部 link/mesh node names，并使用冻结的 Naming role matcher 与 gold 重算 Parts、Nameability、Recall、Richness、Functional Core Coverage 和 Instance Discriminability。Cross-seed 指标本轮未从 sibling GLB 重算。

```json
{
  "asset_count_direct": 33,
  "asset_count_source_semantic": 32,
  "link_count_total": 241,
  "mesh_bearing_link_count_total": 239,
  "parts_per_asset_mean": 7.242424242424242,
  "nameability_micro": 1.0,
  "paper_aligned_richness_candidate_mean": 1.4817708333333333,
  "paper_aligned_richness_candidate_micro": 1.563758389261745,
  "source_role_recall_macro": 0.99375,
  "source_role_recall_micro": 0.9865771812080537,
  "strong_match_sensitivity_macro": 0.8316964285714286,
  "strong_match_sensitivity_micro": 0.8053691275167785,
  "functional_core_coverage_macro": 0.9930555555555556,
  "functional_core_coverage_micro": 0.9836065573770492,
  "instance_discriminability": 0.95,
  "instance_distinguishable_count": 38,
  "instance_required_count": 40,
  "instance_applicable_groups": 18
}
```
