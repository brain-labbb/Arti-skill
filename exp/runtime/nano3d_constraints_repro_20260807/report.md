# Nano3D source-derived Constraints v1

本结果评测 33 个既有模板 × seeds 0–35 的 1,188 个最终 URDF package。它是本地 source-derived/operational constraint audit，不是论文 52 条 hidden-spec GLB constraint 的复现。

| Metric | Result |
|---|---:|
| Coverage | 17706/17706 = 1.000 |
| Satisfaction | 17125/17706 = 0.967 |
| Conditional Accuracy | 17125/17706 = 0.967 |
| Count Pass | 3345/3620 = 0.924 |
| Numeric Pass | 6402/6402 = 1.000 |
| Relational Pass | 3814/4120 = 0.926 |
| Interface Pass | 1188/1188 = 1.000 |
| Kinematic Pass | 1188/1188 = 1.000 |
| Compatibility Pass (valid configs) | 1188/1188 = 1.000 |
| All-Pass Seed Assets | 960/1188 = 0.808 |
| 36/36 All-Pass Templates | 25/33 |
| Invalid Combination Rejection | N/A（无冻结 negative manifest） |

限制：count 是 required-role lower-bound；numeric 是 URDF joint 数值有效性；interface/compatibility 是 operational proxy；详见 `constraints_protocol_v1.json`。
