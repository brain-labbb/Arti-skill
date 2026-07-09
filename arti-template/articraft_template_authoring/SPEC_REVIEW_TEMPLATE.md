# Modular Spec Review Template

用于人工审核 `articraft_template_authoring/specs_modular_v1/<category_slug>.md`。

## Review Target

| 项 | 值 |
|---|---|
| category_slug | <category_slug> |
| spec path | articraft_template_authoring/specs_modular_v1/<category_slug>.md |
| reviewer status | pending / approved / rejected |

## 必查项

- [ ] agent 声明已枚举并完整读取该类别全部 5 星样本的 `model.py / revision.json / record.json`。
- [ ] spec 使用 `SPEC_TEMPLATE.md` 的 modular schema，且 `__modular__ = True`。
- [ ] 核心身份清楚，能区分相邻类别。
- [ ] slot count 合理：每个 slot 都代表真实结构/功能层，且能通过接口点位与相邻 slot 装配。
- [ ] 每个 slot 至少 2 个结构不同的 candidate module；若少于目标 3 个，有基于 5 星样本池的理由。
- [ ] 每个 candidate module 都有真实 5 星样本 `model.py:Lx-Ly` 来源。
- [ ] Candidate module 之间的差异是 part tree、joint topology、chain depth、primitive 或复制逻辑差异，不只是尺寸、颜色或装饰差异。
- [ ] Slot graph 清楚说明 serial / parallel / multiplicity / mixed 装配关系。
- [ ] 接口点位明确：共同 parent、mating face、pivot、rail、socket、axis、contact plane 或 symmetry plane。
- [ ] 跨 module joint 的 type / axis / range / parent-child 语义明确。
- [ ] Multiplicity / Copy Logic 说明 N_range、sampling domain、copied object、naming、placement、joint policy 和 source/gating。
- [ ] 参数表只暴露语义参数、slot/module 选择、必要尺寸和 multiplicity 数量；没有把未实现拓扑塞进 enum。
- [ ] Procedural Sampling / Sweep Plan 说明 deterministic procedural sampling 如何选择 slot/module。
- [ ] Compatibility matrix / gating 清楚说明哪些 module 组合合法、互斥、降级或拒绝。
- [ ] Controlled local parameterization 已写明关键连续 scale、范围、clamp / derived constraints，以及不会破坏接口、clearance、joint origin 或类别 multiplicity。
- [ ] 每个连续 scale 标注了 `约束类型`（independent / equation / inequality / conditional）；标称默认列填基线值而非 `sampled`。
- [ ] 跨部件函数依赖（等式派生 / 联合可行域不等式 / 条件范围）已显式声明，没有把相关尺寸当作互相独立的自由变量；采样顺序符合连续尺寸采样契约。
- [ ] 局部尺寸扰动只是辅助比例/细节多样性；spec 没有用连续尺寸随机替代真实 slot/module/multiplicity 拓扑差异。
- [ ] 可选 regression overrides 有明确失败回归或审核理由；没有把小型 curated / modulo 表作为主 seed domain。
- [ ] Random sweep seed 范围和 viewer 目检重点已写明，并覆盖关键 slot/module、边界配置和高风险组合。
- [ ] Validator 能转成模板内 `run_<slug>_tests` 或 sweep 可检查项。
- [ ] `motion_test_plan` 能转成 `run_<slug>_tests`：非-FIXED joint 默认有 `fail_if_parts_overlap_in_sampled_poses(...)`，关键机制有 targeted `ctx.pose(...)`，需要时声明 `qc_samples` / `qc_sample_values`；若豁免 broad sampled collision，已写 `sampled-pose exemption` 理由和替代 pose 覆盖。
- [ ] Reject cases 覆盖漂浮、穿模、接口错位、joint 方向错误、类别身份丢失、module 组合非法等失败模式。
- [ ] spec 没有使用单一 `primary_anchor` 替代 per-module source table。
- [ ] spec 没有把已读但未采用的样本写入 module source 表。

## 视觉多样性 6 轴必查（对应 SPEC §8.5；权威定义见 VISUAL_DIVERSITY_MODEL.md）

- [ ] §8.5 六轴考察表已填满：每根轴都"有 → 列取值/范围 + 5★来源"或"无 → 写理由"，**无空格**。
- [ ] ① 骨架图：结构形态种类已列（运动学 part-joint 图）；multiplicity 子项引到 §8。
- [ ] ② 关节类型：声明支持的每种 type/轴都会在 sweep 出现。
- [ ] ③ 主体形态家族 / Primary Form Family：形态主导类有 **≥3 个可识别主体形态原型且登记进 `slot_choices`**（不是孤儿轴）；每个 ③ candidate 标明 `form_subtype`（Planar Boundary Form / Volumetric Envelope Form / Macro Surface Construction）；单一原型类已写理由。
- [ ] ④ 表面装饰：style / 装饰数已列；装饰几何由宿主表面逐-z 派生、随 ③⑤ 共形嵌入（非常数半径套外、非平贴曲/斜面）。
- [ ] ⑤ 尺寸/行程：关键比例范围够宽；**每个非-continuous 关节有运动包络 + `motion_test_plan`，覆盖 sampled collision / `qc_samples` / targeted pose 或明确豁免，关节全程不穿模**。
- [ ] ⑥ 涂装：材质大类 + 配色 ≥3-6，覆盖 ≥ ceil(0.5×声明档)。
- [ ] 收尾自检：每个"有"的取值在 batch 0-9 渲染里肉眼出现（形状拉开 / 材质大类出现 / 装饰贴合 / 关节不穿模）。
- [ ] 多样性达标以六轴考察 + 人工 review 为准：未把 N 当结构 distinct，未把 `≥10 distinct` 当多样性目标。

## 审核结论

```text
approved / rejected
```

## 修改意见

```text
...
```

## Template 阶段提醒

审核通过后，agent 必须进入 modular template 实现：读取 `AUTHORING.md`（§A 硬规则 + §B 模块 contract + §C sweep 闭环），改编 source 时参考 `MATURE_TEMPLATE_METHOD.md`；实现 `__modular__ = True`、`slot_choices_for_seed`、procedural `config_from_seed`、module factories、InterfaceSpec、MatingContract 和 `run_<slug>_tests`；运行 `uv run articraft template sweep-pipeline <slug>` 直到 `verdict=pass`，再做 preview / viewer 目检。
