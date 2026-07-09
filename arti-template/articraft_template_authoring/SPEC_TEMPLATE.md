# Modular Spec Template

`articraft_template_authoring/specs_modular_v1/<category_slug>.md` 的唯一字段规范。SPEC_ONLY 阶段必须按本格式产出 spec。

Modular spec 不使用单一 `primary_anchor` 作为主来源，也不要求 `seed=0` 复现固定 anchor 组合。它使用 per-module source table、slot graph、InterfaceSpec / MatingContract 计划和 procedural sampling contract 来描述模板结构。

## 强制字段

### 1. 元信息

```markdown
## 元信息
| 项 | 值 |
|---|---|
| slug | `<category_slug>` |
| template path | `agent/templates/<slug>.py` |
| test path (optional) | `tests/agent/test_<slug>_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `linear_chain` / `parallel_children` / `multiplicity` / `mixed` |
```

`pattern` 说明主要 slot 组装方式：

- `linear_chain`：slot 串成链。
- `parallel_children`：多个 slot 的 part 挂到共同 chassis / parent。
- `multiplicity`：某 slot 负责同构子件 N 次复制。
- `mixed`：混合多种。

### 2. 5 星样本阅读摘要

```markdown
## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | N |
| read_count | N |
| read_scope | all 5-star samples in this category |
| source_index_policy | only adopted module sources are indexed below |
```

### 3. 核心身份

```markdown
## 核心身份

<类别物理含义、主要功能、默认成熟域。说明不该混入的相邻类别。>
```

### 4. 槽位 + 候选模块表

每个 slot 表示一个可替换结构/功能层。每个 candidate module 必须结构不同，并有 5 星样本来源。

```markdown
## 槽位 + 候选模块表

### Slot A：<slot_name_a>

| module_name | source_type | source evidence | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|---|
| <module_alpha> | forked_anchor | rec_<slug>_xxx | L25-L120 | eligible if compatible | part tree / joint / primitive / interface 特征 |
| <module_beta> | forked_anchor | rec_<slug>_yyy | L40-L155 | eligible if compatible | ... |
| <primary_form_extra> | world_knowledge_extrapolation(仅③/④) | anchors: rec_xxx, rec_yyy + reviewer | n/a 或生成函数位置 | eligible if compatible | 同 part tree/interface;只变 Primary Form Family 或 host-conformal decoration |

### Slot B：<slot_name_b>

...
```

硬约束：

- 每个 slot 目标 3-6 个 candidate；样本池不足时可降到 2，但必须说明理由。
- 禁止只有 1 个 candidate 的 slot，除非该 slot 折入相邻 module 或改成 module-local fixed structure。
- 每个普通 ①/②/multiplicity candidate 必须有 `source_type=forked_anchor` 与 `model.py:Lx-Ly` 来源；世界知识可辅助命名和归纳,不得直接新增未被原始资产或 fork anchor 支撑的 skeleton/joint candidate。
- **③ 主体形态家族 / Primary Form Family 例外:** primary-form candidate 可由世界知识直接 author（无单独 `forked_anchor`）,但必须标注 `source_type=world_knowledge_extrapolation`,并写明 `form_subtype ∈ {Planar Boundary Form, Volumetric Envelope Form, Macro Surface Construction}`。前提——(a) 该 ③ 族已有足够 source-backed anchors 证明在本 SDK 可造并覆盖 observed primary-form space 的主要边界(常见 2–5,复杂类更多), (b) 该 candidate 保持**同一 part tree、同一 primitive 家族、同一 interface**、只改变平面边界/体量包络/宏观表面构成的离散形态参数（符合 `AUTHORING.md` §A Rule 3）, (c) 过 sweep（Rule 4 共形 / Rule 5 swept / baseline）+ reviewer 背书类别忠实。
- **④ 表面装饰例外:** decoration candidate 可由真实样本记录 + 世界知识扩展,但必须标注 `source_type=world_knowledge_extrapolation` 或 `record_only`,并且只允许 host-conformal、非结构、非关节、非新功能模块的表面几何/贴附细节（ribs / panel seams / rivets / labels / bands 等）。装饰必须写成宿主 part visual 或由宿主最终表面派生的几何,不得伪装成新 module 或新 joint。
- `sampling eligibility` 说明 candidate 是否进入 deterministic procedural sampler；默认是 `eligible if compatible`。若暂不采样，必须说明阻塞原因和 reviewer 状态。
- Candidate 之间必须有结构差异；只换尺寸、颜色、材质或装饰不是新 candidate。
- 设计 slot 前先过 §8.5 六轴考察：**形态主导类必须有一根登记进 `slot_choices` 的 ③ 主体形态家族 / Primary Form Family slot（≥3 个可识别主体形态原型，或样本不足时 ≥2 并说明理由）**，不能只靠尺寸/涂装撑多样性。换不同 planar boundary / volumetric envelope / macro surface construction 做 candidate 是合法的结构差异（不属于"只换尺寸/装饰"）。

### 5. 槽位图（slot graph）

```markdown
## 槽位图（slot graph）

pattern: <linear_chain / parallel_children / multiplicity / mixed>

<slot A> --[joint_type axis + interface]--> <slot B> --[...]--> <slot C>
```

必须说明：

- slot 顺序或 parent 关系。
- 每条跨 slot 连接的接口点位：mating face、pivot、rail、socket、axis、contact plane 或 symmetry plane。
- 跨 slot joint type、axis、range 或 fixed support policy。
- 哪些 slot 是互斥、可选或由上游 module 派生。

### 6. 每槽位 Module Emits / Interfaces

```markdown
## 每槽位 Module Emits / Interfaces

### Slot A / module <name>
| emits | 描述 | 来源 |
|---|---|---|
| parts | <part names / visual groups> | S1 / model.py:Lx-Ly |
| internal joints | <joint names, type, axis, range> | S1 / model.py:Lx-Ly |
| upstream interface | <face / anchor / parent policy> | S1 / model.py:Lx-Ly |
| downstream interface | <face / anchor / consumer joint> | S1 / model.py:Lx-Ly |
```

要求：

- 活动件必须有 articulation 语义。
- 不动细节写成 parent visual，不作为独立 part。
- Interface 必须能映射到后续 `InterfaceSpec` 和 `MatingContract`。

### 7. 参数范围汇总

```markdown
## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| <slot_choice> | enum | <module names> | — | choice | 由 deterministic procedural sampler 或显式 regression override 选择 | module table |
| <independent_scale> | float | [min, max] | 1.0 | independent | 无；在 `[min,max]` 内独立采样后 clamp | Sx / model.py:Lx-Ly |
| <derived_scale> | float | derived | 1.0 | equation | `= f(<master_param>)`，不独立采样 | Sx / model.py:Lx-Ly |
| (—) | constraint | — | — | inequality | `<联合可行域不等式>`；违反时按比例回缩或拒绝重采 | 接口 / clearance |
```

参数只表达语义选择、尺寸、行程、角度、multiplicity 数量、palette 或 module-local variant。不要把未实现拓扑放进 enum。

**列语义**：

- `标称默认`：采样关闭 / 回归 baseline 时使用的标称值（连续 scale 一般为 `1.0`）。它**不是**"被采样覆盖" 的占位符；不要再写 `sampled`——是否采样由 `约束类型` 表达，default 列只填标称基线。
- `约束类型` 区分连续尺寸之间的关系，避免把所有 scale 当独立自由变量各抽各的：
  - `independent`：在 `[min,max]` 内独立采样，仅 clamp。
  - `equation`：主从派生 `B = f(A)`，B 不独立采样（取值范围列写 `derived`）。包括比例锁定 `B = k·A`（保形）。
  - `inequality`：跨部件 / 跨 module 的联合可行域约束（如自碰撞 `link2 ≤ reach − link1`、链总长 `Σ link_i·scale_i ≤ envelope`、captured-pin 过盈/间隙带），用单独一行声明，`约束 / 函数` 列写不等式及违反时的回缩 / 拒绝策略。
  - `conditional`：合法范围依赖上游 enum / N（如某 scale 上限随所选 module 或 multiplicity 变化）。
- 所有 `equation` / `inequality` / `conditional` 约束都在 `resolve_config` 内求解，禁止留到 builder 才失败。

**连续尺寸采样契约**（写进模板 `config_from_seed` / `resolve_config`，spec 须保证可表达）：

1. 先采所有 `independent` 主尺度（默认在范围内均匀采样；如需中心偏向分布请显式注明）。
2. 按 `equation` 派生所有从属尺度。
3. 用 `inequality` 把组合投影 / 按比例回缩到可行域，无法满足则拒绝并重采。
4. `conditional` 范围在采样前按上游 choice 解析。

scale 默认相互独立；任何相关性必须显式落到 `equation` / `inequality` 行，不能靠隐含约定。

### 7.5 编译预算 / compile budget（必填）

自报本类别的每-seed 编译预算 + 一句依据（库内实测参考：典型模板 5-20s；重布尔
雕刻/复杂放样类 30-60s），模板第一版就按它写。分档 tessellation：小半径特征 ≤32 段，
主体英雄面 ≤64-96 段；N 个相同子件复用同一个 `Mesh`。超出自报预算先降精度再迭代（`AUTHORING.md` §C）。

### 8. Multiplicity / Copy Logic

每个 spec 都必须写本节。若没有模板级复制数量逻辑，也要明确说明没有。

**multiplicity 可以有 0 / 1 / K 根独立轴**（例：直升机 = 主旋翼叶数 + 尾桨叶数）。
**每根轴**都按下面的字段单独声明；`N_range` 与权重档**按小类、按轴定**（栅栏 panel
`[2,100]`、桨叶可能 `[2,8]`），**以人工审核后的取值为准**。下游模板对每根轴各做一次
加权采样（小 N 偏多、尾部稀有）、各自编进 `slot_choices`、各自 clamp、sweep 各自设上限。
（注：跨轴共享的采样 helper 待第二个 multiplicity 模板出现时再抽，不提前抽象。）

有复制数量逻辑时，**每根轴**写：

- `count_param` / `N_range`（本小类本轴产品域；测试偏小、产品全程）/ sampling domain（权重档：小 N 高频、大 N 稀有）
- copied object / naming / placement / joint policy / source/gating

无复制数量逻辑时写：

```markdown
## Multiplicity / Copy Logic

- 无复制数量逻辑：核心结构由固定 named slots 表达，不暴露 `*_count`，也不通过循环复制模板级 visual/part/joint。
```

### 8.5 视觉多样性 6 轴考察-声明（必填，多样性主契约）

> 多样性的唯一权威定义见 `VISUAL_DIVERSITY_MODEL.md`。本节把 §8 multiplicity 的"必须写、无也要声明"
> 纪律推广到全部 6 根轴：**每根轴都必须考察，要么"有 → 列取值/范围 + source_type / 来源"，要么"无 → 写理由"，
> 不准留空。** 结论可以是"无"（如微波炉声明"形状内在单一"即合格），但必须考察过、写明理由，空格即不合格。
>
> 一个 module 可同时命中多根轴（螺帽 = ①+②+③）→ 各记一笔。multiplicity（①的子项）在 §8 详写，
> 本表只勾"有/无"并引到 §8。

```markdown
## 视觉多样性 6 轴考察

| 轴 | 怎么判断（落到唯一主字段） | 有/无 | 若【有】列取值/范围 + source_type / 来源 · 若【无】写理由 |
|---|---|---|---|
| ① 骨架图 | 加/减会动的 part 或一条边 | | 结构形态有哪几种（part-joint 运动学图，非数学拓扑/亏格；洞归 ③）；必须 forked_anchor/source-backed |
| └ multiplicity | 同构件 ×N | | 见 §8（有 → N 域 + 权重档；无 → 声明无） |
| ② 关节类型 | 图不变，某条边换 type/轴 | | revolute / prismatic / continuous / fixed + 轴；必须 forked_anchor/source-backed；声明的每种类型都要在 sweep 里出现 |
| ③ 主体形态家族 / Primary Form Family | 图&关节不变，换核心 part 的可识别几何形态原型（非缩放/换色） | | 有 → source-backed anchors + 可标注 `world_knowledge_extrapolation` 的额外 candidate,且登记进 `slot_choices`；每个 candidate 标 `form_subtype`：Planar Boundary Form（核心面/截面/开口/投影轮廓）、Volumetric Envelope Form（三维包络/厚薄分布/扫掠/旋转/loft 母线）、Macro Surface Construction（改变主体读法的大尺度表面构成）；无 → 单一主体形态 + 理由 |
| ④ 表面装饰 | 原型不变，叠加表面细节 / 改装饰数 | | 标签/印刷/条纹/肋/滚花/饰带/铆钉… + 装饰数量档；可用 `record_only` + `world_knowledge_extrapolation`；装饰几何须**由宿主表面逐-z 派生、随 ③⑤ 共形嵌入**（派生顺序 ③→⑤→④；反例 Container_Tube 常数半径 `label_band` 套在收锥体外） |
| ⑤ 尺寸/行程 | 离散全不变，只连续改尺寸/比例/行程 | | 关键比例 [min,max]（见 §7）+ 关节行程范围；**每个非-continuous 关节写一条运动包络（轴 / 开启方向 / [闭合, 可行上界]）和 `motion_test_plan`：是否跑 sampled collision、是否需要 `qc_samples` / `qc_sample_values`、targeted `ctx.pose(...)` 覆盖哪些 open/closed/extended/folded 状态；关节全程（continuous 则整圈）不得穿模，若 broad sampled collision 不适合必须写 sampled-pose exemption 理由** |
| ⑥ 涂装 | 几何全不变，只改材质/颜色 | | 材质大类（glass/plastic/metal/ceramic/painted）+ 配色 ≥3-6；材质大类覆盖 ≥ ceil(0.5×声明档) |
```

**收尾自检**：本表每个"有"里列的取值，必须在 `template batch` 的 0-9 seed 渲染里**肉眼可见地出现**——
主体形态家族拉得开、材质大类都出现、装饰贴合宿主面不悬空、关节开合全程不穿模。做不到 = 本节未达标。


### 9. 采样与覆盖审计

> 本节记录 deterministic 采样如何选 slot/module、compatibility matrix / gating、sweep seed 范围和 viewer 目检范围。
> **"哪些轴变"的多样性声明在 §8.5**；本节只管"怎么合法采样 + 验收"。

```markdown
## 采样与覆盖审计

总组合数：A × B × C = X
（如有 multiplicity，把 N 的采样数量算进去）

理由：...

seed_domain_policy：procedural_first
Procedural Sampling / Sweep Plan：说明 deterministic procedural sampling 如何选择 slot/module/multiplicity，compatibility matrix / gating 如何避免非法组合，是否存在少量 regression overrides，以及 random sweep / viewer 目检范围。
Topology target：1000-seed slot choice tuple 覆盖用于成熟度观察；富类别建议 >=300，低于 300 需说明真实组合空间、兼容约束或源锚点上限。该指标 report-only，不作为 gate，也不反推上游变体数量。
若使用 regression overrides：说明具体 seed、失败回归或审核理由；不得用小型 curated / modulo 表作为主 seed domain。
Controlled local parameterization：列出初版模板应包含的关键连续 scale，例如 support_width_scale、station_spacing_scale、arm_reach_scale、hub_radius_scale、branch_thickness_scale、terminal_size_scale；说明取值范围、clamp / derived constraints，以及它们不会破坏 InterfaceSpec / MatingContract / multiplicity。按第 7 节的 `约束类型`（independent / equation / inequality / conditional）声明 scale 之间的函数依赖，并遵循连续尺寸采样契约（先采 independent → 派生 equation → 投影/回缩 inequality → 解析 conditional 范围）；跨部件依赖必须显式声明，不得当作互相独立的自由变量各抽各的。

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | <slot order, weighted choices, compatibility gates> | slot_choices_for_seed matches build choices |
| compatibility matrix | <legal / mutually exclusive / fallback policies> | no floating, collision, axis, max multiplicity, bulky module, optional child failures |
| controlled local variation | <safe continuous scale params + clamp policy> | proportions vary without breaking interfaces, clearance, support, joint origin, or category identity |
| regression overrides | none / <seed + reason> | previously failed or reviewer-selected cases only |
| random sweep | e.g. seeds 0-35 for initial pass, 0-999 for maturity audit | contract failures; axis_realization; viewer focus |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A | 4 | yes | yes | |
```

要求：

- 新模板首次实现时就应以 deterministic procedural sampling 作为主 seed domain；`seed=0` 不特殊。
- 新模板首次实现时应包含少量关键局部 scale，但主多样性仍必须来自**离散** slot/module/layout/multiplicity，不能只靠连续 scale 或涂装撑。**category-relative**：形态主导类里，承载主多样性的那个 slot/module **就是 ③ 主体形态家族 / Primary Form Family slot**——换 Planar Boundary / Volumetric Envelope / Macro Surface Construction 原型即合法 module（见 §8.5 + `AUTHORING.md` §B）。不要把每个小零件都做自由随机；所有连续参数必须在 `resolve_config` 中 clamp / 派生，并受接口、clearance、joint range 和类别 identity 约束。
- Compatibility matrix / gating 必须优先排除容易坏的组合：悬空/漂浮风险、穿模/clearance 风险、joint 轴或 range 风险、closed pose 风险、max multiplicity、bulky module、可选 moving child、长链/多子件装配、互斥 gate 或 fallback 降级路径。
- Regression overrides 只能用于已知失败回归或审核指定样本；主体 seed domain 不得无限轮换小型 fixed / curated / modulo 表。

### 10. Validator 和 Reject cases

```markdown
## Validator

- slot_choices_for_seed returns implemented module names
- config_from_seed uses deterministic procedural sampling for all ordinary seeds
- compatibility matrix / gating prevents illegal module combinations
- optional regression overrides are sparse and justified
- final templates do not endlessly cycle a small curated table as the main seed domain
- controlled local scale params are clamped and cannot break interfaces, clearance, joint origin, or category multiplicity
- cross-part scale dependencies (equation / inequality / conditional) are resolved in `resolve_config`, not left to fail in the builder
- critical InterfaceSpec / MatingContract points exist
- key joints have expected type / axis / range
- copied objects follow naming and placement policy

## Reject cases

- <会让模板失败的 5-8 条模式>
```

### 11. 与相邻类别的边界

```markdown
## 与相邻类别的边界

- 不该混入：<相邻类别 1>（理由）
- 不该混入：<相邻类别 2>（理由）
```

### 12. 审核记录

```markdown
## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending / approved / rejected |
| reviewer notes | ... |
```

## 可选字段

### 13. 模板实现备注

```markdown
## 模板实现备注（可选）

- 哪些 module 共享 helper
- 哪些 InterfaceSpec / MatingContract 要特别注意
- 哪些 captured-pin overlap 需要 element-scoped allow_overlap
- 哪些 module 组合暂不进入 seed domain
```

### 14. Module Source Index

如果 slot table 太长，可以在这里汇总 source id。

```markdown
## Module Source Index

| source_id | slot | module | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|---|---|
| S1 | Slot A | module_alpha | rec_xxx | L25-L120 | part tree + upstream interface |
```

## 写完后

spec 的 `stage` 保持 `SPEC_ONLY_DRAFT`，`reviewer status` 保持 `pending`（历史字段，保留兼容）。写完 spec 后**直接进入模板实现**。
