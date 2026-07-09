# Articraft Modular Template Authoring

本目录把 Articraft-10K 新类别转成**模块化参数化模板**。唯一路线：

```text
完整读取目标类别 5 星样本 -> 写 modular spec -> 直接实现 __modular__ 模板 -> sweep-pipeline + viewer 目检
```

新类别模板不使用单一部件清单或 `primary_anchor`；必须用 slot graph、candidate
module、InterfaceSpec、MatingContract、compatibility matrix、procedural sampling
和 viewer 目检表达结构级变化。

## 文档分层

**必读（实现模板前）**

- `AUTHORING.md` — 唯一 authoring 必读。§A 设计判断与 5 条硬规则；§B 模块系统
  / slot / contract / interface / pattern / pitfall；§C sweep 迭代闭环、gate、verdict。

**表单 / schema（照着填）**

- `SPEC_TEMPLATE.md` — 唯一 spec schema（slot/module/source/slot graph/Multiplicity/
  §8.5 六轴/拓扑多样性审计字段）。
- `SPEC_REVIEW_TEMPLATE.md` — 审核 modular spec 的 checklist（可选自查工具）。
- `CATEGORY_BATCH_TEMPLATE.md` — 批量类别输入模板。
- `SPEC_EXAMPLE_INDEX.md` — spec 示例分层清单；定义哪些文件可作为 schema 来源。

**按需参考**

- `VISUAL_DIVERSITY_MODEL.md` — 视觉多样性 6 轴的权威定义（设计多样性轴时查）。
- `MATURE_TEMPLATE_METHOD.md` — 把 5 星源码参数化到成熟水平的补充（改编 source 时查）。

**specs_* 目录（示例来源边界，详见 `SPEC_EXAMPLE_INDEX.md`）**

- `specs_modular_v1/*.md` — 当前 canonical 示例；新 spec **只能**从这里 + `SPEC_TEMPLATE.md` 学格式。
- `specs_modular_transitional/*.md` — 只参考结构思想，不作为 schema 来源。
- `specs_legacy_reference_only/*.md` — 旧格式 / baseline 只读历史；禁止作为新 spec 格式、
  source contract 或新模板路线来源。

## 工作流（spec → 模板连续执行，无中间停点）

### 1. SPEC 阶段

给 agent N 个类别 → 生成 N 个 spec → **直接进入模板阶段**（`SPEC_REVIEW_TEMPLATE.md`
可用作自查/抽查工具）。

```text
N 个类别 -> N 个 specs_modular_v1/<category_slug>.md -> 模板阶段
```

spec 阶段只允许创建 / 修改 `specs_modular_v1/<category_slug>.md`。工作要点：

- **完整读取 5 星样本**：用 storage API / CLI 枚举目标类别全部 retained 样本，过滤 5 星，
  **全部读取不得抽样**（每个读 `model.py` / `revision.json` / `record.json` / prompt /
  category metadata）。5 星样本 <5 个时停止并报告，等人工确认。
- **识别结构变化轴**：只把 part tree / joint count·type·topology / chain depth / 接口点位
  / 同构复制数量的真实差异当变化轴。只改尺寸·比例·颜色·材质·装饰密度的差异不是独立 slot，
  也不是独立 candidate module。
- **设计 slots**：每个 slot 一个可替换结构 / 功能层，典型 2-4 个（3 最常见）。slot 成立需
  ≥2 个结构不同 candidate（目标 3-6），且能与相邻 slot 通过共同 parent / mating face /
  pivot / rail / socket / axis / contact plane 装配。来源不够就折入相邻 module，别为凑
  slot 发明结构。
- **选 candidate modules**：每个必须来自被采纳的 5 星样本代码片段（真实 `model.py:Lx-Ly`），
  part tree 清楚、活动语义明确、primitive 能体现类别身份、接口兼容。不采纳只换色 / 尺寸 /
  装饰、漂浮穿模、joint 语义错误、无法接入 slot graph 的片段。已读但未采用的样本**不写进**
  module source 表。
- **写 spec**：必须含 `SPEC_TEMPLATE.md` 的**全部强制字段（§1–§12，按其顺序）**——`SPEC_TEMPLATE.md`
  是唯一字段来源。两点最易漏：**§8.5 视觉多样性 6 轴考察（必填）**（逐轴声明"有/无"+理由；
  形态主导类必须登记一根 ③ Primary Form Family slot 并为每个 ③ candidate 标 `form_subtype`；
  权威定义见 `VISUAL_DIVERSITY_MODEL.md`）；**§9 拓扑多样性审计 + Procedural Sampling / Sweep
  Plan + compatibility matrix / gating**。不要用单一 `primary_anchor`。
spec 阶段可批量处理多个类别；写完 spec 即照着 spec 进入模板阶段。

### 2. 模板阶段

读取 spec，回溯每个 candidate 的 5 星源码，
按 `AUTHORING.md`（§A 硬规则 + §B 模块 contract + §C sweep 闭环）实现
`agent/templates/<slug>.py`：`__modular__ = True`、procedural `config_from_seed`（seed=0
不特殊）、`slot_choices_for_seed`、module factories、InterfaceSpec / MatingContract、
`run_<slug>_tests`。若一个 spec 覆盖多个不兼容主运动 spine / root coordinate / slot graph，
优先拆 slug；暂不拆则 `config_from_seed` 只采样已实现且测试覆盖的稳定子域。

实现前从 `MATURE_TEMPLATE_METHOD.md` 的 reference map 选 1-3 个相近模板深读（按 slot graph /
运动拓扑 / 接口类型 / multiplicity 选，不按类别名）。跑 sweep-pipeline 闭环直到 `verdict=pass`，
再按根协议做 preview / viewer 目检。

模板阶段默认一次只实现 1 个模板（同大类高度相似的可 2-3 个一组）；每个先达到
`verdict=pass` 并目检，再进入下一个。
