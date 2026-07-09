# Category Batch Input

把需要处理的 Articraft 类别填在这里。agent 先写 modular spec，写完直接照 spec 实现模板（无审核停点）。

## Mode

SPEC_THEN_TEMPLATE

## Categories

| index | category_slug | notes |
|---:|---|---|
| 1 | <category_a> |  |
| 2 | <category_b> |  |
| 3 | <category_c> |  |

## Required Agent Behavior

- 只写 `articraft_template_authoring/specs_modular_v1/<category_slug>.md`。
- 每个新增 spec 必须含 `SPEC_TEMPLATE.md` 的**全部强制字段**（§1–§12：slot/module 表 + 来源、slot graph、Multiplicity、§8.5 六轴考察、拓扑多样性审计、Validator、Reject cases…）。`SPEC_TEMPLATE.md` 是唯一字段来源，细节不在此复述。
- 必须枚举并读取该类别全部 5 星样本；不得抽样、不得只读部分样本。
- 每个 slot candidate 必须有真实 5 星样本 `model.py:Lx-Ly` 来源。
- 已阅读但未采用的 5 星样本，不写入 module source 表。
- 写完 spec 直接照 spec 实现模板（sweep-pipeline verdict=pass 为完成标准）。
- 审核前禁止写 `agent/templates/*.py`、测试、registry。

## Template 阶段原则

审核通过后写模板时，必须走 modular route：读 `AUTHORING.md`（§A 硬规则 + §B 模块 contract + §C sweep 闭环），改编 source 时参考 `MATURE_TEMPLATE_METHOD.md`；实现 `__modular__ = True`、`slot_choices_for_seed`、procedural `config_from_seed`、module factories、InterfaceSpec、MatingContract 和模板内 tests；运行 `uv run articraft template sweep-pipeline <slug>` 直到 `verdict=pass`。通过后做 preview / viewer 目检；发现类别身份、比例、闭合姿态或运动语义问题时继续修复并重新 sweep。
