# Nano3D Naming judge packet v1.1

每个任务对应一个 N=32 source-semantic 资产的 GLB mesh node。预览包含完整资产中的红色目标和三个隔离视角。`blind_tasks.jsonl` 不包含自动 role assignment；`audit_tasks.jsonl` 仅用于事后审计，不应交给独立 judge。

三名 judge 分别填写 `judges/judge_a.jsonl`、`judge_b.jsonl`、`judge_c.jsonl`。必须填写 `judge_verdict` 和 `judge_reason`；`spec_match` 还必须填写 `judge_matched_role`。

重复角色（`min_count>1`）的 `judge_instance_id` 填从节点名和预览可区分的实例身份（如 `left/right` 或 `0/1`）；同一实例的碎片使用同一 ID；其他真实 verdict 填 `not_applicable`。真实节点的 `judge_same_semantic_part_as` 必须显式填 `none`，或填同资产中属于同一语义部件的另一节点名；invalid 填 `not_applicable`，uncertain 的附加字段保持 null。空值不代表 `none`。

`independent_gold_annotation_template.jsonl` 是输出盲的 N=33 gold 标注模板。33/33 已冻结输出无关的类别文本；pictureX 类别另复制原始类别参考图，可开始 core-taxonomy 标注。现有 export 未保存逐 seed 的精确原始生成请求，因此 optional/per-seed gold 仍为 0/33 ready。

当前 reference roles 仍来自 source-derived gold，不是独立 hidden gold；完成三 judge 只能补 semantic validation，不能消除 gold 来源限制。
