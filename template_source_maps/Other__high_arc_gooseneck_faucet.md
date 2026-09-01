# Other / high_arc_gooseneck_faucet — template source map

status: **NOT FORKED（fork 跳过,已由独立 qwen37v pipeline 覆盖）**

## 为什么没有 FORK_VARIANTS 批次

- 该小类在 `other_map.json` 中所有 picture 叶（high_arc_gooseneck_faucet/002.png、007.png）的 parent_record_id 均为 `None`——从未为这些 picture 叶建过 canonical `rec_model-…` 母资产,因此无可 fork 的母资产。
- 现存的通用水龙头母资产（`rec_model-a-brushed-gold-single-hole-kitchen-faucet-…`、`rec_model-a-wall-mounted-bathroom-faucet-…`）形态与"高弧鹅颈"小类不匹配,从其 fork 会产出离类（off-category）变体。
- 该 picture 叶已由**独立的 qwen37v copy-pipeline** 覆盖:**60 条 workbench 记录**（`rec_qwen37v_high_arc_gooseneck_faucet_{002,007}_v01..v30`,各图 30 版,`edit_mode=copy`,源自 brushed-gold 母资产,2026-06-16 生成）。这些不是 FORK_VARIANTS 的结构轴变体,而是另一套 copy 方案。

## 决策

用户决定（2026-06-18）:**记为已覆盖,跳过 fork**。不新建记录,不删改既有 qwen37v 记录。

若后续要纳入 FORK_VARIANTS 样本池,需先为"高弧鹅颈"形态建 canonical 母资产（spout_arc / handle_type / mount_holes 等 slot 待定),再 fork。
