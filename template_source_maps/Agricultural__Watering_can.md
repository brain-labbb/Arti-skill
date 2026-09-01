# Source Map — Agricultural / Watering can

slug `wateringcan` · pattern **mixed**（`can` hub + parallel_children 提握/盖机构 + rib 带
multiplicity）。**注意**：origin 模型给喷头 `rose_plate` 是图里没有的——`nozzle_open` fork 是
picture-true 修正锚。

## Origins（全量对账，1/1 上格）
| id | pic | 建成形态 | 网格角色 |
|---|---|---|---|
| A `rec_use-the-attached-reference-image-as-the-primary-_20260625_155130_522892_2134a642` | 001 | 镀锌圆筒身 + 长直锥 `spout_tube` + 上摆提梁 `bail_handle`(唯一非固定 `can_to_bail` revolute) + 后 C 握把；模型另加了图中没有的 `rose_plate` | body=cylinder / spout=long_straight / end=rose(模型) / handle=swing_bail |

## Slots
- **A body_form（③ Volumetric Envelope）**：cylinder(A) / oval_drum(fork) / bulbous(fork) / conical(fork)
- **B spout_form（③）**：long_straight(A) / gooseneck(fork) / stubby(fork)
- **C spout_end（①）**：rose_sprinkler(A,模型) / open_nozzle(fork,图真) — 可外推 diffuser
- **D handle/lid（② 机构）**：swing_bail(A) / single_D_handle(fork) / hinged_half_lid(fork,+第二关节)

## Slot 候选覆盖
### Slot A：body_form
| 候选 | record_id | 状态 |
|---|---|---|
| cylinder(origin) | A | converged |
| oval_drum | rec_wateringcan_var_body_ovaldrum | converged |
| bulbous | rec_wateringcan_var_body_bulbous | converged |
| conical | rec_wateringcan_var_body_conical | converged |
### Slot B：spout_form
| long_straight(origin) | A | converged |
| gooseneck | rec_wateringcan_var_spout_gooseneck | converged |
| stubby | rec_wateringcan_var_spout_stubby | converged |
### Slot C：spout_end
| rose_sprinkler(origin) | A | converged |
| open_nozzle(picture-true) | rec_wateringcan_var_nozzle_open | converged |
### Slot D：handle/lid mechanism
| swing_bail(origin, `can_to_bail` revolute) | A | converged |
| single_D_handle(`can_to_dhandle` revolute) | rec_wateringcan_var_handle_dhandle | converged |
| hinged_half_lid(`can_to_lid` revolute, 第二关节) | rec_wateringcan_var_lid_hinged | converged |

## Multiplicity / Copy Logic
- count_param: `rib_count` — copied object=波纹带 `body_seam_{i}`(FIXED 装饰)；placement=z 等距；joint policy=全 FIXED，机构关节仍是 `can_to_bail`
- N 样本已覆盖: {3(A),6} → A / rec_wateringcan_var_ribs_dense；模板 N_range ribs [2,10]、rose holes [12,60]

## 视觉多样性 6 轴考察
| 轴 | 处理 | 取值 |
|---|---|---|
| ① 骨架图 | forked_anchor | spout_end 开口 vs rose；body/spout/handle 分层 |
| ② 关节类型 | forked_anchor | bail revolute ±1.15；D-handle revolute；hinged lid revolute(第二关节) |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | body 4 envelope + spout 3；可外推 can-with-shoulder |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | rib 带、`top_rim`/`rolled_foot`、rivet、vertical seam；可外推滚花/嵌铭牌/锤纹 |
| ⑤ 尺寸/行程 | record_only | body H:D [0.7,1.6]、spout:body [0.8,1.6]；bail ±0.9~1.4、lid [0,1.6] |
| ⑥ 涂装 | record_only | galvanized / rusted / enamel green|red|cream / copper / brass / plastic |

## 排除项
- upturned diffuser 喷头 — 真实但留模板外推(niche)，非失败
