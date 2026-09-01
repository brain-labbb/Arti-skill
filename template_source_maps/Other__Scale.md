# Other / Scale — template source map

pattern: mixed（固定 named slots:pedestal_column + pan_hanger + beam_form;主机构 = beam pivot + 2 pan swing（固定 3 关节,不可改 pan_count——天平须 2 盘）;**外加** chain_count 每盘多重性）

parents（1 个母资产,装饰性双盘正义天平）:
- rec_model-a-decorative-two-pan-balance-scale-scales-_20260610_084539_331235_58c45b51 ← picture/Other/Scale/001.png（基线 = column:turned_baluster_square_plinth × pan:three_chain_dished × beam:straight_rect × chain_count:3）

批次：other_scale_qwen37max_20260617（dashscope qwen3.7-max / medium）。7 变体全部 compile=success、workbench-only、≥3 非 fixed joint（beam + 2 pan）。

## 组合数预审

column 3 × pan 3 × beam 2 × chain_count{2,3,4} = 54 ≫ 10。核心机构(beam REVOLUTE + 2 pan REVOLUTE)在全样本保持,结构轴改 form;chain_count 提供每盘复制多重性。

## Slot 候选覆盖

### Slot A:pedestal_column（底座 + 立柱支撑形式）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| turned_baluster_square_plinth（基线） | parent | 方阶基 + 车工立柱 | parent |
| tripod_legs | rec_variant-pedestal-column-tripod-legs-replace-the-_20260617_122134_968459_5bfc3418 | 三脚撑 | converged(3) |
| round_drum_base | rec_variant-pedestal-column-round-drum-base-replace-_20260617_122134_969139_9e45efb9 | 圆鼓基(lathe) | converged(3) |

### Slot B:pan_hanger（秤盘 + 吊挂形式）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| three_chain_dished（基线） | parent | 三链浅碟盘 | parent |
| flat_plate_single_rod | rec_variant-pan-hanger-flat-plate-single-rod-replace_20260617_122134_968589_54123985 | 单杆平盘 | converged(3) |
| deep_bucket_scoop | rec_variant-pan-hanger-deep-bucket-scoop-replace-the_20260617_122134_968934_2f16a2b1 | 深桶斗盘(lathe)| converged(3) |

### Slot C:beam_form（横梁形态）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| straight_rect（基线） | parent | 直矩形梁 | parent |
| ornate_scroll_beam | rec_variant-beam-form-ornate-scroll-beam-replace-the_20260617_122134_970032_c2faa4ca | 涡卷曲梁(swept)| converged(3) |

## Multiplicity / Copy Logic
- count_param: **`chain_count`**（每盘吊链数;基线 3）
  - N 样本: {2, 3, 4}
  - N=2 → rec_variant-chain-count-2-hang-each-pan-from-two-con_20260617_122533_491824_7935158d
  - N=4 → rec_variant-chain-count-4-hang-each-pan-from-four-co_20260617_122823_197341_c5c8338c
  - 模板建议 N_range: [2, 6]；copied object: 单根吊链 rod(`_rod_solid`);naming 循环 index;placement 沿 `ATTACH_ANGLES` 等角;joint policy 随 pan 体动(无独立关节)
  - 核心 pan_count 固定为 2(天平定义,不可作 multiplicity)。

## 格子覆盖

| 槽 | 源池候选数(含基线) | 空格已填 | 模板实际候选数 | 差额来源 |
|---|---|---|---|---|
| A pedestal_column | 3 | tripod / drum(2) | 4 | +twisted_barley_column（③ contracted，无源记录）|
| B pan_hanger | 3 | flat_plate / deep_bucket(2) | 4 | +pierced_openwork_tray（③ contracted，无源记录）|
| C beam_form | 2 | scroll(1) | 4 | +truss_lattice_beam / +tapered_lens_beam（均 ③ contracted，无源记录）|
| multiplicity chain_count | N∈{2,3,4} | N=2 / N=4(2) | N∈{2,...,6} | N=5/6 为等角公式外推，由 packing 不等式作边界证明 |

每槽 ≥2 + multiplicity 3 个 N → 源池本身满足 §8。**Scale 小类样本池就绪。**

**源池 vs 模板的差额（2026-07-28 更新）**：模板 `agent/templates/Other_Scale.py` 的候选数已超过本源池
所能背书的数量。多出的 4 个候选是 ③ `world_knowledge_extrapolation`，契约与 validator 写在
`arti-template/articraft_template_authoring/specs_modular_v1/Other_Scale.md` §Contracted candidates，
status 记为 `contracted` 而非 `converged`。**它们不对应任何 record/revision，不要在本表里当作已 fork 的
空格。** 若后续补 fork 并 converge，把对应行升级为真实 record 引用。

**评分口径更正**：本源池 8 条记录（1 parent + 7 fork）在 `data/records_index.jsonl` 中的
`rating` / `effective_rating` / `secondary_rating` **实测均为 `null`**，并未被打过 5 星；parent 的
`run_status` 为 `draft`，7 个 fork 为 `success`。此前 spec 头部写的 "8 个 5 星样本 … rating=5" 与索引不符。

## 排除项
- pan_count ≠ 2 出"天平"类目,不作轴。
