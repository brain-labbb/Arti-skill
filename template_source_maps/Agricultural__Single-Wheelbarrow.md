# Source Map — Agricultural / Single-Wheelbarrow

slug `wheelbarrow` · pattern **mixed**（`wheel_axle_pivot` 根挂两 child: 车斗 body-tip revolute
+ 单轮 continuous spin；木款侧板为 multiplicity）。双 origin 已把 3 槽的两端锚点占满。

## Origins（全量对账，2/2 上格）
| id | pic | 建成形态 | 网格角色 |
|---|---|---|---|
| A `rec_use-the-attached-reference-image-as-the-primary-_20260625_155130_516298_cabe5db4` | 001 | 现代钢斗:压制钢 `tray_shell` + 弯管 chassis + 单气胎轮；body-tip revolute + wheel continuous | body=steel_pan / wheel=pneumatic / frame=tube_rail |
| B `rec_use-the-attached-reference-image-as-the-primary-_20260625_155130_520600_52ff17ad` | 002 | 木板车:板条箱 `side_*_slat_*` + 木梁 runner frame + 木辐轮 | body=wood_slat_box / wheel=wood_spoked / frame=wood_runner |

## Slots
- **A tub_body（③ 主轴）**：steel_pressed_pan(A) / wood_slat_box(B) / plastic_molded_tub(fork@A) / flatbed_deck(fork@A) — 可外推 wire_mesh_cage(Macro Surface)
- **B wheel_type**：pneumatic_steel_rim(A) / wood_spoked_cart(B) / solid_disc(fork@A)
- **C frame_build（① skeleton）**：tube_rail(A) / wood_runner(B) / welded_flatbar(fork@A)
- **N**：wood side slats ×N {2,3(B),5}；**wheel 辐条数=WheelGeometry `count=` 参数，非 fork 轴(模板连续 knob)**

## Slot 候选覆盖
### Slot A：tub_body
| steel_pressed_pan(origin) | A | converged |
| wood_slat_box(origin) | B | converged |
| plastic_molded_tub | rec_wheelbarrow_var_plastic_tub | converged |
| flatbed_deck | rec_wheelbarrow_var_flatbed | converged |
### Slot B：wheel_type
| pneumatic_steel_rim(origin) | A | converged |
| wood_spoked_cart(origin) | B | converged |
| solid_disc | rec_wheelbarrow_var_solid_wheel | converged |
### Slot C：frame_build
| tube_rail(origin) | A | converged |
| wood_runner(origin) | B | converged |
| welded_flatbar | rec_wheelbarrow_var_flatbar_frame | converged |

## Multiplicity / Copy Logic
- count_param: `side_slat_count` — copied object=木侧板 `side_{side}_slat_{row}`(+`floor_plank_{i}`/`upright_post_{i}` 同族)；placement=等 z；joint policy=全 FIXED 于单 `barrow_body`(随倾倒)
- N 样本已覆盖: {2,3,5} → rec_wheelbarrow_var_slats_n2 / B / rec_wheelbarrow_var_slats_n5；模板 N_range [2,8]

## 视觉多样性 6 轴考察
| 轴 | 处理 | 取值 |
|---|---|---|
| ① 骨架图 | forked_anchor | body 4 + wheel 3 + frame 3；slat N∈{2,3,5} |
| ② 关节类型 | forked_anchor | body-tip revolute 0→1.05；wheel continuous |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | tub_body(pan/slat/poly/flatbed)；可外推 wire_mesh_cage |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | 品牌 emboss、`side_rib`/seam、grip bands、bolts；可外推 decal/flute/rivet |
| ⑤ 尺寸/行程 | record_only | tub 0.86×1.18×0.35-0.55、wheel OD 0.47、handle reach 1.3-1.5；tip 0→1.05 |
| ⑥ 涂装 | record_only | galvanized+green / wood+black / red / blue-orange / poly / yellow / zinc |

## 排除项
- wire_mesh_cage tub — 留模板外推(Macro Surface)，非失败
