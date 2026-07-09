# Urban Environment / Garbage bin — template source map

pattern: parallel_children(固定 named slots:lid_closure 主机构 + body_profile + mobility + lift_interface;ribs/slats 为模板侧可缩放纹理/计数轴。real joint = lid hinge REVOLUTE,wheels = CONTINUOUS roll。所有 lid/slats/feet/casters/fork-pockets/trunnions 均 loop-emitted)

identity: commercial front-load steel waste container（dumpster/bin）— sloped or upright corrugated steel body, hinged steel lid(s) as the primary articulation, base feet or casters, side trunnion pockets + front forklift pockets as the truck lift interface. Variants 仍须读作 commercial dumpster/bin。

## parents
- rec_green-steel-commercial-front-load-dumpster-with-_20260608_164732_054691_3ec067a0 ← picture/Urban Environment/Garbage bin/001.png
  - 绿色钢制前装式垃圾箱:外撇锥形 ribbed 箱体（顶比底宽 front-load taper）+ 卷边顶 rim + 后铰板条 slatted 翻盖（REVOLUTE 横轴@后顶缘,lift front edge up/back）+ 四角固定脚 + 前两个 forklift pockets + 两侧 trunnion pockets。
  - 占格: lid_closure=rear_hinged_slat_lid × body_profile=sloped_front_load_tapered × mobility=fixed_corner_feet × lift_interface=fork_pockets_plus_side_trunnions × N(lid_slats=11)

## 组合数预审 (HARD GATE)
组合数预审: Π(lid_closure 4 × body_profile 2 × mobility 3 × lift_interface 2) × distinct-N(lid_count {1,2} + lid_slats {sparse, parent, many}) = 48 × distinct-N ≥ 10 ✓✓
（仅就 4 个结构槽的候选数 product = 4×2×3×2 = 48 ≥ 10 已独立满足硬门；distinct-N 另由 twin_split_lids(N=2) 与 lid_slat_count(N∈{few, parent, many}) 承担,进一步放大。）

## Slot 候选覆盖

### Slot A:lid_closure（主机构槽——箱口封闭/开启动作；joint 拓扑差异最大，含 lid_count 多重度）
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rear_hinged_slat_lid（基线-parent） | rec_green-steel-commercial-front-load-dumpster-with-_20260608_164732_054691_3ec067a0 | lid / body_to_lid(REVOLUTE -Y@后顶缘) / _lid_mesh(slat loop) | 单片全宽后铰板条翻盖 | converged(parent) |
| twin_split_lids | rec_garbage_bin_var_twin_split_lids | lid_i / body_to_lid_i(REVOLUTE ×2,镜像) / _half_lid_mesh | 中线对开双半盖,各自后铰独立翻起；N=2 多重度 | converged |
| domed_flat_lid | rec_garbage_bin_var_domed_flat_lid | lid / body_to_lid(REVOLUTE) / _domed_lid_mesh(lathe/compound) | 圆拱实心钢罩盖,后铰整体翻起 | converged |
| slot_top_lid | rec_garbage_bin_var_slot_top_lid | drop_flap / body_to_flap(REVOLUTE@槽后缘) / _flap_mesh(slat loop) + fixed top deck inline | 固定投递甲板 + 后铰内摆 spring-flap 投递口 | converged |

### Slot B:body_profile（体形/footprint 家族；连续尺寸由模板缩放,这里只列结构不同形态）
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| sloped_front_load_tapered（基线-parent） | rec_green-steel-commercial-front-load-dumpster-with-_20260608_164732_054691_3ec067a0 | body / _tapered_body + _ribs | 顶比底宽外撇锥形前装箱体 | converged(parent) |
| rectangular_upright | rec_garbage_bin_var_rectangular_body | body / _rect_body(vertical walls) + _ribs | 四壁竖直,口=底 footprint 的直立箱体 | converged |

### Slot C:mobility（落地/移动机构；wheels 为真 CONTINUOUS roll joint）
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| fixed_corner_feet（基线-parent） | rec_green-steel-commercial-front-load-dumpster-with-_20260608_164732_054691_3ec067a0 | _feet_and_pockets(四角脚 loop,inline body visual) | 四角固定钢脚,无轮 | converged(parent) |
| four_caster_mobile | rec_garbage_bin_var_four_caster_mobile | caster_i / wheel_roll_i(CONTINUOUS 横轴 ×4) / _caster_wheel_mesh | 四角 swivel caster + 橡胶轮(各自滚动),替换固定脚 | converged |
| two_caster_tilt | rec_garbage_bin_var_two_caster_tilt | caster_i / wheel_roll_i(CONTINUOUS ×2,前角) / _caster_wheel_mesh + rear fixed feet | 前两角轮 + 后两角固定脚的倾倒推行式 | converged |

### Slot D:lift_interface（卡车举升接口）
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| fork_pockets_plus_side_trunnions（基线-parent） | rec_green-steel-commercial-front-load-dumpster-with-_20260608_164732_054691_3ec067a0 | _feet_and_pockets(forklift pockets loop + side trunnion pockets loop,inline) | 前 forklift pockets + 两侧短 trunnion pockets | converged(parent) |
| continuous_trunnion_lift_bar | rec_garbage_bin_var_trunnion_lift_bar | lift_bar_i + bracket(两长壁 loop,inline body visual) / _lift_bar_mesh | 两侧整条水平举升棒 + 焊接托架,替换短 trunnion pockets | converged |

## Multiplicity / Copy Logic
- count_param 主轴: lid_count（twin_split_lids N=2,parent N=1）+ lid_slats（lid_slat_count 候选:few / parent=11 / many,全部 slat loop + 单 base skin 连成一片盖,绝不浮空板条）。
- 次级 count（body texture,不计样本数,模板侧放大）: RIB_COUNT_SIDE / RIB_COUNT_END（箱壁竖向 corrugation ribs,per-wall loop）。
- N 样本已覆盖: lid_slats {parent 11} by parent; {few, many} by rec_garbage_bin_var_lid_slat_count; lid_count {1 parent, 2 twin}; caster_count {4, 2} by four/two caster 候选; lift_bar/bracket count by trunnion_lift_bar。
- 模板建议 N_range: lid_slats [4, 16]; rib_count_side [6, 12]; caster_count {2, 4}; lid_count {1, 2}。
- copied object / naming / placement / joint policy: 复制对象 = lid slats、half-lids、corner feet、casters+wheels、forklift pockets、side trunnions/lift bars——全部 for-i-in-range loop + name_i + 共享 helper + 规则放置;joint policy 统一(lid=REVOLUTE,wheel=CONTINUOUS,其余 lift/feet hardware 内联为 body visual,无 FIXED 装饰关节)。

### lid_slat_count（多重度专列候选）
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| lid_slat_count | rec_garbage_bin_var_lid_slat_count | lid / body_to_lid(REVOLUTE) / _lid_mesh(slat loop, n=param) + base skin | 板条数参数化(疏/密),单连接 skin 保持一片盖 | converged |

## 格子覆盖
- 1 parent 占 4 槽各一格(A rear_hinged_slat_lid / B sloped_front_load_tapered / C fixed_corner_feet / D fork_pockets_plus_side_trunnions)。
- 8 变体填空格:
  - Slot A 3 空格 → twin_split_lids / domed_flat_lid / slot_top_lid
  - Slot B 1 空格 → rectangular_upright
  - Slot C 2 空格 → four_caster_mobile / two_caster_tilt
  - Slot D 1 空格 → continuous_trunnion_lift_bar
  - Multiplicity 1 专列 → lid_slat_count
- 共 8 新变体(批次规模 = 空格+多重度专列 = 8,在 ~8-10 cap 内)。Garbage bin 小类样本池就绪。

## 排除项(未来 compatibility matrix 素材，本批不立轴)
- color/material（绿/锈/涂鸦/galvanized 等）、纯尺寸缩放(2-yard/4-yard 大小)：非结构轴,§2 不立轴,模板侧作参数/材质。
- 透明/网格篓筐式壁面、家用小翻盖垃圾桶(轮式 pedal-bin)：超出 commercial dumpster 真实形态,易出类目,不立轴。
- side-load / rear-load packer 机构、底部排液阀:真实存在但本批 4 槽已填满;若回补另开 Slot。
