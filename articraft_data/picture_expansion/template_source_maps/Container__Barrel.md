# Container / Barrel — template source map

pattern: parallel_children(固定 named slots:closure 主机构 + body_form + grip;ribs/flutes 为模板侧可缩放 body texture 轴,fork 侧不再铺低质量 handle 关节)

parents:
- rec_blue-plastic-open-top-storage-drum-with-a-lift-o_20260606_074440_612125_4e3a36bf ← picture/Container/Barrel/001.png（蓝色开口储料桶:近直筒 ribbed 桶身 + 直提式 lift-off 盖 PRISMATIC + 前缘 swinging buckle clasp（FIXED clasp_base + REVOLUTE clasp_ring）;占格 closure=lift_off_lid × body=tall_cylindrical_ribbed × grip=swing_buckle_clasp_front）
- rec_blue-ribbed-plastic-barrel-keg-with-a-round-scre_20260606_074449_217026_b410e9f1 ← picture/Container/Barrel/002.png（蓝色多棱桶 keg:鼓腹 lathe 桶身 + 多道 rib 环（loop）+ 圆形螺旋盖（CONTINUOUS lid_rotate + PRISMATIC lid_slide，经 massless carrier 解耦）+ 前面平贴 badge/label 板（无 grip）;占格 closure=screw_cap × body=bulged_belly_ribbed × grip=no_grip_flat_badge）

## 组合数预审

组合数预审: Π(closure 3 × body_form 5 × grip 3) × N(body texture 仅作模板侧放大，不计样本数) = 45 ≥ 10 ✓
（低质量新增 handle/clamp 关节已从候选中移除；多样性主要由 body_form 的桶身轮廓/纹理、closure 的既有盖子运动，以及 grip 的父样本/顶部提梁承担。）

## Slot 候选覆盖

### Slot A:closure（主机构槽——桶口的封闭/开启动作；joint 拓扑差异最大）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| lift_off_lid（基线-drum） | rec_blue-plastic-open-top-storage-drum-with-a-lift-o_20260606_074440_612125_4e3a36bf | lid / lid_lift(PRISMATIC +Z) / _lid_mesh | 直提脱卸盖,带下垂裙边扣住卷边 rim | converged(parent) |
| screw_cap（基线-keg） | rec_blue-ribbed-plastic-barrel-keg-with-a-round-scre_20260606_074449_217026_b410e9f1 | lid / lid_carrier / lid_rotate(CONTINUOUS +Z) + lid_slide(PRISMATIC +Z) / _lid_geometry | 圆形螺旋盖,旋转+升降经 massless carrier 解耦 | converged(parent) |
| flip_top_hinged_lid | rec_container_barrel_var_flip_top_hinged_lid | lid / lid_hinge(REVOLUTE 横轴@neck 后缘) / _flip_lid_mesh | 后铰翻盖绕颈口卷边横轴掀开,非旋脱 | converged |

### Slot B:body_form（体形/footprint 家族;连续尺寸由模板缩放,这里只列结构不同的形态)
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| tall_cylindrical_ribbed（基线-drum） | rec_blue-plastic-open-top-storage-drum-with-a-lift-o_20260606_074440_612125_4e3a36bf | barrel_body / _barrel_solid + _profile_loft | 近直筒微鼓腹,4 道加强环 | converged(parent) |
| bulged_belly_ribbed（基线-keg） | rec_blue-ribbed-plastic-barrel-keg-with-a-round-scre_20260606_074449_217026_b410e9f1 | body / _barrel_body(LatheGeometry) / _body_mesh | 鼓腹 lathe 桶身,多道 rib 环 | converged(parent) |
| conical_tapered_body | rec_container_barrel_var_conical_tapered_body | barrel_body / _barrel_solid(taper) | 锥形:底宽顶窄,rib 带顺锥面 | converged |
| stepped_waisted_body | rec_container_barrel_var_clamp_ring_lid | barrel_body / _barrel_solid(stepped waist) + recessed_panel_i | 阶梯式宽 rib 带 + 中部收腰 + 深色竖向凹纹面板；移除原 clamp/lever 低质量新增关节 | converged |
| scalloped_lobed_body | rec_container_barrel_var_side_swing_bail_handles | body / _barrel_body(one-piece scalloped mesh) | 一体化波瓣桶身：桶体 mesh 自带圆润竖向波瓣、浅凹槽和融合横向加强带；无贴棍/无侧提梁低质量新增关节 | converged |

### Slot C:grip（提握/把手机构)
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| swing_buckle_clasp_front（基线-drum） | rec_blue-plastic-open-top-storage-drum-with-a-lift-o_20260606_074440_612125_4e3a36bf | clasp_base(FIXED) / clasp_ring / ring_swing(REVOLUTE 横轴) / _clasp_base_mesh + _clasp_ring_mesh | 前缘扣具:固定卡座 + 摆动 U 形锁线 | converged(parent) |
| no_grip_flat_badge（基线-keg） | rec_blue-ribbed-plastic-barrel-keg-with-a-round-scre_20260606_074449_217026_b410e9f1 | （前面 badge 板内联为 body 视觉,无 joint） | 无可动 grip,前贴标牌 | converged(parent) |
| top_swing_bail_handle | rec_container_barrel_var_top_swing_bail_handle | top_bail / bail_pivot(REVOLUTE 横轴@对侧 rim 耳) / _top_bail_mesh | 单道顶部拱形提梁,跨盖摆起摆落 | converged |

## Multiplicity / Copy Logic
- count_param: rib_count / flute_count（桶身加强环、竖向凸筋、暗槽纹理均为 body visual 内联，不新增独立关节）。原 `side_handle_count` 已删除，避免为质量不高的把手机构铺 N。
- N 样本已覆盖: rib_count 由 parent drum/keg + stepped_waisted_body + scalloped_lobed_body 覆盖；lobe_count {10} → rec_container_barrel_var_side_swing_bail_handles（现为 scalloped_lobed_body）。
- 模板建议 N_range: rib_count [3, 12]；lobe_count [8, 14]。这两个 N 只改变一体化桶身几何/纹理密度，全部作为 body mesh 生成，不新增独立 visual 棍条。
- copied object / naming / placement / joint policy: 复制对象 = 加强环(rib)、凹纹面板(recessed_panel_i)；scalloped_lobed_body 走一体化 mesh 半径调制，不复制贴片；joint policy 全部内联为 body visual，不新增 handle/clamp REVOLUTE。

## 格子覆盖(2 parent 占 6 格,5 变体填 5 空格)
- Slot A closure 1 空格 → flip_top_hinged_lid
- Slot B body_form 3 空格 → conical_tapered_body / stepped_waisted_body / scalloped_lobed_body
- Slot C grip 1 空格 → top_swing_bail_handle
共 5 变体填满规划空格(批次规模 = 空格数 5，非固定 20)。Barrel 小类样本池就绪。

## 排除项(未来 compatibility matrix 素材)
- color/material（蓝/黑/绿/锈纹等）、纯尺寸缩放(高矮胖瘦)：非结构轴，按 §2 不立轴,模板侧作参数/材质处理。
- 透明展示桶身 / 网格篓筐式壁面：超出 Barrel(实心封闭桶)真实形态,易出类目,不立轴。
- 嵌套堆叠脚 / 桶底排液阀(spigot/tap)：真实存在但本批未排格(grip 与 closure 已填满空格);若后续回补,作 Slot D(底部附件)新轴,届时再开格。
