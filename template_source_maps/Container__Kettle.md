# Container / Kettle — template source map

pattern: parallel_children（固定 named slots:body_form + handle(主握持机构) + lid_closure(主开盖机构) + base/heating;无 multiplicity 复制逻辑——每个功能层都是单一具名件,不存在"同构子件 × N"）

parents（2 个可 fork 母资产,各占 4 个轴上的一个格子）:
- P_electric rec_electric-kettle-with-a-rear-hinged-flip-lid-on-a_20260606_074828_184317_d04a456a ← picture/Container/Kettle/002.png
  （电热水壶:直筒收腰 lathe 壳 + 固定后置 C-handle + 后铰翻盖 lid_hinge REVOLUTE + 独立圆形电源底座 body_lift PRISMATIC。占 body_form=straight_barrel / handle=fixed_c_handle / lid=rear_flip_hinge / base=cordless_power_base 四格。）
- P_stovetop rec_stovetop-whistling-kettle-with-a-lift-off-lid-kn_20260606_074819_789268_89ab783c ← picture/Container/Kettle/001.png
  （炉灶鸣笛壶:钟形 lathe 壳 + 摆动提梁 bail body_to_handle REVOLUTE + 提起式带钮 lid body_to_lid PRISMATIC + 鸣笛 cap spout_to_cap REVOLUTE + 平底直接坐炉面。占 body_form=bell_lathe / handle=swing_bail / lid=liftoff_knob / base=flat_stovetop 四格。）

批次：container_kettle_qwen37max（计划 dashscope qwen3.7-max / medium;P0 已规划并完成 fork）。7 个变体已 fork 收敛(全部 compile success + ≥1 非fixed joint)。

## 组合数预审

组合数预审: Π(body_form 4 × handle 4 × lid_closure 4 × base 3) × N(无) = 192 ≥ 10 ✓
（无 multiplicity 轴;靠 4 根结构轴堆候选,单轴均 ≥3,主机构轴 handle/lid 各 4,远超 2 槽 ×3=9 的下限。）

## Slot 候选覆盖

### Slot A:body_form（壳体轮廓家族 / lathe profile + 出水口形态）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| straight_barrel | P_electric | kettle_body / _body_shell / _loft | 收腰直筒,短上扬 spout | converged(parent) |
| bell_lathe | P_stovetop | body / _body_solid / _loft_z | 钟形(宽底→窄颈),短鼓肚 spout + _spout_root_fairing | converged(parent) |
| gooseneck_pour | rec_container_kettle_var_gooseneck_body | body / 低宽梨形 lathe profile + compact pour_spout helper | 低宽宽肩壶身 + 短顺肩部出水嘴,无高拱 S 形壶管 | revised / converged |
| squat_round | rec_container_kettle_var_squat_round_body | kettle_body / 低矮鼓肚 lathe profile | 矮胖球/卵形腹 | converged |

### Slot B:handle（握持机构——主机构槽）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| swing_bail | P_stovetop | bail_handle / body_to_handle REVOLUTE / _bail_mesh + mount_lug_{0,1} | 拱形提梁绕肩部两 lug 摆落 | converged(parent) |
| fixed_c_handle | P_electric | handle(body.visual) / _handle / _strut | 后置刚性 C 把(无关节,挂在 body 上) | converged(parent) |
| folding_bail | rec_container_kettle_var_folding_bail | bail_legs + grip_span / 双串联 REVOLUTE(肩部 lug + 中部 knuckle) | 两段折叠提梁,grip 可折平 | converged |
| side_loop_handle | rec_container_kettle_var_side_loop_handle | side_loop / loop_pivot REVOLUTE / 后侧两 lug | 侧置 D 形提环绕 lug 线翻起 | converged |

### Slot C:lid_closure（开盖/封口机构——主机构槽）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| liftoff_knob | P_stovetop | lid / body_to_lid PRISMATIC / _lid_mesh + knob_stem/knob_ball | 带钮圆顶盖直提脱离 | converged(parent) |
| rear_flip_hinge | P_electric | lid / lid_hinge REVOLUTE / _lid_solid + hinge_knuckle | 后铰翻盖,前缘上翻后摆 | converged(parent) |
| screw_cap | rec_container_kettle_var_screw_cap_lid | screw_cap / cap_twist REVOLUTE(绕 Z 轴) | 旋拧螺纹盖绕竖直壶轴拧紧 | converged |
| sliding_pour_lid | rec_container_kettle_var_sliding_pour_lid | pour_shutter / shutter_slide PRISMATIC(rim 平面) | 月牙挡板横向滑动开闭出水口 | converged |

### Slot D:base / heating（支撑与加热形式）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_stovetop | P_stovetop | body(平底直接坐炉面,无独立 base part) | 平底坐炉 | converged(parent) |
| cordless_power_base | P_electric | power_base(root) / body_lift PRISMATIC / _base_solid + control_pad/power_button | 独立圆形电源座,壶身直提脱离 | converged(parent) |
| trivet_stand | rec_container_kettle_var_trivet_stand | trivet_stand(root) / body_lift PRISMATIC / 三/四足 trivet 环 | 带足托架,壶身坐其上可提起 | converged |

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots(body / handle / lid / base 各为单一具名功能层)。
- N 样本: 无。
- 模板建议 N_range: 无。
- copied object / naming / placement / joint policy: 无复制件。（trivet_stand 的足、folding_bail 的腿对在各自变体内可用 for-i-in-range 循环发射 leg_{i},但腿数固定属于该候选的内部装饰,不是跨样本的多重性轴;source map 不把它当 multiplicity slot。)

## 排除项（未来 compatibility matrix 素材）
- 暂无;7 个 planned 格子均为真实存在的水壶形态,无已知出类目/不收敛取值。
  （潜在风险待 fork 验证:screw_cap 绕 Z 的 REVOLUTE 与出水口/spout 方位的干涉;sliding_pour_lid 的 prismatic 行程不要穿出 rim——若不收敛回退记此处。）
