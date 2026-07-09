# Equipment / Pipeline — template source map

pattern: mixed (body/operator/port structural named slots + outlet-count multiplicity loop)

slug: pipeline

parents:
- rec_build-a-realistic-articulated-3d-model-of-a-pipe_20260609_180104_185804_d58aec8c ← picture/Equipment/Pipeline/001.png (yellow gas line: 90° elbow drop + inline **gate valve**, bolted flange pairs, rising stem, 3-spoke chrome handwheel) → covers Slot A=inline_gate_elbow, Slot B=handwheel_3spoke, Slot C=bolted_flange_pair
- rec_build-a-realistic-articulated-3d-model-of-a-pipe_20260609_180106_931724_6c9b7c6f ← picture/Equipment/Pipeline/002.png (fire-hydrant standpipe: tall riser + bulbous body, 6-spoke wheel, **two brass outlet nozzles + lift caps + retaining chains**) → covers Slot A=standpipe_riser, Slot B=handwheel_6spoke; **the outlet/loop multiplicity parent (N=2)**
- rec_build-a-realistic-articulated-3d-model-of-a-pipe_20260609_180109_734374_b30771ae ← picture/Equipment/Pipeline/003.png (blue flanged **globe valve**: spherical body, bolted RF flanges, yoke/gland, red 6-spoke handwheel) → covers Slot A=globe_sphere, Slot B=handwheel_6spoke, Slot C=bolted_RF_flange

All three parents loop-emit their repeats (flange bolt ring `for i in range(N_BOLTS)`, wheel spokes `for i in range(N_SPOKES)`, hydrant chain links `for i in range(n_links)`), so the copy-logic is already clean for the multiplicity slot.

## Slot 候选覆盖

### Slot A:body_form(主壳体——被铰接 operator 与端口寄生的固定根体)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| inline_gate_elbow | rec_..._d58aec8c (parent) | part `pipeline_body`;`_pipeline_run`(垂直腿 + `_quarter_torus` 弯头 + 水平段,annular extrude 空心)、`_valve_body`(中段加粗 barrel) | 黄色直管带 90° 下弯肘,中央内联闸阀体 | converged (parent) |
| globe_sphere | rec_..._b30771ae (parent) | part `valve_body`;`_make_body_mesh`(`sphere(BODY_R)` 球体 + `port(sign)` loft 颈 + 沿 X bore 通孔) | 球形铸铁阀体,两侧端口颈穿过球心 | converged (parent) |
| standpipe_riser | rec_..._6c9b7c6f (parent) | part `standpipe_body`;`_riser_shape`(高立管)、`_collar_shape`、`_body_shape`(revolve 鼓形体)、`_bonnet_neck_shape` | 竖立高立管 + 鼓形铸铁阀体,顶置手轮 | converged (parent) |
| straight_through_gate | rec_pipeline_var_body_straightgate (fork d58aec8c) | `_pipeline_run`(改写为直进口段,**去除弯头**)、`_valve_body` | 纯水平直通闸阀,无肘弯 | converged |
| angle_valve_90 | rec_pipeline_var_body_anglevalve (fork d58aec8c) | `_angle_quarter_torus`、`_inlet_flange_pair` / `_outlet_flange_pair`(互相垂直端口)、`_valve_body` | 90° 角阀:进口水平、出口转向,阀体在拐角 | converged |

### Slot B:operator(被铰接的执行控制件)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| handwheel_3spoke | rec_..._d58aec8c (parent) | part `handwheel`;`_handwheel`(rim torus + hub + `for i in range(N_SPOKES)` N_SPOKES=3);joint `valve_handwheel` CONTINUOUS +Z | 铬三辐手轮,绕立轴连续旋转 | converged (parent) |
| handwheel_6spoke | rec_..._b30771ae / rec_..._6c9b7c6f (parents) | `_make_handwheel_mesh` / `_wheel_shape`(N_SPOKES=6 / WHEEL_N_SPOKES=6);joint `body_to_handwheel` / `body_to_wheel` CONTINUOUS +Z | 六辐手轮 | converged (parent) |
| handwheel_3spoke_globe | rec_pipeline_var_operator_3spoke_globe (fork b30771ae) | `_make_handwheel_mesh`(N_SPOKES=3);joint `body_to_handwheel` | 球阀血统上的三辐手轮(spoke 数轴值) | converged |
| handwheel_5spoke | rec_pipeline_var_operator_5spoke (fork b30771ae) | `_make_handwheel_mesh`(N_SPOKES=5);joint `body_to_handwheel` | 五辐手轮 | converged |
| quarter_turn_lever | rec_pipeline_var_operator_lever (fork b30771ae) | part `lever_handle`,visual `lever_bar`;`_make_lever_mesh`(hub + 单根 bar + end_knob)、`_make_stem_mesh`(固定 `fixed_stem`);joint `body_to_lever` **REVOLUTE** +Z,limits [0, π/2] | 四分之一转直杆扳手(球阀/蝶阀式),旋转受限 | converged |
| crossed_tee_bar | rec_pipeline_var_operator_teebar (fork d58aec8c) | part `cross_handle`,visuals `handle_hub` / `handle_bar_0` / `handle_bar_1`;`_handle_hub`、`_handle_bar(index)`(`for i in range(2)` 互转 90°);joint `valve_handle` CONTINUOUS +Z | 十字 tee-bar 手柄替代辐条轮 | converged |

### Slot C:port(管端/管-阀联接形式)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| bolted_flange_pair | rec_..._d58aec8c (parent) | visual `flanges`;`_flange`(raised face + `for i in range(N_BOLTS)` N_BOLTS=12 六角螺栓环)、`_flange_pair`(4 盘:进/出口各一对) | 螺栓法兰对,面对面对夹 | converged (parent) |
| bolted_RF_flange | rec_..._b30771ae (parent) | `_make_body_mesh.port()`(loft 颈 + `circle(FLANGE_R)` 盘 + 凸面 rf)、`_make_bolts_mesh`(`_bolt_ring` N_BOLTS=8) | 凸面 RF 法兰 + 螺栓环 | converged (parent) |
| socket_weld | rec_pipeline_var_port_socketweld (fork d58aec8c) | visuals `socket_collar_0` / `socket_collar_1`;`_socket_collar`(光滑环形 bell 套筒,**无盘、无螺栓**)、`_socket_joint_centers` | 平口承插焊/推入式套筒接头 | converged |
| union_nut | rec_pipeline_var_port_unionnut (fork b30771ae) | visuals `union_nut_0` / `union_nut_1`;`_make_union_coupling_mesh`(`_x_hex_prism` 六角螺母 + `_x_cylinder` 两侧 shoulder 环) | 活接(union)六角螺母联管节 | converged |

注:Slot C 两个 parent 均为螺栓法兰(flange_pair / RF_flange),socket_weld 与 union_nut 两变体补足非法兰端口;法兰格子无需另造变体。

## Multiplicity / Copy Logic
- count_param: `OUTLET_COUNT`(rec_pipeline_var_outlet_1)/ `OUTLET_N`(rec_pipeline_var_outlet_3);parent 6c9b7c6f 的 N=2 是手写 `right_cap`/`left_cap`(`body_to_right_cap`/`body_to_left_cap`),**未循环化**。
- copied object: 一整套出口总成 = nozzle visual `nozzle_{i}`(`_nozzle_shape`)+ 锚耳 `chain_lug_{i}`(`_chain_lug_shape`)+ part `cap_{i}`(`_cap_shape`)+ 一条 retaining 链 `chain_{i}`(`_add_subchain`,链节 part `chain_{i}_{j}`)。
- naming: `nozzle_{i}` / `chain_lug_{i}` / `cap_{i}`,cap 滑动关节 `body_to_cap_{i}`,链节关节 `chain_{i}_swing_{j}`,统一 `for i in range(OUTLET_COUNT/OUTLET_N)` 发射。
- placement: N=1 沿 +X 单出口(direction);N=3 由 `_outlet_angle(i)=2π·i/OUTLET_N` 绕 +Z 等分、`_outlet_local_to_body(angle, local)` 做径向放置;parent N=2 为对侧 ±X。
- joint policy: 每个 cap 独立 PRISMATIC,沿其出口局部径向轴(N=1 为 +X;N=3 经 `rpy=(0,0,angle)` 旋到各自方向),lower=-CAP_DEFAULT_PULL、upper=0.020;每个链节独立 REVOLUTE,limits ±35°(CHAIN_SWING),互不联动。
- N 样本已覆盖: N=1 → rec_pipeline_var_outlet_1 ; N=2 → parent rec_..._6c9b7c6f(手写,未循环);N=3 → rec_pipeline_var_outlet_3
- 模板建议 N_range: **[0, 6]**(规划轴 N{0,1,2,3};N=0 = 无出口的纯立管/直管。采样域大于样本属正常,大 N 留长尾余量)。
- 备注:parent 的 N=2 是手写 `right_cap`/`left_cap`,**不可直接作 copy-logic 源**;multiplicity 模板应以 outlet_1/outlet_3 两变体的 `for i in range(N)` + `cap_{i}` 循环链为源码蓝本(outlet_3 的 `_outlet_angle`/`_outlet_local_to_body` 是任意 N 的径向放置范式)。

## 跨层接口(未来 InterfaceSpec 预填)
- operator ↔ body:所有手轮/扳手/tee 把手的 hub 局部 z=0 即落在 rising stem 顶面;joint origin 贴 stem 顶面(gate 血统 elem `valve_stem`,x=X_VALVE_CENTER;globe 血统 `fixed_stem`/`dome_stem`,x=y=0),axis=+Z。mating face = stem 顶面圆,anchor = stem 轴心。lever 用 REVOLUTE(限位),手轮/tee 用 CONTINUOUS。
- port ↔ pipe-run / valve-body:法兰/承插/活接套筒都包在管-阀耦合处的 joint plane(gate 血统 `_flange_pair` 的 x_in/x_out、`_socket_joint_centers`;globe 血统 `port(sign)` 颈端),mating face = 该平面处 PIPE_OR 环面,沿 ±X 端口轴对夹。
- outlet ↔ body:nozzle root 嵌入鼓形体壁内(NOZZLE_ROOT_X,被 body 捕获),cap 的 PRISMATIC 原点位于 nozzle tip;链根 pin 在 cap 的 boss(`CHAIN_BOSS_LOCAL`,随 cap 走),链尾 link 缠绕 body 肩部 `chain_lug_{i}`(`CHAIN_LUG`)。

## 排除项(未来 compatibility matrix 素材)
- 暂无连续不收敛取值(全部规划格子均 converged)。
- 已知重复轴值(交给 compatibility matrix 去重,不重复造):operator=3 辐同时出现在 gate parent(handwheel_3spoke)与 globe 变体(handwheel_3spoke_globe),二者血统不同;模板应把"spoke 数 = {3,5,6}"视作 operator 子轴,而非两个独立 module。
- 跨格需重推(未强造):Slot A=globe_sphere × Slot C=socket_weld/union_nut 中,`_socket_collar` 写在直管血统(以 PIPE_OR 为基),与 globe 的 loft 颈不共参数;`_make_union_coupling_mesh` 写在 globe 血统(PORT_HALF_SPAN/UNION_NUT_L),反向亦然——跨血统的 body×port 组合留给模板矩阵时再按当时几何重派生。
- 出类目风险(主动排除,未列候选):把手轮换成纯旋钮/拨盘(读作仪表而非阀门);给 inline_gate 直管去掉 operator(失去铰接,出类目)。

---
## Post-fork verification (SEGMENT 1 complete)
All 10 planned variants forked via `articraft fork` from their listed parents, then verified on-disk: last compile = success, ≥1 non-fixed joint present (CONTINUOUS handwheel / REVOLUTE lever / PRISMATIC caps + REVOLUTE chain links), collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Equipment__Pipeline` subcat shard (reconcile rebuilt). Slot A/B/C status cells and the multiplicity N samples reflect built-on-disk records. Ready for SEGMENT 2 (spec authoring).
