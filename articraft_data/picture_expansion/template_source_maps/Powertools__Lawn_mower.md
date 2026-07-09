# Powertools / Lawn mower — template source map

pattern: mixed(mechanism slots A/B/C + multiplicity 轴 = wheel count)
parents: rec_model-a-gas-powered-walk-behind-push-lawn-mower-_20260610_085438_746371_adebd312 ← picture/Powertools/Lawn mower/（gas walk-behind push mower；deck root，engine FIXED，blade CONTINUOUS，4 wheels CONTINUOUS via WHEEL_SPECS loop，handlebar REVOLUTE fold = 6 非 fixed）

## Slot 候选覆盖

### Slot A:power_unit（动力单元，坐 deck 中央 spindle boss）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| gas_engine | rec_model-a-gas-powered-walk-behind-push-lawn-mower-_20260610_085438_746371_adebd312 | `engine`(FIXED) / `blade` + `blade_spin`(continuous) | 立式汽油机壳坐 spindle boss，刀片挂主轴 | converged(parent) |
| corded_electric | rec_lawn_mower_var_electric | `motor`(替 engine) / `blade_spin`(continuous) | 低矮电机壳 + 后出尾随电源线 → deck 导线夹 → 沿把手右侧上行 | converged(workbench, rating pending sync) |
| battery_brushless | rec_lawn_mower_var_battery | `motor` + `battery` / `blade_spin`(continuous) | 紧凑无刷电机毂 + deck 顶可拆矩形电池托架（电机后） | converged(workbench, rating pending sync) |

### Slot B:grass_collection（草屑处理，deck 后/侧排口）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| side_discharge | rec_model-a-gas-powered-walk-behind-push-lawn-mower-_20260610_085438_746371_adebd312 | deck 侧开口(无活动件) | 基线侧抛口，无门无袋 | converged(parent) |
| rear_bag | rec_lawn_mower_var_rearbag | `discharge_door` + `grass_bag` / `discharge_door` hinge(revolute) | 后排口弹簧翻门 + 集草袋（+1 revolute） | converged(workbench, rating pending sync) |
| side_chute | rec_lawn_mower_var_sidechute | `discharge_chute` / chute deflector(revolute, CHUTE_OPEN_UPPER≈1.2rad) | 侧排口曲面导流板，向外上方掀开 | converged(workbench, rating pending sync) |
| mulch_plug | rec_lawn_mower_var_mulchplug | mulch plug insert + rear cover(FIXED) | 后排口封堵曲面塞 + deck 同色后盖，草屑底部回旋粉碎 | converged(workbench, rating pending sync) |

### Slot C:handle（推杆把手，deck 后裙挂点）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| u_handle_fold | rec_model-a-gas-powered-walk-behind-push-lawn-mower-_20260610_085438_746371_adebd312 | `handlebar` / `handlebar_fold`(revolute Y) | 单段 U 形把手，整体绕侧轴折叠 | converged(parent) |
| telescoping_2seg | rec_lawn_mower_var_telehandle | `lower_handle` + `upper_grip` / fold(revolute) + slide(prismatic along rake) | 两段式调高把手，上段沿 rake 棱镜滑移 | converged(workbench, rating pending sync) |
| loop_bullhorn | rec_lawn_mower_var_loophandle | `handlebar` / `handlebar_fold`(revolute Y) | 细管牛角环把手，侧管收束成单 D 形握环 | converged(workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: wheel count via `WHEEL_SPECS` 列表（每条 = (name, x, side, wz, tw, wr, ww)）
- N 样本已覆盖: {4} → parent(front_wheel_0/1 + rear_wheel_0/1) ; {3} → rec_lawn_mower_var_3wheel(centered `front_caster` side=0.0 + rear_wheel_0/1)
- 模板建议 N_range: [3, 4]（仅 push-mower 真实词汇：4-wheel corner 或 front-caster 3-wheel；不外推）
- copied object / naming / placement / joint policy: 复制对象 = 轮（WHEEL_SPECS loop 同时生成 mount 与 `*_spin` joint）；命名 = front_wheel_i / rear_wheel_i / front_caster；放置 = deck 角点轴架，caster 为前中心叉；joint policy = 每轮独立 CONTINUOUS spin

## 排除项(未来 compatibility matrix 素材)
- deck shape：壳形为连续尺寸参数，非候选轴（误当候选会虚胖）
- throttle lever：纯装饰，不建 slot
- 骑乘式 / 卷筒式(reel)割草机：出 Powertools/Lawn mower 类目，排除
- N>4 轮：非 push-mower 真实形态，N_range 上限封 4
- spindle_lock 类附件：保持 parent 基线
