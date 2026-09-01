# Powertools / angle grinder — template source map

pattern: mixed(mechanism slots A/B/C + multiplicity 轴 D = side-handle mount points)
parents: rec_model-a-compact-electric-angle-grinder-in-dewalt_20260610_085450_800276_28a8a3ef ← picture/Powertools/angle grinder/（compact corded DeWalt-style grinder；body root，`spindle_disc` CONTINUOUS + `power_switch` PRISMATIC + `spindle_lock` PRISMATIC = 3 非 fixed；机壳散热缝 `vent_slot_{i}` loop；无 guard）

## Slot 候选覆盖

### Slot A:disc_guard（磨片 + 护罩组合）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| bare_disc | rec_model-a-compact-electric-angle-grinder-in-dewalt_20260610_085450_800276_28a8a3ef | `spindle_disc` / `*_spin`(continuous) | 裸纤维磨片：轴 + 背法兰 + 片 + 六角夹，无护罩 | converged(parent) |
| half_shroud_guard | rec_angle_grinder_var_guard | `guard` / guard_to_body(revolute) | 半圆金属护罩夹于 spindle 颈(guard_collar_r≈0.0165)，绕主轴可调 | converged(workbench, rating pending sync) |
| cutting_wheel_deep_guard | rec_angle_grinder_var_guardcut | `wheel_guard` + `cutting_wheel`(DISC_RADIUS≈0.0575, ~1mm 薄) / guard(revolute) | 薄切割片 + 深 U 形切割护罩 | converged(workbench, rating pending sync) |

### Slot B:switch（电源开关）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| top_slide | rec_model-a-compact-electric-angle-grinder-in-dewalt_20260610_085450_800276_28a8a3ef | `power_switch` / slide(prismatic) | 顶部黑色拨片滑动开关 | converged(parent) |
| deadman_trigger | rec_angle_grinder_var_trigger | `trigger_paddle` / pivot(revolute) | 握把下方死手扳机，握紧时向机壳旋转 | converged(workbench, rating pending sync) |
| top_rocker | rec_angle_grinder_var_rocker | `rocker_switch` / pivot(revolute Y) | 顶部黑色摇臂拨钮，横跨筒身翻转 | converged(workbench, rating pending sync) |

### Slot C:power_source（供电）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| corded | rec_model-a-compact-electric-angle-grinder-in-dewalt_20260610_085450_800276_28a8a3ef | body 尾电源线(无活动件) | 有线，机壳尾出线 | converged(parent) |
| cordless_battery | rec_angle_grinder_var_cordless | `battery_pack` / battery_release(prismatic rail) | 握把尾滑入式可充电池，沿导轨棱镜滑移 | converged(workbench, rating pending sync) |

### Slot D:side_handle_mount（侧握把安装座，multiplicity 轴）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| N1 | rec_model-a-compact-electric-angle-grinder-in-dewalt_20260610_085450_800276_28a8a3ef | 单 +Y boss(FIXED) | 单侧握把螺纹座 | converged(parent) |
| N2 | rec_angle_grinder_var_handle2 | `handle_boss_{i}` ×2 (HANDLE_BOSS_COUNT=2, range, FIXED) | 左右两 flank 各一座 | converged(workbench, rating pending sync) |
| N3 | rec_angle_grinder_var_handle3 | `handle_boss_{i}` ×3 (range(3), FIXED) | 两 flank + 顶面共三座 | converged(workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: `HANDLE_BOSS_COUNT` 驱动 `handle_boss_{i}` loop（boss part + `body_to_handle_boss_{i}` FIXED joint）
- N 样本已覆盖: {1, 2, 3} → parent / rec_angle_grinder_var_handle2 / rec_angle_grinder_var_handle3
- 模板建议 N_range: [1, 3]（真实角磨机侧握把孔位 1–3 个）
- copied object / naming / placement / joint policy: 复制对象 = 螺纹侧握把座；命名 = handle_boss_i；放置 = gear-head ±Y flank（FLANK_Y≈0.032）+ 顶面；joint policy = 全部 FIXED 在 body 上（无独立活动）

## 排除项(未来 compatibility matrix 素材)
- disc type：折入 Slot A（磨片/切割片随护罩候选一并切换）
- body/housing form：单图，机壳形态不另立轴
- cooling-vent count：`vent_slot_{i}` 为模板 N_range 连续装饰，非 fork cell
- spindle_lock：保持 parent 基线，不作候选
