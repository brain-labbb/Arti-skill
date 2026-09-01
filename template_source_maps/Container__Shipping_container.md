# Container / Shipping container — template source map

pattern: mixed（固定 named slots: door_closure + roof_top + wall_surface；door_closure 内含 door-leaf multiplicity 复制轴）

parents:
- rec_white-twenty-foot-shipping-container-with-double_20260606_075009_921544_0e395b54 ← picture/Container/Shipping container/001.png（白色 ISO 20ft 集装箱;body(floor/roof/2 corrugated side_wall/front_wall/door_header/door_sill/post_p/post_n/8 corner_i) + 2 cargo door(door_l/door_r REVOLUTE 侧铰) + 每门 rod_i/keeper_i_j + cam handle(hub/lever/grip REVOLUTE 绕 rod);**door_closure=double_swing_doors / roof_top=flat_solid / wall_surface=corrugated 三轴基线,door 数 N=2**）

## 组合数预审
组合数预审: door_closure 4 × roof_top 3 × wall_surface 3 = 36 ≥ 10 ✓
（door-leaf multiplicity N 采样 {2,4} 在 door_closure 槽内额外放大数量多样性,不计入上式下界）

## Slot 候选覆盖

### Slot A: door_closure（货门/开闭机构 —— 主机构轴;必须保 ≥1 非fixed joint）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| double_swing_doors（基线 N=2） | rec_white-twenty-foot-shipping-container-with-double_20260606_075009_921544_0e395b54 | door_l/door_r · body_to_door_l/r(REVOLUTE z) · _corrugated_door_panel · rod_i/keeper_i_j/handle_*(REVOLUTE) | +X 端双扇侧铰波纹门,各两根锁杆+凸轮把手 | converged(parent) |
| roll_up_door | rec_container_shipping_container_var_roll_up_door | curtain_slat_i loop · body_to_curtain(PRISMATIC +Z) · _slat 共享几何 | +X 端单片分段卷帘门,竖向滑升开启(平移轴) | converged |
| side_access_doors | rec_container_shipping_container_var_side_access_doors | side_door_p/n · body_to_side_door_*(REVOLUTE z 长边铰) | 开口移到长侧壁,沿长度方向开的整长侧门对 | converged |
| single_swing_door | rec_container_shipping_container_var_single_swing_door | door_full · body_to_door_full(REVOLUTE z) · rod_i/handle_* | +X 端单扇满宽侧铰门,锁杆/把手集中在一片上 | converged |

### Slot B: roof_top（顶部结构）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_solid（基线） | rec_white-twenty-foot-shipping-container-with-double_20260606_075009_921544_0e395b54 | body.visual "roof"(固定钢顶板) | 固定平钢顶板 | converged(parent) |
| open_top_tarp | rec_container_shipping_container_var_open_top_tarp | tarp_bow_i loop · _arch_bow 共享几何 · 1 组 bow REVOLUTE 可开 | 去固定顶,改可拆弧形篷弓+软篷盖 | converged |
| hatch_lid | rec_container_shipping_container_var_hatch_lid | roof_lid · body_to_roof_lid(REVOLUTE 沿长顶边) | 整片波纹顶盖沿一条长顶边上翻开顶 | converged |

### Slot C: wall_surface（侧壁/表面结构 —— 非装饰,表面家族更换）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| corrugated（基线） | rec_white-twenty-foot-shipping-container-with-double_20260606_075009_921544_0e395b54 | side_wall_p/n + front_wall · _corrugated_wall / _corrugated_end_wall(竖肋 for-loop) | 竖向波纹钢壁 | converged(parent) |
| louvered_vents | rec_container_shipping_container_var_louvered_vents | louver_slat_i loop · _louver_slat 共享几何 | 侧壁改横向斜置百叶通风片(等距 loop) | converged |
| smooth_reefer_panel | rec_container_shipping_container_var_smooth_reefer_panel | side_wall_p/n + front_wall(平滑齐面蒙皮,无肋) | 去波纹肋,改平滑保温齐面板(冷藏式) | converged |

## Multiplicity / Copy Logic
- count_param: door_count（货门叶片数;parent 当前为手写两元素 door_specs 列表 —— 见排除项/NOTES,multiplicity 变体须改写为 for-i 循环）;次级复制层 corrugation/louver 肋片数(_corrugated_wall / _louver_slat 已是 for-loop) 与 corner_i(8 角,已循环)。
- N 样本已覆盖: door_count {2, 4} → parent / rec_container_shipping_container_var_n4_doorleaves
- 模板建议 N_range: door_count [1, 4]（1=single_swing/roll_up, 2=parent 双扇, 4=多叶）;肋片/百叶 slat_count 由表面长度自适应,建议 [8, 60]
- copied object: 单个货门叶片(door_i = 波纹门板 + rod + keeper + cam handle 链);naming door_i / handle_i / rod_i;placement 沿 +X 开口等宽分割、左右交替铰边;joint policy 每叶独立 REVOLUTE 绕各自竖直铰边(统一 effort/velocity/limits)。

## 排除项（未来 compatibility matrix 素材）
- 暂无不收敛排除项（P0 规划阶段,尚未 fork）。
- 候选数封顶说明: footprint 不作为轴 —— 20ft/40ft/high-cube 之间是纯尺寸(L/H 连续参数,模板侧 controlled local parameterization),不是结构 module,故不立 footprint 槽。
- 颜色/材质(白/灰/锈/集装箱企业涂装)不作为轴,鼓励在结构变化之上自由叠加。
- 跨轴组合(如 roll_up_door × louvered_vents)不专门造变体,留给模板采样器。
