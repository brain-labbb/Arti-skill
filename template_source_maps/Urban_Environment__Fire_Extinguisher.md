# Urban_Environment / Fire Extinguisher — template source map

pattern: parallel_children (固定 named slots: body_shape + operating_head 主机构 + discharge + mounting; 无 multiplicity / 无 for-range 多重部件 — 唯一可重复元素是 pivot lug 对)

identity: portable fire extinguisher — 直立 steel 瓶体 + 圆顶 shoulder + brass neck valve + squeeze 操作杆 (REVOLUTE 主关节) + pressure gauge + 侧挂 discharge hose + nozzle + safety pull-pin/ring。变体必须仍读作灭火器。

## parents

- rec_red-portable-fire-extinguisher-a-cylindrical-ste_20260608_170539_994452_8766eb99 ← picture/Urban Environment/Fire Extinguisher/001.png
  - baseline: body_shape:standard_cylinder × operating_head:squeeze_lever(REVOLUTE,horizontal Y,rear cross-pin) × discharge:hose+nozzle × mounting:none × gauge:present
  - **全批 fork 基线**(本 subcat 仅 1 parent → 所有候选来自 variants;axis 计划 plan thick)

## 可读性 / loop 排查（parent model.py）

- 真正的 for-loop:仅 1 处 —— `for s, tag in ((1,"left"),(-1,"right")):` 生成左右 pivot lug 对(lever_lug_left/right)。符合 suffix 的 name_i / shared-helper 期望。
- 手写重复(候选 loop 化 / helper 化目标,供 fork 改进):
  - 每个 visual 都是单独手写 `body.visual(mesh_from_geometry(...), ...)` 调用(base_ring / bottle / dome / neck / label / head / carry / gauge_stem / gauge_case / gauge_dial / safety_pin / ring / hose / nozzle)—— 无共享 helper,但多为独立部件,非真重复。
  - 两段 banding ring 用内联 `for bz in (0.060, 0.300):` 在 profile 内生成(轻量,已是 loop)。
  - gauge 三件(stem/case/dial)沿 -Y 链式手写,可抽 helper 但非重复多重。
- 结论:parent 读性良好,无大段复制粘贴。fork 变体若引入多重部件(如 wheel valve 的 spokes、wall bracket 的 straps)必须走 for-i-in-range(n) + name_i + 共享 geometry helper。

## 组合数预审（HARD GATE）

4 axes,各 ≥2 候选(含 parent baseline):
- body_shape: 3 (standard[parent] + co2_tall_thin + squat_wide)
- operating_head: 3 (squeeze_lever[parent,REVOLUTE] + wheel_valve[REVOLUTE 立轴] + top_pull_trigger[PRISMATIC 立轴])
- discharge: 3 (hose+nozzle[parent] + co2_horn + hoseless_nozzle)
- mounting: 3 (none[parent] + wall_bracket + floor_stand)

product = 3 × 3 × 3 × 3 = **81 ≫ 10** ✓。

## Slot 候选覆盖

### Slot A: body_shape（瓶体形态 / 比例;结构形态差异,非纯 scale）
| 候选 (future module) | variant | 结构特征 | 状态 |
|---|---|---|---|
| standard_cylinder (基线) | parent | 标准直立钢瓶 + 圆顶 shoulder + 双 banding ring | parent |
| co2_tall_thin | var_co2_tall_thin | 高瘦高压 CO2 型,大 L/D 比 | converged |
| squat_wide | var_squat_wide | 矮胖低宽瓶,小 L/D 比 + 宽 base ring | converged |

### Slot B: operating_head（主机构槽 —— 阀门致动动作；joint 拓扑多样）
| 候选 | variant | 关键 joint | 状态 |
|---|---|---|---|
| squeeze_lever (基线) | parent | 侧 squeeze 杆,REVOLUTE 绕后 cross-pin 横 Y 轴下压 | parent |
| wheel_valve | var_wheel_valve | 顶部带辐 hand-wheel,REVOLUTE 绕竖直体轴旋转开关 | converged |
| top_pull_trigger | var_top_pull_trigger | 顶部 thumb-trigger,PRISMATIC 沿竖直轴下行致动 | converged |

### Slot C: discharge（出料方式）
| 候选 | variant | 结构特征 | 状态 |
|---|---|---|---|
| hose+nozzle (基线) | parent | 细黑软管 + 小 nozzle 侧挂 | parent |
| co2_horn | var_co2_horn | 宽锥黑 CO2 discharge horn + 刚性 swept tube | converged |
| hoseless_nozzle | var_hoseless_nozzle | 无软管,短固定 nozzle 直出 valve head 前 | converged |

### Slot D: mounting（安装/承托）
| 候选 | variant | 结构特征 | 状态 |
|---|---|---|---|
| none (基线) | parent | 仅 base ring 立地 | parent |
| wall_bracket | var_wall_bracket | 红钣金壁挂支架,背板 + 抱箍 cradle strap | converged |
| floor_stand | var_floor_stand | 低管式落地架,底板 + 环箍扶正瓶体 | converged |

## 计划新建变体（8 个,cap 8–10 内）

| record_id | label | axis | slot |
|---|---|---|---|
| rec_fire_extinguisher_var_co2_tall_thin | fire_extinguisher-co2_tall_thin | body_shape | A |
| rec_fire_extinguisher_var_squat_wide | fire_extinguisher-squat_wide | body_shape | A |
| rec_fire_extinguisher_var_wheel_valve | fire_extinguisher-wheel_valve | operating_head | B |
| rec_fire_extinguisher_var_top_pull_trigger | fire_extinguisher-top_pull_trigger | operating_head | B |
| rec_fire_extinguisher_var_co2_horn | fire_extinguisher-co2_horn | discharge | C |
| rec_fire_extinguisher_var_hoseless_nozzle | fire_extinguisher-hoseless_nozzle | discharge | C |
| rec_fire_extinguisher_var_wall_bracket | fire_extinguisher-wall_bracket | mounting | D |
| rec_fire_extinguisher_var_floor_stand | fire_extinguisher-floor_stand | mounting | D |

prompts: /tmp/urb_fire_extinguisher_var_<axis>.txt (single-axis prose + blank line + VERBATIM /tmp/urb_suffix_fire_extinguisher.txt)
manifest: /tmp/manifest_urb_fire_extinguisher.tsv (TAB, 4 fields: record_id / label / prompt_path / parent_record_id)

## Dropped / 不采用的 axis

- **gauge_present(有/无 gauge)**:候选 = present[parent] + gauge-less,纯 2 候选且 gauge-less 多为「删一个 visual」非真结构机构,改动太薄;且 co2/CO2 horn 变体本身常无 gauge,已隐含覆盖。降级为 discharge/body 变体的附带属性,不单列为 axis。
- **color / material / 纯 scale**:禁止(suffix 明令 color/material 不算 change;纯 scale 非结构)。
- **label band 样式 / banding ring 数量**:纯装饰,非功能层,排除。
