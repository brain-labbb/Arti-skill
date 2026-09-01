# Urban Environment / utility box — template source map

pattern: parallel_children(主机构 = front door/lid 开合 + body shell + base + ventilation + roof;door-count / louver-row / leg-count 三处可选 multiplicity)

identity: 街边电力/公用配电柜 (boxy steel enclosure, openable hinged door(s) REVOLUTE — 顶翻 lid 也算, louvered vents, base plinth/legs/pad, roof cap/canopy)。变体一律保持 street electrical/utility cabinet 读法。

## Parents（4 个,预填格子,均为现成 fork 基线）

| parent record_id | 图 | 基线槽位 | 备注 |
|---|---|---|---|
| rec_tall-narrow-street-electrical-utility-cabinet-we_20260608_164447_204319_864f73c7 | 001.png | door:single_revolute_side × body:tall_narrow × vent:louver×2 × base:steel_plinth × roof:flat_drip_cap | 单前门立轴 REVOLUTE,门面 2 条 louver(`door_grille_{k}` 已 loop),+X 铰 |
| rec_grey-galvanized-steel-street-electrical-distribu_20260608_164505_793099_7e9a7c89 | 002.png | door:single_revolute_side × body:medium × vent:louver×1(底) × base:concrete_plinth(宽于体) × roof:flat_drip_cap | 单门 + 警示牌 + 闪电符 + 底 louver + 两 conduit 桩 |
| rec_wide-double-door-street-electrical-cabinet-on-a-_20260608_164520_373943_20824ffb | 003.png | door:double_mimic × body:wide_squat × vent:louver×1/门 × base:stepped_concrete × roof:flat_drip_cap | 双门 mimic(各 outer 铰,镜像不 yaw 翻),中 mullion |
| rec_small-ground-level-steel-utility-junction-box-st_20260608_164538_025408_6f7d1e9a | 004.png | door:top_revolute_lid + hasp_clasp × body:small_cube_low × vent:solid × base:four_short_legs × roof:N/A(lid 即顶) | 顶翻盖(横轴 REVOLUTE) + 前 hasp 卡扣(REVOLUTE),四短腿 |

## 组合数预审（HARD GATE）

槽位候选(含 parent 基线):
- door 主机构槽:single_revolute_side / double_mimic / triple_door_bank(N) / roller_shutter(PRISMATIC) / top_revolute_lid = **5 候选**(含 REVOLUTE-侧立轴 / REVOLUTE-横顶轴 / PRISMATIC 三种 joint 拓扑)
- body footprint:tall_narrow / wide_squat / medium / cube = **4 候选**
- ventilation:solid / louver_rows(N) / mesh_grid = **3 候选**
- base/support:steel_plinth / concrete_plinth / stepped_concrete / four_legs / tall_legs(N) = **5 候选**
- roof:flat_drip_cap / pitched_canopy = **2 候选**

product = door(5) × body(4) × vent(3) × base(5) × roof(2) = **600**;另有 distinct-N 三处(door-count N、louver-row N、leg-count N,各 2–3 个 N)。

## Slot 候选覆盖

### Slot A:door / opening 主机构（柜体开合动作 —— 真正的 joint）
| 候选(未来 module) | variant | 关键 joint / 结构 | 状态 |
|---|---|---|---|
| single_revolute_side（基线） | parent(tall / grey) | 单前门立轴(+Z)REVOLUTE | parent(现成) |
| double_mimic（基线） | parent(wide) | 双门各 outer 立轴 REVOLUTE + mimic | parent(现成) |
| top_revolute_lid + hasp（基线） | parent(small) | 顶翻盖横轴 REVOLUTE + 前 hasp REVOLUTE | parent(现成) |
| triple_door_bank | var_triple_door_bank | N 门横排,各立轴 REVOLUTE,`door_{i}` loop + 共享 helper | converged |
| roller_shutter | var_roller_shutter | 卷帘 curtain 竖直 **PRISMATIC** 升降,`shutter_slat_{i}` loop | converged |

### Slot B:body footprint（体形/比例;连续尺寸由模板侧缩放,这里只列结构形态）
| 候选 | variant | 结构特征 | 状态 |
|---|---|---|---|
| tall_narrow（基线） | parent(tall) | 高窄 | parent |
| wide_squat（基线） | parent(wide) | 宽矮(略宽于高) | parent |
| medium（基线） | parent(grey) | 中等比例 | parent |
| cube | var_cube_footprint | 近立方体 squat 柜体 | converged |

### Slot C:ventilation（通风样式）
| 候选 | variant | 结构特征 | 状态 |
|---|---|---|---|
| louver_slots（基线） | parent(全) | 水平百叶 slot(`door_grille_{k}` 已 loop) | parent |
| solid（基线） | parent(small) | 实心无 vent | parent |
| louver_rows(N) | var_louver_rows | N 行水平百叶满门堆叠,`louver_{i}` loop | converged |
| mesh_grid | var_mesh_grille | 网孔穿孔栅格(`mesh_hole_{i}` 行列 loop) | converged |
| double_louver_doors(N/门) | var_double_louver_doors | 双门 mimic,每门 N 行 louver `louver_{i}` loop（door + vent 复合轴） | converged |

### Slot D:base / support（底座/支撑)
| 候选 | variant | 结构特征 | 状态 |
|---|---|---|---|
| steel_plinth（基线） | parent(tall) | 短钢裙 plinth | parent |
| concrete_plinth（基线） | parent(grey) | 混凝土基座(宽于体) | parent |
| stepped_concrete（基线） | parent(wide) | 两级阶梯混凝土 | parent |
| four_short_legs（基线） | parent(small) | 四短腿 + foot pad(`leg_{i}` 已 loop) | parent |
| tall_legs(N) | var_tall_legs | N 条高细钢腿离地架空,`leg_{i}` loop + 统一 fixed joint | converged |

### Slot E:roof（顶盖)
| 候选 | variant | 结构特征 | 状态 |
|---|---|---|---|
| flat_drip_cap（基线） | parent(tall/grey/wide) | 平顶 drip cap | parent |
| pitched_canopy | var_canopy_roof | 斜坡/坡顶悬挑雨棚(lofted/angled 几何,非 box 占位) | converged |

## Multiplicity / Copy Logic

- door-count N（triple_door_bank):N_range 建议 ~2–4 门(parent 双门 = N2,本 var 取 N3;模板可扩 N4)。`door_{i}` loop + 共享 `_build_door` helper + 规则 X 间距 + 统一 REVOLUTE。
- louver-row N（louver_rows / double_louver_doors):N_range ~3–6 行;`louver_{i}` loop + 共享 grille helper + 规则竖直 pitch。
- leg-count N（tall_legs):N_range = 4(角)为主,可 6(长体加中腿);`leg_{i}` loop。
- roller_shutter 的 `shutter_slat_{i}` 是装饰性 rib loop(非独立 joint 复制),N 跟随 curtain 高度。
- 三处 multiplicity 各 2–3 个 distinct N,满足 SPEC multiplicity 2–3 N 要求。

## Loop / 可读性 notes

- parent shell 墙体 `shell_{i}`(`_hollow_box` enumerate)、hinge `hinge_{j}`、louver `door_grille_{k}`、legs `leg_{li}` / `foot_pad_{li}`、rivet 群均已 loop 发射 ✓ —— parents 读性合格。
- **必须请求 loop 重写的多重性变体**(prompt 已写入 for-i-in-range 要求):
  - var_triple_door_bank → `door_{i}` loop(parent 双门是手写 mirror 两调用,N-门必须改 helper+loop)。
  - var_louver_rows / var_double_louver_doors → `louver_{i}` loop(parent louver 数硬编码 1–2,变 N 需 loop)。
  - var_mesh_grille → `mesh_hole_{i}` 行列 loop。
  - var_tall_legs → `leg_{i}` loop(沿用 parent small 的 leg loop 风格)。
  - var_roller_shutter → `shutter_slat_{i}` loop。
- 无手写重复残留需要单独清理;parent small 的 rivet/rib 群虽多但已 loop 化,变体继承即可。

## 排除项 / dropped axes（未来 compatibility matrix 素材）

- **纯颜色/材质/涂鸦贴纸**:parent 图多含 graffiti/sticker,属表面变化,FORK_VARIANTS 禁作结构轴 —— 不立轴。
- **纯尺寸缩放**(只改 W/D/H 数值不改拓扑):由模板侧连续缩放覆盖,不单列 fork 变体(避免连续尺寸虚胖)。
- **conduit 桩 / 警示牌 / 闪电符 / hasp 装饰**:non-moving 装饰,作 parent visual,不立 FIXED-joint 装饰件,不作独立轴。
- **hinge knuckle 数 / 把手样式**:微观配件差异,归比例/装饰参数,不作结构轴。
- var_canopy_roof 的 roof 轴与 var_cube_footprint 的 body 轴可同体出现(次级特征作参数),写 spec 时按 headline 轴归 module,勿当独立候选。
