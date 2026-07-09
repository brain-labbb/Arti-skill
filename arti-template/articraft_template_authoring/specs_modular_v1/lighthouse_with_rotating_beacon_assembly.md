# Lighthouse With Rotating Beacon Assembly Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `lighthouse_with_rotating_beacon_assembly` |
| template path | `agent/templates/lighthouse_with_rotating_beacon_assembly.py` |
| test path (optional) | `tests/agent/test_lighthouse_with_rotating_beacon_assembly_template.py` — 可选回归网，默认被 pytest 排除；验收以 compile-sweep 为准 |
| stage | `APPROVED` |
| status | `APPROVED` |
| __modular__ | `True` |
| pattern | `mixed`（静态 body 身份链 + 1 个 CONTINUOUS 旋转 beacon 子件 + 可选 gated REVOLUTE 开口件；lantern 解构成 0..N 个 FIXED 子件） |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 34 |
| read_count | 34 grep 全扫；11 个精读 |
| read_scope | 34 个 5 星全部用脚本抽取了 `model.part` 列表 / 每个 `model.articulation`(name,type,parent,child,axis,lower,upper) / 几何 primitive 直方图（Box/Cylinder/Sphere/Cone/Lathe/Torus/Extrude/Mesh/cadquery）/ part-joint 拓扑骨架分类（见下表）。其中 11 个按行精读以采纳可复用源码块：`0001`,`0002`,`0003`,`638131`,`469e7a`,`0cd65f`,`9157e1`,`5f19a5`,`4767dd` 完整精读，`582b41`,`023d0f`,`a02180`,`25675b` 关键段精读 |
| samples_adopted_as_module_sources | 13 |
| samples_read_but_not_adopted | 21（落在已建模结构族里，无新拓扑，仅尺寸/配色/装饰差异） |

全 34 样本身份骨架（`#parts, #FIXED, #REVOLUTE, #CONTINUOUS`）分布：

| 骨架 | 计数 | records |
|---|---|---|
| `(2,0,0,1)` body+beacon | 7 | 25245e, 582b41, 66ebf9, 8dc6e6, 9157e1, a01351, f235b1 |
| `(3,0,1,1)` body+beacon+开口件 | 16 | 0cd65f, 12f0c4, 1725c0, 3e0948, 448f6b, 469e7a, 55a25f, 5adf99, 5f19a5, a02180, a44e6e, b7466f, c444e2, c54cb4, eea009, fdfab4 |
| `(3,1,0,1)` body+FIXED shaft+beacon | 3 | 0001, 0002, 279fff |
| `(4,1,1,1)` body+FIXED pedestal/shaft+beacon+开口件 | 3 | 023d0f, 4767dd, 63fc7f |
| `(5,3,0,1)` body+lantern/roof/pedestal FIXED stack+beacon | 1 | 0003 |
| `(6,3,1,1)` 同上 + 开口件 | 2 | 25675b, 3ebd11 |
| `(7,4,1,1)` gallery/lantern/roof FIXED stack+beacon+开口件 | 1 | 8ad647 |
| `(7,5,0,1)` caisson/tower/gallery/lantern/roof/pedestal FIXED stack+beacon | 1 | 638131 |

身份恒量：**34/34 都恰有一个 CONTINUOUS 关节绕竖直轴 `(0,0,±1)` 旋转 beacon**；其它关节全是 FIXED（结构堆叠）或 1 个可选 REVOLUTE（开口件）。没有任何样本把 beacon 做成 REVOLUTE 限位或 PRISMATIC，也没有第二个旋转 DOF。

样本逐条标注（adopted / not adopted）：
- `0001` — adopted（S1）：lathe 砖石塔身 + Torus 画廊栏杆 + 堆叠 optic-ring-cage beacon；`tower_to_lantern` FIXED 把 lantern_shell 拆成第二件
- `0002` — adopted（S2）：lathe shell 塔顶 + 独立 FIXED `central_shaft` + 侧射反射镜 carriage beacon
- `0003` — adopted（S3）：lantern_room/roof_cap/pedestal 三件 FIXED 解构 + 侧臂 lamp+counterweight light_head
- `638131` — adopted（S4）：caisson→tower→gallery→lantern→roof→pedestal 6 件 FIXED 堆叠 + 双 lens-drum beacon_head
- `469e7a` — adopted（S5）：开放钢桁架 lattice 塔 + 捕获 spindle-clip(Torus) 的 bivalve Fresnel beacon + 侧铰 hatch
- `0cd65f` — adopted（S6）：monolithic tower（全 baked visual）+ birdcage 双 lens carriage + 画廊栏杆 gate
- `9157e1` — adopted（S7）：手写 MeshGeometry 棱柱塔身（`_prism_mesh`/`_frustum_mesh`）+ 双向 bar-lamp beacon
- `5f19a5` — adopted（S8）：cadquery union 塔身（stone_body/gallery/lantern_frame）+ cadquery drum beacon + 弧形 lantern-wall door + 捕获 hinge_pin
- `4767dd` — adopted（S9）：条纹 shell-band 塔身 + 独立 FIXED `pedestal` 件 + 侧臂 lens+counterweight beacon + gallery 甲板 trap_door
- `582b41` — adopted（S10）：lathe parabolic-reflector + lamp searchlight carriage（Torus 轴承环捕获塔身轴），2 件极简骨架
- `023d0f` — adopted（S11）：独立 FIXED `central_shaft` + 高 sleeve `lens_carriage`（`_lens_shell` Fresnel 环带，360° omni）+ 全高 lantern-wall door
- `a02180` — adopted（S12）：lathe-heavy 塔 + 双 lens beacon_carriage + 画廊 access_gate
- `25675b` — adopted（S13）：ExtrudeGeometry plinth + section_loft 塔 + lantern_room/roof_cap/pedestal FIXED 解构 + service_door
- `12f0c4` — not adopted：cadquery 塔 + birdcage beacon + gate，结构同 S6/S8 族
- `1725c0` — not adopted：lathe+loft 塔 + hatch，同 S5 hatch 族
- `3e0948` — not adopted：cadquery lantern + beacon + service_door，同 S8/S11 族
- `448f6b` — not adopted：cadquery 塔 + beacon + door，同 S8 族
- `55a25f` — not adopted：Extrude body + beacon_carriage + service_door，同 S13/S11 族
- `5adf99` — not adopted：lathe+torus 塔 + beacon_drum + service_door，同 S1/S6 族
- `a44e6e` — not adopted：Torus 密集 Fresnel beacon + service_door，同 S5 birdcage 族
- `b7466f` — not adopted：lathe+torus 塔 + beacon_head + hatch，同 S5 族
- `c444e2` — not adopted：cadquery body + beacon + access_panel，同 S8 族
- `c54cb4` — not adopted：lantern + lens_carriage + door，同 S11 omni-drum 族
- `eea009` — not adopted：cadquery 塔 + beacon + gate，同 S8/S12 族
- `fdfab4` — not adopted：Mesh body + beacon + trap_door，同 S7/S9 族
- `3ebd11` — not adopted：tower/gallery/lantern/pedestal FIXED stack + gate，同 S3/S4 族
- `8ad647` — not adopted：gallery/lantern/roof/shaft 多件 FIXED stack + gate，同 S4/S13 族
- `279fff` — not adopted：lathe 塔 + central_shaft + beacon，同 S2 族
- `63fc7f` — not adopted：body + central_shaft + beacon + service_door，同 S2/S11 族
- `25245e` — not adopted：cadquery 极简 2 件，同 S8 族缩水版
- `66ebf9` / `8dc6e6` / `a01351` / `f235b1` — not adopted：2 件 body+beacon 极简，几何分别落在 lathe/torus（S1/S10）、mesh（S7）族

## 核心身份
绕竖直轴持续旋转灯光头的灯塔（lighthouse with rotating beacon）：必须有一座**落地静态塔身**（masonry/混凝土/钢桁架柱体，向上收锥，顶部带 gallery 画廊平台 + lantern room 灯室玻璃罩 + roof 顶盖），以及一个坐在塔顶 lantern 内、绕**竖直 Z 轴 CONTINUOUS 无限位自旋**的 beacon/optic 旋转件（旋转的 Fresnel 透镜 / 反射镜灯组 / 灯光头）。wow 与唯一主运动是 beacon 转动——它像真实灯塔一样把光束扫过一圈。塔身、画廊、灯室、顶盖全部静止（baked visual 或 FIXED 子件），只有 optic 旋转。可选第二自由度是一扇 REVOLUTE 开口件（灯室检修门 / 画廊栏杆门 / 塔壁 hatch / 画廊甲板 trap door），但它是 gated 可选件，不是身份。

边界：
- 不是无旋转件的纯静态塔标/方尖碑/纪念柱（必须有 CONTINUOUS beacon）。
- 不是水平轴风机/风扇（旋转件必须绕**竖直** Z 轴，不是水平 X；无叶片阵列、无机舱偏航）。
- 不是探照灯/云台（lighthouse 必须有完整塔身 + gallery + lantern 灯室罩；旋转件被灯室罩住，不是裸露在三脚架上的探照灯）。
- 不是旋转餐厅/旋转塔楼（旋转的是顶部小 optic 灯头，不是整层楼面；塔身本体静止）。
- beacon 旋转默认 CONTINUOUS 无限位；做成 REVOLUTE 限位扫角或 PRISMATIC 即出界。

## 采用源码索引（Adopted Source Index）
| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| S1 | `rec_lighthouse_with_rotating_beacon_assembly_0001` | `data/records/rec_lighthouse_with_rotating_beacon_assembly_0001/revisions/rev_000001/model.py:L36-L266` | `_polar`/`_add_ring_posts`/`_add_radial_window` helper + `LatheGeometry` 砖石塔身 profile + Torus 画廊栏杆 + 径向窗 visual（Slot A `lathe_masonry_shell`）|
| S2 | `rec_lighthouse_with_rotating_beacon_assembly_0001` | `.../rec_..._0001/revisions/rev_000001/model.py:L388-L473` | 堆叠 optic-ring-cage beacon（turntable + lower/upper optic ring + optic_post 环 + optic_panel + lamp_core）与 `lantern_shell→beacon` CONTINUOUS(Z) 写法（Slot C `stacked_optic_ring_cage`）|
| S3 | `rec_lighthouse_with_rotating_beacon_assembly_0002` | `data/records/rec_lighthouse_with_rotating_beacon_assembly_0002/revisions/rev_000001/model.py:L34-L119` | `LatheGeometry.from_shell_profiles` ring-shell helper + 独立 FIXED `central_shaft` 件 + 侧射 reflector_shell carriage（Slot D `separate_fixed_shaft_part` + Slot C searchlight）|
| S4 | `rec_lighthouse_with_rotating_beacon_assembly_0002` | `.../rec_..._0002/revisions/rev_000001/model.py:L202-L309` | `central_shaft` part(shaft_base+shaft_main) + `tower_to_shaft` FIXED + `central_shaft→beacon_carriage` CONTINUOUS 装配 + 侧射 lamp/reflector carriage（Slot C `searchlight_reflector_armlamp` / Slot D shaft）|
| S5 | `rec_lighthouse_with_rotating_beacon_assembly_0003` | `data/records/rec_lighthouse_with_rotating_beacon_assembly_0003/revisions/rev_000001/model.py:L38-L256` | `ExtrudeWithHolesGeometry` 八角 sill ring + `section_loft` roof + 独立 `lantern_room`/`roof_cap`/`pedestal` 三件 FIXED 解构 + 侧臂 lamp+counterweight light_head（Slot B `separate_lantern_stack` / Slot D pedestal / Slot C searchlight）|
| S6 | `rec_lighthouse_with_rotating_beacon_assembly_638131652b3b48dcbb52120a73a346d1` | `.../revisions/rev_000001/model.py:L50-L418` | `_add_member`/`_rpy_for_cylinder` 框架杆 helper + caisson→tower→gallery→lantern→roof→pedestal 6 件 FIXED 堆叠 + 双 lens-drum beacon_head + `pedestal_to_beacon` CONTINUOUS（Slot B `full_fixed_stack` / Slot C dual-drum）|
| S7 | `rec_lighthouse_with_rotating_beacon_assembly_469e7a80595f4d3ba1d6ab756ddb3545` | `.../revisions/rev_000001/model.py:L21-L297` | `_cylinder_between` 桁架杆 helper + 开放钢 lattice 塔（tapered legs + X-brace）+ Torus `spindle_clip` 捕获 spindle 的 bivalve Fresnel beacon（Slot A `skeletal_lattice_truss` / Slot C `bivalve_birdcage` / Slot D direct + captured-pin overlap）|
| S8 | `rec_lighthouse_with_rotating_beacon_assembly_469e7a80595f4d3ba1d6ab756ddb3545` | `.../revisions/rev_000001/model.py:L299-L434` | 独立 `hatch` part(panel+window+hinge leaf+barrel) + `tower_to_hatch` REVOLUTE + captured hinge-barrel `allow_overlap` 写法（Slot E `tower_hatch`）|
| S9 | `rec_lighthouse_with_rotating_beacon_assembly_0cd65f474d3f48c2b38af86156764d1c` | `data/records/rec_lighthouse_with_rotating_beacon_assembly_0cd65f474d3f48c2b38af86156764d1c/revisions/rev_000001/model.py:L77-L432` | monolithic `tower`（塔身+画廊+灯室+roof+beacon_shaft 全 baked visual，Rule-1 正解）+ bearing-roller 骑中央 shaft 的 birdcage 双 lens carriage（Slot B `baked_into_body` / Slot C birdcage / Slot D direct）|
| S10 | `rec_lighthouse_with_rotating_beacon_assembly_0cd65f474d3f48c2b38af86156764d1c` | `.../revisions/rev_000001/model.py:L434-L497` | 独立 `gallery_gate` part(skeletal 杆框) + `gallery_gate_hinge` REVOLUTE 挂画廊栏杆缺口（Slot E `gallery_rail_gate`）|
| S11 | `rec_lighthouse_with_rotating_beacon_assembly_9157e12f458d4d368956181d3e1e19e8` | `data/records/rec_lighthouse_with_rotating_beacon_assembly_9157e12f458d4d368956181d3e1e19e8/revisions/rev_000001/model.py:L21-L298` | 手写 `_prism_mesh`/`_frustum_mesh` 棱柱塔身 + baked pedestal visual + 双向 bar-lamp(cross_arm + 双 lens drum) beacon（Slot A `mesh_prism_polygonal` / Slot C bivalve）|
| S12 | `rec_lighthouse_with_rotating_beacon_assembly_5f19a52186eb453aa9c037ebaad51c89` | `data/records/rec_lighthouse_with_rotating_beacon_assembly_5f19a52186eb453aa9c037ebaad51c89/revisions/rev_000001/model.py:L21-L309` | cadquery `_make_stone_body`/`_make_gallery`/`_make_lantern_frame`/`_make_beacon` union + 弧形 `_make_curved_door`(annular_sector) lantern-wall door + 捕获 `door_hinge_pin` FIXED visual（Slot A `cadquery_union_shell` / Slot E `lantern_wall_door` / captured-pin）|
| S13 | `rec_lighthouse_with_rotating_beacon_assembly_023d0ff399d9437faaafe8fa0cb0baf1` | `data/records/rec_lighthouse_with_rotating_beacon_assembly_023d0ff399d9437faaafe8fa0cb0baf1/revisions/rev_000001/model.py:L44-L380` | `_ring_shell`/`_lens_shell` helper + 高 sleeve `lens_carriage`(carriage_sleeve + lens_crown_ring + 360° Fresnel `_lens_shell`) + 独立 FIXED `central_shaft` + 全高 `lantern_door` REVOLUTE（Slot C `drum_fresnel_omni` / Slot D shaft / Slot E lantern_wall_door）|

补充精读源（用于 Slot C searchlight 与 Slot D pedestal 细节）：
- `rec_..._582b41bf5c9f443bae01d2d06a6e46df/revisions/rev_000001/model.py:L162-L262` — lathe parabolic-reflector + lamp searchlight carriage（Torus 轴承环捕获塔身轴）。
- `rec_..._4767ddac6d464485811dfa251a239fc7/revisions/rev_000001/model.py:L361-L491` — 独立 FIXED `pedestal` 件(base_flange+column+head) + 侧臂 lens_housing+counterweight beacon + 画廊甲板 `trap_door` REVOLUTE。
- `rec_..._25675b1af6b14a538289d6c44fefdd9c/revisions/rev_000001/model.py:L96-L534` — ExtrudeGeometry plinth + section_loft 塔 + lantern_room/roof_cap/pedestal FIXED 解构 + `service_door` REVOLUTE。

## 槽位 + 候选模块表

### Slot A：tower_body（落地静态塔身构造方式）
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `lathe_masonry_shell` | S1 (0001) | `model.py:L130-L266` | eligible if compatible | `LatheGeometry` 旋转锥形砖石塔壳 + 堆叠 Cylinder plinth/cornice ring + 径向窗 + entry block，全 baked visual，最稳健 |
| `mesh_prism_polygonal` | S11 (9157e1) | `model.py:L21-L228` | eligible if compatible | 手写 `MeshGeometry`：`_prism_mesh`/`_frustum_mesh` 多边棱柱塔身 + 八角灯室棱柱 + Cone roof_cap，无 Lathe/cadquery 依赖 |
| `cadquery_union_shell` | S12 (5f19a5) | `model.py:L21-L243` | eligible if compatible | cadquery `_cylinder`/`_frustum`/`_annular_cylinder` union 成 stone_body + gallery + lantern_frame（`mesh_from_cadquery`），实心石工 union 观感 |
| `skeletal_lattice_truss` | S7 (469e7a) | `model.py:L66-L226` | eligible if compatible | `_cylinder_between` 框架杆：tapered_leg + 各层 ring + X-brace 开放钢桁架塔（非实心柱），顶部 box 灯室框 + Cone roof |

### Slot B：lantern_housing（画廊+灯室+顶盖的 part 解构方式，结构骨架驱动）
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `baked_into_body` | S9 (0cd65f) | `model.py:L95-L286` | eligible if compatible | gallery deck/rail、lantern panel/mullion、roof、beacon 承载 shaft 全部作 `tower` 的 named visual（不动→不是 part，Rule 1）；0 个额外 FIXED 件 |
| `separate_lantern_stack` | S5 (0003) | `model.py:L107-L195,L258-L278` | eligible if compatible | 独立 `lantern_room`(八角 Extrude ring + glazing) + `roof_cap`(section_loft) 两件 FIXED 串在 body 上（+1~+2 part / +1~+2 FIXED 关节） |
| `full_fixed_stack` | S6 (638131) | `model.py:L127-L300,L375-L409` | eligible if compatible | `caisson_base`(可选)→`tower`→`gallery`(独立栏杆件)→`lantern_room`→`roof` 多件全 FIXED 堆叠（+3~+4 part / +3~+4 FIXED 关节），最深骨架 |

### Slot C：beacon_optic（旋转 optic 件构造方式 = wow 件）
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `stacked_optic_ring_cage` | S2 (0001) | `model.py:L388-L456` | eligible if compatible | turntable + lower/upper optic ring + 环列 optic_post(8) + 4 片 optic_panel(glass) + 中央 lamp_column + Sphere lamp_core，对称环笼，无明显朝向 |
| `searchlight_reflector_armlamp` | S4 (0002) + 582b41 | `model.py:L221-L292`（0002）；`582b41 model.py:L162-L262` | eligible if compatible | 侧射单束：bearing collar + frame rail/arm + lathe `reflector_shell` 抛物反射镜 + lamp_housing/bulb 朝 +X，旋转时光束扫一圈（带 counterweight 配重的见 4767dd `L405-L432`） |
| `drum_fresnel_omni` | S13 (023d0f) | `model.py:L281-L336` | eligible if compatible | 高 `carriage_sleeve`(ring_shell) + `lens_crown_ring` + 360° `_lens_shell` Fresnel 环带，整圈发光鼓，omni 旋转 |
| `bivalve_birdcage` | S7 (469e7a) + S11 (9157e1) | `model.py:L228-L287`（469e7a）；`9157e1 model.py:L238-L287` | eligible if compatible | 双瓣对射：中央 hub + 两侧对称 Fresnel 透镜板/lens drum + 杆架 birdcage（469e7a 用 Torus clip/hub + Box lens；9157e1 用 cross_arm + 双 lens drum） |

### Slot D：beacon_bearing_topology（beacon 的轴承/承载 part 拓扑）
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `direct_to_body` | S9 (0cd65f) | `model.py:L276-L281,L474-L482` | eligible if compatible | beacon 直接 parent=body（或 lantern 件）；承载 spindle/pedestal/shaft 是 body 的 baked Cylinder visual；beacon 轴承环/roller 骑住它（469e7a/9157e1/5f19a5 同型） |
| `separate_fixed_pedestal_part` | S5 (0003) + 4767dd | `model.py:L166-L195,L272-L287`（0003）；`4767dd model.py:L361-L484` | eligible if compatible | 独立 `pedestal` 件(base_flange + column + bearing cap)经 `*_to_pedestal` FIXED 串到 body/roof，beacon parent=pedestal（25675b/638131 同型） |
| `separate_fixed_shaft_part` | S3+S4 (0002) + S13 (023d0f) | `model.py:L202-L309`（0002）；`023d0f model.py:L250-L280` | eligible if compatible | 独立细 `central_shaft` 件(shaft_base + shaft_main)经 `tower_to_shaft` FIXED 串到 body，beacon carriage 的 sleeve/collar 抱住 shaft 旋转（279fff/63fc7f 同型） |

### Slot E：access_opening（可选 gated 第二 REVOLUTE DOF）
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| `none` | S1 (0001) / S6 (638131) | `model.py:L457-L473`（0001，仅 beacon 关节） | eligible if compatible | 无开口件，纯 body+beacon（±lantern stack），2-part 极简或多件 FIXED stack；7 个 `(2,0,0,1)` + 数个 stack 样本属此 |
| `lantern_wall_door` | S12 (5f19a5) + S13 (023d0f) | `model.py:L275-L306`（5f19a5）；`023d0f model.py:L337-L380` | eligible if compatible | 灯室壁/塔壁上一扇 REVOLUTE 检修门，axis `(0,0,±1)`，0→~1.35 rad；5f19a5 用弧形 annular_sector 门 + 捕获 FIXED hinge_pin，023d0f 用全高 Box 门 |
| `gallery_rail_gate` | S10 (0cd65f) + a02180 | `model.py:L434-L497`（0cd65f）；`a02180 model.py:L274-L320` | eligible if compatible | 画廊栏杆缺口处一扇 REVOLUTE 栏杆门(skeletal 杆框)，绕竖直轴外/内开，0→~1.35 rad（12f0c4/3ebd11/eea009 同型） |
| `tower_hatch` | S8 (469e7a) | `model.py:L299-L339` | eligible if compatible | 塔壁/灯室壁上一块 REVOLUTE hatch 盖板(panel + window + hinge barrel 捕获 jamb 上 FIXED pin)，0→~1.65 rad（1725c0/b7466f 同型） |
| `gallery_trap_door` | 4767dd | `4767dd model.py:L434-L491` | eligible if compatible | 画廊甲板上一片 REVOLUTE trap door(gate_leaf + 双 hinge + latch)，绕竖直轴掀开，0→~1.35 rad（fdfab4 同型） |

> Slot E 候选 = 5，互斥单选；procedural sampler 可在兼容条件下采样 `none`（最稳健的纯身份骨架）。`none` 与四种开口件分别对应「不增/增一个 REVOLUTE 件」的结构骨架分叉，全部有独立 5 星来源、结构互不相同（无门 / 弧形壁门 / 栏杆门 / hatch 盖 / 甲板掀门），不存在退到更少候选的情况。

## 槽位图（slot graph）
pattern = **mixed**（静态 body 身份链 + 1 个 CONTINUOUS beacon 子件 + lantern 0..N 个 FIXED 解构件 + 可选 1 个 gated REVOLUTE 开口件）

```
[Slot A tower_body]  ← 落地 root（lathe / mesh-prism / cadquery / lattice-truss）
        |
        |  (Slot B lantern_housing 决定塔顶 gallery/lantern/roof 是 baked visual 还是拆成 FIXED 子件)
        |     baked_into_body:        0 个额外件（全 named visual）
        |     separate_lantern_stack: --(FIXED)--> [lantern_room] --(FIXED)--> [roof_cap]
        |     full_fixed_stack:       --(FIXED)--> [gallery] --(FIXED)--> [lantern_room] --(FIXED)--> [roof] (±caisson_base 在 body 下)
        |
        |  (Slot D beacon_bearing_topology 决定 beacon 的 parent 与是否多一个承载件)
        |     direct_to_body:               beacon.parent = body/lantern 件（承载 spindle 是 baked visual）
        |     separate_fixed_pedestal_part: --(FIXED)--> [pedestal] ; beacon.parent = pedestal
        |     separate_fixed_shaft_part:    --(FIXED)--> [central_shaft] ; beacon.parent = central_shaft
        |
        +----(CONTINUOUS, axis (0,0,±1), 无限位)----> [Slot C beacon_optic]   ← 唯一主运动 / wow
        |        stacked_optic_ring_cage | searchlight_reflector_armlamp | drum_fresnel_omni | bivalve_birdcage
        |
        +----(可选 REVOLUTE, axis (0,0,±1), [0, ~1.05~1.92])----> [Slot E access_opening 件]
                 （仅 Slot E≠none 时多一个 REVOLUTE 件；parent = body 或 lantern 件）
```
- 身份恒为：静态塔身 + 1 个 `*_to_beacon` CONTINUOUS(Z) 旋转 optic。
- Slot B 决定塔顶解构出 0 / 2 / 3~4 个 FIXED 子件；Slot D 决定 beacon 之上是否多 1 个 FIXED 承载件（pedestal/shaft）及 beacon 的 parent；Slot E 决定是否多 1 个 REVOLUTE 开口件。三者都是「增/减 part+joint」的结构骨架开关。

## 部件（Parts）

### Slot A / tower_body
| part | visual_count | 描述 | 来源 |
|---|---|---|---|
| `tower` / `lighthouse` / `tower_base` | ~10-40 | 落地静态塔身（lathe/mesh/cadquery/lattice）+ baked 画廊/窗/entry/承载 spindle visual；落地 root | S1 / S7 / S9 / S11 / S12 |

### Slot B / lantern_housing
| part | visual_count | 描述 | 来源 |
|---|---|---|---|
| （baked_into_body）无额外 part | 0 | gallery/lantern/roof 全是 body 的 named visual | S9 |
| `lantern_room` | ~5-18 | 八角/圆 灯室罩（sill ring + mullion + glazing panel），FIXED 在 body 上 | S5 / S6 |
| `roof_cap` / `roof` | ~3-5 | section_loft/Cone 顶盖 + vent stack/cap，FIXED 在 lantern 上 | S5 / S6 |
| `gallery` | ~6-30 | 独立画廊栏杆件(deck + rail Torus + posts + brace)，FIXED 在 body 上（仅 full_fixed_stack）| S6 |
| `caisson_base` | ~4 | 可选离岸基墩(caisson + fender ring + service deck)，FIXED 在 body 下（仅 full_fixed_stack）| S6 |

### Slot C / beacon_optic（旋转件）
| part | visual_count | 描述 | 来源 |
|---|---|---|---|
| `beacon` / `beacon_carriage` / `beacon_head` / `light_head` / `lens_carriage` | ~6-30 | 唯一 CONTINUOUS 旋转 optic：环笼 / 侧射反射镜 / Fresnel 鼓 / 双瓣 birdcage | S2 / S4 / S13 / S7 / S11 |

### Slot D / beacon_bearing_topology
| part | visual_count | 描述 | 来源 |
|---|---|---|---|
| （direct_to_body）无额外 part | 0 | 承载 spindle/pedestal 是 body baked visual，beacon 直挂 body | S9 |
| `pedestal` | ~4-6 | 独立 FIXED 轴承立柱(base_flange + column + bearing cap)，beacon parent | S5 / 4767dd |
| `central_shaft` | ~2-3 | 独立 FIXED 细竖轴(shaft_base + shaft_main)，beacon sleeve 抱住旋转 | S3 / S4 / S13 |

### Slot E / access_opening（可选 REVOLUTE 件）
| part | visual_count | 描述 | 来源 |
|---|---|---|---|
| `door` / `maintenance_door` / `service_door` / `lantern_door` | ~3-6 | 灯室/塔壁 REVOLUTE 检修门(panel + jamb/hinge barrel) | S12 / S13 / S5 |
| `gallery_gate` / `access_gate` | ~8-14 | 画廊栏杆 REVOLUTE 门(skeletal 杆框 + hinge barrel) | S10 |
| `hatch` / `access_hatch` / `access_panel` | ~4-6 | 塔壁 REVOLUTE hatch 盖(panel + window + hinge barrel 捕获 jamb pin) | S8 |
| `trap_door` | ~4 | 画廊甲板 REVOLUTE 掀门(gate_leaf + 双 hinge + latch) | 4767dd |

## 关节（Joints）
| 关节 | 类型 | parent_slot.part | child_slot.part | axis | range | 描述 | 来源 |
|---|---|---|---|---|---|---|---|
| `*_to_beacon` / `beacon_rotation` / `beacon_spin` / `pedestal_to_beacon` / `lens_spin` | continuous | A.body 或 B.lantern_room 或 D.pedestal/central_shaft | C.beacon | `(0,0,1)` 或 `(0,0,-1)` | unlimited（无 lower/upper） | **主关节**：optic 绕竖直轴持续自旋扫光；34/34 都有且仅此一个 CONTINUOUS | S2 / S4 / S6 / S9 / S13 |
| `tower_to_lantern` / `gallery_to_lantern` / `lantern_to_roof` / `tower_to_gallery` / `caisson_to_tower` | fixed | A.body 或 B.gallery/lantern | B.lantern_room/roof/gallery/caisson | n/a | n/a | 仅 Slot B≠baked：把画廊/灯室/顶盖/基墩拆成 FIXED 子件堆叠 | S5 / S6 |
| `tower_to_pedestal` / `*_to_pedestal` | fixed | A.body 或 B.roof | D.pedestal | n/a | n/a | 仅 Slot D=`separate_fixed_pedestal_part`：独立轴承立柱固定 | S5 / 4767dd |
| `tower_to_shaft` / `body_to_shaft` / `structure_to_shaft` | fixed | A.body 或 B.lantern_room | D.central_shaft | n/a | n/a | 仅 Slot D=`separate_fixed_shaft_part`：独立竖轴固定 | S3 / S4 / S13 |
| `*_door_hinge` / `door_hinge` / `wall_to_door` / `frame_to_door` | revolute (gated) | A.body 或 B.lantern_room | E.door | `(0,0,±1)` | `[0, 1.35]` | 仅 Slot E=`lantern_wall_door`：检修门绕竖直轴外开 | S12 / S13 |
| `gallery_gate_hinge` / `rail_to_gate` / `tower_to_gate` | revolute (gated) | A.body 或 B.gallery | E.gallery_gate | `(0,0,±1)` | `[0, 1.35]` | 仅 Slot E=`gallery_rail_gate`：栏杆门绕竖直轴开 | S10 |
| `hatch_hinge` / `tower_to_hatch` / `panel_hinge` | revolute (gated) | A.body 或 B.lantern_room | E.hatch | `(0,0,±1)` | `[0, 1.65]` | 仅 Slot E=`tower_hatch`：塔壁 hatch 盖外开 | S8 |
| `gallery_trap_hinge` / `rail_to_trap_door` | revolute (gated) | A.body | E.trap_door | `(0,0,±1)` | `[0, 1.35]` | 仅 Slot E=`gallery_trap_door`：甲板掀门绕竖直轴掀开 | 4767dd |

## 每槽位 Module Emits / Interfaces

本 spec 是 modular v1 早期写法；每个 slot/module 的 emitted parts、internal joints、upstream/downstream interface 已在「槽位 + 候选模块表」「槽位图」「部件（Parts）」「关节（Joints）」和 adopted source index 中给出。模板实现阶段必须把这些信息逐 module 显式落成 `ModuleBuild`、`InterfaceSpec` 和 `MatingContract`，不能只按全局部件清单拼装。

| emits | 描述 | 来源 |
|---|---|---|
| parts / visuals | 以各 slot candidate 的结构特征和「部件（Parts）」表为准；不动装饰优先作为 parent visual | adopted source index + slot table |
| internal joints | 以「关节（Joints）」表和 slot graph 中的 joint type / axis / range 为准 | adopted source index + joints table |
| upstream interface | 来自 slot graph 中的 parent face、hinge line、socket、rail、axis、contact plane 或 support policy | slot graph + source snippets |
| downstream interface | 消费相邻 slot 的 mating point / axis / face；必须在实现中转成真实 `InterfaceSpec` | slot graph + source snippets |
| mating contracts | 每个 separate moving child 和跨 slot 连接必须有可见支撑路径；captured pin / shaft / bearing overlap 需要局部 allow-overlap 理由 | validator + reject cases |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 默认值 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| `tower_body_style` | enum | `lathe_masonry_shell` / `mesh_prism_polygonal` / `cadquery_union_shell` / `skeletal_lattice_truss` | `lathe_masonry_shell` | 决定 Slot A 塔身构造 | S1 / S11 / S12 / S7 |
| `lantern_housing_style` | enum | `baked_into_body` / `separate_lantern_stack` / `full_fixed_stack` | `baked_into_body` | 决定 Slot B 拆出 0 / 2 / 3~4 个 FIXED 件 | S9 / S5 / S6 |
| `beacon_optic_style` | enum | `stacked_optic_ring_cage` / `searchlight_reflector_armlamp` / `drum_fresnel_omni` / `bivalve_birdcage` | `stacked_optic_ring_cage` | 决定 Slot C 旋转件构造 | S2 / S4 / S13 / S7 |
| `beacon_bearing_topology` | enum | `direct_to_body` / `separate_fixed_pedestal_part` / `separate_fixed_shaft_part` | `direct_to_body` | 决定 Slot D：是否多 1 个 FIXED 承载件 + beacon parent | S9 / S5 / S3 |
| `access_opening` | enum | `none` / `lantern_wall_door` / `gallery_rail_gate` / `tower_hatch` / `gallery_trap_door` | `none` | 决定 Slot E：是否多 1 个 REVOLUTE 开口件 | S1 / S12 / S10 / S8 / 4767dd |
| `has_caisson_base` | bool | derived（仅 `full_fixed_stack` 下可 True）| false | full_fixed_stack 时可在 body 下加离岸基墩 | S6 |
| `tower_height` | float | `1.0 – 33.0`（量级跨度大：squat 桌面 ~1m → 离岸 ~33m）| sampled | beacon_z ≈ tower_height + gallery/lantern 偏置 | S1 / S6 / S9 |
| `tower_base_radius` | float | `0.2 – 4.85` | sampled | 向上收锥，top_radius < base_radius | S1 / S6 / S9 |
| `tower_top_radius` | float | `top < base`（典型 0.5×–0.85× base）| sampled | lantern_radius < tower_top_radius | S1 / S5 / S9 |
| `lantern_radius` | float | `0.4 – 2.2` | sampled | beacon optic 直径 < 2×lantern_radius（必须罩得住）| S5 / S6 / S13 |
| `beacon_optic_radius` | float | derived | derived | `< lantern_radius − clearance`，保证旋转任意姿态在灯室内 | S2 / S9 |
| `lantern_sides` | int/enum | `8`（八角，众数）/ 圆 / `10`/`12`（少数）| 8 | 灯室 mullion/pane 数；多为正八边形 | S3 / S9 / 0cd65f |
| `beacon_axis_sign` | enum | `(0,0,1)` / `(0,0,-1)` | `(0,0,1)` | beacon 自旋轴朝向（两者皆出现，等价）| 全样本 |
| `door_open_upper` | float | `1.05 – 1.92` rad（实测全距；众数 1.35）| 1.35 | 仅 Slot E≠none；REVOLUTE 开口件开角上限 | 全 22 个开口件实测 / S8 / S12 |

## Multiplicity / Copy Logic

- 无模板级复制数量逻辑：核心结构由固定 named slots 和 gated module-local fixed copies 表达，不暴露全局 `*_count` 作为主拓扑采样轴。
- 若某个 module source 内部包含固定成对、环形阵列或若干重复 visual/part，模板实现应把它保留为 module-local construction，并使用稳定命名；不得把未审核的可变复制数量加入 seed domain。
- 未来若要把固定重复结构升级为可变 multiplicity，必须补充来源、N_range、placement、joint policy、regression seeds 和 reviewer 审核记录。

## 拓扑多样性审计

总组合数：`Slot A × Slot B × Slot C × Slot D × Slot E`
= `4 × 3 × 4 × 3 × 5` = **720**。
（`has_caisson_base` 仅在 full_fixed_stack 下二值再分叉，未计入下界。）


仅看「增/减 part 或 joint」的结构骨架（不看几何风格）就已远超门控：
- Slot B：`baked`(+0 part/+0 FIXED) / `separate_lantern_stack`(+2 part/+2 FIXED) / `full_fixed_stack`(+3~4 part/+3~4 FIXED) = 3 种骨架
- Slot D：`direct`(+0 part) / `pedestal`(+1 part/+1 FIXED) / `shaft`(+1 part/+1 FIXED，但 parent 关系不同) = 3 种骨架
- Slot E：`none`(+0 REVOLUTE) / 4 种开口件(+1 part/+1 REVOLUTE) = 至少 2 种骨架（含 vs 不含第二 DOF）

仅 B×D×(E 二值) = 3 × 3 × 2 = **18 种 part/joint 拓扑骨架**（part 数从 1（baked+direct+none）到 ~8（full_fixed_stack+caisson+pedestal+door），关节数从 1（仅 beacon CONTINUOUS）到 ~7），远超 10；叠加 Slot A 塔身构造与 Slot C optic 构造两个几何槽后离散组合达 720。

每槽位候选数：
| slot | candidate_count | 是否 ≥3 | 备注 |
|---|---|---|---|
| Slot A tower_body | 4 | yes | lathe / mesh-prism / cadquery / lattice-truss 四套互不相同的塔身构造 |
| Slot B lantern_housing | 3 | yes | baked / 2 件 stack / 3~4 件 stack |
| Slot C beacon_optic | 4 | yes | 环笼 / 侧射反射镜 / Fresnel 鼓 / 双瓣 birdcage |
| Slot D beacon_bearing_topology | 3 | yes | direct / pedestal / shaft |
| Slot E access_opening | 5 | yes | none / 壁门 / 栏杆门 / hatch / 甲板掀门 |

### Procedural Sampling / Sweep Plan

seed_domain_policy：procedural_first。`config_from_seed(seed)` 对普通 seed 使用 deterministic procedural sampling；`seed=0` 不特殊，不作为 anchor 或 reference seed。Sampling 先选择上游结构槽，再从 compatible 下游 candidate 集合中选择 module / multiplicity / module-local variant。

Compatibility matrix / gating：以「槽位图」「每槽位 Module Emits / Interfaces」「Validator」中定义的接口、joint 轴、支撑面、range 和互斥关系为准；不兼容组合必须在 sampler 或 `resolve_config` 中降级、重采样或拒绝，不能让 builder 后期失败。

Regression overrides：默认无。若未来 sweep 发现稳定失败组合，或 reviewer 指定固定回归样本，可以添加少量显式 regression seed，但必须写明 seed、组合和原因；不得用小型 curated / modulo 表作为主 seed domain。

Random sweep / viewer plan：首次模板验收跑 `uv run articraft template sweep-pipeline <slug>`，依赖 0、0-4、0-19、0-49 的 cumulative random seeds 检查 build、MatingContract、joint origin / axis / range、support、collision 和 。机械通过后 viewer 目检一小批随机 seed，重点看类别身份、比例、closed pose、bulky module、optional moving child、max multiplicity、captured-pin / bearing / hinge / rail 接口。


## Validator（author_run_tests 必须覆盖的点）
- identity：存在 1 个落地 body + 恰好 1 个 beacon；存在恰好 1 个 `*_to_beacon` CONTINUOUS 关节，axis = `(0,0,±1)`，无 lower/upper（无限位）。
- beacon axis：beacon 自旋轴必为竖直 `(0,0,±1)`（不是水平 X/Y）；做成 REVOLUTE 限位或 PRISMATIC 即 fail。
- beacon rotation effect：在 `beacon=0 / π/2 / π / 3π/2` 采样姿态下，optic 上一个朝向特征 visual（lens/reflector/optic_panel）的世界 XY 中心绕轴旋转（半径不变、角度变化），证明确实在转而非平移；turntable 中心 XY/Z 不动（dx,dy,dz < 1e-5）。
- lantern containment：所有 beacon 姿态下 optic 在 XY 投影落在 lantern footprint 内（`expect_within(beacon, lantern/body, axes="xy", outer_elem=lantern_floor/glass)`），不戳出灯室罩。
- chain topology：Slot B≠baked 时 lantern_room/roof/gallery 必经 FIXED 串在 body 侧、不跳父；Slot D=pedestal/shaft 时存在对应 FIXED `*_to_pedestal`/`tower_to_shaft` 且 beacon.parent 正是该承载件。
- bearing mating：Slot D=shaft 时 beacon 的 sleeve/collar 必与 central_shaft 接触（captured，用 `allow_overlap` 声明 Rule-2 grandfather，见 S3/S7/S12）；Slot D=pedestal 时 beacon turntable/bearing_disk 坐在 pedestal bearing cap 上（`expect_contact`）。
- door（Slot E≠none）：开口件为 REVOLUTE，axis `(0,0,±1)`，lower≈0，upper∈[1.0,1.95]（覆盖实测 22 个开口件 1.05–1.92 全距，勿写死成更窄的 [1.35,1.65]——会误拒 5/22 个真 5 星）；闭合位贴 body/lantern 壁，开到 upper 时门 AABB 明显外摆（≥0.05~0.10m）；hatch/door 的 hinge barrel 捕获 jamb pin 用 `allow_overlap` 声明。
- grounding：body 落地（min z ≈ 0）；beacon 在塔顶 lantern 内（beacon_z 显著高于 body 中段）。
- no floating：lantern_room/roof/gallery/caisson/pedestal/shaft/door/gate/hatch/trap 全部 attached 或 FIXED；无悬空件、无 1-3mm 接口盘漂浮装饰（Rule 1/2）。

## Reject cases（必须能识别并拒绝）
- 缺少旋转件（纯静态塔），或 beacon 不是 CONTINUOUS（被做成 FIXED / REVOLUTE 限位 / PRISMATIC）。
- beacon 自旋轴不是竖直 `(0,0,±1)`（如做成水平 X 自旋 → 变成风车/螺旋桨）。
- beacon optic 戳出灯室罩 / 飞在灯室外 / 不在塔顶（origin 在塔身中段或地面）。
- Slot B 声称 `separate_lantern_stack` 却把 lantern_room 直接当 body visual（或反之），part/FIXED 关系自相矛盾。
- Slot D=`separate_fixed_shaft_part`/`pedestal` 声称独立承载件却把 beacon 直挂 body（或 direct_to_body 却多出一个游离 shaft 件），bearing parent 关系自相矛盾。
- beacon 与 shaft/pedestal 之间留几 mm 空气（mating gap），或 beacon 靠一个看不见的 1-3mm 接口盘锚定（Rule 2 phantom anchor）。
- 把不旋转的画廊/灯室/顶盖装饰拆成独立 FIXED part 挂在 1-3mm 接口盘上（违反 Rule 1，应作 body named visual）。
- 开口件（door/gate/hatch/trap）做成 FIXED 装饰而非 REVOLUTE，或绕水平轴（lighthouse 开口件恒绕竖直轴）。
- 把风机 / 探照灯三脚架 / 旋转餐厅 / 静态方尖碑硬塞进本模板。

## 与相邻类别的边界
| 相邻类别 | 区别 |
|---|---|
| `wind_turbine` / 风车 / 螺旋桨 | 风机旋转件绕**水平 X** 轴 + 有叶片阵列 + 机舱偏航；lighthouse 旋转 optic 绕**竖直 Z** 轴、被灯室罩住、无叶片 |
| `searchlight` / `spotlight` / 探照灯云台 | 探照灯是裸露在三脚架/云台上的灯头（常 pan-tilt 两 DOF）；lighthouse 必有完整塔身 + gallery + lantern 灯室玻璃罩，且仅 1 个竖直 CONTINUOUS DOF |
| `radar_antenna` / 旋转雷达 | 雷达绕竖直轴旋转但旋转件是平板/抛物天线、装在桅杆/平台上，无 lantern 灯室罩与 Fresnel optic 语义 |
| `obelisk` / `monument_column` / 静态塔标 | 无任何旋转件；lighthouse 必有 CONTINUOUS beacon |
| `revolving_tower_restaurant` / 旋转餐厅 | 旋转的是整层楼面；lighthouse 旋转的只是顶部小 optic 灯头，塔身静止 |
| `carousel` / 旋转木马 | 旋转件在底部承载乘客、绕竖直轴；lighthouse 旋转件在顶部 lantern 内、是发光 optic，塔身高耸 |

## 模板实现备注（可选）
- Slot A 四套塔身共享 `_polar`（S1）、`_add_member`/`_rpy_for_cylinder`（S6/S7 框架杆）、`_save_mesh`/`mesh_from_geometry` 包装；cadquery 路径用 `mesh_from_cadquery`，若环境无 cadquery 应在实现前与用户确认而非降级 Box（Rule 3）。
- Slot C 四种 optic 的「朝向特征 visual」命名要统一（如 `optic_panel_*` / `reflector_shell` / `fresnel_lens` / `lens_front`），便于 validator 用统一 `expect_within` / 旋转检查。
- Slot D=`separate_fixed_shaft_part` 的 beacon sleeve 抱 shaft、`direct_to_body` 的 spindle-clip 抱 baked spindle、Slot E hatch/door 的 hinge barrel 抱 jamb pin —— 这三类 captured-pin 重叠都必须在 `run_..._tests` 里用 `ctx.allow_overlap(..., reason=...)` 声明（Rule 2 grandfather，见 S3/S7/S8/S12）。
- beacon CONTINUOUS 关节恒 `MotionLimits(effort, velocity)` 无 lower/upper；axis 两个符号都合法但同一模型内 beacon 与（若有）开口件可不同轴向。
- Slot B/D/E 的 FIXED/REVOLUTE 子件每个都要带 `MatingContract` 锚到真实 body/lantern 面（Rule 2），不要用 1-3mm 接口盘。
