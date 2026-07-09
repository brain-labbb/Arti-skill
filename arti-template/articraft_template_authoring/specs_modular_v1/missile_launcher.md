# Missile Launcher Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `missile_launcher` |
| template path | `agent/templates/missile_launcher.py` |
| test path | `tests/agent/test_missile_launcher_template.py` |
| stage | `REVIEWED` |
| status | `approved` |
| __modular__ | `True` |
| pattern | `linear_chain` |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 23 |
| read_count | 23 |
| read_scope | all 5-star samples in this category: `model.py` / `revision.json` / `record.json` / `prompt.txt` were read |
| samples_adopted_as_module_sources | 6 |
| samples_read_but_not_adopted | 17 |
| source_index_policy | only adopted reusable snippets are indexed below |

全量阅读后的结构族分布：

| 结构族 | 样本数 | 说明 |
|---|---:|---|
| pedestal/ground-column -> turntable -> cradle -> box/canister 发射箱 | 8 | base(plinth+column) -> turntable(slew) -> cradle -> launch_pod；yaw(Z) + pitch(-Y)，pod 多为 FIXED |
| vehicle/truck-bed/trailer chassis -> slew turret -> elevating tube bank | 7 | 车架(rails+crossmembers+wheels) -> slew platform -> tube bank(mesh hollow tubes) |
| naval deck pedestal -> yaw carriage -> twin launch-rail frame | 3 | 高 pedestal -> yaw carriage -> rail frame（4-5m 大尺度双轨） |
| tripod/man-portable -> yaw head -> single/twin tube | 4 | 三脚架(cylinder-between legs) -> yaw head -> Lathe/CadQuery 中空发射筒 |
| column/base-ring -> yoke[FIXED] -> canister pack[pitch] | 5 | base ring 上 2-arm fork，canister pack 在 fork 间俯仰 |
| launch pod / tube 构造变体 | 23 | box-cell pod / square-tube canister / round hollow tube bank / twin rail / single tube；count 1/2/4/6/12 |

被采纳样本逐条标注：
- `rec_missile_launcher_0002` — adopted：textbook pedestal launcher，4-part（base/turntable/cradle/launch_pod），全 primitive，参数化 cell-door loop，yaw CONTINUOUS Z + pitch REVOLUTE -Y + pod FIXED。
- `rec_missile_launcher_c51786adcb394b139d90df0a3c886360` — adopted：pedestal + CadQuery 2×2 中空 canister pod，FIXED pod，紧凑。
- `rec_missile_launcher_0003` — adopted：truck-bed -> slew -> tube bank，参数化 3×4 tube grid（Extrude/Cylinder mesh）。
- `rec_missile_launcher_474f263480424a43928f6a45710c2402` — adopted：大型 box-cell launch pod，完整参数化 walls/dividers/cells；pitch axis `(1,0,0)`。
- `rec_missile_launcher_91c8ac060ad846448a2f80737cd3f3d4` — adopted：Box-walled 方筒 canister pack + column->ring->yoke[FIXED]->pack 链。
- `rec_missile_launcher_5b84e0868298446393212fdaf29a1a2d` — adopted：tripod man-portable + `_cylinder_between` 撑杆 helper + Lathe 中空发射筒。
- Remaining 17 samples — read but not adopted：同一 root -> yaw -> pitch cradle 链 + launcher 变体（blast door hinge / reload cover / outriggers / handwheel / sight）已被上述模块覆盖。

## 核心身份
`missile_launcher` 是一个可旋回俯仰的导弹发射器：静态 root（pedestal / vehicle bed / trailer / naval deck / tripod）承载一个绕竖直 Z 轴旋回（azimuth/yaw）的 turntable，再通过 trunnion 抱住一个绕水平轴俯仰（elevation/pitch）的 cradle，cradle 上是 launch pod / 发射筒簇 / 发射箱 / 双轨（FIXED child 或直接 welded 进 cradle）。发射筒/箱是类别身份件。默认成熟域是 two-DOF yaw/pitch 链；blast door、reload cover、folding outrigger、handwheel、sight 都是可选辅助。语料中无 missile-eject prismatic 关节。

边界：
- 不包括固定不动的发射筒/导弹：必须有 yaw + pitch articulation。
- 不混入 remote weapon station：身份件是导弹发射筒/发射箱簇，不是带 barrel + optics 的机枪。
- 不混入 cannon/artillery：是导弹箱/筒，不是炮管炮口。
- 不要发明 missile-eject prismatic 关节（语料 0/23 出现）。

## 采用源码索引（Adopted Source Index）
| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| S1 | `rec_missile_launcher_0002` | `data/records/rec_missile_launcher_0002/revisions/rev_000001/model.py:L33-L234` | canonical 4-part pedestal：base L33-L62、turntable L64-L99、cradle L101-L154、pod(含 cell-door double loop L187-L198)L156-L203、yaw/pitch/FIXED joints L205-L234 |
| S2 | `rec_missile_launcher_c51786adcb394b139d90df0a3c886360` | `data/records/rec_missile_launcher_c51786adcb394b139d90df0a3c886360/revisions/rev_000001/model.py:L20-L228` | CadQuery 中空 canister pod helper L20-L68、base L80-L111、turntable(yoke plates + bearing bosses)L113-L144、cradle L146-L177、pod part L179-L197、joints L199-L228 |
| S3 | `rec_missile_launcher_0003` | `data/records/rec_missile_launcher_0003/revisions/rev_000001/model.py:L70-L272` | truck-bed base_frame(4-foot loop L113-L124)L70-L130、slew_platform L132-L186、launcher_bank(tube grid double-loop L213-L220)L188-L248、joints L250-L272 |
| S4 | `rec_missile_launcher_474f263480424a43928f6a45710c2402` | `data/records/rec_missile_launcher_474f263480424a43928f6a45710c2402/revisions/rev_000001/model.py:L64-L408` | base L64-L103、turntable L105-L158、cradle L160-L230、参数化 box-cell pod(cell loop L358-L366)L232-L372、joints(pitch axis (1,0,0))L374-L408 |
| S5 | `rec_missile_launcher_91c8ac060ad846448a2f80737cd3f3d4` | `data/records/rec_missile_launcher_91c8ac060ad846448a2f80737cd3f3d4/revisions/rev_000001/model.py:L25-L263` | support_column L25-L52、base_ring(8 bolt loop)L54-L82、yoke(2 arms+bearings)L84-L133、Box-walled 方筒 canister_pack(4-tube loop L146-L178)L135-L237、joints(CONT azimuth + FIXED yoke + REV pitch)L239-L263 |
| S6 | `rec_missile_launcher_5b84e0868298446393212fdaf29a1a2d` | `data/records/rec_missile_launcher_5b84e0868298446393212fdaf29a1a2d/revisions/rev_000001/model.py:L72-L344` | tripod_base(leg loop via `_add_member` L112-L139)L72-L139、yaw_head L141-L196、Lathe 中空发射筒(profiles L198-L223)L198-L320、joints L322-L344 |

## 槽位 + 候选模块表

### Slot A：root_base
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `pedestal` | S1 | `model.py:L33-L62` | **yes** | plinth + 圆柱 column + top plate + service box |
| `ground_column` | S5 | `model.py:L25-L52` | | ground plate + 立柱 + top bearing + floor bolts |
| `vehicle_truck_bed` | S3 | `model.py:L70-L130` | | floor plate + rails + crossmembers + 4 feet/wheels + lower ring |
| `tripod` | S6 | `model.py:L72-L139` | | 3 撑腿 via `_add_member` + apex housing + yaw seat |
| `naval_deck` | S1 | `model.py:L33-L62`（放大尺度） | | 高 pedestal/甲板座（4-5m 尺度，twin rail 用） |

### Slot B：yaw_turntable
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `slew_turntable` | S1 | `model.py:L64-L99` | **yes** | slew ring + drive housing + 2 supports + deck plate |
| `yoke_fork_turntable` | S5 | `model.py:L54-L133` | | base ring + 2-arm fork + bearing bosses（捕获 cradle 销） |
| `truck_slew_platform` | S3 | `model.py:L132-L186` | | upper ring + deck + cheeks + trunnion sleeves |
| `yaw_head_compact` | S6 | `model.py:L141-L196` | | tripod 上的小 yaw head |

### Slot C：elevation_cradle + launcher
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `cradle_plus_box_pod` | S1 | `model.py:L101-L203` | **yes** | trunnion cradle + FIXED box-cell pod（cell-door loop） |
| `cradle_plus_canister_cad` | S2 | `model.py:L146-L197` | | cradle + CadQuery 中空 canister pod |
| `tube_bank_welded` | S3 | `model.py:L188-L248` | | tube grid 直接 welded 进 elevating bank |
| `box_cell_pod_param` | S4 | `model.py:L160-L372` | | 完整参数化 box-cell pod（walls/dividers/cells） |
| `canister_pack_boxwall` | S5 | `model.py:L135-L237` | | Box-walled 方筒 canister pack（fork 间俯仰） |
| `single_tube_lathe` | S6 | `model.py:L198-L320` | | Lathe 中空单/双发射筒 + 鞍座 + 卡箍 |

### Slot D：auxiliary_mechanism
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `none` | S1 | `model.py:L205-L234` | **yes** | 只有 yaw + pitch + FIXED pod |
| `blast_doors` | S3 | `model.py:L188-L248`（前 aperture 边）| | 左右 blast cover REVOLUTE 铰链 |
| `reload_cover` | S1 | `model.py:L156-L203`（box 顶后边）| | 顶后 reload cover REVOLUTE |
| `outriggers` | S3 | `model.py:L113-L124`（chassis 侧）| | trailer/车架折叠 outrigger 稳定腿 REVOLUTE |
| `handwheel_or_sight` | S6 | `model.py:L141-L196` | | 装饰 handwheel CONTINUOUS 或 sight module |

## 槽位图（slot graph）
pattern = `linear_chain`

```
[Slot A root_base]
      -- yaw CONTINUOUS/REVOLUTE, axis (0,0,1) -->
[Slot B yaw_turntable]   (yoke 变体可在此 FIXED 出 fork)
      -- pitch REVOLUTE, axis (0,-1,0)  [474: (1,0,0)] -->
[Slot C elevation_cradle]
      -- FIXED (box/canister pod) 或 welded tubes -->
[Slot C launch_pod / tubes / rails]
      -- optional REVOLUTE blast door / reload cover / outrigger -->
[Slot D auxiliary]
```

主约束基准是 yaw vertical axis 与 pitch trunnion line。tube count/grid、cradle 长度、pod origin、blast door 铰链都从这两条基准与 launcher_style 派生。

## 部件（Parts）

### Slot A / root_base
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `base` / `support_column` / `base_frame` / `tripod` / `deck_base` | ~4-30 | 地面/车载/三脚架/甲板静态根，承载下 slew race | S1/S3/S5/S6 |

### Slot B / yaw_turntable
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `turntable` / `slew_platform` / `yaw_head` / `base_ring`(+`yoke`) | ~5-15 | 旋回台 + 上 slew ring + trunnion bushings/fork cheeks | S1/S3/S5/S6 |

### Slot C / elevation_cradle + launcher
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `cradle` / `launcher_bank` / `canister_pack` | ~6-40 | trunnion cradle 或 elevating bank；含 trunnion shaft + collars | S1/S3/S4/S5 |
| `launch_pod` / `tube_*` / `canister_*` / `rail_*` | embedded/独立 | 发射箱/筒簇/双轨（FIXED child 或 welded visuals），category-identity | S1/S2/S3/S4/S5/S6 |

### Slot D / auxiliary_mechanism
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `blast_cover` / `reload_cover` | ~4-7 | 发射口防爆门/再装填盖（REVOLUTE） | S3 / S1 |
| `outrigger_*` | ~4 each | 车架/拖车折叠稳定腿（REVOLUTE） | S3 |
| `sight_module` / `handwheel` | ~2-6 | 可选瞄准模组（FIXED）或装饰手轮（CONTINUOUS） | S6 |

发射筒内 `bore_shadow`/`dark_bore` 暗盘、铭牌、cable、bolt circles 默认用 `parent.visual(...)`，不建独立 part。

## 关节（Joints）
| 关节 | 类型 | parent_slot.part | child_slot.part | axis | range | 描述 | 来源 |
|---|---|---|---|---|---|---|---|
| `azimuth` / `yaw` | CONTINUOUS or REVOLUTE | A.root_base | B.yaw_turntable | `(0,0,1)` | unlimited（CONT）或 `[-pi, pi]` / `±160°` | 旋回，竖直轴，origin 在 base 顶 slew 轴 | S1-S6 |
| `elevation` / `pitch` | REVOLUTE | B.yaw_turntable | C.elevation_cradle | `(0,-1,0)`（主），`(1,0,0)`（474） | `lower∈[-0.25,0.0]`，`upper∈[0.55,1.30]`（≈45-75°） | 俯仰，axis 在 trunnion line，正向抬升 muzzle | S1-S6 |
| `pod_mount` | FIXED | C.cradle | C.launch_pod | n/a | n/a | 发射箱/筒簇 FIXED 在 cradle 鞍座（welded 变体则无此关节） | S1/S2/S5 |
| `yoke_mount` | FIXED | B.base_ring | B.yoke | n/a | n/a | fork 变体中 base ring 上 FIXED 出 yoke | S5 |
| `blast_door_hinge` | REVOLUTE | C.launcher | D.blast_cover | `(0,0,±1)`（左右镜像） | `[0, ~1.9]` | 发射口防爆门开合 | S3 |
| `reload_cover_hinge` | REVOLUTE | C.launcher_box | D.reload_cover | `(0,-1,0)` | `[0, 1.35]` | 再装填盖翻起 | S1 |
| `outrigger_fold` | REVOLUTE | A.chassis | D.outrigger | `(0,0,±1)` | `[0, ~1.45]` | 车架稳定腿外展 | S3 |
| `handwheel_spin` | CONTINUOUS | B.yaw_turntable | D.handwheel | 水平 | unlimited | 装饰手轮 | S6 |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 默认值 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| `root_style` | enum | `pedestal`, `ground_column`, `vehicle_truck_bed`, `trailer_towed`, `naval_deck`, `tripod` | `pedestal` | Slot A module；gating 车载硬件 | S1/S3/S5/S6 |
| `yaw_style` | enum | `slew_turntable`, `yoke_fork`, `truck_slew_platform`, `yaw_head_compact` | `slew_turntable` | Slot B module，受 root_style 约束 | S1/S3/S5/S6 |
| `yaw_joint_type` | enum | `continuous`, `revolute_full`, `revolute_limited` | `continuous` | yaw 关节类型 | S1/S4 |
| `launcher_style` | enum | `box_pod`, `canister_pack`, `hollow_tube_bank`, `twin_rail`, `single_tube` | `box_pod` | Slot C launcher 形态（驱动 cradle 拓扑） | S1-S6 |
| `pod_attachment` | enum | `fixed_pod`, `welded_into_cradle` | `fixed_pod` | FIXED child 还是 welded visuals | S1/S3 |
| `tube_rows` | int | `[1, 3]` | `2` | tube grid 行数 | S3/S4 |
| `tube_cols` | int | `[1, 4]` | `2` | tube grid 列数 | S3/S4 |
| `tube_count` | int | `1,2,4,6,12` | `4` | 由 rows×cols 或 launcher_style 派生 | S1-S6 |
| `tube_radius` | float | `[0.055, 0.16]` | `0.09` | 单筒外半径 | S3/S5 |
| `tube_length` | float | `[0.90, 4.65]` | `1.4` | 筒/轨长度 | S3/S6 |
| `base_size` | float | `[0.56, 4.2]` | `0.78` | base 平面尺度 | S5/S6 |
| `base_height` | float | `[0.16, 2.30]` | `0.36` | base 顶到 yaw 轴高度 | S1/S6 |
| `slew_ring_radius` | float | `[0.105, 0.88]` | `0.15` | 旋回环半径 | S1/S3 |
| `trunnion_height` | float | `[0.075, 1.28]` | `0.185` | turntable 顶到 trunnion 高度 | S1/S4 |
| `pitch_lower` | float | `[-0.25, 0.0]` | `-0.17` | 俯仰下限 | S1/S6 |
| `pitch_upper` | float | `[0.55, 1.30]` | `1.05` | 俯仰上限 | S1/S6 |
| `has_blast_doors` | bool | `True/False` | `False` | 增加 2 个 cover hinge | S3 |
| `has_reload_cover` | bool | `True/False` | `False` | 增加 reload cover hinge | S1 |
| `has_outriggers` | bool | `True/False` | `False` | trailer/车架增加折叠腿 | S3 |
| `has_sight` | bool | `True/False` | `False` | 瞄准模组（part 或 visual） | S6 |
| `has_handwheel` | bool | `True/False` | `False` | 装饰手轮 | S6 |

## Multiplicity / Copy Logic
- 模板级核心是 one root, one yaw turntable, one elevation cradle, one launcher：核心链不变。
- `tube_count` / `tube_rows` × `tube_cols` 是 launcher 内的 template 级可变 N（box-cell / canister / tube-bank / twin-rail），由 launcher_style 与 grid 参数共同约束。
- blast door 固定 2（左右镜像），outrigger 固定 2，yoke arm 2，slew bolt circle（8/12/16）是 helper 内固定/装饰循环，不暴露独立 N。
- 不引入 missile-eject 数量或 prismatic（语料无）。

## 拓扑多样性审计
总组合数（gating 前）：6 root × 4 yaw × 5 launcher × 5 auxiliary ≈ 600（受 root/yaw/launcher 互斥 gating 收窄）。


| slot | candidate_count | 是否 >=3 | 备注 |
|---|---:|---|---|
| root_base | 6 | yes | pedestal/column/truck/trailer/naval/tripod |
| yaw_turntable | 4 | yes | slew / yoke-fork / truck platform / compact head |
| launcher | 5 | yes | box / canister / tube-bank / twin-rail / single |
| auxiliary | 5 | yes | none / blast doors / reload cover / outriggers / handwheel-sight |

## 部件多样性审计（Part Diversity Audit）
| 部件类型 | observed_variation | 连续参数是否足够 | 是否需要离散参数 | 推荐参数 | 说明 |
|---|---|---|---|---|---|
| root / base | pedestal / 三脚架 / 车架(rails+wheels) / 拖车(axle+tow-eye) / 甲板高座 | no | yes | `root_style` | 腿/轮/车架拓扑不同 |
| yaw turntable | slew disk / 2-arm fork / 车载 platform / compact head | partly | yes | `yaw_style`, `yaw_joint_type` | trunnion 支撑结构不同 |
| elevation cradle | rails-only / side-cheek tray / tube-bank frame / fork-captured | partly | yes（随 launcher_style） | `cradle_style`（slaved） | cradle 拓扑随 launcher 变 |
| launch pod / tubes（身份） | box-cell / 方筒 canister / 圆中空 tube bank / twin rail / single；count 1/2/4/6/12；mesh vs box vs cylinder | no | yes（关键） | `launcher_style`, `tube_rows`, `tube_cols`, `pod_attachment` | 数量/排布/截面都是离散 |
| blast door / cover | 无 / 2 侧门(Z) / 1 reload cover(-Y) | no（存在性） | yes | `has_blast_doors`, `has_reload_cover` | 增加 part/joint |
| outrigger / wheels / sight / handwheel | 车载/装饰专属 | size only | yes（存在性，slaved 到 root_style） | `has_outriggers`, `has_sight`, `has_handwheel` | gated by root_style |
| missiles | 语料中无独立可弹射件 | n/a | no | — | 不发明 eject DOF |

## 组合逻辑（Composition Logic）
1. `root_style` 确定 contact plane（地面/车轮/甲板/三脚架）与 yaw 竖直轴；`base_height` 给出 yaw 轴高度。
2. `yaw` 关节 origin 落在 base 顶 slew race（xy=0），`yaw_joint_type` 决定 CONTINUOUS / 限幅 REVOLUTE。
3. `yaw_style` 在 turntable 上放 slew ring + trunnion bushings（或 FIXED 出 fork cheeks）。
4. `pitch` 关节 origin 落在 trunnion line（`trunnion_height` 高度），axis `(0,-1,0)`（或 474 的 (1,0,0)），cradle part origin 与之重合。
5. `launcher_style` + `tube_rows/cols` 派生 launch pod / tube grid 几何；`pod_attachment` 决定 FIXED child 还是 welded 进 cradle。
6. trunnion shaft/collars 与 turntable bearings 之间嵌入用 scoped `allow_overlap`。
7. blast door 铰链落在发射口 aperture 前缘；outrigger 落在 chassis 侧；reload cover 落在 box 顶后缘。

## 已有模板写法参考
`parabolic_dish_on_azimuth_elevation_mount` / `cctv_mast_with_pantilt_camera_head` / `cannon` / `rotary_post` / `revolute_hinge`（参考 yaw/pitch joint metadata、trunnion `allow_overlap`、参数化 tube/cell 循环组织）

## 约束
- 必须有 yaw（竖直 Z）与 pitch（水平轴）两级运动，链序 root -> yaw turntable -> cradle -> launcher。
- 发射筒/箱/双轨必须是可辨识身份件（中空筒口/箱格/导轨），不退化成裸 box。
- pitch 关节 origin 必须落在 trunnion line 上，axis 符号使正向 pitch 抬升 muzzle。
- `pod_attachment=fixed_pod` 时 pod 通过 FIXED 贴 cradle 鞍座；welded 时 tubes 作为 cradle visuals。
- 车载/拖车/甲板硬件只在对应 root_style 下出现。
- 不引入语料中不存在的 missile-eject prismatic 关节。

## Validator
| 检查项 | 标准 |
|---|---|
| identity | 旋回 turntable + 俯仰 cradle + 可辨识发射筒/箱/轨 同时存在 |
| yaw joint | root -> turntable 绕 `(0,0,1)`，CONTINUOUS 或 REVOLUTE |
| pitch joint | turntable -> cradle 绕水平轴 REVOLUTE，origin 在 trunnion line |
| chain order | root -> yaw -> pitch cradle -> launcher，无漂浮 |
| launcher geometry | tube/box 中空/格仓特征存在，tube_count 与 grid 一致 |
| pod attach | FIXED pod 或 welded tubes 二选一，pod 贴 cradle |
| diversity | `root_style` / `launcher_style` / auxiliary 分支均被覆盖 |
| aux | blast door / reload cover / outrigger 正确挂载且不替换主 yaw/pitch |

## Reject cases
- 发射筒固定不动，没有 yaw/pitch 两级运动。
- 发射筒/箱退化成裸 box，没有筒口/格仓，失去身份。
- pitch axis 竖直或 yaw axis 水平（轴向错误）。
- cradle/pod 脱离 turntable 漂浮，或 trunnion 不在 turntable bearings 上。
- 发明 missile-eject prismatic 关节（语料无）。
- 车载硬件（轮/拖车 outrigger）出现在 pedestal/三脚架等不匹配 root_style 上。
- blast door 铰链放在错误面或挡住发射口身份。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | approved |
| reviewer notes | 用户审核通过，进入 TEMPLATE_AFTER_REVIEW |
