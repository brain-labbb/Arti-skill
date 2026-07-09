# Remote Weapon Station Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `remote_weapon_station` |
| template path | `agent/templates/remote_weapon_station.py` |
| test path | `tests/agent/test_remote_weapon_station_template.py` |
| stage | `REVIEWED` |
| status | `approved` |
| __modular__ | `True` |
| pattern | `linear_chain` |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 33 |
| read_count | 33 |
| read_scope | all 5-star samples in this category: `model.py` / `revision.json` / `record.json` / `prompt.txt` were read |
| samples_adopted_as_module_sources | 6 |
| samples_read_but_not_adopted | 27 |
| source_index_policy | only adopted reusable snippets are indexed below |

全量阅读后的结构族分布：

| 结构族 | 样本数 | 说明 |
|---|---:|---|
| 3-part monolithic（base -> turret -> cradle，gun+optics+ammo 全 baked），2 joint | 10 | pan(Z) + elevation(-Y)，身份件都烘进 cradle |
| fork/yoke base turntable -> cradle，2 joint | 3 | turntable 带 2 fork arms + bearing bosses 捕获 trunnion |
| 4-part 带铰链 service hatch / shield / lid / cover | 8-9 | base -> turret -> cradle -> {lid/hatch/flap/cover}，第 3 关节 REVOLUTE |
| 独立可俯仰 optics/sensor pod（自带 tilt 关节） | 9 | sensor pod 挂在 turret 或中间 FIXED mast 上，独立 REVOLUTE tilt |
| split payload（FIXED sensor + FIXED weapon 兄弟件） | 4 | cradle 是 bare frame，gun 与 optics 各自 FIXED 同骑 elevation |
| multi-payload / maximal（5-6 part） | 3 | ammo box + side yoke + tilting optic，或 weapon + missile + optic |
| gun/barrel + optics + ammo 构造变体 | 33 | barrel + shroud + muzzle/flash hider，EO sensor + glass lens，feed/ammo box |

被采纳样本逐条标注：
- `rec_remote_weapon_station_e52974505580453f8ff659d62ecab0f5` — adopted：最干净 canonical 3-part（pedestal / yaw_base / cradle），CONTINUOUS pan Z + REVOLUTE elevation -Y，gun+optics 烘进 cradle，自带 axis validator。
- `rec_remote_weapon_station_b950e8e494cd4635a8327c002a181524` — adopted：annular slew rings（Lathe helper）+ guard_arm REVOLUTE aux 铰链，primitive-only。
- `rec_remote_weapon_station_0002` — adopted：mesh-helper 驱动的 base/housing/cradle + barrel chain + feed chute。
- `rec_remote_weapon_station_4d48291835574d2c9807efdde40c9a41` — adopted：独立 FIXED sensor_mast + 独立可 tilt sensor_pod（Family D）。
- `rec_remote_weapon_station_0004` — adopted：bare cradle_frame + FIXED sensor_pod + FIXED weapon_module（split payload）。
- `rec_remote_weapon_station_fd5b05f1c11b4e75bcfe989a29141a04` — adopted：fork turret（Lathe ring + fork arms）+ elevation + access_hatch 铰链。
- Remaining 27 samples — read but not adopted：同一 base -> turret -> cradle 链 + barrel/optics/ammo 变体与各类 hatch/shield/sensor-tilt 已被上述模块覆盖。

## 核心身份
`remote_weapon_station`（RWS）是一个遥控武器站：static base/pedestal/roof mount 承载一个绕竖直 Z 轴 pan（方位）的 turret housing，再通过 trunnion 抱住一个绕水平轴 elevation（俯仰）的 weapon cradle/mantlet，cradle 上是 gun/barrel（+ muzzle brake/flash hider）、EO/IR optics sensor（带 glass lens）、ammo/feed box。身份件是 barrel + optics pod + ammo can。默认成熟域是 two-DOF pan/elevation 链；service hatch/shield、独立可 tilt sensor pod、split FIXED payload 是可选扩展。语料中无 barrel recoil prismatic 关节（仅静态 recoil sleeve 几何）。

边界：
- 不包括固定不动的炮塔：必须有 pan + elevation articulation。
- 不混入 missile launcher：身份件是带 barrel + optics 的机枪/机炮，不是导弹箱/筒簇（missile pod 仅作为 weapon_style 罕见变体）。
- 不混入 parabolic dish / radar：是武器 + 光电，不是天线。
- 不要发明 barrel recoil prismatic 关节（语料 0/33）。

## 采用源码索引（Adopted Source Index）
| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| S1 | `rec_remote_weapon_station_e52974505580453f8ff659d62ecab0f5` | `data/records/rec_remote_weapon_station_e52974505580453f8ff659d62ecab0f5/revisions/rev_000001/model.py:L48-L227` | canonical 3-part：pedestal L48-L77、yaw_base L78-L126、cradle(gun+optics baked)L127-L208、pan CONTINUOUS + elevation REVOLUTE joints L210-L227 |
| S2 | `rec_remote_weapon_station_b950e8e494cd4635a8327c002a181524` | `data/records/rec_remote_weapon_station_b950e8e494cd4635a8327c002a181524/revisions/rev_000001/model.py:L26-L240` | annular ring Lathe helper L26-L36、yaw_base(slew rings)L38-L67、turret L68-L148、cradle(barrel+muzzle+ammo+sensor)L149-L193、guard_arm L194-L213、joints(pan/elev/guard_hinge)L214-L240 |
| S3 | `rec_remote_weapon_station_0002` | `data/records/rec_remote_weapon_station_0002/revisions/rev_000001/model.py:L69-L345` | mesh helpers(base/housing/cradle)L69-L147、base build L253-L270、housing L271-L300、cradle + barrel chain + feed chute L301-L327、joints L328-L345 |
| S4 | `rec_remote_weapon_station_4d48291835574d2c9807efdde40c9a41` | `data/records/rec_remote_weapon_station_4d48291835574d2c9807efdde40c9a41/revisions/rev_000001/model.py:L28-L248` | pedestal L28-L46、turret L48-L96、weapon_cradle L108-L156、FIXED sensor_mast L168-L204、tilting sensor_pod L214-L238、joints(yaw/pitch/mast FIXED/sensor tilt)L98-L106,L158-L166,L206-L212,L240-L248 |
| S5 | `rec_remote_weapon_station_0004` | `data/records/rec_remote_weapon_station_0004/revisions/rev_000001/model.py:L53-L304` | pedestal L53-L82、turret_body + bearings L84-L131、bare cradle_frame(side plates + trunnions)L133-L180、FIXED sensor_pod L182-L217、FIXED weapon_module L219-L266、joints L268-L304 |
| S6 | `rec_remote_weapon_station_fd5b05f1c11b4e75bcfe989a29141a04` | `data/records/rec_remote_weapon_station_fd5b05f1c11b4e75bcfe989a29141a04/revisions/rev_000001/model.py:L59-L284` | pedestal L59-L88、turret_fork(ring shell + arms + bearing housings)L90-L159、weapon_housing(section-loft shell + trunnions + barrel)L161-L221、access_hatch L223-L246、joints(slew/elevation/hatch hinge)L248-L284 |

## 槽位 + 候选模块表

### Slot A：base_mount
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `ring_pedestal` | S1 | `model.py:L48-L77` | **yes** | 圆柱 column + flange + slew ring + bolt circle |
| `annular_ring_base` | S2 | `model.py:L38-L67` | | Lathe 环形 slew ring 叠层 |
| `roof_deck_mount` | S4 | `model.py:L28-L46` | | 大平板 roof/deck 安装座 |
| `pintle_post` | S6 | `model.py:L59-L88` | | 高柱 + torus ring + 开放支柱 |

### Slot B：turret_yaw_stage
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `housing_turret` | S1 | `model.py:L78-L126` | **yes** | 装甲 housing + deck + cheeks + trunnion bosses |
| `slew_ring_deck` | S2 | `model.py:L68-L148` | | slew ring + deck + cheeks + shields + electronics hump |
| `fork_turret` | S6 | `model.py:L90-L159` | | Lathe ring shell + 2 fork arms + bearing housings |
| `open_frame_turret` | S4 | `model.py:L48-L96` | | 开放框 + 支撑 + trunnion bosses（带独立 sensor mast 时） |

### Slot C：weapon_cradle
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `monolithic_gun_cradle` | S1 | `model.py:L127-L208` | **yes** | trunnion cradle，barrel+shroud+muzzle+optics+ammo 全 baked |
| `cradle_with_feed_chute` | S3 | `model.py:L301-L327` | | cradle + barrel chain + ammo box + feed chute（mesh） |
| `loft_shell_housing` | S6 | `model.py:L161-L221` | | section-loft 武器壳体 + trunnions + barrel |
| `bare_frame_split_payload` | S5 | `model.py:L133-L266` | | bare cradle_frame + FIXED sensor_pod + FIXED weapon_module |

### Slot D：auxiliary_payload_mechanism
| module_name | 5_star_source | model.py:Lx-Ly | seed=0 anchor | 结构特征 |
|---|---|---|---|---|
| `none` | S1 | `model.py:L210-L227` | **yes** | 只有 pan + elevation |
| `guard_or_shield_hinge` | S2 | `model.py:L194-L240` | | guard_arm / shield flap REVOLUTE（轴沿铰链边） |
| `service_hatch` | S6 | `model.py:L223-L284` | | top lid / access hatch REVOLUTE（开盖向上） |
| `tilting_sensor_pod` | S4 | `model.py:L168-L248` | | FIXED sensor_mast + 独立可 tilt sensor_pod REVOLUTE |
| `split_fixed_payload` | S5 | `model.py:L182-L266` | | FIXED sensor + FIXED weapon 兄弟件骑 elevation |

## 槽位图（slot graph）
pattern = `linear_chain`

```
[Slot A base_mount]
      -- pan CONTINUOUS/REVOLUTE, axis (0,0,1) -->
[Slot B turret_yaw_stage]
      -- elevation REVOLUTE, axis (0,-1,0) [naval: (0,1,0)] -->
[Slot C weapon_cradle]   (barrel forward axis 与 elevation axis 符号匹配)
      -- optional FIXED (split payload) / REVOLUTE (hatch / sensor tilt / shield) -->
[Slot D auxiliary]
```

主约束基准是 pan vertical axis 与 elevation trunnion line。turret cheeks、barrel 前向、optics、ammo、hatch 铰链都从这两条基准派生。barrel 前向轴必须与 elevation axis 符号匹配（正向 elevation 抬升 muzzle）。

## 部件（Parts）

### Slot A / base_mount
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `pedestal` / `mount_base` / `roof_base` / `yaw_base` | ~3-14 | 圆柱/环形/平板/柱式 static 安装座 + slew race + bolt circle | S1/S2/S4/S6 |

### Slot B / turret_yaw_stage
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `turret` / `turret_housing` / `azimuth_stage` / `turret_fork` | ~4-25 | 旋转炮塔 + deck + cheeks/fork arms + trunnion bosses | S1/S2/S4/S6 |

### Slot C / weapon_cradle
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `weapon_cradle` / `cradle` / `cradle_frame` / `weapon_housing` | ~6-25 | trunnion cradle/mantlet；barrel+shroud+muzzle+receiver（身份）+ optics + ammo（baked 或 frame） | S1/S3/S5/S6 |

### Slot D / auxiliary_payload_mechanism
| part | visual_count | 描述 | 来源 |
|---|---:|---|---|
| `sensor_pod` / `optic_head` / `sensor_mast` | ~2-7 | 独立可 tilt 光电模组（+ 中间 FIXED mast） | S4 |
| `weapon_module` / `sensor_unit`（split） | ~3-6 | split payload 中 FIXED gun / FIXED optics 兄弟件 | S5 |
| `service_lid` / `access_hatch` / `shield_flap` / `guard_arm` | ~3-6 | 铰链舱盖/护盾/防护臂（REVOLUTE） | S6 / S2 |

ammo box 内 feed、bolt circles、铭牌、recoil sleeve、glass lens 反光等默认用 `parent.visual(...)`，不建独立 part（lens 可作为 optics part 的 visual，材质 glass）。

## 关节（Joints）
| 关节 | 类型 | parent_slot.part | child_slot.part | axis | range | 描述 | 来源 |
|---|---|---|---|---|---|---|---|
| `pan` / `azimuth` / `turret_yaw` | CONTINUOUS or REVOLUTE | A.base_mount | B.turret | `(0,0,1)` | unlimited（CONT）或 `[-pi, pi]` | 方位旋转，竖直轴，origin 在 slew ring 平面（child xy 居中） | S1-S6 |
| `elevation` / `weapon_pitch` | REVOLUTE | B.turret | C.weapon_cradle | `(0,-1,0)`（主），`(0,1,0)`（naval） | `lower∈[-0.35,-0.10]`，`upper∈[0.45,1.10]` | 武器俯仰，origin 在 trunnion line | S1-S6 |
| `optic_tilt` / `sensor_tilt` | REVOLUTE | B.turret / D.sensor_mast | D.sensor_pod | `(0,-1,0)`/`(0,1,0)`/`(1,0,0)` | 小幅，如 `[-0.45, 0.70]` | 独立光电俯仰 | S4 |
| `mast_mount` / `sensor_fixed` / `weapon_fixed` | FIXED | C.cradle / B.turret | D.payload | n/a | n/a | split payload / 固定 mast 座 | S5/S4 |
| `hatch_hinge` / `lid_hinge` | REVOLUTE | C.cradle / B.turret | D.service_lid | `(1,0,0)`/`(0,-1,0)`（沿闭合边） | `[0, ~1.65]` | service hatch/lid 开合 | S6 |
| `shield_flap` / `guard_hinge` | REVOLUTE | C.cradle / B.turret | D.shield/guard | `(0,-1,0)`/`(0,0,±1)` | `[0, ~1.65]` | 护盾/防护臂侧摆 | S2 |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 默认值 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| `base_style` | enum | `ring_pedestal`, `annular_ring`, `roof_deck`, `pintle_post` | `ring_pedestal` | Slot A module | S1/S2/S4/S6 |
| `turret_style` | enum | `housing`, `slew_ring_deck`, `fork`, `open_frame` | `housing` | Slot B module，受 base_style 约束 | S1/S2/S4/S6 |
| `pan_joint_type` | enum | `continuous`, `revolute` | `continuous` | pan 关节类型 | S1/S2 |
| `cradle_style` | enum | `monolithic_gun`, `feed_chute`, `loft_shell`, `bare_frame_split` | `monolithic_gun` | Slot C module | S1/S3/S5/S6 |
| `weapon_style` | enum | `mg`, `autocannon`, `cannon_heavy`, `missile_pod` | `mg` | barrel 尺度/数量 | S1/S2 |
| `muzzle_style` | enum | `brake`, `flash_hider`, `plain` | `brake` | 枪口装置 | S1/S2 |
| `optics_style` | enum | `baked_in`, `fixed_pod`, `tilting_pod`, `side_yoke`, `mast` | `baked_in` | optics 挂载/articulation | S1/S4/S5 |
| `ammo_style` | enum | `none`, `cradle_side`, `turret_deck` | `cradle_side` | ammo box 位置 | S1/S3 |
| `aux_hinge_style` | enum | `none`, `top_lid`, `side_cover`, `front_shield`, `guard_arm`, `mast_fold` | `none` | Slot D 铰链类型（含轴选择） | S2/S6 |
| `base_radius` | float | `[0.14, 0.72]` | `0.24` | 安装座半径 | S1/S2 |
| `base_height` | float | `[0.10, 1.12]` | `0.36` | 安装座顶 = pan origin Z | S1/S2 |
| `turret_size` | float | `[0.26, 0.94]` | `0.5` | turret deck 平面尺度 | S1/S6 |
| `elevation_origin_z` | float | `[0.10, 0.68]` | `0.28` | turret deck 上 trunnion 高度 | S1/S4 |
| `barrel_length` | float | `[0.40, 0.98]` | `0.7` | 枪管长度（不含 muzzle） | S1/S2 |
| `barrel_radius` | float | `[0.010, 0.092]` | `0.03` | 枪管/护套半径 | S1/S2 |
| `optics_size` | float | `[0.10, 0.28]` | `0.18` | 光电模组长度 | S1/S2 |
| `n_optic_lenses` | int | `[1, 2]` | `1` | day/thermal 镜片数 | S2 |
| `elevation_lower` | float | `[-0.35, -0.10]` | `-0.20` | 俯仰下限 | S1/S2 |
| `elevation_upper` | float | `[0.45, 1.10]` | `0.85` | 俯仰上限 | S1/S2 |
| `n_base_bolts` | int | `0,8,12,16` | `8` | 装饰螺栓圈 | S1/S2 |
| `has_ammo` | bool | `True/False` | `True` | ammo can 是否存在 | S1/S3 |
| `has_feed_chute` | bool | `True/False` | `False` | 供弹链导管 mesh | S3 |

## Multiplicity / Copy Logic
- 模板级核心是 one base, one turret, one weapon cradle, optional one auxiliary（hatch / tilting sensor / split payload）：核心链不变。
- `n_optic_lenses`（1-2）、`n_base_bolts`（螺栓圈）是 module-local 可变 N；barrel chain（barrel+shroud+muzzle+receiver）、trunnion bosses（2）、fork arms（2）是 helper 内固定循环。
- 不引入 dual-gun 双管（语料无真正双管）；missile_pod 仅作为 weapon_style 罕见变体，不引入 eject prismatic。

## 拓扑多样性审计
总组合数（gating 前）：4 base × 4 turret × 4 cradle × 6 aux ≈ 384（受 base/turret/optics 互斥 gating）。


| slot | candidate_count | 是否 >=3 | 备注 |
|---|---:|---|---|
| base_mount | 4 | yes | ring/annular/roof/pintle |
| turret_yaw_stage | 4 | yes | housing/slew-deck/fork/open |
| weapon_cradle | 4 | yes | monolithic/feed/loft/split |
| auxiliary | 6 | yes | none/lid/cover/shield/guard/mast |

## 部件多样性审计（Part Diversity Audit）
| 部件类型 | observed_variation | 连续参数是否足够 | 是否需要离散参数 | 推荐参数 | 说明 |
|---|---|---|---|---|---|
| base / ring | 圆柱 pedestal + slew ring / 环形叠层 / 平板 roof / 高柱 pintle；bolt 0/8/12/16 | no | yes | `base_style`, `base_radius`, `base_height`, `n_base_bolts` | 拓扑不同（圆柱 vs 平板 vs 开放柱） |
| turret housing | 装甲 housing / slew-ring deck / 开放 fork arms | no | yes | `turret_style` | housing vs fork vs open 拓扑不同 |
| weapon cradle | monolithic gun-cradle / feed-chute / loft 壳 / bare frame split | mostly | yes | `cradle_style` | payload 挂载方式不同 |
| gun / barrel（身份） | 单管 + shroud + muzzle；机炮；missile pod；muzzle brake/flash/plain | mostly | yes | `weapon_style`, `muzzle_style`, `barrel_length`, `barrel_radius` | 武器形态离散 |
| optics / sensor（身份） | baked-in / 独立 FIXED pod / 独立 tilt pod / side yoke / mast；1-2 镜片 | no | yes | `optics_style`, `n_optic_lenses` | articulation 与挂载拓扑不同 |
| ammo box | 无 / cradle 侧 / turret deck / 带 feed chute | mostly | yes（小） | `ammo_style`, `has_ammo`, `has_feed_chute` | 位置离散 |
| shields / aux hatch | 无 / top lid / side cover / front shield / mast fold / guard arm | no | yes | `aux_hinge_style` | 增加 part/joint，含轴选择 |

## 组合逻辑（Composition Logic）
1. `base_style` 确定 contact plane（地面/车顶/甲板）与 pan 竖直轴；`base_height` 给出 pan origin Z；pan origin 在 slew ring 平面、child xy 居中。
2. `pan_joint_type` 决定 CONTINUOUS / 限幅 REVOLUTE。
3. `turret_style` 在 turret 上对称放 cheeks/fork arms + trunnion bosses，`elevation_origin_z` 给出 trunnion line 高度。
4. `elevation` 关节 origin 落在 trunnion line（cradle part origin 与 trunnion 重合），axis `(0,-1,0)`（naval (0,1,0)）。
5. barrel 前向轴与 elevation axis 符号匹配：aim +X 配 `(0,-1,0)`（barrel rpy=(0,π/2,0)），aim +Y 配 `(0,1,0)`。正向 elevation 抬升 muzzle。
6. `optics_style` 决定 optics baked 进 cradle 还是独立 FIXED/tilt pod；ammo box 按 `ammo_style` 贴 cradle 侧或 turret deck。
7. trunnion 与 cheek/boss 嵌入用 scoped `allow_overlap`（语料 ~15 例 idiom）；hatch/shield/guard 铰链落在对应闭合边。

## 已有模板写法参考
`parabolic_dish_on_azimuth_elevation_mount` / `cctv_mast_with_pantilt_camera_head` / `cannon` / `monitor_mount` / `rotary_post` / `revolute_hinge`（参考 pan/elevation joint metadata、barrel 前向、trunnion `allow_overlap`、glass lens 写法）

## 约束
- 必须有 pan（竖直 Z）与 elevation（水平轴）两级运动，链序 base -> turret -> cradle。
- cradle 必须有可辨识 barrel（+ muzzle 装置）与 optics（带 glass lens），ammo can 推荐。
- elevation 关节 origin 必须落在 trunnion line，barrel 前向轴与 elevation axis 符号匹配。
- pan origin 在 slew ring 平面、child xy 居中。
- 独立 sensor pod 必须挂在 turret 或 FIXED mast 上并有自己的 REVOLUTE tilt；split payload 用 FIXED 同骑 elevation。
- 不引入 barrel recoil prismatic（语料无；recoil 只作静态 sleeve 几何）。

## Validator
| 检查项 | 标准 |
|---|---|
| identity | barrel + muzzle + optics(glass lens) + 旋转 turret 同时存在 |
| pan joint | base -> turret 绕 `(0,0,1)`，CONTINUOUS 或 REVOLUTE，child xy 居中 |
| elevation joint | turret -> cradle 绕水平轴 REVOLUTE，origin 在 trunnion line |
| barrel axis | barrel 前向轴与 elevation axis 符号一致（正向 elevation 抬升 muzzle） |
| chain order | base -> turret -> cradle，无漂浮武器 |
| optics | optics 含 glass lens；独立 pod 有自己的 tilt 关节 |
| diversity | `base_style` / `turret_style` / `weapon_style` / `optics_style` 分支均被覆盖 |
| aux | hatch/shield/sensor pod 正确挂载且不替换主 pan/elevation |

## Reject cases
- 武器固定不动，没有 pan/elevation 两级运动。
- barrel 退化成裸 cylinder 无 muzzle，或缺少 optics/lens，失去身份。
- elevation axis 竖直或 pan axis 水平（轴向错误）。
- barrel 前向与 elevation axis 符号不一致，正向 elevation 反而压低 muzzle。
- cradle 脱离 turret 漂浮，或 trunnion 不在 turret cheeks 上。
- 独立 sensor pod 无 tilt 关节漂浮，或 split payload 未 FIXED。
- 发明 barrel recoil prismatic 关节（语料无）。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | approved |
| reviewer notes | 用户审核通过，进入 TEMPLATE_AFTER_REVIEW |
