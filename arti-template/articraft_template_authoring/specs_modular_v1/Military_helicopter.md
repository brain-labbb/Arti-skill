# Tandem Rotor Helicopter Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `tandem_rotor_helicopter` |
| template path | `agent/templates/Military_helicopter.py` |
| test path | `tests/agent/test_tandem_rotor_helicopter_template.py` (optional) |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` |

## 样本阅读摘要
| 项 | 值 |
|---|---|
| sample_pool | 13 curated workbench variants of one hand-built CH-47 parent (category `tandem_rotor_helicopter`) |
| source_repo | `/mnt/zsn/lyb/arti-skill/articraft_data` (the variants we authored + QC-passed; not yet synced into this repo's `data/records`) |
| read_scope | every variant's `revisions/rev_000001/model.py` was authored/verified in this session; each is a single-axis (or feature-additive) edit of the parent |
| samples_adopted_as_module_sources | parent + 7 structural variants (the rest are pure multiplicity points) |
| source_index_policy | one parent supplies the common airframe/rotor/ramp geometry; each structural variant supplies one module or feature; multiplicity variants (blade/seat/exhaust counts) are sampled, not separate modules |

不同于 `single_rotor_helicopter`(只有 2 个 5 星源、每槽降级到 2 候选),本类目有一组**同源、单轴可控**的样本,可让多数轴拿到 ≥3 候选或宽多重性区间。

## 核心身份

Tandem-rotor heavy-lift helicopter (CH-47 Chinook family): a long box/pod fuselage with **two** large main rotors in tandem — a low front rotor over the cockpit pylon and a taller rear rotor on the aft pylon — counter-rotating, no tail anti-torque rotor. Twin engine nacelles flank the rear pylon, side fuel sponsons carry four-wheel landing gear, and an upswept tail has a rear cargo ramp.

核心运动:
- `front_rotor_spin` / `rear_rotor_spin`: CONTINUOUS, near-vertical axes (tilted ~9°/4° toward the nose), opposite spin senses.
- `*_wheel_spin` ×4: CONTINUOUS about the lateral (Y) axis.
- `ramp_hinge`: REVOLUTE about lateral (-Y) at the upswept tail.
- optional `*_oleo_slide` ×4 (PRISMATIC suspension), `cockpit_door_hinge` / `bulkhead_door_hinge` (REVOLUTE).

边界:
- **不是** single-rotor helicopter: two main lift rotors in tandem, **no tail rotor**.
- **不是** coaxial / drone / multirotor: exactly two main rotors on fore/aft pylons at different heights.
- **不是** airplane: vertical-lift tandem rotors, no fixed wing.

## 采用源码索引（Adopted Source Index）

base path = `/mnt/zsn/lyb/arti-skill/articraft_data/data/records/<id>/revisions/rev_000001/model.py`

| source_id | record_id (title) | 采纳用途 |
|---|---|---|
| P  | `rec_model-a-ch-47-chinook-style-tandem-rotor-militar_20260610_080515_971792_e6faf6f7` (parent) | 公共骨架:`box` 机身(hull L181-191 / inner L217-228 / canopy L251-264 / pylons L275-300 / engines L303-313 / exhaust L315-345 / sponsons L347-356 / windows L359+ / cabin floor+seats L386-418),刚性起落架(L420-476),双旋翼 `_build_rotor`(L480-548),尾货舱坡道(L550-606),run_tests(L611+) |
| H1 | `rec_change-only-the-fuselage-cross-section-profile-f_20260613_164225_095089_217fe6cb` (`hull_bellypod`) | 机身体型模块:深腹圆胖荚(中段 hull/inner 截面下鼓+顶拱+加宽,n=8 loft) |
| H2 | `rec_change-only-the-fuselage-cross-section-profile-t_20260613_164225_081671_13e1ea10` (`hull_humpback`) | 机身体型模块:后背脊驼峰(`dorsal_hump` section_loft,x[-6.4,-3.6] 峰 z~4.9,避前/后桨盘) |
| H3 | `rec_change-only-the-fuselage-longitudinal-proportion_20260613_173231_596812_49522ef1` (`hull_stretch`) | 机身体型模块:货舱纵向拉长 ~5m,后旋翼/起落架随之后移 |
| N  | `rec_change-only-the-front-main-rotor-to-have-6-blade_20260613_164225_083836_00cd0da3` (`front_blades_6_pointed_nose`) | 尖鼻特征:前段 hull(x≥6.3)收尖到 0.11 半宽 + 座舱罩/侧窗跟随收尖(toggle) |
| G  | `rec_change-only-the-cabin-troop-seats-to-14-per-side_20260613_164225_076397_d46255b1` (`seats_14_oleo_gear`) | 伸缩起落架模块:外套筒(fus)+内杆 part(PRISMATIC,0.18m 行程)+轮挂内杆;并提供高座椅多重性取值 |
| DC | `rec_add-an-openable-cockpit-crew-door-on-the-left-forward-fuselage-cut-a-door-shaped_20260614_061355_897317_1a7c073a` (`cockpit_door`) | 外部驾驶舱门模块:左前侧面开洞 + 贴合曲面门板(窗+精致把手)+ 门框,REVOLUTE 竖铰链 |
| DB | `rec_add-an-internal-cockpit-cabin-bulkhead-partition-at-the-forward-end-of-the-troop_20260614_063640_497499_c10fd524` (`bulkhead_door`) | 内部隔板门模块:x=3.0 横向隔墙(挖门洞)+ 隔板门 part(窗+杆把手),REVOLUTE 竖铰链 |

纯多重性取值点(不作独立 module,作为采样区间的源例):`seats_4`,`seats_14`(=4/9/14),`exhaust_1`/`exhaust_4`(=1/2/4),`front_blades_4`/`front_blades_6`,`rear_blades_4`/`rear_blades_6`。

## 槽位 + 候选模块表

### Slot A：airframe（机身体型，grounded root）
| module_name | source | 结构特征 |
|---|---|---|
| `box_airframe` | P | 基线 CH-47 slab 机身(中段方箱,圆角 r0.30) |
| `bellypod_airframe` | H1 | 深腹圆胖荚:中段下鼓(腹 z~0.55)+顶拱(z~3.6)+加宽,大圆角桶截面 |
| `humpback_airframe` | H2 | 后背脊驼峰:基线身 + 后段 dorsal hump 凸起(峰 z~4.9,在两桨盘之间让开) |
| `stretch_airframe` | H3 | 长身重载:货舱纵向 +5m,后旋翼+后起落架后移以覆盖加长机身 |

> 4 候选,均来自结构不同的样本(下鼓 / 上鼓 / 拉长 / 中庸),满足 ≥3。

### Slot B：rotors（串列双旋翼,parallel children of airframe）
始终 2 个旋翼(tandem 是核心身份),变化来自**前/后叶数独立多重性**。不作 enum 候选,作多重性维度(见下)。前旋翼 hub 低、轴前倾 9°;后旋翼 hub 高 ~2m、轴前倾 4°、反向自旋。

### Slot C：landing_gear（起落架,parallel children of airframe）
| module_name | source | 结构特征 |
|---|---|---|
| `fixed_strut_gear` | P | 刚性竖杆 + 横轴 + 自旋轮(轮挂机身);仅 CONTINUOUS 轮关节 |
| `oleo_telescoping_gear` | G | 油气减震:外套筒(机身固定)+ 内杆 part(PRISMATIC 竖直滑动,行程 0.18m)+ 自旋轮挂内杆;增加 4 个 PRISMATIC 关节 |

### Slot D：doors（门/通道,parallel children of airframe）
| module_name | source | 结构特征 |
|---|---|---|
| `none` | — | 无额外门(仅保留尾坡道) |
| `cockpit_door` | DC | 外部驾驶舱门:侧面开洞 + 贴合门板 + 门框,REVOLUTE 竖铰链外摆 |
| `bulkhead_door` | DB | 内部隔板门:横向隔墙 + 通行门,REVOLUTE 竖铰链朝驾驶舱开 |
| `both` | DC+DB | 同时含外部舱门 + 内部隔板门 |

## 槽位图（slot graph）

pattern: `mixed`（airframe 为根,其余为并行子件;固定保留尾坡道）

```text
[Slot A airframe]  (root, grounded; always carries the rear cargo ramp REVOLUTE)
  ├── front_rotor_spin CONTINUOUS, axis ~Z(+9° fwd)   --> front rotor  (Slot B, n2..n8 blades)
  ├── rear_rotor_spin  CONTINUOUS, axis ~Z(+4° fwd)   --> rear rotor   (Slot B, n2..n8 blades)
  ├── 4× wheel_spin CONTINUOUS axis Y  (+ optional 4× oleo_slide PRISMATIC axis Z) --> [Slot C gear]
  └── optional REVOLUTE cockpit / bulkhead door hinges --> [Slot D doors]
```

## 部件（Parts）
| part | slot | 描述 | 来源 |
|---|---|---|---|
| `fuselage` | A | hull(lofted shell, hollow, rear cargo doorway cut)、canopy、front/rear pylons、engines、exhaust(N nozzles)、sponsons、windows、cabin floor + troop seats(N/side)、(可选)door opening cut / bulkhead wall / door frame、刚性起落架套筒或固定杆 | P + H1/H2/H3 + N + G + DC/DB |
| `front_rotor` / `rear_rotor` | B | mast + hub + N tapered blades(径向均布,黄绿叶尖) | P |
| `cargo_ramp` | A | 尾坡道(hinge barrel + plate + lip + ribs) | P |
| `{f/r}_{l/r}_wheel` ×4 | C | tire + hub,CONTINUOUS spin | P |
| `{f/r}_{l/r}_oleo` ×4 | C | 仅 `oleo_telescoping_gear`:内杆 + 轴(PRISMATIC) | G |
| `cockpit_door` | D | 贴合门板 + 窗 + 把手 + 铰链节 | DC |
| `bulkhead_door` | D | 门板 + 窗 + 把手 + 铰链节 | DB |

## 关节（Joints）
| 关节 | 类型 | parent | child | axis | range |
|---|---|---|---|---|---|
| `front_rotor_spin` | CONTINUOUS | fuselage | front_rotor | ~`(sin9°,0,cos9°)` | unbounded |
| `rear_rotor_spin` | CONTINUOUS | fuselage | rear_rotor | ~`(sin4°,0,cos4°)`(反向) | unbounded |
| `{g}_wheel_spin` ×4 | CONTINUOUS | fuselage 或 `{g}_oleo` | wheel | `(0,1,0)` | unbounded |
| `{g}_oleo_slide` ×4 | PRISMATIC | fuselage | oleo piston | `(0,0,1)` | `0..0.18` |
| `ramp_hinge` | REVOLUTE | fuselage | cargo_ramp | `(0,-1,0)` | `0..0.74` |
| `cockpit_door_hinge` | REVOLUTE | fuselage | cockpit_door | `(0,0,1)` | `0..1.30` |
| `bulkhead_door_hinge` | REVOLUTE | fuselage | bulkhead_door | `(0,0,1)` | `0..1.40` |

## 参数范围汇总
| 参数 | 类型 | 取值范围 | 默认 | 来源 |
|---|---|---|---|---|
| `airframe_module` | enum | `box`/`bellypod`/`humpback`/`stretch` | `box` | A |
| `gear_module` | enum | `fixed_strut`/`oleo_telescoping` | `fixed_strut` | C |
| `doors_module` | enum | `none`/`cockpit_door`/`bulkhead_door`/`both` | `none` | D |
| `pointed_nose` | bool | `True`/`False` | `False` | N |
| `front_blade_count` | int | `2..8` | 3 | B + 多重性 |
| `rear_blade_count` | int | `2..8` | 3 | B + 多重性 |
| `seat_count` | int(/side) | `4..18` | 9 | 多重性(源 4/9/14) |
| `exhaust_nozzle_count` | int | `1..4` | 2 | 多重性(源 1/2/4) |
| `fuselage_*`(长/宽/高微扰) | float | 围绕基线 ±10% | 基线 | P |

## Multiplicity / Copy Logic
- `front_blade_count` / `rear_blade_count`:`N=2..8`,各旋翼**独立**采样;blade `i` 相位 `i*2π/N`,根部接 hub;单一移动关节仍是 `{f/r}_rotor_spin`。
- `seat_count`:每侧 `N` 排,沿货舱 x 等距;座椅是 fuselage 内 visual,不增加关节。
- `exhaust_nozzle_count`:`1..4`,后喷口板上对称布孔 + 空心喷管。
- 几何护栏:体型/拉长改变机身包围盒时,旋翼盘离地/离顶间隙、起落架着地(z=0)、坡道触地角必须随之解析(参照样本里已验证的护栏)。

## 拓扑多样性审计
slot_choices 编码:`("airframe",m) × ("gear",m) × ("doors",m) × ("front_rotor",f"n{N}") × ("rear_rotor",f"n{N}") × ("nose","pointed"/"blunt")`(座椅/喷口可另计)。

| slot | candidate_count | ≥3 |
|---|---:|---|
| airframe | 4 | yes |
| landing_gear | 2 | no(机构二元;靠 PRISMATIC 增删撑拓扑) |
| doors | 4 | yes |
| front/rear rotor (multiplicity) | 7 each | yes |

### Procedural Sampling / Sweep Plan
`config_from_seed(seed)`:`rng=random.Random(seed)`,对每个 enum 用 `rng.choice`,叶数/座椅/喷口用 `rng.randint`,尖鼻 `rng.random()<p`。seed 0 不特殊(全程序化采样,符合 Contract 4)。`resolve_config` 做范围 clamp + 由体型/拉长解析旋翼挂点、起落架位置、坡道角等派生量。
兼容性:体型与所有起落架/门/叶数自由组合(互不冲突,几何护栏吸收差异);`stretch` 需重算后旋翼/后起落架 x 偏移。

## Validator（run_tests 必须覆盖）
- 恰好两个主旋翼装配(front+rear),均 CONTINUOUS、近竖轴、反向;**无尾旋翼**。
- 后旋翼 hub 明显高于前旋翼(tandem 特征),两 hub 沿 x 分置首尾。
- 机身承载四轮起落架 + 尾坡道;旋翼/门/轮不悬空(可见支撑路径)。
- 各旋翼叶数 == `*_blade_count`,均布接 hub;喷口数 == `exhaust_nozzle_count`;座椅数 == `seat_count`(每侧)。
- `oleo_telescoping_gear`:4 个 PRISMATIC,q=0 着地、行程内内杆不脱套筒;轮挂内杆。
- 门(若启用):正确铰链轴(竖 Z)、关闭从 flush(q=0),内部隔板门用 `expect_within` 证明被机身包含。
- element-scoped `allow_overlap`:旋翼 mast↔pylon/hull、轮毂↔轴、套筒↔内杆、门铰链↔框/壳;captured-pin 用 expect_contact 证明。

## Reject cases
- 出现尾旋翼,或只有一个主旋翼,或同轴双旋翼。
- 两旋翼等高(失去 tandem 特征)或不分置首尾。
- 叶片脱离 hub、轮悬空、门浮在机身外、伸缩内杆满行程脱出套筒。
- 旋翼盘与机身/驼峰/尖鼻在 rest pose 穿模(未声明)。

## 与相邻类别的边界
- `single_rotor_helicopter`:一主旋翼 + 尾旋翼;本类目两主旋翼、无尾旋翼。
- `drone`:多小旋翼环绕框架,无机身/坡道。
- `airplane`:固定翼,无垂直升力旋翼。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | SPEC_ONLY_DRAFT;源自本会话手搭并 QC 通过的 13 个 tandem-rotor workbench 变体(在 articraft_data),比 single_rotor 的 2-源情形更充分 |
