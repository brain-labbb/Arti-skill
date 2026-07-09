# Cannon Spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `cannon` |
| template path | `agent/templates/cannon.py` |
| test path | `tests/agent/test_cannon_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `SPEC_ONLY_DRAFT` |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 37 |
| read_count | 37 |
| read_scope | all 5-star samples in this category |
| source_index_policy | only adopted reusable snippets are indexed below |

## 核心身份
火炮类对象至少包含承载架或底座、带炮口/炮尾/耳轴特征的炮管，以及炮管绕耳轴俯仰的活动方式。常见高分样本还会加入轮式炮架、甲板旋回座、后坐滑轨、迫击炮楔块或分腿炮架，但这些属于布局变体而不是所有样本的硬性交集。

## 采用源码索引（Adopted Source Index）
| source_id | sample_id | model.py 来源 | 采纳用途 |
|---|---|---|---|
| S1 | rec_cannon_0004 | `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L175-L349` | 轮式野战炮 carriage / barrel / wheels / trunnion elevation / wheel spin |
| S2 | rec_cannon_0003 | `data/records/rec_cannon_0003/revisions/rev_000001/model.py:L75-L228` | 甲板立柱、叉形 yoke、竖直旋回和炮管俯仰 |
| S3 | rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9 | `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L63-L289` | naval carronade 的 deck mount、slide base、recoil prismatic、traverse/elevation joints |
| S4 | rec_cannon_1f8c2c130d494b8b8157820ebe8af346 | `data/records/rec_cannon_1f8c2c130d494b8b8157820ebe8af346/revisions/rev_000001/model.py:L102-L204` | siege mortar/trestle bed、宽短炮管、滑动 elevation wedge |

## 部件（Parts）
| 部件 | 描述 | 参数 | 来源 |
|---|---|---|---|
| support | required；承载炮管的主树根，按 `mount_layout` 生成 carriage / post_mount / slide_base / trestle_bed / barbette 中的一类 | mount_layout, support_width, support_length, support_height, material_style | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L175-L240`; S2 / `data/records/rec_cannon_0003/revisions/rev_000001/model.py:L75-L156`; S4 / `data/records/rec_cannon_1f8c2c130d494b8b8157820ebe8af346/revisions/rev_000001/model.py:L102-L134` |
| barrel | required；可俯仰炮管，包含 barrel shell、muzzle band/flare、breech/cascabel、left/right trunnion | barrel_profile, barrel_length, bore_radius, muzzle_flare, breech_style, trunnion_offset | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L242-L300`; S2 / `data/records/rec_cannon_0003/revisions/rev_000001/model.py:L158-L204`; S4 / `data/records/rec_cannon_1f8c2c130d494b8b8157820ebe8af346/revisions/rev_000001/model.py:L135-L175` |
| wheel_left / wheel_right | optional；轮式炮架的左右轮，连续旋转或 revolute 旋转 | wheel_count, wheel_style, wheel_radius, wheel_track | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L302-L349` |
| yoke_or_rotating_carriage | optional；立柱炮或甲板炮的旋回叉架/旋回炮架 | traverse_style, yoke_gap, yoke_height, traverse_range_deg | S2 / `data/records/rec_cannon_0003/revisions/rev_000001/model.py:L120-L156`; S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L83-L198` |
| recoil_carriage | optional；naval carronade 的滑动 carriage bed | recoil_enabled, recoil_travel, slide_rail_spacing | S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L83-L198` |
| elevation_wedge | optional；迫击炮/攻城炮中支撑炮尾的滑动楔块 | wedge_enabled, wedge_travel, wedge_profile | S4 / `data/records/rec_cannon_1f8c2c130d494b8b8157820ebe8af346/revisions/rev_000001/model.py:L176-L204` |
| trail_legs | optional；field/mountain artillery 的后部单 trail 或 split trail | trail_layout, trail_length, trail_spread_angle | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L212-L235` |

## 关节（Joints）
| 关节 | 类型 | axis | origin | range | 描述 | 来源 |
|---|---|---|---|---|---|---|
| barrel_elevation | revolute | `(0, -1, 0)` 或等价横向轮轴方向 | 炮管耳轴中心，位于炮架 cheeks/yoke 之间 | 建议 `[-0.18, 0.70]` rad；野战炮可收窄到 `[0, 0.26]` | required；炮管绕耳轴俯仰 | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L318-L331`; S2 / `data/records/rec_cannon_0003/revisions/rev_000001/model.py:L215-L228`; S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L276-L289` |
| left_wheel_spin / right_wheel_spin | continuous | `(0, 1, 0)` 或 wheel axle axis | 左右轮中心 | continuous | optional；轮式炮架车轮旋转 | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L332-L349` |
| traverse_joint | continuous / revolute | `(0, 0, 1)` | pedestal/post/deck ring 中心 | continuous 或 `[-55, 55]` deg | optional；甲板炮或 coastal gun 水平旋回 | S2 / `data/records/rec_cannon_0003/revisions/rev_000001/model.py:L206-L214`; S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L248-L261` |
| recoil_slide | prismatic | `(-1, 0, 0)` 或沿炮管后坐方向 | slide_base rail 起点 | `[0, 0.36]` m | optional；carronade carriage 沿滑轨后坐 | S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L262-L275` |
| wedge_slide | prismatic | `(1, 0, 0)` | trestle bed 中央 dovetail slot | `[0, 0.245]` m | optional；迫击炮 elevation wedge 前后滑动 | S4 / `data/records/rec_cannon_1f8c2c130d494b8b8157820ebe8af346/revisions/rev_000001/model.py:L196-L204` |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 默认值 | 派生关系 | 来源 |
|---|---|---|---|---|---|
| mount_layout | enum | `wheeled_field_carriage` / `deck_swivel_yoke` / `naval_slide` / `trestle_mortar` / `barbette_platform` | `wheeled_field_carriage` | 决定 support 子结构和可选 joint | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L175-L349`; S2 / `data/records/rec_cannon_0003/revisions/rev_000001/model.py:L75-L228`; S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L63-L289`; S4 / `data/records/rec_cannon_1f8c2c130d494b8b8157820ebe8af346/revisions/rev_000001/model.py:L102-L204` |
| barrel_profile | enum | `long_smoothbore` / `short_carronade` / `squat_mortar` / `tapered_swivel` | `long_smoothbore` | 派生 barrel_length、radius profile、muzzle/breech 比例 | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L242-L300`; S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L199-L246`; S4 / `data/records/rec_cannon_1f8c2c130d494b8b8157820ebe8af346/revisions/rev_000001/model.py:L135-L175` |
| barrel_length | float | `0.65-2.20` | `1.45` | 受 barrel_profile 约束 | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L242-L300` |
| wheel_style | enum | `none` / `solid_iron` / `spoked_wood` / `iron_shod_wood` | `spoked_wood` | mount_layout 非 wheel 时为 `none` | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L302-L349` |
| traverse_style | enum | `none` / `post_swivel` / `deck_ring` / `barbette_bearing` | `none` | deck/naval/coastal layouts 启用 | S2 / `data/records/rec_cannon_0003/revisions/rev_000001/model.py:L206-L214`; S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L248-L261` |
| recoil_enabled | bool | `true` / `false` | `false` | `mount_layout = naval_slide` 时默认 true | S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L262-L275` |
| wedge_enabled | bool | `true` / `false` | `false` | `mount_layout = trestle_mortar` 时默认 true | S4 / `data/records/rec_cannon_1f8c2c130d494b8b8157820ebe8af346/revisions/rev_000001/model.py:L176-L204` |
| elevation_range_deg | float pair | lower `-10-5`, upper `12-40` | `[-6, 22]` | 转为 barrel_elevation limits | S2 / `data/records/rec_cannon_0003/revisions/rev_000001/model.py:L215-L228`; S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L276-L289` |
| material_style | enum | `bronze_wood` / `black_iron_wood` / `weathered_iron` / `coastal_concrete` | `black_iron_wood` | palette 选择，不改变核心部件 | S1 / `data/records/rec_cannon_0004/revisions/rev_000001/model.py:L170-L174`; S3 / `data/records/rec_cannon_676f94a3eca04699a3b3ef0e6201ebb9/revisions/rev_000001/model.py:L27-L31` |

## 部件多样性审计（Part Diversity Audit）
| 部件类型 | observed_variation | 连续参数是否足够 | 是否需要离散参数 | 推荐参数 | 说明 |
|---|---|---|---|---|---|
| support / mount | wheeled field carriage / deck swivel yoke / naval slide / trestle mortar / barbette platform | no | yes | mount_layout | 这是拓扑差异，不能靠长宽高表达 |
| barrel | long smoothbore / short carronade / squat mortar / tapered swivel | no | yes | barrel_profile | 炮管轮廓、长径比和炮口/炮尾拓扑不同 |
| wheels | none / two large wheels / four-wheel carriage / iron/wood/spoked styles | no | yes | wheel_style, wheel_count | 是否有轮以及轮结构是类别布局差异 |
| traverse assembly | none / vertical post swivel / deck ring / barbette bearing | no | yes | traverse_style | 水平旋回关节和承载结构存在/不存在 |
| recoil slide | none / rail-guided carriage bed | yes for travel after enabled | yes | recoil_enabled | 有无后坐滑轨是拓扑差异；行程才是连续参数 |
| wedge / quoin | none / sliding wedge / fixed block | no | yes | wedge_enabled, wedge_profile | 楔块是独立活动件或支撑件，不能只用炮管角度替代 |
| decorative bolts/bands | band count / bolt count / straps | yes | no | detail_level | 主要是数量和尺寸变化，不作为核心 enum |

## 组合逻辑（Composition Logic）
1. 先根据 `mount_layout` 生成 root support：轮式 carriage、post mount、deck slide、trestle bed 或 barbette。
2. 生成 barrel，并把 trunnion origin 对齐到 support 的 cheek/yoke/cradle 中心。
3. 所有布局都创建 `barrel_elevation`；wheel、traverse、recoil、wedge joints 只在对应布局启用。
4. 非活动的 bands、bolts、straps、muzzle rings、plank seams 用 parent visual 挂载。
5. 若 `mount_layout = naval_slide`，父子链应为 deck_mount -> slide_base -> recoil_carriage -> barrel。
6. 若 `mount_layout = deck_swivel_yoke`，父子链应为 mount -> yoke -> barrel。

## 已有模板写法参考
revolute_hinge / continuous_wheel / prismatic_slide / rotary_post / telescoping_tube

## 约束
- 必须至少有一个 `barrel` part 和一个 `barrel_elevation` revolute joint。
- 炮管俯仰轴必须穿过左右 trunnion 或 yoke 支承点，不能在炮管中心随意放置。
- 炮口、炮尾、cascabel/breech 和左右 trunnion 应跟随 barrel part 移动。
- 轮式布局的左右轮必须位于 carriage 两侧且轴线共线。
- `recoil_slide` 启用时 carriage bed 必须被 slide rails 捕获，滑到极限也不能脱轨。
- `traverse_joint` 启用时旋回轴必须接近世界 Z 轴。
- `wedge_slide` 启用时 wedge 必须位于 barrel breech 下方且在 bed 槽内。

## Validator
| 检查项 | 标准 |
|---|---|
| required parts | 存在 support/root 和 `barrel` |
| barrel joint | 恰好至少 1 个 revolute `barrel_elevation` |
| trunnion axis | barrel_elevation axis 为横向轴，且 origin 接近 trunnion visuals |
| wheel consistency | `wheel_style != none` 时左右轮存在且有 spin joint |
| traverse consistency | `traverse_style != none` 时有 Z 轴 traverse joint |
| recoil consistency | `recoil_enabled` 时有 prismatic recoil joint，axis 与 slide rails 平行 |
| wedge consistency | `wedge_enabled` 时有 wedge part 和 prismatic wedge joint |
| no floating | barrel、wheels、yoke、slide carriage、wedge 全部连接主树 |
| part diversity | `mount_layout` 和 `barrel_profile` 两个离散参数必须存在 |
| identity | 视觉上必须是火炮而不是普通车辆、管道或支架 |

## Reject cases
- 没有可俯仰炮管。
- 炮管没有炮口/炮尾/耳轴特征，看起来像普通圆管。
- `barrel_elevation` 轴线不穿过耳轴或 yoke。
- 轮式炮架的轮子漂浮或不在两侧。
- 后坐滑轨启用但 carriage 没有被 rail 支撑。
- 旋回炮启用但没有竖直 traverse joint。
- 迫击炮/trestle layout 中 wedge 漂浮或不在炮尾下方。
- 装饰件被创建成无意义独立活动 part，破坏主树。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | SPEC_ONLY_DRAFT；等待人工审核 |
