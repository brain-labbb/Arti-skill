# Others / Matchbox — template source map

pattern: mixed（固定 named slots:closure 根机构 slot（sleeve+tray / body+lid / cover+flap）+ match_arrangement + striker_style;外加一根 multiplicity 轴 = match_count N，火柴子件 `match_{i}` 按 N 循环复制并全部 FIXED 在托盘/盒体上。）

parents（1 个母资产，经典抽屉式安全火柴盒）:
- P1 rec_model-a-classic-safety-matchbox-a-cream-beige-ca_20260610_085134_048861_a0e600f3 ← picture/Others/Matchbox/001.png（cream 卡纸外套 sleeve（两端开口 hollow shell：top/bottom panel + 2 side wall + 2 striker strip + 顶面印刷边框）+ 内滑托盘 tray（floor + 4 wall，开顶 hollow），`sleeve_to_tray` PRISMATIC 沿长轴半开姿态；10 根火柴 `match_{i}` 平躺 FIXED 在 tray、头朝开口端；2 根散落火柴 `ground_match_{i}` FIXED 在 sleeve 前方地面。1 非 fixed joint。**全批 fork 母资产**）

批次：others_matchbox_qwen37max_20260620（dashscope qwen3.7-max / medium）。7 变体全部 compile=success（--validate run_tests 通过）、workbench-only（collections=['workbench']、category_slug=None）、≥1 非 fixed joint、单轴控制变量、仍明确读作火柴盒。

## 组合数预审

closure 3 × match_arrangement 2 × striker_style 2 = **12 ≥ 10** ✓（未计 N）。

## Slot 候选覆盖

### Slot A:closure（主机构槽——开盒机构 / 盒体形态）
| 候选(未来 module) | record_id | 关键 part/joint/helper | 结构特征 | 状态 |
|---|---|---|---|---|
| drawer_slide（基线） | P1 | `sleeve` 根 + `tray` 子；`sleeve_to_tray`(PRISMATIC, +X, ±0.020 半开)；`_add_match_visuals` helper；striker `striker_{i}`、印刷边框 inline 在 sleeve | 套筒 + 内滑抽屉托盘，沿长轴抽拉 | parent |
| flip_lid | rec_matchbox_var_flip_lid | `body` 根（开顶托盘 shell 装火柴）+ `lid` 子；`body_to_lid`(REVOLUTE，沿长轴后上缘，rest≈50° 开)，铰链 knuckle 可见；边框移到 lid 顶面 | 一体盒 + 后缘铰链翻盖 | converged（已同步） |
| matchbook | rec_matchbox_var_matchbook | `cover` 根（back+front 折叠纸皮）+ `flap` 子；`cover_to_flap`(REVOLUTE，顶部折线，rest≈70° 开)；内含 paper-match 梳 `match_{i}`(N=10，扁平纸火柴立在 base strip 上，`for i in range(N_MATCHES)`)；striker 移到 front panel | 对折书式纸火柴皮（book of matches） | converged |

> **closure=matchbook 的耦合说明**（供下游 compatibility matrix）：matchbook 形态天然携带自己的火柴子模块——扁平纸火柴梳（竖立在公共 base strip 上），而非 drawer/flip 的圆截面木棍平躺。这是该 closure 候选的固有部件（书式火柴的定义），故 matchbook 同时改了 closure 与 match 子模块的部件类型；模板侧应把「paper-match comb」绑定为 matchbook closure 的 native fill，而 drawer/flip 用 wooden-stick fill。其余两轴（match_arrangement / striker_style）仅在 drawer/flip 系下自由组合。

### Slot B:match_arrangement（盒内火柴排布）
| 候选 | record_id | 关键 part/joint/helper | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_row（基线） | P1 | `_add_match_visuals`（stick Box + ellipsoid head mesh）；`match_{i}` 沿 +X 平躺、Y 等距 pitch、头朝开口端；FIXED `tray_to_match_{i}` | 平躺单层一排，头朝开口 | parent |
| standing_bundle | rec_matchbox_var_standing | `match_{i}`（i∈range(N_MATCHES=16)）竖立（长轴沿 Z）、头朝上、规则矩形 grid 排布在 tray 腔内、头部出 rim；FIXED `tray_to_match_{i}` | 竖插成束（满盒立放观感） | converged |

### Slot C:striker_style（擦火面布局）
| 候选 | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| both_long_sides（基线） | P1 | `striker_{i}`(i∈0..1) 各贴一条长侧壁外面、proud 0.0002 | 两长侧各一条擦火条 | parent |
| one_side_plus_top_patch | rec_matchbox_var_striker_top | 单条擦火条仅贴一长侧；顶面 sleeve panel 旁印刷一块 striker patch；对侧长壁留素 cream | 单侧条 + 顶面擦火块 | converged |

## Multiplicity / Copy Logic
- **count_param: `match_count`**（托盘内火柴根数 N）。火柴是该小类唯一的「同构子件 × N」复制层。
- N 样本已覆盖: {6, 10, 16, 24} → rec_matchbox_var_n6 / P1（基线 10，平躺单层）/ rec_matchbox_var_n16（平躺单层加密）+ rec_matchbox_var_standing（竖立 16）/ rec_matchbox_var_n24（**双层** STICKS_PER_LAYER×NUM_LAYERS=24）。
- 模板建议 N_range: [4, 40]（采样域远大于样本是正常的；满盒木火柴常 20–45 根，纸火柴梳常 10–30 根）。N 撑大时单层放不下要切换双层（见 n24 的 NUM_LAYERS 逻辑）或减小 MATCH_PITCH。
- copied object / naming / placement / joint policy:
  - copied object：单根火柴（木棍 Box `stick` + 椭球头 mesh `head`，经 `_add_match_visuals` 共享 helper；matchbook 系为扁平纸 tab + head）。
  - naming：`match_{i}`（盒内）、`ground_match_{i}`（散落，固定 2 根）。
  - placement：flat_row 沿 +X 平躺、Y 等距 pitch（n24 双层时 row=i//STICKS_PER_LAYER、col=i%STICKS_PER_LAYER）；standing_bundle 竖立矩形 grid（行列由 i 推导）。
  - joint policy：**全部 FIXED** 到 tray/body（火柴本身不活动）；唯一活动关节是 closure（PRISMATIC 抽屉 / REVOLUTE 翻盖或书皮）。散落 ground_match 以 `allow_isolated_part` 显式放行（前方地面孤立件，继承 parent）。

## 排除项（未来 compatibility matrix 素材）
- 无连续不收敛格子：7 个计划格子（flip_lid / matchbook / standing / striker_top / n6 / n16 / n24）全部一次 fork 收敛。母资产已满足 §4 可读性契约（火柴 `for i in range(10)` 循环发射、装饰 inline 为 visual、striker/边框非独立 part），multiplicity 变体直接继承循环写法（n24 进一步参数化为 NUM_LAYERS×STICKS_PER_LAYER）。
- 兼容性提示：matchbook closure 与 match_arrangement / striker_style 两槽不自由组合（matchbook 自带 paper-comb fill 与 front-panel striker，见上 closure 耦合说明）；drawer_slide 与 flip_lid 两个 closure 与 flat_row/standing_bundle、both_sides/one_side+top 自由组合。standing_bundle 下火柴高出 tray rim 较多（满盒立放正常观感），模板若与极浅 tray 组合需校验头部不穿 lid（flip_lid+standing 组合时）。
