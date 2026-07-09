# Door / Folding gate — template source map

> 结构家族 = 伸缩/手风琴折叠安全门 (concertina trellis security gate)。固定矩形钢框 (root)
> 携带一条折叠剪式格栅:N 条竖向 picket 由单一 REVOLUTE "fold" 主驱动 + 全 mimic 耦合链
> 像手风琴一样整体展开/收拢。装饰菱形 strap 各自绕铆钉 mimic ±1 摆动。
> SLUG: folding_gate  SHARD: Folding_gate

pattern: mixed（固定 named slots:lattice_pattern + kinematics + endpost_lock + guide_track;外加
multiplicity 轴 N = 折叠 cell 数 / picket 数）

parents（1 个母资产,唯一;基线 = single_diamond × linear_concertina × mid_latch_knob × dual_track × N=10cells）:
- rec_door_folding_gate ← picture/Door/Folding gate/001.png（terracotta-salmon 钢制伸缩门:固定 frame
  part `frame`(stile_a/stile_b/head_rail/sill_rail/top_track/bottom_track) + 10 cell 剪式格栅 +
  4 banded 菱形 trellis(`asc_{c}_{ri}` / `desc_{c}_{ri}`) + load lambda 链(`lam_up_{c}` /
  `lam_dn_{c}`) + 单 fold REVOLUTE 主驱动(`fold`)+ `lam_elbow_{c}` -2q + 全 mimic 链;上下双导轨 +
  右 stile 中部 `latch_plate` / `latch_knob`;q=0 = 完全展开 hero pose,正向折叠收向左 stile。
  **single-diamond / 线性 concertina / 双导轨 / mid-latch-knob 基线**）

readability：parent 已是 loop-emitted（`for i in range(N_VERTS)` picket、`for c in range(N_CELLS)`
scissor cell、`for ri, z_bot in enumerate(ROW_BOTTOMS)` strap row),含共享 helper `lam_bar()` 与行内
strap 发射、规则放置、统一 mimic 耦合 REVOLUTE joint policy。**符合 readability contract,无需 loop 重写。**
任何 multiplicity 变体只改 `N_CELLS` 常量即可由现有 loop 重新发射(已验证:ncells5 / ncells16)。

## 组合数预审（HARD GATE）

lattice_pattern 3 × kinematics 2 × endpost_lock 2 × guide_track 2 = 24；再叠 multiplicity
N∈{5,10,16}（3 个 distinct N）→ 24 × 3 = **72 ≫ 10 ✓**。
（即便只取 lattice 3 × distinct N 3 = 9 也已逼近 10;轴含 REVOLUTE-scissor ↔ 交替 ±2 REVOLUTE-accordion

GATE P1：每个 slot ≥2 候选(含基线);multiplicity 3 个 distinct N;combo = 24 × 3 = 72 ≥ 10。
**全部由盘上已收敛资产满足 —— 本次 gap-fill 新增 fork = 0。**

## Slot 候选覆盖

### Slot A:lattice_pattern（格栅 cell 填充样式 —— 视觉/拓扑槽）
| 候选（未来 module） | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_diamond（基线） | rec_door_folding_gate | `asc_{c}_{ri}` / `desc_{c}_{ri}` + `cross_rivet` | 每 cell 4 行交叉 strap,banded 菱形 trellis | parent |
| bare_pantograph | rec_folding_gate_var_picket_pantograph | `xup_{c}_{ri}` / `xdn_{c}_{ri}` 全长交叉 bar(无 diamond infill) | 去装饰菱形 strap,每 cell 仅裸两杆 pantograph X | converged |
| solid_panels | rec_folding_gate_var_accordion_panels | `panel_i`.leaf(整片 Box leaf) | 每 cell 一块整片实心 leaf 取代菱形 mesh,隐私折叠 | converged（同时承载 Slot B accordion_hinge — 见下） |

### Slot B:kinematics（折叠运动学 —— 主机构槽）
| 候选 | record_id | 关键 joint | 结构 | 状态 |
|---|---|---|---|---|
| linear_concertina（基线） | rec_door_folding_gate | `fold` REVOLUTE + `lam_elbow_{c}`(-2q) + 1:-2:1 mimic 链 | 剪式 pantograph 沿轨道线性伸缩,picket 始终竖直水平 | parent |
| accordion_hinge | rec_folding_gate_var_accordion_panels | `fold` + `hinge_{i}` REVOLUTE,multiplier 交替 ±2 | 刚性 leaf 沿竖轴(0,0,1)piano-hinge zigzag 折叠成平叠,单驱动 + 交替轴 mimic | converged |

### Slot C:endpost_lock（端柱/锁定样式）
| 候选 | record_id | 关键 joint | 结构 | 状态 |
|---|---|---|---|---|
| mid_latch_knob（基线） | rec_door_folding_gate | `latch_plate` / `latch_knob`（frame 上静态 parent visual） | 右 stile 中部静态 keeper plate + catch knob | parent |
| drop_bolt | rec_folding_gate_var_dropbolt_post | `bolt_slide` PRISMATIC(沿 z 落入 `floor_socket`) | 前导高 end-post + 竖向 drop-bolt 滑入地面 socket 锁定 | converged |

### Slot D:guide_track（导轨样式）
| 候选 | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| dual_track（基线） | rec_door_folding_gate | `top_track` + `bottom_track` + `sill_rail` | 上下双导轨 + sill rail,落地式 | parent |
| top_track_only | rec_folding_gate_var_top_track_only | `top_track`(仅顶轨;无 sill_rail / bottom_track) | 去 sill rail + 地轨,顶轨吊挂 ceiling-hung,picket 离地悬空 | converged |

### Slot E（多重性）:cell_count N（折叠 cell 数 / picket 数）
| N (cells) | record_id | 结构 | 状态 |
|---|---|---|---|
| 5 | rec_folding_gate_var_ncells5 | 短门 5 cell / 6 picket,`N_CELLS=5` for-loop 重新发射 | converged |
| 10（基线） | rec_door_folding_gate | 10 cell / 11 picket | parent |
| 16 | rec_folding_gate_var_ncells16 | 长门 16 cell / 17 picket,`N_CELLS=16` for-loop 重新发射 | converged |

## Multiplicity / Copy Logic
- count_param: **`N_CELLS`**（`N_VERTS = N_CELLS + 1` picket;strap/lambda 随之）
- N 样本已覆盖: {5, 10, 16}（3 个 distinct N,基线 10）→ ncells5 / parent / ncells16
- 模板建议 N_range: [4, 20]（现实伸缩门 cell 数范围;大 N 仍由 cos-pitch 几何安全构造,样本不必铺值域）
- copied object: 一个 scissor cell（picket + lambda 二杆 `lam_up_{c}`/`lam_dn_{c}` + banded strap
  `asc_{c}_{ri}`/`desc_{c}_{ri}`）共享 helper `lam_bar()` 发射
- naming: `picket_{i}` / `lam_up_{c}` / `lam_dn_{c}` / `asc_{c}_{ri}` / `desc_{c}_{ri}`,
  `for c in range(N_CELLS)`
- placement: 沿 X 等距 picket pitch（展开 PITCH_OPEN,折叠按 2*S_HALF*cos(THETA0+q) 收缩）
- joint policy: 单 fold REVOLUTE 主驱动 + 全 mimic（1 : -2 : 1 lambda + ±1 strap）耦合,整门一动

## 格子覆盖（Phase 0/1 实况）

| 槽 | 候选数(含基线) | 已收敛变体 | 状态 |
|---|---|---|---|
| A lattice_pattern（视觉/拓扑） | 3 | bare_pantograph(picket_pantograph) / solid_panels(accordion_panels) | converged ×2 |
| B kinematics（主机构） | 2 | accordion_hinge(accordion_panels) | converged ×1 |
| C endpost_lock | 2 | drop_bolt(dropbolt_post) | converged ×1 |
| D guide_track | 2 | top_track_only | converged ×1 |
| E cell_count（multiplicity） | N∈{5,10,16} | ncells5 / ncells16 | converged ×2 |

盘上已收敛变体 = 6（picket_pantograph、accordion_panels、dropbolt_post、top_track_only、ncells5、
ncells16）+ parent 基线。每槽 ≥2 候选 + multiplicity 3 个 distinct N。Combo = 24 × 3 = 72 ≫ 10。
**GATE P1 已满足;本次 gap-fill 新增 fork = 0。**

> 注:accordion_panels 一格双覆盖 —— 它既把菱形 mesh 换成实心 leaf(Slot A solid_panels),又把线性剪式
> 伸缩换成交替 ±2 竖轴 piano-hinge zigzag(Slot B accordion_hinge)。这是 fork prompt(“flat solid
> bi-fold accordion panels ... zigzag-folds the leaves through alternating mimic-coupled hinges”)
> 本身的复合改动;两槽各自仍 ≥2 候选,故不影响 GATE P1。下游写 spec 时若要把 A/B 解耦,可考虑各补一个纯
> solid_panels(保留线性剪式)与纯 accordion(保留菱形 leaf)的拆分候选,但 P0/P1 门控不要求。

## 排除项（dropped axes）
- **picket cross-section / 框宽 / 整体比例**:纯 scale,模板缩放负责,非结构轴 —— 丢弃。
- **color / material**:禁止作为变化轴 —— 丢弃（允许叠在结构变化之上）。
- **track 弯曲(curved track 转角门)**:运动学上需非线性轨道求解,超出单层结构 fork 范围 —— 本批不收。
- **top_latch_pin / double_diamond / ncells8 等早期 planned 候选**:GATE P1 已由盘上 6 变体满足,
  无需再造;若下游 spec 需要更密 lattice 或更多 distinct N,可在 P2+ 增补,非本次 gap-fill 范围。
