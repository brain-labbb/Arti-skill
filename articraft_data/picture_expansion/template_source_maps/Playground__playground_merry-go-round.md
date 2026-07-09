# Playground / playground merry-go-round — template source map

slug: `playground_merry_go_round`
pattern: radial-cage (single fixed center post + one CONTINUOUS Z cage_spin; the rideable cage is a loop-emitted lattice of latitude rings + N meridian arc planes)

parents (1 — a spherical orbiting merry-go-round: a center post carries a spherical hoop cage that spins on a single continuous Z axis; fully loop-clean):
- rec_model-a-spherical-playground-merry-go-round-orbi_20260610_085349_979414_2229be74 ← picture/Playground/playground merry-go-round — `spherical_merry_go_round`; white square Box post_column + base_plate + journal_upper/lower; hoop_cage = collar_upper/lower lathe sleeves + 4 Torus latitude rings + 6 `meridian_arc_k` candy-striped spline tubes (N_MERIDIAN_PLANES=3) + 24 meridian_stripe + 24 clamp; single CONTINUOUS `cage_spin` (axis Z, origin z=1.10). fills SlotA `square_box_post`, SlotB `spherical_hoop_cage`. converged (parent)

## Slot 候选覆盖

### Slot A:base_post(固定中心支柱 — 承 cage_spin 轴承)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| square_box_post | rec_model-a-spherical-playground-merry-go-round-orbi_20260610_085349_979414_2229be74 | post_column / base_plate / journal_upper-lower | 方柱中心 + 上下轴颈 | converged (parent) |
| round_turned_post | rec_pmgr_var_roundpost | post_column(lathe) | 圆车削柱:base flange→锥轴→journal | workbench (pending sync) — EMPTY cell |
| tripod_stand | rec_pmgr_var_tripod | tripod_leg_i(loop3) / hub | 三外撇腿落地 + 中央毂轴承 | workbench (pending sync) — EMPTY cell |

### Slot B:cage_form(可坐笼体 — 整体随 CONTINUOUS spin 转)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| spherical_hoop_cage | rec_model-a-spherical-playground-merry-go-round-orbi_20260610_085349_979414_2229be74 | ring_upper/equator/yellow/lower / meridian_arc_k | 整球:纬向 Torus 环 + 经向条纹弧管 | converged (parent) |
| half_dome_cage | rec_pmgr_var_dome | dome latitude rings / meridian_arc_k / floor_ring | 半球穹顶笼坐落地环上 | workbench (pending sync) — EMPTY cell |
| cylindrical_drum_cage | rec_pmgr_var_drum | top_ring / bottom_ring / bar_i(loop) | 上下等径环 + 竖直栏杆鼓笼 | workbench (pending sync) — EMPTY cell |

## Multiplicity / Copy Logic
- count_param: `n_meridian`(经向平面 N) — parent N=3;variant 扩 N ∈ {2, 4}
- N 样本已覆盖:{2(var_n2), 3(parent), 4(var_n4)};模板建议 N_range [2, 6]
- 次级 count:latitude ring count(纬环档,parent=4),drum bar count(竖栏) — 采样器扫
- copied object / naming / placement / joint policy:
  - copied object:经向弧 `meridian_arc_k` + `meridian_stripe` + `clamp`;纬向 Torus 环;drum 竖栏 `bar_i`
  - naming:`for k in range(n_meridian)` + `f"meridian_arc_{k}"`;角度 `pi*k/n_meridian`
  - placement:经向弧绕 Z 等角;纬环按纬度堆叠;drum 栏绕圆周等角
  - joint policy:1 个 post→cage CONTINUOUS spin(轴 Z,origin 柱顶 journal);笼内全 fixed 到 cage,整笼单刚体旋转

## 排除项
- n_meridian N 不专门多 fork:parent N=3 + var_n2/var_n4 已三档 → 计数轴交采样器。
- 跨轴组合(round_post × half_dome × N4)交模板采样器。
- color / candy-stripe 配色 / 比例不是结构轴。

---
6 个 variant 填格:
- var_roundpost → SlotA `round_turned_post`(EMPTY)
- var_tripod → SlotA `tripod_stand`(EMPTY)
- var_dome → SlotB `half_dome_cage`(EMPTY)
- var_drum → SlotB `cylindrical_drum_cage`(EMPTY)
- var_n2 → n_meridian N=2
- var_n4 → n_meridian N=4
