# Powertools / drill — template source map

pattern: mixed(mechanism slots A/B + 两条 multiplicity 子轴 N1=chuck jaws / N2=clutch detents)
parents: rec_model-a-handheld-cordless-drill-driver-about-0-2_20260610_085501_032015_611bf0fa ← picture/Powertools/drill/（handheld cordless drill-driver；housing root，`chuck` CONTINUOUS + `clutch_collar` REVOLUTE + `trigger` REVOLUTE + `selector` PRISMATIC + `battery_pack` PRISMATIC = 5 非 fixed；chuck `jaw_{i}` loop n=3，clutch detent loop n=12）

## Slot 候选覆盖

### Slot A:body_form（机身形态）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pistol_grip | rec_model-a-handheld-cordless-drill-driver-about-0-2_20260610_085501_032015_611bf0fa | `housing` / `trigger`(revolute) + `chuck`(continuous) | 标准手枪握把：水平筒身 + 后倾握把 | converged(parent) |
| right_angle | rec_cordless_drill_var_rtangle | `housing`(L 形) / `chuck`(continuous, X 轴) | 横向 gearbox head(X) + 竖向 motor body(Z)，chuck 轴与机身轴垂直 | converged(workbench, rating pending sync) |
| t_handle_compact | rec_cordless_drill_var_thandle | `housing`(T 形) / `trigger`(revolute) | 短粗筒身 + 正下方居中竖握把 → T 形剪影(subcompact) | converged(workbench, rating pending sync) |

### Slot B:battery_mount（电池安装）
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| slide_on_stick | rec_model-a-handheld-cordless-drill-driver-about-0-2_20260610_085501_032015_611bf0fa | `battery_pack` / slide(prismatic) | 握把底插杆式滑入电池 | converged(parent) |
| flat_pod_slide | rec_cordless_drill_var_podbatt | `battery_pack` / slide(prismatic, dovetail) | 握把下扁平 pod 电池，沿燕尾平台滑入 | converged(workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param N1 = chuck jaw count via `jaw_{i}` loop（jaw BoxGeometry，绕轴 `i*2π/N` 等分）
- N1 样本已覆盖: {3} → parent ; {2} → rec_cordless_drill_var_jaw2（rotate_x i*2π/2，self-centering 双爪）
- N1 模板建议 N_range: [2, 3]（4-jaw = 车床卡盘，出类目）
- count_param N2 = clutch detent count via clutch collar tick loop（同步带 collar facet loop）
- N2 样本已覆盖: {12} → parent ; {16} → rec_cordless_drill_var_clutch16（facet range(24)/detent range(16)）; {20} → rec_cordless_drill_var_clutch20（facet range(30)/detent range(20)）
- N2 模板建议 N_range: [10, 24]（真实扭矩档位档数）
- copied object / naming / placement / joint policy: jaw = chuck 上等分 BoxGeometry，FIXED 随 chuck CONTINUOUS 转；detent tick = clutch_collar 圆周等分刻度，随 collar REVOLUTE 转；两 loop 母资产已存在，候选仅改 count

## 排除项(未来 compatibility matrix 素材)
- 4-jaw chuck：车床/钻床卡盘，出 Powertools/drill 类目，N1 上限封 3
- keyed chuck：需参考图，暂折入（无独立候选）
- hammer/gearbox mode + selector style：连续/装饰，非候选轴
- chuck CONTINUOUS spin：跨所有候选保留，不改 joint 类型
