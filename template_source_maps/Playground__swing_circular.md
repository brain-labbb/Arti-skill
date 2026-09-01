# Playground / swing circular — template source map

slug: `circular_ring_swing`
pattern: frame + 2-joint chain (fixed overhead frame → REVOLUTE canopy swing on the hanger → CONTINUOUS spin of a circular ring seat; suspension = 1/3/4-point bridle)

**HALO INVARIANT (category identity — never violated by any candidate):** the seat is a
*circular ring you sit INSIDE*. The ring is always an **open hoop/halo**; its interior is never
filled, decked, disced, or netted over. The wooden seat only lines the **inner lower arc** of the
ring, leaving the whole centre open so a person sits within the ring with clear leg/body space.
Slot B candidates differ only in ring *profile* and *seat-arc depth* — all stay open hoops. (A
solid disc platform or a rope/net saucer is OUT OF CATEGORY and must never be a candidate.)

parents (1 — a circular ring swing: a ring seat hangs from an overhead frame and both swings fore/aft and spins about its own axis):
- rec_model-a-modern-circular-ring-swing-hanging-from-_20260610_085427_611739_83a6bd25 ← picture/Playground/swing circular — `circular_ring_swing_pergola`; 2-post steel pergola_frame (2 posts + top_rail + 34 louver `fin_i` + gussets + 16 rivets); single centerline hanger_chain (anchor_stem + anchor_eye + 4 `chain_link_li` + hook_eye + hook); ring_seat (ring_eyelet + hanger_clevis + 2 `hoop_hi` chrome torus R=0.725 + 22 `seat_plank_si` wooden bench arc on the lower inner ring, open centre); 2 joints: REVOLUTE `canopy_swing` (frame→chain, axis (1,0,0), +/-45deg, z=2.55) + CONTINUOUS `ring_spin` (chain→ring, axis (0,0,1), z=-0.576). fills SlotA `two_post_pergola`, SlotB `bench_ring_seat`, suspension `single_chain`. converged (parent)

## Slot 候选覆盖

### Slot A:support_frame(固定上方支架 — 承 canopy_swing 顶吊点)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| two_post_pergola | rec_model-a-modern-circular-ring-swing-hanging-from-_20260610_085427_611739_83a6bd25 | post(x2) / top_rail / fin_i / gusset | 双柱百叶钢凉棚 | converged (parent) |
| a_frame_support | rec_pcrs_var_aframe | aframe_support / leg(x4 splayed) / ridge_beam / apex_plate_i / foot_plate_i | A 字撇腿对 + 顶脊梁,链从脊梁垂吊 | converged |
| single_arch | rec_pcrs_var_arch | arch_frame / arch_beam(swept) / crown_bracket / foot_plate_i | 单高拱跨越,冠顶 bracket 垂吊链 | converged |

### Slot B:ring_seat(可坐圆环座 — 整体随 CONTINUOUS ring_spin 转;**永远是开口圆环 halo**)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| bench_ring_seat | rec_model-a-modern-circular-ring-swing-hanging-from-_20260610_085427_611739_83a6bd25 | hoop_hi(x2 torus) / seat_plank_si(22, θ197°–343°) | 双铬圆管环 + 下内弧木板凳(开口) | converged (parent) |
| deep_cradle_ring | rec_pcrs_var_cradle | hoop_hi(x2 torus) / seat_plank_si(32, θ163°–377° ≈214°弧) | 双铬环 + 更深木摇篮弧(板条爬高成低靠背),中心仍全开 | converged |
| flat_band_ring | rec_pcrs_var_band | flat_band(CadQuery 环形带 annulus R0.75/壁5mm/宽110mm) / seat_plank_si(22) | 单宽扁钢带环(空心 annulus) + 下内弧木板凳,中心全开 | converged |

> 三个 Slot B 候选都是开口 halo:parent 圆管双环、cradle 更深弧靠背、band 扁带单环。
> 差异在环截面(圆管 ×2 ↔ 扁带 ×1)与座弧深度(146° ↔ 214°),均不填充圆心。

## Multiplicity / Copy Logic
- count_param: `suspension_point_count`(吊点 N) — parent N=1(单链);variant 扩 N ∈ {3, 4}(bridle)
- N 样本已覆盖:{1(parent), 3, 4} → rec_model-a-modern-circular-ring-swing-hanging-from-_20260610_085427_611739_83a6bd25 / rec_pcrs_var_p3 / rec_pcrs_var_p4;模板建议 N_range [1, 4]
  - rec_pcrs_var_p3:`BRIDLE_N=3`,anchors 30°/90°/150°,`bridle_anchor_i` + `bridle_chain_i` for-loop 汇聚到 swivel
  - rec_pcrs_var_p4:`BRIDLE_N=4`,等角 anchors,`bridle_anchor_i` + `bridle_chain_i` for-loop → convergence_eye / swivel_barrel
- 次级 count:seat_plank_count(凳板,parent=22、cradle=32)、fin_count(百叶,parent=34) — 采样器扫
- copied object / naming / placement / joint policy:
  - copied object:吊链 `chain_link_li` / bridle 链 `bridle_chain_i` / bridle 锚 `bridle_anchor_i` / 凳板 `seat_plank_si` / 百叶 `fin_i`
  - naming:`for i in range(suspension_point_count)` + `f"bridle_chain_{i}"`;rim 锚点等角
  - placement:bridle 链从环缘 N 等角点汇聚到顶 swivel;凳板沿环下内弧等角
  - joint policy:2 关节链 — frame→hanger REVOLUTE `canopy_swing`(轴 (1,0,0),±45deg)+ hanger→ring CONTINUOUS `ring_spin`(轴 Z);bridle 多链汇聚仍单 swivel(无闭环)

## 排除项(未来 compatibility matrix 素材)
- **solid_disc_platform / net_saucer 永久排除**:填满圆心的实心圆盘或绳网碟座违反 HALO INVARIANT(人要坐进环里、留出空间),出类目。早期 var_disc / var_net 因此删除,不再作为候选。
- suspension_point_count N 不专门多 fork:parent N=1 + var_p3/var_p4 三档 → 采样器。
- seat_plank / fin 计数交采样器(parent loop-emit 22/34、cradle 32)。
- 跨轴组合(a_frame × flat_band × p4)交模板采样器。
- bridle 多链不展开为闭环约束(保持单 swivel 汇聚 idiom)。
- color / material / 比例不是结构轴。

---
组合预审:Slot A 3 候选 × Slot B 3 候选 = 9 slot 组合 × N 样本 3(1/3/4)= 27 ≥ 10 ✓

6 个 variant 填格:
- var_aframe → SlotA `a_frame_support`
- var_arch → SlotA `single_arch`
- var_cradle → SlotB `deep_cradle_ring`(halo,深摇篮弧)
- var_band → SlotB `flat_band_ring`(halo,扁带单环)
- var_p3 → suspension_point_count N=3
- var_p4 → suspension_point_count N=4
