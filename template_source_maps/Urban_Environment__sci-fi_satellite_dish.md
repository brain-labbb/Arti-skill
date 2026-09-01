# Urban Environment / sci-fi satellite dish — template source map

slug: satellite_dish
shard: sci-fi_satellite_dish
identity: sci-fi communications satellite dish comm unit on an articulated az-el gimbal mount — greebled dark-matte equipment base box (z=0) → azimuth REVOLUTE (+Z) → yoke → elevation REVOLUTE (horizontal axis) → concave reflector with a feed at the prime focus. The dish must AIM via these two real joints; matte-black-with-glowing-accents sci-fi material read is part of the identity. Aiming REVOLUTE joints are preserved in EVERY variant.

pattern: parallel_children(固定 named slots: dish_form + mount_gimbal + feed + panel_segment multiplicity N; az/el 双 REVOLUTE 永远保留为骨架，不作为可变轴)

parents:
- rec_sci-fi-satellite-dish-comm-unit-a-dark-matte-rec_20260612_113210_418481_e7ad375a ← picture/Urban Environment/sci-fi satellite dish/001.png
  (dark matte sci-fi satellite dish comm unit: greebled equipment box [teal slat-grille vent VentGrilleGeometry + glowing teal edge accents + warning-triangle tube greebles + DATA LINK PANEL lime label bars + amber port-light rack + pedestal post/azimuth bearing] → azimuth_yoke single rear leaning post + pivot knuckle → dish_assembly: true LatheGeometry concave parabolic shell + glowing lime rim torus + back hub + neck + trunnion + center-fed ConeGeometry feed horn on axial boom at focus.
   占格 = dish_form=round_parabolic × mount_gimbal=az_el_fork_yoke × feed=center_fed_horn × N=round-lathe-smooth-shell[no loop panels])

## 组合数预审 (HARD GATE)

组合数预审: Π(dish_form 4 × mount_gimbal 3 × feed 3) × distinct-N {8,16,24} = 36 × 3 = 108 ≥ 10 ✓
(即使忽略 N，纯结构组合 4×3×3 = 36 ≥ 10 ✓。每槽 ≥ 2 候选满足。)

## Slot 候选覆盖

### Slot A: dish_form (反射面形态——拓扑差异最大；loop-emit panel_{i})
| 候选 (future module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_parabolic (基线-parent) | rec_sci-fi-satellite-dish-comm-unit-a-dark-matte-rec_20260612_113210_418481_e7ad375a | dish_assembly / reflector_shell(LatheGeometry.from_shell_profiles) / dish_rim(TorusGeometry) | 平滑一体化凹抛物面碗 (无分段 panel) | converged(parent) |
| hex_faceted | rec_satellite_dish_var_hex_faceted | dish_assembly / panel_{i}(hex facets concentric rings) | 六边形蜂窝镜面平铺近似抛物面 | converged |
| segmented_petal | rec_satellite_dish_var_segmented_petal | dish_assembly / panel_{i}(radial petals) + 每瓣 mount bolt loop | 放射状花瓣分段 (deployable space antenna 风) | converged |
| flat_phased_array | rec_satellite_dish_var_flat_phased_array | dish_assembly / panel_{i}(square radiator grid) | 平板相控阵 (无凹面、无前置 feed horn；平板面即 aperture) | converged |

### Slot B: mount_gimbal (云台/支撑结构；双 REVOLUTE 不变)
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| az_el_fork_yoke (基线-parent) | rec_sci-fi-satellite-dish-comm-unit-a-dark-matte-rec_20260612_113210_418481_e7ad375a | azimuth_yoke / yoke_post(单后臂) / yoke_knuckle / azimuth_rotation(REVOLUTE +Z) + elevation_tilt(REVOLUTE -Y) | 单后臂前倾入单 knuckle 抓碟背心 | converged(parent) |
| dual_arm_trunnion_fork | rec_satellite_dish_var_dual_arm_fork | azimuth_yoke / yoke_arm_{i}(对称双臂 loop) / shared trunnion cross-shaft | 双立臂跨碟、共享横轴俯仰 (天文台式摇篮) | converged |
| tilt_tripod | rec_satellite_dish_var_tilt_tripod | azimuth_yoke / tripod_leg_{i}(三脚 loop) + footpad/bolt + central mast | 三脚架座 + 中央桅杆 tilt knuckle | converged |

### Slot C: feed (馈源光学；位于焦点)
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| center_fed_horn (基线-parent) | rec_sci-fi-satellite-dish-comm-unit-a-dark-matte-rec_20260612_113210_418481_e7ad375a | feed_horn(ConeGeometry) / feed_boom / feed_tip | 轴向中心馈源喇叭 + 中央 boom (prime focus) | converged(parent) |
| cassegrain | rec_satellite_dish_var_cassegrain | subreflector disc + strut_{i}(loop) + vertex-recessed horn | 卡塞格林副反射面 + 顶点凹入馈源 (双反射) | converged |
| offset_feed_arm | rec_satellite_dish_var_offset_feed_arm | offset_arm(swept) + offset feed horn + clamp/bolt loop | 偏馈：下缘曲臂伸向离轴焦点 (Ku 风) | converged |

### Slot D: panel_segment multiplicity N (panel_{i} loop——必须 loop 改写；变 N)
| N | record_id | 结构特征 | 状态 |
|---|---|---|---|
| N=8 (low) | rec_satellite_dish_var_panels_n8 | 圆抛物碗显式拆为 8 块 loop panel_0..7 + seam + bolt | converged |
| N=16 (med) | rec_satellite_dish_var_panels_n16 | 16 块 loop panel_0..15 | converged |
| N=24 (high) | rec_satellite_dish_var_panels_n24 | 24 块 loop panel_0..23 (高细节) | converged |

## Loop / Readability notes (HARD requirement for template rewrite)
- Parent ALREADY loop-emits these greebles (keep as-is in template):
  - label_bar_{i} (range 3), port_light_{i} (range 5), rack_slat_{i} (range 3), cable_conduit_{s} (enumerate 2), reflector_shell 内部 profile loop k in range(n_prof+1).
- Parent does NOT loop the reflective surface — it is one smooth LatheGeometry shell with NO panel segments and NO mount bolts. The panel-segment multiplicity axis (Slot D: panels_n8/16/24) is the variant that REQUESTS the loop rewrite: the reflective surface MUST be re-authored as loop-emitted panel_{i} (and seam lines + mount bolts loop-emitted) — this is the readability fix the template will inherit.
- All dish_form variants (hex_faceted / segmented_petal / flat_phased_array) ALSO build their surface via loop-emitted panel_{i}; together with Slot D they give the template a single panel_{i} loop with a count_param.
- Mount/feed variants loop their repeated structure too: yoke_arm_{i} (dual_arm), tripod_leg_{i} (tripod), strut_{i} (cassegrain subreflector struts) — never hand-copy.
- No hand-written panel/rib repeats currently exist to delete (parent surface is a single shell); the rewrite is additive (smooth shell → panel loop).

## Multiplicity / Copy Logic
- count_param: panel_count (反射面分段数；Slot D 主轴). 适用 round_parabolic / hex_faceted / segmented_petal / flat_phased_array 四种 dish_form 的 panel_{i} loop.
- 次级 count: arm_count (dual_arm 2) / leg_count (tripod 3) / strut_count (cassegrain) —— 由 mount/feed 变体覆盖，作模板侧固定或小范围 N，不再单独铺 N 样本.
- N 样本已覆盖: panel_count {8,16,24} → panels_n8/16/24; 形态分段由 hex_faceted/segmented_petal/flat_phased_array 提供拓扑差异.
- 模板建议 N_range: panel_count [6, 32]; petal_count [8, 24]; tripod leg 固定 3; dual arm 固定 2.
- copied object / naming / placement / joint policy: 复制对象 = panel_{i} 反射面分段 + seam + per-segment mount bolt; 共享 geometry helper (单 facet/petal/square 模具); 放置 = 同心环 / 放射 / 行列网格; joint policy = panel/bolt 全部内联为 dish_assembly visual，不新增 FIXED 装饰关节；az/el 双 REVOLUTE 永远保留为骨架.

## 格子覆盖 (1 parent 占 1 格组合, 10 变体填空)
- Slot A dish_form 3 空格 → hex_faceted / segmented_petal / flat_phased_array
- Slot B mount_gimbal 2 空格 → dual_arm_trunnion_fork / tilt_tripod
- Slot C feed 2 空格 → cassegrain / offset_feed_arm
- Slot D panel N 3 样本 → panels_n8 / panels_n16 / panels_n24
共 10 新变体 (批次规模 = 空格数, 上限 ~8-10). sci-fi satellite dish 小类样本池就绪.

## 排除项 (未来 compatibility matrix 素材, 非结构轴)
- color/material (matte black / teal / lime / amber 发光配色)、纯尺寸缩放 (大碟/小碟): §2 不立轴, 模板侧作参数/材质.
- 去掉 az 或 el 关节 (单轴固定碟): 破坏 aiming 身份, 不立轴 (双 REVOLUTE 是硬骨架).
- 把 greeble base box 换成纯净家用碟 / 民用电视天线: 出类目 (sci-fi 身份丢失), 不立轴.
- pure 装饰 greeble 增删 (更多 warning triangle / 更多 port light): 非结构, 模板侧作 N 或内联, 不单独立变体.
