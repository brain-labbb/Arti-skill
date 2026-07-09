# Handtools / Paint roller — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-pain_20260609_163938_915351_ec6d7dba ← picture/Handtools/Paint roller/001.png (mini paint roller: cream foam cover, gray bent-wire Z-crank frame, coral molded grip)

Hand paint roller. Core kinematics shared by all candidates: a `handle_frame` (root: grip
+ steel wire shank/cage) and a single moving `roller_cover` child. The cover free-spins on
the axle via a CONTINUOUS joint `frame_to_roller` (axis +X, origin at the roller center on
the axle line). The journal bearing is the steel axle/shank captured inside the roller
core bore (`expect_overlap` on (wire_frame/shank, roller_core); in the perforated variant
the journal moves to the end-spider hub bores). The frame shank, the user grip, and the
roller cover are the three independent structural slots below; the spin joint itself is
fixed kinematics, not a slot.

## Slot 候选覆盖

### Slot A：cage / frame shank (`handle_frame` root: steel wire path from far stub through roller bore to grip)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| z_crank_wire | rec_..._pain_..._ec6d7dba (parent) | `wire_frame` (single `tube_from_spline_points` over `_frame_wire_path`); 90° BEND_X drop to HANDLE_Z | classic bent steel wire: axle through roller, ~90° crank down to an offset grip; one swept tube | converged |
| birdcage_spider | rec_paint_roller_var_cage_birdcage | `wire_axle` + `handle_stem` + `hub_cap` + `cage_spoke_{i}` (i<6, `for i in range(N_SPOKES)`, `_cage_spoke_path` helper) | straight axle + handle stem drop; birdcage retention cage of 6 radial spokes off a bored hub end-cap holds the cover on | converged |
| straight_inline_shank | rec_paint_roller_var_cage_straight | `shank` (single CylinderGeometry) + `collar` (zinc ferrule); no Z drop | inline straight shank into a collar adapter, grip co-axial with the roller (no crank) | converged |

### Slot B：handle / grip (`handle_grip` revolved coral body at the grip end; wire shank sockets into the collar)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| smooth_molded_grip | rec_..._pain_..._ec6d7dba (parent) | `handle_grip` (`_handle_shape` revolve of a barrel polyline) | plain barrel-tapered molded grip; wire sockets into the collar neck | converged |
| ribbed_scalloped_grip | rec_paint_roller_var_grip_ribbed | `handle_grip` (scalloped peak/valley profile) + `finger_ridge_{i}` (i<5, `TorusGeometry` rings, `_finger_ridge_mesh`, inline visuals) | ergonomic grip: peak/valley revolved body + raised torus finger ridges at regular X stations | converged |
| hollow_tube_sleeve | rec_paint_roller_var_grip_tube | `handle_tube` (stepped-bore cylinder: wide entry + tight socket) + `grip_ring_{i}` (i<6, `_grip_ring_shape`, inline visuals) | open tubular sleeve grip with a visible stepped bore + annular grip rings; wire shank press-fits into the socket bore | converged |
| pole_socket_grip | rec_paint_roller_var_grip_pole | `handle_grip` (flat butt face + bored extension-pole socket) + `thread_turn_{i}` (i<6, helical `tube_from_spline_points`, inline visuals) | molded grip whose butt end is a female-threaded extension-pole receptacle (bore + visible internal thread helix) | converged |

### Slot C：roller cover (`roller_cover` part: cover shell + core/lattice; the moving child)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| smooth_foam_cylinder | rec_..._pain_..._ec6d7dba (parent) | `roller_cover` (cq tube w/ bore) + `roller_core` (solid sleeve, journal bore) | plain hollow foam cylinder over a solid hard core sleeve; open hollow ends | converged |
| napped_pile_fabric | rec_paint_roller_var_cover_nap | `roller_cover` + `roller_core` + `nap_ring_{i}` (i<6, `MeshGeometry` w/ radially-displaced outer verts, `_nap_ring_mesh`) | smooth shell wrapped in procedurally-napped pile fabric ring sections (raised fiber texture), nap clear of hollow ends | converged |
| feathered_taper_edge | rec_paint_roller_var_cover_taper | `roller_cover` (`LatheGeometry` + `boolean_difference` bores) + `roller_core` (CylinderGeometry) | edge/trim roller: lathed body with cylindrical middle narrowing to rounded feathered cone tips at both ends; full-length narrow + mid wide bore | converged |
| perforated_lattice_core | rec_paint_roller_var_cover_perf | `roller_cover` + `lattice_rib_{i}` (i<8) + `lattice_hoop_{i}` (i<5) + `end_spider_{i}` (i<2, hub+arms journal) | solid core replaced by a visible open-cage lattice (longitudinal ribs + hoop rings + 2 end spiders); axle journals in the spider hubs; cage visible through hollow ends | converged |

## Multiplicity / Copy Logic
- count_param: 无顶层 multiplicity 轴。核心结构为固定 named slots(frame shank / grip / cover);整件物体只有 1 个 roller cover、1 个 grip。
- 复制逻辑均为 **candidate 内部** 的装饰/结构子件循环(每个都已用 `for i in range(n)` + `name_{i}` + 共享 helper + 规则化 placement 写好),不是跨样本的拓扑数量轴:
  - birdcage_spider: `cage_spoke_{i}` ×6(等角 2π/N_SPOKES,统一 FIXED-on-frame / inline);
  - ribbed_scalloped_grip: `finger_ridge_{i}` ×5(等距 X,inline 装饰 TorusGeometry);
  - hollow_tube_sleeve: `grip_ring_{i}` ×6(等距 X,inline 装饰);
  - pole_socket_grip: `thread_turn_{i}` ×6(螺旋等距,inline 装饰);
  - napped_pile_fabric: `nap_ring_{i}` ×6(等距 X 段,roller 子件);
  - perforated_lattice_core: `lattice_rib_{i}` ×8(等角)、`lattice_hoop_{i}` ×5(等距 X)、`end_spider_{i}` ×2(两端)。
- 模板侧若要把这些做成参数,它们是各自 candidate module 的局部 count(spoke_count / ridge_count / ring_count / nap_ring_count / rib_count / hoop_count),建议域如 spoke∈[4,8]、ridge∈[3,6]、ring∈[3,8]、rib∈[6,12]、hoop∈[3,7]——但每个都隔在自己的 candidate 内,不构成顶层 N 轴。
- N 样本(顶层): 无。

## 组合数预审
Slot A(3) × Slot B(4) × Slot C(4) = 48 ≥ 10 ✓。每个 slot ≥2 候选(A=3、B=4、C=4)。
pattern = parallel_children(root frame + 单一旋转 roller child),无顶层 multiplicity。无 gap。

## 排除项(未来 compatibility matrix 素材)
- 无不收敛取值;9 个来源(parent + 8 变体)全部 compile + run_tests 收敛,各保留 ≥1 非 fixed joint(`frame_to_roller` CONTINUOUS)。
- 跨槽接口风险待裁决(组合由模板采样器生成,这里仅记风险):
  - Slot A=birdcage_spider 把 journal+retention 放在 +X 端 hub;Slot C=perforated_lattice_core 把 journal 放在 cover 的 end_spider hub —— 两者都改了 axle 捕获面,组合时 hub/spider 可能重复或干涉,需 compatibility 裁决(parent 的 z_crank 走 roller_core 中段 journal,与三个 cover 候选均兼容,是安全基线)。
  - Slot B=hollow_tube_sleeve / pole_socket_grip 改了 grip 的 socket 内腔;Slot A=straight_inline_shank 用 zinc collar 接口 —— collar↔tube/threaded-socket 的插入深度需对齐(模板 InterfaceSpec 的 socket anchor)。
- 纯尺寸(roller 长度/直径、grip 长度、AXLE_R)不作为候选——属模板连续参数(controlled local parameterization),不入 slot。
- candidate 内部子件数(spoke/ridge/ring/rib/hoop count)是局部 count 参数,不是顶层 multiplicity 轴,不入本表的 N 覆盖。
