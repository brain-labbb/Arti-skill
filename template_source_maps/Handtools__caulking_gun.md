# Handtools / caulking gun — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-caul_20260609_163957_890257_24ba16de ← picture/Handtools/caulking gun/001.png (gunmetal half-barrel cradle, bracket+wing grip, ratchet J-hook rod)
- rec_build-a-realistic-articulated-3d-model-of-a-caul_20260609_164000_793564_ff090a9c ← picture/Handtools/caulking gun/002.png (red open skeleton half-pipe frame, pistol grip, thumb-plate rod)
- [EXCLUDED] rec_build-a-realistic-articulated-3d-model-of-a-caul_20260609_164004_317252_e2bcfd32 ← picture/Handtools/caulking gun/003.png — actually a HOT GLUE GUN (purple electric gun, chrome cone, power rocker), out of category. NOT a fork source.

Cartridge dispenser. Shared kinematics across all candidates: a `frame`/`barrel_body` (root)
holds a fixed `cartridge`; a `plunger_rod` advances via a PRISMATIC joint (`plunger_drive` /
`frame_to_plunger`, axis +X) and a `trigger_lever` swings via a REVOLUTE joint
(`trigger_pivot` / `frame_to_trigger`, axis +Y). Three independent structural slots: the
cartridge cradle/frame, the rear grip, and the plunger drive.

## Slot 候选覆盖

### Slot A:cartridge cradle / frame
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| half_barrel_cradle | rec_..._caul_..._24ba16de (parent A) | `barrel_cradle`, `front_collar`, `rear_cap` | lower half-shell cradle, cartridge sits in trough | converged |
| skeleton_halfpipe | rec_..._caul_..._ff090a9c (parent B) | `frame` cradle (rotated half-pipe), `front_ring`, `rear_plate` | open ~300° half-pipe skeleton frame | converged |
| closed_barrel | rec_caulk_var_closedbarrel (primary; from A) | full closed cylindrical barrel (lathe/revolve) enclosing cartridge | professional steel tube fully wrapping cartridge | converged |
| sausage_tube | rec_caulk_var_sausage (primary; from A) | long sealed tube + screw-on front cone cap | bulk/sausage gun, threaded nose cap | converged |
| rib_cage | rec_caulk_var_ribframe (from B) | N rib rods via `for i in range(n)` between front_ring/rear_plate | open rod-cage skeleton (rods, not shell) | converged |

### Slot B:rear grip / handle
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| bracket_wing | rec_..._caul_..._24ba16de (parent A) | `handle_frame` (U-yoke bracket) + `grip_wing` | black bracket + fixed wing blade | converged |
| pistol_grip | rec_..._caul_..._ff090a9c (parent B) | `grip` pistol tongue hanging down/back | curved pistol-grip tongue | converged |
| d_ring_loop | rec_caulk_var_dgrip (from B) | closed D-ring round-bar bow, trigger inside loop | full closed loop hand-through grip | converged |
| inline_handle | rec_caulking_gun_var_inlinehandle (from B) | straight inline rear handle | inline/straight rear handle | converged |

### Slot C:plunger drive / pull end
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| ratchet_jhook | rec_..._caul_..._24ba16de (parent A) | `plunger_rod` + `hook_handle` bent J-hook tail | ratchet rod with J-hook thumb rest | converged |
| thumb_plate | rec_..._caul_..._ff090a9c (parent B) | `plunger_rod` + rear thumb plate | flat rear thumb plate | converged |
| ring_pull | rec_caulking_gun_var_ringpull (from A) | rod with rear ring-pull loop | finger ring pull at rod tail | converged |

## Multiplicity / Copy Logic
- count_param: rib_cage 候选内有 `rib_count`(rib_cage module 专属);核心结构为固定 named slots,无小类级 multiplicity 轴。
- copied object / naming / placement / joint policy: rib_cage 的 rib rods 用 `for i in range(n)` 等角发射、共享 cylinder helper、全部 FIXED 到 frame。建议 rib N_range:[3, 6]。

## 组合数预审
Slot A(5) × Slot B(4) × Slot C(3) = 60 ≥ 10 ✓。每个 slot ≥2 候选。pattern = parallel_children。

## 排除项 / 重复格子(未来 compatibility matrix 素材)
- Parent C (e2bcfd32) 是热熔胶枪,出类目,排除,不作 fork 源。
- 重复格子(prior 批次同格备样,workbench 保留但不作为 primary、不同步):
  - closed_barrel: rec_caulking_gun_var_closedbarrel (from B) — dup of rec_caulk_var_closedbarrel
  - sausage_tube: rec_caulking_gun_var_sausagebarrel (from B) — dup of rec_caulk_var_sausage
- 纯尺寸(枪长/口径)是模板连续参数,不入 slot。
