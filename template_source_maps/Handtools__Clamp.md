# Handtools / Clamp — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-clam_20260609_163926_720602_e34de725 ← picture/Handtools/Clamp/001.png (red C-clamp, swivel pad, T-bar handle)
- rec_build-a-realistic-articulated-3d-model-of-a-clam_20260609_163929_874005_1a4c37c7 ← picture/Handtools/Clamp/002.png (black G-clamp, fixed pad, T-bar with ball ends)

Screw-driven clamp. Core kinematics shared by all candidates: a `frame` (root) and a
`screw` spindle that drives into the throat via a PRISMATIC joint (`frame_to_screw`). The
clamping foot, the user handle, and the frame silhouette are the three independent
structural slots below.

## Slot 候选覆盖

### Slot A:foot / clamping pad (`screw_to_pad` distal end)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| swivel_ball_pad | rec_..._clam_..._e34de725 (parent A) | `pad` part, `screw_to_pad` (REVOLUTE y) on `swivel_pad` | ball-socket swivel foot that tilts to mate angled work; adds a 2nd joint | converged |
| fixed_flat_pad | rec_..._clam_..._1a4c37c7 (parent B) | `swivel_pad` visual fixed to `screw` (no revolute) | pad rigidly fixed to spindle tip; prismatic only | converged |
| anvil_disc_pad | rec_clamp_var_anvilfoot | flat anvil pressing disc on screw tip (revolute removed) | broad flat pressing disc instead of ball; pad fixed | converged |

### Slot B:handle / drive grip (top of `screw`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| tbar_caps | rec_..._clam_..._e34de725 (parent A) | `handle_bar` + `handle_caps` (loop over ±sign) | slim T cross-bar with small end caps | converged |
| tbar_balls | rec_..._clam_..._1a4c37c7 (parent B) | `tbar` + `ball_pos`/`ball_neg` spheres | T cross-bar with large ball ends | converged |
| side_lever | rec_clamp_var_leverhandle | single side-mounted lever handle (adds revolute lever joints) | one-sided swing lever instead of cross-bar | converged |
| butterfly_wing | rec_clamp_var_winghandle | butterfly / wing-nut style twin-wing grip | broad finger wings for hand-twisting | converged |

### Slot C:frame silhouette (`frame_body`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rounded_C | rec_..._clam_..._e34de725 / _1a4c37c7 (parents) | `frame_body` extruded C-profile, filleted corners | classic rounded-throat C/G frame | converged |
| boxy_deep_C | rec_clamp_var_frame | `frame_body` deeper, squarer right-angle corners, longer throat reach | boxy deep-reach frame silhouette | converged |

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots(foot / handle / frame)。
- N 样本: 无 multiplicity 轴。
- copied object / naming / placement / joint policy: parent A `handle_caps` 用 `for sign in (-1,1)`
  对称发射(可读)。parent B 的 `ball_pos`/`ball_neg` 是手写两份(下游若做对称 N 把手需折成循环)。

## 组合数预审
Slot A(3) × Slot B(4) = 12 ≥ 10 ✓(含 Slot C ×2 → 24)。每个 slot ≥2 候选。pattern = parallel_children,无 multiplicity。

## 排除项(未来 compatibility matrix 素材)
- swivel_ball_pad 仅在 parent A 出现;fixed/anvil 两种与 side_lever / butterfly 组合未抽检(组合由模板采样器生成)。
- 纯 throat 尺寸(small/medium/large)不作为候选——属模板连续参数(controlled local parameterization),不入 slot。
