# Science / Capsule — template source map

pattern: mixed
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-caps_20260609_183618_805753_3c571303 ← picture/Science/Capsule/ (two-piece hard gelatin pharma pill capsule: `capsule_body` + `capsule_cap` telescoping along the body axis). Covers Slot A=hemispherical_dome, Slot B=plain_telescoping_prismatic, Multiplicity N=2.

Two-piece hard gelatin pill capsule. `capsule_body` is the root; `capsule_cap` telescopes over it
via a single axial PRISMATIC `body_to_cap`. The batch isolates the end-cap profile and the
seam/closure mechanism as independent slots, plus a multi-compartment multiplicity axis that
rewrites the body+cap pair into a chain of `segment_{i}` shells joined by per-seam PRISMATIC slides.

## Slot 候选覆盖

### Slot A:end-cap profile
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| hemispherical_dome | rec_build-...-caps_20260609_183618_805753_3c571303 (parent) | `capsule_body`, `capsule_cap`, `body_to_cap`(prismatic) | rounded hemispherical domes on both ends (classic gelatin capsule) | converged(parent) |
| flat_caplet | rec_capsule_var_flatcap | `capsule_body`, `capsule_cap`, `body_to_cap`(prismatic) | flattened caplet ends (squared-off oval tablet profile) | converged(workbench, rating pending sync) |
| bullet_tip | rec_capsule_var_bulletip | `capsule_body`, `capsule_cap`, `body_to_cap`(prismatic) | tapered bullet/ogive nose tip on the cap end | converged(workbench, rating pending sync) |

### Slot B:seam / closure mechanism
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| plain_telescoping_prismatic | rec_build-...-caps_20260609_183618_805753_3c571303 (parent) | `body_to_cap`(prismatic) | plain telescoping fit, cap slides axially over body | converged(parent) |
| snap_lock_band | rec_capsule_var_lockband | `capsule_cap`, `body_to_cap`(prismatic) | snap-lock locking band ring at the seam; still a PRISMATIC closure | converged(workbench, rating pending sync) |
| screw_thread_revolute | rec_capsule_var_screwcap | `capsule_cap`, `body_to_cap`(revolute) | threaded screw cap, closure is a REVOLUTE about the body axis (n_starts thread loop) | converged(workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: `segment_count`(multi-compartment shells)
- N 样本已覆盖: {2, 3, 4} → parent / rec_capsule_var_seg3 / rec_capsule_var_seg4
- 模板建议 N_range: [2, 4]
- copied object / naming / placement / joint policy: shells looped as `segment_{i}` chained head-to-tail along the body axis; one PRISMATIC seam per inter-segment join (N-1 joints for N segments, observed 2 prismatic @ N=3, 3 prismatic @ N=4); uniform telescoping policy.

## 组合数预审
Slot A(3) × Slot B(3) × Multiplicity(3) = 27 ≥ 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- Sealed softgel (single fused shell, 0 joints) excluded — fails the ≥1 non-fixed joint gate and carries no closure axis.
- Surface imprint / branding is decoration, not an articulation axis.
- "Pellets/beads inside shell" rejected: free fill with no joint, not a topology candidate.
- Pure scale (length/diameter) is a continuous param, not a slot.
