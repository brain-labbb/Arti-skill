# Urban Environment / bucket1 — template source map

pattern: parallel_children
slug: bucket1   shard: bucket1   picdir: picture/Urban Environment/bucket1
parents:
- rec_red-painted-sheet-metal-conical-fire-bucket-with_20260608_164532_434098_8f71e941 ← picture/Urban Environment/bucket1/001.png (red painted sheet-metal CONICAL fire bucket, apex-down pointed bottom that cannot stand upright; hollow thin-wall revolved cone shell, rolled rim, two riveted pivot lugs, steel-wire swing bail handle REVOLUTE about the +/-Y lug diameter)
- rec_red-painted-sheet-metal-fire-bucket-a-tapered-cy_20260608_164512_348759_c25e8986 ← picture/Urban Environment/bucket1/001.png (red painted sheet-metal TAPERED-CYLINDER fire bucket, wider open top + flat bottom, free-standing; same rolled rim, two lugs, rivets, and steel-wire swing bail handle REVOLUTE about the +/-Y lug diameter)

Identity = red sheet-metal fire bucket / pail. Core invariants: a hollow
thin-wall revolved body (tapered / conical / straight / curved), a rolled top
rim, two riveted pivot ear-lugs on opposite +/-Y rim sides, and a steel-wire
swing **BAIL** handle on a **REVOLUTE** joint whose axis is the +/-Y lug
diameter line — the bail swing REVOLUTE is the defining joint and must survive
every variant (or be replaced by an equally real non-fixed joint, e.g. a hinged
lid). Optional wall-mounting bracket / hook ring. The two parents differ ONLY in
body profile (cone-pointed vs tapered-cylinder); all other layers are
identical.

The four independent structural slots are: **A body profile**, **B handle**,
**C mounting**, **D rim/band detail**.

## Loop / readability notes

- Both parents emit the two pivot lugs + two rivets via a single
  `for sgn, tag in ((+1.0,"pos"),(-1.0,"neg"))` loop over a shared body —
  symmetric, name-suffixed (`lug_pos/lug_neg`, `rivet_pos/rivet_neg`). Good.
- Bail wire is one `tube_from_spline_points` spline (no hand-repeats).
- Body is a `LatheGeometry.from_shell_profiles` revolved thin shell (curved /
  compound geometry, not boxy) — reuse this for all body-profile variants.
- **No reinforcing bands exist in either parent** (only a single rolled-rim
  torus). New band variants MUST loop-emit bands via a shared helper
  (`for i in range(n)`), sized to the local wall radius at each band height —
  do NOT hand-write each band.
- New side-grip / band / hook decorations: inline as parent visuals (no
  FIXED-joint decoration parts); only fixed grips are non-moving.

## Slot 候选覆盖

### Slot A:body profile (revolved shell silhouette)
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| conical_pointed (parent) | rec_red-...conical...8f71e941 (parent) | `bucket` shell via `_conical_shell_mesh`, apex `APEX_R` at z=0 | apex-down sharp cone, no flat base, hangs from bail | converged (parent) |
| tapered_cylinder (parent) | rec_red-...tapered-cy...c25e8986 (parent) | `bucket` shell via `_revolved_shell_mesh`, flat bottom plate | tapered cylinder, wide top + flat base, free-standing | converged (parent) |
| straight_cylindrical_pail | rec_bucket1_var_body_straight_pail | same lathe shell, top_r == bot_r (vertical wall) | untapered straight pail, equal top/bottom diameter | converged |
| hemispherical_bowl | rec_bucket1_var_body_hemispherical | lathe shell with quarter-circle arc profile | rounded half-sphere bowl, low & curved | converged |
| deep_narrow_cone | rec_bucket1_var_body_deep_cone | conical shell, large height:top-diameter aspect | tall slender funnel cone (sharper than parent) | converged |

### Slot B:handle (the carry mechanism — the bail REVOLUTE family)
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| swing_bail_revolute (parent) | both parents | `handle` bail wire, `bucket_to_handle` REVOLUTE axis (0,1,0) at `LUG_Z` | steel-wire bail swings ~180° over the top; defining joint | converged (parent) |
| fixed_side_grips | rec_bucket1_var_handle_fixed_grips | two looped `grip_i` side handles (one revolute fold-flat kept) | two fixed +/-Y D-loop grips; retains one real non-fixed joint | converged |
| no_bail_hinged_lid | rec_bucket1_var_handle_no_handle | `lid` part, `lid_hinge` REVOLUTE on rim-tangent axis | no bail/lugs; hinged circular lid is the new non-fixed joint | converged |

### Slot C:mounting (how the bucket is supported / hung)
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| free_standing (parent) | both parents | flat-bottom rest (cyl) / hangs from bail (cone) | no added mount | converged (parent) |
| wall_bracket | rec_bucket1_var_mount_wall_bracket | `bracket` back-plate (2 bolt holes) + cradle ring | flat vertical wall plate + body cradle; bail still revolute | converged |
| hook_ring | rec_bucket1_var_mount_hook_ring | `mount` torus eyelet + shank on the axis | suspension hook/eyelet above rim for hanging | converged |

### Slot D:rim / band detail (reinforcing hoops — distinct-N copy axis)
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rolled_rim_only (parent) | both parents | single `rolled_rim` torus | rolled rim, no body bands | converged (parent) |
| bands_n2 | rec_bucket1_var_bands_two | `band_i` for i in range(2), shared helper | 2 evenly-spaced reinforcing hoops hugging the wall | converged |
| bands_n3 | rec_bucket1_var_bands_three | `band_i` for i in range(3), shared helper | 3 evenly-spaced reinforcing hoops hugging the wall | converged |

## Combo pre-audit (HARD GATE)

Slot A = 5 candidates, Slot B = 3, Slot C = 3, Slot D = 3 (incl. distinct-N
bands n∈{2,3}). product = 5 × 3 × 3 × 3 = **135 ≥ 10** → PASS. Every slot has
≥ 2 candidates. Distinct-N is carried by Slot D (n = 0/2/3 distinct band counts).

## Variants to fork (9 new, single-axis each)

| record_id | label | axis | parent |
|---|---|---|---|
| rec_bucket1_var_body_straight_pail | bucket1-body_straight_pail | A | tapered-cyl |
| rec_bucket1_var_body_hemispherical | bucket1-body_hemispherical | A | tapered-cyl |
| rec_bucket1_var_body_deep_cone | bucket1-body_deep_cone | A | conical |
| rec_bucket1_var_handle_fixed_grips | bucket1-handle_fixed_grips | B | tapered-cyl |
| rec_bucket1_var_handle_no_handle | bucket1-handle_no_handle | B | tapered-cyl |
| rec_bucket1_var_mount_wall_bracket | bucket1-mount_wall_bracket | C | tapered-cyl |
| rec_bucket1_var_mount_hook_ring | bucket1-mount_hook_ring | C | conical |
| rec_bucket1_var_bands_two | bucket1-bands_two | D | tapered-cyl |
| rec_bucket1_var_bands_three | bucket1-bands_three | D | tapered-cyl |

## Dropped / deferred

- Color/material/pure-scale changes — disallowed (suffix forbids; never count as
  the axis).
- "Rolled rim vs no rim" — rejected: a fire bucket without a rolled rim breaks
  the sheet-metal identity; rim is invariant, so Slot D is carried by *bands*.
- Square/rectangular bucket body — deferred: drifts from the round
  revolved-shell fire-bucket identity toward a generic tote/caddy.
