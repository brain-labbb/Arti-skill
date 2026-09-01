# Nova3D Naming baseline preflight

Status: **BLOCKED**

The local Nova3D row for Table 2 was not evaluated. The authorized workspace
contains no identifiable official Nova3D checkout, no Nova3D-attributed GLB or
URDF output package, and no shared manifest connecting Nova3D outputs to the
frozen Naming inputs. Existing `nano3d_*` runtime artifacts come from this
repository's `seed_exports` or `seed_exports_physics_10` pipeline and are not
relabelled as Nova3D outputs.

## Evidence

- Frozen common protocol: `nano3d_table2_baseline_naming_v1.1` at
  `arti-skill/exp/reference/baseline_naming_protocol_v1.json`.
- Checkout candidates or matching git remotes: 0.
- Nova3D-attributed structured assets or records: 0.
- Nova3D-attributed shared manifests: 0.
- Output-independent Nova3D role gold linked to those assets: no.
- Complete independent blind-judge verdict sets linked to those assets: 0/3.
- Network access: none.

## Required to unblock

1. Place the official Nova3D checkout inside the authorized workspace.
2. Generate or provide Nova3D GLB/URDF outputs for the frozen shared inputs.
3. Add a manifest that explicitly attributes every artifact to Nova3D and
   preserves shared asset/category identity.
4. Run direct Parts/Nameability on mesh-bearing GLB nodes (or report URDF links
   separately); complete output-independent gold and all three blind judges
   before reporting semantic metrics.

Until these inputs exist, all local Nova3D Table 2 metrics remain `N/R` (JSON
`null`); the separate Nova3D paper row is contextual evidence, not a local
rerun. Reporting zeros would conflate missing evidence with measured failure.
