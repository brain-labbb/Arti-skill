# Nano3D Editability reduced benchmark report

Protocol: `nano3d_editability_reduced_v1`  
Scope: 6 existing procedural templates × 3 edit tasks × 16 seeds = 288 edited cases.  
This is a reduced benchmark; it is not the full 18-template / 54-edit Nano3D protocol.

## Deterministic results

| Gate / metric | Result |
|---|---:|
| Target fulfilled (config-level) | 1.000 |
| Edited target addressable + output changed | 1.000 |
| Artifact saved | 1.000 |
| Output changed | 1.000 |
| Valid hierarchy tree | 1.000 |
| Post-edit operational template QC | 1.000 |
| Final deterministic proxy pass | 1.000 |
| Parameter scale contract (96/96) | 1.000 |
| Non-target structural preservation proxy (168/288) | 0.583 |
| Mean geometry locality proxy | 0.805 |
| Mean structural locality proxy | 1.000 |
| 16-seed task propagation (18/18) | 1.000 |

`Post-edit operational template QC` means the template's own deterministic tests,
URDF export and motion-QC path passed. It is not a hidden-spec constraint pass.
The locality and non-target values are structural/URDF proxies, not semantic or
human-reviewed geometry judgments.

## Coverage and unsupported fields

- Anchor correctness: unsupported; no frozen semantic anchor/coordinate gold.
- Regression preservation: unsupported; no independent historical regression manifest.
- Final human review: unsupported; no three-judge blind-review packet or agreement statistic.
- Edit cost: wrapper config diff and wall time are recorded; token/API cost is unavailable.
- Full benchmark: unsupported; this run covers 6/18 planned templates and 18/54 planned edit tasks.

Per-case evidence is in [`records.json`](records.json); the frozen manifest and
edit definitions are in [`manifest.json`](manifest.json).
