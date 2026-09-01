# Table 6: naive same-LLM articulation preflight

- Formal status: **BLOCKED**.
- Evidence class: **PAPER_ONLY**.
- Original-paper local evaluation: **NOT RUN; N=0 attributable assets**.
- Network/GPU/external jobs: none.

The `naive same-LLM` row in `exp/Nano3dresults.md` is the Nova3D paper's
direct Blender-program ablation. Its native product is a Blender program plus
GLB, not an articulated URDF. The existing Table 6 values (`Articulable=no`,
`Joints/Asset=0`, `Native Joint Exposure=0`) remain paper transcription only.
They were not reproduced here. In particular, **zero joints does not imply that
the method generated no assets**: the same document records 31/54 executable,
saved outputs.

No locally attributable original-paper outputs, original 54-item manifest, or
frozen common Table 6 articulation manifest was present in the targeted
evidence inspected. Therefore no real local Table 6 metric can be computed for
this paper row.

## Local Table 6 result

| Method | Local attributable N | Articulable | Joints/Asset | Native Joint Exposure | Remaining joint metrics |
|---|---:|---:|---:|---:|---:|
| naive same-LLM (Nova3D paper ablation) | 0 | N/R | N/R | N/R | N/A |

`N/R` means not reproduced locally, not a zero score. The semantic and
geometric joint columns are `N/A` for the paper's non-articulated output unless
a separate, explicitly frozen articulation-producing intervention is defined.

## Separate local experiment

A runnable and completed local experiment with the same short display name does
exist: `t2_stratified_unseen_authoring_v1`. Its naive arm contains six completed
`gpt-5.6-sol` authoring runs and reports 5/6 executable, 6/6 artifact saved,
4/6 first-shot, 6/6 final success, 6/6 all-36, 34/34 corners, and 75/75
regression retention in
`exp/runtime/t2_formal_v1/authoring/summary.json`.

That experiment authors reusable Articraft templates and evaluates articulated
URDF seed packages. It is not the paper's 54-item direct Blender-program
ablation: task, representation, prompt contract, repair policy, and denominator
all differ. Its outputs are valid local T2 evidence but **must not be relabelled
as the paper baseline or used to fill this Table 6 row**.

## Required to unblock

1. Provenance-linked original-paper programs and generated GLB packages.
2. The frozen 54-item prompt/input/output manifest and failure records.
3. An explicit common Table 6 representation policy for this non-articulated control.
4. Independent joint gold if an articulation-producing variant is introduced.

Machine-readable status and claim boundaries are recorded in `preflight.json`.
