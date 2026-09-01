# Table 6: Nova3D articulation preflight

- Formal status: **BLOCKED**.
- Blocking codes: **CLOSED_BACKEND / PAPER_ASSETS_UNRELEASED**.
- Evidence class: **PAPER_ONLY**.
- Local Nova3D evaluation: **NOT RUN; N=0 assets, N=0 joints**.
- Hosted Nova3D/paid API calls: none.
- GPU work: none.

## Official source audit

The official repository was cloned to
`.cache/table6_sources/nova3d/code` and fixed at commit
`042ee613aa2fb745d287261eab029d42c704646e` (`main`, clean worktree).
Its top-level license is MIT (SHA-256
`10f51bebdb241489cca901c0a4b4986667a2747cf074553971f5f65bf388fff9`).

The official README makes the execution boundary explicit:

- the web app connects to the hosted Nova3D service;
- the repository contains open clients and integrations, while the hosted
  generation backend is closed-source/proprietary;
- `examples/`, described as generated assets plus source programs, is still
  marked `coming soon` and is absent from the checkout.

The checkout contains no `.glb`, `.gltf`, `.urdf`, or `.blend` asset. It also
contains no 12-asset/59-joint case, case manifest, evaluator records, or joint
gold. The public GitHub Releases page lists three Blender Plugin releases, not
the paper articulation assets.

## API/client boundary

The public MCP and Blender integration code is installable client code. The MCP
README says initial generation uses the paid GraphFlow v2 workflow, requires
browser sign-in/readiness checks, and may require purchasing credits. Its
`articulate_model` tool dispatches to that hosted workflow; the articulation
backend implementation is not in this repository. The default API endpoint is
`https://nova3d.xyz/api`.

No hosted endpoint was called and no secret was read. A local MCP unit-test
attempt stopped before collection because the available Python lacks `pytest`;
this does not change the decisive backend/asset blockers.

## Local Table 6 row

| Method | Local N | Articulable | Joints/Asset | Native Joint Exposure | Joint Type Accuracy | Joint Recall | Parent-Child Accuracy | Axis Valid | Origin Valid | Limit Valid | Joint Geom. Valid | Asset Geom. Valid | Full-Range Collision-Free | Generic Range |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Nova3D (official public repo preflight) | 0 | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R |

`N/R` means not reproduced locally. It is not a zero score. The values already
shown in `exp/Nano3dresults.md` remain a **PAPER_ONLY** transcription and must
not be described as results reproduced from this checkout.

## Required to unblock

1. Official release of the 12 articulated assets and all 59 joint records.
2. Frozen input/output manifest and output-independent joint gold.
3. A locally runnable official backend/evaluator, or separately authorized and funded hosted evaluation with retained artifacts.

Machine-readable provenance, hashes, runtime boundary, and blockers are in
`preflight.json`.
