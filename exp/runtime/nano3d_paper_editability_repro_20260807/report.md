# Paper-aligned Nova3D local editability report

Protocol: `nova3d_section9_paper_aligned_reduced_v1`.

The paper protocol is 18 generated assets, one natural edit per asset, 13 additive and 5 modified-existing; deterministic gates precede two blinded human reviews and adjudication.

## Deterministic gates

| Gate | Pass | N |
|---|---:|---:|
| artifact_valid | 18/18 | 18 |
| target_handle | 14/18 | 18 |
| source_glb_changed | 18/18 | 18 |
| hierarchy_preserved | 18/18 | 18 |
| all_gates | 14/18 | 18 |

## Human review status

All human fields remain `N/A`/null until two independent reviewers score the blinded packet and an adjudicator resolves disagreements. The harness does not substitute automated proxies for those labels.

Review packet: `/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_paper_editability_repro_20260807/blind_review_packet/public_packet.json`; private key: `/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_paper_editability_repro_20260807/blind_review_packet/private_key.json`; reviewer template: `/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_paper_editability_repro_20260807/blind_review_packet/reviewer_template.json`.

This is protocol-aligned but not an exact reproduction of the paper's private generated asset IDs, Blender render backend, or released reviewer panels.
