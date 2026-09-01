# Table 6 UniPhysGen Paired-GT Release Audit

## Result

- Status: **RELEASE_PLACEHOLDER**
- Evaluated: **N=0**
- Official source fetched: yes, shallow clone at `742d4e6170ee132144880afb374dac2c1bc46c8a`
- Inference/GPU launched: no
- Checkpoint or dataset downloaded: no
- Paper values substituted for local measurements: no

The official public release is not runnable as of this audit. This is a release-readiness result, not a UniPhysGen experimental result.

## Official source audit

The official repository is cached at:

`/mnt/zsn/lyb/arti-skill/.cache/table6_sources/uniphysgen/repo`

`git ls-remote` resolves both `HEAD` and `main` to `742d4e6170ee132144880afb374dac2c1bc46c8a`. The remote exposes one branch, no tags, and no GitHub releases. The commit tracks only `.gitignore`, `LICENSE`, and a 71-byte, one-line `README.md`; there is no source code, environment, configuration, inference entrypoint, download script, checkpoint link, UniPhys-Bench link, or `AGENTS.md`. GitHub reports a repository size of 7 KiB; the local shallow clone is 51,833 bytes.

The repository declares Apache-2.0 for the released repository contents. This cannot be extended to unreleased checkpoints or datasets: no artifact-specific license is available.

## Artifact API audit

| Source | Query | Result | Download decision |
|---|---|---|---|
| GitHub releases | `breezexian/UniPhysGen` | 0 releases | Nothing to download |
| Hugging Face models API | `search=UniPhysGen` | 0 matches | No weight downloaded |
| Hugging Face datasets API | `search=UniPhys` | 0 matches | No dataset downloaded |

The repository itself contains no Hugging Face or other checkpoint/data link. Consequently, there is no trustworthy file list, byte size, checksum, or license to approve before a large download.

This is consistent with two official issue replies from the repository author on 2026-07-19:

- [Issue 1 author reply](https://github.com/breezexian/UniPhysGen/issues/1#issuecomment-5014913929): checkpoints, UniPhys-40K, and UniPhys-Bench are still being prepared.
- [Issue 2 author reply](https://github.com/breezexian/UniPhysGen/issues/2#issuecomment-5014929332): code, pretrained weights, UniPhys-40K, and UniPhys-Bench will be released once ready.

## Paired-GT gates

| Gate | Status | Evidence |
|---|---|---|
| Frozen Table 6 scope | PASS | UniPhysGen is paired-GT-only and must be recomputed on common GT. |
| Official repository provenance | PASS | Official remote and pinned commit were verified and cached. |
| Runnable official code | FAIL | No code, environment, config, or inference entrypoint is released. |
| Compatible checkpoint | FAIL | No checkpoint or official checkpoint link/release exists. |
| UniPhys-Bench paired data/GT | FAIL | Benchmark, manifest, GT, coordinate conventions, size and data license are unreleased. |
| Output package contract | FAIL | No prediction schema or method output exists to adapt. |
| Paired-GT evaluator | FAIL | No official benchmark evaluator/protocol is released; the local URDF collision harness is not equivalent. |

## Minimum unblock set

1. Runnable official inference code and environment specification.
2. Compatible checkpoint with exact size, checksum, provenance, and license.
3. UniPhys-Bench paired inputs plus independent GT, frozen manifest, units, coordinate frames, size, checksum, and license.
4. Official prediction schema and paired-GT matching/evaluation protocol.

GPU availability was intentionally not queried: all method and data gates fail on CPU-side provenance checks, so a GPU run cannot yet be defined. Until the release supplies the items above, Table 6 must retain `N/R`/`N/A` for UniPhysGen.
