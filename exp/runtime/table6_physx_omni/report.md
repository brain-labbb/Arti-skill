# Table 6 PhysX-Omni paired-GT status

Status: **BLOCKED**. Local result: **N=0 assets, N=0 joints**.

Official code and the released benchmark schema were verified, and the official
CPU-only tiny smoke passed. A PhysX-Omni inference or Table 6 paired-GT
evaluation was not run. No 8B weights, large dataset archive, benchmark image
collection, or GPU job was started, and no paper value was substituted.

## What is locally verified

| Artifact | Revision / size | License metadata | Local state |
|---|---|---|---|
| PhysX-Omni source | commit `46fa1cd0b6883d4d14431d51c3326ef80a85ef64`; 35,523,571 GitHub blob bytes | S-Lab License 1.0, non-commercial | downloaded as fixed archive; SHA-256 `10c8018475c640d169f5cfcd3d6c420336377b847a1d1407e4100d7fc01bf773` |
| PhysX-Omni model | revision `765cd275...`; 8,292,166,656 BF16 parameters; 4 shards; 16,584,414,544 weight bytes | MIT in model metadata | config/index only; weights not downloaded |
| TRELLIS image-large | revision `25e0d31f...`; 3,300,497,168 repository bytes | MIT | not downloaded |
| PhysXVerse | revision `264d5864...`; 112,599,768,644 bytes in three archive parts | CC BY-NC 4.0 | card/merge metadata only |
| PhysX-Mobility | revision `d0768ee9...`; current ZIP 937,374,668 bytes | CC BY-NC 4.0 | card only |
| PhysX-Bench | revision `6b4eb29a...`; 933,955,813 bytes; 1,219 files | MIT | API inventory and three description JSON files downloaded; images not downloaded |

The minimum two model repositories requested by the official `download.py`
total **19,884,911,712 bytes** before environment/package caches and generated
outputs. They were deliberately not downloaded.

All downloaded source and metadata are under
`.cache/table6_sources/physx_omni/`. The three PhysX-Bench description JSON
files match the copies in the fixed source archive by SHA-256.

## Benchmark schema finding

The released kinematic metric is KPS/VAPS. Its manifest contains a condition
image and a standardized articulation video rendered from the method's XML or
URDF. The VLM first creates a common-sense articulation prior from the image,
then scores the observed video motion. This is a plausibility judgment.

It does **not** consume output-independent joint records and does not compute
joint recall, type accuracy, parent-child accuracy, axis angular error, pivot
error, or range overlap. The official PhysX-Bench inventory contains 1,214
condition PNGs and three description JSON files, but zero separately released
joint-GT files. Therefore KPS/VAPS cannot be used to fill the Table 6 paired-GT
columns.

The large PhysXVerse and PhysX-Mobility archives were not inspected. They may
contain training annotations, but that is not evidence of a frozen,
output-independent Table 6 gold subset or a compatible scorer.

## Code smoke

`benchmark/scripts/run_tiny_smoke_test.sh` passed without a GPU:

- one RQS manifest row;
- one fake result aggregated;
- zero denominator mismatches.

This verifies only manifest, aggregation, and denominator plumbing. It is not
a model inference, KPS run, or experimental result.

## Table 6 result

| Method | Local N | Joint Type Accuracy | Joint Recall | Parent-Child Accuracy | Axis Valid | Origin/Pivot Valid | Limit/Range Valid |
|---|---:|---:|---:|---:|---:|---:|---:|
| PhysX-Omni paired-GT | 0 | N/R | N/R | N/R | N/R | N/R | N/R |

`N/R` means not reproduced locally, not a measured zero.

## Minimum unblock set

1. Download and hash the PhysX-Omni and TRELLIS weights after storage and
   license review.
2. Freeze a manageable condition-image/reference-asset subset with immutable
   identities.
3. Supply independent joint topology, type, axis, pivot, and range gold.
4. Implement a common output adapter and deterministic paired scorer; do not
   substitute KPS/VAPS for these metrics.
5. Only then check GPU availability/non-interference, run inference, and score.

Machine-readable revisions, byte counts, gates, and limitations are in
`preflight.json`.
