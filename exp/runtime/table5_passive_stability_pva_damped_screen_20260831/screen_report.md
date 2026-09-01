# PV-A passive-stability metric screen

Run classification: **STOPPED AFTER PV-A CPU SCREEN**.

The existing Table 5a and Table 5b metrics and reports are unchanged. This exploratory screen tested the proposed fixed-root, one-joint-at-a-time, 10 s passive-settling protocol on the frozen 200-asset PV-A cohort before spending GPU time on a complete three-simulator run.

## Screen results

| Variant | PyBullet Settle Success (%) | MuJoCo Settle Success (%) | PyBullet-MuJoCo Agreement (%) |
|---|---:|---:|---:|
| Authored damping only | 72.00 | 48.50 | 61.50 |
| Preserve authored damping, otherwise type default | 64.50 | 38.50 | 55.00 |

The defaulted variant used damping `0.1` for revolute/continuous joints and `1.0` for prismatic joints. Across the 827 planned canonical joint trials, 76 target joints had authored damping and 751 used the type default. PyBullet completed all 827 trials. MuJoCo produced 825 valid trials; asset `pva_0182` exited with signal 11.

One three-simulator smoke asset (`pva_0000`) completed all trials in Genesis and PyBullet but failed MuJoCo settling because its prismatic recoil joint retained motion during the final 1 s window. The same asset also showed large cross-simulator endpoint divergence, so the endpoint metric cannot repair the low settle-success result.

## Decision

The complete Genesis 200-asset run was not submitted. Even with perfect Genesis performance, the damped protocol's All-3 Settle Success cannot exceed MuJoCo's 38.50% asset pass rate. The proposed columns are therefore not suitable for the primary Table 5 under this protocol and are not merged into the completed report.

The main failure is protocol sensitivity to damping, hard-limit behavior, and solver-specific passive dynamics rather than import, DoF mapping, or finite simulation readiness. A future physical-consistency benchmark needs category-specific support semantics and a separately preregistered dynamics-normalization study; it should not be retrofitted into the current Table 5.
