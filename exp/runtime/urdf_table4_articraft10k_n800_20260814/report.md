# Articraft-10K fixed N=800 cohort: URDF Sim-Ready Table 4

Status: **COMPLETE_WITH_RETAINED_FAILURES**

This is the exact frozen Table 2 sample from the Articraft-10K 9,996-asset release universe. It is not a full-release or category-balanced result.

Category macro average: N/E (no authoritative category labels).

| Metric | Result |
|---|---:|
| Rest All-pair CF | 13/800 (1.625%) |
| Rest Non-adjacent CF | 187/800 (23.375%) |
| Single-joint Sweep CF | 156/800 (19.500%) |
| Multi-joint Sobol CF | 147/800 (18.375%) |
| Collision-state Rate | 86157/112165 (76.813%) |
| AOR | N/E |
| Max Penetration | 0.4765525203859249 (fully measured 223/800; observed 223/800; PARTIAL) |
| Collision-free Range | 14292/60165 (23.755%) |
| Strict Collision Pass | 147/800 (18.375%) |

Collision-state Rate is fail-closed. Unexecuted configurations remain in the frozen denominator and count as non-free.

AOR is N/E because no stable exact overlap-volume calculation was run. Sweeps are discrete; no CCD, semantic-joint, or physical-dynamics claim is made.
