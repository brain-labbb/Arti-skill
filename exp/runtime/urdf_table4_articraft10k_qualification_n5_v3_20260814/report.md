# Articraft-10K qualification N=5: URDF Sim-Ready Table 4

Status: **COMPLETE_WITH_RETAINED_FAILURES**

This is the exact frozen Table 2 sample from the Articraft-10K 9,996-asset release universe. It is not a full-release or category-balanced result.

Category macro average: N/E (no authoritative category labels).

| Metric | Result |
|---|---:|
| Rest All-pair CF | 0/5 (0.000%) |
| Rest Non-adjacent CF | 1/5 (20.000%) |
| Single-joint Sweep CF | 1/5 (20.000%) |
| Multi-joint Sobol CF | 1/5 (20.000%) |
| Collision-state Rate | 533/661 (80.635%) |
| AOR | N/E |
| Max Penetration | 0.008483735889913701 (fully measured 1/5; observed 1/5; PARTIAL) |
| Collision-free Range | 63/336 (18.750%) |
| Strict Collision Pass | 1/5 (20.000%) |

Collision-state Rate is fail-closed. Unexecuted configurations remain in the frozen denominator and count as non-free.

AOR is N/E because no stable exact overlap-volume calculation was run. Sweeps are discrete; no CCD, semantic-joint, or physical-dynamics claim is made.
