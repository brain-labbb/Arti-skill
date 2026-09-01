# Articraft-10K qualification N=3: URDF Sim-Ready Table 4

Status: **COMPLETE_WITH_RETAINED_FAILURES**

This is the exact frozen Table 2 sample from the Articraft-10K 9,996-asset release universe. It is not a full-release or category-balanced result.

Category macro average: N/E (no authoritative category labels).

| Metric | Result |
|---|---:|
| Rest All-pair CF | 0/3 (0.000%) |
| Rest Non-adjacent CF | 0/3 (0.000%) |
| Single-joint Sweep CF | 0/3 (0.000%) |
| Multi-joint Sobol CF | 0/3 (0.000%) |
| Collision-state Rate | 447/447 (100.000%) |
| AOR | N/E |
| Max Penetration | None (fully measured 0/3; observed 0/3; PARTIAL) |
| Collision-free Range | 0/252 (0.000%) |
| Strict Collision Pass | 0/3 (0.000%) |

Collision-state Rate is fail-closed. Unexecuted configurations remain in the frozen denominator and count as non-free.

AOR is N/E because no stable exact overlap-volume calculation was run. Sweeps are discrete; no CCD, semantic-joint, or physical-dynamics claim is made.
