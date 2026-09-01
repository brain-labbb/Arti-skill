# Artiverse Table 1 fixed N=800 cohort: URDF Sim-Ready Table 4

Status: **COMPLETE_WITH_RETAINED_FAILURES**

This is the globally fixed Table 1 sample from the Artiverse pre-release 3,544-asset universe. It is not a full-release or category-balanced result.

| Metric | Result |
|---|---:|
| Rest All-pair CF | 12/800 (1.500%) |
| Rest Non-adjacent CF | 320/800 (40.000%) |
| Single-joint Sweep CF | 277/800 (34.625%) |
| Multi-joint Sobol CF | 292/800 (36.500%) |
| Collision-state Rate | 76889/133375 (57.649%) |
| AOR | N/E |
| Max Penetration | 0.6299950933881489 (fully measured 797/800; observed 797/800; PARTIAL) |
| Collision-free Range | 22154/81375 (27.225%) |
| Strict Collision Pass | 254/800 (31.750%) |

Collision-state Rate is fail-closed. Unexecuted configurations remain in the frozen denominator and count as non-free.

AOR is N/E because no stable exact overlap-volume calculation was run. Sweeps are discrete; no CCD, semantic-joint, or physical-dynamics claim is made.
