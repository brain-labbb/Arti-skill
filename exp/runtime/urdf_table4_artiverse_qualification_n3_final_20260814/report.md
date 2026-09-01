# Artiverse Table 1 qualification N=3: URDF Sim-Ready Table 4

Status: **COMPLETE**

This is the globally fixed Table 1 sample from the Artiverse pre-release 3,544-asset universe. It is not a full-release or category-balanced result.

| Metric | Result |
|---|---:|
| Rest All-pair CF | 0/3 (0.000%) |
| Rest Non-adjacent CF | 2/3 (66.667%) |
| Single-joint Sweep CF | 2/3 (66.667%) |
| Multi-joint Sobol CF | 2/3 (66.667%) |
| Collision-state Rate | 68/300 (22.667%) |
| AOR | N/E |
| Max Penetration | 0.15763956305371896 (fully measured 3/3; observed 3/3; COMPLETE) |
| Collision-free Range | 42/105 (40.000%) |
| Strict Collision Pass | 2/3 (66.667%) |

Collision-state Rate is fail-closed. Unexecuted configurations remain in the frozen denominator and count as non-free.

AOR is N/E because no stable exact overlap-volume calculation was run. Sweeps are discrete; no CCD, semantic-joint, or physical-dynamics claim is made.
