# PartNet-Mobility N=800: URDF Sim-Ready Table 4

Status: **COMPLETE_WITH_RETAINED_FAILURES**

This is an outcome-independent N=800 sampled release diagnostic. It is not the 2,347-asset Full Release panel and not the six-method shared-category balanced panel.

| Metric | Result |
|---|---:|
| Rest All-pair CF | 0/3 (0.000%) |
| Rest Non-adjacent CF | 0/3 (0.000%) |
| Single-joint Sweep CF | 0/3 (0.000%) |
| Multi-joint Sobol CF | 0/3 (0.000%) |
| Collision-state Rate | 300/300 (100.000%) |
| AOR | N/E |
| Max Penetration | None |
| Collision-free Range | 0/105 (0.000%) |
| Strict Collision Pass | 0/3 (0.000%) |

Collision-state Rate is fail-closed: unexecuted configurations caused by package, load, child, or timeout failures remain in the denominator and count as non-free. The report separately preserves observed collisions, executed states, and unexecuted states.

AOR is N/E because no stable exact overlap-volume calculation was run; bounding-box overlap is not used as a substitute. All sweeps are discrete, with no CCD claim.
