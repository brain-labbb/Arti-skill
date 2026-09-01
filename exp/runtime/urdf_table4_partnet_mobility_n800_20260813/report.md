# PartNet-Mobility N=800 sampled release cohort: URDF Sim-Ready Table 4

Status: **COMPLETE_WITH_RETAINED_FAILURES**

This is an outcome-independent frozen cohort diagnostic. It is not the 2,347-asset Full Release panel and not the six-method shared-category balanced panel.

| Metric | Result |
|---|---:|
| Rest All-pair CF | 24/800 (3.000%) |
| Rest Non-adjacent CF | 622/800 (77.750%) |
| Single-joint Sweep CF | 591/800 (73.875%) |
| Multi-joint Sobol CF | 579/800 (72.375%) |
| Collision-state Rate | 47881/137638 (34.788%) |
| AOR | N/E |
| Max Penetration | 0.6330174807423453 (fully measured 787/800; observed 787/800; PARTIAL) |
| Collision-free Range | 48011/85638 (56.063%) |
| Strict Collision Pass | 567/800 (70.875%) |

Collision-state Rate is fail-closed: unexecuted configurations caused by package, load, child, or timeout failures remain in the denominator and count as non-free. The report separately preserves observed collisions, executed states, and unexecuted states.

AOR is N/E because no stable exact overlap-volume calculation was run; bounding-box overlap is not used as a substitute. All sweeps are discrete, with no CCD claim.
