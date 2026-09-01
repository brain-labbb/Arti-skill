# Ours / PV-A: Table 4 v4 simulator-free evaluation

Status: **COMPLETE**

N_eval: 1000  \
J_eval: 7885

| Metric | Result |
|---|---:|
| Raw rest all-pair penetration-proxy CF | 0 / 1000 (0.000%) |
| Contact-adjusted rest all-pair penetration-proxy CF | 0 / 1000 (0.000%) |
| Raw rest penetration-proxy CF | 719 / 1000 (71.900%) |
| Contact-adjusted rest penetration-proxy CF | 719 / 1000 (71.900%) |
| Raw single-joint penetration-proxy CF | 637 / 1000 (63.700%) |
| Contact-adjusted single-joint penetration-proxy CF | 637 / 1000 (63.700%) |
| Raw Sobol penetration-proxy CF | 611 / 1000 (61.100%) |
| Contact-adjusted Sobol penetration-proxy CF | 611 / 1000 (61.100%) |
| Raw strict penetration-proxy pass | 603 / 1000 (60.300%) |
| Contact-adjusted strict penetration-proxy pass | 603 / 1000 (60.300%) |
| Raw strict intersection-free pass | 603 / 1000 (60.300%) |
| Contact-adjusted strict intersection-free pass | 603 / 1000 (60.300%) |
| Raw collision-state rate | 130556 / 230585 (56.619%) |
| Contact-adjusted collision-state rate | 130556 / 230585 (56.619%) |
| Raw collision-free range | 52271 / 165585 (31.567%) |
| Contact-adjusted collision-free range | 52271 / 165585 (31.567%) |
| Raw maximum normalized penetration-depth proxy | 0.565346 (1000 / 1000 observed; COMPLETE) |
| Contact-adjusted maximum normalized penetration-depth proxy | 0.565346 (1000 / 1000 observed; COMPLETE) |
| Exact global maximum penetration | N/E |
| AOR | N/E |

No physics simulator is loaded. Exact global mesh penetration depth and AOR remain N/E; the compatibility depth value is explicitly a python-fcl contact-depth proxy.
