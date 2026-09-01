# LAM Genesis supplementary evaluation

Protocol: `urdf_lam_supplementary_n800_genesis_v1` (`genesis_contact_penetration_v1`).

Frozen cohort: N=800 assets, J=2395 movable joints, K=21 intended states per joint.
Scope: pilot qualification smoke; selected ranks=10; terminal fail-closed assets=798.
Verification aggregates SHA256: `1abfaf6333a8348c23742d0a53fcff5633cda63bae39dc2b5897c897e39b0045`.
Strict state records: 50336 intended raw rows (800 rest + 49536 Sobol).

Table-4a uses Genesis contact penetration with a strict illegal threshold of 1e-6 m; signed clearance is N/E because this adapter does not invent a separated-pair signed distance.
Table-2, Table-4b, and Supplementary S1 records remain explicit in the atomic asset rows; empty LAM receipt/allowance registries are preserved.

This is a qualification smoke output only. Non-selected ranks are represented by terminal N/E records and must not be cited as a formal result.
