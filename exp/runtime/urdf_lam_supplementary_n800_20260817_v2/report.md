# LAM Genesis supplementary evaluation

Protocol: `urdf_lam_supplementary_n800_genesis_v1` (`genesis_contact_penetration_v1`).

Frozen cohort: N=800 assets, J=2395 movable joints, K=21 intended states per joint.
Scope: formal Genesis run; selected ranks=800; terminal fail-closed assets=594.
Verification aggregates SHA256: `86ef90bbf33d948809bbe2d444602e7a3e253100a6a4b274bcef88a9e43f09f4`.
Strict state records: 50336 intended raw rows (800 rest + 49536 Sobol).

Table-4a uses Genesis contact penetration with a strict illegal threshold of 1e-6 m; signed clearance is N/E because this adapter does not invent a separated-pair signed distance.
Table-2, Table-4b, and Supplementary S1 records remain explicit in the atomic asset rows; empty LAM receipt/allowance registries are preserved.
