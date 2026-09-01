# PV-A Table 4 status-aware metrics

This is a deterministic, read-only reaggregation of the sealed v3 SQLite result.
It is not a second physics run; the source receipt and hashes are in `metrics.json`.

- Protocol: `urdf_sim_ready_table4_pva_full_release_v3`
- Release assets: `302440`
- Input-bound assets: `302386`
- State-observed assets: `302325`
- Complete assets: `302323`
- Fully measured assets: `302323`
- State coverage: `47994236 / 48006121` (99.9752%)

| Asset metric | Release (fail-closed) | Collision-measured conditional |
|---|---:|---:|
| Rest non-adjacent CF | 227963 / 302440 (75.3746%) | 227963 / 302323 (75.4038%) |
| Single-joint sweep CF | 208637 / 302440 (68.9846%) | 208637 / 302323 (69.0113%) |
| Joint-space Sobol CF | 206908 / 302440 (68.4129%) | 206908 / 302323 (68.4394%) |
| Strict collision pass | 204468 / 302440 (67.6061%) | 204468 / 302323 (67.6323%) |
| Rest adjacent-only diagnostic | 212466 / 302440 (70.2506%) | 212466 / 302323 (70.2778%) |

`complete` means complete discrete state accounting; `state_observed` also includes partial assets.
State-micro, asset-equal, category-macro, DoF-bin, severity, and error-taxonomy views are in `metrics.json`.
Unexecuted states remain fail-closed in release rates and are never relabeled as geometry failures.
