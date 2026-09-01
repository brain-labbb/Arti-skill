# Backfill note

Table 5 (revision-2) with the Articraft-10K row replaced by the 2026-08-30
diagnostic resample re-run. All other seven datasets are unchanged from the
formal eight-dataset run.

- Formal source:  /mnt/zsn/lyb/arti-skill/exp/runtime/table5_v2_r2_formal_eight_datasets/final/summary.json
  summary_sha256: e17acf4bef332ebe38586ab40c2bf7b6425e3cc07614d5f877307b487812ed2d
- Articraft source (diagnostic resample, n=200): /mnt/zsn/lyb/arti-skill/exp/runtime/table5_v2_articraft_diagnostic_resample_20260830/final/summary.json
  summary_sha256: f2ff2ec0b244afe6f4137f89d2581c3e0f4aa5d337b8b689b649de23dbdaae4a
- Protocol check: metric definitions identical between the two runs; only
  cohort_binding hashes differ (resampled cohort), so numbers are comparable.
- Merge: summary-level dataset-block swap for articraft_10k; summary_sha256
  recomputed with table5_v2_aggregate.canonical_sha256.
- Merged summary_sha256: b9ca9f1786eb955f9829477d5d9486dc00550afd2b41718790b3027aeeb1a0f7

Replaced articraft_10k row (formal -> diagnostic):
- Genesis import/stable: 100.00/100.00 -> 98.00/97.50
- PyBullet import/stable: 100.00/100.00 -> 98.00/98.00
- MuJoCo import/stable: 100.00/100.00 -> 92.00/92.00
- Limit violation P95: 39.2613 -> 24.9453 (improved)
