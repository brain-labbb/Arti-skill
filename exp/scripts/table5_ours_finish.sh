#!/usr/bin/env bash
# Wait for the frozen Ours-500K Genesis formal run to reach 500 terminal
# records, then aggregate Table 5a/5b via the frozen aggregate script.
set -u
REC=/mnt/zsn/lyb/arti-skill/exp/runtime/table5_ours_500k_n500_v1
OUT=$REC/aggregate/formal
PY=/mnt/zsn/miniconda3/envs/genesis-main/bin/python
SCRIPTS=/mnt/zsn/lyb/arti-skill/exp/scripts
LOG=/mnt/zsn/lyb/arti-skill/exp/runtime/table5_ours_500k_finish.log

count() { find "$REC/formal/genesis/assets" -name 'ours_*.json' 2>/dev/null | wc -l; }

while [ "$(count)" -lt 500 ]; do sleep 300; done
echo "[finish] $(date -u +%FT%TZ) genesis reached 500 records; aggregating" | tee -a "$LOG"
cd "$SCRIPTS"
"$PY" aggregate_table5_ours.py \
  --receipt-root "$REC" \
  --dataset-root /mnt/zsn/lyb/arti-skill/exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813 \
  --table1-manifest /mnt/zsn/lyb/arti-skill/exp/runtime/table1_ours_500k/manifest.json \
  --table2-root /mnt/zsn/lyb/arti-skill/exp/runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z \
  --table3-root /mnt/zsn/lyb/arti-skill/exp/runtime/urdf_table3_ours_500k_table2_n500_20260819T101000Z \
  --table4-root /mnt/zsn/lyb/arti-skill/exp/runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z \
  --phase formal --out "$OUT" >> "$LOG" 2>&1
echo "[finish] aggregate rc=$?" | tee -a "$LOG"
