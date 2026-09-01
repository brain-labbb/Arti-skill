#!/usr/bin/env bash
# Gate-aware resume loop for the frozen Ours-500K Genesis formal run.
# Launches the runner only while GPU0 is idle; SIGTERMs it (graceful stop,
# terminal records retained) if an external process takes the GPU; resumes
# until all 500 terminal records exist.
set -u
SCRIPTS=/mnt/zsn/lyb/arti-skill/exp/scripts
REC=/mnt/zsn/lyb/arti-skill/exp/runtime/table5_ours_500k_n500_v1
OUT=$REC/formal/genesis
PY=/mnt/zsn/miniconda3/envs/genesis-main/bin/python
LOGROOT=/mnt/zsn/lyb/arti-skill/exp/runtime/table5_ours_500k_genesis_loop_logs
GPU_INDEX=0
TOTAL=500
mkdir -p "$LOGROOT" "$OUT"

gpu_busy() {
  nvidia-smi --id=$GPU_INDEX --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits |
    awk -F', *' '{ if ($1 > 1024 || $2 != 0) exit 0; exit 1 }'
}

count_records() {
  find "$OUT/assets" -name 'ours_*.json' 2>/dev/null | wc -l
}

attempt=0
while true; do
  done_count=$(count_records)
  if [ "$done_count" -ge "$TOTAL" ]; then
    echo "[loop] all $TOTAL terminal records present; exiting"
    break
  fi
  if gpu_busy; then
    echo "[loop] $(date -u +%FT%TZ) GPU$GPU_INDEX busy with $done_count/$TOTAL done; waiting 300s"
    sleep 300
    continue
  fi
  attempt=$((attempt + 1))
  echo "[loop] $(date -u +%FT%TZ) attempt $attempt with $done_count/$TOTAL done; launching runner"
  CUDA_VISIBLE_DEVICES=$GPU_INDEX "$PY" "$SCRIPTS/run_table5_ours.py" \
    --simulator genesis --phase formal \
    --dataset-root /mnt/zsn/lyb/arti-skill/exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813 \
    --table1-manifest /mnt/zsn/lyb/arti-skill/exp/runtime/table1_ours_500k/manifest.json \
    --table2-root /mnt/zsn/lyb/arti-skill/exp/runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z \
    --table3-root /mnt/zsn/lyb/arti-skill/exp/runtime/urdf_table3_ours_500k_table2_n500_20260819T101000Z \
    --table4-root /mnt/zsn/lyb/arti-skill/exp/runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z \
    --manifest "$REC/manifest.json" --protocol "$REC/protocol.json" \
    --out "$OUT" --workers 1 \
    > "$LOGROOT/attempt_$(printf '%03d' $attempt).log" 2>&1 &
  runner_pid=$!
  while kill -0 "$runner_pid" 2>/dev/null; do
    sleep 30
    if gpu_busy; then
      ext_pids=$(nvidia-smi --id=$GPU_INDEX --query-compute-apps=pid --format=csv,noheader 2>/dev/null | tr -d ' ')
      # stop gracefully if someone else is on the GPU (our own child counts too,
      # so only stop when there is a compute PID and the runner is not mid-asset;
      # the runner's own gate re-checks per spawn; here we only protect against
      # sustained external occupancy)
      if [ -n "$ext_pids" ]; then
        used=$(nvidia-smi --id=$GPU_INDEX --query-gpu=memory.used --format=csv,noheader,nounits | tr -d ' ')
        if [ "$used" -gt 8192 ]; then
          echo "[loop] $(date -u +%FT%TZ) external GPU occupancy (used=${used}MiB); SIGTERM runner pid $runner_pid"
          kill -TERM "$runner_pid" 2>/dev/null
        fi
      fi
    fi
  done
  wait "$runner_pid"
  rc=$?
  echo "[loop] attempt $attempt exited rc=$rc; sleeping 120s"
  sleep 120
done
