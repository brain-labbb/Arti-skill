#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/mnt/zsn/lyb/arti-skill"
cd "$REPO_ROOT"

RUNTIME="exp/scripts/table5_stable_v2_runtime.py"
AGGREGATE="exp/scripts/table5_stable_v2_aggregate.py"
FORMAL_PREPARED="exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json"
ARTICRAFT_PREPARED="exp/runtime/table5_v2_articraft_diagnostic_resample_20260830/prepared/manifest.json"
OLD_SUMMARY="exp/runtime/table5_v2_r2_formal_eight_articraft_resample_backfill_20260830/summary.json"
ROOT="exp/runtime/table5_stable_v2_eight_20260830"
FORMAL_DATASETS="lam_released_outputs,artiverse,partnet_mobility,physx_mobility,sketchmobility,infinigen_sim,pva"

mkdir -p "$ROOT/logs"

run_pybullet() {
    python3 "$RUNTIME" run \
        --prepared "$FORMAL_PREPARED" \
        --out "$ROOT/runs/pybullet/formal" \
        --simulator pybullet \
        --datasets "$FORMAL_DATASETS" \
        --workers 16 \
        --executable exp/.venv_low_medium/bin/python
    python3 "$RUNTIME" run \
        --prepared "$ARTICRAFT_PREPARED" \
        --out "$ROOT/runs/pybullet/articraft" \
        --simulator pybullet \
        --datasets articraft_10k \
        --workers 16 \
        --executable exp/.venv_low_medium/bin/python
}

run_mujoco() {
    python3 "$RUNTIME" run \
        --prepared "$FORMAL_PREPARED" \
        --out "$ROOT/runs/mujoco/formal" \
        --simulator mujoco \
        --datasets "$FORMAL_DATASETS" \
        --workers 16 \
        --executable /mnt/zsn/miniconda3/bin/python
    python3 "$RUNTIME" run \
        --prepared "$ARTICRAFT_PREPARED" \
        --out "$ROOT/runs/mujoco/articraft" \
        --simulator mujoco \
        --datasets articraft_10k \
        --workers 16 \
        --executable /mnt/zsn/miniconda3/bin/python
}

run_genesis() {
    python3 "$RUNTIME" run \
        --prepared "$FORMAL_PREPARED" \
        --out "$ROOT/runs/genesis/formal" \
        --simulator genesis \
        --datasets "$FORMAL_DATASETS" \
        --workers 6 \
        --gpus 0,2,3,4,5,7 \
        --executable /mnt/zsn/miniconda3/envs/genesis-main/bin/python
    python3 "$RUNTIME" run \
        --prepared "$ARTICRAFT_PREPARED" \
        --out "$ROOT/runs/genesis/articraft" \
        --simulator genesis \
        --datasets articraft_10k \
        --workers 6 \
        --gpus 0,2,3,4,5,7 \
        --executable /mnt/zsn/miniconda3/envs/genesis-main/bin/python
}

run_pybullet >"$ROOT/logs/pybullet.log" 2>&1 &
pybullet_pid=$!
run_mujoco >"$ROOT/logs/mujoco.log" 2>&1 &
mujoco_pid=$!
run_genesis >"$ROOT/logs/genesis.log" 2>&1 &
genesis_pid=$!

status=0
wait "$pybullet_pid" || status=1
wait "$mujoco_pid" || status=1
wait "$genesis_pid" || status=1
if [ "$status" -ne 0 ]; then
    exit "$status"
fi

python3 "$AGGREGATE" \
    --formal-prepared "$FORMAL_PREPARED" \
    --articraft-prepared "$ARTICRAFT_PREPARED" \
    --run-root "$ROOT/runs" \
    --old-summary "$OLD_SUMMARY" \
    --out "$ROOT/final"
