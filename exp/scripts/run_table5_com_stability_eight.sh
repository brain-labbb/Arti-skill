#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/mnt/zsn/lyb/arti-skill"
cd "$REPO_ROOT"

FORMAL_PREPARED="exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json"
ARTICRAFT_PREPARED="exp/runtime/table5_v2_articraft_diagnostic_resample_20260830/prepared/manifest.json"
ELIGIBILITY="exp/scripts/table5_tipover_eligibility.py"
RUNTIME="exp/scripts/table5_com_stability_runtime.py"
AGGREGATE="exp/scripts/table5_com_stability_aggregate.py"
GENESIS_PYTHON="/mnt/zsn/miniconda3/envs/genesis-main/bin/python"
ROOT="exp/runtime/table5_com_stability_eight_20260901_v2"
ELIGIBLE_ROOT="$ROOT/eligible"
RUN_ROOT="$ROOT/runs"
FINAL_ROOT="$ROOT/final"
LOG_ROOT="$ROOT/logs"

ALL_DATASETS=(
    articraft_10k
    lam_released_outputs
    artiverse
    partnet_mobility
    physx_mobility
    sketchmobility
    infinigen_sim
    pva
)

mkdir -p "$ELIGIBLE_ROOT" "$RUN_ROOT" "$FINAL_ROOT" "$LOG_ROOT"
exec > >(tee -a "$LOG_ROOT/supervisor.log") 2>&1

prepare_eligible_cohorts() {
    local slug prepared source_hash
    local pids=()
    for slug in "${ALL_DATASETS[@]}"; do
        (
            prepared="$FORMAL_PREPARED"
            if [[ "$slug" == "articraft_10k" ]]; then
                prepared="$ARTICRAFT_PREPARED"
            fi
            source_hash="$(jq -r '.manifest_sha256' "$prepared")"
            if [[ -f "$ELIGIBLE_ROOT/$slug/eligibility.json" ]] \
                && [[ -f "$ELIGIBLE_ROOT/$slug/manifest.json" ]] \
                && jq -e \
                    --arg slug "$slug" \
                    --arg source_hash "$source_hash" \
                    --arg protocol_id "table5-free-standing-support-eligibility-v2" \
                    '.dataset_slug == $slug and .source_manifest_sha256 == $source_hash and .protocol_id == $protocol_id' \
                    "$ELIGIBLE_ROOT/$slug/eligibility.json" >/dev/null; then
                echo "[$(date -u +%FT%TZ)] reuse eligibility $slug"
                exit 0
            fi
            python3 "$ELIGIBILITY" \
                --manifest "$prepared" \
                --dataset "$slug" \
                --out "$ELIGIBLE_ROOT/$slug"
        ) &
        pids+=("$!")
    done
    local status=0 pid
    for pid in "${pids[@]}"; do
        wait "$pid" || status=1
    done
    if [[ "$status" -ne 0 ]]; then
        echo "[$(date -u +%FT%TZ)] eligibility preparation failed" >&2
        exit "$status"
    fi
}

run_dataset() {
    local slug="$1"
    local gpu="$2"
    echo "[$(date -u +%FT%TZ)] start $slug on GPU $gpu"
    python3 "$RUNTIME" run \
        --prepared "$ELIGIBLE_ROOT/$slug/manifest.json" \
        --out "$RUN_ROOT/$slug" \
        --datasets "$slug" \
        --workers 1 \
        --gpus "$gpu" \
        --collision-policy robust_visual_collision \
        --recompute-inertia \
        --executable "$GENESIS_PYTHON" \
        >"$LOG_ROOT/$slug.log" 2>&1
    echo "[$(date -u +%FT%TZ)] complete $slug on GPU $gpu"
}

run_lane_one() {
    run_dataset articraft_10k 1 &
    local first=$!
    run_dataset lam_released_outputs 1 &
    local second=$!
    local status=0
    wait "$first" || status=1
    wait "$second" || status=1
    [[ "$status" -eq 0 ]] || return "$status"
    run_dataset partnet_mobility 1 &
    first=$!
    run_dataset sketchmobility 1 &
    second=$!
    wait "$first" || status=1
    wait "$second" || status=1
    return "$status"
}

run_lane_seven() {
    run_dataset artiverse 7 &
    local first=$!
    run_dataset physx_mobility 7 &
    local second=$!
    local status=0
    wait "$first" || status=1
    wait "$second" || status=1
    [[ "$status" -eq 0 ]] || return "$status"
    run_dataset infinigen_sim 7 &
    first=$!
    run_dataset pva 7 &
    second=$!
    wait "$first" || status=1
    wait "$second" || status=1
    return "$status"
}

aggregate_results() {
    local arguments=()
    local slug
    for slug in "${ALL_DATASETS[@]}"; do
        arguments+=(
            --dataset "$slug" "$ELIGIBLE_ROOT/$slug/manifest.json" "$RUN_ROOT/$slug"
        )
    done
    python3 "$AGGREGATE" "${arguments[@]}" --out "$FINAL_ROOT"
}

prepare_eligible_cohorts

run_lane_one &
lane_one_pid=$!
run_lane_seven &
lane_seven_pid=$!

status=0
wait "$lane_one_pid" || status=1
wait "$lane_seven_pid" || status=1
if [[ "$status" -ne 0 ]]; then
    echo "[$(date -u +%FT%TZ)] one or more GPU lanes failed" >&2
    exit "$status"
fi

aggregate_results
echo "[$(date -u +%FT%TZ)] Table 5 COM stability eight-dataset run complete"
