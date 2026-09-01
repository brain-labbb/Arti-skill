#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/mnt/zsn/lyb/arti-skill"
cd "$REPO_ROOT"

ROOT="exp/runtime/table5_finite_rollout_v2_1_eight_20260830"
STRICT_ROOT="exp/runtime/table5_stable_v2_eight_20260830"
RUNNER="exp/scripts/run_table5_finite_rollout_v2_1_eight.sh"
AGGREGATE="exp/scripts/table5_stable_v2_1_aggregate.py"
FORMAL_PREPARED="exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json"
ARTICRAFT_PREPARED="exp/runtime/table5_v2_articraft_diagnostic_resample_20260830/prepared/manifest.json"
OLD_SUMMARY="exp/runtime/table5_v2_r2_formal_eight_articraft_resample_backfill_20260830/summary.json"

complete_summary() {
    local path="$1"
    local slug="$2"
    [[ -f "$path" ]] && jq -e --arg slug "$slug" '
        .runs | length == 1 and
        .[0].complete == true and
        .[0].dataset_slug == $slug and
        .[0].terminal_count == 200
    ' "$path" >/dev/null 2>&1
}

primary_complete() {
    complete_summary "$ROOT/runs/genesis/articraft/summary.json" "articraft_10k" || return 1
    local spec shard_name dataset_slug
    for spec in \
        "lam:lam_released_outputs" \
        "artiverse:artiverse" \
        "partnet:partnet_mobility" \
        "physx:physx_mobility" \
        "sketch:sketchmobility" \
        "infinigen:infinigen_sim" \
        "pva:pva"; do
        shard_name="${spec%%:*}"
        dataset_slug="${spec#*:}"
        complete_summary "$ROOT/shards/genesis_$shard_name/summary.json" "$dataset_slug" || return 1
    done
}

mkdir -p "$ROOT/logs"
while ! primary_complete; do
    total="$(find "$ROOT/runs/genesis/articraft" "$ROOT/shards" -path '*/assets/*.json' -type f 2>/dev/null | wc -l)"
    printf '%s primary terminal records: %s/1600\n' "$(date -u +%FT%TZ)" "$total"
    sleep 60
done

printf '%s primary Genesis shards complete\n' "$(date -u +%FT%TZ)"
bash "$RUNNER" merge-genesis
if [[ ! -e "$ROOT/final" ]]; then
    bash "$RUNNER" aggregate
fi
jq -e '.classification == "COMPLETE"' "$ROOT/final/summary.json" >/dev/null
printf '%s primary final report complete\n' "$(date -u +%FT%TZ)"

while [[ ! -f "$STRICT_ROOT/final/summary.json" ]]; do
    sleep 60
done
jq -e '.classification == "COMPLETE"' "$STRICT_ROOT/final/summary.json" >/dev/null

if [[ ! -e "$ROOT/final_with_strict" ]]; then
    python3 "$AGGREGATE" \
        --formal-prepared "$FORMAL_PREPARED" \
        --articraft-prepared "$ARTICRAFT_PREPARED" \
        --run-root "$ROOT/runs" \
        --old-summary "$OLD_SUMMARY" \
        --strict-summary "$STRICT_ROOT/final/summary.json" \
        --out "$ROOT/final_with_strict"
fi
jq -e '.classification == "COMPLETE"' "$ROOT/final_with_strict/summary.json" >/dev/null
printf '%s strict sensitivity report complete\n' "$(date -u +%FT%TZ)"
