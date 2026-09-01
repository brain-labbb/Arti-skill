#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="/mnt/zsn/lyb/arti-skill"
cd "$REPO_ROOT"

RUNTIME="exp/scripts/table5_stable_v2_1_runtime.py"
AGGREGATE="exp/scripts/table5_stable_v2_1_aggregate.py"
FORMAL_PREPARED="exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json"
ARTICRAFT_PREPARED="exp/runtime/table5_v2_articraft_diagnostic_resample_20260830/prepared/manifest.json"
OLD_SUMMARY="exp/runtime/table5_v2_r2_formal_eight_articraft_resample_backfill_20260830/summary.json"
ROOT="exp/runtime/table5_finite_rollout_v2_1_eight_20260830"
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

merge_genesis_shards() {
    local target_parent="$ROOT/runs/genesis/formal"
    mkdir -p "$target_parent"
    local spec shard_name dataset_slug source target temporary summary asset_count
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
        source="$ROOT/shards/genesis_$shard_name/$dataset_slug"
        target="$target_parent/$dataset_slug"
        summary="$ROOT/shards/genesis_$shard_name/summary.json"
        if [[ -d "$target/genesis/assets" ]]; then
            asset_count="$(find "$target/genesis/assets" -maxdepth 1 -name '*.json' | wc -l)"
            [[ "$asset_count" -eq 200 ]] || {
                echo "existing target is incomplete: $target ($asset_count/200)" >&2
                return 1
            }
            continue
        fi
        [[ ! -e "$target" ]] || {
            echo "existing target is not a complete runtime directory: $target" >&2
            return 1
        }
        [[ -f "$summary" ]] || {
            echo "shard summary is missing: $summary" >&2
            return 1
        }
        jq -e --arg slug "$dataset_slug" '
            .runs | length == 1 and
            .[0].complete == true and
            .[0].dataset_slug == $slug and
            .[0].terminal_count == 200
        ' "$summary" >/dev/null
        asset_count="$(find "$source/genesis/assets" -maxdepth 1 -name '*.json' | wc -l)"
        [[ "$asset_count" -eq 200 ]] || {
            echo "shard is incomplete: $source ($asset_count/200)" >&2
            return 1
        }
        temporary="$target_parent/.$dataset_slug.merge.$$"
        [[ ! -e "$temporary" ]] || {
            echo "temporary merge path exists: $temporary" >&2
            return 1
        }
        cp -a "$source" "$temporary"
        asset_count="$(find "$temporary/genesis/assets" -maxdepth 1 -name '*.json' | wc -l)"
        [[ "$asset_count" -eq 200 ]] || {
            echo "copied shard failed verification: $temporary ($asset_count/200)" >&2
            return 1
        }
        mv "$temporary" "$target"
        echo "merged $dataset_slug (200/200)"
    done
}

aggregate_results() {
    arguments=(
        --formal-prepared "$FORMAL_PREPARED"
        --articraft-prepared "$ARTICRAFT_PREPARED"
        --run-root "$ROOT/runs"
        --old-summary "$OLD_SUMMARY"
        --out "$ROOT/final"
    )
    if [[ -f "exp/runtime/table5_stable_v2_eight_20260830/final/summary.json" ]]; then
        arguments+=(
            --strict-summary
            "exp/runtime/table5_stable_v2_eight_20260830/final/summary.json"
        )
    fi
    python3 "$AGGREGATE" "${arguments[@]}"
}

case "${1:-}" in
    pybullet)
        run_pybullet
        ;;
    mujoco)
        run_mujoco
        ;;
    genesis)
        run_genesis
        ;;
    merge-genesis)
        merge_genesis_shards
        ;;
    aggregate)
        aggregate_results
        ;;
    *)
        echo "usage: $0 {pybullet|mujoco|genesis|merge-genesis|aggregate}" >&2
        exit 2
        ;;
esac
