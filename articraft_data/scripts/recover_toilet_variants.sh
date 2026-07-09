#!/usr/bin/env bash
# Recover the 16 remaining edited-but-unpromoted toilet_001 (wall-hung baseline) WIP variants
# from the killed Jun-12 fork batch. Mirrors the validated v07 path:
#   external fork (scaffold) -> swap in real edited model.py -> external check -> external finalize -> patch hash
set -uo pipefail
cd /mnt/zsn/lyb/arti-skill/articraft_data
source .venv/bin/activate 2>/dev/null

PARENT=$(ls -d data/records/rec_wall-hung-white-ceramic-toilet-with-a-soft-close_* | head -1)
VARIANTS=(v09 v11 v12 v13 v14 v16 v18 v19 v20 v21 v22 v23 v25 v26 v29 v30)

echo "parent=$PARENT"
echo "recovering ${#VARIANTS[@]} variants..."
echo "rid,compile_status,note" > /tmp/toilet_recover_summary.csv

for v in "${VARIANTS[@]}"; do
  rid="rec_qwen37v_toilet_001_${v}"
  staging=$(ls data/cache/runs/*/staging/${rid}/model.py 2>/dev/null | head -1)
  sprompt=$(ls data/cache/runs/*/staging/${rid}/prompt.txt 2>/dev/null | head -1)
  if [ -z "$staging" ] || [ -z "$sprompt" ]; then
    echo "[$v] MISSING staging; skip"; echo "$rid,skip,missing-staging" >> /tmp/toilet_recover_summary.csv; continue
  fi
  if [ -d "data/records/$rid" ]; then
    echo "[$v] already exists in data/records; skip"; echo "$rid,skip,already-exists" >> /tmp/toilet_recover_summary.csv; continue
  fi
  prompt_text=$(cat "$staging" >/dev/null 2>&1; cat "$sprompt")

  # 1. scaffold child record (no LLM)
  uv run --frozen articraft external fork --agent claude-code --model-id qwen3.7-max --thinking-level high \
    --record-id "$rid" --skip-search-index "$PARENT" "$prompt_text" >/dev/null 2>&1

  dest="data/records/$rid/revisions/rev_000001/model.py"
  if [ ! -f "$dest" ]; then echo "[$v] fork failed"; echo "$rid,error,fork-failed" >> /tmp/toilet_recover_summary.csv; continue; fi

  # 2. swap in the real edited model
  cp "$staging" "$dest"

  # 3. compile/check (generates meshes) ; 4. finalize to workbench
  uv run --frozen articraft external check "data/records/$rid" >/tmp/chk_$v.log 2>&1
  uv run --frozen articraft external finalize "data/records/$rid" --skip-search-index >/tmp/fin_$v.log 2>&1
  cstatus=$(grep -oE "compile_status=[a-z]+" /tmp/fin_$v.log | tail -1 | cut -d= -f2)

  # 5. patch stale model_py_sha256 in record.json + revision.json
  python3 - "$rid" <<'PY'
import json,hashlib,sys
rid=sys.argv[1]; base=f"data/records/{rid}"
rec=json.load(open(f"{base}/record.json")); revid=rec["active_revision_id"]
h=hashlib.sha256(open(f"{base}/revisions/{revid}/model.py","rb").read()).hexdigest()
for f in [f"{base}/record.json", f"{base}/revisions/{revid}/revision.json"]:
    j=json.load(open(f))
    if j.get("hashes",{}).get("model_py_sha256"): j["hashes"]["model_py_sha256"]=h
    json.dump(j,open(f,"w"),indent=2)
PY
  echo "[$v] done compile_status=${cstatus:-unknown}"
  echo "$rid,${cstatus:-unknown}," >> /tmp/toilet_recover_summary.csv
done

echo "=== rebuild record index + search index (once) ==="
uv run --frozen articraft data build-record-index 2>&1 | tail -1
uv run --frozen articraft workbench search-index 2>&1 | tail -1
echo "=== SUMMARY ==="
cat /tmp/toilet_recover_summary.csv
