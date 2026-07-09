# VARIANT_PIPELINE.md — Internal API Variant Fork Runbook

> **On-demand runbook.** Read only when the user asks to fork variants for a picture subcategory or names this runbook.

This file tells the agent **how to run** a variant batch. Planning/acceptance rules live in `FORK_VARIANTS.md`. The fixed suffix already lives at `picture_expansion/FORK_SUFFIX.txt` and must be appended mechanically.

Use internal API fork only:
```bash
uv run articraft fork --provider dashscope --model qwen3.7-max --thinking-level high data/records/<parent_id> "<variant prompt>"
```
Do **not** use `articraft external init/fork/check/finalize`.

---

## Inputs

User provides a picture subcategory, e.g. `Stationary/Clip`. Defaults: `dashscope/qwen3.7-max`, thinking level `high`, budget from `FORK_VARIANTS.md` normally 8–30 candidate anchors, fork source = origin record or canonical baseline. Never pad with cosmetic, scale-only, material-only, or out-of-category variants.

---

## Step 1 — Find Parent Records

Parents come from `data/index/subcat/<Category>__<Subcategory>.jsonl`. Rows with no `parent_record_id` are origin parents.
```bash
cd /mnt/zsn/lyb/arti-skill/articraft_data
CAT="Stationary"; SUB="Clip"
uv run python - "$CAT" "$SUB" <<'PY'
import json, sys
from pathlib import Path
from storage.subcat_index import shard_name
cat, sub = sys.argv[1], sys.argv[2]
shard = Path("data/index/subcat") / f"{shard_name(cat, sub)}.jsonl"
rows = [json.loads(l) for l in shard.open()] if shard.exists() else []
originals = [r for r in rows if not r.get("parent_record_id")]
print("records for", f"{cat}/{sub}:", len(rows), "| originals:", [r["record_id"] for r in originals] or "(none)")
for r in originals: print(" ", r["record_id"], "| picture=", r.get("picture_path"))
PY
```
If there is no parent, stop. If multiple parents exist, use all as free anchors and fork each target candidate from the closest origin/canonical baseline.

---

## Step 2 — Plan Variants

Read `FORK_VARIANTS.md` and produce:

1. `subcategory_contract`;
2. slot/candidate grid;
3. mandatory 6-axis diversity audit;
4. multiplicity/copy-logic plan when repeated homogeneous parts exist;
5. budget decision and underfilled/blocked reasons;
6. one `variant_card` per planned fork.

Inspect parent code:
```bash
PARENT=data/records/<PARENT_RECORD_ID>
VM=$PARENT/revisions/rev_000001/model.py
grep -nE "for .* in (range|enumerate)" "$VM" || true
grep -nE "^\s*def |\.part\(|revolute|prismatic|continuous" "$VM" | head -80
```
If repeated homogeneous subparts are hand-written, first create or request a `canonical_baseline` cleanup.

Planning interpretation:

- ①/②/③ may be discovered from origin assets and world knowledge, but must become `origin_anchor` or converged `forked_anchor` before entering the candidate table.
- Multiplicity/N is a structural candidate-anchor axis; cover 2–3 representative N samples when present.
- ④ may be `record_only`, low-risk companion variation, or controlled `world_knowledge_extrapolation`; it need not appear as a dedicated variant.
- ⑤/⑥ are mandatory audit axes but not candidate-anchor axes. Record them for template sampling; they may ride along as low-risk companion variations, but never as standalone variants or to satisfy the 8–30 target.

---

## Step 3 — Write Axis Prompt Files

For each fork, write `/tmp/<slug>_var_<axis>.txt`:
```text
CATEGORY: <Category>
SUBCATEGORY: <Subcategory>
TARGET: change <slot/part/helper name from parent model.py> to <target candidate value>.
DIVERSITY_AXIS: <① skeleton/topology | ② joint/mechanism | ③ primary form family | N multiplicity | compatibility_probe>.
POSITIONING: <real product archetype and why it remains the same subcategory>.
KEEP: preserve these parent functional layers by name: <part_a>, <joint_b>, <helper_c>, ...
STRUCTURAL_DELTA: <specific geometry, joint, interface, and copied-object change>.
MULTIPLICITY: <only if this is the N axis; target N and placement/joint policy>.
COMPANION_VARIATIONS: <optional ④/⑤/⑥ only; no part tree / joint graph / interface / primitive-family change>.
FORBIDDEN: <neighbor categories and bundled-axis changes to avoid>.
COMPATIBILITY_PROBE: <only for probes; combined axes and clearance/interface risk>.
```
Do not paste `FORK_SUFFIX.txt`. The batch script appends it and replaces `<Category>` / `<Subcategory>` placeholders. Record ids: `rec_<subcat_slug>_var_<axis>`; allowed chars `[A-Za-z0-9_.-]`.

---

## Step 4 — Run Forks as One Main-Loop Background Job

Do not spawn one sub-agent per fork. Create `/tmp/run_variant_forks.sh`:
```bash
#!/usr/bin/env bash
set -u
cd /mnt/zsn/lyb/arti-skill/articraft_data || exit 99
set -a; source .env 2>/dev/null || true; set +a
CAT="<Category>"; SUB="<Subcategory>"
PARENT="data/records/<PARENT_RECORD_ID>"
SUFFIX="/mnt/zsn/lyb/arti-skill/articraft_data/picture_expansion/FORK_SUFFIX.txt"
MAXP=40; : > /tmp/variant_forks.status
build_prompt() {
  local af="$1"
  python3 - "$CAT" "$SUB" "$af" "$SUFFIX" <<'PY'
import sys
cat, sub, axis_path, suffix_path = sys.argv[1:5]
axis = open(axis_path, encoding="utf-8").read().rstrip()
suffix = open(suffix_path, encoding="utf-8").read().strip()
print(axis); print(); print(suffix.replace("<Category>", cat).replace("<Subcategory>", sub))
PY
}
run_one() {
  local rid="$1" label="$2" af="$3" prompt
  prompt="$(build_prompt "$af")"
  uv run articraft fork --provider dashscope --model qwen3.7-max --thinking-level high \
    --skip-search-index --record-id "$rid" --label "$label" "$PARENT" "$prompt" \
    > "/tmp/fork_${rid}.log" 2>&1
  echo "EXIT ${rid} = $?" >> /tmp/variant_forks.status
}
export -f build_prompt run_one; export CAT SUB PARENT SUFFIX
printf '%s\n' \
  "rec_<slug>_var_<axis1>  <slug>-<axis1>  /tmp/<slug>_var_<axis1>.txt" \
  "rec_<slug>_var_<axis2>  <slug>-<axis2>  /tmp/<slug>_var_<axis2>.txt" > /tmp/variant_jobs.txt
xargs -P "$MAXP" -n 3 -a /tmp/variant_jobs.txt bash -c 'run_one "$1" "$2" "$3"' _
echo "ALL_DONE" >> /tmp/variant_forks.status
```
Launch and monitor:
```bash
bash /tmp/run_variant_forks.sh
tail -f /tmp/fork_<record_id>.log
tail -f /tmp/variant_forks.status
```
For multiple parents, run one job group per parent or add parent path as a fourth job field. Never use a normal semantic variant as parent.

---

## Step 5 — Rebuild Index Once
```bash
uv run articraft data reconcile
```

---

## Step 6 — Verify Every Variant
```bash
cd /mnt/zsn/lyb/arti-skill/articraft_data
PARENT_MODEL=data/records/<PARENT_RECORD_ID>/revisions/rev_000001/model.py
for rid in rec_<slug>_var_<axis1> rec_<slug>_var_<axis2> ; do
  echo "===== $rid ====="
  URDF=data/cache/record_materialization/$rid/model.urdf
  [ -f "$URDF" ] || uv run articraft compile data/records/$rid
  echo "  non-fixed joints: $(grep -oE 'type=\"[^\"]*\"' "$URDF" | grep -v 'type=\"fixed\"' | wc -l)"
  grep -oE 'type=\"[^\"]*\"' "$URDF" | grep -v 'type=\"fixed\"' | sort | uniq -c | sed 's/^/    /'
  python3 -c "import json;d=json.load(open('data/cache/record_materialization/$rid/compile_report.json'));print('  compile:',d.get('status'),'warnings:',len(d.get('warnings',[])))"
  python3 -c "import json;d=json.load(open('data/records/$rid/record.json'));print('  collections:',d.get('collections'),'category_slug:',d.get('category_slug'))"
  echo "  diff lines vs parent: $(diff "$PARENT_MODEL" data/records/$rid/revisions/rev_000001/model.py | grep -cE '^[<>]')"
done
```
Pass: compile/tests pass; at least one real non-fixed joint unless `static_only`; workbench-only (`dataset` is not in `collections`; `category_slug` may be inherited and is not the promotion gate); diff scoped to primary axis; repeated homogeneous parts loop-emitted; same subcategory; no ④/⑤/⑥-only standalone variants accepted.

---

## Step 7 — Restart Viewer
```bash
pkill -f "uvicorn viewer.api.app:app --host 127.0.0.1 --port 8765"
cd /mnt/zsn/lyb/arti-skill/articraft_data && nohup .venv/bin/python -m uvicorn viewer.api.app:app \
  --host 127.0.0.1 --port 8765 --workers 4 > /tmp/viewer8765.log 2>&1 &
curl -s http://127.0.0.1:8765/api/collections/workbench | grep -o '<slug>_var' | wc -l
```

---

## Step 8 — Cost, Source Map, Registration
```bash
uv run python data/local/build_cost_ledger.py
```
For a complete pool, update `data/local/variant_cost_ledger.csv`, `picture_expansion/template_source_maps/<Category>__<Subcategory>.md`, and `picture_expansion/generated_assets.jsonl`. The source map must include the 6-axis diversity record and the multiplicity/copy-logic record from `FORK_VARIANTS.md`.

---

## Invariants

Fork only from origin/canonical baseline; never from normal semantic variant; one primary axis per ordinary variant; ①/②/③/N must be source-backed; ④ may be recorded/extrapolated/companion but not standalone; ⑤/⑥ are mandatory record/companion axes, not candidate anchors; never use ④/⑤/⑥ to satisfy the 8–30 target; never promote; never use external workflow; append existing `FORK_SUFFIX.txt` mechanically; replace `<Category>` / `<Subcategory>` placeholders; run forks as one main-loop batch; reconcile once; restart viewer.
