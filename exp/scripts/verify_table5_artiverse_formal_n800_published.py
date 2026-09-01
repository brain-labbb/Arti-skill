#!/usr/bin/env python3
"""Independent read-only validator for the Artiverse Table 5 formal N=800 publication.

Verifies, without touching result files or GPUs:
  1. Completion markers (self_check.json, per-simulator summary.json).
  2. Independent recomputation of every Table 5a/5b count from the 2,400
     per-asset records under formal/<sim>/assets/.
  3. SHA-256 integrity binding (aggregate_set.json file_hashes, receipt
     record_file_hashes for all per-asset records).
  4. Cohort provenance: frozen run manifest vs repo Table 1 manifest ordering.
  5. Upstream strict gates recomputed from cohort rows.
  6. Failure-inventory consistency.
  7. Published document values vs aggregate/formal/table5.json.

Exit code 0 = PASS, 1 = FAIL.
"""
import hashlib
import json
import os
import sys

RUN = "/root/.cache/torch/arti-skill/table5_artiverse_table1_n800_gpu_v4"
FORMAL = os.path.join(RUN, "aggregate", "formal")
REPO_MANIFEST = "/mnt/zsn/lyb/arti-skill/exp/runtime/table1_artiverse/manifest.json"
DOC = "/mnt/zsn/lyb/arti-skill/exp/URDF-Sim-Ready-Automatic-Evaluation.md"
SIMS = ["pybullet", "mujoco", "genesis"]
METRICS = ["load", "reset", "settling", "actuation",
           "limit_enforcement", "constraint_drift", "simulator_pass"]
EXPECTED_COHORT = "3e12e86fa61b9af14a411a2571c100e49f3ad49f6286394453366a64caeeb171"
EXPECTED_PROTOCOL = "ebd1e6599f782511b0974208a0294cb2e42a7f1645614ac9a4e49df13c91e551"

errors = []


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def check(cond, msg):
    if not cond:
        errors.append(msg)


# ---- 1. completion markers -------------------------------------------------
sc = json.load(open(os.path.join(FORMAL, "self_check.json")))
check(sc.get("selected_count") == 800, "self_check selected_count != 800")
comp = sc.get("completion") or {}
check(comp.get("formal_claim_complete") is True and comp.get("state") == "complete",
      "self_check completion not formal-complete")
checks = sc.get("checks") or {}
check(bool(checks) and all(v is True for v in checks.values()),
      f"self_check checks not all true: {checks}")

t5 = json.load(open(os.path.join(FORMAL, "table5.json")))
check(t5.get("formal_claim_complete") is True and t5.get("state") == "complete",
      "table5.json not formal_claim_complete/complete")
check(t5.get("run_phase") == "formal", "table5.json run_phase != formal")
intent = t5.get("intent") or {}
ids = intent.get("dataset_ids") or []
check(intent.get("count") == 800 and len(ids) == 800 and len(set(ids)) == 800
      and ids == [f"artiverse_{i:04d}" for i in range(800)],
      "table5.json intent roster is not exactly artiverse_0000..0799")

# ---- 2. independent recomputation from per-asset records -------------------
per_sim = {}
for s in SIMS:
    d = os.path.join(RUN, "formal", s, "assets")
    files = sorted(os.listdir(d))
    check(files == [f"artiverse_{i:04d}.json" for i in range(800)],
          f"{s}: asset roster mismatch")
    counts = {m: 0 for m in METRICS}
    term = {}
    for f in files:
        a = json.load(open(os.path.join(d, f)))
        m = a.get("metrics") or {}
        for k in METRICS:
            if m.get(k) is True:
                counts[k] += 1
        term[a.get("terminal_status")] = term.get(a.get("terminal_status"), 0) + 1
        check(a.get("run_phase") == "formal", f"{s}/{f}: run_phase != formal")
        check(a.get("terminal") is True, f"{s}/{f}: terminal != true")
    per_sim[s] = (counts, term)

    summ = json.load(open(os.path.join(RUN, "formal", s, "summary.json")))
    check(summ.get("complete") is True and summ.get("remaining_count") == 0,
          f"{s}: summary not complete")
    check(summ.get("intent_count") == 800 and summ.get("metric_denominator") == 800
          and summ.get("terminal_count") == 800, f"{s}: summary counts != 800")
    check(summ.get("metric_pass_counts") == counts,
          f"{s}: summary metric_pass_counts {summ.get('metric_pass_counts')} != recomputed {counts}")
    check(summ.get("terminal_status_counts") == term,
          f"{s}: summary terminal_status_counts mismatch")

# table5a / table5b reconciliation
a, b = t5["table5a"], t5["table5b"]
for s in SIMS:
    counts, _ = per_sim[s]
    for k in METRICS:
        cell = a[s][k]
        check(cell["denominator"] == 800 and cell["passed"] == counts[k],
              f"table5a[{s}][{k}] mismatch vs recomputed")
    check(a[s]["strict_collision_pass"] == {"denominator": 800, "passed": 254, "percentage": 31.75},
          f"table5a[{s}].strict_collision_pass unexpected")
    check(b["per_simulator_pass"][s]["passed"] == counts["simulator_pass"],
          f"table5b.per_simulator_pass[{s}] mismatch")

load_all = rt_all = 0
for i in range(800):
    aid = f"artiverse_{i:04d}.json"
    ms = [json.load(open(os.path.join(RUN, "formal", s, "assets", aid))).get("metrics") or {}
          for s in SIMS]
    if all(m.get("load") is True for m in ms):
        load_all += 1
    if all(m.get("simulator_pass") is True for m in ms):
        rt_all += 1
check(b["all_three_load"]["passed"] == load_all, "all_three_load mismatch")
check(b["all_three_runtime_pass"]["passed"] == rt_all, "all_three_runtime_pass mismatch")

# ---- 3. hash binding --------------------------------------------------------
aset = json.load(open(os.path.join(FORMAL, "aggregate_set.json")))
check(aset.get("cohort_sha256") == EXPECTED_COHORT, "aggregate_set cohort hash")
check(aset.get("protocol_sha256") == EXPECTED_PROTOCOL, "aggregate_set protocol hash")
for name, want in (aset.get("file_hashes") or {}).items():
    check(sha256(os.path.join(FORMAL, name)) == want, f"file_hashes mismatch: {name}")

receipt = t5.get("receipt") or {}
check(receipt.get("cohort_sha256") == EXPECTED_COHORT, "receipt cohort hash")
check(receipt.get("protocol_sha256") == EXPECTED_PROTOCOL, "receipt protocol hash")
for s, node in (receipt.get("runtime_inputs") or {}).items():
    recs = node.get("record_file_hashes") or {}
    check(len(recs) == 800, f"receipt[{s}] record hash count != 800")
    for fname, want in recs.items():
        p = os.path.join(RUN, "formal", s, "assets", fname)
        check(os.path.exists(p) and sha256(p) == want, f"receipt record hash mismatch {s}/{fname}")

# ---- 4. cohort provenance ---------------------------------------------------
fm = json.load(open(os.path.join(RUN, "manifest.json")))
check(sha256(os.path.join(RUN, "manifest.json")) == receipt.get("manifest_file_sha256"),
      "frozen manifest hash != receipt manifest_file_sha256")
check(fm.get("cohort_sha256") == EXPECTED_COHORT, "frozen manifest cohort hash")
check(fm.get("protocol_sha256") == EXPECTED_PROTOCOL, "frozen manifest protocol hash")
rows = fm.get("rows") or []
check(len(rows) == 800, "frozen manifest rows != 800")
rm = json.load(open(REPO_MANIFEST))
repo_seq = [x.get("manifest_root") for x in rm.get("assets", [])]
row_seq = [r.get("manifest_root") for r in rows]
check(row_seq == repo_seq, "frozen rows not identical to repo Table 1 manifest order")
omr = (fm.get("selection") or {}).get("ordered_manifest_root_sha256")
check(omr == hashlib.sha256(json.dumps(repo_seq, separators=(",", ":")).encode()).hexdigest(),
      "ordered_manifest_root_sha256 not reproducible from repo manifest")
check([r.get("dataset_id") for r in rows] == [f"artiverse_{i:04d}" for i in range(800)],
      "frozen rows dataset_id ordering mismatch")

# ---- 5. upstream strict gates from cohort rows ------------------------------
urdf = kin = col = 0
for r in rows:
    g = r.get("strict_gates") or {}
    u = (g.get("table2") or {}).get("strict_urdf_pass") is True
    k = (g.get("table3") or {}).get("strict_kinematic_pass") is True
    c = (g.get("table4") or {}).get("strict_collision_pass") is True
    urdf += u; kin += k; col += c
for key, val in (("strict_urdf_pass", urdf), ("strict_kinematic_pass", kin),
                 ("strict_collision_pass", col)):
    cell = b.get(key) or {}
    check(cell.get("denominator") == 800 and cell.get("passed") == val,
          f"table5b.{key} {cell} != cohort-row recompute {val}")

# ---- 6. failure inventory ----------------------------------------------------
fi = json.load(open(os.path.join(FORMAL, "failure_inventory.json")))
recs = fi.get("records") or []
by_sim = {}
for r in recs:
    by_sim[r.get("simulator")] = by_sim.get(r.get("simulator"), 0) + 1
for s in SIMS:
    expected = 800 - per_sim[s][0]["simulator_pass"]
    check(by_sim.get(s, 0) == expected,
          f"failure_inventory records[{s}]={by_sim.get(s, 0)} != expected {expected}")

# ---- 7. document cross-check --------------------------------------------------
doc = open(DOC, encoding="utf-8").read()
expected_doc_strings = [
    "8 / 800 (1.000%)", "0 / 800 (0.000%)",
    "784 / 800 (98.000%)", "278 / 800 (34.750%)", "662 / 800 (82.750%)",
    "Strict URDF 774 / 800 (96.750%)", "Strict Kinematic 762 / 800 (95.250%)",
    "Strict Collision 254 / 800 (31.750%)",
    "revolute joint n=914", "prismatic joint n=1591", "link-pose n=4274",
    "rev max 3095.693403", "prism max 321.180421",
    "trans max 84.273932", "rot max 3.141592",
    "PyBullet completed=784, preflight_failure=3, timeout=13",
    "Genesis completed=754, diagnostic_failure=22, native_crash=5, preflight_failure=3, timeout=12, worker_error=4",
    "MuJoCo completed=662, diagnostic_failure=134, preflight_failure=3, timeout=1",
    EXPECTED_COHORT, EXPECTED_PROTOCOL,
]
for s in expected_doc_strings:
    check(s in doc, f"document missing/changed published value: {s!r}")

# ---- verdict -------------------------------------------------------------------
if errors:
    print("VALIDATION FAILED:")
    for e in errors:
        print(" -", e)
    sys.exit(1)
print("VALIDATION PASS: Artiverse Table 5 formal N=800 publication is internally "
      "consistent, hash-bound, cohort-provenance-closed, and matches the document.")
