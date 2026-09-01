# Articraft-10K S1 Mechanical Evidence Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and run a frozen, fail-closed Supplementary Table S1 evidence audit for the exact Articraft-10K N=800 Table 2 cohort without executing released model recipes or treating later evaluator output as source evidence.

**Architecture:** A dependency-light core module performs byte binding, evidence qualification, rebuild eligibility, source-URDF topology counting, Table 4 joining, and aggregation. A dataset runner freezes all source identities, emits deterministic per-asset records and human/machine summaries, and writes a content-addressed output manifest; a separate verifier independently recomputes joins, aggregates, and artifact hashes from persisted bytes.

**Tech Stack:** Python 3.12 standard library (`argparse`, `hashlib`, `json`, `pathlib`, `subprocess`, `xml.etree.ElementTree`), `unittest`, existing JSON/JSONL runtime artifacts.

**Spec:** `docs/superpowers/specs/2026-08-21-articraft10k-s1-mechanical-evidence-design.md`

## Global Constraints

- Formal cohort is exactly the 800 `.records[]` entries, in stored order, from `exp/runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json`.
- Bind the exact Table 2 manifest file/content/ordered-ID hashes and frozen Table 4 manifest/asset/state hashes listed in the spec before evaluating any asset.
- Recompute each package's recursive byte binding and `model.urdf` SHA256; drift is fail-closed and is never repaired by resampling or replacement.
- Only source-package machine-readable evidence published before this S1 run may qualify as a receipt or allowance registry.
- Frozen Table 4 records are only an independent strict-result source and may never qualify as a published mechanical receipt.
- Never execute released `model.py`, access the network, or mutate source packages.
- Formal rebuild status is `N/E` when rebuild eligibility is zero; do not substitute a current-SDK rebuild.
- Eligible allowance pairs are unordered collision-bearing source-link pairs excluding direct parent-child pairs.
- Persist parse, schema, path, hash, join, and evidence failures in the full metric denominators.
- This implementation and run do not update `exp/URDF-Sim-Ready-Automatic-Evaluation.md`.

---

### Task 1: Core Binding, Receipt, Rebuild, and Topology Contracts

**Files:**
- Create: `exp/scripts/s1_articraft10k_core.py`
- Create: `exp/tests/test_s1_articraft10k.py`

**Interfaces:**
- Produces: `canonical_sha256(value: object) -> str`, `sha256_file(path: Path) -> str`, `package_binding(package: Path) -> dict[str, object]`, `qualify_mechanical_receipt(report: object, *, asset_id: str, package_content_manifest_sha256: str) -> dict[str, object]`, `audit_rebuild_eligibility(record_root: Path, record: dict[str, object], *, commit_resolver: Callable[[str], bool]) -> dict[str, object]`, and `audit_source_topology(urdf_path: Path) -> dict[str, object]`.
- Consumes: source package bytes, official `record.json` plus revision artifacts, and an injected read-only commit resolver.

- [ ] **Step 1: Write failing binding and receipt tests**

```python
def test_receipt_requires_all_bound_fields(self):
    receipt = {
        "asset_id": "rec_fixture",
        "package_content_manifest_sha256": "a" * 64,
        "protocol": {"id": "mechanical-v1", "runner_sha256": "b" * 64},
        "pair_policy": "distinct_non_adjacent_collision_links",
        "threshold": {"kind": "penetration", "value": 1e-6, "unit": "m"},
        "conclusion": {"strict_pass": True},
    }
    result = core.qualify_mechanical_receipt(
        {"mechanical_receipt": receipt},
        asset_id="rec_fixture",
        package_content_manifest_sha256="a" * 64,
    )
    self.assertTrue(result["qualified"])
    del receipt["threshold"]
    self.assertFalse(core.qualify_mechanical_receipt(
        {"mechanical_receipt": receipt},
        asset_id="rec_fixture",
        package_content_manifest_sha256="a" * 64,
    )["qualified"])
```

Also assert that canonical package bindings are path-sorted, reject symlinks/non-regular files, match Table 2's `{path, bytes, sha256}` closure schema, and report asset/hash mismatches rather than accepting a syntactically complete receipt.

- [ ] **Step 2: Run the focused tests and verify RED**

Run: `python -m unittest exp.tests.test_s1_articraft10k.S1CoreContractTest -v`

Expected: FAIL because `exp/scripts/s1_articraft10k_core.py` and the contract functions do not exist.

- [ ] **Step 3: Implement canonical binding and strict receipt qualification**

```python
def qualify_mechanical_receipt(report, *, asset_id, package_content_manifest_sha256):
    issues = []
    receipt = report.get("mechanical_receipt") if isinstance(report, dict) else None
    if not isinstance(receipt, dict):
        return {"qualified": False, "issues": ["mechanical_receipt_missing_or_invalid"]}
    protocol = receipt.get("protocol")
    threshold = receipt.get("threshold")
    conclusion = receipt.get("conclusion")
    required = {
        "asset_identity": receipt.get("asset_id") == asset_id,
        "package_closure": receipt.get("package_content_manifest_sha256") == package_content_manifest_sha256,
        "protocol_id": isinstance(protocol, dict) and bool(protocol.get("id")),
        "runner_identity": isinstance(protocol, dict) and is_sha256(protocol.get("runner_sha256")),
        "pair_policy": bool(receipt.get("pair_policy")),
        "threshold": valid_threshold(threshold),
        "conclusion": isinstance(conclusion, dict) and isinstance(conclusion.get("strict_pass"), bool),
    }
    issues.extend(f"missing_or_mismatched:{name}" for name, passed in required.items() if not passed)
    return {"qualified": not issues, "issues": issues, "contract": required}
```

Use JSON canonicalization with sorted keys and ASCII separators. Walk packages without following symlinks, reject unsafe entries, and recompute byte counts and SHA256 for every regular file.

- [ ] **Step 4: Write failing rebuild and topology tests**

```python
def test_rebuild_requires_recipe_inputs_sdk_and_resolvable_commit(self):
    eligible = core.audit_rebuild_eligibility(
        fixture.root, fixture.record, commit_resolver=lambda commit: commit == "c" * 40
    )
    self.assertTrue(eligible["eligible"])
    fixture.provenance["sdk"]["sdk_fingerprint"] = None
    self.assertFalse(core.audit_rebuild_eligibility(
        fixture.root, fixture.record, commit_resolver=lambda _: True
    )["eligible"])

def test_topology_counts_collision_pairs_minus_direct_edges(self):
    result = core.audit_source_topology(write_urdf(
        links=("base", "door", "handle"), collision_links=("base", "door", "handle"),
        joints=(("base", "door"),),
    ))
    self.assertEqual(result["eligible_nonadjacent_pair_count"], 2)
```

Fixtures must cover missing `model.py`, missing or hash-drifted declared inputs, null SDK fingerprint, unavailable commit, malformed XML, duplicate link names, unknown joint endpoints, and a zero-collision-link URDF.

- [ ] **Step 5: Run the new focused tests and verify RED**

Run: `python -m unittest exp.tests.test_s1_articraft10k.S1RebuildAndTopologyTest -v`

Expected: FAIL because rebuild and topology audit functions are absent.

- [ ] **Step 6: Implement rebuild eligibility and source topology audit**

```python
def audit_source_topology(urdf_path):
    root = ET.parse(urdf_path).getroot()
    links = [element.get("name") for element in root.findall("link")]
    collision_links = {element.get("name") for element in root.findall("link") if element.findall("collision")}
    direct = {
        frozenset((parent.get("link"), child.get("link")))
        for joint in root.findall("joint")
        if (parent := joint.find("parent")) is not None and (child := joint.find("child")) is not None
    }
    eligible = {
        frozenset(pair) for pair in itertools.combinations(sorted(collision_links), 2)
        if frozenset(pair) not in direct
    }
    normalized = sorted([sorted(pair) for pair in eligible])
    return {
        "status": "COMPLETE",
        "eligible_pairs": normalized,
        "eligible_nonadjacent_pair_count": len(normalized),
        "eligible_pairs_sha256": canonical_sha256(normalized),
    }
```

Rebuild eligibility must resolve `record.json` artifact paths beneath the record root, verify the model recipe hash, verify a declared content-addressed input manifest (an explicit empty manifest is complete), require a non-null 64-hex SDK fingerprint, and call the injected resolver for the 40-hex provenance commit. Return booleans and stable reason codes for every gate; never import or execute the recipe.

- [ ] **Step 7: Run all Task 1 tests and verify GREEN**

Run: `python -m unittest exp.tests.test_s1_articraft10k.S1CoreContractTest exp.tests.test_s1_articraft10k.S1RebuildAndTopologyTest -v`

Expected: all tests PASS.

- [ ] **Step 8: Commit Task 1**

```bash
git add exp/scripts/s1_articraft10k_core.py exp/tests/test_s1_articraft10k.py
git commit -m "feat: add Articraft S1 evidence contracts"
```

### Task 2: Frozen Cohort and Table 4 Join

**Files:**
- Modify: `exp/scripts/s1_articraft10k_core.py`
- Modify: `exp/tests/test_s1_articraft10k.py`

**Interfaces:**
- Consumes: Task 1 hash/binding functions; exact Table 2 manifest, Table 4 frozen manifest, and Table 4 JSON array records.
- Produces: `load_frozen_cohort(...) -> dict[str, object]`, `load_table4_strict_records(...) -> list[dict[str, object]]`, and `join_table4_record(source_row: dict[str, object], table4_item: dict[str, object], table4_record: dict[str, object], *, order: int) -> dict[str, object]`.

- [ ] **Step 1: Write failing frozen-identity and join tests**

```python
def test_formal_cohort_matches_exact_table2_order_and_hashes(self):
    cohort = core.load_frozen_cohort(TABLE2_MANIFEST, mode="formal")
    self.assertEqual(len(cohort["records"]), 800)
    self.assertEqual([r["selection_index"] for r in cohort["records"]], list(range(800)))
    self.assertEqual(cohort["ordered_asset_ids_sha256"], EXPECTED_ORDERED_IDS_SHA256)

def test_table4_join_requires_order_asset_package_and_urdf_identity(self):
    joined = core.join_table4_record(source, item, result, order=0)
    self.assertEqual(joined["strict_collision_pass"], result["strict_collision_pass"])
    bad = copy.deepcopy(result)
    bad["model_urdf_sha256"] = "0" * 64
    with self.assertRaisesRegex(ValueError, "model_urdf_sha256"):
        core.join_table4_record(source, item, bad, order=0)
```

Add one mutation test for each frozen top-level hash and for selection order, asset ID, package binding, model hash, Table 4 `dataset_id`, and missing/extra result rows.

- [ ] **Step 2: Run the join tests and verify RED**

Run: `python -m unittest exp.tests.test_s1_articraft10k.S1FrozenJoinTest -v`

Expected: FAIL because frozen loaders and join functions do not exist.

- [ ] **Step 3: Implement fail-fast frozen identity loading and join checks**

```python
EXPECTED_INPUTS = {
    "table2_manifest_sha256": "13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d",
    "table2_content_sha256": "576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3",
    "ordered_asset_ids_sha256": "79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784",
    "table4_manifest_sha256": "6b4275cf3da29244af70c04acecd87094f0c158dee992db20b04e90c05292c20",
    "table4_content_sha256": "1c6ba7d9e19818580fe8573cf95bb1d065bf2235d0699070516888520f86d7b6",
    "table4_asset_records_sha256": "b732a53a464a8aeebb74799d5ec737de75f3cca377c9a5b274a5dd35adbe301b",
    "table4_state_records_sha256": "6efd4031ecebf74f30f8d3ec3c312ae2faf1b521322b5d4a8b57bb732177ac8b",
}
```

Formal loading validates all constants, `manifest_content_sha256`, 800 rows, contiguous stored indices, unique IDs/packages, and the canonical ordered-ID hash. Parse the Table 4 asset file as one JSON array, require exactly 800 items and results, and join positionally plus by `asset_id`, `dataset_id`, `package_content_manifest_sha256`, package binding object, and both URDF hash field names.

- [ ] **Step 4: Run the join tests and verify GREEN**

Run: `python -m unittest exp.tests.test_s1_articraft10k.S1FrozenJoinTest -v`

Expected: all tests PASS.

- [ ] **Step 5: Commit Task 2**

```bash
git add exp/scripts/s1_articraft10k_core.py exp/tests/test_s1_articraft10k.py
git commit -m "feat: bind Articraft S1 frozen inputs"
```

### Task 3: Per-Asset Audit, Allowance Registry, and S1 Aggregation

**Files:**
- Modify: `exp/scripts/s1_articraft10k_core.py`
- Modify: `exp/tests/test_s1_articraft10k.py`

**Interfaces:**
- Consumes: Task 1 audits and Task 2 frozen join.
- Produces: `qualify_allowance_registry(report: object, topology: dict[str, object]) -> dict[str, object]`, `audit_asset(...) -> dict[str, object]`, `aggregate_records(records: Sequence[dict[str, object]], *, intended_count: int) -> dict[str, object]`, and `render_summary_markdown(summary: dict[str, object]) -> str`.

- [ ] **Step 1: Write failing allowance and aggregation tests**

```python
def test_empty_published_registry_has_zero_density_and_gain(self):
    record = core.audit_asset(source, table4_item, table4_result, official_root, commit_resolver=lambda _: False)
    self.assertEqual(record["registered_method_allowance_pair_count"], 0)
    self.assertEqual(record["strict_collision_pass_registered_allowance"], record["strict_collision_pass_no_method_allowance"])

def test_zero_rebuild_eligibility_renders_not_evaluable(self):
    summary = core.aggregate_records(records, intended_count=2)
    self.assertEqual(summary["metrics"]["deterministic_rebuild_match"]["status"], "N/E")
    self.assertEqual(summary["metrics"]["deterministic_rebuild_match"]["eligible"], 0)
    self.assertEqual(summary["metrics"]["registered_allowance_gain_pp"]["value"], 0.0)
```

Also test a concrete in-topology pair qualifies, an unknown/self/adjacent pair fails closed, unexecuted `model.py` allowance calls remain diagnostics only, malformed URDF retains a record with null topology denominator, receipt replay is false for every unqualified receipt, and all rates use the intended asset denominator rather than dropping failures.

- [ ] **Step 2: Run asset/aggregate tests and verify RED**

Run: `python -m unittest exp.tests.test_s1_articraft10k.S1AssetAggregateTest -v`

Expected: FAIL because allowance, asset, aggregate, and rendering functions do not exist.

- [ ] **Step 3: Implement strict allowance qualification and per-asset audit**

```python
def qualify_allowance_registry(report, topology):
    entries = report.get("overlap_allowances") if isinstance(report, dict) else None
    if not isinstance(entries, list):
        return {"qualified": False, "registered_pair_count": 0, "issues": ["registry_missing_or_invalid"]}
    normalized = []
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get("link_a"), str) or not isinstance(entry.get("link_b"), str):
            return {"qualified": False, "registered_pair_count": 0, "issues": ["registry_entry_invalid"]}
        pair = tuple(sorted((entry["link_a"], entry["link_b"])))
        if pair not in topology["eligible_pairs"]:
            return {"qualified": False, "registered_pair_count": 0, "issues": ["registry_pair_not_eligible"]}
        normalized.append(pair)
    return {"qualified": True, "registered_pair_count": len(set(normalized)), "issues": []}
```

The asset record must retain selection order, asset/package/model hashes, compile-report hash and parse status, receipt contract gates, replay status/reason, rebuild gates/reasons, topology counts/hash, allowance qualification/count, Table 4 join bindings, strict no-allowance result, identical registered-allowance result for this empty-registry cohort, and a flat issue list. A missing official record or malformed source file remains a completed fail-closed asset audit, not an omitted row.

- [ ] **Step 4: Implement denominator-preserving aggregates and markdown**

```python
metrics = {
    "receipt_bound_assets": rate(sum(r["receipt_qualified"] for r in records), intended_count),
    "receipt_replay_pass": rate(sum(r["receipt_replay_pass"] for r in records), intended_count),
    "deterministic_rebuild_match": rebuild_metric(records, intended_count),
    "allowance_density": pair_rate(sum(r["registered_method_allowance_pair_count"] for r in records), sum_int(r["eligible_nonadjacent_pair_count"] for r in records)),
    "strict_pass_no_method_allowance": rate(sum(r["strict_collision_pass_no_method_allowance"] for r in records), intended_count),
    "registered_allowance_gain_pp": gain_metric(records, intended_count),
}
```

`rebuild_metric` returns `status: "N/E"`, `matched: null`, `eligible: 0`, `intended: 800`, and `rate: null` when no asset is eligible. Allowance density retains both summed pair counts and reports `0.0` only when the numerator is explicitly zero and at least one topology denominator is audited; malformed topologies are counted in an asset coverage field. Markdown renders exact numerators, denominators, percentages, `N/E`, coverage, and issue counts without inventing values.

- [ ] **Step 5: Run asset/aggregate tests and verify GREEN**

Run: `python -m unittest exp.tests.test_s1_articraft10k.S1AssetAggregateTest -v`

Expected: all tests PASS.

- [ ] **Step 6: Commit Task 3**

```bash
git add exp/scripts/s1_articraft10k_core.py exp/tests/test_s1_articraft10k.py
git commit -m "feat: aggregate Articraft S1 evidence metrics"
```

### Task 4: Runner, Frozen Outputs, and Independent Verifier

**Files:**
- Create: `exp/scripts/run_s1_articraft10k.py`
- Create: `exp/scripts/verify_s1_articraft10k.py`
- Modify: `exp/tests/test_s1_articraft10k.py`

**Interfaces:**
- Consumes: all Task 1-3 core APIs and the approved design spec.
- Produces: CLI `run_s1_articraft10k.py --mode {smoke,formal} [--output-dir PATH] [--smoke-count N]`; independent CLI `verify_s1_articraft10k.py RUN_DIR`; run artifacts `frozen_config.json`, `protocol_snapshot.md`, `asset_records.jsonl`, `summary.json`, `summary.md`, and `manifest.json`.

- [ ] **Step 1: Write failing CLI and artifact tests**

```python
def test_smoke_cli_writes_complete_bound_artifact_set(self):
    completed = subprocess.run([
        sys.executable, str(RUNNER), "--mode", "smoke", "--smoke-count", "3",
        "--output-dir", str(output_dir),
    ], cwd=REPO, text=True, capture_output=True)
    self.assertEqual(completed.returncode, 0, completed.stderr)
    self.assertEqual({p.name for p in output_dir.iterdir()}, {
        "frozen_config.json", "protocol_snapshot.md", "asset_records.jsonl",
        "summary.json", "summary.md", "manifest.json",
    })
    manifest = read_json(output_dir / "manifest.json")
    self.assertEqual(manifest["verification"]["passed"], manifest["verification"]["total"])
```

Add tests that formal mode rejects a non-800 cohort, rerunning into an existing non-empty directory fails, protocol snapshot bytes equal the approved spec bytes, every input/output has a SHA256 entry, and tampering with one record or one artifact hash causes the independent verifier to exit nonzero.

- [ ] **Step 2: Run CLI tests and verify RED**

Run: `python -m unittest exp.tests.test_s1_articraft10k.S1RunnerVerifierTest -v`

Expected: FAIL because runner and verifier CLIs do not exist.

- [ ] **Step 3: Implement the smoke/formal runner**

```python
def main(argv=None):
    args = parse_args(argv)
    cohort = core.load_frozen_cohort(TABLE2_MANIFEST, mode=args.mode)
    table4 = core.load_table4_strict_records(TABLE4_MANIFEST, TABLE4_ASSET_RECORDS, TABLE4_STATE_RECORDS, mode=args.mode)
    selected = cohort["records"] if args.mode == "formal" else cohort["records"][:args.smoke_count]
    records = [audit_one(index, source, table4, args) for index, source in enumerate(selected)]
    summary = core.aggregate_records(records, intended_count=800 if args.mode == "formal" else len(selected))
    write_frozen_artifacts(args.output_dir, records, summary)
    verification = verify_written_run(args.output_dir)
    finalize_manifest(args.output_dir, verification)
    return 0 if verification["passed"] == verification["total"] else 1
```

Use an explicit default output name `s1_articraft10k_<mode>_n<count>_<UTC timestamp>`. Freeze absolute input paths and exact file/content hashes, runner/core/verifier/design hashes, Python identity, mode/count, protocol rules, and a `model_py_execution: false` declaration. Use a temporary sibling directory and atomic rename so incomplete output is never presented as finalized. The commit resolver may run only `git cat-file -e <40hex>^{commit}` against the frozen official source repository; it must not fetch.

- [ ] **Step 4: Implement an independent persisted-byte verifier**

```python
def verify_run(run_dir):
    checks = []
    config = read_json(run_dir / "frozen_config.json")
    records = read_jsonl(run_dir / "asset_records.jsonl")
    summary = read_json(run_dir / "summary.json")
    checks.append(check_frozen_input_hashes(config))
    checks.append(check_order_and_record_bindings(config, records))
    checks.append(check_table4_join_from_source_bytes(config, records))
    checks.append(check_recomputed_metrics(records, summary))
    checks.append(check_output_hashes(run_dir))
    checks.append(check_formal_invariants(config, records, summary))
    return {"passed": sum(checks), "total": len(checks), "checks": checks}
```

Do not import `s1_articraft10k_core` in the verifier. Independently parse JSON/JSONL, canonicalize hashes, reopen Table 2 and Table 4 sources, enforce positional/identity/hash joins, recompute all six S1 metrics and coverage fields, and check every non-manifest artifact hash. The manifest binds itself with `manifest_content_sha256 = canonical_sha256(manifest_without_manifest_content_sha256)`, which the verifier recomputes independently. Formal invariants require 800 rows, receipt 0/800, replay 0/800, rebuild `N/E` with eligibility 0/800, allowance numerator 0, strict pass 147/800, and gain exactly 0 pp; these are verification consequences, not runner hard-codes.

- [ ] **Step 5: Run CLI tests and verify GREEN**

Run: `python -m unittest exp.tests.test_s1_articraft10k.S1RunnerVerifierTest -v`

Expected: all tests PASS.

- [ ] **Step 6: Run the complete S1 test file**

Run: `python -m unittest exp.tests.test_s1_articraft10k -v`

Expected: all tests PASS with no source-package writes.

- [ ] **Step 7: Commit Task 4**

```bash
git add exp/scripts/run_s1_articraft10k.py exp/scripts/verify_s1_articraft10k.py exp/tests/test_s1_articraft10k.py
git commit -m "feat: add frozen Articraft S1 runner"
```

### Task 5: Smoke Gate, Formal N=800 Audit, and Final Verification

**Files:**
- Create at runtime: `exp/runtime/s1_articraft10k_smoke_n8_<timestamp>/`
- Create at runtime: `exp/runtime/s1_articraft10k_formal_n800_<timestamp>/`
- Modify only if a defect is exposed: `exp/scripts/s1_articraft10k_core.py`, `exp/scripts/run_s1_articraft10k.py`, `exp/scripts/verify_s1_articraft10k.py`, `exp/tests/test_s1_articraft10k.py`

**Interfaces:**
- Consumes: Task 4 CLIs and the frozen source bytes.
- Produces: one disposable smoke result, one complete formal N=800 result, and independent verification evidence.

- [ ] **Step 1: Record read-only source-tree state before execution**

Run:

```bash
git status --short
find exp/Articraft-10K/released_urdf exp/baselines/Articraft-10K-official/records -type f -printf '%p\t%s\t%T@\n' | sha256sum
```

Expected: capture the existing dirty-worktree listing and one source-tree metadata digest for post-run comparison; do not clean unrelated changes.

- [ ] **Step 2: Run an 8-asset smoke audit**

Run:

```bash
python exp/scripts/run_s1_articraft10k.py --mode smoke --smoke-count 8
```

Expected: exit 0, exactly 8 ordered records, six output artifacts, and all internal verification checks passed.

- [ ] **Step 3: Independently verify the smoke directory**

Run: `python exp/scripts/verify_s1_articraft10k.py exp/runtime/s1_articraft10k_smoke_n8_<timestamp>`

Expected: exit 0 and every reported check passes.

- [ ] **Step 4: Run the formal 800-asset audit**

Run:

```bash
python exp/scripts/run_s1_articraft10k.py --mode formal
```

Expected: exit 0, all 800 source rows retained in order, no source recipe execution, and a finalized `exp/runtime/s1_articraft10k_formal_n800_<timestamp>` directory.

- [ ] **Step 5: Independently verify formal persisted bytes**

Run: `python exp/scripts/verify_s1_articraft10k.py exp/runtime/s1_articraft10k_formal_n800_<timestamp>`

Expected: exit 0; 800/800 order and byte bindings pass; summary recomputation matches; formal invariants show receipt 0/800, replay 0/800, rebuild `N/E` at 0/800 eligibility, zero registered allowances, strict no-allowance pass 147/800, and registered gain 0 pp.

- [ ] **Step 6: Confirm source bytes were not mutated**

Run:

```bash
find exp/Articraft-10K/released_urdf exp/baselines/Articraft-10K-official/records -type f -printf '%p\t%s\t%T@\n' | sha256sum
```

Expected: digest exactly matches Step 1.

- [ ] **Step 7: Run the final regression suite and inspect the formal summary**

Run:

```bash
python -m unittest exp.tests.test_s1_articraft10k -v
jq '.status, .coverage, .metrics' exp/runtime/s1_articraft10k_formal_n800_<timestamp>/summary.json
git status --short
```

Expected: all tests pass; summary has no unexplained missing records; only scoped implementation/runtime files plus pre-existing user changes appear.

- [ ] **Step 8: Commit any test-driven corrections only**

```bash
git add exp/scripts/s1_articraft10k_core.py exp/scripts/run_s1_articraft10k.py exp/scripts/verify_s1_articraft10k.py exp/tests/test_s1_articraft10k.py
git diff --cached --check
git commit -m "fix: finalize Articraft S1 audit verification"
```

Skip this commit when Steps 2-7 require no implementation correction. Runtime directories remain uncommitted unless the repository's existing evaluation-artifact policy explicitly tracks them.
