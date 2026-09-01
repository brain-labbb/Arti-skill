# Independent protocol audit: PhysX-Omni Table 6 pilot

Snapshot: `2026-08-13T07:46:32Z`

Verdict: **not ready for an unqualified Table 6 method result**. The terminal evidence supports the narrower label **official-method pilot with a disclosed operational adapter on a pre-fixed eight-identity PhysX-Anything mobility test-list cohort**. It does not support `held-out`, a reproducible SHA-256-argmin cohort, collision success, physical inertia fidelity, or independent semantic accuracy. The publication gate remains `false`.

## Findings

1. **HIGH - selection mechanism is not reproducible.** `dependency_integrity.json` enumerated and hashed the exact eight condition images at `2026-08-12T09:06:10Z`, before smoke Stage 1 began at `09:08:54Z`; this supports pre-result identity fixation. However, there is no frozen selection manifest or canonical hash payload. The full 388-ID list, all 388 PartNet category records, salt `physx-omni-official-test-pilot-v1`, and the stated category/asset argmin description do not reproduce the target eight under tested common encodings and category normalizations; the best simple encoding matches 5/8. Report outcome independence of the fixed identities, but mark selection-rule reproducibility `NOT_EVIDENCED` / `FAIL`.

2. **HIGH - this is not demonstrated held-out evaluation.** The pilot uses `testset.npy` from PhysX-Anything commit `7c87bcc252b33cdc10c8583c20ac899136729ad5` (388 numeric IDs; SHA-256 `ccbc2b83...`), not PhysX-Omni's repository-local 400-UUID PhysXVerse test list. PhysX-Omni's pinned README and model card identify PhysX-Mobility as training data, and no exclusion manifest is present. Use "official mobility test-list pilot with potential training overlap."

3. **HIGH - physical collision metrics are not evaluable.** All 7/7 completed URDFs have zero `<collision>` elements. All 47/47 syntactically valid inertials are uniform placeholders (`mass=1`, unit diagonal inertia). Collision geometry, penetration, full-range collision, CCD, and inertia fidelity remain `N/A`/`N/E`; zero contacts are not scored as success.

4. **HIGH - output `102187` has an explicit semantic mismatch.** Upstream ID `102187` is `TrashCan`, while the method output declares `Name: Double Door Refrigerator` and `Category: Appliance`. This is a structural generation success, not a semantic-success example.

5. **MEDIUM - execution used an adapter.** The pristine snapshot is byte-exact to all 334/334 tracked files at official commit `46fa1cd0b6883d4d14431d51c3326ef80a85ef64`. The adapter changes five source files for deterministic ordering/seeds, local model routing, checked subprocesses, and fail-fast diagnostics. The runtime also warns that Transformers selected a fast image processor that may change outputs. Disclose the adapter and freeze the complete package environment.

6. **MEDIUM - terminal execution is 7/8 end-to-end, with one retained incomplete identity.** Stage 1 succeeded for 7/8 identities. All seven Stage-1 successes have Stage 2 cardinality matches, Stage 3 completion, valid static URDF trees, and isolated PyBullet reset/readback passes. ID `102001` retains nine partial part outputs but lacks `allind.npy` and a success marker; it remains incomplete and was not replaced.

7. **MEDIUM - the primary dependency manifest omits the invoked Qwen processor snapshot.** Its original 45/45 entries freshly pass: 23 against expected SHA-256 and 22 against pinned Hugging Face Git blob IDs. This audit separately verifies 11/11 processor/tokenizer files at revision `cc594898...`; the formal self-check must continue to fail this primary-manifest gate.

## Terminal denominators

| Category | ID | Stage 1 | Stage 2 | Stage 3 | Static / PyBullet |
|---|---:|---|---|---|---|
| Display | 4627 | pass, 2 parts | 2/2/2 cardinality pass | complete | pass / pass |
| Clock | 6813 | pass, 6 parts | 6/6/6 cardinality pass | complete | pass / pass |
| Oven | 102001 | **incomplete**, 9 partial parts, no `allind.npy` | N/E | N/E | N/E / N/E |
| Phone | 103593 | pass, 4 parts | 4/4/4 cardinality pass | complete | pass / pass |
| Laptop | 9918 | pass, 3 parts | 3/3/3 cardinality pass | complete | pass / pass |
| TrashCan | 102187 | pass, 6 parts | 6/6/6 cardinality pass | complete | pass / pass; semantic mismatch retained |
| Toaster | 103514 | pass, 4 parts | 4/4/4 cardinality pass | complete | pass / pass |
| Pen | 102916 | pass after harness retry, 2 parts | 2/2/2 cardinality pass | complete | pass / pass |

The first `102916` attempt was a wrapper path error before method execution; the recorded absolute-path retry completed in 124 seconds. It is an infrastructure retry, not a replaced sample.

Terminal aggregate denominators are: intent 8; Stage-1 success 7; end-to-end attributable completion 7; static-valid 7; PyBullet reset/readback 7; collision-evaluable 0; independent-semantic-gold 0. The seven completed assets contain 13 movable joints and 39/39 reset/readback states with maximum absolute error `0.0`.

## Evidence linkage

- `stage_records.json`: SHA-256 `7c434e7673172f4e905da4b847d1b405af70f342b106c33f2d7b86dcad4111ef`; 22 records; Stage 1 intent/success/incomplete = 8/7/1; Stage 2/3 markers = 7/7.
- `asset_records.json`: SHA-256 `b96ee50f80f338d365e287b3642c833dfade43bb79460894bdbb66dfb3ff206b`; attributable/static/PyBullet = 7/7/7; collision-evaluable = 0; semantic-gold = 0.
- `independent_protocol_audit.json` contains the machine-readable per-identity terminal audit and the full fail-closed checklist.

## Remaining formal gates

The execution evidence is terminal, but the formal publication gate remains closed because:

- the selection mechanism has no pre-run canonical selection artifact and cannot be independently reproduced;
- no authoritative training-exclusion manifest supports a held-out claim;
- the primary dependency manifest omits the Qwen processor files and the full runtime environment is not frozen;
- complete per-attempt Stage-1 invocation/timing evidence is not available for every identity;
- no independent semantic gold exists;
- collision metrics are non-evaluable because all completed URDFs have zero collision geometry.

These limitations are intentional `FAIL`, `N/A`, or `N/E` outcomes. They are not hidden by the 7/7 structural and reset/readback execution checks.
