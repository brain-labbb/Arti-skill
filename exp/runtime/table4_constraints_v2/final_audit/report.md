# Table 4 Constraints v2: final independent audit

Status: complete for all requested methods. Artiverse is reported only as a fixed-dataset prompt-only retrieval/reference supplementary, not as generation or a same-prompt method. This audit did not run generation, edit the frozen protocol/scorer/canonicalizer, or modify `exp/Nano3dresults.md`.

## Reporting decision

The paper headline should be **numeric constraint pass over 20 frozen constraints**, reported together with **successful artifact coverage over 18 tasks**. The scorer's `coverage` field is measurable-constraint coverage, not artifact coverage; both are shown separately below. The 32 exact-count constraints are only a supplementary name-matched renderable-node proxy.

## Numeric primary

| Method | Comparison panel | Numeric pass | Numeric measurable | Artifact coverage | Backend disclosure |
|---|---|---:|---:|---:|---|
| BlenderLLM | Native structured | 4/20 (20.0%) | 20/20 (100.0%) | 18/18 (100.0%) | Official pinned model/code |
| Articraft clean v2 | Native structured | 14/20 (70.0%) | 15/20 (75.0%) | 13/18 (72.2%) | Official code + observed DashScope `qwen3.6-plus` backend |
| LAM | Native structured | 10/20 (50.0%) | 10/20 (50.0%) | 8/18 (44.4%) | Official pipeline + DashScope `qwen3.8-max`; `paper_backend_exact=false` |
| Text-to-CadQuery | Native CAD | 0/20 (0.0%) | 13/20 (65.0%) | 11/18 (61.1%) | Official pinned model/code |
| Naive same-LLM | Matched-LLM ablation | 20/20 (100.0%) | 20/20 (100.0%) | 18/18 (100.0%) | `gpt-5.6-sol`, high, repair 0 |
| Ours full public docs | Matched-LLM ablation | 11/20 (55.0%) | 11/20 (55.0%) | 10/18 (55.6%) | `gpt-5.6-sol`, high, repair 0; post-audit amended arm |
| Ours narrow-doc | Diagnostic only | 1/20 (5.0%) | 1/20 (5.0%) | 1/18 (5.6%) | 11/18 tasks fail-closed for documentation-scope violations |

Do not pool the native and matched-LLM panels into an unconditional method ranking. The naive and Ours prompts expose different authoring resources from native method pipelines and therefore constitute a controlled same-LLM ablation.

## Native generation panel

| Method | Modality | Numeric pass | Artifact coverage | Count proxy | Status |
|---|---|---:|---:|---:|---|
| BlenderLLM | Structured | 4/20 | 18/18 | 23/32 | Complete |
| Articraft clean v2 | Structured | 14/20 | 13/18 | 18/32 | Complete, DashScope backend disclosed |
| LAM | Structured | 10/20 | 8/18 | 11/32 | Complete, adapted backend |
| Text-to-CadQuery | CAD | 0/20 | 11/18 | N/A | Complete |
| CAD-Coder | Image-to-CAD supplementary | N/A | 15/18 | N/R | Execution complete; count not reportable |

LAM had 8 pipeline-complete tasks and 10 timeouts. Four timeout checkpoints exist only as supplementary diagnostics and were excluded from the primary scorer. Articraft clean v2 is the only Articraft cohort in this aggregate; the legacy boundary-incident cohort is excluded.

## Matched-LLM ablation

| Arm | Model / effort | Repair | Numeric pass | Artifact coverage | Count proxy |
|---|---|---:|---:|---:|---:|
| Naive same-LLM | `gpt-5.6-sol` / high | 0 | 20/20 | 18/18 | 29/32 |
| Ours full public docs | `gpt-5.6-sol` / high | 0 | 11/20 | 10/18 | 14/32 |

Ours full public docs is a post-audit matched-LLM ablation frozen by an amendment, not an unqualified native-method row. Two workspace-boundary incidents were caused by the post-generation audit agent; `boundary_incident.json` records that neither affected a formal task (`formal_cohort_effect: none`). Its compliance audit also discloses an amendment path-prefix wording discrepancy while confirming all 18 generation tasks used the intended six public documents and public SDK scope.

## Dataset retrieval reference supplementary

| Method | Candidate / eligible assets | Selected artifacts | Numeric pass | Numeric measurable | Count proxy |
|---|---:|---:|---:|---:|---:|
| Artiverse prompt-only CLIP retrieval | 3544 / 3544 | 18/18 (100.0%) | 1/20 (5.0%) | 20/20 (100.0%) | 1/32 (3.1%) |

This is retrieval from a fixed human-authored dataset, not Artiverse generation and not a same-prompt generation method. The selector used each exact frozen prompt for deterministic global top-1 CLIP retrieval over exactly-16-view eligible assets. It used render and identity inputs only; geometry was opened only after the 18-row selection lock. The formal cohort used zero rank fallback, zero repair, and no target-driven rescaling.

The append-only pre-result chain is intact. Addenda 1 and 2 contained future UTC values; Addendum 3 preserved those files, corrected only the time provenance, and required a new cohort with all source gates repeated. The invalid builder attempt ended during the extracted-tree scan with no snapshot, no selector/materializer start, no selection rows, and no scientific result; it is SHA-256 locked and excluded. The corrected pre-result audit passed before formal snapshot/selection existed. Two fresh full embedding runs, prompt embeddings, and selections are byte-identical, and materialization independently recomputed every global top-1 choice before opening geometry. Isolation is accurately scoped as workspace-local read-only snapshot plus code-audited inputs, not an OS sandbox.

## Named-node count proxy

| Method | Proxy pass |
|---|---:|
| Naive same-LLM | 29/32 (90.6%) |
| BlenderLLM | 23/32 (71.9%) |
| Articraft clean v2 | 18/32 (56.2%) |
| Ours full public docs | 14/32 (43.8%) |
| LAM | 11/32 (34.4%) |
| Ours narrow-doc diagnostic | 2/32 (6.2%) |
| Artiverse retrieval reference | 1/32 (3.1%) |

This count is not semantic exact-count accuracy. The frozen scorer matches aliases as substrings over mesh-bearing node names and selects one representation level. Subpart granularity can therefore over-count: `drawer` can match body/front/pull nodes; `wheel`/`caster` can match stem/mount/hub/tire nodes; `door` can match handles; `shelf` can match supports. The naive arm's three count failures are concrete examples of this behavior. Use this table only as a labeled supplementary proxy, never as the headline metric.

## N/R and gated methods

| Method | Attempts | Numeric | Count | Evidence-backed status |
|---|---:|---:|---:|---|
| Text2CAD | 0/18 | N/R | N/R | Official fixed-revision checkpoint unavailable; anonymous request returned HTTP 401 |
| NURBGen | 0/18 | N/R | N/R | Official fixed-revision LoRA unavailable; anonymous request returned HTTP 401 |
| CAD-Coder | 18/18 | N/A | N/R | 15/18 executable STEP+GLB; count judge not pre-frozen and four formal renders were viewed by generation agent |

N/R means the experiment was not reportably scored; it is not a zero. CAD-Coder's only usable outcome is executable artifact coverage, 15/18 (83.3%). Its three native failures were syntax/truncation failures on T4C006, T4C009, and T4C013.

## Reproducibility and evidence

| Method | Manifest rows / successes | Score replay | Integrity | Provenance |
|---|---:|---|---|---|
| BlenderLLM | 18 / 18 | Structured and numeric byte-identical | Final rebuild pass | Commit/model revision and local hashes recorded |
| Articraft clean v2 | 18 / 13 | Structured and numeric byte-identical | Two local runs and final rebuild pass | Clean cohort, zero symlinks pre/post, legacy excluded |
| LAM | 18 / 8 | Structured and numeric byte-identical | Two local runs and final rebuild pass | Official commit; adapted backend explicitly marked |
| Text-to-CadQuery | 18 / 11 | Numeric byte-identical | Final rebuild pass | Official commit/model revision and weight hash recorded |
| Naive same-LLM | 18 / 18 | Structured and numeric byte-identical | Final rebuild pass | 18 unique calls, repair 0, compliance audit pass |
| Ours full public docs | 18 / 10 | Structured and numeric byte-identical | Two local runs and final rebuild pass | Compliance pass; two audit-side incidents disclosed, formal effect none |
| Ours narrow-doc diagnostic | 18 / 1 | Structured and numeric byte-identical | Final rebuild pass under frozen method id `ours` | 11 fail-closed tasks; diagnostic only |
| Artiverse retrieval reference | 18 / 18 | Structured and numeric byte-identical; full selection replay exact | Local and final rebuild pass | Pre-result amendment/addenda/audit chain locked; invalid scan-only attempt excluded; fallback/repair 0 |
| CAD-Coder | 18 / 15 executable | N/A | 15 source/canonical hashes verified by method audit | One attempt/task, repair 0, count N/R |
| Text2CAD | 0 / 0 | N/R | N/R | Gated checkpoint evidence and ready runner recorded |
| NURBGen | 0 / 0 | N/R | N/R | Gated LoRA evidence and ready runner recorded |

The independent final integrity rebuild covered eight scoreable manifests, 144 task rows, and 97 successful artifacts. Two consecutive verifier runs both passed with zero errors and had identical SHA-256 `69b3e94412d7ab7423f90600839c9bb3dc2998b51d7073ad635b6639c1a89876`.

## Audit findings

1. No score contradiction was found among final summary files and their replay copies.
2. The scorer's `coverage` must not be labeled artifact coverage. For example, LAM's numeric measurable coverage is 10/20 (50.0%), while artifact coverage is 8/18 (44.4%).
3. The Ours narrow diagnostic manifest retains method id `ours`; the human-readable diagnostic label is an aggregate display label. Verifying it under a substituted method id correctly fails; verifying the frozen `ours` id passes.
4. The old Articraft boundary cohort is absent from all aggregate values.
5. Overall 52-constraint satisfaction is intentionally not presented as the headline because 32/52 constraints inherit the count proxy's alias/granularity bias.
6. Artiverse is scientifically usable only in the dataset-retrieval reference supplementary. Placing it in the generation or matched-LLM rankings would be a modality/protocol error.
7. The Artiverse future-timestamp history is a provenance defect, not a result contradiction: the affected scan-only attempt produced zero selection/materialization/scoring output and was excluded before the corrected formal cohort.

Machine-readable values and exact source paths are in `summary.json`; source hashes are in `evidence_manifest.json`.
