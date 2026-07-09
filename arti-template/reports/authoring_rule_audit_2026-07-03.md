# Authoring-Rule Corpus Audit — 2026-07-03

Three-bucket classification of every normative rule in the template-authoring
docs, produced to drive the docs-slimming / gate-hardening effort:

- **A = machine-enforced today** (gate raises / API makes violation impossible)
  → doc prose can be demoted to a short "why" note beside the enforcing API.
- **B = machine-enforceable, not enforced yet** → gate-hardening backlog
  (sketch + difficulty per row).
- **C = genuine judgment** (category faithfulness, realism, planning taste)
  → stays in docs, preferably compressed to checklist-table form (§8.5 style).

Docs audited: MODULAR_TEMPLATE_AUTHORING (MTA), TEMPLATE_DESIGN_RULES (TDR),
SPEC_TEMPLATE (SPEC), TEMPLATE_AUTHORING_AGENT (TAA), FORK_VARIANTS (FV).
Totals: **55 rules — 17 A / 13 B / 25 C.**

> Caveats: first-pass agent classification — review before acting. Line
> numbers reference the 2026-07-03 working tree. Known judgment call: MTA §2c
> (`fit_to_upstream`) is bucketed A but the helper only raises when *used*;
> nothing yet forces adoption (an adoption-rate census section would cover
> this). Related live gates added the same day:
> `tests/docs/test_authoring_doc_discipline.py` (doc symbol drift + size
> ratchet vs `authoring_doc_budget.json`).

## Audit Table: Complete Rule Classification

| Doc §/Line | Rule (one-line) | Bucket | Evidence/Gate Sketch |
|---|---|---|---|
| MTA §1 L169 | Use ctx.rng for per-module randomness; don't create new random.Random | C | Code style; no enforcement but enforced via code review of _build_* factories |
| MTA §1 L172 | Inspect ctx.prior_choices when geometry depends on upstream choice | C | Structural judgment; reviewed at module design time |
| MTA §2 L183-192 | Child upstream anchor normal-axis component MUST be 0 | A | _emit_chain_joint ValueError: agent/templates/_modular.py (raises at assembly) |
| MTA §2b L194-204 | Declare iface_key on both sides of pair; key whole slot candidates at once | A | _validate_pair raises on mismatch (agent/templates/_modular.py) |
| MTA §2c L211-228 | Use fit_to_upstream helper for child sizing; never re-state from global config | A* | fit_to_upstream raises on misuse — but adoption itself is not forced (see caveats) |
| MTA §2c L228 | Comment required when bypassing fit_to_upstream for standardized hardware | C | Code documentation; no automated check |
| MTA §3 L236-251 | fail_if_isolated_parts (FAIL), warn_if_disconnected_islands (WARN→FAIL in sweep) | A | agent/compiler.py baseline runs both; sweep promotes WARN to FAIL |
| MTA §3 L248-251 | MUST NOT use allow_disconnected_islands; seat pieces or split into separate FIXED parts | A | Feature removed from SDK; enforcement by absence |
| MTA §3b L264-287 | Use mount_fixed helper for stacked/nested children; never raw model.articulation | A | mount_fixed helper (agent/templates/_modular.py); single-source placement |
| MTA §3b L289-294 | Set tangential_containment=True on MatingContract for stacked/nested mounts | B | Sweep check: flag FIXED-joint MatingContracts without tangential_containment + proximity inspect; medium |
| MTA §3c L298-314 | Factor shared geometric quantities into one named helper or Resolved<Slug>Config field | C | Code structure judgment; manual source review |
| MTA §3d L318-334 | Use hinged_panel, sliding_member, coupled_chain, clamp_joint_limits; comment if raw | C | Idiom choice + rationale; no automated "should have used idiom X" detection |
| MTA §4 L341-345 | Use deterministic procedural sampling for all seeds including seed=0 | A | Contract test: config_from_seed(0) must succeed (test_template_registry_contract.py) |
| MTA §4 L347-349 | Sparse regression overrides only for known regressions or reviewer-selected | C | Governance; human judgment + commit message review |
| MTA §5 L355-362 | Set __modular__ = True at module scope | A | gate checks __modular__ (template_sweep_coverage.py) |
| TDR §1 L15-18 | Decorative sub-elements MUST be parent.visual, NOT separate FIXED-joint parts | B | Detect FIXED-only parts with small visual count/size (topology + heuristic); medium |
| TDR §2 L91-100 | Declare MatingContract on all child-creating articulations; real parent visual required | A | fail_if_joint_mating_has_gap (agent/compiler.py baseline) |
| TDR §3 L173-187 | Module factory MUST preserve part tree, joint semantics, primitive types from 5-star source | B | AST diff of spec's model.py:Lx-Ly vs generated factory; hard |
| TDR §3 L191-194 | MUST NOT downgrade sophisticated primitives (LatheGeometry/Mesh) to Box/Cylinder | B | Scan generated model.py for Box/Cylinder where spec declares Lathe/Mesh; hard |
| TDR §3 L196-206 | Every slotvalues, reachable-aware | A | gate, MIN_DISTINCT_PER_SLOT_KEY=2 (template_sweep_coverage.py) |
| TDR §4 L249-252 | Derive ④ decoration from host's actual surface (per-z radius, specific face) | C | Visual realism judgment; no automated surface-derivation check |
| TDR §4 L261-268 | MUST NOT use constant-radius band; follow host profile across ③/⑤ changes | C | Appearance judgment; visual-only at batch 0-9 |
| TDR §5 L281-286 | Call fail_if_parts_overlap_in_sampled_poses in run_tests unless exemption declared | B | Parse run_tests for the call OR "sampled-pose exemption" comment; easy-medium |
| TDR §5 L287-326 | Verify motion semantics via ctx.pose(...) checks for key mechanisms | C | Motion intent judgment; no completeness check |
| TDR §5 L306-326 | Specify per-axis motion envelope + motion_test_plan for each non-continuous joint | C | Spec documentation requirement; review-time only |
| SPEC §4 L74-78 | Each slot target 3-6 candidates; degrade to 2 only with justification | C | Sample planning judgment |
| SPEC §4 L75 | Ordinary ①/②/multiplicity candidates MUST have source_type=forked_anchor + model.py:Lx-Ly | C | Spec documentation; human review |
| SPEC §4 L76-77 | ③ exception: world_knowledge_extrapolation allowed with source-backed anchors | C | Spec governance; reviewer sign-off |
| SPEC §4 L77 | ④ exception: host-conformal, non-structural, surface-only | C | Spec governance; review + visual validation |
| SPEC §4 L79 | Candidates must differ structurally, not size/color/material only | B | Compare module sources for part tree/joint topology equality; hard |
| SPEC §4 L80-81 | Form-dominated category MUST register ③ slot (≥2-3 distinct form prototypes) | C | Category semantics judgment |
| SPEC §7 L141-150 | All equation/inequality/conditional constraints MUST be solved in resolve_config | B | Config AST + data-flow: flag unclamped independent fields used by builder; medium |
| SPEC §8.5 L182-202 | Each of 6 axes MUST be considered: have/lack + values/ranges or reason | C | Checklist-form already; human review |
| TAA §1 L99 | sweep-pipeline is the authoritative signal; don't rely on pytest/QC/visual alone | C | Process governance |
| TAA §3 L181-195 | Fix largest failure cluster first; no per-seed surgical edits for multi-seed clusters | C | Iteration methodology; guidance only |
| TAA §3 L197 | fails → fix missing module factories or revise spec | B | Report per-key under-coverage when gate fails; easy |
| TAA §3 L207-218 | Escalation on ≥3-sweep cluster survival or no pass-rate improvement | A | streak_count in template_sweep_state |
| TAA §5 L243-253 | MUST NOT declare done on pytest / lower threshold / widen seeds / edit _BASELINE_ARTICULATION_ORIGIN_TOL | A | Protected constant + CLI validation + review |
| FV §2 L35-50 | Before fork: analyze parent code form, list 2-4 structure axes, 3-6 candidates per axis | C | Planning methodology |
| FV §2 L45 | Mark each parent to axis grid; fork from closest parent to target | C | Fork planning judgment; source map |
| FV §2 L47 | MUST NOT fork from variant; never chain variant→variant forks | A | Lineage constraint (parent_record_id must be an original) |
| FV §2 L48-49 | Coverage now per-slot , not tuple count; N covers not counts | A | per-key semantics |
| FV §3 L74 | Primary-axis: exactly 1 main structural axis diff; companions must not touch part tree/joints/interfaces/primitives | B | Parse TARGET/KEEP + AST diff of part tree/joints; hard |
| FV §3 L77 | MUST have ≥1 non-fixed joint | A | grep non-fixed joints in materialized URDF (FV §3 command) |
| FV §3 L78 | MUST not deviate from category (human eye test) | C | Category membership judgment |
| FV §3 L79 | MUST converge (compile + run_tests + baseline + designer_common) | A | compile + run_tests in variant preparation |
| FV §3 L80 | MUST satisfy model.py readability contract (§4) | B | Naming/loop/anchor heuristic checks; hard |
| FV §4 L113 | Naming MUST map slot names directly (base_*, lid_*, handle_*) | B | Slot-name prefix matching vs source map; medium |
| FV §4 L114 | Multiplicity MUST use for i in range(n) + f"<name>_{i}" + shared helper + regular placement | B | AST loop-structure detection; hard |
| FV §4 L114 | MUST NOT copy-paste nearly identical code | B | Clone detection; hard |
| FV §4 L115 | Decorative elements MUST NOT be independent parts (Rule 1 mirror) | B | Same as TDR §1; medium |
| FV §4 L116 | Active child parts MUST have real anchor visuals at real contact face (Rule 2 mirror) | A | fail_if_joint_mating_has_gap |
| FV §4 L117 | Primitives faithful, no downgrade (Rule 3 mirror) | B | Same as TDR §3; hard |
| FV §4 L118 | Cross-layer interface faces axially aligned, describable as face+anchor | C | Interface design judgment |
| FV §5 L149 | Each slot(target 3-6) structurally different values covered | A | (MIN_DISTINCT_PER_SLOT_KEY=2) |
| FV §5 L150 | Multiplicity axis covers 2-3 N values; rest parent baseline | C | Batch planning; source map |
| FV §5 L151 | MUST NOT enumerate combinations; one candidate per grid cell | C | Batch planning discipline |
| FV §5 L152 | Same grid cell not repeated | C | Batch tracking via source map |
| FV §5 L153 | Candidates MUST be structural difference, not size/skin/material | B | AST part-tree/joint equality compare; hard |
| FV §5 L154 | ③ fork enough anchors, don't fork entire space | C | Sampling strategy; source map |
| FV §5 L155 | ④ record real examples, allow controlled extrapolation | C | Sampling strategy; source_type field |
| FV §5 L156-157 | ①② must be source-backed; world knowledge cannot invent candidates | C | Governance; reviewer approval |

## Summary Counts

| Doc | A | B | C | total |
|---|---|---|---|---|
| MODULAR_TEMPLATE_AUTHORING.md | 7 | 1 | 6 | 14 |
| TEMPLATE_DESIGN_RULES.md | 2 | 3 | 2 | 7 |
| SPEC_TEMPLATE.md | 1 | 1 | 6 | 8 |
| TEMPLATE_AUTHORING_AGENT.md | 2 | 1 | 3 | 6 |
| FORK_VARIANTS.md | 5 | 7 | 8 | 20 |
| **Total** | **17** | **13** | **25** | **55** |

## Top 5 B-Bucket Gates by Impact × Ease

| Rank | Rule | Impact | Ease | Suggested Gate |
|---|---|---|---|---|
| 2 | TDR §5 L281-286: sampled-pose check presence | Medium | Easy | Parse run_tests for fail_if_parts_overlap_in_sampled_poses call OR "sampled-pose exemption" comment; flag articulated templates with neither |
| 3 | MTA §3b: tangential_containment on stacked mounts | Medium | Medium | Flag FIXED-joint MatingContracts without tangential_containment where child face ⊂ parent face plausibly required |
| 4 | SPEC §7: constraint resolution audit | Medium | Medium | resolve_config AST + data-flow; flag unclamped independent config fields consumed by builders |
| 5 | TDR §3: primitive downgrade detection | High | Hard (~200 LOC) | Extract primitive types from spec-cited source ranges via AST; compare to generated template; flag downgrades |
