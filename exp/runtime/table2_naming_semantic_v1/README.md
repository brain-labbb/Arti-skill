# Table 2 matched Naming semantic packet v1

This packet contains 1,107 method-blind link tasks from 140 category-matched URDF assets.
Judges receive only blind_tasks.jsonl and previews/. Do not open audit_tasks.jsonl or manifest method counts.

For every task, inspect both the node name and highlighted geometry. Opaque indices such as l_0 do not identify
a semantic part and must be invalid_or_hallucinated, even when the highlighted geometry is real. Use spec_match
only when name and geometry support a frozen required role. Use extra_real_part for a truthful name outside the
required list, including optional roles.

Annotate geometry independently of name correctness. Set judge_geometry_is_real_part to true when the target is
a coherent physical or functional component, and false for a stray fragment or artifact. Set judge_geometry_role
to a frozen required/optional role ID, other_real_part:<short_label>, or unknown. For every geometry-real node,
set judge_same_semantic_part_as to none or another node name when both links are fragments of the same semantic
part; use not_applicable for geometry that is not a real part.

Within each asset, compare all nodes assigned the same geometry role. If there are multiple physical instances,
judge_instance_id records the identity actually conveyed by the node name (left/right, upper/lower, or a stable
ordinal); use ambiguous when the names do not distinguish instances. Use not_applicable for singleton roles.
Every task requires a concise judge_reason. Do not open audit_tasks.jsonl, manifest.json, source URDFs, baseline
summaries, or another judge file.
