#!/usr/bin/env python3
"""Scaffold a modular spec skeleton from a subcategory source map.

Optimization ② (source-map -> spec pre-fill). The upstream source map
(`picture_expansion/template_source_maps/<大类>__<小类>.md` in articraft_data)
already declares slots, candidates (with record id + helper names) and the
multiplicity logic. This tool pre-fills the spec's §4 (slot/candidate tables)
and §8 (multiplicity) so the human only reviews — and crucially **resolves the
`model.py:Lx-Ly` line numbers automatically** from the named helpers (AST), so
nobody hand-types line numbers (they would drift anyway).

It is intentionally tolerant: anything it cannot resolve is emitted as a
`TODO:` marker rather than failing, so the human fixes only the gaps.

Usage:
    python scripts/scaffold_spec.py \
        --source-map /path/to/<大类>__<小类>.md \
        --records-root /path/to/repo/data/records \
        --slug fence_cascade \
        --out articraft_template_authoring/specs_modular_v1/Fence_Cascade_fences_MORE_THAN_1.md

`--records-root` may point at either repo (the source map's records live in
articraft_data until synced via scripts/sync_from_source.py).
"""
from __future__ import annotations

import argparse
import ast
import glob
import os
import re
from dataclasses import dataclass, field


@dataclass
class Candidate:
    name: str
    record_hash: str          # trailing token of the abbreviated rec id
    helpers: list[str]
    feature: str
    raw_record: str


@dataclass
class Slot:
    name: str
    candidates: list[Candidate] = field(default_factory=list)


def _strip_md(cell: str) -> str:
    return cell.replace("`", "").strip()


def _record_hash(rec: str) -> str:
    """The source map abbreviates ids as `rec_..._<hash>`; take the last token."""
    rec = rec.strip().strip("`")
    return rec.rsplit("_", 1)[-1] if "_" in rec else rec


def _parse_source_map(text: str) -> tuple[list[Slot], list[str]]:
    """Best-effort parse of the documented source-map format."""
    slots: list[Slot] = []
    multiplicity: list[str] = []
    cur_slot: Slot | None = None
    in_mult = False

    for line in text.splitlines():
        s = line.strip()
        m_slot = re.match(r"^###\s*Slot\s*[A-Z]?\s*[:：]\s*(.+)$", s)
        if m_slot:
            in_mult = False
            name = re.split(r"[（(]", m_slot.group(1))[0].strip()
            cur_slot = Slot(name=name)
            slots.append(cur_slot)
            continue
        if re.match(r"^##\s*Multiplicity", s) or "Copy Logic" in s and s.startswith("##"):
            in_mult = True
            cur_slot = None
            continue
        if s.startswith("##"):  # any other section ends slot/mult capture
            in_mult = False
            cur_slot = None
            continue
        if in_mult and s and not s.startswith("#"):
            multiplicity.append(line.rstrip())
            continue
        if cur_slot is not None and s.startswith("|"):
            cells = [c.strip() for c in s.strip("|").split("|")]
            if len(cells) < 4:
                continue
            if set(_strip_md(cells[0])) <= {"-", " ", ":"}:  # separator row
                continue
            if "候选" in cells[0] or "module_name" in cells[0]:  # header row
                continue
            cand_name = re.split(r"[（(]", _strip_md(cells[0]))[0].strip()
            rec = _strip_md(cells[1])
            helpers = re.findall(r"`([^`]+)`", cells[2]) or [
                t for t in re.split(r"[\s/，,]+", _strip_md(cells[2])) if t.startswith("_")
            ]
            feature = _strip_md(cells[3]) if len(cells) > 3 else ""
            cur_slot.candidates.append(
                Candidate(cand_name, _record_hash(rec), helpers, feature, rec)
            )
    return slots, multiplicity


def _resolve_record_dir(records_root: str, raw_record: str) -> str | None:
    """Resolve a source-map record token to a record dir, SAFELY.

    Accepts only: (a) an exact full record-id that exists, or (b) an abbreviated
    `rec_..._<hash>` whose trailing token is a real hex hash (>=6) matching
    EXACTLY ONE record dir. Anything ambiguous or a short trailing word returns
    None so the caller emits an explicit `TODO:` — it must NEVER silently bind a
    wrong record (the old `*{token}*` → hits[0] mapped clamshell to a laptop).
    """
    raw = (raw_record or "").strip().strip("`").strip()
    if not raw:
        return None
    exact = os.path.join(records_root, raw)
    if os.path.isdir(exact):  # (a) exact full record id
        return exact
    h = raw.rsplit("_", 1)[-1].strip(".")  # (b) abbreviated rec_..._<hash>
    if re.fullmatch(r"[0-9a-fA-F]{6,}", h):
        hits = glob.glob(os.path.join(records_root, f"*{h}*"))
        if len(hits) == 1:
            return hits[0]
    return None


def _resolve_helper_spans(model_py: str, helpers: list[str]) -> dict[str, str]:
    """Map helper name -> 'Lx-Ly' via AST (end_lineno is exact, never drifts)."""
    try:
        tree = ast.parse(open(model_py, encoding="utf-8").read())
    except Exception as exc:  # noqa: BLE001
        return {h: f"TODO: parse error ({exc})" for h in helpers}
    spans: dict[str, tuple[int, int]] = {}
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            spans[node.name] = (node.lineno, node.end_lineno or node.lineno)
    out: dict[str, str] = {}
    for h in helpers:
        if h in spans:
            lo, hi = spans[h]
            out[h] = f"L{lo}-L{hi}"
        else:
            out[h] = "TODO: helper not found"
    return out


def _candidate_source_cell(records_root: str, c: Candidate) -> tuple[str, str, str]:
    rec_dir = _resolve_record_dir(records_root, c.raw_record)
    if not rec_dir:
        return (c.raw_record, "TODO: record not found in records-root", "")
    rec_id = os.path.basename(rec_dir)
    model_py = os.path.join(rec_dir, "revisions", "rev_000001", "model.py")
    if not os.path.exists(model_py):
        return (rec_id, "TODO: model.py not found", "")
    spans = _resolve_helper_spans(model_py, c.helpers)
    span_str = " ; ".join(f"{h}:{spans[h]}" for h in c.helpers) if c.helpers else "TODO"
    return (rec_id, span_str, model_py)


def _emit_spec(slug: str, slots: list[Slot], multiplicity: list[str], records_root: str) -> str:
    L = []
    L.append(f"# {slug} — Modular Spec (SCAFFOLDED — review required)\n")
    L.append("> Auto-generated by scripts/scaffold_spec.py from the source map.")
    L.append("> Line numbers were resolved from helper names via AST; review the")
    L.append("> slot decomposition, candidate choices, ranges and any `TODO:` below.\n")
    L.append("## 元信息")
    L.append("| 项 | 值 |\n|---|---|")
    L.append(f"| slug | `{slug}` |")
    L.append(f"| template path | `agent/templates/{slug}.py` |")
    L.append("| stage | `SPEC_ONLY_DRAFT` |")
    L.append("| status | `pending` |")
    L.append("| __modular__ | `True` |")
    L.append("| pattern | `TODO: linear_chain / parallel_children / multiplicity / mixed` |\n")

    L.append("## 槽位 + 候选模块表\n")
    for i, slot in enumerate(slots):
        letter = chr(ord("A") + i)
        L.append(f"### Slot {letter}：{slot.name}\n")
        L.append("| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |")
        L.append("|---|---|---|---|---|")
        for c in slot.candidates:
            rec_id, span, _ = _candidate_source_cell(records_root, c)
            L.append(
                f"| {c.name} | {rec_id} | {span} | eligible if compatible | {c.feature} |"
            )
        L.append("")

    L.append("## Multiplicity / Copy Logic\n")
    if multiplicity:
        L.extend(multiplicity)
    else:
        L.append("- 无复制数量逻辑：核心结构由固定 named slots 表达。")
    L.append("")

    for sec in (
        "## 5 星样本阅读摘要", "## 核心身份", "## 槽位图（slot graph）",
        "## 每槽位 Module Emits / Interfaces", "## 参数范围汇总",
        "## 拓扑多样性审计", "## Validator", "## Reject cases",
        "## 与相邻类别的边界", "## 审核记录",
    ):
        L.append(sec)
        L.append("\nTODO: 人工填写（见 SPEC_TEMPLATE.md）。\n")
    return "\n".join(L) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description="Scaffold a modular spec from a source map.")
    ap.add_argument("--source-map", required=True)
    ap.add_argument("--records-root", required=True)
    ap.add_argument("--slug", required=True)
    ap.add_argument("--out", default=None, help="Output spec path (default: stdout).")
    args = ap.parse_args()

    text = open(args.source_map, encoding="utf-8").read()
    slots, multiplicity = _parse_source_map(text)
    spec = _emit_spec(args.slug, slots, multiplicity, args.records_root)

    n_cands = sum(len(s.candidates) for s in slots)
    todos = spec.count("TODO:")
    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        open(args.out, "w", encoding="utf-8").write(spec)
        print(f"Wrote {args.out}: {len(slots)} slots, {n_cands} candidates, {todos} TODO markers.")
    else:
        print(spec)
        print(f"# {len(slots)} slots, {n_cands} candidates, {todos} TODO markers.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
