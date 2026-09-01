#!/usr/bin/env python3
"""Run pinned NURBGen as a distinct CAD-numeric supplementary baseline."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
ARTI_ROOT = WORKSPACE / "arti-skill"
NURBGEN_ROOT = WORKSPACE / "NURBGen"
REFERENCE = ARTI_ROOT / "exp/reference/table4_constraints_v2"
PROMPTS = REFERENCE / "prompts.jsonl"
CANONICALIZER = ARTI_ROOT / "exp/scripts/canonicalize_table4_artifact.py"
SCORER = ARTI_ROOT / "exp/scripts/score_table4_constraints_v2.py"
SOURCE_COMMIT = "62855d4b258082e5fbd220badf056618f7840939"
ADAPTER_REVISION = "f2f88e264e735353506a853e761e96d8545649d9"
BASE_REVISION = "1cfa9a7208912126459214e8b04321603b3df60c"
PROMPT_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
ADAPTER_SHA256 = "d381fa4b5e82c1d4602e4019b5e444d3208ede942c9de841e22d72c793873d54"
BASE_WEIGHT_SHA256 = {
    "model-00001-of-00003.safetensors": "328a91d3122359d5547f9d79521205bc0a46e1f79a792dfe650e99fc2d651223",
    "model-00002-of-00003.safetensors": "6cd087b316306a68c562436b5492edbcf6e16c6dba3a1308279caa5a58e21ca5",
    "model-00003-of-00003.safetensors": "e4bf436957184f4eeb86a80e9db394503f1f56446b2e6b7edeac5b81470f4ca1",
}
SMOKE_PROMPT = (
    "Socket head cap screw with a large countersunk washer. Features a hexagonal socket drive "
    "and a cylindrical threaded shank. Dimensions: length 92.96 mm, width 79.38 mm, height "
    "43.66 mm. Ensure smooth curvature at transitions."
)
MAX_NEW_TOKENS = 8192
TIMEOUT_SECONDS = 1800
MM_TO_M = 0.001


def contained(path: Path) -> Path:
    resolved = path.resolve()
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise ValueError(f"path escapes workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    contained(path).write_text(
        json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8"
    )


def run_checked(command: list[str], *, cwd: Path, timeout: int | None = None) -> None:
    subprocess.run(command, cwd=contained(cwd), check=True, timeout=timeout)


def verify_inputs(base_dir: Path, adapter_dir: Path) -> None:
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "3":
        raise RuntimeError("set CUDA_VISIBLE_DEVICES=3 exactly; no other GPU is authorized")
    if sha256(PROMPTS) != PROMPT_SHA256:
        raise RuntimeError("frozen prompt manifest hash mismatch")
    source_head = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=contained(NURBGEN_ROOT), text=True
    ).strip()
    if source_head != SOURCE_COMMIT:
        raise RuntimeError(f"NURBGen source commit mismatch: {source_head}")
    if sha256(adapter_dir / "adapter_model.safetensors") != ADAPTER_SHA256:
        raise RuntimeError("LoRA adapter hash mismatch")
    for filename, expected in BASE_WEIGHT_SHA256.items():
        if sha256(base_dir / filename) != expected:
            raise RuntimeError(f"base weight hash mismatch: {filename}")


def load_model(base_dir: Path, adapter_dir: Path):
    import torch
    from peft import PeftModel
    from transformers import AutoModelForCausalLM, AutoTokenizer

    tokenizer = AutoTokenizer.from_pretrained(base_dir, local_files_only=True)
    model = AutoModelForCausalLM.from_pretrained(
        base_dir,
        torch_dtype=torch.bfloat16,
        device_map={"": 0},
        attn_implementation="flash_attention_2",
        local_files_only=True,
    )
    model = PeftModel.from_pretrained(model, adapter_dir, local_files_only=True)
    model.eval()
    return torch, tokenizer, model


class GenerationTimeout(TimeoutError):
    pass


def _alarm_handler(_signum, _frame):
    raise GenerationTimeout(f"generation exceeded {TIMEOUT_SECONDS} seconds")


def generate_one(torch, tokenizer, model, prompt: str) -> str:
    messages = [{"role": "user", "content": "Generate NURBS for the following: " + prompt}]
    text = tokenizer.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    inputs = tokenizer(text, return_tensors="pt").to(model.device)
    previous = signal.signal(signal.SIGALRM, _alarm_handler)
    signal.alarm(TIMEOUT_SECONDS)
    try:
        with torch.no_grad():
            outputs = model.generate(
                **inputs,
                max_new_tokens=MAX_NEW_TOKENS,
                do_sample=False,
            )
        return tokenizer.decode(outputs[0][inputs.input_ids.shape[1]:], skip_special_tokens=True)
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, previous)


def export_cad(task_dir: Path, uid: str) -> tuple[Path, Path]:
    raw_dir = task_dir / "raw"
    cad_dir = task_dir / "cad_mm"
    run_checked(
        [
            sys.executable,
            str(NURBGEN_ROOT / "src/nurbs_representation/export.py"),
            "--input_dir",
            str(raw_dir),
            "--output_dir",
            str(cad_dir),
        ],
        cwd=NURBGEN_ROOT / "src/nurbs_representation",
        timeout=TIMEOUT_SECONDS,
    )
    step_path = cad_dir / f"{uid}.step"
    stl_path = cad_dir / f"{uid}.stl"
    if not step_path.is_file() or not stl_path.is_file():
        raise RuntimeError("official NURBGen export did not produce both STEP and STL")
    return step_path, stl_path


def stl_mm_to_glb_m(stl_path: Path, glb_path: Path) -> None:
    import trimesh

    mesh = trimesh.load(contained(stl_path), force="mesh", process=False)
    if mesh.is_empty:
        raise RuntimeError("exported STL has no geometry")
    mesh.apply_scale(MM_TO_M)
    contained(glb_path).write_bytes(mesh.export(file_type="glb"))


def execute_attempt(torch, tokenizer, model, uid: str, prompt: str, task_dir: Path) -> dict[str, Any]:
    task_dir = contained(task_dir)
    attempt_path = task_dir / "attempt.json"
    if attempt_path.exists():
        raise RuntimeError(f"refusing a second attempt for {uid}: {attempt_path}")
    (task_dir / "raw").mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    record: dict[str, Any] = {
        "uid": uid,
        "attempt": 1,
        "repair_count": 0,
        "timeout_seconds": TIMEOUT_SECONDS,
        "decoding": {"max_new_tokens": MAX_NEW_TOKENS, "do_sample": False},
        "status": "failed",
    }
    try:
        response = generate_one(torch, tokenizer, model, prompt)
        (task_dir / "raw/raw_response.txt").write_text(response, encoding="utf-8")
        write_json(
            task_dir / f"raw/{uid}.json",
            {"uid": uid, "caption": prompt, "response": response},
        )
        step_path, stl_path = export_cad(task_dir, uid)
        glb_path = task_dir / "artifact_m.glb"
        stl_mm_to_glb_m(stl_path, glb_path)
        record.update({
            "status": "success",
            "raw_response": str(task_dir / "raw/raw_response.txt"),
            "step_mm": str(step_path),
            "step_sha256": sha256(step_path),
            "glb_m": str(glb_path),
            "glb_sha256": sha256(glb_path),
            "unit_conversion": "STEP/STL millimetres to GLB metres by fixed factor 0.001",
            "target_dependent_scaling": False,
        })
    except Exception as exc:
        record["failure"] = {"type": type(exc).__name__, "message": str(exc)}
    record["elapsed_seconds"] = time.monotonic() - started
    write_json(attempt_path, record)
    return record


def canonicalize_success(task_id: str, record: dict[str, Any], output: Path) -> dict[str, str] | None:
    if record["status"] != "success":
        return None
    canonical_dir = output / "canonical" / task_id
    run_checked(
        [
            sys.executable,
            str(CANONICALIZER),
            "--input",
            record["glb_m"],
            "--artifact-type",
            "glb",
            "--unit-scale-to-m",
            "1.0",
            "--output-dir",
            str(canonical_dir),
        ],
        cwd=ARTI_ROOT,
    )
    return {"task_id": task_id, "canonical_dir": str(canonical_dir)}


def score_twice(manifest: Path, output: Path) -> None:
    summaries = []
    for index in (1, 2):
        score_dir = output / f"score_run{index}"
        run_checked(
            [
                sys.executable,
                str(SCORER),
                "--method",
                "nurbgen",
                "--panel",
                "cad_numeric",
                "--artifact-manifest",
                str(manifest),
                "--output-dir",
                str(score_dir),
            ],
            cwd=ARTI_ROOT,
        )
        summaries.append(json.loads((score_dir / "summary.json").read_text(encoding="utf-8")))
    if summaries[0] != summaries[1]:
        raise RuntimeError("the two scorer runs disagree")
    write_json(output / "score_reproducibility.json", {"identical": True, "summary": summaries[0]})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-dir", type=Path, required=True)
    parser.add_argument("--adapter-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base_dir = contained(args.base_dir)
    adapter_dir = contained(args.adapter_dir)
    output = contained(args.output_dir)
    if output.exists():
        raise RuntimeError(f"fresh output directory required: {output}")
    verify_inputs(base_dir, adapter_dir)
    output.mkdir(parents=True)
    torch, tokenizer, model = load_model(base_dir, adapter_dir)

    smoke = execute_attempt(torch, tokenizer, model, "official_readme_smoke", SMOKE_PROMPT, output / "smoke")
    if smoke["status"] != "success":
        write_json(output / "run_status.json", {"status": "STOPPED_SMOKE_FAILED", "smoke": smoke})
        return 2

    rows = []
    tasks = [json.loads(line) for line in PROMPTS.read_text(encoding="utf-8").splitlines() if line]
    for task in tasks:
        task_id = task["task_id"]
        record = execute_attempt(torch, tokenizer, model, task_id, task["prompt"], output / task_id)
        row = canonicalize_success(task_id, record, output)
        if row is not None:
            rows.append(row)

    manifest = output / "artifact_manifest.jsonl"
    manifest.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8")
    score_twice(manifest, output)
    write_json(
        output / "run_status.json",
        {
            "status": "COMPLETED",
            "source_commit": SOURCE_COMMIT,
            "adapter_revision": ADAPTER_REVISION,
            "base_revision": BASE_REVISION,
            "prompt_manifest_sha256": PROMPT_SHA256,
            "attempted_tasks": len(tasks),
            "registered_artifacts": len(rows),
            "repair_budget": 0,
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
