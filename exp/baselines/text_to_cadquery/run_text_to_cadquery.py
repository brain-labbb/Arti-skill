#!/usr/bin/env python3
"""Run the pinned official Text-to-CadQuery Gemma checkpoint once per prompt."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import subprocess
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cadquery as cq
import numpy as np
import torch
import transformers
import trimesh
from transformers import AutoModelForCausalLM, AutoTokenizer


WORKSPACE = Path("/mnt/zsn/lyb").resolve()
EXP_ROOT = WORKSPACE / "arti-skill/exp"
BASELINE_ROOT = EXP_ROOT / "baselines/text_to_cadquery"
OFFICIAL_ROOT = BASELINE_ROOT / "official"
MODEL_ROOT = BASELINE_ROOT / "models/gemma-1B-SFT-a6f16215"
REFERENCE_ROOT = EXP_ROOT / "reference/table4_constraints_v2"
PROMPTS = REFERENCE_ROOT / "prompts.jsonl"
PROTOCOL = REFERENCE_ROOT / "protocol.json"
CANONICALIZER = EXP_ROOT / "scripts/canonicalize_table4_artifact.py"
MODEL_ID = "ricemonster/gemma-1B-SFT"
MODEL_REVISION = "a6f16215f718e47866e958e30c5dd2de52ab823c"
CODE_REVISION = "4f7f50176f3c642f6d897c4a8670f7a83acb31bd"
PROMPT_MANIFEST_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
EXPORT_RE = re.compile(
    r"(?:(?:cq\.)?exporters\.export)\s*\(\s*([^,]+),\s*['\"][^'\"]+?"
    r"\.(?:stl|step|stp)['\"]\s*\)",
    flags=re.IGNORECASE | re.MULTILINE,
)


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


def write_text(path: Path, text: str) -> None:
    path = contained(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def dump_json(path: Path, payload: Any) -> None:
    write_text(path, json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n")


def load_prompts(smoke: bool, task_ids: set[str] | None) -> list[dict[str, Any]]:
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    if sha256(PROMPTS) != PROMPT_MANIFEST_SHA256:
        raise RuntimeError("frozen prompt manifest hash mismatch")
    if protocol["prompt_manifest_sha256"] != PROMPT_MANIFEST_SHA256:
        raise RuntimeError("protocol prompt hash mismatch")
    if smoke:
        official_test = OFFICIAL_ROOT / "inference/test_filtered.jsonl"
        first = json.loads(official_test.read_text(encoding="utf-8").splitlines()[1])
        return [{
            "task_id": "OFFICIAL_SMOKE_0001",
            "category": "official_test_filtered_row_1",
            "prompt": first["input"],
            "source": str(official_test),
            "source_row_zero_based": 1,
        }]
    rows = [json.loads(line) for line in PROMPTS.read_text(encoding="utf-8").splitlines() if line]
    if len(rows) != int(protocol["task_count"]):
        raise RuntimeError("frozen prompt task count mismatch")
    if task_ids:
        rows = [row for row in rows if row["task_id"] in task_ids]
        missing = task_ids - {row["task_id"] for row in rows}
        if missing:
            raise ValueError(f"unknown task ids: {sorted(missing)}")
    return rows


def native_prompt(text: str) -> str:
    return f"<start_of_turn>user\n{text}\n<end_of_turn>\n<start_of_turn>model\n"


def clean_generated_code(text: str, step_path: Path) -> tuple[str, str]:
    """Apply the official notebook export-redirection rule without a repair turn."""
    cleaned = text.strip()
    if cleaned.startswith("```python") and cleaned.endswith("```"):
        cleaned = cleaned[len("```python") : -len("```")].strip()
    elif cleaned.startswith("```") and cleaned.endswith("```"):
        cleaned = cleaned[len("```") : -len("```")].strip()
    matches = list(EXPORT_RE.finditer(cleaned))
    if not matches:
        raise ValueError("official cleaner found no CadQuery export statement")
    exported_expression = matches[0].group(1).strip()
    cleaned = EXPORT_RE.sub("", cleaned)
    cleaned = "\n".join(
        line for line in cleaned.splitlines()
        if not re.search(r"(?:(?:cq\.)?exporters\.)", line)
    ).strip()
    redirected = (
        cleaned
        + f"\n\n# Fixed evaluation-harness export redirection (repair budget remains zero).\n"
        + f"cq.exporters.export({exported_expression}, {str(step_path)!r})\n"
    )
    return redirected, exported_expression


def run_subprocess(command: list[str], cwd: Path, timeout_s: float) -> dict[str, Any]:
    started = time.monotonic()
    env = os.environ.copy()
    env.update({
        "CUDA_VISIBLE_DEVICES": "3",
        "HF_HOME": str(BASELINE_ROOT / ".hf_home"),
        "HF_HUB_CACHE": str(BASELINE_ROOT / ".hf_home/hub"),
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
        "PYTHONNOUSERSITE": "1",
    })
    try:
        result = subprocess.run(
            command,
            cwd=contained(cwd),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        return {
            "command": command,
            "exit_code": result.returncode,
            "timed_out": False,
            "elapsed_s": time.monotonic() - started,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "command": command,
            "exit_code": 124,
            "timed_out": True,
            "elapsed_s": time.monotonic() - started,
            "stdout": exc.stdout or "",
            "stderr": exc.stderr or "",
        }


def step_to_glb(step: Path, glb: Path, task_id: str) -> dict[str, Any]:
    started = time.monotonic()
    imported = cq.importers.importStep(str(contained(step)))
    shapes = imported.vals()
    if not shapes:
        raise ValueError("STEP contains no CadQuery shapes")
    scene = trimesh.Scene(base_frame="world")
    triangle_count = 0
    vertex_count = 0
    for index, shape in enumerate(shapes):
        vertices, triangles = shape.tessellate(0.1, 0.1)
        vertex_array = np.asarray([vertex.toTuple() for vertex in vertices], dtype=np.float64) * 0.001
        face_array = np.asarray(triangles, dtype=np.int64)
        if not len(vertex_array) or not len(face_array):
            continue
        name = f"{task_id}_shape_{index + 1:04d}"
        mesh = trimesh.Trimesh(vertices=vertex_array, faces=face_array, process=False)
        scene.add_geometry(mesh, node_name=name, geom_name=name)
        vertex_count += len(vertex_array)
        triangle_count += len(face_array)
    if not scene.geometry:
        raise ValueError("STEP tessellation produced no triangular geometry")
    contained(glb).write_bytes(scene.export(file_type="glb"))
    bounds = np.asarray(scene.bounds, dtype=float)
    return {
        "engine": "CadQuery/OCP tessellate -> trimesh GLB container",
        "unit_scale": "STEP millimetres multiplied by 0.001 to GLB metres",
        "linear_tolerance_mm": 0.1,
        "angular_tolerance_rad": 0.1,
        "shape_count": len(scene.geometry),
        "vertex_count": vertex_count,
        "triangle_count": triangle_count,
        "bounds_m": bounds.tolist(),
        "extents_m": (bounds[1] - bounds[0]).tolist(),
        "elapsed_s": time.monotonic() - started,
    }


def canonicalize(glb: Path, task_dir: Path, timeout_s: float) -> dict[str, Any]:
    canonical_dir = task_dir / "canonical"
    result = run_subprocess(
        [
            sys.executable,
            str(CANONICALIZER),
            "--input",
            str(glb),
            "--artifact-type",
            "glb",
            "--unit-scale-to-m",
            "1.0",
            "--output-dir",
            str(canonical_dir),
        ],
        EXP_ROOT.parent,
        timeout_s,
    )
    write_text(task_dir / "canonicalize.stdout.txt", str(result.pop("stdout")))
    write_text(task_dir / "canonicalize.stderr.txt", str(result.pop("stderr")))
    result["canonical_dir"] = str(canonical_dir)
    return result


def generate_one(
    model: Any,
    tokenizer: Any,
    row: dict[str, Any],
    root: Path,
    timeout_s: float,
) -> dict[str, Any]:
    task_dir = contained(root / row["task_id"])
    task_dir.mkdir(parents=True, exist_ok=True)
    prompt_text = native_prompt(row["prompt"])
    write_text(task_dir / "prompt.txt", row["prompt"] + "\n")
    write_text(task_dir / "model_input.txt", prompt_text)
    record: dict[str, Any] = {
        "schema_version": 2,
        "benchmark_id": "official_smoke" if row["task_id"].startswith("OFFICIAL_SMOKE") else "table4_constraints_v2",
        "task_id": row["task_id"],
        "category": row["category"],
        "method": "text_to_cadquery",
        "status": "failed",
        "model_id": MODEL_ID,
        "model_revision": MODEL_REVISION,
        "code_revision": CODE_REVISION,
        "device_policy": "CUDA_VISIBLE_DEVICES=3 (physical GPU 3 appears as cuda:0)",
        "dtype": "bfloat16",
        "decoding": {
            "official_prompt_template": True,
            "do_sample": False,
            "sequence_limit": 1024,
            "repair_budget": 0,
            "final_samples_per_task": 1,
        },
        "prompt_sha256": hashlib.sha256((row["prompt"] + "\n").encode()).hexdigest(),
        "started_at": datetime.now(timezone.utc).isoformat(),
    }
    started = time.monotonic()
    try:
        generation_started = time.monotonic()
        encoded = tokenizer(prompt_text, return_tensors="pt", truncation=True).to("cuda")
        input_tokens = int(encoded["input_ids"].shape[1])
        max_new_tokens = max(1, 1024 - input_tokens)
        with torch.inference_mode():
            generated = model.generate(
                **encoded,
                max_new_tokens=max_new_tokens,
                eos_token_id=tokenizer.eos_token_id,
                pad_token_id=tokenizer.eos_token_id,
                do_sample=False,
            )
        decoded = tokenizer.batch_decode(generated, skip_special_tokens=True)[0]
        response = decoded.split("<start_of_turn>model", 1)[1].strip() if "<start_of_turn>model" in decoded else decoded
        generation_elapsed = time.monotonic() - generation_started
        write_text(task_dir / "generation.stdout.txt", response + "\n")
        write_text(task_dir / "generation.stderr.txt", "")
        raw_code = task_dir / "raw_generation.txt"
        write_text(raw_code, response + "\n")
        step = task_dir / "artifact.step"
        code, exported_expression = clean_generated_code(response, step)
        script = task_dir / "generated_cleaned.py"
        write_text(script, code)
        execution = run_subprocess([sys.executable, str(script)], task_dir, timeout_s)
        write_text(task_dir / "execution.stdout.txt", str(execution.pop("stdout")))
        write_text(task_dir / "execution.stderr.txt", str(execution.pop("stderr")))
        record["generation"] = {
            "elapsed_s": generation_elapsed,
            "input_tokens": input_tokens,
            "max_new_tokens": max_new_tokens,
            "output_tokens_including_prompt": int(generated.shape[1]),
            "new_tokens": int(generated.shape[1]) - input_tokens,
            "raw_output": str(raw_code),
            "raw_output_sha256": sha256(raw_code),
        }
        record["cleaning"] = {
            "rule": "official clean_gemma-1B export redirection, extended only for repository exporters.export spelling",
            "exported_expression": exported_expression,
            "script": str(script),
            "script_sha256": sha256(script),
        }
        record["execution"] = execution
        if execution["exit_code"] != 0 or not step.is_file():
            raise RuntimeError(f"generated code did not produce STEP (exit={execution['exit_code']})")
        glb = task_dir / "artifact_m.glb"
        conversion = step_to_glb(step, glb, row["task_id"])
        canonical = canonicalize(glb, task_dir, timeout_s)
        if canonical["exit_code"] != 0 or not (task_dir / "canonical/artifact.json").is_file():
            raise RuntimeError(f"canonicalization failed (exit={canonical['exit_code']})")
        record.update({
            "status": "success",
            "source_artifact": str(step),
            "source_artifact_sha256": sha256(step),
            "step_size_bytes": step.stat().st_size,
            "metric_glb": str(glb),
            "metric_glb_sha256": sha256(glb),
            "metric_glb_size_bytes": glb.stat().st_size,
            "step_to_glb": conversion,
            "canonicalization": canonical,
            "canonical_dir": str(task_dir / "canonical"),
        })
    except Exception as exc:
        record["failure"] = {
            "type": type(exc).__name__,
            "message": str(exc),
            "traceback": traceback.format_exc(),
        }
        if not (task_dir / "generation.stderr.txt").exists():
            write_text(task_dir / "generation.stderr.txt", record["failure"]["traceback"])
    record["finished_at"] = datetime.now(timezone.utc).isoformat()
    record["elapsed_s"] = time.monotonic() - started
    dump_json(task_dir / "record.json", record)
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--task-id", action="append")
    parser.add_argument("--timeout", type=float, default=1800.0)
    args = parser.parse_args()
    output = contained(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "3":
        raise RuntimeError("must launch with CUDA_VISIBLE_DEVICES=3 for physical GPU 3")
    if not torch.cuda.is_available() or torch.cuda.device_count() != 1:
        raise RuntimeError("expected exactly one visible CUDA device")
    rows = load_prompts(args.smoke, set(args.task_id or []) or None)
    if not rows:
        raise ValueError("no tasks selected")
    run_started = time.monotonic()
    load_started = time.monotonic()
    tokenizer = AutoTokenizer.from_pretrained(
        MODEL_ROOT,
        trust_remote_code=True,
        use_fast=False,
        model_max_length=1024,
        local_files_only=True,
    )
    tokenizer.pad_token = tokenizer.eos_token
    tokenizer.padding_side = "left"
    model = AutoModelForCausalLM.from_pretrained(
        MODEL_ROOT,
        trust_remote_code=True,
        torch_dtype=torch.bfloat16,
        device_map="auto",
        local_files_only=True,
    )
    model.eval()
    load_elapsed = time.monotonic() - load_started
    config = {
        "schema_version": 2,
        "mode": "smoke" if args.smoke else "frozen_benchmark",
        "method": "text_to_cadquery",
        "official_repository": "https://github.com/Text-to-CadQuery/Text-to-CadQuery",
        "code_revision": CODE_REVISION,
        "model_id": MODEL_ID,
        "model_revision": MODEL_REVISION,
        "model_weight_sha256": sha256(MODEL_ROOT / "model.safetensors"),
        "prompt_manifest_sha256": PROMPT_MANIFEST_SHA256 if not args.smoke else None,
        "selected_task_ids": [row["task_id"] for row in rows],
        "repair_budget": 0,
        "timeout_s_per_task": args.timeout,
        "physical_gpu": 3,
        "visible_cuda_device_count": torch.cuda.device_count(),
        "visible_cuda_name": torch.cuda.get_device_name(0),
        "model_load_elapsed_s": load_elapsed,
        "python": platform.python_version(),
        "torch": torch.__version__,
        "transformers": transformers.__version__,
        "cadquery": cq.__version__,
        "trimesh": trimesh.__version__,
        "command": [sys.executable, *sys.argv],
        "started_at": datetime.now(timezone.utc).isoformat(),
    }
    dump_json(output / "run_config.json", config)
    records = []
    for row in rows:
        record = generate_one(model, tokenizer, row, output, args.timeout)
        records.append(record)
        print(json.dumps({
            "task_id": row["task_id"],
            "status": record["status"],
            "elapsed_s": record["elapsed_s"],
        }), flush=True)
        torch.cuda.empty_cache()
    manifest = output / "artifact_manifest.jsonl"
    write_text(manifest, "".join(json.dumps(row, sort_keys=True, ensure_ascii=False) + "\n" for row in records))
    summary = {
        "schema_version": 2,
        "mode": config["mode"],
        "tasks": len(records),
        "success": sum(row["status"] == "success" for row in records),
        "failed": sum(row["status"] != "success" for row in records),
        "step_artifacts": sum(bool(row.get("source_artifact")) for row in records),
        "glb_artifacts": sum(bool(row.get("metric_glb")) for row in records),
        "total_elapsed_s": time.monotonic() - run_started,
        "artifact_manifest": str(manifest),
    }
    dump_json(output / "run_summary.json", summary)
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary["failed"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
