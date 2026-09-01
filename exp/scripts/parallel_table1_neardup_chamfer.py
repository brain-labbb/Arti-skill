#!/usr/bin/env python3
"""Parallelize exact Table 1 Chamfer artifact generation.

This helper only produces the frozen ``chamfer_pairs.bin`` artifact consumed
by ``run_table1_diversity_metrics.py score``.  It imports the runner's exact
Chamfer implementation and writes the same ``<IIf`` candidate identity plus
``<IId`` distance records in candidate-file order.  It does not aggregate or
publish a metric, so the frozen runner remains the sole writer of score and
summary JSON artifacts.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import importlib.util
import json
import multiprocessing as mp
import os
from pathlib import Path
import struct
import sys
from typing import Any

import numpy as np
from scipy.spatial import cKDTree


SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[2]
RUNNER_PATH = SCRIPT_PATH.with_name("run_table1_diversity_metrics.py")
PAIR_DTYPE = np.dtype([("left", "<u4"), ("right", "<u4"), ("distance", "<f4")])
CHAMFER_DTYPE = np.dtype([("left", "<u4"), ("right", "<u4"), ("distance", "<f8")])
CHAMFER_STRUCT = struct.Struct("<IId")


def load_runner() -> Any:
    spec = importlib.util.spec_from_file_location("table1_neardup_runner_for_parallel", RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load runner: {RUNNER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


RUNNER = load_runner()
POINTS: np.ndarray | None = None
PAIRS: np.memmap | None = None


def init_worker(points_path: str, pairs_path: str, pair_count: int) -> None:
    global POINTS, PAIRS
    # The arrays are read-only mmaps; forked workers share the page cache.
    POINTS = np.load(points_path, mmap_mode="r")
    PAIRS = np.memmap(pairs_path, dtype=PAIR_DTYPE, mode="r", shape=(pair_count,))


def compute_chunk(bounds: tuple[int, int]) -> tuple[int, bytes]:
    start, stop = bounds
    if POINTS is None or PAIRS is None:
        raise RuntimeError("worker was not initialized")
    payload = bytearray((stop - start) * CHAMFER_STRUCT.size)
    offset = 0
    for row in PAIRS[start:stop]:
        left_index, right_index = int(row["left"]), int(row["right"])
        left = np.asarray(POINTS[left_index], dtype=np.float64)
        right = np.asarray(POINTS[right_index], dtype=np.float64)
        left_to_right = cKDTree(right).query(left, k=1, workers=1)[0]
        right_to_left = cKDTree(left).query(right, k=1, workers=1)[0]
        distance = float(0.5 * (np.mean(left_to_right) + np.mean(right_to_left)))
        if not np.isfinite(distance) or distance < 0:
            raise ValueError(f"invalid Chamfer distance at candidate offset {start + offset // CHAMFER_STRUCT.size}")
        CHAMFER_STRUCT.pack_into(payload, offset, left_index, right_index, distance)
        offset += CHAMFER_STRUCT.size
    return start, bytes(payload)


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False).encode()


def sha256_file(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: dict[str, Any]) -> None:
    import os
    import tempfile

    result = dict(value)
    result.pop("checkpoint_content_sha256", None)
    result["checkpoint_content_sha256"] = __import__("hashlib").sha256(canonical_bytes(result)).hexdigest()
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=True).encode() + b"\n")
            handle.flush(); os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def read_self_hashed(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    unsigned = dict(value)
    declared = unsigned.pop("checkpoint_content_sha256", None)
    import hashlib

    if declared != hashlib.sha256(canonical_bytes(unsigned)).hexdigest():
        raise ValueError(f"checkpoint self-hash mismatch: {path}")
    return value


@contextmanager
def exclusive_lock(path: Path):
    """Prevent a frozen runner and this helper from sharing one temp file."""
    descriptor = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError(f"score artifact is locked: {path}") from error
        os.ftruncate(descriptor, 0)
        os.write(descriptor, f"pid={os.getpid()}\n".encode())
        os.fsync(descriptor)
        yield
    finally:
        fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)


def validate_prefix(
    temporary: Path, pairs_path: Path, completed: int, pair_count: int
) -> None:
    """Check the durable prefix before appending new distances."""
    if completed == 0:
        return
    stored = np.memmap(temporary, dtype=CHAMFER_DTYPE, mode="r", shape=(completed,))
    source = np.memmap(pairs_path, dtype=PAIR_DTYPE, mode="r", shape=(pair_count,))
    if not np.array_equal(stored["left"], source[:completed]["left"]):
        raise ValueError("checkpoint prefix left-ordinal mismatch")
    if not np.array_equal(stored["right"], source[:completed]["right"]):
        raise ValueError("checkpoint prefix right-ordinal mismatch")
    distances = np.asarray(stored["distance"])
    if not np.isfinite(distances).all() or (distances < 0).any():
        raise ValueError("checkpoint prefix contains an invalid distance")


def score_dataset(output: Path, dataset: str, workers: int, chunk_size: int) -> None:
    formal = output / "near_duplicate" / dataset
    geometry = RUNNER.read_self_hashed_json(formal / "geometry_summary.json", "summary_content_sha256")
    candidates = RUNNER.read_self_hashed_json(formal / "candidate_summary.json", "summary_content_sha256")
    receipt = RUNNER.read_self_hashed_json(output / "near_duplicate/calibration/threshold_receipt.json", "receipt_content_sha256")
    RUNNER.verify_scratch_bindings(geometry)
    RUNNER.verify_candidate_bindings(formal, candidates)
    if receipt.get("status") != "PASS" or receipt.get("chamfer_protocol") != RUNNER.CHAMFER_PROTOCOL:
        raise ValueError("threshold receipt is not a passing frozen Chamfer receipt")
    if receipt.get("run_manifest_sha256") != sha256_file(output / "manifest.json"):
        raise ValueError("threshold receipt is not bound to this run manifest")
    points_path = Path(str(geometry["scratch_points_path"]))
    pairs_path = Path(str(candidates["candidate_pairs_path"]))
    pair_count = int(candidates["candidate_pair_count"])
    destination = formal / "chamfer_pairs.bin"
    temporary = destination.with_name(f".{destination.name}.tmp")
    checkpoint_path = formal / "score_checkpoint.json"
    expected = {
        "dataset_key": dataset,
        "tau": float(receipt["tau"]),
        "threshold_receipt_sha256": sha256_file(output / "near_duplicate/calibration/threshold_receipt.json"),
        "candidate_pairs_sha256": candidates["candidate_pairs_sha256"],
        "candidate_pair_count": pair_count,
        "chamfer_protocol": RUNNER.CHAMFER_PROTOCOL,
    }
    lock_path = formal / ".chamfer_pairs.lock"
    with exclusive_lock(lock_path):
        _score_dataset_locked(
            formal,
            dataset,
            points_path,
            pairs_path,
            pair_count,
            destination,
            temporary,
            checkpoint_path,
            expected,
            workers,
            chunk_size,
        )


def _score_dataset_locked(
    formal: Path,
    dataset: str,
    points_path: Path,
    pairs_path: Path,
    pair_count: int,
    destination: Path,
    temporary: Path,
    checkpoint_path: Path,
    expected: dict[str, Any],
    workers: int,
    chunk_size: int,
) -> None:
    if destination.is_file():
        if destination.stat().st_size != pair_count * CHAMFER_STRUCT.size:
            raise ValueError(f"completed artifact has wrong size: {destination}")
        checkpoint = read_self_hashed(checkpoint_path) if checkpoint_path.is_file() else None
        if checkpoint is not None:
            for field, value in expected.items():
                if checkpoint.get(field) != value:
                    raise ValueError(f"{dataset} completed artifact binding mismatch: {field}")
        validate_prefix(destination, pairs_path, pair_count, pair_count)
        artifact_hash = sha256_file(destination)
        if checkpoint is None or checkpoint.get("state") != "COMPLETE" or checkpoint.get("chamfer_pairs_sha256") != artifact_hash:
            # A kill between os.replace and the final checkpoint write is
            # recoverable: the complete, validated artifact becomes the new
            # atomic boundary.
            write_json(
                checkpoint_path,
                {
                    **expected,
                    "schema_version": "table1_neardup_score_checkpoint_v1",
                    "completed_pair_count": pair_count,
                    "state": "COMPLETE",
                    "chamfer_pairs_sha256": artifact_hash,
                    "updated_at_utc": RUNNER.utc_now(),
                },
            )
        print(f"[parallel-score] {dataset}: already complete ({pair_count})", flush=True)
        return
    formal.mkdir(parents=True, exist_ok=True)
    completed = 0
    if checkpoint_path.is_file():
        checkpoint = read_self_hashed(checkpoint_path)
        for field, value in expected.items():
            if checkpoint.get(field) != value:
                raise ValueError(f"{dataset} checkpoint binding mismatch: {field}")
        completed = int(checkpoint.get("completed_pair_count", -1))
        if not 0 <= completed <= pair_count:
            raise ValueError(f"{dataset} invalid completed count: {completed}")
        if not temporary.is_file():
            raise ValueError(f"{dataset} checkpoint has no temporary artifact")
        with temporary.open("r+b") as handle:
            committed = completed * CHAMFER_STRUCT.size
            if handle.seek(0, 2) < committed:
                raise ValueError(f"{dataset} temporary artifact is shorter than checkpoint")
            handle.truncate(committed); handle.flush(); __import__("os").fsync(handle.fileno())
        validate_prefix(temporary, pairs_path, completed, pair_count)
    else:
        if temporary.exists():
            raise ValueError(f"{dataset} has an unbound temporary artifact without a checkpoint")
        with temporary.open("wb") as handle:
            handle.flush(); __import__("os").fsync(handle.fileno())
        write_json(temporary.with_name("score_checkpoint.json"), {**expected, "schema_version": "table1_neardup_score_checkpoint_v1", "completed_pair_count": 0, "updated_at_utc": RUNNER.utc_now()})

    bounds = [(start, min(start + chunk_size, pair_count)) for start in range(completed, pair_count, chunk_size)]
    context = mp.get_context("fork")
    with temporary.open("ab") as handle:
        with context.Pool(workers, initializer=init_worker, initargs=(str(points_path), str(pairs_path), pair_count)) as pool:
            for start, payload in pool.imap(compute_chunk, bounds, chunksize=1):
                expected_start = completed
                if start != expected_start:
                    raise RuntimeError(f"{dataset} chunk order mismatch: {start} != {expected_start}")
                handle.write(payload); handle.flush(); __import__("os").fsync(handle.fileno())
                completed = start + len(payload) // CHAMFER_STRUCT.size
                write_json(checkpoint_path, {**expected, "schema_version": "table1_neardup_score_checkpoint_v1", "completed_pair_count": completed, "updated_at_utc": RUNNER.utc_now()})
                print(f"[parallel-score] {dataset}: {completed}/{pair_count}", flush=True)
    __import__("os").replace(temporary, destination)
    write_json(checkpoint_path, {**expected, "schema_version": "table1_neardup_score_checkpoint_v1", "completed_pair_count": pair_count, "state": "COMPLETE", "chamfer_pairs_sha256": sha256_file(destination), "updated_at_utc": RUNNER.utc_now()})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--workers", type=int, default=16)
    parser.add_argument("--chunk-size", type=int, default=2048)
    args = parser.parse_args()
    if args.workers < 1 or args.chunk_size < 1:
        raise SystemExit("--workers and --chunk-size must be positive")
    score_dataset(args.output.resolve(), args.dataset, args.workers, args.chunk_size)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
