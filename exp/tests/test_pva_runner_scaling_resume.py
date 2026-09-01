from __future__ import annotations

import gc
import importlib.util
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import textwrap
import time
from typing import Any, Callable
import weakref

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"


def _load_runner() -> Any:
    path = SCRIPT_DIR / "run_pva_table1234_full_release.py"
    name = "pva_runner_scaling_resume_test"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _wait_until(predicate: Callable[[], bool], timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("condition did not become true before timeout")


def _prefix_database(size: int, completed: int) -> sqlite3.Connection:
    connection = sqlite3.connect(":memory:")
    connection.executescript(
        """
        CREATE TABLE assets(
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            row_json TEXT NOT NULL
        );
        CREATE TABLE results(
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE
        );
        """
    )
    connection.executemany(
        "INSERT INTO assets(ordinal, asset_id, row_json) VALUES(?, ?, ?)",
        (
            (index, f"asset-{index}", json.dumps({"ordinal": index, "asset_id": f"asset-{index}"}))
            for index in range(size)
        ),
    )
    connection.executemany(
        "INSERT INTO results(ordinal, asset_id) VALUES(?, ?)",
        ((index, f"asset-{index}") for index in range(completed)),
    )
    connection.commit()
    return connection


def test_pending_rows_uses_a_validated_prefix_cursor_without_rescanning() -> None:
    runner = _load_runner()
    connection = _prefix_database(size=10_000, completed=9_000)
    try:
        next_ordinal = runner._validated_result_prefix(connection)
        assert next_ordinal == 9_000

        vm_steps = 0

        def count_step() -> int:
            nonlocal vm_steps
            vm_steps += 1
            return 0

        connection.set_progress_handler(count_step, 1)
        pending = runner._pending_rows(
            connection,
            start_ordinal=next_ordinal,
            limit=5,
        )
        connection.set_progress_handler(None, 0)

        assert [row["ordinal"] for row in pending] == [9_000, 9_001, 9_002, 9_003, 9_004]
        assert vm_steps < 200
    finally:
        connection.close()


def test_result_prefix_rejects_a_gap_before_evaluation_resumes() -> None:
    runner = _load_runner()
    connection = _prefix_database(size=20, completed=15)
    try:
        connection.execute("DELETE FROM results WHERE ordinal=7")
        connection.commit()
        with pytest.raises(ValueError, match="contiguous prefix"):
            runner._validated_result_prefix(connection)
    finally:
        connection.close()


def test_asset_lock_survives_parent_crash_and_resume_never_relaunches(
    tmp_path: Path,
) -> None:
    runner_path = SCRIPT_DIR / "run_pva_table1234_full_release.py"
    fake_python = tmp_path / "fake-python"
    harness = tmp_path / "harness.py"
    row_path = tmp_path / "row.json"
    output = tmp_path / "evaluation"
    invocation_path = tmp_path / "invocations.txt"
    gate_path = tmp_path / "release-child"
    first_result = tmp_path / "first-result.json"
    second_result = tmp_path / "second-result.json"
    recovered_result = tmp_path / "recovered-result.json"

    fake_python.write_text(
        textwrap.dedent(
            """\
            #!/usr/bin/env python3
            import json
            import os
            from pathlib import Path
            import sys
            import time

            def argument(name):
                return sys.argv[sys.argv.index(name) + 1]

            child_root = Path(argument("--child-output"))
            job = json.loads(Path(argument("--job")).read_text(encoding="utf-8"))
            invocation = Path(os.environ["PVA_LOCK_TEST_INVOCATIONS"])
            gate = Path(os.environ["PVA_LOCK_TEST_GATE"])
            child_root.mkdir(parents=True, exist_ok=True)
            descriptor = os.open(invocation, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
            try:
                os.write(descriptor, f"{os.getpid()}\\n".encode("ascii"))
            finally:
                os.close(descriptor)
            (child_root / "active-sentinel.txt").write_text(str(os.getpid()), encoding="ascii")
            deadline = time.monotonic() + 20.0
            while not gate.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not gate.exists():
                raise SystemExit(3)
            row = job["row"]
            records = {
                "table1.json": {
                    "asset_id": row["asset_id"],
                    "parse_success": True,
                    "error": None,
                },
                "table2.json": {"asset_id": row["asset_id"]},
                "table2_supplementary.json": {"asset_id": row["asset_id"]},
                "table3.json": {"asset_id": row["asset_id"]},
                "table4.json": {
                    "dataset_id": row["asset_id"],
                    "state_records": [],
                },
            }
            for name, record in records.items():
                (child_root / name).write_text(json.dumps(record), encoding="utf-8")
            (child_root / "completion.json").write_text(
                json.dumps({"asset_id": row["asset_id"], "ordinal": row["ordinal"]}),
                encoding="utf-8",
            )
            """
        ),
        encoding="utf-8",
    )
    fake_python.chmod(0o755)
    harness.write_text(
        textwrap.dedent(
            """\
            import importlib.util
            import json
            import os
            from pathlib import Path
            import sys

            runner_path = Path(os.environ["PVA_LOCK_TEST_RUNNER"])
            spec = importlib.util.spec_from_file_location("pva_lock_harness_runner", runner_path)
            runner = importlib.util.module_from_spec(spec)
            assert spec is not None and spec.loader is not None
            sys.modules[spec.name] = runner
            spec.loader.exec_module(runner)
            runner.sys.executable = os.environ["PVA_LOCK_TEST_FAKE_PYTHON"]
            row = json.loads(Path(os.environ["PVA_LOCK_TEST_ROW"]).read_text(encoding="utf-8"))
            payload = runner._execute_asset(
                row,
                Path(os.environ["PVA_LOCK_TEST_OUTPUT"]),
                timeout_seconds=float(os.environ["PVA_LOCK_TEST_TIMEOUT"]),
                run_standard_parser=os.environ["PVA_LOCK_TEST_PARSER"] == "1",
            )
            table1 = json.loads(payload["table1_json"])
            Path(os.environ["PVA_LOCK_TEST_RESULT"]).write_text(
                json.dumps(
                    {
                        "worker_status": payload["worker_status"],
                        "child_root_is_none": payload["child_root"] is None,
                        "table1_parse_success": table1["parse_success"],
                        "table1_error": table1["error"],
                    }
                ),
                encoding="utf-8",
            )
            """
        ),
        encoding="utf-8",
    )
    row = {
        "ordinal": 0,
        "asset_id": "PV-A/Alpha/seed_0000",
        "raw_category": "Alpha",
        "category": "Alpha",
        "joint_count": 1,
        "source_path": str(tmp_path / "package"),
        "primary_urdf_path": str(tmp_path / "package" / "model.urdf"),
        "primary_urdf_relative_path": "model.urdf",
        "primary_urdf_sha256": "0" * 64,
        "package_binding_sha256": "1" * 64,
    }
    row_path.write_text(json.dumps(row), encoding="utf-8")
    environment = {
        **os.environ,
        "PYTHONDONTWRITEBYTECODE": "1",
        "PVA_LOCK_TEST_RUNNER": str(runner_path),
        "PVA_LOCK_TEST_FAKE_PYTHON": str(fake_python),
        "PVA_LOCK_TEST_ROW": str(row_path),
        "PVA_LOCK_TEST_OUTPUT": str(output),
        "PVA_LOCK_TEST_INVOCATIONS": str(invocation_path),
        "PVA_LOCK_TEST_GATE": str(gate_path),
    }
    first_environment = {
        **environment,
        "PVA_LOCK_TEST_TIMEOUT": "15",
        "PVA_LOCK_TEST_PARSER": "1",
        "PVA_LOCK_TEST_RESULT": str(first_result),
    }
    first_parent = subprocess.Popen(
        [sys.executable, str(harness)],
        env=first_environment,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    child_pid: int | None = None
    try:
        _wait_until(lambda: invocation_path.is_file() and invocation_path.stat().st_size > 0)
        child_pid = int(invocation_path.read_text(encoding="ascii").splitlines()[0])
        sentinel = output / "children" / "000000000" / "active-sentinel.txt"

        def original_sentinel_is_complete() -> bool:
            try:
                return sentinel.read_text(encoding="ascii") == str(child_pid)
            except FileNotFoundError:
                return False

        _wait_until(original_sentinel_is_complete)

        os.kill(first_parent.pid, signal.SIGKILL)
        first_parent.wait(timeout=5)

        second_environment = {
            **environment,
            "PVA_LOCK_TEST_TIMEOUT": "0.25",
            "PVA_LOCK_TEST_PARSER": "0",
            "PVA_LOCK_TEST_RESULT": str(second_result),
        }
        completed = subprocess.run(
            [sys.executable, str(harness)],
            env=second_environment,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=8,
            check=False,
        )
        assert completed.returncode == 0, completed.stderr
        observed = json.loads(second_result.read_text(encoding="utf-8"))
        invocations = invocation_path.read_text(encoding="ascii").splitlines()

        assert observed["worker_status"] == "lock_timeout"
        assert observed["child_root_is_none"] is True
        assert observed["table1_parse_success"] is False
        assert "asset_lock_timeout" in observed["table1_error"]
        assert invocations == [str(child_pid)]
        assert sentinel.read_text(encoding="ascii") == str(child_pid)

        gate_path.touch()
        completion = output / "children" / "000000000" / "completion.json"
        _wait_until(completion.is_file)
        recovered_environment = {
            **environment,
            "PVA_LOCK_TEST_TIMEOUT": "5",
            "PVA_LOCK_TEST_PARSER": "1",
            "PVA_LOCK_TEST_RESULT": str(recovered_result),
        }
        recovered = subprocess.run(
            [sys.executable, str(harness)],
            env=recovered_environment,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=8,
            check=False,
        )
        assert recovered.returncode == 0, recovered.stderr
        recovered_payload = json.loads(recovered_result.read_text(encoding="utf-8"))
        assert recovered_payload == {
            "worker_status": "recovered",
            "child_root_is_none": False,
            "table1_parse_success": True,
            "table1_error": None,
        }
        assert invocation_path.read_text(encoding="ascii").splitlines() == [str(child_pid)]
    finally:
        gate_path.touch()
        if first_parent.poll() is None:
            first_parent.kill()
            first_parent.wait(timeout=5)
        if child_pid is not None:
            try:
                os.killpg(child_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def _finalize_fixture(tmp_path: Path) -> tuple[sqlite3.Connection, Path, Path, dict[str, Any], dict[str, Any]]:
    output = tmp_path / "evaluation"
    output.mkdir()
    (output / "manifest.json").write_text("{}\n", encoding="utf-8")
    roster_path = tmp_path / "roster_manifest.json"
    roster_path.write_text("{}\n", encoding="utf-8")
    full_row = {
        "ordinal": 0,
        "asset_id": "PV-A/Alpha/seed_0000",
        "raw_category": "Alpha",
        "category": "Alpha",
        "joint_count": 1,
        "primary_urdf_sha256": "a" * 64,
        "source_path": "/frozen/package",
        "package_binding_sha256": "b" * 64,
        "package_files": [{"path": "large.mesh", "size": 10, "sha256": "c" * 64}],
        "overrides_json": "{\"large\":true}",
    }
    connection = sqlite3.connect(":memory:")
    connection.executescript(
        """
        CREATE TABLE assets(
            ordinal INTEGER PRIMARY KEY,
            asset_id TEXT NOT NULL UNIQUE,
            category TEXT NOT NULL,
            joint_count INTEGER NOT NULL,
            row_json TEXT NOT NULL
        );
        CREATE TABLE results(ordinal INTEGER PRIMARY KEY);
        INSERT INTO results(ordinal) VALUES(0);
        """
    )
    connection.execute(
        "INSERT INTO assets(ordinal, asset_id, category, joint_count, row_json) "
        "VALUES(0, ?, ?, ?, ?)",
        (
            full_row["asset_id"],
            full_row["raw_category"],
            full_row["joint_count"],
            json.dumps(full_row, sort_keys=True),
        ),
    )
    roster = {
        "N_release": 1,
        "release_category_count": 1,
        "roster": {"sha256": "d" * 64},
        "manifest_content_sha256": "e" * 64,
    }
    execution = {"classification": "TEST_FIXTURE"}
    return connection, output, roster_path, roster, execution


def _stub_finalize_dependencies(
    runner: Any,
    monkeypatch: pytest.MonkeyPatch,
    aggregate_hook: Callable[[str, Any], None],
) -> None:
    def table1_aggregate(records: Any, roster: Any) -> dict[str, Any]:
        aggregate_hook("table1", roster)
        return {"cohort": {}}

    def aggregate(name: str) -> Callable[..., dict[str, Any]]:
        def call(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
            aggregate_hook(name, None)
            return {}

        return call

    monkeypatch.setattr(runner.table1, "aggregate_full_release", table1_aggregate)
    monkeypatch.setattr(runner.table2, "aggregate_full_release", aggregate("table2"))
    monkeypatch.setattr(runner.table2sup, "aggregate_records", aggregate("table2_supplementary"))
    monkeypatch.setattr(runner.table3, "aggregate_full_release", aggregate("table3"))
    monkeypatch.setattr(runner.table4, "aggregate_records", aggregate("table4"))
    monkeypatch.setattr(
        runner,
        "_publish_table",
        lambda _output, _name, records, _summary, **_kwargs: {"records_count": len(records)},
    )


def test_finalize_exposes_only_the_minimal_roster_contract(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = _load_runner()
    connection, output, roster_path, roster, execution = _finalize_fixture(tmp_path)
    seen_row: dict[str, Any] = {}

    def capture(name: str, pseudo_roster: Any) -> None:
        if name == "table1":
            seen_row.update(pseudo_roster["rows"][0])

    _stub_finalize_dependencies(runner, monkeypatch, capture)
    monkeypatch.setattr(
        runner,
        "_result_records",
        lambda _connection, _column: iter(({},)),
    )
    try:
        runner._finalize(
            connection,
            output,
            roster_path,
            roster,
            execution,
            n_eval=1,
            j_eval=1,
            category_count=1,
        )
    finally:
        connection.close()

    assert seen_row == {
        "ordinal": 0,
        "asset_id": "PV-A/Alpha/seed_0000",
        "raw_category": "Alpha",
        "category": "Alpha",
        "joint_count": 1,
        "primary_urdf_sha256": "a" * 64,
    }


def test_finalize_releases_one_table_before_materializing_the_next(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner = _load_runner()
    connection, output, roster_path, roster, execution = _finalize_fixture(tmp_path)
    table_by_column = {column: name for name, column in runner.RESULT_COLUMNS.items()}
    active: set[str] = set()

    class TrackedRecord(dict[str, Any]):
        __slots__ = ("__weakref__",)

    def tracked_records(_connection: Any, column: str) -> Any:
        table = table_by_column[column]
        record = TrackedRecord()
        active.add(table)
        weakref.finalize(record, active.discard, table)
        yield record

    def require_single_table(name: str, _pseudo_roster: Any) -> None:
        gc.collect()
        assert active == {name}

    _stub_finalize_dependencies(runner, monkeypatch, require_single_table)
    monkeypatch.setattr(runner, "_result_records", tracked_records)
    try:
        runner._finalize(
            connection,
            output,
            roster_path,
            roster,
            execution,
            n_eval=1,
            j_eval=1,
            category_count=1,
        )
    finally:
        connection.close()
