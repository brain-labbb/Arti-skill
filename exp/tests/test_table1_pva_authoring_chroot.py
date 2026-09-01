from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


EXP_ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = EXP_ROOT / "scripts/run_table1_pva_authoring.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("run_table1_pva_authoring", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


runner = load_runner()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_minimal_jail(root: Path) -> None:
    for relative in (
        "bin",
        "usr/bin",
        "lib",
        "lib64",
        "workspaces",
        "public/sdk",
    ):
        (root / relative).mkdir(parents=True, exist_ok=True)
    tools = [
        Path("/bin/dash"),
        Path("/usr/bin/stat"),
        Path("/usr/bin/id"),
        Path("/usr/bin/sleep"),
    ]
    for source in tools:
        runner.copy_runtime_file(source, root)
    shutil.copy2("/bin/dash", root / "bin/sh")
    fake_codex = root / "bin/codex"
    fake_codex.write_text(
        """#!/bin/sh
set -eu
printf '%s\n' 'TEMPLATE_DOMAIN = "fixture"' > /workspaces/wired/output/template.py
printf '%s\n' '{"type":"turn.completed","usage":{"input_tokens":7,"output_tokens":3}}'
""",
        encoding="utf-8",
    )
    fake_codex.chmod(0o755)
    for dependency in runner.elf_dependencies(tools):
        runner.copy_runtime_file(dependency, root)

    public_file = root / "public/sdk/public_api.py"
    public_file.write_text("PUBLIC = True\n", encoding="utf-8")
    for path in sorted(root.rglob("*"), key=lambda row: len(row.parts), reverse=True):
        os.chown(path, 0, 0)
        path.chmod(0o555 if path.is_dir() or path.stat().st_mode & stat.S_IXUSR else 0o444)
    root.chmod(0o755)
    (root / "workspaces").chmod(0o711)


def add_probe_runtime(root: Path) -> None:
    python = Path("/usr/bin/python3.12")
    runner.copy_runtime_file(python, root)
    shutil.copytree(
        "/usr/lib/python3.12",
        root / "usr/lib/python3.12",
        symlinks=False,
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.pyo"),
    )
    for dependency in runner.elf_dependencies([python]):
        runner.copy_runtime_file(dependency, root)
    evaluator = root / "evaluator/evaluate_t2_generated_template.py"
    evaluator.parent.mkdir(parents=True, exist_ok=True)
    evaluator.write_text(
        """from __future__ import annotations

import importlib.util
import json
import sys
import traceback
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 7 or sys.argv[1] != "--worker":
        raise SystemExit("fixture evaluator requires --worker")
    template = Path(sys.argv[2])
    case_kind = sys.argv[3]
    case_value = int(sys.argv[4])
    case_dir = Path(sys.argv[5])
    output = Path(sys.argv[6])
    payload = {
        "case_kind": case_kind,
        "case_value": case_value,
        "verdict": "fail",
        "artifact_saved": False,
        "elapsed_s": 0.01,
    }
    try:
        case_dir.mkdir(parents=True, exist_ok=True)
        spec = importlib.util.spec_from_file_location("_fixture_template", template)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        module.write_artifacts(case_dir)
        payload.update({"verdict": "pass", "artifact_saved": True})
    except BaseException as exc:
        payload.update({
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        })
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload) + "\\n", encoding="utf-8")
    return 0 if payload["verdict"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
""",
        encoding="utf-8",
    )
    for path in sorted(root.rglob("*"), key=lambda row: len(row.parts), reverse=True):
        if path.is_symlink():
            raise AssertionError(f"probe fixture jail contains symlink: {path}")
        os.chown(path, 0, 0)
        path.chmod(0o555 if path.is_dir() or path.stat().st_mode & stat.S_IXUSR else 0o444)
    root.chmod(0o755)
    (root / "workspaces").chmod(0o711)


class PvaAuthorChrootBoundaryTest(unittest.TestCase):
    def test_execution_summary_uses_frozen_intent_denominator(self):
        observed = {
            "summary": {
                "state": "observed",
                "executable": True,
                "artifact_saved": True,
                "first_shot": False,
                "final_success": True,
                "repair_turns": 1,
            },
            "attempts": [{}, {}],
        }
        not_evaluable = {
            "summary": {
                "state": "not_evaluable",
                "executable": None,
                "artifact_saved": None,
                "first_shot": None,
                "final_success": None,
                "repair_turns": None,
            },
            "attempts": [{}],
        }

        summary = runner.build_execution_summary(
            [observed, not_evaluable], intended_runs=3
        )

        self.assertEqual(summary["strict_denominator"], 3)
        self.assertEqual(summary["attempted_results"], 2)
        self.assertEqual(summary["evaluable_results"], 1)
        self.assertEqual(summary["metrics"]["final_success"], {
            "numerator": 1,
            "denominator": 3,
        })
        self.assertTrue(
            all(metric["denominator"] == 3 for metric in summary["metrics"].values())
        )

    def test_execution_refusal_is_structured_and_zero_call(self):
        refusal = runner.execution_refusal_record(
            protocol_sha256="1" * 64,
            capability_blockers=["author-code read isolation is incomplete"],
            environment_blockers=[],
        )

        self.assertEqual(refusal["status"], "REFUSED_BEFORE_PROVIDER_CALL")
        self.assertEqual(refusal["authoring_attempts"], 0)
        self.assertEqual(refusal["provider_calls_made"], 0)
        self.assertFalse(refusal["api_or_network_accessed"])
        self.assertEqual(
            refusal["capability_blockers"],
            ["author-code read isolation is incomplete"],
        )

    def test_execute_run_wires_jail_and_always_cleans_the_job_workspace(self):
        with tempfile.TemporaryDirectory(dir=EXP_ROOT) as directory:
            temporary = Path(directory)
            jail = temporary / "jail"
            (jail / "workspaces").mkdir(parents=True)
            out = temporary / "out"
            task = {
                "task_id": "FIX-L1-01",
                "prompt": "Build a hinged fixture.",
                "prompt_sha256": hashlib.sha256(
                    b"Build a hinged fixture."
                ).hexdigest(),
            }
            bindings = {
                key: character * 64
                for key, character in (
                    ("protocol_sha256", "1"),
                    ("manifest_sha256", "2"),
                    ("hidden_specs_sha256", "3"),
                    ("result_schema_sha256", "4"),
                    ("common_evaluator_sha256", "5"),
                    ("package_schema_sha256", "6"),
                )
            }
            protocol = {
                "protocol_id": "fixture",
                "max_common_repair_turns": 3,
                "common_model_binding": {
                    "provider": "openai",
                    "model": "gpt-5",
                    "reasoning_effort": "high",
                },
                "methods": {"pva": {}},
                "timeouts": {"native_retry_limit_per_attempt": 2},
            }
            seen: dict[str, object] = {}

            def fake_author(prompt, run_root, output, trace_prefix, model, effort, timeout, **kwargs):
                seen.update(kwargs)
                self.assertIn("/workspaces/pva__FIX-L1-01__r0", prompt)
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_text('TEMPLATE_DOMAIN = "fixture"\n', encoding="utf-8")
                trace_prefix.parent.mkdir(parents=True, exist_ok=True)
                events = trace_prefix.with_suffix(".events.jsonl")
                stderr = trace_prefix.with_suffix(".stderr.txt")
                events.write_text(
                    '{"type":"turn.completed","usage":{"input_tokens":7,"output_tokens":3}}\n',
                    encoding="utf-8",
                )
                stderr.write_text("", encoding="utf-8")
                return {
                    "started_at_utc": "2026-08-13T00:00:00+00:00",
                    "finished_at_utc": "2026-08-13T00:00:01+00:00",
                    "wall_time_s": 1.0,
                    "exit_code": 0,
                    "timed_out": False,
                    "events": runner.rel(events, run_root),
                    "events_sha256": sha256(events),
                    "stderr": runner.rel(stderr, run_root),
                    "stderr_sha256": sha256(stderr),
                    "usage": {"input_tokens": 7, "output_tokens": 3},
                    "provider_request_id_hash": None,
                    "scope_changes_outside_template": [],
                }

            def fake_evaluator(run_root, attempt_root, template, task_id, repeat_id, attempt_index, bound, timeout):
                attempt_root.mkdir(parents=True, exist_ok=True)
                snapshot = attempt_root / "template.py"
                shutil.copy2(template, snapshot)
                urdf = attempt_root / "model.urdf"
                urdf.write_text("<robot name='fixture'><link name='base'/></robot>\n", encoding="utf-8")
                report_path = attempt_root / "common_evaluator_report.json"
                report = {
                    "verdicts": {
                        "executable": True,
                        "artifact_saved": True,
                        "common_qc_pass": True,
                        "urdf_tree_pass": True,
                        "semantic_roles_pass": True,
                        "joint_spec_pass": True,
                    },
                    "binding_checks": {"all": True},
                    "protocol_checks": {"all": True},
                    "task_checks": {"all": True},
                    "feedback": {"failure_codes": []},
                }
                runner.dump_json(report_path, report)
                return report, {
                    "artifacts": {
                        "urdf": {
                            "path": runner.rel(urdf, run_root),
                            "sha256": sha256(urdf),
                        }
                    },
                    "execution_probe": {
                        "started_at_utc": "2026-08-13T00:00:01+00:00",
                        "finished_at_utc": "2026-08-13T00:00:02+00:00",
                    },
                }

            result = runner.execute_run(
                task,
                "r0",
                out,
                protocol,
                bindings,
                10,
                10,
                jail_root=jail,
                provider_api_key="test-only-not-a-real-key",
                author_func=fake_author,
                evaluator_func=fake_evaluator,
            )

            self.assertTrue(result["summary"]["final_success"])
            self.assertEqual(seen["jail_root"], jail)
            self.assertEqual(list((jail / "workspaces").iterdir()), [])

    def test_timeout_kills_author_descendants_before_next_job(self):
        with tempfile.TemporaryDirectory(dir=EXP_ROOT) as directory:
            root = Path(directory) / "jail"
            root.mkdir()
            build_minimal_jail(root)
            job_root, jail_job = runner.prepare_job_workspace(root, "timeout")
            marker = job_root / "output/descendant.txt"
            command = [
                "/bin/sh",
                "-c",
                f"(/usr/bin/sleep 1; echo survived > {jail_job}/output/descendant.txt) & /usr/bin/sleep 10",
            ]
            with self.assertRaises(subprocess.TimeoutExpired):
                runner.run_author_command_in_jail(
                    root,
                    command,
                    timeout=0.2,
                    author_env={"HOME": str(jail_job / "home"), "PATH": "/bin:/usr/bin"},
                )
            import time

            time.sleep(1.1)
            self.assertFalse(marker.exists())
            runner.cleanup_job_workspace(job_root)

    def test_author_prompts_expose_only_public_and_current_job_paths(self):
        output = Path("/workspaces/example/output/template.py")
        feedback = Path("/workspaces/example/input/repair_feedback_a0.json")
        initial = runner.author_prompt(
            {"prompt": "Build a hinged fixture."}, output, "r0"
        )
        repair = runner.repair_prompt(output, feedback, "r0", 1)
        for prompt in (initial, repair):
            self.assertNotIn(str(EXP_ROOT.parent), prompt)
            self.assertIn("/workspaces/example/output/template.py", prompt)
            self.assertIn("/public/sdk", prompt)
        self.assertIn(str(feedback), repair)

    def test_exclusive_jail_lock_refuses_a_second_holder(self):
        with tempfile.TemporaryDirectory(dir=EXP_ROOT) as directory:
            lock_path = Path(directory) / "author.lock"
            with runner.exclusive_file_lock(lock_path):
                completed = subprocess.run(
                    [
                        sys.executable,
                        "-c",
                        (
                            "import fcntl, pathlib; "
                            f"p=pathlib.Path({str(lock_path)!r}); "
                            "h=p.open('a+'); fcntl.flock(h, fcntl.LOCK_EX|fcntl.LOCK_NB)"
                        ),
                    ],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=False,
                )
            self.assertNotEqual(completed.returncode, 0)
            self.assertRegex(completed.stderr, r"BlockingIOError|Resource temporarily unavailable")

    def test_codex_command_freezes_external_isolation_provider_and_retry_contract(self):
        command = runner.build_codex_command(
            Path("/workspaces/example"),
            "public prompt",
            model="gpt-5",
            reasoning_effort="high",
            native_retry_limit=2,
        )
        rendered = "\n".join(command)
        self.assertEqual(command[:2], ["/bin/codex", "exec"])
        for flag in (
            "--json",
            "--ephemeral",
            "--ignore-user-config",
            "--ignore-rules",
            "--strict-config",
            "--dangerously-bypass-approvals-and-sandbox",
        ):
            self.assertIn(flag, command)
        self.assertIn('model_provider="pva_openai_env"', command)
        self.assertIn('model_reasoning_effort="high"', command)
        self.assertIn('shell_environment_policy.inherit="none"', command)
        self.assertIn("request_max_retries=2", rendered)
        self.assertIn("stream_max_retries=2", rendered)
        self.assertIn('env_key="OPENAI_API_KEY"', rendered)
        self.assertIn('requires_openai_auth=false', rendered)
        self.assertNotIn(str(EXP_ROOT.parent), rendered)

    def test_native_settings_require_exclusive_single_job_isolation(self):
        protocol = {
            "max_common_repair_turns": 3,
            "timeouts": {
                "native_retry_limit_per_attempt": 2,
                "model_response_seconds": 1800,
            },
        }
        native = {
            "adapter_schema": runner.PVA_ISOLATION_SCHEMA,
            "codex_cli": {
                "path": str(runner.STATIC_CODEX),
                "version": runner.codex_cli_version(),
                "subcommand": "exec",
                "json_events": True,
                "ephemeral": True,
                "ignore_user_config": True,
                "ignore_rules": True,
                "external_isolation": "exclusive_single_job_chroot_nonroot_uid_65534",
                "internal_sandbox": "bypassed_inside_external_chroot",
                "shell_environment_inherit": "none",
            },
            "provider": {
                "model_provider": runner.PVA_PROVIDER_ID,
                "base_url": runner.PVA_PROVIDER_BASE_URL,
                "env_key": runner.PVA_PROVIDER_ENV_KEY,
                "wire_api": "responses",
                "requires_openai_auth": False,
            },
            "native_retry_limit_per_attempt": 2,
            "model_response_timeout_seconds": 1800,
            "common_repair_turns": 3,
        }

        self.assertTrue(runner.native_settings_checks(protocol, {"native_settings": native}))

        native["codex_cli"]["external_isolation"] = "per_job_chroot_nonroot_uid_65534"
        self.assertFalse(runner.native_settings_checks(protocol, {"native_settings": native}))

    def test_codex_turn_really_runs_inside_jail_and_exports_only_template(self):
        with tempfile.TemporaryDirectory(dir=EXP_ROOT) as directory:
            temporary = Path(directory)
            root = temporary / "jail"
            root.mkdir()
            build_minimal_jail(root)
            job_root, jail_job = runner.prepare_job_workspace(root, "wired")
            outer = temporary / "outer"
            output = outer / "output/template.py"
            turn = runner.codex_turn(
                "public prompt",
                outer,
                output,
                outer / "traces/attempt_0",
                "gpt-5",
                "high",
                10,
                jail_root=root,
                jail_job_root=jail_job,
                provider_api_key="test-only-not-a-real-key",
                native_retry_limit=2,
            )

            self.assertEqual(
                turn["exit_code"],
                0,
                (outer / turn["stderr"]).read_text(encoding="utf-8"),
            )
            self.assertFalse(turn["timed_out"])
            self.assertEqual(turn["scope_changes_outside_template"], [])
            self.assertEqual(turn["usage"]["input_tokens"], 7)
            self.assertEqual(turn["usage"]["output_tokens"], 3)
            self.assertEqual(output.read_text(encoding="utf-8"), 'TEMPLATE_DOMAIN = "fixture"\n')
            self.assertTrue((outer / turn["events"]).is_file())
            runner.cleanup_job_workspace(job_root)

    def test_elf_dependencies_preserve_the_program_interpreter_path(self):
        self.assertIn(
            Path("/lib64/ld-linux-x86-64.so.2"),
            runner.elf_dependencies([Path("/bin/dash")]),
        )

    def test_real_nonroot_chroot_denies_host_private_runtime_proc_and_credentials(self):
        self.assertEqual(os.geteuid(), 0, "real chroot boundary test requires root")
        hidden_host = EXP_ROOT / "reference/table1_reliability_hidden_specs_v1.json"
        self.assertTrue(hidden_host.is_file())
        arbitrary_host = Path("/mnt/zsn/lyb/arti-skill/exp/Nano3d.md")
        self.assertTrue(arbitrary_host.is_file())

        with tempfile.TemporaryDirectory(dir=EXP_ROOT) as directory:
            root = Path(directory) / "jail"
            root.mkdir()
            build_minimal_jail(root)
            job_root, jail_job = runner.prepare_job_workspace(root, "boundary")
            private = root / "workspaces/private"
            private.mkdir(mode=0o700)
            (private / "secret.txt").write_text("private\n", encoding="utf-8")
            os.chown(private, 0, 0)
            os.chown(private / "secret.txt", 0, 0)
            private.chmod(0o700)
            (private / "secret.txt").chmod(0o400)
            public_hash = sha256(root / "public/sdk/public_api.py")
            command = [
                "/bin/sh",
                "-c",
                """
set -eu
facts="$1/output/facts.txt"
mkdir_errors="$1/output"
discard="$1/output/discard.txt"
{
  echo "uid=$(/usr/bin/id -u)"
  echo "gid=$(/usr/bin/id -g)"
  echo "groups=$(/usr/bin/id -G)"
  echo own-output-ok
} > "$facts"
/usr/bin/stat /mnt/zsn/lyb/arti-skill/exp/reference/table1_reliability_hidden_specs_v1.json > "$discard" 2> "$1/output/hidden.err" || true
/usr/bin/stat /mnt/zsn/lyb/arti-skill/exp/Nano3d.md > "$discard" 2> "$1/output/arbitrary.err" || true
/usr/bin/stat /workspaces/private/secret.txt > "$discard" 2> "$1/output/private.err" || true
/bin/sh -c 'echo mutate > /public/sdk/public_api.py' > "$discard" 2> "$1/output/public_write.err" || true
/usr/bin/stat /proc/self/status > "$discard" 2> "$1/output/proc.err" || true
/usr/bin/stat /home/author/.codex/auth.json > "$discard" 2> "$1/output/auth.err" || true
/usr/bin/stat /root/.codex/auth.json > "$discard" 2> "$1/output/root_auth.err" || true
""",
                "boundary-probe",
                str(jail_job),
            ]
            completed = runner.run_author_command_in_jail(
                root,
                command,
                timeout=10,
                author_env={"HOME": "/home/author", "LANG": "C", "PATH": "/bin:/usr/bin"},
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            output = job_root / "output"
            facts = (output / "facts.txt").read_text(encoding="utf-8")
            self.assertIn("uid=65534", facts)
            self.assertIn("gid=65534", facts)
            self.assertIn("groups=65534", facts)
            self.assertIn("own-output-ok", facts)
            self.assertEqual(sha256(root / "public/sdk/public_api.py"), public_hash)
            self.assertIn("No such file or directory", (output / "hidden.err").read_text())
            self.assertIn("No such file or directory", (output / "arbitrary.err").read_text())
            self.assertRegex((output / "private.err").read_text(), r"Permission denied|No such file")
            self.assertIn("Permission denied", (output / "public_write.err").read_text())
            self.assertIn("No such file or directory", (output / "proc.err").read_text())
            self.assertIn("No such file or directory", (output / "auth.err").read_text())
            self.assertIn("No such file or directory", (output / "root_auth.err").read_text())
            runner.cleanup_job_workspace(job_root)
            self.assertEqual(list((root / "workspaces").iterdir()), [private])

    def test_compile_probe_runs_without_host_or_credential_access_and_exports_allowlist(self):
        self.assertEqual(os.geteuid(), 0, "real chroot boundary test requires root")
        with tempfile.TemporaryDirectory(dir=EXP_ROOT) as directory:
            temporary = Path(directory)
            root = temporary / "jail"
            root.mkdir()
            build_minimal_jail(root)
            add_probe_runtime(root)

            hidden = temporary / "hidden.json"
            other_method = temporary / "other_method.txt"
            arbitrary_host = temporary / "host_only.txt"
            secrets = {
                "credential": "PVA_TEST_CREDENTIAL_MUST_NOT_ESCAPE",
                "hidden": "PVA_TEST_HIDDEN_MUST_NOT_ESCAPE",
                "other": "PVA_TEST_OTHER_METHOD_MUST_NOT_ESCAPE",
                "host": "PVA_TEST_HOST_FILE_MUST_NOT_ESCAPE",
            }
            hidden.write_text(secrets["hidden"], encoding="utf-8")
            other_method.write_text(secrets["other"], encoding="utf-8")
            arbitrary_host.write_text(secrets["host"], encoding="utf-8")

            malicious_source = temporary / "malicious_template.py"
            malicious_source.write_text(
                f'''from __future__ import annotations

import os
from pathlib import Path


def _read(path):
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace")
    except BaseException as exc:
        return f"DENIED:{{type(exc).__name__}}"


def write_artifacts(case_dir):
    evidence = [
        os.environ.get("OPENAI_API_KEY", "CREDENTIAL_ABSENT"),
        _read({str(hidden)!r}),
        _read({str(other_method)!r}),
        _read({str(arbitrary_host)!r}),
        _read("/proc/self/environ"),
    ]
    mesh = case_dir / "assets/meshes/part.obj"
    mesh.parent.mkdir(parents=True, exist_ok=True)
    mesh.write_text(
        "# " + " | ".join(evidence) + "\\n"
        "v 0 0 0\\nv 1 0 0\\nv 0 1 0\\nf 1 2 3\\n",
        encoding="utf-8",
    )
    (case_dir / "not_exported.txt").write_text("must stay jailed", encoding="utf-8")
    (case_dir / "model.urdf").write_text(
        "<robot name='fixture'><link name='base'><visual><geometry>"
        "<mesh filename='assets/meshes/part.obj'/></geometry></visual></link></robot>\\n",
        encoding="utf-8",
    )
''',
                encoding="utf-8",
            )

            job_root, jail_job = runner.prepare_job_workspace(root, "malicious_probe")
            malicious_out = temporary / "malicious_out"
            old_key = os.environ.get("OPENAI_API_KEY")
            os.environ["OPENAI_API_KEY"] = secrets["credential"]
            try:
                probe = runner.run_pva_evaluation(
                    malicious_source,
                    malicious_out,
                    10,
                    jail_root=root,
                    jail_job_root=jail_job,
                    attempt_index=0,
                )
            finally:
                if old_key is None:
                    os.environ.pop("OPENAI_API_KEY", None)
                else:
                    os.environ["OPENAI_API_KEY"] = old_key

            self.assertEqual(probe["exit_code"], 0, probe)
            self.assertEqual(probe["record"]["verdict"], "pass")
            exported = sorted(
                path.relative_to(malicious_out).as_posix()
                for path in malicious_out.rglob("*")
                if path.is_file()
            )
            self.assertEqual(
                exported,
                [
                    "adapter.stderr.txt",
                    "adapter.stdout.txt",
                    "cases/seed_000/assets/meshes/part.obj",
                    "cases/seed_000/model.urdf",
                    "cases/seed_000/result.json",
                    "summary.json",
                ],
            )
            exported_bytes = b"".join(path.read_bytes() for path in malicious_out.rglob("*") if path.is_file())
            for secret in secrets.values():
                self.assertNotIn(secret.encode(), exported_bytes)
            mesh_text = (malicious_out / "cases/seed_000/assets/meshes/part.obj").read_text()
            self.assertIn("CREDENTIAL_ABSENT", mesh_text)
            self.assertGreaterEqual(mesh_text.count("DENIED:"), 4)
            self.assertFalse((malicious_out / "cases/seed_000/not_exported.txt").exists())
            runner.cleanup_job_workspace(job_root)

            normal_source = temporary / "normal_template.py"
            normal_source.write_text(
                '''from pathlib import Path


def write_artifacts(case_dir):
    mesh = case_dir / "assets/meshes/part.obj"
    mesh.parent.mkdir(parents=True, exist_ok=True)
    mesh.write_text("v 0 0 0\\nv 1 0 0\\nv 0 1 0\\nf 1 2 3\\n", encoding="utf-8")
    (case_dir / "model.urdf").write_text(
        "<robot name='normal'><link name='base'><visual><geometry>"
        "<mesh filename='assets/meshes/part.obj'/></geometry></visual></link></robot>\\n",
        encoding="utf-8",
    )
''',
                encoding="utf-8",
            )
            normal_job_root, normal_jail_job = runner.prepare_job_workspace(root, "normal_probe")
            normal_out = temporary / "normal_out"
            normal = runner.run_pva_evaluation(
                normal_source,
                normal_out,
                10,
                jail_root=root,
                jail_job_root=normal_jail_job,
                attempt_index=0,
            )
            self.assertEqual(normal["exit_code"], 0, normal)
            self.assertEqual(normal["record"]["verdict"], "pass")
            self.assertTrue((normal_out / "cases/seed_000/model.urdf").is_file())
            self.assertTrue((normal_out / "cases/seed_000/assets/meshes/part.obj").is_file())
            runner.cleanup_job_workspace(normal_job_root)

    def test_job_workspace_root_is_root_owned_and_only_job_subdirs_are_author_writable(self):
        with tempfile.TemporaryDirectory(dir=EXP_ROOT) as directory:
            root = Path(directory) / "jail"
            (root / "workspaces").mkdir(parents=True)
            job_root, _ = runner.prepare_job_workspace(root, "ownership")
            self.assertEqual(job_root.stat().st_uid, 0)
            self.assertEqual(stat.S_IMODE(job_root.stat().st_mode), 0o755)
            for relative in ("home", "home/.codex", "output", "tmp"):
                path = job_root / relative
                self.assertEqual(path.stat().st_uid, 65534)
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o700)
            runner.cleanup_job_workspace(job_root)

    def test_stale_resume_binding_is_refused(self):
        expected = {"protocol_sha256": "a" * 64, "adapter_sha256": "b" * 64}
        stale = {"bindings": {**expected, "adapter_sha256": "c" * 64}}
        with self.assertRaisesRegex(RuntimeError, "stale result"):
            runner.require_exact_resume_bindings(stale, expected, Path("result.json"))


if __name__ == "__main__":
    unittest.main()
