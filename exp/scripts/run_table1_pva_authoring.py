#!/usr/bin/env python3
"""Prepare or execute the frozen Nano3D Table 1 PV-A authoring arm.

Preparation is local and makes no provider calls.  Execution is deliberately
opt-in because the complete arm contains 162 paid, long-running authoring jobs.
"""

from __future__ import annotations

import argparse
import errno
import fcntl
import hashlib
import json
import os
import re
import shutil
import signal
import stat
import subprocess
import tempfile
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable


REPO_ROOT = Path(__file__).resolve().parents[2]
EXP_ROOT = REPO_ROOT / "exp"
TEMPLATE_ROOT = REPO_ROOT / "arti-template"
DEFAULT_MANIFEST = EXP_ROOT / "reference/table1_reliability_common_authoring_v1.json"
DEFAULT_PROTOCOL = EXP_ROOT / "reference/table1_reliability_protocol_v1.json"
DEFAULT_OUT = EXP_ROOT / "runtime/table1_reliability/pva_authoring_v1"
COMMON_PREFLIGHT = EXP_ROOT / "scripts/preflight_table1_authoring_common.py"
COMMON_EVALUATOR = EXP_ROOT / "scripts/evaluate_table1_authoring_common.py"
PVA_EVALUATOR = EXP_ROOT / "scripts/evaluate_t2_generated_template.py"
RESULT_SCHEMA = EXP_ROOT / "reference/table1_authoring_result_schema_v1.json"
PACKAGE_SCHEMA = EXP_ROOT / "reference/table1_authoring_package_schema_v1.json"
CODEX = Path(shutil.which("codex") or "/mnt/zsn/miniconda3/bin/codex")
STATIC_CODEX = Path(
    "/mnt/zsn/miniconda3/lib/node_modules/@openai/codex/node_modules/"
    "@openai/codex-linux-x64/vendor/x86_64-unknown-linux-musl/bin/codex"
)
CODEX_VENDOR_ROOT = STATIC_CODEX.parent.parent
CHROOT = Path(shutil.which("chroot") or "/usr/sbin/chroot")
PVA_ISOLATION_SCHEMA = "pva_chroot_read_isolation_v1"
PVA_ISOLATION_UID = 65534
PVA_ISOLATION_GID = 65534
PVA_EVALUATOR_UID = 65533
PVA_EVALUATOR_GID = 65533
PVA_PROVIDER_ID = "pva_openai_env"
PVA_PROVIDER_BASE_URL = "https://api.openai.com/v1"
PVA_PROVIDER_ENV_KEY = "OPENAI_API_KEY"
JAIL_WORKSPACE = Path("/workspace")
JAIL_TEMPLATE = JAIL_WORKSPACE / "output/template.py"
JAIL_PUBLIC = Path("/public")
JAIL_CODEX_HOME = Path("/home/author/.codex")
JAIL_PYTHON = Path("/usr/bin/python3.12")
JAIL_SITE_PACKAGES = Path("/opt/venv/lib/python3.12/site-packages")
JAIL_EVALUATOR = Path("/evaluator/evaluate_t2_generated_template.py")
JAIL_EVALUATOR_RUNTIME = Path("/evaluator/runtime")
JAIL_PATH = "/bin:/usr/bin"
JAIL_RUNTIME_DIRNAME = ".pva_author_jail_v1"
JAIL_LOCK_NAME = ".pva_author_jail_v1.lock"
JAIL_PUBLIC_PATHS = (
    "sdk",
    "articraft_template_authoring/AUTHORING.md",
    "articraft_template_authoring/MECHANICAL_PRIORS.md",
    "articraft/__init__.py",
    "articraft/values.py",
)
JAIL_EVALUATOR_RUNTIME_PATHS = (
    "agent/__init__.py",
    "agent/compiler.py",
    "agent/feedback.py",
    "agent/models.py",
    "agent/mp_utils.py",
    "agent/prompts/__init__.py",
    "agent/prompts/loader.py",
    "agent/workspace_docs.py",
)
JAIL_TOOLS = (
    Path("/bin/dash"),
    Path("/usr/bin/env"),
    Path("/usr/bin/python3.12"),
    Path("/usr/bin/ls"),
    Path("/usr/bin/sed"),
    Path("/usr/bin/find"),
    Path("/usr/bin/head"),
    Path("/usr/bin/tail"),
    Path("/usr/bin/wc"),
    Path("/usr/bin/cp"),
    Path("/usr/bin/mv"),
    Path("/usr/bin/mkdir"),
    Path("/usr/bin/stat"),
    Path("/usr/bin/sort"),
    Path("/usr/bin/uniq"),
    Path("/usr/bin/cut"),
    Path("/usr/bin/tr"),
    Path("/usr/bin/xargs"),
    Path("/usr/bin/true"),
    Path("/usr/bin/test"),
)
JAIL_SITE_PACKAGE_PREFIXES = (
    "_distutils_hack",
    "_virtualenv.py",
    "cadquery",
    "cadquery-",
    "cadquery_ocp",
    "cadquery_ocp-",
    "cadquery_ocp.libs",
    "casadi",
    "casadi-",
    "ezdxf",
    "ezdxf-",
    "fcl",
    "fontTools",
    "fonttools-",
    "manifold3d",
    "multimethod",
    "multimethod-",
    "nlopt",
    "nlopt-",
    "networkx",
    "networkx-",
    "numpy",
    "numpy-",
    "numpy.libs",
    "OCP",
    "packaging",
    "packaging-",
    "path",
    "path-",
    "pyparsing",
    "pyparsing-",
    "python_fcl",
    "runtype",
    "runtype-",
    "scipy",
    "scipy-",
    "scipy.libs",
    "setuptools",
    "setuptools-",
    "six.py",
    "trimesh",
    "trimesh-",
    "typing_extensions",
    "typing_extensions-",
    "vtk.py",
    "vtkmodules",
)
PROBE_ASSET_SUFFIXES = frozenset({".dae", ".glb", ".gltf", ".obj", ".off", ".ply", ".stl"})
PROBE_MAX_SOURCE_BYTES = 4 * 1024 * 1024
PROBE_MAX_JSON_BYTES = 1024 * 1024
PROBE_MAX_URDF_BYTES = 16 * 1024 * 1024
PROBE_MAX_ASSET_BYTES = 64 * 1024 * 1024
PROBE_MAX_TOTAL_ASSET_BYTES = 512 * 1024 * 1024
PROBE_MAX_ASSET_FILES = 2048
THREAD_ENV = {
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}


def codex_cli_version() -> str | None:
    if not STATIC_CODEX.is_file():
        return None
    try:
        completed = subprocess.run(
            [str(STATIC_CODEX), "--version"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return completed.stdout.strip() if completed.returncode == 0 else None


def pva_isolation_readiness() -> dict[str, Any]:
    """Report only locally provable readiness; never inspect or copy credentials."""

    checks = {
        "linux_root_euid": os.geteuid() == 0,
        "chroot_executable": CHROOT.is_file() and os.access(CHROOT, os.X_OK),
        "static_codex_executable": STATIC_CODEX.is_file()
        and os.access(STATIC_CODEX, os.X_OK),
        "codex_cli_version_identified": bool(codex_cli_version()),
        "isolated_uid_is_nonroot": PVA_ISOLATION_UID != 0,
        "credential_file_forbidden": True,
        "credential_transport_without_secret_file": True,
    }
    return {
        "schema_version": PVA_ISOLATION_SCHEMA,
        "mechanism": "exclusive single-job chroot plus non-root uid/gid and root-owned public runtime",
        "chroot_path": str(CHROOT),
        "static_codex_path": str(STATIC_CODEX),
        "codex_cli_version": codex_cli_version(),
        "author_uid": PVA_ISOLATION_UID,
        "author_gid": PVA_ISOLATION_GID,
        "proc_mounted": False,
        "author_visible_roots": [
            str(JAIL_WORKSPACE),
            str(JAIL_PUBLIC),
            "/bin",
            "/usr",
            "/opt/venv",
            "/etc/ssl",
            "/tmp",
            "/home/author",
        ],
        "codex_session_mode": "ephemeral independent process per common attempt",
        "codex_user_config": "ignored",
        "codex_internal_sandbox": "bypassed_inside_external_chroot",
        "shell_environment_policy": {
            "inherit": "none",
            "set_keys": [
                "HOME",
                "LANG",
                "PATH",
                "PYTHONDONTWRITEBYTECODE",
                "PYTHONPATH",
                "TMPDIR",
            ],
        },
        "credential_policy": (
            "custom provider reads OPENAI_API_KEY only in the Codex parent process; no credential "
            "file is copied or created, and model shell inherits no parent environment"
        ),
        "checks": checks,
        "ready": all(checks.values()),
    }


def pva_environment_readiness() -> dict[str, Any]:
    return {
        "provider": PVA_PROVIDER_ID,
        "required_env_key": PVA_PROVIDER_ENV_KEY,
        "credential_present": bool(os.environ.get(PVA_PROVIDER_ENV_KEY)),
        "credential_value_recorded": False,
        "ready": bool(os.environ.get(PVA_PROVIDER_ENV_KEY)),
    }


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def dump_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def read_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def rel(path: Path, root: Path) -> str:
    return str(path.resolve().relative_to(root.resolve()))


def ensure_beneath(path: Path, root: Path) -> Path:
    resolved = path.resolve()
    resolved.relative_to(root.resolve())
    return resolved


def copy_public_tree(source: Path, destination: Path) -> None:
    if source.is_symlink():
        raise RuntimeError(f"public runtime source cannot be a symlink: {source}")
    if source.is_dir():
        shutil.copytree(
            source,
            destination,
            symlinks=False,
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.pyo"),
        )
    elif source.is_file():
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    else:
        raise FileNotFoundError(source)


def copy_runtime_file(source: Path, jail_root: Path) -> None:
    if not source.is_absolute() or not source.is_file():
        raise FileNotFoundError(source)
    destination = jail_root / source.relative_to("/")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists():
        shutil.copy2(source, destination, follow_symlinks=True)


def elf_dependencies(paths: list[Path]) -> set[Path]:
    pending = [path for path in paths if path.is_file()]
    seen: set[Path] = set()
    libraries: set[Path] = set()
    while pending:
        path = pending.pop()
        identity = path.resolve()
        if identity in seen:
            continue
        seen.add(identity)
        completed = subprocess.run(
            ["ldd", str(path)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=30,
            check=False,
        )
        if completed.returncode != 0:
            continue
        for token in completed.stdout.split():
            if not token.startswith("/"):
                continue
            dependency = Path(token)
            if dependency.is_file() and dependency not in libraries:
                libraries.add(dependency)
                pending.append(dependency)
    return libraries


def site_package_sources() -> list[Path]:
    root = TEMPLATE_ROOT / ".venv/lib/python3.12/site-packages"
    if not root.is_dir():
        raise FileNotFoundError(root)
    selected: list[Path] = []
    for path in sorted(root.iterdir(), key=lambda row: row.name):
        if any(path.name == prefix or path.name.startswith(prefix) for prefix in JAIL_SITE_PACKAGE_PREFIXES):
            selected.append(path)
    required = {"numpy", "manifold3d.cpython-312-x86_64-linux-gnu.so"}
    missing = sorted(required - {path.name for path in selected})
    if missing:
        raise RuntimeError(f"PVA jail site-package selection missing: {missing}")
    return selected


def make_root_owned_read_only(root: Path, writable_roots: tuple[Path, ...]) -> None:
    writable = {path.resolve() for path in writable_roots}
    for path in sorted(root.rglob("*"), key=lambda row: len(row.parts), reverse=True):
        resolved = path.resolve()
        if any(resolved == item or item in resolved.parents for item in writable):
            continue
        if path.is_symlink():
            raise RuntimeError(f"PVA jail must contain no symlinks: {path}")
        os.chown(path, 0, 0)
        path.chmod(0o555 if path.is_dir() or path.stat().st_mode & stat.S_IXUSR else 0o444)
    os.chown(root, 0, 0)
    root.chmod(0o755)


def build_author_jail(jail_root: Path) -> dict[str, Any]:
    """Build one root-owned public runtime. No credentials are read or written."""

    marker = jail_root / "runtime_manifest.json"
    if marker.is_file():
        value = read_object(marker)
        if value.get("schema_version") == PVA_ISOLATION_SCHEMA:
            return value
        raise RuntimeError(f"refusing unrecognized existing PVA jail: {jail_root}")
    if jail_root.exists() and any(jail_root.iterdir()):
        raise RuntimeError(f"refusing nonempty unrecognized PVA jail: {jail_root}")
    jail_root.mkdir(parents=True, mode=0o755, exist_ok=True)
    for directory in (
        "bin",
        "usr/bin",
        "usr/lib",
        "lib",
        "lib64",
        "opt/venv/lib/python3.12/site-packages",
        "public",
        "etc/ssl/certs",
        "evaluator/runtime",
        "home/author/.codex",
        "tmp",
        "workspaces",
    ):
        (jail_root / directory).mkdir(parents=True, exist_ok=True)

    copy_runtime_file(STATIC_CODEX, jail_root)
    codex_in_jail = jail_root / "bin/codex"
    shutil.copy2(STATIC_CODEX, codex_in_jail)
    rg_binary = CODEX_VENDOR_ROOT / "codex-path/rg"
    if rg_binary.is_file():
        shutil.copy2(rg_binary, jail_root / "bin/rg")
    for source in JAIL_TOOLS:
        copy_runtime_file(source, jail_root)
    shell = jail_root / "bin/sh"
    if not shell.exists():
        shutil.copy2(Path("/bin/dash"), shell)

    copy_public_tree(Path("/usr/lib/python3.12"), jail_root / "usr/lib/python3.12")
    for source in site_package_sources():
        copy_public_tree(source, jail_root / JAIL_SITE_PACKAGES.relative_to("/") / source.name)
    for relative in JAIL_PUBLIC_PATHS:
        copy_public_tree(TEMPLATE_ROOT / relative, jail_root / "public" / relative)
    copy_public_tree(PVA_EVALUATOR, jail_root / JAIL_EVALUATOR.relative_to("/"))
    for relative in JAIL_EVALUATOR_RUNTIME_PATHS:
        copy_public_tree(
            TEMPLATE_ROOT / relative,
            jail_root / JAIL_EVALUATOR_RUNTIME.relative_to("/") / relative,
        )
    cert_bundle = Path("/etc/ssl/certs/ca-certificates.crt")
    if cert_bundle.is_file():
        copy_runtime_file(cert_bundle, jail_root)

    executable_roots = [STATIC_CODEX, *JAIL_TOOLS]
    executable_roots.extend(
        path
        for source in site_package_sources()
        for path in ([source] if source.is_file() else source.rglob("*"))
        if path.is_file() and path.name.endswith((".so", ".so.1", ".so.2", ".so.3"))
    )
    for dependency in elf_dependencies(executable_roots):
        copy_runtime_file(dependency, jail_root)

    for protected in (
        jail_root / "tmp",
        jail_root / "home/author",
        jail_root / "home/author/.codex",
    ):
        os.chown(protected, 0, 0)
        protected.chmod(0o555)
    os.chown(jail_root / "workspaces", 0, 0)
    (jail_root / "workspaces").chmod(0o711)
    marker_value = {
        "schema_version": PVA_ISOLATION_SCHEMA,
        "created_at_utc": utc_now(),
        "credentials_copied_or_written": False,
        "proc_present": (jail_root / "proc").exists(),
        "host_bind_mounts": False,
        "static_codex_sha256": sha256_file(STATIC_CODEX),
        "public_sdk_tree": git_output(TEMPLATE_ROOT, "rev-parse", "HEAD:sdk"),
        "public_docs_sha256": {
            name: sha256_file(TEMPLATE_ROOT / name)
            for name in JAIL_PUBLIC_PATHS
            if (TEMPLATE_ROOT / name).is_file()
        },
        "evaluator_sha256": sha256_file(PVA_EVALUATOR),
        "evaluator_runtime_sha256": {
            name: sha256_file(TEMPLATE_ROOT / name)
            for name in JAIL_EVALUATOR_RUNTIME_PATHS
        },
    }
    dump_json(marker, marker_value)
    make_root_owned_read_only(
        jail_root,
        (jail_root / "workspaces",),
    )
    return marker_value


@contextmanager
def exclusive_file_lock(lock_path: Path):
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+", encoding="utf-8") as handle:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"PVA author jail is already in use: {lock_path}") from exc
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def prepare_job_workspace(jail_root: Path, run_id: str) -> tuple[Path, Path]:
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", run_id):
        raise ValueError(f"unsafe run id: {run_id!r}")
    workspaces = jail_root / "workspaces"
    leftovers = list(workspaces.iterdir())
    if leftovers:
        raise RuntimeError(f"PVA jail workspace was not empty at job start: {leftovers[:3]}")
    job_root = workspaces / run_id
    job_root.mkdir(mode=0o755)
    os.chown(job_root, 0, 0)
    for relative in ("home", "home/.codex", "output", "tmp"):
        directory = job_root / relative
        directory.mkdir(mode=0o700)
        os.chown(directory, PVA_ISOLATION_UID, PVA_ISOLATION_GID)
    probe = job_root / "probe"
    probe.mkdir(mode=0o700)
    os.chown(probe, PVA_EVALUATOR_UID, PVA_EVALUATOR_GID)
    return job_root, Path("/workspaces") / run_id


def cleanup_job_workspace(job_root: Path) -> None:
    shutil.rmtree(job_root)
    if job_root.exists():
        raise RuntimeError(f"PVA jail job cleanup was incomplete: {job_root}")


def run_command_in_jail(
    jail_root: Path,
    command: list[str],
    *,
    timeout: float,
    process_env: dict[str, str],
    uid: int,
    gid: int,
    allowed_env: frozenset[str],
) -> subprocess.CompletedProcess[str]:
    """Run one command with an exact environment and no host mounts."""

    root = jail_root.resolve()
    if not root.is_dir():
        raise FileNotFoundError(root)
    if not command or any(not isinstance(item, str) or "\x00" in item for item in command):
        raise ValueError("author command must be a nonempty NUL-free string list")
    if not set(process_env) <= allowed_env:
        raise ValueError(
            f"unsupported isolated environment keys: {sorted(set(process_env) - allowed_env)}"
        )
    if any("\x00" in key or "\x00" in value for key, value in process_env.items()):
        raise ValueError("isolated environment must be NUL-free")
    if uid == 0 or gid == 0:
        raise ValueError("isolated commands must run with non-root uid and gid")
    chroot_command = [
        str(CHROOT),
        f"--groups={gid}",
        f"--userspec={uid}:{gid}",
        str(root),
        *command,
    ]
    process = subprocess.Popen(
        chroot_command,
        env=dict(process_env),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            stdout, stderr = process.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            stdout, stderr = process.communicate()
        exc.stdout = stdout
        exc.stderr = stderr
        raise
    return subprocess.CompletedProcess(
        chroot_command,
        process.returncode,
        stdout,
        stderr,
    )


def run_author_command_in_jail(
    jail_root: Path,
    command: list[str],
    *,
    timeout: float,
    author_env: dict[str, str],
) -> subprocess.CompletedProcess[str]:
    return run_command_in_jail(
        jail_root,
        command,
        timeout=timeout,
        process_env=author_env,
        uid=PVA_ISOLATION_UID,
        gid=PVA_ISOLATION_GID,
        allowed_env=frozenset(
            {
                "HOME",
                "LANG",
                "LC_ALL",
                "OPENAI_API_KEY",
                "PATH",
                "PYTHONDONTWRITEBYTECODE",
                "PYTHONPATH",
                "SSL_CERT_FILE",
                "TMPDIR",
            }
        ),
    )


def run_evaluator_command_in_jail(
    jail_root: Path,
    command: list[str],
    *,
    timeout: float,
    evaluator_env: dict[str, str],
) -> subprocess.CompletedProcess[str]:
    if PVA_PROVIDER_ENV_KEY in evaluator_env:
        raise ValueError("provider credentials are forbidden in the evaluator environment")
    return run_command_in_jail(
        jail_root,
        command,
        timeout=timeout,
        process_env=evaluator_env,
        uid=PVA_EVALUATOR_UID,
        gid=PVA_EVALUATOR_GID,
        allowed_env=frozenset(
            {
                "ARTICRAFT_MP_START_METHOD",
                "HOME",
                "LANG",
                "LC_ALL",
                "OPENBLAS_NUM_THREADS",
                "OMP_NUM_THREADS",
                "MKL_NUM_THREADS",
                "NUMEXPR_NUM_THREADS",
                "PATH",
                "PYTHONDONTWRITEBYTECODE",
                "PYTHONPATH",
                "TMPDIR",
                "URDF_COMPILE_TIMEOUT_SECONDS",
            }
        ),
    )


def require_exact_resume_bindings(
    result: dict[str, Any], expected: dict[str, str], result_path: Path
) -> None:
    if result.get("bindings") != expected:
        raise RuntimeError(f"refusing stale result with different bindings: {result_path}")


def repo_path(raw_path: Any) -> Path | None:
    if not isinstance(raw_path, str) or not raw_path.strip():
        return None
    candidate = (REPO_ROOT / raw_path).resolve()
    try:
        candidate.relative_to(REPO_ROOT.resolve())
    except ValueError:
        return None
    return candidate


def git_output(checkout: Path | None, *args: str) -> str | None:
    if checkout is None or not checkout.is_dir():
        return None
    try:
        completed = subprocess.run(
            ["git", *args],
            cwd=checkout,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return completed.stdout.strip() if completed.returncode == 0 else None


def selected_method(protocol: dict[str, Any]) -> dict[str, Any]:
    methods = protocol.get("methods")
    if isinstance(methods, dict):
        row = methods.get("pva")
        if isinstance(row, dict):
            return row
    if isinstance(methods, list):
        for row in methods:
            if isinstance(row, dict) and row.get("method_id") == "pva":
                return row
    raise ValueError("protocol has no pva method binding")


def model_binding(protocol: dict[str, Any], method: dict[str, Any]) -> dict[str, Any]:
    common = protocol.get("common_model_binding")
    if isinstance(common, dict):
        return common
    candidate = method.get("model_binding")
    if isinstance(candidate, dict):
        return candidate
    roles = method.get("roles")
    if isinstance(roles, list) and roles and isinstance(roles[0], dict):
        return roles[0]
    raise ValueError("protocol has no PV-A model binding")


def protocol_hash_checks(
    protocol: dict[str, Any], manifest_path: Path, manifest: dict[str, Any]
) -> dict[str, bool]:
    method = selected_method(protocol)
    implementation = method.get("implementation")
    implementation = implementation if isinstance(implementation, dict) else {}
    checkout = repo_path(implementation.get("checkout_path"))
    entrypoint = repo_path(implementation.get("entrypoint"))
    adapter = repo_path(method.get("adapter_entrypoint"))
    entrypoint_in_checkout = False
    if checkout is not None and entrypoint is not None:
        try:
            entrypoint.relative_to(checkout)
            entrypoint_in_checkout = True
        except ValueError:
            pass
    return {
        "protocol_frozen": protocol.get("frozen_before_first_run") is True
        or protocol.get("frozen_design") is True,
        "manifest_frozen": manifest.get("frozen") is True,
        "manifest_bound": protocol.get("manifest", {}).get("sha256") == sha256_file(manifest_path),
        "task_count_54": len(manifest.get("tasks", [])) == 54 == protocol.get("expected_task_count"),
        "repeat_ids_exact": manifest.get("repeat_ids") == ["r0", "r1", "r2"]
        and protocol.get("repeat_ids") == ["r0", "r1", "r2"],
        "repair_budget_3": protocol.get("max_common_repair_turns") == 3,
        "evaluator_bound": protocol.get("common_evaluator", {}).get("sha256")
        == sha256_file(COMMON_EVALUATOR),
        "package_schema_bound": protocol.get("package_schema", {}).get("sha256")
        == sha256_file(PACKAGE_SCHEMA),
        "result_schema_bound": protocol.get("result_schema", {}).get("sha256")
        == sha256_file(RESULT_SCHEMA),
        "implementation_checkout_bound": checkout == TEMPLATE_ROOT.resolve()
        and git_output(checkout, "rev-parse", "--show-toplevel") == str(checkout),
        "implementation_commit_bound": isinstance(implementation.get("commit"), str)
        and git_output(checkout, "rev-parse", "HEAD") == implementation.get("commit"),
        "implementation_tree_bound": isinstance(implementation.get("git_tree"), str)
        and git_output(checkout, "rev-parse", "HEAD^{tree}")
        == implementation.get("git_tree"),
        "implementation_tracked_clean": implementation.get("tracked_clean_at_freeze")
        is True
        and git_output(checkout, "status", "--porcelain", "--untracked-files=no") == "",
        "implementation_entrypoint_bound": entrypoint_in_checkout
        and entrypoint is not None
        and entrypoint.is_file()
        and not entrypoint.is_symlink(),
        "implementation_provenance_bound": isinstance(
            implementation.get("provenance"), str
        )
        and bool(implementation["provenance"].strip()),
        "adapter_path_bound": adapter == Path(__file__).resolve()
        and adapter.is_file()
        and not adapter.is_symlink(),
        "adapter_sha256_bound": isinstance(method.get("adapter_sha256"), str)
        and adapter is not None
        and adapter.is_file()
        and sha256_file(adapter) == method.get("adapter_sha256"),
        "request_parameters_frozen": request_parameter_checks(method),
        "native_settings_frozen": native_settings_checks(protocol, method),
    }


def request_parameter_checks(method: dict[str, Any]) -> bool:
    request = method.get("request_parameters")
    if not isinstance(request, dict):
        return False
    reasoning = request.get("reasoning_effort")
    if not (
        isinstance(reasoning, dict)
        and reasoning.get("value") == "high"
        and reasoning.get("configured") is True
        and isinstance(reasoning.get("transport"), str)
        and bool(reasoning["transport"].strip())
    ):
        return False
    for key in ("temperature", "top_p", "max_output_tokens"):
        row = request.get(key)
        if not (
            isinstance(row, dict)
            and row.get("sent") is False
            and row.get("value") is None
            and isinstance(row.get("reason"), str)
            and bool(row["reason"].strip())
        ):
            return False
    return True


def native_settings_checks(protocol: dict[str, Any], method: dict[str, Any]) -> bool:
    native = method.get("native_settings")
    if not isinstance(native, dict):
        return False
    cli = native.get("codex_cli")
    provider = native.get("provider")
    timeouts = protocol.get("timeouts")
    return bool(
        native.get("adapter_schema") == PVA_ISOLATION_SCHEMA
        and isinstance(cli, dict)
        and cli.get("path") == str(STATIC_CODEX)
        and cli.get("version") == codex_cli_version()
        and cli.get("subcommand") == "exec"
        and cli.get("json_events") is True
        and cli.get("ephemeral") is True
        and cli.get("ignore_user_config") is True
        and cli.get("ignore_rules") is True
        and cli.get("external_isolation") == "per_job_chroot_nonroot_uid_65534"
        and cli.get("internal_sandbox") == "bypassed_inside_external_chroot"
        and cli.get("shell_environment_inherit") == "none"
        and isinstance(provider, dict)
        and provider.get("model_provider") == PVA_PROVIDER_ID
        and provider.get("base_url") == PVA_PROVIDER_BASE_URL
        and provider.get("env_key") == PVA_PROVIDER_ENV_KEY
        and provider.get("wire_api") == "responses"
        and provider.get("requires_openai_auth") is False
        and isinstance(timeouts, dict)
        and native.get("native_retry_limit_per_attempt")
        == timeouts.get("native_retry_limit_per_attempt")
        == 2
        and native.get("model_response_timeout_seconds")
        == timeouts.get("model_response_seconds")
        and native.get("common_repair_turns") == protocol.get("max_common_repair_turns") == 3
    )


def usage(events: Path) -> dict[str, int | None]:
    totals = {
        "input_tokens": 0,
        "cached_input_tokens": 0,
        "output_tokens": 0,
        "reasoning_output_tokens": 0,
    }
    observed = False
    if not events.is_file():
        return totals
    for line in events.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if row.get("type") != "turn.completed":
            continue
        reported = row.get("usage") or {}
        observed = True
        for key in totals:
            totals[key] += int(reported.get(key) or 0)
    return totals if observed else {key: None for key in totals}


def file_state(root: Path) -> dict[str, str]:
    rows: dict[str, str] = {}
    for path in root.rglob("*"):
        if path.is_file() and "traces" not in path.parts:
            rows[rel(path, root)] = sha256_file(path)
    return rows


def build_codex_command(
    jail_job_root: Path,
    prompt: str,
    *,
    model: str,
    reasoning_effort: str,
    native_retry_limit: int,
) -> list[str]:
    if not jail_job_root.is_absolute():
        raise ValueError("jail job root must be absolute")
    try:
        jail_job_root.relative_to("/workspaces")
    except ValueError as exc:
        raise ValueError("jail job root must be beneath /workspaces") from exc
    if native_retry_limit < 0:
        raise ValueError("native retry limit cannot be negative")
    provider_prefix = f"model_providers.{PVA_PROVIDER_ID}"
    return [
        "/bin/codex",
        "exec",
        "--json",
        "--ephemeral",
        "--ignore-user-config",
        "--ignore-rules",
        "--strict-config",
        "--model",
        model,
        "-c",
        f'model_provider="{PVA_PROVIDER_ID}"',
        "-c",
        f'model_reasoning_effort="{reasoning_effort}"',
        "-c",
        'shell_environment_policy.inherit="none"',
        "-c",
        (
            "shell_environment_policy.set={"
            f'HOME="{jail_job_root / "home"}",'
            'LANG="C.UTF-8",LC_ALL="C.UTF-8",PATH="/bin:/usr/bin",'
            'PYTHONDONTWRITEBYTECODE="1",PYTHONPATH="/public:/opt/venv/lib/python3.12/site-packages",'
            f'TMPDIR="{jail_job_root / "tmp"}"'
            "}"
        ),
        "-c",
        f'{provider_prefix}.name="OpenAI"',
        "-c",
        f'{provider_prefix}.base_url="{PVA_PROVIDER_BASE_URL}"',
        "-c",
        f'{provider_prefix}.env_key="{PVA_PROVIDER_ENV_KEY}"',
        "-c",
        f'{provider_prefix}.wire_api="responses"',
        "-c",
        f"{provider_prefix}.requires_openai_auth=false",
        "-c",
        f"{provider_prefix}.request_max_retries={native_retry_limit}",
        "-c",
        f"{provider_prefix}.stream_max_retries={native_retry_limit}",
        "--dangerously-bypass-approvals-and-sandbox",
        "--skip-git-repo-check",
        "-C",
        str(jail_job_root),
        prompt,
    ]


def author_prompt(task: dict[str, Any], output: Path, repeat_id: str) -> str:
    return f"""
You are the PV-A method adapter in a frozen reliability experiment. This is an
independent fresh session for repeat `{repeat_id}`. Implement the public task:

{task['prompt']}

Write only `{output}`. Do not search for or read benchmark protocol files,
hidden/evaluator specifications, other runs, existing templates, source maps,
TemplateDesign files, baseline outputs, or network sources. Task-specific
evidence is limited to the prompt above. You may read these common public APIs:

- `/public/articraft_template_authoring/AUTHORING.md`
- `/public/articraft_template_authoring/MECHANICAL_PRIORS.md`
- `/public/sdk`

Create one self-contained procedural Python module using only the public `sdk`.
It must export exactly one `build_*`, exactly one `run_*_tests`,
`config_from_seed`, `TEMPLATE_DOMAIN`, `TEMPLATE_CORNERS`, and a precise
`__all__`. Seed 0 is the neutral canonical object for common Table 1 evaluation;
other seeds must remain deterministic and structurally plausible. Use mesh-backed
named semantic parts, a connected hierarchy, physically meaningful joints in the
stated coordinate frame, finite limits where applicable, and author tests. Do not
import another template or any generated artifact. Run only focused smoke checks
with `PYTHONDONTWRITEBYTECODE=1` and finish with a concise status message.
""".strip()


def repair_prompt(output: Path, feedback: Path, repeat_id: str, attempt: int) -> str:
    return f"""
This is common repair turn {attempt}/3 for independent repeat `{repeat_id}`.
Read only the existing source `{output}`, normalized common-evaluator feedback
`{feedback}`, and these same public SDK/documents allowed in the initial packet:
`/public/sdk`, `/public/articraft_template_authoring/AUTHORING.md`, and
`/public/articraft_template_authoring/MECHANICAL_PRIORS.md`.
Do not search for or read benchmark protocol files, hidden/evaluator
specifications, other runs, templates, source maps, baseline outputs, or network
sources. Edit only `{output}`. Fix the general source-level cause without
hard-coding evaluator text, task IDs, or special-casing seed 0. Preserve passing
behavior, run focused smoke checks with `PYTHONDONTWRITEBYTECODE=1`, and finish
with a concise status message.
""".strip()


def codex_turn(
    prompt: str,
    run_root: Path,
    output: Path,
    trace_prefix: Path,
    model: str,
    reasoning_effort: str,
    timeout: float,
    *,
    jail_root: Path,
    jail_job_root: Path,
    provider_api_key: str,
    native_retry_limit: int,
) -> dict[str, Any]:
    run_root.mkdir(parents=True, exist_ok=True)
    before = file_state(run_root)
    started = utc_now()
    start_wall = time.monotonic()
    jail_host_job = ensure_beneath(
        jail_root / jail_job_root.relative_to("/"), jail_root / "workspaces"
    )
    if not jail_host_job.is_dir():
        raise FileNotFoundError(jail_host_job)
    command = build_codex_command(
        jail_job_root,
        prompt,
        model=model,
        reasoning_effort=reasoning_effort,
        native_retry_limit=native_retry_limit,
    )
    author_env = {
        "HOME": str(jail_job_root / "home"),
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "OPENAI_API_KEY": provider_api_key,
        "PATH": JAIL_PATH,
        "PYTHONDONTWRITEBYTECODE": "1",
        "PYTHONPATH": f"{JAIL_PUBLIC}:{JAIL_SITE_PACKAGES}",
        "SSL_CERT_FILE": "/etc/ssl/certs/ca-certificates.crt",
        "TMPDIR": str(jail_job_root / "tmp"),
    }
    timed_out = False
    try:
        completed = run_author_command_in_jail(
            jail_root,
            command,
            timeout=timeout,
            author_env=author_env,
        )
        exit_code = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        exit_code = 124
        stdout = exc.stdout if isinstance(exc.stdout, str) else ""
        stderr = exc.stderr if isinstance(exc.stderr, str) else ""
    isolated_template = jail_host_job / "output/template.py"
    if isolated_template.exists() or isolated_template.is_symlink():
        metadata = isolated_template.lstat()
        if not stat.S_ISREG(metadata.st_mode) or isolated_template.is_symlink():
            raise RuntimeError("refusing non-regular or symlinked author template output")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(isolated_template.read_bytes())
        output.chmod(0o644)
    trace_prefix.parent.mkdir(parents=True, exist_ok=True)
    events = trace_prefix.with_suffix(".events.jsonl")
    stderr_path = trace_prefix.with_suffix(".stderr.txt")
    events.write_text(stdout, encoding="utf-8")
    stderr_path.write_text(stderr, encoding="utf-8")
    after = file_state(run_root)
    permitted = rel(output, run_root)
    scope_changes = sorted(
        name for name in set(before) | set(after) if before.get(name) != after.get(name) and name != permitted
    )
    return {
        "started_at_utc": started,
        "finished_at_utc": utc_now(),
        "wall_time_s": time.monotonic() - start_wall,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "events": rel(events, run_root),
        "events_sha256": sha256_file(events),
        "stderr": rel(stderr_path, run_root),
        "stderr_sha256": sha256_file(stderr_path),
        "usage": usage(events),
        "provider_request_id_hash": None,
        "scope_changes_outside_template": scope_changes,
    }


def trace_violations(events: Path, forbidden_tokens: list[str]) -> list[dict[str, str]]:
    findings: list[dict[str, str]] = []
    for line_number, line in enumerate(
        events.read_text(encoding="utf-8", errors="replace").splitlines(), start=1
    ):
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        item = event.get("item") or {}
        if item.get("type") == "command_execution":
            auditable = str(item.get("command") or "")
        elif item.get("type") == "file_change":
            auditable = " ".join(
                str(row.get("path") or "")
                for row in item.get("changes", [])
                if isinstance(row, dict)
            )
        else:
            continue
        lowered = auditable.lower()
        for token in forbidden_tokens:
            if token.lower() in lowered:
                findings.append(
                    {
                        "line": str(line_number),
                        "forbidden": token,
                        "event_excerpt": auditable[:1000],
                    }
                )
    return findings


def run_pva_evaluation(template: Path, out: Path, timeout: float) -> dict[str, Any]:
    env = os.environ.copy()
    env.update(THREAD_ENV)
    started = utc_now()
    start_wall = time.monotonic()
    command = [
        "python",
        str(PVA_EVALUATOR),
        "--template",
        str(template),
        "--out",
        str(out),
        "--seeds",
        "0-0",
        "--workers",
        "1",
        "--timeout",
        str(int(timeout)),
    ]
    timed_out = False
    try:
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout + 30.0,
            check=False,
        )
        exit_code = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        exit_code = 124
        stdout = exc.stdout if isinstance(exc.stdout, str) else ""
        stderr = exc.stderr if isinstance(exc.stderr, str) else ""
    out.mkdir(parents=True, exist_ok=True)
    (out / "adapter.stdout.txt").write_text(stdout, encoding="utf-8")
    (out / "adapter.stderr.txt").write_text(stderr, encoding="utf-8")
    summary_path = out / "summary.json"
    summary = read_object(summary_path) if summary_path.is_file() else {}
    record = next(iter(summary.get("records", [])), {})
    return {
        "started_at_utc": started,
        "finished_at_utc": utc_now(),
        "wall_time_s": time.monotonic() - start_wall,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "summary": summary,
        "record": record,
    }


def normalized_feedback(report: dict[str, Any], output: Path) -> None:
    feedback = dict(report.get("feedback") or {})
    feedback["common_qc_pass"] = report.get("verdicts", {}).get("common_qc_pass")
    dump_json(output, feedback)


def evaluate_attempt(
    run_root: Path,
    attempt_root: Path,
    template: Path,
    task_id: str,
    repeat_id: str,
    attempt_index: int,
    bindings: dict[str, str],
    timeout: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    snapshot = attempt_root / "template.py"
    snapshot.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(template, snapshot)
    pva_out = attempt_root / "pva_evaluation"
    probe = run_pva_evaluation(snapshot, pva_out, timeout)
    case_root = pva_out / "cases/seed_000"
    urdf = case_root / "model.urdf"
    package_path = attempt_root / "package.json"
    package = {
        "schema_version": "table1_authoring_package_v1",
        "run_id": run_root.name,
        "method_id": "pva",
        "task_id": task_id,
        "repeat_id": repeat_id,
        "attempt_index": attempt_index,
        "run_root": str(run_root),
        "bindings": {
            "protocol_sha256": bindings["protocol_sha256"],
            "manifest_sha256": bindings["manifest_sha256"],
            "hidden_specs_sha256": bindings["hidden_specs_sha256"],
            "common_evaluator_sha256": bindings["common_evaluator_sha256"],
            "package_schema_sha256": bindings["package_schema_sha256"],
        },
        "artifacts": {
            "source": {"path": rel(snapshot, run_root), "sha256": sha256_file(snapshot)},
            "urdf": {
                "path": rel(urdf, run_root),
                "sha256": sha256_file(urdf) if urdf.is_file() else "0" * 64,
            },
        },
        "execution_probe": {
            "started_at_utc": probe["started_at_utc"],
            "finished_at_utc": probe["finished_at_utc"],
            "wall_time_s": probe["wall_time_s"],
            "exit_code": probe["exit_code"],
            "timed_out": probe["timed_out"],
            "source_sha256": sha256_file(snapshot),
            "stdout_sha256": sha256_file(pva_out / "adapter.stdout.txt"),
            "stderr_sha256": sha256_file(pva_out / "adapter.stderr.txt"),
        },
    }
    dump_json(package_path, package)
    report_path = attempt_root / "common_evaluator_report.json"
    completed = subprocess.run(
        [
            "python",
            str(COMMON_EVALUATOR),
            "--package-manifest",
            str(package_path),
            "--output",
            str(report_path),
        ],
        cwd=REPO_ROOT,
        env={**os.environ, **THREAD_ENV},
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )
    if not report_path.is_file():
        raise RuntimeError(
            f"common evaluator produced no report (exit {completed.returncode}): {completed.stderr[-4000:]}"
        )
    return read_object(report_path), package


def validate_result(result: dict[str, Any]) -> None:
    import jsonschema  # type: ignore[import-not-found]

    jsonschema.Draft202012Validator(read_object(RESULT_SCHEMA)).validate(result)


def execution_refusal_record(
    *,
    protocol_sha256: str,
    capability_blockers: list[str],
    environment_blockers: list[str],
) -> dict[str, Any]:
    return {
        "schema_version": "table1_pva_execution_refusal_v1",
        "generated_at_utc": utc_now(),
        "method_id": "pva",
        "status": "REFUSED_BEFORE_PROVIDER_CALL",
        "protocol_sha256": protocol_sha256,
        "capability_blockers": capability_blockers,
        "environment_blockers": environment_blockers,
        "authoring_attempts": 0,
        "provider_calls_made": 0,
        "api_or_network_accessed": False,
    }


def build_execution_summary(
    results: list[dict[str, Any]], intended_runs: int
) -> dict[str, Any]:
    if intended_runs < len(results):
        raise ValueError("intended run denominator cannot be smaller than attempted results")
    evaluable = [
        row for row in results if row.get("summary", {}).get("state") == "observed"
    ]
    return {
        "schema_version": 1,
        "method_id": "pva",
        "generated_at_utc": utc_now(),
        "intended_runs": intended_runs,
        "strict_denominator": intended_runs,
        "attempted_results": len(results),
        "completed_results": len(results),
        "evaluable_results": len(evaluable),
        "observed_denominator": len(evaluable),
        "metrics": {
            key: {
                "numerator": sum(
                    row.get("summary", {}).get(key) is True for row in evaluable
                ),
                "denominator": intended_runs,
            }
            for key in ("executable", "artifact_saved", "first_shot", "final_success")
        },
        "repair_turns_total": sum(
            int(row["summary"]["repair_turns"])
            for row in evaluable
            if row["summary"].get("repair_turns") is not None
        ),
        "claim_boundary": (
            "Headline success metrics use all frozen intent runs as the denominator; "
            "attempted_results and evaluable_results are reported separately."
        ),
    }


def execute_run(
    task: dict[str, Any],
    repeat_id: str,
    out: Path,
    protocol: dict[str, Any],
    bindings: dict[str, str],
    timeout_model: float,
    timeout_eval: float,
    *,
    jail_root: Path,
    provider_api_key: str,
    author_func: Callable[..., dict[str, Any]] = codex_turn,
    evaluator_func: Callable[..., tuple[dict[str, Any], dict[str, Any]]] = evaluate_attempt,
) -> dict[str, Any]:
    run_id = f"pva__{task['task_id']}__{repeat_id}"
    run_root = out / "runs" / run_id
    result_path = run_root / "result.json"
    if result_path.is_file():
        existing = read_object(result_path)
        validate_result(existing)
        expected_resume_bindings = {
            **bindings,
            "adapter_sha256": sha256_file(Path(__file__).resolve()),
        }
        require_exact_resume_bindings(existing, expected_resume_bindings, result_path)
        if (
            existing.get("run_id") != run_id
            or existing.get("method_id") != "pva"
            or existing.get("task_id") != task["task_id"]
            or existing.get("repeat_id") != repeat_id
        ):
            raise RuntimeError(f"refusing stale result with different job identity: {result_path}")
        return existing
    if run_root.exists():
        raise RuntimeError(f"refusing preexisting incomplete PVA run root: {run_root}")
    run_root.mkdir(parents=True)
    template = run_root / "output/template.py"
    template.parent.mkdir(parents=True, exist_ok=True)
    jail_host_job, jail_job_root = prepare_job_workspace(jail_root, run_id)
    jail_template = jail_job_root / "output/template.py"
    method = selected_method(protocol)
    model = model_binding(protocol, method)
    model_id = str(
        model.get("model_id") or model.get("model") or model.get("exact_model_id") or ""
    )
    effort = str(model.get("reasoning_effort") or "high")
    forbidden = [
        "table1_reliability_hidden_specs_v1",
        "table1_reliability_protocol_v1",
        "agent/templates",
        "source_maps",
        "templatedesign",
        "t2_formal_v1/authoring/runs",
        "exp/baselines",
    ]
    packet = {
        "schema_version": 1,
        "run_id": run_id,
        "task_id": task["task_id"],
        "repeat_id": repeat_id,
        "prompt": task["prompt"],
        "prompt_sha256": task["prompt_sha256"],
        "method_id": "pva",
        "model_id": model_id,
        "reasoning_effort": effort,
        "max_common_repair_turns": 3,
        "bindings": {
            key: value
            for key, value in bindings.items()
            if key not in {"hidden_specs_path", "protocol_path"}
        },
        "hidden_material_exposed": False,
        "forbidden_trace_tokens": forbidden,
    }
    dump_json(run_root / "author_packet.json", packet)
    started_at = utc_now()
    attempts: list[dict[str, Any]] = []
    policy_violations: list[dict[str, str]] = []
    try:
        for attempt_index in range(4):
            if attempt_index == 0:
                prompt = author_prompt(task, jail_template, repeat_id)
            else:
                feedback_source = (
                    run_root / f"attempts/a{attempt_index - 1}/repair_feedback.json"
                )
                feedback_dir = jail_host_job / "input"
                feedback_dir.mkdir(mode=0o755, exist_ok=True)
                os.chown(feedback_dir, 0, 0)
                feedback_target = feedback_dir / f"repair_feedback_a{attempt_index - 1}.json"
                feedback_target.write_bytes(feedback_source.read_bytes())
                os.chown(feedback_target, 0, 0)
                feedback_target.chmod(0o444)
                prompt = repair_prompt(
                    jail_template,
                    jail_job_root / "input" / feedback_target.name,
                    repeat_id,
                    attempt_index,
                )
            turn = author_func(
                prompt,
                run_root,
                template,
                run_root / f"traces/attempt_{attempt_index}",
                model_id,
                effort,
                timeout_model,
                jail_root=jail_root,
                jail_job_root=jail_job_root,
                provider_api_key=provider_api_key,
                native_retry_limit=int(
                    protocol["timeouts"]["native_retry_limit_per_attempt"]
                ),
            )
            event_path = run_root / turn["events"]
            trace_findings = trace_violations(event_path, forbidden)
            policy_violations.extend(trace_findings)
            policy_violations.extend(
                {
                    "line": "scope",
                    "forbidden": path,
                    "event_excerpt": "file modified outside output/template.py",
                }
                for path in turn["scope_changes_outside_template"]
            )
            attempt_root = run_root / f"attempts/a{attempt_index}"
            package: dict[str, Any] = {}
            if template.is_file():
                report, package = evaluator_func(
                    run_root,
                    attempt_root,
                    template,
                    task["task_id"],
                    repeat_id,
                    attempt_index,
                    bindings,
                    timeout_eval,
                )
                verdicts = report.get("verdicts", {})
                normalized_feedback(report, attempt_root / "repair_feedback.json")
                artifact_path = Path(package["artifacts"]["urdf"]["path"])
                artifact_hash = package["artifacts"]["urdf"]["sha256"]
                evaluation = {
                    "state": "observed",
                    "executable": bool(verdicts.get("executable")),
                    "artifact_saved": bool(verdicts.get("artifact_saved")),
                    "common_qc_pass": bool(verdicts.get("common_qc_pass")),
                    "urdf_tree_pass": bool(verdicts.get("urdf_tree_pass")),
                    "semantic_roles_pass": bool(verdicts.get("semantic_roles_pass")),
                    "joint_spec_pass": bool(verdicts.get("joint_spec_pass")),
                    "input_bindings_pass": all(
                        report.get("binding_checks", {}).values()
                    )
                    and all(report.get("protocol_checks", {}).values())
                    and all(report.get("task_checks", {}).values()),
                    "common_evaluator_report_path": rel(
                        attempt_root / "common_evaluator_report.json", run_root
                    ),
                    "common_evaluator_report_sha256": sha256_file(
                        attempt_root / "common_evaluator_report.json"
                    ),
                    "reason": None,
                }
                output_record = {
                    "template_path": rel(attempt_root / "template.py", run_root),
                    "template_sha256": sha256_file(attempt_root / "template.py"),
                    "artifact_path": str(artifact_path),
                    "artifact_sha256": artifact_hash,
                }
            else:
                dump_json(
                    attempt_root / "repair_feedback.json",
                    {
                        "schema_version": 1,
                        "task_id": task["task_id"],
                        "attempt_index": attempt_index,
                        "common_qc_pass": False,
                        "failure_codes": ["SOURCE_NOT_CREATED"],
                        "policy": "normalized evaluator feedback only; hidden material withheld",
                    },
                )
                evaluation = {
                    "state": "observed",
                    "executable": False,
                    "artifact_saved": False,
                    "common_qc_pass": False,
                    "urdf_tree_pass": False,
                    "semantic_roles_pass": False,
                    "joint_spec_pass": False,
                    "input_bindings_pass": None,
                    "common_evaluator_report_path": None,
                    "common_evaluator_report_sha256": None,
                    "reason": "authoring model did not create output/template.py",
                }
                output_record = {
                    "template_path": None,
                    "template_sha256": None,
                    "artifact_path": None,
                    "artifact_sha256": None,
                }
            repair_feedback_hash = (
                sha256_file(
                    run_root
                    / f"attempts/a{attempt_index - 1}/repair_feedback.json"
                )
                if attempt_index > 0
                else None
            )
            attempts.append(
                {
                "attempt_index": attempt_index,
                "attempt_kind": "attempt_0" if attempt_index == 0 else "common_repair",
                "method_id": "pva",
                "task_id": task["task_id"],
                "repeat_id": repeat_id,
                "native_retry_count": 0,
                "native_retry_index": 0,
                "request_started_at_utc": turn["started_at_utc"],
                "response_completed_at_utc": turn["finished_at_utc"],
                "execution_started_at_utc": (
                    package.get("execution_probe", {}).get("started_at_utc")
                    if template.is_file()
                    else None
                ),
                "execution_completed_at_utc": (
                    package.get("execution_probe", {}).get("finished_at_utc")
                    if template.is_file()
                    else None
                ),
                "failure_class": (
                    None
                    if evaluation["common_qc_pass"]
                    else (
                        "authoring_output_missing"
                        if not template.is_file()
                        else "common_evaluator_rejection"
                    )
                ),
                "model_response_sha256": turn["events_sha256"],
                "repair_feedback_sha256": repair_feedback_hash,
                "output": output_record,
                "evaluation": evaluation,
                "telemetry": {
                    "wall_time_seconds": turn["wall_time_s"],
                    "input_tokens": turn["usage"]["input_tokens"],
                    "output_tokens": turn["usage"]["output_tokens"],
                    "provider_request_id_hash": turn["provider_request_id_hash"],
                    "api_cost_usd": None,
                    "missing_reasons": {
                        "input_tokens": (
                            "Codex event stream did not report input token usage"
                            if turn["usage"]["input_tokens"] is None
                            else None
                        ),
                        "output_tokens": (
                            "Codex event stream did not report output token usage"
                            if turn["usage"]["output_tokens"] is None
                            else None
                        ),
                        "provider_request_id_hash": (
                            "Codex event stream does not expose a provider request identifier"
                        ),
                        "api_cost_usd": "Codex event stream does not report API cost",
                    },
                },
                }
            )
            if evaluation["common_qc_pass"] or policy_violations:
                break
    finally:
        cleanup_job_workspace(jail_host_job)
    first = attempts[0]["evaluation"]
    final = attempts[-1]["evaluation"]
    summary_state = "not_evaluable" if policy_violations else "observed"
    result = {
        "schema_version": "table1_authoring_result_v1",
        "protocol_id": protocol["protocol_id"],
        "bindings": {
            **bindings,
            "adapter_sha256": sha256_file(Path(__file__).resolve()),
        },
        "run_id": run_id,
        "method_id": "pva",
        "task_id": task["task_id"],
        "repeat_id": repeat_id,
        "status": "failed" if policy_violations else "completed",
        "started_at_utc": started_at,
        "finished_at_utc": utc_now(),
        "attempts": attempts,
        "summary": {
            "state": summary_state,
            "executable": final["executable"] if not policy_violations else None,
            "artifact_saved": final["artifact_saved"] if not policy_violations else None,
            "first_shot": first["common_qc_pass"] if not policy_violations else None,
            "final_success": final["common_qc_pass"] if not policy_violations else None,
            "repair_turns": len(attempts) - 1 if not policy_violations else None,
            "reason": (
                None
                if not policy_violations
                else f"author trace/output policy violations: {len(policy_violations)}"
            ),
        },
        "error": (
            None
            if not policy_violations
            else {"code": "AUTHOR_POLICY_VIOLATION", "message": json.dumps(policy_violations)[:4000]}
        ),
    }
    validate_result(result)
    dump_json(result_path, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--task-ids", default="", help="comma-separated task IDs")
    parser.add_argument("--repeat-ids", default="", help="comma-separated repeat IDs")
    parser.add_argument("--all-tasks", action="store_true")
    parser.add_argument("--all-repeats", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--acknowledge-paid-run", action="store_true")
    parser.add_argument("--model-timeout", type=float, default=1800.0)
    parser.add_argument("--evaluator-timeout", type=float, default=180.0)
    args = parser.parse_args()

    manifest_path = args.manifest.resolve()
    protocol_path = args.protocol.resolve()
    out = args.out.resolve()
    out.relative_to((EXP_ROOT / "runtime/table1_reliability").resolve())
    out.mkdir(parents=True, exist_ok=True)
    manifest = read_object(manifest_path)
    protocol = read_object(protocol_path)
    checks = protocol_hash_checks(protocol, manifest_path, manifest)
    if not all(checks.values()):
        raise RuntimeError(f"frozen input validation failed: {[key for key, value in checks.items() if not value]}")
    if not CODEX.is_file():
        raise FileNotFoundError(f"codex executable missing: {CODEX}")
    frozen_timeouts = protocol.get("timeouts", {})
    if args.model_timeout != float(frozen_timeouts.get("model_response_seconds", -1)):
        raise ValueError("--model-timeout must equal the frozen protocol value")
    if args.evaluator_timeout != float(
        frozen_timeouts.get("common_evaluator_seconds_per_attempt", -1)
    ) or args.evaluator_timeout != float(
        frozen_timeouts.get("execution_seconds_per_attempt", -1)
    ):
        raise ValueError("--evaluator-timeout must equal both frozen execution/evaluator values")

    all_tasks = manifest["tasks"]
    task_ids = [item for item in args.task_ids.split(",") if item]
    repeat_ids = [item for item in args.repeat_ids.split(",") if item]
    if args.all_tasks:
        tasks = all_tasks
    else:
        by_id = {row["task_id"]: row for row in all_tasks}
        unknown = sorted(set(task_ids) - set(by_id))
        if unknown:
            raise ValueError(f"unknown task IDs: {unknown}")
        tasks = [by_id[item] for item in task_ids]
    repeats = manifest["repeat_ids"] if args.all_repeats else repeat_ids
    if not tasks or not repeats:
        raise ValueError("select tasks and repeats, or use --all-tasks --all-repeats")
    if not set(repeats) <= set(manifest["repeat_ids"]):
        raise ValueError(f"unknown repeat IDs: {sorted(set(repeats) - set(manifest['repeat_ids']))}")
    readiness = protocol.get("execution_readiness", {})
    method_ready = readiness.get("method_adapters_ready", {})
    bindings = {
        "protocol_sha256": sha256_file(protocol_path),
        "manifest_sha256": sha256_file(manifest_path),
        "hidden_specs_sha256": protocol["hidden_specs"]["sha256"],
        "result_schema_sha256": sha256_file(RESULT_SCHEMA),
        "common_evaluator_sha256": sha256_file(COMMON_EVALUATOR),
        "package_schema_sha256": sha256_file(PACKAGE_SCHEMA),
    }
    jobs = [(task, repeat_id) for task in tasks for repeat_id in repeats]
    jobs.sort(
        key=lambda row: hashlib.sha256(f"{row[0]['task_id']}:{row[1]}".encode()).hexdigest()
    )
    method_blockers = readiness.get("method_blockers", {})
    capability_blockers = (
        list(method_blockers.get("pva", []))
        if isinstance(method_blockers, dict)
        and isinstance(method_blockers.get("pva", []), list)
        else ["Frozen protocol has no valid PVA capability-blocker list."]
    )
    environment = pva_environment_readiness()
    environment_blockers = (
        []
        if environment["ready"]
        else ["The selected OpenAI provider credential is absent from the execution environment."]
    )
    experiment_manifest = {
        "schema_version": 1,
        "experiment_id": "nano3d_table1_pva_authoring_v1",
        "created_at_utc": utc_now(),
        "mode": "execute" if args.execute else "prepare_only",
        "method_id": "pva",
        "bindings": bindings,
        "selected_task_count": len(tasks),
        "selected_repeat_count": len(repeats),
        "intended_runs": len(jobs),
        "job_order": [f"pva__{task['task_id']}__{repeat_id}" for task, repeat_id in jobs],
        "provider_calls_made": 0,
        "authoring_attempts": 0,
        "status": "EXECUTING" if args.execute else "PREPARED_EXECUTION_BLOCKED",
        "execution_readiness": readiness,
        "capability_blockers": capability_blockers,
        "environment_readiness": environment,
        "claim_boundary": (
            "Preparation alone is not experimental evidence. A result enters Table 1 only after "
            "schema validation and a hash-bound common-evaluator report."
        ),
    }
    dump_json(out / "experiment_manifest.json", experiment_manifest)
    if not args.execute:
        print(
            json.dumps(
                {
                    "status": experiment_manifest["status"],
                    "runs": len(jobs),
                    "provider_calls_made": 0,
                    "out": str(out),
                }
            )
        )
        return 0
    if not args.acknowledge_paid_run:
        raise ValueError("--execute requires --acknowledge-paid-run")
    ready = bool(
        protocol.get("execution_ready") is True
        and isinstance(readiness, dict)
        and readiness.get("status") == "READY"
        and isinstance(method_ready, dict)
        and method_ready.get("pva") is True
        and not capability_blockers
        and environment["ready"]
    )
    if not ready:
        refusal = execution_refusal_record(
            protocol_sha256=bindings["protocol_sha256"],
            capability_blockers=capability_blockers,
            environment_blockers=environment_blockers,
        )
        experiment_manifest["status"] = "EXECUTION_REFUSED_FAIL_CLOSED"
        dump_json(out / "experiment_manifest.json", experiment_manifest)
        dump_json(out / "execution_refusal.json", refusal)
        print(json.dumps(refusal, sort_keys=True))
        return 2

    provider_api_key = os.environ[PVA_PROVIDER_ENV_KEY]
    jail_parent = (EXP_ROOT / "runtime/table1_reliability").resolve()
    jail_root = jail_parent / JAIL_RUNTIME_DIRNAME
    lock_path = jail_parent / JAIL_LOCK_NAME
    with exclusive_file_lock(lock_path):
        build_author_jail(jail_root)
        results = [
            execute_run(
                task,
                repeat_id,
                out,
                protocol,
                bindings,
                args.model_timeout,
                args.evaluator_timeout,
                jail_root=jail_root,
                provider_api_key=provider_api_key,
            )
            for task, repeat_id in jobs
        ]
    summary = build_execution_summary(results, len(jobs))
    summary["protocol_id"] = protocol["protocol_id"]
    dump_json(out / "summary.json", summary)
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
