#!/usr/bin/env python3
"""Focused protocol-native-contract checks for the Table 1 common preflight."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
import unittest
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
PREFLIGHT_PATH = REPO_ROOT / "exp/scripts/preflight_table1_authoring_common.py"

SPEC = importlib.util.spec_from_file_location("table1_common_preflight", PREFLIGHT_PATH)
assert SPEC is not None and SPEC.loader is not None
PREFLIGHT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PREFLIGHT
SPEC.loader.exec_module(PREFLIGHT)


def input_metadata() -> dict[str, dict[str, Any]]:
    paths = {
        "manifest": PREFLIGHT.DEFAULT_MANIFEST,
        "hidden_specs": PREFLIGHT.DEFAULT_HIDDEN_SPECS,
        "common_evaluator": PREFLIGHT.DEFAULT_EVALUATOR,
        "package_schema": PREFLIGHT.DEFAULT_PACKAGE_SCHEMA,
        "result_schema": PREFLIGHT.DEFAULT_RESULT_SCHEMA,
    }
    return {name: PREFLIGHT.load_json_object(path)[1] for name, path in paths.items()}


def exact_protocol() -> dict[str, Any]:
    protocol = json.loads(PREFLIGHT.DEFAULT_PROTOCOL.read_text(encoding="utf-8"))
    protocol["common_model_binding"] = {
        "provider": "openai",
        "model": "gpt-5",
        "reasoning_effort": "high",
    }

    pva = protocol["methods"]["pva"]
    pva["request_parameters"] = {
        "reasoning_effort": {
            "value": "high",
            "configured": True,
            "transport": "cli_config",
        },
        "temperature": {
            "sent": False,
            "value": None,
            "reason": "Codex CLI exposes no temperature field for this adapter",
        },
        "top_p": {
            "sent": False,
            "value": None,
            "reason": "Codex CLI exposes no top_p field for this adapter",
        },
        "max_output_tokens": {
            "sent": False,
            "value": None,
            "reason": "Codex CLI exposes no output-token cap for this adapter",
        },
    }
    pva["native_settings"] = {
        "adapter_schema": "pva_chroot_read_isolation_v1",
        "codex_cli": {
            "path": "/mnt/zsn/miniconda3/lib/node_modules/@openai/codex/node_modules/@openai/codex-linux-x64/vendor/x86_64-unknown-linux-musl/bin/codex",
            "version": "codex-cli 0.147.0",
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
            "model_provider": "pva_openai_env",
            "base_url": "https://api.openai.com/v1",
            "env_key": "OPENAI_API_KEY",
            "wire_api": "responses",
            "requires_openai_auth": False,
        },
        "native_retry_limit_per_attempt": 2,
        "model_response_timeout_seconds": 1800,
        "common_repair_turns": 3,
    }

    lam = protocol["methods"]["lam"]
    lam["request_parameters"] = {
        "temperature": {
            "sent": False,
            "value": None,
            "reason": "LAM GPT-5 Responses handler omits temperature",
        },
        "top_p": {
            "sent": False,
            "value": None,
            "reason": "LAM GPT-5 Responses handler omits top_p",
        },
        "max_output_tokens": {"sent": True, "value": 64000},
        "verbosity": {"sent": True, "value": "medium"},
    }
    lam["native_settings"] = {
        "python_executable": "exp/runtime/table1_reliability/lam_env_v1/.venv/bin/python",
        "expected_python_version": "Python 3.12.3",
        "harness_python_executable": "/mnt/zsn/miniconda3/bin/python",
        "expected_harness_python_version": "Python 3.13.2",
        "harness_packages": {"jsonschema": "4.26.0", "trimesh": "4.12.2"},
        "node_executable": "/usr/bin/node",
        "expected_node_version": "v18.19.1",
        "npm_executable": "/usr/bin/npm",
        "expected_npm_version": "9.2.0",
        "requirements_sha256": "dbb9014ef1e86f7a9d31fc00b05b41fb0797abd0ea1ad54e25e3f2a25750f01c",
        "package_lock_sha256": "9a15deac933f4ef067d6645890c534c13f06f4610629fda5faed1f7f059039f7",
        "all_roles_use_method_model": True,
        "num_executions": 1,
        "parallel_workers": 0,
        "base_agent_parse_max_attempts": 3,
        "shape_export_validation_max_retries": 3,
        "vlm_critic_enabled": True,
        "vlm_critic_max_iterations": 4,
        "articulation_feedback_enabled": True,
        "articulation_feedback_max_iterations": 3,
        "pointllm_critic_enabled": False,
        "feedback_fusion_enabled": False,
        "outer_native_retry_limit": 2,
        "config_source": "adapter_generated_per_attempt_no_credentials",
    }

    articraft = protocol["methods"]["articraft"]
    articraft["request_parameters"] = {
        "temperature": {
            "sent": False,
            "value": None,
            "reason": "Articraft Responses provider does not expose or send this parameter",
        },
        "top_p": {
            "sent": False,
            "value": None,
            "reason": "Articraft Responses provider does not expose or send this parameter",
        },
        "max_output_tokens": {
            "sent": False,
            "value": None,
            "reason": "Articraft Responses provider does not expose or send an output-token cap",
        },
        "parallel_tool_calls": {"sent": True, "value": True},
        "reasoning_summary": {"sent": True, "value": "auto"},
        "store": {"sent": True, "value": False},
    }
    articraft["native_settings"] = {
        "max_turns": 100,
        "openai_transport": "http",
        "sdk_package": "sdk",
        "max_cost_usd": None,
        "cost_policy": "formal_global_budget_only",
        "python_executable": "articraft_data/.venv/bin/python",
        "python_version": "3.12.3",
        "pyproject_sha256": "fd2cf4ddff0d8aaac5052bbfcadf09114cd70f3a1e9c9318936af22ef6c526be",
        "uv_lock_sha256": "b58b12834c30a894ce4d7fdf6ae41e0fc2947fb3a10ff6acf653344223b9a0fc",
    }
    return protocol


def native_check_results(protocol: dict[str, Any]) -> dict[str, bool]:
    checks: list[dict[str, Any]] = []
    PREFLIGHT.audit_protocol(
        protocol,
        {"error": None},
        input_metadata(),
        checks,
    )
    return {
        row["check_id"]: row["passed"]
        for row in checks
        if row["check_id"].endswith(".native_contract")
        or row["check_id"].endswith(".request_contract")
        or row["check_id"] == "protocol.common_model_binding"
    }


def readiness_check_results(protocol: dict[str, Any]) -> dict[str, bool]:
    checks: list[dict[str, Any]] = []
    PREFLIGHT.audit_protocol(
        protocol,
        {"error": None},
        input_metadata(),
        checks,
    )
    return {
        row["check_id"]: row["passed"]
        for row in checks
        if row["check_id"].startswith("protocol.execution_")
        or row["check_id"].startswith("protocol.method_blockers_")
        or row["check_id"].startswith("protocol.global_blockers_")
    }


class NativeContractChecksTest(unittest.TestCase):
    def test_execution_readiness_fields_are_semantically_consistent(self) -> None:
        exact = exact_protocol()
        exact["execution_ready"] = False
        exact["execution_readiness"] = {
            "status": "BLOCKED_ADAPTERS",
            "frozen_inputs_complete": True,
            "evaluator_bound": True,
            "package_schema_bound": True,
            "result_schema_bound": True,
            "method_adapters_ready": {
                "pva": False,
                "lam": False,
                "articraft": False,
            },
            "method_blockers": {
                "pva": ["pva capability blocker"],
                "lam": ["lam capability blocker"],
                "articraft": ["articraft capability blocker"],
            },
            "blockers": ["authoring adapters are blocked"],
        }
        expected = {
            "protocol.execution_readiness_contract": True,
            "protocol.execution_ready_matches_method_readiness": True,
            "protocol.execution_status_matches_execution_ready": True,
            "protocol.method_blockers_match_method_readiness": True,
            "protocol.global_blockers_match_execution_ready": True,
        }
        self.assertEqual(readiness_check_results(exact), expected)

        contradictory_global = copy.deepcopy(exact)
        contradictory_global["execution_ready"] = True
        self.assertFalse(
            readiness_check_results(contradictory_global)[
                "protocol.execution_ready_matches_method_readiness"
            ]
        )

        contradictory_status = copy.deepcopy(exact)
        contradictory_status["execution_readiness"]["status"] = "NOT_READY"
        self.assertFalse(
            readiness_check_results(contradictory_status)[
                "protocol.execution_status_matches_execution_ready"
            ]
        )

        contradictory_method_blockers = copy.deepcopy(exact)
        contradictory_method_blockers["execution_readiness"]["method_blockers"][
            "pva"
        ] = []
        self.assertFalse(
            readiness_check_results(contradictory_method_blockers)[
                "protocol.method_blockers_match_method_readiness"
            ]
        )

        contradictory_global_blockers = copy.deepcopy(exact)
        contradictory_global_blockers["execution_readiness"]["blockers"] = []
        self.assertFalse(
            readiness_check_results(contradictory_global_blockers)[
                "protocol.global_blockers_match_execution_ready"
            ]
        )

    def test_exact_contracts_pass_and_retry_or_turn_drift_is_method_local(self) -> None:
        exact = exact_protocol()
        expected = {
            "protocol.common_model_binding": True,
            "protocol.method.pva.native_contract": True,
            "protocol.method.pva.request_contract": True,
            "protocol.method.lam.native_contract": True,
            "protocol.method.lam.request_contract": True,
            "protocol.method.articraft.native_contract": True,
            "protocol.method.articraft.request_contract": True,
        }
        self.assertEqual(native_check_results(exact), expected)

        wrong_lam = copy.deepcopy(exact)
        wrong_lam["methods"]["lam"]["native_settings"][
            "shape_export_validation_max_retries"
        ] = 4
        self.assertEqual(
            native_check_results(wrong_lam),
            {**expected, "protocol.method.lam.native_contract": False},
        )

        wrong_articraft = copy.deepcopy(exact)
        wrong_articraft["methods"]["articraft"]["native_settings"]["max_turns"] = 99
        self.assertEqual(
            native_check_results(wrong_articraft),
            {**expected, "protocol.method.articraft.native_contract": False},
        )

        wrong_lam_request = copy.deepcopy(exact)
        wrong_lam_request["methods"]["lam"]["request_parameters"][
            "max_output_tokens"
        ]["value"] = 32000
        self.assertEqual(
            native_check_results(wrong_lam_request),
            {**expected, "protocol.method.lam.request_contract": False},
        )

        wrong_articraft_request = copy.deepcopy(exact)
        wrong_articraft_request["methods"]["articraft"]["request_parameters"][
            "store"
        ]["value"] = True
        self.assertEqual(
            native_check_results(wrong_articraft_request),
            {**expected, "protocol.method.articraft.request_contract": False},
        )

        false_common_sampling_claim = copy.deepcopy(exact)
        false_common_sampling_claim["common_model_binding"]["temperature"] = 1.0
        self.assertEqual(
            native_check_results(false_common_sampling_claim),
            {**expected, "protocol.common_model_binding": False},
        )


if __name__ == "__main__":
    unittest.main()
