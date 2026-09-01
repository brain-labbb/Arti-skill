#!/usr/bin/env python3
"""Application-layer capability boundary for pipeline ablation authoring.

The trusted controller materializes one author-visible packet before creating a
session.  The provider then receives exactly the schemas returned by
``get_tool_schemas`` and routes normalized function calls through
``dispatch_provider_call``.  This module deliberately has no filesystem,
subprocess, compilation, probing, or search primitive.

An authored artifact is valid only after ``require_submission`` succeeds.  Text
returned as an ordinary assistant message is not an accepted artifact.
"""

from __future__ import annotations

import copy
import hashlib
import json
import threading
from dataclasses import dataclass
from typing import Any, Mapping

SCHEMA_VERSION = "pipeline_ablation_authoring_isolation/v1"
READ_PACKET_TOOL = "read_authoring_packet"
SUBMIT_TEMPLATE_TOOL = "submit_template"
ALLOWED_TOOL_NAMES = frozenset({READ_PACKET_TOOL, SUBMIT_TEMPLATE_TOOL})
DEFAULT_MAX_PACKET_BYTES = 8 * 1024 * 1024
DEFAULT_MAX_TEMPLATE_BYTES = 2 * 1024 * 1024


class AuthoringIsolationError(RuntimeError):
    """Base error raised at the trusted-controller boundary."""


class MissingTemplateSubmission(AuthoringIsolationError):
    """Raised when a provider turn ends without an accepted template."""


@dataclass(frozen=True, slots=True)
class SubmittedTemplate:
    """The sole artifact accepted from an isolated authoring session."""

    text: str
    sha256: str
    size_bytes: int


@dataclass(frozen=True, slots=True)
class AuthoringToolResult:
    """Provider-neutral result for one normalized function call."""

    tool_call_id: str
    tool_name: str
    ok: bool
    output: Mapping[str, Any] | None = None
    error_code: str | None = None
    error_message: str | None = None

    def provider_payload(self) -> dict[str, Any]:
        if self.ok:
            return {"result": dict(self.output or {})}
        return {
            "error": {
                "code": self.error_code or "isolation_denied",
                "message": self.error_message or "Authoring tool call denied.",
            }
        }

    def provider_message(self) -> dict[str, str]:
        """Return the generic tool message consumed by current provider codecs."""

        return {
            "role": "tool",
            "tool_call_id": self.tool_call_id,
            "name": self.tool_name,
            "content": json.dumps(
                self.provider_payload(),
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            ),
        }


def _function_schema(
    name: str,
    description: str,
    properties: Mapping[str, Any],
    required: list[str],
) -> dict[str, Any]:
    return {
        "type": "function",
        "function": {
            "name": name,
            "description": description,
            "parameters": {
                "type": "object",
                "properties": dict(properties),
                "required": list(required),
                "additionalProperties": False,
            },
        },
    }


_TOOL_SCHEMAS = (
    _function_schema(
        READ_PACKET_TOOL,
        "Return the complete immutable authoring packet for this run.",
        {},
        [],
    ),
    _function_schema(
        SUBMIT_TEMPLATE_TOOL,
        "Submit the complete self-contained template text as the run's only artifact.",
        {
            "template": {
                "type": "string",
                "description": "Complete template source text, preserved byte-for-byte as UTF-8.",
            }
        },
        ["template"],
    ),
)


def _utf8_size(value: str, *, field: str) -> int:
    try:
        return len(value.encode("utf-8"))
    except UnicodeEncodeError as exc:
        raise ValueError(f"{field} must be valid UTF-8 text") from exc


class AuthoringIsolationSession:
    """Pure-memory, two-capability authoring session.

    ``packet_text`` must already contain every task input the author is allowed
    to observe.  No path loader is provided intentionally.  A successful
    submission closes the session atomically, including under parallel calls.
    """

    def __init__(
        self,
        packet_text: str,
        *,
        max_packet_bytes: int = DEFAULT_MAX_PACKET_BYTES,
        max_template_bytes: int = DEFAULT_MAX_TEMPLATE_BYTES,
    ) -> None:
        if type(packet_text) is not str:
            raise TypeError("packet_text must be a string materialized by the trusted controller")
        if not packet_text:
            raise ValueError("packet_text must not be empty")
        if type(max_packet_bytes) is not int or max_packet_bytes <= 0:
            raise ValueError("max_packet_bytes must be a positive integer")
        if type(max_template_bytes) is not int or max_template_bytes <= 0:
            raise ValueError("max_template_bytes must be a positive integer")

        packet_size = _utf8_size(packet_text, field="packet_text")
        if packet_size > max_packet_bytes:
            raise ValueError("packet_text exceeds the configured byte limit")

        self._packet_text = packet_text
        self._packet_sha256 = hashlib.sha256(packet_text.encode("utf-8")).hexdigest()
        self._packet_size_bytes = packet_size
        self._max_template_bytes = max_template_bytes
        self._max_argument_bytes = max_template_bytes * 8 + 4096
        self._lock = threading.Lock()
        self._submission: SubmittedTemplate | None = None

    def get_tool_schemas(self) -> list[dict[str, Any]]:
        """Return a defensive copy of the complete provider tool surface."""

        return copy.deepcopy(list(_TOOL_SCHEMAS))

    def get_all_tool_names(self) -> tuple[str, str]:
        return (READ_PACKET_TOOL, SUBMIT_TEMPLATE_TOOL)

    @property
    def packet_sha256(self) -> str:
        return self._packet_sha256

    @property
    def has_submission(self) -> bool:
        with self._lock:
            return self._submission is not None

    def dispatch_provider_call(self, tool_call: Mapping[str, Any]) -> AuthoringToolResult:
        """Dispatch a normalized provider function call without exposing host objects.

        Current Articraft codecs normalize OpenAI, Gemini, Anthropic, and
        OpenRouter calls to ``{id, type, function: {name, arguments}}``.  Only
        that function-call shape is accepted here.
        """

        if type(tool_call) is not dict:
            return self._denied("", "<invalid>", "invalid_call", "Invalid function call.")

        call_id = tool_call.get("id")
        if type(call_id) is not str or not call_id:
            call_id = "<missing>"
        if tool_call.get("type", "function") != "function":
            return self._denied(
                call_id,
                "<invalid>",
                "invalid_call",
                "Only declared function calls are accepted.",
            )

        function = tool_call.get("function")
        if type(function) is not dict:
            return self._denied(call_id, "<invalid>", "invalid_call", "Invalid function call.")
        return self.dispatch(
            function.get("name"),
            function.get("arguments", "{}"),
            tool_call_id=call_id,
        )

    def dispatch(
        self,
        tool_name: object,
        arguments: object,
        *,
        tool_call_id: str = "local",
    ) -> AuthoringToolResult:
        """Dispatch one decoded or JSON-encoded call against the allowlist."""

        if type(tool_name) is not str or tool_name not in ALLOWED_TOOL_NAMES:
            return self._denied(
                tool_call_id,
                "<denied>",
                "tool_not_allowed",
                "Tool is not in the isolated authoring allowlist.",
            )

        with self._lock:
            if self._submission is not None:
                return self._denied(
                    tool_call_id,
                    tool_name,
                    "session_closed",
                    "The authoring session closed after its template submission.",
                )

        try:
            params = self._decode_arguments(arguments)
        except ValueError as exc:
            return self._denied(
                tool_call_id,
                tool_name,
                "invalid_arguments",
                str(exc),
            )

        if tool_name == READ_PACKET_TOOL:
            if params:
                return self._denied(
                    tool_call_id,
                    tool_name,
                    "invalid_arguments",
                    "read_authoring_packet accepts no parameters.",
                )
            with self._lock:
                if self._submission is not None:
                    return self._denied(
                        tool_call_id,
                        tool_name,
                        "session_closed",
                        "The authoring session closed after its template submission.",
                    )
                return AuthoringToolResult(
                    tool_call_id=tool_call_id,
                    tool_name=tool_name,
                    ok=True,
                    output={
                        "schema_version": SCHEMA_VERSION,
                        "packet": self._packet_text,
                        "packet_sha256": self._packet_sha256,
                        "packet_size_bytes": self._packet_size_bytes,
                    },
                )

        if set(params) != {"template"} or type(params.get("template")) is not str:
            return self._denied(
                tool_call_id,
                tool_name,
                "invalid_arguments",
                "submit_template requires exactly one string field named template.",
            )

        template = params["template"]
        if not template.strip():
            return self._denied(
                tool_call_id,
                tool_name,
                "invalid_template",
                "Template text must not be empty.",
            )
        if "\x00" in template:
            return self._denied(
                tool_call_id,
                tool_name,
                "invalid_template",
                "Template text must not contain NUL characters.",
            )
        try:
            template_size = _utf8_size(template, field="template")
        except ValueError as exc:
            return self._denied(
                tool_call_id,
                tool_name,
                "invalid_template",
                str(exc),
            )
        if template_size > self._max_template_bytes:
            return self._denied(
                tool_call_id,
                tool_name,
                "template_too_large",
                "Template text exceeds the configured byte limit.",
            )

        submission = SubmittedTemplate(
            text=template,
            sha256=hashlib.sha256(template.encode("utf-8")).hexdigest(),
            size_bytes=template_size,
        )
        with self._lock:
            if self._submission is not None:
                return self._denied(
                    tool_call_id,
                    tool_name,
                    "already_submitted",
                    "Exactly one template submission is allowed.",
                )
            self._submission = submission

        return AuthoringToolResult(
            tool_call_id=tool_call_id,
            tool_name=tool_name,
            ok=True,
            output={
                "accepted": True,
                "template_sha256": submission.sha256,
                "template_size_bytes": submission.size_bytes,
            },
        )

    def require_submission(self) -> SubmittedTemplate:
        """Return the accepted text or fail the run when no tool submission exists."""

        with self._lock:
            if self._submission is None:
                raise MissingTemplateSubmission(
                    "provider turn ended without one successful submit_template call"
                )
            return self._submission

    def _decode_arguments(self, arguments: object) -> dict[str, Any]:
        if type(arguments) is str:
            if _utf8_size(arguments, field="arguments") > self._max_argument_bytes:
                raise ValueError("Tool arguments exceed the configured byte limit.")
            try:
                decoded = json.loads(arguments)
            except json.JSONDecodeError as exc:
                raise ValueError("Tool arguments must be valid JSON.") from exc
        elif type(arguments) is dict:
            decoded = arguments
        else:
            raise ValueError("Tool arguments must be a JSON object.")

        if type(decoded) is not dict:
            raise ValueError("Tool arguments must be a JSON object.")
        if any(type(key) is not str for key in decoded):
            raise ValueError("Tool argument names must be strings.")
        return dict(decoded)

    @staticmethod
    def _denied(
        tool_call_id: str,
        tool_name: str,
        code: str,
        message: str,
    ) -> AuthoringToolResult:
        return AuthoringToolResult(
            tool_call_id=tool_call_id,
            tool_name=tool_name,
            ok=False,
            error_code=code,
            error_message=message,
        )


__all__ = [
    "ALLOWED_TOOL_NAMES",
    "AuthoringIsolationError",
    "AuthoringIsolationSession",
    "AuthoringToolResult",
    "MissingTemplateSubmission",
    "READ_PACKET_TOOL",
    "SCHEMA_VERSION",
    "SUBMIT_TEMPLATE_TOOL",
    "SubmittedTemplate",
]
