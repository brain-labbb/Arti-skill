from __future__ import annotations

from starlette.datastructures import Headers
from starlette.middleware.gzip import GZipMiddleware, GZipResponder, IdentityResponder
from starlette.types import Message, Receive, Scope, Send

# Content types we never gzip on the fly. The viewer serves large static mesh assets
# (OBJ/URDF/GLB) straight from a local materialization cache; compressing multi-MB meshes
# per request burns CPU on the single event loop and serializes concurrent asset fetches,
# so opening one record's geometry can take 20s+ in a browser (which sends
# `Accept-Encoding: gzip`). These transfer fine uncompressed over loopback, so we skip them
# and keep gzip only for the JSON API payloads (e.g. the workbench listing), where it is
# cheap and useful. Starlette's GZipMiddleware only excludes `text/event-stream` and exposes
# no hook to extend that set, so we broaden the content-type exclusion in a thin subclass.
_EXCLUDED_CONTENT_TYPES = (
    "text/event-stream",
    "model/",  # model/obj, model/gltf-binary
    "application/xml",  # URDF
    "application/octet-stream",
    "image/",
    "video/",
    "font/",
)


def _apply_exclusion(responder: IdentityResponder, message: Message) -> None:
    # Mirrors the start-message handling in Starlette's IdentityResponder, but checks the
    # content type against our broader exclusion tuple. The start message is captured (not
    # forwarded yet) exactly as upstream does; body messages are delegated to the parent.
    responder.initial_message = message
    headers = Headers(raw=message["headers"])
    responder.content_encoding_set = "content-encoding" in headers
    responder.content_type_is_excluded = headers.get("content-type", "").startswith(
        _EXCLUDED_CONTENT_TYPES
    )


class _SelectiveGZipResponder(GZipResponder):
    async def send_with_compression(self, message: Message) -> None:
        if message["type"] == "http.response.start":
            _apply_exclusion(self, message)
            return
        await super().send_with_compression(message)


class _SelectiveIdentityResponder(IdentityResponder):
    async def send_with_compression(self, message: Message) -> None:
        if message["type"] == "http.response.start":
            _apply_exclusion(self, message)
            return
        await super().send_with_compression(message)


class SelectiveGZipMiddleware(GZipMiddleware):
    """GZipMiddleware that leaves binary/static file responses uncompressed."""

    async def __call__(self, scope: Scope, receive: Receive, send: Send) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        headers = Headers(scope=scope)
        responder: IdentityResponder
        if "gzip" in headers.get("Accept-Encoding", ""):
            responder = _SelectiveGZipResponder(
                self.app, self.minimum_size, compresslevel=self.compresslevel
            )
        else:
            responder = _SelectiveIdentityResponder(self.app, self.minimum_size)
        await responder(scope, receive, send)
