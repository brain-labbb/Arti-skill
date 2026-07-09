from __future__ import annotations

import logging
import os
import threading
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from viewer.api.file_resolver import ViewerFileResolver
from viewer.api.frontend import install_frontend_routes
from viewer.api.gzip_middleware import SelectiveGZipMiddleware
from viewer.api.routes import (
    collections_router,
    files_router,
    pictures_router,
    records_router,
    runs_router,
    status_router,
)
from viewer.api.store import ViewerStore

# Use uvicorn's configured logger so warmup progress is visible in the server log.
logger = logging.getLogger("uvicorn.error")


def _warm_workbench_snapshot(store: ViewerStore) -> None:
    """Precompute the workbench snapshot so the first real request is instant.

    The snapshot build reads ~1k record.json files serially (~20s cold). Running
    it in a background daemon thread at startup means it is ready before the user
    opens the workbench, without blocking startup or any request. Errors must
    never crash startup, so they are only logged.
    """
    try:
        entries = store.records.list_workbench_entries()
        logger.info("workbench snapshot warmed: %d entries", len(entries))
    except Exception:
        logger.exception("workbench snapshot warmup failed (non-fatal)")


@asynccontextmanager
async def _lifespan(app: FastAPI):
    store = app.state.viewer_store
    threading.Thread(
        target=_warm_workbench_snapshot,
        args=(store,),
        name="workbench-warmup",
        daemon=True,
    ).start()
    yield


def _resolve_repo_root(repo_root: Path | None) -> Path:
    if repo_root is not None:
        return repo_root.resolve()
    configured = os.getenv("ARTICRAFT_REPO_ROOT")
    if configured:
        return Path(configured).resolve()
    return Path.cwd().resolve()


def _install_middleware(app: FastAPI) -> None:
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            "http://127.0.0.1:5173",
            "http://localhost:5173",
            "http://127.0.0.1:8765",
            "http://localhost:8765",
        ],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    app.add_middleware(SelectiveGZipMiddleware, minimum_size=1024)


def create_app(*, repo_root: Path | None = None) -> FastAPI:
    app = FastAPI(title="Articraft Viewer API", lifespan=_lifespan)
    resolved_repo_root = _resolve_repo_root(repo_root)
    store = ViewerStore(resolved_repo_root)

    app.state.repo_root = resolved_repo_root
    app.state.viewer_store = store
    app.state.file_resolver = ViewerFileResolver(store.materialization)

    _install_middleware(app)
    app.include_router(status_router)
    app.include_router(collections_router)
    app.include_router(records_router)
    app.include_router(runs_router)
    app.include_router(files_router)
    app.include_router(pictures_router)
    install_frontend_routes(app)
    return app


app = create_app()
