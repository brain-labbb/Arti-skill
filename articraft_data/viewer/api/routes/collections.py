from __future__ import annotations

import asyncio

from fastapi import APIRouter

from viewer.api.dependencies import ViewerStoreDep
from viewer.api.schemas import DatasetEntryResponse, WorkbenchEntryResponse

router = APIRouter()


@router.get("/api/collections/workbench", response_model=list[WorkbenchEntryResponse])
async def workbench_entries(store: ViewerStoreDep) -> list[WorkbenchEntryResponse]:
    return await asyncio.to_thread(store.records.list_workbench_entries)


@router.get("/api/collections/dataset", response_model=list[DatasetEntryResponse])
async def dataset_entries(store: ViewerStoreDep) -> list[DatasetEntryResponse]:
    return await asyncio.to_thread(store.records.list_dataset_entries)
