from __future__ import annotations

import asyncio

from fastapi import APIRouter
from fastapi.responses import FileResponse

from viewer.api.dependencies import FileResolverDep, ViewerStoreDep
from viewer.api.media import file_cache_control, file_media_type
from viewer.api.schemas import PictureSubcategoryResponse

router = APIRouter()


@router.get("/api/picture-subcategories", response_model=list[PictureSubcategoryResponse])
async def picture_subcategories(store: ViewerStoreDep) -> list[PictureSubcategoryResponse]:
    entries = await asyncio.to_thread(store.picture.picture_subcategories)
    return [PictureSubcategoryResponse(**entry) for entry in entries]


@router.get("/api/picture/{file_path:path}")
async def picture_file(file_path: str, resolver: FileResolverDep) -> FileResponse:
    target = resolver.resolve_picture_target(file_path)
    return FileResponse(
        target,
        media_type=file_media_type(target),
        headers={"Cache-Control": file_cache_control(immutable=False)},
    )
