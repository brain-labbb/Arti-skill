#!/usr/bin/env python3
"""Build a reproducible BlenderKit material source library for Articraft.

The downloader deliberately separates discovery, planning, and mutation:

``snapshot-categories``
    Capture the server taxonomy and current free/license supply counts.
``build-plan``
    Resolve category-first profiles into a deterministic, deduplicated plan.
``sync-plan``
    Download exactly the files (and optionally previews) locked by that plan.

All commands are dry-run/read-only unless ``--execute`` is explicit.  The
legacy ``search-download`` and transactional ``pending-redownload`` commands
remain available for compatibility, but new libraries should use the planned
workflow so license, provenance, quotas, and hashes remain auditable.
"""

from __future__ import annotations

import argparse
import collections
import concurrent.futures
import hashlib
import json
import math
import os
import re
import shutil
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable
from urllib.parse import urljoin, urlparse

import requests

# ``blendkit.com`` is the canonical host used by the current official add-on.
# An environment override keeps staging/test deployments possible without
# putting credentials or mutable server choices in a locked plan.
BLENDERKIT_SERVER = os.environ.get(
    "BLENDERKIT_SERVER", "https://www.blendkit.com"
).rstrip("/")
API_ROOT = f"{BLENDERKIT_SERVER}/api/v1"
SEARCH_URL = f"{API_ROOT}/search/"
DOWNLOAD_URL = f"{API_ROOT}/downloads/{{}}/"
CATEGORIES_URL = f"{API_ROOT}/categories/"
DEFAULT_PBR_ROOT = Path("/mnt/zsn/lyb/arti-skill/pbr_material_library")
DEFAULT_LEGACY_SAVE_DIR = Path("/mnt/zsn/lyb/arti-skill/blenderkit_metal_materials")
DEFAULT_FILE_TYPES = ("resolution_1K", "resolution_0_5K", "blend")
DEFAULT_LIBRARY_FILE_TYPES = ("resolution_1K", "resolution_0_5K")
SUPPORTED_LICENSES = ("cc_zero", "royalty_free")
LICENSE_POLICY_RECORD_ONLY = "record_only"
DEFAULT_MAX_FILE_BYTES = 32 * 1024 * 1024
DEFAULT_PROFILE_PATH = DEFAULT_PBR_ROOT / "config" / "blenderkit_profiles.json"
DEFAULT_PLAN_PATH = DEFAULT_PBR_ROOT / "plans" / "blenderkit_download_plan.json"
DEFAULT_TAXONOMY_PATH = DEFAULT_PBR_ROOT / "taxonomy" / "blenderkit_categories.json"
USER_AGENT = "Articraft-Blendkit-Material-Sync/3"
_COOLDOWN_LOCK = threading.Lock()
_COOLDOWN_UNTIL = 0.0


@dataclass(frozen=True, slots=True)
class MaterialProfile:
    """One category-first BlenderKit discovery profile.

    ``category_slug`` is exactly one official top-level material category.
    Queries and finish hints only refine retrieval inside that category; they
    never become a parallel internal material-family taxonomy.
    """

    profile_id: str
    category_slug: str
    queries: tuple[str, ...]
    finish_hints: tuple[str, ...]
    quota: int
    shader_profile: str
    file_types: tuple[str, ...]
    require_pure_pbr: bool
    require_nonprocedural: bool
    realistic_only: bool
    require_validated: bool
    require_compatible_pbr_type: bool
    max_per_author: int | None


@dataclass(frozen=True, slots=True)
class PendingMaterial:
    material_id: str
    name: str
    family: str
    asset_base_id: str
    asset_version_id: str
    api_file_id: int
    file_type: str
    relative_path: str
    expected_size_bytes: int | None
    old_sha256: str | None
    issues: tuple[str, ...]


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _atomic_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    with temporary.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def _parameter_dict(asset: dict[str, Any]) -> dict[str, Any]:
    direct = asset.get("dictParameters")
    if isinstance(direct, dict):
        return dict(direct)
    result: dict[str, Any] = {}
    for row in asset.get("parameters") or []:
        if not isinstance(row, dict):
            continue
        key = row.get("parameterType")
        if isinstance(key, str) and key:
            result[key] = row.get("value")
    return result


def _as_string_tuple(
    value: Any, *, field: str, allow_empty: bool = False
) -> tuple[str, ...]:
    if isinstance(value, str):
        values = [value]
    elif isinstance(value, list):
        values = value
    else:
        raise ValueError(f"{field} must be a string or list of strings")
    normalized = tuple(str(item).strip() for item in values if str(item).strip())
    if not normalized and not allow_empty:
        raise ValueError(f"{field} must not be empty")
    return normalized


def _string_list_with_empty_default(
    value: Any,
    *,
    field: str,
    default: tuple[str, ...],
) -> tuple[str, ...]:
    if value is None:
        return default
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"{field} must be a list of strings")
    return tuple(dict.fromkeys(item.strip() for item in value)) or default


def _profile_file_types(
    value: Any,
    *,
    field: str,
    category_slug: str,
) -> tuple[str, ...]:
    file_types = _as_string_tuple(value, field=field)
    for file_type in file_types:
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", file_type):
            raise ValueError(f"{field} contains invalid file type: {file_type!r}")
    ordered = tuple(dict.fromkeys(file_types))
    # File-form policy is explicit per profile. In particular, a top-level
    # category such as glass or liquid must not silently opt every fine class
    # into native Blender node graphs.
    del category_slug
    return ordered


def _load_profiles(
    path: Path,
    *,
    official_top_level_slugs: set[str] | None = None,
) -> tuple[dict[str, Any], list[MaterialProfile]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != 2:
        raise ValueError(
            f"unsupported profile schema in {path}: {payload.get('schema_version')!r}"
        )
    defaults = payload.get("defaults") or {}
    if not isinstance(defaults, dict):
        raise ValueError(f"defaults must be an object in {path}")
    rows = payload.get("profiles")
    if not isinstance(rows, list) or not rows:
        raise ValueError(f"profiles must be a non-empty list in {path}")

    profiles: list[MaterialProfile] = []
    seen: set[str] = set()
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f"profile {index} in {path} must be an object")
        profile_id = str(row.get("id") or "").strip()
        if not re.fullmatch(r"[a-z0-9][a-z0-9_.-]*", profile_id):
            raise ValueError(f"profile {index} has invalid id: {profile_id!r}")
        if profile_id in seen:
            raise ValueError(f"duplicate profile id in {path}: {profile_id}")
        seen.add(profile_id)
        if "family" in row or "categories" in row:
            raise ValueError(
                f"profile {profile_id} uses retired family/categories fields; "
                "use one official category_slug"
            )
        category_slug = str(row.get("category_slug") or "").strip()
        if not category_slug:
            raise ValueError(f"profile {profile_id} category_slug must not be empty")
        if (
            official_top_level_slugs is not None
            and category_slug not in official_top_level_slugs
        ):
            raise ValueError(
                f"profile {profile_id} category_slug {category_slug!r} is not an official "
                "top-level BlenderKit material category"
            )
        quota = int(row.get("quota", 0))
        if quota < 1:
            raise ValueError(f"profile {profile_id} quota must be positive")
        queries = _string_list_with_empty_default(
            row.get("queries"),
            field=f"{profile_id}.queries",
            default=("",),
        )
        finish_hints = _string_list_with_empty_default(
            row.get("finish_hints"),
            field=f"{profile_id}.finish_hints",
            default=("",),
        )
        shader_profile = str(
            row.get(
                "shader_profile",
                defaults.get("shader_profile", "opaque_metallic_roughness"),
            )
        )
        if shader_profile not in {
            "opaque_metallic_roughness",
            "optical_principled",
            "native_blender_nodes",
        }:
            raise ValueError(
                f"profile {profile_id} has unsupported shader_profile: {shader_profile!r}"
            )
        default_file_types = defaults.get(
            "file_types", list(DEFAULT_LIBRARY_FILE_TYPES)
        )
        file_types = _profile_file_types(
            row.get("file_types", default_file_types),
            field=f"{profile_id}.file_types",
            category_slug=category_slug,
        )
        raw_max_per_author = row.get(
            "max_per_author",
            defaults.get("max_per_author", max(2, math.ceil(quota / 10))),
        )
        if raw_max_per_author is None:
            max_per_author = None
        else:
            max_per_author = int(raw_max_per_author)
            if max_per_author < 1:
                raise ValueError(
                    f"profile {profile_id} max_per_author must be positive or null"
                )
        profiles.append(
            MaterialProfile(
                profile_id=profile_id,
                category_slug=category_slug,
                queries=queries,
                finish_hints=finish_hints,
                quota=quota,
                shader_profile=shader_profile,
                file_types=file_types,
                require_pure_pbr=bool(
                    row.get("require_pure_pbr", defaults.get("require_pure_pbr", True))
                ),
                require_nonprocedural=bool(
                    row.get(
                        "require_nonprocedural",
                        defaults.get("require_nonprocedural", True),
                    )
                ),
                realistic_only=bool(
                    row.get("realistic_only", defaults.get("realistic_only", True))
                ),
                require_validated=bool(
                    row.get(
                        "require_validated", defaults.get("require_validated", True)
                    )
                ),
                require_compatible_pbr_type=bool(
                    row.get(
                        "require_compatible_pbr_type",
                        defaults.get("require_compatible_pbr_type", True),
                    )
                ),
                max_per_author=max_per_author,
            )
        )
    return defaults, profiles


def _search_assets(
    session: requests.Session,
    query: str,
    *,
    limit: int,
    page_size: int,
    retries: int,
) -> list[dict[str, Any]]:
    if limit < 1:
        return []
    assets: list[dict[str, Any]] = []
    seen: set[str] = set()
    url: str | None = SEARCH_URL
    params: dict[str, Any] | None = {
        "query": query,
        "page_size": min(max(1, page_size), 80),
        "dict_parameters": 1,
    }
    while url and len(assets) < limit:
        response = _request(session, url, params=params, retries=retries)
        payload = response.json()
        for asset in payload.get("results") or []:
            asset_id = str(asset.get("id") or "")
            if not asset_id or asset_id in seen:
                continue
            seen.add(asset_id)
            assets.append(asset)
            if len(assets) >= limit:
                break
        url = payload.get("next")
        params = None
    return assets


def _candidate_file(
    asset: dict[str, Any],
    *,
    file_types: Iterable[str],
    max_file_bytes: int,
) -> tuple[dict[str, Any] | None, str | None]:
    available = {
        str(row.get("fileType")): row
        for row in asset.get("files") or []
        if isinstance(row, dict) and row.get("fileType")
    }
    oversized: list[str] = []
    for file_type in file_types:
        selected = available.get(str(file_type))
        if selected is None:
            continue
        size = selected.get("fileUploadSize")
        if size is None:
            return None, f"missing_file_size:{file_type}"
        if int(size) < 1024:
            return None, f"invalid_file_size:{file_type}:{int(size)}"
        if int(size) > max_file_bytes:
            oversized.append(f"{file_type}:{int(size)}")
            continue
        return selected, None
    if oversized:
        return None, "oversized:" + ",".join(oversized)
    return None, "no_preferred_file"


_TOKEN_RE = re.compile(r"[a-z0-9]+")
_QUERY_STOPWORDS = {
    "asset",
    "material",
    "natural",
    "pbr",
    "realistic",
    "surface",
    "texture",
    "clean",
    "new",
}


def _text_tokens(value: Any) -> set[str]:
    if isinstance(value, list):
        text_value = " ".join(str(item) for item in value)
    else:
        text_value = str(value or "")
    return set(_TOKEN_RE.findall(text_value.lower()))


def _asset_tokens(asset: dict[str, Any]) -> set[str]:
    return (
        _text_tokens(asset.get("name"))
        | _text_tokens(asset.get("displayName"))
        | _text_tokens(asset.get("tags"))
        | _text_tokens(asset.get("description"))
        | _text_tokens(asset.get("category"))
    )


def _useful_query_terms(query_terms: set[str], *, category_slug: str) -> set[str]:
    return query_terms - _QUERY_STOPWORDS - _text_tokens(category_slug)


def _candidate_matches_query(
    asset: dict[str, Any],
    query_terms: set[str],
    *,
    category_slug: str,
) -> bool:
    """Reject provider search false positives before class-pool allocation."""

    useful_query = _useful_query_terms(query_terms, category_slug=category_slug)
    return not useful_query or bool(_asset_tokens(asset) & useful_query)


def _candidate_base_score(
    asset: dict[str, Any],
    query_terms: set[str],
    *,
    category_slug: str,
) -> float:
    tokens = _asset_tokens(asset)
    useful_query = _useful_query_terms(query_terms, category_slug=category_slug)
    term_score = (
        len(tokens & useful_query) / max(1, len(useful_query)) if useful_query else 0.5
    )
    api_score = min(max(float(asset.get("score") or 0.0), 0.0), 1000.0) / 1000.0
    ratings = asset.get("ratingsCount") or {}
    bookmarks = int(ratings.get("bookmarks") or 0) if isinstance(ratings, dict) else 0
    bookmark_score = min(math.log1p(bookmarks) / math.log(201), 1.0)
    params = _parameter_dict(asset)
    metadata_score = (
        sum(
            value not in (None, "", [], {})
            for value in (
                params.get("textureSizeMeters"),
                params.get("condition"),
                params.get("pbrType"),
                asset.get("tags"),
            )
        )
        / 4.0
    )
    return (
        0.45 * term_score
        + 0.25 * api_score
        + 0.20 * bookmark_score
        + 0.10 * metadata_score
    )


def _jaccard(left: set[str], right: set[str]) -> float:
    union = left | right
    return len(left & right) / len(union) if union else 0.0


def _select_diverse(
    candidates: list[dict[str, Any]],
    *,
    quota: int,
    max_per_author: int | None,
    excluded_asset_base_ids: set[str],
) -> list[dict[str, Any]]:
    remaining = [
        row
        for row in candidates
        if str(row["asset"].get("assetBaseId") or "") not in excluded_asset_base_ids
    ]
    max_similarities = [0.0] * len(remaining)
    selected: list[dict[str, Any]] = []
    author_counts: collections.Counter[str] = collections.Counter()
    while remaining and len(selected) < quota:
        best_index: int | None = None
        best_key: tuple[float, str] | None = None
        for index, row in enumerate(remaining):
            asset = row["asset"]
            author = asset.get("author") or {}
            author_id = str(author.get("id") or author.get("fullName") or "unknown")
            if (
                max_per_author is not None
                and author_counts[author_id] >= max_per_author
            ):
                continue
            adjusted = (
                float(row["_base_score"])
                - 0.30 * max_similarities[index]
                - (
                    0.04 * author_counts[author_id]
                    if max_per_author is not None
                    else 0.0
                )
            )
            asset_base_id = str(asset.get("assetBaseId") or asset.get("id") or "")
            key = (adjusted, asset_base_id)
            if best_key is None or key > best_key:
                best_key = key
                best_index = index
        if best_index is None:
            break
        chosen = remaining.pop(best_index)
        max_similarities.pop(best_index)
        author = chosen["asset"].get("author") or {}
        author_id = str(author.get("id") or author.get("fullName") or "unknown")
        author_counts[author_id] += 1
        selected.append(chosen)
        chosen_tokens = chosen["_tokens"]
        for index, row in enumerate(remaining):
            max_similarities[index] = max(
                max_similarities[index],
                _jaccard(row["_tokens"], chosen_tokens),
            )
    return selected


def _source_asset_payload(asset: dict[str, Any]) -> dict[str, Any]:
    author = asset.get("author") or {}
    return {
        "asset_version_id": str(asset.get("id") or ""),
        "asset_base_id": str(asset.get("assetBaseId") or ""),
        "version_number": asset.get("versionNumber"),
        "name": str(asset.get("name") or "material"),
        "display_name": str(
            asset.get("displayName") or asset.get("name") or "material"
        ),
        "category": str(asset.get("category") or "unclassified"),
        "asset_type": str(asset.get("assetType") or ""),
        "is_free": asset.get("isFree") is True,
        "access": asset.get("access"),
        "license": str(asset.get("license") or ""),
        "verification_status": str(asset.get("verificationStatus") or ""),
        "can_download": asset.get("canDownload") is not False,
        "created": asset.get("created"),
        "source_app_name": asset.get("sourceAppName"),
        "source_app_version": asset.get("sourceAppVersion"),
        "addon_version": asset.get("addonVersion"),
        "description": str(asset.get("description") or ""),
        "tags": [str(tag) for tag in asset.get("tags") or []],
        "parameters": _parameter_dict(asset),
        "score": asset.get("score"),
        "ratings_count": asset.get("ratingsCount") or {},
        "author": {
            "id": author.get("id"),
            "name": str(author.get("fullName") or ""),
        },
        "preview_url": (
            asset.get("thumbnailMiddleUrlNonsquaredWebp")
            or asset.get("thumbnailMiddleUrlWebp")
            or asset.get("thumbnailMiddleUrl")
        ),
        "api_url": asset.get("url"),
    }


def _load_pending(catalog_path: Path) -> list[PendingMaterial]:
    records: list[PendingMaterial] = []
    for line_number, line in enumerate(
        catalog_path.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"invalid JSON at {catalog_path}:{line_number}: {exc}"
            ) from exc
        validation = row.get("validation") or {}
        if validation.get("status") != "normalization_pending":
            continue
        source = row.get("source") or {}
        selected = row.get("selected_file") or {}
        required = {
            "material_id": row.get("material_id"),
            "asset_base_id": source.get("asset_base_id"),
            "asset_version_id": source.get("asset_version_id"),
            "api_file_id": selected.get("api_file_id"),
            "relative_path": selected.get("relative_path"),
            "file_type": selected.get("file_type"),
        }
        missing = [key for key, value in required.items() if value in (None, "")]
        if missing:
            raise ValueError(
                f"pending record at {catalog_path}:{line_number} is missing {missing}"
            )
        records.append(
            PendingMaterial(
                material_id=str(required["material_id"]),
                name=str(row.get("name") or required["material_id"]),
                family=str(row.get("family") or "unclassified"),
                asset_base_id=str(required["asset_base_id"]),
                asset_version_id=str(required["asset_version_id"]),
                api_file_id=int(required["api_file_id"]),
                file_type=str(required["file_type"]),
                relative_path=str(required["relative_path"]),
                expected_size_bytes=(
                    int(selected["expected_size_bytes"])
                    if selected.get("expected_size_bytes") is not None
                    else None
                ),
                old_sha256=(
                    str(validation["sha256"]) if validation.get("sha256") else None
                ),
                issues=tuple(str(value) for value in validation.get("issues", [])),
            )
        )
    material_ids = [record.material_id for record in records]
    paths = [record.relative_path for record in records]
    if len(material_ids) != len(set(material_ids)):
        raise ValueError("pending catalog contains duplicate material IDs")
    if len(paths) != len(set(paths)):
        raise ValueError("pending catalog contains duplicate target paths")
    return sorted(records, key=lambda record: (record.family, record.material_id))


def _request(
    session: requests.Session,
    url: str,
    *,
    params: dict[str, Any] | None = None,
    stream: bool = False,
    retries: int = 4,
    timeout: tuple[float, float] = (20.0, 300.0),
) -> requests.Response:
    global _COOLDOWN_UNTIL
    last_error: Exception | None = None
    for attempt in range(retries):
        with _COOLDOWN_LOCK:
            wait_for = max(0.0, _COOLDOWN_UNTIL - time.monotonic())
        if wait_for:
            time.sleep(min(wait_for, 30.0))
        try:
            response = session.get(url, params=params, stream=stream, timeout=timeout)
            if response.status_code == 200:
                return response
            if response.status_code not in {408, 425, 429, 500, 502, 503, 504}:
                response.raise_for_status()
            last_error = requests.HTTPError(
                f"HTTP {response.status_code} for {response.url}: {response.text[:300]}"
            )
            retry_after = response.headers.get("Retry-After")
        except requests.RequestException as exc:
            last_error = exc
            retry_after = None
        if attempt + 1 < retries:
            try:
                requested_wait = float(retry_after) if retry_after is not None else 0.0
            except ValueError:
                requested_wait = 0.0
            backoff = min(30.0, max(float(2 ** (attempt + 1)), requested_wait))
            if isinstance(last_error, requests.HTTPError) and "HTTP 429" in str(
                last_error
            ):
                with _COOLDOWN_LOCK:
                    _COOLDOWN_UNTIL = max(_COOLDOWN_UNTIL, time.monotonic() + backoff)
            time.sleep(backoff)
    assert last_error is not None
    raise last_error


def _new_session(*, include_api_key: bool = True) -> requests.Session:
    session = requests.Session()
    session.headers.update({"User-Agent": USER_AGENT})
    api_key = os.environ.get("BLENDERKIT_API_KEY") if include_api_key else None
    if api_key:
        session.headers.update({"Authorization": f"Bearer {api_key}"})
    return session


def _download_file(
    *,
    api_file_id: int,
    api_download_url: str | None,
    destination: Path,
    scene_uuid: str,
    retries: int,
    expected_size_bytes: int | None = None,
) -> tuple[int, str]:
    api_session = _new_session()
    cdn_session: requests.Session | None = None
    try:
        api_endpoint = urljoin(
            f"{BLENDERKIT_SERVER}/",
            api_download_url or DOWNLOAD_URL.format(api_file_id),
        )
        parsed_endpoint = urlparse(api_endpoint)
        parsed_server = urlparse(BLENDERKIT_SERVER)
        if (
            parsed_endpoint.scheme != parsed_server.scheme
            or parsed_endpoint.netloc != parsed_server.netloc
            or not parsed_endpoint.path.startswith("/api/")
        ):
            raise ValueError(
                f"refusing untrusted BlenderKit API download URL for file {api_file_id}"
            )
        signed = _request(
            api_session,
            api_endpoint,
            params={"scene_uuid": scene_uuid},
            retries=retries,
        )
        payload = signed.json()
        file_url = payload.get("filePath")
        if not file_url:
            raise ValueError(
                f"download response for file {api_file_id} has no filePath"
            )
        # The Bearer token is valid only for the API host.  Never forward it to
        # a signed CDN URL returned by the API.
        cdn_session = _new_session(include_api_key=False)
        response = _request(cdn_session, str(file_url), stream=True, retries=retries)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(
            f".{destination.name}.download-{os.getpid()}-{uuid.uuid4().hex}"
        )
        digest = hashlib.sha256()
        size = 0
        try:
            with temporary.open("wb") as handle:
                for chunk in response.iter_content(chunk_size=1024 * 1024):
                    if not chunk:
                        continue
                    handle.write(chunk)
                    digest.update(chunk)
                    size += len(chunk)
                handle.flush()
                os.fsync(handle.fileno())
            if size < 1024:
                raise ValueError(
                    f"downloaded file {api_file_id} is unexpectedly small: {size}"
                )
            if expected_size_bytes is not None and size != expected_size_bytes:
                raise ValueError(
                    f"downloaded file {api_file_id} has {size} bytes; "
                    f"API metadata expected {expected_size_bytes}"
                )
            os.replace(temporary, destination)
        finally:
            if temporary.exists():
                temporary.unlink()
        return size, digest.hexdigest()
    finally:
        if cdn_session is not None:
            cdn_session.close()
        api_session.close()


def _pending_payload(record: PendingMaterial) -> dict[str, Any]:
    return {
        "material_id": record.material_id,
        "name": record.name,
        "family": record.family,
        "asset_base_id": record.asset_base_id,
        "asset_version_id": record.asset_version_id,
        "api_file_id": record.api_file_id,
        "file_type": record.file_type,
        "relative_path": record.relative_path,
        "expected_size_bytes": record.expected_size_bytes,
        "old_sha256": record.old_sha256,
        "issues": list(record.issues),
    }


def _redownload_one(
    record: PendingMaterial,
    *,
    pbr_root: Path,
    backup_root: Path,
    previous_sha256: str,
    scene_uuid: str,
    retries: int,
) -> dict[str, Any]:
    destination = pbr_root / record.relative_path
    backup = backup_root / record.relative_path
    started = time.monotonic()
    try:
        size, new_sha256 = _download_file(
            api_file_id=record.api_file_id,
            api_download_url=None,
            destination=destination,
            scene_uuid=scene_uuid,
            retries=retries,
            expected_size_bytes=record.expected_size_bytes,
        )
        return {
            **_pending_payload(record),
            "status": "downloaded",
            "size_bytes": size,
            "new_sha256": new_sha256,
            "previous_sha256": previous_sha256,
            "identical_to_previous": new_sha256 == previous_sha256,
            "identical_to_catalog": new_sha256 == record.old_sha256,
            "expected_size_matches": (
                None
                if record.expected_size_bytes is None
                else size == record.expected_size_bytes
            ),
            "elapsed_seconds": round(time.monotonic() - started, 3),
        }
    except Exception as exc:  # report and restore every individual failure
        if backup.is_file():
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(backup, destination)
            restored = True
        else:
            restored = False
        return {
            **_pending_payload(record),
            "status": "failed_restored" if restored else "failed_missing",
            "error": f"{type(exc).__name__}: {exc}",
            "elapsed_seconds": round(time.monotonic() - started, 3),
        }


def pending_redownload(args: argparse.Namespace) -> int:
    pbr_root = args.pbr_root.expanduser().resolve()
    catalog = (args.catalog or pbr_root / "catalog" / "materials.jsonl").resolve()
    records = _load_pending(catalog)
    if args.expected_count is not None and len(records) != args.expected_count:
        raise ValueError(
            f"expected {args.expected_count} pending materials, found {len(records)} in {catalog}"
        )
    missing = [
        record.relative_path
        for record in records
        if not (pbr_root / record.relative_path).is_file()
    ]
    if missing:
        raise FileNotFoundError(
            f"{len(missing)} pending source files are already missing: {missing[:5]}"
        )
    previous_hashes = {
        record.material_id: _sha256(pbr_root / record.relative_path)
        for record in records
    }
    family_counts: dict[str, int] = {}
    for record in records:
        family_counts[record.family] = family_counts.get(record.family, 0) + 1
    plan = {
        "schema_version": 1,
        "mode": "pending-redownload",
        "catalog": os.fspath(catalog),
        "pbr_root": os.fspath(pbr_root),
        "pending_count": len(records),
        "family_counts": dict(sorted(family_counts.items())),
        "materials": [
            {
                **_pending_payload(record),
                "previous_sha256": previous_hashes[record.material_id],
                "catalog_hash_matches": previous_hashes[record.material_id]
                == record.old_sha256,
            }
            for record in records
        ],
    }
    if not args.execute:
        print(json.dumps(plan, indent=2, ensure_ascii=False, sort_keys=True))
        print(
            "Dry run only. Add --execute to back up and re-download these files.",
            file=sys.stderr,
        )
        return 0

    backup_root = (
        args.backup_root.expanduser().resolve()
        if args.backup_root
        else pbr_root / "backups" / f"pending_redownload_{_utc_stamp()}"
    )
    if backup_root.exists() and any(backup_root.iterdir()):
        raise FileExistsError(f"backup directory is not empty: {backup_root}")
    backup_root.mkdir(parents=True, exist_ok=True)
    plan["backup_root"] = os.fspath(backup_root)
    plan["started_at"] = datetime.now(timezone.utc).isoformat()
    _atomic_json(backup_root / "backup_manifest.json", plan)

    moved: list[PendingMaterial] = []
    try:
        for record in records:
            source = pbr_root / record.relative_path
            backup = backup_root / record.relative_path
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(source, backup)
            moved.append(record)
    except Exception:
        for record in reversed(moved):
            source = pbr_root / record.relative_path
            backup = backup_root / record.relative_path
            if backup.is_file() and not source.exists():
                source.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(backup, source)
        raise

    scene_uuid = str(uuid.uuid4())
    rows: list[dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(
        max_workers=max(1, args.workers)
    ) as executor:
        futures = {
            executor.submit(
                _redownload_one,
                record,
                pbr_root=pbr_root,
                backup_root=backup_root,
                previous_sha256=previous_hashes[record.material_id],
                scene_uuid=scene_uuid,
                retries=args.retries,
            ): record
            for record in records
        }
        completed = 0
        for future in concurrent.futures.as_completed(futures):
            row = future.result()
            rows.append(row)
            completed += 1
            print(
                f"[{completed:02d}/{len(records):02d}] {row['status']}: "
                f"{row['family']}/{row['name']}"
            )

    rows.sort(key=lambda row: (str(row["family"]), str(row["material_id"])))
    report = {
        "schema_version": 1,
        "mode": "pending-redownload",
        "catalog": os.fspath(catalog),
        "pbr_root": os.fspath(pbr_root),
        "backup_root": os.fspath(backup_root),
        "started_at": plan["started_at"],
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "pending_count": len(records),
        "downloaded_count": sum(row["status"] == "downloaded" for row in rows),
        "failed_count": sum(row["status"] != "downloaded" for row in rows),
        "identical_count": sum(
            row.get("identical_to_previous") is True for row in rows
        ),
        "changed_count": sum(row.get("identical_to_previous") is False for row in rows),
        "family_counts": dict(sorted(family_counts.items())),
        "materials": rows,
    }
    report_path = (
        args.report.expanduser().resolve()
        if args.report
        else backup_root / "redownload_report.json"
    )
    _atomic_json(report_path, report)
    print(
        json.dumps(
            {
                key: report[key]
                for key in (
                    "pending_count",
                    "downloaded_count",
                    "failed_count",
                    "identical_count",
                    "changed_count",
                    "backup_root",
                )
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    print(f"Report: {report_path}")
    return 0 if report["failed_count"] == 0 else 1


def _safe_filename(name: str, asset_base_id: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("._") or "material"
    return f"{clean}_{asset_base_id[:8]}.blend"


def _choose_file(
    asset: dict[str, Any], file_types: Iterable[str]
) -> dict[str, Any] | None:
    available = {str(row.get("fileType")): row for row in asset.get("files") or []}
    return next(
        (available[file_type] for file_type in file_types if file_type in available),
        None,
    )


def _material_category_tree(payload: Any) -> dict[str, Any]:
    rows = payload if isinstance(payload, list) else payload.get("results") or []
    for row in rows:
        if row.get("slug") == "material":
            return row
    raise ValueError("BlenderKit categories response has no material root")


def _normalized_category_node(node: dict[str, Any]) -> dict[str, Any]:
    slug = str(node.get("slug") or "").strip()
    if not slug:
        raise ValueError("BlenderKit taxonomy contains a category without a slug")
    if not re.fullmatch(r"[a-z0-9][a-z0-9_-]*", slug):
        raise ValueError(f"BlenderKit taxonomy contains an invalid slug: {slug!r}")
    children = node.get("children") or []
    if not isinstance(children, list):
        raise ValueError(f"category {slug!r} children must be a list")
    return {
        "name": str(node.get("name") or slug),
        "slug": slug,
        "asset_count": int(node.get("assetCount", node.get("asset_count", 0)) or 0),
        "asset_count_cumulative": int(
            node.get("assetCountCumulative", node.get("asset_count_cumulative", 0)) or 0
        ),
        "children": [_normalized_category_node(child) for child in children],
    }


def _taxonomy_metadata(material_tree: dict[str, Any]) -> dict[str, Any]:
    # Re-normalize so diagnostic fields such as supply counts never affect the
    # structural taxonomy hashes.
    material_tree = _normalized_category_node(material_tree)
    slug_index: list[dict[str, Any]] = []
    seen: set[str] = set()

    def walk(node: dict[str, Any], path: tuple[str, ...], depth: int) -> None:
        slug = str(node["slug"])
        if slug in seen:
            raise ValueError(f"BlenderKit taxonomy has duplicate slug: {slug}")
        seen.add(slug)
        current_path = path + (slug,)
        slug_index.append(
            {
                "slug": slug,
                "name": str(node.get("name") or slug),
                "depth": depth,
                "path": list(current_path),
                "top_level_slug": current_path[1] if len(current_path) > 1 else None,
            }
        )
        for child in node.get("children") or []:
            walk(child, current_path, depth + 1)

    walk(material_tree, (), 0)
    top_level_slugs = [
        str(child["slug"]) for child in material_tree.get("children") or []
    ]
    canonical_tree = json.dumps(
        material_tree,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")
    canonical_index = json.dumps(
        sorted(slug_index, key=lambda row: str(row["slug"])),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")
    return {
        "slug_index": sorted(slug_index, key=lambda row: str(row["slug"])),
        "top_level_slugs": top_level_slugs,
        "taxonomy_tree_sha256": hashlib.sha256(canonical_tree).hexdigest(),
        "slug_index_sha256": hashlib.sha256(canonical_index).hexdigest(),
    }


def _load_taxonomy(path: Path) -> tuple[dict[str, Any], set[str]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != 2:
        raise ValueError(
            f"unsupported taxonomy schema in {path}: {payload.get('schema_version')!r}"
        )
    material_tree = payload.get("material_tree")
    if not isinstance(material_tree, dict) or material_tree.get("slug") != "material":
        raise ValueError(f"taxonomy {path} has no normalized material_tree")
    metadata = _taxonomy_metadata(material_tree)
    for hash_field in ("taxonomy_tree_sha256", "slug_index_sha256"):
        if payload.get(hash_field) != metadata[hash_field]:
            raise ValueError(
                f"taxonomy {path} failed {hash_field} integrity validation"
            )
    if payload.get("slug_index") != metadata["slug_index"]:
        raise ValueError(f"taxonomy {path} slug_index does not match the category tree")
    declared = payload.get("top_level_slugs")
    if declared != metadata["top_level_slugs"]:
        raise ValueError(
            f"taxonomy {path} top_level_slugs do not match the category tree"
        )
    return payload, set(metadata["top_level_slugs"])


def _search_count(query: str, *, retries: int) -> int:
    session = _new_session()
    try:
        response = _request(
            session,
            SEARCH_URL,
            params={"query": query, "page_size": 1},
            retries=retries,
        )
        return int(response.json().get("count") or 0)
    finally:
        session.close()


def snapshot_categories(args: argparse.Namespace) -> int:
    session = _new_session()
    try:
        response = _request(session, CATEGORIES_URL, retries=args.retries)
        raw = response.json()
    finally:
        session.close()
    material_tree = _normalized_category_node(_material_category_tree(raw))
    metadata = _taxonomy_metadata(material_tree)
    top_categories = material_tree["children"]
    licenses = tuple(dict.fromkeys(args.licenses or SUPPORTED_LICENSES))

    def collect(category: dict[str, Any]) -> tuple[str, dict[str, int]]:
        slug = category["slug"]
        counts = {
            "free": _search_count(
                f"asset_type:material is_free:true category_subtree:{slug}",
                retries=args.retries,
            )
        }
        for license_name in licenses:
            counts[f"free_{license_name}"] = _search_count(
                " ".join(
                    (
                        "asset_type:material",
                        "is_free:true",
                        f"license:{license_name}",
                        f"category_subtree:{slug}",
                    )
                ),
                retries=args.retries,
            )
        return slug, counts

    counts_by_slug: dict[str, dict[str, int]] = {}
    with concurrent.futures.ThreadPoolExecutor(
        max_workers=max(1, args.workers)
    ) as executor:
        futures = [executor.submit(collect, category) for category in top_categories]
        for future in concurrent.futures.as_completed(futures):
            slug, counts = future.result()
            counts_by_slug[slug] = counts
    for category in top_categories:
        category["supply"] = counts_by_slug[category["slug"]]

    snapshot = {
        "schema_version": 2,
        "source": "blenderkit",
        "api_root": API_ROOT,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "material_tree": material_tree,
        "top_level_slugs": metadata["top_level_slugs"],
        "slug_index": metadata["slug_index"],
        "taxonomy_tree_sha256": metadata["taxonomy_tree_sha256"],
        "slug_index_sha256": metadata["slug_index_sha256"],
        "licenses_counted": list(licenses),
        "raw_taxonomy_sha256": hashlib.sha256(
            json.dumps(raw, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest(),
    }
    summary = {
        "categories": len(top_categories),
        "free_sum_across_top_categories": sum(
            category["supply"]["free"] for category in top_categories
        ),
        **{
            f"free_{license_name}_sum_across_top_categories": sum(
                category["supply"][f"free_{license_name}"]
                for category in top_categories
            )
            for license_name in licenses
        },
    }
    if not args.execute:
        print(
            json.dumps(
                {"summary": summary, "snapshot": snapshot}, indent=2, ensure_ascii=False
            )
        )
        print(
            "Dry run only. Add --execute to write the taxonomy snapshot.",
            file=sys.stderr,
        )
        return 0
    output = args.output.expanduser().resolve()
    _atomic_json(output, snapshot)
    print(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True))
    print(f"Snapshot: {output}")
    return 0


def _build_search_query(
    *,
    keywords: str,
    category: str,
    require_nonprocedural: bool,
) -> str:
    return " ".join(
        token
        for token in (
            keywords.strip(),
            "asset_type:material",
            "is_free:true",
            f"category_subtree:{category}",
            "procedural:false" if require_nonprocedural else "",
            "order:-score,_score",
        )
        if token
    )


def _candidate_rejection(
    asset: dict[str, Any],
    *,
    profile: MaterialProfile,
    licenses: set[str],
    filter_licenses: bool,
    file_types: tuple[str, ...],
    max_file_bytes: int,
) -> tuple[str | None, dict[str, Any] | None]:
    if asset.get("assetType") != "material":
        return "not_material", None
    if asset.get("isFree") is not True:
        return "not_free", None
    if filter_licenses and str(asset.get("license") or "").lower() not in licenses:
        return "license_not_allowed", None
    if asset.get("canDownload") is False:
        return "cannot_download", None
    if profile.require_validated and asset.get("verificationStatus") != "validated":
        return "not_validated", None
    parameters = _parameter_dict(asset)
    if profile.require_pure_pbr and parameters.get("purePbr") is not True:
        return "not_pure_pbr", None
    if profile.require_nonprocedural and parameters.get("procedural") is not False:
        return "procedural_or_unknown", None
    if profile.realistic_only and parameters.get("materialStyle") != "realistic":
        return "not_realistic", None
    pbr_type = parameters.get("pbrType")
    if (
        profile.require_compatible_pbr_type
        and profile.shader_profile == "opaque_metallic_roughness"
        and pbr_type
        not in (
            None,
            "",
            "metallic",
        )
    ):
        return "incompatible_pbr_type", None
    selected_file, file_issue = _candidate_file(
        asset,
        file_types=file_types,
        max_file_bytes=max_file_bytes,
    )
    if selected_file is None:
        return str(file_issue or "no_file"), None
    return None, selected_file


def _canonical_json_sha256(payload: Any) -> str:
    return hashlib.sha256(
        json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
        ).encode("utf-8")
    ).hexdigest()


def _plan_content_sha256(plan: dict[str, Any]) -> str:
    digest_payload = {key: value for key, value in plan.items() if key != "plan_sha256"}
    return _canonical_json_sha256(digest_payload)


def _validated_page_size(value: int) -> int:
    page_size = int(value)
    if not 1 <= page_size <= 80:
        raise ValueError("page_size must be between 1 and 80 for the BlenderKit API")
    return page_size


def _catalog_asset_base_id(row: dict[str, Any]) -> str | None:
    """Return a BlenderKit source ID from raw or normalized catalog metadata."""

    source = row.get("source")
    source = source if isinstance(source, dict) else {}
    for value in (
        source.get("asset_base_id"),
        source.get("assetBaseId"),
        row.get("asset_base_id"),
        row.get("assetBaseId"),
    ):
        asset_base_id = str(value or "").strip()
        if asset_base_id:
            return asset_base_id
    return None


def _load_exclude_catalogs(
    paths: Iterable[Path],
) -> tuple[set[str], list[dict[str, Any]]]:
    excluded_asset_base_ids: set[str] = set()
    records: list[dict[str, Any]] = []
    for path_value in paths:
        path = path_value.expanduser().resolve()
        rows = _read_catalog_rows(path)
        extracted_ids = [_catalog_asset_base_id(row) for row in rows]
        catalog_asset_base_ids = {
            asset_base_id
            for asset_base_id in extracted_ids
            if asset_base_id is not None
        }
        excluded_asset_base_ids.update(catalog_asset_base_ids)
        records.append(
            {
                "path": os.fspath(path),
                "sha256": _sha256(path),
                "row_count": len(rows),
                "asset_base_id_count": len(catalog_asset_base_ids),
                "rows_without_asset_base_id": sum(
                    asset_base_id is None for asset_base_id in extracted_ids
                ),
            }
        )
    return excluded_asset_base_ids, records


def build_plan(args: argparse.Namespace) -> int:
    taxonomy_path = args.taxonomy.expanduser().resolve()
    taxonomy, official_top_level_slugs = _load_taxonomy(taxonomy_path)
    profile_path = args.profiles.expanduser().resolve()
    defaults, profiles = _load_profiles(
        profile_path,
        official_top_level_slugs=official_top_level_slugs,
    )
    license_policy = str(defaults.get("license_policy", LICENSE_POLICY_RECORD_ONLY))
    if license_policy != LICENSE_POLICY_RECORD_ONLY:
        raise ValueError(
            f"unsupported license_policy {license_policy!r}; use {LICENSE_POLICY_RECORD_ONLY!r}"
        )
    license_values = (
        args.accepted_licenses
        or defaults.get("accepted_licenses")
        or defaults.get("licenses")
        or list(SUPPORTED_LICENSES)
    )
    accepted_licenses = tuple(
        dict.fromkeys(
            value.lower()
            for value in _as_string_tuple(
                license_values, field="defaults.accepted_licenses"
            )
        )
    )
    if not accepted_licenses:
        raise ValueError("accepted_licenses must not be empty")
    unsupported_licenses = sorted(set(accepted_licenses) - set(SUPPORTED_LICENSES))
    if unsupported_licenses:
        raise ValueError(
            "accepted_licenses contains unsupported BlenderKit material licenses: "
            f"{unsupported_licenses}"
        )
    filter_licenses = bool(defaults.get("filter_licenses", True))
    global_file_types = (
        tuple(
            dict.fromkeys(
                args.file_types
                or _as_string_tuple(
                    defaults.get("file_types", list(DEFAULT_LIBRARY_FILE_TYPES)),
                    field="defaults.file_types",
                )
            )
        )
        if args.file_types
        else None
    )
    if global_file_types:
        for file_type in global_file_types:
            if not re.fullmatch(r"[A-Za-z0-9_.-]+", file_type):
                raise ValueError(f"invalid --file-types value: {file_type!r}")
    max_file_bytes = int(
        args.max_file_bytes
        if args.max_file_bytes is not None
        else defaults.get("max_file_bytes", DEFAULT_MAX_FILE_BYTES)
    )
    if max_file_bytes < 1024:
        raise ValueError("max_file_bytes must be at least 1024")
    pool_multiplier = max(
        1,
        int(
            args.pool_multiplier
            if args.pool_multiplier is not None
            else defaults.get("candidate_pool_multiplier", 5)
        ),
    )
    page_size = _validated_page_size(args.page_size)
    excluded_asset_base_ids, exclude_catalog_records = _load_exclude_catalogs(
        getattr(args, "exclude_catalog", None) or ()
    )

    session = _new_session()
    pools: dict[str, list[dict[str, Any]]] = {}
    diagnostics: dict[str, dict[str, Any]] = {}
    try:
        for number, profile in enumerate(profiles, 1):
            profile_file_types = global_file_types or profile.file_types
            by_base_id: dict[str, dict[str, Any]] = {}
            rejected: collections.Counter[str] = collections.Counter()
            query_log: list[dict[str, Any]] = []
            combinations = max(1, len(profile.queries) * len(profile.finish_hints))
            per_query_limit = max(
                page_size,
                math.ceil(profile.quota * pool_multiplier / combinations),
            )
            for base_query in profile.queries:
                for finish_hint in profile.finish_hints:
                    keywords = " ".join(
                        value for value in (base_query, finish_hint) if value.strip()
                    )
                    query = _build_search_query(
                        keywords=keywords,
                        category=profile.category_slug,
                        require_nonprocedural=profile.require_nonprocedural,
                    )
                    assets = _search_assets(
                        session,
                        query,
                        limit=per_query_limit,
                        page_size=page_size,
                        retries=args.retries,
                    )
                    accepted_for_query = 0
                    for asset in assets:
                        reason, selected_file = _candidate_rejection(
                            asset,
                            profile=profile,
                            licenses=set(accepted_licenses),
                            filter_licenses=filter_licenses,
                            file_types=profile_file_types,
                            max_file_bytes=max_file_bytes,
                        )
                        if reason is not None or selected_file is None:
                            rejected[str(reason)] += 1
                            continue
                        query_terms = _text_tokens(keywords)
                        if not _candidate_matches_query(
                            asset,
                            query_terms,
                            category_slug=profile.category_slug,
                        ):
                            rejected["fine_class_query_mismatch"] += 1
                            continue
                        asset_base_id = str(asset.get("assetBaseId") or "")
                        if not asset_base_id:
                            rejected["missing_asset_base_id"] += 1
                            continue
                        if asset_base_id in excluded_asset_base_ids:
                            rejected["existing_catalog_asset"] += 1
                            continue
                        row = {
                            "asset": asset,
                            "selected_file": selected_file,
                            "_tokens": _asset_tokens(asset),
                            "_base_score": _candidate_base_score(
                                asset,
                                query_terms,
                                category_slug=profile.category_slug,
                            ),
                            "discovered_by": {
                                "category_slug": profile.category_slug,
                                "query": base_query,
                                "finish_hint": finish_hint,
                            },
                        }
                        previous = by_base_id.get(asset_base_id)
                        if previous is None or float(row["_base_score"]) > float(
                            previous["_base_score"]
                        ):
                            by_base_id[asset_base_id] = row
                        accepted_for_query += 1
                    query_log.append(
                        {
                            "query": query,
                            "returned": len(assets),
                            "accepted_before_dedup": accepted_for_query,
                        }
                    )
            pools[profile.profile_id] = list(by_base_id.values())
            diagnostics[profile.profile_id] = {
                "profile_id": profile.profile_id,
                "category_slug": profile.category_slug,
                "queries": list(profile.queries),
                "finish_hints": list(profile.finish_hints),
                "shader_profile": profile.shader_profile,
                "file_types": list(profile_file_types),
                "require_validated": profile.require_validated,
                "require_compatible_pbr_type": profile.require_compatible_pbr_type,
                "quota": profile.quota,
                "candidate_count": len(by_base_id),
                "rejected": dict(sorted(rejected.items())),
                "searches": query_log,
            }
            print(
                f"[{number:02d}/{len(profiles):02d}] {profile.profile_id} "
                f"({profile.category_slug}): {len(by_base_id)} candidates for quota "
                f"{profile.quota}",
                file=sys.stderr,
            )
    finally:
        session.close()

    # Allocate scarce profiles first so a broad query cannot consume a source
    # needed by a narrow profile. Asset-base IDs remain globally unique.
    allocation_order = sorted(
        profiles,
        key=lambda profile: (
            len(pools[profile.profile_id]) / max(1, profile.quota),
            profile.profile_id,
        ),
    )
    # Seed the global allocation set with existing catalog sources as a second
    # line of defence: no profile can re-plan an already-owned BlenderKit asset.
    used_asset_base_ids: set[str] = set(excluded_asset_base_ids)
    selected_rows: list[dict[str, Any]] = []
    for profile in allocation_order:
        chosen = _select_diverse(
            pools[profile.profile_id],
            quota=profile.quota,
            max_per_author=profile.max_per_author,
            excluded_asset_base_ids=used_asset_base_ids,
        )
        diagnostics[profile.profile_id]["selected_count"] = len(chosen)
        diagnostics[profile.profile_id]["shortfall"] = profile.quota - len(chosen)
        for row in chosen:
            asset = row["asset"]
            selected_file = row["selected_file"]
            source = _source_asset_payload(asset)
            asset_base_id = source["asset_base_id"]
            used_asset_base_ids.add(asset_base_id)
            file_type = str(selected_file.get("fileType") or "blend")
            preview_url = str(source.get("preview_url") or "")
            preview_suffix = ".webp" if ".webp" in preview_url.lower() else ".png"
            version_id = re.sub(r"[^A-Za-z0-9_-]+", "_", source["asset_version_id"])
            file_id = int(selected_file["id"])
            source_dir = f"raw/blenderkit/material/{profile.category_slug}/{asset_base_id}/{version_id}"
            selected_rows.append(
                {
                    "material_id": f"blenderkit:{asset_base_id}:{source['asset_version_id']}",
                    "profile_id": profile.profile_id,
                    "category_slug": profile.category_slug,
                    "shader_profile": profile.shader_profile,
                    "selection_score": round(float(row["_base_score"]), 8),
                    "discovered_by": row["discovered_by"],
                    "source": source,
                    "selected_file": {
                        "api_file_id": file_id,
                        "file_type": file_type,
                        "expected_size_bytes": int(selected_file["fileUploadSize"]),
                        "api_download_url": selected_file.get("downloadUrl"),
                        "server_filename": selected_file.get("filename"),
                    },
                    "relative_path": f"{source_dir}/{file_type}_{file_id}.blend",
                    "preview_relative_path": (
                        f"{source_dir}/source_preview{preview_suffix}"
                        if preview_url
                        else None
                    ),
                }
            )
    selected_rows.sort(
        key=lambda row: (
            str(row["category_slug"]),
            str(row["profile_id"]),
            str(row["source"]["asset_base_id"]),
        )
    )
    total_bytes = sum(
        int(row["selected_file"]["expected_size_bytes"]) for row in selected_rows
    )
    budget_bytes = int(args.budget_gib * (1024**3))
    if budget_bytes < 1024:
        raise ValueError("budget_gib must provide at least 1024 bytes")
    shortfall = sum(int(row.get("shortfall") or 0) for row in diagnostics.values())
    plan = {
        "schema_version": 3,
        "mode": "blenderkit-category-first-library-plan",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "api_root": API_ROOT,
        "taxonomy_path": os.fspath(taxonomy_path),
        "taxonomy_file_sha256": _sha256(taxonomy_path),
        "taxonomy_tree_sha256": taxonomy["taxonomy_tree_sha256"],
        "taxonomy_slug_index_sha256": taxonomy["slug_index_sha256"],
        "profile_path": os.fspath(profile_path),
        "profile_sha256": _sha256(profile_path),
        "license_policy": license_policy,
        "accepted_licenses": list(accepted_licenses),
        "filter_licenses": filter_licenses,
        "max_file_bytes": max_file_bytes,
        "candidate_pool_multiplier": pool_multiplier,
        "page_size": page_size,
        "exclude_catalogs": exclude_catalog_records,
        "excluded_asset_base_id_count": len(excluded_asset_base_ids),
        "selected_count": len(selected_rows),
        "expected_download_bytes": total_bytes,
        "budget_bytes": budget_bytes,
        "shortfall": shortfall,
        "profiles": [diagnostics[profile.profile_id] for profile in profiles],
        "materials": selected_rows,
    }
    plan["plan_sha256"] = _plan_content_sha256(plan)
    if total_bytes > budget_bytes:
        raise ValueError(
            f"plan needs {total_bytes / 1024**3:.2f} GiB, exceeding "
            f"the {args.budget_gib:.2f} GiB budget"
        )
    summary = {
        "selected_count": len(selected_rows),
        "expected_download_gib": round(total_bytes / 1024**3, 3),
        "shortfall": shortfall,
        "license_policy": license_policy,
        "accepted_licenses": list(accepted_licenses),
        "filter_licenses": filter_licenses,
        "excluded_asset_base_id_count": len(excluded_asset_base_ids),
        "categories": dict(
            sorted(
                collections.Counter(
                    row["category_slug"] for row in selected_rows
                ).items()
            )
        ),
    }
    if not args.execute:
        print(
            json.dumps(
                {"summary": summary, "profiles": plan["profiles"]},
                indent=2,
                ensure_ascii=False,
            )
        )
        print(
            "Dry run only. Add --execute to write the locked download plan.",
            file=sys.stderr,
        )
        return 0 if (shortfall == 0 or not args.strict) else 2
    output = args.output.expanduser().resolve()
    _atomic_json(output, plan)
    print(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True))
    print(f"Plan: {output}")
    return 0 if (shortfall == 0 or not args.strict) else 2


def _relative_library_path(value: Any, *, field: str) -> Path:
    text_value = str(value or "")
    path = Path(text_value)
    if (
        not text_value
        or "\\" in text_value
        or "\x00" in text_value
        or path.is_absolute()
        or ".." in path.parts
        or path == Path(".")
    ):
        raise ValueError(f"{field} must be a safe relative path: {text_value!r}")
    return path


def _load_locked_plan(path: Path) -> dict[str, Any]:
    plan = json.loads(path.read_text(encoding="utf-8"))
    if plan.get("schema_version") != 3:
        raise ValueError(
            f"unsupported plan schema in {path}: {plan.get('schema_version')!r}"
        )
    if plan.get("mode") != "blenderkit-category-first-library-plan":
        raise ValueError(f"unsupported plan mode in {path}: {plan.get('mode')!r}")
    if plan.get("license_policy") != LICENSE_POLICY_RECORD_ONLY:
        raise ValueError("sync-plan only accepts a record_only license plan")
    expected_plan_hash = str(plan.get("plan_sha256") or "")
    if not re.fullmatch(r"[0-9a-f]{64}", expected_plan_hash):
        raise ValueError(f"plan {path} has no valid plan_sha256")
    if _plan_content_sha256(plan) != expected_plan_hash:
        raise ValueError(f"plan {path} failed plan_sha256 integrity validation")

    accepted_licenses = {
        str(value).lower() for value in plan.get("accepted_licenses") or []
    }
    if not accepted_licenses or not accepted_licenses <= set(SUPPORTED_LICENSES):
        raise ValueError(f"plan {path} has invalid accepted_licenses")
    filter_licenses = bool(plan.get("filter_licenses", True))
    materials = plan.get("materials")
    if not isinstance(materials, list):
        raise ValueError(f"plan {path} materials must be a list")
    if int(plan.get("selected_count", -1)) != len(materials):
        raise ValueError(f"plan {path} selected_count does not match materials")

    material_ids: set[str] = set()
    relative_paths: set[str] = set()
    preview_paths: set[str] = set()
    expected_total = 0
    max_file_bytes = int(plan.get("max_file_bytes") or 0)
    if max_file_bytes < 1024:
        raise ValueError(f"plan {path} has invalid max_file_bytes")
    for index, row in enumerate(materials):
        if not isinstance(row, dict):
            raise ValueError(f"plan material {index} must be an object")
        material_id = str(row.get("material_id") or "")
        if not material_id or material_id in material_ids:
            raise ValueError(
                f"plan material {index} has a missing or duplicate material_id"
            )
        material_ids.add(material_id)
        category_slug = str(row.get("category_slug") or "")
        discovered_by = row.get("discovered_by") or {}
        if discovered_by.get("category_slug") != category_slug:
            raise ValueError(
                f"plan material {material_id} has inconsistent category metadata"
            )
        source = row.get("source") or {}
        if source.get("asset_type") != "material" or source.get("is_free") is not True:
            raise ValueError(f"plan material {material_id} is not a free material")
        exact_license = str(source.get("license") or "")
        if not exact_license:
            raise ValueError(f"plan material {material_id} has no recorded license")
        if filter_licenses and exact_license.lower() not in accepted_licenses:
            raise ValueError(f"plan material {material_id} has an unaccepted license")
        relative_path = _relative_library_path(
            row.get("relative_path"),
            field=f"materials[{index}].relative_path",
        )
        path_key = relative_path.as_posix()
        if path_key in relative_paths:
            raise ValueError(f"plan {path} has duplicate target path: {path_key}")
        relative_paths.add(path_key)
        if row.get("preview_relative_path") is not None:
            preview_path = _relative_library_path(
                row.get("preview_relative_path"),
                field=f"materials[{index}].preview_relative_path",
            )
            preview_key = preview_path.as_posix()
            if preview_key in preview_paths or preview_key in relative_paths:
                raise ValueError(
                    f"plan {path} has duplicate target path: {preview_key}"
                )
            preview_paths.add(preview_key)
        selected_file = row.get("selected_file") or {}
        try:
            api_file_id = int(selected_file.get("api_file_id"))
            expected_size = int(selected_file.get("expected_size_bytes"))
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"plan material {material_id} has invalid selected_file"
            ) from exc
        if api_file_id < 1:
            raise ValueError(f"plan material {material_id} has invalid api_file_id")
        if not 1024 <= expected_size <= max_file_bytes:
            raise ValueError(
                f"plan material {material_id} size {expected_size} is outside plan limits"
            )
        expected_total += expected_size

    collisions = relative_paths & preview_paths
    if collisions:
        raise ValueError(
            f"plan {path} reuses file and preview paths: {sorted(collisions)[:5]}"
        )

    if expected_total != int(plan.get("expected_download_bytes", -1)):
        raise ValueError(
            f"plan {path} expected_download_bytes does not match its files"
        )
    if expected_total > int(plan.get("budget_bytes", -1)):
        raise ValueError(f"plan {path} exceeds its locked download budget")
    return plan


def _download_preview(
    *,
    url: str,
    destination: Path,
    retries: int,
    max_bytes: int,
) -> tuple[int, str]:
    session = _new_session(include_api_key=False)
    try:
        response = _request(session, url, stream=True, retries=retries)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(
            f".{destination.name}.download-{os.getpid()}-{uuid.uuid4().hex}"
        )
        digest = hashlib.sha256()
        size = 0
        try:
            with temporary.open("wb") as handle:
                for chunk in response.iter_content(chunk_size=256 * 1024):
                    if not chunk:
                        continue
                    size += len(chunk)
                    if size > max_bytes:
                        raise ValueError(
                            f"preview {url} exceeded the {max_bytes} byte per-file limit"
                        )
                    handle.write(chunk)
                    digest.update(chunk)
                handle.flush()
                os.fsync(handle.fileno())
            if size < 1:
                raise ValueError(f"preview {url} was empty")
            os.replace(temporary, destination)
        finally:
            if temporary.exists():
                temporary.unlink()
        return size, digest.hexdigest()
    finally:
        session.close()


def _read_catalog_rows(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"invalid catalog JSON at {path}:{line_number}: {exc}"
            ) from exc
        if not isinstance(row, dict) or not row.get("material_id"):
            raise ValueError(f"invalid catalog row at {path}:{line_number}")
        rows.append(row)
    return rows


def _resolve_sync_provenance_path(
    provenance: Path | None,
    *,
    pbr_root: Path,
    plan_sha256: str,
) -> Path:
    filename = f"sync_{plan_sha256[:16]}_{_utc_stamp()}.json"
    if provenance is None:
        return pbr_root / "provenance" / filename
    explicit = provenance.expanduser().resolve()
    if explicit.is_dir():
        return explicit / filename
    # A missing suffix does not imply directory intent. Nonexistent paths and
    # existing files preserve the historical explicit-file behavior.
    return explicit


def _sync_material_file(
    *,
    row: dict[str, Any],
    pbr_root: Path,
    scene_uuid: str,
    retries: int,
    reuse_existing: bool,
) -> dict[str, Any]:
    selected_file = row["selected_file"]
    destination = pbr_root / _relative_library_path(
        row["relative_path"], field=f"{row['material_id']}.relative_path"
    )
    expected_size = int(selected_file["expected_size_bytes"])
    if reuse_existing:
        size = destination.stat().st_size
        digest = _sha256(destination)
        acquisition = "resumed_existing"
    else:
        size, digest = _download_file(
            api_file_id=int(selected_file["api_file_id"]),
            api_download_url=(
                str(selected_file["api_download_url"])
                if selected_file.get("api_download_url")
                else None
            ),
            destination=destination,
            scene_uuid=scene_uuid,
            retries=retries,
            expected_size_bytes=expected_size,
        )
        acquisition = "downloaded"
    if size != expected_size or not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ValueError(f"download validation failed for {row['material_id']}")
    return {
        "row": row,
        "size": size,
        "sha256": digest,
        "acquisition": acquisition,
    }


def sync_plan(args: argparse.Namespace) -> int:
    plan_path = args.plan.expanduser().resolve()
    plan = _load_locked_plan(plan_path)
    pbr_root = args.pbr_root.expanduser().resolve()
    catalog_path = (
        args.catalog.expanduser().resolve()
        if args.catalog
        else pbr_root / "catalog" / "materials.jsonl"
    )
    plan_bytes = int(plan["expected_download_bytes"])
    cli_budget_bytes = (
        int(args.budget_gib * (1024**3)) if args.budget_gib is not None else None
    )
    if cli_budget_bytes is not None and plan_bytes > cli_budget_bytes:
        raise ValueError(
            f"plan needs {plan_bytes / 1024**3:.3f} GiB, exceeding the sync cap "
            f"of {args.budget_gib:.3f} GiB"
        )
    if args.max_file_bytes is not None:
        oversized = [
            row["material_id"]
            for row in plan["materials"]
            if int(row["selected_file"]["expected_size_bytes"]) > args.max_file_bytes
        ]
        if oversized:
            raise ValueError(
                f"{len(oversized)} plan files exceed --max-file-bytes: {oversized[:5]}"
            )

    workers = max(1, int(getattr(args, "workers", 1)))
    resume_existing = bool(getattr(args, "resume_existing", False))
    overwrite = bool(getattr(args, "overwrite", False))
    if resume_existing and overwrite:
        raise ValueError("--resume-existing and --overwrite are mutually exclusive")

    summary = {
        "plan_sha256": plan["plan_sha256"],
        "selected_count": len(plan["materials"]),
        "expected_download_bytes": plan_bytes,
        "include_previews": bool(args.include_previews),
        "resume_existing": resume_existing,
        "workers": workers,
        "pbr_root": os.fspath(pbr_root),
        "catalog": os.fspath(catalog_path),
    }
    if not args.execute:
        print(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True))
        print(
            "Dry run only. Add --execute to download the locked plan.", file=sys.stderr
        )
        return 0

    provenance_path = _resolve_sync_provenance_path(
        args.provenance,
        pbr_root=pbr_root,
        plan_sha256=str(plan["plan_sha256"]),
    )

    conflicts: list[str] = []
    invalid_existing: list[str] = []
    reused_material_ids: set[str] = set()
    reused_preview_ids: set[str] = set()
    reused_preview_bytes = 0
    for row in plan["materials"]:
        material_id = str(row["material_id"])
        target = pbr_root / _relative_library_path(
            row["relative_path"], field=f"{material_id}.relative_path"
        )
        if target.exists():
            if resume_existing:
                expected_size = int(row["selected_file"]["expected_size_bytes"])
                if not target.is_file() or target.stat().st_size != expected_size:
                    invalid_existing.append(os.fspath(target))
                else:
                    reused_material_ids.add(material_id)
            elif not overwrite:
                conflicts.append(os.fspath(target))
        if args.include_previews and row.get("preview_relative_path"):
            preview_target = pbr_root / _relative_library_path(
                row["preview_relative_path"],
                field=f"{material_id}.preview_relative_path",
            )
            if preview_target.exists():
                if resume_existing:
                    preview_size = (
                        preview_target.stat().st_size if preview_target.is_file() else 0
                    )
                    if not 1 <= preview_size <= args.max_preview_bytes:
                        invalid_existing.append(os.fspath(preview_target))
                    else:
                        reused_preview_ids.add(material_id)
                        reused_preview_bytes += preview_size
                elif not overwrite:
                    conflicts.append(os.fspath(preview_target))
    if invalid_existing:
        raise FileExistsError(
            f"{len(invalid_existing)} existing targets cannot be resumed because "
            f"their type or size does not match: {invalid_existing[:5]}"
        )
    if conflicts:
        raise FileExistsError(
            f"{len(conflicts)} planned targets already exist; use --resume-existing "
            f"to verify and reuse them or --overwrite: {conflicts[:5]}"
        )

    started_at = datetime.now(timezone.utc).isoformat()
    scene_uuid = str(uuid.uuid4())
    catalog_updates: list[dict[str, Any]] = []
    preview_total = reused_preview_bytes
    preview_budget_bytes = int(args.preview_budget_mib * (1024**2))
    if preview_total > preview_budget_bytes:
        raise ValueError("resumed previews exceed the preview download budget")

    def sync_one(row: dict[str, Any]) -> dict[str, Any]:
        return _sync_material_file(
            row=row,
            pbr_root=pbr_root,
            scene_uuid=scene_uuid,
            retries=args.retries,
            reuse_existing=str(row["material_id"]) in reused_material_ids,
        )

    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as executor:
        # executor.map preserves plan order even when downloads finish out of order.
        material_results = list(executor.map(sync_one, plan["materials"]))

    plan_file_sha256 = _sha256(plan_path)
    downloaded_at = datetime.now(timezone.utc).isoformat()
    for index, result in enumerate(material_results, 1):
        row = result["row"]
        selected_file = row["selected_file"]
        size = int(result["size"])
        digest = str(result["sha256"])
        acquisition = str(result["acquisition"])

        preview_record: dict[str, Any] | None = None
        preview_url = str(row["source"].get("preview_url") or "")
        if args.include_previews and row.get("preview_relative_path") and preview_url:
            preview_destination = pbr_root / _relative_library_path(
                row["preview_relative_path"],
                field=f"{row['material_id']}.preview_relative_path",
            )
            if str(row["material_id"]) in reused_preview_ids:
                preview_size = preview_destination.stat().st_size
                preview_sha256 = _sha256(preview_destination)
                preview_acquisition = "resumed_existing"
            else:
                remaining_preview_budget = preview_budget_bytes - preview_total
                if remaining_preview_budget < 1:
                    raise ValueError("preview download budget exhausted")
                preview_size, preview_sha256 = _download_preview(
                    url=preview_url,
                    destination=preview_destination,
                    retries=args.retries,
                    max_bytes=min(args.max_preview_bytes, remaining_preview_budget),
                )
                preview_total += preview_size
                preview_acquisition = "downloaded"
            preview_record = {
                "relative_path": row["preview_relative_path"],
                "source_url": preview_url,
                "size_bytes": preview_size,
                "sha256": preview_sha256,
                "acquisition": preview_acquisition,
            }

        catalog_updates.append(
            {
                "schema_version": 1,
                "material_id": row["material_id"],
                "name": row["source"]["display_name"],
                "category_slug": row["category_slug"],
                "profile_id": row["profile_id"],
                "shader_profile": row["shader_profile"],
                "source": row["source"],
                "license": row["source"]["license"],
                "license_policy": plan["license_policy"],
                "selected_file": {
                    **selected_file,
                    "relative_path": row["relative_path"],
                    "size_bytes": size,
                    "sha256": digest,
                },
                "preview": preview_record,
                "provenance": {
                    "plan_sha256": plan["plan_sha256"],
                    "plan_file_sha256": plan_file_sha256,
                    "downloaded_at": downloaded_at,
                    "acquisition": acquisition,
                },
                "validation": {"status": "downloaded_verified"},
            }
        )
        print(f"[{index:03d}/{len(plan['materials']):03d}] {row['material_id']}")

    catalog_by_id = {
        str(row["material_id"]): row for row in _read_catalog_rows(catalog_path)
    }
    catalog_by_id.update({str(row["material_id"]): row for row in catalog_updates})
    merged_catalog = sorted(
        catalog_by_id.values(),
        key=lambda row: (str(row.get("category_slug") or ""), str(row["material_id"])),
    )
    _atomic_jsonl(catalog_path, merged_catalog)
    catalog_sha256 = _sha256(catalog_path)

    finished_at = datetime.now(timezone.utc).isoformat()
    provenance_materials = sorted(
        (
            {
                "material_id": row["material_id"],
                "category_slug": row["category_slug"],
                "license": row["license"],
                "relative_path": row["selected_file"]["relative_path"],
                "sha256": row["selected_file"]["sha256"],
                "size_bytes": row["selected_file"]["size_bytes"],
                "acquisition": row["provenance"]["acquisition"],
            }
            for row in catalog_updates
        ),
        key=lambda row: (str(row["category_slug"]), str(row["material_id"])),
    )
    downloaded_rows = [
        row
        for row in catalog_updates
        if row["provenance"]["acquisition"] == "downloaded"
    ]
    reused_rows = [
        row
        for row in catalog_updates
        if row["provenance"]["acquisition"] == "resumed_existing"
    ]
    provenance = {
        "schema_version": 1,
        "mode": "blenderkit-category-first-sync",
        "license_policy": plan["license_policy"],
        "plan_path": os.fspath(plan_path),
        "plan_file_sha256": plan_file_sha256,
        "plan_sha256": plan["plan_sha256"],
        "started_at": started_at,
        "finished_at": finished_at,
        "pbr_root": os.fspath(pbr_root),
        "catalog_path": os.fspath(catalog_path),
        "catalog_sha256": catalog_sha256,
        "catalog_material_count": len(merged_catalog),
        "synced_count": len(catalog_updates),
        "downloaded_count": len(downloaded_rows),
        "downloaded_bytes": sum(
            int(row["selected_file"]["size_bytes"]) for row in downloaded_rows
        ),
        "reused_existing_count": len(reused_rows),
        "reused_existing_bytes": sum(
            int(row["selected_file"]["size_bytes"]) for row in reused_rows
        ),
        "preview_count": sum(row["preview"] is not None for row in catalog_updates),
        "preview_bytes": preview_total,
        "materials": provenance_materials,
    }
    _atomic_json(provenance_path, provenance)
    summary.update(
        {
            "downloaded_count": provenance["downloaded_count"],
            "downloaded_bytes": provenance["downloaded_bytes"],
            "reused_existing_count": provenance["reused_existing_count"],
            "reused_existing_bytes": provenance["reused_existing_bytes"],
            "preview_count": provenance["preview_count"],
            "preview_bytes": preview_total,
            "catalog_sha256": catalog_sha256,
            "provenance": os.fspath(provenance_path),
        }
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True))
    return 0


def search_download(args: argparse.Namespace) -> int:
    if not args.execute:
        print("Dry run only. Add --execute to search and download.", file=sys.stderr)
        print(json.dumps(vars(args), indent=2, default=os.fspath, sort_keys=True))
        return 0
    save_dir = args.save_dir.expanduser().resolve()
    save_dir.mkdir(parents=True, exist_ok=True)
    session = _new_session()
    assets: list[dict[str, Any]] = []
    url: str | None = SEARCH_URL
    params: dict[str, Any] | None = {
        "query": args.query,
        "page_size": min(args.page_size, 100),
    }
    try:
        while url and len(assets) < args.num_download:
            response = _request(session, url, params=params, retries=args.retries)
            payload = response.json()
            assets.extend(payload.get("results") or [])
            url = payload.get("next")
            params = None
    finally:
        session.close()
    downloaded = 0
    scene_uuid = str(uuid.uuid4())
    for asset in assets:
        if downloaded >= args.num_download:
            break
        selected = _choose_file(asset, args.file_types)
        if selected is None:
            continue
        destination = save_dir / _safe_filename(
            str(asset.get("name") or "material"), str(asset["assetBaseId"])
        )
        if destination.exists() and not args.overwrite:
            continue
        size, digest = _download_file(
            api_file_id=int(selected["id"]),
            api_download_url=(
                str(selected["downloadUrl"]) if selected.get("downloadUrl") else None
            ),
            destination=destination,
            scene_uuid=scene_uuid,
            retries=args.retries,
            expected_size_bytes=(
                int(selected["fileUploadSize"])
                if selected.get("fileUploadSize") is not None
                else None
            ),
        )
        downloaded += 1
        print(
            f"[{downloaded}/{args.num_download}] {destination.name} {size} bytes sha256={digest}"
        )
    print(f"Finished: {downloaded}")
    return 0 if downloaded == args.num_download else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    snapshot = subparsers.add_parser(
        "snapshot-categories",
        help="Recursively snapshot official BlenderKit material categories and supply",
    )
    snapshot.add_argument("--output", type=Path, default=DEFAULT_TAXONOMY_PATH)
    snapshot.add_argument(
        "--licenses",
        nargs="+",
        default=list(SUPPORTED_LICENSES),
        choices=list(SUPPORTED_LICENSES),
        help="License values to count for diagnostics; discovery remains is_free:true",
    )
    snapshot.add_argument("--workers", type=int, default=4)
    snapshot.add_argument("--retries", type=int, default=4)
    snapshot.add_argument(
        "--execute",
        action="store_true",
        help="Write the snapshot; otherwise only print it",
    )
    snapshot.set_defaults(handler=snapshot_categories)

    planner = subparsers.add_parser(
        "build-plan",
        help="Build a locked category-first free-material download plan",
    )
    planner.add_argument("--profiles", type=Path, default=DEFAULT_PROFILE_PATH)
    planner.add_argument("--taxonomy", type=Path, default=DEFAULT_TAXONOMY_PATH)
    planner.add_argument("--output", type=Path, default=DEFAULT_PLAN_PATH)
    planner.add_argument(
        "--accepted-licenses",
        nargs="+",
        choices=list(SUPPORTED_LICENSES),
        help="Accepted exact license records (default: cc_zero and royalty_free)",
    )
    planner.add_argument(
        "--file-types",
        nargs="+",
        help="Optional global override; profiles normally own ordered file types",
    )
    planner.add_argument("--max-file-bytes", type=int)
    planner.add_argument("--pool-multiplier", type=int)
    planner.add_argument(
        "--exclude-catalog",
        action="append",
        type=Path,
        default=[],
        help=(
            "Existing raw or normalized JSONL catalog whose BlenderKit source IDs "
            "must be excluded; repeat for multiple catalogs"
        ),
    )
    planner.add_argument("--page-size", type=int, default=80)
    planner.add_argument("--budget-gib", type=float, default=4.0)
    planner.add_argument("--retries", type=int, default=4)
    planner.add_argument(
        "--strict",
        action="store_true",
        help="Return status 2 when one or more profile quotas have a shortfall",
    )
    planner.add_argument(
        "--execute",
        action="store_true",
        help="Write the locked plan; otherwise only print diagnostics",
    )
    planner.set_defaults(handler=build_plan)

    sync = subparsers.add_parser(
        "sync-plan",
        help="Validate and optionally download exactly one locked plan",
    )
    sync.add_argument("--plan", type=Path, default=DEFAULT_PLAN_PATH)
    sync.add_argument("--pbr-root", type=Path, default=DEFAULT_PBR_ROOT)
    sync.add_argument("--catalog", type=Path)
    sync.add_argument(
        "--provenance",
        type=Path,
        help=(
            "Provenance JSON file path, or an existing directory in which a "
            "sync_<planhash>_<timestamp>.json file will be created"
        ),
    )
    sync.add_argument("--budget-gib", type=float)
    sync.add_argument("--max-file-bytes", type=int)
    sync.add_argument("--include-previews", action="store_true")
    sync.add_argument("--preview-budget-mib", type=float, default=256.0)
    sync.add_argument("--max-preview-bytes", type=int, default=4 * 1024 * 1024)
    sync.add_argument("--retries", type=int, default=4)
    sync.add_argument(
        "--workers",
        type=int,
        default=1,
        help="Concurrent material downloads (8 is a practical bulk-sync starting point)",
    )
    existing_policy = sync.add_mutually_exclusive_group()
    existing_policy.add_argument("--overwrite", action="store_true")
    existing_policy.add_argument(
        "--resume-existing",
        action="store_true",
        help=(
            "Reuse existing planned targets only after validating their size and computing SHA-256"
        ),
    )
    sync.add_argument(
        "--execute",
        action="store_true",
        help="Download and update catalog/provenance; otherwise only validate",
    )
    sync.set_defaults(handler=sync_plan)

    pending = subparsers.add_parser(
        "pending-redownload",
        help="Back up and re-download the catalog's normalization_pending materials",
    )
    pending.add_argument("--pbr-root", type=Path, default=DEFAULT_PBR_ROOT)
    pending.add_argument("--catalog", type=Path)
    pending.add_argument("--backup-root", type=Path)
    pending.add_argument("--report", type=Path)
    pending.add_argument("--expected-count", type=int, default=79)
    pending.add_argument("--workers", type=int, default=4)
    pending.add_argument("--retries", type=int, default=4)
    pending.add_argument(
        "--execute",
        action="store_true",
        help="Actually move old files to backup and download; otherwise print the plan",
    )
    pending.set_defaults(handler=pending_redownload)

    search = subparsers.add_parser(
        "search-download",
        help="Parameterized replacement for the original broad search downloader",
    )
    search.add_argument(
        "--query",
        default="asset_type:material category_subtree:metal is_free:true",
    )
    search.add_argument("--num-download", type=int, default=1000)
    search.add_argument("--page-size", type=int, default=80)
    search.add_argument("--save-dir", type=Path, default=DEFAULT_LEGACY_SAVE_DIR)
    search.add_argument("--file-types", nargs="+", default=list(DEFAULT_FILE_TYPES))
    search.add_argument("--retries", type=int, default=4)
    search.add_argument("--overwrite", action="store_true")
    search.add_argument("--execute", action="store_true")
    search.set_defaults(handler=search_download)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if getattr(args, "workers", 1) < 1:
        parser.error("--workers must be at least 1")
    if getattr(args, "retries", 1) < 1:
        parser.error("--retries must be at least 1")
    if getattr(args, "page_size", 1) > 80 or getattr(args, "page_size", 1) < 1:
        parser.error("--page-size must be between 1 and 80")
    budget_gib = getattr(args, "budget_gib", None)
    if budget_gib is not None and budget_gib <= 0:
        parser.error("--budget-gib must be positive")
    if getattr(args, "preview_budget_mib", 1.0) <= 0:
        parser.error("--preview-budget-mib must be positive")
    if getattr(args, "max_preview_bytes", 1) < 1:
        parser.error("--max-preview-bytes must be positive")
    max_file_bytes = getattr(args, "max_file_bytes", None)
    if max_file_bytes is not None and max_file_bytes < 1024:
        parser.error("--max-file-bytes must be at least 1024")
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
