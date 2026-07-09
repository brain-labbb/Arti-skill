from __future__ import annotations

import re
from typing import Any

from storage.picture_binding import resolve_binding
from viewer.api.store_components import ViewerStoreComponent

# (picture_category, picture_subcategory)
SubcategoryRef = tuple[str | None, str | None]


def _slugify(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_")


class ViewerPictureStore(ViewerStoreComponent):
    """Resolve a record's picture 小类 (subcategory) + list the 小类 browser.

    The single source of truth is the per-record ``data/records/<id>/picture.json``
    sidecar (written by ``external seed`` / fork inheritance / backfill, resolved with
    lineage). The 小类 browser is a live walk of the ``picture/`` tree, so a freshly
    added ``picture/<大类>/<小类>/`` folder appears immediately. (The legacy fuzzy
    ``external_assets_map.json`` has been retired — it was a stale second source whose
    roles are now covered by the sidecar and the folder walk.)
    """

    def resolve_subcategory(
        self,
        record_id: str,
        origin_record_id: str | None = None,
        parent_record_id: str | None = None,
        category_slug: str | None = None,
    ) -> SubcategoryRef:
        # The per-record picture.json sidecar (incl. lineage inheritance) is authoritative.
        binding = resolve_binding(
            self.repo,
            record_id,
            origin_record_id=origin_record_id,
            parent_record_id=parent_record_id,
        )
        if binding is not None:
            return (binding.category, binding.subcategory)
        # Fallback for a (now-rare) sidecar-less record whose category_slug names a unique
        # 小类 folder. Derived from the picture/ tree; only computed when actually reached.
        if category_slug:
            ref = self._slug_to_subcategory().get(category_slug.strip())
            if ref is not None:
                return ref
        return (None, None)

    def _walk_picture_catalog(self) -> dict[tuple[str, str], list[str]]:
        """List (大类, 小类) -> reference images straight from the ``picture/`` tree."""
        root = self.repo.root / "picture"
        if not root.is_dir():
            return {}
        exts = {".png", ".jpg", ".jpeg", ".webp", ".gif"}
        found: dict[tuple[str, str], list[str]] = {}
        for category_dir in sorted(p for p in root.iterdir() if p.is_dir()):
            for subcat_dir in sorted(p for p in category_dir.iterdir() if p.is_dir()):
                images = sorted(
                    f"picture/{category_dir.name}/{subcat_dir.name}/{img.name}"
                    for img in subcat_dir.iterdir()
                    if img.is_file() and img.suffix.lower() in exts
                )
                found[(category_dir.name, subcat_dir.name)] = images
        return found

    def _slug_to_subcategory(self) -> dict[str, tuple[str, str]]:
        """slugified 小类 -> unique (大类, 小类) from the folder tree (collisions dropped)."""
        slug_counts: dict[str, set[tuple[str, str]]] = {}
        for category, subcategory in self._walk_picture_catalog():
            slug_counts.setdefault(_slugify(subcategory), set()).add((category, subcategory))
        return {slug: next(iter(pairs)) for slug, pairs in slug_counts.items() if len(pairs) == 1}

    def picture_subcategories(self) -> list[dict[str, Any]]:
        results: list[dict[str, Any]] = []
        for (category, subcategory), images in self._walk_picture_catalog().items():
            results.append(
                {
                    "key": f"{category}/{subcategory}",
                    "category": category,
                    "subcategory": subcategory,
                    "reference_images": list(images),
                }
            )
        results.sort(key=lambda item: item["key"].casefold())
        return results
