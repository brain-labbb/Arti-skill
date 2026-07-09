from __future__ import annotations

import logging

import pytest

from storage.revisions import (
    VariantParentError,
    assert_parent_not_variant,
    variant_parent_origin,
)

_ORIGIN = {
    "record_id": "rec_origin",
    "lineage": {
        "origin_record_id": "rec_origin",
        "parent_record_id": None,
        "parent_revision_id": None,
        "edit_mode": "root",
    },
}

_VARIANT = {
    "record_id": "rec_variant",
    "lineage": {
        "origin_record_id": "rec_origin",
        "parent_record_id": "rec_origin",
        "parent_revision_id": "rev_000001",
        "edit_mode": "copy",
    },
}


def test_variant_parent_origin_none_for_origin() -> None:
    assert variant_parent_origin("rec_origin", _ORIGIN) is None


def test_variant_parent_origin_detects_variant() -> None:
    assert variant_parent_origin("rec_variant", _VARIANT) == "rec_origin"


def test_variant_parent_origin_detects_via_origin_mismatch_only() -> None:
    # Defensive: parent_record_id dropped but origin still points elsewhere.
    record = {
        "record_id": "rec_variant",
        "lineage": {"origin_record_id": "rec_origin", "parent_record_id": None},
    }
    assert variant_parent_origin("rec_variant", record) == "rec_origin"


def test_assert_parent_not_variant_allows_origin() -> None:
    # Must not raise for an origin/root record.
    assert_parent_not_variant("rec_origin", _ORIGIN)


def test_assert_parent_not_variant_rejects_variant() -> None:
    with pytest.raises(VariantParentError) as exc_info:
        assert_parent_not_variant("rec_variant", _VARIANT)
    message = str(exc_info.value)
    assert "rec_variant" in message
    assert "rec_origin" in message
    assert "--allow-variant-parent" in message


def test_assert_parent_not_variant_flag_proceeds_with_warning(
    caplog: pytest.LogCaptureFixture,
) -> None:
    with caplog.at_level(logging.WARNING):
        # Escape hatch: proceeds (no raise) and logs a warning.
        assert_parent_not_variant("rec_variant", _VARIANT, allow_variant_parent=True)
    assert any("rec_variant" in record.message for record in caplog.records)
