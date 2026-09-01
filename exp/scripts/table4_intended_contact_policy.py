#!/usr/bin/env python3
"""Fail-closed matching for reviewed, local Table 4 intended contacts.

This module does not change the headline Table 4 pair policy by itself.  It
provides a strict supplementary-policy primitive that callers may apply only
after binding a registry to the evaluated URDF and emitting both raw and
policy-adjusted collision evidence.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import re
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET


REGISTRY_SCHEMA = "table4_intended_contact_registry_v1"
KNOWN_PHASES = frozenset(("rest", "single_joint_sweep", "multi_joint_sobol"))
SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
ISO_UTC_RE = re.compile(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z\Z")
WILDCARD_CHARS = frozenset("*?[]{}")

REGISTRY_FIELDS = frozenset(("schema_version", "policy_id", "entries"))
ENTRY_FIELDS = frozenset(
    (
        "registration_id",
        "dataset",
        "asset_id",
        "urdf_sha256",
        "link_pair",
        "allowed_phases",
        "local_regions_m",
        "max_penetration_m",
        "reason",
        "review",
    )
)
REGION_FIELDS = frozenset(("component", "collision_elements", "minimum", "maximum"))
REVIEW_FIELDS = frozenset(("status", "reviewer", "approved_at", "evidence_sha256"))


class IntendedContactPolicyError(ValueError):
    """The registry is malformed or is not bound to the evaluated URDF."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _exact_fields(
    value: Mapping[str, Any], expected: frozenset[str], label: str
) -> None:
    observed = set(value)
    if observed != expected:
        raise IntendedContactPolicyError(
            f"{label} fields mismatch: missing={sorted(expected - observed)}, "
            f"extra={sorted(observed - expected)}"
        )


def _exact_identifier(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value.strip() != value:
        raise IntendedContactPolicyError(f"{label} must be a non-empty exact string")
    if any(character in value for character in WILDCARD_CHARS):
        raise IntendedContactPolicyError(f"{label} must not contain wildcard syntax")
    return value


def _sha256(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
        raise IntendedContactPolicyError(f"{label} must be a lowercase SHA-256")
    return value


def _finite_vector(value: Any, label: str) -> tuple[float, float, float]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != 3
    ):
        raise IntendedContactPolicyError(f"{label} must contain three numbers")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise IntendedContactPolicyError(f"{label} must be finite")
    return result  # type: ignore[return-value]


def _validate_region(value: Any, link_name: str, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise IntendedContactPolicyError(f"{label} must be an object")
    _exact_fields(value, REGION_FIELDS, label)
    component = _exact_identifier(value["component"], f"{label}.component")
    collisions = value["collision_elements"]
    if (
        not isinstance(collisions, list)
        or not collisions
        or any(not isinstance(item, str) for item in collisions)
    ):
        raise IntendedContactPolicyError(
            f"{label}.collision_elements must be a non-empty string list"
        )
    collision_elements = tuple(
        _exact_identifier(item, f"{label}.collision_elements") for item in collisions
    )
    if len(set(collision_elements)) != len(collision_elements):
        raise IntendedContactPolicyError(
            f"{label}.collision_elements contains duplicates"
        )
    minimum = _finite_vector(value["minimum"], f"{label}.minimum")
    maximum = _finite_vector(value["maximum"], f"{label}.maximum")
    if any(lower >= upper for lower, upper in zip(minimum, maximum)):
        raise IntendedContactPolicyError(
            f"{label} must have a positive-width local AABB on every axis"
        )
    return {
        "link_name": link_name,
        "component": component,
        "collision_elements": collision_elements,
        "minimum": minimum,
        "maximum": maximum,
    }


def _validate_review(value: Any, label: str) -> dict[str, str]:
    if not isinstance(value, Mapping):
        raise IntendedContactPolicyError(f"{label} must be an object")
    _exact_fields(value, REVIEW_FIELDS, label)
    if value["status"] != "approved":
        raise IntendedContactPolicyError(f"{label}.status must be approved")
    reviewer = _exact_identifier(value["reviewer"], f"{label}.reviewer")
    approved_at = value["approved_at"]
    if not isinstance(approved_at, str) or ISO_UTC_RE.fullmatch(approved_at) is None:
        raise IntendedContactPolicyError(
            f"{label}.approved_at must be an explicit UTC timestamp"
        )
    return {
        "status": "approved",
        "reviewer": reviewer,
        "approved_at": approved_at,
        "evidence_sha256": _sha256(
            value["evidence_sha256"], f"{label}.evidence_sha256"
        ),
    }


def validate_registry(value: Any) -> dict[str, Any]:
    """Validate and normalize a registry without consulting asset payloads."""

    if not isinstance(value, Mapping):
        raise IntendedContactPolicyError("registry must be an object")
    _exact_fields(value, REGISTRY_FIELDS, "registry")
    if value["schema_version"] != REGISTRY_SCHEMA:
        raise IntendedContactPolicyError(f"registry schema must be {REGISTRY_SCHEMA}")
    policy_id = _exact_identifier(value["policy_id"], "registry.policy_id")
    raw_entries = value["entries"]
    if not isinstance(raw_entries, list):
        raise IntendedContactPolicyError("registry.entries must be a list")

    entries: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    for index, raw in enumerate(raw_entries):
        label = f"registry.entries[{index}]"
        if not isinstance(raw, Mapping):
            raise IntendedContactPolicyError(f"{label} must be an object")
        _exact_fields(raw, ENTRY_FIELDS, label)
        registration_id = _exact_identifier(
            raw["registration_id"], f"{label}.registration_id"
        )
        if registration_id in seen_ids:
            raise IntendedContactPolicyError(
                f"duplicate registration_id: {registration_id}"
            )
        seen_ids.add(registration_id)
        dataset = _exact_identifier(raw["dataset"], f"{label}.dataset")
        asset_id = _exact_identifier(raw["asset_id"], f"{label}.asset_id")
        urdf_sha256 = _sha256(raw["urdf_sha256"], f"{label}.urdf_sha256")

        raw_pair = raw["link_pair"]
        if (
            not isinstance(raw_pair, list)
            or len(raw_pair) != 2
            or any(not isinstance(item, str) for item in raw_pair)
        ):
            raise IntendedContactPolicyError(
                f"{label}.link_pair must contain exactly two link names"
            )
        pair = tuple(_exact_identifier(item, f"{label}.link_pair") for item in raw_pair)
        if pair[0] == pair[1]:
            raise IntendedContactPolicyError(f"{label}.link_pair links must differ")

        phases = raw["allowed_phases"]
        if not isinstance(phases, list) or not phases:
            raise IntendedContactPolicyError(
                f"{label}.allowed_phases must be a non-empty list"
            )
        phase_set = frozenset(phases)
        if len(phase_set) != len(phases) or not phase_set <= KNOWN_PHASES:
            raise IntendedContactPolicyError(
                f"{label}.allowed_phases must be unique known phases"
            )

        raw_regions = raw["local_regions_m"]
        if not isinstance(raw_regions, Mapping) or set(raw_regions) != set(pair):
            raise IntendedContactPolicyError(
                f"{label}.local_regions_m must bind both and only the link pair"
            )
        regions = {
            link_name: _validate_region(
                raw_regions[link_name],
                link_name,
                f"{label}.local_regions_m[{link_name!r}]",
            )
            for link_name in pair
        }
        maximum = raw["max_penetration_m"]
        if isinstance(maximum, bool) or not isinstance(maximum, (int, float)):
            raise IntendedContactPolicyError(
                f"{label}.max_penetration_m must be numeric"
            )
        maximum = float(maximum)
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise IntendedContactPolicyError(
                f"{label}.max_penetration_m must be finite and positive"
            )
        reason = _exact_identifier(raw["reason"], f"{label}.reason")
        review = _validate_review(raw["review"], f"{label}.review")
        entries.append(
            {
                "registration_id": registration_id,
                "dataset": dataset,
                "asset_id": asset_id,
                "urdf_sha256": urdf_sha256,
                "link_pair": pair,
                "allowed_phases": phase_set,
                "local_regions_m": regions,
                "max_penetration_m": maximum,
                "reason": reason,
                "review": review,
            }
        )
    return {
        "schema_version": REGISTRY_SCHEMA,
        "policy_id": policy_id,
        "entries": entries,
    }


def validate_entry_urdf_binding(entry: Mapping[str, Any], urdf: Path) -> None:
    """Require the registered hash, links, and collision names to match a URDF."""

    urdf = Path(urdf).resolve(strict=True)
    if sha256_file(urdf) != entry["urdf_sha256"]:
        raise IntendedContactPolicyError(
            "registered URDF SHA-256 does not match payload"
        )
    root = ET.parse(urdf).getroot()
    link_rows = root.findall("link")
    link_names = [str(link.get("name") or "") for link in link_rows]
    if any(not name for name in link_names) or len(link_names) != len(set(link_names)):
        raise IntendedContactPolicyError("URDF link names must be non-empty and unique")
    links = dict(zip(link_names, link_rows))
    for link_name in entry["link_pair"]:
        link = links.get(link_name)
        if link is None:
            raise IntendedContactPolicyError(
                f"registered link is absent from URDF: {link_name}"
            )
        names = [
            str(collision.get("name") or "") for collision in link.findall("collision")
        ]
        if any(not name for name in names) or len(names) != len(set(names)):
            raise IntendedContactPolicyError(
                f"URDF collision names must be non-empty and unique on link: {link_name}"
            )
        requested = set(entry["local_regions_m"][link_name]["collision_elements"])
        missing = requested - set(names)
        if missing:
            raise IntendedContactPolicyError(
                f"registered collision elements are absent on {link_name}: {sorted(missing)}"
            )


def bind_registry_for_asset(
    registry: Mapping[str, Any], *, dataset: str, asset_id: str, urdf: Path
) -> dict[str, Any]:
    """Bind exact asset entries to the on-disk URDF or reject stale registrations."""

    dataset = _exact_identifier(dataset, "dataset")
    asset_id = _exact_identifier(asset_id, "asset_id")
    entries = [
        entry
        for entry in registry["entries"]
        if entry["dataset"] == dataset and entry["asset_id"] == asset_id
    ]
    for entry in entries:
        validate_entry_urdf_binding(entry, urdf)
    return {
        "schema_version": registry["schema_version"],
        "policy_id": registry["policy_id"],
        "entries": entries,
    }


def _world_to_link_local(
    bullet: Any,
    body: int,
    client: int,
    link_index: int,
    world_position: Sequence[float],
) -> tuple[float, float, float]:
    if link_index == -1:
        world_inertial_position, world_inertial_orientation = (
            bullet.getBasePositionAndOrientation(
                body, physicsClientId=client
            )
        )
        dynamics = bullet.getDynamicsInfo(body, -1, physicsClientId=client)
        link_to_inertial_position, link_to_inertial_orientation = (
            dynamics[3],
            dynamics[4],
        )
        inertial_to_link_position, inertial_to_link_orientation = (
            bullet.invertTransform(
                link_to_inertial_position, link_to_inertial_orientation
            )
        )
        position, orientation = bullet.multiplyTransforms(
            world_inertial_position,
            world_inertial_orientation,
            inertial_to_link_position,
            inertial_to_link_orientation,
        )
    else:
        state = bullet.getLinkState(
            body,
            link_index,
            computeForwardKinematics=True,
            physicsClientId=client,
        )
        position, orientation = state[4], state[5]
    inverse_position, inverse_orientation = bullet.invertTransform(
        position, orientation
    )
    local_position, _ = bullet.multiplyTransforms(
        inverse_position,
        inverse_orientation,
        world_position,
        (0.0, 0.0, 0.0, 1.0),
    )
    return _finite_vector(local_position, "PyBullet local contact position")


def pybullet_contact_evidence(
    bullet: Any,
    *,
    body: int,
    client: int,
    contact: Sequence[Any],
    link_names: Mapping[int, str],
) -> dict[str, Any]:
    """Extract link-local evidence from a PyBullet ``getContactPoints`` row."""

    if len(contact) < 9:
        raise IntendedContactPolicyError("PyBullet contact row is incomplete")
    link_a = int(contact[3])
    link_b = int(contact[4])
    if link_a not in link_names or link_b not in link_names:
        raise IntendedContactPolicyError(
            "PyBullet contact link name mapping is incomplete"
        )
    return {
        "link_a_name": link_names[link_a],
        "link_b_name": link_names[link_b],
        "penetration_depth_m": max(0.0, -float(contact[8])),
        "local_position_a_m": _world_to_link_local(
            bullet, body, client, link_a, contact[5]
        ),
        "local_position_b_m": _world_to_link_local(
            bullet, body, client, link_b, contact[6]
        ),
    }


def _inside(point: tuple[float, float, float], region: Mapping[str, Any]) -> bool:
    return all(
        lower <= coordinate <= upper
        for coordinate, lower, upper in zip(point, region["minimum"], region["maximum"])
    )


def match_contact(
    registry: Mapping[str, Any],
    *,
    dataset: str,
    asset_id: str,
    urdf_sha256: str,
    phase: str,
    link_a_name: str,
    link_b_name: str,
    penetration_depth_m: float,
    local_position_a_m: Sequence[float] | None,
    local_position_b_m: Sequence[float] | None,
    collision_element_a_name: str | None = None,
    collision_element_b_name: str | None = None,
) -> dict[str, Any]:
    """Match one contact; every incomplete, over-bound, or ambiguous case fails closed."""

    try:
        depth = float(penetration_depth_m)
    except (TypeError, ValueError):
        return {"intended_contact": False, "reason": "invalid_penetration_depth"}
    if not math.isfinite(depth) or depth < 0.0:
        return {"intended_contact": False, "reason": "invalid_penetration_depth"}
    try:
        point_a = _finite_vector(local_position_a_m, "local_position_a_m")
        point_b = _finite_vector(local_position_b_m, "local_position_b_m")
    except (IntendedContactPolicyError, TypeError, ValueError):
        return {
            "intended_contact": False,
            "reason": "missing_or_invalid_local_position",
        }
    try:
        element_a = _exact_identifier(
            collision_element_a_name, "collision_element_a_name"
        )
        element_b = _exact_identifier(
            collision_element_b_name, "collision_element_b_name"
        )
    except IntendedContactPolicyError:
        return {
            "intended_contact": False,
            "reason": "missing_or_invalid_collision_element",
        }

    link_pair = frozenset((link_a_name, link_b_name))
    if len(link_pair) != 2:
        return {"intended_contact": False, "reason": "invalid_link_pair"}
    local_points = {link_a_name: point_a, link_b_name: point_b}
    collision_elements = {link_a_name: element_a, link_b_name: element_b}
    candidates = []
    identity_match_seen = False
    for entry in registry["entries"]:
        if (
            entry["dataset"] != dataset
            or entry["asset_id"] != asset_id
            or entry["urdf_sha256"] != urdf_sha256
        ):
            continue
        identity_match_seen = True
        if frozenset(entry["link_pair"]) != link_pair:
            continue
        if phase not in entry["allowed_phases"]:
            continue
        if depth > entry["max_penetration_m"]:
            continue
        if not all(
            _inside(local_points[name], entry["local_regions_m"][name])
            for name in entry["link_pair"]
        ):
            continue
        if not all(
            collision_elements[name]
            in entry["local_regions_m"][name]["collision_elements"]
            for name in entry["link_pair"]
        ):
            continue
        candidates.append(entry)

    if len(candidates) != 1:
        reason = (
            "ambiguous_registry_match"
            if candidates
            else (
                "contact_outside_registered_scope"
                if identity_match_seen
                else "asset_or_urdf_not_registered"
            )
        )
        return {"intended_contact": False, "reason": reason}
    entry = candidates[0]
    return {
        "intended_contact": True,
        "reason": "approved_local_intended_contact",
        "policy_id": registry["policy_id"],
        "registration_id": entry["registration_id"],
        "max_penetration_m": entry["max_penetration_m"],
        "review_evidence_sha256": entry["review"]["evidence_sha256"],
    }
