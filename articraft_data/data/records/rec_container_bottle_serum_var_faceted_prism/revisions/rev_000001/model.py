from __future__ import annotations

# Small amber-glass serum bottle with a white dropper cap — FACETED variant.
# Frame: bottle axis along +Z, bottle base at z=0, dropper rising at +Z.
# Construction:
#   - body (root): faceted octagonal-prism amber-glass shell (flat facets
#     running up the wall) + rounded shoulder transitioning to a circular
#     neck, with an internal polygonal-to-circular bore (open mouth).
#     A white paper label band wraps the middle of the faceted body.
#   - dropper assembly (ONE rigid part): white collar that grips the neck, a
#     rounded white rubber squeeze bulb on top, and a thin clear-glass pipette
#     that runs down through the neck into the bottle.
# Articulation:
#   - dropper assembly: PRISMATIC, pulls straight UP out of the neck, lifting
#     the pipette clear of the bottle mouth.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- bottle body dimensions ----
FACET_N = 8             # octagonal cross-section (8 flat facets)
BODY_APOTHEM = 0.015    # inscribed-circle radius (flat-to-center, ~0.03 m across flats)
BODY_CIRCUM_R = BODY_APOTHEM / math.cos(math.pi / FACET_N)  # vertex radius
BODY_DIAM = 2.0 * BODY_CIRCUM_R  # circumscribed-circle diameter for polygon()
BODY_Z0 = 0.0           # base
BODY_TOP = 0.060        # top of the straight faceted wall
SHOULDER_TOP = 0.070    # top of the rounded shoulder (transitions to round)
NECK_R = 0.0080         # neck outer radius (circular)
NECK_TOP = 0.0820       # top of the neck (bottle mouth)
WALL = 0.0018           # glass wall thickness
BORE_R = NECK_R - WALL  # inner bore radius at the neck

# polygon inner diameter (for cavity)
BODY_INNER_APOTHEM = BODY_APOTHEM - WALL
BODY_INNER_CIRCUM_R = BODY_INNER_APOTHEM / math.cos(math.pi / FACET_N)
BODY_INNER_DIAM = 2.0 * BODY_INNER_CIRCUM_R

# ---- label band ----
LABEL_Z0 = 0.012
LABEL_Z1 = 0.044
LABEL_APOTHEM = BODY_APOTHEM + 0.0006
LABEL_CIRCUM_R = LABEL_APOTHEM / math.cos(math.pi / FACET_N)
LABEL_DIAM = 2.0 * LABEL_CIRCUM_R
LABEL_INNER_APOTHEM = BODY_APOTHEM - 0.0002
LABEL_INNER_CIRCUM_R = LABEL_INNER_APOTHEM / math.cos(math.pi / FACET_N)
LABEL_INNER_DIAM = 2.0 * LABEL_INNER_CIRCUM_R

# ---- dropper assembly dimensions ----
COLLAR_R = 0.0105       # white collar grips over the neck
COLLAR_Z0 = 0.066       # collar bottom (overlaps the shoulder/neck slightly)
COLLAR_TOP = 0.090      # collar top
BULB_R = 0.0095         # squeeze bulb radius
BULB_CZ = 0.1075        # bulb center height
PIPETTE_R = 0.0024      # thin glass pipette radius
PIPETTE_BOTTOM = 0.018  # pipette tip height at rest (deep in the bottle)

DROPPER_TRAVEL = 0.068  # prismatic pull-up distance (lifts pipette clear of mouth)


def _polygon_profile(wp, diameter, n_sides=FACET_N):
    """Create a regular polygon wire on the given workplane."""
    return wp.polygon(n_sides, diameter)


def _body_glass_mesh():
    # Hollow amber-glass shell: faceted octagonal body, rounded shoulder that
    # transitions from octagonal to circular, short circular neck,
    # with an internal bore cut so the bottle reads as a real open container.

    # Straight faceted body walls (octagonal prism)
    outer_body = (
        cq.Workplane("XY")
        .polygon(FACET_N, BODY_DIAM)
        .extrude(BODY_TOP)
    )

    # Shoulder: loft from faceted polygon at body top to circle at shoulder top.
    # This creates the visual transition from flat facets to the round neck.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .polygon(FACET_N, BODY_DIAM)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R + 0.0015)
        .loft(ruled=False)
    )

    # Circular neck
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(NECK_R)
        .extrude(NECK_TOP - SHOULDER_TOP)
    )

    solid = outer_body.union(shoulder).union(neck)

    # Interior cavity: polygonal bore through the body, transitioning via loft
    # to a circular bore through the neck (open at the top mouth).
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .polygon(FACET_N, BODY_INNER_DIAM)       # z = WALL, body interior
        .workplane(offset=(BODY_TOP - WALL))
        .polygon(FACET_N, BODY_INNER_DIAM)       # z = BODY_TOP, body interior
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(BORE_R)                          # z = SHOULDER_TOP, shrinking into neck
        .workplane(offset=(NECK_TOP - SHOULDER_TOP + 0.001))
        .circle(BORE_R)                          # through the mouth (open top)
        .loft(ruled=False)
    )
    solid = solid.cut(cavity)
    return mesh_from_cadquery(solid, "body_glass")


def _label_band_mesh():
    # Polygonal label ring that wraps the faceted body flats.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LABEL_Z0)
        .polygon(FACET_N, LABEL_DIAM)
        .extrude(LABEL_Z1 - LABEL_Z0)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=LABEL_Z0 - 0.0005)
        .polygon(FACET_N, LABEL_INNER_DIAM)
        .extrude(LABEL_Z1 - LABEL_Z0 + 0.001)
    )
    band = outer.cut(inner)
    return mesh_from_cadquery(band, "label_band")


def _collar_mesh():
    # White collar: an outer ring that grips the neck, with a small bore that
    # the pipette passes through, plus a thin top cap the bulb sits on.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0)
        .circle(COLLAR_R)
        .extrude(COLLAR_TOP - COLLAR_Z0)
    )
    # hollow it out so it slips over the neck (open at the bottom)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_Z0 - 0.001)
        .circle(NECK_R + 0.0006)
        .extrude((COLLAR_TOP - 0.002) - (COLLAR_Z0 - 0.001))
    )
    collar = outer.cut(inner)
    return mesh_from_cadquery(collar, "collar")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serum_bottle_faceted")

    amber = model.material("amber_glass", rgba=(0.45, 0.24, 0.06, 0.5))
    white = model.material("white_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.97, 0.96, 1.0))
    clear = model.material("clear_glass", rgba=(0.85, 0.90, 0.92, 0.35))

    # ---- body (root) ----
    body = model.part("body")
    body.visual(_body_glass_mesh(), material=amber, name="body_glass")
    body.visual(_label_band_mesh(), material=label_white, name="label_band")

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_CIRCUM_R, BODY_TOP),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    # ---- dropper assembly (collar + bulb + pipette as ONE rigid part) ----
    dropper = model.part("dropper")

    # white collar
    dropper.visual(_collar_mesh(), material=white, name="collar")

    # rounded white rubber squeeze bulb on top of the collar
    dropper.visual(
        Sphere(BULB_R),
        origin=Origin(xyz=(0.0, 0.0, BULB_CZ)),
        material=white,
        name="bulb",
    )
    # short neck connecting the collar top to the bulb so they read as one piece
    dropper.visual(
        Cylinder(0.0045, (BULB_CZ - BULB_R) - COLLAR_TOP + 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLLAR_TOP + (BULB_CZ - BULB_R)) / 2.0)),
        material=white,
        name="bulb_stem",
    )

    # thin clear-glass pipette running down from inside the collar into the body
    pip_len = COLLAR_TOP - PIPETTE_BOTTOM
    dropper.visual(
        Cylinder(PIPETTE_R, pip_len),
        origin=Origin(xyz=(0.0, 0.0, PIPETTE_BOTTOM + pip_len / 2.0)),
        material=clear,
        name="pipette",
    )

    dropper.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_TOP - PIPETTE_BOTTOM),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, (PIPETTE_BOTTOM + BULB_CZ) / 2.0)),
    )

    model.articulation(
        "body_to_dropper",
        ArticulationType.PRISMATIC,
        parent=body,
        child=dropper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DROPPER_TRAVEL,
            effort=5.0,
            velocity=0.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dropper = object_model.get_part("dropper")
    pull = object_model.get_articulation("body_to_dropper")

    # --- bottle is short and amber-glass body sits at the base ---
    body_aabb = ctx.part_world_aabb(body)
    bmn, bmx = body_aabb
    body_h = bmx[2] - bmn[2]
    body_dx = bmx[0] - bmn[0]
    body_dy = bmx[1] - bmn[1]
    ctx.check(
        "bottle body is short (squat) — height under ~0.09 m",
        body_h < 0.090,
        details=f"body height={body_h:.4f}",
    )
    ctx.check(
        "bottle body is narrow (~0.03 m across)",
        0.020 < max(body_dx, body_dy) < 0.040,
        details=f"body dx={body_dx:.4f}, dy={body_dy:.4f}",
    )

    # Verify amber glass material
    body_glass_vis = body.get_visual("body_glass")
    mat = body_glass_vis.material
    rgba = getattr(mat, "rgba", None)
    ctx.check(
        "body glass is amber-tinted and translucent",
        rgba is not None and rgba[0] > rgba[2] and rgba[3] < 0.95,
        details=f"amber_glass rgba={rgba}",
    )

    # --- FACETED BODY PROOF ---
    # The body XY extent should match the octagonal circumradius (vertex-to-vertex),
    # which is larger than the inscribed apothem. This proves the polygon was used
    # (a circle of radius BODY_APOTHEM would give extent = 2*BODY_APOTHEM = 0.030).
    max_xy = max(body_dx, body_dy)
    expected_vertex_diam = 2.0 * BODY_CIRCUM_R  # ~0.0325
    expected_circle_diam = 2.0 * BODY_APOTHEM   # 0.030 (if it were a circle)
    ctx.check(
        "faceted body XY extent matches octagonal circumradius, not inscribed circle",
        max_xy > expected_circle_diam + 0.001,
        details=f"XY extent={max_xy:.4f}, octagon vertex dia={expected_vertex_diam:.4f}, circle dia={expected_circle_diam:.4f}",
    )
    # The body width should be significantly wider than the neck, proving the
    # faceted shoulder transition from wide polygon to narrow circle.
    neck_diam = 2.0 * NECK_R  # 0.016
    ctx.check(
        "faceted body is much wider than the circular neck (shoulder transition exists)",
        max_xy > neck_diam * 1.5,
        details=f"body XY={max_xy:.4f}, neck dia={neck_diam:.4f}",
    )

    # --- white label band wraps the faceted body ---
    label_aabb = ctx.part_element_world_aabb(body, elem="label_band")
    lmn, lmx = label_aabb
    ctx.check(
        "label band is on the body wall, around the middle",
        lmn[2] > 0.005 and lmx[2] < SHOULDER_TOP,
        details=f"label z=[{lmn[2]:.4f},{lmx[2]:.4f}]",
    )

    # --- dropper seated in the neck at rest, pipette inserted into the bottle ---
    pip_rest = ctx.part_element_world_aabb(dropper, elem="pipette")
    ctx.check(
        "pipette tip is seated deep inside the bottle at rest",
        pip_rest[0][2] < SHOULDER_TOP - 0.010,
        details=f"pipette bottom z(rest)={pip_rest[0][2]:.4f}",
    )
    # collar grips the neck (projected overlap with the body along z)
    ctx.allow_overlap(
        dropper,
        body,
        elem_a="collar",
        elem_b="body_glass",
        reason="The white collar intentionally slips over the bottle neck (seated grip).",
    )
    ctx.allow_overlap(
        dropper,
        body,
        elem_a="pipette",
        elem_b="body_glass",
        reason="The thin glass pipette is intentionally inserted down through the neck into the bottle.",
    )
    ctx.expect_overlap(
        dropper,
        body,
        axes="z",
        elem_a="collar",
        elem_b="body_glass",
        min_overlap=0.002,
        name="collar seated over the neck at rest",
    )

    # --- dropper pulls straight UP and the pipette clears the neck/mouth ---
    rest_pos = ctx.part_world_position(dropper)
    with ctx.pose({pull: DROPPER_TRAVEL}):
        up_pos = ctx.part_world_position(dropper)
        pip_up = ctx.part_element_world_aabb(dropper, elem="pipette")
    ctx.check(
        "dropper translates straight up when pulled (no lateral shift)",
        up_pos[2] > rest_pos[2] + 0.05
        and abs(up_pos[0] - rest_pos[0]) < 1e-4
        and abs(up_pos[1] - rest_pos[1]) < 1e-4,
        details=f"rest={rest_pos}, up={up_pos}",
    )
    ctx.check(
        "pipette tip clears the bottle mouth at full extension",
        pip_up[0][2] >= NECK_TOP - 0.001,
        details=f"pipette bottom z(extended)={pip_up[0][2]:.4f}, mouth={NECK_TOP:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
