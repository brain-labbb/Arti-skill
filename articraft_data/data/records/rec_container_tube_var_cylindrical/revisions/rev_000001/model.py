from __future__ import annotations

# Blue sunscreen squeeze tube — cylindrical barrel variant.
#
# Frame:
#   - Tube stands upright; body axis along +Z.
#   - z=0 is the softly rounded bottom; the bottle rises in +Z as a blue
#     straight cylindrical barrel of constant round radius.
#   - The top is blue with a round raised neck and an annular open mouth. The
#     bore continues down into a visible internal cavity instead of being sealed.
#
# Articulation:
#   - lift cap: PRISMATIC along +Z. Default q=0 is the closed/seated position
#     where the white cap sits down over the neck, contacting the body deck.
#     Sliding to +CAP_LIFT raises the cap vertically above the mouth, revealing
#     the open bore.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
BODY_RADIUS = 0.0180  # constant cylindrical barrel radius
BODY_TOP_Z = 0.104
BODY_BOTTOM_Z = 0.0

NECK_BASE_Z = BODY_TOP_Z
NECK_TOP_Z = 0.125
NECK_R = 0.0088
NECK_INNER_R = 0.0051
MOUTH_LIP_R = 0.0100
CAVITY_BOTTOM_Z = 0.035

CAP_LIFT = 0.028
# Articulation origin: where the cap sits when closed (q=0).
# The cap skirt extends slightly below this to contact the body deck.
CAP_ORIGIN_Z = NECK_BASE_Z + 0.001  # 0.105
CAP_HEIGHT = 0.026


def _tube_body() -> cq.Workplane:
    """Straight cylindrical barrel of constant round radius with a softly
    rounded bottom and a flat top deck."""
    body_height = BODY_TOP_Z - BODY_BOTTOM_Z
    # Main cylinder
    body = (
        cq.Workplane("XY")
        .workplane(offset=BODY_BOTTOM_Z + 0.006)
        .circle(BODY_RADIUS)
        .extrude(body_height - 0.006)
    )
    # Rounded bottom cap — a short tapered base
    bottom_taper = (
        cq.Workplane("XY")
        .workplane(offset=BODY_BOTTOM_Z)
        .circle(BODY_RADIUS * 0.88)
        .workplane(offset=0.006)
        .circle(BODY_RADIUS)
        .loft()
    )
    body = body.union(bottom_taper)

    # Internal cavity bore (visible from above through the neck opening)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=CAVITY_BOTTOM_Z)
        .circle(NECK_INNER_R * 1.10)
        .extrude(BODY_TOP_Z - CAVITY_BOTTOM_Z + 0.004)
    )
    return body.cut(cavity)


def _open_neck() -> cq.Workplane:
    """Blue raised neck with an actual through-bore; the inner cylindrical wall is
    visible from above and connects to the cavity cut in the body."""
    h = NECK_TOP_Z - NECK_BASE_Z
    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BASE_Z)
        .circle(NECK_R)
        .circle(NECK_INNER_R)
        .extrude(h)
    )
    lip = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - 0.0032)
        .circle(MOUTH_LIP_R)
        .circle(NECK_INNER_R)
        .extrude(0.0032)
    )
    lower_ring = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BASE_Z + 0.003)
        .circle(NECK_R + 0.0012)
        .circle(NECK_INNER_R)
        .extrude(0.0024)
    )
    return neck.union(lip).union(lower_ring)


def _cap() -> cq.Workplane:
    """Vertical lift cap with circular cross-section.
    In the cap's local frame, the articulation origin is at local z=0.
    At q=0 (closed), the cap origin is at CAP_ORIGIN_Z (0.105) in world.
    The skirt extends slightly below the origin to seat on the body deck.
    At q=+CAP_LIFT (open), the cap is raised, revealing the bore."""
    cap_r = BODY_RADIUS * 0.94  # ~0.01692
    inner_r = BODY_RADIUS * 0.56  # ~0.01008, slightly larger than MOUTH_LIP_R

    # Outer shell: skirt from slightly below origin to dome top
    skirt_bottom = -0.002  # 2mm below articulation origin; seats on body deck at q=0
    skirt_top = 0.015
    dome_top = CAP_HEIGHT - 0.002  # 0.024

    cap = (
        cq.Workplane("XY")
        .workplane(offset=skirt_bottom)
        .circle(cap_r)
        .workplane(offset=(skirt_top - skirt_bottom))
        .circle(cap_r)
        .workplane(offset=(dome_top - skirt_top))
        .circle(cap_r * 0.90)
        .loft(ruled=False)
    )

    # Inner hollow: breaks through the bottom so the cap is an open-bottom
    # shell. This produces one connected mesh (outer wall → dome → inner wall).
    inner_bottom = skirt_bottom - 0.003
    inner_top = dome_top - 0.003
    inner = (
        cq.Workplane("XY")
        .workplane(offset=inner_bottom)
        .circle(inner_r)
        .workplane(offset=(skirt_top - inner_bottom))
        .circle(inner_r)
        .workplane(offset=(inner_top - skirt_top))
        .circle(inner_r * 0.65)
        .loft(ruled=False)
    )
    return cap.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sunscreen_squeeze_tube")

    blue = model.material("tube_blue", rgba=(0.62, 0.80, 0.92, 1.0))
    white = model.material("cap_white", rgba=(0.95, 0.96, 0.97, 1.0))
    label_white = model.material("label_white", rgba=(1.0, 1.0, 1.0, 1.0))

    # ---- body (root): cylindrical blue barrel + open neck ----
    body = model.part("body")
    body.visual(mesh_from_cadquery(_tube_body(), "tube_body"), material=blue, name="tube_body")
    body.visual(mesh_from_cadquery(_open_neck(), "open_neck"), material=blue, name="open_neck")

    # Minimal raised white label marks on the front face of the cylindrical body
    for i in range(8):
        body.visual(
            Box((0.0007, 0.0023, 0.0060)),
            origin=Origin(
                xyz=(-BODY_RADIUS - 0.00035, 0.0, 0.074 - i * 0.0065)
            ),
            material=label_white,
            name=f"vertical_brand_mark_{i}",
        )
    for i in range(3):
        body.visual(
            Box((0.0007, 0.013, 0.0012)),
            origin=Origin(xyz=(-BODY_RADIUS - 0.00035, 0.0, 0.065 - i * 0.010)),
            material=label_white,
            name=f"small_front_label_{i}",
        )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_RADIUS, length=0.126),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
    )

    # ---- lift cap: prismatic vertical motion over the open mouth ----
    cap = model.part("lift_cap")
    cap.visual(mesh_from_cadquery(_cap(), "cap_shell"), material=white, name="cap_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_RADIUS * 0.94, length=CAP_HEIGHT),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT * 0.4)),
    )

    # Vertical slide: q=0 is closed/seated; q=+CAP_LIFT raises the cap.
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5, velocity=0.8, lower=0.0, upper=CAP_LIFT
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("lift_cap")
    lift = object_model.get_articulation("cap_lift")

    # --- Cylindrical barrel body: near-equal X and Y extents ---
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "tube body is a near-circular cylinder (X and Y extents nearly equal)",
        abs(body_ext[0] - body_ext[1]) < 0.008,
        details=f"body extents (x,y,z)={body_ext}",
    )
    ctx.check(
        "tube is tall (stands upright)",
        body_ext[2] > 0.11,
        details=f"body extents={body_ext}",
    )
    tube_aabb = ctx.part_element_world_aabb(body, elem="tube_body")
    tube_x = tube_aabb[1][0] - tube_aabb[0][0]
    tube_y = tube_aabb[1][1] - tube_aabb[0][1]
    ctx.check(
        "blue cylindrical barrel has near-equal width and depth",
        abs(tube_x - tube_y) < 0.006 and 0.030 < tube_x < 0.042,
        details=f"tube_body x-depth={tube_x}, y-width={tube_y}",
    )

    # The closed cap seats on the body deck with a small embed for visual contact.
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="tube_body",
        reason="Closed cap skirt seats slightly into the body top deck for visual contact.",
    )
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="open_neck",
        reason="Closed cap wraps around the open neck when seated.",
    )

    # --- Open mouth: visible bore into the internal cavity ---
    neck_aabb = ctx.part_element_world_aabb(body, elem="open_neck")
    neck_ext = _ext(neck_aabb)
    ctx.check(
        "raised blue neck is annular with a visible open bore",
        NECK_INNER_R > 0.0045 and NECK_R - NECK_INNER_R > 0.003,
        details=f"neck outer_r={NECK_R}, inner_r={NECK_INNER_R}",
    )
    ctx.check(
        "neck bore continues down into the body cavity",
        CAVITY_BOTTOM_Z < BODY_TOP_Z - 0.050,
        details=f"cavity_bottom_z={CAVITY_BOTTOM_Z}, body_top_z={BODY_TOP_Z}",
    )
    ctx.check(
        "open neck protrudes above the blue top deck",
        neck_ext[2] > 0.018 and neck_aabb[1][2] > BODY_TOP_Z,
        details=f"neck extents={neck_ext}, neck aabb={neck_aabb}",
    )

    # --- Default pose (q=0) is closed: cap is seated on the body ---
    with ctx.pose({lift: 0.0}):
        closed_cap_aabb = ctx.part_world_aabb(cap)
        closed_bottom_z = closed_cap_aabb[0][2]
        closed_center_x = (closed_cap_aabb[0][0] + closed_cap_aabb[1][0]) * 0.5
        closed_center_y = (closed_cap_aabb[0][1] + closed_cap_aabb[1][1]) * 0.5
        # Cap should be near the body top deck when closed
        ctx.check(
            "closed cap sits low over the neck, near body deck",
            closed_bottom_z < NECK_TOP_Z and closed_bottom_z > BODY_TOP_Z - 0.005,
            details=f"closed cap bottom z={closed_bottom_z}",
        )
        ctx.expect_overlap(
            cap, body, axes="xy", elem_a="cap_shell", elem_b="open_neck",
            min_overlap=0.004, name="closed cap footprint covers the open mouth",
        )

    # --- Open pose: cap raised to reveal the bore ---
    with ctx.pose({lift: CAP_LIFT}):
        open_cap_aabb = ctx.part_world_aabb(cap)
        open_bottom_z = open_cap_aabb[0][2]
        open_center_x = (open_cap_aabb[0][0] + open_cap_aabb[1][0]) * 0.5
        open_center_y = (open_cap_aabb[0][1] + open_cap_aabb[1][1]) * 0.5
        ctx.check(
            "open cap is raised above the neck top, revealing the bore",
            open_bottom_z > NECK_TOP_Z + 0.004,
            details=f"open cap bottom z={open_bottom_z}, neck top={NECK_TOP_Z}",
        )

    # --- Cap motion is purely vertical translation ---
    ctx.check(
        "cap opens by translating straight upward, not rotating",
        open_bottom_z > closed_bottom_z + CAP_LIFT * 0.8
        and abs(open_center_x - closed_center_x) < 1e-6
        and abs(open_center_y - closed_center_y) < 1e-6,
        details=f"open_bottom={open_bottom_z}, closed_bottom={closed_bottom_z}, "
        f"open_center=({open_center_x}, {open_center_y}), "
        f"closed_center=({closed_center_x}, {closed_center_y})",
    )
    ctx.check(
        "cap lift is prismatic about +Z with lower=0 (closed) to upper=CAP_LIFT (open)",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift.axis) == (0.0, 0.0, 1.0)
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper > 0.0,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
