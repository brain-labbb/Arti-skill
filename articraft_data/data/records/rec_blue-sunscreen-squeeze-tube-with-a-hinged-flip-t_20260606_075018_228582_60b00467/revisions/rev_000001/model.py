from __future__ import annotations

# Blue sunscreen squeeze tube ("MOODYSERUM / Airy sunstick") with a vertical lift cap.
#
# Frame:
#   - Tube stands upright; body axis along +Z.
#   - z=0 is the softly rounded bottom; the bottle rises in +Z as a blue
#     rounded-rectangle slab like the reference image.
#   - The top is blue with a round raised neck and an annular open mouth. The
#     bore continues down into a visible internal cavity instead of being sealed.
#
# Articulation:
#   - lift cap: PRISMATIC along +Z. Default q=0 leaves the white cap raised
#     vertically above the mouth so the bore is visible; sliding to -CAP_LIFT
#     lowers it straight down over the neck.

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
TUBE_HALF_W = 0.0265  # half width in Y at the widest point
TUBE_HALF_T = 0.0140  # half thickness in X; rounded-rectangle slab depth
BODY_TOP_Z = 0.104
BODY_BOTTOM_Z = 0.0

NECK_BASE_Z = BODY_TOP_Z
NECK_TOP_Z = 0.125
NECK_R = 0.0088
NECK_INNER_R = 0.0051
MOUTH_LIP_R = 0.0100
CAVITY_BOTTOM_Z = 0.035

CAP_LIFT = 0.028
CAP_CLOSED_BOTTOM_Z = NECK_BASE_Z + 0.001
CAP_HEIGHT = 0.026


def _oval_loop(z: float, half_t: float, half_w: float, n: int = 48):
    # A rounded-rectangle/superellipse closed loop in the XY plane at height z.
    # Wide along Y (half_w), thin along X (half_t). Exponent >2 gives the
    # softened rectangular sunscreen-bottle silhouette from the reference.
    pts = []
    p = 3.4
    for i in range(n):
        a = 2.0 * math.pi * i / n
        c = math.cos(a)
        s = math.sin(a)
        x = half_t * math.copysign(abs(c) ** (2.0 / p), c)
        y = half_w * math.copysign(abs(s) ** (2.0 / p), s)
        pts.append((x, y, z))
    return pts


def _loop_xy(half_t: float, half_w: float):
    return [(p[0], p[1]) for p in _oval_loop(0.0, half_t, half_w)]


def _tube_body() -> cq.Workplane:
    # Rounded-rectangle body: softly pinched at the bottom, broad vertical
    # sides, and a flat blue top deck like the open product in the reference.
    sections = [
        (BODY_BOTTOM_Z, TUBE_HALF_T * 0.78, TUBE_HALF_W * 0.92),
        (0.010, TUBE_HALF_T * 0.96, TUBE_HALF_W * 0.99),
        (0.055, TUBE_HALF_T, TUBE_HALF_W),
        (0.092, TUBE_HALF_T * 0.99, TUBE_HALF_W * 0.99),
        (BODY_TOP_Z, TUBE_HALF_T * 0.94, TUBE_HALF_W * 0.96),
    ]
    wp = cq.Workplane("XY")
    prev = 0.0
    first = True
    for z, ht, hw in sections:
        off = z if first else z - prev
        wp = wp.workplane(offset=off)
        wp = wp.polyline(_loop_xy(ht, hw)).close()
        prev = z
        first = False
    body = wp.loft(ruled=False)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=CAVITY_BOTTOM_Z)
        .circle(NECK_INNER_R * 1.10)
        .extrude(BODY_TOP_Z - CAVITY_BOTTOM_Z + 0.004)
    )
    return body.cut(cavity)


def _open_neck() -> cq.Workplane:
    # Blue raised neck with an actual through-bore; the inner cylindrical wall is
    # visible from above and connects to the cavity cut in the body.
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
    # Vertical lift cap in the joint-local frame. The cap is centered over the
    # mouth and translated upward by CAP_LIFT at q=0; q=-CAP_LIFT lowers it
    # straight down over the open neck.
    skirt_bottom = CAP_LIFT
    skirt_top = CAP_LIFT + 0.016
    dome_top = CAP_LIFT + CAP_HEIGHT

    cap = (
        cq.Workplane("XY")
        .workplane(offset=skirt_bottom)
        .polyline(_loop_xy(0.0150, TUBE_HALF_W * 0.94))
        .close()
        .workplane(offset=(skirt_top - skirt_bottom))
        .polyline(_loop_xy(0.0150, TUBE_HALF_W * 0.94))
        .close()
        .workplane(offset=(dome_top - skirt_top))
        .polyline(_loop_xy(0.0130, TUBE_HALF_W * 0.84))
        .close()
        .loft(ruled=False)
    )

    inner = (
        cq.Workplane("XY")
        .workplane(offset=skirt_bottom - 0.002)
        .polyline(_loop_xy(0.0112, TUBE_HALF_W * 0.78))
        .close()
        .workplane(offset=(skirt_top - skirt_bottom + 0.002))
        .polyline(_loop_xy(0.0112, TUBE_HALF_W * 0.78))
        .close()
        .workplane(offset=(dome_top - skirt_top - 0.004))
        .polyline(_loop_xy(0.0065, TUBE_HALF_W * 0.52))
        .close()
        .loft(ruled=False)
    )
    return cap.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sunscreen_squeeze_tube")

    blue = model.material("tube_blue", rgba=(0.62, 0.80, 0.92, 1.0))
    white = model.material("cap_white", rgba=(0.95, 0.96, 0.97, 1.0))
    label_white = model.material("label_white", rgba=(1.0, 1.0, 1.0, 1.0))

    # ---- body (root): rounded blue slab + open neck ----
    body = model.part("body")
    body.visual(mesh_from_cadquery(_tube_body(), "tube_body"), material=blue, name="tube_body")
    body.visual(mesh_from_cadquery(_open_neck(), "open_neck"), material=blue, name="open_neck")
    # Minimal raised white label marks on the front face, echoing the reference
    # without relying on text geometry support.
    for i in range(8):
        body.visual(
            Box((0.0007, 0.0023, 0.0060)),
            origin=Origin(
                xyz=(-TUBE_HALF_T - 0.00035, TUBE_HALF_W * 0.52, 0.074 - i * 0.0065)
            ),
            material=label_white,
            name=f"vertical_brand_mark_{i}",
        )
    for i in range(3):
        body.visual(
            Box((0.0007, 0.013, 0.0012)),
            origin=Origin(xyz=(-TUBE_HALF_T - 0.00035, -0.002, 0.065 - i * 0.010)),
            material=label_white,
            name=f"small_front_label_{i}",
        )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.126),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
    )

    # ---- lift cap: prismatic vertical motion over the open mouth ----
    cap = model.part("lift_cap")
    cap.visual(mesh_from_cadquery(_cap(), "cap_shell"), material=white, name="cap_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.026),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, CAP_LIFT + CAP_HEIGHT * 0.5)),
    )

    # Vertical slide frame: q=0 is raised/open; q=-CAP_LIFT lowers the cap
    # directly down over the neck. No rotational hinge is used.
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_CLOSED_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5, velocity=0.8, lower=-CAP_LIFT, upper=0.0
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

    # --- Rounded-rectangle blue body, matching the reference product family ---
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "tube body is a rounded-rectangle slab (wider in Y than thick in X)",
        body_ext[1] > body_ext[0] + 0.015,
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
        "blue body has broad rounded front and narrow side depth",
        0.025 < tube_x < 0.034 and tube_y > 0.048,
        details=f"tube_body x-depth={tube_x}, y-width={tube_y}",
    )

    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="open_neck",
        reason="Vertical lift cap intentionally slides down over the open neck when closed.",
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

    # --- Default pose is open, so the mouth is visible without moving the joint ---
    with ctx.pose({lift: 0.0}):
        open_cap_aabb = ctx.part_world_aabb(cap)
        ctx.check(
            "default lift cap is raised vertically above the bottle mouth",
            open_cap_aabb[0][2] > NECK_TOP_Z + 0.006
            and open_cap_aabb[0][0] < neck_aabb[0][0]
            and open_cap_aabb[1][0] > neck_aabb[1][0]
            and open_cap_aabb[0][1] < neck_aabb[0][1]
            and open_cap_aabb[1][1] > neck_aabb[1][1],
            details=f"open cap aabb={open_cap_aabb}, neck aabb={neck_aabb}",
        )
        open_bottom_z = open_cap_aabb[0][2]
        open_center_x = (open_cap_aabb[0][0] + open_cap_aabb[1][0]) * 0.5
        open_center_y = (open_cap_aabb[0][1] + open_cap_aabb[1][1]) * 0.5

    # --- Cap slides straight down to close over the open neck ---
    with ctx.pose({lift: -CAP_LIFT}):
        closed_cap_aabb = ctx.part_world_aabb(cap)
        closed_bottom_z = closed_cap_aabb[0][2]
        closed_center_x = (closed_cap_aabb[0][0] + closed_cap_aabb[1][0]) * 0.5
        closed_center_y = (closed_cap_aabb[0][1] + closed_cap_aabb[1][1]) * 0.5
        ctx.expect_overlap(
            cap, body, axes="xy", elem_a="cap_shell", elem_b="open_neck",
            min_overlap=0.004, name="closed cap footprint covers the open mouth",
        )

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
        "cap lift is prismatic about +Z with an open-to-closed range",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift.axis) == (0.0, 0.0, 1.0)
        and lift.motion_limits.lower < 0.0
        and lift.motion_limits.upper == 0.0,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
