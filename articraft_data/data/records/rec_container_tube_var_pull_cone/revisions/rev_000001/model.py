from __future__ import annotations

# Blue sunscreen squeeze tube fork: pointed conical dispensing nozzle tip
# capped by a small pointed pull-cap that detaches straight up via a PRISMATIC
# pull-off joint.
#
# Frame:
#   - Tube stands upright; body axis along +Z.
#   - z=0 is the softly rounded bottom; the bottle rises in +Z as a blue
#     rounded-rectangle slab (unchanged from parent).
#   - The top has a pointed conical dispensing nozzle with a through-bore
#     connecting to the internal cavity.
#
# Articulation:
#   - pull_cap: PRISMATIC along +Z. Default q=0 leaves the cap seated on the
#     nozzle; sliding to q=PULL_LIFT pulls it straight up to expose the nozzle.

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

# Internal bore cavity (unchanged from parent)
NECK_INNER_R = 0.0051
CAVITY_BOTTOM_Z = 0.035

# Conical dispensing nozzle
NOZZLE_BASE_Z = BODY_TOP_Z
NOZZLE_BASE_R = 0.0080
NOZZLE_TIP_R = 0.0015
NOZZLE_HEIGHT = 0.024
NOZZLE_TOP_Z = NOZZLE_BASE_Z + NOZZLE_HEIGHT
NOZZLE_BORE_R = 0.0020
COLLAR_HEIGHT = 0.003
COLLAR_R = NOZZLE_BASE_R + 0.0015

# Pointed pull-cap
CAP_BASE_R = NOZZLE_BASE_R + 0.003
CAP_TIP_R = 0.0018
CAP_HEIGHT = NOZZLE_HEIGHT + 0.004
PULL_LIFT = 0.034

# Joint origin at the nozzle base contact surface
JOINT_Z = NOZZLE_BASE_Z


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
    # sides, and a flat blue top deck (unchanged from parent).
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


def _nozzle_tip() -> cq.Workplane:
    # Pointed conical dispensing nozzle with a through-bore connecting to the
    # internal cavity. Built from a base collar and a tapered cone section.
    # Collar ring at the nozzle base
    collar = (
        cq.Workplane("XY")
        .workplane(offset=NOZZLE_BASE_Z)
        .circle(COLLAR_R)
        .circle(NOZZLE_BORE_R)
        .extrude(COLLAR_HEIGHT)
    )
    # Tapered cone from collar top to pointed nozzle tip
    cone_base_z = NOZZLE_BASE_Z + COLLAR_HEIGHT
    cone_height = NOZZLE_TOP_Z - cone_base_z
    cone = (
        cq.Workplane("XY")
        .workplane(offset=cone_base_z)
        .circle(NOZZLE_BASE_R)
        .workplane(offset=cone_height)
        .circle(NOZZLE_TIP_R)
        .loft()
    )
    # Through bore connecting nozzle to body cavity
    bore = (
        cq.Workplane("XY")
        .workplane(offset=NOZZLE_BASE_Z - 0.005)
        .circle(NOZZLE_BORE_R)
        .extrude(NOZZLE_HEIGHT + COLLAR_HEIGHT + 0.01)
    )
    return collar.union(cone).cut(bore)


def _pull_cap() -> cq.Workplane:
    # Small pointed pull-cap in the joint-local frame. The cap origin is at
    # z=0 (the seating surface at the nozzle base). The cap extends upward
    # as a hollow pointed cone that fits over the nozzle.
    # Outer pointed cone shell with a wider base lip for grip readability
    outer = (
        cq.Workplane("XY")
        .circle(CAP_BASE_R + 0.0012)  # wider base lip
        .workplane(offset=0.005)
        .circle(CAP_BASE_R)  # transition to main cone
        .workplane(offset=CAP_HEIGHT * 0.55)
        .circle(CAP_BASE_R * 0.42)
        .workplane(offset=CAP_HEIGHT * 0.4)
        .circle(CAP_TIP_R)
        .loft()
    )
    # Through-bore from bottom to near-top, leaving a small solid tip
    # This creates a connected hollow shell instead of a disconnected blind hole
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(NOZZLE_BASE_R + 0.001)
        .workplane(offset=NOZZLE_HEIGHT + 0.002)
        .circle(NOZZLE_TIP_R + 0.001)
        .loft()
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sunscreen_squeeze_tube_nozzle")

    blue = model.material("tube_blue", rgba=(0.62, 0.80, 0.92, 1.0))
    cap_orange = model.material("cap_orange", rgba=(0.95, 0.65, 0.25, 1.0))
    label_white = model.material("label_white", rgba=(1.0, 1.0, 1.0, 1.0))

    # ---- body (root): rounded blue slab + conical nozzle tip ----
    body = model.part("body")
    body.visual(mesh_from_cadquery(_tube_body(), "tube_body"), material=blue, name="tube_body")
    body.visual(mesh_from_cadquery(_nozzle_tip(), "nozzle_tip"), material=blue, name="nozzle_tip")
    # Minimal raised white label marks on the front face, embedded slightly
    # into the body surface for geometric connectivity.
    for i in range(8):
        body.visual(
            Box((0.0010, 0.0023, 0.0060)),
            origin=Origin(
                xyz=(-TUBE_HALF_T + 0.0001, TUBE_HALF_W * 0.52, 0.074 - i * 0.0065)
            ),
            material=label_white,
            name=f"vertical_brand_mark_{i}",
        )
    for i in range(3):
        body.visual(
            Box((0.0010, 0.013, 0.0012)),
            origin=Origin(xyz=(-TUBE_HALF_T + 0.0001, -0.002, 0.065 - i * 0.010)),
            material=label_white,
            name=f"small_front_label_{i}",
        )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.126),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
    )

    # ---- pull cap: prismatic pull-off motion over the nozzle tip ----
    cap = model.part("pull_cap")
    cap.visual(mesh_from_cadquery(_pull_cap(), "cap_shell"), material=cap_orange, name="cap_shell")
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=CAP_HEIGHT),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT * 0.5)),
    )

    # Prismatic pull-off joint: q=0 is seated/closed; q=PULL_LIFT pulls the
    # cap straight up to expose the pointed nozzle.
    model.articulation(
        "cap_pull",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5, velocity=0.8, lower=0.0, upper=PULL_LIFT
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("pull_cap")
    pull = object_model.get_articulation("cap_pull")

    # --- Rounded-rectangle blue body (unchanged from parent) ---
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

    # --- Internal bore cavity (unchanged from parent) ---
    ctx.check(
        "nozzle bore continues down into the body cavity",
        CAVITY_BOTTOM_Z < BODY_TOP_Z - 0.050,
        details=f"cavity_bottom_z={CAVITY_BOTTOM_Z}, body_top_z={BODY_TOP_Z}",
    )

    # --- Pointed conical nozzle tip ---
    nozzle_aabb = ctx.part_element_world_aabb(body, elem="nozzle_tip")
    nozzle_ext = _ext(nozzle_aabb)
    ctx.check(
        "nozzle tip protrudes above the blue top deck",
        nozzle_ext[2] > 0.020 and nozzle_aabb[1][2] > BODY_TOP_Z,
        details=f"nozzle extents={nozzle_ext}, nozzle aabb={nozzle_aabb}",
    )
    ctx.check(
        "nozzle is narrow and pointed (small XY footprint vs height)",
        nozzle_ext[0] < 0.025 and nozzle_ext[1] < 0.025 and nozzle_ext[2] > nozzle_ext[0] * 0.8,
        details=f"nozzle extents={nozzle_ext}",
    )
    ctx.check(
        "nozzle has a dispensing bore (small bore radius)",
        NOZZLE_BORE_R > 0.001 and NOZZLE_BORE_R < NOZZLE_BASE_R * 0.4,
        details=f"bore_r={NOZZLE_BORE_R}, base_r={NOZZLE_BASE_R}",
    )
    ctx.check(
        "nozzle tapers to a pointed tip (tip much smaller than base)",
        NOZZLE_TIP_R < NOZZLE_BASE_R * 0.3,
        details=f"tip_r={NOZZLE_TIP_R}, base_r={NOZZLE_BASE_R}",
    )

    # --- Pull-cap seating and pull-off overlap ---
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_shell", elem_b="nozzle_tip",
        reason="Pull-cap inner bore intentionally fits over the conical nozzle when seated.",
    )

    # --- Default pose (q=0): cap is seated on the nozzle ---
    with ctx.pose({pull: 0.0}):
        seated_cap_aabb = ctx.part_world_aabb(cap)
        ctx.expect_overlap(
            cap, body, axes="xy", elem_a="cap_shell", elem_b="nozzle_tip",
            min_overlap=0.004, name="seated cap footprint covers the nozzle",
        )
        ctx.check(
            "seated cap bottom is at the nozzle base contact surface",
            seated_cap_aabb[0][2] < NOZZLE_BASE_Z + 0.003,
            details=f"seated cap bottom z={seated_cap_aabb[0][2]}, nozzle_base={NOZZLE_BASE_Z}",
        )
        seated_center_x = (seated_cap_aabb[0][0] + seated_cap_aabb[1][0]) * 0.5
        seated_center_y = (seated_cap_aabb[0][1] + seated_cap_aabb[1][1]) * 0.5

    # --- Pulled pose (q=PULL_LIFT): cap is pulled straight up ---
    with ctx.pose({pull: PULL_LIFT}):
        pulled_cap_aabb = ctx.part_world_aabb(cap)
        pulled_bottom_z = pulled_cap_aabb[0][2]
        pulled_center_x = (pulled_cap_aabb[0][0] + pulled_cap_aabb[1][0]) * 0.5
        pulled_center_y = (pulled_cap_aabb[0][1] + pulled_cap_aabb[1][1]) * 0.5
        ctx.check(
            "pulled cap clears the nozzle tip vertically",
            pulled_bottom_z > NOZZLE_TOP_Z + 0.002,
            details=f"pulled cap bottom z={pulled_bottom_z}, nozzle_top={NOZZLE_TOP_Z}",
        )

    # --- Cap moves straight up (prismatic, no lateral drift) ---
    seated_bottom_z = seated_cap_aabb[0][2]
    ctx.check(
        "cap opens by translating straight upward, not rotating",
        pulled_bottom_z > seated_bottom_z + PULL_LIFT * 0.8
        and abs(pulled_center_x - seated_center_x) < 1e-6
        and abs(pulled_center_y - seated_center_y) < 1e-6,
        details=f"seated_bottom={seated_bottom_z}, pulled_bottom={pulled_bottom_z}, "
        f"seated_center=({seated_center_x}, {seated_center_y}), "
        f"pulled_center=({pulled_center_x}, {pulled_center_y})",
    )
    ctx.check(
        "cap pull is prismatic about +Z with seated-to-pulled range",
        pull.articulation_type == ArticulationType.PRISMATIC
        and tuple(pull.axis) == (0.0, 0.0, 1.0)
        and pull.motion_limits.lower == 0.0
        and pull.motion_limits.upper > 0.0,
        details=f"axis={pull.axis}, limits={pull.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
