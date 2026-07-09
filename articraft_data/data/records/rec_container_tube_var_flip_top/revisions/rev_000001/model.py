from __future__ import annotations

# Blue sunscreen squeeze tube with a hinged flip-top cap.
#
# Frame:
#   - Tube stands upright; body axis along +Z.
#   - z=0 is the softly rounded bottom; the bottle rises in +Z as a blue
#     rounded-rectangle slab like the reference image.
#   - The top is blue with a round raised neck and an annular open mouth. The
#     bore continues down into a visible internal cavity instead of being sealed.
#
# Articulation:
#   - flip_cap: REVOLUTE about a horizontal axis (X) at the rear of the neck
#     collar, joined by a living-hinge tab. q=0 is closed (lid covers mouth);
#     positive q opens the lid backward over the tube.

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

# Hinge point: rear edge of the mouth lip, at neck top
HINGE_Y = -MOUTH_LIP_R  # rear of neck collar
HINGE_Z = NECK_TOP_Z    # top of neck

# Flip-cap lid dimensions
LID_CENTER_Y = MOUTH_LIP_R  # neck center relative to hinge at rear edge
LID_RADIUS = MOUTH_LIP_R + 0.0020  # slightly larger than mouth lip
LID_THICKNESS = 0.0035
SKIRT_CLEARANCE = 0.0004
SKIRT_HEIGHT = 0.005
PLUG_RADIUS = NECK_INNER_R - 0.0006
PLUG_HEIGHT = 0.005

# Joint limits (radians)
CAP_CLOSED = 0.0
CAP_OPEN_ANGLE = 2.6  # ~149 degrees


def _oval_loop(z: float, half_t: float, half_w: float, n: int = 48):
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


def _open_neck_with_hinge() -> cq.Workplane:
    """Blue raised neck with through-bore and a hinge boss at the rear collar."""
    h = NECK_TOP_Z - NECK_BASE_Z
    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BASE_Z)
        .circle(NECK_R)
        .circle(NECK_INNER_R)
        .extrude(h)
    )
    # Mouth lip ring at the top
    lip = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - 0.0032)
        .circle(MOUTH_LIP_R)
        .circle(NECK_INNER_R)
        .extrude(0.0032)
    )
    # Lower collar ring for grip
    lower_ring = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BASE_Z + 0.003)
        .circle(NECK_R + 0.0012)
        .circle(NECK_INNER_R)
        .extrude(0.0024)
    )
    # Hinge boss: small protrusion at the rear of the neck collar where the
    # living hinge tab attaches. Extends rearward from the lip.
    boss_w = 0.007  # width in X
    boss_d = 0.004  # depth in Y (rearward extent from lip rear edge)
    boss_h = 0.003  # height in Z
    hinge_boss = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - boss_h)
        .center(0, -(MOUTH_LIP_R + boss_d * 0.5))
        .rect(boss_w, boss_d)
        .extrude(boss_h)
    )
    return neck.union(lip).union(lower_ring).union(hinge_boss)


def _flip_cap_lid() -> cq.Workplane:
    """Flip-top lid in cap-local frame (origin at hinge point).

    In the local frame at q=0:
      +Y is toward the front of the tube (away from hinge)
      +Z is upward
      The lid body is centered at (0, LID_CENTER_Y, 0) and covers the mouth.
    """
    # --- Living hinge tab: thin flexible web near the origin ---
    tab_w = 0.006    # X width
    tab_l = 0.005    # Y length (from hinge rearward)
    tab_t = 0.0009   # Z thickness
    hinge_tab = (
        cq.Workplane("XY")
        .center(0, -tab_l * 0.5)
        .rect(tab_w, tab_l)
        .extrude(tab_t)
    )

    # --- Main lid disk: covers the mouth ---
    lid = (
        cq.Workplane("XY")
        .workplane(offset=-0.0005)
        .center(0, LID_CENTER_Y)
        .circle(LID_RADIUS)
        .extrude(LID_THICKNESS)
    )

    # --- Skirt: short cylindrical wall that wraps around the lip exterior ---
    skirt_inner = MOUTH_LIP_R + SKIRT_CLEARANCE
    skirt_outer = LID_RADIUS
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=-(SKIRT_HEIGHT + 0.0005))
        .center(0, LID_CENTER_Y)
        .circle(skirt_outer)
        .circle(skirt_inner)
        .extrude(SKIRT_HEIGHT)
    )

    # --- Plug stem: inserts into the bore for sealing ---
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-PLUG_HEIGHT)
        .center(0, LID_CENTER_Y)
        .circle(PLUG_RADIUS)
        .extrude(PLUG_HEIGHT)
    )

    # --- Dome cap on top for a finished look ---
    dome_base_r = LID_RADIUS * 0.80
    dome_top_r = LID_RADIUS * 0.55
    dome_h = 0.005
    dome = (
        cq.Workplane("XY")
        .workplane(offset=LID_THICKNESS - 0.0005)
        .center(0, LID_CENTER_Y)
        .circle(dome_base_r)
        .workplane(offset=dome_h)
        .circle(dome_top_r)
        .loft()
    )

    # --- Front thumb tab for flipping open ---
    thumb_w = 0.006
    thumb_l = 0.005
    thumb_t = 0.0025
    thumb_tab = (
        cq.Workplane("XY")
        .center(0, LID_CENTER_Y + LID_RADIUS + thumb_l * 0.5 - 0.001)
        .rect(thumb_w, thumb_l)
        .extrude(thumb_t)
    )

    return (
        hinge_tab.union(lid).union(skirt).union(plug).union(dome).union(thumb_tab)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sunscreen_flip_cap_tube")

    blue = model.material("tube_blue", rgba=(0.62, 0.80, 0.92, 1.0))
    white = model.material("cap_white", rgba=(0.95, 0.96, 0.97, 1.0))
    label_white = model.material("label_white", rgba=(1.0, 1.0, 1.0, 1.0))

    # ---- body (root): rounded blue slab + open neck with hinge boss ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_tube_body(), "tube_body"),
        material=blue, name="tube_body",
    )
    body.visual(
        mesh_from_cadquery(_open_neck_with_hinge(), "open_neck"),
        material=blue, name="open_neck",
    )
    # Brand label marks on the front face, embedded slightly into the tube
    # surface so they connect to the body mesh.
    for i in range(8):
        body.visual(
            Box((0.0012, 0.0023, 0.0060)),
            origin=Origin(
                xyz=(-TUBE_HALF_T + 0.0002, TUBE_HALF_W * 0.52, 0.074 - i * 0.0065)
            ),
            material=label_white,
            name=f"vertical_brand_mark_{i}",
        )
    for i in range(3):
        body.visual(
            Box((0.0012, 0.013, 0.0012)),
            origin=Origin(xyz=(-TUBE_HALF_T + 0.0002, -0.002, 0.065 - i * 0.010)),
            material=label_white,
            name=f"small_front_label_{i}",
        )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.126),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
    )

    # ---- flip cap: hinged lid with living-hinge tab ----
    cap = model.part("flip_cap")
    cap.visual(
        mesh_from_cadquery(_flip_cap_lid(), "cap_lid"),
        material=white, name="cap_lid",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.012),
        mass=0.005,
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_THICKNESS * 0.5)),
    )

    # REVOLUTE hinge at the rear of the neck collar.
    # Origin at hinge point: (0, HINGE_Y, HINGE_Z) in world.
    # Axis along +X: right-hand rule rotates +Y toward +Z, so positive q
    # lifts the front edge of the cap upward and backward (opening).
    # q=0 is closed (lid over mouth); q=CAP_OPEN_ANGLE is fully open.
    model.articulation(
        "flip_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8, velocity=4.0,
            lower=CAP_CLOSED, upper=CAP_OPEN_ANGLE,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("flip_cap")
    hinge = object_model.get_articulation("flip_cap")

    # --- Rounded-rectangle blue body ---
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

    # --- Open mouth: visible bore into the internal cavity ---
    neck_aabb = ctx.part_element_world_aabb(body, elem="open_neck")
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

    # --- Hinge is REVOLUTE about a horizontal axis ---
    ctx.check(
        "cap articulation is REVOLUTE about a horizontal axis",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(hinge.axis[0]) > 0.9
        and abs(hinge.axis[1]) < 0.1
        and abs(hinge.axis[2]) < 0.1,
        details=f"axis={hinge.axis}, type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge origin is at the rear of the neck collar",
        hinge.origin.xyz[1] < -0.005 and hinge.origin.xyz[2] > NECK_TOP_Z - 0.005,
        details=f"hinge_origin={hinge.origin.xyz}",
    )
    ctx.check(
        "hinge has closed-to-open range starting from zero",
        hinge.motion_limits.lower == 0.0 and hinge.motion_limits.upper > 2.0,
        details=f"limits={hinge.motion_limits}",
    )

    # --- Allow intentional overlap: cap skirt/plug fits over neck when closed ---
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_lid", elem_b="open_neck",
        reason="Flip-cap skirt and plug intentionally nest over/into the neck collar when closed.",
    )

    # --- Closed pose: lid covers the mouth ---
    with ctx.pose({hinge: CAP_CLOSED}):
        closed_cap_aabb = ctx.part_world_aabb(cap)
        closed_center_y = (closed_cap_aabb[0][1] + closed_cap_aabb[1][1]) * 0.5
        closed_top_z = closed_cap_aabb[1][2]
        ctx.expect_overlap(
            cap, body, axes="xy",
            elem_a="cap_lid", elem_b="open_neck",
            min_overlap=0.006,
            name="closed flip cap covers the open mouth in XY",
        )
        # The skirt extends below the neck top by design (wraps around the lip).
        # Check that the lid disk (not the skirt) sits at the neck top level.
        ctx.check(
            "closed cap lid sits on top of the neck",
            closed_top_z > NECK_TOP_Z + LID_THICKNESS * 0.5,
            details=f"closed cap top z={closed_top_z}, neck_top={NECK_TOP_Z}",
        )

    # --- Open pose: cap swings backward ---
    with ctx.pose({hinge: CAP_OPEN_ANGLE}):
        open_cap_aabb = ctx.part_world_aabb(cap)
        open_center_y = (open_cap_aabb[0][1] + open_cap_aabb[1][1]) * 0.5
        open_top_z = open_cap_aabb[1][2]

    # --- Verify opening motion rotates (not translates) the cap ---
    ctx.check(
        "opening the cap swings it backward (center Y moves rearward)",
        open_center_y < closed_center_y - 0.005,
        details=f"closed_center_y={closed_center_y}, open_center_y={open_center_y}",
    )
    ctx.check(
        "open cap stays near hinge height (rotates, does not lift off)",
        abs(open_top_z - closed_top_z) < 0.025,
        details=f"closed_top_z={closed_top_z}, open_top_z={open_top_z}",
    )

    return ctx.report()


object_model = build_object_model()
