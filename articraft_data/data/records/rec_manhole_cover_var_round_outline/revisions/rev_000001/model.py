from __future__ import annotations

# Round cast-iron drain cover seated in a circular cast-iron frame ring. The cover
# disc lifts straight up out of the circular recessed frame (prismatic +Z) to
# reveal the round shaft void below. The cover has a raised circular rim band,
# an anti-slip diamond stud pattern across its top, a central recessed panel,
# and two opposite-edge lifting-key recesses.

import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters). A ~450 mm diameter round cast-iron drain cover.
# ---------------------------------------------------------------------------
COVER_RADIUS = 0.225  # cover disc radius (diameter ~450 mm)
COVER_BASE_THICK = 0.022  # base plate thickness of the cast cover
RIM_HEIGHT = 0.014  # raised perimeter rim height above the base plate top
RIM_WIDTH = 0.030  # radial width of the raised rim band

STUD_PITCH = 0.034  # center-to-center spacing of the diamond studs
STUD_SIZE = 0.020  # diagonal foot size of each diamond stud (before rotation)
STUD_HEIGHT = 0.009  # stud relief height above the base plate top

PANEL_RADIUS = 0.065  # central recessed panel radius (circular)
PANEL_DEPTH = 0.006  # central panel recess depth into the rim-top plane

KEY_LEN = 0.070  # lifting-key recess length (tangential)
KEY_WID = 0.030  # lifting-key recess width (radial)
KEY_DEPTH = 0.012  # lifting-key recess depth

# Cast-iron seating frame (circular).
FRAME_SEAT_GAP = 0.006  # radial gap between cover skirt and frame inner wall
FRAME_WALL = 0.055  # radial width of the visible frame band
FRAME_INNER_R = COVER_RADIUS + FRAME_SEAT_GAP
FRAME_OUTER_R = FRAME_INNER_R + FRAME_WALL
FRAME_HEIGHT = 0.075  # total frame depth into the ground
FRAME_LEDGE = 0.022  # inward seat ledge the cover rests on
SHAFT_RADIUS = FRAME_INNER_R - FRAME_LEDGE

# The cover skirt drops into the frame; the rim/base top sits proud of the frame.
SEAT_DROP = COVER_BASE_THICK - 0.004  # base-plate underside sits below frame top
SEAT_LEDGE_TOP_Z = FRAME_HEIGHT - SEAT_DROP
SEAT_EMBED = 0.003
COVER_REST_BOTTOM_Z = SEAT_LEDGE_TOP_Z - SEAT_EMBED

SHAFT_DEPTH = 0.28
LIFT_TRAVEL = SEAT_DROP + RIM_HEIGHT + 0.08  # clear the frame and rim


def _cover_solid() -> cq.Workplane:
    """Round cast-iron cover disc authored in its own local frame: centered in
    plan, base-plate bottom at local z=0. Top features (rim, diamond studs,
    central panel, lifting recesses) are built on top of the base plate."""
    # Base disc plate.
    base = (
        cq.Workplane("XY")
        .circle(COVER_RADIUS)
        .extrude(COVER_BASE_THICK)
    )
    top_z = COVER_BASE_THICK  # base-plate top plane

    # Raised perimeter rim: annular ring on the base-plate top.
    rim_outer_r = COVER_RADIUS
    rim_inner_r = COVER_RADIUS - RIM_WIDTH
    rim = (
        cq.Workplane("XY")
        .circle(rim_outer_r)
        .circle(rim_inner_r)
        .extrude(RIM_HEIGHT)
        .translate((0.0, 0.0, top_z))
    )

    cover = base.union(rim)

    # Anti-slip diamond stud grid across the inner field (inside the rim).
    field_radius = rim_inner_r - 0.003
    half = STUD_SIZE / 2.0
    # Diamond profile: a square rotated 45 deg, tapering to a flat top (frustum).
    diamond_pts = [(half, 0.0), (0.0, half), (-half, 0.0), (0.0, -half)]
    n = int(2.0 * field_radius // STUD_PITCH)
    if n % 2 == 0:
        n -= 1  # keep it centered/odd
    start = -(n - 1) / 2.0 * STUD_PITCH
    studs = []
    for ix in range(n):
        for iy in range(n):
            cx = start + ix * STUD_PITCH
            cy = start + iy * STUD_PITCH
            # Skip studs that fall outside the circular field or under the central panel.
            r = math.sqrt(cx * cx + cy * cy)
            if r > field_radius:
                continue
            if r < PANEL_RADIUS + 0.006:
                continue
            stud = (
                cq.Workplane("XY")
                .polyline(diamond_pts)
                .close()
                .workplane(offset=STUD_HEIGHT)
                .polyline([(p[0] * 0.45, p[1] * 0.45) for p in diamond_pts])
                .close()
                .loft()
                .translate((cx, cy, top_z))
            )
            studs.append(stud)
    if studs:
        stud_compound = studs[0]
        for s in studs[1:]:
            stud_compound = stud_compound.union(s)
        cover = cover.union(stud_compound)

    # Central recessed panel (circular), recessed below the rim top.
    panel_top = top_z + RIM_HEIGHT
    panel_cut = (
        cq.Workplane("XY")
        .circle(PANEL_RADIUS)
        .extrude(PANEL_DEPTH + STUD_HEIGHT + 0.02)
        .translate((0.0, 0.0, panel_top - (PANEL_DEPTH + STUD_HEIGHT + 0.02) + PANEL_DEPTH))
    )
    cover = cover.cut(panel_cut)

    # Two opposite-edge lifting-key recesses cut into the rim at 0 and 180 degrees.
    edge_off = COVER_RADIUS - RIM_WIDTH / 2.0
    for sx in (-1.0, 1.0):
        key = (
            cq.Workplane("XY")
            .box(KEY_WID, KEY_LEN, KEY_DEPTH + 0.02)
            .translate((sx * edge_off, 0.0, panel_top - (KEY_DEPTH + 0.02) / 2.0))
        )
        cover = cover.cut(key)

    return cover


def _frame_solid() -> cq.Workplane:
    """Circular cast-iron seating frame with a recessed cover seat and a through throat."""
    # Outer ring body.
    frame = (
        cq.Workplane("XY")
        .circle(FRAME_OUTER_R)
        .extrude(FRAME_HEIGHT)
    )
    # Cover-seat recess down to the seat ledge (annular cut from top).
    recess_top = FRAME_HEIGHT + 0.02
    recess_bot = SEAT_LEDGE_TOP_Z
    recess_h = recess_top - recess_bot
    recess = (
        cq.Workplane("XY")
        .circle(FRAME_INNER_R)
        .extrude(recess_h)
        .translate((0.0, 0.0, recess_bot))
    )
    frame = frame.cut(recess)
    # Through throat (shaft void) - circular bore.
    throat = (
        cq.Workplane("XY")
        .circle(SHAFT_RADIUS)
        .extrude(FRAME_HEIGHT + 0.04)
        .translate((0.0, 0.0, -0.02))
    )
    frame = frame.cut(throat)
    # Chamfer top edges.
    frame = frame.faces(">Z").edges().chamfer(0.005)
    return frame


def _shaft_solid() -> cq.Workplane:
    """Hollow circular chamber shaft below the frame (top abutting the frame underside)."""
    wall = 0.025
    outer_r = SHAFT_RADIUS + wall
    h = SHAFT_DEPTH
    # Outer cylinder.
    tube = (
        cq.Workplane("XY")
        .circle(outer_r)
        .extrude(h)
        .translate((0.0, 0.0, -h))
    )
    # Inner bore.
    bore = (
        cq.Workplane("XY")
        .circle(SHAFT_RADIUS)
        .extrude(h + 0.04)
        .translate((0.0, 0.0, -h - 0.02))
    )
    return tube.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_cast_iron_drain_cover")
    model.material("cast_iron", rgba=(0.34, 0.35, 0.37, 1.0))
    model.material("frame_iron", rgba=(0.42, 0.43, 0.45, 1.0))
    model.material("shaft_void", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame_ring")
    frame.visual(mesh_from_cadquery(_frame_solid(), "frame_ring"), material="frame_iron")

    shaft = model.part("shaft")
    shaft.visual(mesh_from_cadquery(_shaft_solid(), "shaft"), material="shaft_void")
    model.articulation(
        "frame_to_shaft",
        ArticulationType.FIXED,
        parent=frame,
        child=shaft,
        origin=Origin(),
    )

    cover = model.part("drain_cover")
    cover.visual(
        mesh_from_cadquery(_cover_solid(), "drain_cover"),
        material="cast_iron",
        name="cover_disc",
    )
    model.articulation(
        "frame_to_cover",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, COVER_REST_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2000.0, velocity=0.15, lower=0.0, upper=LIFT_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_ring")
    shaft = object_model.get_part("shaft")
    cover = object_model.get_part("drain_cover")
    lift = object_model.get_articulation("frame_to_cover")

    ctx.allow_overlap(
        cover,
        frame,
        reason="Cover disc seats a few mm onto the recessed circular frame ledge (seated insertion).",
    )

    # Primary articulation contract: +Z prismatic lift.
    ctx.check(
        "primary joint is prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=str(lift.articulation_type),
    )
    ctx.check(
        "lift axis is +Z",
        tuple(round(a, 6) for a in lift.axis) == (0.0, 0.0, 1.0),
        details=str(lift.axis),
    )
    lim = lift.motion_limits
    ctx.check(
        "lift travel clears frame + rim",
        lim is not None and lim.lower == 0.0 and lim.upper >= RIM_HEIGHT + SEAT_DROP,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # Frame footprint at z~0.
    f_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame footprint at z~0",
        f_aabb is not None and abs(f_aabb[0][2]) <= 0.002,
        details=str(f_aabb),
    )

    with ctx.pose({lift: 0.0}):
        c_aabb = ctx.part_world_aabb(cover)
        # Cover seated (in contact) on the frame ledge.
        ctx.expect_contact(
            cover, frame, contact_tol=0.001,
            name="cover seated in contact with frame ledge",
        )
        # Cover spans the frame opening in plan.
        ctx.expect_overlap(
            cover, frame, axes="xy", min_overlap=0.30,
            name="cover spans the frame opening in plan",
        )
        # Cover stays within the frame outer boundary.
        ctx.expect_within(
            cover, frame, axes="xy", margin=0.0,
            name="cover sits within the frame outer boundary",
        )
        # Cover top (rim) proud of the frame top.
        ctx.check(
            "cover proud of frame top",
            c_aabb is not None and f_aabb is not None
            and c_aabb[1][2] >= f_aabb[1][2] - 0.001,
            details=f"cover_top={c_aabb[1][2] if c_aabb else None} frame_top={f_aabb[1][2] if f_aabb else None}",
        )
        # Relief height: top of cover well above the base plate (rim + studs).
        ctx.check(
            "cover has raised relief above base plate",
            c_aabb is not None
            and (c_aabb[1][2] - c_aabb[0][2]) >= COVER_BASE_THICK + RIM_HEIGHT - 0.001,
            details=f"cover_height={(c_aabb[1][2] - c_aabb[0][2]) if c_aabb else None}",
        )

    # Lifted pose: cover rises clear, void exposed.
    rest_pos = ctx.part_world_position(cover)
    with ctx.pose({lift: LIFT_TRAVEL}):
        up_pos = ctx.part_world_position(cover)
        up_aabb = ctx.part_world_aabb(cover)
        ctx.check(
            "cover lifts upward along +Z",
            rest_pos is not None and up_pos is not None
            and up_pos[2] > rest_pos[2] + 0.05,
            details=f"rest={rest_pos}, up={up_pos}",
        )
        ctx.check(
            "lifted cover clears the frame",
            up_aabb is not None and f_aabb is not None
            and up_aabb[0][2] >= f_aabb[1][2] - 0.001,
            details=f"cover_bottom={up_aabb[0][2] if up_aabb else None} frame_top={f_aabb[1][2] if f_aabb else None}",
        )

    # Shaft void below the seat, centered under the opening.
    s_aabb = ctx.part_world_aabb(shaft)
    ctx.check(
        "shaft void below seat ledge",
        s_aabb is not None and s_aabb[1][2] <= SEAT_LEDGE_TOP_Z + 0.001,
        details=str(s_aabb),
    )
    ctx.expect_overlap(
        shaft, frame, axes="xy", min_overlap=0.20,
        name="shaft centered under frame opening",
    )

    # Round geometry checks: cover, frame, and shaft are all circular.
    # The cover XY span should be approximately equal in X and Y (circular symmetry).
    with ctx.pose({lift: 0.0}):
        c_aabb = ctx.part_world_aabb(cover)
        if c_aabb is not None:
            dx = c_aabb[1][0] - c_aabb[0][0]
            dy = c_aabb[1][1] - c_aabb[0][1]
            ctx.check(
                "cover is round (equal X and Y span)",
                abs(dx - dy) < 0.010,
                details=f"dx={dx:.4f}, dy={dy:.4f}",
            )
        f_aabb_check = ctx.part_world_aabb(frame)
        if f_aabb_check is not None:
            fdx = f_aabb_check[1][0] - f_aabb_check[0][0]
            fdy = f_aabb_check[1][1] - f_aabb_check[0][1]
            ctx.check(
                "frame is round (equal X and Y span)",
                abs(fdx - fdy) < 0.010,
                details=f"fdx={fdx:.4f}, fdy={fdy:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
