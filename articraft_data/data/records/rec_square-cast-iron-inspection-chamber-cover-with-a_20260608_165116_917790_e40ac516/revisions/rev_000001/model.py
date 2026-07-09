from __future__ import annotations

# Square cast-iron inspection chamber cover. The heavy cover has a raised border
# rim, an anti-slip diamond/grid stud pattern across its top, a central recessed
# panel, and two edge lifting-key recesses. The cover lifts straight up out of a
# cast-iron seating frame (prismatic +Z) to reveal the chamber shaft below.

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
# Real-world dimensions (meters). A ~450 mm class cast-iron inspection cover.
# ---------------------------------------------------------------------------
COVER_SIDE = 0.450  # square cover plan side (clear of the rim)
COVER_BASE_THICK = 0.022  # base plate thickness of the cast cover
RIM_HEIGHT = 0.014  # raised perimeter rim height above the base plate top
RIM_WIDTH = 0.030  # radial width of the raised rim band

STUD_PITCH = 0.034  # center-to-center spacing of the diamond studs
STUD_SIZE = 0.020  # diagonal foot size of each diamond stud (before rotation)
STUD_HEIGHT = 0.009  # stud relief height above the base plate top

PANEL_LEN = 0.190  # central recessed panel length
PANEL_WID = 0.110  # central recessed panel width
PANEL_DEPTH = 0.006  # central panel recess depth into the rim-top plane

KEY_LEN = 0.070  # lifting-key recess length (along the edge)
KEY_WID = 0.030  # lifting-key recess width (radial)
KEY_DEPTH = 0.012  # lifting-key recess depth

# Cast-iron seating frame.
FRAME_SEAT_GAP = 0.006  # perimeter gap between cover skirt and frame inner wall
FRAME_WALL = 0.055  # radial width of the visible frame band
FRAME_INNER = COVER_SIDE + 2.0 * FRAME_SEAT_GAP
FRAME_OUTER = FRAME_INNER + 2.0 * FRAME_WALL
FRAME_HEIGHT = 0.075  # total frame depth into the ground
FRAME_LEDGE = 0.022  # inward seat ledge the cover rests on
SHAFT_SIDE = FRAME_INNER - 2.0 * FRAME_LEDGE

# The cover skirt drops into the frame; the rim/base top sits proud of the frame.
SEAT_DROP = COVER_BASE_THICK - 0.004  # base-plate underside sits below frame top
SEAT_LEDGE_TOP_Z = FRAME_HEIGHT - SEAT_DROP
SEAT_EMBED = 0.003
COVER_REST_BOTTOM_Z = SEAT_LEDGE_TOP_Z - SEAT_EMBED

SHAFT_DEPTH = 0.28
FRAME_TOP_BAND = 0.030
LIFT_TRAVEL = SEAT_DROP + RIM_HEIGHT + 0.08  # clear the frame and rim


def _cover_solid() -> cq.Workplane:
    """Cast-iron cover authored in its own local frame: base plate centered in
    plan, base-plate bottom at local z=0. Top features (rim, diamond studs,
    central panel, lifting recesses) are built on top of the base plate."""
    base = (
        cq.Workplane("XY")
        .box(COVER_SIDE, COVER_SIDE, COVER_BASE_THICK)
        .translate((0.0, 0.0, COVER_BASE_THICK / 2.0))
    )
    base = base.edges("|Z").chamfer(0.004)
    top_z = COVER_BASE_THICK  # base-plate top plane

    # Raised perimeter rim: square frame band on the base-plate top.
    outer = (
        cq.Workplane("XY")
        .box(COVER_SIDE, COVER_SIDE, RIM_HEIGHT)
        .translate((0.0, 0.0, top_z + RIM_HEIGHT / 2.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(COVER_SIDE - 2.0 * RIM_WIDTH, COVER_SIDE - 2.0 * RIM_WIDTH, RIM_HEIGHT + 0.02)
        .translate((0.0, 0.0, top_z + RIM_HEIGHT / 2.0))
    )
    rim = outer.cut(inner)

    cover = base.union(rim)

    # Anti-slip diamond stud grid across the inner field (inside the rim).
    field = COVER_SIDE - 2.0 * RIM_WIDTH - 0.006
    half = STUD_SIZE / 2.0
    # Diamond profile: a square rotated 45 deg, tapering to a flat top (frustum).
    diamond_pts = [(half, 0.0), (0.0, half), (-half, 0.0), (0.0, -half)]
    n = int(field // STUD_PITCH)
    if n % 2 == 0:
        n -= 1  # keep it centered/odd
    start = -(n - 1) / 2.0 * STUD_PITCH
    studs = []
    for ix in range(n):
        for iy in range(n):
            cx = start + ix * STUD_PITCH
            cy = start + iy * STUD_PITCH
            # Skip studs that fall under the central recessed panel footprint.
            if abs(cx) <= PANEL_LEN / 2.0 + 0.006 and abs(cy) <= PANEL_WID / 2.0 + 0.006:
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

    # Central recessed panel (lettering/lifting plate), recessed below the rim top.
    panel_top = top_z + RIM_HEIGHT
    panel_cut = (
        cq.Workplane("XY")
        .box(PANEL_LEN, PANEL_WID, PANEL_DEPTH + STUD_HEIGHT + 0.02)
        .translate(
            (0.0, 0.0, panel_top - (PANEL_DEPTH + STUD_HEIGHT + 0.02) / 2.0 + PANEL_DEPTH)
        )
    )
    cover = cover.cut(panel_cut)

    # Two opposite-edge lifting-key recesses cut down through the rim.
    edge_off = COVER_SIDE / 2.0 - RIM_WIDTH / 2.0
    for sx in (-1.0, 1.0):
        key = (
            cq.Workplane("XY")
            .box(KEY_WID, KEY_LEN, KEY_DEPTH + 0.02)
            .translate((sx * edge_off, 0.0, panel_top - (KEY_DEPTH + 0.02) / 2.0))
        )
        cover = cover.cut(key)

    return cover


def _frame_solid() -> cq.Workplane:
    """Cast-iron seating frame with a recessed cover seat and a through throat."""
    frame = (
        cq.Workplane("XY")
        .box(FRAME_OUTER, FRAME_OUTER, FRAME_HEIGHT)
        .translate((0.0, 0.0, FRAME_HEIGHT / 2.0))
    )
    # Cover-seat recess down to the seat ledge.
    recess_top = FRAME_HEIGHT + 0.02
    recess_bot = SEAT_LEDGE_TOP_Z
    recess = (
        cq.Workplane("XY")
        .box(FRAME_INNER, FRAME_INNER, recess_top - recess_bot)
        .translate((0.0, 0.0, (recess_top + recess_bot) / 2.0))
    )
    frame = frame.cut(recess)
    # Through throat (shaft void).
    throat = (
        cq.Workplane("XY")
        .box(SHAFT_SIDE, SHAFT_SIDE, FRAME_HEIGHT + 0.04)
        .translate((0.0, 0.0, FRAME_HEIGHT / 2.0))
    )
    frame = frame.cut(throat)
    frame = frame.faces(">Z").edges().chamfer(0.005)
    return frame


def _shaft_solid() -> cq.Workplane:
    """Hollow chamber shaft below the frame (top abutting the frame underside)."""
    wall = 0.025
    outer = SHAFT_SIDE + 2.0 * wall
    h = SHAFT_DEPTH
    box = cq.Workplane("XY").box(outer, outer, h).translate((0.0, 0.0, -h / 2.0))
    bore = (
        cq.Workplane("XY")
        .box(SHAFT_SIDE, SHAFT_SIDE, h + 0.04)
        .translate((0.0, 0.0, -h / 2.0))
    )
    return box.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_inspection_cover")
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

    cover = model.part("inspection_cover")
    cover.visual(
        mesh_from_cadquery(_cover_solid(), "inspection_cover"),
        material="cast_iron",
        name="cover_body",
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
    cover = object_model.get_part("inspection_cover")
    lift = object_model.get_articulation("frame_to_cover")

    ctx.allow_overlap(
        cover,
        frame,
        reason="Cover skirt seats a few mm onto the recessed frame ledge (seated insertion).",
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
        # Cover stays within the frame opening (seating gap around it).
        ctx.expect_within(
            cover, frame, axes="xy", margin=0.0,
            name="cover sits within the frame opening",
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

    return ctx.report()


object_model = build_object_model()
