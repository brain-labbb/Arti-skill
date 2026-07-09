from __future__ import annotations

# Square cast-iron inspection chamber cover with a hinged swing-up mechanism.
# The heavy cover has a raised border rim, anti-slip diamond/grid stud pattern,
# a central recessed panel, and two edge lifting-key recesses. A knuckle hinge
# along the +X edge joins the cover to the cast-iron seating frame, allowing
# the cover to swing up and open (REVOLUTE around +Y) like a trapdoor to reveal
# the chamber shaft void below.

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

# ---------------------------------------------------------------------------
# Hinge dimensions and positioning
# ---------------------------------------------------------------------------
HINGE_PIN_D = 0.010  # hinge pin diameter
HINGE_KNUCKLE_OD = 0.020  # knuckle barrel outer diameter
HINGE_KNUCKLE_LEN = 0.040  # each frame knuckle length along Y
HINGE_LUG_WIDTH = 0.055  # cover hinge lug width along Y
HINGE_Y_SPREAD = 0.080  # Y offset of frame knuckle centers from centre

# Hinge pin axis sits half a knuckle OD above the frame top.
HINGE_X = COVER_SIDE / 2.0  # hinge at the cover +X edge
HINGE_Z = FRAME_HEIGHT + HINGE_KNUCKLE_OD / 2.0  # 0.085

# Cover visual offset in the part frame (origin at hinge axis).
COVER_VISUAL_DX = -HINGE_X  # shift cover so +X edge at part origin
COVER_VISUAL_DZ = COVER_REST_BOTTOM_Z - HINGE_Z  # seat the base plate

# Lug barrel Z in cover local frame (maps to HINGE_Z in world at q=0).
LUG_BARREL_Z_LOCAL = -COVER_VISUAL_DZ  # 0.031

HINGE_OPEN_ANGLE = 2.0  # radians (~115°), past vertical

# Recess to accommodate frame knuckle barrels in the cover rim.
KNUCKLE_RECESS_R = HINGE_KNUCKLE_OD / 2.0 + 0.002
KNUCKLE_RECESS_LEN = HINGE_KNUCKLE_LEN + 0.006


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _cover_solid() -> cq.Workplane:
    """Cast-iron cover with base plate, rim, studs, panel, key recesses,
    hinge lug, and knuckle recesses. All in the cover local frame:
    centered in XY, base-plate bottom at z=0."""
    base = (
        cq.Workplane("XY")
        .box(COVER_SIDE, COVER_SIDE, COVER_BASE_THICK)
        .translate((0.0, 0.0, COVER_BASE_THICK / 2.0))
    )
    base = base.edges("|Z").chamfer(0.004)
    top_z = COVER_BASE_THICK  # base-plate top plane

    # --- Raised perimeter rim ---
    outer = (
        cq.Workplane("XY")
        .box(COVER_SIDE, COVER_SIDE, RIM_HEIGHT)
        .translate((0.0, 0.0, top_z + RIM_HEIGHT / 2.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(
            COVER_SIDE - 2.0 * RIM_WIDTH,
            COVER_SIDE - 2.0 * RIM_WIDTH,
            RIM_HEIGHT + 0.02,
        )
        .translate((0.0, 0.0, top_z + RIM_HEIGHT / 2.0))
    )
    rim = outer.cut(inner)
    cover = base.union(rim)

    # --- Anti-slip diamond stud grid ---
    field = COVER_SIDE - 2.0 * RIM_WIDTH - 0.006
    half = STUD_SIZE / 2.0
    diamond_pts = [(half, 0.0), (0.0, half), (-half, 0.0), (0.0, -half)]
    n = int(field // STUD_PITCH)
    if n % 2 == 0:
        n -= 1
    start = -(n - 1) / 2.0 * STUD_PITCH
    studs = []
    for ix in range(n):
        for iy in range(n):
            cx = start + ix * STUD_PITCH
            cy = start + iy * STUD_PITCH
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

    # --- Central recessed panel ---
    panel_top = top_z + RIM_HEIGHT
    panel_cut = (
        cq.Workplane("XY")
        .box(PANEL_LEN, PANEL_WID, PANEL_DEPTH + STUD_HEIGHT + 0.02)
        .translate(
            (0.0, 0.0, panel_top - (PANEL_DEPTH + STUD_HEIGHT + 0.02) / 2.0 + PANEL_DEPTH)
        )
    )
    cover = cover.cut(panel_cut)

    # --- Two opposite-edge lifting-key recesses ---
    edge_off = COVER_SIDE / 2.0 - RIM_WIDTH / 2.0
    for sx in (-1.0, 1.0):
        key = (
            cq.Workplane("XY")
            .box(KEY_WID, KEY_LEN, KEY_DEPTH + 0.02)
            .translate((sx * edge_off, 0.0, panel_top - (KEY_DEPTH + 0.02) / 2.0))
        )
        cover = cover.cut(key)

    # --- Hinge lug barrel + connecting tab at +X edge ---
    lug_r = HINGE_KNUCKLE_OD / 2.0 - 0.0015
    lug_bore_r = HINGE_PIN_D / 2.0 + 0.001

    # Barrel along Y at (COVER_SIDE/2, 0, LUG_BARREL_Z_LOCAL)
    lug_barrel = (
        cq.Workplane("XZ")
        .circle(lug_r)
        .extrude(HINGE_LUG_WIDTH)
        .translate((COVER_SIDE / 2.0, -HINGE_LUG_WIDTH / 2.0, LUG_BARREL_Z_LOCAL))
    )
    lug_bore = (
        cq.Workplane("XZ")
        .circle(lug_bore_r)
        .extrude(HINGE_LUG_WIDTH + 0.01)
        .translate((COVER_SIDE / 2.0, -(HINGE_LUG_WIDTH + 0.01) / 2.0, LUG_BARREL_Z_LOCAL))
    )
    lug = lug_barrel.cut(lug_bore)

    # Connecting tab from barrel down into base plate for solid connectivity
    tab_x_size = 0.030
    tab_y_size = HINGE_LUG_WIDTH * 0.90
    tab_embed = 0.015  # tab extends deep into base plate for connectivity
    tab_z_bot = COVER_BASE_THICK - tab_embed
    tab_z_top = LUG_BARREL_Z_LOCAL + 0.005  # above barrel center for overlap
    tab_z_size = tab_z_top - tab_z_bot
    tab_cx = COVER_SIDE / 2.0 - tab_x_size / 2.0
    tab_cz = (tab_z_bot + tab_z_top) / 2.0
    tab = (
        cq.Workplane("XY")
        .box(tab_x_size, tab_y_size, tab_z_size)
        .translate((tab_cx, 0.0, tab_cz))
    )
    lug = lug.union(tab)
    cover = cover.union(lug)

    # --- Knuckle recesses: cylindrical cuts for frame knuckle barrels ---
    for y_sign in (-1.0, 1.0):
        recess = (
            cq.Workplane("XZ")
            .circle(KNUCKLE_RECESS_R)
            .extrude(KNUCKLE_RECESS_LEN)
            .translate(
                (
                    COVER_SIDE / 2.0,
                    y_sign * HINGE_Y_SPREAD - KNUCKLE_RECESS_LEN / 2.0,
                    LUG_BARREL_Z_LOCAL,
                )
            )
        )
        cover = cover.cut(recess)

    return cover


def _frame_solid() -> cq.Workplane:
    """Cast-iron seating frame with integrated hinge knuckle barrels and bosses.
    Frame body centered in XY, bottom at z=0, top at z=FRAME_HEIGHT."""
    frame = (
        cq.Workplane("XY")
        .box(FRAME_OUTER, FRAME_OUTER, FRAME_HEIGHT)
        .translate((0.0, 0.0, FRAME_HEIGHT / 2.0))
    )

    # Cover-seat recess
    recess_top = FRAME_HEIGHT + 0.02
    recess_bot = SEAT_LEDGE_TOP_Z
    recess = (
        cq.Workplane("XY")
        .box(FRAME_INNER, FRAME_INNER, recess_top - recess_bot)
        .translate((0.0, 0.0, (recess_top + recess_bot) / 2.0))
    )
    frame = frame.cut(recess)

    # Through throat (shaft void)
    throat = (
        cq.Workplane("XY")
        .box(SHAFT_SIDE, SHAFT_SIDE, FRAME_HEIGHT + 0.04)
        .translate((0.0, 0.0, FRAME_HEIGHT / 2.0))
    )
    frame = frame.cut(throat)
    frame = frame.faces(">Z").edges().chamfer(0.005)

    # --- Hinge knuckle barrels + bosses at +X edge ---
    knuckle_r = HINGE_KNUCKLE_OD / 2.0
    bore_r = HINGE_PIN_D / 2.0 + 0.001
    boss_x = HINGE_KNUCKLE_OD + 0.012
    boss_y = HINGE_KNUCKLE_OD + 0.008
    boss_embed = 0.020  # boss extends deep into frame body for solid connectivity
    boss_z_bot = FRAME_HEIGHT - boss_embed
    boss_z_top = HINGE_Z + 0.005  # above barrel center for overlap
    boss_z = boss_z_top - boss_z_bot

    for i in range(2):
        y_sign = -1.0 if i == 0 else 1.0
        y_pos = y_sign * HINGE_Y_SPREAD

        # Knuckle barrel along Y
        barrel = (
            cq.Workplane("XZ")
            .circle(knuckle_r)
            .extrude(HINGE_KNUCKLE_LEN)
            .translate((HINGE_X, y_pos - HINGE_KNUCKLE_LEN / 2.0, HINGE_Z))
        )
        # Pin bore
        bore = (
            cq.Workplane("XZ")
            .circle(bore_r)
            .extrude(HINGE_KNUCKLE_LEN + 0.01)
            .translate((HINGE_X, y_pos - (HINGE_KNUCKLE_LEN + 0.01) / 2.0, HINGE_Z))
        )
        knuckle = barrel.cut(bore)

        # Mounting boss: connects barrel into frame body, extends outward (+X)
        boss_cx = HINGE_X + boss_x / 2.0
        boss_cz = (boss_z_bot + boss_z_top) / 2.0
        boss = (
            cq.Workplane("XY")
            .box(boss_x, boss_y, boss_z)
            .translate((boss_cx, y_pos, boss_cz))
        )
        knuckle = knuckle.union(boss)
        # Use fuse with clean=True to create single connected solid
        frame = frame.union(knuckle)

    return frame


def _shaft_solid() -> cq.Workplane:
    """Hollow chamber shaft below the frame."""
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


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_inspection_cover")
    model.material("cast_iron", rgba=(0.34, 0.35, 0.37, 1.0))
    model.material("frame_iron", rgba=(0.42, 0.43, 0.45, 1.0))
    model.material("shaft_void", rgba=(0.08, 0.08, 0.09, 1.0))

    # --- Frame (root) with integrated hinge knuckles ---
    frame = model.part("frame_ring")
    frame.visual(
        mesh_from_cadquery(_frame_solid(), "frame_ring"),
        material="frame_iron",
        name="frame_body",
    )

    # --- Shaft (fixed below frame) ---
    shaft = model.part("shaft")
    shaft.visual(
        mesh_from_cadquery(_shaft_solid(), "shaft"),
        material="shaft_void",
        name="shaft_body",
    )
    model.articulation(
        "frame_to_shaft",
        ArticulationType.FIXED,
        parent=frame,
        child=shaft,
        origin=Origin(),
    )

    # --- Cover (hinged) with integrated lug ---
    cover = model.part("inspection_cover")
    cover.visual(
        mesh_from_cadquery(_cover_solid(), "inspection_cover"),
        origin=Origin(xyz=(COVER_VISUAL_DX, 0.0, COVER_VISUAL_DZ)),
        material="cast_iron",
        name="cover_body",
    )

    # Revolute hinge: pin axis along +Y at the +X edge of the frame top.
    # Cover extends in local -X from hinge; axis=+Y → positive q lifts far edge up.
    model.articulation(
        "frame_to_cover",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cover,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2000.0,
            velocity=0.8,
            lower=0.0,
            upper=HINGE_OPEN_ANGLE,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_ring")
    shaft = object_model.get_part("shaft")
    cover = object_model.get_part("inspection_cover")
    hinge = object_model.get_articulation("frame_to_cover")

    # --- Intentional overlap allowance: seated insertion ---
    ctx.allow_overlap(
        cover,
        frame,
        reason="Cover skirt seats a few mm into the recessed frame ledge (seated insertion).",
    )

    # --- Primary articulation contract: REVOLUTE hinge ---
    ctx.check(
        "primary joint is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=str(hinge.articulation_type),
    )
    ctx.check(
        "hinge axis is +Y",
        tuple(round(a, 6) for a in hinge.axis) == (0.0, 1.0, 0.0),
        details=str(hinge.axis),
    )
    lim = hinge.motion_limits
    ctx.check(
        "hinge opens past vertical",
        lim is not None
        and lim.lower == 0.0
        and lim.upper is not None
        and lim.upper > math.pi / 2.0,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # --- Frame footprint at z~0 ---
    f_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame footprint at z~0",
        f_aabb is not None and abs(f_aabb[0][2]) <= 0.002,
        details=str(f_aabb),
    )

    # --- Closed pose (q=0): cover seated in frame ---
    with ctx.pose({hinge: 0.0}):
        c_aabb = ctx.part_world_aabb(cover)
        ctx.expect_contact(
            cover,
            frame,
            contact_tol=0.001,
            name="cover seated in contact with frame ledge",
        )
        ctx.expect_overlap(
            cover,
            frame,
            axes="xy",
            min_overlap=0.30,
            name="cover spans the frame opening in plan",
        )
        ctx.expect_within(
            cover,
            frame,
            axes="xy",
            margin=0.0,
            name="cover sits within the frame opening",
        )
        # Cover top above the nominal frame top surface (excluding hinge knuckles).
        ctx.check(
            "cover rim above frame seat level",
            c_aabb is not None
            and c_aabb[1][2] >= FRAME_HEIGHT + RIM_HEIGHT - 0.003,
            details=f"cover_top={c_aabb[1][2] if c_aabb else None}",
        )
        # Cover has raised relief (rim + studs).
        ctx.check(
            "cover has raised relief above base plate",
            c_aabb is not None
            and (c_aabb[1][2] - c_aabb[0][2]) >= COVER_BASE_THICK + RIM_HEIGHT - 0.001,
            details=f"cover_height={(c_aabb[1][2] - c_aabb[0][2]) if c_aabb else None}",
        )
        closed_max_z = c_aabb[1][2] if c_aabb else 0.0

    # --- Open pose: cover swings up, far edge well above frame ---
    open_angle = HINGE_OPEN_ANGLE * 0.6  # ~69° for a decisive open check
    with ctx.pose({hinge: open_angle}):
        up_aabb = ctx.part_world_aabb(cover)
        # Cover AABB max Z increases significantly when opened.
        ctx.check(
            "cover swings upward when opened",
            up_aabb is not None and up_aabb[1][2] > closed_max_z + 0.08,
            details=f"closed_max_z={closed_max_z}, open_max_z={up_aabb[1][2] if up_aabb else None}",
        )
        # Cover bottom clears the frame top when open.
        ctx.check(
            "open cover clears the frame top",
            up_aabb is not None
            and f_aabb is not None
            and up_aabb[0][2] >= FRAME_HEIGHT - 0.01,
            details=f"cover_bottom={up_aabb[0][2] if up_aabb else None} frame_top={FRAME_HEIGHT}",
        )

    # --- Shaft void below the seat, centered under the opening ---
    s_aabb = ctx.part_world_aabb(shaft)
    ctx.check(
        "shaft void below seat ledge",
        s_aabb is not None and s_aabb[1][2] <= SEAT_LEDGE_TOP_Z + 0.001,
        details=str(s_aabb),
    )
    ctx.expect_overlap(
        shaft,
        frame,
        axes="xy",
        min_overlap=0.20,
        name="shaft centered under frame opening",
    )

    return ctx.report()


object_model = build_object_model()
