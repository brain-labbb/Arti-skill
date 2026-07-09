from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# High-arc gooseneck faucet variant with pull-out ribbed spray head.
# Fork of the gloss-black monobloc kitchen mixer tap (~0.38 m tall).
#
# Changes from parent:
# - Chrome dock collar replaces the tip sleeve + outlet aerator at the spout end
# - Separate spray_head part with longitudinal ribbing, angled aerator face,
#   and distinct hollow outlet bore
# - Prismatic hose joint: spray head slides out downward (0..0.06 m)
# - All parent articulations retained (spout swivel, pin lever pivots)
#
# Layout (world frame, deck plane at z = 0, +X toward user/sink):
# - Chrome base disc on deck; gloss-black column rises on Z axis.
# - Cross-cylinder valve bodies with flat end caps and pin levers.
# - Chrome collar → gooseneck spout swivels about vertical axis.
# - Spout arcs to apex ~0.38 m, drop leg ends at chrome dock collar.
# - Spray head docks below the collar, pulls out on prismatic joint.
# ---------------------------------------------------------------------------

# Base + column (unchanged from parent)
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# Cross valve cylinder (unchanged)
CROSS_Z = 0.085
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0

# Pin levers (unchanged)
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Swivel collar + gooseneck (unchanged geometry)
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R  # 0.144 m horizontal reach
DROP_END = 0.124  # spout-local z of the tube end

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m
SWIVEL_LIMIT = math.radians(110.0)

# Dock collar (replaces parent tip_sleeve + outlet_aerator)
DOCK_R = 0.017
DOCK_LEN = 0.008
DOCK_CENTER_Z = DROP_END - DOCK_LEN / 2.0  # spout-local z

# Spray head
SPRAY_R = 0.016  # body outer radius (rib peaks)
SPRAY_LEN = 0.050  # body length
N_RIBS = 10  # number of longitudinal ribs
RIB_DEPTH = 0.0012  # groove depth (rib valley below peak)
AERATOR_ANGLE = math.radians(20.0)  # outlet face tilt from horizontal
OUTLET_R = 0.007  # hollow outlet bore radius
OUTLET_DEPTH = 0.015  # bore depth into body
SLIDE_LIMIT = 0.060  # prismatic travel in metres

# Prismatic joint origin in spout-local frame (bottom of dock collar)
JOINT_Z_SPOUT = DROP_END - DOCK_LEN


def _gooseneck_shape() -> cq.Workplane:
    """Swan-neck tube: straight riser, high semicircular arc, short drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def _spray_head_body() -> cq.Workplane:
    """Ribbed spray head body with angled outlet face and hollow bore.

    Built with z = 0 at the bottom (outlet end) and z = SPRAY_LEN at the top
    (dock end).  The visual origin translates the mesh so the top face sits at
    the part origin and the body hangs below.
    """
    R = SPRAY_R
    L = SPRAY_LEN

    # --- 1. Ribbed cross-section profile (star polygon) -----------------
    pts: list[tuple[float, float]] = []
    for i in range(N_RIBS * 2):
        angle = i * math.pi / N_RIBS
        r = R if i % 2 == 0 else R - RIB_DEPTH
        pts.append((r * math.cos(angle), r * math.sin(angle)))

    body = cq.Workplane("XY").polyline(pts).close().extrude(L)

    # --- 2. Angled bottom face (aerator tilt) ----------------------------
    # Tilted plane at z ≈ 0: higher at −X, lower at +X.
    cut_h = R * 1.2 * math.tan(AERATOR_ANGLE)
    margin = R * 2.5
    wedge = (
        cq.Workplane("XZ")
        .moveTo(-margin, cut_h + 0.001)
        .lineTo(margin, -cut_h - 0.001)
        .lineTo(margin, -0.025)
        .lineTo(-margin, -0.025)
        .close()
        .extrude(margin * 2.0)
        .translate((0.0, -margin, 0.0))
    )
    body = body.cut(wedge)

    # --- 3. Hollow outlet bore from the bottom face ----------------------
    bore = cq.Workplane("XY").circle(OUTLET_R).extrude(OUTLET_DEPTH)
    body = body.cut(bore)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.05, 0.05, 0.06, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        Cylinder(radius=BASE_DISC_R, length=BASE_DISC_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_H / 2.0)),
        material=chrome,
        name="base_disc",
    )
    column.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_TOP - 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_TOP + 0.004) / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=CROSS_R, length=CROSS_TUBE_LEN),
        origin=Origin(xyz=(0.0, 0.0, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="cross_tube",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_0",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, -CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_1",
    )
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    # Chrome dock collar at the bottom of the drop leg (retaining ring)
    spout.visual(
        Cylinder(radius=DOCK_R, length=DOCK_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DOCK_CENTER_Z)),
        material=chrome,
        name="dock_collar",
    )
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5, lower=-SWIVEL_LIMIT, upper=SWIVEL_LIMIT
        ),
    )

    # ------------------------------------------------------------- spray head
    spray = model.part("spray_head")
    # CadQuery body: z = 0 (bottom/outlet) → z = SPRAY_LEN (top/dock face).
    # Visual origin shifts mesh so the top face is at part origin, body below.
    spray.visual(
        mesh_from_cadquery(_spray_head_body(), "spray_body"),
        origin=Origin(xyz=(0.0, 0.0, -SPRAY_LEN)),
        material=gloss_black,
        name="spray_body",
    )
    # Visible outlet opening — dark insert seated inside the bore, slightly
    # wider than the bore so it overlaps with the body walls (connectivity).
    spray.visual(
        Cylinder(radius=OUTLET_R + 0.001, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -SPRAY_LEN + OUTLET_DEPTH * 0.5)),
        material=outlet_dark,
        name="outlet_opening",
    )
    # Prismatic hose joint: spray head slides downward out of the dock
    model.articulation(
        "spray_slide",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray,
        origin=Origin(xyz=(REACH_X, 0.0, JOINT_Z_SPOUT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.5, lower=0.0, upper=SLIDE_LIMIT
        ),
    )

    # ------------------------------------------------------------- pin levers
    for idx, y_sign in ((0, 1.0), (1, -1.0)):
        lever = model.part(f"pin_lever_{idx}")
        lever.visual(
            Cylinder(radius=BOSS_R, length=BOSS_LEN),
            origin=Origin(xyz=(0.0, 0.0, BOSS_Z)),
            material=gloss_black,
            name="lever_boss",
        )
        lever.visual(
            Cylinder(radius=PIN_R, length=PIN_LEN),
            origin=Origin(xyz=(0.0, 0.0, PIN_Z0 + PIN_LEN / 2.0)),
            material=gloss_black,
            name="lever_pin",
        )
        model.articulation(
            f"lever_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=column,
            child=lever,
            origin=Origin(xyz=(0.0, y_sign * LEVER_Y, CROSS_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=2.0, lower=-math.pi / 2.0, upper=0.0
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    spray = object_model.get_part("spray_head")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    slide = object_model.get_articulation("spray_slide")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

    # ---- intentional seated overlaps ----
    ctx.allow_overlap(
        lever_0, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss seats into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1, column,
        elem_a="lever_boss", elem_b="cross_tube",
        reason="Lever boss seats into the valve cylinder.",
    )
    ctx.allow_overlap(
        spray, spout,
        elem_a="spray_body", elem_b="dock_collar",
        reason="Spray head body docks into the retaining collar at rest.",
    )

    # ---- grounding and overall scale ----
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "faucet grounded on deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m",
        spout_aabb is not None and 0.370 <= spout_aabb[1][2] <= 0.390,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.140,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- dock collar at spout end ----
    dock = ctx.part_element_world_aabb(spout, elem="dock_collar")
    ctx.check(
        "chrome dock collar sits at the drop-leg end",
        dock is not None
        and 0.240 <= dock[0][2] <= 0.270
        and 0.030 <= (dock[1][0] - dock[0][0]) <= 0.038,
        details=f"dock_collar={dock}",
    )

    # ---- spray head geometry ----
    spray_aabb = ctx.part_world_aabb(spray)
    ctx.check(
        "spray head hangs below the dock collar",
        spray_aabb is not None
        and dock is not None
        and spray_aabb[0][2] < dock[0][2] - 0.02
        and spray_aabb[1][2] <= dock[0][2] + 0.003,
        details=f"spray={spray_aabb}, dock={dock}",
    )

    spray_body_aabb = ctx.part_element_world_aabb(spray, elem="spray_body")
    ctx.check(
        "spray body is ~0.05 m long",
        spray_body_aabb is not None
        and 0.042 <= (spray_body_aabb[1][2] - spray_body_aabb[0][2]) <= 0.058,
        details=f"spray_body aabb={spray_body_aabb}",
    )
    ctx.check(
        "spray body diameter ~0.032 m (ribbed profile)",
        spray_body_aabb is not None
        and 0.028 <= (spray_body_aabb[1][0] - spray_body_aabb[0][0]) <= 0.036,
        details=f"spray_body aabb={spray_body_aabb}",
    )

    # ---- distinct hollow outlet opening ----
    outlet = ctx.part_element_world_aabb(spray, elem="outlet_opening")
    ctx.check(
        "distinct hollow outlet opening inside the spray body bore",
        outlet is not None
        and spray_body_aabb is not None
        and outlet[0][2] >= spray_body_aabb[0][2] - 0.002
        and outlet[1][2] <= spray_body_aabb[1][2] - 0.005
        and 0.012 <= (outlet[1][0] - outlet[0][0]) <= 0.020,
        details=f"outlet={outlet}, spray_body={spray_body_aabb}",
    )

    # ---- spray head seats against dock collar at rest ----
    ctx.expect_contact(
        spray, spout,
        elem_a="spray_body", elem_b="dock_collar",
        contact_tol=0.003,
        name="spray head contacts dock collar at rest",
    )

    # ---- joint types and parameters ----
    ctx.check(
        "spout swivel is revolute about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )
    ctx.check(
        "spray_slide is prismatic along -Z, 0..0.06 m",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, -1.0)
        and slide.motion_limits is not None
        and abs(slide.motion_limits.lower) < 1e-6
        and abs(slide.motion_limits.upper - SLIDE_LIMIT) < 1e-3,
    )
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute about valve left-right axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6,
        )

    # ---- prismatic pose: spray head slides downward ----
    rest_pos = ctx.part_world_position(spray)
    with ctx.pose({slide: SLIDE_LIMIT}):
        ext_pos = ctx.part_world_position(spray)
    ctx.check(
        "spray head moves downward when slide extended",
        rest_pos is not None
        and ext_pos is not None
        and ext_pos[2] < rest_pos[2] - 0.04,
        details=f"rest_z={rest_pos[2] if rest_pos else None}, ext_z={ext_pos[2] if ext_pos else None}",
    )

    # ---- swivel pose: spout sweeps sideways ----
    rest_dock = ctx.part_element_world_aabb(spout, elem="dock_collar")
    with ctx.pose({swivel: 1.0}):
        sw_dock = ctx.part_element_world_aabb(spout, elem="dock_collar")
    ctx.check(
        "spout swivel sweeps dock collar sideways",
        rest_dock is not None
        and sw_dock is not None
        and abs(0.5 * (rest_dock[0][1] + rest_dock[1][1])) < 1e-6
        and abs(0.5 * (sw_dock[0][1] + sw_dock[1][1])) > 0.05,
        details=f"rest={rest_dock}, swiveled={sw_dock}",
    )

    # ---- lever pose: tilt toward user ----
    rest_0 = ctx.part_world_aabb(lever_0)
    with ctx.pose({pivot_0: -math.pi / 2.0}):
        tilted_0 = ctx.part_world_aabb(lever_0)
    ctx.check(
        "lever 0 tilts toward user at q=-90 deg",
        rest_0 is not None
        and tilted_0 is not None
        and tilted_0[1][0] > rest_0[1][0] + 0.08,
        details=f"rest={rest_0}, tilted={tilted_0}",
    )

    return ctx.report()


object_model = build_object_model()
