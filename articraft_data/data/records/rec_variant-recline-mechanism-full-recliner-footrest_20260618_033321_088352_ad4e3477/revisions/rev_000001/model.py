from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    place_on_face,
)

# ---------------------------------------------------------------------------
# Modern winged swivel lounge RECLINER with ottoman and deploying footrest.
# Variant: full_recliner_footrest
#
# Changes from parent armchair:
#  - Backrest recline range expanded to a full recliner sweep (~0 to -1.2 rad).
#  - New footrest part deploys from under the seat front on a PRISMATIC slide.
#  - Two coordinated live joints: recline tilt + footrest extension.
#  - Swivel base and ottoman companion retained.
#
# Chair faces +X; the ottoman sits in front of it along +X.
# Grey fabric cushions, tan/brown leather-look outer shells, matte black
# 4-spoke star pedestals.
# ---------------------------------------------------------------------------

COLUMN_TOP_Z = 0.27  # world height of the swivel joint / pedestal column top

BUCKET_W = 0.78  # seat bucket width  (Y)
BUCKET_D = 0.70  # seat bucket depth  (X)
BUCKET_WALL = 0.05
BUCKET_BOTTOM = 0.03  # seat-frame z of the bucket underside
BUCKET_FRONT_TOP = 0.14  # seat-frame z of the lowered front wall top

HINGE_X = -0.37  # recline hinge location in the seat frame
HINGE_Z = 0.18

BACK_TILT = math.radians(10.0)  # backrest rest rake, baked into the visuals
PANEL_T = 0.07
PANEL_W = 0.70
PANEL_H = 0.60

OTTOMAN_X = 0.78  # fixed ottoman offset in front of the chair

# Footrest geometry & joint parameters (seat-frame coordinates).
# The joint origin sits BELOW the bucket bottom so the stowed footrest panel
# clears the bucket shell in Z.
FOOTREST_JOINT_XYZ = (0.22, 0.0, -0.02)  # prismatic joint origin under seat front
FOOTREST_TRAVEL = 0.32  # max prismatic extension along +X (meters)
FOOTREST_PANEL_LENGTH = 0.34  # footrest cushion length (X)
FOOTREST_PANEL_WIDTH = 0.50  # footrest cushion width (Y)
FOOTREST_PANEL_THICK = 0.040  # footrest cushion thickness
# Guide blocks on the seat underside that capture the slide rails.
GUIDE_Z_CENTER = 0.02  # seat-frame z, chosen so guides share geometry with bucket bottom
GUIDE_Y_OFFSET = 0.20
# Slide-rail parameters (footrest-frame coordinates).
RAIL_Z_CENTER = 0.04  # footrest-frame z, aligns with guide blocks through the joint
RAIL_LENGTH = 0.54  # long enough to stay engaged at max travel
RAIL_Y_OFFSET = GUIDE_Y_OFFSET


def _spoke(
    root_r: float,
    length: float,
    w0: float,
    h0: float,
    z0_lo: float,
    z0_hi: float,
    w1: float,
    h1: float,
    z1_lo: float,
    z1_hi: float,
) -> cq.Workplane:
    """One tapered star-base spoke extending along +X, lofted from a tall root
    section near the hub to a thin flat tip that touches the floor."""
    zc0 = 0.5 * (z0_lo + z0_hi)
    zc1 = 0.5 * (z1_lo + z1_hi)
    return (
        cq.Workplane("YZ")
        .workplane(offset=root_r)
        .center(0.0, zc0)
        .rect(w0, h0)
        .workplane(offset=length)
        .center(0.0, zc1 - zc0)
        .rect(w1, h1)
        .loft()
    )


def _chair_spoke() -> cq.Workplane:
    return _spoke(0.02, 0.36, 0.075, 0.055, 0.010, 0.065, 0.034, 0.016, 0.0, 0.016)


def _ottoman_spoke() -> cq.Workplane:
    return _spoke(0.018, 0.242, 0.060, 0.045, 0.010, 0.055, 0.030, 0.014, 0.0, 0.014)


def _seat_bucket() -> cq.Workplane:
    """Deep curved bucket shell: rounded tub with thick walls, a lowered front
    wall, and arm-height side/rear walls."""
    outer = (
        cq.Workplane("XY")
        .box(BUCKET_D, BUCKET_W, 0.24, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.09)
    )
    pocket = (
        cq.Workplane("XY")
        .box(BUCKET_D - 2 * BUCKET_WALL, BUCKET_W - 2 * BUCKET_WALL, 0.30, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.05)
        .translate((0.0, 0.0, BUCKET_WALL))
    )
    front_cut = (
        cq.Workplane("XY")
        .box(0.16, BUCKET_W + 0.12, 0.30, centered=(True, True, False))
        .translate((BUCKET_D / 2.0, 0.0, BUCKET_FRONT_TOP - BUCKET_BOTTOM))
    )
    return outer.cut(pocket).cut(front_cut).translate((0.0, 0.0, BUCKET_BOTTOM))


def _seat_cushion() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.58, 0.66, 0.13, centered=(True, True, False))
        .edges()
        .fillet(0.035)
        .translate((0.0, 0.0, 0.07))
    )


def _back_panel() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PANEL_T, PANEL_W, PANEL_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.03)
        .translate((-PANEL_T / 2.0, 0.0, 0.0))
    )


def _wing(side: float) -> cq.Workplane:
    """Angled wing panel at the top of the backrest, canted forward/outward."""
    return (
        cq.Workplane("XY")
        .box(0.05, 0.18, 0.27)
        .edges("|Z")
        .fillet(0.015)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), -35.0 * side)
        .translate((0.0, 0.345 * side, 0.465))
    )


def _back_cushion() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.06, 0.56, 0.48, centered=(True, True, False))
        .edges()
        .fillet(0.025)
        .translate((0.018, 0.0, 0.10))
    )


def _footrest_panel() -> cq.Workplane:
    """Padded footrest cushion — a chamfered slab that stows flat under the
    seat front and deploys forward when the slide extends."""
    slab = (
        cq.Workplane("XY")
        .box(FOOTREST_PANEL_LENGTH, FOOTREST_PANEL_WIDTH, FOOTREST_PANEL_THICK, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z or <Z")
        .fillet(0.008)
    )
    # Place the near edge at local x=0 (the joint origin) and the top surface
    # at local z=0 so the panel hangs below the slide axis. The panel center
    # ends up at (LENGTH/2, 0, -THICK).
    slab = slab.translate((FOOTREST_PANEL_LENGTH / 2.0, 0.0, -FOOTREST_PANEL_THICK))
    return slab


def _footrest_rail(side: float) -> cq.Workplane:
    """One telescoping slide rail with an integral mounting flange.

    The horizontal bar runs at the guide-block height so it slides through
    the seat guides. The vertical flange drops down to overlap with the
    footrest panel top, providing mesh connectivity within the footrest part.
    """
    bar_z_lo = RAIL_Z_CENTER - 0.009  # bottom of the horizontal bar
    # Horizontal slide bar: thin and long, at the guide-block height.
    bar = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, 0.022, 0.018, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((-0.08, RAIL_Y_OFFSET * side, bar_z_lo))
    )
    # Vertical mounting flange: drops from inside the bar down into the panel
    # top (panel extends from z=-0.04 to z=0 in footrest frame). Extends 4 mm
    # into the bar for mesh connectivity.
    flange_z_lo = -0.005  # 5 mm into the panel for mesh connectivity
    flange_z_hi = bar_z_lo + 0.004  # 4 mm into the bar for connectivity
    flange_h = flange_z_hi - flange_z_lo
    flange = (
        cq.Workplane("XY")
        .box(0.14, 0.022, flange_h, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((-0.04, RAIL_Y_OFFSET * side, flange_z_lo))
    )
    return bar.union(flange)


def _ottoman_tray() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.41, 0.56, 0.05, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.06)
        .translate((0.0, 0.0, 0.21))
    )


def _ottoman_cushion() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.45, 0.60, 0.15, centered=(True, True, False))
        .edges()
        .fillet(0.04)
        .translate((0.0, 0.0, 0.25))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="winged_swivel_recliner_with_ottoman_and_footrest")

    fabric = model.material("fabric_grey", rgba=(0.52, 0.50, 0.47, 1.0))
    leather = model.material("leather_tan", rgba=(0.58, 0.42, 0.27, 1.0))
    metal = model.material("metal_black", rgba=(0.10, 0.10, 0.10, 1.0))

    # ---------------------------------------------------------------- pedestal
    pedestal = model.part("pedestal")
    chair_spoke_mesh = mesh_from_cadquery(_chair_spoke(), "base_spoke")
    for i in range(4):
        pedestal.visual(
            chair_spoke_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.radians(45.0 + 90.0 * i))),
            material=metal,
            name=f"base_spoke_{i}",
        )
    pedestal.visual(
        Cylinder(0.055, 0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=metal,
        name="base_hub",
    )
    pedestal.visual(
        Cylinder(0.034, 0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=metal,
        name="pedestal_column",
    )

    # -------------------------------------------------------------------- seat
    # Seat frame origin sits at the column top (swivel joint).
    seat = model.part("seat")
    seat.visual(
        Cylinder(0.055, 0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=metal,
        name="swivel_boss",
    )
    seat.visual(mesh_from_cadquery(_seat_bucket(), "seat_bucket"), material=leather, name="seat_bucket")
    seat.visual(mesh_from_cadquery(_seat_cushion(), "seat_cushion"), material=fabric, name="seat_cushion")
    for i, sy in enumerate((1.0, -1.0)):
        seat.visual(
            Box((0.055, 0.04, 0.06)),
            origin=Origin(xyz=(-0.3575, 0.28 * sy, HINGE_Z)),
            material=metal,
            name=f"hinge_bracket_{i}",
        )
    # Footrest slide housing: two guide blocks bolted to the bucket underside
    # that capture the slide rails. Placed at z=GUIDE_Z_CENTER so they share
    # geometry with the bucket bottom plate (connectivity).
    for i, sy in enumerate((1.0, -1.0)):
        seat.visual(
            Box((0.10, 0.034, 0.028)),
            origin=Origin(xyz=(0.22, GUIDE_Y_OFFSET * sy, GUIDE_Z_CENTER)),
            material=metal,
            name=f"footrest_guide_{i}",
        )

    # ---------------------------------------------------------------- backrest
    # Backrest frame origin sits on the recline hinge line; the rest rake is
    # baked into the visuals via a pitch rotation.
    backrest = model.part("backrest")
    tilt = (0.0, -BACK_TILT, 0.0)
    backrest.visual(
        mesh_from_cadquery(_back_panel(), "backrest_panel"),
        origin=Origin(rpy=tilt),
        material=leather,
        name="backrest_shell",
    )
    for i, sy in enumerate((1.0, -1.0)):
        backrest.visual(
            mesh_from_cadquery(_wing(sy), f"wing_{i}"),
            origin=Origin(rpy=tilt),
            material=leather,
            name=f"wing_{i}",
        )
    backrest.visual(
        mesh_from_cadquery(_back_cushion(), "back_cushion"),
        origin=Origin(rpy=tilt),
        material=fabric,
        name="back_cushion",
    )

    # ---------------------------------------------------------------- footrest
    # Footrest frame origin sits at the prismatic joint origin under the seat
    # front. Positive q translates the assembly along +X (forward, out from
    # under the seat).
    footrest = model.part("footrest")
    footrest.visual(
        mesh_from_cadquery(_footrest_panel(), "footrest_panel"),
        material=fabric,
        name="footrest_panel",
    )
    for i, sy in enumerate((1.0, -1.0)):
        footrest.visual(
            mesh_from_cadquery(_footrest_rail(sy), f"footrest_rail_{i}"),
            material=metal,
            name=f"footrest_rail_{i}",
        )

    # ----------------------------------------------------------------- ottoman
    ottoman = model.part("ottoman")
    ottoman_spoke_mesh = mesh_from_cadquery(_ottoman_spoke(), "ottoman_spoke")
    for i in range(4):
        ottoman.visual(
            ottoman_spoke_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.radians(45.0 + 90.0 * i))),
            material=metal,
            name=f"ottoman_spoke_{i}",
        )
    ottoman.visual(
        Cylinder(0.05, 0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=metal,
        name="ottoman_hub",
    )
    ottoman.visual(
        Cylinder(0.028, 0.17),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=metal,
        name="ottoman_column",
    )
    ottoman.visual(mesh_from_cadquery(_ottoman_tray(), "ottoman_tray"), material=leather, name="ottoman_tray")
    ottoman.visual(
        mesh_from_cadquery(_ottoman_cushion(), "ottoman_cushion"),
        material=fabric,
        name="ottoman_cushion",
    )

    # ----------------------------------------------------------- articulations
    model.articulation(
        "swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0),
    )
    model.articulation(
        "recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # Negative q about +Y leans the backrest top backward (-X).
        # Full-recliner range: 0 (upright/raked) to -1.2 rad (~69° additional
        # recline past the baked 10° rake, ~79° total from vertical).
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.8, lower=-1.2, upper=0.0),
    )
    model.articulation(
        "footrest_slide",
        ArticulationType.PRISMATIC,
        parent=seat,
        child=footrest,
        origin=Origin(xyz=FOOTREST_JOINT_XYZ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.3, lower=0.0, upper=FOOTREST_TRAVEL),
    )
    model.articulation(
        "ottoman_mount",
        ArticulationType.FIXED,
        parent=pedestal,
        child=ottoman,
        origin=Origin(xyz=(OTTOMAN_X, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")
    ottoman = object_model.get_part("ottoman")
    swivel = object_model.get_articulation("swivel")
    recline = object_model.get_articulation("recline")
    footrest_slide = object_model.get_articulation("footrest_slide")

    # Intentional local embeddings of the mechanism interfaces.
    ctx.allow_overlap(
        seat,
        pedestal,
        elem_a="swivel_boss",
        elem_b="pedestal_column",
        reason="The pedestal column tip is intentionally seated inside the seat swivel boss (captured shaft).",
    )
    ctx.allow_overlap(
        backrest,
        seat,
        elem_a="backrest_shell",
        elem_b="hinge_bracket_0",
        reason="The recline hinge bracket intentionally captures the backrest shell at the pivot line.",
    )
    ctx.allow_overlap(
        backrest,
        seat,
        elem_a="backrest_shell",
        elem_b="hinge_bracket_1",
        reason="The recline hinge bracket intentionally captures the backrest shell at the pivot line.",
    )
    # Footrest rails slide through the seat-mounted guide blocks.
    ctx.allow_overlap(
        footrest,
        seat,
        elem_a="footrest_rail_0",
        elem_b="footrest_guide_0",
        reason="The footrest slide rail is intentionally captured inside the seat guide block (prismatic sleeve).",
    )
    ctx.allow_overlap(
        footrest,
        seat,
        elem_a="footrest_rail_1",
        elem_b="footrest_guide_1",
        reason="The footrest slide rail is intentionally captured inside the seat guide block (prismatic sleeve).",
    )
    # The slide rails also pass close to the bucket underside (mesh tessellation
    # contact at the mechanism mounting interface).
    ctx.allow_overlap(
        footrest,
        seat,
        elem_a="footrest_rail_0",
        elem_b="seat_bucket",
        reason="The footrest slide rail runs along the bucket underside as part of the prismatic slide mechanism.",
    )
    ctx.allow_overlap(
        footrest,
        seat,
        elem_a="footrest_rail_1",
        elem_b="seat_bucket",
        reason="The footrest slide rail runs along the bucket underside as part of the prismatic slide mechanism.",
    )
    ctx.allow_isolated_part(
        ottoman,
        reason="Prompt requires the ottoman as a fixed companion body offset in front of the chair; it stands apart on its own star base.",
    )

    # --- variant mechanism: two live non-fixed joints -------------------
    recline_art = object_model.get_articulation("recline")
    slide_art = object_model.get_articulation("footrest_slide")
    ctx.check(
        "recline joint exists and is REVOLUTE",
        recline_art.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={recline_art.articulation_type}",
    )
    ctx.check(
        "footrest slide joint exists and is PRISMATIC",
        slide_art.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide_art.articulation_type}",
    )
    ctx.check(
        "recline joint has full-recliner range (at least 1.0 rad)",
        recline_art.motion_limits.lower is not None
        and recline_art.motion_limits.upper is not None
        and (recline_art.motion_limits.upper - recline_art.motion_limits.lower) >= 1.0,
        details=f"limits=({recline_art.motion_limits.lower}, {recline_art.motion_limits.upper})",
    )
    ctx.check(
        "footrest prismatic has meaningful travel (at least 0.25 m)",
        slide_art.motion_limits.upper is not None
        and slide_art.motion_limits.lower is not None
        and (slide_art.motion_limits.upper - slide_art.motion_limits.lower) >= 0.25,
        details=f"limits=({slide_art.motion_limits.lower}, {slide_art.motion_limits.upper})",
    )

    # --- star bases -------------------------------------------------------
    chair_spokes = sum(1 for v in pedestal.visuals if v.name and v.name.startswith("base_spoke"))
    ctx.check("chair pedestal has a 4-spoke star base", chair_spokes == 4, details=f"spokes={chair_spokes}")
    ottoman_spokes = sum(1 for v in ottoman.visuals if v.name and v.name.startswith("ottoman_spoke"))
    ctx.check("ottoman has its own 4-spoke star base", ottoman_spokes == 4, details=f"spokes={ottoman_spokes}")

    ped_aabb = ctx.part_world_aabb(pedestal)
    ctx.check(
        "star base spoke tips rest on the floor",
        ped_aabb is not None and -0.002 <= ped_aabb[0][2] <= 0.004,
        details=f"pedestal_aabb={ped_aabb}",
    )
    ot_aabb = ctx.part_world_aabb(ottoman)
    ctx.check(
        "ottoman base rests on the floor and stands about 0.4 m tall",
        ot_aabb is not None and -0.002 <= ot_aabb[0][2] <= 0.004 and 0.36 <= ot_aabb[1][2] <= 0.44,
        details=f"ottoman_aabb={ot_aabb}",
    )

    # --- swivel column seated under the seat shell -------------------------
    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="swivel_boss",
        negative_elem="pedestal_column",
        max_gap=0.0,
        max_penetration=0.02,
        name="seat swivel boss seats over the pedestal column tip",
    )

    # --- recline hinge brackets capture the backrest shell -----------------
    ctx.expect_overlap(
        backrest,
        seat,
        axes="xz",
        elem_a="backrest_shell",
        elem_b="hinge_bracket_0",
        min_overlap=0.004,
        name="hinge bracket engages the backrest shell",
    )
    ctx.expect_overlap(
        backrest,
        seat,
        axes="xz",
        elem_a="backrest_shell",
        elem_b="hinge_bracket_1",
        min_overlap=0.004,
        name="second hinge bracket engages the backrest shell",
    )

    # --- footrest geometry claims -----------------------------------------
    footrest_panel_aabb = ctx.part_element_world_aabb(footrest, elem="footrest_panel")
    ctx.check(
        "footrest has a padded panel visual",
        footrest_panel_aabb is not None,
        details=f"footrest_panel_aabb={footrest_panel_aabb}",
    )
    footrest_rails = sum(
        1 for v in footrest.visuals if v.name and v.name.startswith("footrest_rail")
    )
    ctx.check(
        "footrest has two slide rails",
        footrest_rails == 2,
        details=f"rails={footrest_rails}",
    )
    # Footrest guide blocks on the seat prove the rails are captured.
    seat_guides = sum(
        1 for v in seat.visuals if v.name and v.name.startswith("footrest_guide")
    )
    ctx.check(
        "seat has two footrest guide blocks that capture the slide rails",
        seat_guides == 2,
        details=f"guides={seat_guides}",
    )
    # Rails slide through the guides: prove lateral centering on Y (the slide
    # axis X is proved by the deployment pose checks below). Z containment is
    # intentionally relaxed because the rail includes a mounting flange that
    # drops below the guide to connect to the panel.
    for i in range(2):
        ctx.expect_within(
            footrest,
            seat,
            axes="y",
            inner_elem=f"footrest_rail_{i}",
            outer_elem=f"footrest_guide_{i}",
            margin=0.012,
            name=f"footrest rail {i} is laterally centered in its guide block",
        )

    # --- footrest deployed pose: panel translates forward ---------------
    rest_panel_pos = ctx.part_world_position(footrest)
    with ctx.pose({footrest_slide: FOOTREST_TRAVEL}):
        extended_panel_pos = ctx.part_world_position(footrest)
        extended_panel_aabb = ctx.part_element_world_aabb(footrest, elem="footrest_panel")
    ctx.check(
        "footrest deploys forward along +X when the slide extends",
        rest_panel_pos is not None
        and extended_panel_pos is not None
        and extended_panel_pos[0] > rest_panel_pos[0] + 0.20,
        details=f"rest={rest_panel_pos}, extended={extended_panel_pos}",
    )
    ctx.check(
        "footrest lateral position is stable during deployment (no Y drift)",
        rest_panel_pos is not None
        and extended_panel_pos is not None
        and abs(extended_panel_pos[1] - rest_panel_pos[1]) < 1e-6,
        details=f"rest_y={rest_panel_pos[1] if rest_panel_pos else None}, extended_y={extended_panel_pos[1] if extended_panel_pos else None}",
    )
    ctx.check(
        "deployed footrest panel reaches well in front of the seat bucket",
        extended_panel_aabb is not None
        and extended_panel_aabb[1][0] > 0.55,
        details=f"extended_panel_aabb={extended_panel_aabb}",
    )

    # --- cushions ----------------------------------------------------------
    bucket_aabb = ctx.part_element_world_aabb(seat, elem="seat_bucket")
    cushion_aabb = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    ctx.check(
        "seat cushion nests inside the bucket shell",
        bucket_aabb is not None
        and cushion_aabb is not None
        and cushion_aabb[0][0] >= bucket_aabb[0][0]
        and cushion_aabb[1][0] <= bucket_aabb[1][0]
        and cushion_aabb[0][1] >= bucket_aabb[0][1]
        and cushion_aabb[1][1] <= bucket_aabb[1][1],
        details=f"bucket={bucket_aabb}, cushion={cushion_aabb}",
    )
    ctx.check(
        "seat cushion sits on the bucket floor and rises above the lowered front wall",
        bucket_aabb is not None
        and cushion_aabb is not None
        and bucket_aabb[0][2] < cushion_aabb[0][2] < bucket_aabb[0][2] + 0.06
        and cushion_aabb[1][2] > COLUMN_TOP_Z + BUCKET_FRONT_TOP + 0.02,
        details=f"bucket={bucket_aabb}, cushion={cushion_aabb}",
    )
    back_cushion_aabb = ctx.part_element_world_aabb(backrest, elem="back_cushion")
    ctx.check(
        "back cushion covers the backrest up between the wings",
        back_cushion_aabb is not None
        and back_cushion_aabb[1][2] > 0.95
        and back_cushion_aabb[0][2] > 0.52,
        details=f"back_cushion={back_cushion_aabb}",
    )

    # --- winged top --------------------------------------------------------
    wing0_aabb = ctx.part_element_world_aabb(backrest, elem="wing_0")
    wing1_aabb = ctx.part_element_world_aabb(backrest, elem="wing_1")
    ctx.check(
        "two angled wing panels flare at the top of the backrest",
        wing0_aabb is not None
        and wing1_aabb is not None
        and wing0_aabb[1][2] > 0.98
        and wing1_aabb[1][2] > 0.98
        and wing0_aabb[1][1] > 0.40
        and wing1_aabb[0][1] < -0.40,
        details=f"wing_0={wing0_aabb}, wing_1={wing1_aabb}",
    )

    rest_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "chair stands about 1.05 m tall at rest",
        rest_aabb is not None and 0.98 <= rest_aabb[1][2] <= 1.10,
        details=f"backrest_aabb={rest_aabb}",
    )

    # --- full-recliner pose: backrest leans far back ---------------------
    with ctx.pose({recline: -1.1}):
        deep_lean_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "full-recliner pose (q=-1.1 rad) drops the backrest top far behind and below the rest pose",
        rest_aabb is not None
        and deep_lean_aabb is not None
        and deep_lean_aabb[0][0] < rest_aabb[0][0] - 0.40
        and deep_lean_aabb[1][2] < rest_aabb[1][2] - 0.25,
        details=f"rest={rest_aabb}, deep_lean={deep_lean_aabb}",
    )

    # --- coordinated recline + footrest deployment -----------------------
    with ctx.pose({recline: -0.9, footrest_slide: FOOTREST_TRAVEL}):
        coordinated_backrest = ctx.part_world_aabb(backrest)
        coordinated_footrest = ctx.part_world_aabb(footrest)
    ctx.check(
        "coordinated recline+footrest pose: backrest leans back and footrest extends forward simultaneously",
        coordinated_backrest is not None
        and coordinated_footrest is not None
        and coordinated_backrest[0][0] < rest_aabb[0][0] - 0.20
        and coordinated_footrest[1][0] > 0.55,
        details=f"backrest={coordinated_backrest}, footrest={coordinated_footrest}",
    )

    # --- swivel pose: whole shell (incl. footrest) rotates, ottoman stays -
    ottoman_pos0 = ctx.part_world_position(ottoman)
    with ctx.pose({swivel: math.pi / 2.0}):
        backrest_pos = ctx.part_world_position(backrest)
        footrest_pos = ctx.part_world_position(footrest)
        ottoman_pos1 = ctx.part_world_position(ottoman)
    ctx.check(
        "swivel rotates the chair shell about the vertical column axis",
        backrest_pos is not None
        and abs(backrest_pos[0]) < 0.02
        and abs(backrest_pos[1] + abs(HINGE_X)) < 0.02,
        details=f"backrest_pos_at_quarter_turn={backrest_pos}",
    )
    ctx.check(
        "footrest swivels with the seat (it is mounted to the seat, not the pedestal)",
        footrest_pos is not None
        and abs(footrest_pos[0]) < 0.05
        and abs(footrest_pos[1] - FOOTREST_JOINT_XYZ[0]) < 0.05,
        details=f"footrest_at_quarter_turn={footrest_pos}",
    )
    ctx.check(
        "ottoman stays fixed while the chair swivels",
        ottoman_pos0 is not None
        and ottoman_pos1 is not None
        and abs(ottoman_pos0[0] - ottoman_pos1[0]) < 1e-9
        and abs(ottoman_pos0[1] - ottoman_pos1[1]) < 1e-9,
        details=f"before={ottoman_pos0}, after={ottoman_pos1}",
    )

    # --- ottoman placement --------------------------------------------------
    ctx.expect_origin_gap(
        ottoman,
        pedestal,
        axis="x",
        min_gap=0.60,
        max_gap=0.95,
        name="ottoman is offset in front of the chair",
    )
    ctx.expect_gap(
        ottoman,
        seat,
        axis="x",
        min_gap=0.01,
        name="ottoman clears the chair seat shell",
    )

    return ctx.report()


object_model = build_object_model()
