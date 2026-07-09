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
)

# ---------------------------------------------------------------------------
# Modern winged swivel lounge armchair with a matching ottoman.
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
    model = ArticulatedObject(name="winged_swivel_lounge_armchair_with_ottoman")

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
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.3, upper=0.0),
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
    ottoman = object_model.get_part("ottoman")
    swivel = object_model.get_articulation("swivel")
    recline = object_model.get_articulation("recline")

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
    ctx.allow_isolated_part(
        ottoman,
        reason="Prompt requires the ottoman as a fixed companion body offset in front of the chair; it stands apart on its own star base.",
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
        "chair stands about 1.05 m tall",
        rest_aabb is not None and 0.98 <= rest_aabb[1][2] <= 1.10,
        details=f"backrest_aabb={rest_aabb}",
    )

    # --- recline pose: backrest leans backward -----------------------------
    with ctx.pose({recline: -0.3}):
        leaned_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "recline -0.3 rad leans the backrest top backward and down",
        rest_aabb is not None
        and leaned_aabb is not None
        and leaned_aabb[0][0] < rest_aabb[0][0] - 0.08
        and leaned_aabb[1][2] < rest_aabb[1][2] - 0.02,
        details=f"rest={rest_aabb}, leaned={leaned_aabb}",
    )

    # --- swivel pose: whole shell rotates, ottoman stays put ----------------
    ottoman_pos0 = ctx.part_world_position(ottoman)
    with ctx.pose({swivel: math.pi / 2.0}):
        backrest_pos = ctx.part_world_position(backrest)
        ottoman_pos1 = ctx.part_world_position(ottoman)
    ctx.check(
        "swivel rotates the chair shell about the vertical column axis",
        backrest_pos is not None
        and abs(backrest_pos[0]) < 0.02
        and abs(backrest_pos[1] + abs(HINGE_X)) < 0.02,
        details=f"backrest_pos_at_quarter_turn={backrest_pos}",
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
