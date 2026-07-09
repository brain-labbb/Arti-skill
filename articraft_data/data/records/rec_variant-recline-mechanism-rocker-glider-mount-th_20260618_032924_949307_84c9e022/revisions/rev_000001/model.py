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
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

# ---------------------------------------------------------------------------
# Modern winged swivel lounge armchair on a curved rocker/glider base,
# with a matching ottoman.
# Chair faces +X; the ottoman sits in front of it along +X.
# Grey fabric cushions, tan/brown leather-look outer shells, matte black
# metal base and frame.
#
# Variant: rocker_glider
#   - Curved rocker arcs under the seat instead of a star pedestal
#   - REVOLUTE rocking joint (horizontal Y axis, low under the seat)
#   - CONTINUOUS swivel preserved above the carriage
#   - REVOLUTE recline preserved on the backrest
# ---------------------------------------------------------------------------

# ---- rocker geometry constants -------------------------------------------
ROCK_JOINT_Z = 0.15          # rock pivot height in rocker_base frame (m)
RAIL_RADIUS = 0.60           # rocker arc radius (m)
RAIL_CENTER_Z_LOCAL = 0.509  # arc center Z in carriage local frame (m)
# Arc bottom in carriage local z = RAIL_CENTER_Z_LOCAL - RAIL_RADIUS = -0.091
# Rail bottom (with profile) = -0.091 - 0.019 = -0.110
# Rail bottom in world z = ROCK_JOINT_Z + (-0.110) = 0.040 (touching base runners)
RAIL_HALF_ANGLE_DEG = 20.0   # half-span of the visible arc (degrees)
RAIL_Y_OFFSET = 0.28         # lateral offset of each rail from center (m)

SWIVEL_Z_LOCAL = 0.23        # swivel joint Z in carriage local frame (m)
# Swivel world z = ROCK_JOINT_Z + SWIVEL_Z_LOCAL = 0.38

# ---- seat / backrest constants (unchanged from parent) -------------------
BUCKET_W = 0.78
BUCKET_D = 0.70
BUCKET_WALL = 0.05
BUCKET_BOTTOM = 0.03
BUCKET_FRONT_TOP = 0.14

HINGE_X = -0.37
HINGE_Z = 0.18

BACK_TILT = math.radians(10.0)
PANEL_T = 0.07
PANEL_W = 0.70
PANEL_H = 0.60

OTTOMAN_X = 0.78

ROCK_LIMIT = 0.12            # ±0.12 rad rocking range


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _rocker_arc_points(
    center_z: float,
    radius: float,
    half_angle_deg: float,
    n: int = 11,
    y_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    """Generate 3D points along a vertical arc in the XZ plane at a given Y."""
    pts: list[tuple[float, float, float]] = []
    for i in range(n):
        t = math.radians(-half_angle_deg + 2.0 * half_angle_deg * i / (n - 1))
        x = radius * math.sin(t)
        z = center_z - radius * math.cos(t)
        pts.append((x, y_offset, z))
    return pts


def _rocker_rail_mesh(y_sign: float) -> "Mesh":
    """One curved rocker rail as a managed mesh, swept rounded-rect profile."""
    pts = _rocker_arc_points(
        RAIL_CENTER_Z_LOCAL, RAIL_RADIUS, RAIL_HALF_ANGLE_DEG,
        y_offset=RAIL_Y_OFFSET * y_sign,
    )
    geom = sweep_profile_along_spline(
        pts,
        profile=rounded_rect_profile(0.050, 0.038, radius=0.010),
        samples_per_segment=10,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    name = "rocker_rail_left" if y_sign < 0 else "rocker_rail_right"
    return mesh_from_geometry(geom, name)


def _base_plate() -> cq.Workplane:
    """Fixed floor runner plate under the rocker arcs."""
    return (
        cq.Workplane("XY")
        .box(0.62, 0.64, 0.018, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.04)
    )


def _base_runner() -> cq.Workplane:
    """Low guide runner ridge on the base plate (one side)."""
    return (
        cq.Workplane("XY")
        .box(0.56, 0.050, 0.022, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .edges(">Z")
        .fillet(0.008)
    )


def _cross_bar() -> cq.Workplane:
    """Horizontal cross bar connecting the two rocker arcs."""
    return (
        cq.Workplane("XY")
        .box(0.040, 2.0 * RAIL_Y_OFFSET + 0.04, 0.030, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )


def _bridge_plate() -> cq.Workplane:
    """Central bridge plate spanning between the two rails, supporting the column."""
    return (
        cq.Workplane("XY")
        .box(0.12, 2.0 * RAIL_Y_OFFSET + 0.02, 0.025, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.006)
    )





def _seat_bucket() -> cq.Workplane:
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


def _ottoman_spoke() -> cq.Workplane:
    """Tapered spoke for the ottoman star base (unchanged from parent)."""
    root_r, length = 0.018, 0.242
    w0, h0 = 0.060, 0.045
    z0_lo, z0_hi = 0.010, 0.055
    w1, h1 = 0.030, 0.014
    z1_lo, z1_hi = 0.0, 0.014
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


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="winged_rocker_lounge_armchair_with_ottoman")

    fabric = model.material("fabric_grey", rgba=(0.52, 0.50, 0.47, 1.0))
    leather = model.material("leather_tan", rgba=(0.58, 0.42, 0.27, 1.0))
    metal = model.material("metal_black", rgba=(0.10, 0.10, 0.10, 1.0))

    # ----------------------------------------------------------- rocker_base
    # Fixed floor runner — root part.
    rocker_base = model.part("rocker_base")
    rocker_base.visual(
        mesh_from_cadquery(_base_plate(), "base_plate"),
        material=metal,
        name="base_plate",
    )
    for i, sy in enumerate((1.0, -1.0)):
        rocker_base.visual(
            mesh_from_cadquery(_base_runner(), "base_runner"),
            origin=Origin(xyz=(0.0, RAIL_Y_OFFSET * sy, 0.018)),
            material=metal,
            name=f"base_runner_{i}",
        )

    # ------------------------------------------------------------ carriage
    # Moving rocker frame: curved arcs + cross bars + column + hub.
    # Carriage origin sits at the rock joint = (0, 0, ROCK_JOINT_Z) in base frame.
    carriage = model.part("carriage")

    # Two curved rocker arcs
    for i, sy in enumerate((1.0, -1.0)):
        carriage.visual(
            _rocker_rail_mesh(sy),
            material=metal,
            name=f"rocker_rail_{i}",
        )

    # Front and rear cross bars connecting the arcs
    end_angle = math.radians(RAIL_HALF_ANGLE_DEG)
    end_x = RAIL_RADIUS * math.sin(end_angle)
    end_z = RAIL_CENTER_Z_LOCAL - RAIL_RADIUS * math.cos(end_angle)
    for i, sx in enumerate((1.0, -1.0)):
        carriage.visual(
            mesh_from_cadquery(_cross_bar(), "cross_bar"),
            origin=Origin(xyz=(sx * end_x, 0.0, end_z)),
            material=metal,
            name=f"cross_bar_{i}",
        )

    # Bridge plate spanning between the rails, positioned above the arc bottoms
    # The arc at x=0 is at local z = RAIL_CENTER_Z_LOCAL - RAIL_RADIUS = -0.08
    # Rail top at x=0: local z = -0.08 + 0.019 = -0.061
    arc_bottom_local_z = RAIL_CENTER_Z_LOCAL - RAIL_RADIUS
    rail_top_at_center = arc_bottom_local_z + 0.019  # half of profile height
    bridge_plate_bottom = -0.0125  # bridge plate centered at z=0, so bottom at -0.0125
    carriage.visual(
        mesh_from_cadquery(_bridge_plate(), "bridge_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal,
        name="bridge_plate",
    )

    # Vertical struts from rails up to the bridge plate (one per side)
    strut_height = bridge_plate_bottom - rail_top_at_center  # from rail top to bridge bottom
    for i, sy in enumerate((1.0, -1.0)):
        strut_center_z = rail_top_at_center + strut_height / 2.0
        carriage.visual(
            Box((0.025, 0.025, strut_height)),
            origin=Origin(xyz=(0.0, RAIL_Y_OFFSET * sy, strut_center_z)),
            material=metal,
            name=f"vertical_strut_{i}",
        )

    # Column rising from the bridge plate to the swivel level
    column_base_z = 0.0125  # top of bridge plate (centered at z=0, thickness 0.025)
    column_height = SWIVEL_Z_LOCAL - column_base_z
    carriage.visual(
        Cylinder(0.034, column_height),
        origin=Origin(xyz=(0.0, 0.0, column_base_z + column_height / 2.0)),
        material=metal,
        name="carriage_column",
    )

    # --------------------------------------------------------------- seat
    # Seat frame origin sits at the swivel joint (column top).
    seat = model.part("seat")
    seat.visual(
        Cylinder(0.055, 0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=metal,
        name="swivel_boss",
    )
    seat.visual(
        mesh_from_cadquery(_seat_bucket(), "seat_bucket"),
        material=leather,
        name="seat_bucket",
    )
    seat.visual(
        mesh_from_cadquery(_seat_cushion(), "seat_cushion"),
        material=fabric,
        name="seat_cushion",
    )
    for i, sy in enumerate((1.0, -1.0)):
        seat.visual(
            Box((0.055, 0.04, 0.06)),
            origin=Origin(xyz=(-0.3575, 0.28 * sy, HINGE_Z)),
            material=metal,
            name=f"hinge_bracket_{i}",
        )

    # ---------------------------------------------------------- backrest
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

    # ----------------------------------------------------------- ottoman
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
    ottoman.visual(
        mesh_from_cadquery(_ottoman_tray(), "ottoman_tray"),
        material=leather,
        name="ottoman_tray",
    )
    ottoman.visual(
        mesh_from_cadquery(_ottoman_cushion(), "ottoman_cushion"),
        material=fabric,
        name="ottoman_cushion",
    )

    # ----------------------------------------------------- articulations

    # Rock: carriage rocks on the fixed base (horizontal Y axis, low under seat)
    model.articulation(
        "rock",
        ArticulationType.REVOLUTE,
        parent=rocker_base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, ROCK_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=1.5,
            lower=-ROCK_LIMIT, upper=ROCK_LIMIT,
        ),
    )

    # Swivel: seat rotates on the carriage column (vertical Z axis)
    model.articulation(
        "swivel",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z_LOCAL)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0),
    )

    # Recline: backrest leans relative to the seat
    model.articulation(
        "recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.3, upper=0.0),
    )

    # Ottoman: fixed companion body
    model.articulation(
        "ottoman_mount",
        ArticulationType.FIXED,
        parent=rocker_base,
        child=ottoman,
        origin=Origin(xyz=(OTTOMAN_X, 0.0, 0.0)),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rocker_base = object_model.get_part("rocker_base")
    carriage = object_model.get_part("carriage")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    ottoman = object_model.get_part("ottoman")
    rock = object_model.get_articulation("rock")
    swivel = object_model.get_articulation("swivel")
    recline = object_model.get_articulation("recline")

    # --- intentional local embeddings ----------------------------------
    ctx.allow_overlap(
        seat,
        carriage,
        elem_a="swivel_boss",
        elem_b="carriage_column",
        reason="The carriage column tip is intentionally seated inside the seat swivel boss (captured shaft).",
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
    ctx.allow_overlap(
        carriage,
        rocker_base,
        elem_a="rocker_rail_0",
        elem_b="base_runner_0",
        reason="The curved rocker rail intentionally rests on the base runner at the contact surface for rocking motion.",
    )
    ctx.allow_overlap(
        carriage,
        rocker_base,
        elem_a="rocker_rail_1",
        elem_b="base_runner_1",
        reason="The curved rocker rail intentionally rests on the base runner at the contact surface for rocking motion.",
    )
    
    # Proof checks for rocker rail contact with base runners
    ctx.expect_contact(
        carriage,
        rocker_base,
        elem_a="rocker_rail_0",
        elem_b="base_runner_0",
        contact_tol=0.01,
        name="rocker rail 0 contacts base runner 0",
    )
    ctx.expect_contact(
        carriage,
        rocker_base,
        elem_a="rocker_rail_1",
        elem_b="base_runner_1",
        contact_tol=0.01,
        name="rocker rail 1 contacts base runner 1",
    )
    
    ctx.allow_isolated_part(
        ottoman,
        reason="Prompt requires the ottoman as a fixed companion body offset in front of the chair; it stands apart on its own star base.",
    )

    # --- rocker base visible geometry ----------------------------------
    rail_count = sum(
        1 for v in carriage.visuals if v.name and v.name.startswith("rocker_rail")
    )
    ctx.check(
        "carriage has two curved rocker rails",
        rail_count == 2,
        details=f"rocker_rails={rail_count}",
    )
    cross_count = sum(
        1 for v in carriage.visuals if v.name and v.name.startswith("cross_bar")
    )
    ctx.check(
        "carriage has front and rear cross bars",
        cross_count == 2,
        details=f"cross_bars={cross_count}",
    )

    base_aabb = ctx.part_world_aabb(rocker_base)
    ctx.check(
        "rocker base rests on the floor",
        base_aabb is not None and -0.002 <= base_aabb[0][2] <= 0.004,
        details=f"rocker_base_aabb={base_aabb}",
    )

    # --- rocker rail arcs are visibly curved (not flat boxes) -----------
    rail0_aabb = ctx.part_element_world_aabb(carriage, elem="rocker_rail_0")
    rail1_aabb = ctx.part_element_world_aabb(carriage, elem="rocker_rail_1")
    ctx.check(
        "rocker rails extend front-to-back and rise above the floor",
        rail0_aabb is not None
        and rail1_aabb is not None
        and (rail0_aabb[1][0] - rail0_aabb[0][0]) > 0.30
        and rail0_aabb[0][2] < 0.06
        and rail0_aabb[1][2] > 0.04,
        details=f"rail_0={rail0_aabb}, rail_1={rail1_aabb}",
    )

    # --- swivel boss seated on column ------------------------------------
    ctx.expect_gap(
        seat,
        carriage,
        axis="z",
        positive_elem="swivel_boss",
        negative_elem="carriage_column",
        max_gap=0.0,
        max_penetration=0.025,
        name="seat swivel boss seats over the carriage column tip",
    )

    # --- recline hinge brackets ------------------------------------------
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

    # --- cushions --------------------------------------------------------
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

    back_cushion_aabb = ctx.part_element_world_aabb(backrest, elem="back_cushion")
    ctx.check(
        "back cushion covers the backrest up between the wings",
        back_cushion_aabb is not None
        and back_cushion_aabb[1][2] > 0.80
        and back_cushion_aabb[0][2] > 0.40,
        details=f"back_cushion={back_cushion_aabb}",
    )

    # --- winged top ------------------------------------------------------
    wing0_aabb = ctx.part_element_world_aabb(backrest, elem="wing_0")
    wing1_aabb = ctx.part_element_world_aabb(backrest, elem="wing_1")
    ctx.check(
        "two angled wing panels flare at the top of the backrest",
        wing0_aabb is not None
        and wing1_aabb is not None
        and wing0_aabb[1][2] > 0.85
        and wing1_aabb[1][2] > 0.85
        and wing0_aabb[1][1] > 0.38
        and wing1_aabb[0][1] < -0.38,
        details=f"wing_0={wing0_aabb}, wing_1={wing1_aabb}",
    )

    # --- chair height ----------------------------------------------------
    rest_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "chair backrest rises to about 0.9-1.2 m",
        rest_aabb is not None and 0.85 <= rest_aabb[1][2] <= 1.25,
        details=f"backrest_aabb={rest_aabb}",
    )

    # --- rocking pose: chair tilts backward at negative rock limit -------
    with ctx.pose({rock: -ROCK_LIMIT}):
        rocked_aabb = ctx.part_world_aabb(backrest)
        seat_rocked_pos = ctx.part_world_position(seat)
    ctx.check(
        "rocking at -0.12 rad tilts the backrest top backward",
        rest_aabb is not None
        and rocked_aabb is not None
        and rocked_aabb[0][0] < rest_aabb[0][0] - 0.02,
        details=f"rest={rest_aabb}, rocked={rocked_aabb}",
    )

    with ctx.pose({rock: ROCK_LIMIT}):
        rocked_fwd_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "rocking at +0.12 rad tilts the backrest top forward",
        rest_aabb is not None
        and rocked_fwd_aabb is not None
        and rocked_fwd_aabb[1][0] > rest_aabb[1][0] + 0.02,
        details=f"rest={rest_aabb}, rocked_fwd={rocked_fwd_aabb}",
    )

    # --- swivel pose: shell rotates, ottoman stays put --------------------
    ottoman_pos0 = ctx.part_world_position(ottoman)
    with ctx.pose({swivel: math.pi / 2.0}):
        backrest_pos = ctx.part_world_position(backrest)
        ottoman_pos1 = ctx.part_world_position(ottoman)
    ctx.check(
        "swivel rotates the chair shell about the vertical column axis",
        backrest_pos is not None
        and abs(backrest_pos[0]) < 0.05
        and abs(backrest_pos[1] + abs(HINGE_X)) < 0.05,
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

    # --- recline pose ----------------------------------------------------
    with ctx.pose({recline: -0.3}):
        leaned_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "recline -0.3 rad leans the backrest top backward and down",
        rest_aabb is not None
        and leaned_aabb is not None
        and leaned_aabb[0][0] < rest_aabb[0][0] - 0.06
        and leaned_aabb[1][2] < rest_aabb[1][2] - 0.01,
        details=f"rest={rest_aabb}, leaned={leaned_aabb}",
    )

    # --- ottoman placement -----------------------------------------------
    ctx.expect_origin_gap(
        ottoman,
        rocker_base,
        axis="x",
        min_gap=0.55,
        max_gap=0.95,
        name="ottoman is offset in front of the chair",
    )

    ot_aabb = ctx.part_world_aabb(ottoman)
    ctx.check(
        "ottoman stands about 0.4 m tall on its own star base",
        ot_aabb is not None and -0.002 <= ot_aabb[0][2] <= 0.004 and 0.36 <= ot_aabb[1][2] <= 0.44,
        details=f"ottoman_aabb={ot_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
