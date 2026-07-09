from __future__ import annotations

"""Wall-mounted mini-split air conditioner indoor unit — single outlet door variant.

Glossy white rounded horizontal body ~0.90 m wide x 0.30 m tall x 0.22 m deep,
back flat against the wall plane (y=0), bottom at z=0. The lower front is a
quarter-round curve carrying one large outlet opening covered by a single
flush-fitting door panel. The door is hinged along the outlet's lower lip and
swings down and outward (revolute about the unit's width axis) to open the
airflow path. The large framed front face panel is a service cover hinged
along its top edge, swinging upward to reveal a shallow filter cavity with a
removable-look filter panel. The interior behind the outlet is a hollow dark
plenum.

Coordinate convention:
- X: unit width (left-right along the wall)
- Y: depth, +Y points away from the wall (front)
- Z: up, bottom of the unit at z = 0
"""

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

# ----------------------------------------------------------------------------
# Master dimensions (meters)
# ----------------------------------------------------------------------------
BODY_W = 0.90  # width along X (x in [-0.45, 0.45])
BODY_D = 0.22  # overall depth target along Y
BODY_H = 0.30  # height along Z (z in [0, 0.30])

# Side profile control points (y = depth from wall, z = height from bottom).
ARC_CY, ARC_CZ, ARC_R = 0.085, 0.13, 0.13  # bottom-front quarter-round
FRONT_LO = (0.215, 0.13)  # bottom of the leaning front face
FRONT_HI = (0.205, 0.285)  # top of the leaning front face
TOP_FRONT = (0.185, 0.30)  # front edge of the flat top (small bevel)

# Front service-panel zone (chassis is recessed behind the hinged panel).
PANEL_RECESS_Y = 0.198  # recessed chassis face behind the panel
PANEL_ZONE_Z0 = 0.142  # panel zone starts above the lower curve lip

# Hinged front panel.
PANEL_W = 0.85
PANEL_T = 0.013
PANEL_H = 0.1465
PANEL_HINGE_Y = 0.1995
PANEL_HINGE_Z = 0.290
PANEL_BORDER = 0.025
PANEL_OPEN_MAX = math.radians(60.0)

# Filter cavity + filter behind the front panel.
POCKET_W, POCKET_Y0, POCKET_Y1 = 0.74, 0.15, 0.21
POCKET_Z0, POCKET_Z1 = 0.185, 0.272

# Outlet door on the lower front curve.
DOOR_THETA_LOW = math.radians(15.0)   # hinge at lower lip
DOOR_THETA_HIGH = math.radians(82.0)  # top of door
DOOR_W = 0.78                          # door width along X
DOOR_T = 0.006                         # panel thickness
DOOR_OPEN_MAX = math.radians(75.0)    # max opening angle (swings down/out)

# Outlet opening cut in the housing (slightly smaller than door so door
# overlaps the frame edges when closed).
OPENING_THETA_LOW = math.radians(18.0)
OPENING_THETA_HIGH = math.radians(80.0)
OPENING_W = 0.74

# Hollow dark plenum behind the outlet opening (cross-flow blower bay).
PLENUM_R = 0.08
PLENUM_HALF_LEN = 0.41
LINER_R = 0.075
LINER_HALF_LEN = 0.42  # ends embed into the chassis side walls


def _arc_point(theta: float) -> tuple[float, float]:
    """(y, z) point on the bottom-front quarter-round at arc angle theta."""
    return (ARC_CY + ARC_R * math.sin(theta), ARC_CZ - ARC_R * math.cos(theta))


def _outlet_opening_cut() -> cq.Workplane:
    """Curved cutting tool for the outlet opening.

    Follows the housing arc from OPENING_THETA_LOW to OPENING_THETA_HIGH,
    cutting from slightly outside the outer surface to well past the inner
    wall thickness, with width OPENING_W centered on X.
    """
    R_outer = ARC_R + 0.003  # slightly proud of the housing surface
    R_inner = PLENUM_R - 0.002  # cut down to just inside the plenum liner
    n = 32
    outer_pts: list[tuple[float, float]] = []
    inner_pts: list[tuple[float, float]] = []
    for i in range(n + 1):
        t = OPENING_THETA_LOW + (OPENING_THETA_HIGH - OPENING_THETA_LOW) * i / n
        outer_pts.append((
            ARC_CY + R_outer * math.sin(t),
            ARC_CZ - R_outer * math.cos(t),
        ))
        inner_pts.append((
            ARC_CY + R_inner * math.sin(t),
            ARC_CZ - R_inner * math.cos(t),
        ))
    wp = cq.Workplane("YZ")
    wp = wp.moveTo(outer_pts[0][0], outer_pts[0][1])
    for pt in outer_pts[1:]:
        wp = wp.lineTo(pt[0], pt[1])
    wp = wp.lineTo(inner_pts[-1][0], inner_pts[-1][1])
    for pt in reversed(inner_pts[:-1]):
        wp = wp.lineTo(pt[0], pt[1])
    wp = wp.close()
    return wp.extrude(OPENING_W / 2.0, both=True)


def _housing_shape() -> cq.Workplane:
    mid = _arc_point(math.radians(45.0))
    body = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(ARC_CY, 0.0)
        .threePointArc(mid, FRONT_LO)
        .lineTo(*FRONT_HI)
        .lineTo(*TOP_FRONT)
        .lineTo(0.0, BODY_H)
        .close()
        .extrude(BODY_W / 2.0, both=True)
    )

    # Recess the front wall behind the hinged service panel.
    body = body.cut(
        cq.Workplane("XY")
        .box(BODY_W + 0.02, 0.102, 0.208)
        .translate((0.0, PANEL_RECESS_Y + 0.051, PANEL_ZONE_Z0 + 0.104))
    )
    # Shallow framed inset on the top face (air-intake panel look).
    body = body.cut(
        cq.Workplane("XY").box(0.80, 0.141, 0.013).translate((0.0, 0.0925, 0.3035))
    )
    # Filter cavity pocket behind the front panel.
    body = body.cut(
        cq.Workplane("XY")
        .box(POCKET_W, POCKET_Y1 - POCKET_Y0, POCKET_Z1 - POCKET_Z0)
        .translate(
            (0.0, (POCKET_Y0 + POCKET_Y1) / 2.0, (POCKET_Z0 + POCKET_Z1) / 2.0)
        )
    )
    # Hollow blower plenum behind the outlet opening, capped by a back wall.
    plenum = (
        cq.Workplane("YZ")
        .center(ARC_CY, ARC_CZ)
        .circle(PLENUM_R)
        .extrude(PLENUM_HALF_LEN, both=True)
        .intersect(
            cq.Workplane("XY").box(0.82, 0.19, 0.30).translate((0.0, 0.11, 0.15))
        )
    )
    body = body.cut(plenum)
    # Large outlet opening through the curved lower-front wall: a curved
    # cutting tool that follows the housing arc, clearing the wall from
    # slightly outside the outer surface down past the plenum liner.
    body = body.cut(_outlet_opening_cut())
    return body


def _front_panel_shape() -> cq.Workplane:
    """Hinged front face panel in its joint-local frame.

    Local frame sits on the hinge line (top edge): the plate hangs along -Z
    and its thickness extends along +Y (away from the chassis).
    """
    plate = (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_T, PANEL_H)
        .translate((0.0, PANEL_T / 2.0, -PANEL_H / 2.0))
    )
    # Shallow face recess leaving a thin raised border frame.
    recess = (
        cq.Workplane("XY")
        .box(PANEL_W - 2.0 * PANEL_BORDER, 0.006, PANEL_H - 2.0 * PANEL_BORDER)
        .translate((0.0, PANEL_T, -PANEL_H / 2.0))
    )
    return plate.cut(recess)


def _outlet_door_shape() -> cq.Workplane:
    """Curved outlet door panel in hinge-local frame.

    The hinge-local frame has its origin at the hinge point (lower lip of the
    outlet). The door extends along the housing arc from DOOR_THETA_LOW to
    DOOR_THETA_HIGH, with thickness DOOR_T inward from the outer surface.
    """
    y_h, z_h = _arc_point(DOOR_THETA_LOW)
    # Arc center relative to hinge origin.
    cy_loc = ARC_CY - y_h
    cz_loc = ARC_CZ - z_h
    R_out = ARC_R
    R_in = ARC_R - DOOR_T

    # Discretize the arc cross-section.
    n = 32
    outer_pts: list[tuple[float, float]] = []
    inner_pts: list[tuple[float, float]] = []
    for i in range(n + 1):
        t = DOOR_THETA_LOW + (DOOR_THETA_HIGH - DOOR_THETA_LOW) * i / n
        outer_pts.append((
            cy_loc + R_out * math.sin(t),
            cz_loc - R_out * math.cos(t),
        ))
        inner_pts.append((
            cy_loc + R_in * math.sin(t),
            cz_loc - R_in * math.cos(t),
        ))

    # Build closed cross-section wire on YZ plane: outer arc forward,
    # end cap to inner arc, inner arc backward, close.
    wp = cq.Workplane("YZ")
    wp = wp.moveTo(outer_pts[0][0], outer_pts[0][1])
    for pt in outer_pts[1:]:
        wp = wp.lineTo(pt[0], pt[1])
    wp = wp.lineTo(inner_pts[-1][0], inner_pts[-1][1])
    for pt in reversed(inner_pts[:-1]):
        wp = wp.lineTo(pt[0], pt[1])
    wp = wp.close()

    door = wp.extrude(DOOR_W / 2.0, both=True)

    # Small raised lip at the top free edge (grab point for opening).
    lip_y, lip_z = (
        cy_loc + R_out * math.sin(DOOR_THETA_HIGH),
        cz_loc - R_out * math.cos(DOOR_THETA_HIGH),
    )
    lip = (
        cq.Workplane("XY")
        .box(DOOR_W * 0.6, 0.010, 0.008)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(DOOR_THETA_HIGH) - 90.0)
        .translate((0.0, lip_y, lip_z))
    )
    door = door.union(lip)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_split_indoor_unit")

    model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    model.material("panel_white", rgba=(0.96, 0.965, 0.97, 1.0))
    model.material("door_white", rgba=(0.91, 0.92, 0.935, 1.0))
    model.material("cavity_dark", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("filter_gray", rgba=(0.80, 0.81, 0.82, 1.0))
    model.material("mesh_gray", rgba=(0.30, 0.32, 0.34, 1.0))

    # ------------------------------------------------------------------ body
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing_shell"),
        material="shell_white",
        name="housing_shell",
    )
    # Dark blower-bay liner: visible through the outlet opening, ends embedded
    # in the chassis side walls so it reads as the hollow dark interior.
    housing.visual(
        Cylinder(radius=LINER_R, length=2.0 * LINER_HALF_LEN),
        origin=Origin(xyz=(0.0, ARC_CY, ARC_CZ), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="cavity_dark",
        name="plenum_liner",
    )
    # Removable-look filter panel seated against the back of the filter
    # cavity (frame slightly embedded into the cavity back wall).
    housing.visual(
        Box((0.70, 0.005, 0.082)),
        origin=Origin(xyz=(0.0, 0.152, 0.2285)),
        material="filter_gray",
        name="filter_frame",
    )
    housing.visual(
        Box((0.66, 0.004, 0.072)),
        origin=Origin(xyz=(0.0, 0.1555, 0.2285)),
        material="mesh_gray",
        name="filter_mesh",
    )

    # ---------------------------------------------------------- front panel
    front_panel = model.part("front_panel")
    front_panel.visual(
        mesh_from_cadquery(_front_panel_shape(), "front_panel"),
        material="panel_white",
        name="panel_plate",
    )
    # Hinge knuckles along the top back edge: they seat into the recessed
    # chassis wall and carry the cover (intentional local embed).
    for idx, kx in enumerate((-0.30, 0.30)):
        front_panel.visual(
            Box((0.04, 0.006, 0.012)),
            origin=Origin(xyz=(kx, -0.00125, -0.004)),
            material="panel_white",
            name=f"hinge_knuckle_{idx}",
        )
    model.articulation(
        "front_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_panel,
        origin=Origin(xyz=(0.0, PANEL_HINGE_Y, PANEL_HINGE_Z)),
        # Hinge along the panel's top edge; the closed plate hangs along
        # local -Z, so positive rotation about +X swings the free bottom
        # edge outward (+Y) and up: the service cover lifts open.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=1.5, lower=0.0, upper=PANEL_OPEN_MAX
        ),
    )

    # --------------------------------------------------------- outlet door
    outlet_door = model.part("outlet_door")
    outlet_door.visual(
        mesh_from_cadquery(_outlet_door_shape(), "outlet_door_panel"),
        material="door_white",
        name="door_panel",
    )
    # End pivot pins at the hinge axis, seated into the housing side walls.
    for idx, px in enumerate((-0.405, 0.405)):
        outlet_door.visual(
            Cylinder(radius=0.004, length=0.04),
            origin=Origin(xyz=(px, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="door_white",
            name=f"door_pivot_pin_{idx}",
        )

    hinge_y, hinge_z = _arc_point(DOOR_THETA_LOW)
    model.articulation(
        "outlet_door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=outlet_door,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        # Hinge at the lower lip of the outlet. The door extends upward
        # along the curved surface from the hinge. Axis (-1, 0, 0) means
        # positive q swings the free top edge outward (+Y) and downward
        # (-Z): the door opens down and out to expose the airflow path.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN_MAX
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    panel = object_model.get_part("front_panel")
    hinge = object_model.get_articulation("front_panel_hinge")
    door = object_model.get_part("outlet_door")
    door_joint = object_model.get_articulation("outlet_door_hinge")

    # ---- real-world scale and grounding -----------------------------------
    hb = ctx.part_world_aabb(housing)
    assert hb is not None
    ctx.check(
        "housing width ~0.90 m",
        abs((hb[1][0] - hb[0][0]) - BODY_W) < 0.02,
        details=f"width={hb[1][0] - hb[0][0]:.4f}",
    )
    ctx.check(
        "housing height ~0.30 m",
        abs((hb[1][2] - hb[0][2]) - BODY_H) < 0.02,
        details=f"height={hb[1][2] - hb[0][2]:.4f}",
    )
    ctx.check(
        "housing depth ~0.22 m",
        0.20 <= (hb[1][1] - hb[0][1]) <= 0.23,
        details=f"depth={hb[1][1] - hb[0][1]:.4f}",
    )
    ctx.check(
        "unit bottom sits at z=0 with flat back at the wall plane y=0",
        abs(hb[0][2]) < 0.005 and abs(hb[0][1]) < 0.005,
        details=f"min={hb[0]}",
    )

    # ---- intentional hinge/pivot embeds ------------------------------------
    for idx in range(2):
        ctx.allow_overlap(
            panel,
            housing,
            elem_a=f"hinge_knuckle_{idx}",
            elem_b="housing_shell",
            reason="Hinge knuckle is intentionally seated into the recessed chassis wall to carry the hinged service cover.",
        )
        ctx.expect_contact(
            panel,
            housing,
            elem_a=f"hinge_knuckle_{idx}",
            elem_b="housing_shell",
            name=f"hinge knuckle {idx} is seated on the chassis",
        )
    for idx in range(2):
        ctx.allow_overlap(
            door,
            housing,
            elem_a=f"door_pivot_pin_{idx}",
            elem_b="housing_shell",
            reason="Door end pivot pin is intentionally captured in the outlet frame side wall.",
        )
        ctx.expect_contact(
            door,
            housing,
            elem_a=f"door_pivot_pin_{idx}",
            elem_b="housing_shell",
            name=f"door pivot pin {idx} is captured in the frame side",
        )
    # Door panel seats flush against the housing frame around the outlet
    # opening: the outer surface of the curved door is at the housing outer
    # radius, so the door edge frame area has intentional local embed.
    ctx.allow_overlap(
        door,
        housing,
        elem_a="door_panel",
        elem_b="housing_shell",
        reason="Door panel outer surface is intentionally flush with the housing curve; the frame-edge contact represents the seated cover fit.",
    )
    ctx.expect_contact(
        door,
        housing,
        elem_a="door_panel",
        elem_b="housing_shell",
        name="door panel is seated on the housing frame",
    )

    # ---- front service panel: closed seating and hinge --------------------
    ctx.expect_within(panel, housing, axes="x", margin=0.002, name="panel within body width")
    ctx.expect_overlap(
        panel, housing, axes="xz", min_overlap=0.05, name="closed panel covers the front face"
    )
    limits = hinge.motion_limits
    ctx.check(
        "panel hinge opens upward 0..~60 deg about the width axis",
        limits is not None
        and abs(limits.lower) < 1e-6
        and abs(limits.upper - PANEL_OPEN_MAX) < 0.02
        and tuple(hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={hinge.axis}, limits=({limits.lower}, {limits.upper})",
    )
    pb_closed = ctx.part_world_aabb(panel)
    plate_closed = ctx.part_element_world_aabb(panel, elem="panel_plate")
    assert pb_closed is not None and plate_closed is not None
    ctx.check(
        "closed panel plate sits proud of the recessed chassis face",
        plate_closed[0][1] > PANEL_RECESS_Y + 0.0005,
        details=f"plate min y={plate_closed[0][1]:.4f}",
    )
    with ctx.pose({hinge: 1.0}):
        pb_open = ctx.part_world_aabb(panel)
        assert pb_open is not None
        ctx.check(
            "open panel swings outward and its bottom edge lifts",
            pb_open[1][1] > hb[1][1] + 0.08 and pb_open[0][2] > pb_closed[0][2] + 0.05,
            details=f"open aabb={pb_open}",
        )
        # Filter cavity contents are revealed behind the lifted cover.
        filt = ctx.part_element_world_aabb(housing, elem="filter_mesh")
        assert filt is not None
        ctx.check(
            "filter panel sits inside the cavity behind the cover",
            POCKET_Y0 - 0.001 <= filt[0][1]
            and filt[1][1] <= POCKET_Y1
            and POCKET_Z0 <= filt[0][2]
            and filt[1][2] <= POCKET_Z1,
            details=f"filter aabb={filt}",
        )

    # ---- outlet door: closed flush, opens down and outward ----------------
    # Door sits on the lower-front curve area.
    door_closed = ctx.part_world_aabb(door)
    assert door_closed is not None
    ctx.check(
        "door spans the lower-front outlet width",
        door_closed[1][0] - door_closed[0][0] > 0.70,
        details=f"door x-extent={door_closed[1][0] - door_closed[0][0]:.4f}",
    )
    ctx.expect_within(door, housing, axes="x", margin=0.002, name="door within housing width")

    # Joint configuration: revolute about width X axis, opens at positive q.
    dlimits = door_joint.motion_limits
    ctx.check(
        "door hinge is revolute about the width axis with 0..~75 deg range",
        dlimits is not None
        and abs(dlimits.lower) < 1e-6
        and abs(dlimits.upper - DOOR_OPEN_MAX) < 0.05
        and tuple(door_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={door_joint.axis}, limits=({dlimits.lower}, {dlimits.upper})",
    )

    # Closed door is near the housing surface (flush fit on the lower curve).
    hinge_y, hinge_z = _arc_point(DOOR_THETA_LOW)
    ctx.check(
        "door hinge origin is at the lower lip of the outlet",
        abs(door_joint.origin.xyz[1] - hinge_y) < 0.005
        and abs(door_joint.origin.xyz[2] - hinge_z) < 0.005,
        details=f"origin={door_joint.origin.xyz}, expected~(0, {hinge_y:.4f}, {hinge_z:.4f})",
    )

    # Decisive motion check: opening the door moves its free edge outward
    # (+Y) and downward (-Z), exposing the plenum behind.
    door_pos_closed = ctx.part_world_position(door)
    with ctx.pose({door_joint: DOOR_OPEN_MAX}):
        door_pos_open = ctx.part_world_position(door)
        door_open_aabb = ctx.part_world_aabb(door)
        assert door_open_aabb is not None
        ctx.check(
            "open door swings outward: max-y increases significantly",
            door_open_aabb[1][1] > door_closed[1][1] + 0.02,
            details=f"closed max-y={door_closed[1][1]:.4f}, open max-y={door_open_aabb[1][1]:.4f}",
        )
        ctx.check(
            "open door drops below the closed position: min-z decreases",
            door_open_aabb[0][2] < door_closed[0][2] - 0.02,
            details=f"closed min-z={door_closed[0][2]:.4f}, open min-z={door_open_aabb[0][2]:.4f}",
        )

    # Dark plenum is visible behind the outlet area.
    liner = ctx.part_element_world_aabb(housing, elem="plenum_liner")
    assert liner is not None
    ctx.check(
        "dark plenum liner spans behind the outlet opening",
        liner[0][0] < -0.40
        and liner[1][0] > 0.40
        and liner[0][2] < 0.06
        and liner[1][2] > 0.19,
        details=f"liner aabb={liner}",
    )

    return ctx.report()


object_model = build_object_model()
