from __future__ import annotations

"""Wall-mounted mini-split air conditioner indoor unit (single-wide-deflector variant).

Glossy white rounded horizontal body ~0.90 m wide x 0.30 m tall x 0.22 m deep,
back flat against the wall plane (y=0), bottom at z=0. The lower front is a
quarter-round curve carrying one wide airflow outlet with a single full-width
deflector blade hinged along its top edge, swinging down and out to direct
airflow (revolute about the unit's width axis). The large framed front face
panel is a service cover hinged along its top edge, swinging upward to reveal
a shallow filter cavity with a removable-look filter panel. The interior
behind the outlet is a hollow dark plenum.

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

# Single wide outlet on the lower front curve + full-width deflector blade.
# The outlet is one rectangular opening centered at OUTLET_THETA_MID on the
# arc; the deflector is hinged at HINGE_THETA near the top of the outlet.
OUTLET_THETA_MID = math.radians(55.0)
OUTLET_W = 0.80
OUTLET_DEPTH = 0.14  # cut depth along the surface normal (into housing)
OUTLET_H = 0.10  # opening height measured along the surface tangent

DEFLECTOR_W = 0.78
DEFLECTOR_H = 0.09
DEFLECTOR_T = 0.005
HINGE_THETA = math.radians(80.0)
DEFLECTOR_SWING_LO = math.radians(-10.0)
DEFLECTOR_SWING_HI = math.radians(60.0)

# Hollow dark plenum behind the outlet (cross-flow blower bay).
PLENUM_R = 0.08
PLENUM_HALF_LEN = 0.41
LINER_R = 0.075
LINER_HALF_LEN = 0.42  # ends embed into the chassis side walls


def _arc_point(theta: float) -> tuple[float, float]:
    """(y, z) point on the bottom-front quarter-round at arc angle theta."""
    return (ARC_CY + ARC_R * math.sin(theta), ARC_CZ - ARC_R * math.cos(theta))


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
    # Hollow blower plenum behind the outlet, capped by a back wall.
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

    # Single wide outlet opening on the lower front curve.
    # The cut box is oriented so its Y axis aligns with the outward surface
    # normal and its Z axis aligns with the surface tangent at theta_mid.
    ay_mid, az_mid = _arc_point(OUTLET_THETA_MID)
    outlet = (
        cq.Workplane("XY")
        .box(OUTLET_W, OUTLET_DEPTH, OUTLET_H)
        .rotate(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
            math.degrees(OUTLET_THETA_MID) - 90.0,
        )
        .translate((0.0, ay_mid, az_mid))
    )
    body = body.cut(outlet)
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


def _deflector_shape() -> cq.Workplane:
    """Full-width deflector blade in its joint-local frame.

    Local frame sits on the hinge line (top edge): the blade hangs along -Z
    and its thickness extends along +Y (away from the chassis). A small
    return lip at the free (bottom) edge aids airflow direction and grip.
    """
    blade = (
        cq.Workplane("XY")
        .box(DEFLECTOR_W, DEFLECTOR_T, DEFLECTOR_H)
        .translate((0.0, DEFLECTOR_T / 2.0, -DEFLECTOR_H / 2.0))
    )
    # Small return lip at the free edge, angled outward from the blade face.
    lip = (
        cq.Workplane("XY")
        .box(DEFLECTOR_W - 0.04, 0.007, 0.005)
        .translate((0.0, DEFLECTOR_T + 0.0035, -DEFLECTOR_H + 0.0025))
    )
    return blade.union(lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_split_indoor_unit")

    model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    model.material("panel_white", rgba=(0.96, 0.965, 0.97, 1.0))
    model.material("deflector_white", rgba=(0.88, 0.895, 0.91, 1.0))
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
    # Dark blower-bay liner: visible through the outlet, ends embedded
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

    # --------------------------------------------------------- deflector
    deflector = model.part("deflector")
    deflector.visual(
        mesh_from_cadquery(_deflector_shape(), "deflector_blade"),
        material="deflector_white",
        name="deflector_blade",
    )
    # End pivot pins on the blade axis, seated into the outlet end walls.
    for idx, px in enumerate((-0.395, 0.395)):
        deflector.visual(
            Cylinder(radius=0.004, length=0.04),
            origin=Origin(xyz=(px, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="deflector_white",
            name=f"pivot_pin_{idx}",
        )

    ay_h, az_h = _arc_point(HINGE_THETA)
    model.articulation(
        "deflector_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=deflector,
        # Joint frame rotated about X so local +Y matches the outward
        # normal of the curved front at the hinge position. The blade
        # hangs along local -Z; positive rotation about +X swings the
        # free (bottom) edge outward (+Y), directing airflow down and out.
        origin=Origin(
            xyz=(0.0, ay_h, az_h),
            rpy=(HINGE_THETA - math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=DEFLECTOR_SWING_LO,
            upper=DEFLECTOR_SWING_HI,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    panel = object_model.get_part("front_panel")
    hinge = object_model.get_articulation("front_panel_hinge")
    deflector = object_model.get_part("deflector")
    deflector_joint = object_model.get_articulation("deflector_pivot")

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
            deflector,
            housing,
            elem_a=f"pivot_pin_{idx}",
            elem_b="housing_shell",
            reason="Deflector end pivot pin is intentionally captured in the outlet end wall.",
        )
        ctx.expect_contact(
            deflector,
            housing,
            elem_a=f"pivot_pin_{idx}",
            elem_b="housing_shell",
            name=f"deflector pivot pin {idx} is captured in the outlet end",
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

    # ---- deflector: single full-width blade, placement and motion ---------
    db_closed = ctx.part_world_aabb(deflector)
    assert db_closed is not None

    # Deflector spans the full outlet width (at least 0.74 m).
    closed_width = db_closed[1][0] - db_closed[0][0]
    ctx.check(
        "deflector spans the full outlet width",
        closed_width > 0.74,
        details=f"width={closed_width:.4f}",
    )

    # Deflector is centered within the housing width.
    ctx.expect_within(
        deflector, housing, axes="x", margin=0.005, name="deflector within housing width"
    )

    # Deflector hinge is on the lower front curve at the expected position.
    ay_h, az_h = _arc_point(HINGE_THETA)
    ctx.check(
        "deflector hinge sits on the lower front curve near the outlet top",
        0.08 < az_h < 0.15 and 0.19 < ay_h < 0.23,
        details=f"hinge=({ay_h:.4f}, {az_h:.4f})",
    )

    # Joint configuration: revolute about X with the expected limits.
    dlimits = deflector_joint.motion_limits
    ctx.check(
        "deflector pivots about the width axis with correct swing limits",
        dlimits is not None
        and abs(dlimits.lower - DEFLECTOR_SWING_LO) < 0.02
        and abs(dlimits.upper - DEFLECTOR_SWING_HI) < 0.02
        and tuple(deflector_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={deflector_joint.axis}, limits=({dlimits.lower}, {dlimits.upper})",
    )

    # Decisive motion check: opening the deflector swings the blade outward
    # (+Y) from its rest position hanging along the curved surface. The blade
    # rotates from nearly-vertical (covering the outlet) to angled outward,
    # creating a redirecting surface that directs airflow down and out.
    with ctx.pose({deflector_joint: DEFLECTOR_SWING_HI}):
        db_open = ctx.part_world_aabb(deflector)
    assert db_open is not None
    ctx.check(
        "deflector swings outward when opened (larger Y extent)",
        db_open[1][1] > db_closed[1][1] + 0.02,
        details=f"closed max y={db_closed[1][1]:.4f}, open max y={db_open[1][1]:.4f}",
    )
    # Z-extent shrinks as the blade tilts from hanging down to pointing out,
    # proving real rotation about the width axis.
    z_extent_closed = db_closed[1][2] - db_closed[0][2]
    z_extent_open = db_open[1][2] - db_open[0][2]
    ctx.check(
        "deflector blade tilts when opened (z-extent shrinks)",
        z_extent_closed > z_extent_open + 0.01,
        details=f"z-extent closed={z_extent_closed:.4f}, open={z_extent_open:.4f}",
    )
    # At full open, the blade extends significantly beyond the housing front.
    ctx.check(
        "open deflector extends beyond the housing front face",
        db_open[1][1] > hb[1][1] + 0.01,
        details=f"open max y={db_open[1][1]:.4f}, housing front y={hb[1][1]:.4f}",
    )

    # Verify there is exactly one airflow joint (the single deflector),
    # not the three independent vanes from the parent model.
    all_joint_names = [a.name for a in object_model.articulations]
    ctx.check(
        "no independent louver vane joints remain from the parent",
        not any("louver" in n for n in all_joint_names),
        details=f"joints={all_joint_names}",
    )
    ctx.check(
        "deflector_pivot is the single airflow articulation",
        "deflector_pivot" in all_joint_names,
        details=f"joints={all_joint_names}",
    )

    # ---- hollow dark interior behind the outlet -----------------------------
    liner = ctx.part_element_world_aabb(housing, elem="plenum_liner")
    assert liner is not None
    ctx.check(
        "dark plenum liner spans behind the outlet",
        liner[0][0] < -0.40
        and liner[1][0] > 0.40
        and liner[0][2] < 0.06
        and liner[1][2] > 0.19,
        details=f"liner aabb={liner}",
    )

    return ctx.report()


object_model = build_object_model()
