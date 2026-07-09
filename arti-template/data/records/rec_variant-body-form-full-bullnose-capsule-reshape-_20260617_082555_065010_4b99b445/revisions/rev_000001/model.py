from __future__ import annotations

"""Wall-mounted mini-split air conditioner indoor unit — full-bullnose variant.

Glossy white rounded horizontal body ~0.90 m wide x 0.30 m tall x 0.22 m deep,
back flat against the wall plane (y=0), bottom at z=0. The side profile is a
smooth rounded-end horizontal capsule: BOTH the top-front and bottom-front
edges are rounded over with the same bullnose radius, giving a capsule-like
silhouette. The flat front face between the two curves carries three long
slim louver slots on the lower curve, each with its own independently pivoting
airflow vane (revolute about the unit's width axis). The large framed front
face panel is a service cover hinged along its top edge, swinging upward to
reveal a shallow filter cavity with a removable-look filter panel. The
interior behind the louver slots is a hollow dark plenum.

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

# Full bullnose: same rounding radius on both top-front and bottom-front.
BULLNOSE_R = 0.09

# Bottom bullnose arc center and derived geometry.
ARC_BOT_CY = BODY_D - BULLNOSE_R  # 0.13
ARC_BOT_CZ = BULLNOSE_R  # 0.09

# Top bullnose arc center and derived geometry.
ARC_TOP_CY = BODY_D - BULLNOSE_R  # 0.13
ARC_TOP_CZ = BODY_H - BULLNOSE_R  # 0.21

# Front service-panel zone (chassis is recessed behind the hinged panel).
PANEL_RECESS_Y = 0.195  # recessed chassis face behind the panel
PANEL_ZONE_Z0 = 0.095  # panel zone starts just above the lower curve

# Hinged front panel.
PANEL_W = 0.85
PANEL_T = 0.013
PANEL_H = 0.115
PANEL_HINGE_Y = 0.197
PANEL_HINGE_Z = 0.210
PANEL_BORDER = 0.020
PANEL_OPEN_MAX = math.radians(60.0)

# Filter cavity + filter behind the front panel.
POCKET_W, POCKET_Y0, POCKET_Y1 = 0.74, 0.148, 0.195
POCKET_Z0, POCKET_Z1 = 0.115, 0.195

# Louver slots / vanes on the lower bullnose curve, parameterized by the arc
# angle theta measured from straight-down (0 deg = bottom, 90 deg = front).
VANE_SPECS = (
    ("lower", math.radians(30.0)),
    ("middle", math.radians(52.0)),
    ("upper", math.radians(74.0)),
)
SLOT_LEN = 0.78  # slot cut length along X
SLOT_OPEN = 0.018  # slot opening measured along the surface tangent
SLOT_DEPTH = 0.10  # cut depth along the surface normal (pierces into plenum)
VANE_LEN = 0.74
VANE_CHORD = 0.015
VANE_T = 0.004
VANE_SWING = math.radians(45.0)

# Hollow dark plenum behind the louver slots (cross-flow blower bay).
PLENUM_R = 0.06
PLENUM_HALF_LEN = 0.41
LINER_R = 0.055
LINER_HALF_LEN = 0.42  # ends embed into the chassis side walls


def _arc_point(theta: float) -> tuple[float, float]:
    """(y, z) point on the bottom-front bullnose arc at arc angle theta."""
    return (ARC_BOT_CY + BULLNOSE_R * math.sin(theta),
            ARC_BOT_CZ - BULLNOSE_R * math.cos(theta))


def _top_arc_point(theta: float) -> tuple[float, float]:
    """(y, z) point on the top-front bullnose arc at arc angle theta.

    theta=0 is horizontal (front-most point at y=BODY_D), theta=pi/2 is
    vertical (top-most point at z=BODY_H).
    """
    return (ARC_TOP_CY + BULLNOSE_R * math.cos(theta),
            ARC_TOP_CZ + BULLNOSE_R * math.sin(theta))


def _housing_shape() -> cq.Workplane:
    """Full-bullnose capsule housing: rounded at both top-front and bottom-front."""
    sq2_2 = math.sin(math.radians(45.0))  # sin(45) = cos(45) ≈ 0.7071

    # Midpoints for threePointArc calls.
    bot_arc_mid = (ARC_BOT_CY + BULLNOSE_R * sq2_2,
                   ARC_BOT_CZ - BULLNOSE_R * sq2_2)
    top_arc_mid = (ARC_TOP_CY + BULLNOSE_R * sq2_2,
                   ARC_TOP_CZ + BULLNOSE_R * sq2_2)

    body = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(ARC_BOT_CY, 0.0)
        .threePointArc(bot_arc_mid, (BODY_D, BULLNOSE_R))
        .lineTo(BODY_D, BODY_H - BULLNOSE_R)
        .threePointArc(top_arc_mid, (ARC_TOP_CY, BODY_H))
        .lineTo(0.0, BODY_H)
        .close()
        .extrude(BODY_W / 2.0, both=True)
    )

    # Recess the front wall behind the hinged service panel.
    recess_dy = BODY_D - PANEL_RECESS_Y + 0.01
    recess_dz = PANEL_HINGE_Z - PANEL_ZONE_Z0 + 0.005
    body = body.cut(
        cq.Workplane("XY")
        .box(BODY_W + 0.02, recess_dy, recess_dz)
        .translate((0.0,
                    PANEL_RECESS_Y + recess_dy / 2.0,
                    PANEL_ZONE_Z0 + recess_dz / 2.0 - 0.002))
    )
    # Shallow framed inset on the top face (air-intake panel look).
    body = body.cut(
        cq.Workplane("XY")
        .box(0.78, 0.09, 0.013)
        .translate((0.0, 0.058, 0.3035))
    )
    # Filter cavity pocket behind the front panel.
    body = body.cut(
        cq.Workplane("XY")
        .box(POCKET_W, POCKET_Y1 - POCKET_Y0, POCKET_Z1 - POCKET_Z0)
        .translate(
            (0.0, (POCKET_Y0 + POCKET_Y1) / 2.0, (POCKET_Z0 + POCKET_Z1) / 2.0)
        )
    )
    # Hollow blower plenum behind the louver slots, capped by a back wall.
    plenum = (
        cq.Workplane("YZ")
        .center(ARC_BOT_CY, ARC_BOT_CZ)
        .circle(PLENUM_R)
        .extrude(PLENUM_HALF_LEN, both=True)
        .intersect(
            cq.Workplane("XY").box(0.82, 0.18, 0.20).translate((0.0, 0.11, 0.09))
        )
    )
    body = body.cut(plenum)
    # Three long louver slots through the bottom bullnose wall.
    for _, theta in VANE_SPECS:
        ay, az = _arc_point(theta)
        slot = (
            cq.Workplane("XY")
            .box(SLOT_LEN, SLOT_DEPTH, SLOT_OPEN)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(theta) - 90.0)
            .translate((0.0, ay, az))
        )
        body = body.cut(slot)
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_split_indoor_unit")

    model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    model.material("panel_white", rgba=(0.96, 0.965, 0.97, 1.0))
    model.material("vane_white", rgba=(0.88, 0.895, 0.91, 1.0))
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
    # Dark blower-bay liner: visible through the louver slots, ends embedded
    # in the chassis side walls so it reads as the hollow dark interior.
    housing.visual(
        Cylinder(radius=LINER_R, length=2.0 * LINER_HALF_LEN),
        origin=Origin(xyz=(0.0, ARC_BOT_CY, ARC_BOT_CZ), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="cavity_dark",
        name="plenum_liner",
    )
    # Removable-look filter panel seated against the back of the filter
    # cavity (frame slightly embedded into the cavity back wall).
    housing.visual(
        Box((0.70, 0.005, 0.070)),
        origin=Origin(xyz=(0.0, 0.150, 0.155)),
        material="filter_gray",
        name="filter_frame",
    )
    housing.visual(
        Box((0.66, 0.004, 0.060)),
        origin=Origin(xyz=(0.0, 0.153, 0.155)),
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

    # --------------------------------------------------------- louver vanes
    for i, (label, theta) in enumerate(VANE_SPECS):
        vane = model.part(f"louver_vane_{i}")
        # Blade lies in the slot's tangent plane at q=0: local +Y is the
        # outward surface normal, local +Z the surface tangent (chord).
        vane.visual(
            Box((VANE_LEN, VANE_T, VANE_CHORD)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="vane_white",
            name=f"vane_blade_{i}",
        )
        # End pivot pins on the blade axis, seated into the slot end walls.
        for j, px in enumerate((-0.385, 0.385)):
            vane.visual(
                Cylinder(radius=0.003, length=0.04),
                origin=Origin(xyz=(px, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material="vane_white",
                name=f"pivot_pin_{i}_{j}",
            )
        ay, az = _arc_point(theta)
        model.articulation(
            f"louver_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=vane,
            # Joint frame rotated about X so local +Y matches the outward
            # normal of the curved front at this slot.
            origin=Origin(
                xyz=(0.0, ay, az), rpy=(theta - math.pi / 2.0, 0.0, 0.0)
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.0, lower=-VANE_SWING, upper=VANE_SWING
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    panel = object_model.get_part("front_panel")
    hinge = object_model.get_articulation("front_panel_hinge")
    vanes = [object_model.get_part(f"louver_vane_{i}") for i in range(len(VANE_SPECS))]
    pivots = [
        object_model.get_articulation(f"louver_pivot_{i}") for i in range(len(VANE_SPECS))
    ]

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

    # ---- full bullnose profile: both front corners are rounded -------------
    # The bottom curve places vanes below z = BULLNOSE_R, confirming the
    # bottom-front bullnose. The top curve sets the hinge below the flat top
    # edge, confirming the top-front bullnose.
    ay_lower, az_lower = _arc_point(VANE_SPECS[0][1])
    ctx.check(
        "lower vane on bottom bullnose curve near the unit base",
        az_lower < BODY_H * 0.25 and ay_lower > BODY_D * 0.7,
        details=f"lower slot at y={ay_lower:.4f}, z={az_lower:.4f}",
    )
    ctx.check(
        "panel hinge set back from front face confirming top bullnose rounding",
        PANEL_HINGE_Y < BODY_D - 0.005 and PANEL_HINGE_Z < BODY_H - BULLNOSE_R + 0.005,
        details=f"hinge y={PANEL_HINGE_Y:.4f}, z={PANEL_HINGE_Z:.4f}",
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
    for i, vane in enumerate(vanes):
        for j in range(2):
            ctx.allow_overlap(
                vane,
                housing,
                elem_a=f"pivot_pin_{i}_{j}",
                elem_b="housing_shell",
                reason="Vane end pivot pin is intentionally captured in the louver slot end wall.",
            )
            ctx.expect_contact(
                vane,
                housing,
                elem_a=f"pivot_pin_{i}_{j}",
                elem_b="housing_shell",
                name=f"vane {i} pin {j} is captured in the slot end",
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
            pb_open[1][1] > hb[1][1] + 0.05 and pb_open[0][2] > pb_closed[0][2] + 0.03,
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

    # ---- louver vanes: placement, independence, motion ---------------------
    prev_z = -1.0
    prev_y = -1.0
    for i, ((label, theta), vane, pivot) in enumerate(zip(VANE_SPECS, vanes, pivots)):
        vb = ctx.part_world_aabb(vane)
        assert vb is not None
        cz = 0.5 * (vb[0][2] + vb[1][2])
        cy = 0.5 * (vb[0][1] + vb[1][1])
        ay, az = _arc_point(theta)
        ctx.check(
            f"{label} vane is centered in its slot on the bottom bullnose curve",
            abs(cz - az) < 0.004 and abs(cy - ay) < 0.004,
            details=f"center=({cy:.4f},{cz:.4f}), slot=({ay:.4f},{az:.4f})",
        )
        ctx.check(
            f"{label} vane stacks above the previous vane and further forward",
            cz > prev_z and cy > prev_y,
            details=f"cy={cy:.4f}, cz={cz:.4f}",
        )
        prev_z, prev_y = cz, cy
        ctx.expect_within(vane, housing, axes="x", margin=0.001, name=f"{label} vane within slot length")
        vlimits = pivot.motion_limits
        ctx.check(
            f"{label} vane pivots +/-45 deg about the width axis",
            vlimits is not None
            and abs(vlimits.lower + VANE_SWING) < 0.02
            and abs(vlimits.upper - VANE_SWING) < 0.02
            and tuple(pivot.axis) == (1.0, 0.0, 0.0),
            details=f"axis={pivot.axis}, limits=({vlimits.lower}, {vlimits.upper})",
        )
        # Decisive motion check: tilting the blade toward the airflow-down
        # extreme makes its world z-extent grow versus the opposite extreme,
        # proving rotation happens about the X (width) axis at the slot.
        with ctx.pose({pivot: VANE_SWING}):
            up = ctx.part_world_aabb(vane)
        with ctx.pose({pivot: -VANE_SWING}):
            dn = ctx.part_world_aabb(vane)
        assert up is not None and dn is not None
        ext_up = up[1][2] - up[0][2]
        ext_dn = dn[1][2] - dn[0][2]
        ctx.check(
            f"{label} vane blade actually tilts in its slot",
            ext_up > ext_dn + 0.002,
            details=f"z-extent +45deg={ext_up:.4f}, -45deg={ext_dn:.4f}",
        )

    # ---- hollow dark interior behind the slots -----------------------------
    liner = ctx.part_element_world_aabb(housing, elem="plenum_liner")
    assert liner is not None
    ctx.check(
        "dark plenum liner spans behind all three louver slots",
        liner[0][0] < -0.40
        and liner[1][0] > 0.40
        and liner[0][2] < ARC_BOT_CZ - 0.01
        and liner[1][2] > ARC_BOT_CZ + 0.01,
        details=f"liner aabb={liner}",
    )

    return ctx.report()


object_model = build_object_model()
