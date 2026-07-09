from __future__ import annotations

"""Wall-mounted mini-split air conditioner indoor unit (vertical-vane variant).

Glossy white rounded horizontal body ~0.90 m wide x 0.30 m tall x 0.22 m deep,
back flat against the wall plane (y=0), bottom at z=0.  The lower front curve
carries a single wide horizontal outlet slot.  Inside the outlet a bank of
vertical deflector vanes is evenly spaced across the width; each vane pivots
left-right about the vertical Z axis to direct airflow horizontally.

The large framed front face panel is a service cover hinged along its top
edge, swinging upward to reveal a shallow filter cavity with a removable-look
filter panel.  The interior behind the outlet slot is a hollow dark plenum.

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

# ---------------------------------------------------------------------------
# Single horizontal outlet with vertical deflector vane bank
# ---------------------------------------------------------------------------
N_VERTICAL_VANES = 12
OUTLET_THETA = math.radians(55.0)  # arc angle for the outlet centre
OUTLET_LEN = 0.78  # outlet width along X
OUTLET_OPEN = 0.048  # slot opening along the surface tangent
OUTLET_DEPTH = 0.12  # cut depth along the surface normal (into plenum)
VVANE_HEIGHT = 0.026  # vane blade height along Z
VVANE_CHORD = 0.020  # vane chord (depth along Y at q=0)
VVANE_T = 0.003  # vane plate thickness
VVANE_SWING = math.radians(45.0)
# Vertical pivot shaft: extends above the slot cut into the housing wall
# so the vane reads as captured on a vertical pivot.
VVANE_SHAFT_R = 0.004  # pivot shaft radius
VVANE_SHAFT_LEN = 0.058  # shaft total length along Z
VVANE_SHAFT_Z_OFF = 0.006  # shaft center offset above blade center

# Hollow dark plenum behind the outlet slot (cross-flow blower bay).
PLENUM_R = 0.08
PLENUM_HALF_LEN = 0.41
LINER_R = 0.075
LINER_HALF_LEN = 0.42  # ends embed into the chassis side walls


def _arc_point(theta: float) -> tuple[float, float]:
    """(y, z) point on the bottom-front quarter-round at arc angle theta."""
    return (ARC_CY + ARC_R * math.sin(theta), ARC_CZ - ARC_R * math.cos(theta))


def _vane_center_x(i: int) -> float:
    """X position of the i-th vertical vane, evenly spaced across the outlet."""
    spacing = OUTLET_LEN / N_VERTICAL_VANES
    return -OUTLET_LEN / 2.0 + spacing * (i + 0.5)


def _vertical_vane_blade_shape() -> cq.Workplane:
    """Shared vertical deflector vane blade geometry.

    Thin plate: thickness along X, chord (depth) along Y, height along Z.
    At q=0 the chord faces outward (+Y) and rotation about Z sweeps it
    left-right to deflect airflow.
    """
    return cq.Workplane("XY").box(VVANE_T, VVANE_CHORD, VVANE_HEIGHT)


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
    # One wide horizontal outlet slot through the curved lower-front wall.
    ay, az = _arc_point(OUTLET_THETA)
    outlet = (
        cq.Workplane("XY")
        .box(OUTLET_LEN, OUTLET_DEPTH, OUTLET_OPEN)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(OUTLET_THETA) - 90.0)
        .translate((0.0, ay, az))
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
    # Dark blower-bay liner: visible through the outlet slot, ends embedded
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

    # ----------------------------------------- vertical deflector vane bank
    ay, az = _arc_point(OUTLET_THETA)
    for i in range(N_VERTICAL_VANES):
        vane = model.part(f"vertical_vane_{i}")
        # Blade: thin vertical plate, chord along Y at q=0.
        vane.visual(
            mesh_from_cadquery(_vertical_vane_blade_shape(), f"vane_blade_{i}"),
            material="vane_white",
            name=f"vane_blade_{i}",
        )
        # Vertical pivot shaft: extends above the slot cut into the housing
        # wall so the vane is captured on a real vertical pivot.  The shaft
        # also extends slightly below the blade for the bottom bearing seat.
        vane.visual(
            Cylinder(radius=VVANE_SHAFT_R, length=VVANE_SHAFT_LEN),
            origin=Origin(xyz=(0.0, 0.0, VVANE_SHAFT_Z_OFF)),
            material="vane_white",
            name=f"vane_shaft_{i}",
        )
        vx = _vane_center_x(i)
        model.articulation(
            f"vane_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=vane,
            origin=Origin(xyz=(vx, ay, az)),
            # Vertical axis: positive q sweeps the blade chord toward +X.
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.0,
                lower=-VVANE_SWING,
                upper=VVANE_SWING,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    panel = object_model.get_part("front_panel")
    hinge = object_model.get_articulation("front_panel_hinge")
    vanes = [
        object_model.get_part(f"vertical_vane_{i}")
        for i in range(N_VERTICAL_VANES)
    ]
    pivots = [
        object_model.get_articulation(f"vane_pivot_{i}")
        for i in range(N_VERTICAL_VANES)
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

    # ---- intentional hinge embeds -----------------------------------------
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

    # ---- vane shaft embeds into the housing wall above the outlet ----------
    for i in range(N_VERTICAL_VANES):
        vane = vanes[i]
        ctx.allow_overlap(
            vane,
            housing,
            elem_a=f"vane_shaft_{i}",
            elem_b="housing_shell",
            reason="Vertical pivot shaft is intentionally captured in the housing wall above the outlet slot to mount the deflector vane.",
        )
        ctx.expect_contact(
            vane,
            housing,
            elem_a=f"vane_shaft_{i}",
            elem_b="housing_shell",
            name=f"vane {i} shaft is captured in the housing wall",
        )

    # ---- front service panel: closed seating and hinge --------------------
    ctx.expect_within(
        panel, housing, axes="x", margin=0.002, name="panel within body width"
    )
    ctx.expect_overlap(
        panel,
        housing,
        axes="xz",
        min_overlap=0.05,
        name="closed panel covers the front face",
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
            pb_open[1][1] > hb[1][1] + 0.08
            and pb_open[0][2] > pb_closed[0][2] + 0.05,
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

    # ---- vertical vane bank: count, spacing, joint policy -----------------
    ctx.check(
        f"vertical vane bank has {N_VERTICAL_VANES} vanes",
        len(vanes) == N_VERTICAL_VANES and len(pivots) == N_VERTICAL_VANES,
        details=f"vanes={len(vanes)}, pivots={len(pivots)}",
    )
    prev_x = -float("inf")
    for i in range(N_VERTICAL_VANES):
        vane = vanes[i]
        pivot = pivots[i]
        vb = ctx.part_world_aabb(vane)
        assert vb is not None
        cx = 0.5 * (vb[0][0] + vb[1][0])
        expected_x = _vane_center_x(i)

        # Even spacing across the outlet width.
        ctx.check(
            f"vane {i} is at its expected X position",
            abs(cx - expected_x) < 0.005,
            details=f"cx={cx:.4f}, expected={expected_x:.4f}",
        )
        if i > 0:
            ctx.check(
                f"vane {i} is rightward of vane {i - 1}",
                cx > prev_x + 0.01,
                details=f"cx={cx:.4f}, prev={prev_x:.4f}",
            )
        prev_x = cx

        # Uniform joint policy: every vane pivots about Z with the same limits.
        vlimits = pivot.motion_limits
        ctx.check(
            f"vane {i} pivots +/-45 deg about the vertical Z axis",
            vlimits is not None
            and abs(vlimits.lower + VVANE_SWING) < 0.02
            and abs(vlimits.upper - VVANE_SWING) < 0.02
            and tuple(pivot.axis) == (0.0, 0.0, 1.0),
            details=f"axis={pivot.axis}, limits=({vlimits.lower}, {vlimits.upper})",
        )
        ctx.expect_within(
            vane,
            housing,
            axes="x",
            margin=0.002,
            name=f"vane {i} within housing width",
        )

    # Decisive motion check on the middle vane: rotating about Z sweeps the
    # chord from Y into X, so the world X-extent must grow at +45 deg.
    mid = N_VERTICAL_VANES // 2
    mid_vane = vanes[mid]
    mid_pivot = pivots[mid]
    with ctx.pose({mid_pivot: 0.0}):
        rest = ctx.part_world_aabb(mid_vane)
    with ctx.pose({mid_pivot: VVANE_SWING}):
        swung = ctx.part_world_aabb(mid_vane)
    assert rest is not None and swung is not None
    rest_dx = rest[1][0] - rest[0][0]
    swung_dx = swung[1][0] - swung[0][0]
    ctx.check(
        f"vane {mid} blade sweeps left-right when rotated about Z",
        swung_dx > rest_dx + 0.005,
        details=f"rest dx={rest_dx:.4f}, swung dx={swung_dx:.4f}",
    )
    # And the Y-extent shrinks as the chord rotates away from Y.
    rest_dy = rest[1][1] - rest[0][1]
    swung_dy = swung[1][1] - swung[0][1]
    ctx.check(
        f"vane {mid} chord rotates out of Y when swept",
        rest_dy > swung_dy + 0.003,
        details=f"rest dy={rest_dy:.4f}, swung dy={swung_dy:.4f}",
    )

    # ---- hollow dark interior behind the outlet -----------------------------
    liner = ctx.part_element_world_aabb(housing, elem="plenum_liner")
    assert liner is not None
    ctx.check(
        "dark plenum liner spans behind the outlet slot",
        liner[0][0] < -0.40
        and liner[1][0] > 0.40
        and liner[0][2] < 0.06
        and liner[1][2] > 0.19,
        details=f"liner aabb={liner}",
    )

    return ctx.report()


object_model = build_object_model()
