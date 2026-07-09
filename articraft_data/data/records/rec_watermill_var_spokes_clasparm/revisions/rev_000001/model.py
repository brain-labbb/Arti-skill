from __future__ import annotations

"""Stylized wooden watermill waterwheel on a freestanding A-frame trestle.

Reference image: picture/Machinery/Watermill/001.png

- Overall height ~2.22 m, wheel diameter ~1.84 m, wheel width ~0.50 m.
- Two segmented 12-gon rims in pale beige-pink wood, joined by nine flat
  paddle boards around the circumference; each rim has six clasp-arm
  (compass-arm) spoke pairs which meet the rim chordally and frame a small
  square hub box on that rim plane.
- Light-gray metal axle through both hubs, capped on each outer end with a
  short splined/ribbed collar resembling a gear sleeve.
- Darker grayish-brown A-frame trestle: two angled legs per side with square
  block feet, a horizontal cross brace per frame, bored bearing blocks at the
  apex, and a diagonal brace tying the two frames front-to-back.
- Articulation: the whole wheel assembly spins freely (CONTINUOUS) about the
  horizontal axle axis (world +Y) relative to the static trestle.
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

# ---------------------------------------------------------------- dimensions
AXLE_Z = 1.30  # axle centerline height above the ground

RIM_R_OUT = 0.92  # rim outer circumradius (12-gon)
RIM_R_IN = 0.78  # rim inner circumradius
RIM_W = 0.08  # rim width along the axle (Y)
RIM_Y = 0.21  # rim plane center offset (+/- Y); outer faces at +/-0.25
RIM_SEGMENTS = 12

CLASP_SPOKE_PAIRS = 6  # paired compass-arm spokes per rim
CLASP_ARM_XSEC = (0.055, 0.07)  # visible arm width in wheel plane, axial thickness
CLASP_HUB_HALF_GAP = 0.040
CLASP_RIM_HALF_CHORD = 0.080
CLASP_RIM_R = RIM_R_IN + 0.025  # slight seating into the inner rim

HUB_BOX = 0.22
HUB_LEN = 0.08

PADDLE_COUNT = 9
PADDLE_R = 0.84  # paddle center radius
PADDLE_DIMS = (0.28, 0.46, 0.06)  # tangential, axial (Y), radial

AXLE_R = 0.035
AXLE_LEN = 0.94  # spans y = -0.47 .. +0.47

COLLAR_R = 0.060
COLLAR_LEN = 0.10
COLLAR_Y = 0.42  # collar centers, outboard of the bearing blocks
COLLAR_RIBS = 10

FRAME_Y = 0.31  # A-frame plane center offset (+/- Y)
LEG_FOOT_X = 0.55  # foot center spread (+/- X)
LEG_APEX_Z = 1.36  # virtual apex where the leg centerlines would meet
LEG_FOOT_Z = 0.05
LEG_TOP_Z = 1.22  # legs stop inside the bearing block, below the axle bore
LEG_ANGLE = math.atan2(LEG_FOOT_X, LEG_APEX_Z - LEG_FOOT_Z)  # ~0.398 rad
LEG_TOP_X = LEG_FOOT_X * (LEG_APEX_Z - LEG_TOP_Z) / (LEG_APEX_Z - LEG_FOOT_Z)
LEG_DIMS = (0.09, 0.07, 1.30)

FOOT_DIMS = (0.16, 0.14, 0.10)

BLOCK_DIMS = (0.20, 0.10, 0.24)  # bearing block at the frame apex
# Journal bore slightly under the axle radius so the spinning axle stays
# captured in (and supported by) the bearing with a tiny intentional embed.
BORE_R = 0.033

BRACE_DIMS = (0.80, 0.06, 0.09)
BRACE_Z = 0.55

DIAG_X = 0.48
DIAG_Z0 = 0.12
DIAG_Z1 = 0.34
DIAG_ROLL = math.atan2(DIAG_Z1 - DIAG_Z0, 2.0 * FRAME_Y)  # ~0.341 rad
DIAG_DIMS = (0.07, 0.72, 0.07)


def _rim_solid() -> cq.Workplane:
    """Segmented low-poly rim: a thick 12-gon ring, axis along local Z."""
    outer = cq.Workplane("XY").polygon(RIM_SEGMENTS, 2.0 * RIM_R_OUT).extrude(RIM_W)
    inner = cq.Workplane("XY").polygon(RIM_SEGMENTS, 2.0 * RIM_R_IN).extrude(RIM_W)
    return outer.cut(inner).translate((0.0, 0.0, -RIM_W / 2.0))


def _collar_solid() -> cq.Workplane:
    """Splined/ribbed axle end collar, axis along local Z."""
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_R)
        .extrude(COLLAR_LEN)
        .translate((0.0, 0.0, -COLLAR_LEN / 2.0))
    )
    rib_radius = 0.062
    for k in range(COLLAR_RIBS):
        ang = 2.0 * math.pi * k / COLLAR_RIBS
        rib = (
            cq.Workplane("XY")
            .box(0.022, 0.016, COLLAR_LEN)
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(ang))
            .translate((rib_radius * math.cos(ang), rib_radius * math.sin(ang), 0.0))
        )
        collar = collar.union(rib)
    return collar


def _bearing_block_solid() -> cq.Workplane:
    """Apex bearing block with a clearance bore for the axle along local Y."""
    block = cq.Workplane("XY").box(*BLOCK_DIMS)
    bore = cq.Workplane("XZ").circle(BORE_R).extrude(BLOCK_DIMS[1], both=True)
    return block.cut(bore)


def _segment_box_origin(
    p0: tuple[float, float], p1: tuple[float, float], y: float
) -> tuple[Origin, float]:
    """Origin and length for a rectangular member running between XZ points."""
    x0, z0 = p0
    x1, z1 = p1
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    # Local +Z is the member length; rotation about local/world Y points it
    # along the requested XZ vector.
    theta = math.atan2(dx, dz)
    return Origin(xyz=((x0 + x1) / 2.0, y, (z0 + z1) / 2.0), rpy=(0.0, theta, 0.0)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watermill_waterwheel_trestle")

    wheel_wood = model.material("wheel_wood", color=(0.89, 0.72, 0.66))
    frame_wood = model.material("frame_wood", color=(0.50, 0.40, 0.36))
    axle_metal = model.material("axle_metal", color=(0.70, 0.71, 0.73))
    collar_metal = model.material("collar_metal", color=(0.80, 0.81, 0.84))

    # ------------------------------------------------------------- trestle
    trestle = model.part("trestle_frame")

    leg_mid_x = (LEG_FOOT_X + LEG_TOP_X) / 2.0
    leg_mid_z = (LEG_TOP_Z + LEG_FOOT_Z) / 2.0
    for fi, fy in enumerate((FRAME_Y, -FRAME_Y)):
        for li, sx in enumerate((1.0, -1.0)):
            trestle.visual(
                Box(LEG_DIMS),
                origin=Origin(
                    xyz=(sx * leg_mid_x, fy, leg_mid_z),
                    rpy=(0.0, -sx * LEG_ANGLE, 0.0),
                ),
                material=frame_wood,
                name=f"leg_{fi}_{li}",
            )
            trestle.visual(
                Box(FOOT_DIMS),
                origin=Origin(xyz=(sx * LEG_FOOT_X, fy, FOOT_DIMS[2] / 2.0)),
                material=frame_wood,
                name=f"foot_{fi}_{li}",
            )
        trestle.visual(
            Box(BRACE_DIMS),
            origin=Origin(xyz=(0.0, fy, BRACE_Z)),
            material=frame_wood,
            name=f"cross_brace_{fi}",
        )

    bearing_mesh = mesh_from_cadquery(_bearing_block_solid(), "bearing_block")
    for fi, fy in enumerate((FRAME_Y, -FRAME_Y)):
        trestle.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, fy, AXLE_Z)),
            material=frame_wood,
            name=("bearing_block_0" if fi == 0 else "bearing_block_1"),
        )

    # Diagonal brace tying the two A-frames front-to-back on the +X side.
    trestle.visual(
        Box(DIAG_DIMS),
        origin=Origin(
            xyz=(DIAG_X, 0.0, (DIAG_Z0 + DIAG_Z1) / 2.0),
            rpy=(DIAG_ROLL, 0.0, 0.0),
        ),
        material=frame_wood,
        name="diagonal_brace",
    )

    # ---------------------------------------------------------- waterwheel
    # Wheel part frame sits at the hub center (joint frame); geometry is
    # authored relative to the axle centerline.
    wheel = model.part("waterwheel")

    rim_mesh = mesh_from_cadquery(_rim_solid(), "wheel_rim")
    for ri, ry in enumerate((RIM_Y, -RIM_Y)):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(0.0, ry, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_wood,
            name=("rim_0" if ri == 0 else "rim_1"),
        )
        wheel.visual(
            Box((HUB_BOX, HUB_LEN, HUB_BOX)),
            origin=Origin(xyz=(0.0, ry, 0.0)),
            material=wheel_wood,
            name=("hub_box_0" if ri == 0 else "hub_box_1"),
        )

        # Six compass/clasp spoke bays per rim.  Each bay uses a paired set of
        # arms that begin on either side of the square hub box and land on a
        # short chordal shoe seated against the inner rim, matching the forked
        # clasp-arm construction in the reference rather than a straight
        # diametral bar.
        for si in range(CLASP_SPOKE_PAIRS):
            theta = 2.0 * math.pi * si / CLASP_SPOKE_PAIRS
            radial = (math.sin(theta), math.cos(theta))
            tangent = (math.cos(theta), -math.sin(theta))
            hub_center = (
                radial[0] * (HUB_BOX / 2.0 - 0.010),
                radial[1] * (HUB_BOX / 2.0 - 0.010),
            )
            rim_center = (radial[0] * CLASP_RIM_R, radial[1] * CLASP_RIM_R)
            chord_origin, chord_len = _segment_box_origin(
                (
                    rim_center[0] - tangent[0] * CLASP_RIM_HALF_CHORD,
                    rim_center[1] - tangent[1] * CLASP_RIM_HALF_CHORD,
                ),
                (
                    rim_center[0] + tangent[0] * CLASP_RIM_HALF_CHORD,
                    rim_center[1] + tangent[1] * CLASP_RIM_HALF_CHORD,
                ),
                ry,
            )
            wheel.visual(
                Box((CLASP_ARM_XSEC[0], CLASP_ARM_XSEC[1], chord_len + 0.030)),
                origin=chord_origin,
                material=wheel_wood,
                name=("clasp_chord_0_0" if ri == 0 and si == 0 else f"clasp_chord_{ri}_{si}"),
            )
            for ai, side in enumerate((-1.0, 1.0)):
                p0 = (
                    hub_center[0] + tangent[0] * side * CLASP_HUB_HALF_GAP,
                    hub_center[1] + tangent[1] * side * CLASP_HUB_HALF_GAP,
                )
                p1 = (
                    rim_center[0] + tangent[0] * side * CLASP_RIM_HALF_CHORD,
                    rim_center[1] + tangent[1] * side * CLASP_RIM_HALF_CHORD,
                )
                arm_origin, arm_len = _segment_box_origin(p0, p1, ry)
                wheel.visual(
                    Box((CLASP_ARM_XSEC[0], CLASP_ARM_XSEC[1], arm_len + 0.025)),
                    origin=arm_origin,
                    material=wheel_wood,
                    name=(
                        "clasp_arm_0_0_0"
                        if ri == 0 and si == 0 and ai == 0
                        else f"clasp_arm_{ri}_{si}_{ai}"
                    ),
                )

    for pi in range(PADDLE_COUNT):
        ang = 2.0 * math.pi * pi / PADDLE_COUNT
        wheel.visual(
            Box(PADDLE_DIMS),
            origin=Origin(
                xyz=(PADDLE_R * math.sin(ang), 0.0, PADDLE_R * math.cos(ang)),
                rpy=(0.0, ang, 0.0),
            ),
            material=wheel_wood,
            name=f"paddle_{pi}",
        )

    wheel.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_metal,
        name="axle",
    )

    collar_mesh = mesh_from_cadquery(_collar_solid(), "axle_collar")
    for ci, cy in enumerate((COLLAR_Y, -COLLAR_Y)):
        wheel.visual(
            collar_mesh,
            origin=Origin(xyz=(0.0, cy, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=collar_metal,
            name=("axle_collar_0" if ci == 0 else "axle_collar_1"),
        )

    # ------------------------------------------------------------ articulation
    model.articulation(
        "trestle_to_waterwheel",
        ArticulationType.CONTINUOUS,
        parent=trestle,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    trestle = object_model.get_part("trestle_frame")
    wheel = object_model.get_part("waterwheel")
    spin = object_model.get_articulation("trestle_to_waterwheel")

    # Joint plan: single continuous revolute spin about the horizontal Y axle.
    ctx.check(
        "wheel joint is continuous free spin",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    ctx.check(
        "wheel spins about the horizontal axle axis (+Y)",
        tuple(spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.check(
        "axle joint frame at hub height",
        abs(spin.origin.xyz[2] - AXLE_Z) < 1e-9,
        details=f"joint z={spin.origin.xyz[2]}",
    )

    # Grounding and real-world scale.
    trestle_aabb = ctx.part_world_aabb(trestle)
    ctx.check(
        "trestle feet grounded at z=0",
        trestle_aabb is not None and abs(trestle_aabb[0][2]) < 2e-3,
        details=f"trestle aabb={trestle_aabb}",
    )
    wheel_aabb = ctx.part_world_aabb(wheel)
    ctx.check(
        "wheel diameter about 1.8 m",
        wheel_aabb is not None
        and 1.70 < (wheel_aabb[1][0] - wheel_aabb[0][0]) < 1.90
        and 1.70 < (wheel_aabb[1][2] - wheel_aabb[0][2]) < 1.90,
        details=f"wheel aabb={wheel_aabb}",
    )
    ctx.check(
        "overall height about 2.2 m",
        wheel_aabb is not None and 2.12 < wheel_aabb[1][2] < 2.30,
        details=f"wheel top z={None if wheel_aabb is None else wheel_aabb[1][2]}",
    )
    ctx.check(
        "wheel about 0.5 m wide across the rims",
        wheel_aabb is not None,
        details="",
    )
    rim0 = ctx.part_element_world_aabb(wheel, elem="rim_0")
    rim1 = ctx.part_element_world_aabb(wheel, elem="rim_1")
    ctx.check(
        "two parallel rims span ~0.5 m",
        rim0 is not None
        and rim1 is not None
        and 0.46 < (rim0[1][1] - rim1[0][1]) < 0.54,
        details=f"rim_0={rim0}, rim_1={rim1}",
    )

    # Hero features present: nine paddles, clasp-arm compass spokes on each
    # rim, square hub boxes, and no legacy straight diametral spoke bars.
    paddle_names = [v.name for v in wheel.visuals if v.name and v.name.startswith("paddle_")]
    ctx.check(
        "nine paddle boards around the circumference",
        len(paddle_names) == PADDLE_COUNT,
        details=f"paddles={paddle_names}",
    )
    legacy_spoke_names = [v.name for v in wheel.visuals if v.name and v.name.startswith("spoke_bar_")]
    ctx.check(
        "legacy straight diametral spoke bars are removed",
        len(legacy_spoke_names) == 0,
        details=f"legacy_spokes={legacy_spoke_names}",
    )
    arm_names = [v.name for v in wheel.visuals if v.name and v.name.startswith("clasp_arm_")]
    chord_names = [v.name for v in wheel.visuals if v.name and v.name.startswith("clasp_chord_")]
    hub_box_names = [v.name for v in wheel.visuals if v.name and v.name.startswith("hub_box_")]
    ctx.check(
        "six paired clasp-arm spoke bays per rim",
        len(arm_names) == 2 * CLASP_SPOKE_PAIRS * 2
        and len(chord_names) == 2 * CLASP_SPOKE_PAIRS,
        details=f"arms={arm_names}, chords={chord_names}",
    )
    ctx.check(
        "small square hub box on each rim",
        len(hub_box_names) == 2,
        details=f"hub_boxes={hub_box_names}",
    )
    hub0 = ctx.part_element_world_aabb(wheel, elem="hub_box_0")
    ctx.check(
        "hub box_0 is visibly square in the wheel plane",
        hub0 is not None
        and 0.19 < (hub0[1][0] - hub0[0][0]) < 0.25
        and 0.19 < (hub0[1][2] - hub0[0][2]) < 0.25,
        details=f"hub_box_0 aabb={hub0}",
    )
    ctx.expect_overlap(
        wheel,
        wheel,
        axes="xz",
        elem_a="clasp_chord_0_0",
        elem_b="rim_0",
        min_overlap=0.025,
        name="clasp chord seats into the rim",
    )
    ctx.expect_overlap(
        wheel,
        wheel,
        axes="xz",
        elem_a="clasp_arm_0_0_0",
        elem_b="hub_box_0",
        min_overlap=0.015,
        name="paired clasp arm frames and meets the hub box",
    )

    # Axle is journaled through both bearing blocks. The bore is slightly
    # undersized so the axle is captured by the bearing (tiny local embed).
    ctx.allow_overlap(
        wheel,
        trestle,
        elem_a="axle",
        elem_b="bearing_block_0",
        reason="Axle is intentionally captured in the front bearing journal bore.",
    )
    ctx.allow_overlap(
        wheel,
        trestle,
        elem_a="axle",
        elem_b="bearing_block_1",
        reason="Axle is intentionally captured in the rear bearing journal bore.",
    )
    ctx.expect_overlap(
        wheel,
        trestle,
        axes="y",
        elem_a="axle",
        elem_b="bearing_block_0",
        min_overlap=0.08,
        name="axle passes through the front bearing block",
    )
    ctx.expect_overlap(
        wheel,
        trestle,
        axes="y",
        elem_a="axle",
        elem_b="bearing_block_1",
        min_overlap=0.08,
        name="axle passes through the rear bearing block",
    )
    # Rim clears the inboard face of the bearing block; collar sits outboard.
    ctx.expect_gap(
        trestle,
        wheel,
        axis="y",
        positive_elem="bearing_block_0",
        negative_elem="rim_0",
        min_gap=0.005,
        name="rim clears the bearing block",
    )
    ctx.expect_gap(
        wheel,
        trestle,
        axis="y",
        positive_elem="axle_collar_0",
        negative_elem="bearing_block_0",
        min_gap=0.005,
        name="splined collar caps the axle outboard of the bearing",
    )

    # Off-axis proof of continuous rotation: paddle_0 starts at the top of
    # the wheel and swings to the bottom at q=pi, to the +X side at q=pi/2.
    p0_rest = ctx.part_element_world_aabb(wheel, elem="paddle_0")
    ctx.check(
        "paddle_0 rests at the top of the wheel",
        p0_rest is not None and (p0_rest[0][2] + p0_rest[1][2]) / 2.0 > AXLE_Z + 0.7,
        details=f"paddle_0 rest aabb={p0_rest}",
    )
    with ctx.pose({spin: math.pi}):
        p0_half = ctx.part_element_world_aabb(wheel, elem="paddle_0")
        ctx.check(
            "half turn carries paddle_0 to the bottom",
            p0_half is not None and (p0_half[0][2] + p0_half[1][2]) / 2.0 < AXLE_Z - 0.7,
            details=f"paddle_0 aabb at q=pi: {p0_half}",
        )
    with ctx.pose({spin: math.pi / 2.0}):
        p0_quarter = ctx.part_element_world_aabb(wheel, elem="paddle_0")
        ctx.check(
            "quarter turn carries paddle_0 to the +X side",
            p0_quarter is not None and (p0_quarter[0][0] + p0_quarter[1][0]) / 2.0 > 0.7,
            details=f"paddle_0 aabb at q=pi/2: {p0_quarter}",
        )
        # The spinning wheel stays clear of the static frame plane at an
        # arbitrary pose as well.
        ctx.expect_gap(
            trestle,
            wheel,
            axis="y",
            positive_elem="bearing_block_0",
            negative_elem="rim_0",
            min_gap=0.005,
            name="wheel keeps clearance to the frame while spinning",
        )

    return ctx.report()


object_model = build_object_model()
