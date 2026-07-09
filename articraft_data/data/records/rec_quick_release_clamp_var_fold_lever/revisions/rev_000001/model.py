from __future__ import annotations

from math import pi, radians

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Layout constants (meters). World frame: +Z is the collar bore axis (up),
# the open Omega throat faces -X, the cross bolt runs along Y through the two
# clamping feet. The folding release lever lives outside the +Y bolt head. The
# bolt, head, and hinge pin are fixed hardware; the flat lever itself hinges up
# about an X-axis pin and lies down along the bolt axis when closed.
# ---------------------------------------------------------------------------
BORE_R = 0.016  # inner bore radius (0.032 m diameter per prompt)
BAND_OUTER_R = 0.021  # bore + 0.005 wall
BAND_H = 0.015  # band height per prompt

# Omega throat: the ring stays OPEN on the -X side. A wedge sector is removed so
# the bore breaks out into the gap between the two feet, giving the Omega shape.
THROAT_OPEN_OUT = 0.0098  # half width of the throat at the far (outer) edge
THROAT_APEX_X = 0.0015  # wedge apex just past bore center toward +X

LUG_X_MIN, LUG_X_MAX = -0.038, -0.016  # flat boss feet flanking the throat
LUG_Y_IN, LUG_Y_OUT = 0.0050, 0.0100  # foot inner/outer faces in |y|
LUG_LEN = LUG_X_MAX - LUG_X_MIN
LUG_XC = 0.5 * (LUG_X_MIN + LUG_X_MAX)
LUG_T = LUG_Y_OUT - LUG_Y_IN
LUG_YC = 0.5 * (LUG_Y_IN + LUG_Y_OUT)

PIVOT_X = -0.0305  # cross-bolt axis
PIVOT_Z = 0.5 * BAND_H

BOLT_R = 0.0025
BOLT_Y_MIN, BOLT_Y_MAX = -0.0225, 0.0162  # spans nut, feet, and the fixed bolt head

WASHER_R = 0.0064
WASHER_LEN = 0.0016

HEAD_R = 0.0059
HEAD_LEN = 0.0046
HEAD_YC = LUG_Y_OUT + WASHER_LEN + 0.0003 + 0.5 * HEAD_LEN

HINGE_Y = 0.0224
PIN_R = 0.00255
PIN_LEN = 0.0185
LEVER_EYE_R = 0.0054
LEVER_PIN_CLEAR_R = 0.00255
LEVER_EYE_HALF_X = 0.0050
FORK_CHEEK_X = 0.0068
FORK_CHEEK_T = 0.0022
FORK_CHEEK_LEN = 0.0082
FORK_CHEEK_H = 0.0115

# Folding lever lives in the local X-Y plane and is thickened along local Z.
# In the zero pose it extends along local +Y, collinear with the cross bolt; a
# positive hinge angle about +X raises its free end toward +Z.
LEVER_Z_HALF = 0.00155
LEVER_LEN = 0.060

NUT_D = 0.022  # knurled adjuster barrel nut per prompt
NUT_LEN = 0.020
NUT_YC = -LUG_Y_OUT - WASHER_LEN - 0.0004 - 0.5 * NUT_LEN  # outside the -Y foot face

LEVER_OPEN = radians(95.0)


def _throat_notch_solid() -> cq.Workplane:
    """Wedge sector removed from the -X side to open the ring into an Omega."""
    return (
        cq.Workplane("XY")
        .polyline(
            [
                (THROAT_APEX_X, 0.0),
                (-(BAND_OUTER_R + 0.005), THROAT_OPEN_OUT),
                (-(BAND_OUTER_R + 0.005), -THROAT_OPEN_OUT),
            ]
        )
        .close()
        .extrude(BAND_H + 0.012)
        .translate((0.0, 0.0, -0.006))
    )


def _collar_band_solid() -> cq.Workplane:
    """Open split-ring band in an Omega profile: the ring wraps the bore but
    stays OPEN on the -X (foot) side, where the bore breaks out into the gap
    between the two clamping feet."""
    band = cq.Workplane("XY").circle(BAND_OUTER_R).circle(BORE_R).extrude(BAND_H)
    # Shallow counterbore step at the top opening (visible seat lip).
    counterbore = (
        cq.Workplane("XY", origin=(0.0, 0.0, BAND_H - 0.005)).circle(0.0175).extrude(0.007)
    )
    band = band.cut(counterbore)
    # Omega opening on the -X side.
    return band.cut(_throat_notch_solid())


def _annular_ring_solid(inner_r: float, outer_r: float, height: float) -> cq.Workplane:
    """Annular ring with the Omega throat removed so it never re-closes the gap."""
    ring = cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(height)
    return ring.cut(_throat_notch_solid())


def _lug_solid(yc: float) -> cq.Workplane:
    lug = (
        cq.Workplane("XY")
        .box(LUG_LEN, LUG_T, BAND_H, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.0010)
        .edges(">Z or <Z")
        .fillet(0.00045)
    )
    return lug.translate((LUG_XC, yc, 0.5 * BAND_H))


def _washer_solid() -> cq.Workplane:
    # Inner radius < bolt radius so the washer grips the bolt (stays connected).
    return _annular_washer_solid(BOLT_R * 0.8, WASHER_R, WASHER_LEN)


def _annular_washer_solid(inner_r: float, outer_r: float, height: float) -> cq.Workplane:
    """Plain (un-notched) annular ring, for washers riding on the bolt."""
    return cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(height)


def _folding_lever_solid() -> cq.Workplane:
    """Flat one-piece folding lever with a bored hinge eye.

    Local frame: the hinge pin is the local X axis through the origin. In the
    closed pose the lever blade extends along +Y and lies flat around Z=0.
    """
    eye = (
        cq.Workplane("YZ")
        .circle(LEVER_EYE_R)
        .circle(LEVER_PIN_CLEAR_R)
        .extrude(LEVER_EYE_HALF_X, both=True)
    )
    blade = (
        cq.Workplane("XY")
        .moveTo(-0.0043, 0.0008)
        .spline([(-0.0045, 0.018), (-0.0069, 0.039), (-0.0046, LEVER_LEN)], includeCurrent=True)
        .radiusArc((0.0046, LEVER_LEN), 0.0046)
        .spline([(0.0069, 0.039), (0.0045, 0.018), (0.0043, 0.0008)], includeCurrent=True)
        .close()
        .extrude(LEVER_Z_HALF, both=True)
    )
    spine = (
        cq.Workplane("XY")
        .moveTo(-0.0022, 0.0070)
        .lineTo(-0.0016, 0.047)
        .radiusArc((0.0016, 0.047), 0.0016)
        .lineTo(0.0022, 0.0070)
        .close()
        .extrude(0.00045)
        .translate((0.0, 0.0, LEVER_Z_HALF - 0.00005))
    )
    solid = eye.union(blade).union(spine)
    pivot_bore = (
        cq.Workplane("YZ")
        .circle(LEVER_PIN_CLEAR_R)
        .extrude(0.012, both=True)
    )
    return solid.cut(pivot_bore)


def _fork_cheek_solid(xc: float) -> cq.Workplane:
    """One fixed cheek of the bolt-head clevis supporting the lever hinge pin."""
    return (
        cq.Workplane("XY")
        .box(FORK_CHEEK_T, FORK_CHEEK_LEN, FORK_CHEEK_H, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.00045)
        .edges(">Z or <Z")
        .fillet(0.00025)
        .translate((xc, 0.5 * (BOLT_Y_MAX + HINGE_Y), PIVOT_Z))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quick_release_seat_clamp")

    alu_body = model.material("brushed_aluminum_body", color=(0.66, 0.67, 0.69, 1.0))
    alu_lever = model.material("brushed_aluminum_lever", color=(0.72, 0.73, 0.75, 1.0))
    alu_nut = model.material("brushed_aluminum_nut", color=(0.69, 0.70, 0.72, 1.0))
    steel = model.material("silver_steel_bolt", color=(0.55, 0.56, 0.58, 1.0))
    brushed_dark = model.material("fine_brushed_shadow_lines", color=(0.42, 0.43, 0.45, 1.0))
    brushed_light = model.material("polished_edge_highlights", color=(0.82, 0.83, 0.84, 1.0))

    # ------------------------------------------------------------------ collar
    collar = model.part("collar")
    collar.visual(
        mesh_from_cadquery(_collar_band_solid(), "collar_band"),
        origin=Origin(),
        material=alu_body,
        name="collar_band",
    )
    collar.visual(
        mesh_from_cadquery(_lug_solid(LUG_YC), "lug_cap_side"),
        origin=Origin(),
        material=alu_body,
        name="lug_cap_side",
    )
    collar.visual(
        mesh_from_cadquery(_lug_solid(-LUG_YC), "lug_nut_side"),
        origin=Origin(),
        material=alu_body,
        name="lug_nut_side",
    )
    for z, name in ((0.0004, "lower_machined_lip"), (BAND_H - 0.0008, "upper_machined_lip")):
        collar.visual(
            mesh_from_cadquery(
                _annular_ring_solid(BORE_R + 0.0004, BAND_OUTER_R + 0.0003, 0.00045), name
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brushed_light,
            name=name,
        )
    for z, name in ((0.0043, "lower_brushed_groove"), (0.0107, "upper_brushed_groove")):
        collar.visual(
            mesh_from_cadquery(
                _annular_ring_solid(BAND_OUTER_R + 0.00005, BAND_OUTER_R + 0.00025, 0.00018), name
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brushed_dark,
            name=name,
        )
    collar.visual(
        Cylinder(radius=BOLT_R, length=BOLT_Y_MAX - BOLT_Y_MIN),
        origin=Origin(
            xyz=(PIVOT_X, 0.5 * (BOLT_Y_MIN + BOLT_Y_MAX), PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)
        ),
        material=steel,
        name="cross_bolt",
    )
    collar.visual(
        mesh_from_cadquery(_washer_solid(), "cam_side_thrust_washer"),
        origin=Origin(
            xyz=(PIVOT_X, LUG_Y_OUT + 0.5 * WASHER_LEN, PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)
        ),
        material=brushed_light,
        name="cam_side_thrust_washer",
    )
    collar.visual(
        mesh_from_cadquery(_washer_solid(), "nut_side_thrust_washer"),
        origin=Origin(
            xyz=(PIVOT_X, -LUG_Y_OUT - 0.5 * WASHER_LEN, PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)
        ),
        material=brushed_light,
        name="nut_side_thrust_washer",
    )
    collar.visual(
        Cylinder(radius=HEAD_R, length=HEAD_LEN),
        origin=Origin(
            xyz=(PIVOT_X, HEAD_YC, PIVOT_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="bolt_head",
    )
    for i, x in enumerate((PIVOT_X - FORK_CHEEK_X, PIVOT_X + FORK_CHEEK_X)):
        collar.visual(
            mesh_from_cadquery(_fork_cheek_solid(x), f"fork_cheek_{i}"),
            origin=Origin(),
            material=steel,
            name=f"fork_cheek_{i}",
        )
    collar.visual(
        Cylinder(radius=PIN_R, length=PIN_LEN),
        origin=Origin(
            xyz=(PIVOT_X, HINGE_Y, PIVOT_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=steel,
        name="hinge_pin",
    )
    for i, x in enumerate((PIVOT_X - 0.5 * PIN_LEN - 0.0006, PIVOT_X + 0.5 * PIN_LEN + 0.0006)):
        collar.visual(
            Cylinder(radius=PIN_R * 1.35, length=0.0012),
            origin=Origin(xyz=(x, HINGE_Y, PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_light,
            name=f"pin_head_{i}",
        )

    # ---------------------------------------------------------- folding lever
    folding_lever = model.part("folding_lever")
    folding_lever.visual(
        mesh_from_cadquery(_folding_lever_solid(), "flat_lever"),
        origin=Origin(),
        material=alu_lever,
        name="flat_lever",
    )
    # ------------------------------------------------------------ adjuster nut
    adjuster_nut = model.part("adjuster_nut")
    knurled = KnobGeometry(
        NUT_D,
        NUT_LEN,
        body_style="cylindrical",
        edge_radius=0.0008,
        grip=KnobGrip(style="knurled", count=40, depth=0.0006, helix_angle_deg=0.0),
        bore=KnobBore(style="round", diameter=0.0046, through=False),
        center=True,
    )
    adjuster_nut.visual(
        mesh_from_geometry(knurled, "knurled_nut"),
        # Knob local +Z -> world -Y, so the bore opening faces the foot/bolt.
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=alu_nut,
        name="knurled_nut",
    )

    # ------------------------------------------------------------ articulations
    model.articulation(
        "lever_hinge",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=folding_lever,
        origin=Origin(xyz=(PIVOT_X, HINGE_Y, PIVOT_Z)),
        # About +X: positive q folds the free end upward to release the clamp.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.0, lower=0.0, upper=LEVER_OPEN),
        motion_properties=MotionProperties(damping=0.2, friction=0.05),
    )
    model.articulation(
        "adjuster_nut_spin",
        ArticulationType.CONTINUOUS,
        parent=collar,
        child=adjuster_nut,
        origin=Origin(xyz=(PIVOT_X, NUT_YC, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    collar = object_model.get_part("collar")
    folding_lever = object_model.get_part("folding_lever")
    adjuster_nut = object_model.get_part("adjuster_nut")
    hinge = object_model.get_articulation("lever_hinge")
    spin = object_model.get_articulation("adjuster_nut_spin")

    # Intentional captured-pin fit on the fixed clamp hardware.
    ctx.allow_overlap(
        collar,
        adjuster_nut,
        elem_a="cross_bolt",
        elem_b="knurled_nut",
        reason="The bolt end threads into the adjuster nut bore (thread engagement proxy).",
    )
    ctx.allow_overlap(
        collar,
        folding_lever,
        elem_a="hinge_pin",
        elem_b="flat_lever",
        reason="The hinge pin is intentionally modeled as a zero-clearance captured pin through the folding lever eye.",
    )

    # Joint semantics match the requested fork variant: no cam-over-center
    # barrel, and the flat lever folds upward about a hinge pin on the bolt head.
    ctx.check(
        "folding lever is a revolute hinge about the pin X axis",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge.axis) == (1.0, 0.0, 0.0)
        and hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(hinge.motion_limits.upper - radians(95.0)) < 1e-6
        and abs(hinge.origin.xyz[0] - PIVOT_X) < 1e-9
        and abs(hinge.origin.xyz[1] - HINGE_Y) < 1e-9
        and abs(hinge.origin.xyz[2] - PIVOT_Z) < 1e-9,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, origin={hinge.origin.xyz}, limits={hinge.motion_limits}",
    )
    all_part_names = tuple(part.name for part in object_model.parts)
    all_visual_names = tuple(visual.name for part in object_model.parts for visual in part.visuals)
    ctx.check(
        "cam over-center handle and cam barrel visuals are removed",
        "cam_lever" not in all_part_names
        and "lever_cam_pivot" not in tuple(art.name for art in object_model.articulations)
        and "fixed_cam_barrel" not in all_visual_names
        and "lever_handle" not in all_visual_names,
        details=f"parts={all_part_names}, visuals={all_visual_names}",
    )
    ctx.check(
        "adjuster nut spins continuously about the same bolt axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 1.0, 0.0)
        and abs(spin.origin.xyz[0] - hinge.origin.xyz[0]) < 1e-9
        and abs(spin.origin.xyz[2] - hinge.origin.xyz[2]) < 1e-9,
        details=f"type={spin.articulation_type}, axis={spin.axis}, origin={spin.origin.xyz}",
    )

    # Omega throat: the ring is genuinely open on the -X side. There must be no
    # collar band material spanning the throat near the bore mouth.
    band_aabb = ctx.part_element_world_aabb(collar, elem="collar_band")
    ctx.check(
        "collar band leaves an open Omega throat on the -X side",
        band_aabb is not None and band_aabb[0][0] <= -BORE_R + 1e-4,
        details=f"collar_band aabb={band_aabb}",
    )

    # Fixed bolt-head clevis hardware anchors the moving lever visibly at the
    # contact surface instead of relying on an invisible pad.
    bolt_head = ctx.part_element_world_aabb(collar, elem="bolt_head")
    ctx.check(
        "bolt head sits outside the cap-side foot and replaces the cam barrel",
        bolt_head is not None and bolt_head[0][1] >= LUG_Y_OUT + WASHER_LEN - 0.0003,
        details=f"bolt_head aabb={bolt_head}, cap-side foot outer y={LUG_Y_OUT}",
    )
    ctx.expect_within(
        collar,
        folding_lever,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="flat_lever",
        margin=0.001,
        name="lever eye is seated around the visible hinge pin",
    )
    ctx.expect_contact(
        collar,
        folding_lever,
        elem_a="hinge_pin",
        elem_b="flat_lever",
        contact_tol=0.0002,
        name="captured hinge pin bears on the lever eye",
    )
    ctx.expect_overlap(
        collar,
        folding_lever,
        axes="xz",
        elem_a="hinge_pin",
        elem_b="flat_lever",
        min_overlap=0.004,
        name="hinge pin passes through the folding lever eye",
    )
    for i in range(2):
        cheek = ctx.part_element_world_aabb(collar, elem=f"fork_cheek_{i}")
        ctx.check(
            f"fork cheek {i} is a visible bolt-head clevis cheek",
            cheek is not None
            and cheek[0][1] <= BOLT_Y_MAX + 0.001
            and cheek[1][1] >= HINGE_Y - 0.001
            and ((i == 0 and cheek[1][0] < PIVOT_X - LEVER_EYE_HALF_X + 0.0002) or (i == 1 and cheek[0][0] > PIVOT_X + LEVER_EYE_HALF_X - 0.0002)),
            details=f"fork_cheek_{i} aabb={cheek}, hinge_x={PIVOT_X}",
        )

    closed_pin = ctx.part_element_world_aabb(collar, elem="hinge_pin")
    with ctx.pose({hinge: LEVER_OPEN}):
        open_pin = ctx.part_element_world_aabb(collar, elem="hinge_pin")
        ctx.check(
            "opening the lever does not move the cross bolt or hinge pin hardware",
            closed_pin is not None and open_pin == closed_pin,
            details=f"closed_pin={closed_pin}, open_pin={open_pin}",
        )

    # Closed pose: the flat lever lies along the bolt axis and stays low/flush.
    closed = ctx.part_element_world_aabb(folding_lever, elem="flat_lever")
    ctx.check(
        "closed lever is a flat solid mesh with visible but slim thickness",
        closed is not None
        and (closed[1][2] - closed[0][2]) >= 2.0 * LEVER_Z_HALF - 0.0004
        and (closed[1][2] - closed[0][2]) <= 0.0125,
        details=f"closed lever z-extent={None if closed is None else closed[1][2]-closed[0][2]}",
    )
    ctx.check(
        "closed lever lies lengthwise along the bolt outside the cap-side foot",
        closed is not None and closed[0][1] > LUG_Y_OUT and closed[1][1] > HINGE_Y + 0.055,
        details=f"closed lever aabb={closed}, cap-side foot outer y={LUG_Y_OUT}",
    )
    ctx.check(
        "closed lever stays flush near the bolt height",
        closed is not None and closed[1][2] <= PIVOT_Z + LEVER_EYE_R + 0.001,
        details=f"closed lever aabb={closed}, bolt_z={PIVOT_Z}",
    )

    # Open pose: the lever folds upward to release, not around a vertical cam
    # barrel in plan view.
    with ctx.pose({hinge: LEVER_OPEN}):
        blade_open = ctx.part_element_world_aabb(folding_lever, elem="flat_lever")
        ctx.check(
            "fully open lever has hinged upward to release",
            blade_open is not None
            and closed is not None
            and blade_open[1][2] > closed[1][2] + 0.045,
            details=f"closed lever aabb={closed}, open lever aabb={blade_open}",
        )
        ctx.check(
            "fully open lever folds about X instead of sweeping around Z",
            blade_open is not None
            and closed is not None
            and blade_open[1][1] < closed[1][1] - 0.045
            and abs(blade_open[0][0] - closed[0][0]) < 0.001
            and abs(blade_open[1][0] - closed[1][0]) < 0.001,
            details=f"closed lever aabb={closed}, open lever aabb={blade_open}",
        )

    # Adjuster nut seats against the nut-side foot face, centered on the bolt.
    ctx.expect_gap(
        collar,
        adjuster_nut,
        axis="y",
        positive_elem="nut_side_thrust_washer",
        negative_elem="knurled_nut",
        min_gap=-0.001,
        max_gap=0.001,
        name="knurled nut seats against the nut-side thrust washer",
    )
    ctx.expect_overlap(
        adjuster_nut,
        collar,
        axes="xz",
        elem_a="knurled_nut",
        elem_b="cross_bolt",
        min_overlap=0.004,
        name="knurled nut is coaxial with the cross bolt",
    )
    with ctx.pose({spin: pi}):
        ctx.expect_gap(
            collar,
            adjuster_nut,
            axis="y",
            positive_elem="nut_side_thrust_washer",
            negative_elem="knurled_nut",
            min_gap=-0.001,
            max_gap=0.001,
            name="spinning the nut keeps it seated on the thrust washer",
        )

    return ctx.report()


object_model = build_object_model()
