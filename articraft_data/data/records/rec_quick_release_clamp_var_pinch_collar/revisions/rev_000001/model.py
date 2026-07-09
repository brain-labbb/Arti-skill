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
# the pinch slit is a thin radial saw-cut on -X, and the cross bolt runs along
# Y through the two boss lugs straddling that slit. The quick-release handle
# lives outside the +Y lug. The bolt and cam barrel are fixed hardware; the
# handle itself is the articulated solid mesh and turns about a local vertical
# Z-axis at the side pivot.
# ---------------------------------------------------------------------------
BORE_R = 0.016  # inner bore radius (0.032 m diameter per prompt)
BAND_OUTER_R = 0.021  # bore + 0.005 wall
BAND_H = 0.015  # band height per prompt

# Full-circle pinch collar: only a thin saw-cut crosses the wall on the -X side.
# The bore remains a round through-bore rather than opening into an Omega mouth.
PINCH_SLIT_HALF_W = 0.0006
PINCH_SLIT_X_IN = -BORE_R - 0.00025
PINCH_SLIT_X_OUT = -BAND_OUTER_R - 0.0020

LUG_X_MIN, LUG_X_MAX = -0.038, -0.016  # flat pinch-boss lugs flanking the slit
LUG_Y_IN, LUG_Y_OUT = 0.0008, 0.0100  # boss inner/outer faces in |y|
LUG_LEN = LUG_X_MAX - LUG_X_MIN
LUG_XC = 0.5 * (LUG_X_MIN + LUG_X_MAX)
LUG_T = LUG_Y_OUT - LUG_Y_IN
LUG_YC = 0.5 * (LUG_Y_IN + LUG_Y_OUT)

PIVOT_X = -0.0305  # cross-bolt axis
PIVOT_Z = 0.5 * BAND_H

BOLT_R = 0.0025
BOLT_Y_MIN, BOLT_Y_MAX = -0.0225, 0.0250  # spans nut, feet, and the external cam

WASHER_R = 0.0064
WASHER_LEN = 0.0016

BARREL_R = 0.0057  # external cam barrel outside the +Y foot
BARREL_LEN = 0.0110
VISIBLE_BARREL_LEN = 0.0010
LEVER_YC = LUG_Y_OUT + WASHER_LEN + 0.0004 + 0.5 * BARREL_LEN

# Lever handle lives in the local X-Y plane and is thickened along local Z, so
# it is a single solid mesh rather than a thin open ribbon.
HANDLE_Z_HALF = 0.0030  # half thickness of the handle mesh (along Z)

NUT_D = 0.022  # knurled adjuster barrel nut per prompt
NUT_LEN = 0.020
NUT_YC = -LUG_Y_OUT - WASHER_LEN - 0.0004 - 0.5 * NUT_LEN  # outside the -Y foot face

LEVER_OPEN = radians(170.0)


def _pinch_slit_solid() -> cq.Workplane:
    """Thin radial saw-cut through the -X wall of the pinch collar."""
    return (
        cq.Workplane("XY")
        .rect(abs(PINCH_SLIT_X_OUT - PINCH_SLIT_X_IN), 2.0 * PINCH_SLIT_HALF_W)
        .extrude(BAND_H + 0.012)
        .translate((0.5 * (PINCH_SLIT_X_IN + PINCH_SLIT_X_OUT), 0.0, -0.006))
    )


def _collar_band_solid() -> cq.Workplane:
    """Full-circle split-ring band with only one thin -X radial pinch slit.

    The circular seat-post bore stays round and continuous; unlike the parent
    Omega mouth, this cut is only a narrow saw slit through the wall.
    """
    band = cq.Workplane("XY").circle(BAND_OUTER_R).circle(BORE_R).extrude(BAND_H)
    # Shallow counterbore step at the top opening (visible seat lip).
    counterbore = (
        cq.Workplane("XY", origin=(0.0, 0.0, BAND_H - 0.005)).circle(0.0175).extrude(0.007)
    )
    band = band.cut(counterbore)
    # Thin radial split on the -X side only.
    return band.cut(_pinch_slit_solid())


def _annular_ring_solid(inner_r: float, outer_r: float, height: float) -> cq.Workplane:
    """Annular trim ring with the same thin radial slit as the collar band."""
    ring = cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(height)
    return ring.cut(_pinch_slit_solid())


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


def _lever_handle_solid() -> cq.Workplane:
    """One-piece solid quick-release handle in the lever-local X-Y plane."""
    blade = (
        cq.Workplane("XY")
        # outer edge: root -> swept tip
        .moveTo(-0.0085, -0.0040)
        .spline(
            [
                (-0.0020, 0.0070),
                (0.0110, 0.0185),
                (0.0270, 0.0300),
                (0.0450, 0.0360),
                (0.0585, 0.0345),
            ],
            includeCurrent=True,
        )
        # rounded tip + inner edge back to the root
        .spline(
            [
                (0.0560, 0.0270),
                (0.0400, 0.0200),
                (0.0240, 0.0120),
                (0.0100, 0.0050),
                (0.0010, 0.0008),
                (-0.0070, 0.0005),
            ],
            includeCurrent=True,
        )
        .close()
        .extrude(HANDLE_Z_HALF, both=True)
    )
    hub = cq.Workplane("XY").circle(0.0072).extrude(HANDLE_Z_HALF + 0.0006, both=True)
    return hub.union(blade)


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
        Cylinder(radius=BARREL_R, length=VISIBLE_BARREL_LEN),
        origin=Origin(
            xyz=(PIVOT_X, LUG_Y_OUT - 0.5 * VISIBLE_BARREL_LEN, PIVOT_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=alu_lever,
        name="fixed_cam_barrel",
    )

    # --------------------------------------------------------------- cam lever
    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        mesh_from_cadquery(_lever_handle_solid(), "lever_handle"),
        origin=Origin(),
        material=alu_lever,
        name="lever_handle",
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
        "lever_cam_pivot",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=cam_lever,
        origin=Origin(xyz=(PIVOT_X, LEVER_YC, PIVOT_Z)),
        # About +Z: positive q swings the solid side handle around its vertical pivot.
        axis=(0.0, 0.0, 1.0),
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
    cam_lever = object_model.get_part("cam_lever")
    adjuster_nut = object_model.get_part("adjuster_nut")
    pivot = object_model.get_articulation("lever_cam_pivot")
    spin = object_model.get_articulation("adjuster_nut_spin")

    # Intentional captured-pin fit on the fixed clamp hardware.
    ctx.allow_overlap(
        collar,
        adjuster_nut,
        elem_a="cross_bolt",
        elem_b="knurled_nut",
        reason="The bolt end threads into the adjuster nut bore (thread engagement proxy).",
    )

    # Joint semantics match the corrected handle behavior.
    ctx.check(
        "cam lever is a revolute joint about the vertical Z axis with ~170 deg travel",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(pivot.axis) == (0.0, 0.0, 1.0)
        and pivot.motion_limits is not None
        and abs(pivot.motion_limits.lower - 0.0) < 1e-9
        and abs(pivot.motion_limits.upper - radians(170.0)) < 1e-6,
        details=f"type={pivot.articulation_type}, axis={pivot.axis}, limits={pivot.motion_limits}",
    )
    ctx.check(
        "adjuster nut spins continuously about the same bolt axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 1.0, 0.0)
        and abs(spin.origin.xyz[0] - pivot.origin.xyz[0]) < 1e-9
        and abs(spin.origin.xyz[2] - pivot.origin.xyz[2]) < 1e-9,
        details=f"type={spin.articulation_type}, axis={spin.axis}, origin={spin.origin.xyz}",
    )

    # Revised pinch-collar body: the -X split is now a single thin radial slit,
    # not a broad Omega mouth. The slit stops outside the round bore wall so the
    # through-bore never breaks out into an open throat.
    band_aabb = ctx.part_element_world_aabb(collar, elem="collar_band")
    ctx.check(
        "collar band remains a full-circle annulus around the bore",
        band_aabb is not None
        and band_aabb[0][0] <= -BAND_OUTER_R + 0.001
        and band_aabb[1][0] >= BAND_OUTER_R - 0.001,
        details=f"collar_band aabb={band_aabb}",
    )
    ctx.expect_gap(
        collar,
        collar,
        axis="y",
        positive_elem="lug_cap_side",
        negative_elem="lug_nut_side",
        min_gap=2.0 * PINCH_SLIT_HALF_W,
        max_gap=0.0025,
        name="pinch boss pair straddles one thin radial slit",
    )
    ctx.check(
        "radial slit is a narrow saw cut outside the round bore, not an Omega mouth",
        2.0 * PINCH_SLIT_HALF_W <= 0.0015
        and PINCH_SLIT_X_OUT < -BAND_OUTER_R
        and PINCH_SLIT_X_IN < -BORE_R,
        details=(
            f"slit_half_width={PINCH_SLIT_HALF_W}, x_in={PINCH_SLIT_X_IN}, "
            f"x_out={PINCH_SLIT_X_OUT}, bore_r={BORE_R}, outer_r={BAND_OUTER_R}"
        ),
    )
    band_volume = _collar_band_solid().val().Volume()
    full_annulus_volume = pi * (BAND_OUTER_R**2 - BORE_R**2) * BAND_H
    ctx.check(
        "collar retains nearly full annular wall volume after replacing the Omega mouth",
        band_volume > 0.78 * full_annulus_volume,
        details=f"band_volume={band_volume}, full_annulus_volume={full_annulus_volume}",
    )

    # The fixed cam barrel is now a short flush boss, not a protruding cylinder
    # that pokes through the moving handle.
    barrel_home = ctx.part_element_world_aabb(collar, elem="fixed_cam_barrel")
    ctx.check(
        "shortened cam barrel does not protrude past the cap-side foot",
        barrel_home is not None and barrel_home[1][1] <= LUG_Y_OUT + 1e-6,
        details=f"fixed_cam_barrel aabb={barrel_home}, cap-side foot outer y={LUG_Y_OUT}",
    )
    ctx.expect_gap(
        cam_lever,
        collar,
        axis="y",
        positive_elem="lever_handle",
        negative_elem="fixed_cam_barrel",
        min_gap=0.0001,
        max_gap=0.001,
        name="solid handle clears the shortened cam barrel",
    )
    ctx.expect_within(
        collar,
        collar,
        axes="xz",
        inner_elem="cross_bolt",
        outer_elem="fixed_cam_barrel",
        margin=0.0,
        name="cross bolt stays captured inside the fixed cam barrel cross-section",
    )

    closed_barrel = ctx.part_element_world_aabb(collar, elem="fixed_cam_barrel")
    with ctx.pose({pivot: LEVER_OPEN}):
        open_barrel = ctx.part_element_world_aabb(collar, elem="fixed_cam_barrel")
        ctx.check(
            "opening the handle does not move the middle rod or cam barrel",
            closed_barrel is not None and open_barrel == closed_barrel,
            details=f"closed_barrel={closed_barrel}, open_barrel={open_barrel}",
        )

    # Closed pose: the handle is a one-piece solid mesh that sits outside the
    # cap-side foot and sweeps back over the collar.
    closed = ctx.part_element_world_aabb(cam_lever, elem="lever_handle")
    ctx.check(
        "closed handle is a solid mesh with visible Z thickness",
        closed is not None and (closed[1][2] - closed[0][2]) >= 2.0 * HANDLE_Z_HALF - 0.0004,
        details=f"closed handle z-extent={None if closed is None else closed[1][2]-closed[0][2]}",
    )
    ctx.check(
        "closed handle sits outside the cap-side foot, not in the throat",
        closed is not None and closed[0][1] > LUG_Y_OUT,
        details=f"closed handle aabb={closed}, cap-side foot outer y={LUG_Y_OUT}",
    )
    ctx.check(
        "closed handle sweeps back over the collar toward +X",
        closed is not None and closed[1][0] > 0.020,
        details=f"closed handle aabb={closed}",
    )

    # Open pose: the handle rotates around Z in plan view while staying on the
    # outside of the clamp.
    with ctx.pose({pivot: LEVER_OPEN}):
        blade_open = ctx.part_element_world_aabb(cam_lever, elem="lever_handle")
        ctx.check(
            "fully open handle has swung around the Z-axis pivot",
            blade_open is not None and blade_open[1][0] < PIVOT_X + 0.010,
            details=f"open handle aabb={blade_open}, pivot x={PIVOT_X}",
        )
        ctx.check(
            "fully open handle rotates in plan view without pitching about the middle rod",
            blade_open is not None
            and closed is not None
            and abs(blade_open[0][2] - closed[0][2]) < 1e-9
            and abs(blade_open[1][2] - closed[1][2]) < 1e-9,
            details=f"closed handle aabb={closed}, open handle aabb={blade_open}",
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
