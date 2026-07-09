from __future__ import annotations

from math import cos, pi, radians, sin

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
# Layout constants (meters). World frame: +Z is the collar bore axis (up), the
# clamp/lever end is on -X, and the new watchband barrel hinge sits on +X.  The
# collar is now two real half-arcs: the +Y cam arc is the root and the -Y nut arc
# is hinged to it about a visible vertical barrel hinge.  The cross bolt still
# clamps the -X lugs; the cam lever still rotates about its own vertical pivot,
# and the knurled adjuster still spins about the bolt axis.
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
HINGE_OPEN = radians(62.0)

HINGE_X = BAND_OUTER_R + 0.0040
HINGE_R = 0.0030
HINGE_PIN_R = 0.00105
HINGE_INNER_R = HINGE_PIN_R
HINGE_SPLIT_ANGLE = radians(13.0)
CLAMP_END_ANGLE = radians(151.5)
HINGE_ORIGIN = (HINGE_X, 0.0, 0.0)


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


def _arc_profile_points(
    side: int,
    inner_r: float,
    outer_r: float,
    *,
    samples: int = 54,
) -> list[tuple[float, float]]:
    """Polygon points for one annular half of the hinged collar.

    side=+1 is the cam/lever half (+Y); side=-1 is the nut half (-Y).  The
    endpoints leave a visible clamp slit at -X and a second seam at +X where the
    barrel hinge leaves attach.
    """
    if side > 0:
        start, end = HINGE_SPLIT_ANGLE, CLAMP_END_ANGLE
    else:
        start, end = -CLAMP_END_ANGLE, -HINGE_SPLIT_ANGLE

    outer = [
        (outer_r * cos(start + (end - start) * i / samples), outer_r * sin(start + (end - start) * i / samples))
        for i in range(samples + 1)
    ]
    inner = [
        (inner_r * cos(end + (start - end) * i / samples), inner_r * sin(end + (start - end) * i / samples))
        for i in range(samples + 1)
    ]
    return outer + inner


def _arc_band_solid(side: int, inner_r: float, outer_r: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").polyline(_arc_profile_points(side, inner_r, outer_r)).close().extrude(height)


def _collar_arc_solid(side: int) -> cq.Workplane:
    """One of the two hinged seat-clamp collar arcs, including the top counterbore."""
    arc = _arc_band_solid(side, BORE_R, BAND_OUTER_R, BAND_H)
    counterbore = _arc_band_solid(side, BORE_R, 0.0175, 0.007).translate((0.0, 0.0, BAND_H - 0.005))
    return arc.cut(counterbore)


def _lug_solid(yc: float) -> cq.Workplane:
    bolt_hole = (
        cq.Workplane("XZ")
        .center(PIVOT_X, PIVOT_Z)
        .circle(BOLT_R * 1.18)
        .extrude(LUG_T + 0.006, both=True)
        .translate((0.0, yc, 0.0))
    )
    lug = (
        cq.Workplane("XY")
        .box(LUG_LEN, LUG_T, BAND_H, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.0010)
        .edges(">Z or <Z")
        .fillet(0.00045)
    )
    return lug.translate((LUG_XC, yc, 0.5 * BAND_H)).cut(bolt_hole)


def _hinge_leaf_solid(side: int) -> cq.Workplane:
    """Flat forged hinge leaf tying one collar arc to the +X barrel knuckles."""
    y_inner = 0.0008 * side
    y_outer = 0.0060 * side
    leaf = (
        cq.Workplane("XY")
        .polyline(
            [
                (BAND_OUTER_R - 0.0010, y_inner),
                (HINGE_X, y_inner),
                (HINGE_X, y_outer),
                (BAND_OUTER_R - 0.0014, y_outer),
            ]
        )
        .close()
        .extrude(BAND_H)
    )
    return leaf.edges("|Z").fillet(0.00065)


def _vertical_tube_solid(inner_r: float, outer_r: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(height)


def _hinge_pin_solid() -> cq.Workplane:
    return cq.Workplane("XY").circle(HINGE_PIN_R).extrude(BAND_H + 0.0012).translate(
        (HINGE_X, 0.0, -0.0006)
    )


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

    # ------------------------------------------------------------ hinged collar
    cam_arc = model.part("cam_arc")
    nut_arc = model.part("nut_arc")

    def add_arc_visual(part, side: int, geometry: cq.Workplane, name: str, material) -> None:
        # The nut-side half is an articulated child whose local frame is at the
        # +X hinge axis, so its authored rest geometry is shifted back by the
        # hinge origin.  The root cam-side half remains in the object frame.
        origin = Origin(xyz=(-HINGE_X, 0.0, 0.0)) if side < 0 else Origin()
        part.visual(mesh_from_cadquery(geometry, name), origin=origin, material=material, name=name)

    for side, part, prefix, lug_y in ((1, cam_arc, "cam", LUG_YC), (-1, nut_arc, "nut", -LUG_YC)):
        add_arc_visual(part, side, _collar_arc_solid(side), f"{prefix}_collar_arc", alu_body)
        add_arc_visual(part, side, _lug_solid(lug_y), f"{prefix}_clamp_lug", alu_body)
        add_arc_visual(part, side, _hinge_leaf_solid(side), f"{prefix}_hinge_leaf", alu_body)
        for i, z in enumerate((0.0004, BAND_H - 0.0008)):
            add_arc_visual(
                part,
                side,
                _arc_band_solid(side, BORE_R + 0.0004, BAND_OUTER_R + 0.0003, 0.00045).translate(
                    (0.0, 0.0, z)
                ),
                f"{prefix}_machined_lip_{i}",
                brushed_light,
            )
        for i, z in enumerate((0.0043, 0.0107)):
            add_arc_visual(
                part,
                side,
                _arc_band_solid(side, BAND_OUTER_R + 0.00005, BAND_OUTER_R + 0.00025, 0.00018).translate(
                    (0.0, 0.0, z)
                ),
                f"{prefix}_brushed_groove_{i}",
                brushed_dark,
            )

    hinge_segment_h = 0.0046
    hinge_gap = 0.00035
    for i, z0 in enumerate((0.0, 2.0 * (hinge_segment_h + hinge_gap))):
        cam_arc.visual(
            mesh_from_cadquery(
                _vertical_tube_solid(HINGE_INNER_R, HINGE_R, hinge_segment_h).translate((HINGE_X, 0.0, z0)),
                f"cam_hinge_knuckle_{i}",
            ),
            origin=Origin(),
            material=alu_body,
            name=f"cam_hinge_knuckle_{i}",
        )
    nut_arc.visual(
        mesh_from_cadquery(
            _vertical_tube_solid(HINGE_INNER_R, HINGE_R, hinge_segment_h).translate(
                (HINGE_X, 0.0, hinge_segment_h + hinge_gap)
            ),
            "nut_hinge_knuckle",
        ),
        origin=Origin(xyz=(-HINGE_X, 0.0, 0.0)),
        material=alu_body,
        name="nut_hinge_knuckle",
    )
    cam_arc.visual(
        mesh_from_cadquery(_hinge_pin_solid(), "hinge_pin"),
        origin=Origin(),
        material=steel,
        name="hinge_pin",
    )

    cam_arc.visual(
        Cylinder(radius=BOLT_R, length=BOLT_Y_MAX - BOLT_Y_MIN),
        origin=Origin(
            xyz=(PIVOT_X, 0.5 * (BOLT_Y_MIN + BOLT_Y_MAX), PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)
        ),
        material=steel,
        name="cross_bolt",
    )
    cam_arc.visual(
        mesh_from_cadquery(_washer_solid(), "cam_side_thrust_washer"),
        origin=Origin(
            xyz=(PIVOT_X, LUG_Y_OUT + 0.5 * WASHER_LEN, PIVOT_Z), rpy=(pi / 2.0, 0.0, 0.0)
        ),
        material=brushed_light,
        name="cam_side_thrust_washer",
    )
    nut_arc.visual(
        mesh_from_cadquery(
            _annular_washer_solid(BOLT_R * 1.18, WASHER_R, WASHER_LEN), "nut_side_thrust_washer"
        ),
        origin=Origin(
            xyz=(PIVOT_X - HINGE_X, -LUG_Y_OUT - 0.5 * WASHER_LEN + 0.00105, PIVOT_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=brushed_light,
        name="nut_side_thrust_washer",
    )
    cam_arc.visual(
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
        "barrel_hinge",
        ArticulationType.REVOLUTE,
        parent=cam_arc,
        child=nut_arc,
        origin=Origin(xyz=HINGE_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=0.0, upper=HINGE_OPEN),
        motion_properties=MotionProperties(damping=0.3, friction=0.08),
    )
    model.articulation(
        "lever_cam_pivot",
        ArticulationType.REVOLUTE,
        parent=cam_arc,
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
        parent=nut_arc,
        child=adjuster_nut,
        origin=Origin(xyz=(PIVOT_X - HINGE_X, NUT_YC, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cam_arc = object_model.get_part("cam_arc")
    nut_arc = object_model.get_part("nut_arc")
    cam_lever = object_model.get_part("cam_lever")
    adjuster_nut = object_model.get_part("adjuster_nut")
    hinge = object_model.get_articulation("barrel_hinge")
    pivot = object_model.get_articulation("lever_cam_pivot")
    spin = object_model.get_articulation("adjuster_nut_spin")

    # Intentional captured-pin fit on the fixed clamp hardware.
    ctx.allow_overlap(
        cam_arc,
        adjuster_nut,
        elem_a="cross_bolt",
        elem_b="knurled_nut",
        reason="The bolt end threads into the adjuster nut bore (thread engagement proxy).",
    )

    # New requested watchband construction: two collar arcs joined by a real
    # visible barrel hinge at +X, while the -X lugs stay clamped by the existing
    # lever bolt hardware.
    ctx.check(
        "nut-side collar arc is a revolute child on the visible +X barrel hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge.axis) == (0.0, 0.0, 1.0)
        and hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(hinge.motion_limits.upper - HINGE_OPEN) < 1e-6,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge.motion_limits}",
    )
    ctx.expect_gap(
        cam_arc,
        nut_arc,
        axis="y",
        positive_elem="cam_collar_arc",
        negative_elem="nut_collar_arc",
        min_gap=0.001,
        name="collar band is split into separate cam-side and nut-side arcs",
    )
    ctx.expect_overlap(
        cam_arc,
        nut_arc,
        axes="xz",
        elem_a="cam_clamp_lug",
        elem_b="nut_clamp_lug",
        min_overlap=0.010,
        name="opposed -X clamp lugs remain aligned for the lever bolt",
    )
    ctx.expect_within(
        cam_arc,
        nut_arc,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="nut_hinge_knuckle",
        margin=0.0003,
        name="hinge pin runs through the nut-side barrel knuckle",
    )
    closed_nut_lug = ctx.part_element_world_aabb(nut_arc, elem="nut_clamp_lug")
    with ctx.pose({hinge: HINGE_OPEN}):
        open_nut_lug = ctx.part_element_world_aabb(nut_arc, elem="nut_clamp_lug")
        ctx.check(
            "positive barrel-hinge travel opens the nut arc like a watchband",
            closed_nut_lug is not None
            and open_nut_lug is not None
            and open_nut_lug[1][1] < closed_nut_lug[1][1] - 0.020,
            details=f"closed_nut_lug={closed_nut_lug}, open_nut_lug={open_nut_lug}",
        )
        ctx.expect_within(
            cam_arc,
            nut_arc,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="nut_hinge_knuckle",
            margin=0.0003,
            name="opened nut arc remains carried on the hinge pin",
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
        and abs((spin.origin.xyz[0] + HINGE_X) - pivot.origin.xyz[0]) < 1e-9
        and abs(spin.origin.xyz[2] - pivot.origin.xyz[2]) < 1e-9,
        details=f"type={spin.articulation_type}, axis={spin.axis}, origin={spin.origin.xyz}",
    )

    # The two arc meshes remain true hollow collar halves around the seatpost
    # bore, not a single solid block.
    cam_band_aabb = ctx.part_element_world_aabb(cam_arc, elem="cam_collar_arc")
    nut_band_aabb = ctx.part_element_world_aabb(nut_arc, elem="nut_collar_arc")
    ctx.check(
        "both hinged collar halves wrap the hollow 32 mm seatpost bore",
        cam_band_aabb is not None
        and nut_band_aabb is not None
        and cam_band_aabb[0][0] < -BORE_R
        and nut_band_aabb[0][0] < -BORE_R
        and cam_band_aabb[1][0] > BORE_R
        and nut_band_aabb[1][0] > BORE_R,
        details=f"cam_arc={cam_band_aabb}, nut_arc={nut_band_aabb}",
    )

    # The fixed cam barrel is now a short flush boss, not a protruding cylinder
    # that pokes through the moving handle.
    barrel_home = ctx.part_element_world_aabb(cam_arc, elem="fixed_cam_barrel")
    ctx.check(
        "shortened cam barrel does not protrude past the cap-side foot",
        barrel_home is not None and barrel_home[1][1] <= LUG_Y_OUT + 1e-6,
        details=f"fixed_cam_barrel aabb={barrel_home}, cap-side foot outer y={LUG_Y_OUT}",
    )
    ctx.expect_gap(
        cam_lever,
        cam_arc,
        axis="y",
        positive_elem="lever_handle",
        negative_elem="fixed_cam_barrel",
        min_gap=0.0001,
        max_gap=0.001,
        name="solid handle clears the shortened cam barrel",
    )
    ctx.expect_within(
        cam_arc,
        cam_arc,
        axes="xz",
        inner_elem="cross_bolt",
        outer_elem="fixed_cam_barrel",
        margin=0.0,
        name="cross bolt stays captured inside the fixed cam barrel cross-section",
    )

    closed_barrel = ctx.part_element_world_aabb(cam_arc, elem="fixed_cam_barrel")
    with ctx.pose({pivot: LEVER_OPEN}):
        open_barrel = ctx.part_element_world_aabb(cam_arc, elem="fixed_cam_barrel")
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
        nut_arc,
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
        cam_arc,
        axes="xz",
        elem_a="knurled_nut",
        elem_b="cross_bolt",
        min_overlap=0.004,
        name="knurled nut is coaxial with the cross bolt",
    )
    with ctx.pose({spin: pi}):
        ctx.expect_gap(
            nut_arc,
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
