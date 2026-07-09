from __future__ import annotations

"""Classic tilt-head kitchen stand mixer in glossy apple green.

Reference image: picture/Kitchen/mixer/001.png

Layout (world frame, meters):
- +X is forward (toward the bowl and the head nose), +Z is up, +Y is left.
- The green pedestal base sits on the floor (z=0); a C-shaped column rises at
  the rear and carries the tilt-head hinge at (-0.105, 0, 0.275).
- The motor head cantilevers forward over the white ceramic bowl that locks
  onto a chromed circular base plate centered at x=0.08.

Articulations:
1. column_to_head_tilt  REVOLUTE  axis (0,-1,0), 0..50 deg, head tilts up/back.
2. head_to_whisk_spin   CONTINUOUS vertical spindle under the head nose.
3. head_to_speed_lever  PRISMATIC  0..0.04 m front-to-back on the head's right.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Key world dimensions
# ---------------------------------------------------------------------------
PIVOT_X = -0.105
PIVOT_Z = 0.275
BOWL_X = 0.08
BASE_TOP_Z = 0.05
BOWL_SEAT_Z = 0.0565
HEAD_TILT_MAX = math.radians(50.0)
LEVER_TRAVEL = 0.04
WHISK_LOCAL = (0.185, 0.0, -0.044)  # spindle joint in head frame


def _yz_section(
    x: float, w: float, h: float, cz: float
) -> list[tuple[float, float, float]]:
    """Rounded-rect cross-section in the YZ plane at station x (loft along X)."""
    r = 0.36 * min(w, h)
    return [(x, y, z + cz) for z, y in rounded_rect_profile(h, w, r)]


def _xy_section(
    z: float, cx: float, depth: float, width: float
) -> list[tuple[float, float, float]]:
    """Rounded-rect cross-section in the XY plane at height z (loft along Z)."""
    r = 0.30 * min(depth, width)
    return [(cx + px, py, z) for px, py in rounded_rect_profile(depth, width, r)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_stand_mixer")

    apple_green = model.material("apple_green", rgba=(0.58, 0.72, 0.20, 1.0))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    white_trim = model.material("white_trim", rgba=(0.95, 0.95, 0.92, 1.0))
    ceramic_white = model.material("ceramic_white", rgba=(0.94, 0.93, 0.89, 1.0))
    steel_wire = model.material("steel_wire", rgba=(0.74, 0.75, 0.78, 1.0))

    # ------------------------------------------------------------------
    # Pedestal: green base slab + rear column (C-form) + chrome bowl plate
    # + hinge ears and hinge barrel at the column top.
    # ------------------------------------------------------------------
    pedestal = model.part("pedestal")

    base_geom = ExtrudeGeometry.from_z0(rounded_rect_profile(0.34, 0.20, 0.06), 0.05)
    pedestal.visual(
        mesh_from_geometry(base_geom, "base_foot"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=apple_green,
        name="base_foot",
    )

    pedestal.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(xyz=(BOWL_X, 0.0, 0.052)),
        material=chrome,
        name="bowl_plate",
    )

    column_geom = section_loft(
        [
            _xy_section(0.000, -0.097, 0.136, 0.155),
            _xy_section(0.060, -0.103, 0.124, 0.142),
            _xy_section(0.125, -0.106, 0.112, 0.132),
            _xy_section(0.188, -0.106, 0.108, 0.128),
        ]
    )
    pedestal.visual(
        mesh_from_geometry(column_geom, "rear_column"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=apple_green,
        name="rear_column",
    )

    for side, sy in (("left", 1.0), ("right", -1.0)):
        pedestal.visual(
            Box((0.050, 0.013, 0.080)),
            origin=Origin(xyz=(PIVOT_X, sy * 0.062, 0.247)),
            material=apple_green,
            name=f"{side}_hinge_ear",
        )

    pedestal.visual(
        Cylinder(radius=0.015, length=0.140),
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )

    # ------------------------------------------------------------------
    # Bowl: hollow lathed white ceramic shell with a loop handle, locked
    # onto the chrome base plate (FIXED).
    # ------------------------------------------------------------------
    bowl = model.part("bowl")
    bowl_outer = [
        (0.048, 0.000),
        (0.086, 0.006),
        (0.104, 0.032),
        (0.113, 0.072),
        (0.117, 0.118),
        (0.119, 0.152),
        (0.1225, 0.166),
        (0.1225, 0.170),
    ]
    bowl_inner = [
        (0.000, 0.007),
        (0.080, 0.012),
        (0.098, 0.034),
        (0.107, 0.072),
        (0.111, 0.118),
        (0.113, 0.152),
        (0.1165, 0.170),
    ]
    bowl_geom = LatheGeometry.from_shell_profiles(bowl_outer, bowl_inner, segments=64)
    bowl.visual(
        mesh_from_geometry(bowl_geom, "bowl_shell"),
        material=ceramic_white,
        name="bowl_shell",
    )

    handle_geom = tube_from_spline_points(
        [
            (0.0, 0.106, 0.155),
            (0.0, 0.150, 0.150),
            (0.0, 0.174, 0.120),
            (0.0, 0.177, 0.085),
            (0.0, 0.160, 0.052),
            (0.0, 0.128, 0.038),
            (0.0, 0.098, 0.042),
        ],
        radius=0.012,
        radial_segments=20,
    )
    bowl.visual(
        mesh_from_geometry(handle_geom, "bowl_handle"),
        material=ceramic_white,
        name="bowl_handle",
    )

    model.articulation(
        "pedestal_to_bowl",
        ArticulationType.FIXED,
        parent=pedestal,
        child=bowl,
        origin=Origin(xyz=(BOWL_X, 0.0, BOWL_SEAT_Z)),
    )

    # ------------------------------------------------------------------
    # Motor head: lofted green housing, chrome nose band with white inset
    # stripe, chrome hub cap on the nose, attachment pin on top, spindle
    # boss underneath, and the speed-lever track on the right side.
    # ------------------------------------------------------------------
    head = model.part("motor_head")

    head_geom = section_loft(
        [
            _yz_section(-0.022, 0.072, 0.082, 0.012),
            _yz_section(0.000, 0.100, 0.112, 0.022),
            _yz_section(0.060, 0.112, 0.122, 0.024),
            _yz_section(0.150, 0.114, 0.124, 0.023),
            _yz_section(0.205, 0.108, 0.116, 0.018),
            _yz_section(0.245, 0.086, 0.094, 0.008),
            _yz_section(0.272, 0.046, 0.052, 0.000),
        ]
    )
    head.visual(
        mesh_from_geometry(head_geom, "head_shell"),
        material=apple_green,
        name="head_shell",
    )

    band_geom = section_loft(
        [
            _yz_section(0.228, 0.100, 0.108, 0.012),
            _yz_section(0.260, 0.070, 0.077, 0.004),
        ]
    )
    head.visual(
        mesh_from_geometry(band_geom, "nose_band_chrome"),
        material=chrome,
        name="nose_band_chrome",
    )

    stripe_geom = section_loft(
        [
            _yz_section(0.238, 0.0955, 0.1035, 0.0100),
            _yz_section(0.252, 0.0825, 0.0905, 0.0072),
        ]
    )
    head.visual(
        mesh_from_geometry(stripe_geom, "nose_band_white"),
        material=white_trim,
        name="nose_band_white",
    )

    head.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.272, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hub_cap",
    )

    head.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(0.040, 0.0, 0.083)),
        material=chrome,
        name="attachment_pin",
    )

    head.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(WHISK_LOCAL[0], 0.0, -0.042)),
        material=apple_green,
        name="spindle_boss",
    )

    head.visual(
        Box((0.075, 0.005, 0.026)),
        origin=Origin(xyz=(0.100, -0.056, 0.022)),
        material=chrome,
        name="speed_track",
    )

    model.articulation(
        "column_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=head,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        # The head extends along local +X from the rear pivot; -Y makes
        # positive q tilt the nose up and back, clearing the bowl.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0, velocity=1.5, lower=0.0, upper=HEAD_TILT_MAX
        ),
    )

    # ------------------------------------------------------------------
    # Whisk spindle: chrome collar seated in the head boss, tapered shaft,
    # ferrule, and a balloon cage of bent stainless wires.
    # ------------------------------------------------------------------
    whisk = model.part("whisk_spindle")

    whisk.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=chrome,
        name="spindle_collar",
    )

    cage_geom = CylinderGeometry(radius=0.006, height=0.036).translate(0.0, 0.0, -0.034)
    cage_geom.merge(
        CylinderGeometry(radius=0.011, height=0.020).translate(0.0, 0.0, -0.062)
    )
    n_loops = 6
    for i in range(n_loops):
        ang = i * math.pi / n_loops
        c, s = math.cos(ang), math.sin(ang)
        pts = [
            (0.009 * c, 0.009 * s, -0.064),
            (0.027 * c, 0.027 * s, -0.085),
            (0.044 * c, 0.044 * s, -0.106),
            (0.040 * c, 0.040 * s, -0.124),
            (0.0, 0.0, -0.140),
            (-0.040 * c, -0.040 * s, -0.124),
            (-0.044 * c, -0.044 * s, -0.106),
            (-0.027 * c, -0.027 * s, -0.085),
            (-0.009 * c, -0.009 * s, -0.064),
        ]
        cage_geom.merge(tube_from_spline_points(pts, radius=0.0016))
    whisk.visual(
        mesh_from_geometry(cage_geom, "whisk_cage"),
        material=steel_wire,
        name="whisk_cage",
    )

    model.articulation(
        "head_to_whisk_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=WHISK_LOCAL),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    # ------------------------------------------------------------------
    # Speed control lever: small chrome slider riding the track on the
    # head's right side, prismatic front-to-back travel.
    # ------------------------------------------------------------------
    lever = model.part("speed_lever")
    lever.visual(
        Box((0.012, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, -0.0025, 0.0)),
        material=chrome,
        name="lever_stem",
    )
    lever.visual(
        Box((0.020, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.0095, 0.0)),
        material=chrome,
        name="lever_grip",
    )

    model.articulation(
        "head_to_speed_lever",
        ArticulationType.PRISMATIC,
        parent=head,
        child=lever,
        origin=Origin(xyz=(0.082, -0.0585, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=0.1, lower=0.0, upper=LEVER_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("motor_head")
    whisk = object_model.get_part("whisk_spindle")
    lever = object_model.get_part("speed_lever")
    tilt = object_model.get_articulation("column_to_head_tilt")
    spin = object_model.get_articulation("head_to_whisk_spin")
    slide = object_model.get_articulation("head_to_speed_lever")

    # Intentional local embeddings that represent real mechanical seats.
    ctx.allow_overlap(
        pedestal,
        head,
        elem_a="hinge_barrel",
        elem_b="head_shell",
        reason="Chrome hinge pin is captured inside the head's rear hinge bore.",
    )
    ctx.allow_overlap(
        head,
        whisk,
        elem_a="spindle_boss",
        elem_b="spindle_collar",
        reason="Whisk collar seats up into the drive boss under the head nose.",
    )
    ctx.allow_overlap(
        head,
        lever,
        elem_a="speed_track",
        elem_b="lever_stem",
        reason="Speed lever stem rides in the slider track slot.",
    )
    ctx.allow_overlap(
        pedestal,
        bowl,
        elem_a="bowl_plate",
        elem_b="bowl_shell",
        reason="Bowl foot locks 1.5 mm down onto the chromed base plate.",
    )

    # --- grounding and true scale -------------------------------------
    ped_aabb = ctx.part_world_aabb(pedestal)
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "base grounded at z=0",
        ped_aabb is not None and abs(ped_aabb[0][2]) < 1e-6,
        details=f"pedestal aabb={ped_aabb}",
    )
    ctx.check(
        "overall height about 0.36 m",
        head_aabb is not None and 0.34 <= head_aabb[1][2] <= 0.375,
        details=f"head aabb={head_aabb}",
    )

    bowl_aabb = ctx.part_element_world_aabb(bowl, elem="bowl_shell")
    ctx.check(
        "bowl diameter about 0.24 m",
        bowl_aabb is not None and 0.225 <= (bowl_aabb[1][1] - bowl_aabb[0][1]) <= 0.26,
        details=f"bowl shell aabb={bowl_aabb}",
    )
    ctx.check(
        "bowl about 0.18 m tall",
        bowl_aabb is not None and 0.15 <= (bowl_aabb[1][2] - bowl_aabb[0][2]) <= 0.19,
        details=f"bowl shell aabb={bowl_aabb}",
    )

    # --- bowl seated on the chrome plate, handle sticks out ------------
    ctx.expect_gap(
        bowl,
        pedestal,
        axis="z",
        positive_elem="bowl_shell",
        negative_elem="bowl_plate",
        max_penetration=0.003,
        max_gap=0.0005,
        name="bowl foot seats on the chrome base plate",
    )
    ctx.expect_overlap(
        bowl,
        pedestal,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="bowl_plate",
        min_overlap=0.10,
        name="bowl is centered over the base plate",
    )
    handle_aabb = ctx.part_element_world_aabb(bowl, elem="bowl_handle")
    ctx.check(
        "loop handle protrudes from the bowl side",
        handle_aabb is not None
        and bowl_aabb is not None
        and handle_aabb[1][1] > bowl_aabb[1][1] + 0.03,
        details=f"handle aabb={handle_aabb}",
    )

    # --- closed head clears the bowl rim --------------------------------
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        positive_elem="head_shell",
        negative_elem="bowl_shell",
        min_gap=0.003,
        max_gap=0.03,
        name="closed head nose hovers just above the bowl rim",
    )

    # --- whisk hangs centered inside the hollow bowl --------------------
    ctx.expect_within(
        whisk,
        bowl,
        axes="xy",
        inner_elem="whisk_cage",
        outer_elem="bowl_shell",
        margin=0.0,
        name="balloon whisk stays inside the bowl footprint",
    )
    cage_aabb = ctx.part_element_world_aabb(whisk, elem="whisk_cage")
    ctx.check(
        "whisk cage clears the bowl floor",
        cage_aabb is not None and cage_aabb[0][2] > 0.072,
        details=f"cage aabb={cage_aabb}",
    )
    ctx.check(
        "whisk wires balloon well off the spindle axis",
        cage_aabb is not None and (cage_aabb[1][0] - cage_aabb[0][0]) > 0.07,
        details=f"cage aabb={cage_aabb}",
    )

    # --- chrome nose details ---------------------------------------------
    shell_aabb = ctx.part_element_world_aabb(head, elem="head_shell")
    cap_aabb = ctx.part_element_world_aabb(head, elem="hub_cap")
    ctx.check(
        "hub cap protrudes from the front nose",
        cap_aabb is not None
        and shell_aabb is not None
        and cap_aabb[1][0] > shell_aabb[1][0] + 0.004,
        details=f"hub cap aabb={cap_aabb}",
    )
    band_aabb = ctx.part_element_world_aabb(head, elem="nose_band_chrome")
    ctx.check(
        "chrome band wraps the head nose region",
        band_aabb is not None and 0.10 <= 0.5 * (band_aabb[0][0] + band_aabb[1][0]) <= 0.17,
        details=f"band aabb={band_aabb}",
    )

    # --- joint plan metadata ----------------------------------------------
    ctx.check(
        "head tilt is revolute 0..~50 deg",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and abs(tilt.motion_limits.lower) < 1e-9
        and 0.80 <= tilt.motion_limits.upper <= 0.95,
        details=f"tilt limits={tilt.motion_limits}",
    )
    ctx.check(
        "whisk spindle is a continuous rotary joint",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details=f"spin limits={spin.motion_limits}",
    )
    ctx.check(
        "speed lever is prismatic with 0.04 m travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and abs(slide.motion_limits.upper - 0.04) < 1e-9,
        details=f"slide limits={slide.motion_limits}",
    )

    # --- decisive pose checks ----------------------------------------------
    whisk_rest = ctx.part_world_position(whisk)
    with ctx.pose({tilt: HEAD_TILT_MAX}):
        whisk_up = ctx.part_world_position(whisk)
        ctx.check(
            "tilting the head lifts the whisk",
            whisk_rest is not None
            and whisk_up is not None
            and whisk_up[2] > whisk_rest[2] + 0.10,
            details=f"rest={whisk_rest}, tilted={whisk_up}",
        )
        open_cage = ctx.part_element_world_aabb(whisk, elem="whisk_cage")
        ctx.check(
            "tilted whisk swings clear above the bowl rim",
            open_cage is not None and open_cage[0][2] > 0.24,
            details=f"open cage aabb={open_cage}",
        )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_within(
            whisk,
            bowl,
            axes="xy",
            inner_elem="whisk_cage",
            outer_elem="bowl_shell",
            margin=0.0,
            name="spinning whisk stays inside the bowl",
        )

    lever_rest = ctx.part_world_position(lever)
    with ctx.pose({slide: LEVER_TRAVEL}):
        lever_fwd = ctx.part_world_position(lever)
        ctx.check(
            "speed lever slides 0.04 m toward the nose",
            lever_rest is not None
            and lever_fwd is not None
            and abs((lever_fwd[0] - lever_rest[0]) - LEVER_TRAVEL) < 1e-6
            and abs(lever_fwd[1] - lever_rest[1]) < 1e-9,
            details=f"rest={lever_rest}, slid={lever_fwd}",
        )

    return ctx.report()


object_model = build_object_model()
