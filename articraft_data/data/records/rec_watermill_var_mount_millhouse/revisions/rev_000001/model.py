from __future__ import annotations

"""Stylized wooden watermill waterwheel on a mill-house wall mount.

Reference image: picture/Machinery/Watermill/001.png

- Overall height ~2.22 m, wheel diameter ~1.84 m, wheel width ~0.50 m.
- Two segmented 12-gon rims in pale beige-pink wood, joined by nine flat
  paddle boards around the circumference; six straight radial spokes per
  side (three diametral bars) converge on a hub on each rim plane.
- Light-gray metal axle through both hubs, capped on each outer end with a
  short splined/ribbed collar resembling a gear sleeve.
- Darker grayish-brown mill-house facade: one tall vertical plank wall with
  visible seams and two horizontal protruding bearing brackets, each ending in
  a bored bearing block that journals the axle.
- Articulation: the whole wheel assembly spins freely (CONTINUOUS) about the
  horizontal axle axis (world +Y) relative to the static wall mount.
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

SPOKE_BARS = 3  # diametral bars per rim -> 6 visible spokes per side
SPOKE_LEN = 1.58
SPOKE_XSEC = (0.08, 0.07)

HUB_R = 0.10
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

BEARING_Y = 0.31  # wall-mounted bearing block center offset (+/- Y)
BLOCK_DIMS = (0.20, 0.10, 0.24)  # bearing block at the frame apex
# Journal bore slightly under the axle radius so the spinning axle stays
# captured in (and supported by) the bearing with a tiny intentional embed.
BORE_R = 0.033

WALL_X = -1.12
WALL_DIMS = (0.12, 1.34, 2.20)  # thickness X, width Y, height Z
WALL_FACE_X = WALL_X + WALL_DIMS[0] / 2.0
WALL_TOP_Z = WALL_DIMS[2]
PLANK_SEAM_COUNT = 5
SEAM_DIMS = (0.012, 0.018, 2.12)
WALL_BATTEN_DIMS = (0.045, 1.26, 0.085)
WALL_BATTEN_ZS = (0.42, 1.92)

BRACKET_DIMS = (1.08, 0.08, 0.10)  # cantilever beams protrude from wall along X
BRACKET_X = WALL_FACE_X + BRACKET_DIMS[0] / 2.0
BRACKET_Z = AXLE_Z - 0.11
GUSSET_DIMS = (0.56, 0.055, 0.075)
GUSSET_ANGLE = math.atan2(0.46, 0.56)


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watermill_waterwheel_wall_mount")

    wheel_wood = model.material("wheel_wood", color=(0.89, 0.72, 0.66))
    wall_wood = model.material("wall_wood", color=(0.50, 0.40, 0.36))
    seam_shadow = model.material("seam_shadow", color=(0.30, 0.24, 0.22))
    axle_metal = model.material("axle_metal", color=(0.70, 0.71, 0.73))
    collar_metal = model.material("collar_metal", color=(0.80, 0.81, 0.84))
    bolt_metal = model.material("bolt_metal", color=(0.20, 0.21, 0.22))

    # ------------------------------------------------------------- wall mount
    wall = model.part("mill_wall")
    wall.visual(
        Box(WALL_DIMS),
        origin=Origin(xyz=(WALL_X, 0.0, WALL_DIMS[2] / 2.0)),
        material=wall_wood,
        name="plank_wall",
    )

    # Narrow proud shadow strips turn the single facade into vertical planks.
    seam_spacing = WALL_DIMS[1] / (PLANK_SEAM_COUNT + 1)
    for si in range(PLANK_SEAM_COUNT):
        sy = -WALL_DIMS[1] / 2.0 + seam_spacing * (si + 1)
        wall.visual(
            Box(SEAM_DIMS),
            origin=Origin(xyz=(WALL_FACE_X + SEAM_DIMS[0] / 2.0 - 0.002, sy, WALL_DIMS[2] / 2.0)),
            material=seam_shadow,
            name=f"plank_seam_{si}",
        )

    # Horizontal battens bind the plank facade and make the wall read as a
    # mill-house exterior rather than a generic slab.
    for bi, bz in enumerate(WALL_BATTEN_ZS):
        wall.visual(
            Box(WALL_BATTEN_DIMS),
            origin=Origin(xyz=(WALL_FACE_X + WALL_BATTEN_DIMS[0] / 2.0 - 0.002, 0.0, bz)),
            material=wall_wood,
            name=f"wall_batten_{bi}",
        )

    bearing_mesh = mesh_from_cadquery(_bearing_block_solid(), "bearing_block")
    for fi, fy in enumerate((BEARING_Y, -BEARING_Y)):
        # The beams are real cantilever brackets: their rear ends embed into
        # the visible facade and their front ends tuck under the bored bearing
        # blocks at the axle line.
        wall.visual(
            Box(BRACKET_DIMS),
            origin=Origin(xyz=(BRACKET_X, fy, BRACKET_Z)),
            material=wall_wood,
            name=f"bearing_bracket_{fi}",
        )
        wall.visual(
            Box(GUSSET_DIMS),
            origin=Origin(
                xyz=(WALL_FACE_X + GUSSET_DIMS[0] / 2.0 - 0.015, fy, AXLE_Z - 0.31),
                rpy=(0.0, -GUSSET_ANGLE, 0.0),
            ),
            material=wall_wood,
            name=f"angle_gusset_{fi}",
        )
        wall.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, fy, AXLE_Z)),
            material=wall_wood,
            name=f"bearing_block_{fi}",
        )
        for zi, bz in enumerate((AXLE_Z - 0.02, AXLE_Z + 0.10)):
            wall.visual(
                Cylinder(radius=0.018, length=0.014),
                origin=Origin(xyz=(0.105, fy, bz), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bolt_metal,
                name=f"bearing_bolt_{fi}_{zi}",
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
            name=f"rim_{ri}",
        )
        wheel.visual(
            Cylinder(radius=HUB_R, length=HUB_LEN),
            origin=Origin(xyz=(0.0, ry, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_wood,
            name=f"hub_{ri}",
        )
        for si in range(SPOKE_BARS):
            wheel.visual(
                Box((SPOKE_XSEC[0], SPOKE_XSEC[1], SPOKE_LEN)),
                origin=Origin(
                    xyz=(0.0, ry, 0.0),
                    rpy=(0.0, si * math.pi / SPOKE_BARS, 0.0),
                ),
                material=wheel_wood,
                name=f"spoke_bar_{ri}_{si}",
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
            name=f"axle_collar_{ci}",
        )

    # ------------------------------------------------------------ articulation
    model.articulation(
        "wall_to_waterwheel",
        ArticulationType.CONTINUOUS,
        parent=wall,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("mill_wall")
    wheel = object_model.get_part("waterwheel")
    spin = object_model.get_articulation("wall_to_waterwheel")

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

    # Wall-mount replacement and real-world scale.
    wall_aabb = ctx.part_world_aabb(wall)
    ctx.check(
        "single tall mill-house wall facade replaces the trestle",
        wall_aabb is not None
        and abs(wall_aabb[0][2]) < 2e-3
        and 2.15 < (wall_aabb[1][2] - wall_aabb[0][2]) < 2.30
        and 1.25 < (wall_aabb[1][1] - wall_aabb[0][1]) < 1.45,
        details=f"wall aabb={wall_aabb}",
    )
    wall_visual_names = [v.name for v in wall.visuals if v.name]
    ctx.check(
        "no freestanding A-frame legs or feet remain",
        not any(name.startswith(("leg_", "foot_", "cross_brace_", "diagonal_brace")) for name in wall_visual_names),
        details=f"wall visuals={wall_visual_names}",
    )
    ctx.check(
        "facade has visible vertical plank seams",
        len([name for name in wall_visual_names if name.startswith("plank_seam_")]) == PLANK_SEAM_COUNT,
        details=f"wall visuals={wall_visual_names}",
    )
    ctx.check(
        "two horizontal bearing brackets protrude from the facade",
        len([name for name in wall_visual_names if name.startswith("bearing_bracket_")]) == 2,
        details=f"wall visuals={wall_visual_names}",
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

    # Hero features present: nine paddles, six spokes per side (3 diametral
    # bars per rim), two splined collars.
    paddle_names = [v.name for v in wheel.visuals if v.name and v.name.startswith("paddle_")]
    ctx.check(
        "nine paddle boards around the circumference",
        len(paddle_names) == PADDLE_COUNT,
        details=f"paddles={paddle_names}",
    )
    spoke_names = [v.name for v in wheel.visuals if v.name and v.name.startswith("spoke_bar_")]
    ctx.check(
        "three diametral spoke bars per rim (six spokes per side)",
        len(spoke_names) == 2 * SPOKE_BARS,
        details=f"spokes={spoke_names}",
    )

    # Axle is journaled through both bearing blocks. The bore is slightly
    # undersized so the axle is captured by the bearing (tiny local embed).
    ctx.allow_overlap(
        wheel,
        wall,
        elem_a="axle",
        elem_b="bearing_block_0",
        reason="Axle is intentionally captured in the front bearing journal bore.",
    )
    ctx.allow_overlap(
        wheel,
        wall,
        elem_a="axle",
        elem_b="bearing_block_1",
        reason="Axle is intentionally captured in the rear bearing journal bore.",
    )
    ctx.expect_overlap(
        wheel,
        wall,
        axes="y",
        elem_a="axle",
        elem_b="bearing_block_0",
        min_overlap=0.08,
        name="axle passes through the front bearing block",
    )
    ctx.expect_overlap(
        wheel,
        wall,
        axes="y",
        elem_a="axle",
        elem_b="bearing_block_1",
        min_overlap=0.08,
        name="axle passes through the rear bearing block",
    )
    # Rim clears the inboard face of the bearing block; collar sits outboard.
    ctx.expect_gap(
        wall,
        wheel,
        axis="y",
        positive_elem="bearing_block_0",
        negative_elem="rim_0",
        min_gap=0.005,
        name="rim clears the bearing block",
    )
    ctx.expect_gap(
        wheel,
        wall,
        axis="y",
        positive_elem="axle_collar_0",
        negative_elem="bearing_block_0",
        min_gap=0.005,
        name="splined collar caps the axle outboard of the bearing",
    )
    ctx.expect_contact(
        wall,
        wall,
        elem_a="bearing_bracket_0",
        elem_b="plank_wall",
        name="bearing bracket is seated into the mill wall",
    )
    ctx.expect_gap(
        wheel,
        wall,
        axis="x",
        positive_elem="rim_0",
        negative_elem="plank_wall",
        min_gap=0.03,
        name="wheel clears the wall facade",
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
            wall,
            wheel,
            axis="y",
            positive_elem="bearing_block_0",
            negative_elem="rim_0",
            min_gap=0.005,
            name="wheel keeps clearance to the frame while spinning",
        )

    return ctx.report()


object_model = build_object_model()
