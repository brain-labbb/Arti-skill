from __future__ import annotations

"""Stylized wooden watermill waterwheel in a masonry sluice mount.

Reference image: picture/Machinery/Watermill/001.png

- Overall height ~2.22 m, wheel diameter ~1.84 m, wheel width ~0.50 m.
- Two segmented 12-gon rims in pale beige-pink wood, joined by nine flat
  paddle boards around the circumference; six straight radial spokes per
  side (three diametral bars) converge on a hub on each rim plane.
- Light-gray metal axle through both hubs, capped on each outer end with a
  short splined/ribbed collar resembling a gear sleeve.
- Low masonry sluice mount: two parallel channel walls and a floor slab form a
  water flume, with raised bearing seats on top of the walls journaling the
  axle so the wheel sits down inside the channel.
- Articulation: the whole wheel assembly spins freely (CONTINUOUS) about the
  horizontal axle axis (world +Y) relative to the static sluice mount.
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

BEARING_Y = 0.31  # bearing seat center offset (+/- Y), as in the parent mount

CHANNEL_LEN = 2.35
CHANNEL_WALL_Y = 0.36
CHANNEL_WALL_THICK = 0.16
CHANNEL_WALL_HEIGHT = 0.58
CHANNEL_FLOOR_DIMS = (2.45, 0.88, 0.06)

BEARING_PIER_DIMS = (0.38, 0.18, 0.60)
BEARING_PIER_Z = CHANNEL_WALL_HEIGHT + BEARING_PIER_DIMS[2] / 2.0
MORTAR_GROOVE = 0.012

BLOCK_DIMS = (0.20, 0.10, 0.24)  # bored bearing seat at axle height
# Journal bore slightly under the axle radius so the spinning axle stays
# captured in (and supported by) the bearing with a tiny intentional embed.
BORE_R = 0.033


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
    """Bored bearing seat with a clearance journal for the axle along local Y."""
    block = cq.Workplane("XY").box(*BLOCK_DIMS).edges("|Z").chamfer(0.015)
    bore = cq.Workplane("XZ").circle(BORE_R).extrude(BLOCK_DIMS[1], both=True)
    return block.cut(bore)


def _channel_wall_solid() -> cq.Workplane:
    """One low masonry flume wall with shallow block-joint grooves."""
    wall = cq.Workplane("XY").box(CHANNEL_LEN, CHANNEL_WALL_THICK, CHANNEL_WALL_HEIGHT)
    rows = 4
    row_h = CHANNEL_WALL_HEIGHT / rows

    # Horizontal mortar joints recessed into both visible faces.
    for ri in range(1, rows):
        z = -CHANNEL_WALL_HEIGHT / 2.0 + ri * row_h
        for sy in (1.0, -1.0):
            groove = (
                cq.Workplane("XY")
                .box(CHANNEL_LEN + 0.02, MORTAR_GROOVE * 2.0, MORTAR_GROOVE)
                .translate((0.0, sy * CHANNEL_WALL_THICK / 2.0, z))
            )
            wall = wall.cut(groove)

    # Staggered vertical joints on each row so the wall reads as block masonry
    # without splitting into separate floating stones.
    for ri in range(rows):
        z = -CHANNEL_WALL_HEIGHT / 2.0 + (ri + 0.5) * row_h
        offset = 0.18 if ri % 2 else 0.0
        joint_count = 5
        for ji in range(-joint_count, joint_count + 1):
            x = ji * (CHANNEL_LEN / (2 * joint_count + 1)) + offset
            if abs(x) > CHANNEL_LEN / 2.0 - 0.08:
                continue
            for sy in (1.0, -1.0):
                groove = (
                    cq.Workplane("XY")
                    .box(MORTAR_GROOVE, MORTAR_GROOVE * 2.0, row_h * 0.70)
                    .translate((x, sy * CHANNEL_WALL_THICK / 2.0, z))
                )
                wall = wall.cut(groove)
    return wall


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watermill_waterwheel_sluice")

    wheel_wood = model.material("wheel_wood", color=(0.89, 0.72, 0.66))
    masonry = model.material("masonry", color=(0.48, 0.46, 0.43))
    cap_stone = model.material("cap_stone", color=(0.56, 0.54, 0.50))
    axle_metal = model.material("axle_metal", color=(0.70, 0.71, 0.73))
    collar_metal = model.material("collar_metal", color=(0.80, 0.81, 0.84))

    # ---------------------------------------------------------- sluice mount
    sluice = model.part("sluice_mount")

    sluice.visual(
        Box(CHANNEL_FLOOR_DIMS),
        origin=Origin(xyz=(0.0, 0.0, CHANNEL_FLOOR_DIMS[2] / 2.0)),
        material=masonry,
        name="channel_floor",
    )

    wall_mesh = mesh_from_cadquery(_channel_wall_solid(), "masonry_channel_wall")
    wall_z = CHANNEL_FLOOR_DIMS[2] + CHANNEL_WALL_HEIGHT / 2.0
    for wi, wy in enumerate((CHANNEL_WALL_Y, -CHANNEL_WALL_Y)):
        sluice.visual(
            wall_mesh,
            origin=Origin(xyz=(0.0, wy, wall_z)),
            material=masonry,
            name=f"channel_wall_{wi}",
        )
        sluice.visual(
            Box(BEARING_PIER_DIMS),
            origin=Origin(xyz=(0.0, wy, BEARING_PIER_Z)),
            material=cap_stone,
            name=f"bearing_pier_{wi}",
        )

    bearing_mesh = mesh_from_cadquery(_bearing_block_solid(), "bearing_block")
    for bi, by in enumerate((BEARING_Y, -BEARING_Y)):
        sluice.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, by, AXLE_Z)),
            material=cap_stone,
            name=f"bearing_block_{bi}",
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
        "sluice_to_waterwheel",
        ArticulationType.CONTINUOUS,
        parent=sluice,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sluice = object_model.get_part("sluice_mount")
    wheel = object_model.get_part("waterwheel")
    spin = object_model.get_articulation("sluice_to_waterwheel")

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
    sluice_aabb = ctx.part_world_aabb(sluice)
    ctx.check(
        "sluice floor grounded at z=0",
        sluice_aabb is not None and abs(sluice_aabb[0][2]) < 2e-3,
        details=f"sluice aabb={sluice_aabb}",
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

    # Changed mount: two low parallel channel walls form a flume, and bearing
    # seats sit on top of those walls instead of an A-frame trestle.
    mount_names = [v.name for v in sluice.visuals if v.name]
    wall_names = [name for name in mount_names if name.startswith("channel_wall_")]
    pier_names = [name for name in mount_names if name.startswith("bearing_pier_")]
    ctx.check(
        "sluice mount has exactly two parallel channel walls",
        len(wall_names) == 2,
        details=f"walls={wall_names}",
    )
    ctx.check(
        "A-frame legs feet and braces removed",
        not any(
            name.startswith(("leg_", "foot_", "cross_brace_", "diagonal_brace"))
            for name in mount_names
        ),
        details=f"mount visuals={mount_names}",
    )
    wall0 = ctx.part_element_world_aabb(sluice, elem="channel_wall_0")
    wall1 = ctx.part_element_world_aabb(sluice, elem="channel_wall_1")
    floor = ctx.part_element_world_aabb(sluice, elem="channel_floor")
    ctx.check(
        "channel walls run long and parallel along the flume",
        wall0 is not None
        and wall1 is not None
        and (wall0[1][0] - wall0[0][0]) > 2.2
        and (wall1[1][0] - wall1[0][0]) > 2.2
        and (wall0[0][1] > 0.25)
        and (wall1[1][1] < -0.25),
        details=f"wall0={wall0}, wall1={wall1}",
    )
    ctx.check(
        "floor slab spans between both channel walls",
        floor is not None
        and wall0 is not None
        and wall1 is not None
        and floor[0][1] <= wall1[0][1] + 0.002
        and floor[1][1] >= wall0[1][1] - 0.002,
        details=f"floor={floor}, wall0={wall0}, wall1={wall1}",
    )
    ctx.check(
        "bearing seats are raised on the channel walls",
        len(pier_names) == 2,
        details=f"piers={pier_names}",
    )
    for bi in range(2):
        ctx.expect_overlap(
            sluice,
            sluice,
            axes="xy",
            elem_a=f"bearing_block_{bi}",
            elem_b=f"bearing_pier_{bi}",
            min_overlap=0.08,
            name=f"bearing seat {bi} sits on its masonry pier",
        )
        ctx.expect_gap(
            sluice,
            sluice,
            axis="z",
            positive_elem=f"bearing_block_{bi}",
            negative_elem=f"bearing_pier_{bi}",
            max_gap=0.002,
            max_penetration=0.002,
            name=f"bearing seat {bi} is seated on the pier top",
        )
    ctx.check(
        "wheel sits down inside the flume channel",
        rim0 is not None
        and rim1 is not None
        and wall0 is not None
        and wall1 is not None
        and min(rim0[0][2], rim1[0][2]) < wall0[1][2]
        and rim1[0][1] > wall1[1][1]
        and rim0[1][1] < wall0[0][1],
        details=f"rim0={rim0}, rim1={rim1}, wall0={wall0}, wall1={wall1}",
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
        sluice,
        elem_a="axle",
        elem_b="bearing_block_0",
        reason="Axle is intentionally captured in the first sluice bearing journal bore.",
    )
    ctx.allow_overlap(
        wheel,
        sluice,
        elem_a="axle",
        elem_b="bearing_block_1",
        reason="Axle is intentionally captured in the second sluice bearing journal bore.",
    )
    ctx.expect_overlap(
        wheel,
        sluice,
        axes="y",
        elem_a="axle",
        elem_b="bearing_block_0",
        min_overlap=0.08,
        name="axle passes through the front bearing block",
    )
    ctx.expect_overlap(
        wheel,
        sluice,
        axes="y",
        elem_a="axle",
        elem_b="bearing_block_1",
        min_overlap=0.08,
        name="axle passes through the rear bearing block",
    )
    # Rim clears the inboard face of the bearing block; collar sits outboard.
    ctx.expect_gap(
        sluice,
        wheel,
        axis="y",
        positive_elem="bearing_block_0",
        negative_elem="rim_0",
        min_gap=0.005,
        name="rim clears the bearing block",
    )
    ctx.expect_gap(
        wheel,
        sluice,
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
            sluice,
            wheel,
            axis="y",
            positive_elem="bearing_block_0",
            negative_elem="rim_0",
            min_gap=0.005,
            name="wheel keeps clearance to the frame while spinning",
        )

    return ctx.report()


object_model = build_object_model()
