from __future__ import annotations

"""Rusty chaldron-style mine cart with a tipping tub, end latch hoop, and rolling wheelsets.

Layout (world frame, meters, z-up):
- X runs along the track; the latch hoop end is at -X, the tip pivot end at +X.
- Root part is the riveted iron chassis; wheels roll on the ground plane z=0.
- The curved, flared tub pivots on a transverse trunnion pin near the +X end,
  so positive tip motion raises the latch end and dumps over the +X lip.
- A gravity latch hook on the end hoop swings up/outward to release the tub.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---------------------------------------------------------------- dimensions
FRAME_TOP = 0.50
AXLE_Z = 0.24
AXLE_X = 0.40
WHEEL_RADIUS = 0.24
WHEEL_Y = 0.33

TUB_BASE_Z = 0.655  # outer bottom of the tub
TUB_MID_Z = 1.03
TUB_TOP_Z = 1.26  # loft top before the saddle rim cut
SADDLE_R = 1.6
SADDLE_CZ = 2.66  # rim mid height = SADDLE_CZ - SADDLE_R = 1.06
WALL_T = 0.035

TIP_PIVOT = (0.45, 0.0, 0.62)
LATCH_PIVOT = (-0.792, 0.0, 0.94)


def _lin(z: float, z0: float, v0: float, z1: float, v1: float) -> float:
    return v0 + (z - z0) * (v1 - v0) / (z1 - z0)


def tub_length(z: float) -> float:
    if z <= TUB_MID_Z:
        return _lin(z, TUB_BASE_Z, 0.95, TUB_MID_Z, 1.24)
    return _lin(z, TUB_MID_Z, 1.24, TUB_TOP_Z, 1.52)


def tub_width(z: float) -> float:
    if z <= TUB_MID_Z:
        return _lin(z, TUB_BASE_Z, 0.62, TUB_MID_Z, 0.84)
    return _lin(z, TUB_MID_Z, 0.84, TUB_TOP_Z, 1.04)


def tub_corner(z: float) -> float:
    if z <= TUB_MID_Z:
        return _lin(z, TUB_BASE_Z, 0.10, TUB_MID_Z, 0.13)
    return _lin(z, TUB_MID_Z, 0.13, TUB_TOP_Z, 0.16)


def rim_z(x: float) -> float:
    """Height of the saddle-curved rim at longitudinal station x."""
    return SADDLE_CZ - math.sqrt(SADDLE_R * SADDLE_R - x * x)


def _loft(stations: list[tuple[float, float, float, float]]) -> cq.Workplane:
    """Ruled loft through rounded-rect sections given as (z, length, width, corner_r)."""
    z0, length0, width0, r0 = stations[0]
    wp = cq.Workplane("XY").workplane(offset=z0)
    wp = wp.polyline(rounded_rect_profile(length0, width0, r0, corner_segments=8)).close()
    prev_z = z0
    for z, length, width, r in stations[1:]:
        wp = wp.workplane(offset=z - prev_z)
        wp = wp.polyline(rounded_rect_profile(length, width, r, corner_segments=8)).close()
        prev_z = z
    return wp.loft(ruled=True)


def _tub_stations(d_len: float, d_wid: float, d_r: float, z_top: float) -> list:
    zs = (TUB_BASE_Z, TUB_MID_Z, z_top)
    return [(z, tub_length(z) + d_len, tub_width(z) + d_wid, tub_corner(z) + d_r) for z in zs]


def _saddle_cutter() -> cq.Solid:
    return cq.Solid.makeCylinder(
        SADDLE_R, 2.4, cq.Vector(0.0, -1.2, SADDLE_CZ), cq.Vector(0.0, 1.0, 0.0)
    )


def build_tub_shell() -> cq.Workplane:
    outer = _loft(_tub_stations(0.0, 0.0, 0.0, TUB_TOP_Z))
    outer = outer.cut(_saddle_cutter())
    # Hollow cavity: 35 mm walls, 40 mm floor; the cavity loft extends above the
    # saddle rim so the tub reads open from above.
    inner_stations = [
        (TUB_BASE_Z + 0.04, tub_length(TUB_BASE_Z + 0.04) - 2 * WALL_T,
         tub_width(TUB_BASE_Z + 0.04) - 2 * WALL_T, tub_corner(TUB_BASE_Z + 0.04) - 0.02),
        (TUB_MID_Z, tub_length(TUB_MID_Z) - 2 * WALL_T,
         tub_width(TUB_MID_Z) - 2 * WALL_T, tub_corner(TUB_MID_Z) - 0.02),
        (1.30, tub_length(1.30) - 2 * WALL_T, tub_width(1.30) - 2 * WALL_T,
         tub_corner(1.30) - 0.02),
    ]
    return outer.cut(_loft(inner_stations))


def build_strap_band(z_lo: float, z_hi: float) -> cq.Workplane:
    """Wrap-around iron strap band, 18 mm proud of the tub wall, embedded 6 mm."""
    shell = _loft(_tub_stations(0.036, 0.036, 0.018, TUB_TOP_Z))
    shell = shell.cut(_loft(_tub_stations(-0.012, -0.012, -0.006, TUB_TOP_Z)))
    slab = cq.Workplane("XY").box(2.4, 1.6, z_hi - z_lo).translate((0.0, 0.0, (z_lo + z_hi) / 2))
    return shell.intersect(slab)


def build_hoop() -> cq.Workplane:
    """Black iron stirrup hoop standing on the latch-end beam."""
    outer = (
        cq.Workplane("YZ", origin=(-0.7575, 0.0, 0.0))
        .center(0.0, 0.795)
        .rect(0.40, 0.61)
        .extrude(0.035)
        .edges("|X and >Z")
        .fillet(0.16)
    )
    cutter = (
        cq.Workplane("YZ", origin=(-0.7625, 0.0, 0.0))
        .center(0.0, 0.67)
        .rect(0.28, 0.58)
        .extrude(0.045)
        .edges("|X and >Z")
        .fillet(0.12)
    )
    return outer.cut(cutter)


def _rivet_row(points: list[tuple[float, float, float]], radius: float = 0.012) -> MeshGeometry:
    merged = MeshGeometry()
    for x, y, z in points:
        dome = SphereGeometry(radius, width_segments=10, height_segments=8)
        dome.translate(x, y, z)
        merged.merge(dome)
    return merged


def build_rim_rivets() -> MeshGeometry:
    pts: list[tuple[float, float, float]] = []
    for x in [-0.44 + 0.11 * i for i in range(9)]:
        z = rim_z(x)
        y = tub_width(z) / 2 - WALL_T / 2
        pts.append((x, y, z - 0.004))
        pts.append((x, -y, z - 0.004))
    # Three studs across each curved end rim.
    z_end = rim_z(0.72)
    for sx in (1.0, -1.0):
        for y in (-0.16, 0.0, 0.16):
            pts.append((sx * 0.725, y, z_end - 0.004))
    return _rivet_row(pts)


def build_band_rivets() -> MeshGeometry:
    pts: list[tuple[float, float, float]] = []
    for z, xs, ys_end in (
        (0.81, [-0.40 + 0.10 * i for i in range(9)], (-0.18, 0.0, 0.18)),
        (0.98, [-0.44 + 0.11 * i for i in range(9)], (-0.20, 0.0, 0.20)),
    ):
        y_side = tub_width(z) / 2 + 0.018 - 0.007
        for x in xs:
            pts.append((x, y_side, z))
            pts.append((x, -y_side, z))
        x_end = tub_length(z) / 2 + 0.018 - 0.007
        for y in ys_end:
            pts.append((x_end, y, z))
            pts.append((-x_end, y, z))
    return _rivet_row(pts)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chaldron_mine_cart")

    rust_body = model.material("rust_body", rgba=(0.455, 0.235, 0.125, 1.0))
    rust_frame = model.material("rust_frame", rgba=(0.40, 0.205, 0.11, 1.0))
    iron_black = model.material("iron_black", rgba=(0.135, 0.125, 0.12, 1.0))
    iron_wheel = model.material("iron_wheel", rgba=(0.115, 0.11, 0.105, 1.0))

    # ------------------------------------------------------------- chassis
    chassis = model.part("chassis")

    for i, sy in enumerate((1.0, -1.0)):
        chassis.visual(
            Box((1.56, 0.08, 0.12)),
            origin=Origin(xyz=(0.0, sy * 0.44, 0.44)),
            material=rust_frame,
            name=f"side_frame_beam_{i}",
        )
    for i, sx in enumerate((-1.0, 1.0)):
        chassis.visual(
            Box((0.08, 0.96, 0.12)),
            origin=Origin(xyz=(sx * 0.74, 0.0, 0.44)),
            material=rust_frame,
            name=f"end_frame_beam_{i}",
        )
    chassis.visual(
        Box((0.16, 0.88, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=rust_frame,
        name="center_cross_plank",
    )
    # Transverse cradle bolsters that carry the tub.
    for i, sx in enumerate((-1.0, 1.0)):
        chassis.visual(
            Box((0.12, 0.80, 0.15)),
            origin=Origin(xyz=(sx * 0.22, 0.0, 0.573)),
            material=rust_frame,
            name=f"saddle_bolster_{i}",
        )
    # Raked cradle braces from the end beams up to the bolsters.
    brace_idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            chassis.visual(
                Box((0.46, 0.05, 0.04)),
                origin=Origin(xyz=(sx * 0.50, sy * 0.20, 0.565), rpy=(0.0, sx * 0.223, 0.0)),
                material=rust_frame,
                name=f"cradle_brace_{brace_idx}",
            )
            brace_idx += 1
    # Trunnion bolster, brackets, and transverse tip pivot pin.
    chassis.visual(
        Box((0.10, 0.96, 0.06)),
        origin=Origin(xyz=(TIP_PIVOT[0], 0.0, 0.528)),
        material=rust_frame,
        name="trunnion_bolster",
    )
    for i, sy in enumerate((1.0, -1.0)):
        chassis.visual(
            Box((0.07, 0.04, 0.095)),
            origin=Origin(xyz=(TIP_PIVOT[0], sy * 0.33, 0.5975)),
            material=iron_black,
            name=f"trunnion_bracket_{i}",
        )
    chassis.visual(
        Cylinder(radius=0.022, length=0.72),
        origin=Origin(xyz=TIP_PIVOT, rpy=(math.pi / 2, 0.0, 0.0)),
        material=iron_black,
        name="tip_pivot_pin",
    )
    # Axleboxes (pedestals) that capture the axle journals.
    box_idx = 0
    for sx in (-1.0, 1.0):
        for sy in (1.0, -1.0):
            chassis.visual(
                Box((0.12, 0.06, 0.22)),
                origin=Origin(xyz=(sx * AXLE_X, sy * 0.445, 0.29)),
                material=iron_black,
                name=f"axlebox_{box_idx}",
            )
            box_idx += 1
    # Dumb buffers on both headstocks.
    buf_idx = 0
    for sx in (-1.0, 1.0):
        for sy in (1.0, -1.0):
            chassis.visual(
                Box((0.13, 0.12, 0.13)),
                origin=Origin(xyz=(sx * 0.835, sy * 0.30, 0.44)),
                material=rust_frame,
                name=f"buffer_block_{buf_idx}",
            )
            chassis.visual(
                Box((0.03, 0.15, 0.15)),
                origin=Origin(xyz=(sx * 0.915, sy * 0.30, 0.44)),
                material=rust_frame,
                name=f"buffer_head_{buf_idx}",
            )
            buf_idx += 1
    # Latch hoop and latch hinge hardware at the -X end.
    chassis.visual(
        mesh_from_cadquery(build_hoop(), "latch_hoop"),
        material=iron_black,
        name="latch_hoop",
    )
    chassis.visual(
        Box((0.02, 0.10, 0.16)),
        origin=Origin(xyz=(-0.7645, 0.0, 0.98)),
        material=iron_black,
        name="latch_mount_plate",
    )
    for i, sy in enumerate((1.0, -1.0)):
        chassis.visual(
            Box((0.045, 0.025, 0.08)),
            origin=Origin(xyz=(-0.792, sy * 0.0475, 0.94)),
            material=iron_black,
            name=f"latch_hinge_ear_{i}",
        )
    chassis.visual(
        Cylinder(radius=0.014, length=0.13),
        origin=Origin(xyz=LATCH_PIVOT, rpy=(math.pi / 2, 0.0, 0.0)),
        material=iron_black,
        name="latch_pin",
    )

    # ----------------------------------------------------------------- tub
    tub = model.part("tub")
    tub_off = Origin(xyz=(-TIP_PIVOT[0], 0.0, -TIP_PIVOT[2]))
    tub.visual(
        mesh_from_cadquery(build_tub_shell(), "tub_shell"),
        origin=tub_off,
        material=rust_body,
        name="tub_shell",
    )
    tub.visual(
        mesh_from_cadquery(build_strap_band(0.78, 0.84), "strap_band_lower"),
        origin=tub_off,
        material=iron_black,
        name="strap_band_lower",
    )
    tub.visual(
        mesh_from_cadquery(build_strap_band(0.95, 1.01), "strap_band_upper"),
        origin=tub_off,
        material=iron_black,
        name="strap_band_upper",
    )
    tub.visual(
        mesh_from_geometry(build_rim_rivets(), "rim_rivets"),
        origin=tub_off,
        material=iron_black,
        name="rim_rivets",
    )
    tub.visual(
        mesh_from_geometry(build_band_rivets(), "band_rivets"),
        origin=tub_off,
        material=iron_black,
        name="band_rivets",
    )
    # Pivot lugs that wrap the chassis trunnion pin.
    for i, sy in enumerate((1.0, -1.0)):
        tub.visual(
            Box((0.10, 0.04, 0.10)),
            origin=Origin(xyz=(0.0, sy * 0.28, 0.615 - TIP_PIVOT[2])),
            material=iron_black,
            name=f"pivot_lug_{i}",
        )
    # Catch lug on the latch-end wall, held down by the hook toe.
    tub.visual(
        Box((0.145, 0.07, 0.05)),
        origin=Origin(xyz=(-0.575 - TIP_PIVOT[0], 0.0, 0.75 - TIP_PIVOT[2])),
        material=iron_black,
        name="catch_lug",
    )

    model.articulation(
        "tub_tip",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=tub,
        origin=Origin(xyz=TIP_PIVOT),
        # Positive q raises the latch end and dumps the load over the +X lip.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.8, lower=0.0, upper=0.6),
    )

    # ---------------------------------------------------------- latch hook
    latch = model.part("latch_hook")
    latch.visual(
        Box((0.03, 0.06, 0.29)),
        origin=Origin(xyz=(-0.801 - LATCH_PIVOT[0], 0.0, 0.805 - LATCH_PIVOT[2])),
        material=iron_black,
        name="hook_bar",
    )
    latch.visual(
        Box((0.216, 0.06, 0.05)),
        origin=Origin(xyz=(-0.708 - LATCH_PIVOT[0], 0.0, 0.685 - LATCH_PIVOT[2])),
        material=iron_black,
        name="latch_toe",
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=latch,
        origin=Origin(xyz=LATCH_PIVOT),
        # Positive q swings the toe outward (-X), releasing the catch lug.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=1.1),
    )

    # ------------------------------------------------------------ wheelsets
    wheel_geo = WheelGeometry(
        WHEEL_RADIUS,
        0.07,
        rim=WheelRim(inner_radius=0.19, flange_height=0.018, flange_thickness=0.012),
        hub=WheelHub(radius=0.06, width=0.09, cap_style="domed"),
        spokes=WheelSpokes(style="straight", count=8, thickness=0.022),
    )
    wheel_mesh = mesh_from_geometry(wheel_geo, "spoked_wheel")

    for i, sx in enumerate((-1.0, 1.0)):
        wheelset = model.part(f"wheelset_{i}")
        wheelset.visual(
            Cylinder(radius=0.032, length=0.88),
            origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
            material=iron_wheel,
            name="axle",
        )
        for j, sy in enumerate((1.0, -1.0)):
            wheelset.visual(
                wheel_mesh,
                # Wheel geometry spins about local X; yaw it so the hub cap faces outboard.
                origin=Origin(xyz=(0.0, sy * WHEEL_Y, 0.0), rpy=(0.0, 0.0, sy * math.pi / 2)),
                material=iron_wheel,
                name=f"wheel_{j}",
            )
        model.articulation(
            f"wheelset_{i}_roll",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=wheelset,
            origin=Origin(xyz=(sx * AXLE_X, 0.0, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=150.0, velocity=10.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    tub = object_model.get_part("tub")
    latch = object_model.get_part("latch_hook")
    ws0 = object_model.get_part("wheelset_0")
    ws1 = object_model.get_part("wheelset_1")
    tip = object_model.get_articulation("tub_tip")
    latch_joint = object_model.get_articulation("latch_pivot")

    # Intentional captured-pin / journal embeddings.
    for lug in ("pivot_lug_0", "pivot_lug_1"):
        ctx.allow_overlap(
            chassis,
            tub,
            elem_a="tip_pivot_pin",
            elem_b=lug,
            reason="Trunnion pin is intentionally captured inside the tub pivot lug.",
        )
    ctx.allow_overlap(
        chassis,
        latch,
        elem_a="latch_pin",
        elem_b="hook_bar",
        reason="Latch hinge pin is intentionally captured inside the hook bar.",
    )
    for box, ws in (("axlebox_0", ws0), ("axlebox_1", ws0), ("axlebox_2", ws1), ("axlebox_3", ws1)):
        ctx.allow_overlap(
            chassis,
            ws,
            elem_a=box,
            elem_b="axle",
            reason="Axle journal is intentionally seated inside the axlebox pedestal.",
        )

    # Tub is seated just above the cradle bolsters and clear of the pivot pin.
    ctx.expect_gap(
        tub,
        chassis,
        axis="z",
        positive_elem="tub_shell",
        negative_elem="saddle_bolster_0",
        min_gap=0.0,
        max_gap=0.02,
        name="tub sits just above the cradle bolsters",
    )
    ctx.expect_gap(
        tub,
        chassis,
        axis="z",
        positive_elem="tub_shell",
        negative_elem="tip_pivot_pin",
        min_gap=0.0,
        max_gap=0.05,
        name="tub shell clears the trunnion pin",
    )
    ctx.expect_overlap(
        tub,
        chassis,
        axes="yz",
        elem_a="pivot_lug_0",
        elem_b="tip_pivot_pin",
        min_overlap=0.01,
        name="pivot lug wraps the trunnion pin",
    )
    ctx.expect_overlap(
        tub,
        chassis,
        axes="xy",
        min_overlap=0.3,
        name="tub is carried over the chassis",
    )

    # Latch: at rest the hook toe sits under the tub catch lug and retains it.
    ctx.expect_gap(
        tub,
        latch,
        axis="z",
        positive_elem="catch_lug",
        negative_elem="latch_toe",
        min_gap=0.004,
        max_gap=0.03,
        name="hook toe sits just under the catch lug",
    )
    ctx.expect_overlap(
        latch,
        tub,
        axes="x",
        elem_a="latch_toe",
        elem_b="catch_lug",
        min_overlap=0.02,
        name="hook toe reaches under the catch lug (x)",
    )
    ctx.expect_overlap(
        latch,
        tub,
        axes="y",
        elem_a="latch_toe",
        elem_b="catch_lug",
        min_overlap=0.04,
        name="hook toe reaches under the catch lug (y)",
    )

    # Hoop arch rises over the latch end.
    hoop_aabb = ctx.part_element_world_aabb(chassis, elem="latch_hoop")
    ctx.check(
        "latch hoop arches above the frame at the latch end",
        hoop_aabb is not None and hoop_aabb[1][2] > 1.05 and hoop_aabb[0][0] < -0.72,
        details=f"hoop aabb={hoop_aabb}",
    )

    # Rim studs exist along the saddle-curved rim.
    rivet_aabb = ctx.part_element_world_aabb(tub, elem="rim_rivets")
    ctx.check(
        "rivet studs follow the tub rim",
        rivet_aabb is not None and rivet_aabb[1][2] > 1.05 and rivet_aabb[1][0] > 0.6,
        details=f"rim rivets aabb={rivet_aabb}",
    )

    # Wheels rest on the ground plane and span the full gauge.
    for name, ws in (("wheelset_0", ws0), ("wheelset_1", ws1)):
        aabb = ctx.part_world_aabb(ws)
        ctx.check(
            f"{name} wheels touch the ground plane",
            aabb is not None and abs(aabb[0][2]) <= 0.01,
            details=f"{name} aabb={aabb}",
        )
        ctx.check(
            f"{name} spans both rails",
            aabb is not None and (aabb[1][1] - aabb[0][1]) >= 0.7,
            details=f"{name} aabb={aabb}",
        )

    # Decisive pose: tipping raises the latch-end catch lug well above rest.
    lug_rest = ctx.part_element_world_aabb(tub, elem="catch_lug")
    with ctx.pose({tip: 0.45}):
        lug_tipped = ctx.part_element_world_aabb(tub, elem="catch_lug")
        ctx.expect_overlap(
            tub,
            chassis,
            axes="y",
            min_overlap=0.3,
            name="tipped tub stays over the chassis",
        )
    ctx.check(
        "tipping raises the latch end of the tub",
        lug_rest is not None
        and lug_tipped is not None
        and lug_tipped[0][2] > lug_rest[0][2] + 0.15,
        details=f"rest={lug_rest}, tipped={lug_tipped}",
    )

    # Decisive pose: swinging the latch pulls the toe clear of the catch lug.
    with ctx.pose({latch_joint: 1.0}):
        toe_released = ctx.part_element_world_aabb(latch, elem="latch_toe")
        lug_now = ctx.part_element_world_aabb(tub, elem="catch_lug")
        ctx.check(
            "released hook toe swings clear of the catch lug",
            toe_released is not None
            and lug_now is not None
            and toe_released[1][0] < lug_now[0][0] - 0.02,
            details=f"toe={toe_released}, lug={lug_now}",
        )

    return ctx.report()


object_model = build_object_model()
