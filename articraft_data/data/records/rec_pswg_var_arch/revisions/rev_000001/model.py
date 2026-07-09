from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Commercial playground swing set, glossy red steel ARCH frame.
#   X = width (3.0 m top rail), Y = depth (1.6 m at the feet), Z = up (2.4 m).
# Root part: red arch frame (two curved inverted-U arch tubes + top rail +
# bolt-hole base plates) plus four galvanized clevis hangers with pins. Two
# rigid pendulum swing parts (chain pair + sagged rubber belt seat) each hang
# on a revolute joint whose axis runs along the rail through the hanger pins.
#
# Variant: the A-frame end supports are replaced by single tall curved arches
# (inverted-U tubes) spanning from ground foot to ground foot over the swings.
# ---------------------------------------------------------------------------

RED = Material(name="glossy_red_steel", rgba=(0.72, 0.045, 0.06, 1.0))
GALV = Material(name="galvanized_steel", rgba=(0.74, 0.76, 0.79, 1.0))
RUBBER = Material(name="black_rubber", rgba=(0.06, 0.06, 0.065, 1.0))

RAIL_Z = 2.33  # top rail centerline height
RAIL_SIZE = (3.0, 0.08, 0.08)  # rail top ends at 2.37 -> ~2.4 m overall
END_X = 1.38  # arch end planes (same as parent A-frame positions)
FOOT_Y = 0.76  # arch foot centers -> ~1.6 m deep at the feet
ARCH_TUBE_R = 0.035  # arch tube radius (70 mm OD structural tube)
ARCH_APEX_Z = RAIL_Z + RAIL_SIZE[2] / 2.0  # 2.37 arch centerline peak
PLATE_T = 0.014  # base plate thickness
FOOT_Z = PLATE_T / 2.0  # arch foot centerline at plate mid-height (embedded in plate)

PIVOT_Z = 2.228  # hanger pin axis = swing revolute axis height
SWING_X = 0.70  # swing centers at +/-0.70
CHAIN_DX = 0.22  # chain half-spacing on each swing
HANGER_XS = (-SWING_X - CHAIN_DX, -SWING_X + CHAIN_DX, SWING_X - CHAIN_DX, SWING_X + CHAIN_DX)

# Chain link (oval ring swept tube)
LINK_A = 0.0255  # half-length of tube centerline oval (long axis along chain)
LINK_B = 0.0105  # half-width of tube centerline oval
LINK_R = 0.0045  # tube radius
LINK_PITCH = 0.044  # > 2*(A-R)=0.042 so consecutive links interpenetrate ~2 mm
N_LINKS = 34  # ~1.5 m of chain
PIN_R = 0.004
# Top link hangs on the pin: ring center sits below the pin axis so the pin
# beds ~0.8 mm into the inner top of the oval (captured-pin contact).
HANG_DROP = LINK_A - LINK_R - PIN_R + 0.0008  # 0.0178

CLAMP_Z = -(HANG_DROP + (N_LINKS - 1) * LINK_PITCH) - 0.0352  # ~ -1.505
SEAT_END_Z = CLAMP_Z - 0.010
SEAT_SAG = 0.113  # belt bottom dips below the clamped ends


def _oval_link_geometry():
    """Oval chain link as a closed swept tube in the YZ plane, long axis Z."""
    pts = []
    n = 20
    for i in range(n):
        t = 2.0 * math.pi * i / n
        pts.append((0.0, LINK_B * math.sin(t), LINK_A * math.cos(t)))
    return tube_from_spline_points(
        pts,
        radius=LINK_R,
        samples_per_segment=4,
        closed_spline=True,
        radial_segments=10,
        up_hint=(1.0, 0.0, 0.0),
    )


def _base_plate_geometry():
    """Flat rectangular base plate with four bolt holes."""
    outer = rounded_rect_profile(0.20, 0.16, 0.02)
    holes = []
    for hx in (-0.075, 0.075):
        for hy in (-0.058, 0.058):
            holes.append(
                [
                    (hx + 0.007 * math.cos(t), hy + 0.007 * math.sin(t))
                    for t in [2.0 * math.pi * i / 16 for i in range(16)]
                ]
            )
    return ExtrudeWithHolesGeometry(outer, holes, PLATE_T, cap=True, center=True)


def _arch_geometry():
    """Curved inverted-U arch tube as a half-ellipse in the YZ plane at x=0.

    The arch centerline runs from (0, -FOOT_Y, FOOT_Z) up through
    (0, 0, ARCH_APEX_Z) and back down to (0, +FOOT_Y, FOOT_Z), forming a
    smooth half-elliptical arch.
    """
    n_pts = 25  # 24 segments for a smooth half-ellipse
    pts = []
    for i in range(n_pts):
        theta = math.pi * i / (n_pts - 1)
        y = -FOOT_Y * math.cos(theta)
        z = FOOT_Z + (ARCH_APEX_Z - FOOT_Z) * math.sin(theta)
        pts.append((0.0, y, z))
    return tube_from_spline_points(
        pts,
        radius=ARCH_TUBE_R,
        samples_per_segment=8,
        closed_spline=False,
        radial_segments=16,
        cap_ends=True,
        up_hint=(1.0, 0.0, 0.0),
    )


def _seat_belt_geometry():
    """Flexible belt seat sagged into a shallow U, swept along X."""
    path = [
        (-CHAIN_DX, 0.0, SEAT_END_Z),
        (-0.185, 0.0, SEAT_END_Z - 0.063),
        (-0.10, 0.0, SEAT_END_Z - 0.102),
        (0.0, 0.0, SEAT_END_Z - SEAT_SAG),
        (0.10, 0.0, SEAT_END_Z - 0.102),
        (0.185, 0.0, SEAT_END_Z - 0.063),
        (CHAIN_DX, 0.0, SEAT_END_Z),
    ]
    return sweep_profile_along_spline(
        path,
        profile=rounded_rect_profile(0.14, 0.012, 0.004),
        samples_per_segment=8,
        cap_profile=True,
    )


def _add_arch_end(frame, end_index: int, x_end: float, plate_mesh, arch_mesh) -> None:
    """One curved inverted-U arch with two base plates at the feet."""
    # The arch tube is generated at x=0; place it at x_end.
    frame.visual(
        arch_mesh,
        origin=Origin(xyz=(x_end, 0.0, 0.0)),
        material=RED,
        name=f"arch_{end_index}",
    )
    # Two base plates, one under each arch foot.
    for foot_index, sy in enumerate((-1.0, 1.0)):
        frame.visual(
            plate_mesh,
            origin=Origin(xyz=(x_end, sy * FOOT_Y, PLATE_T / 2.0)),
            material=RED,
            name=f"base_plate_{end_index}_{foot_index}",
        )


def _add_hanger(frame, index: int, x_h: float) -> None:
    """Galvanized clevis hanger bolted under the rail, with a transverse pin."""
    frame.visual(
        Box((0.05, 0.09, 0.012)),
        origin=Origin(xyz=(x_h, 0.0, 2.287)),
        material=GALV,
        name=f"hanger_mount_{index}",
    )
    frame.visual(
        Box((0.034, 0.022, 0.018)),
        origin=Origin(xyz=(x_h, 0.0, 2.274)),
        material=GALV,
        name=f"hanger_body_{index}",
    )
    for side, sx in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.006, 0.022, 0.046)),
            origin=Origin(xyz=(x_h + sx * 0.0145, 0.0, 2.245)),
            material=GALV,
            name=f"hanger_cheek_{index}_{side}",
        )
    frame.visual(
        Cylinder(radius=PIN_R, length=0.040),
        origin=Origin(xyz=(x_h, 0.0, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=GALV,
        name=f"hanger_pin_{index}",
    )


def _add_swing(model, frame, index: int, x_center: float, link_yz, link_xz, belt_mesh):
    """One rigid pendulum: two chains + clamps + sagged belt, in pivot frame."""
    swing = model.part(f"swing_{index}")
    for c, sx in enumerate((-1.0, 1.0)):
        cx = sx * CHAIN_DX
        for k in range(N_LINKS):
            cz = -(HANG_DROP + k * LINK_PITCH)
            mesh = link_yz if k % 2 == 0 else link_xz
            name = f"chain_{c}_top_link" if k == 0 else f"chain_{c}_link_{k}"
            swing.visual(
                mesh,
                origin=Origin(xyz=(cx, 0.0, cz)),
                material=GALV,
                name=name,
            )
        swing.visual(
            Box((0.05, 0.09, 0.04)),
            origin=Origin(xyz=(cx, 0.0, CLAMP_Z)),
            material=RED,
            name=f"clamp_{c}",
        )
    swing.visual(belt_mesh, origin=Origin(), material=RUBBER, name="seat_belt")

    model.articulation(
        f"swing_{index}_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing,
        origin=Origin(xyz=(x_center, 0.0, PIVOT_Z)),
        # Axis horizontal along the rail; positive q swings the seat to +Y.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=4.0, lower=-1.0, upper=1.0),
    )
    return swing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_playground_swing_set")

    frame = model.part("frame")
    frame.visual(
        Box(RAIL_SIZE),
        origin=Origin(xyz=(0.0, 0.0, RAIL_Z)),
        material=RED,
        name="top_rail",
    )

    plate_mesh = mesh_from_geometry(_base_plate_geometry(), "base_plate")
    arch_mesh = mesh_from_geometry(_arch_geometry(), "arch_tube")

    for end_index, x_end in enumerate((-END_X, END_X)):
        _add_arch_end(frame, end_index, x_end, plate_mesh, arch_mesh)

    for index, x_h in enumerate(HANGER_XS):
        _add_hanger(frame, index, x_h)

    link_yz_geom = _oval_link_geometry()
    link_xz_geom = link_yz_geom.copy().rotate_z(math.pi / 2.0)
    link_yz = mesh_from_geometry(link_yz_geom, "chain_link_yz")
    link_xz = mesh_from_geometry(link_xz_geom, "chain_link_xz")
    belt_mesh = mesh_from_geometry(_seat_belt_geometry(), "seat_belt")

    _add_swing(model, frame, 0, -SWING_X, link_yz, link_xz, belt_mesh)
    _add_swing(model, frame, 1, SWING_X, link_yz, link_xz, belt_mesh)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    swing_0 = object_model.get_part("swing_0")
    swing_1 = object_model.get_part("swing_1")
    pivot_0 = object_model.get_articulation("swing_0_pivot")
    pivot_1 = object_model.get_articulation("swing_1_pivot")

    # Captured-pin hangs: each top chain link intentionally beds ~0.8 mm onto
    # its clevis pin so the rigid pendulum is physically supported by the rail.
    ctx.allow_overlap(
        frame,
        swing_0,
        elem_a="hanger_pin_0",
        elem_b="chain_0_top_link",
        reason="Top chain link hangs on the clevis pin (captured pin bearing).",
    )
    ctx.allow_overlap(
        frame,
        swing_0,
        elem_a="hanger_pin_1",
        elem_b="chain_1_top_link",
        reason="Top chain link hangs on the clevis pin (captured pin bearing).",
    )
    ctx.allow_overlap(
        frame,
        swing_1,
        elem_a="hanger_pin_2",
        elem_b="chain_0_top_link",
        reason="Top chain link hangs on the clevis pin (captured pin bearing).",
    )
    ctx.allow_overlap(
        frame,
        swing_1,
        elem_a="hanger_pin_3",
        elem_b="chain_1_top_link",
        reason="Top chain link hangs on the clevis pin (captured pin bearing).",
    )

    # --- Overall envelope: ~3.0 m wide, ~2.4 m tall, ~1.6 m deep at the feet.
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame is ~3.0 m wide along the rail",
        frame_aabb is not None and 2.9 <= frame_aabb[1][0] - frame_aabb[0][0] <= 3.1,
        details=f"frame aabb={frame_aabb}",
    )
    ctx.check(
        "frame is ~2.4 m tall",
        frame_aabb is not None and 2.30 <= frame_aabb[1][2] <= 2.50,
        details=f"frame aabb={frame_aabb}",
    )
    ctx.check(
        "arch feet span ~1.5-1.7 m deep",
        frame_aabb is not None and 1.4 <= frame_aabb[1][1] - frame_aabb[0][1] <= 1.8,
        details=f"frame aabb={frame_aabb}",
    )
    ctx.check(
        "base plates rest on the ground plane",
        frame_aabb is not None and abs(frame_aabb[0][2]) <= 0.005,
        details=f"frame aabb={frame_aabb}",
    )

    # --- Top rail spans between the arch end planes.
    rail_aabb = ctx.part_element_world_aabb(frame, elem="top_rail")
    ctx.check(
        "top rail sits at the apex height and spans both ends",
        rail_aabb is not None
        and rail_aabb[0][0] <= -END_X
        and rail_aabb[1][0] >= END_X
        and rail_aabb[0][2] >= 2.25,
        details=f"rail aabb={rail_aabb}",
    )

    # --- Arch geometry: each end has a curved inverted-U tube.
    for end_index in range(2):
        arch_name = f"arch_{end_index}"
        arch_aabb = ctx.part_element_world_aabb(frame, elem=arch_name)
        ctx.check(
            f"{arch_name} is a tall curved arch reaching above 2.2 m",
            arch_aabb is not None and arch_aabb[1][2] >= 2.2,
            details=f"{arch_name} aabb={arch_aabb}",
        )
        ctx.check(
            f"{arch_name} spans the full foot depth (~1.5-1.7 m)",
            arch_aabb is not None and 1.3 <= arch_aabb[1][1] - arch_aabb[0][1] <= 1.8,
            details=f"{arch_name} aabb={arch_aabb}",
        )
        ctx.check(
            f"{arch_name} reaches down to ground level",
            arch_aabb is not None and arch_aabb[0][2] <= 0.06,
            details=f"{arch_name} aabb={arch_aabb}",
        )

    # --- Each chain is seated on its hanger pin (chains carry the seat).
    ctx.expect_contact(swing_0, frame, elem_a="chain_0_top_link", elem_b="hanger_pin_0")
    ctx.expect_contact(swing_0, frame, elem_a="chain_1_top_link", elem_b="hanger_pin_1")
    ctx.expect_contact(swing_1, frame, elem_a="chain_0_top_link", elem_b="hanger_pin_2")
    ctx.expect_contact(swing_1, frame, elem_a="chain_1_top_link", elem_b="hanger_pin_3")

    # --- Chains ~1.5 m long from hanger to seat clamps.
    top_link_aabb = ctx.part_element_world_aabb(swing_0, elem="chain_0_top_link")
    clamp_aabb = ctx.part_element_world_aabb(swing_0, elem="clamp_0")
    ctx.check(
        "chain runs ~1.5 m from hanger pin to seat clamp",
        top_link_aabb is not None
        and clamp_aabb is not None
        and 1.40 <= top_link_aabb[1][2] - clamp_aabb[0][2] <= 1.65,
        details=f"top_link={top_link_aabb}, clamp={clamp_aabb}",
    )

    # --- Belt seat: sagged U, hung at child height, clamped at both chains.
    for swing, label in ((swing_0, "swing_0"), (swing_1, "swing_1")):
        belt_aabb = ctx.part_element_world_aabb(swing, elem="seat_belt")
        clamp0 = ctx.part_element_world_aabb(swing, elem="clamp_0")
        clamp1 = ctx.part_element_world_aabb(swing, elem="clamp_1")
        ctx.check(
            f"{label} belt seat hangs at child seat height",
            belt_aabb is not None and 0.45 <= belt_aabb[0][2] <= 0.75,
            details=f"belt aabb={belt_aabb}",
        )
        ctx.check(
            f"{label} belt sags into a U below its clamped ends",
            belt_aabb is not None
            and clamp0 is not None
            and belt_aabb[0][2] < clamp0[0][2] - 0.05,
            details=f"belt={belt_aabb}, clamp={clamp0}",
        )
        ctx.check(
            f"{label} belt is a wide flexible strap (~0.14 m deep)",
            belt_aabb is not None and 0.10 <= belt_aabb[1][1] - belt_aabb[0][1] <= 0.18,
            details=f"belt aabb={belt_aabb}",
        )
        for clamp_name, clamp_aabb_i in (("clamp_0", clamp0), ("clamp_1", clamp1)):
            ctx.check(
                f"{label} {clamp_name} grips the belt end",
                belt_aabb is not None
                and clamp_aabb_i is not None
                and belt_aabb[0][0] <= clamp_aabb_i[1][0]
                and belt_aabb[1][0] >= clamp_aabb_i[0][0]
                and belt_aabb[1][2] >= clamp_aabb_i[0][2],
                details=f"belt={belt_aabb}, clamp={clamp_aabb_i}",
            )

    # --- Articulation: each swing is an independent fore-aft pendulum.
    belt0_rest = ctx.part_element_world_aabb(swing_0, elem="seat_belt")
    belt1_rest = ctx.part_element_world_aabb(swing_1, elem="seat_belt")
    with ctx.pose({pivot_0: 1.0}):
        belt0_fwd = ctx.part_element_world_aabb(swing_0, elem="seat_belt")
        belt1_still = ctx.part_element_world_aabb(swing_1, elem="seat_belt")
        ctx.check(
            "swing_0 at +1.0 rad swings its seat forward (+Y) and up",
            belt0_rest is not None
            and belt0_fwd is not None
            and (belt0_fwd[0][1] + belt0_fwd[1][1]) / 2.0 > 0.8
            and belt0_fwd[0][2] > belt0_rest[0][2] + 0.3,
            details=f"rest={belt0_rest}, fwd={belt0_fwd}",
        )
        ctx.check(
            "swing_1 stays at rest while swing_0 swings (independent)",
            belt1_rest is not None
            and belt1_still is not None
            and abs((belt1_still[0][1] + belt1_still[1][1]) / 2.0) < 0.02
            and abs(belt1_still[0][2] - belt1_rest[0][2]) < 0.005,
            details=f"rest={belt1_rest}, posed={belt1_still}",
        )
        # Hanger bearing keeps the chain captured at full forward swing.
        ctx.expect_contact(
            swing_0,
            frame,
            elem_a="chain_0_top_link",
            elem_b="hanger_pin_0",
            name="swing_0 chain stays seated on its pin at +1.0 rad",
        )
    with ctx.pose({pivot_1: -1.0}):
        belt1_back = ctx.part_element_world_aabb(swing_1, elem="seat_belt")
        ctx.check(
            "swing_1 at -1.0 rad swings its seat backward (-Y)",
            belt1_back is not None and (belt1_back[0][1] + belt1_back[1][1]) / 2.0 < -0.8,
            details=f"posed={belt1_back}",
        )
        ctx.expect_contact(
            swing_1,
            frame,
            elem_a="chain_1_top_link",
            elem_b="hanger_pin_3",
            name="swing_1 chain stays seated on its pin at -1.0 rad",
        )

    return ctx.report()


object_model = build_object_model()
