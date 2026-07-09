from __future__ import annotations

"""Dark riveted steel mine cart on a narrow-gauge rail track.

Structure (root -> leaves):
- rail_track (root): 7 wooden sleepers + two steel rails.
- chassis: prismatic "cart_travel" joint lets the cart roll along the track.
- wheel_0..wheel_3: continuous "wheel_spin_*" joints, flanged iron wheels
  seated on the rail heads.
- hopper: revolute "hopper_tip" joint on a transverse trunnion pin seated in
  saddle brackets, so the tub tips to dump toward one track end.
- shovel: fixed prop strapped flat against one tub side wall.
"""

from math import cos, pi, sin

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
    sample_catmull_rom_spline_2d,
)

# ---------------------------------------------------------------- dimensions

TRACK_LEN = 2.00
RAIL_Y = 0.275  # rail centerline half-gauge
SLEEPER = (0.20, 1.00, 0.06)
RAIL_TOP_Z = 0.06 + 0.022 + 0.055 + 0.028  # sleeper + foot + web + head

WHEEL_R = 0.13
WHEEL_X = 0.36
AXLE_Z = RAIL_TOP_Z + WHEEL_R - 0.0002  # tread seated on rail head (0.2 mm seat)

BEAM_Z = 0.40  # frame beam center height
PIVOT_Z = 0.60  # raised trunnion pin center height
TRUNNION_R = 0.032

TUB_HALF_BOT = 0.40
TUB_HALF_RIM = 0.50
TUB_HALF_LIP = 0.685
TUB_RIM_Z = 0.50  # in hopper local frame (pivot at local z=0)
TUB_BOT_Z = 0.005
TUB_HALF_W = 0.36
TUB_WALL = 0.03


def _mirror_x(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(-x, z) for x, z in pts]


def _build_tub_mesh():
    """Hollow riveted tub: sloped end walls that flare into curled end lips."""
    # Outer silhouette in the XZ plane (x along track, z up, hopper local frame).
    right_outer = sample_catmull_rom_spline_2d(
        [
            (TUB_HALF_BOT, TUB_BOT_Z),
            (0.47, 0.10),
            (0.535, 0.22),
            (0.585, 0.34),
            (0.625, 0.45),
            (0.655, 0.545),
            (TUB_HALF_LIP, 0.615),
        ],
        samples_per_segment=6,
    )
    # Inner descent of the curled lip, from the crest back to the side-rim line.
    right_crest = sample_catmull_rom_spline_2d(
        [
            (TUB_HALF_LIP, 0.615),
            (0.62, 0.585),
            (0.55, 0.535),
            (TUB_HALF_RIM, TUB_RIM_Z),
        ],
        samples_per_segment=6,
    )
    outer_loop = (
        right_outer
        + right_crest[1:]
        + _mirror_x(right_crest)[::-1][1:]
        + _mirror_x(right_outer)[::-1][1:]
    )

    # Inner cavity silhouette (leaves ~0.02 floor and ~0.03 end walls).
    right_inner = sample_catmull_rom_spline_2d(
        [
            (TUB_HALF_BOT - 0.035, 0.025),
            (0.437, 0.12),
            (0.503, 0.24),
            (0.553, 0.36),
            (0.593, 0.47),
            (0.61, 0.53),
        ],
        samples_per_segment=6,
    )
    inner_loop = (
        right_inner
        + [(0.61, 0.95), (-0.61, 0.95)]
        + _mirror_x(right_inner)[::-1]
    )

    def _extrude(loop: list[tuple[float, float]], half_width: float):
        wp = cq.Workplane("XZ").polyline(loop).close()
        return wp.extrude(half_width, both=True)

    outer = _extrude(outer_loop, TUB_HALF_W)
    cavity = _extrude(inner_loop, TUB_HALF_W - TUB_WALL)
    tub = outer.cut(cavity)
    return mesh_from_cadquery(tub, "hopper_tub.obj")


def _build_pivot_saddle_mesh():
    """Raised bearing block with a half-round groove for the trunnion pin."""
    block_h = 0.07
    block = cq.Workplane("XY").box(0.16, 0.055, block_h)
    groove = (
        cq.Workplane("XZ")
        .center(0.0, block_h / 2.0)
        .circle(TRUNNION_R)
        .extrude(0.08, both=True)
    )
    saddle = block.cut(groove)
    return mesh_from_cadquery(saddle, "raised_pivot_saddle.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tipping_mine_cart_on_rails")

    wood = ("sleeper_wood", (0.44, 0.31, 0.19, 1.0))
    rail_steel = ("rail_steel", (0.60, 0.61, 0.63, 1.0))
    tub_steel = ("tub_steel", (0.15, 0.15, 0.165, 1.0))
    strap_steel = ("strap_steel", (0.21, 0.21, 0.225, 1.0))
    frame_steel = ("frame_steel", (0.10, 0.10, 0.115, 1.0))
    wheel_iron = ("wheel_iron", (0.13, 0.13, 0.145, 1.0))
    hub_iron = ("hub_iron", (0.24, 0.24, 0.255, 1.0))
    blade_metal = ("blade_metal", (0.72, 0.73, 0.76, 1.0))
    handle_wood = ("handle_wood", (0.50, 0.36, 0.22, 1.0))

    from sdk import Material

    mats = {name: Material(name=name, rgba=rgba) for name, rgba in (
        wood, rail_steel, tub_steel, strap_steel, frame_steel,
        wheel_iron, hub_iron, blade_metal, handle_wood,
    )}

    # ------------------------------------------------------------ rail track
    track = model.part("rail_track")
    for i in range(7):
        x = -0.90 + 0.30 * i
        track.visual(
            Box(SLEEPER),
            origin=Origin(xyz=(x, 0.0, SLEEPER[2] / 2.0)),
            material=mats["sleeper_wood"],
            name=f"sleeper_{i}",
        )
    rail_len = 1.95
    for j, side in enumerate((1.0, -1.0)):
        y = side * RAIL_Y
        z0 = SLEEPER[2]
        track.visual(
            Box((rail_len, 0.090, 0.022)),
            origin=Origin(xyz=(0.0, y, z0 + 0.011)),
            material=mats["rail_steel"],
            name=f"rail_foot_{j}",
        )
        track.visual(
            Box((rail_len, 0.024, 0.055)),
            origin=Origin(xyz=(0.0, y, z0 + 0.022 + 0.0275)),
            material=mats["rail_steel"],
            name=f"rail_web_{j}",
        )
        track.visual(
            Box((rail_len, 0.050, 0.028)),
            origin=Origin(xyz=(0.0, y, z0 + 0.022 + 0.055 + 0.014)),
            material=mats["rail_steel"],
            name=f"rail_head_{j}",
        )

    # -------------------------------------------------------------- chassis
    chassis = model.part("chassis")
    for side in (1.0, -1.0):
        chassis.visual(
            Box((1.16, 0.075, 0.075)),
            origin=Origin(xyz=(0.0, side * 0.15, BEAM_Z)),
            material=mats["frame_steel"],
            name=f"frame_beam_{0 if side > 0 else 1}",
        )
    for k, x in enumerate((-0.5425, -0.18, 0.18, 0.5425)):
        chassis.visual(
            Box((0.075, 0.44, 0.075)),
            origin=Origin(xyz=(x, 0.0, BEAM_Z)),
            material=mats["frame_steel"],
            name=f"cross_beam_{k}",
        )
    for k, x in enumerate((-0.6125, 0.6125)):
        chassis.visual(
            Box((0.065, 0.16, 0.10)),
            origin=Origin(xyz=(x, 0.0, BEAM_Z)),
            material=mats["frame_steel"],
            name=f"buffer_block_{k}",
        )
    # Axle journal boxes hanging under the beams, one per wheel.
    idx = 0
    for sx in (-1.0, 1.0):
        for sy in (1.0, -1.0):
            chassis.visual(
                Box((0.11, 0.070, 0.13)),
                origin=Origin(xyz=(sx * WHEEL_X, sy * 0.185, 0.34)),
                material=mats["frame_steel"],
                name=f"axle_box_{idx}",
            )
            idx += 1
    # Cross axles between the journal boxes.
    for k, sx in enumerate((-1.0, 1.0)):
        chassis.visual(
            Cylinder(radius=0.020, length=0.31),
            origin=Origin(xyz=(sx * WHEEL_X, 0.0, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=mats["frame_steel"],
            name=f"axle_{k}",
        )
    # Raised saddle brackets that carry the higher tipping trunnion pin.
    for k, side in enumerate((1.0, -1.0)):
        chassis.visual(
            Box((0.14, 0.050, 0.065)),
            origin=Origin(xyz=(0.0, side * 0.16, 0.4325)),
            material=mats["frame_steel"],
            name=f"pivot_support_post_{k}",
        )
        chassis.visual(
            Box((0.100, 0.044, 0.065)),
            origin=Origin(xyz=(0.0, side * 0.16, 0.4975)),
            material=mats["frame_steel"],
            name=f"pivot_support_neck_{k}",
        )
        chassis.visual(
            _build_pivot_saddle_mesh(),
            origin=Origin(xyz=(0.0, side * 0.16, PIVOT_Z - 0.035)),
            material=mats["frame_steel"],
            name=f"pivot_bracket_{k}",
        )

    model.articulation(
        "cart_travel",
        ArticulationType.PRISMATIC,
        parent=track,
        child=chassis,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=1.5, lower=-0.30, upper=0.30),
    )

    # --------------------------------------------------------------- wheels
    wheel_positions = [
        (-WHEEL_X, RAIL_Y),
        (-WHEEL_X, -RAIL_Y),
        (WHEEL_X, RAIL_Y),
        (WHEEL_X, -RAIL_Y),
    ]
    for i, (wx, wy) in enumerate(wheel_positions):
        inner = -1.0 if wy > 0 else 1.0  # local Y direction toward track center
        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            Cylinder(radius=WHEEL_R, length=0.050),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=mats["wheel_iron"],
            name="tread",
        )
        wheel.visual(
            Cylinder(radius=0.155, length=0.012),
            origin=Origin(xyz=(0.0, inner * 0.041, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=mats["wheel_iron"],
            name="flange",
        )
        wheel.visual(
            Cylinder(radius=0.045, length=0.030),
            origin=Origin(xyz=(0.0, -inner * 0.032, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=mats["hub_iron"],
            name="hub_cap",
        )
        wheel.visual(
            Cylinder(radius=0.030, length=0.045),
            origin=Origin(xyz=(0.0, inner * 0.0375, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=mats["hub_iron"],
            name="hub_stub",
        )
        model.articulation(
            f"wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=wheel,
            origin=Origin(xyz=(wx, wy, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=12.0),
        )

    # --------------------------------------------------------------- hopper
    hopper = model.part("hopper")
    hopper.visual(
        _build_tub_mesh(),
        material=mats["tub_steel"],
        name="tub_shell",
    )
    # Transverse trunnion pin welded under the tub floor, seated in the saddles.
    hopper.visual(
        Cylinder(radius=TRUNNION_R, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["strap_steel"],
        name="trunnion_pin",
    )
    # Riveted reinforcement: rim angles, horizontal rib and vertical straps.
    for k, side in enumerate((1.0, -1.0)):
        hopper.visual(
            Box((0.98, 0.038, 0.018)),
            origin=Origin(xyz=(0.0, side * 0.345, TUB_RIM_Z + 0.009)),
            material=mats["strap_steel"],
            name=f"rim_angle_{k}",
        )
        hopper.visual(
            Box((0.94, 0.012, 0.050)),
            origin=Origin(xyz=(0.0, side * 0.366, 0.25)),
            material=mats["strap_steel"],
            name=f"rib_strap_{k}",
        )
        for m, x in enumerate((-0.22, 0.22)):
            hopper.visual(
                Box((0.045, 0.012, 0.42)),
                origin=Origin(xyz=(x, side * 0.366, 0.26)),
                material=mats["strap_steel"],
                name=f"seam_strap_{k}_{m}",
            )
    # Retaining straps that hold the shovel shaft against the side wall.
    shovel_pitch = 0.9
    sdir = (sin(shovel_pitch), 0.0, cos(shovel_pitch))
    sbase = (-0.30, 0.08)  # shovel joint origin in hopper local XZ
    for k, t in enumerate((0.40, 0.60)):
        px = sbase[0] + sdir[0] * t
        pz = sbase[1] + sdir[2] * t
        hopper.visual(
            Box((0.060, 0.045, 0.035)),
            origin=Origin(xyz=(px, 0.375, pz), rpy=(0.0, shovel_pitch, 0.0)),
            material=mats["strap_steel"],
            name=f"shovel_strap_{k}",
        )

    model.articulation(
        "hopper_tip",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=hopper,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=1.0, lower=0.0, upper=0.35),
    )

    # --------------------------------------------------------------- shovel
    shovel = model.part("shovel")
    shovel.visual(
        Box((0.20, 0.014, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=mats["blade_metal"],
        name="blade",
    )
    shovel.visual(
        Cylinder(radius=0.014, length=0.62),
        origin=Origin(xyz=(0.0, 0.015, 0.575)),
        material=mats["handle_wood"],
        name="shaft",
    )
    shovel.visual(
        Cylinder(radius=0.016, length=0.16),
        origin=Origin(xyz=(0.0, 0.015, 0.885), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["handle_wood"],
        name="t_grip",
    )
    model.articulation(
        "shovel_mount",
        ArticulationType.FIXED,
        parent=hopper,
        child=shovel,
        origin=Origin(xyz=(sbase[0], 0.3785, sbase[1]), rpy=(0.0, shovel_pitch, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    track = object_model.get_part("rail_track")
    chassis = object_model.get_part("chassis")
    hopper = object_model.get_part("hopper")
    shovel = object_model.get_part("shovel")
    wheels = [object_model.get_part(f"wheel_{i}") for i in range(4)]

    travel = object_model.get_articulation("cart_travel")
    tip = object_model.get_articulation("hopper_tip")
    spins = [object_model.get_articulation(f"wheel_spin_{i}") for i in range(4)]

    # -- intentional captured fits ------------------------------------------
    for i, wheel in enumerate(wheels):
        ctx.allow_overlap(
            chassis,
            wheel,
            reason="wheel hub stub is a captured shaft inserted into its axle journal box",
            elem_a=f"axle_box_{i}",
            elem_b="hub_stub",
        )
    for k in range(2):
        ctx.allow_overlap(
            chassis,
            hopper,
            reason="tipping trunnion pin seats into the chassis saddle bracket",
            elem_a=f"pivot_bracket_{k}",
            elem_b="trunnion_pin",
        )
    ctx.allow_overlap(
        hopper,
        shovel,
        reason="shovel blade rests seated against the riveted rib strap (1 mm seat)",
        elem_a="rib_strap_0",
        elem_b="blade",
    )
    ctx.allow_overlap(
        hopper,
        shovel,
        reason="shovel blade rests seated against the vertical seam strap (1 mm seat)",
        elem_a="seam_strap_0_0",
        elem_b="blade",
    )
    for k in range(2):
        ctx.allow_overlap(
            hopper,
            shovel,
            reason="retaining strap wraps and captures the shovel shaft",
            elem_a=f"shovel_strap_{k}",
            elem_b="shaft",
        )

    # -- joint semantics ----------------------------------------------------
    ctx.check(
        "cart_travel_is_prismatic_along_track",
        travel.articulation_type == ArticulationType.PRISMATIC
        and abs(travel.axis[0]) == 1.0,
        f"type={travel.articulation_type} axis={travel.axis}",
    )
    ctx.check(
        "hopper_tip_is_limited_revolute_about_lateral_axis",
        tip.articulation_type == ArticulationType.REVOLUTE
        and abs(tip.axis[1]) == 1.0
        and tip.motion_limits is not None
        and tip.motion_limits.upper is not None
        and 0.25 <= tip.motion_limits.upper <= 0.4,
        f"type={tip.articulation_type} axis={tip.axis}",
    )
    ctx.check(
        "four_continuous_wheel_spins",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in spins),
        "wheel spin joints must be continuous",
    )

    # -- rest pose: wheels ride the rails, tub sits over the frame ----------
    for i, wheel in enumerate(wheels):
        rail_head = track.get_visual(f"rail_head_{0 if i % 2 == 0 else 1}")
        ctx.expect_gap(
            wheel,
            track,
            axis="z",
            max_gap=0.003,
            max_penetration=0.001,
            elem_a="tread",
            elem_b=rail_head,
            name=f"wheel_{i}_tread_rides_rail_head",
        )
        ctx.expect_contact(
            wheel,
            chassis,
            elem_a="hub_stub",
            elem_b=f"axle_box_{i}",
            name=f"wheel_{i}_hub_engages_axle_box",
        )
    ctx.expect_overlap(hopper, chassis, axes="xy", min_overlap=0.4)
    ctx.expect_gap(
        hopper,
        chassis,
        axis="z",
        max_gap=0.20,
        max_penetration=0.0,
        elem_a="tub_shell",
        elem_b="frame_beam_0",
        name="raised_tub_clears_frame_beams",
    )
    for k in range(2):
        ctx.expect_contact(
            hopper,
            chassis,
            elem_a="trunnion_pin",
            elem_b=f"pivot_bracket_{k}",
            name=f"trunnion_seated_in_saddle_{k}",
        )

    # -- shovel is seated against the tub side, not floating ----------------
    ctx.expect_contact(
        shovel,
        hopper,
        elem_a="blade",
        elem_b="rib_strap_0",
        name="shovel_blade_rests_on_tub_side_strap",
    )
    ctx.expect_contact(
        shovel,
        hopper,
        elem_a="shaft",
        elem_b="shovel_strap_0",
        name="shovel_shaft_held_by_strap",
    )

    # -- decisive articulated poses ------------------------------------------
    with ctx.pose({travel: 0.28}):
        ctx.expect_overlap(
            chassis,
            track,
            axes="x",
            min_overlap=0.5,
            name="cart_stays_on_track_at_travel_limit",
        )
    with ctx.pose({tip: 0.3}):
        ctx.expect_overlap(
            hopper,
            chassis,
            axes="y",
            min_overlap=0.3,
            name="tipped_tub_still_carried_by_chassis",
        )

    return ctx.report()


object_model = build_object_model()
