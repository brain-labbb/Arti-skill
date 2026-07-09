from __future__ import annotations

# Tall utility cabinet with segmented roller shutter (dark charcoal steel).
#
# World layout: the cabinet front faces +X; width is along Y; height along +Z.
# The base plinth rests at z=0. The carcass is a hollow shell (back wall, two
# side walls, top, bottom) enclosing a cavity. The front opening is divided
# into an upper fixed panel and a lower roller-shutter zone. The shutter is a
# stack of thin horizontal slats with a bottom lift bar that slides vertically
# upward in side guide channels on a PRISMATIC joint.
#
# Primary articulation: the roller shutter (PRISMATIC, +Z), sliding up in the
# side channels to reveal the lower portion of the front opening.

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_shutter_slat(part, index, *, slat_depth, slat_width, slat_h,
                      center_z, material):
    """Add one interlocking roller-shutter slat visual to *part*.

    The slat is a thin horizontal panel centred at local (0, 0, center_z).
    Adjacent slats are expected to overlap vertically by ~2 mm to mimic the
    hook-profile interlock of a real roller shutter.
    """
    part.visual(
        Box((slat_depth, slat_width, slat_h)),
        origin=Origin(xyz=(0.0, 0.0, center_z)),
        material=material,
        name=f"slat_{index}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roller_shutter_utility_cabinet")

    # ---------- materials ----------
    charcoal = model.material("charcoal_steel", rgba=(0.15, 0.16, 0.18, 1.0))
    charcoal_dark = model.material("charcoal_dark", rgba=(0.075, 0.080, 0.090, 1.0))
    interior_dark = model.material("interior_dark_steel", rgba=(0.105, 0.110, 0.125, 1.0))
    channel_grey = model.material("channel_grey", rgba=(0.12, 0.12, 0.14, 1.0))
    lift_bar_dark = model.material("lift_bar_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    badge_red = model.material("badge_red", rgba=(0.78, 0.20, 0.13, 1.0))

    # ---------- key dimensions (meters) ----------
    W = 0.380          # width (Y)
    D = 0.600          # depth (X), front at x=D, back at x=0
    H = 1.300          # total height (Z)
    wall = 0.014       # carcass wall thickness
    plinth_h = 0.060   # recessed base plinth height
    plinth_inset = 0.020

    # ===================================================================
    # ROOT: cabinet carcass (shell)
    # ===================================================================
    cab = model.part("cabinet")

    # Recessed base plinth (the cabinet stands on this).
    cab.visual(
        Box((D - 2 * plinth_inset, W - 2 * plinth_inset, plinth_h)),
        origin=Origin(xyz=(D / 2.0, 0.0, plinth_h / 2.0)),
        material=charcoal_dark,
        name="base_plinth",
    )

    body_bottom_z = plinth_h
    body_h = H - plinth_h

    # Bottom panel of the carcass (on the plinth).
    cab.visual(
        Box((D, W, wall)),
        origin=Origin(xyz=(D / 2.0, 0.0, body_bottom_z + wall / 2.0)),
        material=charcoal,
        name="bottom_panel",
    )
    # Top panel.
    cab.visual(
        Box((D, W, wall)),
        origin=Origin(xyz=(D / 2.0, 0.0, H - wall / 2.0)),
        material=charcoal,
        name="top_panel",
    )
    # Back wall.
    cab.visual(
        Box((wall, W, body_h)),
        origin=Origin(xyz=(wall / 2.0, 0.0, body_bottom_z + body_h / 2.0)),
        material=charcoal,
        name="back_wall",
    )
    # Two side walls.
    for s, tag in ((1, "left"), (-1, "right")):
        cab.visual(
            Box((D, wall, body_h)),
            origin=Origin(xyz=(D / 2.0, s * (W / 2.0 - wall / 2.0),
                               body_bottom_z + body_h / 2.0)),
            material=charcoal,
            name=f"side_wall_{tag}",
        )

    # ---------- front opening layout ----------
    open_top_z = H - wall              # under the top panel
    open_bot_z = body_bottom_z + wall  # above the bottom panel
    opening_h = open_top_z - open_bot_z
    inner_W = W - 2 * wall

    # The front opening is divided into two zones:
    #   upper ~45 %: fixed sheet-metal front panel (covers the top portion)
    #   lower ~55 %: roller shutter zone (sliding slats cover this when closed)
    fixed_panel_ratio = 0.45
    fixed_panel_h = fixed_panel_ratio * opening_h
    shutter_zone_h = opening_h - fixed_panel_h

    # Side guide channels (vertical U-tracks for the shutter slat edges).
    channel_w = 0.010    # protrusion from inner wall face into the opening
    channel_d = 0.018    # depth in X (near the front face)
    for s, tag in ((1, "left"), (-1, "right")):
        cab.visual(
            Box((channel_d, channel_w, opening_h)),
            origin=Origin(xyz=(D - channel_d / 2.0,
                               s * (W / 2.0 - wall - channel_w / 2.0),
                               open_bot_z + opening_h / 2.0)),
            material=channel_grey,
            name=f"side_channel_{tag}",
        )

    # Fixed upper front panel (slightly proud of the shutter plane so that
    # raised slats pass behind it).
    fixed_panel_thk = 0.008
    fixed_panel_bot_z = open_top_z - fixed_panel_h
    cab.visual(
        Box((fixed_panel_thk, inner_W - 0.004, fixed_panel_h)),
        origin=Origin(xyz=(D - fixed_panel_thk / 2.0, 0.0,
                           fixed_panel_bot_z + fixed_panel_h / 2.0)),
        material=charcoal,
        name="fixed_front_panel",
    )

    # Meeting rail: horizontal strip at the boundary between the fixed panel
    # and the shutter zone, acting as a visual divider and a travel stop cue.
    rail_h = 0.010
    cab.visual(
        Box((0.012, inner_W - 0.004, rail_h)),
        origin=Origin(xyz=(D - 0.006, 0.0, fixed_panel_bot_z - rail_h / 2.0)),
        material=charcoal_dark,
        name="meeting_rail",
    )

    # Top fascia: a short valance across the top front edge that hides the
    # accumulated slats when the shutter is fully raised.
    fascia_h = 0.030
    cab.visual(
        Box((0.020, inner_W - 0.004, fascia_h)),
        origin=Origin(xyz=(D - 0.010, 0.0, open_top_z - fascia_h / 2.0)),
        material=charcoal_dark,
        name="top_fascia",
    )

    # Bottom threshold / sill where the shutter rests when closed.
    threshold_h = 0.008
    cab.visual(
        Box((0.020, inner_W - 0.004, threshold_h)),
        origin=Origin(xyz=(D - 0.010, 0.0, open_bot_z + threshold_h / 2.0)),
        material=charcoal_dark,
        name="bottom_threshold",
    )

    # Interior back panel visible through the opening (darker finish).
    # Positioned to overlap the back wall slightly so the panel reads as
    # bonded to the carcass shell rather than floating inside.
    cab.visual(
        Box((0.004, inner_W - 0.010, opening_h)),
        origin=Origin(xyz=(wall + 0.001, 0.0, open_bot_z + opening_h / 2.0)),
        material=interior_dark,
        name="interior_back",
    )

    # Maker badge near the top front edge (red, like the reference image).
    cab.visual(
        Box((0.006, 0.055, 0.022)),
        origin=Origin(xyz=(D + 0.001, -0.105, H - 0.030)),
        material=badge_red,
        name="top_badge",
    )

    cab.inertial = Inertial.from_geometry(Box((D, W, H)), mass=35.0)

    # ===================================================================
    # ROLLER SHUTTER (PRISMATIC, slides upward along +Z)
    # ===================================================================
    shutter = model.part("shutter")

    # Shutter geometry parameters.
    # Each slat is slightly taller than the pitch so adjacent slats overlap
    # by ~2 mm, mimicking the interlocking hook profile of a real roller
    # shutter and ensuring the slat stack reads as one connected assembly.
    slat_h = 0.028          # slat height (taller than pitch for interlock)
    slat_pitch = 0.026      # vertical spacing between slat centers
    slat_depth = 0.005      # slat thickness in X
    slat_width = inner_W - 2 * channel_w - 0.010  # fits between channels
    lift_bar_h = 0.038      # bottom lift bar height
    lift_bar_depth = 0.014  # lift bar thickness in X (proud for grip)

    # Number of slats to fill the shutter zone below the fixed panel.
    available_for_slats = shutter_zone_h - lift_bar_h
    n_slats = int(available_for_slats / slat_pitch)

    # The shutter part frame sits at the slat x-center; the lift bar is
    # offset forward (+X local) so its front face is flush with the cabinet
    # front. The slats are slightly recessed behind the fixed front panel
    # so raised slats pass behind it without 3D overlap.
    slat_x_world = D - 0.012       # slat center in world X
    lift_bar_x_local = 0.005       # offset from part frame to lift bar center

    # Bottom lift bar (grab handle at the base of the shutter).
    shutter.visual(
        Box((lift_bar_depth, slat_width + 0.006, lift_bar_h)),
        origin=Origin(xyz=(lift_bar_x_local, 0.0, 0.0)),
        material=lift_bar_dark,
        name="lift_bar",
    )

    # Slat stack: each slat is a thin horizontal panel that overlaps the
    # one below it by ~2 mm, mimicking the interlocking hook profile of a
    # real roller shutter. The first slat also overlaps the lift bar.
    slat_base_z = lift_bar_h / 2.0 - 0.002 + slat_h / 2.0
    for i in range(n_slats):
        _add_shutter_slat(
            shutter, i,
            slat_depth=slat_depth, slat_width=slat_width, slat_h=slat_h,
            center_z=slat_base_z + i * slat_pitch,
            material=charcoal,
        )

    shutter.inertial = Inertial.from_geometry(
        Box((slat_depth, slat_width, shutter_zone_h)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, shutter_zone_h / 2.0)),
    )

    # Prismatic joint: shutter slides upward along +Z in the side channels.
    # Joint origin places the shutter part frame so that the lift bar rests
    # just above the bottom threshold when closed (q = 0).
    joint_z = open_bot_z + threshold_h + 0.002 + lift_bar_h / 2.0

    # Travel: the shutter reveals roughly half the opening. The raised slats
    # stay within the cabinet height (below the top panel).
    travel = 0.50

    model.articulation(
        "cabinet_to_shutter",
        ArticulationType.PRISMATIC,
        parent=cab,
        child=shutter,
        origin=Origin(xyz=(slat_x_world, 0.0, joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0, velocity=0.3, lower=0.0, upper=travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cab = object_model.get_part("cabinet")
    shutter = object_model.get_part("shutter")
    joint = object_model.get_articulation("cabinet_to_shutter")

    # --- Cabinet is tall and upright, base rests at floor (z ≈ 0) ---
    cb = ctx.part_world_aabb(cab)
    assert cb is not None
    ctx.check(
        "base_at_floor",
        abs(cb[0][2]) < 0.005,
        details=f"cabinet min z={cb[0][2]:.4f}",
    )
    ctx.check(
        "tall_cabinet",
        cb[1][2] > 1.2,
        details=f"cabinet top z={cb[1][2]:.3f}",
    )
    width_y = cb[1][1] - cb[0][1]
    height_z = cb[1][2] - cb[0][2]
    ctx.check(
        "upright_proportions",
        height_z > width_y * 2.5,
        details=f"h={height_z:.3f}, w={width_y:.3f}",
    )

    # --- Carcass retains the same shell structure as the parent family ---
    required_shell = {
        "base_plinth", "bottom_panel", "top_panel",
        "back_wall", "side_wall_left", "side_wall_right",
    }
    cab_visuals = {v.name for v in cab.visuals}
    ctx.check(
        "carcass_shell_panels",
        required_shell.issubset(cab_visuals),
        details=f"missing={required_shell - cab_visuals}",
    )

    # --- Side guide channels are present for the shutter ---
    ctx.check(
        "side_channels_present",
        "side_channel_left" in cab_visuals
        and "side_channel_right" in cab_visuals,
        details=f"visuals={sorted(cab_visuals)}",
    )

    # --- Fixed upper front panel covers part of the opening ---
    ctx.check(
        "fixed_front_panel",
        "fixed_front_panel" in cab_visuals,
        details=f"visuals={sorted(cab_visuals)}",
    )

    # --- Shutter is a segmented stack of slats with a bottom lift bar ---
    shutter_visuals = {v.name for v in shutter.visuals}
    ctx.check(
        "shutter_has_lift_bar",
        "lift_bar" in shutter_visuals,
        details=f"visuals={sorted(shutter_visuals)}",
    )
    slat_names = [f"slat_{i}" for i in range(20)]
    ctx.check(
        "shutter_has_segmented_slats",
        all(name in shutter_visuals for name in slat_names),
        details=f"found {sum(1 for n in slat_names if n in shutter_visuals)}/20 slats",
    )

    # --- Joint axis is vertical (+Z) ---
    ctx.check(
        "joint_axis_vertical",
        abs(joint.axis[2]) > 0.99,
        details=f"axis={joint.axis}",
    )

    # --- Shutter slides upward when the joint is opened ---
    rest_pos = ctx.part_world_position(shutter)
    assert rest_pos is not None
    with ctx.pose({joint: joint.motion_limits.upper}):
        open_pos = ctx.part_world_position(shutter)
        # The lift bar should be well above its rest position.
        lift_open = ctx.part_element_world_aabb(shutter, elem="lift_bar")
    assert open_pos is not None and lift_open is not None
    ctx.check(
        "shutter_slides_upward",
        open_pos[2] > rest_pos[2] + 0.20,
        details=f"rest z={rest_pos[2]:.3f}, open z={open_pos[2]:.3f}",
    )

    # --- At full open, the bottom of the lift bar clears a meaningful
    #     portion of the opening (at least 0.30 m above the threshold) ---
    threshold_top_z = cb[0][2] + 0.060 + 0.014 + 0.008  # plinth + floor + threshold
    ctx.check(
        "shutter_reveals_opening",
        lift_open[0][2] > threshold_top_z + 0.30,
        details=f"lift_bar_bottom={lift_open[0][2]:.3f}, threshold={threshold_top_z:.3f}",
    )

    # --- Shutter stays within the cabinet footprint on XY ---
    ctx.expect_within(
        shutter, cab, axes="xy", margin=0.015,
        name="shutter_within_cabinet_footprint",
    )

    return ctx.report()


object_model = build_object_model()
