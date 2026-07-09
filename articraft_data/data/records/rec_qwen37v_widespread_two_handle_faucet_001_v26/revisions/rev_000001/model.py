from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Deck-mounted widespread two-handle faucet in polished gold brass.
#
# Frame conventions:
#   - The deck surface is the horizontal XY plane at z = 0.
#   - The faucet rises above the deck along +Z.
#   - The bridge bar runs along X, linking the three posts.
#   - The spout projects forward along +Y, then curves downward.
#   - Viewer-left is world -X.
# ---------------------------------------------------------------------------

# Layout
POST_PITCH_X = 0.10  # post centers at x = 0, +/-0.10
BRIDGE_BAR_W = 0.24  # bridge bar total width
BRIDGE_BAR_H = 0.012  # bridge bar thickness (height)
BRIDGE_BAR_D = 0.020  # bridge bar depth (front-to-back)
DECK_SEAM_R = 0.024   # seam ring outer radius at deck base
DECK_SEAM_T = 0.002   # seam ring thickness

# Spout
SPOUT_TUBE_R = 0.015
SPOUT_BORE_R = 0.0105
SPOUT_RISE = 0.12        # vertical rise from bridge bar top
SPOUT_REACH_Y = 0.14     # forward projection along +Y
SPOUT_BEND_R = 0.05
SPOUT_DROP_Z = -0.08     # outlet below the spout peak

# Spout escutcheon (post base)
SPOUT_ESC_R = 0.022
SPOUT_ESC_H = 0.020

# Valve posts
VALVE_ESC_R = 0.022
VALVE_ESC_H = 0.020
VALVE_BODY_R = 0.014
VALVE_BODY_H = 0.030

# Cross handle (rotates about vertical axis for deck mount)
HANDLE_ROD_R = 0.004
HANDLE_ROD_LEN = 0.090  # tip-to-tip
HUB_R = 0.012
HUB_H = 0.018
KNURL_R = 0.013
STEM_R = 0.006

# Diverter knob
DIV_R = 0.010
DIV_H = 0.018
DIV_STEM_R = 0.005
DIV_STEM_H = 0.015
DIV_TRAVEL = 0.030  # max prismatic travel

# Computed for hollow-bore verification
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_spout_solid() -> cq.Workplane:
    """Spout in its local frame: base at origin, rises +Z, then curves
    forward (+Y) and downward to a hollow open outlet."""
    # Path: rise straight up, then arc forward and down
    peak_z = SPOUT_RISE
    arc_start = (0.0, peak_z)
    mid = (SPOUT_REACH_Y * 0.5, peak_z - SPOUT_BEND_R * 0.3)
    arc_end = (SPOUT_REACH_Y, peak_z - SPOUT_BEND_R)

    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, peak_z - SPOUT_BEND_R)
        .threePointArc(mid, arc_end)
        .lineTo(SPOUT_REACH_Y, SPOUT_DROP_Z + peak_z - SPOUT_BEND_R)
    )
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, -0.003)
        .lineTo(0.0, peak_z - SPOUT_BEND_R)
        .threePointArc(mid, arc_end)
        .lineTo(SPOUT_REACH_Y, SPOUT_DROP_Z + peak_z - SPOUT_BEND_R - 0.003)
    )

    # "XY" workplanes have +Z normals, matching the rise direction
    tube = cq.Workplane("XY").circle(SPOUT_TUBE_R).sweep(path)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )
    # Base escutcheon post
    esc = cq.Workplane("XY").circle(SPOUT_ESC_R).extrude(SPOUT_ESC_H)

    unbored = tube.union(esc)
    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME, SPOUT_UNBORED_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()
    return solid


def _build_hub_solid() -> cq.Workplane:
    """Cross-handle hub in the handle frame: axis +Z (vertical for deck mount),
    base at z=0, knurled band and domed cap on top."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_H)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.010)
    )
    dome = cq.Workplane("XY").workplane(offset=HUB_H - 0.002).sphere(0.011)
    return hub.union(knurl).union(dome)


def _build_valve_post(valve, gold, seam_mat) -> None:
    """Valve post: escutcheon base with deck seam, valve body rising up."""
    # Deck seam ring (dark accent)
    valve.visual(
        Cylinder(radius=DECK_SEAM_R, length=DECK_SEAM_T),
        origin=Origin(xyz=(0.0, 0.0, DECK_SEAM_T / 2.0)),
        material=seam_mat,
        name="deck_seam",
    )
    # Escutcheon post
    valve.visual(
        Cylinder(radius=VALVE_ESC_R, length=VALVE_ESC_H),
        origin=Origin(xyz=(0.0, 0.0, DECK_SEAM_T + VALVE_ESC_H / 2.0)),
        material=gold,
        name="escutcheon",
    )
    # Valve body cylinder
    valve.visual(
        Cylinder(radius=VALVE_BODY_R, length=VALVE_BODY_H),
        origin=Origin(
            xyz=(0.0, 0.0, DECK_SEAM_T + VALVE_ESC_H + VALVE_BODY_H / 2.0)
        ),
        material=gold,
        name="valve_body",
    )


def _add_handle_visuals(handle, hub_mesh, gold) -> None:
    """Four-arm cross handle rotating about vertical Z axis (deck mount).
    Joint frame sits at the top of the valve body."""
    # Stem seats down into the valve body
    handle.visual(
        Cylinder(radius=STEM_R, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=gold,
        name="stem",
    )
    handle.visual(hub_mesh, material=gold, name="hub")
    # Two crossing spoke rods in the XY plane
    half = HANDLE_ROD_LEN / 2.0
    spoke_z = HUB_H * 0.5
    # Along X
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="spokes_x",
    )
    # Along Y
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, spoke_z), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)
        ),
        material=gold,
        name="spokes_y",
    )
    # Rounded spoke tips
    for name, (dx, dy) in (
        ("tip_px", (half, 0.0)),
        ("tip_nx", (-half, 0.0)),
        ("tip_py", (0.0, half)),
        ("tip_ny", (0.0, -half)),
    ):
        handle.visual(
            Sphere(radius=HANDLE_ROD_R),
            origin=Origin(xyz=(dx, dy, spoke_z)),
            material=gold,
            name=name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.25, 0.20, 0.10, 1.0))

    # --- Bridge bar (root): horizontal bar linking three posts ---
    bridge = model.part("bridge_bar")
    bridge.visual(
        Box((BRIDGE_BAR_W, BRIDGE_BAR_D, BRIDGE_BAR_H)),
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_BAR_H / 2.0)),
        material=gold,
        name="bar",
    )
    # Decorative end caps
    for sx in (-1.0, 1.0):
        bridge.visual(
            Sphere(radius=BRIDGE_BAR_H * 0.6),
            origin=Origin(xyz=(sx * BRIDGE_BAR_W / 2.0, 0.0, BRIDGE_BAR_H / 2.0)),
            material=gold,
            name=f"end_cap_{'left' if sx < 0 else 'right'}",
        )

    # --- Central spout post (fixed to bridge bar) ---
    spout = model.part("center_spout")
    # Deck seam ring (dark accent at base)
    spout.visual(
        Cylinder(radius=DECK_SEAM_R, length=DECK_SEAM_T),
        origin=Origin(xyz=(0.0, 0.0, DECK_SEAM_T / 2.0)),
        material=seam_dark,
        name="deck_seam",
    )
    spout.visual(
        mesh_from_cadquery(_build_spout_solid(), "spout"),
        origin=Origin(xyz=(0.0, 0.0, DECK_SEAM_T)),
        material=gold,
        name="spout_body",
    )
    model.articulation(
        "bridge_to_spout",
        ArticulationType.FIXED,
        parent=bridge,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_BAR_H)),
    )

    # --- Valve posts and cross handles ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")
    valve_top_z = DECK_SEAM_T + VALVE_ESC_H + VALVE_BODY_H

    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _build_valve_post(valve, gold, seam_dark)
        model.articulation(
            f"bridge_to_{side}_valve",
            ArticulationType.FIXED,
            parent=bridge,
            child=valve,
            origin=Origin(xyz=(sx * POST_PITCH_X, 0.0, BRIDGE_BAR_H)),
        )

        handle = model.part(f"{side}_handle")
        _add_handle_visuals(handle, hub_mesh, gold)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            # Joint at top of valve body, axis is vertical (+Z)
            origin=Origin(xyz=(0.0, 0.0, valve_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    # --- Diverter knob (prismatic, behind spout) ---
    diverter = model.part("diverter")
    # Diverter body: small cylindrical knob (vertical, user-facing)
    diverter.visual(
        Cylinder(radius=DIV_R, length=DIV_H),
        origin=Origin(xyz=(0.0, 0.0, DIV_H / 2.0)),
        material=gold,
        name="knob",
    )
    # Horizontal stem extending toward spout center (+Y in diverter frame)
    stem_len = SPOUT_ESC_R + 0.008  # reaches well into the spout post
    diverter.visual(
        Cylinder(radius=DIV_STEM_R, length=stem_len),
        origin=Origin(
            xyz=(0.0, stem_len / 2.0, DIV_H * 0.4),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gold,
        name="stem",
    )
    # Small indicator ridge on top
    diverter.visual(
        Box((DIV_R * 1.6, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, DIV_H + 0.0015)),
        material=gold,
        name="indicator",
    )
    model.articulation(
        "spout_to_diverter",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=diverter,
        # Behind the spout (negative Y), at escutcheon mid-height
        origin=Origin(
            xyz=(0.0, -(SPOUT_ESC_R + DIV_R + 0.002), DECK_SEAM_T + SPOUT_ESC_H * 0.5)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=0.5, lower=0.0, upper=DIV_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bridge = object_model.get_part("bridge_bar")
    spout = object_model.get_part("center_spout")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    diverter = object_model.get_part("diverter")

    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")
    div_joint = object_model.get_articulation("spout_to_diverter")

    # --- Bridge bar exists and links all three posts ---
    bridge_aabb = ctx.part_world_aabb(bridge)
    assert bridge_aabb is not None
    (bx0, by0, bz0), (bx1, by1, bz1) = bridge_aabb
    ctx.check(
        "bridge_bar_spans_posts",
        (bx1 - bx0) >= 0.22,
        f"bridge width={bx1 - bx0:.3f}",
    )
    ctx.check(
        "bridge_bar_above_deck",
        bz0 >= -0.003 and bz1 < 0.05,
        f"bridge z=({bz0:.3f}, {bz1:.3f})",
    )

    # --- Handle joints are revolute about vertical axis ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_is_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - 1.0) < 1e-9,
            f"axis={ax}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_full_turn_range",
            lim is not None
            and abs(lim.lower + math.pi) < 1e-6
            and abs(lim.upper - math.pi) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # --- Diverter is prismatic along Z ---
    ctx.check(
        "diverter_is_prismatic",
        str(div_joint.joint_type).lower().endswith("prismatic"),
        f"type={div_joint.joint_type}",
    )
    div_ax = div_joint.axis
    ctx.check(
        "diverter_axis_vertical",
        abs(div_ax[0]) < 1e-9 and abs(div_ax[1]) < 1e-9 and abs(div_ax[2] - 1.0) < 1e-9,
        f"axis={div_ax}",
    )
    div_lim = div_joint.motion_limits
    ctx.check(
        "diverter_travel_range",
        div_lim is not None
        and abs(div_lim.lower) < 1e-6
        and 0.020 <= div_lim.upper <= 0.040,
        f"limits=({div_lim.lower}, {div_lim.upper})",
    )

    # --- Diverter slides up when actuated ---
    div_rest = ctx.part_world_position(diverter)
    with ctx.pose({div_joint: DIV_TRAVEL}):
        div_up = ctx.part_world_position(diverter)
    assert div_rest is not None and div_up is not None
    ctx.check(
        "diverter_slides_up",
        div_up[2] > div_rest[2] + 0.015,
        f"rest z={div_rest[2]:.4f}, up z={div_up[2]:.4f}",
    )

    # --- Spout is hollow (bore volume check) ---
    ctx.check(
        "spout_tube_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
    )

    # --- Deck seams exist at all three post bases ---
    for name in ("center_spout", "left_valve", "right_valve"):
        post = object_model.get_part(name)
        seam_vis = post.get_visual("deck_seam")
        ctx.check(
            f"{name}_has_deck_seam",
            seam_vis is not None,
            f"part {name} missing deck_seam visual",
        )

    # --- Posts flank the spout symmetrically ---
    lp = ctx.part_world_position(left_valve)
    rp = ctx.part_world_position(right_valve)
    sp = ctx.part_world_position(spout)
    assert lp is not None and rp is not None and sp is not None
    ctx.check(
        "three_post_widespread_layout",
        abs(lp[0] + POST_PITCH_X) < 1e-3
        and abs(rp[0] - POST_PITCH_X) < 1e-3
        and abs(sp[0]) < 1e-3,
        f"left={lp[0]:.3f}, center={sp[0]:.3f}, right={rp[0]:.3f}",
    )

    # --- Handle stems embed in valve bodies ---
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("valve_body"),
        reason="handle stem is seated inside the valve body bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("valve_body"),
        reason="handle stem is seated inside the valve body bore and turns with the handle",
    )

    # --- Diverter stem embeds in spout post ---
    ctx.allow_overlap(
        diverter,
        spout,
        elem_a=diverter.get_visual("stem"),
        elem_b=spout.get_visual("spout_body"),
        reason="diverter stem slides inside the spout post bore",
    )

    # --- Handle rotation proof ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        # At 45 deg, the cross should have a different footprint than at rest
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        ctx.check(
            "left_handle_stays_on_valve_axis_while_rotating",
            abs(cen[0] - lp[0]) < 1e-3 and abs(cen[1] - lp[1]) < 1e-3,
            f"handle origin={cen}, valve={lp}",
        )

    # --- Overall width about 0.30 m ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.26 <= total_w <= 0.34,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- Bridge bar grounded at deck level ---
    ctx.check(
        "bridge_bar_grounded_at_deck",
        abs(bz0) < 0.005,
        f"bridge zmin={bz0:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
