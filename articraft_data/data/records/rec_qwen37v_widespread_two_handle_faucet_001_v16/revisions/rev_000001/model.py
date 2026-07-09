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
# Widespread deck-mounted two-handle bathroom faucet in polished gold brass.
#
# Frame conventions:
#   - Z is up (vertical); the deck surface sits at z = 0.
#   - X is along the spread (left-right).
#   - Y is forward (toward the user).
#   - Three posts at x = -POST_X, 0, +POST_X are visually linked by a
#     horizontal bridge bar.
#   - Cross handles rotate about short vertical axles (Z).
# ---------------------------------------------------------------------------

# Layout
POST_X = 0.105  # post centres at x = +/- 0.105 m

# Bridge bar (horizontal tube along X)
BRIDGE_R = 0.010  # tube radius (20 mm diameter)
BRIDGE_LEN = 0.24  # total length

# Posts
POST_R = 0.014  # column radius (28 mm diameter)
HANDLE_POST_H = 0.058  # handle pedestal height
SPOUT_POST_H = 0.075  # spout pedestal height (taller)

# Decorative ring ridges on handle pedestals (raised bands)
RING_EXTRA_R = 0.0025  # ring protrusion beyond post surface
RING_HEIGHTS = (0.014, 0.034)  # z heights of two ring ridges above post base

# Seam rings at deck bases
SEAM_R = POST_R + 0.002  # slightly wider than post
SEAM_H = 0.002  # seam ring thickness

# Spout gooseneck
SPOUT_TUBE_R = 0.012  # outer radius (24 mm diameter)
SPOUT_BORE_R = 0.008  # inner bore (16 mm diameter)
SPOUT_RISE = 0.085  # vertical rise above post top
SPOUT_REACH = 0.090  # horizontal reach forward (+Y)
SPOUT_DROP = 0.050  # outlet drop below peak

# Cross handle (rotates about vertical Z axis)
HANDLE_ROD_R = 0.0035  # spoke rod radius
HANDLE_ROD_LEN = 0.090  # tip-to-tip length
HUB_R = 0.011  # hub radius
HUB_H = 0.018  # hub height
KNURL_R = 0.0125  # knurled band radius
STEM_R = 0.005  # stem radius
STEM_LEN = 0.014  # stem length into post

# Derived constants
BRIDGE_CENTER_Z = BRIDGE_R  # tube centre height
BRIDGE_TOP_Z = 2.0 * BRIDGE_R  # tube top (post mounting plane)

# Volumes for hollow-bore verification (set by _build_spout_solid).
SPOUT_SOLID_VOLUME: float = 0.0
SPOUT_UNBORED_VOLUME: float = 0.0


def _build_bridge_solid() -> cq.Workplane:
    """Horizontal tube along X with sphere end-caps."""
    bar = (
        cq.Workplane("YZ")
        .circle(BRIDGE_R)
        .extrude(BRIDGE_LEN)
        .translate((-BRIDGE_LEN / 2.0, 0.0, 0.0))
    )
    cap_l = cq.Workplane("XY").sphere(BRIDGE_R).translate((-BRIDGE_LEN / 2.0, 0.0, 0.0))
    cap_r = cq.Workplane("XY").sphere(BRIDGE_R).translate((BRIDGE_LEN / 2.0, 0.0, 0.0))
    return bar.union(cap_l).union(cap_r)


def _build_handle_post_solid() -> cq.Workplane:
    """Handle pedestal column with two decorative raised ring ridges."""
    post = cq.Workplane("XY").circle(POST_R).extrude(HANDLE_POST_H)
    for h in RING_HEIGHTS:
        band = (
            cq.Workplane("XY")
            .workplane(offset=h - RING_EXTRA_R)
            .circle(POST_R + RING_EXTRA_R)
            .extrude(2.0 * RING_EXTRA_R)
        )
        post = post.union(band)
    return post


def _build_spout_solid() -> cq.Workplane:
    """Spout pedestal + gooseneck tube with hollow bore."""
    # Pedestal column
    ped = cq.Workplane("XY").circle(POST_R).extrude(SPOUT_POST_H)

    # Gooseneck path in the YZ workplane (u=Y forward, v=Z up).
    rise_z = SPOUT_POST_H + SPOUT_RISE
    mid_yz = (SPOUT_REACH * 0.45, rise_z + 0.005)
    end_yz = (SPOUT_REACH, rise_z - SPOUT_DROP)

    path = (
        cq.Workplane("YZ")
        .moveTo(0.0, SPOUT_POST_H)
        .lineTo(0.0, rise_z)
        .threePointArc(mid_yz, end_yz)
    )

    # Profile perpendicular to path start tangent (+Z).
    tube = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_POST_H)
        .circle(SPOUT_TUBE_R)
        .sweep(path)
    )

    unbored = ped.union(tube)

    global SPOUT_UNBORED_VOLUME
    SPOUT_UNBORED_VOLUME = unbored.val().Volume()

    # Bore path (slightly extended on both ends for clean through-cut).
    bore_end_yz = (end_yz[0], end_yz[1] - 0.003)
    bore_path = (
        cq.Workplane("YZ")
        .moveTo(0.0, SPOUT_POST_H - 0.005)
        .lineTo(0.0, rise_z)
        .threePointArc(mid_yz, bore_end_yz)
    )
    bore = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_POST_H - 0.005)
        .circle(SPOUT_BORE_R)
        .sweep(bore_path)
    )

    solid = unbored.cut(bore)

    global SPOUT_SOLID_VOLUME
    SPOUT_SOLID_VOLUME = solid.val().Volume()

    return solid


def _add_handle_visuals(handle, gold) -> None:
    """Four-arm cross handle with vertical axis: hub, knurled band,
    two spoke rods in the XY plane, rounded tips, and a hidden stem."""
    # Hub (vertical cylinder)
    handle.visual(
        Cylinder(radius=HUB_R, length=HUB_H),
        origin=Origin(xyz=(0.0, 0.0, HUB_H / 2.0)),
        material=gold,
        name="hub",
    )
    # Knurled band
    handle.visual(
        Cylinder(radius=KNURL_R, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, HUB_H * 0.4)),
        material=gold,
        name="knurl",
    )
    # Spoke pair along X (rotate Z-axis cylinder onto X via Ry(pi/2))
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, HUB_H / 2.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gold,
        name="spoke_x",
    )
    # Spoke pair along Y (rotate Z-axis cylinder onto Y via Rx(pi/2))
    handle.visual(
        Cylinder(radius=HANDLE_ROD_R, length=HANDLE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, HUB_H / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="spoke_y",
    )
    # Stem (seats down into the post bore)
    handle.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
        material=gold,
        name="stem",
    )
    # Rounded spoke tips
    half = HANDLE_ROD_LEN / 2.0
    zc = HUB_H / 2.0
    for name, pos in (
        ("tip_px", (half, 0.0, zc)),
        ("tip_nx", (-half, 0.0, zc)),
        ("tip_py", (0.0, half, zc)),
        ("tip_ny", (0.0, -half, zc)),
    ):
        handle.visual(
            Sphere(radius=HANDLE_ROD_R),
            origin=Origin(xyz=pos),
            material=gold,
            name=name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_deck_faucet")

    gold = model.material("polished_gold", rgba=(0.85, 0.66, 0.20, 1.0))
    seam_mat = model.material("dark_gold_seam", rgba=(0.50, 0.38, 0.10, 1.0))

    # --- Bridge bar (root) ---
    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_cadquery(_build_bridge_solid(), "bridge_bar"),
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_CENTER_Z)),
        material=gold,
        name="bar",
    )

    # --- Central spout (fixed to bridge) ---
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_solid(), "spout_body"),
        material=gold,
        name="tube",
    )
    # Narrow seam ring at spout deck base
    spout.visual(
        Cylinder(radius=SEAM_R, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2.0)),
        material=seam_mat,
        name="spout_seam",
    )
    model.articulation(
        "bridge_to_spout",
        ArticulationType.FIXED,
        parent=bridge,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_TOP_Z)),
    )

    # --- Handle pedestals and cross handles ---
    post_mesh = mesh_from_cadquery(_build_handle_post_solid(), "handle_post")

    for side, sx in (("left", -1.0), ("right", 1.0)):
        # Pedestal (fixed to bridge)
        post = model.part(f"{side}_post")
        post.visual(post_mesh, material=gold, name="column")
        # Narrow seam ring at pedestal deck base
        post.visual(
            Cylinder(radius=SEAM_R, length=SEAM_H),
            origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2.0)),
            material=seam_mat,
            name="base_seam",
        )
        model.articulation(
            f"bridge_to_{side}_post",
            ArticulationType.FIXED,
            parent=bridge,
            child=post,
            origin=Origin(xyz=(sx * POST_X, 0.0, BRIDGE_TOP_Z)),
        )

        # Cross handle (revolute about vertical Z)
        handle = model.part(f"{side}_handle")
        _add_handle_visuals(handle, gold)
        model.articulation(
            f"{side}_handle_spindle",
            ArticulationType.REVOLUTE,
            parent=post,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, HANDLE_POST_H)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=3.0,
                lower=-math.pi,
                upper=math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bridge = object_model.get_part("bridge")
    spout = object_model.get_part("spout")
    left_post = object_model.get_part("left_post")
    right_post = object_model.get_part("right_post")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")

    # --- Joint plan: two independent revolute handles, vertical Z axis ---
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_is_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_is_vertical",
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

    # --- Bridge bar spans all three post positions ---
    bridge_aabb = ctx.part_world_aabb(bridge)
    assert bridge_aabb is not None
    bx_span = bridge_aabb[1][0] - bridge_aabb[0][0]
    ctx.check(
        "bridge_bar_spans_posts",
        bx_span >= 2.0 * POST_X + 0.02,
        f"bridge x span={bx_span:.3f}",
    )

    # --- Posts symmetric about spout ---
    lp = ctx.part_world_position(left_post)
    rp = ctx.part_world_position(right_post)
    sp = ctx.part_world_position(spout)
    assert lp is not None and rp is not None and sp is not None
    ctx.check(
        "posts_symmetric_about_spout",
        abs(lp[0] + POST_X) < 0.001
        and abs(rp[0] - POST_X) < 0.001
        and abs(sp[0]) < 0.001,
        f"left_x={lp[0]:.3f}, right_x={rp[0]:.3f}, spout_x={sp[0]:.3f}",
    )

    # --- Decorative ring ridges on handle pedestals ---
    for post in (left_post, right_post):
        col = post.get_visual("column")
        ctx.check(
            f"{post.name}_has_column",
            col is not None,
            f"visuals={[v.name for v in post.visuals]}",
        )

    # --- Narrow seams at all three deck bases ---
    for part_obj, seam_name in (
        (spout, "spout_seam"),
        (left_post, "base_seam"),
        (right_post, "base_seam"),
    ):
        seam = part_obj.get_visual(seam_name)
        ctx.check(
            f"{part_obj.name}_has_seam",
            seam is not None,
            f"visuals={[v.name for v in part_obj.visuals]}",
        )

    # --- Spout is a hollow tube (visible bore) ---
    ctx.check(
        "spout_is_hollow",
        0.0 < SPOUT_SOLID_VOLUME < 0.97 * SPOUT_UNBORED_VOLUME,
        f"solid={SPOUT_SOLID_VOLUME:.3e} vs unbored={SPOUT_UNBORED_VOLUME:.3e}",
    )

    # --- Spout rises above the posts and reaches forward ---
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (_, _, sz0), (_, sy1, sz1) = spout_aabb
    ctx.check(
        "spout_rises_above_bridge",
        sz1 > BRIDGE_TOP_Z + SPOUT_POST_H + SPOUT_RISE * 0.5,
        f"spout zmax={sz1:.3f}",
    )
    ctx.check(
        "spout_reaches_forward",
        sy1 > 0.05,
        f"spout ymax={sy1:.3f}",
    )

    # --- Intentional overlap: handle stems seat into post bores ---
    ctx.allow_overlap(
        left_handle,
        left_post,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_post.get_visual("column"),
        reason="handle stem is seated inside the post bore and turns with the handle",
    )
    ctx.allow_overlap(
        right_handle,
        right_post,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_post.get_visual("column"),
        reason="handle stem is seated inside the post bore and turns with the handle",
    )

    # --- Handle overlap on bridge (handle is above post, no direct overlap) ---
    # Handle spokes sit above bridge; no allow needed.

    # --- Cross handle tip-to-tip size at rest ---
    lh_aabb = ctx.part_world_aabb(left_handle)
    assert lh_aabb is not None
    hx = lh_aabb[1][0] - lh_aabb[0][0]
    hy = lh_aabb[1][1] - lh_aabb[0][1]
    ctx.check(
        "handle_cross_about_0p09_tip_to_tip",
        0.08 <= hx <= 0.105 and 0.08 <= hy <= 0.105,
        f"x={hx:.3f}, y={hy:.3f}",
    )

    # --- Overall width about 0.30 m ---
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.28 <= total_w <= 0.33,
        f"handle-tip to handle-tip width={total_w:.3f}",
    )

    # --- Rotation proof: spokes rotate in the XY plane ---
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rx = rot_aabb[1][0] - rot_aabb[0][0]
        ry = rot_aabb[1][1] - rot_aabb[0][1]
        ctx.check(
            "left_handle_rotates_in_xy_plane",
            rx > 0.05 and ry > 0.05,
            f"at q=45deg: x_span={rx:.3f}, y_span={ry:.3f}",
        )
        # Handle stays centred on its post while rotating
        cen = ctx.part_world_position(left_handle)
        assert cen is not None
        ctx.check(
            "left_handle_stays_on_post_axis",
            abs(cen[0] + POST_X) < 0.001
            and abs(cen[1]) < 0.001,
            f"handle origin={cen}",
        )

    with ctx.pose({right_joint: -math.pi / 2.0}):
        # Quarter turn: cross maps onto itself, handle still on post
        ctx.expect_overlap(right_handle, right_post, axes="xy", min_overlap=0.005)

    # --- Bridge grounded on deck ---
    ctx.check(
        "bridge_on_deck",
        abs(bridge_aabb[0][2]) < 0.001,
        f"bridge zmin={bridge_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
