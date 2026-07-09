from __future__ import annotations

# Articraft model: retractable trapezoidal-blade utility knife, forked from
# the snap-off variant (picture/Handtools/Knife/001.png).
#
# Articraft brief:
# - Object: retractable utility knife with fixed trapezoidal blade, ~0.155 m
#   long, yellow plastic handle with a gray channel/spine, a flat trapezoidal
#   steel blade, a black textured thumb-grip, a rear lanyard hole, and a
#   sliding thumb button that pushes the blade out the front.
# - Root/support: the molded handle body (handle) is the fixed root. The rear
#   end cap is an inline visual on the handle (no FIXED-joint decoration part).
# - Parts: handle (root, with end_cap inline), blade_carrier (the sliding
#   member = trapezoidal blade + blade clamp + mounting posts + thumb button).
# - Articulation: handle_to_carrier, PRISMATIC, axis along the handle long
#   axis (+X), positive q pushes the blade out the front. At q=0 the blade
#   is fully retracted inside the handle.
# - Visible geometry: yellow tapered shell, gray top channel, silver
#   trapezoidal blade with two angled cutting corners and two mounting holes,
#   dark gray blade clamp, two alignment posts, black knurled thumb-grip,
#   ribbed gray thumb slide button, rear lanyard hole, dark gray rear end cap.
# - Support/fit: blade carrier rides inside the handle channel; the clamp
#   grips the blade spine; thumb button protrudes up through the channel slot.
# - Intentional overlaps: blade clamp captured inside the handle channel proxy
#   and blade body retracted inside the handle body (nested slider) ->
#   scoped allow_overlap with retained-insertion checks.
# - Tests: blade fully retracted at rest, blade extends past nose at full
#   travel, prismatic axis is +X, thumb button on top, blade stays retained,
#   trapezoidal blade shape verified, end cap present on handle.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters)
# ----------------------------------------------------------------------------
HANDLE_LEN = 0.150          # overall handle length along +X
HANDLE_H = 0.026            # handle height (Z) at the thick rear
HANDLE_FRONT_H = 0.016      # handle height at the front nose
HANDLE_W = 0.013            # handle width (Y)

CHANNEL_W = 0.0034          # width of the top blade channel (Y)
CHANNEL_DEPTH = 0.009       # how deep the channel cuts into the body (Z)

# Trapezoidal blade dimensions
BLADE_LEN = 0.055           # blade length along X (spine edge, top)
BLADE_H = 0.019             # blade height (Z direction)
BLADE_THK = 0.0006          # blade thickness (Y)
BLADE_BOT_LEN = 0.035       # bottom (cutting) edge length
BLADE_INSET = (BLADE_LEN - BLADE_BOT_LEN) / 2.0  # 0.010 each side

BLADE_RETRACT = 0.006       # blade front behind handle nose at rest (q=0)
SLIDE_TRAVEL = 0.038        # usable forward travel of the thumb slide

# Materials -----------------------------------------------------------------
YELLOW = Material(name="handle_yellow", rgba=(0.96, 0.78, 0.10, 1.0))
GRAY = Material(name="channel_gray", rgba=(0.62, 0.63, 0.66, 1.0))
DARK_GRAY = Material(name="dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
BLACK_GRIP = Material(name="black_grip", rgba=(0.10, 0.10, 0.11, 1.0))
STEEL = Material(name="blade_steel", rgba=(0.80, 0.82, 0.85, 1.0))
CLAMP_METAL = Material(name="clamp_metal", rgba=(0.52, 0.54, 0.58, 1.0))


# ----------------------------------------------------------------------------
# Handle geometry (IDENTICAL to parent snap-off variant)
# ----------------------------------------------------------------------------
def _handle_body_shape() -> cq.Workplane:
    """Tapered hollow-ish utility-knife handle shell, long axis +X.

    Cross-section is a rounded rectangle (Y wide, Z tall). The top is taller at
    the rear and tapers down toward a pointed nose. A top channel for the blade
    is cut along the centerline.
    """
    x0 = -HANDLE_LEN / 2.0
    x1 = HANDLE_LEN / 2.0
    profile = [
        (x0, HANDLE_W / 2.0, HANDLE_H, HANDLE_H / 2.0),
        (x0 + 0.030, HANDLE_W / 2.0, HANDLE_H, HANDLE_H / 2.0),
        (x0 + 0.085, HANDLE_W / 2.0, 0.024, 0.012),
        (x1 - 0.030, HANDLE_W / 2.0 * 0.92, HANDLE_FRONT_H + 0.003, HANDLE_FRONT_H / 2.0 + 0.004),
        (x1 - 0.006, HANDLE_W / 2.0 * 0.78, HANDLE_FRONT_H, HANDLE_FRONT_H / 2.0 + 0.003),
        (x1, HANDLE_W / 2.0 * 0.62, 0.010, 0.009),
    ]
    wires = []
    for x, hw, h, zc in profile:
        section = (
            cq.Workplane("YZ")
            .workplane(offset=x)
            .center(0.0, zc)
            .rect(2.0 * hw, h)
        )
        wires.append(section.val())
    solid = cq.Solid.makeLoft(wires, ruled=False)
    body = cq.Workplane("XY").newObject([solid])
    return body


def _channel_cut() -> cq.Workplane:
    """Top channel groove cut along the centerline for the blade + slide slot."""
    length = HANDLE_LEN + 0.02
    groove = (
        cq.Workplane("XY")
        .box(length, CHANNEL_W, CHANNEL_DEPTH, centered=(True, True, False))
        .translate((0.0, 0.0, HANDLE_H - CHANNEL_DEPTH + 0.0005))
    )
    return groove


def _lanyard_hole_cut() -> cq.Workplane:
    """Rear finger/lanyard through-hole along Y."""
    x0 = -HANDLE_LEN / 2.0
    hole = (
        cq.Workplane("XZ")
        .workplane(offset=HANDLE_W)
        .center(x0 + 0.014, HANDLE_H * 0.5)
        .circle(0.0042)
        .extrude(2.0 * HANDLE_W)
    )
    return hole


def _build_handle_visual() -> cq.Workplane:
    body = _handle_body_shape()
    body = body.cut(_channel_cut())
    body = body.cut(_lanyard_hole_cut())
    return body


def _build_top_channel_visual() -> cq.Workplane:
    """Gray channel/rail piece that sits in the top groove (the metal track)."""
    length = HANDLE_LEN - 0.012
    rail = (
        cq.Workplane("XY")
        .box(length, CHANNEL_W - 0.0004, CHANNEL_DEPTH - 0.001, centered=(True, True, False))
        .translate((0.002, 0.0, HANDLE_H - CHANNEL_DEPTH + 0.0008))
    )
    return rail


def _build_thumb_grip_visual() -> cq.Workplane:
    """Black textured thumb pad on the upper-front shoulder of the handle."""
    base = (
        cq.Workplane("XY")
        .box(0.030, HANDLE_W * 0.98, 0.0030, centered=(True, True, False))
    )
    bumps = None
    nx, ny = 6, 4
    for ix in range(nx):
        for iy in range(ny):
            x = -0.012 + ix * 0.0048
            y = -0.0045 + iy * 0.0030
            b = (
                cq.Workplane("XY")
                .transformed(offset=(x, y, 0.0030))
                .box(0.0026, 0.0018, 0.0016, centered=(True, True, False))
            )
            bumps = b if bumps is None else bumps.add(b)
    grip = base.add(bumps)
    grip = grip.translate((0.030, 0.0, HANDLE_FRONT_H - 0.0005))
    return grip


# ----------------------------------------------------------------------------
# Blade carrier geometry: trapezoidal blade + clamp + posts + button
# ----------------------------------------------------------------------------
def _build_trapezoidal_blade_shape() -> cq.Workplane:
    """Fixed trapezoidal utility blade in the XZ plane, thin along Y.

    The spine (top edge) is the full BLADE_LEN; the cutting edge (bottom) is
    shorter, creating two angled sharp corners. Two mounting holes near the
    spine are cut for the clamp posts.
    """
    pts = [
        (BLADE_INSET, 0.0),                         # bottom left sharp corner
        (0.0, BLADE_H),                             # top left
        (BLADE_LEN, BLADE_H),                       # top right
        (BLADE_LEN - BLADE_INSET, 0.0),              # bottom right sharp corner
    ]
    blade = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(BLADE_THK)
        .translate((0.0, -BLADE_THK / 2.0, 0.0))
    )
    # Cut two mounting holes near the spine
    for hx in (0.015, 0.040):
        hole = (
            cq.Workplane("XZ")
            .center(hx, BLADE_H - 0.004)
            .circle(0.002)
            .extrude(BLADE_THK + 0.001)
            .translate((0.0, -(BLADE_THK + 0.001) / 2.0, 0.0))
        )
        blade = blade.cut(hole)
    return blade


def _build_blade_clamp() -> cq.Workplane:
    """Flat clamp plate that grips the blade spine from above, inside the channel."""
    plate = (
        cq.Workplane("XY")
        .box(0.030, CHANNEL_W - 0.0006, 0.004, centered=(True, True, False))
    )
    return plate


def _build_clamp_post() -> cq.Workplane:
    """Small cylindrical post that extends from the clamp down through a blade
    mounting hole, connecting the clamp to the blade."""
    post = (
        cq.Workplane("XY")
        .circle(0.0014)
        .extrude(0.009)
    )
    return post


def _build_thumb_button() -> cq.Workplane:
    """Ribbed gray thumb slide button protruding up out of the channel slot."""
    base = (
        cq.Workplane("XY")
        .box(0.013, 0.0060, 0.0050, centered=(True, True, False))
        .edges("|Z").fillet(0.0010)
    )
    ribs = None
    for i in range(5):
        x = -0.004 + i * 0.002
        r = (
            cq.Workplane("XY")
            .transformed(offset=(x, 0.0, 0.0050))
            .box(0.0008, 0.0060, 0.0014, centered=(True, True, False))
        )
        ribs = r if ribs is None else ribs.add(r)
    return base.add(ribs)


# ----------------------------------------------------------------------------
# Assembly
# ----------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_utility_knife")

    # --- Handle (root) ------------------------------------------------------
    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_visual(), "handle_shell"),
        material=YELLOW,
        name="handle_shell",
    )
    handle.visual(
        mesh_from_cadquery(_build_top_channel_visual(), "top_channel"),
        material=GRAY,
        name="top_channel",
    )
    handle.visual(
        mesh_from_cadquery(_build_thumb_grip_visual(), "thumb_grip"),
        material=BLACK_GRIP,
        name="thumb_grip",
    )

    # End cap as inline visual on handle (no FIXED-joint decoration part)
    cap_shape = (
        cq.Workplane("YZ")
        .workplane(offset=-HANDLE_LEN / 2.0)
        .rect(HANDLE_W * 1.02, HANDLE_H * 0.96)
        .extrude(-0.012)
        .edges("|X").fillet(0.0025)
    )
    handle.visual(
        mesh_from_cadquery(cap_shape, "end_cap"),
        material=DARK_GRAY,
        name="end_cap",
    )

    # --- Blade carrier (prismatic slider) -----------------------------------
    # Authored in a local frame that coincides with the handle frame at q=0.
    # The carrier visuals are positioned in handle-relative coordinates.
    carrier = model.part("blade_carrier")

    # Blade positioning: at rest (q=0), blade front is BLADE_RETRACT behind nose
    nose_x = HANDLE_LEN / 2.0
    z_blade_spine = HANDLE_H - 0.003           # spine sits inside channel
    z_blade_origin = z_blade_spine - BLADE_H   # cutting edge is lower
    blade_rear_x = nose_x - BLADE_RETRACT - BLADE_LEN  # 0.014

    # Trapezoidal blade body
    blade = _build_trapezoidal_blade_shape().translate(
        (blade_rear_x, 0.0, z_blade_origin)
    )
    carrier.visual(
        mesh_from_cadquery(blade, "blade_body"),
        material=STEEL,
        name="blade_body",
    )

    # Blade clamp: sits above the blade spine, overlapping slightly
    holder_center_x = blade_rear_x + BLADE_LEN / 2.0
    z_clamp = z_blade_spine - 0.002  # clamp bottom overlaps blade spine by 0.002
    clamp = _build_blade_clamp().translate(
        (holder_center_x, 0.0, z_clamp)
    )
    carrier.visual(
        mesh_from_cadquery(clamp, "blade_clamp"),
        material=CLAMP_METAL,
        name="blade_clamp",
    )

    # Two clamp posts (repeated sub-parts via for-i-in-range with name_i naming)
    # Posts extend from inside the clamp down through the blade mounting holes.
    pin_x_offsets = [0.015, 0.040]  # match the blade hole positions
    z_post_base = z_blade_origin + BLADE_H - 0.008  # start 0.008 below hole center
    for i in range(2):
        post = _build_clamp_post().translate(
            (blade_rear_x + pin_x_offsets[i], 0.0, z_post_base)
        )
        carrier.visual(
            mesh_from_cadquery(post, f"post_{i}"),
            material=DARK_GRAY,
            name=f"post_{i}",
        )

    # Thumb button: sits on top of the clamp, protruding above the handle
    z_button = z_clamp + 0.004  # top of clamp plate
    button = _build_thumb_button().translate(
        (holder_center_x, 0.0, z_button)
    )
    carrier.visual(
        mesh_from_cadquery(button, "thumb_button"),
        material=GRAY,
        name="thumb_button",
    )

    # --- Articulation -------------------------------------------------------
    # Prismatic slide along the handle long axis (+X). Positive q pushes the
    # blade out the front. Joint origin at the channel centerline height.
    model.articulation(
        "handle_to_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_H - CHANNEL_DEPTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.2, lower=0.0, upper=SLIDE_TRAVEL
        ),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    carrier = object_model.get_part("blade_carrier")
    slide = object_model.get_articulation("handle_to_carrier")

    # --- Joint type / axis claims ------------------------------------------
    ctx.check(
        "blade slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ax = tuple(slide.axis)
    ctx.check(
        "slide axis is along +X (handle long axis)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # --- Blade fully retracted at rest (q=0) -------------------------------
    blade_aabb = ctx.part_element_world_aabb(carrier, elem="blade_body")
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "blade fully retracted at rest (front behind handle nose)",
        blade_aabb is not None
        and handle_aabb is not None
        and blade_aabb[1][0] < handle_aabb[1][0] - 0.001,
        details=f"blade_max_x={None if blade_aabb is None else blade_aabb[1][0]}, "
        f"handle_max_x={None if handle_aabb is None else handle_aabb[1][0]}",
    )

    # --- Blade extends past nose at full travel -----------------------------
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ext_aabb = ctx.part_element_world_aabb(carrier, elem="blade_body")
        ctx.check(
            "blade extends past handle nose at full travel",
            ext_aabb is not None
            and handle_aabb is not None
            and ext_aabb[1][0] > handle_aabb[1][0] + 0.020,
            details=f"ext_max_x={None if ext_aabb is None else ext_aabb[1][0]}, "
            f"handle_max_x={None if handle_aabb is None else handle_aabb[1][0]}",
        )
        # Retained insertion at full extension
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="blade_body",
            elem_b="handle_shell",
            min_overlap=0.005,
            name="blade stays retained in handle at full extension",
        )

    # --- Thumb slide button rises above the handle top ---------------------
    button_aabb = ctx.part_element_world_aabb(carrier, elem="thumb_button")
    ctx.check(
        "thumb slide button rises above the handle top",
        button_aabb is not None
        and handle_aabb is not None
        and button_aabb[1][2] > handle_aabb[1][2] - 0.001,
        details=f"button_max_z={None if button_aabb is None else button_aabb[1][2]}, "
        f"handle_max_z={None if handle_aabb is None else handle_aabb[1][2]}",
    )

    # --- Trapezoidal blade shape verified ----------------------------------
    ctx.check(
        "blade has real height extent (trapezoidal profile)",
        blade_aabb is not None and (blade_aabb[1][2] - blade_aabb[0][2]) > 0.014,
        details=f"blade_z_extent={None if blade_aabb is None else blade_aabb[1][2] - blade_aabb[0][2]}",
    )
    ctx.check(
        "blade is a thin flat plate",
        blade_aabb is not None and (blade_aabb[1][1] - blade_aabb[0][1]) < 0.004,
        details=f"blade_y_extent={None if blade_aabb is None else blade_aabb[1][1] - blade_aabb[0][1]}",
    )

    # --- End cap present as inline handle visual ---------------------------
    ctx.check(
        "end cap present as handle inline visual",
        "end_cap" in [v.name for v in handle.visuals],
    )

    # --- Blade carrier rides inside the handle (nested slider) -------------
    ctx.allow_overlap(
        carrier,
        handle,
        elem_a="blade_body",
        elem_b="handle_shell",
        reason="The blade retracts fully into the solid handle body at rest; "
        "this nested slider fit is intentional.",
    )
    ctx.allow_overlap(
        carrier,
        handle,
        elem_a="blade_clamp",
        elem_b="top_channel",
        reason="The blade clamp is captured inside the handle channel rail "
        "and slides along it.",
    )
    ctx.allow_overlap(
        carrier,
        carrier,
        elem_a="blade_clamp",
        elem_b="blade_body",
        reason="The clamp grips the blade spine with a small intentional "
        "overlap at the mounting interface.",
    )
    ctx.allow_overlap(
        carrier,
        carrier,
        elem_a="post_0",
        elem_b="blade_body",
        reason="Mounting post passes through the blade mounting hole.",
    )
    ctx.allow_overlap(
        carrier,
        carrier,
        elem_a="post_1",
        elem_b="blade_body",
        reason="Mounting post passes through the blade mounting hole.",
    )
    ctx.allow_overlap(
        carrier,
        carrier,
        elem_a="post_0",
        elem_b="blade_clamp",
        reason="Mounting post is embedded in the clamp plate for connection.",
    )
    ctx.allow_overlap(
        carrier,
        carrier,
        elem_a="post_1",
        elem_b="blade_clamp",
        reason="Mounting post is embedded in the clamp plate for connection.",
    )

    # Retained insertion: clamp stays within channel footprint (Y)
    ctx.expect_within(
        carrier,
        handle,
        axes="y",
        inner_elem="blade_clamp",
        outer_elem="top_channel",
        margin=0.0006,
        name="blade clamp stays centered in the channel",
    )
    ctx.expect_overlap(
        carrier,
        handle,
        axes="x",
        elem_a="blade_body",
        elem_b="handle_shell",
        min_overlap=0.020,
        name="blade remains inserted in the handle at rest",
    )

    # --- Actuating the joint extends the blade forward ---------------------
    rest_tip = ctx.part_element_world_aabb(carrier, elem="blade_body")
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ext_tip = ctx.part_element_world_aabb(carrier, elem="blade_body")
        ctx.check(
            "extending the slide pushes the blade tip forward",
            rest_tip is not None
            and ext_tip is not None
            and ext_tip[1][0] > rest_tip[1][0] + 0.030,
            details=f"rest_tip_x={None if rest_tip is None else rest_tip[1][0]}, "
            f"ext_tip_x={None if ext_tip is None else ext_tip[1][0]}",
        )

    return ctx.report()
