from __future__ import annotations

# Articraft model: 18 mm snap-off utility knife (box cutter), modeled from
# picture/Handtools/Knife/001.png.
#
# Articraft brief:
# - Object: 18 mm snap-off blade utility knife, ~0.155 m long, yellow plastic
#   handle with a gray channel/spine, a segmented silver snap-off blade, a black
#   textured thumb-grip, a rear lanyard hole, and a sliding thumb ratchet that
#   pushes the blade out the front.
# - Root/support: the molded handle body (handle) is the fixed root. It is a
#   tapered shell with a top channel that houses the blade and carries the slide
#   rail; the rear has a finger/lanyard hole.
# - Parts: handle (root), blade_carrier (the sliding member = blade spine plus
#   the exposed snap-off blade plus the thumb slide button; one rigid moving
#   part), end_cap (rear gray snap-off / cap, fixed to the handle).
# - Articulation: handle_to_carrier, PRISMATIC, axis along the handle long axis
#   (+X), positive q pushes the blade out the front. Origin at the rear seating
#   plane of the channel so the carrier stays retained at full extension.
# - Visible geometry: yellow tapered shell, gray top channel, silver segmented
#   blade with score lines and a dark worn tip, black knurled thumb-grip pad,
#   ribbed gray thumb slide button, rear lanyard hole, gray rear end cap.
# - Support/fit: the blade carrier rides inside the handle channel; the thumb
#   button protrudes up through the channel slot. The end cap is fixed to the
#   rear of the handle. The blade spine is captured inside the channel proxy.
# - Intentional overlaps: the blade spine slides inside the handle channel
#   proxy (nested slider) -> scoped allow_overlap with retained-insertion checks.
# - Tests: blade present in front, thumb button on top, prismatic axis is +X,
#   extending pushes the blade tip forward, blade stays retained in the channel,
#   end cap seated at the rear, lanyard hole present.

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

BLADE_LEN = 0.060           # full snap-off blade length (spine + exposed)
BLADE_W = 0.018             # 18 mm blade
BLADE_THK = 0.0006          # blade thickness
BLADE_EXPOSED_REST = 0.012  # exposed blade length at the retracted rest pose

SLIDE_TRAVEL = 0.034        # usable forward travel of the thumb slide

# Materials -----------------------------------------------------------------
YELLOW = Material(name="handle_yellow", rgba=(0.96, 0.78, 0.10, 1.0))
GRAY = Material(name="channel_gray", rgba=(0.62, 0.63, 0.66, 1.0))
DARK_GRAY = Material(name="dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
BLACK_GRIP = Material(name="black_grip", rgba=(0.10, 0.10, 0.11, 1.0))
STEEL = Material(name="blade_steel", rgba=(0.80, 0.82, 0.85, 1.0))
BLADE_TIP = Material(name="blade_tip", rgba=(0.30, 0.34, 0.46, 1.0))


def _handle_body_shape() -> cq.Workplane:
    """Tapered hollow-ish utility-knife handle shell, long axis +X.

    Cross-section is a rounded rectangle (Y wide, Z tall). The top is taller at
    the rear and tapers down toward a pointed nose. A top channel for the blade
    is cut along the centerline.
    """
    # Side silhouette in the XZ plane: pointed nose at +X, tall blocky rear at -X
    # with a finger-hole bulge. Points go counter-clockwise.
    x0 = -HANDLE_LEN / 2.0
    x1 = HANDLE_LEN / 2.0
    z_bot = 0.0
    # Build the body by lofting rounded-rect cross sections along X so the width
    # also tapers slightly toward the nose.
    sections = []
    # (x, half_width_y, height_z, z_center)
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
    x0 = -HANDLE_LEN / 2.0
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
    # Knurled bumps for grip texture.
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
    # Place on the front-top shoulder. Top of body near the front is ~ at z given
    # the loft; sit the pad just below the channel top on the side shoulder.
    grip = grip.translate((0.030, 0.0, HANDLE_FRONT_H - 0.0005))
    return grip


# ----------------------------------------------------------------------------
# Blade carrier (moving prismatic member): blade spine + exposed blade + button
# ----------------------------------------------------------------------------
def _build_blade_shape() -> cq.Workplane:
    """Snap-off blade: a flat plate in the XZ plane, segmented with score lines.

    The blade lies flat (thin along Y), its length along +X, its width along +Z.
    The cutting edge is the bottom (-Z) angled edge; segments are scored
    diagonals. The spine (top edge) is captured by the carrier.
    """
    # Outline in XZ: a parallelogram-ish blade. Origin at the spine rear corner.
    # Cutting edge slopes from the rear-bottom up to a sharp front point.
    bl = BLADE_LEN
    bw = BLADE_W
    pts = [
        (0.0, 0.0),          # rear top (spine)
        (bl, 0.0),           # front top
        (bl, -bw + 0.004),   # front, just above the point
        (bl - 0.006, -bw),   # sharp front point (cutting tip)
        (0.0, -bw + 0.010),  # rear bottom (heel)
    ]
    blade = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(BLADE_THK)
        .translate((0.0, -BLADE_THK / 2.0, 0.0))
    )
    return blade


def _build_blade_score_lines() -> cq.Workplane:
    """Diagonal snap-off score grooves across the blade (visible v-notches)."""
    grooves = None
    bw = BLADE_W
    for i in range(5):
        x = 0.006 + i * 0.011
        groove = (
            cq.Workplane("XZ")
            .center(x, -bw / 2.0)
            .rect(0.0012, bw + 0.004)
            .extrude(BLADE_THK + 0.0006)
            .translate((0.0, -(BLADE_THK + 0.0006) / 2.0, 0.0))
            .rotate((x, 0.0, -bw / 2.0), (x, 1.0, -bw / 2.0), 18.0)
        )
        grooves = groove if grooves is None else grooves.add(groove)
    return grooves


def _build_blade_tip_visual() -> cq.Workplane:
    """Dark front segment of the blade (worn / coated front tip seen in image)."""
    bl = BLADE_LEN
    bw = BLADE_W
    pts = [
        (bl - 0.012, 0.0),
        (bl, 0.0),
        (bl, -bw + 0.004),
        (bl - 0.006, -bw),
        (bl - 0.012, -bw + 0.006),
    ]
    tip = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(BLADE_THK + 0.0002)
        .translate((0.0, -(BLADE_THK + 0.0002) / 2.0, 0.0))
    )
    return tip


def _build_blade_spine_carrier() -> cq.Workplane:
    """The carrier block that grips the blade spine and rides the channel.

    A small gray block over the top edge of the blade that connects the blade to
    the thumb button. Sits inside the channel proxy.
    """
    spine = (
        cq.Workplane("XY")
        .box(0.020, CHANNEL_W - 0.0006, 0.005, centered=(True, True, False))
    )
    return spine


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_off_utility_knife")

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

    # --- Rear end cap (fixed to handle) ------------------------------------
    end_cap = model.part("end_cap")
    cap_shape = (
        cq.Workplane("YZ")
        .workplane(offset=-HANDLE_LEN / 2.0)
        .rect(HANDLE_W * 1.02, HANDLE_H * 0.96)
        .extrude(-0.012)
        .edges("|X").fillet(0.0025)
    )
    end_cap.visual(
        mesh_from_cadquery(cap_shape, "end_cap"),
        material=DARK_GRAY,
        name="end_cap_body",
    )

    # --- Blade carrier (moving prismatic member) ---------------------------
    # Authored in a local frame whose origin is the rear seating point of the
    # carrier in the channel at the retracted rest pose. The blade extends along
    # +X. The spine of the blade sits in the channel (top), the cutting edge
    # points down (-Z), and the thumb button rises above the channel.
    carrier = model.part("blade_carrier")

    # The blade: positioned so that, at rest, only BLADE_EXPOSED_REST sticks out
    # past the handle nose. Place the blade's top spine at the channel level.
    z_blade_top = HANDLE_H - 0.0035            # spine sits inside the top channel
    # Blade local origin so the exposed length at rest matches the image.
    nose_x = HANDLE_LEN / 2.0
    blade_rear_x = nose_x - (BLADE_LEN - BLADE_EXPOSED_REST)

    blade = _build_blade_shape().translate((blade_rear_x, 0.0, z_blade_top))
    blade_tip = _build_blade_tip_visual().translate((blade_rear_x, 0.0, z_blade_top))
    spine = _build_blade_spine_carrier().translate(
        (blade_rear_x + 0.010, 0.0, z_blade_top - 0.0015)
    )
    button = _build_thumb_button().translate(
        (blade_rear_x + 0.012, 0.0, HANDLE_H - 0.0035)
    )

    # Blade body (steel), with the dark front tip and score lines as separate
    # visuals for distinct materials.
    blade_body = blade.cut(_build_blade_score_lines().translate(
        (blade_rear_x, 0.0, z_blade_top)
    ))
    carrier.visual(
        mesh_from_cadquery(blade_body, "blade_steel"),
        material=STEEL,
        name="blade_steel",
    )
    carrier.visual(
        mesh_from_cadquery(blade_tip, "blade_tip"),
        material=BLADE_TIP,
        name="blade_tip",
    )
    carrier.visual(
        mesh_from_cadquery(spine, "blade_spine"),
        material=GRAY,
        name="blade_spine",
    )
    carrier.visual(
        mesh_from_cadquery(button, "thumb_button"),
        material=GRAY,
        name="thumb_button",
    )

    # --- Articulations ------------------------------------------------------
    # Prismatic slide of the blade carrier along the handle long axis (+X).
    # Positive q pushes the blade out the front. Joint origin at the rear of the
    # carrier's rest position so the carrier stays retained at full extension.
    model.articulation(
        "handle_to_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.2, lower=0.0, upper=SLIDE_TRAVEL
        ),
    )

    # Rear cap fixed to the handle.
    model.articulation(
        "handle_to_cap",
        ArticulationType.FIXED,
        parent=handle,
        child=end_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    carrier = object_model.get_part("blade_carrier")
    end_cap = object_model.get_part("end_cap")
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

    # --- Hero geometry present: blade in front, button on top --------------
    blade_aabb = ctx.part_element_world_aabb(carrier, elem="blade_steel")
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "segmented blade protrudes past the handle nose at rest",
        blade_aabb is not None
        and handle_aabb is not None
        and blade_aabb[1][0] > handle_aabb[1][0] + 0.004,
        details=f"blade_max_x={None if blade_aabb is None else blade_aabb[1][0]}, "
        f"handle_max_x={None if handle_aabb is None else handle_aabb[1][0]}",
    )
    button_aabb = ctx.part_element_world_aabb(carrier, elem="thumb_button")
    ctx.check(
        "thumb slide button rises above the handle top",
        button_aabb is not None
        and handle_aabb is not None
        and button_aabb[1][2] > handle_aabb[1][2] - 0.001,
        details=f"button_max_z={None if button_aabb is None else button_aabb[1][2]}, "
        f"handle_max_z={None if handle_aabb is None else handle_aabb[1][2]}",
    )
    # Blade cutting edge points downward (blade bottom well below the spine).
    ctx.check(
        "blade has real width (cutting edge below the spine)",
        blade_aabb is not None and (blade_aabb[1][2] - blade_aabb[0][2]) > 0.012,
        details=f"blade_z_extent={None if blade_aabb is None else blade_aabb[1][2] - blade_aabb[0][2]}",
    )
    tip_aabb = ctx.part_element_world_aabb(carrier, elem="blade_tip")
    ctx.check(
        "dark blade tip is at the very front",
        tip_aabb is not None
        and blade_aabb is not None
        and tip_aabb[1][0] >= blade_aabb[1][0] - 0.001,
        details=f"tip_max_x={None if tip_aabb is None else tip_aabb[1][0]}",
    )

    # --- End cap seated at the rear ----------------------------------------
    ctx.expect_contact(end_cap, handle, name="end cap seated against handle rear")
    cap_aabb = ctx.part_world_aabb(end_cap)
    ctx.check(
        "end cap is at the rear (-X) of the handle",
        cap_aabb is not None
        and handle_aabb is not None
        and cap_aabb[0][0] <= handle_aabb[0][0] + 0.002,
        details=f"cap_min_x={None if cap_aabb is None else cap_aabb[0][0]}",
    )

    # --- Blade carrier rides inside the channel proxy (nested slider) ------
    ctx.allow_overlap(
        carrier,
        handle,
        elem_a="blade_spine",
        elem_b="top_channel",
        reason="The blade spine carrier is intentionally captured inside the "
        "handle's top channel rail and slides along it.",
    )
    ctx.allow_overlap(
        carrier,
        handle,
        elem_a="blade_steel",
        elem_b="top_channel",
        reason="The blade root rides inside the handle channel groove as the "
        "carrier slides; this nested fit is intentional.",
    )

    # Retained insertion: the spine stays within the channel footprint (Y) at
    # rest and the blade overlaps the handle body along the slide axis.
    ctx.expect_within(
        carrier,
        handle,
        axes="y",
        inner_elem="blade_spine",
        outer_elem="top_channel",
        margin=0.0006,
        name="blade spine stays centered in the channel",
    )
    ctx.expect_overlap(
        carrier,
        handle,
        axes="x",
        elem_a="blade_steel",
        elem_b="handle_shell",
        min_overlap=0.020,
        name="blade remains inserted in the handle at rest",
    )

    # --- Actuating the joint extends the blade forward ---------------------
    rest_tip = ctx.part_element_world_aabb(carrier, elem="blade_tip")
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ext_tip = ctx.part_element_world_aabb(carrier, elem="blade_tip")
        ctx.check(
            "extending the slide pushes the blade tip forward",
            rest_tip is not None
            and ext_tip is not None
            and ext_tip[1][0] > rest_tip[1][0] + 0.030,
            details=f"rest_tip_x={None if rest_tip is None else rest_tip[1][0]}, "
            f"ext_tip_x={None if ext_tip is None else ext_tip[1][0]}",
        )
        # Even fully extended, the blade root must still overlap the handle body
        # along X so the blade stays retained (not ejected).
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="blade_steel",
            elem_b="handle_shell",
            min_overlap=0.005,
            name="blade stays retained in the handle at full extension",
        )

    return ctx.report()
