from __future__ import annotations

# Articraft model: 18 mm serrated sheepsfoot utility knife (box cutter variant),
# forked from the snap-off knife platform. Same handle, slide mechanism, and
# end cap; blade profile changed to a serrated sheepsfoot with toothed cutting
# edge and a blunt rounded tip.
#
# Articraft brief:
# - Object: 18 mm serrated sheepsfoot blade utility knife, ~0.155 m long,
#   yellow plastic handle with a gray channel/spine, a silver serrated
#   sheepsfoot blade (toothed cutting edge, blunt rounded tip), a black
#   textured thumb-grip, a rear lanyard hole, and a sliding thumb ratchet.
# - Root/support: the molded handle body (handle) is the fixed root.
# - Parts: handle (root), blade_carrier (sliding member = blade spine + the
#   exposed serrated sheepsfoot blade + thumb slide button), end_cap (rear cap).
# - Articulation: handle_to_carrier, PRISMATIC, axis +X, positive q pushes the
#   blade out the front. Same travel and origin as the parent.
# - Visible geometry: yellow tapered shell, gray top channel, silver sheepsfoot
#   blade with serrated (toothed) cutting edge and a blunt rounded tip with a
#   dark tip accent, black knurled thumb-grip, ribbed gray thumb slide button,
#   rear lanyard hole, gray rear end cap.
# - Support/fit: blade carrier rides inside the handle channel; thumb button
#   protrudes up through the channel slot. End cap fixed to the rear.
# - Intentional overlaps: blade spine slides inside the handle channel proxy.
# - Tests: blade present, serrated edge visible, blunt tip (no sharp point),
#   prismatic slide extends blade forward, retained at full extension.

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
def _sheepsfoot_outline_pts() -> list[tuple[float, float]]:
    """Return the (X, Z) outline points of a serrated sheepsfoot blade.

    Sheepsfoot profile in XZ plane:
    - Straight spine (top, z=0) from the rear to ~75% of blade length.
    - Spine curves downward toward the front, meeting the cutting edge region
      at a blunt rounded tip (no sharp point).
    - Cutting edge (bottom) is mostly straight with triangular serration teeth.
    - The heel connects back to the rear spine corner.

    Origin is at the rear spine corner (0, 0). Points go counter-clockwise.
    """
    bl = BLADE_LEN
    bw = BLADE_W
    pts: list[tuple[float, float]] = []

    # --- Spine (top edge): straight then curving down ---
    pts.append((0.0, 0.0))                     # rear spine corner
    pts.append((bl * 0.72, 0.0))               # end of straight spine

    # Curved drop: spine descends to meet the cutting edge region at the tip.
    n_curve = 8
    curve_start_x = bl * 0.72
    curve_end_x = bl - 0.002
    for i in range(1, n_curve + 1):
        t = i / n_curve
        x = curve_start_x + t * (curve_end_x - curve_start_x)
        # Ease-in curve: spine drops to about -bw*0.60 at the tip junction
        z = -bw * 0.60 * (t ** 1.6)
        pts.append((x, z))

    # Blunt rounded tip: a small arc of points rounding off the front.
    tip_cx = curve_end_x
    tip_cz = -bw * 0.60
    tip_r = 0.0025  # rounding radius at the blunt tip
    for i in range(1, 5):
        angle = math.pi / 2.0 * (i / 4.0)  # 0 to 90 degrees
        x = tip_cx + tip_r * math.sin(angle)
        z = tip_cz - tip_r * (1.0 - math.cos(angle))
        pts.append((x, z))

    # --- Serrated cutting edge (bottom): from front to rear ---
    cutting_baseline = -bw + 0.004   # baseline of the cutting edge
    # Serration teeth: triangular teeth along most of the cutting edge
    n_teeth = 14
    tooth_start_x = bl * 0.88        # teeth start behind the tip curve
    tooth_span = tooth_start_x - 0.003
    tooth_pitch = tooth_span / n_teeth
    tooth_depth = 0.0018             # how far teeth protrude below baseline

    # Connect from the tip rounding to the first tooth peak
    pts.append((tooth_start_x, cutting_baseline))

    # Generate teeth: each tooth is a valley + next peak (no duplicate points)
    for i in range(n_teeth):
        x_peak = tooth_start_x - i * tooth_pitch
        x_valley = x_peak - tooth_pitch * 0.5
        x_next_peak = x_peak - tooth_pitch
        pts.append((x_valley, cutting_baseline - tooth_depth))
        pts.append((x_next_peak, cutting_baseline))

    # --- Heel: rear bottom corner back up to spine ---
    pts.append((0.0, cutting_baseline))
    pts.append((0.0, -bw + 0.010))       # heel bevel

    return pts


def _build_blade_shape() -> cq.Workplane:
    """Serrated sheepsfoot blade: flat plate in the XZ plane.

    The blade lies flat (thin along Y), length along +X, width along +Z.
    The cutting edge (bottom) has serration teeth; the spine curves down at
    the front to a blunt rounded tip.
    """
    pts = _sheepsfoot_outline_pts()
    blade = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(BLADE_THK)
        .translate((0.0, -BLADE_THK / 2.0, 0.0))
    )
    return blade


def _build_blade_tip_visual() -> cq.Workplane:
    """Dark accent on the blunt rounded sheepsfoot tip (front curved section)."""
    bl = BLADE_LEN
    bw = BLADE_W

    # The tip accent covers the curved front portion of the blade where the
    # spine drops down and the tip rounds off.
    pts: list[tuple[float, float]] = []
    tip_region_start = bl * 0.72

    # Top edge follows the spine curve
    pts.append((tip_region_start, 0.0))
    n_curve = 6
    curve_end_x = bl - 0.002
    for i in range(1, n_curve + 1):
        t = i / n_curve
        x = tip_region_start + t * (curve_end_x - tip_region_start)
        z = -bw * 0.60 * (t ** 1.6)
        pts.append((x, z))

    # Rounded tip
    tip_cx = curve_end_x
    tip_cz = -bw * 0.60
    tip_r = 0.0025
    for i in range(1, 5):
        angle = math.pi / 2.0 * (i / 4.0)
        x = tip_cx + tip_r * math.sin(angle)
        z = tip_cz - tip_r * (1.0 - math.cos(angle))
        pts.append((x, z))

    # Bottom edge of the tip region (straight line back)
    cutting_baseline = -bw + 0.004
    pts.append((tip_region_start + 0.002, cutting_baseline))
    pts.append((tip_region_start, cutting_baseline))

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

    # Blade body (steel): serrated sheepsfoot with teeth already in the outline.
    # The dark tip accent is a separate visual for a distinct material.
    blade_body = blade
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
        "serrated sheepsfoot blade protrudes past the handle nose at rest",
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
    # Blade has real width: cutting edge (with serrations) is well below the spine.
    ctx.check(
        "blade has real width (serrated cutting edge below the spine)",
        blade_aabb is not None and (blade_aabb[1][2] - blade_aabb[0][2]) > 0.012,
        details=f"blade_z_extent={None if blade_aabb is None else blade_aabb[1][2] - blade_aabb[0][2]}",
    )

    # --- Sheepsfoot profile: blunt rounded tip, no sharp point -------------
    tip_aabb = ctx.part_element_world_aabb(carrier, elem="blade_tip")
    ctx.check(
        "blunt sheepsfoot tip accent is at the front of the blade",
        tip_aabb is not None
        and blade_aabb is not None
        and tip_aabb[1][0] >= blade_aabb[1][0] - 0.003,
        details=f"tip_max_x={None if tip_aabb is None else tip_aabb[1][0]}, "
        f"blade_max_x={None if blade_aabb is None else blade_aabb[1][0]}",
    )
    # The blade front is blunt: the tip accent region's Z extent is less than
    # the full blade extent because the spine curves down at the sheepsfoot
    # front and the tip region does not span the full serrated cutting edge.
    ctx.check(
        "sheepsfoot tip is blunt (tip Z extent < full blade extent)",
        tip_aabb is not None
        and blade_aabb is not None
        and (tip_aabb[1][2] - tip_aabb[0][2]) < (blade_aabb[1][2] - blade_aabb[0][2]) - 0.001,
        details=f"tip_z_extent={None if tip_aabb is None else tip_aabb[1][2] - tip_aabb[0][2]}, "
        f"blade_z_extent={None if blade_aabb is None else blade_aabb[1][2] - blade_aabb[0][2]}",
    )

    # --- Serrated cutting edge visible: teeth protrude below the blade body
    # The blade_steel AABB min_z should extend below the cutting edge baseline
    # because of the serration teeth (tooth_depth = 0.0018m below baseline).
    # The teeth are part of the blade outline, so the full blade Z extent
    # includes them. The cutting baseline is at -bw+0.004 = -0.014, teeth add
    # 0.0018 more, so total extent from spine (z=0) is ~0.0158.
    ctx.check(
        "serrated cutting edge teeth extend blade below baseline",
        blade_aabb is not None
        and (blade_aabb[1][2] - blade_aabb[0][2]) > BLADE_W - 0.003,
        details=f"blade_z_extent={None if blade_aabb is None else blade_aabb[1][2] - blade_aabb[0][2]}, "
        f"expected>{BLADE_W - 0.003}",
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
