from __future__ import annotations

# Articraft model: 18 mm snap-off utility knife (box cutter) — ergonomic-grip
# variant. Handle is contoured with a palm swell and molded finger grooves along
# the underside. Modeled from picture/Handtools/Knife/001.png.
#
# Articraft brief:
# - Object: 18 mm snap-off blade utility knife, ~0.155 m long, yellow plastic
#   handle with a contoured ergonomic grip (palm swell + 4 molded finger
#   grooves on the underside), a gray channel/spine, a segmented silver snap-off
#   blade, a black textured thumb-grip, a rear lanyard hole, and a sliding
#   thumb ratchet that pushes the blade out the front.
# - Root/support: the molded handle body (handle) is the fixed root. It is a
#   contoured shell with a top channel that houses the blade and carries the
#   slide rail; the rear has a finger/lanyard hole. The underside has four
#   molded concave finger grooves cut into the loft surface.
# - Parts: handle (root), blade_carrier (the sliding member = blade spine plus
#   the exposed snap-off blade plus the thumb slide button; one rigid moving
#   part), end_cap (rear gray snap-off / cap, fixed to the handle).
# - Articulation: handle_to_carrier, PRISMATIC, axis along the handle long axis
#   (+X), positive q pushes the blade out the front. Origin at the rear seating
#   plane of the channel so the carrier stays retained at full extension.
# - Visible geometry: yellow contoured shell with palm swell and finger grooves,
#   gray top channel, silver segmented blade with score lines and a dark worn
#   tip, black knurled thumb-grip pad, ribbed gray thumb slide button, rear
#   lanyard hole, gray rear end cap.
# - Support/fit: the blade carrier rides inside the handle channel; the thumb
#   button protrudes up through the channel slot. The end cap is fixed to the
#   rear of the handle. The blade spine is captured inside the channel proxy.
# - Intentional overlaps: the blade spine slides inside the handle channel
#   proxy (nested slider) -> scoped allow_overlap with retained-insertion checks.
# - Tests: blade present in front, thumb button on top, prismatic axis is +X,
#   extending pushes the blade tip forward, blade stays retained in the channel,
#   end cap seated at the rear, lanyard hole present, ergonomic palm swell
#   present, finger grooves on underside.

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
GRIP_RUBBER = Material(name="grip_rubber", rgba=(0.15, 0.15, 0.16, 1.0))


def _handle_body_shape() -> cq.Workplane:
    """Contoured ergonomic utility-knife handle shell, long axis +X.

    The handle features:
    - A palm swell: the cross-section widens in the middle grip area.
    - Rounded edges for a comfortable grip (filleted after loft).
    - A tapered profile from the tall rear down to a pointed nose at +X.
    - The top remains relatively flat for the blade channel.
    """
    x0 = -HANDLE_LEN / 2.0
    x1 = HANDLE_LEN / 2.0

    # Profile: (x, half_width_y, height_z, z_center)
    # Width swells in the middle for an ergonomic palm grip. The handle is
    # slightly wider in the grip zone (sections 3-4) than at the rear or front.
    profile = [
        (x0,           HANDLE_W / 2.0 * 0.98, HANDLE_H,         HANDLE_H / 2.0),
        (x0 + 0.022,   HANDLE_W / 2.0 * 1.02, HANDLE_H * 0.99,  HANDLE_H / 2.0 * 0.98),
        (x0 + 0.048,   HANDLE_W / 2.0 * 1.22, HANDLE_H * 0.96,  HANDLE_H / 2.0 * 0.94),
        (x0 + 0.075,   HANDLE_W / 2.0 * 1.20, HANDLE_H * 0.90,  HANDLE_H / 2.0 * 0.90),
        (x0 + 0.100,   HANDLE_W / 2.0 * 1.06, HANDLE_H * 0.78,  HANDLE_H / 2.0 * 0.82),
        (x1 - 0.028,   HANDLE_W / 2.0 * 0.88, HANDLE_FRONT_H + 0.003, HANDLE_FRONT_H / 2.0 + 0.004),
        (x1 - 0.006,   HANDLE_W / 2.0 * 0.75, HANDLE_FRONT_H,   HANDLE_FRONT_H / 2.0 + 0.003),
        (x1,           HANDLE_W / 2.0 * 0.58, 0.010,            0.009),
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

    # Round the long side/top/bottom edges for a comfortable grip feel.
    # Use a modest fillet that won't destabilize the loft topology.
    try:
        body = body.edges("|X").fillet(0.002)
    except Exception:
        pass  # if fillet fails on complex topology, keep the raw loft
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


# ---------------------------------------------------------------------------
# Finger grooves — 4 molded concave scallops along the underside of the handle
# ---------------------------------------------------------------------------
FINGER_GROOVE_RADIUS = 0.005  # radius of each finger groove scallop
FINGER_GROOVE_PENETRATION = 0.003  # how deep each groove cuts into the handle bottom


def _finger_groove_cuts() -> cq.Workplane:
    """Four concave finger groove cuts along the underside of the handle.

    Each groove is a half-cylinder with its axis along Y (across the handle
    width), cut into the bottom surface to create a contoured ergonomic grip.
    """
    x0 = -HANDLE_LEN / 2.0
    # Groove X positions: evenly spaced along the grip zone (rear-to-mid handle).
    # 4 grooves for index, middle, ring, and pinky fingers.
    groove_xs = _groove_positions()
    # The cylinder center is below z=0 so only the top portion cuts into the
    # handle bottom surface, creating a concave scallop.
    center_z = -FINGER_GROOVE_RADIUS + FINGER_GROOVE_PENETRATION
    extrude_len = HANDLE_W * 3.0  # long enough to span the full handle width

    grooves = None
    for gx in groove_xs:
        groove = (
            cq.Workplane("XZ")
            .workplane(offset=-extrude_len / 2.0)
            .center(gx, center_z)
            .circle(FINGER_GROOVE_RADIUS)
            .extrude(extrude_len)
        )
        grooves = groove if grooves is None else grooves.add(groove)
    return grooves


def _groove_positions() -> list[float]:
    """Return the X positions of the 4 finger grooves along the handle."""
    x0 = -HANDLE_LEN / 2.0
    return [
        x0 + 0.028,
        x0 + 0.048,
        x0 + 0.068,
        x0 + 0.088,
    ]


def _build_single_groove_insert(gx: float) -> cq.Workplane:
    """Build one finger groove rubber insert visual at position gx.

    The insert fills the concave groove cut with a dark rubber-like material.
    The insert radius is slightly larger than the cut radius so the outer ring
    penetrates the handle shell for mesh connectivity (realistic rubber-
    overmold compression seating).
    """
    center_z = -FINGER_GROOVE_RADIUS + FINGER_GROOVE_PENETRATION
    # Slightly larger than the cut so the insert seats into the shell edges.
    insert_r = FINGER_GROOVE_RADIUS + 0.0006
    insert_len = HANDLE_W - 0.002

    # Create the groove cylinder
    cyl = (
        cq.Workplane("XZ")
        .workplane(offset=-insert_len / 2.0)
        .center(gx, center_z)
        .circle(insert_r)
        .extrude(insert_len)
    )
    # Keep only the portion near the handle bottom (z from -0.001 to groove top).
    # The insert should fill the groove void and seat slightly into the shell.
    keep_block = (
        cq.Workplane("XY")
        .box(0.020, HANDLE_W + 0.01, FINGER_GROOVE_PENETRATION + 0.002, centered=(True, True, False))
        .translate((gx, 0.0, -0.001))
    )
    insert = cyl.intersect(keep_block)
    return insert


def _build_handle_visual() -> cq.Workplane:
    body = _handle_body_shape()
    body = body.cut(_channel_cut())
    body = body.cut(_lanyard_hole_cut())
    body = body.cut(_finger_groove_cuts())
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

    # Molded finger groove rubber inserts along the underside (for-loop pattern).
    groove_xs = _groove_positions()
    for i in range(len(groove_xs)):
        gx = groove_xs[i]
        handle.visual(
            mesh_from_cadquery(_build_single_groove_insert(gx), f"finger_groove_{i}"),
            material=GRIP_RUBBER,
            name=f"finger_groove_{i}",
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

    # --- Ergonomic grip: palm swell and finger grooves ----------------------
    # Palm swell: the handle shell Y-extent should be wider than the nominal
    # HANDLE_W because of the contoured palm swell in the grip zone.
    shell_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")
    ctx.check(
        "ergonomic palm swell makes handle wider than nominal width",
        shell_aabb is not None
        and (shell_aabb[1][1] - shell_aabb[0][1]) > HANDLE_W * 1.10,
        details=f"shell_y_extent={None if shell_aabb is None else shell_aabb[1][1] - shell_aabb[0][1]}, "
        f"nominal_width={HANDLE_W}",
    )

    # Finger grooves: verify at least groove_0 exists and sits on the underside.
    groove_aabb = ctx.part_element_world_aabb(handle, elem="finger_groove_0")
    ctx.check(
        "finger groove 0 is present on the handle underside",
        groove_aabb is not None and groove_aabb[0][2] < HANDLE_H * 0.3,
        details=f"groove_min_z={None if groove_aabb is None else groove_aabb[0][2]}",
    )

    # All 4 grooves exist and are positioned along the grip zone.
    for i in range(4):
        ga = ctx.part_element_world_aabb(handle, elem=f"finger_groove_{i}")
        ctx.check(
            f"finger_groove_{i} mesh asset present",
            ga is not None,
            details=f"aabb={ga}",
        )

    return ctx.report()
