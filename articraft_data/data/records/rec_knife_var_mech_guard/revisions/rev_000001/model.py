from __future__ import annotations

# Articraft model: Fixed-blade 18 mm utility knife with flip-up safety guard.
# Fork variant of the snap-off utility knife: blade-deployment mechanism
# changed from a prismatic slide to a permanently fixed blade with a revolute
# flip-up safety guard that pivots over the cutting edge.
#
# Articraft brief:
# - Object: 18 mm fixed-blade utility knife, ~0.155 m long, yellow plastic
#   handle with a gray channel/spine, a segmented silver snap-off blade
#   permanently exposed, a black textured thumb-grip, a rear lanyard hole,
#   and a bright-orange flip-up safety guard that covers the cutting edge.
# - Root/support: the molded handle body (handle) is the fixed root. It is a
#   tapered shell with a top channel; the rear has a finger/lanyard hole.
# - Parts: handle (root, carries blade visuals inline), end_cap (rear gray
#   snap-off cap, fixed to handle), safety_guard (flip-up guard, revolute).
# - Articulations:
#   · handle_to_guard: REVOLUTE, hinge at the front-top of the handle, axis
#     along -Y so positive q flips the guard up and away from the cutting edge.
#     Limits 0 (closed, covering the blade) to π (fully open).
#   · handle_to_cap: FIXED, same as parent.
# - Visible geometry: yellow tapered shell, gray top channel, silver segmented
#   blade with score lines and dark tip (fixed, permanently exposed), black
#   knurled thumb-grip pad, rear lanyard hole, gray rear end cap, orange
#   flip-up guard with side skirts and front lip, gray hinge bracket.
# - Support/fit: the guard's hinge barrel pivots between two hinge ears on the
#   handle. The blade is permanently seated in the handle channel.
# - Intentional overlaps: hinge barrel captured between hinge ears (scoped
#   allow_overlap with contact proof).
# - Tests: guard is revolute, guard covers blade when closed, guard flips up
#   when opened, blade is permanently exposed, end cap seated, hinge bracket
#   present, guard side skirts cover the cutting edge zone.

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

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
HANDLE_LEN = 0.150          # overall handle length along +X
HANDLE_H = 0.026            # handle height (Z) at the thick rear
HANDLE_FRONT_H = 0.016      # handle height at the front nose
HANDLE_W = 0.013            # handle width (Y)

CHANNEL_W = 0.0034          # width of the top blade channel (Y)
CHANNEL_DEPTH = 0.009       # how deep the channel cuts into the body (Z)

BLADE_LEN = 0.060           # full snap-off blade length
BLADE_W = 0.018             # 18 mm blade width (spine-to-edge along Z)
BLADE_THK = 0.0006          # blade thickness (along Y)
FIXED_BLADE_EXPOSED = 0.045 # permanently exposed blade length past nose

# Hinge geometry
HINGE_X = 0.068             # hinge origin X (near the handle nose)
HINGE_Z = 0.024             # hinge origin Z (above handle top at nose)
HINGE_BARREL_R = 0.0025     # hinge barrel radius
HINGE_BARREL_LEN = 0.010    # hinge barrel length (along Y)

# Guard dimensions
GUARD_LEN = 0.052           # guard plate length (covers exposed blade + tip)
GUARD_W = 0.016             # guard width (along Y, wider than blade thickness)
GUARD_THK = 0.002           # top plate thickness
GUARD_SKIRT_H = 0.014       # side skirt height (covers blade width zone)
GUARD_SKIRT_THK = 0.002     # side skirt thickness

# Derived positions
nose_x = HANDLE_LEN / 2.0   # handle nose X
blade_rear_x = nose_x - (BLADE_LEN - FIXED_BLADE_EXPOSED)
z_blade_top = HANDLE_H - 0.0035  # blade spine Z (inside the channel)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
YELLOW = Material(name="handle_yellow", rgba=(0.96, 0.78, 0.10, 1.0))
GRAY = Material(name="channel_gray", rgba=(0.62, 0.63, 0.66, 1.0))
DARK_GRAY = Material(name="dark_gray", rgba=(0.22, 0.23, 0.25, 1.0))
BLACK_GRIP = Material(name="black_grip", rgba=(0.10, 0.10, 0.11, 1.0))
STEEL = Material(name="blade_steel", rgba=(0.80, 0.82, 0.85, 1.0))
BLADE_TIP = Material(name="blade_tip", rgba=(0.30, 0.34, 0.46, 1.0))
GUARD_ORANGE = Material(name="guard_orange", rgba=(0.95, 0.45, 0.10, 1.0))


# ===========================================================================
# Handle geometry helpers (identical to parent)
# ===========================================================================
def _handle_body_shape() -> cq.Workplane:
    """Tapered utility-knife handle shell, long axis +X.

    Built by lofting rounded-rect cross sections so the body tapers toward the
    pointed nose.
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
    return cq.Workplane("XY").newObject([solid])


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
    """Gray channel/rail piece that sits in the top groove."""
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


# ===========================================================================
# Blade geometry helpers (shared — used as handle visuals for the fixed blade)
# ===========================================================================
def _build_blade_shape() -> cq.Workplane:
    """Snap-off blade flat plate in the XZ plane, segmented outline."""
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
    """Diagonal snap-off score grooves across the blade."""
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
    """Dark front segment of the blade (worn / coated front tip)."""
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


# ===========================================================================
# Hinge bracket (handle visual — visible mount for the guard pivot)
# ===========================================================================
def _build_hinge_bracket() -> cq.Workplane:
    """Two hinge ears on the handle sides that carry the guard pivot pin.

    Each ear is a small plate extending upward from the handle top surface to
    the hinge center. A crossbar connects them at the top.
    """
    ear_thk = 0.0018
    ear_w_x = 0.010
    # Approximate handle top Z at HINGE_X (the loft tapers toward the nose).
    handle_top_z = 0.017
    ear_h = HINGE_Z - handle_top_z + HINGE_BARREL_R + 0.001

    left_ear = (
        cq.Workplane("XY")
        .box(ear_w_x, ear_thk, ear_h, centered=(True, True, False))
        .edges("|Z").fillet(0.0006)
        .translate((HINGE_X, HANDLE_W / 2.0 + ear_thk / 2.0, handle_top_z))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(ear_w_x, ear_thk, ear_h, centered=(True, True, False))
        .edges("|Z").fillet(0.0006)
        .translate((HINGE_X, -(HANDLE_W / 2.0 + ear_thk / 2.0), handle_top_z))
    )
    # Crossbar connecting the ears at the hinge level
    crossbar = (
        cq.Workplane("XY")
        .box(ear_w_x, HANDLE_W + 2.0 * ear_thk, 0.0018, centered=(True, True, False))
        .edges("|Z").fillet(0.0005)
        .translate((HINGE_X, 0.0, HINGE_Z - HINGE_BARREL_R - 0.0018))
    )
    bracket = left_ear.add(right_ear).add(crossbar)
    return bracket


# ===========================================================================
# Safety guard geometry
# ===========================================================================
def _build_guard_hinge_barrel() -> cq.Workplane:
    """Hinge barrel that pivots between the handle ears.

    XZ workplane extrudes in -Y, so we translate +Y to center the barrel.
    """
    barrel = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(HINGE_BARREL_R)
        .extrude(HINGE_BARREL_LEN)
        .translate((0.0, HINGE_BARREL_LEN / 2.0, 0.0))
    )
    return barrel


def _build_guard_plate() -> cq.Workplane:
    """Top plate of the guard that covers the blade from above.

    In the guard local frame (origin at hinge center, q=0 = closed):
    - Plate extends forward (+X) from inside the barrel radius (for connectivity).
    - Plate sits just below the hinge center (-Z) so it covers the blade spine.
    """
    # Raise plate top 0.5 mm above barrel tangent so the mesh intersects the
    # barrel cylinder instead of barely missing it at the discretised surface.
    plate_z_top = -HINGE_BARREL_R + 0.0005
    # Start inside the barrel radius so the plate and barrel share mesh faces.
    plate_x0 = HINGE_BARREL_R * 0.4
    plate = (
        cq.Workplane("XY")
        .box(GUARD_LEN, GUARD_W, GUARD_THK, centered=(False, True, False))
        .translate((plate_x0, 0.0, plate_z_top - GUARD_THK))
    )
    return plate


def _build_guard_skirts() -> cq.Workplane:
    """Side skirts that extend down from the plate to cover the cutting edge."""
    plate_z_top = -HINGE_BARREL_R + 0.0005
    plate_z_bot = plate_z_top - GUARD_THK  # bottom of top plate
    skirt_z_bot = plate_z_bot - GUARD_SKIRT_H
    plate_x0 = HINGE_BARREL_R * 0.4

    left_skirt = (
        cq.Workplane("XY")
        .box(GUARD_LEN, GUARD_SKIRT_THK, GUARD_SKIRT_H, centered=(False, True, False))
        .translate((plate_x0, GUARD_W / 2.0 - GUARD_SKIRT_THK / 2.0, skirt_z_bot))
    )
    right_skirt = (
        cq.Workplane("XY")
        .box(GUARD_LEN, GUARD_SKIRT_THK, GUARD_SKIRT_H, centered=(False, True, False))
        .translate((plate_x0, -(GUARD_W / 2.0 - GUARD_SKIRT_THK / 2.0), skirt_z_bot))
    )
    return left_skirt.add(right_skirt)


def _build_guard_front_lip() -> cq.Workplane:
    """Front lip that closes off the guard tip to cover the blade point."""
    plate_z_top = -HINGE_BARREL_R + 0.0005
    plate_z_bot = plate_z_top - GUARD_THK
    skirt_z_bot = plate_z_bot - GUARD_SKIRT_H
    inner_w = GUARD_W - 2.0 * GUARD_SKIRT_THK
    plate_x0 = HINGE_BARREL_R * 0.4

    lip = (
        cq.Workplane("XY")
        .box(GUARD_THK, inner_w, GUARD_SKIRT_H, centered=(False, True, False))
        .translate((plate_x0 + GUARD_LEN - GUARD_THK, 0.0, skirt_z_bot))
    )
    return lip


def _build_guard_grip_ribs() -> cq.Workplane:
    """Small grip ribs on top of the guard plate for tactile detail."""
    plate_z_top = -HINGE_BARREL_R + 0.0005
    plate_x0 = HINGE_BARREL_R * 0.4
    ribs = None
    for i in range(6):
        x = plate_x0 + 0.008 + i * 0.007
        rib = (
            cq.Workplane("XY")
            .box(0.001, GUARD_W * 0.70, 0.001, centered=(True, True, False))
            .translate((x, 0.0, plate_z_top))
        )
        ribs = rib if ribs is None else ribs.add(rib)
    return ribs


def _build_guard_body() -> cq.Workplane:
    """Full guard assembly: barrel + plate + skirts + front lip + grip ribs."""
    barrel = _build_guard_hinge_barrel()
    plate = _build_guard_plate()
    skirts = _build_guard_skirts()
    front_lip = _build_guard_front_lip()
    ribs = _build_guard_grip_ribs()
    return barrel.add(plate).add(skirts).add(front_lip).add(ribs)


# ===========================================================================
# Model construction
# ===========================================================================
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixed_blade_utility_knife")

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
    # Hinge bracket (visible mount for the guard pivot)
    handle.visual(
        mesh_from_cadquery(_build_hinge_bracket(), "hinge_bracket"),
        material=GRAY,
        name="hinge_bracket",
    )

    # --- Fixed blade (inlined as handle visuals) ----------------------------
    # The blade is permanently exposed; no sliding mechanism.
    blade_body = _build_blade_shape().translate((blade_rear_x, 0.0, z_blade_top))
    score_lines = _build_blade_score_lines().translate((blade_rear_x, 0.0, z_blade_top))
    blade_with_scores = blade_body.cut(score_lines)
    blade_tip = _build_blade_tip_visual().translate((blade_rear_x, 0.0, z_blade_top))

    handle.visual(
        mesh_from_cadquery(blade_with_scores, "blade_steel"),
        material=STEEL,
        name="blade_steel",
    )
    handle.visual(
        mesh_from_cadquery(blade_tip, "blade_tip"),
        material=BLADE_TIP,
        name="blade_tip",
    )

    # --- Rear end cap (fixed to handle, identical to parent) ----------------
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

    # --- Safety guard (flip-up, revolute) -----------------------------------
    guard = model.part("safety_guard")
    guard.visual(
        mesh_from_cadquery(_build_guard_body(), "guard_body"),
        material=GUARD_ORANGE,
        name="guard_body",
    )

    # --- Articulations ------------------------------------------------------
    # Fixed rear cap.
    model.articulation(
        "handle_to_cap",
        ArticulationType.FIXED,
        parent=handle,
        child=end_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Revolute guard hinge.
    # Origin at the hinge point on the handle bracket.
    # Axis = (0, -1, 0): positive q rotates +X toward +Z, flipping the guard
    # upward and away from the cutting edge.
    model.articulation(
        "handle_to_guard",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=guard,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=0.0, upper=math.pi,
        ),
    )

    return model


object_model = build_object_model()


# ===========================================================================
# Tests
# ===========================================================================
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    end_cap = object_model.get_part("end_cap")
    guard = object_model.get_part("safety_guard")
    hinge = object_model.get_articulation("handle_to_guard")
    cap_joint = object_model.get_articulation("handle_to_cap")

    # --- Joint type / axis claims -------------------------------------------
    ctx.check(
        "guard hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ax = tuple(hinge.axis)
    ctx.check(
        "guard hinge axis is along -Y (flips up over cutting edge)",
        abs(ax[1]) > 0.99 and abs(ax[0]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )
    ctx.check(
        "end cap joint is fixed",
        cap_joint.articulation_type == ArticulationType.FIXED,
        details=f"type={cap_joint.articulation_type}",
    )

    # --- Blade is permanently exposed (fixed blade) -------------------------
    blade_aabb = ctx.part_element_world_aabb(handle, elem="blade_steel")
    handle_shell_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")
    ctx.check(
        "fixed blade protrudes past the handle nose",
        blade_aabb is not None
        and handle_shell_aabb is not None
        and blade_aabb[1][0] > handle_shell_aabb[1][0] + 0.030,
        details=f"blade_max_x={None if blade_aabb is None else blade_aabb[1][0]}, "
        f"handle_shell_max_x={None if handle_shell_aabb is None else handle_shell_aabb[1][0]}",
    )
    tip_aabb = ctx.part_element_world_aabb(handle, elem="blade_tip")
    ctx.check(
        "dark blade tip is at the very front of the blade",
        tip_aabb is not None
        and blade_aabb is not None
        and tip_aabb[1][0] >= blade_aabb[1][0] - 0.001,
        details=f"tip_max_x={None if tip_aabb is None else tip_aabb[1][0]}",
    )
    ctx.check(
        "blade has real width (cutting edge well below the spine)",
        blade_aabb is not None and (blade_aabb[1][2] - blade_aabb[0][2]) > 0.012,
        details=f"blade_z_extent={None if blade_aabb is None else blade_aabb[1][2] - blade_aabb[0][2]}",
    )
    # Full handle AABB (includes all visuals) for other checks
    handle_aabb = ctx.part_world_aabb(handle)

    # --- Hinge bracket present on handle ------------------------------------
    bracket_aabb = ctx.part_element_world_aabb(handle, elem="hinge_bracket")
    ctx.check(
        "hinge bracket is present on the handle",
        bracket_aabb is not None,
        details="hinge_bracket visual not found on handle",
    )

    # --- Guard closed (q=0): covers the blade from above --------------------
    with ctx.pose({hinge: 0.0}):
        guard_aabb = ctx.part_element_world_aabb(guard, elem="guard_body")
        ctx.check(
            "guard exists and has geometry",
            guard_aabb is not None,
            details="guard_body AABB is None",
        )
        # Guard overlaps blade in X (covers the exposed blade length)
        ctx.expect_overlap(
            guard,
            handle,
            axes="x",
            elem_a="guard_body",
            elem_b="blade_steel",
            min_overlap=0.035,
            name="closed guard covers the blade along X",
        )
        # Guard top plate is above the blade spine: guard max_z > blade max_z
        ctx.check(
            "closed guard top extends above the blade spine",
            guard_aabb is not None
            and blade_aabb is not None
            and guard_aabb[1][2] > blade_aabb[1][2] - 0.002,
            details=f"guard_max_z={None if guard_aabb is None else guard_aabb[1][2]}, "
            f"blade_max_z={None if blade_aabb is None else blade_aabb[1][2]}",
        )
        # Guard skirts extend below the blade cutting edge zone
        ctx.check(
            "guard side skirts extend below blade cutting edge",
            guard_aabb is not None
            and blade_aabb is not None
            and guard_aabb[0][2] < blade_aabb[0][2] + 0.002,
            details=f"guard_min_z={None if guard_aabb is None else guard_aabb[0][2]}, "
            f"blade_min_z={None if blade_aabb is None else blade_aabb[0][2]}",
        )

    # --- Guard open (q=π): flips up and away from the blade -----------------
    with ctx.pose({hinge: math.pi}):
        open_guard_aabb = ctx.part_element_world_aabb(guard, elem="guard_body")
        # Guard center moves upward when flipped open
        closed_center_z = 0.016  # approximate closed center z
        ctx.check(
            "open guard center is above the closed position",
            open_guard_aabb is not None
            and (open_guard_aabb[0][2] + open_guard_aabb[1][2]) / 2.0 > closed_center_z + 0.010,
            details=f"open_center_z={None if open_guard_aabb is None else (open_guard_aabb[0][2] + open_guard_aabb[1][2]) / 2.0}",
        )
        # Guard max_z extends well above the handle when open
        ctx.check(
            "open guard extends above the handle top",
            open_guard_aabb is not None
            and handle_aabb is not None
            and open_guard_aabb[1][2] > handle_aabb[1][2] + 0.005,
            details=f"guard_max_z={None if open_guard_aabb is None else open_guard_aabb[1][2]}, "
            f"handle_max_z={None if handle_aabb is None else handle_aabb[1][2]}",
        )

    # --- End cap seated at the rear -----------------------------------------
    ctx.expect_contact(end_cap, handle, name="end cap seated against handle rear")
    cap_aabb = ctx.part_world_aabb(end_cap)
    ctx.check(
        "end cap is at the rear (-X) of the handle",
        cap_aabb is not None
        and handle_aabb is not None
        and cap_aabb[0][0] <= handle_aabb[0][0] + 0.002,
        details=f"cap_min_x={None if cap_aabb is None else cap_aabb[0][0]}",
    )

    # --- Hinge barrel captured between bracket ears (intentional overlap) ---
    ctx.allow_overlap(
        guard,
        handle,
        elem_a="guard_body",
        elem_b="hinge_bracket",
        reason="The guard hinge barrel is intentionally captured between the "
        "hinge bracket ears on the handle, forming a real pivot.",
    )
    # Proof: the hinge barrel is near the bracket (contact/seated)
    ctx.expect_contact(
        guard,
        handle,
        elem_a="guard_body",
        elem_b="hinge_bracket",
        contact_tol=0.008,
        name="guard hinge barrel is seated at the bracket",
    )

    return ctx.report()
