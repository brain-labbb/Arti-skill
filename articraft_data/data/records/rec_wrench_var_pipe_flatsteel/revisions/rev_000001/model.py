from __future__ import annotations

# Stillson-type pipe wrench — flat forged steel handle variant.
#
# Identity (from picture/Handtools/Wrench/002.png):
#   A heavy steel pipe wrench. A fixed curved hook jaw with serrated teeth on
#   its underside crowns a steel frame/housing. A movable lower jaw, also
#   serrated, rides in a front channel of the housing and is driven along the
#   tool axis by a knurled adjusting nut to grip pipes of different diameters.
#   The lower half is a FLAT FORGED STEEL HANDLE: a bright-steel flat bar
#   tapering slightly toward a closed hex ring butt end.
#
# Articraft brief:
# - Object: pipe wrench, ~0.45 m long, head/frame in cast steel, flat forged
#   steel handle. The tool LIES FLAT on the ground like a wrench on a table:
#   the long axis runs along world +X (head toward +X, handle butt toward -X),
#   the flat side of the head faces down, and z-min = 0.
# - Authoring frames: geometry is authored in a "tool frame" with the tool
#   axis along local +Z and the jaw mouth toward local +X. The whole assembly
#   is laid down by LAY_RPY/_lay(): tool x -> world +Y, tool y -> world +Z,
#   tool z -> world +X, plus a GROUND_LIFT so the frame body touches z = 0.
# - Parts:
#     head_frame  - root: fixed hook jaw, housing with nut window and front
#                    jaw channel, shank, and the flat forged steel handle.
#     movable_jaw - lower serrated jaw (bar + toothed head) sliding in the
#                    housing channel along the tool axis.
#     adjust_nut  - knurled adjusting nut seated in the housing window.
# - Articulations:
#     frame_to_jaw : PRISMATIC along the tool long axis (world +X after the
#                    lay-down rotation; local joint axis +Z). Positive q moves
#                    the movable jaw toward the fixed hook jaw, closing the
#                    pipe mouth. Travel 0 .. 0.024 m.
#     frame_to_nut : CONTINUOUS about the same tool-axis line (collinear with
#                    the slide), the knurled nut spinning in place.
# - Visible geometry: curved hook jaw with down-pointing teeth, open mouth in
#   front of the housing, movable jaw with up-pointing teeth in the channel,
#   knurled nut visible through the window, flat shank, flat bright-steel
#   handle bar tapering to a closed hex ring end.
# - Intentional overlaps: nut/jaw-bar nest inside the housing window, and the
#   jaw bar slides inside the housing channel (scoped allowances with reasons).
# - Tests: lying-flat AABB (long in X, short in Z, z-min ~ 0); prismatic slide
#   closes the mouth gap along world +X without the jaws colliding; jaw stays
#   engaged in the housing at full travel; nut axis collinear with the slide
#   and spinning in place; nut seated in the window; flat handle is thin in
#   world Z and wide in world Y; hex ring present at the grip end.
# - Assumptions: generic Stillson proportions; screw thread represented by the
#   prismatic jaw slide + spinning nut rather than literal helical threads.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Dimensions (meters), in the authoring "tool frame":
#   tool +Z = long axis (head at high z), tool +X = front (jaw mouth side).
# ---------------------------------------------------------------------------
SHANK_W = 0.022          # shank thickness in tool X
SHANK_T = 0.016          # shank depth in tool Y
SHANK_BOTTOM = 0.040     # tool z where shank meets the handle
SHANK_TOP = 0.235        # tool z where shank meets the frame body

FRAME_BODY_W = 0.058     # frame body width in tool X
FRAME_BODY_T = 0.030     # frame body depth in tool Y
FRAME_BODY_BOTTOM = SHANK_TOP - 0.006
FRAME_BODY_TOP = 0.300

WINDOW_W = 0.028         # adjustment window width (tool X)
WINDOW_H = 0.029         # adjustment window height (tool Z), hugs the nut
WINDOW_Z = 0.262         # window center tool z (nut seat / jaw joint origin)

# Front channel in the housing where the movable jaw rides.
SLOT_HALF_T = 0.009      # channel half-depth in tool Y
SLOT_Z0 = 0.268          # channel bottom (tool z)
SLOT_Z1 = 0.312          # channel top (cuts through the housing top)

# Hook jaw (fixed upper jaw) profile landmarks (tool frame).
HOOK_T_HALF = 0.013      # hook half-thickness in tool Y
HOOK_TOP = 0.362         # crown of the hook
HOOK_TEETH_FACE = 0.329  # tool z of the hook tooth root line (underside)
HOOK_TOOTH_H = 0.0045
HOOK_TEETH_TIP = HOOK_TEETH_FACE - HOOK_TOOTH_H  # 0.3245, lowest tooth tip

# Movable jaw (child frame: same orientation as tool frame, origin at the
# nut seat WINDOW_Z).
JAW_HEAD_Z0 = 0.010      # toothed head bottom (child z)
JAW_HEAD_Z1 = 0.030      # toothed head top (child z)
JAW_TEETH_FACE = 0.029   # tooth root line on the head top (child z)
JAW_TOOTH_H = 0.0045
JAW_TEETH_TIP = JAW_TEETH_FACE + JAW_TOOTH_H     # 0.0335, highest tooth tip
JAW_BAR_TOP = 0.024      # bar stays below the toothed head tips
JAW_TRAVEL = 0.024       # prismatic travel (m)

# Flat forged steel handle dimensions (tool frame).
HANDLE_TOP = SHANK_BOTTOM            # 0.040, where handle meets the shank
HANDLE_BOTTOM = -0.085               # lowest tool z of the ring end
HANDLE_T = 0.010                     # bar thickness in tool Y (flat direction)
HANDLE_W_TOP = 0.026                 # bar width at shank junction (tool X)
HANDLE_W_RING = 0.020                # bar width near ring junction (tool X)
RING_OD = 0.034                      # ring outer diameter
RING_AF = 0.019                      # hex hole across-flats dimension

# ---------------------------------------------------------------------------
# Lay-down placement: rotate the authored tool frame so the wrench lies flat.
# URDF rpy (roll=pi/2, yaw=pi/2) maps tool x->world +Y, tool y->world +Z,
# tool z->world +X. GROUND_LIFT raises it so the frame body (thickest part
# in tool Y) touches z = 0.
# ---------------------------------------------------------------------------
LAY_RPY = (math.pi / 2, 0.0, math.pi / 2)
GROUND_LIFT = FRAME_BODY_T / 2  # 0.015; frame body governs


def _lay(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Map a point from the authoring tool frame to the lying-flat world."""
    return (z, x, y + GROUND_LIFT)


# Materials
STEEL = ("steel", (0.74, 0.75, 0.77, 1.0))
DARK_STEEL = ("dark_steel", (0.45, 0.46, 0.49, 1.0))
KNURL_STEEL = ("knurl_steel", (0.58, 0.59, 0.62, 1.0))
BRIGHT_STEEL = ("bright_steel", (0.82, 0.84, 0.86, 1.0))


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery), authored in the tool frame
# ---------------------------------------------------------------------------
def _add_teeth_x(
    wp: cq.Workplane,
    *,
    x0: float,
    x1: float,
    z_face: float,
    y_half: float,
    tooth_h: float,
    n_teeth: int,
    pointing_z: float,
) -> cq.Workplane:
    """Union a row of triangular teeth along X onto a jaw face at z=z_face.

    pointing_z = -1 makes teeth point downward (-Z, hook jaw underside);
    pointing_z = +1 makes teeth point upward (+Z, movable jaw top).
    The tooth root line at z_face must be embedded in the host solid.
    """
    span = x1 - x0
    pitch = span / n_teeth
    result = wp
    for i in range(n_teeth):
        xc = x0 + (i + 0.5) * pitch
        tooth = (
            cq.Workplane("XZ")
            .moveTo(xc - pitch * 0.31, z_face)
            .lineTo(xc, z_face + pointing_z * tooth_h)
            .lineTo(xc + pitch * 0.31, z_face)
            .close()
            .extrude(y_half, both=True)
        )
        result = result.union(tooth)
    return result


def build_head_frame_geometry() -> cq.Workplane:
    """Fixed head: housing + shank + window + jaw channel + serrated hook."""
    # Housing body block.
    frame = (
        cq.Workplane("XY")
        .box(
            FRAME_BODY_W,
            FRAME_BODY_T,
            FRAME_BODY_TOP - FRAME_BODY_BOTTOM,
            centered=(True, True, False),
        )
        .translate((0, 0, FRAME_BODY_BOTTOM))
        .edges("|Z")
        .fillet(0.004)
    )
    # Shift housing so its front (+X) face sits forward; the mouth is on +X.
    frame = frame.translate((0.006, 0, 0))

    # Shank: flat steel bar going down to the handle.
    shank = (
        cq.Workplane("XY")
        .box(SHANK_W, SHANK_T, SHANK_TOP - SHANK_BOTTOM + 0.012, centered=(True, True, False))
        .translate((-0.004, 0, SHANK_BOTTOM))
        .edges("|Z")
        .fillet(0.003)
    )
    # Shallow longitudinal relief groove on the shank front (cosmetic).
    groove = (
        cq.Workplane("XY")
        .box(0.006, 0.010, 0.090, centered=(True, True, False))
        .translate((-0.004 + SHANK_W / 2 - 0.002, 0, SHANK_BOTTOM + 0.030))
    )
    shank = shank.cut(groove)
    head = frame.union(shank)

    # Nut window through the housing (through Y), snug around the knurled nut.
    window = (
        cq.Workplane("XY")
        .box(WINDOW_W, FRAME_BODY_T + 0.02, WINDOW_H, centered=True)
        .translate((0.006, 0, WINDOW_Z))
    )
    head = head.cut(window)

    # Front channel where the movable jaw rides; opens the housing front/top.
    slot = (
        cq.Workplane("XY")
        .box(0.050, 2 * SLOT_HALF_T, SLOT_Z1 - SLOT_Z0, centered=(False, True, False))
        .translate((-0.002, 0, SLOT_Z0))
    )
    head = head.cut(slot)

    # Hook jaw: shaped profile rising from the housing top, arcing forward to
    # a down-turned nose with a serrated underside over the mouth.
    hook = (
        cq.Workplane("XZ")
        .moveTo(-0.022, 0.292)   # base, sunk into the housing top
        .lineTo(-0.022, 0.350)   # back of the hook
        .lineTo(0.008, 0.362)    # crown
        .lineTo(0.040, 0.356)    # forward top
        .lineTo(0.058, 0.340)    # nose outer
        .lineTo(0.056, 0.326)    # nose tip
        .lineTo(0.016, 0.328)    # serrated underside back toward the throat
        .close()                 # diagonal down to the base over the channel
        .extrude(HOOK_T_HALF, both=True)
    )
    head = head.union(hook)

    # Hook teeth on the underside, pointing down into the mouth.
    head = _add_teeth_x(
        head,
        x0=0.020,
        x1=0.052,
        z_face=HOOK_TEETH_FACE,
        y_half=HOOK_T_HALF - 0.002,
        tooth_h=HOOK_TOOTH_H,
        n_teeth=6,
        pointing_z=-1.0,
    )

    return head


def build_movable_jaw_geometry() -> cq.Workplane:
    """Lower movable jaw: screw bar + forward toothed head, teeth pointing up.

    Authored in the child frame (same orientation as the tool frame); z=0 is
    the nut-seat plane (WINDOW_Z in the parent). The bar extends down through
    the nut and housing; the toothed head rides in the housing front channel.
    """
    # Jaw screw bar: runs down through the nut window; long retained length.
    bar = (
        cq.Workplane("XY")
        .box(0.013, 0.011, JAW_BAR_TOP + 0.085, centered=(True, True, False))
        .translate((0.004, 0, -0.085))
        .edges("|Z")
        .fillet(0.002)
    )
    # Toothed head: forward block whose top face carries the gripping teeth.
    head = (
        cq.Workplane("XY")
        .box(0.046, 2 * (SLOT_HALF_T - 0.001), JAW_HEAD_Z1 - JAW_HEAD_Z0, centered=(True, True, False))
        .translate((0.031, 0, JAW_HEAD_Z0))
        .edges("|Z")
        .fillet(0.002)
    )
    jaw = bar.union(head)
    # Teeth on the head top, pointing up toward the hook.
    jaw = _add_teeth_x(
        jaw,
        x0=0.016,
        x1=0.050,
        z_face=JAW_TEETH_FACE,
        y_half=SLOT_HALF_T - 0.003,
        tooth_h=JAW_TOOTH_H,
        n_teeth=6,
        pointing_z=1.0,
    )
    return jaw


def build_flat_handle_geometry() -> cq.Workplane:
    """Flat forged steel handle: tapered bar + closed hex ring at the butt end.

    Authored in the tool frame. The bar is wide in tool X, thin in tool Y
    (flat direction), and runs along tool Z from the shank down to the ring.
    The ring at the butt end has a hexagonal through-hole for a box-end grip.
    """
    half_t = HANDLE_T / 2  # 0.005
    cx = -0.004  # handle center X, aligned with the shank

    z_top = HANDLE_TOP           # 0.040
    z_bar_end = HANDLE_BOTTOM + 0.024  # -0.061, where bar meets ring
    z_ring_c = HANDLE_BOTTOM + 0.014   # -0.071, ring center

    # Tapered bar profile in the XZ plane, extruded through Y (thin direction).
    bar = (
        cq.Workplane("XZ")
        .moveTo(cx - HANDLE_W_TOP / 2, z_top)
        .lineTo(cx + HANDLE_W_TOP / 2, z_top)
        .lineTo(cx + HANDLE_W_RING / 2, z_bar_end)
        .lineTo(cx - HANDLE_W_RING / 2, z_bar_end)
        .close()
        .extrude(half_t, both=True)
    )

    # Ring at the butt end: cylinder in XZ plane, thin in Y.
    ring_outer = (
        cq.Workplane("XZ")
        .circle(RING_OD / 2)
        .extrude(half_t, both=True)
        .translate((cx, 0.0, z_ring_c))
    )

    # Hex through-hole in the ring (along Y, the thin direction).
    # polygon(6, d) uses circumscribed-circle diameter d.
    # For across-flats = RING_AF: d = RING_AF / cos(30°) = RING_AF / (√3/2).
    hex_d = RING_AF / (math.sqrt(3.0) / 2.0)
    hex_cut = (
        cq.Workplane("XZ")
        .polygon(6, hex_d)
        .extrude(half_t + 0.002, both=True)
        .translate((cx, 0.0, z_ring_c))
    )
    ring = ring_outer.cut(hex_cut)

    # Union bar and ring into one handle solid.
    handle = bar.union(ring)

    # Add shallow forged grip texture: a few cross-grooves on the +Y face.
    n_grooves = 4
    groove_span = z_top - z_bar_end - 0.030  # usable bar length for grooves
    for i in range(n_grooves):
        z_g = z_top - 0.018 - i * (groove_span / (n_grooves - 1))
        groove = (
            cq.Workplane("XY")
            .box(HANDLE_W_RING + 0.004, 0.0008, 0.0018, centered=True)
            .translate((cx, half_t - 0.0004, z_g))
        )
        handle = handle.cut(groove)

    return handle


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pipe_wrench")
    for mat_name, rgba in (
        STEEL,
        DARK_STEEL,
        KNURL_STEEL,
        BRIGHT_STEEL,
    ):
        model.material(mat_name, rgba=rgba)

    # --- Root: head + frame + shank + flat steel handle, laid flat ---------
    head_frame = model.part("head_frame")

    head_frame.visual(
        mesh_from_cadquery(build_head_frame_geometry(), "head_frame_steel"),
        origin=Origin(xyz=_lay(0.0, 0.0, 0.0), rpy=LAY_RPY),
        material=STEEL[0],
        name="frame_steel",
    )

    # Flat forged steel handle: tapered bar with hex ring end.
    head_frame.visual(
        mesh_from_cadquery(build_flat_handle_geometry(), "flat_handle_steel"),
        origin=Origin(xyz=_lay(0.0, 0.0, 0.0), rpy=LAY_RPY),
        material=BRIGHT_STEEL[0],
        name="flat_handle",
    )

    # --- Movable jaw -------------------------------------------------------
    movable_jaw = model.part("movable_jaw")
    movable_jaw.visual(
        mesh_from_cadquery(build_movable_jaw_geometry(), "movable_jaw_steel"),
        material=DARK_STEEL[0],
        name="jaw_steel",
    )

    # --- Adjusting nut (knurled) ------------------------------------------
    adjust_nut = model.part("adjust_nut")
    nut_knob = KnobGeometry(
        diameter=0.024,
        height=WINDOW_W - 0.001,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=30, depth=0.0008, helix_angle_deg=18.0),
    )
    nut_mesh = mesh_from_geometry(nut_knob, "adjust_nut_knurled")
    # KnobGeometry is built centered along local +Z, which is the joint spin
    # axis (the tool long axis after lay-down); leave it on the axis.
    adjust_nut.visual(
        nut_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=KNURL_STEEL[0],
        name="nut_knurled",
    )

    # --- Articulations -----------------------------------------------------
    # Movable jaw slides along the tool long axis (world +X after lay-down;
    # joint-frame +Z). Positive q moves it toward the fixed hook jaw, closing
    # the mouth. Joint frame sits at the nut seat (window center).
    model.articulation(
        "frame_to_jaw",
        ArticulationType.PRISMATIC,
        parent=head_frame,
        child=movable_jaw,
        origin=Origin(xyz=_lay(0.0, 0.0, WINDOW_Z), rpy=LAY_RPY),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.05, lower=0.0, upper=JAW_TRAVEL),
    )

    # Knurled adjusting nut spins about the jaw screw-bar centerline (tool
    # x = 0.004), collinear with the slide axis direction.
    model.articulation(
        "frame_to_nut",
        ArticulationType.CONTINUOUS,
        parent=head_frame,
        child=adjust_nut,
        origin=Origin(xyz=_lay(0.004, 0.0, WINDOW_Z), rpy=LAY_RPY),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    head_frame = object_model.get_part("head_frame")
    movable_jaw = object_model.get_part("movable_jaw")
    adjust_nut = object_model.get_part("adjust_nut")
    jaw_joint = object_model.get_articulation("frame_to_jaw")
    nut_joint = object_model.get_articulation("frame_to_nut")

    # --- Joint type / axis claims -----------------------------------------
    ctx.check(
        "jaw joint is prismatic",
        jaw_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={jaw_joint.articulation_type}",
    )
    ctx.check(
        "nut joint is continuous",
        nut_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={nut_joint.articulation_type}",
    )
    # Both joints share the same lay-down frame rotation, so identical local
    # axes mean the nut spin axis is collinear with the jaw slide axis (the
    # tool long axis, world +X).
    jx, jy, jz = jaw_joint.axis
    ctx.check(
        "jaw slides along the joint-frame Z (= tool long axis, world +X)",
        abs(jz) > 0.99 and abs(jx) < 0.01 and abs(jy) < 0.01,
        details=f"axis={jaw_joint.axis}",
    )
    ctx.check(
        "nut spin axis collinear with the jaw slide axis",
        tuple(nut_joint.axis) == tuple(jaw_joint.axis),
        details=f"jaw_axis={jaw_joint.axis}, nut_axis={nut_joint.axis}",
    )

    # --- Lying-flat rest pose ----------------------------------------------
    part_aabbs = [
        a
        for a in (
            ctx.part_world_aabb(head_frame),
            ctx.part_world_aabb(movable_jaw),
            ctx.part_world_aabb(adjust_nut),
        )
        if a is not None
    ]
    if part_aabbs:
        mins = [min(a[0][i] for a in part_aabbs) for i in range(3)]
        maxs = [max(a[1][i] for a in part_aabbs) for i in range(3)]
        x_extent = maxs[0] - mins[0]
        z_extent = maxs[2] - mins[2]
        ctx.check(
            "wrench rests on the ground (z-min ~ 0, nothing below)",
            -1e-3 < mins[2] < 2e-3,
            details=f"z_min={mins[2]:.4f}",
        )
        ctx.check(
            "wrench lies flat: long along X, short along Z",
            x_extent > 0.35 and z_extent < 0.08 and x_extent > 3.0 * z_extent,
            details=f"x_extent={x_extent:.3f}, z_extent={z_extent:.3f}",
        )

    # --- Layout along the tool axis (world X) ------------------------------
    frame_aabb = ctx.part_element_world_aabb(head_frame, elem="frame_steel")
    handle_aabb = ctx.part_element_world_aabb(head_frame, elem="flat_handle")
    if frame_aabb is not None and handle_aabb is not None:
        ctx.check(
            "steel head sits beyond the flat handle along +X",
            frame_aabb[1][0] > handle_aabb[1][0] + 0.10,
            details=f"frame_max_x={frame_aabb[1][0]:.3f}, handle_max_x={handle_aabb[1][0]:.3f}",
        )
    if handle_aabb is not None:
        ctx.check(
            "handle extends back to the grip end (negative X)",
            handle_aabb[0][0] < 0.0,
            details=f"handle_min_x={handle_aabb[0][0]:.3f}",
        )

    # --- Flat handle geometry claims ---------------------------------------
    if handle_aabb is not None:
        handle_dy = handle_aabb[1][1] - handle_aabb[0][1]  # world Y extent
        handle_dz = handle_aabb[1][2] - handle_aabb[0][2]  # world Z extent
        ctx.check(
            "flat handle is thin in Z (thickness < 15mm)",
            handle_dz < 0.015,
            details=f"handle_dz={handle_dz:.4f}",
        )
        ctx.check(
            "flat handle is wider in Y than thick in Z (reads as flat bar)",
            handle_dy > handle_dz * 1.5,
            details=f"handle_dy={handle_dy:.4f}, handle_dz={handle_dz:.4f}",
        )
        ctx.check(
            "handle ring end is wider than the bar (ring OD visible)",
            handle_dy > HANDLE_W_RING * 0.95,
            details=f"handle_dy={handle_dy:.4f}, ring_od={RING_OD}",
        )

    # Movable jaw and nut both present and seated in the head region.
    jaw_aabb = ctx.part_world_aabb(movable_jaw)
    nut_aabb = ctx.part_world_aabb(adjust_nut)
    ctx.check(
        "movable jaw present in the head region",
        jaw_aabb is not None and jaw_aabb[1][0] > WINDOW_Z,
        details=f"jaw_aabb={jaw_aabb}",
    )
    ctx.check(
        "knurled nut seated at the window",
        nut_aabb is not None
        and (nut_aabb[0][0] < WINDOW_Z < nut_aabb[1][0]),
        details=f"nut_aabb={nut_aabb}",
    )
    # Nut barrel is roughly the window-sized knurled cylinder (not a sliver);
    # its diameter now spans world Y.
    if nut_aabb is not None:
        nut_dia_y = nut_aabb[1][1] - nut_aabb[0][1]
        ctx.check(
            "nut reads as a fat knurled barrel",
            nut_dia_y > 0.018,
            details=f"nut_y_extent={nut_dia_y:.4f}",
        )

    # --- Mechanism: sliding the jaw closes the mouth along world +X --------
    # The fixed hook teeth tips sit at world x = HOOK_TEETH_TIP; the movable
    # jaw's teeth tips are its AABB max x. The gap shrinks with q but the
    # jaws must never collide.
    rest_aabb = ctx.part_world_aabb(movable_jaw)
    rest_pos = ctx.part_world_position(movable_jaw)
    with ctx.pose({jaw_joint: JAW_TRAVEL}):
        raised_aabb = ctx.part_world_aabb(movable_jaw)
        raised_pos = ctx.part_world_position(movable_jaw)
        # Jaw must still overlap the housing along the tool axis (retained).
        ctx.expect_overlap(
            movable_jaw,
            head_frame,
            axes="x",
            elem_b="frame_steel",
            min_overlap=0.020,
            name="jaw stays engaged in frame at full travel",
        )
        if raised_aabb is not None:
            ctx.check(
                "jaw stays above ground and inside the head channel at full travel",
                raised_aabb[0][2] > 0.0 and raised_aabb[1][2] < 0.04,
                details=f"raised_z=({raised_aabb[0][2]:.4f}, {raised_aabb[1][2]:.4f})",
            )
    ctx.check(
        "sliding moves the jaw along +X toward the hook",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[0] > rest_pos[0] + JAW_TRAVEL - 0.001
        and abs(raised_pos[1] - rest_pos[1]) < 1e-6
        and abs(raised_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    if rest_aabb is not None and raised_aabb is not None:
        gap_rest = HOOK_TEETH_TIP - rest_aabb[1][0]
        gap_raised = HOOK_TEETH_TIP - raised_aabb[1][0]
        ctx.check(
            "mouth gap visibly closes with the slide but jaws never collide",
            gap_rest > 0.020 and 0.001 < gap_raised < gap_rest - 0.020,
            details=f"gap_rest={gap_rest:.4f}, gap_raised={gap_raised:.4f}",
        )

    # --- Nut spins in place without translating ---------------------------
    nut_rest = ctx.part_world_position(adjust_nut)
    with ctx.pose({nut_joint: math.pi / 2}):
        nut_spun = ctx.part_world_position(adjust_nut)
    if nut_rest is not None and nut_spun is not None:
        drift = max(abs(nut_spun[i] - nut_rest[i]) for i in range(3))
        ctx.check(
            "nut spins in place (center does not translate)",
            drift < 1e-4,
            details=f"rest={nut_rest}, spun={nut_spun}",
        )

    # --- Intentional nesting allowances -----------------------------------
    ctx.allow_overlap(
        adjust_nut,
        head_frame,
        elem_a="nut_knurled",
        elem_b="frame_steel",
        reason="The knurled adjusting nut is captured inside the frame window.",
    )
    ctx.allow_overlap(
        adjust_nut,
        movable_jaw,
        elem_a="nut_knurled",
        elem_b="jaw_steel",
        reason="The nut threads onto the jaw screw bar and intentionally encircles it.",
    )
    ctx.allow_overlap(
        movable_jaw,
        head_frame,
        elem_a="jaw_steel",
        elem_b="frame_steel",
        reason="The movable jaw bar slides inside the housing channel and nut window.",
    )

    return ctx.report()


object_model = build_object_model()
