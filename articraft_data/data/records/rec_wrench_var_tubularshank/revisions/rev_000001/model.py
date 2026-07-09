from __future__ import annotations

# Stillson-type pipe wrench — tubular steel handle variant.
#
# Identity (from picture/Handtools/Wrench/002.png, fork variant):
#   A heavy steel pipe wrench. A fixed curved hook jaw with serrated teeth on
#   its underside crowns a steel frame/housing. A movable lower jaw, also
#   serrated, rides in a front channel of the housing and is driven along the
#   tool axis by a knurled adjusting nut to grip pipes of different diameters.
#   The lower half is now a straight hollow tubular steel shank of constant
#   round section, running back to a plain capped butt (no ferrule, no tapered
#   wooden handle).
#
# Articraft brief:
# - Object: pipe wrench, ~0.45 m long, head/frame in cast steel, tubular steel
#   handle. The tool LIES FLAT on the ground like a wrench on a table:
#   the long axis runs along world +X (head toward +X, handle butt toward -X),
#   the flat side of the head faces down, and z-min = 0.
# - Authoring frames: geometry is authored in a "tool frame" with the tool
#   axis along local +Z and the jaw mouth toward local +X. The whole assembly
#   is laid down by LAY_RPY/_lay(): tool x -> world +Y, tool y -> world +Z,
#   tool z -> world +X, plus a GROUND_LIFT so the tube touches z = 0.
# - Parts:
#     head_frame  - root: fixed hook jaw, housing with nut window and front
#                    jaw channel, flat shank, tubular steel handle with grip
#                    ribs, and plain butt cap.
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
#   knurled nut visible through the window, straight hollow tube handle of
#   constant round section with grip ribs and plain disc butt cap.
# - Intentional overlaps: nut/jaw-bar nest inside the housing window, and the
#   jaw bar slides inside the housing channel (scoped allowances with reasons).
# - Tests: lying-flat AABB (long in X, short in Z, z-min ~ 0); tube handle
#   constant round section; butt cap at handle end; grip ribs evenly spaced;
#   prismatic slide closes the mouth gap along world +X without the jaws
#   colliding; jaw stays engaged in the housing at full travel; nut axis
#   collinear with the slide and spinning in place.
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
SHANK_BOTTOM = 0.040     # tool z where shank meets the tube handle
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

# --- Tubular steel handle (constant round section) -----------------------
HANDLE_TOP = SHANK_BOTTOM         # tube starts where shank ends
HANDLE_BOTTOM = -0.085            # tube butt end (tool z)
TUBE_OR = 0.016                   # tube outer radius
TUBE_WALL = 0.006                 # wall thickness (thick enough for shank-wall overlap)
TUBE_IR = TUBE_OR - TUBE_WALL    # tube inner radius (0.010)

# Grip ribs on the lower tube.
GRIP_RIB_COUNT = 6
GRIP_RIB_SPACING = 0.012          # center-to-center along tool Z
GRIP_RIB_START_Z = HANDLE_BOTTOM + 0.025  # first rib position

# ---------------------------------------------------------------------------
# Lay-down placement: rotate the authored tool frame so the wrench lies flat.
# URDF rpy (roll=pi/2, yaw=pi/2) maps tool x->world +Y, tool y->world +Z,
# tool z->world +X. GROUND_LIFT raises it so the tube touches z = 0.
# ---------------------------------------------------------------------------
LAY_RPY = (math.pi / 2, 0.0, math.pi / 2)
GROUND_LIFT = TUBE_OR


def _lay(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Map a point from the authoring tool frame to the lying-flat world."""
    return (z, x, y + GROUND_LIFT)


# Materials
STEEL = ("steel", (0.74, 0.75, 0.77, 1.0))
DARK_STEEL = ("dark_steel", (0.45, 0.46, 0.49, 1.0))
KNURL_STEEL = ("knurl_steel", (0.58, 0.59, 0.62, 1.0))
RED_PAINT = ("red_paint", (0.74, 0.13, 0.11, 1.0))
GRIP_RUBBER = ("grip_rubber", (0.22, 0.22, 0.24, 1.0))
BUTT_STEEL = ("butt_steel", (0.38, 0.38, 0.40, 1.0))


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

    # Shank: flat steel bar going down into the tube handle (5mm insertion
    # for a realistic press-fit/weld junction; the shank edges overlap the
    # tube wall radially, connecting frame_steel to handle_tube).
    SHANK_INSERTION = 0.005
    shank = (
        cq.Workplane("XY")
        .box(
            SHANK_W,
            SHANK_T,
            SHANK_TOP - (SHANK_BOTTOM - SHANK_INSERTION) + 0.012,
            centered=(True, True, False),
        )
        .translate((-0.004, 0, SHANK_BOTTOM - SHANK_INSERTION))
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


def build_tube_handle_geometry() -> cq.Workplane:
    """Straight hollow tubular steel handle of constant round section.

    A hollow cylinder running from HANDLE_TOP down to HANDLE_BOTTOM along
    the tool Z axis, centered at (-0.004, 0) in tool XY (aligned with the
    flat shank above).
    """
    length = HANDLE_TOP - HANDLE_BOTTOM
    # Outer cylinder.
    outer = (
        cq.Workplane("XY")
        .circle(TUBE_OR)
        .extrude(length)
        .translate((0, 0, HANDLE_BOTTOM))
    )
    # Inner bore (slightly over-length for a clean through-cut).
    inner = (
        cq.Workplane("XY")
        .circle(TUBE_IR)
        .extrude(length + 0.004)
        .translate((0, 0, HANDLE_BOTTOM - 0.002))
    )
    tube = outer.cut(inner)
    return tube


def build_butt_cap_geometry() -> cq.Workplane:
    """Plain flat disc cap sealing the tube butt end."""
    cap_thickness = 0.003
    cap = (
        cq.Workplane("XY")
        .circle(TUBE_OR)
        .extrude(cap_thickness + 0.001)
        .translate((0, 0, HANDLE_BOTTOM - cap_thickness))
    )
    return cap


def build_grip_rib_geometry() -> cq.Workplane:
    """Thin annular grip rib proud of the tube surface.

    Centered at z=0 in the local frame; the caller positions it along the tube.
    The inner radius overlaps the tube for geometric connectivity.
    """
    rib_h = 0.002
    rib_outer_r = TUBE_OR + 0.001    # 0.5mm proud of tube
    rib_inner_r = TUBE_OR - 0.001    # 1mm inside tube for overlap
    rib = (
        cq.Workplane("XY")
        .circle(rib_outer_r)
        .circle(rib_inner_r)
        .extrude(rib_h)
        .translate((0, 0, -rib_h / 2))
    )
    return rib


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pipe_wrench")
    for mat_name, rgba in (
        STEEL,
        DARK_STEEL,
        KNURL_STEEL,
        RED_PAINT,
        GRIP_RUBBER,
        BUTT_STEEL,
    ):
        model.material(mat_name, rgba=rgba)

    # --- Root: head + frame + shank + tube handle, laid flat ---------------
    head_frame = model.part("head_frame")

    head_frame.visual(
        mesh_from_cadquery(build_head_frame_geometry(), "head_frame_steel"),
        origin=Origin(xyz=_lay(0.0, 0.0, 0.0), rpy=LAY_RPY),
        material=STEEL[0],
        name="frame_steel",
    )

    # Tubular steel handle: straight hollow tube of constant round section.
    head_frame.visual(
        mesh_from_cadquery(build_tube_handle_geometry(), "handle_tube"),
        origin=Origin(xyz=_lay(-0.004, 0.0, 0.0), rpy=LAY_RPY),
        material=RED_PAINT[0],
        name="handle_tube",
    )

    # Plain butt cap at the tube end.
    head_frame.visual(
        mesh_from_cadquery(build_butt_cap_geometry(), "butt_cap_disc"),
        origin=Origin(xyz=_lay(-0.004, 0.0, 0.0), rpy=LAY_RPY),
        material=BUTT_STEEL[0],
        name="butt_cap",
    )

    # Grip ribs: repeated annular rings along the lower tube.
    rib_mesh = mesh_from_cadquery(build_grip_rib_geometry(), "grip_rib")
    for i in range(GRIP_RIB_COUNT):
        z_rib = GRIP_RIB_START_Z + i * GRIP_RIB_SPACING
        head_frame.visual(
            rib_mesh,
            origin=Origin(xyz=_lay(-0.004, 0.0, z_rib), rpy=LAY_RPY),
            material=GRIP_RUBBER[0],
            name=f"grip_rib_{i}",
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
    tube_aabb = ctx.part_element_world_aabb(head_frame, elem="handle_tube")
    if frame_aabb is not None and tube_aabb is not None:
        ctx.check(
            "steel head sits beyond the tube handle along +X",
            frame_aabb[1][0] > tube_aabb[1][0] + 0.10,
            details=f"frame_max_x={frame_aabb[1][0]:.3f}, tube_max_x={tube_aabb[1][0]:.3f}",
        )
    if tube_aabb is not None:
        ctx.check(
            "tube handle extends back to the grip end (negative X)",
            tube_aabb[0][0] < 0.0,
            details=f"tube_min_x={tube_aabb[0][0]:.3f}",
        )

    # --- Tube handle: constant round section --------------------------------
    if tube_aabb is not None:
        tube_dx = tube_aabb[1][0] - tube_aabb[0][0]
        tube_dy = tube_aabb[1][1] - tube_aabb[0][1]
        tube_dz = tube_aabb[1][2] - tube_aabb[0][2]
        expected_dia = 2 * TUBE_OR
        ctx.check(
            "tube has constant round section (Y and Z extents match diameter)",
            abs(tube_dy - tube_dz) < 0.004 and tube_dy > 0.8 * expected_dia,
            details=f"dy={tube_dy:.4f}, dz={tube_dz:.4f}, expected={expected_dia:.4f}",
        )
        ctx.check(
            "tube extends the full handle length (>0.10 m)",
            tube_dx > 0.10,
            details=f"tube_length={tube_dx:.4f}",
        )

    # --- Butt cap: plain disc at the tube end ------------------------------
    cap_aabb = ctx.part_element_world_aabb(head_frame, elem="butt_cap")
    if cap_aabb is not None and tube_aabb is not None:
        ctx.check(
            "butt cap at the handle end (min_x near tube min_x)",
            cap_aabb[0][0] < tube_aabb[0][0] + 0.005,
            details=f"cap_min_x={cap_aabb[0][0]:.4f}, tube_min_x={tube_aabb[0][0]:.4f}",
        )
        cap_dy = cap_aabb[1][1] - cap_aabb[0][1]
        tube_dy = tube_aabb[1][1] - tube_aabb[0][1]
        ctx.check(
            "butt cap matches tube diameter (plain flat cap)",
            abs(cap_dy - tube_dy) < 0.004,
            details=f"cap_dy={cap_dy:.4f}, tube_dy={tube_dy:.4f}",
        )

    # --- Grip ribs: evenly spaced along the tube ---------------------------
    rib_aabbs = []
    for i in range(GRIP_RIB_COUNT):
        aabb = ctx.part_element_world_aabb(head_frame, elem=f"grip_rib_{i}")
        if aabb is not None:
            rib_aabbs.append(aabb)
    ctx.check(
        "all grip ribs present",
        len(rib_aabbs) == GRIP_RIB_COUNT,
        details=f"found={len(rib_aabbs)}, expected={GRIP_RIB_COUNT}",
    )
    if len(rib_aabbs) >= 2:
        rib_centers_x = [(a[0][0] + a[1][0]) / 2 for a in rib_aabbs]
        spacings = [
            rib_centers_x[i + 1] - rib_centers_x[i]
            for i in range(len(rib_centers_x) - 1)
        ]
        avg_spacing = sum(spacings) / len(spacings)
        ctx.check(
            "grip ribs are evenly spaced along the tube",
            all(abs(s - avg_spacing) < 0.003 for s in spacings) and avg_spacing > 0.005,
            details=f"spacings={[round(s, 4) for s in spacings]}, avg={avg_spacing:.4f}",
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
    if nut_aabb is not None:
        nut_dia_y = nut_aabb[1][1] - nut_aabb[0][1]
        ctx.check(
            "nut reads as a fat knurled barrel",
            nut_dia_y > 0.018,
            details=f"nut_y_extent={nut_dia_y:.4f}",
        )

    # --- Mechanism: sliding the jaw closes the mouth along world +X --------
    rest_aabb = ctx.part_world_aabb(movable_jaw)
    rest_pos = ctx.part_world_position(movable_jaw)
    with ctx.pose({jaw_joint: JAW_TRAVEL}):
        raised_aabb = ctx.part_world_aabb(movable_jaw)
        raised_pos = ctx.part_world_position(movable_jaw)
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
