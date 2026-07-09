from __future__ import annotations

# Folding A-frame plastic floor caution sign -- swing-handle variant.
# Forked from the integrated-grab-hole parent: the fixed molded carry bar
# is replaced by a separate swing-up bail handle that pivots on its own
# revolute joint mounted across the top of the apex hinge barrel.
# At rest the bail folds flat down against the apex; lifting it swings the
# bar up about the side-to-side (X) axis for carrying, exactly like a
# paint-can bail or a real folding floor-sign swing handle.
#
# Authoring convention for both panels (keeps the hinge math trivial):
#   * Each panel's local frame origin sits ON the hinge line (top edge).
#   * The panel body HANGS DOWN, occupying local z in [-PANEL_H, 0].
#   * The panel face lies in the X-Z plane; wall thickness runs along +Y,
#     i.e. the back (hidden) face is +Y, the visible printed face is -Y.
# The two panels are tilted into their A-frame stance entirely by joints, not
# by baked-in geometry rotation.

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
PANEL_H = 0.620  # panel height from foot to hinge line
BASE_W = 0.300  # panel width at the base (widest)
TOP_W = 0.110  # panel width at the top (near the hinge)
PANEL_T = 0.014  # panel wall thickness
HALF_SPLAY = math.radians(13.0)  # each panel leans this far from vertical (open)

HINGE_R = 0.013  # radius of the molded hinge barrel
HINGE_LEN = TOP_W + 0.018  # hinge barrel runs slightly past panel width

# The panel top edge stops just below the hinge line so the two tilted slabs do
# not cross through each other at the apex; the hinge barrel bridges the gap.
TOP_GAP = 0.020
# Each panel's top (hinge) edge is held this far off the centerline (in its own
# -Y, the inner/printed side) so the two splayed slabs never interpenetrate at
# the apex. The molded hinge barrel + skirt bridges this gap.
HINGE_SEP = 0.030

# --- Bail carry handle dimensions ---
BAIL_HALF_W = 0.048  # bail half-span along X (pivot spacing)
BAIL_BAR_R = 0.005  # bail bar cross-section radius (10 mm diameter rod)
BAIL_ARCH_H = 0.068  # arch apex height above the pivot level
BAIL_LEG_H = 0.014  # short vertical leg before the curve begins

PIVOT_EAR_R = 0.008  # pivot ear boss radius on top of barrel
PIVOT_EAR_H = 0.010  # pivot ear height above the barrel top surface

YELLOW = Material(name="caution_yellow", rgba=(0.96, 0.78, 0.06, 1.0))
DARK_GRAPHIC = Material(name="graphic_ink", rgba=(0.14, 0.14, 0.16, 1.0))
HINGE_PLASTIC = Material(name="hinge_plastic", rgba=(0.86, 0.70, 0.05, 1.0))
HANDLE_YELLOW = Material(name="handle_yellow", rgba=(0.94, 0.74, 0.04, 1.0))


# ---------------------------------------------------------------------------
# Geometry builders (authored in each part's local frame, in meters)
# ---------------------------------------------------------------------------
def _panel_profile(width_top: float, width_base: float, height: float):
    """Front-view trapezoid hanging below the hinge line.

    The narrow top edge stops at z = -TOP_GAP (just under the hinge barrel) and
    the wide base edge is at z = -height; widest at the base.
    """
    half_t = width_top / 2.0
    half_b = width_base / 2.0
    return [
        (-half_t, -TOP_GAP),
        (half_t, -TOP_GAP),
        (half_b, -height),
        (-half_b, -height),
    ]


def _build_panel_shell(y_shift: float = 0.0) -> cq.Workplane:
    """One tapered, shelled sign panel hanging from a z=0 hinge edge.

    Local frame: panel face in the X-Z plane, thickness along +Y, hinge edge at
    z = 0, base feet at z = -PANEL_H. Shelled open on the hidden (+Y) back side
    so the molded plastic reads hollow. Lower third has ventilation slots.

    `y_shift` translates the whole panel along local Y so the two splayed panels
    can be held apart at the hinge line.
    """
    profile = _panel_profile(TOP_W, BASE_W, PANEL_H)

    # Extrude the trapezoid along +Y to give the panel its wall thickness.
    body = cq.Workplane("XZ").polyline(profile).close().extrude(PANEL_T)

    # Hollow the panel: shell out the back (+Y) face so it reads as molded.
    try:
        body = body.faces(">Y").shell(-0.0035)
    except Exception:
        pass  # keep the panel solid if the shell op cannot resolve

    # Molded base feet: thicken the two bottom corners into small floor pads
    # that widen the stance.
    foot = cq.Workplane("XY").box(0.055, 0.062, 0.022, centered=(True, True, False))
    foot = foot.translate((0.0, PANEL_T / 2.0, -PANEL_H))
    left_foot = foot.translate((-(BASE_W / 2.0 - 0.032), 0.0, 0.0))
    right_foot = foot.translate((+(BASE_W / 2.0 - 0.032), 0.0, 0.0))
    body = body.union(left_foot).union(right_foot)

    # Ventilation / drain ribs near the lower third (horizontal slots, cut
    # clean through so the panel reads hollow there).
    for i in range(4):
        z = -PANEL_H + 0.085 + i * 0.030
        slot_w = BASE_W * 0.55 - i * 0.012
        slot = (
            cq.Workplane("XZ")
            .center(0.0, z)
            .rect(slot_w, 0.010)
            .extrude(PANEL_T + 0.02)
            .translate((0.0, -0.005, 0.0))
        )
        body = body.cut(slot)

    if y_shift != 0.0:
        body = body.translate((0.0, y_shift, 0.0))

    return body


def _build_graphic_plate() -> cq.Workplane:
    """Thin dark printed-placard inset, proud on the front (-Y) face."""
    plate = (
        cq.Workplane("XZ")
        .center(0.0, -PANEL_H * 0.52)
        .rect(BASE_W * 0.62, PANEL_H * 0.66)
        .extrude(-0.0016)  # toward -Y (the visible printed face)
    )
    plate = plate.edges("|Y").fillet(0.006)
    return plate


def _build_hinge_barrel() -> cq.Workplane:
    """Molded hinge barrel with skirt and two pivot ear bosses.

    Local frame is centered on the hinge axis: the barrel runs along X at
    z = 0. Two small cylindrical ear bosses sit on top of the barrel at
    the bail pivot points. No integrated carry handle -- the bail is a
    separate part that pivots on these ears.
    """
    # Hinge barrel along X, centered on the origin.
    barrel = (
        cq.Workplane("YZ")
        .circle(HINGE_R)
        .extrude(HINGE_LEN / 2.0, both=True)
    )

    # Molded saddle skirt below the barrel that reaches down to meet the two
    # panel top edges (which stop at z = -TOP_GAP, splayed to +/-Y). This is
    # the continuous-moulding web that joins both panels at the hinge.
    skirt_h = TOP_GAP + 0.018
    skirt_w_y = 2.0 * HINGE_SEP + 0.030  # spans from the front to the back panel
    skirt = (
        cq.Workplane("XY")
        .box(TOP_W + 0.006, skirt_w_y, skirt_h, centered=(True, True, False))
        .translate((0.0, 0.0, -skirt_h))
        .edges("|X")
        .fillet(0.006)
    )
    barrel = barrel.union(skirt)

    # Pivot ear bosses: two small cylindrical pads on top of the barrel at
    # the bail pivot spacing. The bail handle pivots on these ears.
    for i in range(2):
        sign = -1.0 + 2.0 * i  # -1 for left, +1 for right
        ear = (
            cq.Workplane("XY")
            .circle(PIVOT_EAR_R)
            .extrude(PIVOT_EAR_H)
            .translate((sign * BAIL_HALF_W, 0.0, HINGE_R))
        )
        barrel = barrel.union(ear)

    return barrel


def _build_carry_bail():
    """Curved bail carry handle as a tubular mesh arch.

    Local frame origin is at the pivot axis centre (top of the pivot ears).
    The bail arch extends upward in +Z (the 'up' / carrying position). The
    articulation pre-rotates this frame so that q=0 folds the bail flat.
    """
    # Spline points for the arch in the X-Z plane.
    # The arch goes from one pivot point, up and over, to the other.
    hw = BAIL_HALF_W
    ah = BAIL_ARCH_H
    lh = BAIL_LEG_H
    points = [
        (-hw, 0.0, 0.0),
        (-hw, 0.0, lh),
        (-hw * 0.82, 0.0, ah * 0.50),
        (-hw * 0.50, 0.0, ah * 0.82),
        (-hw * 0.15, 0.0, ah * 0.97),
        (0.0, 0.0, ah),
        (hw * 0.15, 0.0, ah * 0.97),
        (hw * 0.50, 0.0, ah * 0.82),
        (hw * 0.82, 0.0, ah * 0.50),
        (hw, 0.0, lh),
        (hw, 0.0, 0.0),
    ]

    bail = tube_from_spline_points(
        points,
        radius=BAIL_BAR_R,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    return bail


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_caution_sign_swing_handle")
    model.materials.extend([YELLOW, DARK_GRAPHIC, HINGE_PLASTIC, HANDLE_YELLOW])

    # World layout: the hinge line is placed at world height z = HINGE_Z,
    # centered at the origin. The front panel hangs down-and-forward (toward
    # -Y at its base); the back panel hangs down-and-backward (toward +Y).
    hinge_z_world = PANEL_H * math.cos(HALF_SPLAY)

    # --- FRONT PANEL (root) -------------------------------------------------
    front = model.part("front_panel")
    front_tilt = Origin(xyz=(0.0, 0.0, hinge_z_world), rpy=(-HALF_SPLAY, 0.0, 0.0))
    front.visual(
        mesh_from_cadquery(_build_panel_shell(y_shift=-HINGE_SEP), "front_panel"),
        origin=front_tilt,
        material=YELLOW,
        name="front_shell",
    )
    front.visual(
        mesh_from_cadquery(_build_graphic_plate(), "front_graphic"),
        origin=Origin(
            xyz=(0.0, -0.0035 - HINGE_SEP, hinge_z_world),
            rpy=(-HALF_SPLAY, 0.0, 0.0),
        ),
        material=DARK_GRAPHIC,
        name="front_graphic",
    )

    # --- HINGE BARREL (fixed to the front panel, on the hinge line) ---------
    hinge = model.part("hinge_barrel")
    hinge.visual(
        mesh_from_cadquery(_build_hinge_barrel(), "hinge_barrel"),
        material=HINGE_PLASTIC,
        name="hinge_barrel",
    )
    model.articulation(
        "front_to_hinge",
        ArticulationType.FIXED,
        parent=front,
        child=hinge,
        origin=Origin(xyz=(0.0, 0.0, hinge_z_world)),
    )

    # --- CARRY HANDLE BAIL (revolute on top of the hinge barrel) ------------
    # The bail is a separate molded part that pivots on the ear bosses.
    # Joint origin at the top of the pivot ears (the actual contact surface).
    # The bail is modeled with its arch extending in local +Z (upright).
    # rpy = (-pi/2, 0, 0) pre-rotates the child frame so that at q=0,
    # local +Z maps to world +Y (folded flat toward the back panel).
    # Positive q then rotates the arch toward +Z (upright for carrying).
    carry = model.part("carry_handle")
    carry.visual(
        mesh_from_geometry(_build_carry_bail(), "carry_bail"),
        material=HANDLE_YELLOW,
        name="carry_bail",
    )
    pivot_z_local = HINGE_R + PIVOT_EAR_H  # top of ear in hinge_barrel frame
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=hinge,
        child=carry,
        origin=Origin(
            xyz=(0.0, 0.0, pivot_z_local),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,  # q=0: folded flat (bail in +Y)
            upper=math.pi / 2.0 + 0.08,  # slightly past vertical for over-center
        ),
    )

    # --- BACK PANEL (child, folds about the hinge line) ---------------------
    back = model.part("back_panel")
    back.visual(
        mesh_from_cadquery(_build_panel_shell(y_shift=+HINGE_SEP), "back_panel"),
        material=YELLOW,
        name="back_shell",
    )
    model.articulation(
        "fold_back_panel",
        ArticulationType.REVOLUTE,
        parent=front,
        child=back,
        origin=Origin(xyz=(0.0, 0.0, hinge_z_world), rpy=(HALF_SPLAY, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-2.0 * HALF_SPLAY,
            upper=0.05,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front = object_model.get_part("front_panel")
    back = object_model.get_part("back_panel")
    hinge = object_model.get_part("hinge_barrel")
    carry = object_model.get_part("carry_handle")
    fold = object_model.get_articulation("fold_back_panel")
    fix = object_model.get_articulation("front_to_hinge")
    pivot = object_model.get_articulation("handle_pivot")

    # --- Mechanism type / axis ---------------------------------------------
    ctx.check(
        "fold joint is revolute",
        fold.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {fold.articulation_type}",
    )
    ctx.check(
        "fold axis is the horizontal left-right hinge line (X)",
        abs(fold.axis[0]) > 0.99
        and abs(fold.axis[1]) < 1e-6
        and abs(fold.axis[2]) < 1e-6,
        details=f"axis={fold.axis}",
    )
    ctx.check(
        "hinge barrel is rigidly fixed to the front panel",
        fix.articulation_type == ArticulationType.FIXED
        and fix.parent == "front_panel"
        and fix.child == "hinge_barrel",
        details=f"{fix.parent}->{fix.child} {fix.articulation_type}",
    )

    # --- Handle pivot joint ------------------------------------------------
    ctx.check(
        "handle pivot is a real revolute joint",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {pivot.articulation_type}",
    )
    ctx.check(
        "handle pivot axis runs side-to-side along the panel width (X)",
        abs(pivot.axis[0]) > 0.99
        and abs(pivot.axis[1]) < 1e-6
        and abs(pivot.axis[2]) < 1e-6,
        details=f"axis={pivot.axis}",
    )
    ctx.check(
        "handle pivot parent is the hinge barrel",
        pivot.parent == "hinge_barrel",
        details=f"parent={pivot.parent}",
    )
    ctx.check(
        "handle pivot child is the carry handle",
        pivot.child == "carry_handle",
        details=f"child={pivot.child}",
    )

    # --- Hero geometry presence --------------------------------------------
    ctx.check(
        "front panel carries a printed graphic placard",
        front.get_visual("front_graphic") is not None,
        details="front_graphic visual missing",
    )
    ctx.check(
        "hinge barrel part exists with pivot ears",
        hinge.get_visual("hinge_barrel") is not None,
        details="hinge_barrel visual missing",
    )
    ctx.check(
        "carry handle bail part exists",
        carry.get_visual("carry_bail") is not None,
        details="carry_bail visual missing",
    )

    # --- Scale sanity: tall, wide floor-sign proportions --------------------
    f_aabb = ctx.part_world_aabb(front)
    assert f_aabb is not None
    f_height = f_aabb[1][2] - f_aabb[0][2]
    f_width = f_aabb[1][0] - f_aabb[0][0]
    ctx.check(
        "front panel reads at floor-sign scale (>0.5 m tall)",
        f_height > 0.5,
        details=f"front height={f_height:.3f} m",
    )
    ctx.check(
        "front panel base is wider than 0.25 m",
        f_width > 0.25,
        details=f"front width={f_width:.3f} m",
    )

    # --- Hinge barrel and carry handle sit at the top of the sign ----------
    h_aabb = ctx.part_world_aabb(hinge)
    assert h_aabb is not None
    ctx.check(
        "hinge barrel sits at the top of the sign",
        h_aabb[1][2] >= f_aabb[1][2] - 0.02,
        details=f"hinge top z={h_aabb[1][2]:.3f}, panel top z={f_aabb[1][2]:.3f}",
    )

    c_aabb = ctx.part_world_aabb(carry)
    assert c_aabb is not None
    ctx.check(
        "carry handle sits at or above the hinge barrel",
        c_aabb[1][2] >= h_aabb[0][2] - 0.005,
        details=f"carry min z={c_aabb[0][2]:.3f}, hinge min z={h_aabb[0][2]:.3f}",
    )

    # --- Handle swing: rest pose is folded; upper limit swings upright -----
    pivot_upper = pivot.motion_limits.upper

    # At rest (q=0), bail is folded flat -- small Z extent.
    rest_aabb = ctx.part_world_aabb(carry)
    assert rest_aabb is not None
    rest_z_extent = rest_aabb[1][2] - rest_aabb[0][2]

    with ctx.pose({pivot: pivot_upper}):
        up_aabb = ctx.part_world_aabb(carry)
        assert up_aabb is not None
        up_z_extent = up_aabb[1][2] - up_aabb[0][2]
        up_top_z = up_aabb[1][2]

    ctx.check(
        "handle swings up: Z extent at upper limit >> Z extent at rest",
        up_z_extent > rest_z_extent + 0.030,
        details=f"rest z_extent={rest_z_extent:.3f}, up z_extent={up_z_extent:.3f}",
    )
    ctx.check(
        "handle upright apex is well above the hinge barrel top",
        up_top_z > h_aabb[1][2] + 0.020,
        details=f"up top z={up_top_z:.3f}, hinge top z={h_aabb[1][2]:.3f}",
    )

    # --- Hinge line is shared: both panels reach the top hinge height -------
    b_aabb_rest = ctx.part_world_aabb(back)
    assert b_aabb_rest is not None
    ctx.check(
        "both panels meet at the same hinge height",
        abs(b_aabb_rest[1][2] - f_aabb[1][2]) < 0.03,
        details=f"front top z={f_aabb[1][2]:.3f}, back top z={b_aabb_rest[1][2]:.3f}",
    )

    # --- Folding: rest pose (q=0) is OPEN; lower limit folds it flat ---------
    fold_folded = fold.motion_limits.lower

    back_open = ctx.part_world_aabb(back)
    front_min_y_open = f_aabb[0][1]
    with ctx.pose({fold: fold_folded}):
        back_folded = ctx.part_world_aabb(back)

    assert back_open is not None and back_folded is not None
    open_back_reach = back_open[1][1]
    folded_back_reach = back_folded[1][1]
    ctx.check(
        "open A-frame holds the back panel base swung backward (+Y)",
        open_back_reach > folded_back_reach + 0.05,
        details=f"open +Y reach={open_back_reach:.3f}, folded +Y reach={folded_back_reach:.3f}",
    )
    ctx.check(
        "open A-frame spans a stable front-to-back footprint",
        open_back_reach - front_min_y_open > 0.12,
        details=f"front -Y={front_min_y_open:.3f}, back +Y={open_back_reach:.3f}",
    )

    ctx.expect_overlap(
        front,
        back,
        axes="x",
        min_overlap=0.05,
        name="panels overlap across the hinge width at the hinge line",
    )

    # --- Two real non-fixed joints -----------------------------------------
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "sign has at least two real non-fixed joints (fold + handle pivot)",
        len(non_fixed) >= 2,
        details=f"non-fixed joints: {[a.name for a in non_fixed]}",
    )

    # --- Intentional molded captures at the hinge line ----------------------
    ctx.allow_overlap(
        hinge,
        front,
        elem_a="hinge_barrel",
        elem_b="front_shell",
        reason="Molded hinge barrel is one piece with the front panel top edge.",
    )
    ctx.allow_overlap(
        hinge,
        back,
        elem_a="hinge_barrel",
        elem_b="back_shell",
        reason="Molded hinge barrel captures the back panel top edge at the hinge.",
    )
    ctx.expect_contact(
        hinge,
        front,
        elem_a="hinge_barrel",
        elem_b="front_shell",
        name="hinge barrel meets the front panel top edge",
    )
    ctx.expect_contact(
        hinge,
        back,
        elem_a="hinge_barrel",
        elem_b="back_shell",
        name="hinge barrel meets the back panel top edge",
    )

    # --- Bail pivot contact with ear bosses ---------------------------------
    # The bail tube sits on the ear bosses at the pivot. Small intentional
    # overlap where the bail wraps around the pivot axis is expected.
    ctx.allow_overlap(
        carry,
        hinge,
        elem_a="carry_bail",
        elem_b="hinge_barrel",
        reason="Bail handle pivots on the ear bosses of the hinge barrel; "
               "tube cross-section embeds slightly into the ear at the pivot.",
    )
    ctx.expect_contact(
        carry,
        hinge,
        elem_a="carry_bail",
        elem_b="hinge_barrel",
        name="bail pivots on the hinge barrel ear bosses",
    )

    return ctx.report()


object_model = build_object_model()
