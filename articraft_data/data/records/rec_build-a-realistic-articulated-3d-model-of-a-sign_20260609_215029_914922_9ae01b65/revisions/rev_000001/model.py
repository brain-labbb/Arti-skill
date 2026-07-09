from __future__ import annotations

# Folding A-frame plastic floor caution sign (maintenance / "wet floor" style).
# Reference: picture/Sign/sign/002.png -- a molded yellow polypropylene
# two-panel sign joined by an integrated top hinge with a built-in carry
# handle. The two tapered panels splay into an "A" when open and fold flat
# for storage. The primary mechanism is the fold about the top hinge line.
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

HANDLE_W = 0.090  # hand-hole opening width
HANDLE_H = 0.030  # hand-hole opening height

# The panel top edge stops just below the hinge line so the two tilted slabs do
# not cross through each other at the apex; the hinge barrel bridges the gap.
TOP_GAP = 0.020
# Each panel's top (hinge) edge is held this far off the centerline (in its own
# -Y, the inner/printed side) so the two splayed slabs never interpenetrate at
# the apex. The molded hinge barrel + skirt bridges this gap.
HINGE_SEP = 0.030

YELLOW = Material(name="caution_yellow", rgba=(0.96, 0.78, 0.06, 1.0))
DARK_GRAPHIC = Material(name="graphic_ink", rgba=(0.14, 0.14, 0.16, 1.0))
HINGE_PLASTIC = Material(name="hinge_plastic", rgba=(0.86, 0.70, 0.05, 1.0))


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


def _build_hinge_handle() -> cq.Workplane:
    """Molded hinge barrel + carry-handle bar with a hand-hole.

    Local frame is centered on the hinge axis: the barrel runs along X at
    z = 0, and the handle bar rises above it (+z) with a hand opening cut
    through it along Y.
    """
    # Hinge barrel along X, centered on the origin.
    barrel = (
        cq.Workplane("YZ")
        .circle(HINGE_R)
        .extrude(HINGE_LEN / 2.0, both=True)
    )

    # Molded saddle skirt below the barrel that reaches down to meet the two
    # panel top edges (which stop at z = -TOP_GAP, splayed to +/-Y). This is the
    # continuous-moulding web that joins both panels at the hinge.
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

    # Carry handle: a chunky bar above the barrel.
    bar_w = TOP_W + 0.020
    bar_block = (
        cq.Workplane("XY")
        .box(bar_w, 0.034, 0.052, centered=(True, True, False))
        .translate((0.0, 0.0, HINGE_R - 0.004))
        .edges("|Y")
        .fillet(0.012)
    )
    # Hand-hole through the bar (cut along Y so a hand can pass through).
    hole = (
        cq.Workplane("XZ")
        .center(0.0, HINGE_R + 0.022)
        .rect(HANDLE_W, HANDLE_H)
        .extrude(0.06, both=True)
    )
    handle = bar_block.cut(hole)

    return barrel.union(handle)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_caution_sign")
    model.materials.extend([YELLOW, DARK_GRAPHIC, HINGE_PLASTIC])

    # World layout: the hinge line is placed at world height z = HINGE_Z,
    # centered at the origin. The front panel hangs down-and-forward (toward
    # -Y at its base); the back panel hangs down-and-backward (toward +Y).
    hinge_z_world = PANEL_H * math.cos(HALF_SPLAY)

    # --- FRONT PANEL (root) -------------------------------------------------
    # The root part frame is placed at the world hinge line. The panel mesh is
    # authored hanging from local z=0; we tilt the visual so its base leans to
    # -Y (toward the viewer), giving the front leg of the "A".
    # The front panel slab is held on the -Y (viewer) side of the hinge line by
    # HINGE_SEP so its top edge clears the back panel; the hinge skirt bridges.
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

    # --- HINGE / HANDLE (fixed to the front panel, on the hinge line) -------
    hinge = model.part("hinge_handle")
    hinge.visual(
        mesh_from_cadquery(_build_hinge_handle(), "hinge_handle"),
        material=HINGE_PLASTIC,
        name="hinge_barrel",
    )
    model.articulation(
        "front_to_hinge",
        ArticulationType.FIXED,
        parent=front,
        child=hinge,
        # Hinge part frame sits at the world hinge line.
        origin=Origin(xyz=(0.0, 0.0, hinge_z_world)),
    )

    # --- BACK PANEL (child, folds about the hinge line) ---------------------
    # The back panel slab is held on its +Y side by HINGE_SEP so it mirrors the
    # front and clears it at the hinge line.
    back = model.part("back_panel")
    back.visual(
        mesh_from_cadquery(_build_panel_shell(y_shift=+HINGE_SEP), "back_panel"),
        material=YELLOW,
        name="back_shell",
    )
    # Joint frame at the world hinge line. The back panel hangs from local z=0.
    # axis = +X (the left-right hinge line). The joint frame is pre-rotated by
    # +HALF_SPLAY about X so that the REST pose (q=0) is the OPEN A-frame: the
    # back panel mirrors the front's lean, base toward +Y. Folding the sign flat
    # for storage is NEGATIVE q (swings the base back toward the front panel).
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
            # Negative q folds the back panel flat against the front (storage);
            # at q=0 the sign stands open in its stable A-frame.
            lower=-2.0 * HALF_SPLAY,
            upper=0.05,  # tiny over-open headroom
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
    hinge = object_model.get_part("hinge_handle")
    fold = object_model.get_articulation("fold_back_panel")
    fix = object_model.get_articulation("front_to_hinge")

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
        "hinge/handle is rigidly fixed to the front panel",
        fix.articulation_type == ArticulationType.FIXED
        and fix.parent == "front_panel"
        and fix.child == "hinge_handle",
        details=f"{fix.parent}->{fix.child} {fix.articulation_type}",
    )

    # --- Hero geometry presence --------------------------------------------
    ctx.check(
        "front panel carries a printed graphic placard",
        front.get_visual("front_graphic") is not None,
        details="front_graphic visual missing",
    )
    ctx.check(
        "carry-handle / hinge part exists",
        hinge.get_visual("hinge_barrel") is not None,
        details="hinge_barrel visual missing",
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

    # --- Carry handle sits at the very top of the sign ----------------------
    h_aabb = ctx.part_world_aabb(hinge)
    assert h_aabb is not None
    ctx.check(
        "carry handle/hinge sits at the top of the sign",
        h_aabb[1][2] >= f_aabb[1][2] - 0.02,
        details=f"hinge top z={h_aabb[1][2]:.3f}, panel top z={f_aabb[1][2]:.3f}",
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
    fold_folded = fold.motion_limits.lower  # negative -> folded for storage

    # Rest (open) pose.
    back_open = ctx.part_world_aabb(back)
    front_min_y_open = f_aabb[0][1]
    with ctx.pose({fold: fold_folded}):
        back_folded = ctx.part_world_aabb(back)

    assert back_open is not None and back_folded is not None
    open_back_reach = back_open[1][1]  # max +Y extent when open
    folded_back_reach = back_folded[1][1]  # max +Y extent when folded flat
    ctx.check(
        "open A-frame holds the back panel base swung backward (+Y)",
        open_back_reach > folded_back_reach + 0.05,
        details=f"open +Y reach={open_back_reach:.3f}, folded +Y reach={folded_back_reach:.3f}",
    )

    # The open A-frame straddles a front-to-back base footprint: front base
    # toward -Y, back base toward +Y -> stable stance wider than one panel.
    ctx.check(
        "open A-frame spans a stable front-to-back footprint",
        open_back_reach - front_min_y_open > 0.12,
        details=f"front -Y={front_min_y_open:.3f}, back +Y={open_back_reach:.3f}",
    )

    # The two panels still overlap across the hinge width (share the top edge).
    ctx.expect_overlap(
        front,
        back,
        axes="x",
        min_overlap=0.05,
        name="panels overlap across the hinge width at the hinge line",
    )

    # --- Intentional molded captures at the hinge line ----------------------
    # The molded hinge barrel/handle and the two panel top edges are one
    # continuous plastic moulding in the real part, so the barrel embeds
    # slightly into both panel top edges. Allow those local, mostly hidden
    # embeds and prove they are genuine contact, not free-floating parts.
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

    return ctx.report()


object_model = build_object_model()
