from __future__ import annotations

# Nike shoe box (Footwear / Shoe box, ref 002.png).
#
# Classic two-piece athletic-shoe box with a "lift-off telescoping lid":
#   - BASE TRAY: a hollow rectangular cardboard tray that holds the shoe and
#     rests on the ground. Glossy black walls, a white/black spec label on the
#     front-left short end, and a large gray Nike swoosh + wordmark on the long
#     front face.
#   - LID: a separate shallow cap whose down-turned skirt slips DOWN over the
#     top of the base tray (it telescopes over the outside of the base walls).
#     Glossy black top with a big white "NIKE" wordmark, a small Nike swoosh on
#     the lid's front-left corner, and a scalloped finger-grip notch cut into
#     the lid's front-left edge so it can be lifted off.
#
# The lid is NOT hinged. On a lift-off box the lid translates straight UP off
# the base to come free, so the primary mechanism is a PRISMATIC joint along
# +Z. At rest the lid skirt overlaps the outside of the base walls (telescoped
# fit); lifting it up disengages the skirt.
#
# Coordinate convention: +Z up, box rests on the ground at z=0. Long axis along
# X (the box is much longer in X than deep in Y).
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters) -- US men's ~8.5 athletic shoe box.
# ---------------------------------------------------------------------------
# Base tray outer footprint and height.
BASE_X = 0.340  # length (long axis)
BASE_Y = 0.205  # depth
BASE_H = 0.110  # tray wall height

WALL_T = 0.0035  # cardboard wall thickness
FLOOR_T = 0.005  # tray floor thickness

# Lid: a cap that telescopes DOWN over the OUTSIDE of the base walls, so it is
# slightly larger in XY than the base and its skirt hangs partway down.
LID_GAP = 0.0  # snug telescoped fit: lid skirt inner wall contacts base wall
LID_WALL_T = 0.0035  # lid wall thickness
LID_TOP_T = 0.006  # lid top-panel thickness
LID_SKIRT_H = 0.040  # how far the lid skirt hangs down over the base walls
LID_OVERLAP = 0.030  # how much the skirt overlaps the base wall at rest

LID_OUT_X = BASE_X + 2.0 * (LID_GAP + LID_WALL_T)
LID_OUT_Y = BASE_Y + 2.0 * (LID_GAP + LID_WALL_T)

# Vertical placement: the lid sits so its skirt overlaps the top of the base.
# Lid top panel sits just above the base rim; skirt hangs down over base wall.
LID_BASE_Z = BASE_H - LID_OVERLAP  # bottom of the lid skirt at rest (closed)
LID_TOP_Z = LID_BASE_Z + LID_SKIRT_H  # underside of the lid top panel
# (LID_TOP_Z lands just above BASE_H, giving the closed silhouette.)

# Decal thickness (printed graphics modeled as a very thin proud layer).
DECAL_T = 0.0006

# Finger-grip notch cut into the lid front-left edge.
NOTCH_W = 0.060  # width along X
NOTCH_D = 0.018  # how deep it bites into the skirt (down from lid top)

# Colors.
BLACK = (0.045, 0.045, 0.05, 1.0)  # glossy box black
WHITE = (0.94, 0.94, 0.95, 1.0)  # white wordmark
GRAY = (0.62, 0.62, 0.64, 1.0)  # gray swoosh
LABEL_BG = (0.97, 0.97, 0.97, 1.0)  # spec-label background


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _sample_cubic(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    steps: int,
) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for i in range(steps + 1):
        t = i / steps
        u = 1.0 - t
        pts.append(
            (
                u**3 * p0[0] + 3.0 * u**2 * t * p1[0] + 3.0 * u * t**2 * p2[0] + t**3 * p3[0],
                u**3 * p0[1] + 3.0 * u**2 * t * p1[1] + 3.0 * u * t**2 * p2[1] + t**3 * p3[1],
            )
        )
    return pts


def _swoosh_profile(length: float, height: float) -> list[tuple[float, float]]:
    """A smooth, tapered Nike-swoosh-like profile, centered near origin."""
    top = _sample_cubic(
        (-0.55, -0.02),
        (-0.22, 0.03),
        (0.30, 0.18),
        (0.60, 0.31),
        18,
    )
    belly = _sample_cubic(
        (0.60, 0.31),
        (0.20, 0.06),
        (-0.18, -0.54),
        (-0.55, -0.20),
        22,
    )
    return [(x * length, y * height) for x, y in (top + belly[1:])]


def _poly_plate(points: list[tuple[float, float]], thickness: float = DECAL_T) -> cq.Workplane:
    return cq.Workplane("XY").polyline(points).close().extrude(thickness)


def _stroke_between(
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    thickness: float = DECAL_T,
) -> cq.Workplane:
    x0, y0 = start
    x1, y1 = end
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    nx = -dy / length * width / 2.0
    ny = dx / length * width / 2.0
    return _poly_plate(
        [
            (x0 + nx, y0 + ny),
            (x0 - nx, y0 - ny),
            (x1 - nx, y1 - ny),
            (x1 + nx, y1 + ny),
        ],
        thickness,
    )


def _slanted_rect(
    center: tuple[float, float],
    width: float,
    height: float,
    slant: float,
    thickness: float = DECAL_T,
) -> cq.Workplane:
    cx, cy = center
    return _poly_plate(
        [
            (cx - width / 2.0 - slant / 2.0, cy - height / 2.0),
            (cx + width / 2.0 - slant / 2.0, cy - height / 2.0),
            (cx + width / 2.0 + slant / 2.0, cy + height / 2.0),
            (cx - width / 2.0 + slant / 2.0, cy + height / 2.0),
        ],
        thickness,
    )


def _union_plates(plates: list[cq.Workplane]) -> cq.Workplane:
    result = plates[0]
    for plate in plates[1:]:
        result = result.union(plate)
    return result


def _build_base():
    """Hollow base tray: outer shell minus inner cavity, open top, with floor."""
    outer = (
        cq.Workplane("XY")
        .box(BASE_X, BASE_Y, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            BASE_X - 2.0 * WALL_T,
            BASE_Y - 2.0 * WALL_T,
            BASE_H,  # open at the top: cut all the way through the rim
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, FLOOR_T))
    )
    tray = outer.cut(cavity)
    return tray


def _build_lid():
    """Hollow lift-off lid (cap) authored in its OWN local frame.

    Local frame: the lid top panel underside sits at local z=0; the skirt hangs
    DOWN into local -z. So the lid occupies z in [-LID_SKIRT_H, +LID_TOP_T].
    The mounting origin then places this frame at LID_TOP_Z in the base frame.
    """
    # Top panel.
    top = (
        cq.Workplane("XY")
        .box(LID_OUT_X, LID_OUT_Y, LID_TOP_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )

    # Down-turned skirt: outer wall band minus inner band, hanging in -z.
    skirt_outer = (
        cq.Workplane("XY")
        .box(LID_OUT_X, LID_OUT_Y, LID_SKIRT_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, -LID_SKIRT_H))
    )
    skirt_hole = (
        cq.Workplane("XY")
        .box(
            LID_OUT_X - 2.0 * LID_WALL_T,
            LID_OUT_Y - 2.0 * LID_WALL_T,
            LID_SKIRT_H + 0.01,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -LID_SKIRT_H - 0.005))
    )
    skirt = skirt_outer.cut(skirt_hole)

    lid = top.union(skirt)

    # Scalloped finger-grip notch cut into the front-left of the skirt. Front is
    # -Y face; left is -X. The notch is a rounded bite removed from the lower
    # edge of the front skirt so a hand can grip and lift the lid.
    notch_cx = -LID_OUT_X / 2.0 + NOTCH_W / 2.0 + 0.015
    notch_cutter = (
        cq.Workplane("XZ")
        .moveTo(notch_cx, -LID_SKIRT_H)  # in XZ plane: (x, z)
        .ellipse(NOTCH_W / 2.0, NOTCH_D)
        .extrude(LID_OUT_Y, both=True)
    )
    # The ellipse is built in the XZ plane (X across the skirt, Z vertical) and
    # extruded through Y so it cuts the full front skirt depth. Center it on the
    # lower edge of the skirt so it reads as a half-moon scallop.
    lid = lid.cut(notch_cutter)

    return lid


def _label_panel():
    """White spec label (thin plate) for the front-left short region of the tray."""
    return cq.Workplane("XY").box(0.058, 0.036, DECAL_T, centered=(True, True, False))


def _swoosh_decal(length: float, height: float):
    """Thin extruded swoosh decal lying in its own local XY plane (z thickness)."""
    profile = _swoosh_profile(length, height)
    return cq.Workplane("XY").polyline(profile).close().extrude(DECAL_T)


def _wordmark_decal(length: float, height: float):
    """Thin block-italic NIKE wordmark made from separate raised strokes."""
    stroke = height * 0.18
    slant = height * 0.26
    plates = [
        # N
        _slanted_rect((-0.43 * length, 0.0), stroke, height, slant),
        _stroke_between((-0.46 * length, 0.47 * height), (-0.30 * length, -0.47 * height), stroke),
        _slanted_rect((-0.28 * length, 0.0), stroke, height, slant),
        # I
        _slanted_rect((-0.13 * length, 0.0), stroke, height, slant),
        # K
        _slanted_rect((0.02 * length, 0.0), stroke, height, slant),
        _stroke_between((0.05 * length, 0.02 * height), (0.21 * length, 0.48 * height), stroke),
        _stroke_between((0.05 * length, -0.02 * height), (0.23 * length, -0.48 * height), stroke),
        # E
        _slanted_rect((0.31 * length, 0.0), stroke, height, slant),
        _slanted_rect((0.42 * length, 0.40 * height), 0.19 * length, stroke, slant * 0.35),
        _slanted_rect((0.41 * length, 0.02 * height), 0.15 * length, stroke, slant * 0.25),
        _slanted_rect((0.42 * length, -0.40 * height), 0.20 * length, stroke, slant * 0.35),
    ]
    return _union_plates(plates)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nike_shoe_box")

    black = model.material("box_black", rgba=BLACK)
    white = model.material("logo_white", rgba=WHITE)
    gray = model.material("logo_gray", rgba=GRAY)
    label_bg = model.material("label_bg", rgba=LABEL_BG)

    # --- BASE TRAY (root) ---------------------------------------------------
    base = model.part("base_tray")
    base.visual(mesh_from_cadquery(_build_base(), "base_tray"), material=black)
    base.inertial = Inertial.from_geometry(
        Box((BASE_X, BASE_Y, BASE_H)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
    )

    # Spec label on the front (-Y) face, near the left (-X) end. The decal is
    # built with thickness along local +Z; rpy=(pi/2,0,0) maps that to world -Y
    # so it stands proud of the front face. Seat the base plane just inside the
    # wall surface so it reads as printed-on (no floating island).
    base.visual(
        mesh_from_cadquery(_label_panel(), "base_label_bg"),
        origin=Origin(
            xyz=(-BASE_X / 2.0 + 0.060, -BASE_Y / 2.0 + 0.0003, 0.060),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=label_bg,
        name="base_label",
    )
    # Large gray NIKE mark on the long front (-Y) face of the tray (centered-right).
    base.visual(
        mesh_from_cadquery(_wordmark_decal(0.150, 0.038), "base_wordmark"),
        origin=Origin(
            xyz=(0.052, -BASE_Y / 2.0 + 0.0003, 0.068),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gray,
        name="base_wordmark",
    )
    base.visual(
        mesh_from_cadquery(_swoosh_decal(0.185, 0.070), "base_swoosh"),
        origin=Origin(
            xyz=(0.046, -BASE_Y / 2.0 + 0.0003, 0.037),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gray,
        name="base_swoosh",
    )

    # --- LID (lift-off cap) -------------------------------------------------
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_build_lid(), "lid"), material=black)
    lid.inertial = Inertial.from_geometry(
        Box((LID_OUT_X, LID_OUT_Y, LID_SKIRT_H + LID_TOP_T)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, LID_TOP_T / 2.0 - LID_SKIRT_H / 2.0)),
    )

    # Big white NIKE wordmark across the lid top (authored in lid-local frame:
    # top surface is at local +LID_TOP_T).
    lid.visual(
        mesh_from_cadquery(_wordmark_decal(0.230, 0.044), "lid_wordmark"),
        origin=Origin(xyz=(0.010, 0.0, LID_TOP_T - 0.0003)),
        material=white,
        name="lid_wordmark",
    )
    lid.visual(
        mesh_from_cadquery(_swoosh_decal(0.245, 0.082), "lid_top_swoosh"),
        origin=Origin(xyz=(0.010, -0.050, LID_TOP_T - 0.0004)),
        material=white,
        name="lid_top_swoosh",
    )
    # Small Nike swoosh on the lid front-left corner top (white).
    lid.visual(
        mesh_from_cadquery(_swoosh_decal(0.050, 0.024), "lid_swoosh"),
        origin=Origin(xyz=(-LID_OUT_X / 2.0 + 0.045, BASE_Y / 2.0 - 0.030, LID_TOP_T - 0.0003)),
        material=white,
        name="lid_swoosh",
    )

    # --- Lift-off lid joint: PRISMATIC straight up off the base ------------
    # The lid local frame origin (top-panel underside) is placed at LID_TOP_Z in
    # the base frame. Positive q lifts the lid straight up (+Z) off the tray.
    model.articulation(
        "base_to_lid",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.3,
            lower=0.0,
            upper=0.090,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_tray")
    lid = object_model.get_part("lid")
    joint = object_model.get_articulation("base_to_lid")

    # --- Identity / proportions: long shoe box resting on the ground. -------
    blo, bhi = ctx.part_world_aabb(base)
    base_x = bhi[0] - blo[0]
    base_y = bhi[1] - blo[1]
    ctx.check(
        "box longer in X than deep in Y",
        base_x > base_y + 0.08,
        details=f"x={base_x:.3f}, y={base_y:.3f}",
    )
    ctx.check(
        "base rests on ground at z~0",
        abs(blo[2]) < 0.005,
        details=f"min_z={blo[2]:.4f}",
    )
    ctx.check(
        "base realistic shoebox length",
        0.28 < base_x < 0.40,
        details=f"len={base_x:.3f}",
    )

    # --- Hollow tray (thin walls, not a solid block). -----------------------
    ctx.check(
        "tray wall is thin (hollow)",
        WALL_T < 0.01 and WALL_T < BASE_H / 8.0,
        details=f"wall_t={WALL_T}",
    )

    # --- Joint is a straight lift-off PRISMATIC along +Z. -------------------
    ctx.check(
        "lid joint is prismatic",
        str(joint.articulation_type).lower().endswith("prismatic"),
        details=f"type={joint.articulation_type}",
    )
    axis = joint.axis
    ctx.check(
        "lid joint axis is +Z",
        abs(axis[2]) > 0.99 and abs(axis[0]) < 1e-6 and abs(axis[1]) < 1e-6,
        details=f"axis={axis}",
    )

    # --- Lid telescopes OVER the base (slightly larger XY, skirt overlaps). --
    llo, lhi = ctx.part_world_aabb(lid)
    lid_x = lhi[0] - llo[0]
    lid_y = lhi[1] - llo[1]
    ctx.check(
        "lid footprint slightly larger than base (telescopes over)",
        lid_x > base_x - 0.001 and lid_y > base_y - 0.001,
        details=f"lid=({lid_x:.3f},{lid_y:.3f}) base=({base_x:.3f},{base_y:.3f})",
    )

    # At rest (closed) the lid sits on top: its top is above the base rim, and
    # the skirt overlaps the base walls in Z (retained telescoped fit).
    with ctx.pose({joint: 0.0}):
        c_llo, c_lhi = ctx.part_world_aabb(lid)
        rest_top = c_lhi[2]
        ctx.check(
            "closed lid top sits above base rim",
            rest_top > bhi[2] - 0.001,
            details=f"lid_top={rest_top:.3f}, base_top={bhi[2]:.3f}",
        )
        # Skirt overlaps the base wall band in Z (telescoped retained insertion).
        ctx.expect_overlap(
            lid,
            base,
            axes="z",
            min_overlap=0.015,
            name="closed lid skirt overlaps base wall in Z",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.10,
            name="closed lid covers base footprint",
        )

    # --- Lifting the lid moves it straight UP and clears the base. ----------
    with ctx.pose({joint: 0.090}):
        o_llo, o_lhi = ctx.part_world_aabb(lid)
        lifted_bottom = o_llo[2]
        ctx.check(
            "lifted lid clears the base rim",
            lifted_bottom > bhi[2] - 0.002,
            details=f"lid_bottom={lifted_bottom:.3f}, base_top={bhi[2]:.3f}",
        )
        # The lid only moves in Z, not sideways.
        ctx.check(
            "lid lifts straight up (no XY drift)",
            abs(((o_llo[0] + o_lhi[0]) / 2) - ((c_llo[0] + c_lhi[0]) / 2)) < 1e-3
            and abs(((o_llo[1] + o_lhi[1]) / 2) - ((c_llo[1] + c_lhi[1]) / 2)) < 1e-3,
            details="centroid XY should be unchanged",
        )

    # --- Telescoped skirt/wall nesting is an intentional captured fit. ------
    ctx.allow_overlap(
        base,
        lid,
        reason="The lid skirt intentionally telescopes over the outside of the "
        "base tray walls at the closed pose (lift-off box retained fit).",
    )

    # --- Hero graphics present: white wordmark + swooshes. ------------------
    wordmark = lid.get_visual("lid_wordmark")
    base_swoosh = base.get_visual("base_swoosh")
    ctx.check(
        "lid carries a NIKE wordmark decal",
        wordmark is not None,
        details="lid_wordmark visual missing",
    )
    ctx.check(
        "base carries a swoosh decal",
        base_swoosh is not None,
        details="base_swoosh visual missing",
    )
    # Wordmark sits on the lid top surface (proud, near the top plane).
    wlo, whi = ctx.part_element_world_aabb(lid, elem="lid_wordmark")
    ctx.check(
        "wordmark sits on lid top surface",
        whi[2] > bhi[2],
        details=f"wordmark_top={whi[2]:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
