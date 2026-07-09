from __future__ import annotations

# Digipak-style book-fold CD case (lying open).
#
# Coordinate convention:
#   - The case lies flat in the XY plane, "up" is +Z.
#   - The spine fold line runs along Y at x=0; the tray panel is on the +X
#     side, the cover panel folds from -X over the tray about the spine.
#   - z=0 is the underside of the panels; the assembly stacks upward in +Z.
#
# Parts / articulations:
#   - tray_panel (root): cardboard board + spine strip + dark plastic disc tray
#     with raised hub, rosette teeth, and front finger notches. STATIC.
#   - cover_panel: cardboard cover that folds shut like a book about the spine.
#     REVOLUTE about the spine fold line (axis ~ +Y), opens 0 -> ~170 deg.
#   - disc: silver CD. CONTINUOUS spin about the vertical hub axis (+Z).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Inertial,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) -------------------------------------------------
CASE_W = 0.142       # closed-footprint width (matches original jewel case)
CASE_D = 0.125       # case depth (Y extent)
PANEL_T = 0.0025     # cardboard panel thickness (~2.5 mm)
SPINE_W = 0.008      # spine strip width (X)
PANEL_W = CASE_W - SPINE_W  # each panel width = 0.134 m

TRAY_W = PANEL_W - 0.010   # disc tray width (margin from panel edges)
TRAY_D = CASE_D - 0.010    # disc tray depth
TRAY_FLOOR_T = 0.0022      # tray floor thickness

# Tray board center X in tray_panel frame (board starts at x=SPINE_W)
TRAY_CENTER_X = SPINE_W + PANEL_W / 2.0

DISC_R = 0.060        # CD outer radius (120 mm disc)
DISC_T = 0.0012       # CD thickness
DISC_HOLE_R = 0.0075  # CD center hole radius

HUB_R = 0.009         # raised tray hub radius
HUB_H = 0.006         # hub height above tray floor

# Z in tray_panel frame where the disc seats (on rosette teeth top)
DISC_SEAT_Z = PANEL_T + TRAY_FLOOR_T + HUB_H * 0.7


# ---- geometry helpers --------------------------------------------------------

def _cardboard_panel(w: float, d: float, t: float, centered_z: bool = False) -> cq.Workplane:
    """Flat cardboard panel. Bottom at z=0 when centered_z=False,
    centered about z=0 when centered_z=True."""
    return cq.Workplane("XY").box(w, d, t, centered=(True, True, centered_z))


def _disc_tray(tw: float, td: float, floor_t: float) -> cq.Workplane:
    """Dark molded plastic disc tray with hub, rosette teeth, and finger notches.
    Built at origin (floor bottom at z=0); caller positions via visual origin."""
    # Tray floor
    tray = cq.Workplane("XY").box(tw, td, floor_t, centered=(True, True, False))
    # Raised circular hub boss at the center
    hub = (
        cq.Workplane("XY")
        .workplane(offset=floor_t)
        .circle(HUB_R)
        .extrude(HUB_H)
    )
    tray = tray.union(hub)
    # Rosette teeth: small radial cylinders around the hub (disc-retaining flexure)
    n_teeth = 8
    for i in range(n_teeth):
        a = 2 * math.pi * i / n_teeth
        tx = (HUB_R + 0.0015) * math.cos(a)
        ty = (HUB_R + 0.0015) * math.sin(a)
        tooth = (
            cq.Workplane("XY")
            .workplane(offset=floor_t)
            .moveTo(tx, ty)
            .circle(0.0016)
            .extrude(HUB_H * 0.7)
        )
        tray = tray.union(tooth)
    # Front finger-notch cutouts on the -Y edge (two scoops to lift the disc)
    for nx in (-0.026, 0.026):
        notch = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .moveTo(nx, -td / 2.0)
            .circle(0.013)
            .extrude(floor_t + 0.003)
        )
        tray = tray.cut(notch)
    return tray


def _disc_solid() -> cq.Workplane:
    """Silver CD: thin cylinder with central hole."""
    return (
        cq.Workplane("XY")
        .circle(DISC_R)
        .extrude(DISC_T)
        .faces(">Z")
        .workplane()
        .hole(DISC_HOLE_R * 2.0)
    )


# ---- build -------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cd_digipak_case")

    cardboard = model.material("cardboard", rgba=(0.76, 0.70, 0.58, 1.0))
    cover_card = model.material("cover_cardboard", rgba=(0.22, 0.26, 0.36, 1.0))
    tray_dark = model.material("tray_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    silver = model.material("disc_silver", rgba=(0.80, 0.82, 0.85, 1.0))

    # ---- tray_panel (root): cardboard board + spine + disc tray ----
    tray_panel = model.part("tray_panel")

    # Cardboard tray board (+X side from spine)
    tray_panel.visual(
        mesh_from_cadquery(_cardboard_panel(PANEL_W, CASE_D, PANEL_T), "tray_board"),
        origin=Origin(xyz=(SPINE_W + PANEL_W / 2.0, 0.0, 0.0)),
        material=cardboard,
        name="tray_board",
    )
    # Spine strip at the fold line (connects the two panels)
    tray_panel.visual(
        mesh_from_cadquery(_cardboard_panel(SPINE_W, CASE_D, PANEL_T), "spine"),
        origin=Origin(xyz=(SPINE_W / 2.0, 0.0, 0.0)),
        material=cardboard,
        name="spine",
    )
    # Dark molded plastic disc tray mounted on top of the tray board
    tray_panel.visual(
        mesh_from_cadquery(_disc_tray(TRAY_W, TRAY_D, TRAY_FLOOR_T), "disc_tray"),
        origin=Origin(xyz=(TRAY_CENTER_X, 0.0, PANEL_T)),
        material=tray_dark,
        name="disc_tray",
    )
    tray_panel.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, PANEL_T + TRAY_FLOOR_T + HUB_H)),
        mass=0.080,
        origin=Origin(xyz=(SPINE_W + PANEL_W / 2.0, 0.0, PANEL_T / 2.0)),
    )

    # ---- cover_panel: cardboard cover that folds about the spine ----
    cover_panel = model.part("cover_panel")

    # Cover board: centered_z=True so it spans z=-PANEL_T/2..+PANEL_T/2 in local
    # frame. Since the part origin is at the joint origin (z=PANEL_T/2 in world),
    # this maps to z=0..PANEL_T in world when the fold is at q=0.
    cover_panel.visual(
        mesh_from_cadquery(
            _cardboard_panel(PANEL_W, CASE_D, PANEL_T, centered_z=True),
            "cover_board",
        ),
        origin=Origin(xyz=(-PANEL_W / 2.0, 0.0, 0.0)),
        material=cover_card,
        name="cover_board",
    )
    cover_panel.inertial = Inertial.from_geometry(
        Box((PANEL_W, CASE_D, PANEL_T)),
        mass=0.040,
        origin=Origin(xyz=(-PANEL_W / 2.0, 0.0, 0.0)),
    )

    # Spine fold articulation: cover folds about the Y axis at the left edge of
    # the spine (x=0). At q=0 cover is flat open (-X); positive q swings it up
    # and over the tray (+Z then +X).
    model.articulation(
        "spine_fold",
        ArticulationType.REVOLUTE,
        parent=tray_panel,
        child=cover_panel,
        origin=Origin(xyz=(0.0, 0.0, PANEL_T / 2.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0,
            lower=0.0, upper=math.radians(170.0),
        ),
    )

    # ---- disc: silver CD, continuous spin about the vertical hub axis ----
    disc = model.part("disc")
    disc.visual(
        mesh_from_cadquery(_disc_solid(), "disc_body"),
        material=silver,
        name="disc_body",
    )
    # Small off-center label marker so rotation is visually/numerically detectable
    disc.visual(
        Cylinder(radius=0.004, length=DISC_T),
        origin=Origin(xyz=(0.030, 0.0, DISC_T / 2.0)),
        material=tray_dark,
        name="disc_marker",
    )
    disc.inertial = Inertial.from_geometry(
        Cylinder(radius=DISC_R, length=DISC_T),
        mass=0.016,
        origin=Origin(xyz=(0.0, 0.0, DISC_T / 2.0)),
    )
    # Disc center hole drops over the hub; rests on the rosette teeth.
    model.articulation(
        "hub_to_disc",
        ArticulationType.CONTINUOUS,
        parent=tray_panel,
        child=disc,
        origin=Origin(xyz=(TRAY_CENTER_X, 0.0, DISC_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


# ---- tests -------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tray_panel = object_model.get_part("tray_panel")
    cover_panel = object_model.get_part("cover_panel")
    disc = object_model.get_part("disc")
    spine_fold = object_model.get_articulation("spine_fold")
    disc_joint = object_model.get_articulation("hub_to_disc")

    # --- panels are opaque cardboard (not transparent plastic) ---
    tray_mat = tray_panel.get_visual("tray_board").material
    tray_alpha = tray_mat.rgba[3] if tray_mat.rgba is not None else 1.0
    ctx.check(
        "tray panel is opaque cardboard (alpha >= 0.95)",
        tray_alpha >= 0.95,
        details=f"tray_board alpha={tray_alpha}",
    )
    cover_mat = cover_panel.get_visual("cover_board").material
    cover_alpha = cover_mat.rgba[3] if cover_mat.rgba is not None else 1.0
    ctx.check(
        "cover panel is opaque cardboard (alpha >= 0.95)",
        cover_alpha >= 0.95,
        details=f"cover_board alpha={cover_alpha}",
    )

    # --- spine fold: cover swings upward and over the tray ---
    closed_top = ctx.part_world_aabb(cover_panel)[1][2]
    closed_left_x = ctx.part_world_aabb(cover_panel)[0][0]
    with ctx.pose({spine_fold: math.radians(90.0)}):
        open_aabb = ctx.part_world_aabb(cover_panel)
        open_top = open_aabb[1][2]
        open_left_x = open_aabb[0][0]
    ctx.check(
        "cover folds upward from flat open position",
        open_top > closed_top + 0.03,
        details=f"closed_top={closed_top:.4f}, open_top={open_top:.4f}",
    )
    ctx.check(
        "folding swings cover from -X toward +X (over the tray side)",
        open_left_x > closed_left_x + 0.02,
        details=f"closed_left_x={closed_left_x:.4f}, open_left_x={open_left_x:.4f}",
    )

    # Spine fold axis runs along Y (book-style fold)
    ctx.check(
        "spine fold axis runs along Y",
        abs(spine_fold.axis[1]) > 0.9,
        details=f"axis={spine_fold.axis}",
    )

    # --- disc spins about vertical hub axis ---
    marker_rest = ctx.part_element_world_aabb(disc, elem="disc_marker")
    rest_cx = 0.5 * (marker_rest[0][0] + marker_rest[1][0])
    rest_cy = 0.5 * (marker_rest[0][1] + marker_rest[1][1])
    with ctx.pose({disc_joint: math.radians(90.0)}):
        marker_q = ctx.part_element_world_aabb(disc, elem="disc_marker")
        q_cx = 0.5 * (marker_q[0][0] + marker_q[1][0])
        q_cy = 0.5 * (marker_q[0][1] + marker_q[1][1])
    ctx.check(
        "disc spin moves the off-center marker around the hub axis",
        abs(q_cx - rest_cx) > 0.02 and abs(q_cy - rest_cy) > 0.02,
        details=f"rest=({rest_cx:.4f},{rest_cy:.4f}) quarter=({q_cx:.4f},{q_cy:.4f})",
    )
    ctx.check(
        "disc spin axis is vertical (+Z)",
        abs(disc_joint.axis[2]) > 0.9,
        details=f"axis={disc_joint.axis}",
    )

    # --- disc sits on the tray hub, centered over it ---
    ctx.allow_overlap(
        disc,
        tray_panel,
        elem_a="disc_body",
        elem_b="disc_tray",
        reason="The CD center hole drops over the raised tray hub/rosette; the hub "
               "intentionally pokes through the disc bore so the disc is captured.",
    )
    disc_pos = ctx.part_world_position(disc)
    ctx.check(
        "disc is centered over the tray hub axis",
        abs(disc_pos[0] - TRAY_CENTER_X) < 0.005 and abs(disc_pos[1]) < 0.005,
        details=f"disc origin={disc_pos}, expected x~{TRAY_CENTER_X:.4f}",
    )
    disc_bottom = ctx.part_world_aabb(disc)[0][2]
    ctx.check(
        "disc rests on the tray hub above the panel surface",
        PANEL_T < disc_bottom < PANEL_T + TRAY_FLOOR_T + HUB_H + 0.003,
        details=f"disc_bottom={disc_bottom:.4f}, panel_top={PANEL_T}",
    )

    # --- disc tray sits within the tray board footprint ---
    ctx.expect_within(
        tray_panel,
        tray_panel,
        axes="xy",
        inner_elem="disc_tray",
        outer_elem="tray_board",
        margin=0.001,
        name="disc tray sits within the tray board footprint",
    )

    # --- cover contacts spine at the fold line when open flat ---
    ctx.expect_contact(
        cover_panel,
        tray_panel,
        elem_a="cover_board",
        elem_b="spine",
        contact_tol=0.001,
        name="cover contacts spine at the fold line when open",
    )

    return ctx.report()


object_model = build_object_model()
