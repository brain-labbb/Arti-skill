from __future__ import annotations

# Slimline clear-plastic CD jewel case (lying open).
#
# This is a slimline fork of the standard jewel case: the base is a thin flat
# clear plate (no deep frame skirt), and the lid doubles as the tray-carrying
# half — the dark inner disc tray with hub, rosette teeth, and finger notches
# is mounted on the lid's inner face.  The closed height is roughly half that
# of a standard jewel case (~5 mm vs ~10 mm).
#
# Coordinate convention:
#   - The case lies flat in the XY plane, "up" is +Z.
#   - X is the case width (~0.142 m), Y is the case depth (~0.125 m).
#   - The rear hinge runs left-right along the back +Y edge; the lid swings
#     about that hinge (axis parallel to X).
#   - The front finger-notches are on the -Y edge.
#   - z=0 is the underside of the base plate; the assembly stacks upward in +Z.
#
# Parts / articulations:
#   - base (root): thin clear base plate. STATIC.
#   - lid: clear cover shell + dark inner tray with hub, rosette, and finger
#     notches on its inner face. REVOLUTE about the rear hinge (axis ~ +X),
#     opens 0 -> ~70 deg.
#   - disc: silver CD. CONTINUOUS spin about the vertical hub axis (+Z),
#     clipped onto the lid-carried hub.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ------------------------------------------------
CASE_W = 0.142  # X extent (same footprint as standard case)
CASE_D = 0.125  # Y extent

BASE_PLATE_T = 0.001  # thin clear base plate (1 mm)
COVER_T = 0.001       # lid cover-plate thickness (1 mm)
TRAY_T = 0.0008       # tray floor thickness on lid inner face (0.8 mm)
HUB_H = 0.0022        # hub height (shorter than standard for slimline)
LID_WALL = 0.001      # lid perimeter wall thickness

# Total lid assembly depth below the hinge line.
LID_DEPTH = COVER_T + TRAY_T + HUB_H  # 4 mm

HINGE_Y = CASE_D / 2.0                # rear hinge line at +Y edge
HINGE_Z = BASE_PLATE_T + LID_DEPTH    # hinge at top of closed case (~5 mm)

DISC_R = 0.060       # CD outer radius (120 mm disc)
DISC_T = 0.0012      # CD thickness
DISC_HOLE_R = 0.0075 # CD center hole radius

HUB_R = 0.009        # hub radius (same as standard)

# ---- shared geometry helpers ------------------------------------------------

def _thin_plate(w: float, d: float, t: float) -> cq.Workplane:
    """Flat rectangular plate from z=0 to z=t, centered in XY."""
    return cq.Workplane("XY").box(w, d, t, centered=(True, True, False))


def _tray_with_hub(
    tw: float,
    td: float,
    floor_t: float,
    floor_z_top: float,
    hub_r: float,
    hub_h: float,
) -> cq.Workplane:
    """Build a rectangular tray floor with a center hub, rosette teeth, and
    two front finger-notch cutouts.  The hub and teeth extend downward (-Z)
    from the tray floor bottom (slimline lid-carried tray convention).

    *floor_z_top* is the Z of the tray floor's upper face.
    The floor extends from z=floor_z_top-floor_t to z=floor_z_top.
    The hub extends from z=floor_z_top-floor_t downward by hub_h.
    """
    floor_z_bot = floor_z_top - floor_t
    # Tray floor plate.
    tray = (
        cq.Workplane("XY")
        .workplane(offset=floor_z_bot)
        .box(tw, td, floor_t, centered=(True, True, False))
    )
    # Hub boss: extends downward from tray floor bottom.
    hub_z_bot = floor_z_bot - hub_h
    hub = (
        cq.Workplane("XY")
        .workplane(offset=hub_z_bot)
        .circle(hub_r)
        .extrude(hub_h)
    )
    tray = tray.union(hub)

    # Rosette teeth: small radial cylinders around the hub, extending
    # downward from the tray floor bottom (same direction as hub).
    n_teeth = 8
    tooth_h = hub_h * 0.7
    teeth_z_bot = floor_z_bot - tooth_h
    for i in range(n_teeth):
        a = 2 * math.pi * i / n_teeth
        tx = (hub_r + 0.0015) * math.cos(a)
        ty = (hub_r + 0.0015) * math.sin(a)
        tooth = (
            cq.Workplane("XY")
            .workplane(offset=teeth_z_bot)
            .moveTo(tx, ty)
            .circle(0.0016)
            .extrude(tooth_h)
        )
        tray = tray.union(tooth)

    # Front finger-notch cutouts on the -Y edge.
    notch_z = floor_z_bot - 0.001
    notch = cq.Workplane("XY").workplane(offset=notch_z)
    for nx in (-0.026, 0.026):
        notch = (
            notch.moveTo(nx, -td / 2.0)
            .circle(0.013)
            .extrude(floor_t + 0.003)
        )
    tray = tray.cut(notch)
    return tray


def _disc_solid() -> cq.Workplane:
    """Silver CD: thin cylinder with a central hole."""
    return (
        cq.Workplane("XY")
        .circle(DISC_R)
        .extrude(DISC_T)
        .faces(">Z")
        .workplane()
        .hole(DISC_HOLE_R * 2.0)
    )


# ---- part geometry ----------------------------------------------------------

def _base_plate_geom() -> cq.Workplane:
    """Thin flat clear base plate (no deep frame skirt)."""
    return _thin_plate(CASE_W, CASE_D, BASE_PLATE_T)


def _lid_cover_geom() -> cq.Workplane:
    """Clear lid cover: thin top plate + perimeter walls, open at bottom.

    In the lid local frame the hinge is at the origin (rear +Y edge).
    The cover extends from z=-LID_DEPTH to z=0.
    Top plate: z = -COVER_T to z = 0  (COVER_T thick).
    Walls:     z = -LID_DEPTH to z = -COVER_T  (LID_WALL thick).
    """
    outer = (
        cq.Workplane("XY")
        .workplane(offset=-LID_DEPTH)
        .box(CASE_W, CASE_D, LID_DEPTH, centered=(True, True, False))
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-LID_DEPTH)
        .box(
            CASE_W - 2 * LID_WALL,
            CASE_D - 2 * LID_WALL,
            LID_DEPTH - COVER_T,
            centered=(True, True, False),
        )
    )
    return outer.cut(inner)


def _lid_tray_geom() -> cq.Workplane:
    """Dark tray with hub, rosette, and finger notches, mounted on the lid
    inner face (hub protrudes downward toward the base plate when closed).
    """
    tw = CASE_W - 2 * LID_WALL - 0.0004
    td = CASE_D - 2 * LID_WALL - 0.0004
    floor_z_top = -COVER_T  # tray sits just below the cover plate
    return _tray_with_hub(tw, td, TRAY_T, floor_z_top, HUB_R, HUB_H)


# ---- object model -----------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slimline_cd_case")

    clear = model.material("clear_plastic", rgba=(0.55, 0.62, 0.70, 0.28))
    tray_dark = model.material("tray_dark", rgba=(0.08, 0.08, 0.10, 1.0))
    silver = model.material("disc_silver", rgba=(0.80, 0.82, 0.85, 1.0))

    # ---- base (root): thin clear base plate ----
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_plate_geom(), "base_plate"),
        material=clear,
        name="base_plate",
    )
    base.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, BASE_PLATE_T)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_T / 2.0)),
    )

    # ---- lid: clear cover + dark tray with hub (tray-carrying half) ----
    lid = model.part("lid")
    # Offset lid visuals by -HINGE_Y so the cover/tray are centered on the
    # case (the lid part origin sits on the hinge line at +Y).
    lid.visual(
        mesh_from_cadquery(_lid_cover_geom(), "lid_cover"),
        material=clear,
        name="lid_cover",
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.0)),
    )
    lid.visual(
        mesh_from_cadquery(_lid_tray_geom(), "lid_tray"),
        material=tray_dark,
        name="lid_tray",
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.0)),
    )
    lid.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, LID_DEPTH)),
        mass=0.030,
        origin=Origin(xyz=(0.0, -HINGE_Y, -LID_DEPTH / 2.0)),
    )

    # Hinge at the rear +Y edge.  At q=0 the lid is closed (flat on top of
    # the base plate).  Axis +X so positive q lifts the front edge upward.
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0,
            lower=0.0, upper=math.radians(70.0),
        ),
    )

    # ---- disc: silver CD, continuous spin on the lid-carried hub ----
    disc = model.part("disc")
    disc.visual(
        mesh_from_cadquery(_disc_solid(), "disc_body"),
        material=silver,
        name="disc_body",
    )
    # Small off-center label marker so rotation is visually/numerically detectable.
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

    # Disc clips onto the hub rosette on the lid inner face.  The hub extends
    # downward from z=-COVER_T-TRAY_T; the disc sits at ~70% of hub depth.
    disc_z_local = -(COVER_T + TRAY_T + HUB_H * 0.7)
    model.articulation(
        "hub_to_disc",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=disc,
        # Disc is centered on the case (y=-HINGE_Y in lid frame = y=0 world).
        origin=Origin(xyz=(0.0, -HINGE_Y, disc_z_local)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


# ---- helpers for tests ------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


# ---- tests ------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    disc = object_model.get_part("disc")
    lid_joint = object_model.get_articulation("base_to_lid")
    disc_joint = object_model.get_articulation("hub_to_disc")

    # --- slimline closed height is roughly half of a standard jewel case ---
    base_top = ctx.part_world_aabb(base)[1][2]
    lid_top = ctx.part_world_aabb(lid)[1][2]
    closed_height = max(base_top, lid_top)
    ctx.check(
        "slimline closed height is roughly half of standard (~5 mm)",
        0.003 < closed_height < 0.008,
        details=f"closed_height={closed_height:.4f} m",
    )

    # --- base is a thin flat plate (no deep frame skirt) ---
    base_dims = _ext(ctx.part_world_aabb(base))
    ctx.check(
        "base plate is thin (Z extent < 2 mm, no deep frame)",
        base_dims[2] < 0.002,
        details=f"base Z extent={base_dims[2]:.4f} m",
    )

    # --- tray is on the lid, not the base ---
    tray_visual = lid.get_visual("lid_tray")
    ctx.check(
        "tray is a visual on the lid (lid carries the tray)",
        tray_visual is not None,
        details="lid_tray visual not found on lid part",
    )
    base_visuals = [v.name for v in base.visuals]
    ctx.check(
        "base has no inner_tray visual (tray moved to lid)",
        "inner_tray" not in base_visuals,
        details=f"base visuals: {base_visuals}",
    )

    # --- lid and base are clear/transparent ---
    lid_mat = lid.get_visual("lid_cover").material
    alpha = lid_mat.rgba[3] if lid_mat.rgba is not None else 1.0
    ctx.check(
        "lid cover is transparent (alpha < 1)",
        alpha < 0.6,
        details=f"lid alpha={alpha}",
    )
    base_mat = base.get_visual("base_plate").material
    base_alpha = base_mat.rgba[3] if base_mat.rgba is not None else 1.0
    ctx.check(
        "base plate is clear plastic (alpha < 1)",
        base_alpha < 0.6,
        details=f"base alpha={base_alpha}",
    )

    # --- lid hinges about the rear edge: opening lifts the front edge up ---
    closed_top = ctx.part_world_aabb(lid)[1][2]
    closed_front_y = ctx.part_world_aabb(lid)[0][1]
    with ctx.pose({lid_joint: math.radians(60.0)}):
        open_aabb = ctx.part_world_aabb(lid)
        open_top = open_aabb[1][2]
        open_front_y = open_aabb[0][1]
    ctx.check(
        "lid swings open and lifts upward",
        open_top > closed_top + 0.02,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )
    ctx.check(
        "opening swings the free edge toward the hinge",
        open_front_y > closed_front_y + 0.01,
        details=f"closed_front_y={closed_front_y}, open_front_y={open_front_y}",
    )

    # --- hinge axis runs along X (left-right) ---
    ctx.check(
        "hinge axis runs along X (left-right)",
        abs(lid_joint.axis[0]) > 0.9,
        details=f"axis={lid_joint.axis}",
    )

    # --- disc spins about the vertical hub axis ---
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

    # --- disc moves with the lid when opened (disc is parented to lid) ---
    disc_closed_z = ctx.part_world_position(disc)[2]
    with ctx.pose({lid_joint: math.radians(60.0)}):
        disc_open_z = ctx.part_world_position(disc)[2]
    ctx.check(
        "disc travels with the lid when opened (parented to lid)",
        abs(disc_open_z - disc_closed_z) > 0.02,
        details=f"closed_z={disc_closed_z:.4f}, open_z={disc_open_z:.4f}",
    )

    # --- disc sits on the lid-carried hub (centered, within case footprint) ---
    ctx.allow_overlap(
        disc,
        lid,
        elem_a="disc_body",
        elem_b="lid_tray",
        reason="The CD center hole drops over the raised lid-carried hub/rosette; "
        "the hub intentionally pokes through the disc bore so the disc is captured.",
    )
    ctx.expect_within(
        disc,
        lid,
        axes="xy",
        inner_elem="disc_body",
        outer_elem="lid_cover",
        margin=0.002,
        name="disc sits within the lid cover footprint",
    )
    disc_pos = ctx.part_world_position(disc)
    ctx.check(
        "disc is centered over the hub axis",
        abs(disc_pos[0]) < 0.005 and abs(disc_pos[1]) < 0.005,
        details=f"disc origin={disc_pos}",
    )

    # --- tray nests inside the lid cover shell ---
    ctx.allow_overlap(
        lid,
        lid,
        elem_a="lid_tray",
        elem_b="lid_cover",
        reason="The dark tray is press-fit inside the clear lid cover shell; "
        "its floor edges intentionally seat into the cover walls.",
    )
    ctx.expect_within(
        lid,
        lid,
        axes="xy",
        inner_elem="lid_tray",
        outer_elem="lid_cover",
        margin=0.002,
        name="lid tray nests within the lid cover footprint",
    )

    # --- hub/tooth features exist on the tray ---
    ctx.check(
        "tray visual exists on the lid part",
        lid.get_visual("lid_tray") is not None,
        details="lid_tray visual missing",
    )

    return ctx.report()


object_model = build_object_model()
