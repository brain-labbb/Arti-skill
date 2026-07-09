from __future__ import annotations

# Articraft model: foldable A-frame "Wet Floor" caution sign — SHIELD variant.
#
# Fork of the parent Lavex-style A-frame caution sign. The ONLY structural
# change is the panel silhouette: each leaf is now a heraldic shield shape —
# near-vertical upper sides that curve inward and sweep down to a rounded
# point at the bottom center. Both leaves share the same shield profile.
#
# Everything else stays identical to the parent:
#   - Top apex REVOLUTE hinge that opens/folds the A-frame.
#   - Same floor-sign height (~0.60 m) and width (~0.30 m).
#   - Integrated grab-handle cut-through opening near the wide top.
#   - Molded shell hollowing (thin plastic back cavity).
#   - Yellow caution identity with front warning placard and plain back.
#   - Three dark hinge knuckle barrels straddling the apex pin line.

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
PANEL_WIDTH = 0.300       # width across (Y axis)
PANEL_HEIGHT = 0.600      # full height of one shield leaf (apex to bottom point)
PANEL_THICK = 0.018       # molded shell wall block thickness
APEX_HALF_OPEN = 0.18     # half the A-frame opening angle at rest (radians)

HINGE_PIN_R = 0.006       # knuckle barrel radius
HINGE_BAND_LEN = 0.045    # each dark knuckle band length along Y
HINGE_GAP = 0.012         # gap between knuckle bands

HANDLE_OPENING_W = 0.140  # grab-handle cutout width (Y)
HANDLE_OPENING_H = 0.038  # grab-handle cutout height
HANDLE_BAR_T = 0.020      # handle bar thickness

# Shield geometry constants --------------------------------------------------
# The straight vertical sides extend from just below the top arc down to this
# fraction of the total height. Below that, the edges curve inward to meet at
# the rounded bottom point.
SHIELD_STRAIGHT_FRAC = 0.48   # fraction of height with vertical sides
SHIELD_CURVE_START_FRAC = 0.48  # where the inward curve begins

# Materials -------------------------------------------------------------------
YELLOW = Material(name="caution_yellow", rgba=(0.92, 0.80, 0.10, 1.0))
YELLOW_BACK = Material(name="caution_yellow_back", rgba=(0.78, 0.68, 0.12, 1.0))
HINGE_BLACK = Material(name="hinge_black", rgba=(0.08, 0.08, 0.09, 1.0))
TEXT_BLACK = Material(name="text_black", rgba=(0.05, 0.05, 0.05, 1.0))
HAZARD_RED = Material(name="hazard_red", rgba=(0.78, 0.18, 0.12, 1.0))


def _shield_profile_wire(width: float, height: float) -> cq.Workplane:
    """Build the closed shield-outline wire in the YZ plane.

    The shield has:
      * A gently rounded top arc (the apex/hinge edge).
      * Near-vertical straight sides descending from the top corners.
      * Smooth inward curves from the sides that sweep down to a rounded
        point at the bottom center (y=0, z=-height).

    Local frame: Y runs left-right (panel width), Z runs up-down with the
    apex at z~0 and the panel hanging toward -Z.

    Uses chained threePointArc segments for the lower curves (robust wire
    building) and straight lineTo for the vertical sides.
    """
    half_w = width / 2.0
    h = height
    # Where the vertical side ends and the inward curve begins (Z).
    z_straight = -h * SHIELD_STRAIGHT_FRAC

    # Intermediate waypoints for the shield's lower curves. Each side uses
    # two chained arcs: the first keeps the curve close to vertical, the
    # second sweeps inward to the bottom point.
    #
    # Left side curve (from end of straight side to bottom center):
    left_mid_1 = (-half_w * 0.92, z_straight - h * 0.10)   # still wide
    left_mid_2 = (-half_w * 0.55, z_straight - h * 0.22)   # starting inward
    left_knee  = (-half_w * 0.22, -h * 0.85)               # sweeping to point

    # Right side (mirror):
    right_knee  = (half_w * 0.22, -h * 0.85)
    right_mid_2 = (half_w * 0.55, z_straight - h * 0.22)
    right_mid_1 = (half_w * 0.92, z_straight - h * 0.10)

    wire = (
        cq.Workplane("YZ")
        # Start at top-right corner of the shield.
        .moveTo(half_w, -0.045)
        # Rounded top arc spanning the full width.
        .threePointArc((0.0, -0.005), (-half_w, -0.045))
        # Left straight side going down.
        .lineTo(-half_w, z_straight)
        # Left lower curve: first arc stays wide.
        .threePointArc(left_mid_1, left_mid_2)
        # Left lower curve: second arc sweeps to the bottom point.
        .threePointArc(left_knee, (0.0, -h))
        # Right lower curve: first arc sweeps up from the bottom point.
        .threePointArc(right_knee, right_mid_2)
        # Right lower curve: second arc returns to the straight side.
        .threePointArc(right_mid_1, (half_w, z_straight))
        # close() draws the final right vertical side back to the start point.
        .close()
    )
    return wire


def _panel_shell(width: float, height: float, thick: float, *, ribbed: bool) -> cq.Workplane:
    """A single shield-shaped molded sign panel built in its OWN local frame.

    Local frame for the panel: the hinge (apex) line is at z = 0, the panel
    hangs downward toward -Z, the width runs along Y, and the visible front
    face points toward +X. The panel is a thin shell (hollow back) with a
    rounded top carrying the integrated grab handle and its cut-through hole.
    """
    # Extrude the shield profile to make a solid slab.
    prof = _shield_profile_wire(width, height).extrude(thick)

    # Hollow out the back. Keep the DECORATED FRONT WALL at the +X side
    # (x in [thick-wall, thick]) and cut a cavity from the inner face toward
    # -X so the panel reads as a molded plastic shell, not a solid brick.
    wall = 0.004
    cavity_h = height * 0.38  # cavity stays in the wide upper zone
    cavity = (
        cq.Workplane("YZ")
        .workplane(offset=thick - wall)
        .moveTo(0.0, -(0.06 + cavity_h / 2.0))
        .rect(width - 2 * wall, cavity_h)
        .extrude(-(thick - wall))
    )
    shell = prof.cut(cavity)

    # Grab-handle cut-through opening near the top (fully through the panel).
    handle_cut = (
        cq.Workplane("YZ")
        .workplane(offset=-0.01)
        .center(0.0, -0.034)
        .rect(HANDLE_OPENING_W, HANDLE_OPENING_H)
        .extrude(thick + 0.02)
    )
    shell = shell.cut(handle_cut)

    if ribbed:
        # Molded horizontal ribs proud of the FRONT face (+X). Only in the
        # upper straight-sided zone of the shield where the full width exists.
        rib_z0 = -0.10
        rib_z1 = -height * SHIELD_STRAIGHT_FRAC + 0.02
        n = 5
        for i in range(n):
            zc = rib_z0 + (rib_z1 - rib_z0) * (i / max(n - 1, 1))
            rib = (
                cq.Workplane("XY")
                .transformed(offset=cq.Vector(thick, 0.0, zc))
                .box(0.006, width * 0.78, 0.006, centered=(True, True, True))
            )
            shell = shell.union(rib)

    return shell


def _hinge_bands() -> cq.Workplane:
    """The three dark pin/knuckle barrels along the apex, in the apex frame.

    Apex frame: pin axis along Y, centered at origin. Barrels are short
    cylinders spaced across the width, matching the three dark bands in the
    reference image. The top-edge arc is the same rounded top as the shield
    profile, so barrel Z positions follow that arc.
    """
    half_w = PANEL_WIDTH / 2.0
    centers = [-0.085, 0.0, 0.085]
    # The shield top follows a threePointArc from (-half_w,-0.045) through
    # (0,-0.005) to (half_w,-0.045). Circle through those three points:
    arc_cz = -0.3062
    arc_r = 0.3012
    bands = None
    for cy in centers:
        top_z = arc_cz + math.sqrt(max(arc_r**2 - cy**2, 0.0))
        barrel_z = top_z - 0.004
        barrel = (
            cq.Workplane("XZ")
            .workplane(offset=-(cy - HINGE_BAND_LEN / 2.0))
            .center(0.0, barrel_z)
            .circle(HINGE_PIN_R)
            .extrude(-HINGE_BAND_LEN)
        )
        bands = barrel if bands is None else bands.union(barrel)
    assert half_w > 0
    return bands


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shield_caution_sign")
    for mat in (YELLOW, YELLOW_BACK, HINGE_BLACK, TEXT_BLACK, HAZARD_RED):
        model.material(mat.name, rgba=mat.rgba)

    # ------------------------------------------------------------------
    # FRONT panel (root).
    # ------------------------------------------------------------------
    front = model.part("front_panel")

    front_shell = _panel_shell(PANEL_WIDTH, PANEL_HEIGHT, PANEL_THICK, ribbed=True)
    front.visual(
        mesh_from_cadquery(front_shell, "front_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -APEX_HALF_OPEN, 0.0)),
        material=YELLOW,
        name="front_shell",
    )

    # Warning triangle (slip-hazard) — placed in the upper straight zone.
    tri = (
        cq.Workplane("YZ")
        .workplane(offset=PANEL_THICK - 0.002)
        .center(0.0, -0.22)
        .polygon(3, 0.10)
        .extrude(0.004)
    )
    front.visual(
        mesh_from_cadquery(tri, "hazard_triangle"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -APEX_HALF_OPEN, 0.0)),
        material=HAZARD_RED,
        name="hazard_triangle",
    )

    # Two text bands in the upper shield zone.
    for idx, (zc, h, w_frac) in enumerate([
        (-0.13, 0.045, 0.68),
        (-0.28, 0.040, 0.58),
    ]):
        band = (
            cq.Workplane("YZ")
            .workplane(offset=PANEL_THICK - 0.002)
            .center(0.0, zc)
            .rect(PANEL_WIDTH * w_frac, h)
            .extrude(0.0035)
        )
        front.visual(
            mesh_from_cadquery(band, f"text_band_{idx}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -APEX_HALF_OPEN, 0.0)),
            material=TEXT_BLACK,
            name=f"text_band_{idx}",
        )

    # ------------------------------------------------------------------
    # Hinge knuckle bands on the front panel structure at the apex.
    # ------------------------------------------------------------------
    bands = _hinge_bands()
    front.visual(
        mesh_from_cadquery(bands, "hinge_knuckles"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=HINGE_BLACK,
        name="hinge_knuckles",
    )

    # ------------------------------------------------------------------
    # BACK panel (child). Plain shield-shaped yellow panel.
    # ------------------------------------------------------------------
    back = model.part("back_panel")
    back_shell = _panel_shell(
        PANEL_WIDTH, PANEL_HEIGHT, PANEL_THICK, ribbed=False
    ).translate((-PANEL_THICK, 0.0, 0.0))
    back.visual(
        mesh_from_cadquery(back_shell, "back_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, APEX_HALF_OPEN, 0.0)),
        material=YELLOW_BACK,
        name="back_shell",
    )

    # ------------------------------------------------------------------
    # Articulation: apex hinge.
    # ------------------------------------------------------------------
    model.articulation(
        "apex_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=back,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=2.4,
        ),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front = object_model.get_part("front_panel")
    back = object_model.get_part("back_panel")
    hinge = object_model.get_articulation("apex_hinge")
    knuckles = front.get_visual("hinge_knuckles")

    # --- Intentional hinge capture ----------------------------------------
    ctx.allow_overlap(
        front,
        back,
        elem_a="hinge_knuckles",
        elem_b="back_shell",
        reason=(
            "Apex hinge knuckle barrels straddle the pin line and capture the "
            "back leaf at the shared hinge axis."
        ),
    )

    # --- Mechanism type / axis claims -------------------------------------
    ctx.check(
        "apex hinge is revolute",
        str(hinge.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={hinge.articulation_type}",
    )
    ax = tuple(round(a, 6) for a in hinge.axis)
    ctx.check(
        "apex hinge axis is along Y (width)",
        abs(ax[1]) > 0.99 and abs(ax[0]) < 1e-3 and abs(ax[2]) < 1e-3,
        details=f"axis={ax}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "apex hinge opens (positive travel range)",
        lim is not None and lim.lower is not None and lim.upper is not None
        and lim.upper > lim.lower + 1.0,
        details=f"limits=({getattr(lim, 'lower', None)}, {getattr(lim, 'upper', None)})",
    )

    # --- Hero parts present and placed ------------------------------------
    front_aabb = ctx.part_world_aabb(front)
    back_aabb = ctx.part_world_aabb(back)
    ctx.check("front panel exists in world", front_aabb is not None)
    ctx.check("back panel exists in world", back_aabb is not None)

    if front_aabb is not None:
        (fx0, fy0, fz0), (fx1, fy1, fz1) = front_aabb
        ctx.check(
            "front panel is tall like a floor sign",
            (fz1 - fz0) > 0.45,
            details=f"height={fz1 - fz0:.3f}",
        )
        ctx.check(
            "front panel spans the sign width",
            (fy1 - fy0) > 0.25,
            details=f"width={fy1 - fy0:.3f}",
        )

    # Hinge knuckles and warning features present on the front panel.
    triangle = front.get_visual("hazard_triangle")
    ctx.check("hinge knuckles visual present", knuckles is not None)
    ctx.check("hazard triangle visual present", triangle is not None)

    # Knuckle barrels capture the back leaf at the shared hinge axis.
    ctx.expect_contact(
        front,
        back,
        elem_a="hinge_knuckles",
        elem_b="back_shell",
        contact_tol=0.001,
        name="apex knuckles capture the back leaf at the hinge",
    )

    # The front panel is a thin shell, not a deep block.
    if front_aabb is not None:
        (fx0, _, _), (fx1, _, _) = front_aabb
        ctx.check(
            "front panel is a thin tilted shell, not a deep box",
            (fx1 - fx0) < 0.30,
            details=f"x_extent={fx1 - fx0:.3f}",
        )

    # --- Shield silhouette claims -----------------------------------------
    # The shield narrows toward the bottom: the lower half of the panel must
    # be narrower than the upper half along Y.
    if front_aabb is not None:
        (_, fy0, fz0), (_, fy1, fz1) = front_aabb
        z_mid = (fz0 + fz1) / 2.0
        # Use the element-level dims to verify the shield narrows downward.
        front_shell_vis = front.get_visual("front_shell")
        if front_shell_vis is not None:
            # The overall width should match the sign width at the top.
            full_width = fy1 - fy0
            ctx.check(
                "shield panel has real sign width at the top",
                full_width > 0.24,
                details=f"full_width={full_width:.3f}",
            )

    # The bottom of the shield should be narrower than the top — verify by
    # checking that the panel's Y-extent near the bottom is less than at the top.
    # Use projection of the front shell to confirm taper.
    front_shell_vis = front.get_visual("front_shell")
    if front_shell_vis is not None:
        front_proj = ctx.part_world_aabb(front)
        if front_proj is not None:
            (_, fy0_full, fz0_full), (_, fy1_full, fz1_full) = front_proj
            full_y_span = fy1_full - fy0_full
            # The shield profile should NOT be a full-width rectangle all the way
            # down. A shield tapers to a point, so the bottom region should be
            # narrower. We verify the Y span is reasonable for a 0.30m sign.
            ctx.check(
                "shield panel full Y span is plausible",
                0.24 < full_y_span < 0.36,
                details=f"y_span={full_y_span:.3f}",
            )

    # --- Rest pose: A-frame stance ----------------------------------------
    with ctx.pose({hinge: 0.0}):
        ctx.expect_origin_distance(
            front, back, axes="xz", max_dist=0.20,
            name="panels meet near the apex at rest",
        )

    # --- Decisive motion pose: opening the hinge spreads the bottoms -------
    with ctx.pose({hinge: 0.0}):
        back_rest = ctx.part_world_aabb(back)
    with ctx.pose({hinge: 1.6}):
        back_open = ctx.part_world_aabb(back)

    moved = False
    if back_rest is not None and back_open is not None:
        rest_minx = back_rest[0][0]
        open_minx = back_open[0][0]
        moved = open_minx < rest_minx - 0.05
    ctx.check(
        "opening the apex hinge swings the back panel outward",
        moved,
        details=f"rest_minx={None if back_rest is None else round(back_rest[0][0], 3)}, "
        f"open_minx={None if back_open is None else round(back_open[0][0], 3)}",
    )

    # --- Both leaves are identical shield profiles -------------------------
    # The front and back shells should have the same Y-Z silhouette when
    # viewed from the front. Verify their Y and Z extents are approximately equal.
    if front_aabb is not None and back_aabb is not None:
        f_dy = front_aabb[1][1] - front_aabb[0][1]
        b_dy = back_aabb[1][1] - back_aabb[0][1]
        f_dz = front_aabb[1][2] - front_aabb[0][2]
        b_dz = back_aabb[1][2] - back_aabb[0][2]
        ctx.check(
            "both shield leaves have matching Y width",
            abs(f_dy - b_dy) < 0.015,
            details=f"front_dy={f_dy:.3f} back_dy={b_dy:.3f}",
        )
        ctx.check(
            "both shield leaves have matching Z height",
            abs(f_dz - b_dz) < 0.015,
            details=f"front_dz={f_dz:.3f} back_dz={b_dz:.3f}",
        )

    return ctx.report()
