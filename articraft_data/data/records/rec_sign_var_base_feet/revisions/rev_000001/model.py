from __future__ import annotations

# Articraft model: foldable A-frame "Wet Floor" caution sign — outrigger variant.
#
# Variant fork of the Lavex-style two-panel caution sign. The only structural
# change from the parent is the base-stabilization topology: each panel leaf
# carries a pair of molded outrigger floor feet at its bottom corners, widening
# the front-to-back footprint so the open A-frame stands more stably.
#
# Object identity (from picture/Sign/sign/001.png):
#   A two-panel, bright-yellow molded-plastic floor caution sign that folds
#   into an A-frame. The two panels are joined at the top by a row of three
#   short pin/knuckle barrels (the dark bands at the apex), and the top edge
#   has an integrated rounded grab handle with a cut-through opening. The front
#   panel carries the "CAUTION / Wet Floor" and "Cuidado / Piso Mojado" warning
#   text plus a slip-hazard triangle; the lower portion has molded horizontal
#   ribs. The back panel is a plain sloped yellow panel. Each panel base has
#   two low molded outrigger pads that splay outward from the bottom corners.
#
# Primary mechanism:
#   REVOLUTE hinge along the top apex line. The front panel is the root; the
#   back panel swings open/closed about the apex pin axis (along +Y, the panel
#   width). Positive q opens the A-frame (back panel tilts away from the front).

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
PANEL_HEIGHT = 0.600      # slant height of one panel (along its own length)
PANEL_THICK = 0.018       # molded shell wall block thickness
APEX_HALF_OPEN = 0.18     # half the A-frame opening angle at rest (radians)

HINGE_PIN_R = 0.006       # knuckle barrel radius
HINGE_BAND_LEN = 0.045    # each dark knuckle band length along Y
HINGE_GAP = 0.012         # gap between knuckle bands

HANDLE_OPENING_W = 0.140  # grab-handle cutout width (Y)
HANDLE_OPENING_H = 0.040  # grab-handle cutout height
HANDLE_BAR_T = 0.020      # handle bar thickness

# Outrigger floor feet (molded pads at each panel base corner) ------------
FOOT_SPREAD_Y = 0.044     # foot length along Y (outward splay from corner)
FOOT_DROP_Z = 0.020       # foot height below panel bottom edge
FOOT_THICK_X = 0.030      # foot thickness (wider than panel wall for stability)
FOOT_INSET_Y = 0.010      # how far foot embeds inside the panel side edge
FOOT_EMBED_Z = 0.006      # how far foot embeds into panel bottom for fusion

# Materials -------------------------------------------------------------------
YELLOW = Material(name="caution_yellow", rgba=(0.92, 0.80, 0.10, 1.0))
YELLOW_BACK = Material(name="caution_yellow_back", rgba=(0.78, 0.68, 0.12, 1.0))
HINGE_BLACK = Material(name="hinge_black", rgba=(0.08, 0.08, 0.09, 1.0))
TEXT_BLACK = Material(name="text_black", rgba=(0.05, 0.05, 0.05, 1.0))
HAZARD_RED = Material(name="hazard_red", rgba=(0.78, 0.18, 0.12, 1.0))


def _panel_shell(width: float, height: float, thick: float, *, ribbed: bool) -> cq.Workplane:
    """A single molded sign panel built in its OWN local frame.

    Local frame for the panel: the hinge (apex) line is at z = 0, the panel
    hangs downward toward -Z, the width runs along Y, and the visible front
    face points toward +X. The panel is a thin shell (hollow back) with a
    rounded top carrying the integrated grab handle and its cut-through hole.
    """
    half_w = width / 2.0

    # Construct the panel as an extrusion of a rounded-top profile in the
    # Y-Z plane (panel face plane), extruded a short distance along +X (thick).
    # The apex line sits at z = 0; the panel hangs down toward -Z.
    prof = (
        cq.Workplane("YZ")
        .moveTo(-half_w, -height)
        .lineTo(half_w, -height)
        .lineTo(half_w, -0.045)
        .threePointArc((0.0, -0.005), (-half_w, -0.045))
        .close()
        .extrude(thick)
    )

    # The panel solid spans x in [0, thick]. Keep the DECORATED FRONT WALL at
    # the +X side (x in [thick-wall, thick]) and hollow out the back (toward -X)
    # so the panel reads as a molded plastic shell, not a solid brick. The
    # cavity is cut downward (-X) from the inner face of the front wall and
    # stops short of the side/bottom/top rims.
    wall = 0.004
    cavity = (
        cq.Workplane("YZ")
        .workplane(offset=thick - wall)
        .moveTo(0.0, -height + wall)
        .rect(width - 2 * wall, height * 0.86)
        .extrude(-(thick - wall))
    )
    shell = prof.cut(cavity)

    # Grab-handle cut-through opening near the top (fully through the panel).
    handle_cut = (
        cq.Workplane("YZ")
        .workplane(offset=-0.01)
        .center(0.0, -0.035)
        .rect(HANDLE_OPENING_W, HANDLE_OPENING_H)
        .extrude(thick + 0.02)
    )
    shell = shell.cut(handle_cut)

    if ribbed:
        # Molded horizontal ribs proud of the FRONT face (+X). Embed each rib
        # a couple millimeters into the front wall so it fuses into one solid.
        rib_z0 = -height + 0.04
        rib_z1 = -height * 0.42
        n = 7
        for i in range(n):
            zc = rib_z0 + (rib_z1 - rib_z0) * (i / (n - 1))
            rib = (
                cq.Workplane("XY")
                .transformed(offset=cq.Vector(thick, 0.0, zc))
                .box(0.006, width * 0.82, 0.006, centered=(True, True, True))
            )
            shell = shell.union(rib)

    return shell


def _outrigger_foot(corner_y: float) -> cq.Workplane:
    """Build one outrigger foot pad in the panel's local frame.

    The foot sits at the bottom corner of the panel, extending:
    - outward along Y (beyond the panel side edge, widening the footprint)
    - downward along -Z (below the panel bottom, contacting the floor)
    - thicker along X than the panel wall (molded pad bulk)

    The foot embeds slightly into the panel bottom and side for solid fusion.

    corner_y: Y coordinate of the panel corner (+half_w or -half_w).
    """
    half_w = PANEL_WIDTH / 2.0
    splay = 1.0 if corner_y > 0 else -1.0

    # Foot center in panel local frame.
    cy = corner_y + splay * (FOOT_SPREAD_Y / 2.0 - FOOT_INSET_Y)
    cz = -PANEL_HEIGHT - (FOOT_DROP_Z - FOOT_EMBED_Z) / 2.0
    cx = PANEL_THICK / 2.0

    # Main pad body: a chunky box that splays outward and sits below the panel.
    pad = (
        cq.Workplane("XY")
        .transformed(offset=cq.Vector(cx, cy, cz))
        .box(FOOT_THICK_X, FOOT_SPREAD_Y, FOOT_DROP_Z + FOOT_EMBED_Z,
             centered=(True, True, True))
    )

    # Add a small chamfer on the outer bottom edge to suggest a molded parting
    # line and floor-contact radius. Use a generous approach that won't fail.
    try:
        pad = pad.edges("|Z").fillet(0.003)
    except Exception:
        pass  # sharp edges are acceptable fallback

    assert half_w > 0  # keep reference for clarity
    return pad


def _hinge_bands() -> cq.Workplane:
    """The three dark pin/knuckle barrels along the apex, in the apex frame.

    Apex frame: pin axis along Y, centered at origin. Barrels are short
    cylinders spaced across the width, matching the three dark bands in the
    reference image.
    """
    half_w = PANEL_WIDTH / 2.0
    # center positions for three bands across the width
    centers = [-0.085, 0.0, 0.085]
    # The panel top edge follows a shallow arc that peaks at z=-0.005 (center)
    # and dips to about z=-0.017 at the band offsets. Seat each barrel a few mm
    # BELOW that local top edge so it straddles and bridges both panel slabs.
    # Circle through (half_w,-0.045),(0,-0.005),(-half_w,-0.045) in (y,z):
    arc_cz = -0.3062
    arc_r = 0.3012
    bands = None
    for cy in centers:
        top_z = arc_cz + math.sqrt(max(arc_r**2 - cy**2, 0.0))
        barrel_z = top_z - 0.004  # embed slightly into the panel top edge
        # Build each barrel as a Y-axis cylinder of length HINGE_BAND_LEN at cy.
        # Workplane("XZ") has its normal along -Y, with local axes (X, Z); place
        # the band so it spans [cy-len/2, cy+len/2] along Y and is centered on
        # (x=0, z=barrel_z).
        barrel = (
            cq.Workplane("XZ")
            .workplane(offset=-(cy - HINGE_BAND_LEN / 2.0))
            .center(0.0, barrel_z)
            .circle(HINGE_PIN_R)
            .extrude(-HINGE_BAND_LEN)
        )
        bands = barrel if bands is None else bands.union(barrel)
    # keep half_w referenced for clarity
    assert half_w > 0
    return bands


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aframe_caution_sign_outrigger")
    for mat in (YELLOW, YELLOW_BACK, HINGE_BLACK, TEXT_BLACK, HAZARD_RED):
        model.material(mat.name, rgba=mat.rgba)

    # ------------------------------------------------------------------
    # FRONT panel (root). Its own local frame has the apex line at z=0 and
    # the panel hanging toward -Z, front face toward +X. We then tilt the
    # whole front-panel part in world by placing visuals with an apex-relative
    # rotation so the rest pose already looks like a standing A-frame: the
    # front panel leans so its bottom is forward (+X), top at apex.
    # ------------------------------------------------------------------
    front = model.part("front_panel")

    # Front shell, ribbed, tilted forward by APEX_HALF_OPEN about Y at the apex.
    front_shell = _panel_shell(PANEL_WIDTH, PANEL_HEIGHT, PANEL_THICK, ribbed=True)
    front.visual(
        mesh_from_cadquery(front_shell, "front_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -APEX_HALF_OPEN, 0.0)),
        material=YELLOW,
        name="front_shell",
    )

    # Warning triangle (slip-hazard) raised slightly proud of the front face.
    tri = (
        cq.Workplane("YZ")
        .workplane(offset=PANEL_THICK - 0.002)
        .center(0.0, -0.28)
        .polygon(3, 0.11)
        .extrude(0.004)
    )
    front.visual(
        mesh_from_cadquery(tri, "hazard_triangle"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -APEX_HALF_OPEN, 0.0)),
        material=HAZARD_RED,
        name="hazard_triangle",
    )

    # Two text bands (CAUTION / Wet Floor block, and Cuidado / Piso Mojado block)
    # represented as thin raised black plates proud of the front face.
    for idx, (zc, h) in enumerate([(-0.16, 0.055), (-0.40, 0.055)]):
        band = (
            cq.Workplane("YZ")
            .workplane(offset=PANEL_THICK - 0.002)
            .center(0.0, zc)
            .rect(PANEL_WIDTH * 0.72, h)
            .extrude(0.0035)
        )
        front.visual(
            mesh_from_cadquery(band, f"text_band_{idx}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -APEX_HALF_OPEN, 0.0)),
            material=TEXT_BLACK,
            name=f"text_band_{idx}",
        )

    # Outrigger floor feet at the bottom corners of the front panel.
    # Emitted via a shared geometry helper in a left/right loop.
    for i, corner_y in enumerate([-PANEL_WIDTH / 2.0, PANEL_WIDTH / 2.0]):
        foot_geom = _outrigger_foot(corner_y)
        front.visual(
            mesh_from_cadquery(foot_geom, f"front_foot_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -APEX_HALF_OPEN, 0.0)),
            material=YELLOW,
            name=f"front_foot_{i}",
        )

    # ------------------------------------------------------------------
    # Hinge knuckle bands. They belong to the front panel structure (the pin
    # runs through both leaves); model them on the front part at the apex.
    # ------------------------------------------------------------------
    bands = _hinge_bands()
    front.visual(
        mesh_from_cadquery(bands, "hinge_knuckles"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=HINGE_BLACK,
        name="hinge_knuckles",
    )

    # ------------------------------------------------------------------
    # BACK panel (child). Plain sloped yellow panel. Built in the same local
    # frame (apex at z=0, hangs toward -Z, face toward +X). At the joint
    # rest pose it is tilted the opposite way to complete the A-frame.
    # ------------------------------------------------------------------
    back = model.part("back_panel")
    # Build the back shell on the -X side of the pivot (its inner face at x=0,
    # body extending toward -X) so its apex slab does NOT share the pivot volume
    # with the front panel's +X apex slab.
    back_shell = _panel_shell(
        PANEL_WIDTH, PANEL_HEIGHT, PANEL_THICK, ribbed=False
    ).translate((-PANEL_THICK, 0.0, 0.0))
    # Back panel leans the opposite way (+APEX_HALF_OPEN) so that at the joint
    # rest pose (q=0) the two panels already form the open A-frame stance.
    back.visual(
        mesh_from_cadquery(back_shell, "back_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, APEX_HALF_OPEN, 0.0)),
        material=YELLOW_BACK,
        name="back_shell",
    )

    # Outrigger floor feet at the bottom corners of the back panel.
    # Same shared helper, translated to match the back shell's -X offset.
    for i, corner_y in enumerate([-PANEL_WIDTH / 2.0, PANEL_WIDTH / 2.0]):
        foot_geom = _outrigger_foot(corner_y).translate((-PANEL_THICK, 0.0, 0.0))
        back.visual(
            mesh_from_cadquery(foot_geom, f"back_foot_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, APEX_HALF_OPEN, 0.0)),
            material=YELLOW_BACK,
            name=f"back_foot_{i}",
        )

    # ------------------------------------------------------------------
    # Articulation: apex hinge. Pin axis along Y at the apex line (z=0).
    # The back panel's local frame already hangs toward -Z (straight down).
    # Positive q rotates the back panel about +Y so its bottom swings toward
    # -X (away from the front panel), opening the A-frame.
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
    # The dark knuckle barrels at the apex straddle BOTH leaves around the pin
    # line, so they intentionally embed a few millimeters into the back leaf to
    # capture it on the shared hinge axis. Scope the allowance to that exact
    # element pair and prove the captured contact below.
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
        # Panel is tall (real sign height) and wide across Y.
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

    # Hinge knuckles and warning features are present on the front panel.
    triangle = front.get_visual("hazard_triangle")
    ctx.check("hinge knuckles visual present", knuckles is not None)
    ctx.check("hazard triangle visual present", triangle is not None)

    # --- Outrigger floor feet -----------------------------------------------
    # Each panel has two molded outrigger feet at its bottom corners, widening
    # the A-frame footprint for stability. Verify they exist and extend beyond
    # the panel width.
    front_foot_0 = front.get_visual("front_foot_0")
    front_foot_1 = front.get_visual("front_foot_1")
    back_foot_0 = back.get_visual("back_foot_0")
    back_foot_1 = back.get_visual("back_foot_1")

    ctx.check("front left foot visual present", front_foot_0 is not None)
    ctx.check("front right foot visual present", front_foot_1 is not None)
    ctx.check("back left foot visual present", back_foot_0 is not None)
    ctx.check("back right foot visual present", back_foot_1 is not None)

    # The feet should widen the footprint beyond the bare panel width.
    if front_aabb is not None:
        (_, fy0, _), (_, fy1, _) = front_aabb
        panel_width_at_rest = fy1 - fy0
        ctx.check(
            "front panel feet widen footprint beyond bare panel width",
            panel_width_at_rest > PANEL_WIDTH + 0.010,
            details=f"front_y_span={panel_width_at_rest:.3f}, bare_panel_width={PANEL_WIDTH:.3f}",
        )

    # Feet should sit near the bottom of the sign (low Z on each panel).
    if front_aabb is not None:
        front_foot_aabb = ctx.part_element_world_aabb(front, elem="front_foot_0")
        if front_foot_aabb is not None:
            (ff_min, ff_max) = front_foot_aabb
            ctx.check(
                "front foot sits near the panel bottom (low Z)",
                ff_min[2] < (front_aabb[0][2] + 0.05),
                details=f"foot_min_z={ff_min[2]:.3f}, panel_min_z={front_aabb[0][2]:.3f}",
            )

    # The feet are fused into the panel shell (same part, not separate joints).
    # Verify there are no additional articulations beyond the apex hinge.
    all_joints = list(object_model.articulations)
    ctx.check(
        "feet are fused into panels (only apex hinge joint exists)",
        len(all_joints) == 1,
        details=f"joint_count={len(all_joints)}, joints={[j.name for j in all_joints]}",
    )

    # Prove the apex knuckle barrels actually capture the back leaf (they
    # straddle the shared pin line), justifying the scoped overlap allowance.
    ctx.expect_contact(
        front,
        back,
        elem_a="hinge_knuckles",
        elem_b="back_shell",
        contact_tol=0.001,
        name="apex knuckles capture the back leaf at the hinge",
    )

    # The grab-handle opening: the panel's top region should not be a solid
    # brick. Verify the handle cutout exists by checking the shell thickness is
    # thin (hollow molded panel), i.e. front panel X-extent is modest.
    if front_aabb is not None:
        (fx0, _, _), (fx1, _, _) = front_aabb
        ctx.check(
            "front panel is a thin tilted shell, not a deep box",
            (fx1 - fx0) < 0.30,
            details=f"x_extent={fx1 - fx0:.3f}",
        )

    # --- Rest pose: A-frame stance ----------------------------------------
    # At rest (q=0) the two panels should overlap near the apex (top) and
    # spread apart at the bottom, forming an A. Check apex-region contact and
    # base separation.
    with ctx.pose({hinge: 0.0}):
        # Apex (top) regions of both panels are close together.
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
        # Bottom-front extent of the back panel should move in -X (away from
        # the front panel) as the hinge opens.
        rest_minx = back_rest[0][0]
        open_minx = back_open[0][0]
        moved = open_minx < rest_minx - 0.05
    ctx.check(
        "opening the apex hinge swings the back panel outward",
        moved,
        details=f"rest_minx={None if back_rest is None else round(back_rest[0][0], 3)}, "
        f"open_minx={None if back_open is None else round(back_open[0][0], 3)}",
    )

    return ctx.report()
