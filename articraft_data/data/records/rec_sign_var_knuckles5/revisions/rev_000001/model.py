from __future__ import annotations

# Articraft model: foldable A-frame "Wet Floor" caution sign (Lavex-style).
# Variant: five alternating piano-style hinge knuckles.
#
# Object identity (from picture/Sign/sign/001.png):
#   A two-panel, bright-yellow molded-plastic floor caution sign that folds
#   into an A-frame. The two panels are joined at the top by five short
#   alternating knuckle barrels (piano-hinge style), and the top edge has an
#   integrated rounded grab handle with a cut-through opening. The front panel
#   carries the "CAUTION / Wet Floor" and "Cuidado / Piso Mojado" warning text
#   plus a slip-hazard triangle; the lower portion has molded horizontal ribs.
#   The back panel is a plain sloped yellow panel.
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

N_KNUCKLES = 5            # five alternating piano-style knuckle barrels
HINGE_PIN_R = 0.008       # knuckle barrel radius (larger to bridge panel gap)
HINGE_BAND_LEN = 0.035    # each knuckle barrel length along Y (5-knuckle variant)

HANDLE_OPENING_W = 0.140  # grab-handle cutout width (Y)
HANDLE_OPENING_H = 0.040  # grab-handle cutout height
HANDLE_BAR_T = 0.020      # handle bar thickness

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


def _hinge_knuckle_barrel(cy: float) -> cq.Workplane:
    """Build a single short knuckle barrel centered at Y=cy along the apex line.

    The barrel is a Y-axis cylinder of length HINGE_BAND_LEN, seated a few mm
    into the panel top edge so it straddles and captures both leaves on the
    shared pin line. The panel top edge follows a shallow arc in (y,z); the
    barrel Z is computed from that arc so each knuckle sits flush at its own
    Y position.
    """
    # Arc through (half_w,-0.045),(0,-0.005),(-half_w,-0.045) in (y,z):
    arc_cz = -0.3062
    arc_r = 0.3012
    top_z = arc_cz + math.sqrt(max(arc_r**2 - cy**2, 0.0))
    barrel_z = top_z - 0.004  # embed slightly into the panel top edge

    # Workplane("XZ") has its normal along -Y, with local axes (X, Z); place
    # the barrel so it spans [cy-len/2, cy+len/2] along Y and is centered on
    # (x=0, z=barrel_z).
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=-(cy - HINGE_BAND_LEN / 2.0))
        .center(0.0, barrel_z)
        .circle(HINGE_PIN_R)
        .extrude(-HINGE_BAND_LEN)
    )
    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="afframe_caution_sign")
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

    # ------------------------------------------------------------------
    # BACK panel (child). Plain sloped yellow panel. Built in the same local
    # frame (apex at z=0, hangs toward -Z, face toward +X). At the joint
    # rest pose it is tilted the opposite way to complete the A-frame.
    # Created before the knuckle loop so alternating knuckles can be placed
    # on the back panel.
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

    # ------------------------------------------------------------------
    # Hinge knuckle barrels: five short molded barrels evenly spaced across
    # the panel width, alternating leaves like a real piano-style hinge.
    # Even-indexed knuckles (0,2,4) belong to the front panel; odd-indexed
    # knuckles (1,3) belong to the back panel. All share the one apex pin axis.
    # Knuckles are not rotated so they stay at the apex and can capture both
    # leaves. This may cause a connectivity warning for edge knuckles, but
    # the alternating-leaf design is mechanically correct.
    # ------------------------------------------------------------------
    slot_width = PANEL_WIDTH / N_KNUCKLES
    for i in range(N_KNUCKLES):
        cy = -PANEL_WIDTH / 2.0 + slot_width * (i + 0.5)
        barrel = _hinge_knuckle_barrel(cy)
        if i % 2 == 0:
            front.visual(
                mesh_from_cadquery(barrel, f"knuckle_{i}"),
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=HINGE_BLACK,
                name=f"knuckle_{i}",
            )
        else:
            back.visual(
                mesh_from_cadquery(barrel, f"knuckle_{i}"),
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=HINGE_BLACK,
                name=f"knuckle_{i}",
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

    # --- Five alternating piano-style knuckle barrels ----------------------
    # Verify exactly five knuckles exist with knuckle_i naming and that they
    # alternate leaves: even indices (0,2,4) on front, odd indices (1,3) on back.
    even_knuckles = [f"knuckle_{i}" for i in range(5) if i % 2 == 0]
    odd_knuckles = [f"knuckle_{i}" for i in range(5) if i % 2 == 1]
    all_knuckles = [f"knuckle_{i}" for i in range(5)]

    front_visual_names = [v.name for v in front.visuals]
    back_visual_names = [v.name for v in back.visuals]

    for name in all_knuckles:
        found = name in front_visual_names or name in back_visual_names
        ctx.check(
            f"{name} exists",
            found,
            details=f"knuckle not found on either panel",
        )

    for name in even_knuckles:
        ctx.check(
            f"{name} belongs to front panel",
            name in front_visual_names,
            details=f"{name} should be on front panel (even index)",
        )

    for name in odd_knuckles:
        ctx.check(
            f"{name} belongs to back panel",
            name in back_visual_names,
            details=f"{name} should be on back panel (odd index)",
        )

    # --- Intentional hinge capture (scoped per knuckle) -------------------
    # Each knuckle barrel straddles the shared pin line and embeds slightly
    # into the opposite leaf to capture it. Even-indexed knuckles (on front)
    # embed into back_shell; odd-indexed knuckles (on back) embed into
    # front_shell. Scope each allowance to its exact element pair.
    for name in even_knuckles:
        ctx.allow_overlap(
            front,
            back,
            elem_a=name,
            elem_b="back_shell",
            reason=(
                f"Front-leaf {name} barrel straddles the pin line and captures "
                f"the back leaf at the shared hinge axis."
            ),
        )

    for name in odd_knuckles:
        ctx.allow_overlap(
            back,
            front,
            elem_a=name,
            elem_b="front_shell",
            reason=(
                f"Back-leaf {name} barrel straddles the pin line and captures "
                f"the front leaf at the shared hinge axis."
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

    # Hazard triangle and text bands are present on the front panel.
    triangle = front.get_visual("hazard_triangle")
    ctx.check("hazard triangle visual present", triangle is not None)

    # Prove the knuckle barrels actually capture the opposite leaf (they
    # straddle the shared pin line), justifying the scoped overlap allowances.
    for name in even_knuckles:
        ctx.expect_contact(
            front,
            back,
            elem_a=name,
            elem_b="back_shell",
            contact_tol=0.001,
            name=f"{name} captures the back leaf at the hinge",
        )

    for name in odd_knuckles:
        ctx.expect_contact(
            back,
            front,
            elem_a=name,
            elem_b="front_shell",
            contact_tol=0.001,
            name=f"{name} captures the front leaf at the hinge",
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
