from __future__ import annotations

# Articraft model: foldable A-frame "Wet Floor" caution sign — GABLED variant.
#
# Variant change from parent: The panel outline is changed from a rounded-top
# taper to a peaked, gabled (house-shaped / pentagon) silhouette. Each leaf
# rises from a wide flat base, runs up two slightly inward-leaning sides, and
# converges to a centered shallow peak just below the hinge barrel. The
# integrated grab-handle cut-through opening is carried near the peak. Both
# leaves are identical in the new gabled profile so they meet cleanly along
# the apex hinge line.
#
# Everything else stays identical to the parent: same top apex revolute hinge,
# same width and floor-sign height, same carry handle, same molded shell
# hollowing and base feet, same overall yellow caution identity.
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

# Gabled profile parameters -------------------------------------------------
GABLE_PEAK_Z = -0.008     # z of the centered shallow peak (just below hinge)
GABLE_SHOULDER_Z = -0.065 # z where sides transition from near-vertical to gable slope
GABLE_INSET = 0.006       # inward lean of sides at shoulder relative to base edge

HINGE_PIN_R = 0.006       # knuckle barrel radius
HINGE_BAND_LEN = 0.045    # each dark knuckle band length along Y
HINGE_GAP = 0.012         # gap between knuckle bands

HANDLE_OPENING_W = 0.070  # grab-handle cutout width (Y) — sized to fit within gable
HANDLE_OPENING_H = 0.024  # grab-handle cutout height
HANDLE_BAR_T = 0.020      # handle bar thickness
HANDLE_CENTER_Z = -0.040  # handle cutout center z (near peak, in gable region)

# Materials -------------------------------------------------------------------
YELLOW = Material(name="caution_yellow", rgba=(0.92, 0.80, 0.10, 1.0))
YELLOW_BACK = Material(name="caution_yellow_back", rgba=(0.78, 0.68, 0.12, 1.0))
HINGE_BLACK = Material(name="hinge_black", rgba=(0.08, 0.08, 0.09, 1.0))
TEXT_BLACK = Material(name="text_black", rgba=(0.05, 0.05, 0.05, 1.0))
HAZARD_RED = Material(name="hazard_red", rgba=(0.78, 0.18, 0.12, 1.0))


def _gable_top_z(y: float) -> float:
    """Return the z-coordinate of the gabled panel top edge at a given y.

    The gable slope runs linearly from (±(half_w - inset), shoulder_z) to
    (0, peak_z). Outside the shoulder width, the top edge stays at shoulder_z.
    """
    half_w = PANEL_WIDTH / 2.0
    half_w_inset = half_w - GABLE_INSET
    abs_y = abs(y)
    if abs_y >= half_w_inset:
        return GABLE_SHOULDER_Z
    t = abs_y / half_w_inset  # 0 at center, 1 at shoulder
    return GABLE_PEAK_Z + t * (GABLE_SHOULDER_Z - GABLE_PEAK_Z)


def _panel_shell(width: float, height: float, thick: float, *, ribbed: bool) -> cq.Workplane:
    """A single molded sign panel with a GABLED (peaked / house-shaped) silhouette.

    Local frame: hinge (apex) line at z = 0, panel hangs toward -Z, width
    along Y, visible front face toward +X. The panel is a thin shell with a
    gabled profile — wide flat base, slightly inward-leaning sides, and a
    centered shallow peak just below the hinge barrel.
    """
    half_w = width / 2.0
    half_w_inset = half_w - GABLE_INSET

    # Gabled pentagon profile in the Y-Z plane, extruded along +X (thickness).
    # Five vertices: bottom-left, bottom-right, right shoulder, peak, left shoulder.
    prof = (
        cq.Workplane("YZ")
        .moveTo(-half_w, -height)                    # bottom-left
        .lineTo(half_w, -height)                      # bottom-right
        .lineTo(half_w_inset, GABLE_SHOULDER_Z)       # right shoulder
        .lineTo(0.0, GABLE_PEAK_Z)                    # centered shallow peak
        .lineTo(-half_w_inset, GABLE_SHOULDER_Z)      # left shoulder
        .close()
        .extrude(thick)
    )

    # Hollow out the back to create a molded plastic shell. The cavity is a
    # large rectangular subtraction from the inner face of the front wall,
    # leaving rims on all sides.
    wall = 0.004
    cavity = (
        cq.Workplane("YZ")
        .workplane(offset=thick - wall)
        .center(0.0, -height * 0.55)
        .rect(width - 2 * wall, height * 0.78)
        .extrude(-(thick - wall))
    )
    shell = prof.cut(cavity)

    # Grab-handle cut-through opening near the peak (in the gable region).
    # The rectangular cut intersects with the gabled outline, creating an
    # opening that narrows toward the peak — echoing the gabled silhouette.
    handle_cut = (
        cq.Workplane("YZ")
        .workplane(offset=-0.01)
        .center(0.0, HANDLE_CENTER_Z)
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


def _hinge_bands() -> cq.Workplane:
    """Three dark knuckle barrels along the apex, seated on the gabled top edge.

    Each barrel is centered partway into the panel thickness so it has real
    volume overlap with the shell mesh (not just tangent-line contact at the
    gable slope). The barrels protrude slightly from the back face (-X side)
    to remain visible as dark knuckle bands.
    """
    centers = [-0.085, 0.0, 0.085]
    # Place barrel center slightly into front panel so it has real volume overlap
    # with the shell mesh (not just tangent-line contact at the gable slope).
    # Barrel spans x in [-0.002, 0.010] to capture both panels.
    barrel_x = PANEL_THICK * 0.22
    bands = None
    for cy in centers:
        top_z = _gable_top_z(cy)
        barrel_z = top_z - HINGE_PIN_R  # embed one full radius below top edge
        barrel = (
            cq.Workplane("XZ")
            .workplane(offset=-(cy - HINGE_BAND_LEN / 2.0))
            .center(barrel_x, barrel_z)
            .circle(HINGE_PIN_R)
            .extrude(-HINGE_BAND_LEN)
        )
        bands = barrel if bands is None else bands.union(barrel)
    return bands


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gabled_caution_sign")
    for mat in (YELLOW, YELLOW_BACK, HINGE_BLACK, TEXT_BLACK, HAZARD_RED):
        model.material(mat.name, rgba=mat.rgba)

    # ------------------------------------------------------------------
    # FRONT panel (root). Its own local frame has the apex line at z=0 and
    # the panel hanging toward -Z, front face toward +X. We tilt the whole
    # front-panel part in world by placing visuals with an apex-relative
    # rotation so the rest pose already looks like a standing A-frame.
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
    # BACK panel (child). Plain gabled panel, same silhouette as the front.
    # Built in the same local frame (apex at z=0, hangs toward -Z, face toward
    # +X). At the joint rest pose it is tilted the opposite way to complete
    # the A-frame.
    # ------------------------------------------------------------------
    back = model.part("back_panel")
    # The back shell is NOT translated in X — it occupies the same x-range as
    # the front shell at the apex, creating the hinge contact. The overlap at
    # the apex is intentional and handled by allow_overlap below.
    back_shell = _panel_shell(
        PANEL_WIDTH, PANEL_HEIGHT, PANEL_THICK, ribbed=False
    )
    # Back panel leans the opposite way (+APEX_HALF_OPEN) so that at the joint
    # rest pose (q=0) the two panels already form the open A-frame stance.
    back.visual(
        mesh_from_cadquery(back_shell, "back_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, APEX_HALF_OPEN, 0.0)),
        material=YELLOW_BACK,
        name="back_shell",
    )

    # ------------------------------------------------------------------
    # Articulation: apex hinge. Pin axis along Y at the apex line (z=0).
    # Positive q rotates the back panel about +Y so its bottom swings toward
    # -X (away from the front panel), opening the A-frame wider.
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
    # The back shell apex region intentionally overlaps with the front shell
    # apex region to create hinge contact. Both shells occupy the same x-range
    # at the apex line (z=0), which is mechanically correct for a hinge joint.
    ctx.allow_overlap(
        front,
        back,
        elem_a="front_shell",
        elem_b="back_shell",
        reason=(
            "Back shell apex region overlaps with front shell apex region to "
            "create hinge contact at the shared pivot line."
        ),
    )
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

    # Prove the apex knuckle barrels actually capture the back leaf.
    ctx.expect_contact(
        front,
        back,
        elem_a="hinge_knuckles",
        elem_b="back_shell",
        contact_tol=0.001,
        name="apex knuckles capture the back leaf at the hinge",
    )

    # The panel is a thin tilted shell, not a deep box.
    if front_aabb is not None:
        (fx0, _, _), (fx1, _, _) = front_aabb
        ctx.check(
            "front panel is a thin tilted shell, not a deep box",
            (fx1 - fx0) < 0.30,
            details=f"x_extent={fx1 - fx0:.3f}",
        )

    # --- Gabled silhouette proof ------------------------------------------
    # Both panels use the same _panel_shell helper, so they share identical
    # gabled profiles. Verify the Y-widths match.
    front_shell_aabb = ctx.part_element_world_aabb(front, elem="front_shell")
    back_shell_aabb = ctx.part_element_world_aabb(back, elem="back_shell")

    if front_shell_aabb is not None and back_shell_aabb is not None:
        front_y = front_shell_aabb[1][1] - front_shell_aabb[0][1]
        back_y = back_shell_aabb[1][1] - back_shell_aabb[0][1]
        ctx.check(
            "both panels share the same gabled profile width",
            abs(front_y - back_y) < 0.010,
            details=f"front_y={front_y:.3f}, back_y={back_y:.3f}",
        )

    # The gabled panel narrows toward the peak: the shell AABB Y-width is
    # determined by the base (widest point). The base should span the full
    # sign width (≥ 0.28m) while the gabled top is narrower.
    if front_shell_aabb is not None:
        aabb_y = front_shell_aabb[1][1] - front_shell_aabb[0][1]
        ctx.check(
            "gabled panel base spans the full sign width",
            aabb_y > 0.28,
            details=f"aabb_y_width={aabb_y:.3f}",
        )

    # The gabled peak creates a pointed top: the panel Z-extent should reach
    # close to the apex (z ≈ 0 in the local frame, tilted in world). A flat-
    # topped panel would be shorter. The tilted gabled panel's world Z-extent
    # should be at least as tall as the slant height * cos(tilt) minus the
    # peak offset contribution.
    if front_aabb is not None:
        (_, _, fz0), (_, _, fz1) = front_aabb
        z_extent = fz1 - fz0
        # Tilted panel Z ≈ height * cos(0.18) + small gable contribution
        expected_min = PANEL_HEIGHT * math.cos(APEX_HALF_OPEN) * 0.95
        ctx.check(
            "gabled panel Z-extent reflects the full slant height with peak",
            z_extent > expected_min,
            details=f"z_extent={z_extent:.3f}, expected_min={expected_min:.3f}",
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

    return ctx.report()
