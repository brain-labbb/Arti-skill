from __future__ import annotations

# Orange Tide liquid laundry detergent bottle — TALL CYLINDRICAL (round) variant.
# Frame: bottle axis along +Z (vertical). Bottle bottom sits at z=0, total
# height ~0.26 m, ~0.11 m diameter (round cross-section). The integrated side
# loop handle is on the +X side. The "Tide" bullseye label and the "he" badge
# sit on the -Y face of the cylinder.
#
# This is a body-footprint fork of the flat-oval Tide jug parent: only the body
# cross-section family changes from rounded-rectangle (flat-oval) to circular
# (round cylinder). Neck, handle, spout, cap, and articulations are preserved.
#
# Construction:
#   - body (root): glossy orange HDPE round bottle (contoured shoulder via a
#     vertical circle-section loft), an integrated side loop handle (slab with
#     an oval cut-out), a short threaded neck, and a FIXED pour spout.
#     Bullseye label + he badge on the -Y face.
#   - cap_carrier: massless decoupled carrier (no visuals).
#   - cap: translucent blue measuring-cup screw cap (tapered hollow cup) + an
#     off-axis marker tab so rotation is observable.
# Articulations (both share the neck-top +Z axis, decoupled through carrier):
#   - cap_rotate: CONTINUOUS spin of the carrier about +Z at the neck top.
#   - cap_slide:  PRISMATIC lift of the cap up off the neck.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key heights / radii (meters) ----
BODY_BOTTOM_Z = 0.0
SHOULDER_Z = 0.175  # where the body starts necking in
NECK_BOTTOM_Z = 0.198
NECK_TOP_Z = 0.214  # threaded neck top
CAP_SEAT_Z = 0.193  # where the cap skirt seats over the neck; cap-rotate axis origin
CAP_HEIGHT = 0.058
NECK_R = 0.0235
SPOUT_TOP_Z = 0.210


def _body_shell() -> cq.Workplane:
    # Round cylindrical jug body: a vertical loft of circular cross-sections
    # giving a tall round bottle with a contoured shoulder that necks in toward
    # the spout. Cross-sections are (z, radius).
    BODY_R = 0.055  # main body radius
    sections = [
        (0.004, 0.050),  # bottom with slight inset
        (0.020, BODY_R),  # main body radius
        (0.090, BODY_R),
        (0.150, 0.053),  # slight taper before shoulder
        (0.175, 0.044),  # shoulder begins
        (0.190, 0.030),  # shoulder necks in
        (0.198, 0.025),  # base of neck (blends into NECK_R)
    ]
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, r) in enumerate(sections):
        off = z if i == 0 else z - prev_z
        wp = wp.workplane(offset=off) if i == 0 else wp.workplane(offset=off)
        wp = wp.circle(r)
        prev_z = z
    body = wp.loft(ruled=False)

    # Short threaded neck on top.
    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM_Z)
        .circle(NECK_R)
        .extrude(NECK_TOP_Z - NECK_BOTTOM_Z + 0.004)
    )
    body = body.union(neck)

    # Open pour mouth: bore a through-channel down the neck axis into the body
    # so the neck/spout reads as a real opening (贯通), not a solid plug.
    mouth = (
        cq.Workplane("XY")
        .workplane(offset=0.120)
        .circle(0.013)
        .extrude(0.115)  # up through the neck top (~z=0.235)
    )
    body = body.cut(mouth)
    return body


def _handle_solid() -> cq.Workplane:
    # Integrated side loop handle on the +X side: a contoured slab fused into
    # the upper cylindrical body, with a fully-enclosed finger gap so a solid
    # outer grip bar plus top/bottom connectors remain (a real closed loop you
    # can hook fingers through, not a window that breaks the outer wall).
    # Handle slab inner edge embeds into the round body (radius 0.055) for
    # structural fusion.
    slab = (
        cq.Workplane("XZ")
        .transformed(offset=(0.060, 0.110, 0.0))
        .rect(0.052, 0.150)  # x-extent (toward +X), z-extent (height)
        .extrude(0.022, both=True)  # thickness along Y
    )
    try:
        slab = slab.edges("|Y").fillet(0.012)
    except Exception:
        pass
    # Oval finger cut-out kept inside the slab footprint so the outer grip bar
    # stays solid. Cut-out center slightly outward so the outer bar remains.
    hole = (
        cq.Workplane("XZ")
        .transformed(offset=(0.062, 0.110, 0.0))
        .ellipse(0.016, 0.048)
        .extrude(0.06, both=True)
    )
    slab = slab.cut(hole)
    return slab


def _spout_mesh():
    # Raised pour spout continuing the bored neck mouth: a short hollow lip
    # rising just above the neck rim, open at the top and aligned with the body
    # bore so the opening passes straight through (贯通).
    o = cq.Workplane("XY").workplane(offset=0.206).circle(0.018).extrude(0.026)
    i = cq.Workplane("XY").workplane(offset=0.204).circle(0.013).extrude(0.032)
    spout = o.cut(i)
    return mesh_from_cadquery(spout, "pour_spout")


def _cap_mesh():
    # Translucent blue measuring-cup cap: a slightly tapered hollow cup, open at
    # the bottom so it screws down over the neck/spout. Built bottom-up; the cap
    # part frame has its open rim near z=0 and the closed dome at +Z so it sits
    # on top of the neck.
    r_bottom = 0.0285
    r_top = 0.0250
    wall = 0.0028
    h = CAP_HEIGHT
    outer = (
        cq.Workplane("XY")
        .circle(r_bottom)
        .workplane(offset=h)
        .circle(r_top)
        .loft(ruled=True)
    )
    # Hollow it out from the bottom, leaving a closed top.
    inner = (
        cq.Workplane("XY")
        .circle(r_bottom - wall)
        .workplane(offset=h - 0.010)
        .circle(r_top - wall)
        .loft(ruled=True)
    )
    cap = outer.cut(inner)
    return mesh_from_cadquery(cap, "cap_cup")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tide_detergent_bottle_round")

    orange = model.material("tide_orange", rgba=(0.95, 0.46, 0.07, 1.0))
    blue_cap = model.material("cap_blue", rgba=(0.30, 0.55, 0.85, 0.5))  # translucent
    label_yellow = model.material("label_yellow", rgba=(0.98, 0.83, 0.10, 1.0))
    label_blue = model.material("label_blue", rgba=(0.10, 0.22, 0.62, 1.0))
    he_silver = model.material("he_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    marker_white = model.material("marker_white", rgba=(0.92, 0.92, 0.95, 1.0))

    # ---------------- body (root) ----------------
    body = model.part("body")

    shell = _body_shell().union(_handle_solid())
    body.visual(mesh_from_cadquery(shell, "jug_shell"), material=orange, name="jug_shell")

    # Fixed pour spout under the cap.
    body.visual(_spout_mesh(), material=orange, name="pour_spout")

    # "Tide" bullseye label on the front (-Y) cylindrical face: yellow disc +
    # blue band underneath it. Labels sit just proud of the round body surface
    # (body radius = 0.055).
    BODY_R = 0.055
    body.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.0, -(BODY_R - 0.002), 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=label_yellow,
        name="bullseye_label",
    )
    body.visual(
        Box((0.060, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, -(BODY_R - 0.002), 0.060)),
        material=label_blue,
        name="label_band",
    )
    # "he" badge near the shoulder.
    body.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=(0.0, -(BODY_R - 0.002), 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=he_silver,
        name="he_badge",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.21), mass=1.1, origin=Origin(xyz=(0.0, 0.0, 0.105))
    )

    # ---------------- cap_carrier (massless, decoupled) ----------------
    carrier = model.part("cap_carrier")  # NO visuals
    carrier.inertial = Inertial.from_geometry(Box((0.01, 0.01, 0.01)), mass=1e-4)

    # ---------------- cap (translucent blue measuring cup) ----------------
    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=blue_cap, name="cap_cup")
    # Off-axis marker tab so rotation about +Z is observable.
    cap.visual(
        Box((0.006, 0.006, 0.010)),
        origin=Origin(xyz=(0.026, 0.0, 0.030)),
        material=marker_white,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0285, length=CAP_HEIGHT), mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
    )

    # ---- cap_rotate: continuous spin of carrier about +Z at the neck top ----
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, CAP_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    # ---- cap_slide: prismatic lift of cap up off the neck ----
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_HEIGHT, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("cap_carrier")
    cap = object_model.get_part("cap")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    # Cap is the blue cup seated on top of the neck; allow the small intentional
    # embed where the cup skirt screws down over the neck/spout.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_cup",
        elem_b="jug_shell",
        reason="The measuring-cup cap screws down over the threaded neck (skirt overlaps neck).",
    )
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_cup",
        elem_b="pour_spout",
        reason="The fixed pour spout sits inside the inverted measuring cup when capped.",
    )

    # Cap sits at the top of the jug (its dome is the highest geometry).
    cap_aabb = ctx.part_world_aabb(cap)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "cap is on top of the jug",
        cap_aabb is not None and body_aabb is not None
        and cap_aabb[1][2] >= body_aabb[1][2] - 1e-6,
        details=f"cap z-max={cap_aabb[1][2]}, body z-max={body_aabb[1][2]}",
    )

    # Cap reads as the blue translucent cup (alpha < 1).
    mat = cap.get_visual("cap_cup").material
    alpha = mat.rgba[3] if mat is not None and mat.rgba is not None else 1.0
    ctx.check(
        "cap is translucent blue (alpha < 1)",
        alpha < 1.0 and mat.rgba[2] > mat.rgba[0],
        details=f"cap rgba={None if mat is None else mat.rgba}",
    )

    # Cap spins about +Z: the off-axis marker swings around the axis (its world
    # XY position changes under a quarter turn of cap_rotate).
    marker0 = ctx.part_element_world_aabb(cap, elem="cap_marker")
    mc0 = ((marker0[0][0] + marker0[1][0]) / 2.0, (marker0[0][1] + marker0[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        marker90 = ctx.part_element_world_aabb(cap, elem="cap_marker")
        mc90 = ((marker90[0][0] + marker90[1][0]) / 2.0, (marker90[0][1] + marker90[1][1]) / 2.0)
    swing = math.hypot(mc90[0] - mc0[0], mc90[1] - mc0[1])
    ctx.check(
        "cap rotation swings the off-axis marker about +Z",
        swing > 0.02,
        details=f"marker rest={mc0}, quarter-turn={mc90}, swing={swing}",
    )

    # Cap slides straight up off the neck along +Z.
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_HEIGHT}):
        lifted_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap slides up off the neck",
        lifted_z > rest_z + 0.04,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # Decoupled: rotating the cap does not lift it (slide stays put under rotate).
    with ctx.pose({rotate: math.pi}):
        rot_only_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "rotation is decoupled from lift",
        abs(rot_only_z - rest_z) < 1e-4,
        details=f"rest_z={rest_z}, rotated_z={rot_only_z}",
    )

    # Integrated side loop handle with a real through cut-out: probe the handle
    # region (+X side) and confirm the slab protrudes from the cylindrical body.
    handle_aabb = ctx.part_element_world_aabb(body, elem="jug_shell")
    ctx.check(
        "jug extends out to the side loop handle on +X",
        handle_aabb is not None and handle_aabb[1][0] > 0.075,
        details=f"jug_shell x-max={None if handle_aabb is None else handle_aabb[1][0]}",
    )

    # Round body variant: the cylindrical body cross-section should be roughly
    # circular, i.e. the X and Y extents of the main body (excluding handle)
    # should be approximately equal. We check that the Y extent is at least 80%
    # of the X extent at mid-body height (the flat-oval parent had Y < 70% of X).
    # Measure the body AABB excluding the handle protrusion on +X: the -X side
    # of the body is at about -BODY_R = -0.055, so total X extent of the body
    # proper is ~0.11. The Y extent should also be ~0.11 for a round body.
    if handle_aabb is not None:
        body_x_min = handle_aabb[0][0]  # -X side of the cylinder
        body_y_extent = handle_aabb[1][1] - handle_aabb[0][1]
        # The -X bound of the shell should be near -0.055 (body radius)
        body_x_extent_from_radius = 2.0 * abs(body_x_min) if body_x_min < -0.04 else 0.11
        ctx.check(
            "body has round (not flat-oval) cross-section: Y extent ≈ X diameter",
            body_y_extent > 0.80 * body_x_extent_from_radius,
            details=f"y_extent={body_y_extent:.4f}, x_diameter≈{body_x_extent_from_radius:.4f}",
        )

    # Bullseye label present on the front face.
    ctx.expect_contact(body, body, elem_a="bullseye_label", elem_b="jug_shell",
                       name="bullseye label is on the jug body")

    return ctx.report()


object_model = build_object_model()
