from __future__ import annotations

# Orange Tide liquid laundry detergent bottle — flip-top cap variant.
# Fork of the parent measuring-cup jug: the body (jug shell, integrated side
# loop handle, threaded neck, pour spout, labels) is identical; only the
# closure mechanism changes. The measuring-cup screw cap is replaced by a
# hinged flip-top snap lid attached to the neck collar by a living hinge that
# swings open and shut on a REVOLUTE joint about a horizontal axis.
#
# Frame: bottle axis along +Z (vertical). Bottom at z=0, height ~0.26 m,
# ~0.15 m wide. Broad faces face +/-Y; integrated side loop handle on +X.
# "Tide" bullseye label and "he" badge on the -Y broad face.
#
# Parts:
#   body (root): orange HDPE jug shell + handle + threaded neck + neck collar
#                ring + pour spout + labels.
#   lid:         flip-top snap cap with a living-hinge tab at its rear edge.
#
# Articulation:
#   body_to_lid: REVOLUTE about a horizontal axis (+X) at the rear of the
#                neck collar. Positive q opens the lid upward/backward.
#                Limits 0 (closed) to ~2.0 rad (fully open).

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
SHOULDER_Z = 0.175
NECK_BOTTOM_Z = 0.198
NECK_TOP_Z = 0.218       # top of threaded neck
COLLAR_TOP_Z = 0.222     # top of the raised collar ring
NECK_R = 0.0235
COLLAR_R = 0.027         # collar outer radius (slightly wider than neck)
SPOUT_TOP_Z = 0.216

# Hinge geometry
HINGE_Y = -COLLAR_R      # rear of collar
HINGE_Z = COLLAR_TOP_Z   # hinge sits at the collar top
HINGE_ORIGIN = (0.0, HINGE_Y, HINGE_Z)

# Lid dimensions (in lid-local frame: origin at hinge, lid extends +Y and +Z)
LID_RADIUS = 0.031       # broad cap top that fully covers the neck opening
LID_CENTER_Y = -HINGE_Y  # centers the closed cap on the bottle axis
LID_THICKNESS = 0.004    # cap wall thickness
LID_SKIRT_HEIGHT = 0.009 # skirt that snaps around, not through, the collar
LID_DISC_BOTTOM_Z = 0.0025
LID_SKIRT_INNER_R = COLLAR_R + 0.0025
LID_SKIRT_OUTER_R = LID_SKIRT_INNER_R + 0.0035


def _body_shell() -> cq.Workplane:
    """Jug body: vertical loft of rounded-rectangle cross-sections."""
    sections = [
        (0.004, 0.070, 0.044),
        (0.020, 0.075, 0.048),
        (0.090, 0.075, 0.048),
        (0.150, 0.073, 0.046),
        (0.175, 0.064, 0.040),
        (0.190, 0.040, 0.030),
        (0.198, 0.028, 0.028),
    ]
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, hx, hy) in enumerate(sections):
        off = z if i == 0 else z - prev_z
        wp = wp.workplane(offset=off) if i == 0 else wp.workplane(offset=off)
        wp = wp.rect(2.0 * hx, 2.0 * hy)
        prev_z = z
    body = wp.loft(ruled=False)
    try:
        body = body.edges("|Z").fillet(0.012)
    except Exception:
        pass

    # Short threaded neck on top.
    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM_Z)
        .circle(NECK_R)
        .extrude(NECK_TOP_Z - NECK_BOTTOM_Z + 0.004)
    )
    body = body.union(neck)

    # Open pour mouth: bore a through-channel down the neck axis.
    mouth = (
        cq.Workplane("XY")
        .workplane(offset=0.120)
        .circle(0.013)
        .extrude(0.115)
    )
    body = body.cut(mouth)
    return body


def _handle_solid() -> cq.Workplane:
    """Integrated side loop handle on the +X side."""
    slab = (
        cq.Workplane("XZ")
        .transformed(offset=(0.072, 0.110, 0.0))
        .rect(0.066, 0.150)
        .extrude(0.022, both=True)
    )
    try:
        slab = slab.edges("|Y").fillet(0.014)
    except Exception:
        pass
    hole = (
        cq.Workplane("XZ")
        .transformed(offset=(0.074, 0.110, 0.0))
        .ellipse(0.020, 0.050)
        .extrude(0.06, both=True)
    )
    slab = slab.cut(hole)
    return slab


def _neck_collar() -> cq.Workplane:
    """Raised collar ring at the top of the neck where the hinge attaches."""
    collar = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z)
        .circle(COLLAR_R)
        .extrude(COLLAR_TOP_Z - NECK_TOP_Z)
    )
    # Hollow center so the pour opening still passes through.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - 0.001)
        .circle(NECK_R - 0.002)
        .extrude(COLLAR_TOP_Z - NECK_TOP_Z + 0.002)
    )
    return collar.cut(bore)


def _spout_mesh():
    """Raised pour spout: short hollow lip that the flip-top lid seals against.
    Top stays below the closed lid disc bottom so it doesn't
    penetrate the lid."""
    o = cq.Workplane("XY").workplane(offset=0.206).circle(0.018).extrude(0.010)
    i = cq.Workplane("XY").workplane(offset=0.204).circle(0.013).extrude(0.014)
    spout = o.cut(i)
    return mesh_from_cadquery(spout, "pour_spout")


def _lid_mesh():
    """Flip-top snap lid with living-hinge tab at the rear edge.

    The lid part frame origin is at the hinge point (rear of collar top).
    In local coordinates the lid disc center sits at (0, +LID_CENTER_Y, 0)
    with its lower face raised above the collar top. The skirt is an annulus
    with enough radial clearance to wrap around the neck collar without
    intersecting the bottle mouth.
    A thin living-hinge tab bridges from the origin to the disc.
    """
    # Solid disc top: covers the neck opening.
    disc = (
        cq.Workplane("XY")
        .workplane(offset=LID_DISC_BOTTOM_Z + LID_THICKNESS)
        .center(0.0, LID_CENTER_Y)
        .circle(LID_RADIUS)
        .extrude(-LID_THICKNESS)
    )

    # Skirt: annular ring hanging below the disc edge with clearance around the
    # collar and spout, avoiding the neck/cap interpenetration seen in preview.
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=LID_DISC_BOTTOM_Z)
        .center(0.0, LID_CENTER_Y)
        .circle(LID_SKIRT_OUTER_R)
        .circle(LID_SKIRT_INNER_R)
        .extrude(-LID_SKIRT_HEIGHT)
    )
    lid = disc.union(skirt)

    # Living-hinge tab: a thin flexible bridge from the origin (hinge line)
    # to the rear edge of the disc. Overlaps the disc volume so the union
    # creates shared geometry.
    tab_length = LID_RADIUS * 0.6
    tab = (
        cq.Workplane("XY")
        .workplane(offset=LID_DISC_BOTTOM_Z + 0.001)
        .center(0.0, tab_length / 2.0)
        .rect(0.014, tab_length)
        .extrude(0.003)
    )
    lid = lid.union(tab)

    # Small raised grip nub on top front of the lid for finger opening.
    # Embeds into the disc so the union creates shared volume.
    nub = (
        cq.Workplane("XY")
        .workplane(offset=LID_DISC_BOTTOM_Z + LID_THICKNESS - 0.001)
        .center(0.0, LID_CENTER_Y + LID_RADIUS * 0.55)
        .circle(0.006)
        .extrude(0.008)
    )
    lid = lid.union(nub)

    return mesh_from_cadquery(lid, "flip_lid")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tide_detergent_bottle_flipcap")

    orange = model.material("tide_orange", rgba=(0.95, 0.46, 0.07, 1.0))
    blue_lid = model.material("lid_blue", rgba=(0.15, 0.40, 0.78, 1.0))
    label_yellow = model.material("label_yellow", rgba=(0.98, 0.83, 0.10, 1.0))
    label_blue = model.material("label_blue", rgba=(0.10, 0.22, 0.62, 1.0))
    he_silver = model.material("he_silver", rgba=(0.78, 0.80, 0.82, 1.0))

    # ---------------- body (root) ----------------
    body = model.part("body")

    shell = _body_shell().union(_handle_solid()).union(_neck_collar())
    body.visual(mesh_from_cadquery(shell, "jug_shell"), material=orange, name="jug_shell")

    # Fixed pour spout under the lid.
    body.visual(_spout_mesh(), material=orange, name="pour_spout")
    body.visual(
        Box((0.012, 0.008, LID_DISC_BOTTOM_Z + 0.001)),
        origin=Origin(
            xyz=(
                0.0,
                HINGE_Y + 0.004,
                COLLAR_TOP_Z + (LID_DISC_BOTTOM_Z + 0.001) / 2.0,
            )
        ),
        material=orange,
        name="hinge_riser",
    )

    # "Tide" bullseye label on the front (-Y) broad face.
    body.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(-0.010, -0.0475, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=label_yellow,
        name="bullseye_label",
    )
    body.visual(
        Box((0.060, 0.004, 0.018)),
        origin=Origin(xyz=(-0.010, -0.0475, 0.060)),
        material=label_blue,
        name="label_band",
    )
    # "he" badge near the shoulder.
    body.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=(-0.012, -0.047, 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=he_silver,
        name="he_badge",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.15, 0.095, 0.22)), mass=1.1, origin=Origin(xyz=(0.0, 0.0, 0.10))
    )

    # ---------------- lid (flip-top snap cap) ----------------
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=blue_lid, name="flip_lid")
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=LID_RADIUS, length=LID_THICKNESS + LID_SKIRT_HEIGHT),
        mass=0.015,
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_DISC_BOTTOM_Z - LID_SKIRT_HEIGHT / 2.0)),
    )

    # ---- body_to_lid: REVOLUTE hinge at rear of neck collar ----
    # Hinge origin at the back of the collar top. Axis along +X so positive
    # rotation lifts the lid (which extends in local +Y from the hinge) upward
    # toward +Z by the right-hand rule.
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=HINGE_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    # The pour spout sits inside the lid skirt when closed; the cap top is
    # raised and the skirt has radial clearance so the bottle mouth does not
    # penetrate the lid.
    disc_bottom_world_z = HINGE_Z + LID_DISC_BOTTOM_Z
    ctx.check(
        "closed lid disc is above the pour spout",
        disc_bottom_world_z >= SPOUT_TOP_Z + 0.001,
        details=f"disc bottom z={disc_bottom_world_z}, spout top z={SPOUT_TOP_Z}",
    )
    ctx.check(
        "closed lid disc is above the neck collar",
        disc_bottom_world_z >= COLLAR_TOP_Z + 0.001,
        details=f"disc bottom z={disc_bottom_world_z}, collar top z={COLLAR_TOP_Z}",
    )
    ctx.check(
        "lid skirt clears the collar and neck radially",
        LID_SKIRT_INNER_R >= COLLAR_R + 0.001 and LID_SKIRT_INNER_R >= NECK_R + 0.001,
        details=(
            f"skirt inner r={LID_SKIRT_INNER_R}, collar r={COLLAR_R}, neck r={NECK_R}"
        ),
    )

    # Lid covers the neck opening when closed: lid center should be above the
    # neck top and overlap the body in XY.
    ctx.expect_overlap(
        lid, body, axes="xy",
        min_overlap=0.015,
        name="closed lid overlaps neck region in XY",
    )

    # Closed lid sits at the top of the jug (above the shoulder).
    lid_aabb = ctx.part_world_aabb(lid)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "closed lid is near the top of the jug",
        lid_aabb is not None and body_aabb is not None
        and lid_aabb[1][2] >= body_aabb[1][2] - 0.010,
        details=f"lid z-max={lid_aabb[1][2]}, body z-max={body_aabb[1][2]}",
    )

    # Hinge opens the lid upward: at positive angle the lid z-max increases.
    rest_z_max = lid_aabb[1][2] if lid_aabb else 0.0
    with ctx.pose({hinge: 1.0}):
        open_aabb = ctx.part_world_aabb(lid)
    open_z_max = open_aabb[1][2] if open_aabb else 0.0
    ctx.check(
        "hinge opens the lid upward (z-max rises at positive angle)",
        open_z_max > rest_z_max + 0.005,
        details=f"rest z-max={rest_z_max}, open(1.0) z-max={open_z_max}",
    )

    # Fully open pose: lid swings well past 90 degrees.
    with ctx.pose({hinge: 2.0}):
        full_aabb = ctx.part_world_aabb(lid)
    full_z_min = full_aabb[0][2] if full_aabb else 0.0
    ctx.check(
        "lid swings fully open without penetrating the body",
        full_z_min >= HINGE_Z - 0.030,
        details=f"fully-open lid z-min={full_z_min}, hinge z={HINGE_Z}",
    )

    # The hinge axis is horizontal (no Z component).
    axis = hinge.axis
    ctx.check(
        "hinge axis is horizontal",
        abs(axis[2]) < 1e-6,
        details=f"hinge axis={axis}",
    )

    # Integrated side loop handle still present on +X.
    handle_aabb = ctx.part_element_world_aabb(body, elem="jug_shell")
    ctx.check(
        "jug extends out to the side loop handle on +X",
        handle_aabb is not None and handle_aabb[1][0] > 0.085,
        details=f"jug_shell x-max={None if handle_aabb is None else handle_aabb[1][0]}",
    )

    # Bullseye label present on the front face.
    ctx.expect_contact(
        body, body, elem_a="bullseye_label", elem_b="jug_shell",
        name="bullseye label is on the jug body",
    )

    # Lid is opaque blue (distinct from parent translucent cap).
    mat = lid.get_visual("flip_lid").material
    ctx.check(
        "lid is solid blue",
        mat is not None and mat.rgba[2] > mat.rgba[0] and mat.rgba[3] >= 0.99,
        details=f"lid rgba={None if mat is None else mat.rgba}",
    )

    return ctx.report()


object_model = build_object_model()
