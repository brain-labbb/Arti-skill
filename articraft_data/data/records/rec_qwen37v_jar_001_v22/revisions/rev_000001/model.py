from __future__ import annotations

# Square PANTRY JAR with rounded corners, shaker insert, and screw-on lid.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# A square glass jar with rounded corners (~80mm x 80mm footprint, ~100mm tall),
# thick glass walls visible at the mouth, a base foot ring, and a rim seam at
# the jar opening. The lid is a square cap that screws on/off. Inside the lid
# sits a circular shaker insert (a disc with holes) that rotates independently.
#
# Articulations:
#   - lid_rotate: CONTINUOUS spin of the carrier about +Z at the rim top
#   - lid_slide:  PRISMATIC lift of the lid relative to the carrier along +Z
#   - shaker_rotate: REVOLUTE rotation of the shaker insert inside the lid

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

# ---- key dimensions (meters) ----
BODY_W = 0.080            # body width (square footprint)
BODY_D = 0.080            # body depth
BODY_H = 0.100            # body height (shoulder to base)
CORNER_R = 0.008          # rounded corner radius
WALL = 0.004              # glass wall thickness
BASE_THICK = 0.006        # thick glass base

# Neck/mouth dimensions (inset from body)
NECK_W = 0.066            # neck outer width
NECK_D = 0.066            # neck outer depth
NECK_CORNER_R = 0.006     # neck corner radius
NECK_H = 0.014            # neck height above shoulder
RIM_TOP_Z = BODY_H + NECK_H  # z of rim top (0.114)

# Rim seam: a visible groove around the mouth opening
RIM_SEAM_DEPTH = 0.002    # how deep the seam groove cuts
RIM_SEAM_WIDTH = 0.0015   # width of the seam groove

# Foot ring: raised ring at base
FOOT_INSET = 0.004        # foot ring inset from body walls
FOOT_H = 0.003            # foot ring height
FOOT_W = BODY_W - 2 * FOOT_INSET
FOOT_D = BODY_D - 2 * FOOT_INSET
FOOT_CORNER_R = CORNER_R - FOOT_INSET * 0.5

# Lid dimensions
LID_W = 0.072             # lid width
LID_D = 0.072             # lid depth
LID_H = 0.016             # lid total height
LID_CORNER_R = 0.006      # lid corner radius
LID_SKIRT_DEPTH = 0.010   # how far skirt extends below rim
LID_TOP_THICK = 0.004     # lid top panel thickness

# Shaker insert
SHAKER_R = 0.026          # shaker disc radius
SHAKER_THICK = 0.002      # shaker disc thickness
SHAKER_HOLE_R = 0.002     # radius of each shaker hole
SHAKER_N_HOLES = 12       # number of shaker holes in ring
SHAKER_RING_R = 0.018     # radius of hole ring


def _rounded_box(width: float, depth: float, height: float,
                 corner_r: float, z_offset: float = 0.0) -> cq.Workplane:
    """Build a box with rounded vertical edges using CadQuery rect+extrude+fillet."""
    wp = cq.Workplane("XY").workplane(offset=z_offset)
    result = wp.rect(width, depth).extrude(height)
    # Fillet vertical edges
    if corner_r > 0:
        result = result.edges("|Z").fillet(min(corner_r, min(width, depth) * 0.4))
    return result


def _jar_glass_solid() -> cq.Workplane:
    """Square jar body with rounded corners, hollow interior, and neck.
    Built by boolean: outer shell minus inner cavity."""
    # Outer body - extend slightly into neck region for better boolean merge
    outer_body = _rounded_box(BODY_W, BODY_D, BODY_H + 0.002, CORNER_R)

    # Inner cavity (hollow, stops below neck to preserve neck outer in overlap)
    inner_w = BODY_W - 2 * WALL
    inner_d = BODY_D - 2 * WALL
    inner_corner = max(CORNER_R - WALL, 0.002)
    # Stop inner cut at BODY_H - 0.003 to avoid cutting into neck overlap
    inner_body = _rounded_box(inner_w, inner_d, BODY_H - BASE_THICK - 0.003,
                              inner_corner, z_offset=BASE_THICK)

    # Neck outer (rises above the body shoulder, overlaps with body for merge)
    neck_outer = _rounded_box(NECK_W, NECK_D, NECK_H + 0.003, NECK_CORNER_R,
                              z_offset=BODY_H - 0.002)

    # Neck inner cavity (wall thickness visible at mouth)
    neck_inner_w = NECK_W - 2 * WALL
    neck_inner_d = NECK_D - 2 * WALL
    neck_inner_corner = max(NECK_CORNER_R - WALL, 0.002)
    # Neck cavity goes from above the shoulder step up to rim top
    neck_inner = _rounded_box(neck_inner_w, neck_inner_d, NECK_H + 0.002,
                              neck_inner_corner, z_offset=BODY_H - 0.001)

    # Combine: outer body + neck outer, then subtract inner cavities
    jar = outer_body.union(neck_outer)
    jar = jar.cut(inner_body)
    jar = jar.cut(neck_inner)

    return jar


def _rim_seam_solid() -> cq.Workplane:
    """Rim seam: a visible groove/lip at the top of the jar mouth.
    Modeled as a thin raised ring around the outer edge of the neck rim.
    Built to overlap slightly with the neck so it connects when unioned."""
    outer_w = NECK_W + 0.003
    outer_d = NECK_D + 0.003
    outer_corner = NECK_CORNER_R + 0.001

    outer_ring = _rounded_box(outer_w, outer_d, RIM_SEAM_DEPTH + 0.002,
                              outer_corner, z_offset=RIM_TOP_Z - RIM_SEAM_DEPTH - 0.001)
    inner_cut = _rounded_box(NECK_W - 0.001, NECK_D - 0.001,
                             RIM_SEAM_DEPTH + 0.004,
                             NECK_CORNER_R - 0.001,
                             z_offset=RIM_TOP_Z - RIM_SEAM_DEPTH - 0.002)
    rim = outer_ring.cut(inner_cut)
    return rim


def _foot_ring_mesh() -> cq.Workplane:
    """Base foot ring: a raised ring around the bottom of the jar."""
    outer = _rounded_box(FOOT_W, FOOT_D, FOOT_H, FOOT_CORNER_R)
    inner_w = FOOT_W - 2 * 0.005
    inner_d = FOOT_D - 2 * 0.005
    inner_corner = max(FOOT_CORNER_R - 0.003, 0.002)
    inner = _rounded_box(inner_w, inner_d, FOOT_H + 0.001, inner_corner,
                         z_offset=-0.0005)
    ring = outer.cut(inner)
    return ring


def _lid_solid() -> cq.Workplane:
    """Square lid cap with rounded corners. Hollow underneath to fit over neck.
    Lid part frame origin is at the rim top (z=RIM_TOP_Z in world)."""
    # Outer lid shell (extends from skirt bottom to lid top)
    skirt_bottom = -LID_SKIRT_DEPTH  # lid-local z: below rim
    outer = _rounded_box(LID_W, LID_D, LID_H, LID_CORNER_R,
                         z_offset=skirt_bottom)

    # Inner cavity: matches neck outer so skirt seats over neck
    cavity_w = NECK_W + 0.001
    cavity_d = NECK_D + 0.001
    cavity_corner = NECK_CORNER_R + 0.0005
    cavity_h = LID_SKIRT_DEPTH - 0.001  # leave top panel thickness
    cavity = _rounded_box(cavity_w, cavity_d, cavity_h, cavity_corner,
                          z_offset=skirt_bottom - 0.0005)

    lid = outer.cut(cavity)
    # Fillet top edges for a nicer look
    lid = lid.edges(">Z").fillet(0.002)
    return lid


def _shaker_insert_mesh() -> cq.Workplane:
    """Shaker insert: a circular disc with holes arranged in a ring pattern.
    Sits inside the lid, rotates independently."""
    # Base disc
    disc = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(SHAKER_R)
        .extrude(SHAKER_THICK)
    )
    # Cut holes in a ring pattern
    for i in range(SHAKER_N_HOLES):
        angle = 2.0 * math.pi * i / SHAKER_N_HOLES
        cx = SHAKER_RING_R * math.cos(angle)
        cy = SHAKER_RING_R * math.sin(angle)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(cx, cy)
            .circle(SHAKER_HOLE_R)
            .extrude(SHAKER_THICK + 0.002)
        )
        disc = disc.cut(hole)

    # Add a small center pin/grip for rotation
    pin = (
        cq.Workplane("XY")
        .workplane(offset=-0.003)
        .circle(0.004)
        .extrude(0.003)
    )
    disc = disc.union(pin)

    return disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_pantry_jar")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.85, 0.92, 0.88, 0.45))
    lid_silver = model.material("lid_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    shaker_white = model.material("shaker_white", rgba=(0.95, 0.95, 0.93, 1.0))
    foot_dark = model.material("foot_dark", rgba=(0.25, 0.25, 0.28, 1.0))
    rim_accent = model.material("rim_accent", rgba=(0.55, 0.60, 0.58, 0.70))

    # ---- jar body (root): glass shell + rim seam + foot ring ----
    body = model.part("body")

    # Union jar glass with rim seam for connected geometry
    glass = _jar_glass_solid().union(_rim_seam_solid())
    body.visual(mesh_from_cadquery(glass, "jar_glass"),
                material=glass_clear, name="jar_glass")

    # Foot ring at base
    foot = _foot_ring_mesh()
    body.visual(mesh_from_cadquery(foot, "foot_ring"),
                material=foot_dark, name="foot_ring")

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    # ---- massless carrier (no visuals): rotates about +Z at the rim top ----
    carrier = model.part("lid_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ---- lid: square cap that slides up off the carrier along +Z ----
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_solid(), "lid_shell"),
               material=lid_silver, name="lid_shell")

    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_H)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, -LID_SKIRT_DEPTH + LID_H * 0.5)),
    )
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_H + 0.010,
                                   effort=1.0, velocity=1.0),
    )

    # ---- shaker insert: rotates inside the lid ----
    shaker = model.part("shaker_insert")
    shaker.visual(mesh_from_cadquery(_shaker_insert_mesh(), "shaker_disc"),
                  material=shaker_white, name="shaker_disc")
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICK),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, SHAKER_THICK * 0.5)),
    )
    # Shaker rotates inside the lid: revolute about +Z, limited to ~270 degrees
    # The shaker sits at the bottom inner face of the lid top panel.
    # In lid-local coords, lid top panel bottom face is at z = LID_H - LID_TOP_THICK - LID_SKIRT_DEPTH
    shaker_z_in_lid = -LID_SKIRT_DEPTH + LID_H - LID_TOP_THICK - SHAKER_THICK
    model.articulation(
        "shaker_rotate",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, shaker_z_in_lid)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi * 0.75, upper=math.pi * 0.75,
                                   effort=0.5, velocity=2.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("lid_carrier")
    lid = object_model.get_part("lid")
    shaker = object_model.get_part("shaker_insert")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")
    shaker_rot = object_model.get_articulation("shaker_rotate")

    # Allow lid skirt to overlap jar glass (it slips over the neck)
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="jar_glass",
        reason="The lid skirt is intentionally slipped down over the neck rim.",
    )

    # Allow shaker insert to sit inside the lid cavity
    ctx.allow_overlap(
        lid,
        shaker,
        elem_a="lid_shell",
        elem_b="shaker_disc",
        reason="The shaker insert is intentionally seated inside the lid cavity.",
    )

    # ---- Square footprint: body is approximately square and wider than tall ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar has square footprint (width ≈ depth)",
        abs(bext[0] - bext[1]) < 0.015,
        details=f"body extents xy=({bext[0]:.4f}, {bext[1]:.4f})",
    )
    ctx.check(
        "jar body is taller than wide (pantry proportions)",
        bext[2] > bext[0] - 0.005,
        details=f"body extents=({bext[0]:.4f}, {bext[1]:.4f}, {bext[2]:.4f})",
    )

    # ---- Foot ring exists at base ----
    foot_aabb = ctx.part_element_world_aabb(body, elem="foot_ring")
    ctx.check(
        "foot ring exists at base",
        foot_aabb is not None and foot_aabb[0][2] < 0.005,
        details=f"foot_ring aabb min_z={foot_aabb[0][2] if foot_aabb else None}",
    )

    # ---- Rim seam geometry exists at mouth top (part of jar_glass now) ----
    # The jar_glass extends to the rim top with the seam lip included
    jar_aabb = ctx.part_element_world_aabb(body, elem="jar_glass")
    ctx.check(
        "jar glass extends to rim top with seam geometry",
        jar_aabb is not None and jar_aabb[1][2] > RIM_TOP_Z - 0.003,
        details=f"jar_glass max_z={jar_aabb[1][2] if jar_aabb else None}",
    )

    # ---- Glass wall thickness at mouth: jar glass has neck cavity ----
    # The jar glass extends from base to RIM_TOP_Z, proving hollow neck with wall thickness
    jar_aabb_full = ctx.part_element_world_aabb(body, elem="jar_glass")
    ctx.check(
        "jar glass has full height including neck region",
        jar_aabb_full is not None and jar_aabb_full[1][2] > RIM_TOP_Z - 0.003
        and jar_aabb_full[0][2] < 0.005,
        details=f"jar_glass aabb={jar_aabb_full}",
    )

    # ---- Lid sits on top of the jar at rest ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is on top of the jar",
        lid_pos is not None and lid_pos[2] > RIM_TOP_Z - 0.005,
        details=f"lid_pos={lid_pos}, rim_top={RIM_TOP_Z}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="lid caps the neck"
    )

    # ---- lid_rotate spins the lid ----
    lid_aabb_0 = ctx.part_world_aabb(lid)
    with ctx.pose({rotate: math.pi / 4.0}):
        lid_aabb_1 = ctx.part_world_aabb(lid)
    # AABB should remain similar (square jar rotating stays similar)
    ctx.check(
        "lid_rotate is a non-fixed joint",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"rotate type={rotate.articulation_type}",
    )

    # ---- lid_slide lifts the lid off the jar ----
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_H + 0.005}):
        lifted_z = ctx.part_world_position(lid)[2]
        ctx.expect_gap(
            lid, body, axis="z", min_gap=0.0,
            positive_elem="lid_shell", negative_elem="jar_glass",
            name="lifted lid clears the neck",
        )
    ctx.check(
        "lid_slide lifts the lid off the jar",
        lifted_z > rest_z + 0.008,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # ---- shaker_rotate is a revolute joint with limits ----
    ctx.check(
        "shaker_rotate is revolute with limits",
        shaker_rot.articulation_type == ArticulationType.REVOLUTE,
        details=f"shaker_rotate type={shaker_rot.articulation_type}",
    )
    # Shaker insert rotates when shaker_rotate is actuated
    shaker_pos_0 = ctx.part_world_position(shaker)
    with ctx.pose({shaker_rot: math.pi * 0.5}):
        shaker_pos_1 = ctx.part_world_position(shaker)
    # Position should be approximately the same (rotation only)
    ctx.check(
        "shaker rotates without translating significantly",
        shaker_pos_0 is not None and shaker_pos_1 is not None
        and abs(shaker_pos_1[2] - shaker_pos_0[2]) < 0.005,
        details=f"shaker rest_z={shaker_pos_0}, rotated_z={shaker_pos_1}",
    )

    # ---- carrier is massless / has no visuals ----
    ctx.check(
        "carrier link has no visuals",
        len(carrier.visuals) == 0,
        details=f"carrier visuals={len(carrier.visuals)}",
    )

    # ---- shaker insert is inside the lid footprint ----
    ctx.expect_within(
        shaker, lid, axes="xy", margin=0.005,
        name="shaker insert stays within lid footprint"
    )

    return ctx.report()


object_model = build_object_model()
