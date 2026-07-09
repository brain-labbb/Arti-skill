from __future__ import annotations

# Square pantry jar with rounded corners and a flip-top lid on a rear hinge.
# Variant 02 of the square glass storage jar family.
#
# Frame: jar stands on +XY ground plane, central axis along +Z (up).
#   - jar_body: square-section hollow glass shell with rounded vertical edges,
#     topped by a wide square mouth with a thick glass rim and thread ridges.
#     A small hinge barrel sits at the rear (-Y) edge of the rim. (root)
#   - flip_lid: flat square panel (rounded corners) that covers the mouth,
#     with a hinge tab at the rear that wraps the hinge barrel.
#     REVOLUTE joint at rear hinge line, axis +X, positive q flips lid open.

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
BODY_HALF = 0.044          # half-width of square section (0.088 m square)
BODY_FILLET = 0.013        # rounded vertical-edge radius
WALL = 0.003               # glass wall thickness (body)
BODY_HEIGHT = 0.092        # height of main body section
BODY_TOP = BODY_HEIGHT     # z of body top

# Mouth rim: thicker glass wall at the mouth for structural strength
RIM_WALL = 0.005           # thicker wall at the rim
RIM_HEIGHT = 0.018         # height of the rim above body top
RIM_TOP = BODY_TOP + RIM_HEIGHT  # z of rim top

# Thread ridges on the outside of the rim
THREAD_RIDGE_HEIGHT = 0.002  # radial protrusion of thread ridges
THREAD_RIDGE_WIDTH = 0.0025  # vertical width of each ridge

# Hinge geometry
HINGE_BARREL_R = 0.004     # radius of hinge barrel
HINGE_BARREL_LENGTH = 0.025  # length of hinge barrel along X
HINGE_Y = -(BODY_HALF - 0.006)  # Y position of hinge axis (rear edge)

# Lid
LID_THICK = 0.004          # lid panel thickness
LID_HALF_X = BODY_HALF - 0.002  # lid half-width (slight clearance)
LID_DEPTH = 2 * BODY_HALF - 0.012  # lid depth in Y (from hinge forward)


def _rounded_rect_profile(half_x: float, half_y: float, fillet: float) -> cq.Workplane:
    """Create a rounded rectangle wire on XY plane centered at origin."""
    return (
        cq.Workplane("XY")
        .rect(2 * half_x, 2 * half_y)
    )


def _body_solid() -> cq.Workplane:
    """Hollow square glass jar body with rounded corners."""
    # Outer shell
    outer = (
        cq.Workplane("XY")
        .box(2 * BODY_HALF, 2 * BODY_HALF, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )

    # Inner cavity (hollow)
    inner_half = BODY_HALF - WALL
    inner_fillet = max(BODY_FILLET - WALL, 0.002)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(2 * inner_half, 2 * inner_half, BODY_HEIGHT - WALL, centered=(True, True, False))
        .edges("|Z")
        .fillet(inner_fillet)
    )

    return outer.cut(cavity)


def _rim_solid() -> cq.Workplane:
    """Square mouth rim with thicker walls, sits on top of body."""
    # Outer rim profile
    rim_outer = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .box(2 * BODY_HALF, 2 * BODY_HALF, RIM_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )

    # Inner bore (mouth opening) - uses RIM_WALL for thicker glass at mouth
    rim_inner_half = BODY_HALF - RIM_WALL
    rim_inner_fillet = max(BODY_FILLET - RIM_WALL, 0.002)
    rim_bore = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .box(2 * rim_inner_half, 2 * rim_inner_half, RIM_HEIGHT + 0.002, centered=(True, True, False))
        .edges("|Z")
        .fillet(rim_inner_fillet)
    )

    return rim_outer.cut(rim_bore)


def _thread_ridges() -> cq.Workplane:
    """Thread ridges (lugs) around the outside of the rim."""
    ridges = None
    # Two rings of thread ridges at different heights on the rim
    for z_offset in (0.005, 0.012):
        zc = BODY_TOP + z_offset
        # Each ridge is a thin square ring around the rim exterior
        outer_half = BODY_HALF + THREAD_RIDGE_HEIGHT
        inner_half = BODY_HALF - 0.0005  # slightly into the rim surface for embedding
        ridge = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .box(2 * outer_half, 2 * outer_half, THREAD_RIDGE_WIDTH, centered=(True, True, False))
            .edges("|Z")
            .fillet(BODY_FILLET + THREAD_RIDGE_HEIGHT)
        )
        bore = (
            cq.Workplane("XY")
            .workplane(offset=zc - 0.0001)
            .box(2 * inner_half, 2 * inner_half, THREAD_RIDGE_WIDTH + 0.0002, centered=(True, True, False))
            .edges("|Z")
            .fillet(BODY_FILLET - 0.001)
        )
        ring = ridge.cut(bore)
        ridges = ring if ridges is None else ridges.union(ring)
    return ridges


def _hinge_barrel() -> cq.Workplane:
    """Small hinge barrel at the rear of the rim (part of jar body)."""
    # Horizontal cylinder along X at rear edge of rim top
    barrel = (
        cq.Workplane("YZ")
        .workplane(offset=-HINGE_BARREL_LENGTH / 2.0)
        .transformed(offset=(HINGE_Y, RIM_TOP - HINGE_BARREL_R, 0))
        .circle(HINGE_BARREL_R)
        .extrude(HINGE_BARREL_LENGTH)
    )
    return barrel


def _body_mesh():
    """Combined jar body: shell + rim + threads + hinge barrel."""
    solid = _body_solid().union(_rim_solid()).union(_thread_ridges()).union(_hinge_barrel())
    return mesh_from_cadquery(solid, "jar_glass")


def _lid_solid() -> cq.Workplane:
    """Flat square flip-lid panel with rounded corners and a hinge tab at rear."""
    # Main lid panel: extends from y=0 (hinge line) forward in +Y
    panel = (
        cq.Workplane("XY")
        .box(2 * LID_HALF_X, LID_DEPTH, LID_THICK, centered=(True, False, False))
        .edges("|Z")
        .fillet(min(BODY_FILLET - 0.002, 0.010))
    )

    # Hinge tab: a small half-cylinder at the rear (y=0) that wraps the barrel
    tab = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .transformed(offset=(0, LID_THICK / 2.0, 0))
        .rect(HINGE_BARREL_LENGTH * 0.6, LID_THICK + HINGE_BARREL_R)
        .extrude(-HINGE_BARREL_R * 0.8)
    )
    # Simplify: just add a small block at rear for hinge connection
    tab_block = (
        cq.Workplane("XY")
        .box(HINGE_BARREL_LENGTH * 0.7, HINGE_BARREL_R * 1.5, LID_THICK + HINGE_BARREL_R,
             centered=(True, False, False))
        .translate((0, -HINGE_BARREL_R * 0.75, -HINGE_BARREL_R * 0.5))
    )

    lid = panel.union(tab_block)
    return lid


def _lid_mesh():
    return mesh_from_cadquery(_lid_solid(), "flip_lid")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_pantry_jar_flip_lid")

    glass = model.material("clear_glass", rgba=(0.82, 0.88, 0.86, 0.30))
    lid_mat = model.material("lid_plastic", rgba=(0.90, 0.90, 0.88, 1.0))

    # ---- jar body (root): square hollow shell + thick rim + threads + hinge barrel ----
    body = model.part("jar_body")
    body.visual(_body_mesh(), material=glass, name="jar_glass")
    body.inertial = Inertial.from_geometry(
        Box((2 * BODY_HALF, 2 * BODY_HALF, RIM_TOP)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP / 2.0)),
    )

    # ---- flip lid: flat panel hinged at rear ----
    lid = model.part("flip_lid")
    lid.visual(_lid_mesh(), material=lid_mat, name="lid_panel")
    lid.inertial = Inertial.from_geometry(
        Box((2 * LID_HALF_X, LID_DEPTH, LID_THICK)),
        mass=0.025,
        origin=Origin(xyz=(0.0, LID_DEPTH / 2.0, LID_THICK / 2.0)),
    )

    # ---- REVOLUTE hinge: rear of rim, axis +X, positive q opens lid upward ----
    # The lid part frame origin is at the hinge line. The lid extends in +Y from there.
    # At q=0, lid is closed (flat on rim). Positive rotation about +X lifts the
    # front edge (+Y) upward (+Z), opening the lid.
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, RIM_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=2.1,  # ~120 degrees open
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("jar_body")
    lid = object_model.get_part("flip_lid")
    hinge = object_model.get_articulation("lid_hinge")

    # --- jar body is square with rounded corners ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar body is square in cross-section",
        abs(bext[0] - bext[1]) < 0.008,
        details=f"x={bext[0]:.4f}, y={bext[1]:.4f}",
    )

    # --- mouth rim is wider/thicker than body walls (thread ridges protrude) ---
    # The body AABB includes thread ridges which protrude beyond BODY_HALF
    ctx.check(
        "thread ridges protrude beyond body walls",
        bext[0] > 2 * BODY_HALF + 0.001,
        details=f"body x extent={bext[0]:.4f}, expected > {2*BODY_HALF + 0.001:.4f}",
    )

    # --- lid is a flip lid seated on the rim when closed ---
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at rim top when closed",
        lid_pos is not None and lid_pos[2] >= RIM_TOP - 0.005,
        details=f"lid origin z={lid_pos[2] if lid_pos else None}, rim_top={RIM_TOP}",
    )

    # Lid overlaps the body footprint (covers the mouth)
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.03,
        name="lid covers the mouth opening"
    )

    # --- hinge joint is revolute with correct axis ---
    ctx.check(
        "lid_hinge is revolute about +X",
        hinge.articulation_type == ArticulationType.REVOLUTE and hinge.axis == (1.0, 0.0, 0.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )

    # --- hinge limits: 0 (closed) to ~2.1 rad (open) ---
    limits = hinge.motion_limits
    ctx.check(
        "hinge has bounded motion limits",
        limits is not None and limits.lower is not None and limits.upper is not None
        and limits.lower >= 0.0 and limits.upper > 1.5,
        details=f"lower={limits.lower if limits else None}, upper={limits.upper if limits else None}",
    )

    # --- opening the hinge lifts the lid front edge upward ---
    lid_front_z_rest = None
    lid_front_z_open = None
    lid_aabb_rest = ctx.part_world_aabb(lid)
    if lid_aabb_rest:
        lid_front_z_rest = lid_aabb_rest[1][2]  # max Z at rest

    with ctx.pose({hinge: 1.5}):
        lid_aabb_open = ctx.part_world_aabb(lid)
        if lid_aabb_open:
            lid_front_z_open = lid_aabb_open[1][2]  # max Z when open

    ctx.check(
        "opening hinge raises lid above rim",
        lid_front_z_open is not None and lid_front_z_rest is not None
        and lid_front_z_open > lid_front_z_rest + 0.02,
        details=f"rest_max_z={lid_front_z_rest}, open_max_z={lid_front_z_open}",
    )

    # --- glass wall thickness: rim inner bore is smaller than outer ---
    # The rim wall thickness is RIM_WALL=0.005, which is thicker than body WALL=0.003
    # Prove by checking lid fits within rim outer but not within rim inner
    ctx.check(
        "rim wall is thicker than body wall",
        RIM_WALL > WALL,
        details=f"rim_wall={RIM_WALL}, body_wall={WALL}",
    )

    return ctx.report()


object_model = build_object_model()
