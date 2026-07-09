from __future__ import annotations

# Round cast-iron manhole / well cover with a waffle-grid top, seated in a
# circular cast-iron frame ring, hinged at one edge to swing open and reveal the
# shaft void below.  A stout cast-iron pick-bar is seated across a recessed
# channel on the lid and pivots up on a REVOLUTE joint for lifting-key hook
# engagement.
#
# Coordinate convention:
#   - up is +Z; the frame ring sits on the ground at z=0.
#   - the cover "front" (the pick-hole / lifting edge) faces +X; the hinge is at
#     the rear edge (-X).
#
# Structure:
#   - frame (root, static): cast-iron frame ring with recessed seat and shaft.
#   - cover (REVOLUTE about -Y at the rear hinge): round lid with waffle grid,
#     pick-hole, recessed pick-bar channel, and cast pivot lugs.
#   - hinge_pin (FIXED to frame): exposed hinge pin / knuckle.
#   - pick_bar (REVOLUTE about -Y at the pivot lugs): cast-iron lifting bar
#     that swings from flush stowed to roughly upright.

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

# --- key dimensions (meters) ---
COVER_DIA = 0.610
COVER_R = COVER_DIA / 2.0
COVER_T = 0.042
GRID_RELIEF = 0.010
GRID_PITCH = 0.052
GRID_GROOVE = 0.012

FRAME_OUTER_R = 0.370
FRAME_INNER_R = COVER_R + 0.006
FRAME_H = 0.110
SEAT_DROP = 0.028
COLLAR_R = COVER_R - 0.030
SHAFT_FLOOR_Z = 0.016

SEAT_Z = FRAME_H - SEAT_DROP
COVER_BOTTOM_Z = SEAT_Z
COVER_TOP_Z = COVER_BOTTOM_Z + COVER_T

HINGE_X = -COVER_R - 0.004
HINGE_Z = COVER_BOTTOM_Z + COVER_T * 0.5

# --- pick-bar dimensions (mesh-local coordinates of the cover blank) ---
BAR_LENGTH = 0.200
BAR_WIDTH = 0.026
BAR_THICKNESS = 0.014
CHANNEL_LENGTH = 0.230
CHANNEL_WIDTH = 0.050
CHANNEL_DEPTH = 0.018
CHANNEL_X_OFFSET = 0.010  # channel center slightly forward of cover center
PIVOT_X_MESH = CHANNEL_X_OFFSET - CHANNEL_LENGTH / 2.0 + 0.014
CHANNEL_FLOOR_Z = COVER_T + GRID_RELIEF - CHANNEL_DEPTH
BAR_STOWED_CENTER_Z = CHANNEL_FLOOR_Z + BAR_THICKNESS / 2.0 - 0.001
PIVOT_Z_MESH = BAR_STOWED_CENTER_Z

LUG_HEIGHT = 0.016
LUG_WIDTH = 0.010
LUG_LENGTH = 0.024
LUG_Y_CENTER = 0.019


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------

def _build_frame_mesh():
    """Cast-iron frame ring with a recessed cover seat and a shaft void."""
    ring = (
        cq.Workplane("XY")
        .circle(FRAME_OUTER_R)
        .extrude(FRAME_H)
    )
    upper_bore = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z)
        .circle(FRAME_INNER_R)
        .extrude(FRAME_H - SEAT_Z + 0.002)
    )
    ring = ring.cut(upper_bore)
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=SHAFT_FLOOR_Z)
        .circle(COLLAR_R)
        .extrude(SEAT_Z - SHAFT_FLOOR_Z + 0.002)
    )
    ring = ring.cut(shaft)
    ring = ring.edges(">Z").fillet(0.006)

    lug = (
        cq.Workplane("XY")
        .center(-(FRAME_INNER_R + 0.018), 0.0)
        .box(0.060, 0.110, 0.050, centered=(True, True, False))
        .edges("|X")
        .fillet(0.008)
        .translate((0, 0, SEAT_Z - 0.004))
    )
    ring = ring.union(lug)

    return mesh_from_cadquery(ring, "manhole_frame", unit_scale=1.0)


def _build_cover_mesh():
    """Round lid with waffle grid, pick-hole, recessed pick-bar channel and
    cast pivot lugs for the lifting bar."""
    # Base plate.
    plate = (
        cq.Workplane("XY")
        .circle(COVER_R)
        .extrude(COVER_T)
        .edges(">Z")
        .fillet(0.006)
    )
    # Raised waffle grid slab.
    grid_slab = (
        cq.Workplane("XY")
        .workplane(offset=COVER_T)
        .circle(COVER_R - 0.024)
        .extrude(GRID_RELIEF)
    )
    n = int((2 * (COVER_R - 0.024)) // GRID_PITCH) + 2
    groove_cutters = None
    for i in range(-(n // 2), n // 2 + 1):
        pos = i * GRID_PITCH
        gx = (
            cq.Workplane("XY")
            .workplane(offset=COVER_T + GRID_RELIEF / 2.0)
            .center(0.0, pos)
            .box(2 * COVER_R, GRID_GROOVE, GRID_RELIEF + 0.004, centered=(True, True, True))
        )
        gy = (
            cq.Workplane("XY")
            .workplane(offset=COVER_T + GRID_RELIEF / 2.0)
            .center(pos, 0.0)
            .box(GRID_GROOVE, 2 * COVER_R, GRID_RELIEF + 0.004, centered=(True, True, True))
        )
        groove_cutters = gx if groove_cutters is None else groove_cutters.union(gx)
        groove_cutters = groove_cutters.union(gy)
    grid_slab = grid_slab.cut(groove_cutters)
    cover = plate.union(grid_slab)

    # Pick-hole slot near the front edge.
    pick = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .center(COVER_R - 0.060, 0.0)
        .slot2D(0.060, 0.024, 0.0)
        .extrude(COVER_T + GRID_RELIEF + 0.01)
    )
    cover = cover.cut(pick)

    # --- pick-bar channel ---
    # Recessed channel through the waffle grid for the stowed bar.
    channel_center_x = CHANNEL_X_OFFSET
    channel_cut = (
        cq.Workplane("XY")
        .workplane(offset=CHANNEL_FLOOR_Z)
        .center(channel_center_x, 0.0)
        .box(CHANNEL_LENGTH + 0.004, CHANNEL_WIDTH, CHANNEL_DEPTH + 0.004, centered=(True, True, False))
    )
    cover = cover.cut(channel_cut)

    # --- pivot lugs at the rear end of the channel ---
    for i in range(2):
        sign = -1 if i == 0 else 1
        lug_y = sign * LUG_Y_CENTER
        lug = (
            cq.Workplane("XY")
            .workplane(offset=CHANNEL_FLOOR_Z - 0.001)
            .center(PIVOT_X_MESH, lug_y)
            .box(LUG_LENGTH, LUG_WIDTH, LUG_HEIGHT + 0.001, centered=(True, True, False))
            .edges(">Z")
            .fillet(0.003)
        )
        cover = cover.union(lug)

    # Hinge knuckles at the rear edge.
    for ky in (-0.030, 0.030):
        knuckle = (
            cq.Workplane("YZ")
            .workplane(offset=-COVER_R + 0.006)
            .center(ky, COVER_T * 0.5)
            .circle(0.016)
            .extrude(-0.018)
        )
        bore = (
            cq.Workplane("YZ")
            .workplane(offset=-COVER_R + 0.010)
            .center(ky, COVER_T * 0.5)
            .circle(0.009)
            .extrude(-0.026)
        )
        cover = cover.union(knuckle.cut(bore))

    return mesh_from_cadquery(cover, "manhole_cover", unit_scale=1.0)


def _build_pick_bar_mesh():
    """Cast-iron pick-bar: pivot at the origin, bar extends along +X.

    Authored so the pivot axis passes through the local origin and the bar
    body lies along +X when stowed (q=0).
    """
    # Main bar body.
    bar = (
        cq.Workplane("XY")
        .center(BAR_LENGTH / 2.0 + 0.008, 0.0)
        .box(BAR_LENGTH, BAR_WIDTH, BAR_THICKNESS, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.003)
    )
    # Pivot tab at the base (rounded for cast strength, fits between lugs).
    tab = (
        cq.Workplane("XY")
        .box(0.026, BAR_WIDTH - 0.004, BAR_THICKNESS, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.005)
    )
    bar = bar.union(tab)
    # Slight grip bulge near the free end for visual identification.
    grip = (
        cq.Workplane("XY")
        .center(BAR_LENGTH + 0.002, 0.0)
        .box(0.020, BAR_WIDTH + 0.006, BAR_THICKNESS + 0.004, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.005)
    )
    bar = bar.union(grip)
    return mesh_from_cadquery(bar, "pick_bar", unit_scale=1.0)


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manhole_well_cover")

    iron = model.material("cast_iron", rgba=(0.22, 0.21, 0.20, 1.0))
    iron_dark = model.material("cast_iron_dark", rgba=(0.16, 0.15, 0.15, 1.0))
    void_black = model.material("shaft_void", rgba=(0.04, 0.04, 0.05, 1.0))
    pin_steel = model.material("hinge_pin", rgba=(0.45, 0.45, 0.47, 1.0))
    bar_iron = model.material("bar_iron", rgba=(0.26, 0.24, 0.21, 1.0))

    # --- frame (root) ---
    frame = model.part("frame")
    frame.visual(_build_frame_mesh(), material=iron, name="frame_ring")
    frame.visual(
        Cylinder(radius=COLLAR_R - 0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_FLOOR_Z + 0.001)),
        material=void_black,
        name="shaft_floor",
    )
    frame.inertial = Inertial.from_geometry(
        Cylinder(radius=FRAME_OUTER_R, length=FRAME_H),
        mass=60.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_H / 2.0)),
    )

    # --- hinge pin (fixed to frame) ---
    pin = model.part("hinge_pin")
    pin.visual(
        Cylinder(radius=0.009, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="pin",
    )
    pin.inertial = Inertial.from_geometry(Cylinder(radius=0.009, length=0.092), mass=0.1)
    model.articulation(
        "frame_to_pin",
        ArticulationType.FIXED,
        parent=frame,
        child=pin,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
    )

    # --- cover (revolute about the rear hinge) ---
    cover = model.part("cover")
    cover.visual(
        _build_cover_mesh(),
        origin=Origin(xyz=(COVER_R + 0.004, 0.0, -COVER_T * 0.5)),
        material=iron,
        name="cover_plate",
    )
    cover.inertial = Inertial.from_geometry(
        Cylinder(radius=COVER_R, length=COVER_T),
        mass=45.0,
        origin=Origin(xyz=(COVER_R + 0.004, 0.0, 0.0)),
    )
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cover,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    # --- pick-bar (revolute pivot on the cover) ---
    pick_bar = model.part("pick_bar")
    pick_bar.visual(
        _build_pick_bar_mesh(),
        material=bar_iron,
        name="bar",
    )
    pick_bar.inertial = Inertial.from_geometry(
        Box((BAR_LENGTH + 0.020, BAR_WIDTH + 0.006, BAR_THICKNESS + 0.004)),
        mass=1.2,
        origin=Origin(xyz=(BAR_LENGTH / 2.0, 0.0, 0.0)),
    )

    # Pivot origin expressed in cover-part coordinates.
    pivot_x_cover = COVER_R + 0.004 + PIVOT_X_MESH
    pivot_z_cover = -COVER_T * 0.5 + PIVOT_Z_MESH
    model.articulation(
        "cover_to_bar",
        ArticulationType.REVOLUTE,
        parent=cover,
        child=pick_bar,
        origin=Origin(xyz=(pivot_x_cover, 0.0, pivot_z_cover)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=1.50),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    cover = object_model.get_part("cover")
    pin = object_model.get_part("hinge_pin")
    pick_bar = object_model.get_part("pick_bar")
    hinge = object_model.get_articulation("cover_hinge")
    bar_joint = object_model.get_articulation("cover_to_bar")

    # --- hero parts present ---
    ctx.check("has_frame", frame is not None, "Expected a frame part.")
    ctx.check("has_cover", cover is not None, "Expected a cover part.")
    ctx.check("has_pin", pin is not None, "Expected a hinge pin part.")
    ctx.check("has_pick_bar", pick_bar is not None, "Expected a pick_bar part.")

    # --- cover hinge: type and axis ---
    ctx.check(
        "cover_revolute",
        str(hinge.articulation_type).endswith("REVOLUTE"),
        f"type={hinge.articulation_type}",
    )
    ctx.check(
        "cover_axis_y",
        abs(abs(hinge.axis[1]) - 1.0) < 1e-6 and abs(hinge.axis[0]) < 1e-6 and abs(hinge.axis[2]) < 1e-6,
        f"axis={hinge.axis}",
    )

    # --- pick-bar joint: type and axis ---
    ctx.check(
        "bar_revolute",
        str(bar_joint.articulation_type).endswith("REVOLUTE"),
        f"type={bar_joint.articulation_type}",
    )
    ctx.check(
        "bar_axis_y",
        abs(abs(bar_joint.axis[1]) - 1.0) < 1e-6 and abs(bar_joint.axis[0]) < 1e-6 and abs(bar_joint.axis[2]) < 1e-6,
        f"axis={bar_joint.axis}",
    )

    # --- bar joint limits ---
    ctx.check(
        "bar_lower_zero",
        abs(bar_joint.motion_limits.lower) < 1e-6,
        f"lower={bar_joint.motion_limits.lower}",
    )
    ctx.check(
        "bar_upper_upright",
        1.2 <= bar_joint.motion_limits.upper <= 1.8,
        f"upper={bar_joint.motion_limits.upper}",
    )

    # --- frame on the ground ---
    fa = ctx.part_world_aabb(frame)
    if fa is not None:
        ctx.check("frame_on_ground", abs(fa[0][2]) < 0.012, f"min_z={fa[0][2]:.4f}")
        size = tuple(fa[1][i] - fa[0][i] for i in range(3))
        ctx.check("frame_diameter", 0.68 <= max(size[0], size[1]) <= 0.80, f"size={size!r}")

    # --- cover scale and round footprint ---
    ca = ctx.part_world_aabb(cover)
    if ca is not None:
        csize = tuple(ca[1][i] - ca[0][i] for i in range(3))
        ctx.check("cover_diameter", 0.58 <= max(csize[0], csize[1]) <= 0.66, f"size={csize!r}")
        ctx.check(
            "cover_flush_top",
            abs(ca[1][2] - FRAME_H) < 0.03,
            f"cover_top={ca[1][2]:.3f}, frame_h={FRAME_H}",
        )

    # --- closed cover seats inside the frame ---
    ctx.expect_within(cover, frame, axes="xy", margin=0.02, name="cover_within_frame")
    ctx.allow_overlap(
        cover,
        frame,
        reason="The cover drops onto the recessed frame seat ledge with a small seated overlap.",
    )
    ctx.expect_contact(cover, frame, contact_tol=0.012, name="cover_seated_on_ledge")

    # --- pin captured inside cover knuckles ---
    ctx.allow_overlap(
        cover,
        pin,
        reason="The hinge pin is captured inside the cover knuckle bores.",
    )
    ctx.expect_contact(cover, pin, contact_tol=0.006, name="cover_knuckle_on_pin")

    # --- pick-bar stowed: within cover footprint, close to cover surface ---
    ctx.expect_within(pick_bar, cover, axes="xy", margin=0.02, name="bar_within_cover_footprint")
    # The pick-bar pivot tab nests between the cast lugs inside the recessed
    # channel; allow the small local overlap at that interface.
    ctx.allow_overlap(
        pick_bar,
        cover,
        elem_a="bar",
        elem_b="cover_plate",
        reason="The pick-bar pivot tab nests between the cast pivot lugs inside the recessed channel on the lid.",
    )
    ctx.expect_contact(pick_bar, cover, contact_tol=0.012, name="bar_seated_in_channel")

    # --- pick-bar swings up: deployed pose lifts the free end well above the lid ---
    bar_stowed_top = ctx.part_world_aabb(pick_bar)[1][2] if ctx.part_world_aabb(pick_bar) else None
    with ctx.pose({bar_joint: 1.4}):
        bar_deployed = ctx.part_world_aabb(pick_bar)
        if bar_stowed_top is not None and bar_deployed is not None:
            ctx.check(
                "bar_swings_up",
                bar_deployed[1][2] > bar_stowed_top + 0.10,
                f"stowed_top={bar_stowed_top:.3f}, deployed_top={bar_deployed[1][2]:.3f}",
            )
            ctx.check(
                "bar_deployed_above_cover",
                bar_deployed[1][2] > COVER_TOP_Z + 0.08,
                f"deployed_top={bar_deployed[1][2]:.3f}, cover_top={COVER_TOP_Z:.3f}",
            )

    # --- cover opens upward (hinge still works with bar attached) ---
    closed = ctx.part_world_aabb(cover)
    with ctx.pose({hinge: 1.6}):
        opened = ctx.part_world_aabb(cover)
        ctx.check(
            "open_clears_opening_center",
            opened[0][0] < -0.10 or opened[0][2] > FRAME_H + 0.05,
            f"open_minx={opened[0][0]:.3f}, open_minz={opened[0][2]:.3f}",
        )
    if closed is not None and opened is not None:
        ctx.check(
            "cover_opens_upward",
            opened[1][2] > closed[1][2] + 0.20,
            f"closed_top={closed[1][2]:.3f}, open_top={opened[1][2]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
