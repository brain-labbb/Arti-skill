from __future__ import annotations

# Round cast-iron manhole / well cover with a waffle-grid top, seated in a
# circular cast-iron frame ring, hinged at one edge to swing open and reveal the
# shaft void below.
#
# Coordinate convention:
#   - up is +Z; the frame ring sits on the ground at z=0.
#   - the cover "front" (the pick-hole / lifting edge) faces +X; the hinge is at
#     the rear edge (-X).
#
# Structure:
#   - frame (root, static): a heavy cast-iron frame ring set in the ground, with
#     a recessed seating ledge for the cover, a downward collar forming the
#     visible shaft wall, and a hollow cylindrical shaft void inside.
#   - cover (REVOLUTE about +Y at the rear edge): the round lid with a raised
#     square waffle grid on top, a beveled rim, and a pick-hole slot; it lifts
#     up and back on a barrel hinge to expose the shaft.
#   - hinge_pin (FIXED to the frame): the visible exposed hinge knuckle/pin at
#     the rear edge.

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
COVER_T = 0.042  # cover plate thickness
GRID_RELIEF = 0.010  # raised waffle pattern height
GRID_PITCH = 0.052  # square cell pitch
GRID_GROOVE = 0.012  # groove width between raised squares

FRAME_OUTER_R = 0.370
FRAME_INNER_R = COVER_R + 0.006  # the cover drops into this opening
FRAME_H = 0.110  # total frame ring height (curb above ground)
SEAT_DROP = 0.028  # how far the cover seat is recessed below the frame top
COLLAR_R = COVER_R - 0.030  # inner shaft wall radius
SHAFT_FLOOR_Z = 0.016  # thin cast floor left at the bottom of the shaft

# Cover seat top sits SEAT_DROP below the frame top.
SEAT_Z = FRAME_H - SEAT_DROP  # z of the seating ledge
COVER_BOTTOM_Z = SEAT_Z  # cover rests here
COVER_TOP_Z = COVER_BOTTOM_Z + COVER_T

# Hinge at the rear edge, at the cover mid-thickness height.
HINGE_X = -COVER_R - 0.004
HINGE_Z = COVER_BOTTOM_Z + COVER_T * 0.5


def _build_frame_mesh():
    """Cast-iron frame ring with a recessed cover seat and a shaft void."""
    # Outer curb ring.
    ring = (
        cq.Workplane("XY")
        .circle(FRAME_OUTER_R)
        .extrude(FRAME_H)
    )
    # Bore the central opening down to the seat, then a wider opening above the
    # seat (so the cover drops onto the ledge), leaving the shaft below.
    # Upper opening: frame inner radius from the seat up to the top.
    upper_bore = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z)
        .circle(FRAME_INNER_R)
        .extrude(FRAME_H - SEAT_Z + 0.002)
    )
    ring = ring.cut(upper_bore)
    # Shaft void below the seat: the collar inner radius down to a thin cast
    # floor (left at the bottom so the frame stays one connected solid).
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=SHAFT_FLOOR_Z)
        .circle(COLLAR_R)
        .extrude(SEAT_Z - SHAFT_FLOOR_Z + 0.002)
    )
    ring = ring.cut(shaft)
    # Bevel the top outer edge of the curb slightly.
    ring = ring.edges(">Z").fillet(0.006)

    # Hinge mounting lug at the rear (-X) edge of the frame top.
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
    """Round lid with a raised square waffle grid, beveled rim and pick-hole.

    Authored in the cover-local frame: z=0 at the cover bottom, plate centered
    on the XY origin. The hinge frame later places the rear edge on the pin.
    """
    # Base plate.
    plate = (
        cq.Workplane("XY")
        .circle(COVER_R)
        .extrude(COVER_T)
        .edges(">Z")
        .fillet(0.006)  # rounded top rim
    )
    # Raised waffle grid: start from a full raised slab on top, then cut a cross
    # grid of grooves into it so square pads remain.
    grid_slab = (
        cq.Workplane("XY")
        .workplane(offset=COVER_T)
        .circle(COVER_R - 0.024)  # leave a smooth outer border ring
        .extrude(GRID_RELIEF)
    )
    # Cut grooves along X and Y to leave a square pad pattern.
    n = int((2 * (COVER_R - 0.024)) // GRID_PITCH) + 2
    half = (n // 2) * GRID_PITCH
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

    # Pick-hole slot near the front edge for lifting.
    pick = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .center(COVER_R - 0.060, 0.0)
        .slot2D(0.060, 0.024, 0.0)
        .extrude(COVER_T + GRID_RELIEF + 0.01)
    )
    cover = cover.cut(pick)

    # Two hinge knuckles at the rear edge wrapping the pin.
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manhole_well_cover")

    iron = model.material("cast_iron", rgba=(0.22, 0.21, 0.20, 1.0))
    iron_dark = model.material("cast_iron_dark", rgba=(0.16, 0.15, 0.15, 1.0))
    void_black = model.material("shaft_void", rgba=(0.04, 0.04, 0.05, 1.0))
    pin_steel = model.material("hinge_pin", rgba=(0.45, 0.45, 0.47, 1.0))

    # --- frame (root) ---
    frame = model.part("frame")
    frame.visual(_build_frame_mesh(), material=iron, name="frame_ring")
    # A dark disk resting on the cast shaft floor reads as the void / darkness.
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

    # --- hinge pin (fixed to the frame, exposed knuckle/pin at the rear) ---
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
    # Cover mesh frame: cover bottom at local z=0, plate centered on XY; place it
    # so the rear edge knuckles sit on the hinge pin. The hinge frame origin is
    # at the pin; the cover plate center is +COVER_R forward of it and lowered to
    # the seat level.
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
        # Cover plate extends along local +X from the hinge; -Y lifts the free
        # front edge up and back as q increases.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    cover = object_model.get_part("cover")
    pin = object_model.get_part("hinge_pin")
    hinge = object_model.get_articulation("cover_hinge")

    # --- hero parts present ---
    ctx.check("has_frame", frame is not None, "Expected a frame part.")
    ctx.check("has_cover", cover is not None, "Expected a cover part.")
    ctx.check("has_pin", pin is not None, "Expected a hinge pin part.")

    # --- joint type/axis ---
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

    # --- frame sits on the ground at z approx 0 ---
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
        # Cover top should be near the frame top (flush-ish) when closed.
        ctx.check(
            "cover_flush_top",
            abs(ca[1][2] - FRAME_H) < 0.03,
            f"cover_top={ca[1][2]:.3f}, frame_h={FRAME_H}",
        )

    # --- closed cover seats inside the frame opening (footprint within frame) ---
    ctx.expect_within(cover, frame, axes="xy", margin=0.02, name="cover_within_frame")
    # The cover rests on the frame seat ledge: it should be in contact, with the
    # seat lip overlapping slightly. Allow that small seated overlap and prove it.
    ctx.allow_overlap(
        cover,
        frame,
        reason="The cover drops onto the recessed frame seat ledge with a small seated overlap.",
    )
    ctx.expect_contact(cover, frame, contact_tol=0.012, name="cover_seated_on_ledge")

    # --- pin is captured inside the cover knuckles ---
    ctx.allow_overlap(
        cover,
        pin,
        reason="The hinge pin is captured inside the cover knuckle bores.",
    )
    ctx.expect_contact(cover, pin, contact_tol=0.006, name="cover_knuckle_on_pin")

    # --- opening reveals the shaft: the cover lifts up and clears the opening ---
    closed = ctx.part_world_aabb(cover)
    with ctx.pose({hinge: 1.6}):
        opened = ctx.part_world_aabb(cover)
        # When open, the cover front edge has swung up and back, clearing the
        # frame opening center so the void is exposed.
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
