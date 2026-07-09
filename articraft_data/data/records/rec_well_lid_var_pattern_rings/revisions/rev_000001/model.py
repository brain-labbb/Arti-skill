from __future__ import annotations

# Round cast-iron manhole / well cover with concentric raised rings on top,
# seated in a circular cast-iron frame ring, hinged at one edge to swing open
# and reveal the shaft void below.
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
#   - cover (REVOLUTE about +Y at the rear edge): the round lid with concentric
#     raised cast ribs (rings) on top, a beveled rim, a smooth outer border,
#     a small plain center hub, and a pick-hole slot; it lifts up and back on
#     a barrel hinge to expose the shaft.
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

# --- concentric ring pattern ---
RING_RELIEF = 0.010       # height of each raised ring rib
RING_WIDTH = 0.018        # radial width of each ring rib
GROOVE_WIDTH = 0.012      # radial width of recessed groove between rings
RING_STEP = RING_WIDTH + GROOVE_WIDTH  # 0.030 pitch
HUB_R = 0.022             # center hub radius
HUB_HEIGHT = 0.014        # hub stands slightly proud of rings
N_RINGS = 8               # number of concentric raised rings
BORDER_INSET = 0.024      # smooth outer border ring width


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


def _build_annular_rib(inner_r: float, outer_r: float, height: float, name: str):
    """Shared geometry helper: a cast annular rib (raised ring).

    Returns a managed mesh for an annular ring standing on z=0, extruded upward
    by `height`, with a slight cast-iron fillet on the top edges.
    """
    rib = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(height)
    )
    fillet_r = min(0.003, height * 0.30, (outer_r - inner_r) * 0.18)
    if fillet_r > 0.0008:
        rib = rib.edges(">Z").fillet(fillet_r)
    return mesh_from_cadquery(rib, name, unit_scale=1.0)


def _build_cover_plate_mesh():
    """Round lid base plate with beveled rim, pick-hole and hinge knuckles.

    Authored in the cover-local frame: z=0 at the cover bottom, plate centered
    on the XY origin. No top pattern — rings are added as separate visuals.
    """
    # Base plate.
    plate = (
        cq.Workplane("XY")
        .circle(COVER_R)
        .extrude(COVER_T)
        .edges(">Z")
        .fillet(0.006)  # rounded top rim
    )

    # Pick-hole slot near the front edge for lifting.
    pick = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .center(COVER_R - 0.060, 0.0)
        .slot2D(0.060, 0.024, 0.0)
        .extrude(COVER_T + RING_RELIEF + 0.01)
    )
    plate = plate.cut(pick)

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
        plate = plate.union(knuckle.cut(bore))

    return mesh_from_cadquery(plate, "manhole_cover", unit_scale=1.0)


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
        _build_cover_plate_mesh(),
        origin=Origin(xyz=(COVER_R + 0.004, 0.0, -COVER_T * 0.5)),
        material=iron,
        name="cover_plate",
    )

    # --- concentric raised rings on the cover top ---
    # Each ring is authored in the mesh frame with z=0 at bottom, extruded up.
    # The visual origin places it on top of the plate in the cover part frame.
    ring_origin = Origin(xyz=(COVER_R + 0.004, 0.0, COVER_T * 0.5))
    for i in range(N_RINGS):
        inner_r = HUB_R + GROOVE_WIDTH + i * RING_STEP
        outer_r = inner_r + RING_WIDTH
        cover.visual(
            _build_annular_rib(inner_r, outer_r, RING_RELIEF, f"ring_{i}"),
            origin=ring_origin,
            material=iron,
            name=f"ring_{i}",
        )

    # --- plain center hub (slightly taller than the rings) ---
    hub_solid = (
        cq.Workplane("XY")
        .circle(HUB_R)
        .extrude(HUB_HEIGHT)
        .edges(">Z")
        .fillet(0.004)
    )
    cover.visual(
        mesh_from_cadquery(hub_solid, "center_hub", unit_scale=1.0),
        origin=ring_origin,
        material=iron_dark,
        name="center_hub",
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

    # --- concentric ring pattern ---
    # All N_RINGS ring visuals and the center hub must exist on the cover.
    cover_visual_names = {v.name for v in cover.visuals}
    for i in range(N_RINGS):
        ctx.check(
            f"has_ring_{i}",
            f"ring_{i}" in cover_visual_names,
            f"ring_{i} visual not found on cover",
        )
    ctx.check(
        "has_center_hub",
        "center_hub" in cover_visual_names,
        "center_hub visual not found on cover",
    )

    # Outermost ring must fit within the cover plate footprint (concentricity proxy).
    ctx.expect_within(
        cover, cover,
        axes="xy",
        inner_elem=f"ring_{N_RINGS - 1}",
        outer_elem="cover_plate",
        margin=0.005,
        name="outermost_ring_within_plate",
    )
    # Innermost ring must enclose the hub center in XY.
    ctx.expect_within(
        cover, cover,
        axes="xy",
        inner_elem="center_hub",
        outer_elem="ring_0",
        margin=0.005,
        name="hub_within_innermost_ring",
    )

    # Rings sit above the plate top surface (small positive gap in Z).
    ctx.expect_gap(
        cover, cover,
        axis="z",
        positive_elem="ring_0",
        negative_elem="cover_plate",
        min_gap=-0.002,
        max_gap=RING_RELIEF + 0.003,
        name="ring_sits_on_plate_top",
    )

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
