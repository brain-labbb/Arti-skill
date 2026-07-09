from __future__ import annotations

# Round cast-iron manhole / well cover with a radial sunburst spoke pattern on
# top, seated in a circular cast-iron frame ring, hinged at one edge to swing
# open and reveal the shaft void below.
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
#   - cover (REVOLUTE about +Y at the rear edge): the round lid with raised
#     radial spokes fanning from a central medallion hub to a smooth outer
#     border ring, a beveled rim, and a pick-hole slot; it lifts up and back on
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
RIB_RELIEF = 0.010  # raised spoke rib height

# Radial spoke pattern
N_SPOKES = 18
HUB_R = 0.040  # central medallion hub radius
HUB_EXTRA_H = 0.004  # hub is taller than spokes for medallion emphasis
BORDER_OUTER_R = COVER_R - 0.018  # smooth outer border ring outer radius
BORDER_INNER_R = BORDER_OUTER_R - 0.020  # border ring width ~20mm
SPOKE_OUTER_R = BORDER_INNER_R  # spokes connect directly to the border ring
SPOKE_W_INNER = 0.014  # spoke width at the hub end
SPOKE_W_OUTER = 0.022  # spoke width at the outer end
SPOKE_ANGLE_OFFSET = math.pi / N_SPOKES  # half-step offset to keep pick-hole clear

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
    # Upper opening: frame inner radius from the seat up to the top.
    upper_bore = (
        cq.Workplane("XY")
        .workplane(offset=SEAT_Z)
        .circle(FRAME_INNER_R)
        .extrude(FRAME_H - SEAT_Z + 0.002)
    )
    ring = ring.cut(upper_bore)
    # Shaft void below the seat: collar inner radius down to a thin cast floor.
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
    """Round lid base plate with beveled rim and pick-hole (no top pattern).

    Authored in the cover-local frame: z=0 at the cover bottom, plate centered
    on the XY origin. The hinge frame later places the rear edge on the pin.
    The raised radial pattern is authored as separate visuals on the cover part.
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
        .extrude(COVER_T + RIB_RELIEF + 0.01)
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


def _build_radial_rib_mesh():
    """Shared radial-rib geometry helper: one raised spoke at angle 0 along +X.

    Trapezoidal plan view — narrower at the hub, wider at the outer end —
    extruded from z=0 to z=RIB_RELIEF, centered on y=0.
    """
    half_wi = SPOKE_W_INNER / 2.0
    half_wo = SPOKE_W_OUTER / 2.0
    spoke = (
        cq.Workplane("XY")
        .moveTo(HUB_R, -half_wi)
        .lineTo(SPOKE_OUTER_R, -half_wo)
        .lineTo(SPOKE_OUTER_R, half_wo)
        .lineTo(HUB_R, half_wi)
        .close()
        .extrude(RIB_RELIEF)
    )
    return mesh_from_cadquery(spoke, "radial_rib", unit_scale=1.0)


def _build_hub_mesh():
    """Central medallion hub disk, slightly taller than the spoke ribs."""
    hub = (
        cq.Workplane("XY")
        .circle(HUB_R)
        .extrude(RIB_RELIEF + HUB_EXTRA_H)
        .edges(">Z")
        .fillet(0.004)
    )
    return mesh_from_cadquery(hub, "medallion_hub", unit_scale=1.0)


def _build_border_ring_mesh():
    """Smooth raised annular border ring near the cover rim."""
    outer = (
        cq.Workplane("XY")
        .circle(BORDER_OUTER_R)
        .extrude(RIB_RELIEF)
    )
    inner = (
        cq.Workplane("XY")
        .circle(BORDER_INNER_R)
        .extrude(RIB_RELIEF + 0.002)
    )
    ring = outer.cut(inner)
    return mesh_from_cadquery(ring, "border_ring", unit_scale=1.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manhole_well_cover")

    iron = model.material("cast_iron", rgba=(0.22, 0.21, 0.20, 1.0))
    iron_dark = model.material("cast_iron_dark", rgba=(0.16, 0.15, 0.15, 1.0))
    void_black = model.material("shaft_void", rgba=(0.04, 0.04, 0.05, 1.0))
    pin_steel = model.material("hinge_pin", rgba=(0.45, 0.45, 0.47, 1.0))
    rib_iron = model.material("rib_iron", rgba=(0.25, 0.24, 0.22, 1.0))

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

    # Base plate: cover bottom at local z=0, plate centered on XY; the plate
    # center is +COVER_R forward of the hinge pin.
    plate_origin = Origin(xyz=(COVER_R + 0.004, 0.0, -COVER_T * 0.5))
    cover.visual(
        _build_cover_mesh(),
        origin=plate_origin,
        material=iron,
        name="cover_plate",
    )

    # Top pattern mounting frame: on top of the plate, centered on the plate.
    top_z = COVER_T * 0.5  # z of plate top in cover part frame
    top_center = (COVER_R + 0.004, 0.0, top_z)

    # Central medallion hub
    cover.visual(
        _build_hub_mesh(),
        origin=Origin(xyz=top_center),
        material=rib_iron,
        name="hub",
    )

    # Smooth outer border ring
    cover.visual(
        _build_border_ring_mesh(),
        origin=Origin(xyz=top_center),
        material=rib_iron,
        name="border_ring",
    )

    # Radial spokes: emitted via for-loop with uniform angular step
    rib_mesh = _build_radial_rib_mesh()
    angle_step = 2.0 * math.pi / N_SPOKES
    for i in range(N_SPOKES):
        angle = SPOKE_ANGLE_OFFSET + i * angle_step
        cover.visual(
            rib_mesh,
            origin=Origin(xyz=top_center, rpy=(0.0, 0.0, angle)),
            material=rib_iron,
            name=f"spoke_{i}",
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
        abs(abs(hinge.axis[1]) - 1.0) < 1e-6
        and abs(hinge.axis[0]) < 1e-6
        and abs(hinge.axis[2]) < 1e-6,
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
    # The cover rests on the frame seat ledge: allow that small seated overlap.
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

    # --- radial spoke pattern checks ---
    # All spoke_i visuals exist on the cover part
    spoke_count = 0
    for i in range(N_SPOKES):
        v = cover.get_visual(f"spoke_{i}")
        if v is not None:
            spoke_count += 1
    ctx.check(
        "spoke_count",
        spoke_count == N_SPOKES,
        f"Expected {N_SPOKES} radial spokes, found {spoke_count}.",
    )

    # Central medallion hub exists
    ctx.check(
        "has_hub",
        cover.get_visual("hub") is not None,
        "Expected a central medallion hub visual on the cover.",
    )

    # Smooth outer border ring exists
    ctx.check(
        "has_border_ring",
        cover.get_visual("border_ring") is not None,
        "Expected a smooth outer border ring visual on the cover.",
    )

    # Verify uniform angular spacing of the spoke visuals
    yaws = []
    for i in range(N_SPOKES):
        v = cover.get_visual(f"spoke_{i}")
        if v is not None:
            yaws.append(v.origin.rpy[2])
    if len(yaws) == N_SPOKES:
        expected_step = 2.0 * math.pi / N_SPOKES
        diffs = [
            (yaws[(i + 1) % N_SPOKES] - yaws[i]) % (2.0 * math.pi)
            for i in range(N_SPOKES)
        ]
        max_deviation = max(abs(d - expected_step) for d in diffs)
        ctx.check(
            "uniform_spoke_spacing",
            max_deviation < 0.01,
            f"max yaw-step deviation from uniform {expected_step:.4f} rad: {max_deviation:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
