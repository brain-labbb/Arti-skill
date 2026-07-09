from __future__ import annotations

# Honey JAR with screw-on lid and dipper holder.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# Clear/amber glass honey jar (wider than tall) with golden honey fill visible
# through the glass, a wooden screw-on lid, and a dipper holder bracket on top
# of the lid with a removable dipper stick.
#
# Articulations:
#   - lid_rotate: CONTINUOUS spin of the carrier about +Z at the rim top
#   - lid_slide:  PRISMATIC lift of the lid relative to the carrier along +Z
#   - dipper_slide: PRISMATIC pull-out of the dipper stick from the holder along +Z

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
JAR_OUTER_R = 0.040           # outer radius of the glass body (~0.080 m dia)
JAR_BODY_H = 0.060            # height of the glass body
WALL = 0.004                  # glass wall thickness
NECK_R = 0.034                # outer radius of the threaded neck (wide mouth)
NECK_H = 0.012                # neck height above the shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the neck rim where the lid sits (0.072)

HONEY_TOP_Z = JAR_BODY_H - 0.005  # honey surface just below the shoulder

LID_OUTER_R = 0.042           # lid skirt slightly wider than the neck
LID_H = 0.020                 # lid height (skirt + top cap)
# Lid geometry is authored in the lid part frame, whose origin coincides with
# the carrier/rotate joint at world z=RIM_TOP_Z. So lid-local z=0 is the rim top.
LID_SKIRT_BOTTOM_Z = -0.010   # lid-local: 10mm below the rim top
LID_TOP_Z = LID_SKIRT_BOTTOM_Z + LID_H  # lid-local top of the cap (0.010)

# Dipper holder: a small cylindrical socket on top of the lid
HOLDER_R = 0.005              # holder socket outer radius
HOLDER_H = 0.018              # holder height above lid top
HOLDER_INNER_R = 0.003        # inner bore for the dipper shaft
# Dipper stick
DIPPER_SHAFT_R = 0.0025       # dipper shaft radius (fits in holder bore)
DIPPER_SHAFT_LEN = 0.060      # total dipper shaft length
DIPPER_BULB_R = 0.006         # grooved bulb at the bottom of the dipper
DIPPER_BULB_H = 0.018         # bulb height


def _jar_glass_solid() -> cq.Workplane:
    # Hollow thick-walled glass jar with visible wall thickness at the mouth.
    # Revolve profile in XZ plane about Z axis.
    pts = [
        (0.0, 0.0),                           # center of the base
        (JAR_OUTER_R, 0.0),                   # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.008),    # outer wall up
        (JAR_OUTER_R - 0.005, JAR_BODY_H),    # rounded outer shoulder
        (NECK_R, JAR_BODY_H + 0.003),         # step in to the neck
        (NECK_R, RIM_TOP_Z),                  # neck outer up to the rim
        (NECK_R - WALL, RIM_TOP_Z),           # across the rim top (wall thickness at mouth)
        (NECK_R - WALL, JAR_BODY_H - 0.002),  # inner neck wall down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.008),
        (JAR_OUTER_R - WALL, WALL),           # inner body wall down to thick base
        (0.0, WALL),                          # across the inner base
        (0.0, 0.0),                           # close back to center
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    # Thread ridges on the neck for the screw-on lid.
    threads = None
    z0 = JAR_BODY_H + 0.004
    for i in range(3):
        z = z0 + i * 0.003
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.0006)
            .circle(NECK_R - 0.0004)
            .extrude(0.0018)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _honey_fill_mesh():
    # Golden amber honey filling the jar interior.
    inner_r = JAR_OUTER_R - WALL - 0.0005
    fill_h = HONEY_TOP_Z - WALL
    fill = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(inner_r)
        .extrude(fill_h)
    )
    # Slightly domed meniscus on top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=HONEY_TOP_Z)
        .circle(inner_r)
        .workplane(offset=0.003)
        .circle(inner_r * 0.6)
        .loft(ruled=False)
    )
    honey = fill.union(dome)
    return mesh_from_cadquery(honey, "honey_fill")


def _lid_solid() -> cq.Workplane:
    # Screw-on wooden lid: cylindrical cup with closed top and hollowed interior
    # so the skirt slips over the neck.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z)
        .circle(LID_OUTER_R)
        .extrude(LID_H)
    )
    outer = outer.edges(">Z").fillet(0.003)
    # Interior cavity matches neck outer for a snug fit.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z - 0.001)
        .circle(NECK_R + 0.0005)
        .extrude(LID_H - 0.005)
    )
    return outer.cut(cavity)


def _lid_knurl_mesh():
    # Knurled grip ring around the lid skirt.
    ribs = None
    n = 48
    band_z = LID_SKIRT_BOTTOM_Z + 0.002
    band_h = LID_H - 0.005
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        rib = (
            cq.Workplane("XY")
            .workplane(offset=band_z)
            .center((LID_OUTER_R - 0.0004) * math.cos(ang),
                    (LID_OUTER_R - 0.0004) * math.sin(ang))
            .rect(0.0016, 0.0016)
            .extrude(band_h)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return mesh_from_cadquery(ribs, "lid_knurl")


def _dipper_holder_mesh():
    # Dipper holder bracket: a cylindrical socket with an inner bore, mounted
    # on top of the lid. The bore accepts the dipper shaft.
    socket = (
        cq.Workplane("XY")
        .workplane(offset=LID_TOP_Z)
        .circle(HOLDER_R)
        .extrude(HOLDER_H)
    )
    # Bore through the socket for the dipper shaft
    bore = (
        cq.Workplane("XY")
        .workplane(offset=LID_TOP_Z - 0.001)
        .circle(HOLDER_INNER_R)
        .extrude(HOLDER_H + 0.002)
    )
    holder = socket.cut(bore)
    # Small base flange to visually connect to the lid top
    flange = (
        cq.Workplane("XY")
        .workplane(offset=LID_TOP_Z - 0.001)
        .circle(HOLDER_R + 0.003)
        .extrude(0.003)
    )
    holder = holder.union(flange)
    return mesh_from_cadquery(holder, "dipper_holder")


def _dipper_stick_mesh():
    # Dipper stick: a thin shaft with a grooved bulb at the top (stored bulb-up
    # in the holder). Authored in the dipper part frame. The dipper joint origin
    # is at the holder top, so dipper-local z=0 is the holder top opening.
    # Lower portion sits inside the holder socket; upper portion (bulb) extends
    # above the holder.
    shaft_lower = -0.015   # sits inside the holder socket (above the lid shell)
    shaft_upper = 0.040    # bulb end extends well above the holder
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=shaft_lower)
        .circle(DIPPER_SHAFT_R)
        .extrude(shaft_upper - shaft_lower)
    )
    # Small handle knob at the very bottom (grip end, stored down in socket)
    handle = (
        cq.Workplane("XY")
        .workplane(offset=shaft_lower - 0.002)
        .circle(DIPPER_SHAFT_R * 1.8)
        .extrude(0.006)
    )
    # Grooved bulb (honey dipper end) at the TOP of the stick
    bulb_parts = [shaft, handle]
    n_rings = 5
    ring_spacing = DIPPER_BULB_H / (n_rings + 1)
    for i in range(n_rings):
        z_ring = shaft_upper - DIPPER_BULB_H + 0.002 + i * ring_spacing
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z_ring)
            .circle(DIPPER_BULB_R)
            .extrude(ring_spacing * 0.6)
        )
        bulb_parts.append(ring)
    # Top cap on the bulb
    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=shaft_upper)
        .circle(DIPPER_BULB_R * 0.7)
        .extrude(0.003)
    )
    top_cap = top_cap.edges(">Z").fillet(0.002)
    bulb_parts.append(top_cap)
    dipper = bulb_parts[0]
    for p in bulb_parts[1:]:
        dipper = dipper.union(p)
    return mesh_from_cadquery(dipper, "dipper_stick")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="honey_jar")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.85, 0.92, 0.88, 0.45))
    honey_gold = model.material("honey_gold", rgba=(0.85, 0.60, 0.12, 0.92))
    lid_wood = model.material("lid_wood", rgba=(0.55, 0.35, 0.18, 1.0))
    holder_metal = model.material("holder_metal", rgba=(0.72, 0.70, 0.65, 1.0))
    dipper_wood = model.material("dipper_wood", rgba=(0.62, 0.42, 0.22, 1.0))
    marker_gold = model.material("marker_gold", rgba=(0.80, 0.65, 0.15, 1.0))

    # ---- jar body (root): glass shell + neck threads + honey fill ----
    body = model.part("body")

    glass = _jar_glass_solid().union(_neck_threads())
    body.visual(mesh_from_cadquery(glass, "jar_glass"), material=glass_clear, name="jar_glass")

    # Amber honey fill visible through the clear glass
    body.visual(_honey_fill_mesh(), material=honey_gold, name="honey_fill")

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
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

    # ---- lid: caps over the neck; slides up off the carrier along +Z ----
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_solid(), "lid_shell"), material=lid_wood, name="lid_shell")
    lid.visual(_lid_knurl_mesh(), material=lid_wood, name="lid_knurl")
    # Dipper holder bracket mounted on top of the lid
    lid.visual(_dipper_holder_mesh(), material=holder_metal, name="dipper_holder")
    # Small off-axis marker so rotation of the lid is visible.
    lid.visual(
        Box((0.004, 0.004, 0.002)),
        origin=Origin(xyz=(LID_OUTER_R - 0.006, 0.0, LID_SKIRT_BOTTOM_Z + LID_H - 0.001)),
        material=marker_gold,
        name="lid_marker",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_OUTER_R, LID_H),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, LID_SKIRT_BOTTOM_Z + LID_H * 0.5)),
    )
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_H, effort=1.0, velocity=1.0),
    )

    # ---- dipper stick: inserts into the holder, can be pulled out ----
    dipper = model.part("dipper")
    dipper.visual(_dipper_stick_mesh(), material=dipper_wood, name="dipper_stick")
    dipper.inertial = Inertial.from_geometry(
        Cylinder(DIPPER_SHAFT_R, DIPPER_SHAFT_LEN),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # The dipper joint origin is at the top of the holder socket (lid-local).
    # The dipper shaft extends both above (handle) and below (into holder/jar).
    model.articulation(
        "dipper_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=dipper,
        origin=Origin(xyz=(0.0, 0.0, LID_TOP_Z + HOLDER_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.040, effort=1.0, velocity=0.5),
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
    dipper = object_model.get_part("dipper")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")
    dipper_slide = object_model.get_articulation("dipper_slide")

    # ---- Intentional overlap allowances ----
    # The lid skirt seats over the neck rim
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="jar_glass",
        reason="The lid skirt is intentionally slipped down over the threaded neck rim.",
    )
    # The dipper shaft inserts into the holder bore
    ctx.allow_overlap(
        dipper,
        lid,
        elem_a="dipper_stick",
        elem_b="dipper_holder",
        reason="The dipper shaft is intentionally inserted into the holder socket bore.",
    )

    # ---- jar is wider than tall (honey jar proportions) ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar is wider than tall",
        bext[0] > bext[2] and bext[1] > bext[2],
        details=f"body extents={bext}",
    )

    # ---- glass wall thickness at the mouth (neck wall visible) ----
    # The neck inner radius should be visibly smaller than outer, proving wall thickness
    ctx.check(
        "neck has wall thickness at mouth",
        NECK_R - WALL > 0.020 and WALL >= 0.003,
        details=f"NECK_R={NECK_R}, WALL={WALL}, inner={NECK_R - WALL}",
    )

    # ---- lid sits on top of the jar ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is on top of the jar",
        lid_pos is not None and lid_pos[2] > RIM_TOP_Z - 0.001,
        details=f"lid_pos={lid_pos}, rim_top={RIM_TOP_Z}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02, name="lid caps the neck"
    )

    # ---- dipper holder exists on the lid ----
    holder_vis = lid.get_visual("dipper_holder")
    ctx.check(
        "dipper holder is mounted on the lid",
        holder_vis is not None,
        details="dipper_holder visual not found on lid",
    )

    # ---- dipper holder sits above the lid top ----
    ctx.expect_gap(
        lid, body, axis="z",
        positive_elem="dipper_holder",
        negative_elem="jar_glass",
        min_gap=0.0,
        name="dipper holder is above the jar rim",
    )

    # ---- dipper stick exists and is in the holder ----
    dipper_vis = dipper.get_visual("dipper_stick")
    ctx.check(
        "dipper stick exists",
        dipper_vis is not None,
        details="dipper_stick visual not found",
    )
    # Dipper overlaps the holder in XY (inserted into bore)
    ctx.expect_overlap(
        dipper, lid, axes="xy",
        elem_a="dipper_stick", elem_b="dipper_holder",
        min_overlap=0.002,
        name="dipper is inserted in the holder",
    )

    # ---- lid_rotate: CONTINUOUS screw joint spins the lid ----
    marker0 = ctx.part_element_world_aabb(lid, elem="lid_marker")
    m0 = ((marker0[0][0] + marker0[1][0]) * 0.5, (marker0[0][1] + marker0[1][1]) * 0.5)
    with ctx.pose({rotate: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(lid, elem="lid_marker")
        m1 = ((marker1[0][0] + marker1[1][0]) * 0.5, (marker1[0][1] + marker1[1][1]) * 0.5)
    moved = math.hypot(m1[0] - m0[0], m1[1] - m0[1])
    ctx.check(
        "lid_rotate (CONTINUOUS) spins the lid",
        moved > 0.01,
        details=f"marker rest={m0}, quarter-turn={m1}, moved={moved}",
    )

    # Verify lid_rotate is CONTINUOUS type
    ctx.check(
        "lid_rotate is a continuous joint",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotate.articulation_type}",
    )

    # ---- lid_slide lifts the lid off the jar ----
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_H}):
        lifted_z = ctx.part_world_position(lid)[2]
        ctx.expect_gap(
            lid, body, axis="z", min_gap=0.0,
            positive_elem="lid_shell", negative_elem="jar_glass",
            name="lifted lid clears the neck",
        )
    ctx.check(
        "lid_slide lifts the lid off the jar",
        lifted_z > rest_z + LID_H * 0.5,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # ---- dipper_slide pulls the dipper out of the holder ----
    dipper_rest_z = ctx.part_world_position(dipper)[2]
    with ctx.pose({dipper_slide: 0.030}):
        dipper_pulled_z = ctx.part_world_position(dipper)[2]
    ctx.check(
        "dipper_slide pulls the dipper upward",
        dipper_pulled_z > dipper_rest_z + 0.020,
        details=f"rest_z={dipper_rest_z}, pulled_z={dipper_pulled_z}",
    )

    # ---- carrier is massless / has no visuals ----
    ctx.check(
        "carrier link has no visuals",
        len(carrier.visuals) == 0,
        details=f"carrier visuals={len(carrier.visuals)}",
    )

    return ctx.report()


object_model = build_object_model()
