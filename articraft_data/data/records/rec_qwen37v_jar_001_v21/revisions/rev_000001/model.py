from __future__ import annotations

# Squat cosmetic cream jar with thick screw lid and clamp bail closure.
# Frame: jar axis along +Z, base resting on z=0, centered at origin.
#
# Structure:
#   body         - squat amber glass jar (wider than tall), wide mouth,
#                  hollow interior, thread ridges, cream fill, metal bail
#                  collar, two side pivot pins, brand label
#   lid_carrier  - massless carrier for screw rotation (decoupled from lift)
#   lid          - thick gold screw-on lid with knurled grip and marker
#   bail         - U-shaped wire clamp bail that pivots on side hinges
#
# Articulations:
#   lid_rotate   - CONTINUOUS spin of carrier about +Z at the rim top
#   lid_slide    - PRISMATIC lift of lid along +Z off the carrier
#   bail_pivot   - REVOLUTE swing of bail about +Y (horizontal axis through
#                  the two side pivot pins); positive q opens the bail

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
JAR_OUTER_R = 0.040            # 80 mm diameter squat body
JAR_BODY_H = 0.038             # short body height
WALL = 0.004                   # thick glass wall
NECK_R = 0.034                 # wide mouth neck (68 mm outer)
NECK_H = 0.012                 # neck height above shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # 0.050

GEL_TOP_Z = JAR_BODY_H - 0.010   # cream surface well below rim (0.028)

LID_OUTER_R = 0.038            # lid skirt slightly wider than neck
LID_H = 0.022                  # thick lid
LID_SKIRT_BOTTOM_Z = -0.009    # lid-local: skirt dips below the rim top
LID_TOP_Z = LID_SKIRT_BOTTOM_Z + LID_H  # 0.013

# Bail collar and pivot geometry
COLLAR_BOTTOM_Z = JAR_BODY_H + 0.002   # 0.040 (sits on the shoulder)
COLLAR_H = 0.006                        # collar height
COLLAR_OR = NECK_R + 0.005             # 0.039 collar outer radius
COLLAR_IR = NECK_R + 0.001             # 0.035 collar inner radius
BAIL_PIVOT_Z = COLLAR_BOTTOM_Z + COLLAR_H * 0.5  # 0.043 (mid-collar)
BAIL_ARM_Y = COLLAR_OR + 0.003         # 0.042 arm offset (outside collar)
BAIL_ARM_LEN = (RIM_TOP_Z + LID_TOP_Z) - BAIL_PIVOT_Z  # 0.020
BAIL_WIRE_R = 0.0015                   # 3 mm wire diameter
BAIL_UPPER = 1.05                      # ~60 deg opening limit


# ---------- geometry builders ----------

def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled squat jar with wide mouth opening.

    Revolved half-profile in the XZ plane about the Z axis.  The profile
    traces the outer wall up through a rounded shoulder into the wide neck,
    across the rim top, down the inner neck wall, and back to the base.
    """
    pts = [
        (0.0, 0.0),                                    # center base
        (JAR_OUTER_R, 0.0),                            # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.004),             # outer wall up
        (JAR_OUTER_R - 0.003, JAR_BODY_H),             # rounded outer shoulder
        (NECK_R, JAR_BODY_H + 0.003),                  # step in to the neck
        (NECK_R, RIM_TOP_Z),                           # neck outer up to rim
        (NECK_R - WALL, RIM_TOP_Z),                    # across rim top
        (NECK_R - WALL, JAR_BODY_H + 0.003),           # inner neck wall down
        (NECK_R - WALL + 0.003, JAR_BODY_H - 0.002),  # inner shoulder
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.004),     # inner body wall top
        (JAR_OUTER_R - WALL, WALL),                    # inner body wall down
        (0.0, WALL),                                   # inner base
        (0.0, 0.0),                                    # close
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    """Thread ridges on the neck exterior (three stacked rings)."""
    threads = None
    z0 = JAR_BODY_H + 0.005
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


def _bail_collar() -> cq.Workplane:
    """Metal ring around the neck that carries the bail pivot pins."""
    return (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_BOTTOM_Z)
        .circle(COLLAR_OR)
        .circle(COLLAR_IR)
        .extrude(COLLAR_H)
    )


def _cream_surface_mesh():
    """White cream filling visible through the wide mouth.

    Fills the jar interior from the inner floor to just below the rim so the
    cream contacts the glass on all interior surfaces.
    """
    inner_r = JAR_OUTER_R - WALL  # match inner wall radius for contact
    fill_bottom = WALL            # inner floor of the jar
    fill_height = GEL_TOP_Z - fill_bottom
    disc = (
        cq.Workplane("XY")
        .workplane(offset=fill_bottom)
        .circle(inner_r)
        .extrude(fill_height)
    )
    dome = (
        cq.Workplane("XY")
        .workplane(offset=GEL_TOP_Z)
        .circle(inner_r)
        .workplane(offset=0.003)
        .circle(inner_r * 0.55)
        .loft(ruled=False)
    )
    cream = disc.union(dome)
    return mesh_from_cadquery(cream, "cream_surface")


def _label_band():
    """Thin label ring wrapped around the body mid-height."""
    return (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H * 0.5 - 0.008)
        .circle(JAR_OUTER_R + 0.0005)
        .circle(JAR_OUTER_R - 0.0005)
        .extrude(0.016)
    )


def _lid_solid() -> cq.Workplane:
    """Thick screw-on lid: crowned cup with hollowed skirt."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z)
        .circle(LID_OUTER_R)
        .extrude(LID_H)
    )
    outer = outer.edges(">Z").fillet(0.003)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z - 0.001)
        .circle(NECK_R)
        .extrude(LID_H - 0.005)
    )
    return outer.cut(cavity)


def _lid_knurl_mesh():
    """Knurled grip ribs around the lid skirt."""
    ribs = None
    n = 48
    band_z = LID_SKIRT_BOTTOM_Z + 0.002
    band_h = LID_H - 0.005
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        rib = (
            cq.Workplane("XY")
            .workplane(offset=band_z)
            .center(
                (LID_OUTER_R - 0.0004) * math.cos(ang),
                (LID_OUTER_R - 0.0004) * math.sin(ang),
            )
            .rect(0.0016, 0.0016)
            .extrude(band_h)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return mesh_from_cadquery(ribs, "lid_knurl")


def _bail_mesh():
    """U-shaped wire bail: two vertical arms + crossbar at top."""
    wr = BAIL_WIRE_R
    ay = BAIL_ARM_Y
    al = BAIL_ARM_LEN

    # Left arm (+Y side): cylinder along Z from z=0 to z=al
    left = (
        cq.Workplane("XY")
        .center(0, ay)
        .circle(wr)
        .extrude(al)
    )
    # Right arm (-Y side)
    right = (
        cq.Workplane("XY")
        .center(0, -ay)
        .circle(wr)
        .extrude(al)
    )
    # Crossbar: cylinder along Y at z=al, spanning y from -ay to +ay
    crossbar = (
        cq.Workplane("XZ")
        .workplane(offset=-ay)
        .center(0, al)
        .circle(wr)
        .extrude(ay * 2)
    )
    bail = left.union(right).union(crossbar)
    return mesh_from_cadquery(bail, "bail_wire")


# ---------- model assembly ----------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bail_clamp_cream_jar")

    # Materials
    glass_amber = model.material("glass_amber", rgba=(0.62, 0.44, 0.22, 0.50))
    cream_white = model.material("cream_white", rgba=(0.97, 0.95, 0.90, 1.0))
    lid_gold = model.material("lid_gold", rgba=(0.83, 0.69, 0.35, 1.0))
    metal_silver = model.material("metal_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    label_beige = model.material("label_beige", rgba=(0.94, 0.88, 0.76, 1.0))
    marker_dark = model.material("marker_dark", rgba=(0.12, 0.10, 0.08, 1.0))

    # ---- body (root): glass jar + threads + cream + collar + pins + label ----
    body = model.part("body")

    glass = _jar_glass_solid().union(_neck_threads())
    body.visual(
        mesh_from_cadquery(glass, "jar_glass"),
        material=glass_amber, name="jar_glass",
    )

    # Cream fill visible through the wide mouth
    body.visual(_cream_surface_mesh(), material=cream_white, name="cream_surface")

    # Metal bail collar around the neck
    body.visual(
        mesh_from_cadquery(_bail_collar(), "bail_collar"),
        material=metal_silver, name="bail_collar",
    )

    # Two pivot pins protruding from the collar (side hinge points)
    pin_len = 0.010
    pin_r = 0.0025
    pin_cy = COLLAR_OR + 0.002  # pin centre Y offset from jar axis
    for sign, name in [(+1, "pivot_pin_left"), (-1, "pivot_pin_right")]:
        body.visual(
            Cylinder(pin_r, pin_len),
            origin=Origin(
                xyz=(0.0, sign * pin_cy, BAIL_PIVOT_Z),
                rpy=(-math.pi / 2, 0, 0),  # align local +Z with world Y
            ),
            material=metal_silver,
            name=name,
        )

    # Brand label band on the body
    body.visual(
        mesh_from_cadquery(_label_band(), "brand_label"),
        material=label_beige, name="brand_label",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H + NECK_H),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, (JAR_BODY_H + NECK_H) * 0.5)),
    )

    # ---- massless carrier: rotates about +Z at the rim top ----
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

    # ---- lid: thick screw cap, slides up off carrier along +Z ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=lid_gold, name="lid_shell",
    )
    lid.visual(_lid_knurl_mesh(), material=lid_gold, name="lid_knurl")
    # Off-axis marker so lid rotation is visible
    lid.visual(
        Box((0.004, 0.004, 0.002)),
        origin=Origin(xyz=(LID_OUTER_R - 0.006, 0.0, LID_TOP_Z - 0.001)),
        material=marker_dark,
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
        motion_limits=MotionLimits(
            lower=0.0, upper=LID_H, effort=1.0, velocity=1.0,
        ),
    )

    # ---- bail: U-shaped wire clamp that pivots on the side pins ----
    bail = model.part("bail")
    bail.visual(_bail_mesh(), material=metal_silver, name="bail_wire")
    bail.inertial = Inertial.from_geometry(
        Box((0.004, BAIL_ARM_Y * 2, BAIL_ARM_LEN)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, BAIL_ARM_LEN * 0.5)),
    )
    # REVOLUTE about +Y: the axis connects the two side pivot pins.
    # At q=0 the bail arms point up (+Z) and the crossbar clamps the lid.
    # Positive q swings the bail open (crossbar moves toward +X and down).
    model.articulation(
        "bail_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, BAIL_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=BAIL_UPPER, effort=3.0, velocity=2.0,
        ),
    )

    return model


# ---------- helpers ----------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


# ---------- tests ----------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("lid_carrier")
    lid = object_model.get_part("lid")
    bail = object_model.get_part("bail")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")
    bail_joint = object_model.get_articulation("bail_pivot")

    # ---- intentional overlaps ----

    # Lid skirt slips over the threaded neck
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_shell", elem_b="jar_glass",
        reason="The lid skirt is intentionally slipped over the threaded neck rim.",
    )

    # Bail crossbar clamps down on the lid top at q=0
    ctx.allow_overlap(
        bail, lid,
        elem_a="bail_wire", elem_b="lid_shell",
        reason="The bail crossbar clamps down on the lid top when closed.",
    )

    # Bail arm bottoms wrap around the pivot pins
    ctx.allow_overlap(
        bail, body,
        elem_a="bail_wire", elem_b="pivot_pin_left",
        reason="The bail arm wraps around the left pivot pin at the hinge.",
    )
    ctx.allow_overlap(
        bail, body,
        elem_a="bail_wire", elem_b="pivot_pin_right",
        reason="The bail arm wraps around the right pivot pin at the hinge.",
    )

    # ---- jar is squat (wider than tall) ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar is squat (wider than tall)",
        bext[0] > bext[2] + 0.005 and bext[1] > bext[2] + 0.005,
        details=f"body extents={bext}",
    )

    # ---- wide mouth: neck inner diameter >= 50 mm ----
    mouth_dia = 2 * (NECK_R - WALL)
    ctx.check(
        "wide mouth opening (>=50 mm)",
        mouth_dia >= 0.050,
        details=f"neck inner dia={mouth_dia:.3f}",
    )

    # ---- lid sits on top of the jar ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is on top of the jar",
        lid_pos is not None and lid_pos[2] > RIM_TOP_Z - 0.001,
        details=f"lid_pos={lid_pos}, rim_top={RIM_TOP_Z}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="lid caps the neck",
    )

    # ---- bail exists and has a revolute joint ----
    ctx.check(
        "bail joint is revolute",
        bail_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={bail_joint.articulation_type}",
    )
    # Bail joint axis is along Y (horizontal, connecting the two side pivots)
    ctx.check(
        "bail axis is horizontal (Y)",
        abs(bail_joint.axis[1]) > 0.9 and abs(bail_joint.axis[0]) < 0.1 and abs(bail_joint.axis[2]) < 0.1,
        details=f"axis={bail_joint.axis}",
    )

    # ---- bail closed pose: crossbar above the lid ----
    bail_pos = ctx.part_world_position(bail)
    ctx.check(
        "bail origin at pivot height",
        bail_pos is not None and abs(bail_pos[2] - BAIL_PIVOT_Z) < 0.002,
        details=f"bail_pos={bail_pos}, pivot_z={BAIL_PIVOT_Z}",
    )

    # ---- bail opens: crossbar moves away from lid centre ----
    crossbar0 = ctx.part_element_world_aabb(bail, elem="bail_wire")
    cx0 = (crossbar0[0][0] + crossbar0[1][0]) * 0.5
    cz0 = crossbar0[1][2]  # top of bail AABB (crossbar height)
    with ctx.pose({bail_joint: BAIL_UPPER}):
        crossbar1 = ctx.part_element_world_aabb(bail, elem="bail_wire")
        cx1 = (crossbar1[0][0] + crossbar1[1][0]) * 0.5
        cz1 = crossbar1[1][2]
    ctx.check(
        "bail_pivot opens the bail (crossbar moves)",
        abs(cx1 - cx0) > 0.005 or abs(cz1 - cz0) > 0.005,
        details=f"closed=({cx0:.4f},{cz0:.4f}), open=({cx1:.4f},{cz1:.4f})",
    )

    # ---- lid_rotate spins the lid ----
    marker0 = ctx.part_element_world_aabb(lid, elem="lid_marker")
    m0 = (
        (marker0[0][0] + marker0[1][0]) * 0.5,
        (marker0[0][1] + marker0[1][1]) * 0.5,
    )
    with ctx.pose({rotate: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(lid, elem="lid_marker")
        m1 = (
            (marker1[0][0] + marker1[1][0]) * 0.5,
            (marker1[0][1] + marker1[1][1]) * 0.5,
        )
    moved = math.hypot(m1[0] - m0[0], m1[1] - m0[1])
    ctx.check(
        "lid_rotate spins the lid (marker moves)",
        moved > 0.01,
        details=f"marker rest={m0}, quarter-turn={m1}, moved={moved}",
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

    # ---- carrier is massless / has no visuals ----
    ctx.check(
        "carrier link has no visuals",
        len(carrier.visuals) == 0,
        details=f"carrier visuals={len(carrier.visuals)}",
    )

    # ---- two pivot pins exist on the body ----
    pin_names = [v.name for v in body.visuals if v.name and "pivot_pin" in v.name]
    ctx.check(
        "two side pivot pins (hinge points)",
        len(pin_names) == 2,
        details=f"pins={pin_names}",
    )

    return ctx.report()


object_model = build_object_model()
