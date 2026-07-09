from __future__ import annotations

# Cosmetic JAR with hinged flip-top lid and clamp bail.
# Frame: jar axis along +Z, base at z=0, centered at (0,0).
# Squat glass jar with wide mouth, visible hollow opening,
# hinged flip-top lid at rear, and U-shaped wire clamp bail.
#
# Articulations:
#   - lid_hinge: REVOLUTE, lid pivots at rear rim, opens upward
#   - bail_pivot: REVOLUTE, bail pivots on side lugs, swings forward to open

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
    tube_from_spline_points,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
JAR_OUTER_R = 0.036           # outer radius (~72mm dia)
JAR_BODY_H = 0.044            # body height
WALL = 0.0045                 # glass wall thickness
MOUTH_R = JAR_OUTER_R - WALL - 0.001  # wide mouth opening radius
RIM_H = 0.005                 # short rim above body
RIM_TOP_Z = JAR_BODY_H + RIM_H  # top of rim (0.049)

GEL_TOP_Z = JAR_BODY_H - 0.003  # gel surface inside

# Lid dimensions
LID_R = MOUTH_R + 0.0015      # lid radius (slightly larger than mouth)
LID_THICK = 0.004             # lid thickness

# Bail dimensions
BAIL_WIRE_R = 0.0018          # wire radius
BAIL_ARCH_H = 0.028           # arch height above pivot
BAIL_PIVOT_Z = JAR_BODY_H * 0.78  # bail pivot height on body

# Hinge dimensions
HINGE_LUG_W = 0.008           # hinge lug width
HINGE_LUG_H = 0.006           # hinge lug height
HINGE_LUG_D = 0.005           # hinge lug depth
HINGE_Y = -(JAR_OUTER_R - 0.003)  # rear of jar for hinge


def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled glass jar with wide mouth opening."""
    # Revolved profile in XZ plane about Z axis
    pts = [
        (0.0, 0.0),                          # center of base
        (JAR_OUTER_R, 0.0),                  # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.006),   # outer wall up
        (JAR_OUTER_R - 0.004, JAR_BODY_H),   # rounded shoulder
        (JAR_OUTER_R, JAR_BODY_H + 0.001),   # step out to rim base
        (JAR_OUTER_R, RIM_TOP_Z),            # rim outer up
        (MOUTH_R, RIM_TOP_Z),                # across rim top (wide mouth)
        (MOUTH_R, JAR_BODY_H - 0.002),       # inner neck wall down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.006),
        (JAR_OUTER_R - WALL, WALL),          # inner body wall down
        (0.0, WALL),                         # across inner base
        (0.0, 0.0),                          # close back to center
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _hinge_lugs() -> cq.Workplane:
    """Two small lugs at the rear for the lid hinge pin."""
    # Single centered lug at rear - positioned below the rim top
    lug = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - HINGE_LUG_H)
        .center(0, HINGE_Y)
        .rect(HINGE_LUG_W, HINGE_LUG_D)
        .extrude(HINGE_LUG_H)
    )
    return lug


def _bail_lugs() -> cq.Workplane:
    """Two small lugs on the sides for the bail pivot."""
    lug_offset_y = 0.002  # slightly outward from jar body
    lug_h = 0.006
    lug_w = 0.005
    lug_d = 0.004
    
    # Left lug
    lug_l = (
        cq.Workplane("XY")
        .workplane(offset=BAIL_PIVOT_Z - lug_h / 2)
        .center(-(JAR_OUTER_R + lug_offset_y), 0)
        .rect(lug_w, lug_d)
        .extrude(lug_h)
    )
    # Right lug
    lug_r = (
        cq.Workplane("XY")
        .workplane(offset=BAIL_PIVOT_Z - lug_h / 2)
        .center(JAR_OUTER_R + lug_offset_y, 0)
        .rect(lug_w, lug_d)
        .extrude(lug_h)
    )
    return lug_l.union(lug_r)


def _gel_surface_mesh():
    """Cream/gel filling visible through the wide mouth."""
    # Make the gel disc radius match the inner wall to ensure contact
    inner_r = JAR_OUTER_R - WALL - 0.0002  # slightly smaller than inner wall
    # Disc filling
    disc = (
        cq.Workplane("XY")
        .workplane(offset=GEL_TOP_Z - 0.010)
        .circle(inner_r)
        .extrude(0.010)
    )
    # Slight dome on top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=GEL_TOP_Z)
        .circle(inner_r)
        .workplane(offset=0.003)
        .circle(inner_r * 0.6)
        .loft(ruled=False)
    )
    gel = disc.union(dome)
    return mesh_from_cadquery(gel, "gel_surface")


def _lid_shell() -> cq.Workplane:
    """Flat disc lid with integrated hinge barrel."""
    lid = (
        cq.Workplane("XY")
        .circle(LID_R)
        .extrude(LID_THICK)
    )
    
    # Hinge barrel at the rear edge (y=0 edge, along X axis)
    barrel_r = 0.0025
    barrel_len = HINGE_LUG_W - 0.002
    barrel = (
        cq.Workplane("YZ")
        .workplane(offset=-barrel_len / 2)
        .circle(barrel_r)
        .extrude(barrel_len)
    )
    # Position barrel at the rear edge of the lid, centered on the disc thickness
    barrel = barrel.translate((-LID_R + barrel_r, 0, LID_THICK / 2))
    
    return lid.union(barrel)


def _bail_wire_mesh():
    """U-shaped wire bail using tube sweep."""
    # Bail arch from left pivot to right pivot
    # In bail local frame: origin at pivot center, bail arches upward
    half_span = JAR_OUTER_R + 0.003
    
    # Spline points for the arch (in XZ plane at y=0)
    # Start at left pivot, arch up and over, end at right pivot
    points = [
        (-half_span, 0.0, 0.0),              # left pivot
        (-half_span, 0.0, BAIL_ARCH_H * 0.5),  # left leg up
        (-half_span * 0.7, 0.0, BAIL_ARCH_H * 0.9),  # arch rising
        (0.0, 0.0, BAIL_ARCH_H),             # arch peak
        (half_span * 0.7, 0.0, BAIL_ARCH_H * 0.9),   # arch falling
        (half_span, 0.0, BAIL_ARCH_H * 0.5),   # right leg down
        (half_span, 0.0, 0.0),               # right pivot
    ]
    
    bail_tube = tube_from_spline_points(
        points,
        radius=BAIL_WIRE_R,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),  # keep the frame stable
    )
    
    return mesh_from_geometry(bail_tube, "bail_wire")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_top_jar")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.55, 0.75, 0.85, 0.50))
    gel_cream = model.material("gel_cream", rgba=(0.98, 0.96, 0.92, 1.0))
    lid_white = model.material("lid_white", rgba=(0.95, 0.95, 0.97, 1.0))
    wire_silver = model.material("wire_silver", rgba=(0.72, 0.75, 0.78, 1.0))

    # ---- jar body (root): glass shell + hinge lugs + bail lugs + gel ----
    body = model.part("body")

    # Glass jar with wide mouth
    glass = _jar_glass_solid().union(_hinge_lugs()).union(_bail_lugs())
    body.visual(mesh_from_cadquery(glass, "jar_glass"), material=glass_clear, name="jar_glass")

    # Gel/cream surface inside (visible through mouth)
    body.visual(_gel_surface_mesh(), material=gel_cream, name="gel_surface")

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- lid: flip-top disc hinged at rear ----
    lid = model.part("lid")
    
    # Lid disc with integrated hinge barrel
    # Lid part frame origin at the hinge line (rear of jar at rim height)
    # In lid local frame: hinge is at origin, lid extends in +Y direction
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell"),
        origin=Origin(xyz=(0.0, LID_R, 0.0)),  # offset so hinge edge is at origin
        material=lid_white,
        name="lid_shell",
    )

    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, LID_THICK),
        mass=0.025,
        origin=Origin(xyz=(0.0, LID_R, LID_THICK / 2)),
    )

    # Lid hinge articulation: REVOLUTE at rear rim
    # Hinge at (0, HINGE_Y, RIM_TOP_Z) in world
    # Lid extends in +Y from hinge, so axis=(1,0,0) makes positive q open upward
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, RIM_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=2.3,  # ~132 degrees open
        ),
    )

    # ---- bail: U-shaped wire clamp ----
    bail = model.part("bail")
    
    # Bail wire - arches upward from pivot
    bail.visual(
        _bail_wire_mesh(),
        material=wire_silver,
        name="bail_wire",
    )

    bail.inertial = Inertial.from_geometry(
        Box((JAR_OUTER_R * 2, 0.01, BAIL_ARCH_H)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, BAIL_ARCH_H / 2)),
    )

    # Bail pivot articulation: REVOLUTE on side lugs
    # Pivot at (0, 0, BAIL_PIVOT_Z) - midpoint between side lugs
    # Bail arches upward (+Z), so axis=(-1,0,0) makes positive q swing forward (+Y)
    model.articulation(
        "bail_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, BAIL_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=0.0,
            upper=2.5,  # ~143 degrees open
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    bail = object_model.get_part("bail")
    lid_hinge = object_model.get_articulation("lid_hinge")
    bail_pivot = object_model.get_articulation("bail_pivot")

    # ---- jar has wide mouth (hollow opening) ----
    # The mouth radius should be close to the jar outer radius (not a narrow neck)
    ctx.check(
        "jar has wide mouth opening",
        MOUTH_R > JAR_OUTER_R * 0.8,
        details=f"mouth_r={MOUTH_R}, jar_r={JAR_OUTER_R}",
    )

    # ---- lid covers the mouth at rest ----
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02,
        name="lid covers the jar mouth at rest",
    )

    # ---- lid hinge opens the lid upward ----
    lid_rest_aabb = ctx.part_world_aabb(lid)
    lid_rest_z = (lid_rest_aabb[0][2] + lid_rest_aabb[1][2]) / 2
    with ctx.pose({lid_hinge: 1.2}):  # ~69 degrees open
        lid_open_aabb = ctx.part_world_aabb(lid)
        lid_open_z = (lid_open_aabb[0][2] + lid_open_aabb[1][2]) / 2
        # Lid center should move upward when opened
        ctx.check(
            "lid_hinge opens lid upward",
            lid_open_z > lid_rest_z + 0.01,
            details=f"rest_z={lid_rest_z}, open_z={lid_open_z}",
        )
        # Lid should clear the jar mouth (allow small contact at hinge)
        ctx.expect_gap(
            lid, body, axis="z", max_penetration=0.001,
            positive_elem="lid_shell", negative_elem="jar_glass",
            name="opened lid mostly clears the jar mouth",
        )

    # ---- bail arches over the jar when closed ----
    bail_pos = ctx.part_world_position(bail)
    ctx.check(
        "bail is positioned on the jar",
        bail_pos[2] > JAR_BODY_H * 0.5 and bail_pos[2] < RIM_TOP_Z + BAIL_ARCH_H,
        details=f"bail_z={bail_pos[2]}",
    )

    # ---- bail pivot swings the bail forward ----
    bail_rest_aabb = ctx.part_world_aabb(bail)
    bail_rest_y = (bail_rest_aabb[0][1] + bail_rest_aabb[1][1]) / 2
    with ctx.pose({bail_pivot: 1.5}):  # ~86 degrees open
        bail_open_aabb = ctx.part_world_aabb(bail)
        bail_open_y = (bail_open_aabb[0][1] + bail_open_aabb[1][1]) / 2
        # Bail should swing forward (positive Y direction)
        ctx.check(
            "bail_pivot swings bail forward",
            bail_open_y > bail_rest_y + 0.01,
            details=f"rest_y={bail_rest_y}, open_y={bail_open_y}",
        )

    # ---- lid hinge has correct limits ----
    lid_limits = lid_hinge.motion_limits
    ctx.check(
        "lid_hinge has positive opening range",
        lid_limits.upper > 1.5 and lid_limits.lower >= 0.0,
        details=f"lower={lid_limits.lower}, upper={lid_limits.upper}",
    )

    # ---- bail pivot has correct limits ----
    bail_limits = bail_pivot.motion_limits
    ctx.check(
        "bail_pivot has positive opening range",
        bail_limits.upper > 1.5 and bail_limits.lower >= 0.0,
        details=f"lower={bail_limits.lower}, upper={bail_limits.upper}",
    )

    # ---- gel surface is visible inside the jar ----
    gel_aabb = ctx.part_element_world_aabb(body, elem="gel_surface")
    ctx.check(
        "gel surface exists inside the jar",
        gel_aabb is not None and gel_aabb[0][2] > 0 and gel_aabb[1][2] < RIM_TOP_Z,
        details=f"gel_z_range=[{gel_aabb[0][2]}, {gel_aabb[1][2]}]",
    )

    return ctx.report()


object_model = build_object_model()
