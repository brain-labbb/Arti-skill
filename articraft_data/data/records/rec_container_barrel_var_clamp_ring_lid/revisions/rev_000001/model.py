from __future__ import annotations

# Blue plastic storage drum (~30 L) with a stepped, waisted body.
# Fork of the open-top drum: the low-quality clamp-ring/lever hardware has
# been removed.  The variant now changes the barrel body itself: broad raised
# ribs, a narrower middle waist, darker recessed vertical panels, and a smaller
# rolled top rim.  The lift-off lid remains the single real joint.
#
# Frame: vertical drum axis along +Z, base at z=0, centerline at x=y=0.
#
# Articulation:
#   - lid_lift: PRISMATIC lift-off disc (+Z, 0..0.05 m)

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tessellate_cadquery,
)

# ---- key dimensions (meters) --------------------------------------------------
BARREL_H = 0.35
BASE_R = 0.136
WAIST_R = 0.112
TOP_R = 0.108
BARREL_R = BASE_R
RIM_Z = BARREL_H
LID_Z = RIM_Z - 0.002
PANEL_COUNT = 8


def _profile_loft(sections) -> cq.Workplane:
    wp = cq.Workplane("XY")
    prev = 0.0
    for i, (z, r) in enumerate(sections):
        off = z if i == 0 else z - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(r)
        prev = z
    return wp.loft(ruled=False)


def _cq_geom(solid: cq.Workplane) -> MeshGeometry:
    verts, faces = tessellate_cadquery(solid)
    geom = MeshGeometry()
    for vx, vy, vz in verts:
        geom.add_vertex(vx, vy, vz)
    for a, b, c in faces:
        geom.add_face(a, b, c)
    return geom


def _barrel_solid() -> cq.Workplane:
    """Open-top stepped drum body with a visible middle waist."""
    outer = _profile_loft(
        [
            (0.000, 0.124),
            (0.018, BASE_R),
            (0.055, 0.126),
            (0.088, 0.139),  # lower raised belt
            (0.122, 0.121),
            (0.172, WAIST_R),  # recessed waist
            (0.215, 0.123),
            (0.248, 0.138),  # upper raised belt
            (0.300, 0.120),
            (0.330, 0.114),
            (RIM_Z, TOP_R),
        ]
    )
    shell = outer.faces(">Z").shell(-0.012)

    # Wide raised reinforcement belts.  These are deliberately more varied
    # than the parent drum's even ring stack.
    rib_specs = [
        (0.032, 0.137, 0.010),
        (0.090, 0.142, 0.014),
        (0.145, 0.119, 0.007),
        (0.205, 0.119, 0.007),
        (0.250, 0.141, 0.014),
        (0.305, 0.121, 0.009),
    ]
    for z, r, h in rib_specs:
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(r)
            .circle(max(0.030, r - 0.026))
            .extrude(h)
        )
        shell = shell.union(ring)

    # Rolled top rim lip for the lift-off lid.
    lip = (
        cq.Workplane("XY")
        .workplane(offset=RIM_Z - 0.014)
        .circle(TOP_R + 0.007)
        .circle(TOP_R - 0.011)
        .extrude(0.016)
    )
    shell = shell.union(lip)
    return shell


def _lid_mesh():
    lid_outer = TOP_R + 0.010
    disc = cq.Workplane("XY").circle(lid_outer).extrude(0.010)
    lip = (
        cq.Workplane("XY")
        .workplane(offset=-0.008)
        .circle(TOP_R + 0.002)
        .circle(TOP_R - 0.007)
        .extrude(0.010)
    )
    lid = disc.union(lip)
    return mesh_from_cadquery(lid, "lid_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="storage_drum_stepped_waisted_body")

    blue = model.material("drum_blue", rgba=(0.15, 0.34, 0.70, 1.0))
    dark_blue = model.material("recessed_panel_blue", rgba=(0.08, 0.20, 0.46, 1.0))
    black = model.material("lid_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("barrel_body")
    body.visual(
        mesh_from_cadquery(_barrel_solid(), "barrel_shell"),
        material=blue,
        name="barrel_shell",
    )

    # Dark recessed-looking vertical panels make the surface language visibly
    # different from the parent and from the conical tapered sample.
    panel_geom = BoxGeometry((0.010, 0.004, 0.150))
    panel_r = WAIST_R + 0.004
    for i in range(PANEL_COUNT):
        a = 2.0 * math.pi * i / PANEL_COUNT
        body.visual(
            mesh_from_geometry(panel_geom, f"recessed_panel_{i}_mesh"),
            material=dark_blue,
            name=f"recessed_panel_{i}",
            origin=Origin(
                xyz=(math.cos(a) * panel_r, math.sin(a) * panel_r, 0.175),
                rpy=(0.0, 0.0, a),
            ),
        )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BARREL_R, length=BARREL_H),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, BARREL_H / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=black, name="lid_shell")
    boss_geom = CylinderGeometry(0.024, 0.012).translate(0.0, 0.0, 0.006)
    lid.visual(
        mesh_from_geometry(boss_geom, "lid_boss"),
        material=black,
        name="lid_boss",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_R + 0.010, length=0.03),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.2, lower=0.0, upper=0.05),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _radius_band(vertices, z_min: float, z_max: float) -> list[float]:
    return [math.hypot(x, y) for x, y, z in vertices if z_min <= z <= z_max]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("barrel_body")
    lid = object_model.get_part("lid")
    lid_lift = object_model.get_articulation("lid_lift")

    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "barrel remains taller than wide",
        bext[2] > bext[0] + 0.04 and bext[2] > bext[1] + 0.04,
        details=f"barrel extents={bext}",
    )

    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits on top of the barrel",
        lid_pos is not None and lid_pos[2] > 0.30,
        details=f"lid origin={lid_pos}",
    )
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="barrel_shell",
        reason="Lid locating lip seats inside the rolled rim lip for registration.",
    )

    lz0 = ctx.part_world_position(lid)[2]
    with ctx.pose({lid_lift: 0.05}):
        lifted = ctx.part_world_position(lid)
    ctx.check(
        "lid_lift remains the real motion",
        lifted[2] > lz0 + 0.045
        and abs(lifted[0] - lid_pos[0]) < 1e-4
        and abs(lifted[1] - lid_pos[1]) < 1e-4,
        details=f"rest_z={lz0}, lifted={lifted}",
    )

    geom = _cq_geom(_barrel_solid())
    waist = _radius_band(geom.vertices, 0.160, 0.185)
    lower_belt = _radius_band(geom.vertices, 0.085, 0.105)
    upper_belt = _radius_band(geom.vertices, 0.245, 0.265)
    ctx.check(
        "body has a visible narrow waist between raised belts",
        bool(waist)
        and bool(lower_belt)
        and bool(upper_belt)
        and max(lower_belt) > max(waist) + 0.020
        and max(upper_belt) > max(waist) + 0.020,
        details=(
            f"waist={max(waist) if waist else None}, "
            f"lower={max(lower_belt) if lower_belt else None}, "
            f"upper={max(upper_belt) if upper_belt else None}"
        ),
    )

    for i in range(PANEL_COUNT):
        ctx.check(
            f"body has recessed_panel_{i}",
            body.get_visual(f"recessed_panel_{i}") is not None,
        )

    return ctx.report()


object_model = build_object_model()
