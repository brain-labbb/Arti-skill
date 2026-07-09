"""Cupola-style pressurised module end with hinged debris shutters.

Identity features from the reference picture:
- grey cylindrical module hull with panel seam ribs, narrowing through a
  conical adapter to a raised collar that carries the windowed cupola dome
- hexagonal-frustum cupola: six trapezoidal windows in white frames around
  a central circular top window with a silver bezel
- every window has a grid-textured debris shutter hinged at its outer edge,
  shown swung open outward like splayed petals
- gold hinge lugs and a ring of bolts around the dome base

Articulation: six trapezoidal window shutters plus the central round
shutter, each on its own revolute hinge (rest pose = open as pictured;
negative motion closes the shutter over its window).
"""

from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- layout ----
HULL_R = 1.85            # module hull outer radius
HULL_WALL = 0.15
HULL_TOP = 1.50          # hull tube top
CONE_TOP = 1.78          # adapter cone top
COLLAR_TOP = 1.95        # cupola support collar top

HEX_BASE_C = 1.38        # dome base hexagon circumradius
HEX_TOP_C = 0.82         # dome top hexagon circumradius
RB = HEX_BASE_C * math.cos(math.pi / 6.0)   # base apothem
RT = HEX_TOP_C * math.cos(math.pi / 6.0)    # top apothem
ZB = 2.02                # dome facet base height
DOME_H = 0.72
ZT = ZB + DOME_H         # dome facet top height
PHI = math.atan2(RB - RT, DOME_H)           # facet lean from vertical
SLANT = math.hypot(RB - RT, DOME_H)         # facet slant length
WB = HEX_BASE_C          # facet width at base (hex side length)
WT = HEX_TOP_C           # facet width at top

PLATE_T = 0.055
HINGE_N = 0.07           # hinge pin offset along facet normal
SHUTTER_OPEN = 2.0       # rest open angle (rad) of the six side shutters

TOP_PLATE_Z = ZT         # crown plate bottom
TOP_PLATE_T = 0.06
TOP_HOLE_R = 0.46
CENTRAL_HINGE_R = 0.62
CENTRAL_HINGE_Z = 2.855
CENTRAL_HINGE_A = math.radians(210.0)
CENTRAL_OPEN = 2.6       # rest open angle of the round crown shutter

FACET_ANGLES = tuple(math.radians(60.0 * i) for i in range(6))


def _u(a: float) -> tuple[float, float]:
    return (math.cos(a), math.sin(a))


def _facet_xyz(a: float, s: float, ty: float, n: float) -> tuple[float, float, float]:
    """World point from facet coordinates: s up-slope, ty tangential, n outward normal."""
    ux, uy = _u(a)
    tx, tyv = -math.sin(a), math.cos(a)
    # up-slope and normal unit vectors of the leaning facet
    ex = (-math.sin(PHI) * ux, -math.sin(PHI) * uy, math.cos(PHI))
    en = (math.cos(PHI) * ux, math.cos(PHI) * uy, math.sin(PHI))
    return (
        RB * ux + s * ex[0] + ty * tx + n * en[0],
        RB * uy + s * ex[1] + ty * tyv + n * en[1],
        ZB + s * ex[2] + n * en[2],
    )


def _facet_rpy(a: float) -> tuple[float, float, float]:
    """Orientation whose local x is down-slope, y tangential, z the facet normal."""
    return (0.0, math.pi / 2.0 - PHI, a)


def _hex_pts(circum: float) -> list[tuple[float, float]]:
    return [
        (circum * math.cos(math.radians(30.0 + 60.0 * i)),
         circum * math.sin(math.radians(30.0 + 60.0 * i)))
        for i in range(6)
    ]


def _trap(x0: float, w0: float, x1: float, w1: float, t: float) -> cq.Workplane:
    """Trapezoidal plate spanning x0..x1 with widths w0/w1, extruded 0..t in z."""
    pts = [(x0, -w0 / 2.0), (x0, w0 / 2.0), (x1, w1 / 2.0), (x1, -w1 / 2.0)]
    return cq.Workplane("XY").polyline(pts).close().extrude(t)


def _ring(ro: float, ri: float, h: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(ro).circle(ri).extrude(h)


def _panel_width(s: float) -> float:
    """Facet width at slant coordinate s."""
    return WB + (WT - WB) * (s / SLANT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cupola_pressurised_module_end")

    hull_grey = model.material("hull_grey", rgba=(0.70, 0.71, 0.72, 1.0))
    seam_grey = model.material("seam_grey", rgba=(0.55, 0.56, 0.57, 1.0))
    frame_white = model.material("frame_white", rgba=(0.86, 0.86, 0.84, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.09, 0.10, 0.12, 1.0))
    shutter_dark = model.material("shutter_dark", rgba=(0.27, 0.28, 0.30, 1.0))
    grid_silver = model.material("grid_silver", rgba=(0.62, 0.63, 0.65, 1.0))
    gold = model.material("gold_fitting", rgba=(0.76, 0.63, 0.30, 1.0))
    steel = model.material("steel_grey", rgba=(0.48, 0.49, 0.51, 1.0))
    bezel_silver = model.material("bezel_silver", rgba=(0.74, 0.75, 0.77, 1.0))

    # ---------------------------------------------------------- module hull --
    hull = model.part("module_hull")
    hull_tube = mesh_from_cadquery(_ring(HULL_R, HULL_R - HULL_WALL, HULL_TOP), "hull_tube")
    hull.visual(hull_tube, origin=Origin(xyz=(0.0, 0.0, 0.0)), material=hull_grey,
                name="hull_tube")
    hull.visual(
        Cylinder(radius=HULL_R, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=hull_grey,
        name="hull_end_cap",
    )
    # Circumferential seam bands.
    seam_mesh = mesh_from_cadquery(_ring(HULL_R + 0.02, HULL_R - 0.01, 0.05), "hull_seam")
    for i, sz in enumerate((0.45, 1.05)):
        hull.visual(seam_mesh, origin=Origin(xyz=(0.0, 0.0, sz)), material=seam_grey,
                    name=f"hull_seam_{i}")
    # Longitudinal panel seam ribs around the hull.
    for i in range(12):
        a = math.radians(30.0 * i + 15.0)
        ux, uy = _u(a)
        hull.visual(
            Box((0.05, 0.05, 1.30)),
            origin=Origin(xyz=(HULL_R * ux, HULL_R * uy, 0.75), rpy=(0.0, 0.0, a)),
            material=seam_grey,
            name=f"hull_rib_{i}",
        )
    # Conical adapter (hollow) up to the cupola collar.
    cone_outer = cq.Workplane("XY").circle(HULL_R).workplane(
        offset=CONE_TOP - HULL_TOP).circle(1.30).loft(combine=True)
    cone_inner = cq.Workplane("XY").circle(HULL_R - HULL_WALL).workplane(
        offset=CONE_TOP - HULL_TOP).circle(1.15).loft(combine=True)
    cone_mesh = mesh_from_cadquery(cone_outer.cut(cone_inner), "adapter_cone")
    hull.visual(cone_mesh, origin=Origin(xyz=(0.0, 0.0, HULL_TOP)), material=hull_grey,
                name="adapter_cone")
    collar_mesh = mesh_from_cadquery(_ring(1.30, 1.15, COLLAR_TOP - CONE_TOP), "cupola_collar")
    hull.visual(collar_mesh, origin=Origin(xyz=(0.0, 0.0, CONE_TOP)), material=seam_grey,
                name="cupola_collar")

    # ----------------------------------------------------------- cupola dome --
    dome = model.part("cupola_dome")

    # Hexagonal base ring seating on the collar.
    slab = cq.Workplane("XY").polyline(_hex_pts(1.42)).close().extrude(0.08)
    slab = slab.cut(cq.Workplane("XY").circle(1.05).extrude(0.3).translate((0, 0, -0.1)))
    slab_mesh = mesh_from_cadquery(slab, "dome_base_ring")
    dome.visual(slab_mesh, origin=Origin(xyz=(0.0, 0.0, 1.94)), material=frame_white,
                name="dome_base_ring")

    # One trapezoidal facet frame with a window cut, reused six times.
    hole_x = SLANT / 2.0 - 0.10
    hole_wb = _panel_width(0.10) - 0.24
    hole_wt = _panel_width(SLANT - 0.10) - 0.24
    frame_solid = _trap(SLANT / 2.0, WB, -SLANT / 2.0, WT, PLATE_T)
    hole_solid = _trap(hole_x, hole_wb, -hole_x, hole_wt, PLATE_T * 4.0).translate(
        (0.0, 0.0, -PLATE_T * 1.5))
    frame_mesh = mesh_from_cadquery(frame_solid.cut(hole_solid), "facet_frame")
    glass_mesh = mesh_from_cadquery(
        _trap(hole_x + 0.05, hole_wb + 0.10, -(hole_x + 0.05), hole_wt + 0.10, 0.02),
        "facet_glass",
    )
    for i, a in enumerate(FACET_ANGLES):
        centre = _facet_xyz(a, SLANT / 2.0, 0.0, -PLATE_T / 2.0)
        dome.visual(frame_mesh, origin=Origin(xyz=centre, rpy=_facet_rpy(a)),
                    material=frame_white, name=f"facet_frame_{i}")
        dome.visual(glass_mesh, origin=Origin(xyz=centre, rpy=_facet_rpy(a)),
                    material=glass_dark, name=f"window_glass_{i}")

    # Crown plate with the central round window.
    top_plate = cq.Workplane("XY").polyline(_hex_pts(0.86)).close().extrude(TOP_PLATE_T)
    top_plate = top_plate.cut(
        cq.Workplane("XY").circle(TOP_HOLE_R).extrude(0.3).translate((0, 0, -0.1)))
    dome.visual(mesh_from_cadquery(top_plate, "crown_plate"),
                origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_Z)), material=frame_white,
                name="crown_plate")
    dome.visual(
        Cylinder(radius=0.52, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_Z - 0.01)),
        material=glass_dark,
        name="central_window_glass",
    )
    bezel_mesh = mesh_from_cadquery(_ring(0.56, TOP_HOLE_R, 0.03), "central_bezel")
    dome.visual(bezel_mesh, origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_Z + TOP_PLATE_T)),
                material=bezel_silver, name="central_bezel")

    # Bolt ring around the dome base.
    for i in range(12):
        a = math.radians(15.0 + 30.0 * i)
        ux, uy = _u(a)
        dome.visual(
            Cylinder(radius=0.024, length=0.035),
            origin=Origin(xyz=(1.18 * ux, 1.18 * uy, 2.035)),
            material=steel,
            name=f"ring_bolt_{i}",
        )

    # Hinge pins + support blocks for the six side shutters.
    for i, a in enumerate(FACET_ANGLES):
        pin_pos = _facet_xyz(a, 0.0, 0.0, HINGE_N)
        dome.visual(
            Cylinder(radius=0.018, length=0.64),
            origin=Origin(xyz=pin_pos, rpy=(math.pi / 2.0, 0.0, a)),
            material=gold,
            name=f"shutter_pin_{i}",
        )
        for k, ty in enumerate((-0.28, 0.28)):
            bpos = _facet_xyz(a, 0.03, ty, 0.0312)
            dome.visual(
                Box((0.06, 0.06, 0.075)),
                origin=Origin(xyz=bpos, rpy=_facet_rpy(a)),
                material=gold,
                name=f"pin_block_{i}_{k}",
            )

    # Hinge pin + pedestal for the central round shutter.
    cux, cuy = _u(CENTRAL_HINGE_A)
    ctx_, cty = -math.sin(CENTRAL_HINGE_A), math.cos(CENTRAL_HINGE_A)
    dome.visual(
        Cylinder(radius=0.016, length=0.52),
        origin=Origin(
            xyz=(CENTRAL_HINGE_R * cux, CENTRAL_HINGE_R * cuy, CENTRAL_HINGE_Z),
            rpy=(math.pi / 2.0, 0.0, CENTRAL_HINGE_A),
        ),
        material=gold,
        name="central_pin",
    )
    for k, ty in enumerate((-0.22, 0.22)):
        dome.visual(
            Box((0.06, 0.06, 0.085)),
            origin=Origin(
                xyz=(CENTRAL_HINGE_R * cux + ty * ctx_,
                     CENTRAL_HINGE_R * cuy + ty * cty,
                     2.8125),
                rpy=(0.0, 0.0, CENTRAL_HINGE_A),
            ),
            material=gold,
            name=f"central_pin_block_{k}",
        )

    model.articulation(
        "hull_to_dome",
        ArticulationType.FIXED,
        parent=hull,
        child=dome,
    )

    # ------------------------------------------------- six petal shutters ----
    panel_x0, panel_x1 = -0.028, -(SLANT - 0.05)
    pw0 = _panel_width(0.04) - 0.10
    pw1 = _panel_width(SLANT - 0.05) - 0.06
    panel_mesh = mesh_from_cadquery(
        _trap(panel_x0, pw0, panel_x1, pw1, 0.03), "shutter_panel")
    rim_outer = _trap(panel_x0, pw0, panel_x1, pw1, 0.02)
    rim_inner = _trap(-0.12, _panel_width(0.12) - 0.26,
                      -(SLANT - 0.13), _panel_width(SLANT - 0.13) - 0.22,
                      0.08).translate((0.0, 0.0, -0.03))
    rim_mesh = mesh_from_cadquery(rim_outer.cut(rim_inner), "shutter_rim")

    for i, a in enumerate(FACET_ANGLES):
        shutter = model.part(f"window_shutter_{i}")
        shutter.visual(panel_mesh, origin=Origin(), material=shutter_dark,
                       name="shutter_panel")
        shutter.visual(rim_mesh, origin=Origin(xyz=(0.0, 0.0, 0.03)),
                       material=grid_silver, name="shutter_rim")
        # Grid texture: cross ribs with trapezoid-following lengths.
        for k in range(6):
            s_k = 0.10 + k * 0.12
            w_k = _panel_width(s_k) - 0.18
            shutter.visual(
                Box((0.02, w_k, 0.016)),
                origin=Origin(xyz=(-s_k, 0.0, 0.038)),
                material=grid_silver,
                name=f"grid_cross_{k}",
            )
        for j, gy in enumerate((-0.26, 0.0, 0.26)):
            shutter.visual(
                Box((0.62, 0.02, 0.016)),
                origin=Origin(xyz=(-0.42, gy, 0.038)),
                material=grid_silver,
                name=f"grid_long_{j}",
            )
        # Hinge lugs wrapping the dome-side pin.
        for k, ly in enumerate((-0.20, 0.20)):
            shutter.visual(
                Box((0.07, 0.05, 0.07)),
                origin=Origin(xyz=(0.0, ly, 0.0)),
                material=gold,
                name=f"hinge_lug_{k}",
            )
        model.articulation(
            f"window_shutter_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=dome,
            child=shutter,
            origin=Origin(
                xyz=_facet_xyz(a, 0.0, 0.0, HINGE_N),
                rpy=(0.0, math.pi / 2.0 - PHI + SHUTTER_OPEN, a),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0, velocity=1.5, lower=-SHUTTER_OPEN, upper=0.05),
        )

    # ---------------------------------------------- central round shutter ----
    central = model.part("central_shutter")
    central.visual(
        Cylinder(radius=0.55, length=0.03),
        origin=Origin(xyz=(-0.66, 0.0, 0.045)),
        material=shutter_dark,
        name="shutter_disc",
    )
    disc_rim = mesh_from_cadquery(_ring(0.55, 0.47, 0.015), "central_disc_rim")
    central.visual(disc_rim, origin=Origin(xyz=(-0.66, 0.0, 0.06)),
                   material=grid_silver, name="disc_rim")
    for k in range(5):
        xo = -0.36 + k * 0.18
        half = math.sqrt(max(0.55 ** 2 - xo ** 2, 0.01)) - 0.06
        central.visual(
            Box((0.02, 2.0 * half, 0.014)),
            origin=Origin(xyz=(-0.66 + xo, 0.0, 0.067)),
            material=grid_silver,
            name=f"disc_grid_cross_{k}",
        )
    for j, yo in enumerate((-0.27, 0.0, 0.27)):
        half = math.sqrt(max(0.55 ** 2 - yo ** 2, 0.01)) - 0.06
        central.visual(
            Box((2.0 * half, 0.02, 0.014)),
            origin=Origin(xyz=(-0.66, yo, 0.067)),
            material=grid_silver,
            name=f"disc_grid_long_{j}",
        )
    for k, ly in enumerate((-0.14, 0.14)):
        central.visual(
            Box((0.36, 0.06, 0.07)),
            origin=Origin(xyz=(-0.145, ly, 0.01)),
            material=gold,
            name=f"hinge_arm_{k}",
        )
    model.articulation(
        "central_shutter_hinge",
        ArticulationType.REVOLUTE,
        parent=dome,
        child=central,
        origin=Origin(
            xyz=(CENTRAL_HINGE_R * cux, CENTRAL_HINGE_R * cuy, CENTRAL_HINGE_Z),
            rpy=(0.0, CENTRAL_OPEN, CENTRAL_HINGE_A),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=1.5, lower=-CENTRAL_OPEN, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dome = object_model.get_part("cupola_dome")
    hull = object_model.get_part("module_hull")

    # ---------------------------------------------- intentional embeddings --
    ctx.allow_overlap(
        "module_hull",
        "cupola_dome",
        elem_a="cupola_collar",
        elem_b="dome_base_ring",
        reason="The hexagonal dome base ring is seated onto the collar top rim.",
    )
    for i in range(6):
        for k in range(2):
            ctx.allow_overlap(
                "cupola_dome",
                f"window_shutter_{i}",
                elem_a=f"shutter_pin_{i}",
                elem_b=f"hinge_lug_{k}",
                reason="Hinge pin is captured inside the shutter hinge lugs.",
            )
    for k in range(2):
        ctx.allow_overlap(
            "cupola_dome",
            "central_shutter",
            elem_a="central_pin",
            elem_b=f"hinge_arm_{k}",
            reason="Central hinge pin is captured inside the shutter arm roots.",
        )

    def _center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    # -------------------------------------------------- structural checks --
    hull_aabb = ctx.part_world_aabb(hull)
    ctx.check(
        "module hull is a ~3.7 m diameter cylinder",
        hull_aabb is not None and (hull_aabb[1][0] - hull_aabb[0][0]) >= 3.6,
        details=f"hull={hull_aabb}",
    )
    crown = ctx.part_element_world_aabb(dome, elem="crown_plate")
    ctx.check(
        "crown plate caps the dome above the facets",
        crown is not None and crown[1][2] >= ZT + 0.05,
        details=f"crown={crown}",
    )
    for i in range(6):
        glass = ctx.part_element_world_aabb(dome, elem=f"window_glass_{i}")
        ctx.check(f"facet window glass {i} present", glass is not None, details=str(glass))
    cglass = ctx.part_element_world_aabb(dome, elem="central_window_glass")
    ctx.check(
        "central round window glass sits under the crown hole",
        cglass is not None and abs(_center(cglass)[0]) < 0.05 and abs(_center(cglass)[1]) < 0.05,
        details=f"central_glass={cglass}",
    )
    bolts = [v for v in dome.visuals if v.name and v.name.startswith("ring_bolt_")]
    ctx.check("twelve dome-base bolts authored", len(bolts) == 12, details=str(len(bolts)))

    # ---------------------------------------------------- shutter checks --
    shutter_joints = [object_model.get_articulation(f"window_shutter_hinge_{i}")
                      for i in range(6)]
    ctx.check("six side-shutter hinges authored", len(shutter_joints) == 6)
    for joint in shutter_joints:
        ctx.check(
            f"{joint.name} is revolute and closes through {-SHUTTER_OPEN:.1f} rad",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits is not None
            and abs(joint.motion_limits.lower + SHUTTER_OPEN) < 1e-6,
            details=str(joint.motion_limits),
        )

    shutter0 = object_model.get_part("window_shutter_0")
    grid_ribs = [v for v in shutter0.visuals if v.name and v.name.startswith("grid_")]
    ctx.check(
        "shutter grid texture uses at least 9 ribs",
        len(grid_ribs) >= 9,
        details=str(len(grid_ribs)),
    )

    # At rest the shutters are splayed open outward past the dome silhouette.
    for i, a in enumerate(FACET_ANGLES):
        part = object_model.get_part(f"window_shutter_{i}")
        panel = ctx.part_element_world_aabb(part, elem="shutter_panel")
        c = _center(panel) if panel is not None else None
        ctx.check(
            f"shutter {i} rests swung open outward",
            c is not None and math.hypot(c[0], c[1]) > 1.45 and c[2] > ZB,
            details=f"panel_center={c}",
        )

    # Closing the hinge lays the shutter back over its window facet.
    j0 = shutter_joints[0]
    with ctx.pose({j0: -SHUTTER_OPEN}):
        panel_closed = ctx.part_element_world_aabb(shutter0, elem="shutter_panel")
    facet0_c = _facet_xyz(FACET_ANGLES[0], SLANT / 2.0, 0.0, 0.085)
    pc = _center(panel_closed) if panel_closed is not None else None
    ctx.check(
        "closed shutter 0 covers its facet window",
        pc is not None
        and math.hypot(pc[0] - facet0_c[0], pc[1] - facet0_c[1]) < 0.12
        and abs(pc[2] - facet0_c[2]) < 0.12,
        details=f"closed_center={pc}, facet_center={facet0_c}",
    )

    # Central round shutter closes concentric over the crown window.
    cjoint = object_model.get_articulation("central_shutter_hinge")
    cshutter = object_model.get_part("central_shutter")
    ctx.check(
        "central shutter hinge is revolute",
        cjoint.articulation_type == ArticulationType.REVOLUTE,
    )
    disc_open = ctx.part_element_world_aabb(cshutter, elem="shutter_disc")
    with ctx.pose({cjoint: -CENTRAL_OPEN}):
        disc_closed = ctx.part_element_world_aabb(cshutter, elem="shutter_disc")
    do = _center(disc_open) if disc_open is not None else None
    dc = _center(disc_closed) if disc_closed is not None else None
    ctx.check(
        "central shutter swings from open to concentric closed",
        do is not None
        and dc is not None
        and math.hypot(dc[0], dc[1]) < 0.10
        and math.hypot(do[0], do[1]) > 0.60,
        details=f"open={do}, closed={dc}",
    )
    ctx.check(
        "closed central shutter hovers just above the bezel",
        dc is not None and 2.83 < dc[2] < 2.95,
        details=f"closed_center={dc}",
    )

    return ctx.report()


object_model = build_object_model()
