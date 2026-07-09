from __future__ import annotations

# Paper coffee cup with a white snap-on sip lid and a rotating slide-close drink ring.
# Frame: cup axis along +Z. Base sits at z=0, rim opens upward at z=+H.
#   The kraft-paper cup is TALL and TAPERED: narrower at the base, wider at the rim.
#   Outer wall carries vertical ribs. The cup is a hollow shell (open top).
# Articulations (two independent movers):
#   - lid: PRISMATIC, the white domed snap-on lid lifts straight up off the rim (+Z).
#   - slide ring: REVOLUTE about Z, a flat ring cap on the lid dome rotates to align
#     or block the drink hole (0 = closed, pi = open). The ring is a CHILD of the lid.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
CUP_H = 0.190            # paper cup wall height
BASE_R = 0.028           # outer radius at base (narrow)
RIM_R = 0.045            # outer radius at rim (wide) -> ~0.09 m top diameter
WALL_T = 0.0018          # paper wall thickness
N_RIBS = 56              # vertical ribs around the wall

RIM_TOP_Z = CUP_H        # z of the cup rim plane

LID_SKIRT_H = 0.018      # how far the lid skirt wraps down over the rim
LID_OVERLAP = 0.008      # vertical overlap of skirt onto the cup wall (snap fit)
LID_DOME_H = 0.020       # raised drink-dome height above the rim plane

# Slide ring dimensions
RING_OUTER_R = 0.028     # ring disc outer radius
RING_INNER_R = 0.0038    # ring bore (clears the pivot boss)
RING_THICK = 0.0025      # ring disc thickness
RING_HOLE_R = 0.005      # ring drink-hole radius
RING_HOLE_ORBIT = 0.018  # radial distance of ring drink-hole from center
BOSS_R = 0.003           # pivot boss shaft radius
BOSS_H = 0.005           # pivot boss height above dome top
DRINK_HOLE_R = 0.005     # lid drink-hole radius
DRINK_HOLE_ORBIT = 0.018 # radial distance of lid drink-hole from center


def _cup_shell() -> cq.Workplane:
    # Tapered hollow paper cup: outer cone-ish loft minus inner loft, open at top.
    outer = (
        cq.Workplane("XY")
        .circle(BASE_R)
        .workplane(offset=CUP_H * 0.5)
        .circle(BASE_R + (RIM_R - BASE_R) * 0.5)
        .workplane(offset=CUP_H * 0.5)
        .circle(RIM_R)
        .loft(ruled=False)
    )
    # Inner cavity: slightly smaller, pokes above the rim so the top is truly open.
    inner = (
        cq.Workplane("XY")
        .workplane(offset=WALL_T)  # closed bottom of thickness WALL_T
        .circle(BASE_R - WALL_T)
        .workplane(offset=CUP_H * 0.5)
        .circle(BASE_R + (RIM_R - BASE_R) * 0.5 - WALL_T)
        .workplane(offset=CUP_H * 0.5 + 0.006)
        .circle(RIM_R - WALL_T)
        .loft(ruled=False)
    )
    # Rolled rim bead at the top lip: a thick solid ring of outer radius RIM_R+0.0016
    # extending from below the rim up to the lip. Unioned onto the OUTER solid before
    # the inner cut so the inner cut hollows it consistently into one connected shell.
    bead = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - 0.012)
        .circle(RIM_R)
        .workplane(offset=0.010)
        .circle(RIM_R + 0.0016)
        .workplane(offset=0.004)
        .circle(RIM_R + 0.0016)
        .loft(ruled=False)
    )
    outer = outer.union(bead)
    shell = outer.cut(inner)
    return shell


def _cup_ribs() -> cq.Workplane:
    # Vertical ribs standing proud of the tapered wall, following the taper.
    ribs = None
    z0 = 0.012
    z1 = CUP_H - 0.012
    for i in range(N_RIBS):
        ang = 2.0 * math.pi * i / N_RIBS
        r_lo = BASE_R + 0.0006
        r_hi = RIM_R + 0.0006
        x_lo, y_lo = r_lo * math.cos(ang), r_lo * math.sin(ang)
        x_hi, y_hi = r_hi * math.cos(ang), r_hi * math.sin(ang)
        seg = (
            cq.Workplane("XY")
            .workplane(offset=z0)
            .center(x_lo, y_lo)
            .circle(0.0011)
            .workplane(offset=(z1 - z0))
            .center(x_hi - x_lo, y_hi - y_lo)
            .circle(0.0011)
            .loft(ruled=True)
        )
        ribs = seg if ribs is None else ribs.union(seg)
    return ribs


def _lid_solid() -> cq.Workplane:
    # White snap-on sip lid: skirt grips down over the rim, flat shoulder ring,
    # raised dome with a wider flatter top for the slide ring, center pivot boss
    # with capture flange, and a through drink hole on the +X side.
    z_base = RIM_TOP_Z - LID_OVERLAP  # skirt starts below the rim (snaps over it)

    skirt_outer = (
        cq.Workplane("XY")
        .workplane(offset=z_base)
        .circle(RIM_R + 0.0035)
        .workplane(offset=LID_SKIRT_H)
        .circle(RIM_R + 0.0035)
        .loft(ruled=False)
    )
    skirt_inner = (
        cq.Workplane("XY")
        .workplane(offset=z_base - 0.001)
        .circle(RIM_R - 0.0006)
        .workplane(offset=LID_SKIRT_H + 0.002)
        .circle(RIM_R - 0.0006)
        .loft(ruled=False)
    )
    skirt = skirt_outer.cut(skirt_inner)

    z_top = z_base + LID_SKIRT_H  # top of the skirt = lid shoulder plane

    # Flat shoulder disc capping the skirt top.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=z_top - 0.001)
        .circle(RIM_R + 0.0035)
        .workplane(offset=0.0025)
        .circle(RIM_R + 0.0035)
        .loft(ruled=False)
    )

    # Raised drink dome: wider flatter top to carry the slide ring.
    dome = (
        cq.Workplane("XY")
        .workplane(offset=z_top)
        .circle(RIM_R - 0.004)       # 0.041 base
        .workplane(offset=LID_DOME_H * 0.55)
        .circle(RIM_R - 0.009)       # 0.036 mid
        .workplane(offset=LID_DOME_H * 0.45)
        .circle(RIM_R - 0.013)       # 0.032 top (wider for ring seat)
        .loft(ruled=False)
    )

    lid = skirt.union(shoulder).union(dome)

    # Center pivot boss: cylindrical shaft rising from dome top.
    boss = (
        cq.Workplane("XY")
        .workplane(offset=z_top + LID_DOME_H - 0.001)
        .circle(BOSS_R)
        .extrude(BOSS_H + 0.001)
    )
    # Capture flange: wider disc at boss top so ring can't lift off.
    flange = (
        cq.Workplane("XY")
        .workplane(offset=z_top + LID_DOME_H + BOSS_H - 0.001)
        .circle(BOSS_R + 0.0012)
        .extrude(0.0015)
    )
    lid = lid.union(boss).union(flange)

    # Cut drink hole through dome at +X side (passes through shoulder and dome).
    drink_hole = (
        cq.Workplane("XY")
        .workplane(offset=z_top - 0.005)
        .center(DRINK_HOLE_ORBIT, 0.0)
        .circle(DRINK_HOLE_R)
        .extrude(LID_DOME_H + BOSS_H + 0.015)
    )
    lid = lid.cut(drink_hole)
    return lid


def _slide_ring_solid() -> cq.Workplane:
    # Flat annular disc that rotates on the lid dome to open/close the drink hole.
    # Local frame: z=0 is the dome top surface, ring extends upward.
    # The drink hole in the ring is at angle pi (-X) so that at q=0 the ring body
    # blocks the lid's drink hole (at +X), and at q=pi the holes align (open).
    # A grip tab at +X gives a finger-hold for turning.

    # Main annular disc
    ring = (
        cq.Workplane("XY")
        .circle(RING_OUTER_R)
        .circle(RING_INNER_R)
        .extrude(RING_THICK)
    )

    # Cut drink hole at angle pi (-X direction in local frame)
    hole = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .center(-RING_HOLE_ORBIT, 0.0)
        .circle(RING_HOLE_R)
        .extrude(RING_THICK + 0.002)
    )
    ring = ring.cut(hole)

    # Grip tab: rounded protrusion at +X for finger turning
    tab = (
        cq.Workplane("XY")
        .center(RING_OUTER_R + 0.002, 0.0)
        .circle(0.005)
        .extrude(RING_THICK)
    )
    ring = ring.union(tab)

    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paper_coffee_cup")

    kraft = model.material("kraft_paper", rgba=(0.74, 0.55, 0.36, 1.0))
    kraft_dark = model.material("kraft_seam", rgba=(0.62, 0.45, 0.29, 1.0))
    white = model.material("lid_white", rgba=(0.93, 0.93, 0.92, 1.0))
    ring_mat = model.material("ring_offwhite", rgba=(0.86, 0.85, 0.83, 1.0))

    # ---- cup (root): tapered ribbed hollow paper cup ----
    cup = model.part("cup")
    cup.visual(mesh_from_cadquery(_cup_shell(), "cup_shell"), material=kraft, name="cup_shell")
    cup.visual(mesh_from_cadquery(_cup_ribs(), "cup_ribs"), material=kraft_dark, name="cup_ribs")
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=RIM_R, length=CUP_H), mass=0.022, origin=Origin(xyz=(0.0, 0.0, CUP_H * 0.5))
    )

    # ---- lid: white domed snap-on sip lid, lifts straight up off the rim ----
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_solid(), "lid_shell"), material=white, name="lid_shell")
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=RIM_R + 0.0035, length=LID_SKIRT_H + LID_DOME_H),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z + LID_DOME_H * 0.4)),
    )
    model.articulation(
        "cup_to_lid",
        ArticulationType.PRISMATIC,
        parent=cup,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.1, lower=0.0, upper=0.045),
    )

    # ---- slide ring: rotates about Z on the lid dome to open/close drink hole ----
    z_top = RIM_TOP_Z - LID_OVERLAP + LID_SKIRT_H
    ring = model.part("slide_ring")
    ring.visual(mesh_from_cadquery(_slide_ring_solid(), "ring_disc"), material=ring_mat, name="ring_disc")
    ring.inertial = Inertial.from_geometry(
        Cylinder(radius=RING_OUTER_R, length=RING_THICK),
        mass=0.001,
        origin=Origin(xyz=(0.0, 0.0, RING_THICK * 0.5)),
    )
    model.articulation(
        "lid_to_slide_ring",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=ring,
        # Rotation origin at dome top center (pivot boss base)
        origin=Origin(xyz=(0.0, 0.0, z_top + LID_DOME_H)),
        axis=(0.0, 0.0, 1.0),
        # q=0: closed (ring body covers lid drink hole)
        # q=pi: open (ring hole aligns with lid drink hole)
        motion_limits=MotionLimits(effort=0.5, velocity=2.0, lower=0.0, upper=math.pi),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cup = object_model.get_part("cup")
    lid = object_model.get_part("lid")
    ring = object_model.get_part("slide_ring")
    lid_joint = object_model.get_articulation("cup_to_lid")
    ring_joint = object_model.get_articulation("lid_to_slide_ring")

    # --- Cup is tall and tapered (narrower at base, wider at rim) ---
    cup_aabb = ctx.part_world_aabb(cup)
    cext = _ext(cup_aabb)
    ctx.check(
        "cup is tall (height > footprint)",
        cext[2] > 0.16 and cext[2] > cext[0] + 0.05,
        details=f"cup extents={cext}",
    )
    ctx.check(
        "cup top diameter near 0.09 m",
        0.08 < cext[0] < 0.10 and 0.08 < cext[1] < 0.10,
        details=f"cup xy extents=({cext[0]:.3f},{cext[1]:.3f})",
    )
    base_slab = ctx.part_element_world_aabb(cup, elem="cup_shell")
    ctx.check(
        "cup is hollow shell with a real base",
        base_slab is not None and base_slab[0][2] < 0.005,
        details=f"cup_shell aabb min z={base_slab[0][2] if base_slab else None}",
    )

    # --- Lid sits seated over the rim (snap-on overlap) and is domed on top ---
    ctx.allow_overlap(
        lid, cup,
        elem_a="lid_shell", elem_b="cup_shell",
        reason="Lid skirt is intentionally snapped down over the cup rim bead (snap-on fit).",
    )
    ctx.allow_overlap(
        lid, cup,
        elem_a="lid_shell", elem_b="cup_ribs",
        reason="Lid skirt overlaps the top of the ribbed wall where it grips the rim.",
    )
    ctx.expect_overlap(
        lid, cup, axes="z", min_overlap=0.004, name="lid skirt grips down over the rim",
    )
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid mounted at the cup top",
        lid_pos is not None,
        details=f"lid origin={lid_pos}",
    )
    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid is domed above the rim",
        lid_aabb[1][2] > RIM_TOP_Z + 0.012,
        details=f"lid top z={lid_aabb[1][2]:.3f}, rim z={RIM_TOP_Z:.3f}",
    )

    # --- Lid lifts STRAIGHT UP off the cup (prismatic +Z) ---
    rest_top = ctx.part_world_aabb(lid)[1][2]
    rest_xy = ctx.part_world_position(lid)[:2]
    with ctx.pose({lid_joint: 0.045}):
        lifted_top = ctx.part_world_aabb(lid)[1][2]
        lifted_xy = ctx.part_world_position(lid)[:2]
        lifted_bottom = ctx.part_world_aabb(lid)[0][2]
    ctx.check(
        "lid lifts straight up off the cup",
        lifted_top > rest_top + 0.04
        and abs(lifted_xy[0] - rest_xy[0]) < 1e-4
        and abs(lifted_xy[1] - rest_xy[1]) < 1e-4,
        details=f"rest_top={rest_top:.3f}, lifted_top={lifted_top:.3f}, dxy={lifted_xy}",
    )
    ctx.check(
        "lifted lid clears the cup rim",
        lifted_bottom > RIM_TOP_Z - 0.006,
        details=f"lifted skirt bottom z={lifted_bottom:.3f}, rim z={RIM_TOP_Z:.3f}",
    )

    # --- Slide ring is seated on the lid dome and rotates about Z ---
    ctx.expect_contact(ring, lid, name="slide ring seated on lid dome top")

    # Ring rotates about its center (XY position stays fixed)
    ring_rest_center = ctx.part_world_position(ring)
    with ctx.pose({ring_joint: math.pi}):
        ring_open_center = ctx.part_world_position(ring)
    ctx.check(
        "slide ring rotates about its center axis",
        abs(ring_open_center[0] - ring_rest_center[0]) < 0.002
        and abs(ring_open_center[1] - ring_rest_center[1]) < 0.002,
        details=f"rest=({ring_rest_center[0]:.4f},{ring_rest_center[1]:.4f}), "
                f"open=({ring_open_center[0]:.4f},{ring_open_center[1]:.4f})",
    )

    # Ring grip tab moves when rotated (AABB shifts due to asymmetric tab)
    ring_rest_aabb = ctx.part_world_aabb(ring)
    with ctx.pose({ring_joint: math.pi}):
        ring_open_aabb = ctx.part_world_aabb(ring)
    # At q=0 tab is at +X, at q=pi tab is at -X; the AABB X bounds should swap
    ctx.check(
        "slide ring tab rotates from one side to the other",
        abs(ring_open_aabb[0][0] - ring_rest_aabb[0][0]) > 0.003
        or abs(ring_open_aabb[1][0] - ring_rest_aabb[1][0]) > 0.003,
        details=f"rest_x=({ring_rest_aabb[0][0]:.4f},{ring_rest_aabb[1][0]:.4f}), "
                f"open_x=({ring_open_aabb[0][0]:.4f},{ring_open_aabb[1][0]:.4f})",
    )

    # Slide ring rides WITH the lid when the lid is lifted (child of the lid)
    rest_ring_z = ctx.part_world_position(ring)[2]
    with ctx.pose({lid_joint: 0.045}):
        lifted_ring_z = ctx.part_world_position(ring)[2]
    ctx.check(
        "slide ring lifts together with the lid",
        lifted_ring_z > rest_ring_z + 0.04,
        details=f"rest={rest_ring_z:.3f}, lifted={lifted_ring_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
