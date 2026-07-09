from __future__ import annotations

# Paper coffee cup with a permanently-attached flip-top hinged drink lid.
# Frame: cup axis along +Z. Base sits at z=0, rim opens upward at z=+H.
#   The kraft-paper cup is TALL and TAPERED: narrower at the base, wider at the rim.
#   Outer wall carries vertical ribs. The cup is a hollow shell (open top).
#   The white snap-on lid is FIXED to the cup rim (no removal joint).
#   A large drink-spout flap swings open/closed about a side hinge (REVOLUTE)
#   as the sole drinking mechanism.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
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

# Spout opening (large flip-top drink spout on the -Y side of the dome)
SPOUT_W = 0.030          # spout width (X)
SPOUT_D = 0.022          # spout depth (Y)
SPOUT_CY = -(RIM_R - 0.023)  # spout center Y

# Flap dimensions (slightly oversized to cover the spout opening)
FLAP_W = SPOUT_W - 0.002
FLAP_D = SPOUT_D + 0.004   # extends past the spout front edge
FLAP_T = 0.003             # panel thickness

# Hinge: at the back edge of the spout, on the dome surface
HINGE_Y = SPOUT_CY - SPOUT_D * 0.5 - 0.003


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
    # Rolled rim bead at the top lip.
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
    # White snap-on lid permanently fixed to the cup rim. Skirt grips down over
    # the rim, flat shoulder ring, raised drink dome with a LARGE spout opening
    # on the -Y side for the flip-top flap.
    z_base = RIM_TOP_Z - LID_OVERLAP

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

    z_top = z_base + LID_SKIRT_H

    # Flat shoulder disc capping the skirt top.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=z_top - 0.001)
        .circle(RIM_R + 0.0035)
        .workplane(offset=0.0025)
        .circle(RIM_R + 0.0035)
        .loft(ruled=False)
    )

    # Raised drink dome.
    dome = (
        cq.Workplane("XY")
        .workplane(offset=z_top)
        .circle(RIM_R - 0.004)
        .workplane(offset=LID_DOME_H * 0.55)
        .circle(RIM_R - 0.012)
        .workplane(offset=LID_DOME_H * 0.45)
        .circle(RIM_R - 0.022)
        .loft(ruled=False)
    )

    lid = skirt.union(shoulder).union(dome)

    # Hinge mounting bar: spans wider than the spout, positioned at the hinge
    # line behind the spout opening. Added BEFORE the spout cut so it remains
    # connected to the dome body after the cut trims its front edge.
    hinge_z = z_top + 0.007
    bar_w = SPOUT_W + 0.012
    bar_d = 0.007
    bar_h = 0.006
    hinge_bar = (
        cq.Workplane("XY")
        .workplane(offset=hinge_z - bar_h * 0.5)
        .center(0.0, HINGE_Y + 0.001)
        .rect(bar_w, bar_d)
        .extrude(bar_h)
    )
    lid = lid.union(hinge_bar)

    # Large spout opening: rectangular cut through the dome + shoulder on -Y side.
    spout_cut = (
        cq.Workplane("XY")
        .workplane(offset=z_top - 0.002)
        .center(0.0, SPOUT_CY)
        .rect(SPOUT_W, SPOUT_D)
        .extrude(LID_DOME_H + 0.012)
    )
    lid = lid.cut(spout_cut)

    return lid


def _flap_panel() -> cq.Workplane:
    # Large drink-spout flap that covers the spout opening when closed.
    # Authored with hinge edge at local y=0, panel extends to +Y.
    panel = (
        cq.Workplane("XY")
        .moveTo(-FLAP_W * 0.5, 0.0)
        .lineTo(FLAP_W * 0.5, 0.0)
        .lineTo(FLAP_W * 0.5 - 0.003, FLAP_D)
        .lineTo(-FLAP_W * 0.5 + 0.003, FLAP_D)
        .close()
        .extrude(FLAP_T)
    )
    # Raised grip tab at the free edge for finger access.
    tab = (
        cq.Workplane("XY")
        .workplane(offset=FLAP_T)
        .center(0.0, FLAP_D - 0.005)
        .rect(0.012, 0.006)
        .extrude(0.004)
    )
    # Seal ridge on the underside that drops into the spout opening when closed.
    seal = (
        cq.Workplane("XY")
        .center(0.0, FLAP_D * 0.45)
        .rect(FLAP_W - 0.008, FLAP_D - 0.008)
        .extrude(-0.003)
    )
    return panel.union(tab).union(seal)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paper_coffee_cup")

    kraft = model.material("kraft_paper", rgba=(0.74, 0.55, 0.36, 1.0))
    kraft_dark = model.material("kraft_seam", rgba=(0.62, 0.45, 0.29, 1.0))
    white = model.material("lid_white", rgba=(0.93, 0.93, 0.92, 1.0))

    # ---- cup (root): tapered ribbed hollow paper cup + permanently fixed lid ----
    cup = model.part("cup")
    cup.visual(mesh_from_cadquery(_cup_shell(), "cup_shell"), material=kraft, name="cup_shell")
    cup.visual(mesh_from_cadquery(_cup_ribs(), "cup_ribs"), material=kraft_dark, name="cup_ribs")
    # Lid is permanently attached — inlined as a cup visual (no separate part).
    cup.visual(mesh_from_cadquery(_lid_solid(), "lid_shell"), material=white, name="lid_shell")
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=RIM_R, length=CUP_H), mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, CUP_H * 0.5)),
    )

    # ---- drink flap: large flip-top spout cover, hinged at the back edge ----
    z_top = RIM_TOP_Z - LID_OVERLAP + LID_SKIRT_H
    hinge_z = z_top + 0.007

    flap = model.part("drink_flap")
    # Hinge knuckle: barrel cylinder at the hinge line.
    flap.visual(
        mesh_from_geometry(
            CylinderGeometry(0.0025, SPOUT_W * 0.55).rotate_y(math.pi / 2.0),
            "flap_knuckle",
        ),
        material=white, name="flap_knuckle",
    )
    # The flap panel itself.
    flap.visual(
        mesh_from_cadquery(_flap_panel(), "flap_panel"),
        material=white, name="flap_panel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((FLAP_W, FLAP_D, FLAP_T + 0.004)), mass=0.0015,
        origin=Origin(xyz=(0.0, FLAP_D * 0.5, FLAP_T * 0.5)),
    )

    # Single revolute joint: flap swings about X-axis at the hinge line.
    model.articulation(
        "cup_to_flap",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=flap,
        origin=Origin(xyz=(0.0, HINGE_Y, hinge_z)),
        # Flap extends to local +Y from hinge; +X axis lifts the free edge up (+Z).
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5, velocity=2.0,
            lower=0.0, upper=math.radians(120.0),
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cup = object_model.get_part("cup")
    flap = object_model.get_part("drink_flap")
    flap_joint = object_model.get_articulation("cup_to_flap")

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

    # --- Lid is fixed on the cup (no prismatic lift-off joint) ---
    ctx.check(
        "no prismatic lift-off joint exists",
        not any(
            a.articulation_type == ArticulationType.PRISMATIC
            for a in object_model.articulations
        ),
        details="Only the revolute flip-top joint should exist",
    )

    # --- Lid is domed above the rim (lid_shell visual on cup) ---
    lid_aabb = ctx.part_element_world_aabb(cup, elem="lid_shell")
    ctx.check(
        "lid is domed above the rim",
        lid_aabb is not None and lid_aabb[1][2] > RIM_TOP_Z + 0.012,
        details=f"lid_shell top z={lid_aabb[1][2] if lid_aabb else None}, rim z={RIM_TOP_Z:.3f}",
    )

    # --- Only one articulation (the flap revolute) ---
    ctx.check(
        "exactly one articulation (flip-top flap)",
        len(object_model.articulations) == 1,
        details=f"articulation count={len(object_model.articulations)}",
    )

    # --- Flap overlaps: hinge knuckle seated at lid hinge bar, panel seated in spout ---
    ctx.allow_overlap(
        flap, cup,
        elem_a="flap_knuckle", elem_b="lid_shell",
        reason="Flap hinge knuckle barrel is intentionally embedded at the lid hinge mounting bar.",
    )
    ctx.allow_overlap(
        flap, cup,
        elem_a="flap_panel", elem_b="lid_shell",
        reason="Flap seal ridge intentionally seats into the spout opening as a flip-top closure.",
    )

    # --- Flap closed: positioned at the lid level over the spout ---
    flap_closed_z = ctx.part_world_position(flap)[2]
    ctx.check(
        "flap is at the lid level when closed",
        flap_closed_z is not None and flap_closed_z > RIM_TOP_Z,
        details=f"flap origin z={flap_closed_z:.3f}, rim z={RIM_TOP_Z:.3f}",
    )

    # --- Flap swings open: free edge rises above rest position ---
    rest_top_z = ctx.part_world_aabb(flap)[1][2]
    with ctx.pose({flap_joint: math.radians(110.0)}):
        open_top_z = ctx.part_world_aabb(flap)[1][2]
    ctx.check(
        "flap swings open (free edge rises significantly)",
        open_top_z > rest_top_z + 0.015,
        details=f"rest_top_z={rest_top_z:.3f}, open_top_z={open_top_z:.3f}",
    )

    # --- Hinge axis correct: at 90° open, flap extends upward from hinge ---
    rest_cy = (ctx.part_world_aabb(flap)[0][1] + ctx.part_world_aabb(flap)[1][1]) * 0.5
    with ctx.pose({flap_joint: math.radians(90.0)}):
        open_cy = (ctx.part_world_aabb(flap)[0][1] + ctx.part_world_aabb(flap)[1][1]) * 0.5
        open_top_90 = ctx.part_world_aabb(flap)[1][2]
    ctx.check(
        "flap rotates about hinge (center shifts toward hinge at 90°)",
        open_cy < rest_cy - 0.005,
        details=f"rest_cy={rest_cy:.4f}, open_cy={open_cy:.4f}",
    )
    ctx.check(
        "flap extends well above hinge at 90° open",
        open_top_90 > RIM_TOP_Z + LID_DOME_H + 0.010,
        details=f"open_top_90={open_top_90:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
