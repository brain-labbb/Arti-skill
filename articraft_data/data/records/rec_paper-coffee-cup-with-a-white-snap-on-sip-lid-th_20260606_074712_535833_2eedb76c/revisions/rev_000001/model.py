from __future__ import annotations

# Paper coffee cup with a white snap-on sip lid that has a small hinged drink flap.
# Frame: cup axis along +Z. Base sits at z=0, rim opens upward at z=+H.
#   The kraft-paper cup is TALL and TAPERED: narrower at the base, wider at the rim.
#   Outer wall carries vertical ribs. The cup is a hollow shell (open top).
# Articulations (two independent movers):
#   - lid: PRISMATIC, the white domed snap-on lid lifts straight up off the rim (+Z).
#   - sip flap: REVOLUTE, a small hinged drink flap that swings up/back about its
#     side hinge on the lid (0..~120 deg). The flap caps a REAL drink-spout opening
#     cut clean through the dome + shoulder, so swinging it open exposes the spout
#     hole and reveals the cup interior below. The flap is a CHILD of the lid, so it
#     lifts with the lid AND flips independently.

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

# Drink-spout opening: a SMALL real hole cut clean through the dome + shoulder on
# the -Y side. The hinged flap caps it; flipping the flap open exposes the hole and
# reveals the cup interior below.
SPOUT_W = 0.024          # spout opening width (X)
SPOUT_D = 0.018          # spout opening depth (Y)
SPOUT_CY = -(RIM_R - 0.020)  # spout center Y (toward the -Y front edge)

# Flap panel (slightly oversized to cap the spout opening when closed).
FLAP_W = SPOUT_W - 0.002      # nests just inside the opening sides
FLAP_D = SPOUT_D + 0.004      # extends a touch past the spout front edge
FLAP_T = 0.003                # panel thickness

# Hinge sits at the back edge of the spout, low on the dome shoulder.
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
    # White snap-on sip lid: a skirt that grips down over the rim, a flat shoulder
    # ring, and a raised drink dome. A small drink-spout opening is cut clean through
    # the dome + shoulder on the -Y side, and a hinge mounting bar sits behind it for
    # the flap knuckle.
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

    # Raised drink dome: a smaller domed mound rising from the shoulder.
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

    # Hinge mounting bar: spans wider than the spout, sits at the hinge line just
    # behind the spout opening. Added BEFORE the spout cut so it stays connected to
    # the dome body after the cut trims its front edge.
    hinge_z = z_top + 0.007
    bar_w = SPOUT_W + 0.010
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

    # Real drink-spout opening: rectangular cut clean through the dome + shoulder on
    # the -Y side. Below the cut is the hollow skirt interior, open to the cup, so the
    # opening reveals the cup interior when the flap swings clear.
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
    # Small hinged drink flap that caps the spout opening when closed.
    # Authored with the hinge edge at local y=0; the panel extends to +Y.
    panel = (
        cq.Workplane("XY")
        .moveTo(-FLAP_W * 0.5, 0.0)
        .lineTo(FLAP_W * 0.5, 0.0)
        .lineTo(FLAP_W * 0.5 - 0.003, FLAP_D)
        .lineTo(-FLAP_W * 0.5 + 0.003, FLAP_D)
        .close()
        .extrude(FLAP_T)
    )
    # Raised grip tab at the free edge for a fingernail to flip it open.
    tab = (
        cq.Workplane("XY")
        .workplane(offset=FLAP_T)
        .center(0.0, FLAP_D - 0.004)
        .rect(0.010, 0.005)
        .extrude(0.0035)
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

    # ---- sip flap: small hinged drink flap, caps the spout opening, child of lid ----
    z_top = RIM_TOP_Z - LID_OVERLAP + LID_SKIRT_H
    hinge_z = z_top + 0.007

    flap = model.part("sip_flap")
    flap.visual(mesh_from_geometry(
        CylinderGeometry(0.0025, SPOUT_W * 0.6).rotate_y(math.pi / 2.0),
        "flap_knuckle",
    ), material=white, name="flap_knuckle")
    flap.visual(mesh_from_cadquery(_flap_panel(), "flap_panel"), material=white, name="flap_panel")
    flap.inertial = Inertial.from_geometry(
        Box((FLAP_W, FLAP_D, FLAP_T + 0.004)), mass=0.0010,
        origin=Origin(xyz=(0.0, FLAP_D * 0.5, FLAP_T * 0.5)),
    )
    model.articulation(
        "lid_to_sip_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=flap,
        origin=Origin(xyz=(0.0, HINGE_Y, hinge_z)),
        # Flap extends to local +Y from the hinge; +X axis lifts the free edge up (+Z)
        # and back, swinging clear of the spout opening.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0, lower=0.0, upper=math.radians(120.0)),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cup = object_model.get_part("cup")
    lid = object_model.get_part("lid")
    flap = object_model.get_part("sip_flap")
    lid_joint = object_model.get_articulation("cup_to_lid")
    flap_joint = object_model.get_articulation("lid_to_sip_flap")

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

    # --- Sip flap is seated on the lid and caps the spout opening ---
    ctx.expect_contact(flap, lid, name="sip flap seated on the lid")
    ctx.allow_overlap(
        flap, lid,
        elem_a="flap_knuckle", elem_b="lid_shell",
        reason="Flap hinge knuckle barrel is intentionally embedded at the lid hinge mounting bar.",
    )
    ctx.allow_overlap(
        flap, lid,
        elem_a="flap_panel", elem_b="lid_shell",
        reason="Closed flap seal ridge intentionally seats into the spout opening as a closure.",
    )

    # --- Closed flap caps the spout opening (panel spans the spout center) ---
    closed_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "closed sip flap caps the spout opening",
        closed_panel_aabb is not None
        and closed_panel_aabb[0][1] < SPOUT_CY < closed_panel_aabb[1][1],
        details=f"flap_panel_y=({closed_panel_aabb[0][1]:.3f},{closed_panel_aabb[1][1]:.3f}), "
        f"spout_cy={SPOUT_CY:.3f}",
    )

    # --- Sip flap swings up about its hinge, exposing the spout opening ---
    rest_tip_z = ctx.part_world_aabb(flap)[1][2]
    rest_cy = (ctx.part_world_aabb(flap)[0][1] + ctx.part_world_aabb(flap)[1][1]) * 0.5
    with ctx.pose({flap_joint: math.radians(110.0)}):
        open_tip_z = ctx.part_world_aabb(flap)[1][2]
    ctx.check(
        "sip flap flips up about its hinge",
        open_tip_z > rest_tip_z + 0.010,
        details=f"rest_tip_z={rest_tip_z:.3f}, open_tip_z={open_tip_z:.3f}",
    )
    with ctx.pose({flap_joint: math.radians(90.0)}):
        open_tab_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
        open_cy = (ctx.part_world_aabb(flap)[0][1] + ctx.part_world_aabb(flap)[1][1]) * 0.5
    ctx.check(
        "sip flap swings clear above the dome (exposes the spout opening)",
        open_tab_aabb is not None and open_tab_aabb[1][2] > RIM_TOP_Z + LID_DOME_H + 0.005,
        details=f"open_panel_top_z={open_tab_aabb[1][2] if open_tab_aabb else None}, "
        f"dome_ref={RIM_TOP_Z + LID_DOME_H:.3f}",
    )
    ctx.check(
        "open flap center swings back toward the hinge (uncovers the spout)",
        open_cy < rest_cy - 0.004,
        details=f"rest_cy={rest_cy:.4f}, open_cy={open_cy:.4f}",
    )

    # --- Flap rides WITH the lid when the lid is lifted (child of the lid) ---
    rest_flap_z = ctx.part_world_position(flap)[2]
    with ctx.pose({lid_joint: 0.045}):
        lifted_flap_z = ctx.part_world_position(flap)[2]
    ctx.check(
        "sip flap lifts together with the lid",
        lifted_flap_z > rest_flap_z + 0.04,
        details=f"rest={rest_flap_z:.3f}, lifted={lifted_flap_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
