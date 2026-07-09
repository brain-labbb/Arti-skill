from __future__ import annotations

# Paper coffee cup with a white screw-thread twist lid and hinged drink flap.
# Frame: cup axis along +Z. Base sits at z=0, rim opens upward at z=+H.
#   The kraft-paper cup is TALL and TAPERED: narrower at the base, wider at the rim.
#   Outer wall carries vertical ribs. The cup is a hollow shell (open top).
#   External thread lugs on the rim collar engage matching internal lugs in the lid skirt.
# Articulations (two independent movers):
#   - lid: REVOLUTE about Z, the white domed lid screws on/off the threaded rim.
#   - sip flap: REVOLUTE, the small drink-hole tab flips up/back about its hinge on
#     the lid (0..~120 deg). The flap is a CHILD of the lid, so it twists with the lid
#     AND flips independently.

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
LID_OVERLAP = 0.008      # vertical overlap of skirt onto the cup wall (thread engagement)
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

# ---- thread dimensions ----
N_THREADS = 3            # number of thread starts
THREAD_LUG_W = 0.007     # lug circumferential width
THREAD_LUG_H = 0.006     # lug axial height
THREAD_LUG_D = 0.0025    # lug radial depth (stands proud of surface)
THREAD_Z_CENTER = RIM_TOP_Z - 0.006  # vertical center of thread engagement zone


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


def _cup_thread_lug(index: int) -> cq.Workplane:
    """One external thread lug on the cup rim collar.

    N_THREADS equally spaced thread starts, each with 2 lug segments along the
    thread path to give the visual impression of a screw thread on the rim.
    """
    base_angle = 2.0 * math.pi * index / N_THREADS
    # Outer surface of the rim bead
    r_outer = RIM_R + 0.0016
    lug = None
    for j in range(2):
        # Two lug segments per thread start, with a slight angular pitch offset
        angle = base_angle + j * 0.12
        x_c = r_outer * math.cos(angle)
        y_c = r_outer * math.sin(angle)
        # Rotate the box to be tangent to the circumference
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        # Lug center on the outer rim surface
        single = (
            cq.Workplane("XY")
            .workplane(offset=THREAD_Z_CENTER - THREAD_LUG_H * 0.5)
            .center(x_c, y_c)
            .transformed(rotate=(0, 0, math.degrees(angle)))
            .rect(THREAD_LUG_D, THREAD_LUG_W)
            .extrude(THREAD_LUG_H)
        )
        lug = single if lug is None else lug.union(single)
    return lug


def _cup_threads() -> cq.Workplane:
    """All external thread lugs on the cup rim collar."""
    result = None
    for i in range(N_THREADS):
        lug = _cup_thread_lug(i)
        result = lug if result is None else result.union(lug)
    return result


def _lid_thread_lug(index: int) -> cq.Workplane:
    """One internal thread lug inside the lid skirt, matching the cup threads.
    
    Authored in lid local frame where z=0 is the rim top plane.
    """
    base_angle = 2.0 * math.pi * index / N_THREADS
    # Inner surface of the lid skirt
    r_inner = RIM_R - 0.0006
    # Thread center in local frame (relative to rim top)
    local_thread_z = THREAD_Z_CENTER - RIM_TOP_Z
    lug = None
    for j in range(2):
        angle = base_angle + j * 0.12
        x_c = r_inner * math.cos(angle)
        y_c = r_inner * math.sin(angle)
        single = (
            cq.Workplane("XY")
            .workplane(offset=local_thread_z - THREAD_LUG_H * 0.5)
            .center(x_c, y_c)
            .transformed(rotate=(0, 0, math.degrees(angle)))
            .rect(THREAD_LUG_D, THREAD_LUG_W)
            .extrude(THREAD_LUG_H)
        )
        lug = single if lug is None else lug.union(single)
    return lug


def _lid_threads() -> cq.Workplane:
    """All internal thread lugs inside the lid skirt."""
    result = None
    for i in range(N_THREADS):
        lug = _lid_thread_lug(i)
        result = lug if result is None else result.union(lug)
    return result


def _lid_solid() -> cq.Workplane:
    # White screw-on sip lid: a skirt that threads down over the rim, a flat shoulder
    # ring, and a raised drink dome with a sip recess near the back.
    # Authored in lid local frame where z=0 is the rim top plane.
    z_base = -LID_OVERLAP  # skirt starts below the rim (threads engage here)

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

    z_top = z_base + LID_SKIRT_H  # top of the skirt = lid shoulder plane (local)

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
    thread_mat = model.material("rim_thread", rgba=(0.68, 0.50, 0.33, 1.0))
    white = model.material("lid_white", rgba=(0.93, 0.93, 0.92, 1.0))
    thread_lid_mat = model.material("lid_thread", rgba=(0.88, 0.88, 0.86, 1.0))

    # ---- cup (root): tapered ribbed hollow paper cup with threaded rim ----
    cup = model.part("cup")
    cup.visual(mesh_from_cadquery(_cup_shell(), "cup_shell"), material=kraft, name="cup_shell")
    cup.visual(mesh_from_cadquery(_cup_ribs(), "cup_ribs"), material=kraft_dark, name="cup_ribs")
    cup.visual(mesh_from_cadquery(_cup_threads(), "cup_threads"), material=thread_mat, name="cup_threads")
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=RIM_R, length=CUP_H), mass=0.022, origin=Origin(xyz=(0.0, 0.0, CUP_H * 0.5))
    )

    # ---- lid: white domed screw-on sip lid, twists on/off the threaded rim ----
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_solid(), "lid_shell"), material=white, name="lid_shell")
    lid.visual(mesh_from_cadquery(_lid_threads(), "lid_threads"), material=thread_lid_mat, name="lid_threads")
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=RIM_R + 0.0035, length=LID_SKIRT_H + LID_DOME_H),
        mass=0.006,
        # Inertial origin in lid local frame: centered at the dome mid-height above rim plane.
        origin=Origin(xyz=(0.0, 0.0, LID_DOME_H * 0.4)),
    )
    model.articulation(
        "cup_to_lid",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=lid,
        # Joint origin at the rim contact plane, centered on the cup axis.
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        # Rotation about the cup central Z axis. Positive q unscrews (CCW from above).
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0,
            lower=0.0,                          # q=0: fully sealed/tightened
            upper=math.radians(270.0),          # ~3/4 turn to fully unscrew
        ),
    )

    # ---- sip flap: flips up/back about its hinge on the lid (child of the lid) ----
    # Flap hinge is in the lid's local frame (z=0 is rim top plane).
    z_top = -LID_OVERLAP + LID_SKIRT_H  # lid shoulder plane in local frame
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
    # Taper: base footprint is narrower than rim footprint (slice element AABBs by z).
    base_slab = ctx.part_element_world_aabb(cup, elem="cup_shell")
    ctx.check(
        "cup is hollow shell with a real base",
        base_slab is not None and base_slab[0][2] < 0.005,
        details=f"cup_shell aabb min z={base_slab[0][2] if base_slab else None}",
    )

    # --- Cup has external thread lugs on the rim ---
    thread_aabb = ctx.part_element_world_aabb(cup, elem="cup_threads")
    ctx.check(
        "cup has visible thread lugs on the rim",
        thread_aabb is not None
        and thread_aabb[0][2] > RIM_TOP_Z - 0.015
        and thread_aabb[1][2] < RIM_TOP_Z + 0.005,
        details=f"cup_threads aabb={thread_aabb}",
    )

    # --- Lid sits seated over the rim (thread engagement) and is domed on top ---
    ctx.allow_overlap(
        lid, cup,
        elem_a="lid_shell", elem_b="cup_shell",
        reason="Lid skirt threads down over the cup rim collar (screw-on thread engagement).",
    )
    ctx.allow_overlap(
        lid, cup,
        elem_a="lid_shell", elem_b="cup_ribs",
        reason="Lid skirt overlaps the top of the ribbed wall where threads engage.",
    )
    ctx.allow_overlap(
        lid, cup,
        elem_a="lid_threads", elem_b="cup_threads",
        reason="Internal lid thread lugs engage external cup thread lugs (screw fit).",
    )
    ctx.allow_overlap(
        lid, cup,
        elem_a="lid_threads", elem_b="cup_shell",
        reason="Lid thread lugs sit inside the rim collar engagement zone.",
    )
    ctx.allow_overlap(
        lid, cup,
        elem_a="lid_shell", elem_b="cup_threads",
        reason="Cup external thread lugs engage inside the lid skirt inner surface (screw thread fit).",
    )
    ctx.expect_overlap(
        lid, cup, axes="z", min_overlap=0.004, name="lid skirt threads down over the rim",
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

    # --- Lid has internal thread lugs matching the cup ---
    lid_thread_aabb = ctx.part_element_world_aabb(lid, elem="lid_threads")
    ctx.check(
        "lid has visible internal thread lugs",
        lid_thread_aabb is not None,
        details=f"lid_threads aabb={lid_thread_aabb}",
    )

    # --- Lid TWISTS about Z axis (revolute screw-on, not prismatic lift) ---
    # At q=0 the lid is sealed; at the upper limit it should be unscrewed.
    rest_center = ctx.part_world_position(lid)
    rest_aabb = ctx.part_world_aabb(lid)
    rest_bottom = rest_aabb[0][2]

    with ctx.pose({lid_joint: math.radians(270.0)}):
        unscrewed_center = ctx.part_world_position(lid)
        unscrewed_aabb = ctx.part_world_aabb(lid)
        unscrewed_bottom = unscrewed_aabb[0][2]

    # The lid should stay at the same Z height when twisting (no vertical lift)
    ctx.check(
        "lid rotates about Z axis without lifting off",
        rest_center is not None and unscrewed_center is not None
        and abs(unscrewed_center[2] - rest_center[2]) < 0.002,
        details=f"rest_z={rest_center[2] if rest_center else None}, "
                f"unscrewed_z={unscrewed_center[2] if unscrewed_center else None}",
    )

    # XY center should stay the same (rotation about Z doesn't translate)
    ctx.check(
        "lid stays centered on the cup axis when twisting",
        rest_center is not None and unscrewed_center is not None
        and abs(unscrewed_center[0] - rest_center[0]) < 0.002
        and abs(unscrewed_center[1] - rest_center[1]) < 0.002,
        details=f"rest_xy=({rest_center[0]:.4f},{rest_center[1]:.4f}), "
                f"unscrewed_xy=({unscrewed_center[0]:.4f},{unscrewed_center[1]:.4f})",
    )

    # Prove the lid actually rotates (sip flap position moves circumferentially)
    rest_flap_pos = ctx.part_world_position(flap)
    with ctx.pose({lid_joint: math.radians(90.0)}):
        rotated_flap_pos = ctx.part_world_position(flap)
    ctx.check(
        "lid rotation moves the flap circumferentially",
        rest_flap_pos is not None and rotated_flap_pos is not None
        and (abs(rotated_flap_pos[0] - rest_flap_pos[0]) > 0.005
             or abs(rotated_flap_pos[1] - rest_flap_pos[1]) > 0.005),
        details=f"rest_flap_xy=({rest_flap_pos[0]:.4f},{rest_flap_pos[1]:.4f}), "
                f"rotated_flap_xy=({rotated_flap_pos[0]:.4f},{rotated_flap_pos[1]:.4f})",
    )

    # Thread engagement proof: cup threads and lid threads overlap in Z (engaged)
    ctx.expect_overlap(
        lid, cup, axes="z",
        elem_a="lid_threads", elem_b="cup_threads",
        min_overlap=0.003,
        name="thread lugs engaged in Z when lid is sealed",
    )
    # Cup thread lugs contact the lid skirt inner surface (proof for the allowance)
    ctx.expect_contact(
        lid, cup,
        elem_a="lid_shell", elem_b="cup_threads",
        name="cup thread lugs contact lid skirt (screw engagement)",
    )

    # Confirm the joint is REVOLUTE (not prismatic)
    ctx.check(
        "lid joint is revolute (twist)",
        lid_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint type={lid_joint.articulation_type}",
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

    # --- Flap rides WITH the lid when the lid is twisted (child of the lid) ---
    rest_flap_z = ctx.part_world_position(flap)[2]
    with ctx.pose({lid_joint: math.radians(90.0)}):
        twisted_flap_z = ctx.part_world_position(flap)[2]
    ctx.check(
        "sip flap stays with the lid when twisting",
        twisted_flap_z is not None and rest_flap_z is not None
        and abs(twisted_flap_z - rest_flap_z) < 0.002,
        details=f"rest={rest_flap_z:.3f}, twisted={twisted_flap_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
