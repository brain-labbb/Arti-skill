from __future__ import annotations

# Travel tumbler with a waisted contour and white snap-on sip lid with hinged drink flap.
# Frame: tumbler axis along +Z. Base sits at z=0, rim opens upward at z=+H.
# The body has a WAISTED (hourglass) profile: wider at base and rim, narrower at
# the grip waist, built from a multi-station smooth loft.
# Outer wall carries vertical ribs that follow the waisted contour.
# The cup is a hollow shell (open top).
# Articulations (two independent movers):
#   - lid: PRISMATIC, the white domed snap-on lid lifts straight up off the rim (+Z).
#   - sip flap: REVOLUTE, the small drink-hole tab flips up/back about its hinge on
#     the lid (0..~120 deg). The flap is a CHILD of the lid, so it lifts with the lid
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
CUP_H = 0.190            # cup wall height
WALL_T = 0.0018          # wall thickness
N_RIBS = 32              # vertical ribs following the contour

# Waisted profile stations: (z_height_m, outer_radius_m)
# Wide at base and rim, narrowest at the grip waist (~mid-height).
PROFILE = [
    (0.000, 0.037),    # base (wide for stability)
    (0.050, 0.042),    # lower belly
    (0.095, 0.033),    # waist (narrowest — grip zone)
    (0.150, 0.043),    # upper swell
    (0.190, 0.045),    # rim (unchanged for lid compatibility)
]

RIM_R = PROFILE[-1][1]   # rim radius (0.045 m → ~90 mm diameter)
RIM_TOP_Z = CUP_H

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


def _profile_r(z: float) -> float:
    """Linearly interpolate the outer radius at height z from PROFILE stations."""
    if z <= PROFILE[0][0]:
        return PROFILE[0][1]
    if z >= PROFILE[-1][0]:
        return PROFILE[-1][1]
    for i in range(len(PROFILE) - 1):
        z0, r0 = PROFILE[i]
        z1, r1 = PROFILE[i + 1]
        if z0 <= z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + t * (r1 - r0)
    return PROFILE[-1][1]


def _cup_shell() -> cq.Workplane:
    # Waisted hollow tumbler shell: outer loft through profile stations minus inner loft.

    # --- outer loft through all waisted profile stations ---
    wp = cq.Workplane("XY").circle(PROFILE[0][1])
    for i in range(1, len(PROFILE)):
        dz = PROFILE[i][0] - PROFILE[i - 1][0]
        wp = wp.workplane(offset=dz).circle(PROFILE[i][1])
    outer = wp.loft(ruled=False)

    # --- inner cavity loft (offset inward by WALL_T, closed bottom, open top) ---
    inner_stations = [(WALL_T, PROFILE[0][1] - WALL_T)]
    for z, r in PROFILE[1:-1]:
        inner_stations.append((z, r - WALL_T))
    inner_stations.append((CUP_H + 0.006, PROFILE[-1][1] - WALL_T))

    wp = (
        cq.Workplane("XY")
        .workplane(offset=inner_stations[0][0])
        .circle(inner_stations[0][1])
    )
    for i in range(1, len(inner_stations)):
        dz = inner_stations[i][0] - inner_stations[i - 1][0]
        wp = wp.workplane(offset=dz).circle(inner_stations[i][1])
    inner = wp.loft(ruled=False)

    # --- rolled rim bead at the top lip ---
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
    # Vertical ribs that follow the waisted contour, standing proud of the outer wall.
    # Rib z-stations track the profile (with small margins from top/bottom edges).
    rib_z = [0.012]
    for z, _ in PROFILE[1:-1]:
        rib_z.append(z)
    rib_z.append(CUP_H - 0.012)

    ribs = None
    for i in range(N_RIBS):
        ang = 2.0 * math.pi * i / N_RIBS
        cos_a, sin_a = math.cos(ang), math.sin(ang)

        # Rib center at each station follows the outer contour (+ small proud offset).
        positions = []
        for z in rib_z:
            r = _profile_r(z) + 0.0006
            positions.append((r * cos_a, r * sin_a))

        # Multi-station ruled loft for this rib.
        wp = (
            cq.Workplane("XY")
            .workplane(offset=rib_z[0])
            .center(positions[0][0], positions[0][1])
            .circle(0.0010)
        )
        for j in range(1, len(rib_z)):
            dz = rib_z[j] - rib_z[j - 1]
            dx = positions[j][0] - positions[j - 1][0]
            dy = positions[j][1] - positions[j - 1][1]
            wp = wp.workplane(offset=dz).center(dx, dy).circle(0.0010)

        seg = wp.loft(ruled=True)
        ribs = seg if ribs is None else ribs.union(seg)
    return ribs


def _lid_solid() -> cq.Workplane:
    # White snap-on sip lid: skirt gripping down over the rim, flat shoulder ring,
    # raised drink dome with a sip recess near the back.
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

    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=z_top - 0.001)
        .circle(RIM_R + 0.0035)
        .workplane(offset=0.0025)
        .circle(RIM_R + 0.0035)
        .loft(ruled=False)
    )

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
    model = ArticulatedObject(name="travel_tumbler")

    kraft = model.material("kraft_paper", rgba=(0.74, 0.55, 0.36, 1.0))
    kraft_dark = model.material("kraft_seam", rgba=(0.62, 0.45, 0.29, 1.0))
    white = model.material("lid_white", rgba=(0.93, 0.93, 0.92, 1.0))

    # ---- cup (root): waisted ribbed hollow tumbler body ----
    cup = model.part("cup")
    cup.visual(mesh_from_cadquery(_cup_shell(), "cup_shell"), material=kraft, name="cup_shell")
    cup.visual(mesh_from_cadquery(_cup_ribs(), "cup_ribs"), material=kraft_dark, name="cup_ribs")
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=RIM_R, length=CUP_H), mass=0.025, origin=Origin(xyz=(0.0, 0.0, CUP_H * 0.5))
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

    # --- Waisted contour: waist must be narrower than both base and rim ---
    radii = [r for _, r in PROFILE]
    waist_r = min(radii)
    base_r = PROFILE[0][1]
    rim_r = PROFILE[-1][1]
    belly_r = max(radii)
    ctx.check(
        "waisted contour: waist narrower than base and rim",
        waist_r < base_r - 0.002 and waist_r < rim_r - 0.002,
        details=f"waist_r={waist_r:.4f}, base_r={base_r:.4f}, rim_r={rim_r:.4f}",
    )
    ctx.check(
        "waisted contour: belly and swell wider than waist",
        belly_r > waist_r + 0.005,
        details=f"belly_r={belly_r:.4f}, waist_r={waist_r:.4f}",
    )
    waist_z = [z for z, r in PROFILE if r == waist_r][0]
    ctx.check(
        "waist is near mid-height",
        0.3 * CUP_H < waist_z < 0.7 * CUP_H,
        details=f"waist_z={waist_z:.3f}, cup_h={CUP_H:.3f}",
    )

    # --- Cup is tall ---
    cup_aabb = ctx.part_world_aabb(cup)
    cext = _ext(cup_aabb)
    ctx.check(
        "cup is tall (height > footprint)",
        cext[2] > 0.16 and cext[2] > cext[0] + 0.05,
        details=f"cup extents={cext}",
    )

    # --- Rim diameter unchanged (lid compatibility) ---
    ctx.check(
        "rim diameter near 0.09 m",
        0.08 < cext[0] < 0.10 and 0.08 < cext[1] < 0.10,
        details=f"cup xy extents=({cext[0]:.3f},{cext[1]:.3f})",
    )

    # --- Cup is hollow shell with real base ---
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
