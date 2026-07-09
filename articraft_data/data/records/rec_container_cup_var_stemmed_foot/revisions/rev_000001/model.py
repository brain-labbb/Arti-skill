from __future__ import annotations

# Stemmed footed coffee cup: tapered bowl on a lathe-turned pedestal
# (narrow stem + flared foot), with white snap-on sip lid and hinged drink flap.
#
# Frame: cup axis along +Z. Foot base at z=0, rim opens upward at z=RIM_TOP_Z.
# Pedestal (foot + stem) carries the bowl. Lid lifts off (+Z prismatic).
# Sip flap flips up/back (revolute, child of lid).

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

# ---- Pedestal dimensions (meters) ----
FOOT_H = 0.012             # foot height
FOOT_R = 0.040             # foot outer radius (80 mm diameter base)
STEM_H = 0.058             # stem height
STEM_R = 0.008             # stem radius (16 mm diameter)
PEDISTAL_Z = FOOT_H + STEM_H  # 0.070  z of pedestal-bowl junction

# ---- Bowl dimensions ----
BOWL_H = 0.120             # bowl wall height
BOWL_BASE_R = 0.024        # outer radius at bowl bottom
BOWL_RIM_R = 0.045         # outer radius at rim (~90 mm top diameter)
WALL_T = 0.0018            # wall thickness
N_RIBS = 48                # vertical ribs around the bowl

RIM_TOP_Z = PEDISTAL_Z + BOWL_H  # 0.190  z of rim plane
RIM_R = BOWL_RIM_R               # alias for lid/flap geometry

# ---- Lid dimensions (unchanged) ----
LID_SKIRT_H = 0.018        # skirt wraps down over the rim
LID_OVERLAP = 0.008        # vertical overlap of skirt onto bowl wall
LID_DOME_H = 0.020         # raised dome height above rim plane

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


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _cup_body() -> cq.Workplane:
    """Lathe-revolved pedestal (foot+stem) united with hollow tapered bowl."""
    # Pedestal: half-profile on XZ plane, revolved around global Z axis.
    ped = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(FOOT_R, 0.0)
        .lineTo(FOOT_R, FOOT_H * 0.18)
        .lineTo(FOOT_R * 0.88, FOOT_H * 0.40)
        .lineTo(FOOT_R * 0.60, FOOT_H * 0.65)
        .lineTo(STEM_R + 0.008, FOOT_H * 0.85)
        .lineTo(STEM_R + 0.003, FOOT_H * 0.95)
        .lineTo(STEM_R, FOOT_H + 0.008)
        .lineTo(STEM_R, PEDISTAL_Z - 0.010)
        .lineTo(STEM_R + 0.004, PEDISTAL_Z - 0.006)
        .lineTo(STEM_R + 0.010, PEDISTAL_Z - 0.003)
        .lineTo(BOWL_BASE_R, PEDISTAL_Z)
        .lineTo(0.0, PEDISTAL_Z)
        .close()
        .revolve(360, (0, 0), (0, 1))
    )

    # Bowl outer shell (starts 1 mm below pedestal top for reliable union)
    bowl_outer = (
        cq.Workplane("XY")
        .workplane(offset=PEDISTAL_Z - 0.001)
        .circle(BOWL_BASE_R)
        .workplane(offset=BOWL_H * 0.5 + 0.001)
        .circle(BOWL_BASE_R + (BOWL_RIM_R - BOWL_BASE_R) * 0.5)
        .workplane(offset=BOWL_H * 0.5)
        .circle(BOWL_RIM_R)
        .loft(ruled=False)
    )

    # Rolled rim bead at the lip
    bead = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - 0.012)
        .circle(BOWL_RIM_R)
        .workplane(offset=0.010)
        .circle(BOWL_RIM_R + 0.0016)
        .workplane(offset=0.004)
        .circle(BOWL_RIM_R + 0.0016)
        .loft(ruled=False)
    )

    body = ped.union(bowl_outer).union(bead)

    # Bowl inner cavity (hollow, open top)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=PEDISTAL_Z + WALL_T)
        .circle(BOWL_BASE_R - WALL_T)
        .workplane(offset=BOWL_H * 0.5)
        .circle(BOWL_BASE_R + (BOWL_RIM_R - BOWL_BASE_R) * 0.5 - WALL_T)
        .workplane(offset=BOWL_H * 0.5 + 0.006)
        .circle(BOWL_RIM_R - WALL_T)
        .loft(ruled=False)
    )

    return body.cut(inner)


def _bowl_radius_at(z: float) -> float:
    """Linear approximation of the bowl outer radius at height z."""
    t = max(0.0, min(1.0, (z - PEDISTAL_Z) / BOWL_H))
    return BOWL_BASE_R + (BOWL_RIM_R - BOWL_BASE_R) * t


def _one_rib(i: int, z_lo: float, z_hi: float) -> cq.Workplane:
    """Single vertical rib on the bowl surface."""
    ang = 2.0 * math.pi * i / N_RIBS
    r_lo = _bowl_radius_at(z_lo) + 0.0006
    r_hi = _bowl_radius_at(z_hi) + 0.0006
    x_lo, y_lo = r_lo * math.cos(ang), r_lo * math.sin(ang)
    x_hi, y_hi = r_hi * math.cos(ang), r_hi * math.sin(ang)
    return (
        cq.Workplane("XY")
        .workplane(offset=z_lo)
        .center(x_lo, y_lo)
        .circle(0.0011)
        .workplane(offset=(z_hi - z_lo))
        .center(x_hi - x_lo, y_hi - y_lo)
        .circle(0.0011)
        .loft(ruled=True)
    )


def _cup_ribs() -> cq.Workplane:
    """Vertical ribs around the bowl portion of the cup."""
    z_lo = PEDISTAL_Z + 0.012
    z_hi = RIM_TOP_Z - 0.012
    result = None
    for i in range(N_RIBS):
        seg = _one_rib(i, z_lo, z_hi)
        result = seg if result is None else result.union(seg)
    return result


def _lid_solid() -> cq.Workplane:
    """White snap-on sip lid: skirt, shoulder, dome, sip-hole recess."""
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


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stemmed_coffee_cup")

    ceramic = model.material("ceramic_cream", rgba=(0.92, 0.87, 0.78, 1.0))
    ceramic_dark = model.material("ceramic_ribs", rgba=(0.80, 0.73, 0.63, 1.0))
    white = model.material("lid_white", rgba=(0.93, 0.93, 0.92, 1.0))

    # ---- Cup (root): pedestal + bowl + ribs ----
    cup = model.part("cup")
    cup.visual(
        mesh_from_cadquery(_cup_body(), "cup_body"),
        material=ceramic,
        name="cup_body",
    )
    cup.visual(
        mesh_from_cadquery(_cup_ribs(), "cup_ribs"),
        material=ceramic_dark,
        name="cup_ribs",
    )
    cup.inertial = Inertial.from_geometry(
        Cylinder(radius=BOWL_RIM_R, length=RIM_TOP_Z),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z * 0.45)),
    )

    # ---- Lid: white snap-on sip lid, prismatic lift ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=white,
        name="lid_shell",
    )
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

    # ---- Sip flap: revolute hinge on lid dome ----
    z_top = RIM_TOP_Z - LID_OVERLAP + LID_SKIRT_H
    hinge_z = z_top + 0.007

    flap = model.part("sip_flap")
    flap.visual(
        mesh_from_geometry(
            CylinderGeometry(0.0025, SPOUT_W * 0.6).rotate_y(math.pi / 2.0),
            "flap_knuckle",
        ),
        material=white,
        name="flap_knuckle",
    )
    flap.visual(
        mesh_from_cadquery(_flap_panel(), "flap_panel"),
        material=white,
        name="flap_panel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((FLAP_W, FLAP_D, FLAP_T + 0.004)),
        mass=0.0008,
        origin=Origin(xyz=(0.0, FLAP_D * 0.5, FLAP_T * 0.5)),
    )
    model.articulation(
        "lid_to_sip_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=flap,
        origin=Origin(xyz=(0.0, HINGE_Y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5, velocity=2.0,
            lower=0.0, upper=math.radians(120.0),
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

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

    # --- Pedestal structure: foot + stem below bowl ---
    cup_aabb = ctx.part_world_aabb(cup)
    cext = _ext(cup_aabb)

    body_aabb = ctx.part_element_world_aabb(cup, elem="cup_body")
    ctx.check(
        "cup body exists",
        body_aabb is not None,
        details=f"cup_body aabb={body_aabb}",
    )
    body_ext = _ext(body_aabb)
    ctx.check(
        "cup body includes pedestal (height > bowl + 0.04 m)",
        body_ext[2] > BOWL_H + 0.04,
        details=f"body height={body_ext[2]:.3f}, bowl_h={BOWL_H:.3f}",
    )
    ctx.check(
        "foot base at ground level",
        abs(body_aabb[0][2]) < 0.005,
        details=f"body min z={body_aabb[0][2]:.4f}",
    )
    ctx.check(
        "foot is substantial (diameter > 0.06 m)",
        body_ext[0] > 0.06,
        details=f"body x extent={body_ext[0]:.3f}",
    )

    # --- Bowl rim and overall height ---
    ctx.check(
        "bowl rim diameter near 0.09 m",
        0.07 < cext[0] < 0.11,
        details=f"cup x extent={cext[0]:.3f}",
    )
    ctx.check(
        "cup is tall (total height > 0.16 m)",
        cext[2] > 0.16,
        details=f"cup height={cext[2]:.3f}",
    )

    # --- Bowl elevated: pedestal extends well below bowl junction ---
    ctx.check(
        "pedestal extends below bowl (foot below junction)",
        body_aabb[0][2] < PEDISTAL_Z * 0.3,
        details=f"body min z={body_aabb[0][2]:.4f}, pedestal_z={PEDISTAL_Z:.3f}",
    )

    # --- Lid seated on rim (snap-on overlap) ---
    ctx.allow_overlap(
        lid, cup,
        elem_a="lid_shell", elem_b="cup_body",
        reason="Lid skirt snaps over the bowl rim bead (snap-on fit).",
    )
    ctx.allow_overlap(
        lid, cup,
        elem_a="lid_shell", elem_b="cup_ribs",
        reason="Lid skirt overlaps top of ribbed bowl wall where it grips.",
    )
    ctx.expect_overlap(
        lid, cup, axes="z", min_overlap=0.004,
        name="lid skirt grips down over the rim",
    )

    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid domed above the rim",
        lid_aabb[1][2] > RIM_TOP_Z + 0.012,
        details=f"lid top z={lid_aabb[1][2]:.3f}, rim z={RIM_TOP_Z:.3f}",
    )

    # --- Lid lifts straight up (prismatic +Z) ---
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
        details=f"rest_top={rest_top:.3f}, lifted_top={lifted_top:.3f}",
    )
    ctx.check(
        "lifted lid clears the bowl rim",
        lifted_bottom > RIM_TOP_Z - 0.006,
        details=f"lifted bottom z={lifted_bottom:.3f}, rim z={RIM_TOP_Z:.3f}",
    )

    # --- Sip flap seated on lid and flips up ---
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

    # --- Flap rides with the lid (child of lid) ---
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
