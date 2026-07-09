from __future__ import annotations

# Exterior wall lantern (galvanized / weathered-zinc finish).
#
# The object is a wall-mounted carriage lantern:
#   - a decorative leaf / fleur-shaped wall mounting plate bolted to the wall,
#   - a curved scroll "gooseneck" bracket arm that sweeps up and out, then
#     curls back down into a hook,
#   - a hanging lantern suspended from that hook by a chain link: a finial cap,
#     a wide flared conical sheet-metal roof (the shade), a translucent glass
#     cylinder caged by vertical metal straps and horizontal ring bands, and a
#     bottom retaining ring with a small drip finial.
#
# Real mechanism: the lantern hangs from the hook on its chain link and
# physically SWINGS like a pendulum in the wind. That swing is the single real
# articulation (REVOLUTE about a horizontal axis parallel to the wall, located
# at the hook eye). Everything in the bracket (plate + scroll arm + hook) is one
# rigid cast-metal assembly and is the fixed root.
#
# Convention (real meters, Z-up):
#   wall plane is X-Z at y = 0; the bracket extends out into +Y away from wall.
#   X -> horizontal across the wall (lateral / swing axis is along Y... see below)
#   +Y -> outward from the wall
#   +Z -> up
# The pendulum swing axis is X (a horizontal line lying in the wall plane), so
# the lantern swings toward/away from the wall (in the Y-Z plane).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------

# Wall mounting plate (decorative leaf / fleur silhouette).
PLATE_H = 0.230          # plate height (Z)
PLATE_W = 0.120          # plate width (X)
PLATE_T = 0.014          # plate thickness (Y) standing off the wall
PLATE_ZC = 0.000         # plate vertical center (its own local Z origin)

# Scroll bracket arm (swept tube). Leaves the top of the plate, sweeps out and
# up, then curls forward and back down to the hook eye well in front of the wall.
ARM_R = 0.012            # arm tube radius

# Hook eye location (where the chain link / lantern hangs). In bracket frame.
HOOK_Y = 0.235           # outward distance of the hook eye from the wall
HOOK_Z = 0.060           # height of the hook eye above the plate center
HOOK_R = 0.012           # hook stock radius
HOOK_RING_R = 0.026      # radius of the hook curl

# Chain link connecting hook eye to lantern top. Made tall enough that its top
# arc threads the hook curl while its bottom arc drops clear below the hook to
# capture the finial loop without the finial fouling the hook.
LINK_R = 0.034           # link major radius (torus center radius)
LINK_TUBE = 0.006        # link wire radius

# Lantern: the child part. Authored in its OWN local frame whose origin is the
# pendulum pivot (hook eye). The lantern hangs DOWN from there.
LINK_DROP = 0.030        # vertical drop from pivot to top of finial via the link

# Finial cap (small turned dome on top of the roof).
FINIAL_R = 0.018
FINIAL_H = 0.030

# Roof / shade: flat cylindrical cap (cylindrical-cage variant).
CAP_R = 0.100            # cap disk radius (slightly wider than bar cage)
CAP_H = 0.016            # cap disk thickness
CAP_LIP_H = 0.010        # downward lip at cap edge
NECK_R = 0.026           # central neck collar radius for finial
NECK_H = 0.012           # neck collar height above cap top

# Glass body: translucent cylinder inside the bar cage.
GLASS_R = 0.082
GLASS_H = 0.150

# Cage: round vertical bars and horizontal ring bands around the glass.
BAR_N = 8                # number of vertical bars
BAR_R = 0.006            # bar cross-section radius (12 mm dia rods)
BAND_N = 3               # horizontal ring bands
BAND_RING_R = 0.004      # band ring cross-section radius

# Light bulb inside glass.
BULB_R = 0.022           # bulb globe radius
BULB_STEM_R = 0.008      # bulb socket/stem radius
BULB_STEM_H = 0.018      # socket height

# Bottom retaining ring + small drip finial.
BOTTOM_RING_R = GLASS_R + BAR_R * 2 + 0.004
BOTTOM_RING_H = 0.018
DRIP_R = 0.014
DRIP_H = 0.022

# Tessellation.
TOL = 0.0012
ATOL = 0.2


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _lathe_z(prof_rz: list[tuple[float, float]]) -> cq.Workplane:
    """Solid of revolution about the world Z axis.

    `prof_rz` is an ordered list of (radius, z) points describing the half
    profile. It is authored in the XY plane (x = radius, y = z), revolved about
    the world Y axis, then rotated +90 deg about X so the revolution axis maps
    to world Z (an upright lathe). The profile must close back to the axis.
    """
    pts = [(float(r), float(z)) for (r, z) in prof_rz]
    solid = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )
    return solid.rotate((0, 0, 0), (1, 0, 0), 90.0)


# ---------------------------------------------------------------------------
# Geometry builders -- BRACKET (fixed root)
# ---------------------------------------------------------------------------


def _wall_plate_shape() -> cq.Workplane:
    """Decorative leaf / fleur-de-lis style wall mounting plate.

    Built as an extruded silhouette standing off the wall in +Y. The silhouette
    is a tall pointed-top, pointed-bottom leaf with side lobes (the cast scroll
    backplate in the reference). Two countersunk bolt holes anchor it.
    The plate's flat back face is the wall contact at y = 0.
    """
    hw = PLATE_W / 2.0
    hh = PLATE_H / 2.0
    # Closed leaf/fleur silhouette in the X-Z plane (authored as XY profile,
    # later the extrude axis is +Y). Points go counter-clockwise.
    pts = [
        (0.0, hh),                       # top point
        (0.32 * hw, 0.55 * hh),
        (0.95 * hw, 0.62 * hh),          # upper side lobe
        (0.62 * hw, 0.28 * hh),
        (0.85 * hw, 0.0),                # mid side cusp
        (0.62 * hw, -0.28 * hh),
        (0.95 * hw, -0.62 * hh),         # lower side lobe
        (0.32 * hw, -0.55 * hh),
        (0.0, -hh),                      # bottom point
        (-0.32 * hw, -0.55 * hh),
        (-0.95 * hw, -0.62 * hh),
        (-0.62 * hw, -0.28 * hh),
        (-0.85 * hw, 0.0),
        (-0.62 * hw, 0.28 * hh),
        (-0.95 * hw, 0.62 * hh),
        (-0.32 * hw, 0.55 * hh),
    ]
    plate = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(PLATE_T)
    )
    # The XZ workplane normal is -Y, so extrude(+PLATE_T) spans y in
    # [-PLATE_T, 0]. Mirror across the XZ plane so the plate stands off in +Y
    # with its flat back face at the wall (y = 0) and its front at y = PLATE_T.
    plate = plate.mirror("XZ")
    # Soften the front edges so it reads as a cast plate, not a flat cutout.
    try:
        plate = plate.edges("|Y").fillet(0.004)
    except Exception:
        pass
    # Two bolt holes through the plate body (top and bottom anchor points).
    # The plate now spans y in [0, PLATE_T]; cut a Y-axis cylinder through it.
    for z in (0.55 * hh, -0.55 * hh):
        hole = (
            cq.Workplane("XZ")
            .center(0.0, z)
            .circle(0.006)
            .extrude(-(PLATE_T + 0.02))      # spans y in [0, PLATE_T+0.02]
            .translate((0.0, -0.01, 0.0))    # -> y in [-0.01, PLATE_T+0.01]
        )
        plate = plate.cut(hole)
    return plate


def _scroll_arm_mesh():
    """Curved scroll gooseneck arm swept from the plate top out to the hook.

    A smooth S-curve tube: rises from just above the plate, arcs forward/up,
    crests, then descends forward to the hook eye. Lives in the x = 0 plane and
    is built as a smooth circular tube along the (y, z) centerline.
    """
    path = [
        (0.0, 0.020, 0.080),            # root, just off the plate front near top
        (0.0, 0.070, 0.140),            # rise and forward
        (0.0, 0.140, 0.150),            # crest of the gooseneck
        (0.0, 0.205, 0.120),            # descend forward
        (0.0, HOOK_Y, HOOK_Z + 0.020),  # approach to hook eye top
    ]
    return tube_from_spline_points(
        path,
        radius=ARM_R,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
        up_hint=(1.0, 0.0, 0.0),
    )


def _hook_mesh():
    """Downward hook / curl at the end of the scroll arm.

    A near-circular open curl in the Y-Z plane that the chain link hangs in.
    Centered on the hook eye, opening toward the wall so the link sits captured.
    """
    cy, cz = HOOK_Y, HOOK_Z
    pts = []
    a0 = math.radians(70.0)
    a1 = math.radians(70.0 + 300.0)
    n = 28
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        y = cy + HOOK_RING_R * math.cos(a)
        z = cz + HOOK_RING_R * math.sin(a)
        pts.append((0.0, y, z))
    return tube_from_spline_points(
        pts,
        radius=HOOK_R,
        samples_per_segment=8,
        radial_segments=14,
        cap_ends=True,
        up_hint=(1.0, 0.0, 0.0),
    )


# ---------------------------------------------------------------------------
# Geometry builders -- LANTERN (swinging child)
# ---------------------------------------------------------------------------
# The lantern is authored in its OWN local frame: origin at the pendulum pivot
# (the hook eye). It hangs straight down. The articulation origin will place
# this frame at the hook eye in world space.

# Vertical layout of the lantern in its local frame (origin at pivot, +Z up).
# The hook curl (in the bracket) is centered at the pivot (local z = 0) with
# stock reaching down to roughly local z = -0.038. The chain link's top arc is
# threaded UP into that curl so the link interlinks the hook (cross-part
# support), and its bottom arc meets the finial top (within-part connection).
# Hook curl center is at the pivot (local z = 0); its lower stock sits around
# local z = -(HOOK_RING_R) with stock radius HOOK_R, i.e. roughly local
# z in [-0.038, -0.014]. Put the link's top arc up into that band so the link
# top-inner curve wraps over the hook's lower stock (true ring interlink).
_LINK_TOP_Z = -0.018                        # link top arc rests on the hook stock
_LINK_CZ = _LINK_TOP_Z - LINK_R             # link ring center
_LINK_BOT_Z = _LINK_CZ - LINK_R             # link bottom arc
# The finial neck threads UP through the chain-link bottom (captured loop), so
# the finial top sits clearly above the link bottom arc for a true intersection.
_FINIAL_TOP_Z = _LINK_BOT_Z + 0.014         # finial neck pierces link bottom
_FINIAL_BASE_Z = _FINIAL_TOP_Z - FINIAL_H   # finial base (sits on cap neck)
_CAP_TOP_Z = _FINIAL_BASE_Z + 0.006         # cap top overlaps finial base
_CAP_BOT_Z = _CAP_TOP_Z - CAP_H             # cap disk bottom
_GLASS_TOP_Z = _CAP_BOT_Z - CAP_LIP_H + 0.004  # glass tucks inside cap lip
_GLASS_BOT_Z = _GLASS_TOP_Z - GLASS_H
_BOTTOM_RING_TOP_Z = _GLASS_BOT_Z + 0.004
_BOTTOM_RING_BOT_Z = _BOTTOM_RING_TOP_Z - BOTTOM_RING_H
# Bulb center height (inside glass, roughly at glass vertical center).
_BULB_CZ = _GLASS_BOT_Z + GLASS_H * 0.45


def _chain_link_shape() -> cq.Workplane:
    """A single chain link (torus) interlinking the hook eye and the finial.

    The ring lies in the X-Z plane (perpendicular to the hook curl, which lies
    in the Y-Z plane) so the two rings interlock like real chain links.
    """
    link = (
        cq.Workplane("XY")
        .add(cq.Solid.makeTorus(LINK_R, LINK_TUBE))
    )
    # Default torus ring is in XY (hole along Z). Rotate 90 about X so the ring
    # plane becomes XZ (hole along Y); the link then hangs vertically.
    link = link.rotate((0, 0, 0), (1, 0, 0), 90.0)
    return link.translate((0.0, 0.0, _LINK_CZ))


def _finial_shape() -> cq.Workplane:
    """Turned finial / collar cap that crowns the roof and holds the link.

    The base is a wide collar flange that caps and seats over the roof's neck
    (so the finial connects to the roof), rising through a turned bulb to a thin
    neck stub at the top that the chain link captures.
    """
    base_r = NECK_R + 0.004              # collar flange wider than the cap neck
    # Closed half profile (radius, z), z measured from the finial base up.
    prof = [
        (0.004, 0.0),
        (base_r, 0.002),                 # wide collar flange seats on roof neck
        (base_r, 0.010),
        (FINIAL_R, FINIAL_H * 0.45),     # turned bulb
        (FINIAL_R * 0.45, FINIAL_H * 0.80),
        (0.004, FINIAL_H),               # thin neck stub at top (link capture)
        (0.0, FINIAL_H),
        (0.0, 0.0),
    ]
    finial = _lathe_z(prof)
    return finial.translate((0.0, 0.0, _FINIAL_BASE_Z))


def _roof_shape() -> cq.Workplane:
    """Flat cylindrical cap with a central neck collar and downward edge lip."""
    # Main disk (z = 0 at cap bottom, extends up to CAP_H).
    cap = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .extrude(CAP_H)
    )
    # Downward lip ring at outer edge (hangs below the cap to capture glass top).
    lip = (
        cq.Workplane("XY")
        .circle(CAP_R)
        .circle(CAP_R - 0.008)
        .extrude(CAP_LIP_H)
        .translate((0.0, 0.0, -CAP_LIP_H))
    )
    # Central neck collar rising above the cap (seats the finial base).
    neck = (
        cq.Workplane("XY")
        .circle(NECK_R)
        .circle(NECK_R - 0.006)
        .extrude(CAP_H + NECK_H)
    )
    roof = cap.union(lip).union(neck)
    return roof.translate((0.0, 0.0, _CAP_BOT_Z))


def _glass_shape() -> cq.Workplane:
    """Translucent glass body cylinder."""
    glass = (
        cq.Workplane("XY")
        .circle(GLASS_R)
        .extrude(GLASS_H)
    )
    return glass.translate((0.0, 0.0, _GLASS_BOT_Z))


def _cage_bar_shape(index: int) -> cq.Workplane:
    """Single vertical round cage bar at angular position `index` of BAR_N."""
    bar_h = GLASS_H + 0.008
    a = 2.0 * math.pi * index / BAR_N
    rr = GLASS_R + BAR_R  # bar center sits just outside the glass surface
    bar = (
        cq.Workplane("XY")
        .circle(BAR_R)
        .extrude(bar_h)
        .translate((rr, 0.0, 0.0))
        .rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
        .translate((0.0, 0.0, _GLASS_BOT_Z - 0.004))
    )
    return bar


def _cage_band_shape(index: int) -> cq.Workplane:
    """Single horizontal ring band at vertical position `index` of BAND_N."""
    band_zs = [
        _GLASS_BOT_Z + GLASS_H - 0.010,
        _GLASS_BOT_Z + GLASS_H * 0.5,
        _GLASS_BOT_Z + 0.010,
    ]
    z = band_zs[index]
    outer_r = GLASS_R + BAR_R * 2 + 0.002
    inner_r = GLASS_R - 0.002
    band = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(BAND_RING_R * 2)
        .translate((0.0, 0.0, z - BAND_RING_R))
    )
    return band


def _bulb_shape() -> cq.Workplane:
    """Light bulb (globe + socket mount) inside the glass body, as a single lathe.

    The socket mount extends down through the bottom ring floor plate so the
    bulb is physically supported by the lantern floor (realistic socket mount
    and connected mesh).
    """
    # The floor plate top is at _BOTTOM_RING_BOT_Z (local). Extend the mount
    # base 2 mm into the floor plate for a seated overlap.
    mount_base_local = _BOTTOM_RING_BOT_Z - 0.002  # penetrates into floor plate
    # Globe center in the lantern local frame.
    globe_base = _BULB_CZ - BULB_R + 0.002  # bottom of globe sphere
    stem_top_local = globe_base - 0.003       # top of stem just below globe shoulder
    globe_cz_local = _BULB_CZ                 # globe center z (lantern local)
    globe_top_local = _BULB_CZ + BULB_R

    # Half profile in lantern-local z (no translation needed afterward).
    prof = [
        (0.002, mount_base_local),
        (BULB_STEM_R * 1.4, mount_base_local),           # wide socket base
        (BULB_STEM_R * 1.4, mount_base_local + 0.005),
        (BULB_STEM_R, mount_base_local + 0.008),         # stem shoulder
        (BULB_STEM_R, stem_top_local),
        (BULB_R * 0.75, stem_top_local + 0.003),        # shoulder into globe
        (BULB_R, globe_cz_local),                        # globe equator
        (BULB_R * 0.85, globe_cz_local + BULB_R * 0.65),
        (BULB_R * 0.4, globe_top_local - 0.002),
        (0.002, globe_top_local),
        (0.002, mount_base_local),
    ]
    return _lathe_z(prof)


def _bottom_ring_shape() -> cq.Workplane:
    """Bottom retaining ring + small drip finial under the glass."""
    ring = (
        cq.Workplane("XY")
        .circle(BOTTOM_RING_R)
        .circle(GLASS_R - 0.004)
        .extrude(BOTTOM_RING_H)
        .translate((0.0, 0.0, _BOTTOM_RING_BOT_Z))
    )
    # Closed bottom plate (the lantern floor) capping the glass.
    floor = (
        cq.Workplane("XY")
        .circle(BOTTOM_RING_R)
        .extrude(0.008)
        .translate((0.0, 0.0, _BOTTOM_RING_BOT_Z - 0.008))
    )
    body = ring.union(floor)
    # Small drip finial pointing down (lathe profile, z measured downward).
    drip_prof = [
        (DRIP_R, 0.0),
        (DRIP_R * 0.5, -DRIP_H * 0.6),
        (0.002, -DRIP_H),
        (0.0, -DRIP_H),
        (0.0, 0.0),
    ]
    drip = _lathe_z(drip_prof)
    drip = drip.translate((0.0, 0.0, _BOTTOM_RING_BOT_Z - 0.008))
    return body.union(drip)


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exterior_wall_lantern")

    model.material("zinc", rgba=(0.66, 0.68, 0.69, 1.0))      # galvanized metal
    model.material("zinc_dark", rgba=(0.55, 0.57, 0.59, 1.0))
    model.material("iron_rust", rgba=(0.55, 0.40, 0.34, 1.0))  # reddish straps
    model.material("glass", rgba=(0.62, 0.74, 0.74, 0.45))     # translucent

    # --- BRACKET (fixed root): plate + scroll arm + hook ---
    bracket = model.part("wall_bracket")
    bracket.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="zinc",
        name="wall_plate",
    )
    bracket.visual(
        mesh_from_geometry(_scroll_arm_mesh(), "scroll_arm"),
        material="zinc",
        name="scroll_arm",
    )
    bracket.visual(
        mesh_from_geometry(_hook_mesh(), "bracket_hook"),
        material="zinc",
        name="bracket_hook",
    )

    # --- LANTERN (swinging child): link + finial + roof + glass + cage + base ---
    lantern = model.part("lantern")
    lantern.visual(
        mesh_from_cadquery(_chain_link_shape(), "chain_link",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="zinc_dark",
        name="chain_link",
    )
    lantern.visual(
        mesh_from_cadquery(_finial_shape(), "finial",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="zinc",
        name="finial",
    )
    lantern.visual(
        mesh_from_cadquery(_roof_shape(), "roof_cap",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="zinc",
        name="roof_cap",
    )
    lantern.visual(
        mesh_from_cadquery(_glass_shape(), "glass",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="glass",
        name="glass",
    )
    # Individual vertical cage bars (round rods, name_i pattern).
    for i in range(BAR_N):
        lantern.visual(
            mesh_from_cadquery(_cage_bar_shape(i), f"bar_{i}",
                               tolerance=TOL, angular_tolerance=ATOL),
            material="iron_rust",
            name=f"bar_{i}",
        )
    # Individual horizontal ring bands (name_i pattern).
    for i in range(BAND_N):
        lantern.visual(
            mesh_from_cadquery(_cage_band_shape(i), f"band_{i}",
                               tolerance=TOL, angular_tolerance=ATOL),
            material="iron_rust",
            name=f"band_{i}",
        )
    # Light bulb inside the glass.
    lantern.visual(
        mesh_from_cadquery(_bulb_shape(), "bulb",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="glass",
        name="bulb",
    )
    lantern.visual(
        mesh_from_cadquery(_bottom_ring_shape(), "bottom_ring",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="iron_rust",
        name="bottom_ring",
    )

    # --- ARTICULATION: pendulum swing of the lantern about the hook eye ---
    # The lantern frame origin is the pivot (hook eye). Swing axis is X (a
    # horizontal line in the wall plane), so positive q swings the lantern
    # outward/inward in the Y-Z plane. Limits: a modest wind sway both ways.
    model.articulation(
        "lantern_swing",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=lantern,
        origin=Origin(xyz=(0.0, HOOK_Y, HOOK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.45, upper=0.45),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bracket = object_model.get_part("wall_bracket")
    lantern = object_model.get_part("lantern")
    swing = object_model.get_articulation("lantern_swing")

    # --- Intentional interlinks (chain capture). ---
    # The chain link and the bracket hook are two rings threaded through each
    # other like real chain links; they intentionally interpenetrate so the
    # lantern is captured by and hangs from the hook.
    ctx.allow_overlap(
        lantern, bracket,
        elem_a="chain_link", elem_b="bracket_hook",
        reason="Chain link is interlinked through the hook curl so the lantern "
               "hangs captured from the bracket (real chain-link capture).",
    )
    # The finial neck is threaded up through the bottom of the chain link, so
    # the link captures the finial loop. Small intentional embed.
    ctx.allow_overlap(
        lantern, lantern,
        elem_a="chain_link", elem_b="finial",
        reason="Finial neck threads through the chain link so the link captures "
               "the lantern's top loop (real suspension capture).",
    )
    # The bulb socket mount seats into the bottom ring floor plate (small
    # intentional embed for physical support of the bulb).
    ctx.allow_overlap(
        lantern, lantern,
        elem_a="bulb", elem_b="bottom_ring",
        reason="Bulb socket mount penetrates the bottom ring floor plate to "
               "provide physical support for the bulb (realistic seated socket).",
    )

    # Prove the chain capture: link contacts/interlinks both hook and finial.
    ctx.expect_contact(
        lantern, bracket, elem_a="chain_link", elem_b="bracket_hook",
        contact_tol=0.001, name="chain link interlinks the hook",
    )
    ctx.expect_contact(
        lantern, lantern, elem_a="chain_link", elem_b="finial",
        contact_tol=0.001, name="chain link captures the finial",
    )

    # --- Joint identity: pendulum swing is REVOLUTE about X at the hook eye. ---
    ctx.check(
        "swing is revolute",
        str(swing.articulation_type).lower().endswith("revolute"),
        details=f"type={swing.articulation_type}",
    )
    ax = tuple(round(c, 6) for c in swing.axis)
    ctx.check(
        "swing axis is X (horizontal in wall plane)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )
    jo = swing.origin.xyz
    ctx.check(
        "swing pivot at the hook eye",
        abs(jo[0]) < 1e-6 and abs(jo[1] - HOOK_Y) < 1e-6 and abs(jo[2] - HOOK_Z) < 1e-6,
        details=f"origin={jo}",
    )

    # --- Hero parts present and placed (rest pose). ---
    # Wall plate sits at the wall (y ~ 0..PLATE_T) and is the lowest-Y bracket
    # element; the hook reaches far out in +Y.
    plate_aabb = ctx.part_element_world_aabb(bracket, elem="wall_plate")
    hook_aabb = ctx.part_element_world_aabb(bracket, elem="bracket_hook")
    ctx.check(
        "wall plate mounts at the wall plane (y near 0)",
        plate_aabb is not None and plate_aabb[0][1] <= 0.002 and plate_aabb[1][1] <= PLATE_T + 0.01,
        details=f"plate_aabb={plate_aabb}",
    )
    ctx.check(
        "hook reaches out in front of the wall",
        hook_aabb is not None and hook_aabb[1][1] > HOOK_Y - HOOK_RING_R - 0.01,
        details=f"hook_aabb={hook_aabb}",
    )

    # Lantern hero geometry: flat cylindrical cap above a glass body caged
    # by round vertical bars, and a bottom retaining ring below the glass.
    cap_aabb = ctx.part_element_world_aabb(lantern, elem="roof_cap")
    glass_aabb = ctx.part_element_world_aabb(lantern, elem="glass")
    ring_aabb = ctx.part_element_world_aabb(lantern, elem="bottom_ring")
    bar0_aabb = ctx.part_element_world_aabb(lantern, elem="bar_0")
    bulb_aabb = ctx.part_element_world_aabb(lantern, elem="bulb")
    ctx.check(
        "roof cap is wider than the glass body",
        cap_aabb is not None and glass_aabb is not None
        and (cap_aabb[1][0] - cap_aabb[0][0]) > (glass_aabb[1][0] - glass_aabb[0][0]) - 0.005,
        details=f"cap_w={None if cap_aabb is None else cap_aabb[1][0]-cap_aabb[0][0]}, "
                f"glass_w={None if glass_aabb is None else glass_aabb[1][0]-glass_aabb[0][0]}",
    )
    ctx.check(
        "roof cap sits above the glass body",
        cap_aabb is not None and glass_aabb is not None
        and cap_aabb[1][2] > glass_aabb[1][2],
        details=f"cap_top_z={None if cap_aabb is None else cap_aabb[1][2]}, "
                f"glass_top_z={None if glass_aabb is None else glass_aabb[1][2]}",
    )
    ctx.check(
        "bottom ring is below the glass body",
        ring_aabb is not None and glass_aabb is not None
        and ring_aabb[0][2] < glass_aabb[0][2] + 0.002,
        details=f"ring_bot_z={None if ring_aabb is None else ring_aabb[0][2]}, "
                f"glass_bot_z={None if glass_aabb is None else glass_aabb[0][2]}",
    )

    # Cage bars surround the glass: bar_0 contacts the glass surface (tangent
    # at the +X side), and bars are positioned at the cage radius just outside
    # the glass. Check contact between bar_0 and glass.
    ctx.expect_contact(
        lantern, lantern,
        elem_a="bar_0",
        elem_b="glass",
        contact_tol=BAR_R + 0.002,
        name="cage bar_0 contacts the glass surface",
    )

    # Roof cap covers the glass body from above (cap contains glass on xy).
    ctx.expect_within(
        lantern, lantern,
        axes="xy",
        inner_elem="glass",
        outer_elem="roof_cap",
        margin=0.005,
        name="glass body sits under the roof cap",
    )

    # Bulb is inside the glass (contained in xy and z).
    ctx.expect_within(
        lantern, lantern,
        axes="xy",
        inner_elem="bulb",
        outer_elem="glass",
        margin=0.002,
        name="bulb is inside the glass body (xy)",
    )
    ctx.check(
        "bulb globe top is below the glass top",
        bulb_aabb is not None and glass_aabb is not None
        and bulb_aabb[1][2] < glass_aabb[1][2] + 0.005,
        details=f"bulb_top_z={None if bulb_aabb is None else bulb_aabb[1][2]}, "
                f"glass_top_z={None if glass_aabb is None else glass_aabb[1][2]}",
    )
    ctx.check(
        "bulb mount extends below glass into the floor (socket seated)",
        bulb_aabb is not None and glass_aabb is not None
        and bulb_aabb[0][2] < glass_aabb[0][2] + 0.005,
        details=f"bulb_bot_z={None if bulb_aabb is None else bulb_aabb[0][2]}, "
                f"glass_bot_z={None if glass_aabb is None else glass_aabb[0][2]}",
    )
    # Prove the bulb socket is seated in the bottom ring floor.
    ctx.expect_contact(
        lantern, lantern,
        elem_a="bulb", elem_b="bottom_ring",
        contact_tol=0.005,
        name="bulb socket seats into the bottom ring floor",
    )

    # Bars span the full glass height (tall enough to cage the body).
    ctx.check(
        "cage bar_0 spans the glass height",
        bar0_aabb is not None and glass_aabb is not None
        and bar0_aabb[1][2] >= glass_aabb[1][2] - 0.005
        and bar0_aabb[0][2] <= glass_aabb[0][2] + 0.005,
        details=f"bar0_z={bar0_aabb}, glass_z={glass_aabb}",
    )

    # --- Mechanism: actuating the swing moves the lantern as a pendulum. ---
    # At rest the lantern hangs straight down (low Z). Swinging it should move
    # the bottom of the lantern outward in Y (away from straight-down) and raise
    # it slightly. We compare the bottom-ring world AABB at rest vs swung.
    rest_ring = ctx.part_element_world_aabb(lantern, elem="bottom_ring")
    with ctx.pose({swing: 0.45}):
        swung_ring = ctx.part_element_world_aabb(lantern, elem="bottom_ring")
    rest_yc = None if rest_ring is None else (rest_ring[0][1] + rest_ring[1][1]) / 2.0
    swung_yc = None if swung_ring is None else (swung_ring[0][1] + swung_ring[1][1]) / 2.0
    rest_zc = None if rest_ring is None else (rest_ring[0][2] + rest_ring[1][2]) / 2.0
    swung_zc = None if swung_ring is None else (swung_ring[0][2] + swung_ring[1][2]) / 2.0
    ctx.check(
        "positive swing moves the lantern bottom outward in Y",
        rest_yc is not None and swung_yc is not None and swung_yc > rest_yc + 0.05,
        details=f"rest_yc={rest_yc}, swung_yc={swung_yc}",
    )
    ctx.check(
        "swinging raises the lantern bottom (pendulum arc)",
        rest_zc is not None and swung_zc is not None and swung_zc > rest_zc + 0.01,
        details=f"rest_zc={rest_zc}, swung_zc={swung_zc}",
    )

    # Chain link bridges the pivot (hook) and the finial: at rest the link top
    # is near the pivot height (HOOK_Z) and the finial sits just below it.
    link_aabb = ctx.part_element_world_aabb(lantern, elem="chain_link")
    finial_aabb = ctx.part_element_world_aabb(lantern, elem="finial")
    ctx.check(
        "chain link hangs from the hook eye",
        link_aabb is not None and abs(link_aabb[1][2] - HOOK_Z) < 0.02,
        details=f"link_top_z={None if link_aabb is None else link_aabb[1][2]}, hook_z={HOOK_Z}",
    )
    ctx.check(
        "finial is directly below the chain link",
        link_aabb is not None and finial_aabb is not None
        and finial_aabb[1][2] <= link_aabb[1][2] + 0.001,
        details=f"finial_top_z={None if finial_aabb is None else finial_aabb[1][2]}, "
                f"link_top_z={None if link_aabb is None else link_aabb[1][2]}",
    )

    return ctx.report()


object_model = build_object_model()
