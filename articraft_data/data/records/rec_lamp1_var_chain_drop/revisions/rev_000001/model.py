from __future__ import annotations

# Exterior wall lantern (galvanized / weathered-zinc finish) — CHAIN-DROP variant.
#
# The object is a wall-mounted carriage lantern:
#   - a decorative leaf / fleur-shaped wall mounting plate bolted to the wall,
#   - a curved scroll "gooseneck" bracket arm that sweeps up and out, then
#     curls back down into a hook,
#   - a hanging lantern suspended from that hook by a multi-link chain drop
#     (repeated interlocking torus loop links), a finial cap, a wide flared
#     conical sheet-metal roof (the shade), a translucent glass cylinder caged
#     by vertical metal straps and horizontal ring bands, and a bottom retaining
#     ring with a small drip finial.
#
# Real mechanism: the lantern hangs from the hook on its chain and physically
# SWINGS like a pendulum in the wind. That swing is the single real
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

# Hook eye location (where the chain / lantern hangs). In bracket frame.
HOOK_Y = 0.235           # outward distance of the hook eye from the wall
HOOK_Z = 0.060           # height of the hook eye above the plate center
HOOK_R = 0.012           # hook stock radius
HOOK_RING_R = 0.026      # radius of the hook curl

# Chain drop: N repeated interlocking torus loop links forming a longer
# suspension chain between the hook curl and the lantern finial.
CHAIN_LINK_R = 0.018         # link major radius (torus center radius)
CHAIN_LINK_TUBE = 0.004      # link wire radius
N_CHAIN_LINKS = 5            # number of repeated chain links
CHAIN_PITCH = 2.0 * CHAIN_LINK_R - CHAIN_LINK_TUBE  # center-to-center vertical spacing

# Lantern: the child part. Authored in its OWN local frame whose origin is the
# pendulum pivot (hook eye). The lantern hangs DOWN from there.

# Finial cap (small turned dome on top of the roof).
FINIAL_R = 0.018
FINIAL_H = 0.030

# Roof / shade: wide flared cone of galvanized sheet metal.
ROOF_TOP_R = 0.040       # small radius where it meets the finial neck
ROOF_BOT_R = 0.140       # wide flared eave radius
ROOF_H = 0.110           # roof slant height (Z)

# Glass body: translucent cylinder under the roof eave.
GLASS_R = 0.082
GLASS_H = 0.150

# Cage: vertical straps and horizontal ring bands around the glass (rust/iron).
STRAP_N = 6
STRAP_W = 0.012
STRAP_T = 0.006
BAND_N = 3
BAND_T = 0.010           # band radial thickness (proud of glass)
BAND_H = 0.014

# Bottom retaining ring + small drip finial.
BOTTOM_RING_R = GLASS_R + 0.006
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

    A near-circular open curl in the Y-Z plane that the chain hangs in.
    Centered on the hook eye, opening toward the wall so the top chain link
    sits captured.
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
# Chain link 0 (top) threads up into the hook curl so it interlinks the hook
# (cross-part support). Each successive link interlinks the previous one, with
# alternating ring-plane orientation (even links in XZ, odd links in YZ).
# The finial neck threads up through the last chain link (captured loop).
_LINK_0_TOP_Z = -0.018                              # link 0 top arc rests on hook stock
_LINK_0_CZ = _LINK_0_TOP_Z - CHAIN_LINK_R           # link 0 ring center


def _chain_link_cz(index: int) -> float:
    """Center-Z of chain link `index` in the lantern local frame."""
    return _LINK_0_CZ - index * CHAIN_PITCH


_LAST_LINK_CZ = _chain_link_cz(N_CHAIN_LINKS - 1)
_LAST_LINK_BOT_Z = _LAST_LINK_CZ - CHAIN_LINK_R
# The finial neck threads UP through the last chain-link bottom (captured loop).
_FINIAL_TOP_Z = _LAST_LINK_BOT_Z + 0.012
_FINIAL_BASE_Z = _FINIAL_TOP_Z - FINIAL_H
_ROOF_TOP_Z = _FINIAL_BASE_Z + 0.006
_ROOF_BOT_Z = _ROOF_TOP_Z - ROOF_H
_GLASS_TOP_Z = _ROOF_TOP_Z - 0.020
_GLASS_BOT_Z = _GLASS_TOP_Z - GLASS_H
_BOTTOM_RING_TOP_Z = _GLASS_BOT_Z + 0.004
_BOTTOM_RING_BOT_Z = _BOTTOM_RING_TOP_Z - BOTTOM_RING_H


def _chain_link_shape(index: int) -> cq.Workplane:
    """A single chain-link torus at its computed Z, with alternating orientation.

    Even-indexed links have their ring in the XZ plane (hole along Y) so they
    interlock with the hook curl (which lies in the YZ plane). Odd-indexed links
    have their ring in the YZ plane (hole along X) so they interlock with the
    even links above and below them — like real chain.
    """
    cz = _chain_link_cz(index)
    link = cq.Workplane("XY").add(cq.Solid.makeTorus(CHAIN_LINK_R, CHAIN_LINK_TUBE))
    if index % 2 == 0:
        # Ring in XZ plane (hole along Y) — perpendicular to hook curl.
        link = link.rotate((0, 0, 0), (1, 0, 0), 90.0)
    else:
        # Ring in YZ plane (hole along X) — perpendicular to even links.
        link = link.rotate((0, 0, 0), (0, 1, 0), 90.0)
    return link.translate((0.0, 0.0, cz))


def _finial_shape() -> cq.Workplane:
    """Turned finial / collar cap that crowns the roof and holds the last link.

    The base is a wide collar flange that caps and seats over the roof's neck
    (so the finial connects to the roof), rising through a turned bulb to a thin
    neck stub at the top that the last chain link captures.
    """
    base_r = ROOF_TOP_R * 0.6 + 0.008    # collar flange wider than the roof neck
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
    """Wide flared conical sheet-metal roof (the shade) as a thin shell."""
    # Outer cone profile + inner (offset) profile -> a closed thin conical
    # sheet. Profile is (radius, z) with z = 0 at the eave.
    out = [
        (ROOF_BOT_R, 0.0),
        (ROOF_TOP_R, ROOF_H),
        (ROOF_TOP_R * 0.6, ROOF_H + 0.012),  # small upturned neck collar
    ]
    inn = [
        (ROOF_TOP_R * 0.6 - 0.004, ROOF_H + 0.012),
        (ROOF_TOP_R - 0.004, ROOF_H),
        (ROOF_BOT_R - 0.010, 0.0),
    ]
    roof = _lathe_z(out + inn)
    return roof.translate((0.0, 0.0, _ROOF_BOT_Z))


def _glass_shape() -> cq.Workplane:
    """Translucent glass body cylinder."""
    glass = (
        cq.Workplane("XY")
        .circle(GLASS_R)
        .extrude(GLASS_H)
    )
    return glass.translate((0.0, 0.0, _GLASS_BOT_Z))


def _cage_shape() -> cq.Workplane:
    """Vertical iron straps + horizontal ring bands caging the glass body.

    The straps run top-to-bottom of the glass; the bands wrap around at the
    top, middle, and bottom. One unioned iron piece that hugs the glass.
    """
    cage = None
    # Vertical straps, evenly spaced, proud of the glass surface.
    strap_h = GLASS_H + 0.010
    rr = GLASS_R + STRAP_T / 2.0 - 0.001
    for i in range(STRAP_N):
        a = 2.0 * math.pi * i / STRAP_N
        strap = (
            cq.Workplane("XY")
            .box(STRAP_T, STRAP_W, strap_h, centered=(True, True, True))
            .translate((rr, 0.0, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
            .translate((0.0, 0.0, _GLASS_BOT_Z + GLASS_H / 2.0))
        )
        cage = strap if cage is None else cage.union(strap)
    # Horizontal ring bands (thin tube rings) at top, mid, bottom of glass.
    band_zs = [
        _GLASS_BOT_Z + GLASS_H - 0.012,
        _GLASS_BOT_Z + GLASS_H / 2.0,
        _GLASS_BOT_Z + 0.012,
    ]
    for z in band_zs[:BAND_N]:
        ring = (
            cq.Workplane("XY")
            .circle(GLASS_R + BAND_T)
            .circle(GLASS_R - 0.001)
            .extrude(BAND_H)
            .translate((0.0, 0.0, z - BAND_H / 2.0))
        )
        cage = ring if cage is None else cage.union(ring)
    return cage


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
    model = ArticulatedObject(name="exterior_wall_lantern_chain_drop")

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

    # --- LANTERN (swinging child): chain links + finial + roof + glass + cage + base ---
    lantern = model.part("lantern")

    # Repeated chain links (for-loop with name_i naming, shared geometry helper).
    for i in range(N_CHAIN_LINKS):
        lantern.visual(
            mesh_from_cadquery(_chain_link_shape(i), f"chain_link_{i}",
                               tolerance=TOL, angular_tolerance=ATOL),
            material="zinc_dark",
            name=f"chain_link_{i}",
        )

    lantern.visual(
        mesh_from_cadquery(_finial_shape(), "finial",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="zinc",
        name="finial",
    )
    lantern.visual(
        mesh_from_cadquery(_roof_shape(), "roof",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="zinc",
        name="roof",
    )
    lantern.visual(
        mesh_from_cadquery(_glass_shape(), "glass",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="glass",
        name="glass",
    )
    lantern.visual(
        mesh_from_cadquery(_cage_shape(), "cage",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="iron_rust",
        name="cage",
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
    # Top chain link (link 0) interlinks through the bracket hook curl — two
    # rings threaded through each other like real chain.
    ctx.allow_overlap(
        lantern, bracket,
        elem_a="chain_link_0", elem_b="bracket_hook",
        reason="Chain link 0 is interlinked through the hook curl so the lantern "
               "hangs captured from the bracket (real chain-link capture).",
    )
    # Adjacent chain links interlink with each other (alternating orientation).
    for i in range(N_CHAIN_LINKS - 1):
        ctx.allow_overlap(
            lantern, lantern,
            elem_a=f"chain_link_{i}", elem_b=f"chain_link_{i + 1}",
            reason=f"Chain link {i} and link {i + 1} are interlinked like real "
                   "chain (alternating-plane torus capture).",
        )
    # Last chain link captures the finial neck (finial threads through it).
    last_name = f"chain_link_{N_CHAIN_LINKS - 1}"
    ctx.allow_overlap(
        lantern, lantern,
        elem_a=last_name, elem_b="finial",
        reason="Finial neck threads through the last chain link so the link "
               "captures the lantern's top loop (real suspension capture).",
    )

    # Prove the chain capture: contacts along the suspension path.
    ctx.expect_contact(
        lantern, bracket, elem_a="chain_link_0", elem_b="bracket_hook",
        contact_tol=0.001, name="chain link 0 interlinks the hook",
    )
    for i in range(N_CHAIN_LINKS - 1):
        ctx.expect_contact(
            lantern, lantern,
            elem_a=f"chain_link_{i}", elem_b=f"chain_link_{i + 1}",
            contact_tol=0.002, name=f"chain link {i} interlinks link {i + 1}",
        )
    ctx.expect_contact(
        lantern, lantern, elem_a=last_name, elem_b="finial",
        contact_tol=0.002, name="last chain link captures the finial",
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

    # Lantern hero geometry: wide flared roof above a narrower glass body, and a
    # bottom retaining ring below the glass.
    roof_aabb = ctx.part_element_world_aabb(lantern, elem="roof")
    glass_aabb = ctx.part_element_world_aabb(lantern, elem="glass")
    ring_aabb = ctx.part_element_world_aabb(lantern, elem="bottom_ring")
    ctx.check(
        "roof is wider than the glass body (flared shade)",
        roof_aabb is not None and glass_aabb is not None
        and (roof_aabb[1][0] - roof_aabb[0][0]) > (glass_aabb[1][0] - glass_aabb[0][0]) + 0.06,
        details=f"roof_w={None if roof_aabb is None else roof_aabb[1][0]-roof_aabb[0][0]}, "
                f"glass_w={None if glass_aabb is None else glass_aabb[1][0]-glass_aabb[0][0]}",
    )
    ctx.check(
        "roof sits above the glass body",
        roof_aabb is not None and glass_aabb is not None
        and roof_aabb[1][2] > glass_aabb[1][2],
        details=f"roof_top_z={None if roof_aabb is None else roof_aabb[1][2]}, "
                f"glass_top_z={None if glass_aabb is None else glass_aabb[1][2]}",
    )
    ctx.check(
        "bottom ring is below the glass body",
        ring_aabb is not None and glass_aabb is not None
        and ring_aabb[0][2] < glass_aabb[0][2] + 0.002,
        details=f"ring_bot_z={None if ring_aabb is None else ring_aabb[0][2]}, "
                f"glass_bot_z={None if glass_aabb is None else glass_aabb[0][2]}",
    )

    # Cage straps hug the glass.
    ctx.expect_overlap(
        lantern, lantern,
        axes="xy",
        elem_a="cage",
        elem_b="glass",
        min_overlap=GLASS_R,
        name="cage wraps the glass body",
    )

    # Roof eave hangs over the glass.
    ctx.expect_within(
        lantern, lantern,
        axes="xy",
        inner_elem="glass",
        outer_elem="roof",
        margin=0.001,
        name="glass body sits under the roof eave",
    )

    # --- Chain drop: repeated links span from hook to finial. ---
    # Verify all N chain links exist and are vertically ordered (link 0 highest,
    # link N-1 lowest).
    link_aabbs = []
    for i in range(N_CHAIN_LINKS):
        aabb = ctx.part_element_world_aabb(lantern, elem=f"chain_link_{i}")
        link_aabbs.append(aabb)
    ctx.check(
        f"all {N_CHAIN_LINKS} chain links present",
        all(a is not None for a in link_aabbs),
        details=f"aabbs_none={[a is None for a in link_aabbs]}",
    )
    if all(a is not None for a in link_aabbs):
        # Link 0 top should be near hook height.
        ctx.check(
            "chain link 0 hangs from the hook eye",
            abs(link_aabbs[0][1][2] - HOOK_Z) < 0.02,
            details=f"link0_top_z={link_aabbs[0][1][2]}, hook_z={HOOK_Z}",
        )
        # Links are vertically ordered: each successive link center is lower.
        link_centers = [(a[0][2] + a[1][2]) / 2.0 for a in link_aabbs]
        ordered = all(link_centers[i] > link_centers[i + 1] for i in range(N_CHAIN_LINKS - 1))
        ctx.check(
            "chain links descend vertically (link 0 highest)",
            ordered,
            details=f"centers={link_centers}",
        )
        # Last link is directly above the finial.
        finial_aabb = ctx.part_element_world_aabb(lantern, elem="finial")
        ctx.check(
            "finial is directly below the last chain link",
            finial_aabb is not None
            and finial_aabb[1][2] <= link_aabbs[-1][1][2] + 0.001,
            details=f"finial_top_z={None if finial_aabb is None else finial_aabb[1][2]}, "
                    f"last_link_top_z={link_aabbs[-1][1][2]}",
        )
        # Chain drop is visibly longer than a single link: total chain vertical
        # span should exceed 2 * CHAIN_LINK_R (the diameter of one link).
        chain_span = link_aabbs[0][1][2] - link_aabbs[-1][0][2]
        ctx.check(
            "chain drop is longer than a single link diameter",
            chain_span > 3.0 * CHAIN_LINK_R,
            details=f"chain_span={chain_span:.4f}, single_diameter={2*CHAIN_LINK_R:.4f}",
        )

    # --- Mechanism: actuating the swing moves the lantern as a pendulum. ---
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

    return ctx.report()


object_model = build_object_model()
