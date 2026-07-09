from __future__ import annotations

# Futuristic hexagonal-framed sci-fi blast door with radial iris leaves.
#
# Articraft brief:
# - Object: a wall-mounted sci-fi gate, ~1.86 m wide x ~2.05 m tall x ~0.22 m
#   deep, standing on the floor (geometry from z=0 upward). Faces +Y; width
#   along X. The chamfered-hexagon opening is a true through-opening (no back
#   panel), so the open gate reveals empty space behind it.
# - Root/support: the grey frame slab with a hex-beveled opening, the dark
#   top/bottom guide bars spanning the opening, the cyan light-strip assemblies
#   flanking the opening, the recessed top band, chamfer trim strips, frame
#   bolts, the two dark clamp/latch blocks at mid-height, a central iris hub,
#   and 8 radial guide rails on the frame face. All FIXED on the frame part.
# - Parts: frame (root); leaf_0..leaf_7, eight 45-degree pie-wedge leaves
#   arranged radially around the opening center, each sliding outward along
#   its own radial prismatic axis into the surrounding bulkhead.
# - Articulations: leaf_0_slide (PRISMATIC, +radial) drives leaf_0 outward;
#   leaf_1_slide..leaf_7_slide mimic-couple at 1:1 so one travel value opens
#   all eight leaves symmetrically into radial pockets.
# - Visible geometry: hex opening cut into the frame, glowing twin cyan strips
#   on dark backing channels, 8 dark armored pie-wedge leaves with hazard
#   yellow arc bands near the outer edge, a dark central hub, radial guide
#   rails, dark guide bars, clamp blocks with status lamps, chamfer trim, bolts.
# - Support/fit: each leaf inner edge is captured by the central hub; each leaf
#   slides along a radial guide rail mounted on the frame face.
# - Intentional overlaps: leaf inner edges embed into the hub (allowed + proven);
#   hazard stripes seat a few mm into their own leaf face (allowed + proven).
# - Tests: 8 leaves exist, closed pose covers the central opening, open pose
#   retracts all leaves radially outward, leaves stay within frame bounds,
#   strips/clamps/hub/rails stay fixed, frame stands on the floor.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------
FRAME_W = 1.86          # overall width along X (wide enough for side pockets)
FRAME_H = 2.05          # overall height along Z (stands on z=0)
FRAME_D = 0.22          # frame slab depth along Y
FRAME_FACE_Y = FRAME_D / 2.0  # front face plane (+Y)

# Hexagonal opening cut through the frame (chamfered top/bottom corners)
OPEN_W = 1.18           # opening width (flat-side to flat-side) along X
OPEN_H = 1.74           # opening height along Z
OPEN_CHAMFER = 0.30     # how far the angled corner cuts run
OPEN_CY = 1.04          # opening vertical center (z)
OPEN_HW = OPEN_W / 2.0  # 0.59
OPEN_TOP = OPEN_CY + OPEN_H / 2.0
OPEN_BOT = OPEN_CY - OPEN_H / 2.0

# Leaf geometry
N_LEAVES = 8
LEAF_SPAN_DEG = 360.0 / N_LEAVES        # 45 degrees per leaf
LEAF_GAP_DEG = 1.2                       # angular clearance between leaves
LEAF_HALF_SPAN_DEG = (LEAF_SPAN_DEG - LEAF_GAP_DEG) / 2.0  # 21.9 degrees
LEAF_OUTER_R = 0.55                      # outer radius of each wedge
LEAF_INNER_R = 0.05                      # inner radius (hub capture zone)
LEAF_THICKNESS = 0.05                    # leaf thickness along Y
A_FRONT_Y = 0.065                        # front face Y of the leaves
LEAF_TRAVEL = 0.36                       # radial outward travel (0.55+0.36=0.91 < 0.93)

# Central iris hub
HUB_R = 0.08                             # hub disk radius
HUB_DEPTH = LEAF_THICKNESS + 0.025       # slightly thicker than leaves

# Fixed guide bars (the chunky dark bars across the opening in the reference).
BAR_HALF_LEN = 0.90     # reaches into the frame on both sides
BAR_Y_LO = -0.08        # encloses both sliding planes
BAR_Y_HI = 0.075
TOP_BAR_Z = (1.78, 1.92)   # covers the top apex
BOT_BAR_Z = (0.16, 0.30)   # covers the bottom apex


def _hex_opening_profile(w: float, h: float, chamf: float) -> list[tuple[float, float]]:
    """Return a centered elongated-hexagon outline (chamfered top/bottom)."""
    hw = w / 2.0
    hh = h / 2.0
    cz = h / 2.0 - chamf  # where the vertical sides end and the chamfer begins
    return [
        (-hw, -cz),
        (-hw, cz),
        (-hw + chamf, hh),
        (hw - chamf, hh),
        (hw, cz),
        (hw, -cz),
        (hw - chamf, -hh),
        (-hw + chamf, -hh),
    ]


def _frame_mesh():
    """Grey frame slab with a chamfered-hexagon opening cut through it."""
    slab = cq.Workplane("XY").box(FRAME_W, FRAME_D, FRAME_H, centered=(True, True, False))
    pts = _hex_opening_profile(OPEN_W, OPEN_H, OPEN_CHAMFER)
    # Cut the opening through the full depth at the opening center height.
    cutter = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(FRAME_D * 2.0, both=True)
        .translate((0.0, 0.0, OPEN_CY))
    )
    frame = slab.cut(cutter)
    # Soften the front outer edges so it reads as a beveled armored frame.
    frame = frame.edges("|Y").edges(">Y").chamfer(0.012)
    return frame


def _guide_bar_mesh(z_lo: float, z_hi: float):
    """A fixed dark guide bar spanning the opening; captures the leaf edges."""
    bar = (
        cq.Workplane("XY")
        .box(2.0 * BAR_HALF_LEN, BAR_Y_HI - BAR_Y_LO, z_hi - z_lo, centered=(True, True, True))
        .translate((0.0, (BAR_Y_LO + BAR_Y_HI) / 2.0, (z_lo + z_hi) / 2.0))
    )
    return bar


def _top_recess_mesh():
    """Dark recessed band across the top of the frame face (above the opening)."""
    band_lo = OPEN_TOP + 0.02
    band_hi = FRAME_H - 0.04
    cz = (band_lo + band_hi) / 2.0
    h = band_hi - band_lo
    band = (
        cq.Workplane("XY")
        .box(OPEN_W - 0.10, 0.05, h, centered=(True, True, True))
        # Seat it a touch into the front face so it is a recess on the frame.
        .translate((0.0, FRAME_FACE_Y - 0.024, cz))
    )
    return band


def _clamp_block_mesh(sign: float):
    """Dark mechanical clamp/latch block at mid-height on one side of the opening."""
    x = sign * (OPEN_HW + 0.085)
    body = (
        cq.Workplane("XY")
        .box(0.18, 0.16, 0.26, centered=(True, True, True))
        .edges("|Y").fillet(0.02)
        .translate((x, FRAME_FACE_Y + 0.02, OPEN_CY))
    )
    # A short cylindrical actuator boss on the front face of the block (long axis +Y).
    pin = (
        cq.Workplane("XY")
        .cylinder(0.07, 0.04, centered=(True, True, True))
        .rotate((0, 0, 0), (1, 0, 0), 90.0)  # long axis from Z to Y
        .translate((x, FRAME_FACE_Y + 0.10, OPEN_CY))
    )
    return body.union(pin)


# ---------------------------------------------------------------------------
# Leaf geometry helpers (shared for all 8 radial pie-wedge leaves)
# ---------------------------------------------------------------------------

def _leaf_wedge_mesh(index: int):
    """One pie-wedge leaf in part-local frame (origin = opening center at Z=0).

    The sector spans from LEAF_INNER_R to LEAF_OUTER_R, centered on the leaf's
    angular position. Built on the XZ workplane (u=X, v=Z) and extruded in -Y.
    """
    theta_center = math.radians(index * LEAF_SPAN_DEG)
    half_span = math.radians(LEAF_HALF_SPAN_DEG)
    a0 = theta_center - half_span
    a1 = theta_center + half_span

    n_arc = 14
    pts: list[tuple[float, float]] = []
    # Inner arc (from a0 to a1)
    for k in range(n_arc + 1):
        a = a0 + (a1 - a0) * k / n_arc
        pts.append((LEAF_INNER_R * math.cos(a), LEAF_INNER_R * math.sin(a)))
    # Outer arc (reversed, from a1 to a0)
    for k in range(n_arc + 1):
        a = a1 - (a1 - a0) * k / n_arc
        pts.append((LEAF_OUTER_R * math.cos(a), LEAF_OUTER_R * math.sin(a)))

    wedge = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(LEAF_THICKNESS)
    )
    # Extrude goes from Y=0 to Y=-LEAF_THICKNESS; shift so front face is at A_FRONT_Y
    wedge = wedge.translate((0.0, A_FRONT_Y, 0.0))
    return wedge


def _leaf_stripe_mesh(index: int):
    """Hazard-yellow arc band near the outer edge of one pie-wedge leaf.

    A thin annular sector sitting proud of the leaf front face, inset from the
    outer edge. Seats a few mm into the leaf for visual embed.
    """
    theta_center = math.radians(index * LEAF_SPAN_DEG)
    half_span = math.radians(LEAF_HALF_SPAN_DEG * 0.75)  # narrower than the leaf
    a0 = theta_center - half_span
    a1 = theta_center + half_span
    r0 = LEAF_OUTER_R - 0.10   # inner radius of stripe
    r1 = LEAF_OUTER_R - 0.02   # outer radius of stripe
    band_t = 0.012

    n_arc = 10
    pts: list[tuple[float, float]] = []
    for k in range(n_arc + 1):
        a = a0 + (a1 - a0) * k / n_arc
        pts.append((r0 * math.cos(a), r0 * math.sin(a)))
    for k in range(n_arc + 1):
        a = a1 - (a1 - a0) * k / n_arc
        pts.append((r1 * math.cos(a), r1 * math.sin(a)))

    stripe = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(band_t)
    )
    # Seat the stripe proud of the leaf face with a small embed
    embed = 0.005
    stripe = stripe.translate((0.0, A_FRONT_Y - embed + band_t, 0.0))
    return stripe


def _leaf_rib_mesh(index: int):
    """Raised radial rib along the centerline of one leaf (darker accent)."""
    theta = math.radians(index * LEAF_SPAN_DEG)
    rib_len = LEAF_OUTER_R - LEAF_INNER_R - 0.06
    rib_w = 0.018
    rib_h = 0.010

    mid_r = (LEAF_INNER_R + LEAF_OUTER_R) / 2.0
    # Build a thin box along X then rotate to the radial direction
    rib = cq.Workplane("XY").box(rib_len, rib_w, rib_h, centered=(True, True, True))
    # Rotate around Y by -theta so +X aligns with (cos theta, 0, sin theta)
    rib = rib.rotate((0, 0, 0), (0, 1, 0), -math.degrees(theta))
    # Position at the midpoint along the radial direction, on the leaf front face
    rib = rib.translate((
        mid_r * math.cos(theta),
        A_FRONT_Y + rib_h / 2.0,
        mid_r * math.sin(theta),
    ))
    return rib


def _hub_mesh():
    """Central iris hub boss at the opening center; captures leaf inner edges
    and extends forward through the frame face to connect with the guide rails.

    The hub is a cylindrical boss that protrudes from the frame front face,
    capturing the leaf inner edges behind and providing a visible mounting
    point for the radial guide rails on the frame face.
    """
    y_back = A_FRONT_Y - LEAF_THICKNESS - 0.008   # behind the leaves
    y_front = FRAME_FACE_Y + 0.014                 # proud of the frame face
    depth = y_front - y_back
    hub = (
        cq.Workplane("XZ")
        .circle(HUB_R)
        .extrude(depth)
        .translate((0.0, y_front, OPEN_CY))
    )
    return hub


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scifi_hex_door")

    grey = model.material("frame_grey", rgba=(0.85, 0.86, 0.88, 1.0))
    panel_grey = model.material("hex_panel_grey", rgba=(0.40, 0.42, 0.45, 1.0))
    cyan = model.material("glow_cyan", rgba=(0.45, 0.98, 1.0, 1.0))
    clamp_dark = model.material("clamp_charcoal", rgba=(0.09, 0.10, 0.11, 1.0))
    armor = model.material("door_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    hazard_yellow = model.material("hazard_yellow", rgba=(1.0, 0.86, 0.04, 1.0))
    status_red = model.material("status_red", rgba=(0.85, 0.16, 0.12, 1.0))
    rib_dark = model.material("rib_dark", rgba=(0.12, 0.13, 0.14, 1.0))

    # ---- Fixed root: frame + guide bars + light strips + clamps + trim ----
    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_frame_mesh(), "frame_slab"), material=grey, name="frame_slab")
    # No back panel: the hex opening is a true through-opening.
    frame.visual(
        mesh_from_cadquery(_top_recess_mesh(), "top_recess"),
        material=clamp_dark,
        name="top_recess",
    )
    frame.visual(
        mesh_from_cadquery(_guide_bar_mesh(*TOP_BAR_Z), "top_track"),
        material=clamp_dark,
        name="top_track",
    )
    frame.visual(
        mesh_from_cadquery(_guide_bar_mesh(*BOT_BAR_Z), "bottom_track"),
        material=clamp_dark,
        name="bottom_track",
    )
    frame.visual(
        mesh_from_cadquery(_clamp_block_mesh(-1.0), "clamp_0"),
        material=clamp_dark,
        name="clamp_0",
    )
    frame.visual(
        mesh_from_cadquery(_clamp_block_mesh(1.0), "clamp_1"),
        material=clamp_dark,
        name="clamp_1",
    )
    # Red status lamp on each clamp block front face.
    for s, lamp in ((-1.0, "clamp_lamp_0"), (1.0, "clamp_lamp_1")):
        frame.visual(
            Box((0.05, 0.012, 0.03)),
            origin=Origin(xyz=(s * (OPEN_HW + 0.085), FRAME_FACE_Y + 0.106, OPEN_CY + 0.09)),
            material=status_red,
            name=lamp,
        )

    # Twin glowing cyan strips on a dark backing channel, embedded into each
    # inner jamb edge of the opening (in front of the sliding leaf planes).
    strip_h = 1.16
    backing_w = 0.055
    backing_cx = OPEN_HW + 0.01 - backing_w / 2.0  # embeds 0.01 into the jamb
    for s, tag in ((-1.0, "0"), (1.0, "1")):
        frame.visual(
            Box((backing_w, 0.023, strip_h)),
            origin=Origin(xyz=(s * backing_cx, 0.0835, OPEN_CY)),
            material=clamp_dark,
            name=f"strip_backing_{tag}",
        )
        # Twin glow strips, both seated on the backing channel face.
        frame.visual(
            Box((0.018, 0.012, strip_h - 0.06)),
            origin=Origin(xyz=(s * (backing_cx - 0.0155), 0.101, OPEN_CY)),
            material=cyan,
            name=f"light_strip_{tag}",
        )
        frame.visual(
            Box((0.018, 0.012, strip_h - 0.06)),
            origin=Origin(xyz=(s * (backing_cx + 0.0125), 0.101, OPEN_CY)),
            material=cyan,
            name=f"light_strip_{tag}_inner",
        )

    # Dark trim strips along the four opening chamfers, proud on the frame face.
    chamf_mid_x = OPEN_HW - OPEN_CHAMFER / 2.0  # 0.44
    chamf_top_z = OPEN_TOP - OPEN_CHAMFER / 2.0  # 1.76
    chamf_bot_z = OPEN_BOT + OPEN_CHAMFER / 2.0  # 0.32
    chamf_len = OPEN_CHAMFER * math.sqrt(2.0) - 0.02
    for sx in (-1.0, 1.0):
        for cz, sz, vtag in ((chamf_top_z, 1.0, "t"), (chamf_bot_z, -1.0, "b")):
            # Pitch the trim so its long axis follows the 45-degree chamfer:
            # R_y(p) maps +X to (cos p, 0, -sin p), so the top-right chamfer
            # (running (+1, -1) in XZ) needs p = +45 deg.
            pitch = sx * sz * math.pi / 4.0
            frame.visual(
                Box((chamf_len, 0.012, 0.05)),
                origin=Origin(
                    xyz=(sx * (chamf_mid_x + 0.035), FRAME_FACE_Y + 0.006, cz + sz * 0.035),
                    rpy=(0.0, pitch, 0.0),
                ),
                material=clamp_dark,
                name=f"chamfer_trim_{'l' if sx < 0 else 'r'}{vtag}",
            )

    # Bolt studs flush on the frame front face along the outer margins.
    for sx in (-1.0, 1.0):
        for bz, btag in ((0.30, "lo"), (1.04, "mid"), (1.78, "hi")):
            frame.visual(
                Box((0.03, 0.012, 0.03)),
                origin=Origin(xyz=(sx * 0.80, FRAME_FACE_Y + 0.006, bz)),
                material=clamp_dark,
                name=f"bolt_{'l' if sx < 0 else 'r'}_{btag}",
            )

    # ---- Central iris hub (fixed, on the frame) ----
    frame.visual(
        mesh_from_cadquery(_hub_mesh(), "hub"),
        material=clamp_dark,
        name="hub",
    )

    # ---- Radial guide rails on the frame face ----
    # Each rail starts slightly inside the hub (structural overlap) and extends
    # outward past the leaf travel range into the frame pocket.
    for i in range(N_LEAVES):
        theta_i = math.radians(i * LEAF_SPAN_DEG)
        rail_inner_r = HUB_R - 0.015  # embeds into hub for structural connection
        rail_outer_r = LEAF_OUTER_R + LEAF_TRAVEL + 0.03
        rail_len = rail_outer_r - rail_inner_r
        mid_r = (rail_inner_r + rail_outer_r) / 2.0
        frame.visual(
            Box((rail_len, 0.020, 0.012)),
            origin=Origin(
                xyz=(
                    mid_r * math.cos(theta_i),
                    FRAME_FACE_Y + 0.006,
                    OPEN_CY + mid_r * math.sin(theta_i),
                ),
                rpy=(0.0, -theta_i, 0.0),
            ),
            material=clamp_dark,
            name=f"rail_{i}",
        )

    # ---- 8 radial pie-wedge iris leaves ----
    leaves = []
    for i in range(N_LEAVES):
        leaf_part = model.part(f"leaf_{i}")
        leaf_part.visual(
            mesh_from_cadquery(_leaf_wedge_mesh(i), f"wedge_{i}"),
            material=armor,
            name="wedge",
        )
        leaf_part.visual(
            mesh_from_cadquery(_leaf_stripe_mesh(i), f"stripe_{i}"),
            material=hazard_yellow,
            name="stripe",
        )
        leaf_part.visual(
            mesh_from_cadquery(_leaf_rib_mesh(i), f"rib_{i}"),
            material=rib_dark,
            name="rib",
        )
        leaves.append(leaf_part)

    # ---- Articulations: radial prismatic slides (one driver + mimics) ----
    driver_joint = None
    for i in range(N_LEAVES):
        theta_i = math.radians(i * LEAF_SPAN_DEG)
        axis = (math.cos(theta_i), 0.0, math.sin(theta_i))

        art = model.articulation(
            f"leaf_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=leaves[i],
            origin=Origin(xyz=(0.0, 0.0, OPEN_CY)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=900.0, velocity=0.5, lower=0.0, upper=LEAF_TRAVEL
            ),
        )
        if i == 0:
            driver_joint = art
        else:
            # Mimic the driver at 1:1 so all leaves retract in unison
            art.mimic = Mimic(joint=driver_joint.name, multiplier=1.0, offset=0.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    leaves = [object_model.get_part(f"leaf_{i}") for i in range(N_LEAVES)]
    driver = object_model.get_articulation("leaf_0_slide")

    # --- Intentional local overlaps ---
    # Each leaf inner edge is captured by the central hub.
    for i in range(N_LEAVES):
        ctx.allow_overlap(
            leaves[i],
            frame,
            elem_a="wedge",
            elem_b="hub",
            reason=f"Leaf {i} inner edge is captured inside the central iris hub.",
        )
    # Each hazard stripe seats a few mm into its own leaf face.
    for i in range(N_LEAVES):
        ctx.allow_overlap(
            leaves[i],
            leaves[i],
            elem_a="stripe",
            elem_b="wedge",
            reason="Hazard-stripe arc band is seated into the leaf front face.",
        )
    # Each rib sits proud on the leaf face but may slightly embed.
    for i in range(N_LEAVES):
        ctx.allow_overlap(
            leaves[i],
            leaves[i],
            elem_a="rib",
            elem_b="wedge",
            reason="Raised radial rib is mounted onto the leaf front face.",
        )

    # --- Frame geometry present ---
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame stands on the floor and reaches full height",
        frame_aabb is not None
        and abs(frame_aabb[0][2]) < 0.02
        and frame_aabb[1][2] > FRAME_H - 0.05,
        details=f"frame aabb={frame_aabb}",
    )

    # Cyan light strips flank the opening on the +Y face.
    strip0 = ctx.part_element_world_aabb(frame, elem="light_strip_0")
    strip1 = ctx.part_element_world_aabb(frame, elem="light_strip_1")
    ctx.check(
        "cyan light strips flank the opening left and right",
        strip0 is not None
        and strip1 is not None
        and strip0[1][0] < 0.0 < strip1[0][0],
        details=f"strip0={strip0}, strip1={strip1}",
    )

    # Clamp blocks sit at mid-height, proud of the frame front face.
    clamp0 = ctx.part_element_world_aabb(frame, elem="clamp_0")
    ctx.check(
        "clamp block sits at mid-height proud of the front face",
        clamp0 is not None
        and clamp0[1][1] > FRAME_FACE_Y
        and clamp0[0][2] < OPEN_CY < clamp0[1][2],
        details=f"clamp0={clamp0}",
    )

    # Central hub present at the opening center.
    hub_aabb = ctx.part_element_world_aabb(frame, elem="hub")
    ctx.check(
        "central iris hub is at the opening center",
        hub_aabb is not None
        and hub_aabb[0][2] < OPEN_CY < hub_aabb[1][2]
        and hub_aabb[0][0] < 0.0 < hub_aabb[1][0],
        details=f"hub aabb={hub_aabb}",
    )

    # --- 8 radial leaves present ---
    ctx.check(
        "8 radial pie-wedge leaves present",
        len(leaves) == N_LEAVES,
        details=f"found {len(leaves)} leaves",
    )

    # The doorway is a true through-opening: no fixed frame furniture blocks
    # the central window of the opening.
    window_x = 0.35
    window_z = (0.55, 1.50)
    for elem in ("top_recess", "top_track", "bottom_track", "strip_backing_0", "strip_backing_1"):
        aabb = ctx.part_element_world_aabb(frame, elem=elem)
        assert aabb is not None
        blocks = (
            aabb[0][0] < window_x
            and aabb[1][0] > -window_x
            and aabb[0][2] < window_z[1]
            and aabb[1][2] > window_z[0]
        )
        ctx.check(
            f"frame '{elem}' stays clear of the open doorway window",
            not blocks,
            details=f"{elem} aabb={aabb}",
        )

    # --- Closed pose: leaves cover the central opening ---
    closed_positions: dict[str, tuple[float, float, float]] = {}
    with ctx.pose({driver: 0.0}):
        # Each leaf overlaps with the hub region (inner edges captured)
        for i in range(N_LEAVES):
            ctx.expect_overlap(
                leaves[i],
                frame,
                axes="xy",
                elem_a="wedge",
                elem_b="hub",
                min_overlap=0.01,
                name=f"leaf_{i} inner edge overlaps the hub when closed",
            )
        # Adjacent leaves meet at their shared radial edges (overlap in XY footprint)
        for i in range(N_LEAVES):
            j = (i + 1) % N_LEAVES
            ctx.expect_overlap(
                leaves[i],
                leaves[j],
                axes="xy",
                elem_a="wedge",
                elem_b="wedge",
                min_overlap=0.001,
                name=f"leaf_{i} and leaf_{j} share a radial edge when closed",
            )
        # Collect rest positions
        for i in range(N_LEAVES):
            pos = ctx.part_world_position(leaves[i])
            assert pos is not None
            closed_positions[f"leaf_{i}"] = pos

    # --- Open pose: all leaves retract radially outward ---
    half_frame = FRAME_W / 2.0
    with ctx.pose({driver: LEAF_TRAVEL}):
        for i in range(N_LEAVES):
            theta_i = math.radians(i * LEAF_SPAN_DEG)
            pos = ctx.part_world_position(leaves[i])
            assert pos is not None
            closed_pos = closed_positions[f"leaf_{i}"]

            # Radial displacement from rest
            dx = pos[0] - closed_pos[0]
            dz = pos[2] - closed_pos[2]
            radial_disp = dx * math.cos(theta_i) + dz * math.sin(theta_i)

            ctx.check(
                f"leaf_{i} retracts radially outward when open",
                radial_disp > LEAF_TRAVEL * 0.85,
                details=f"radial_disp={radial_disp:.4f}, expected>{LEAF_TRAVEL * 0.85:.4f}",
            )

            # Leaf stays within frame bounds
            aabb = ctx.part_world_aabb(leaves[i])
            assert aabb is not None
            ctx.check(
                f"leaf_{i} stays within frame silhouette when open",
                aabb[0][0] >= -half_frame - 0.01
                and aabb[1][0] <= half_frame + 0.01
                and aabb[0][2] >= -0.01
                and aabb[1][2] <= FRAME_H + 0.01,
                details=f"leaf_{i} open aabb=({aabb[0]}, {aabb[1]})",
            )

        # Central passage is clear when open: adjacent leaves have separated
        leaf0_aabb = ctx.part_world_aabb(leaves[0])
        leaf4_aabb = ctx.part_world_aabb(leaves[4])
        assert leaf0_aabb is not None and leaf4_aabb is not None
        ctx.check(
            "opposing leaves separate to clear a central passage",
            leaf4_aabb[1][0] < -0.15 and leaf0_aabb[0][0] > 0.15,
            details=f"leaf0 x=({leaf0_aabb[0][0]:.3f}, {leaf0_aabb[1][0]:.3f}), "
                    f"leaf4 x=({leaf4_aabb[0][0]:.3f}, {leaf4_aabb[1][0]:.3f})",
        )

    return ctx.report()


object_model = build_object_model()
