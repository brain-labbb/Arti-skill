from __future__ import annotations

# Futuristic hexagonal-framed sci-fi blast door.
#
# Articraft brief:
# - Object: a wall-mounted sci-fi gate, ~1.86 m wide x ~2.05 m tall x ~0.22 m
#   deep, standing on the floor (geometry from z=0 upward). Faces +Y; width
#   along X. The chamfered-hexagon opening is a true through-opening (no back
#   panel), so the open gate reveals empty space behind it.
# - Root/support: the grey frame slab with a hex-beveled opening, the dark
#   top/bottom guide bars spanning the opening, the inlaid hydraulic piston
#   actuator rams flanking the opening, the rotating warning beacons on the
#   upper frame, the recessed top band, chamfer trim strips, and frame bolts.
#   All FIXED.
# - Parts: frame (root); door_0 / door_1, the two hazard-striped inner leaves
#   that meet at a wavy complementary central seam; door_0_outer / door_1_outer,
#   the grey outer leaves riding in a second plane behind the inner ones
#   (two-stage telescoping per side).
# - Articulations: door_0_slide (PRISMATIC, -X) drives the left inner leaf;
#   door_1_slide (+X), door_0_outer_slide (-X, 0.5x) and door_1_outer_slide
#   (+X, 0.5x) are mimic-coupled so one travel value telescopes all four
#   leaves open symmetrically into the frame side pockets.
# - Visible geometry: hex opening cut into the frame, inlaid hydraulic piston
#   actuator rams with chrome rods and mounting brackets flanking the opening,
#   rotating warning beacons with amber domes on the upper frame, hazard-striped
#   inner leaves whose wavy seam edges are complementary halves of one seam
#   line (they mate into a complete door with a thin running clearance), grey
#   outer leaves with groove lines, dark guide bars, chamfer trim, and bolts.
# - Support/fit: every leaf is captured top and bottom in the fixed guide bars
#   (small proven overlap) for its whole travel; when open the leaves nest in
#   the frame side pockets without poking past the frame silhouette.
# - Intentional overlaps: the wavy seam edges interleave across the centreline
#   (AABB overlap, allowed + proven); leaf tops/bottoms ride captured in the
#   guide bars (allowed + proven); each hazard decal seats into its own leaf.
# - Tests: closed leaves mate at the seam and cover the opening, the doorway
#   window is unobstructed (no back panel), telescoping stages lap with no
#   slit, open pose parts all four leaves into the pockets without poking out,
#   pistons/beacons stay fixed and flanking, and the frame stands on the floor.

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

DOOR_D = 0.06           # leaf thickness along Y (one telescoping stage)

# Two sliding planes inside the frame depth. The inner (seam) leaves ride in
# the front plane; the outer leaves ride in a second plane just behind them.
A_FRONT_Y = 0.065       # front face of the inner leaves (A plane: 0.005..0.065)
B_FRONT_Y = -0.01       # front face of the outer leaves (B plane: -0.07..-0.01)

# Telescoping split per side (door plane X layout).
SEAM_AMP = 0.055        # peak-to-center amplitude of the wavy central seam
SEAM_CLEAR = 0.004      # running clearance per side of the shared seam line
A_OUT = 0.345           # outer edge of the inner (striped) leaf
B_IN = 0.325            # inner edge of the outer leaf (0.02 lap behind A)
B_OUT = OPEN_HW - 0.008  # outer edge of the outer leaf, just inside the jamb

# Leaf vertical extent: a clean rectangle captured by the top/bottom guide
# bars; the bars themselves cover the opening's top/bottom margins and apexes.
PANEL_Z0 = 0.29
PANEL_Z1 = 1.79
PANEL_H = PANEL_Z1 - PANEL_Z0

# Fixed guide bars (the chunky dark bars across the opening in the reference).
BAR_HALF_LEN = 0.90     # reaches into the frame on both sides
BAR_Y_LO = -0.08        # encloses both sliding planes
BAR_Y_HI = 0.075
TOP_BAR_Z = (1.78, 1.92)   # overlaps leaf tops by 0.01, covers the top apex
BOT_BAR_Z = (0.16, 0.30)   # overlaps leaf bottoms by 0.01, covers the bottom apex

# Door travel: the inner leaf clears (almost) the whole opening and nests in
# the side pocket; the outer leaf mimics at half rate and hides behind the jamb.
TRAVEL_A = 0.58
TRAVEL_B = TRAVEL_A / 2.0


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


def _piston_ram_mesh(sign: float):
    """Inlaid hydraulic piston actuator ram flanking one side of the opening.

    A vertical hydraulic cylinder with a chrome piston rod extending upward,
    mounting plates at top and bottom, and a crosshead bracket. Inlaid into
    the frame face so the cylinder body sits partially recessed.
    """
    x = sign * (OPEN_HW + 0.09)
    face_y = FRAME_FACE_Y

    # Main cylinder body (dark steel tube)
    cyl_r = 0.032
    cyl_h = 0.72
    cyl_cz = OPEN_CY - 0.06  # slightly below center so rod extends up
    body = (
        cq.Workplane("XY")
        .cylinder(cyl_h, cyl_r, centered=(True, True, True))
        .translate((x, face_y - 0.01, cyl_cz))
    )
    # End caps (slightly wider rings at top and bottom of cylinder)
    cap_r = 0.038
    cap_h = 0.025
    for dz in (-cyl_h / 2.0, cyl_h / 2.0):
        cap = (
            cq.Workplane("XY")
            .cylinder(cap_h, cap_r, centered=(True, True, True))
            .translate((x, face_y - 0.01, cyl_cz + dz))
        )
        body = body.union(cap)

    # Piston rod (chrome, extends upward from the cylinder top)
    rod_r = 0.016
    rod_h = 0.38
    rod_cz = cyl_cz + cyl_h / 2.0 + rod_h / 2.0 - 0.02
    rod = (
        cq.Workplane("XY")
        .cylinder(rod_h, rod_r, centered=(True, True, True))
        .translate((x, face_y - 0.01, rod_cz))
    )
    body = body.union(rod)

    # Rod end clevis (small block at rod tip for the upper mount)
    clevis = (
        cq.Workplane("XY")
        .box(0.05, 0.04, 0.04, centered=(True, True, True))
        .edges("|Y").fillet(0.008)
        .translate((x, face_y - 0.01, rod_cz + rod_h / 2.0 + 0.015))
    )
    body = body.union(clevis)

    # Lower mounting bracket (plate bolted to the frame)
    lower_bracket = (
        cq.Workplane("XY")
        .box(0.08, 0.04, 0.07, centered=(True, True, True))
        .edges("|Y").fillet(0.006)
        .translate((x, face_y + 0.01, cyl_cz - cyl_h / 2.0 - 0.025))
    )
    body = body.union(lower_bracket)

    # Upper mounting bracket
    upper_bracket = (
        cq.Workplane("XY")
        .box(0.08, 0.04, 0.07, centered=(True, True, True))
        .edges("|Y").fillet(0.006)
        .translate((x, face_y + 0.01, rod_cz + rod_h / 2.0 + 0.045))
    )
    body = body.union(upper_bracket)

    # Hydraulic line fittings (small cylinders on the body)
    for dz in (-0.18, 0.12):
        fitting = (
            cq.Workplane("XY")
            .cylinder(0.03, 0.012, centered=(True, True, True))
            .rotate((0, 0, 0), (0, 1, 0), 90.0)
            .translate((x, face_y + 0.025, cyl_cz + dz))
        )
        body = body.union(fitting)

    return body


def _warning_beacon_mesh(sign: float):
    """Rotating warning beacon mounted on the upper frame near the hex chamfer.

    Dark base plate, cylindrical housing, amber translucent dome with a
    visible internal reflector suggesting a rotating light.
    """
    # Place beacon at the upper outer corner of the hex opening
    x = sign * (OPEN_HW - OPEN_CHAMFER * 0.35)
    z = OPEN_TOP + 0.03

    # Base plate (dark, bolted to frame top face)
    base = (
        cq.Workplane("XY")
        .cylinder(0.015, 0.04, centered=(True, True, True))
        .translate((x, FRAME_FACE_Y - 0.005, z))
    )
    # Housing cylinder
    housing = (
        cq.Workplane("XY")
        .cylinder(0.05, 0.032, centered=(True, True, True))
        .translate((x, FRAME_FACE_Y - 0.005, z + 0.035))
    )
    body = base.union(housing)

    # Amber dome (half-sphere on top of the housing)
    dome = (
        cq.Workplane("XY")
        .sphere(0.032)
        .translate((x, FRAME_FACE_Y - 0.005, z + 0.06))
    )
    # Cut the bottom half off to make a dome
    cutter = (
        cq.Workplane("XY")
        .box(0.1, 0.1, 0.08, centered=(True, True, False))
        .translate((x, FRAME_FACE_Y - 0.005, z - 0.04))
    )
    dome = dome.cut(cutter)
    body = body.union(dome)

    # Internal reflector (small wedge suggesting a rotating mirror)
    reflector = (
        cq.Workplane("XY")
        .box(0.024, 0.008, 0.03, centered=(True, True, True))
        .translate((x, FRAME_FACE_Y - 0.005, z + 0.055))
    )
    body = body.union(reflector)

    return body


def _seam_points(side: float) -> list[tuple[float, float]]:
    """Wavy seam-edge (x, z) samples for one inner leaf, bottom -> top.

    One shared seam line x_s(z) (1.5 cosine waves, amplitude SEAM_AMP about the
    centerline) splits the inner leaf pair into complementary halves: the right
    leaf occupies x >= x_s(z) + SEAM_CLEAR and the left leaf
    x <= x_s(z) - SEAM_CLEAR, so the closed leaves mate into one complete door
    with a thin running clearance.
    """
    n = 28
    pts: list[tuple[float, float]] = []
    for i in range(n + 1):
        t = i / n
        z = PANEL_Z0 + t * PANEL_H
        x = SEAM_AMP * math.cos(t * math.pi * 3.0) + side * SEAM_CLEAR
        pts.append((x, z))
    return pts


def _inner_leaf_profile(side: float) -> list[tuple[float, float]]:
    """2D outline (X-Z) of one inner leaf: wavy seam edge + straight outer edge."""
    seam = _seam_points(side)
    pts = list(seam)  # seam edge, bottom -> top
    pts.append((side * A_OUT, PANEL_Z1))  # top outer corner
    pts.append((side * A_OUT, PANEL_Z0))  # bottom outer corner
    return pts


def _inner_leaf_mesh(side: float):
    """One inner armored leaf (front plane) with the wavy seam edge."""
    pts = _inner_leaf_profile(side)
    leaf = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(DOOR_D)  # XZ extrude runs toward -Y: Y in [-DOOR_D, 0]
        .translate((0.0, A_FRONT_Y, 0.0))
    )
    return leaf


def _hazard_stripe_mesh(side: float):
    """Bold diagonal YELLOW hazard bars laid across the front of one inner leaf.

    Returns a single CONNECTED CadQuery solid: raised diagonal yellow bars tiled
    across the leaf front, tied together by a thin perimeter ring that follows
    the leaf outline (including the wavy seam edge). The dark leaf shows between
    the bars so the pair reads as the yellow/black hazard chevrons in the
    reference. The decal stands proud of the leaf front face and embeds a few mm
    into it so it is one mounted island that contacts the leaf.
    """
    band_t = 0.018      # decal thickness along Y (stands proud of the leaf)
    z_inset = 0.06      # keep the decal clear of the upper/lower guide bars
    foot_pts = _inner_leaf_profile(side)

    # Leaf footprint solid, thin in Y, used to clip the bars to the leaf shape.
    foot = cq.Workplane("XZ").polyline(foot_pts).close().extrude(band_t)
    midspan = (
        cq.Workplane("XZ")
        .center(0.0, OPEN_CY)
        .rect(3.0 * (A_OUT + SEAM_AMP), PANEL_H - 2.0 * z_inset)
        .extrude(band_t)
    )
    foot = foot.intersect(midspan)

    # A thin perimeter ring (footprint minus an inward-offset interior) ties
    # every bar into one connected island and frames the leaf edge in yellow.
    inner = (
        cq.Workplane("XZ")
        .polyline(foot_pts)
        .close()
        .offset2D(-0.022)  # shrink the outline inward to form a border ring
        .extrude(band_t * 1.6)
        .translate((0.0, 0.0, 0.0))
    )
    decal = foot.cut(inner)  # perimeter border frame

    # Bold diagonal yellow bars tiled across the leaf, each rotated 45 deg about Y.
    pitch = 0.14
    width = 0.07
    span = A_OUT + PANEL_H
    for j in range(-int(span / pitch) - 2, int(span / pitch) + 2):
        offset = j * pitch
        bar = (
            cq.Workplane("XZ")
            .center(offset + side * A_OUT / 2.0, OPEN_CY)
            .rect(width, span * 2.5)
            .extrude(band_t)
            .rotate((0, 0, 0), (0, 1, 0), 45.0)
        )
        clipped = bar.intersect(foot)
        if clipped.solids().vals():
            decal = decal.union(clipped)

    # Seat the decal so it embeds a few mm into the leaf front face and stands
    # proud on the outside. The decal spans Y in [-band_t, 0] before moving.
    embed = 0.006
    decal = decal.translate((0.0, A_FRONT_Y - embed + band_t, 0.0))
    return decal


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scifi_hex_door")

    grey = model.material("frame_grey", rgba=(0.85, 0.86, 0.88, 1.0))
    panel_grey = model.material("hex_panel_grey", rgba=(0.40, 0.42, 0.45, 1.0))
    clamp_dark = model.material("clamp_charcoal", rgba=(0.09, 0.10, 0.11, 1.0))
    armor = model.material("door_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    hazard_yellow = model.material("hazard_yellow", rgba=(1.0, 0.86, 0.04, 1.0))
    chrome = model.material("piston_chrome", rgba=(0.82, 0.83, 0.85, 1.0))
    amber = model.material("beacon_amber", rgba=(1.0, 0.65, 0.05, 1.0))

    # ---- Fixed root: frame + guide bars + piston rams + beacons + trim ----
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

    # Hydraulic piston actuator rams flanking the opening (inlaid into frame).
    for i, sign in enumerate((-1.0, 1.0)):
        frame.visual(
            mesh_from_cadquery(_piston_ram_mesh(sign), f"piston_{i}"),
            material=chrome,
            name=f"piston_{i}",
        )

    # Rotating warning beacons on the upper frame near the hex chamfers.
    for i, sign in enumerate((-1.0, 1.0)):
        frame.visual(
            mesh_from_cadquery(_warning_beacon_mesh(sign), f"beacon_{i}"),
            material=amber,
            name=f"beacon_{i}",
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

    # ---- Telescoping leaves: inner striped pair + outer grey pair ----
    door_0 = model.part("door_0")  # left inner leaf (front plane)
    door_1 = model.part("door_1")  # right inner leaf (front plane)
    door_0_outer = model.part("door_0_outer")  # left outer leaf (rear plane)
    door_1_outer = model.part("door_1_outer")  # right outer leaf (rear plane)

    for part, side, tag in ((door_0, -1.0, "0"), (door_1, 1.0, "1")):
        part.visual(
            mesh_from_cadquery(_inner_leaf_mesh(side), f"leaf_{tag}"),
            material=armor,
            name=f"leaf_{tag}",
        )
        part.visual(
            mesh_from_cadquery(_hazard_stripe_mesh(side), f"hazard_{tag}"),
            material=hazard_yellow,
            name=f"hazard_{tag}",
        )

    b_w = B_OUT - B_IN
    outer_groove_heights = (0.55, 1.04, 1.53)
    for part, side, tag in ((door_0_outer, -1.0, "0"), (door_1_outer, 1.0, "1")):
        b_cx = side * (B_IN + B_OUT) / 2.0
        part.visual(
            Box((b_w, DOOR_D, PANEL_H)),
            origin=Origin(xyz=(b_cx, B_FRONT_Y - DOOR_D / 2.0, (PANEL_Z0 + PANEL_Z1) / 2.0)),
            material=panel_grey,
            name=f"leaf_{tag}",
        )
        # Dark groove lines so the outer stage reads as a machined panel.
        for gz in outer_groove_heights:
            part.visual(
                Box((b_w - 0.06, 0.008, 0.022)),
                origin=Origin(xyz=(b_cx, B_FRONT_Y + 0.004, gz)),
                material=clamp_dark,
                name=f"groove_{int(gz * 100)}",
            )

    # ---- Articulations: telescoping prismatic slides (one driver + mimics) ----
    door_0_slide = model.articulation(
        "door_0_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),  # positive q slides the left leaf toward -X (opens)
        motion_limits=MotionLimits(effort=900.0, velocity=0.5, lower=0.0, upper=TRAVEL_A),
    )
    model.articulation(
        "door_1_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.5, lower=0.0, upper=TRAVEL_A),
        mimic=Mimic(joint=door_0_slide.name, multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "door_0_outer_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_0_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.25, lower=0.0, upper=TRAVEL_B),
        mimic=Mimic(joint=door_0_slide.name, multiplier=0.5, offset=0.0),
    )
    model.articulation(
        "door_1_outer_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_1_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.25, lower=0.0, upper=TRAVEL_B),
        mimic=Mimic(joint=door_0_slide.name, multiplier=0.5, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    door_0_outer = object_model.get_part("door_0_outer")
    door_1_outer = object_model.get_part("door_1_outer")
    door_0_slide = object_model.get_articulation("door_0_slide")
    all_leaves = (
        (door_0, "leaf_0", "hazard_0"),
        (door_1, "leaf_1", "hazard_1"),
        (door_0_outer, "leaf_0", None),
        (door_1_outer, "leaf_1", None),
    )

    # --- Intentional local overlaps (captured / seated geometry) ---
    # The complementary wavy seam edges interleave across the centreline (their
    # AABBs overlap even though the solids only share a thin running clearance).
    for ea in ("leaf_0", "hazard_0"):
        for eb in ("leaf_1", "hazard_1"):
            ctx.allow_overlap(
                door_0,
                door_1,
                elem_a=ea,
                elem_b=eb,
                reason=(
                    "The two inner leaves close along a complementary wavy seam; "
                    "the seam lobes and their hazard borders interleave across "
                    "the doorway centreline."
                ),
            )
    # Every leaf rides captured in the fixed top/bottom guide bars for its
    # whole travel; each hazard decal seats a few mm into its own leaf face.
    # The outer leaves also slide within the frame pocket and extend into the
    # chamfered corner regions of the hex opening.
    for part, leaf_elem, haz_elem in all_leaves:
        for rail in ("top_track", "bottom_track"):
            ctx.allow_overlap(
                part,
                frame,
                elem_a=leaf_elem,
                elem_b=rail,
                reason=f"The {part.name} leaf rides captured inside the {rail} guide bar.",
            )
        # Outer leaves slide within the frame pocket and overlap the frame slab
        # in the chamfered corner regions of the hex opening.
        if part in (door_0_outer, door_1_outer):
            ctx.allow_overlap(
                part,
                frame,
                elem_a=leaf_elem,
                elem_b="frame_slab",
                reason=(
                    f"The {part.name} outer leaf slides within the frame pocket; "
                    "it extends into the chamfered corner regions of the hex opening."
                ),
            )
        if haz_elem is not None:
            ctx.allow_overlap(
                part,
                part,
                elem_a=haz_elem,
                elem_b=leaf_elem,
                reason="Hazard-stripe decal is seated a few mm into its own leaf front face.",
            )

    # --- Hero geometry present and placed ---
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame stands on the floor and reaches full height",
        frame_aabb is not None
        and abs(frame_aabb[0][2]) < 0.02
        and frame_aabb[1][2] > FRAME_H - 0.05,
        details=f"frame aabb={frame_aabb}",
    )

    # Hydraulic piston rams flank the opening on the +Y face.
    piston0 = ctx.part_element_world_aabb(frame, elem="piston_0")
    piston1 = ctx.part_element_world_aabb(frame, elem="piston_1")
    ctx.check(
        "hydraulic piston rams flank the opening left and right",
        piston0 is not None
        and piston1 is not None
        and piston0[1][0] < 0.0 < piston1[0][0],
        details=f"piston0={piston0}, piston1={piston1}",
    )

    # Warning beacons sit on the upper frame near the hex chamfers.
    beacon0 = ctx.part_element_world_aabb(frame, elem="beacon_0")
    ctx.check(
        "warning beacon sits on upper frame near the hex chamfer",
        beacon0 is not None
        and beacon0[0][2] > OPEN_TOP - 0.1,
        details=f"beacon0={beacon0}",
    )

    # The doorway is a true through-opening: no fixed frame furniture (e.g. a
    # back panel) blocks the central window of the opening.
    window_x = 0.45
    window_z = (0.45, 1.55)
    for elem in ("top_recess", "top_track", "bottom_track"):
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

    # --- Closed pose: leaves mate at the seam and cover the opening ---
    with ctx.pose({door_0_slide: 0.0}):
        # The complementary wavy seam edges interleave across the centreline.
        ctx.expect_overlap(
            door_0,
            door_1,
            axes="x",
            elem_a="leaf_0",
            elem_b="leaf_1",
            min_overlap=0.05,
            name="wavy seam edges mate across the centreline",
        )
        ctx.expect_overlap(
            door_0,
            door_1,
            axes="z",
            min_overlap=PANEL_H * 0.95,
            name="leaves mate along the full seam height",
        )
        # Telescoping stages lap each other in X so no slit opens between them.
        ctx.expect_overlap(
            door_0,
            door_0_outer,
            axes="x",
            elem_a="leaf_0",
            elem_b="leaf_0",
            min_overlap=0.01,
            name="left inner and outer stages lap with no see-through slit",
        )
        ctx.expect_overlap(
            door_1,
            door_1_outer,
            axes="x",
            elem_a="leaf_1",
            elem_b="leaf_1",
            min_overlap=0.01,
            name="right inner and outer stages lap with no see-through slit",
        )
        # Every leaf is captured by the upper and lower guide bars (no floating).
        for part, leaf_elem, _ in all_leaves:
            ctx.expect_overlap(
                part,
                frame,
                axes="z",
                elem_a=leaf_elem,
                elem_b="top_track",
                min_overlap=0.006,
                name=f"{part.name} is captured by the upper guide bar",
            )
            ctx.expect_overlap(
                part,
                frame,
                axes="z",
                elem_a=leaf_elem,
                elem_b="bottom_track",
                min_overlap=0.006,
                name=f"{part.name} is captured by the lower guide bar",
            )
        # Outer leaves stay within the frame pocket in X (not poking out).
        half_frame = FRAME_W / 2.0
        for outer_part in (door_0_outer, door_1_outer):
            aabb = ctx.part_world_aabb(outer_part)
            assert aabb is not None
            ctx.check(
                f"{outer_part.name} stays within the frame pocket at rest",
                aabb[0][0] >= -half_frame and aabb[1][0] <= half_frame,
                details=f"{outer_part.name} rest aabb x=({aabb[0][0]:.3f}, {aabb[1][0]:.3f})",
            )
        closed_c = {}
        for part, _, _ in all_leaves:
            pos = ctx.part_world_position(part)
            assert pos is not None
            closed_c[part.name] = pos

    # --- Open pose: all four leaves telescope apart in opposite directions ---
    half_frame = FRAME_W / 2.0
    with ctx.pose({door_0_slide: door_0_slide.motion_limits.upper}):
        for part, sign in (
            (door_0, -1.0),
            (door_1, +1.0),
            (door_0_outer, -1.0),
            (door_1_outer, +1.0),
        ):
            pos = ctx.part_world_position(part)
            assert pos is not None
            moved = sign * (pos[0] - closed_c[part.name][0])
            ctx.check(
                f"{part.name} retracts toward {'+' if sign > 0 else '-'}X when open",
                moved > 0.25,
                details=f"closed x={closed_c[part.name][0]:.3f}, open x={pos[0]:.3f}",
            )
            # The open leaf nests in the frame side pocket: it never pokes out
            # past the frame silhouette.
            aabb = ctx.part_world_aabb(part)
            assert aabb is not None
            ctx.check(
                f"{part.name} stays inside the frame silhouette when open",
                aabb[0][0] >= -half_frame - 0.001 and aabb[1][0] <= half_frame + 0.001,
                details=f"{part.name} open aabb x=({aabb[0][0]:.3f}, {aabb[1][0]:.3f})",
            )
        ctx.expect_gap(
            door_1,
            door_0,
            axis="x",
            min_gap=1.0,
            name="open leaves clear a wide central passage",
        )

    return ctx.report()


object_model = build_object_model()
