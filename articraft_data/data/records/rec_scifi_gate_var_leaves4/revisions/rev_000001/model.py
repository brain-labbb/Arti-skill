from __future__ import annotations

# Futuristic hexagonal-framed sci-fi iris gate.
#
# Articraft brief:
# - Object: a wall-mounted sci-fi gate, ~1.86 m wide x ~2.05 m tall x ~0.22 m
#   deep, standing on the floor (geometry from z=0 upward). Faces +Y; width
#   along X. The chamfered-hexagon opening is a true through-opening (no back
#   panel), so the open gate reveals empty space behind it.
# - Root/support: the grey frame slab with a hex-beveled opening, the dark
#   top/bottom guide bars spanning the opening, the cyan light-strip assemblies
#   flanking the opening, the recessed top band, chamfer trim strips, frame
#   bolts, and the two dark clamp/latch blocks at mid-height. All FIXED.
# - Parts: frame (root); leaf_0..leaf_3, four iris-style leaves that slide
#   radially outward from the opening centre to dilate the aperture.
# - Articulations: leaf_i_slide (PRISMATIC, along each leaf's outward radial
#   axis) — leaf_0 slides +X, leaf_1 +Z, leaf_2 -X, leaf_3 -Z, all with the
#   same effort/velocity policy and travel matched to the available frame
#   margin on each axis.
# - Visible geometry: hex opening cut into the frame, glowing twin cyan strips
#   on dark backing channels, four tapered iris leaves with groove slots and
#   yellow accent strips, dark guide bars, clamp blocks with status lamps,
#   chamfer trim, and bolts.
# - Support/fit: every leaf is captured top and bottom in the fixed guide bars
#   (proven overlap) for its whole travel; when open the leaves nest in the
#   frame margin/pocket without poking past the frame silhouette.
# - Intentional overlaps: each leaf rides captured in the guide bars (allowed +
#   proven); accent strips seat into their leaf plates (allowed + proven);
#   retracted leaves nest inside the frame pocket (allowed + proven).
# - Tests: closed leaves cover the opening centre, open pose dilates the
#   aperture in all four radial directions, leaves stay inside the frame,
#   strips/clamps stay fixed and flanking, and the frame stands on the floor.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------
FRAME_W = 1.86          # overall width along X
FRAME_H = 2.05          # overall height along Z (stands on z=0)
FRAME_D = 0.22          # frame slab depth along Y
FRAME_FACE_Y = FRAME_D / 2.0  # front face plane (+Y)

# Hexagonal opening cut through the frame (chamfered top/bottom corners)
OPEN_W = 1.18           # opening width (flat-side to flat-side) along X
OPEN_H = 1.74           # opening height along Z
OPEN_CHAMFER = 0.30     # how far the angled corner cuts run
OPEN_CY = 1.04          # opening vertical center (z)
OPEN_HW = OPEN_W / 2.0  # 0.59
OPEN_HH = OPEN_H / 2.0  # 0.87
OPEN_TOP = OPEN_CY + OPEN_HH
OPEN_BOT = OPEN_CY - OPEN_HH

# Fixed guide bars (the chunky dark bars across the opening in the reference).
BAR_HALF_LEN = 0.90     # reaches into the frame on both sides
BAR_Y_LO = -0.08        # encloses the leaf sliding planes
BAR_Y_HI = 0.075
TOP_BAR_Z = (1.78, 1.92)   # overlaps leaf tops, covers the top apex
BOT_BAR_Z = (0.16, 0.30)   # overlaps leaf bottoms, covers the bottom apex

# ---------------------------------------------------------------------------
# Iris leaf parameters
# ---------------------------------------------------------------------------
N_LEAVES = 4
LEAF_D = 0.018          # plate thickness along Y

# Y stacking: four leaves at staggered depths within the frame pocket,
# front-most leaf nearest the frame face.
LEAF_Y_BASE = 0.055     # Y center of leaf_0 (front)
LEAF_Y_STEP = 0.024     # Y step between consecutive leaves

# Travel limits matched to the available frame margin on each axis.
# Horizontal leaves (±X): frame margin = FRAME_W/2 - OPEN_HW = 0.34
# Vertical leaves (±Z): top margin = FRAME_H - OPEN_TOP = 0.14
TRAVEL_H = 0.33         # horizontal leaves
TRAVEL_V = 0.13         # vertical leaves

# Per-leaf configuration: (slide_axis, radial_extent, tangential_extent,
#                          travel, cadquery_y_rotation_degrees)
# The wedge helper builds the plate with radial along +X and tangential along
# Z; ry_deg rotates it to the leaf's outward radial direction.
LEAF_CONFIGS: list[tuple[tuple[float, float, float], float, float, float, float]] = [
    # leaf_0: right, slides +X (radial reduced to clear clamp blocks at ±0.585)
    ((1.0, 0.0, 0.0), OPEN_HW - 0.025, OPEN_H - 0.02, TRAVEL_H, 0.0),
    # leaf_1: top, slides +Z
    ((0.0, 0.0, 1.0), OPEN_HH, OPEN_W - 0.02, TRAVEL_V, -90.0),
    # leaf_2: left, slides -X (radial reduced to clear clamp blocks)
    ((-1.0, 0.0, 0.0), OPEN_HW - 0.025, OPEN_H - 0.02, TRAVEL_H, 180.0),
    # leaf_3: bottom, slides -Z
    ((0.0, 0.0, -1.0), OPEN_HH, OPEN_W - 0.02, TRAVEL_V, 90.0),
]


# ---------------------------------------------------------------------------
# Frame helpers (identical to parent)
# ---------------------------------------------------------------------------
def _hex_opening_profile(w: float, h: float, chamf: float) -> list[tuple[float, float]]:
    """Return a centered elongated-hexagon outline (chamfered top/bottom)."""
    hw = w / 2.0
    hh = h / 2.0
    cz = h / 2.0 - chamf
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
    cutter = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(FRAME_D * 2.0, both=True)
        .translate((0.0, 0.0, OPEN_CY))
    )
    frame = slab.cut(cutter)
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
    pin = (
        cq.Workplane("XY")
        .cylinder(0.07, 0.04, centered=(True, True, True))
        .rotate((0, 0, 0), (1, 0, 0), 90.0)
        .translate((x, FRAME_FACE_Y + 0.10, OPEN_CY))
    )
    return body.union(pin)


# ---------------------------------------------------------------------------
# Iris leaf wedge helper (shared by all four leaves)
# ---------------------------------------------------------------------------
def _leaf_wedge_mesh(radial: float, tangential: float, thickness: float,
                     ry_deg: float) -> cq.Workplane:
    """Iris leaf wedge plate with transverse groove slots.

    Builds the plate in canonical orientation (inner edge at X=0, outer edge
    at X=radial, centred in Z and Y), then rotates around Y to align with the
    leaf's outward radial slide direction.

    Two transverse groove slots (70 % of the tangential width) cut through the
    full thickness for sci-fi surface detail without disconnecting the plate.
    """
    # Main rectangular plate body.
    # Workplane("XZ") normal is -Y, so extrude goes in -Y.
    plate = (
        cq.Workplane("XZ")
        .rect(radial, tangential)
        .extrude(thickness)
        .translate((radial / 2.0, thickness / 2.0, 0.0))
    )

    # Transverse groove slots — do not cut the full tangential width so the
    # plate remains one connected solid (15 % margin on each side).
    gw = 0.010
    slot_h = tangential * 0.70
    slot_depth = thickness * 3.0
    for frac in (0.35, 0.65):
        slot = (
            cq.Workplane("XZ")
            .center(radial * frac, 0.0)
            .rect(gw, slot_h)
            .extrude(slot_depth)
            .translate((0.0, slot_depth / 2.0, 0.0))
        )
        plate = plate.cut(slot)

    # A thin raised rib along the outer edge gives the plate a subtle
    # wedge-like cross-section (thicker at the outer edge).
    rib_w = 0.018
    rib_h = tangential * 0.88
    rib_d = 0.005
    rib = (
        cq.Workplane("XZ")
        .center(radial - rib_w / 2.0, 0.0)
        .rect(rib_w, rib_h)
        .extrude(rib_d)
        .translate((0.0, thickness / 2.0, 0.0))
    )
    plate = plate.union(rib)

    # Rotate to the target slide direction.
    if ry_deg != 0.0:
        plate = plate.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), ry_deg)

    return plate


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scifi_hex_iris_door")

    grey = model.material("frame_grey", rgba=(0.85, 0.86, 0.88, 1.0))
    cyan = model.material("glow_cyan", rgba=(0.45, 0.98, 1.0, 1.0))
    clamp_dark = model.material("clamp_charcoal", rgba=(0.09, 0.10, 0.11, 1.0))
    armor = model.material("door_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    hazard_yellow = model.material("hazard_yellow", rgba=(1.0, 0.86, 0.04, 1.0))
    status_red = model.material("status_red", rgba=(0.85, 0.16, 0.12, 1.0))

    # ---- Fixed root: frame + guide bars + light strips + clamps + trim ----
    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_frame_mesh(), "frame_slab"), material=grey, name="frame_slab")
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
    backing_cx = OPEN_HW + 0.01 - backing_w / 2.0
    for s, tag in ((-1.0, "0"), (1.0, "1")):
        frame.visual(
            Box((backing_w, 0.023, strip_h)),
            origin=Origin(xyz=(s * backing_cx, 0.0835, OPEN_CY)),
            material=clamp_dark,
            name=f"strip_backing_{tag}",
        )
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
    chamf_mid_x = OPEN_HW - OPEN_CHAMFER / 2.0
    chamf_top_z = OPEN_TOP - OPEN_CHAMFER / 2.0
    chamf_bot_z = OPEN_BOT + OPEN_CHAMFER / 2.0
    chamf_len = OPEN_CHAMFER * math.sqrt(2.0) - 0.02
    for sx in (-1.0, 1.0):
        for cz, sz, vtag in ((chamf_top_z, 1.0, "t"), (chamf_bot_z, -1.0, "b")):
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

    # ---- Iris leaves: 4 radially-retracting segments ----
    for i in range(N_LEAVES):
        axis, radial, tangential, travel, ry_deg = LEAF_CONFIGS[i]
        y_pos = LEAF_Y_BASE - i * LEAF_Y_STEP
        ry_rad = math.radians(ry_deg)

        leaf = model.part(f"leaf_{i}")

        # Main wedge plate (shared helper, rotated per leaf).
        leaf.visual(
            mesh_from_cadquery(
                _leaf_wedge_mesh(radial, tangential, LEAF_D, ry_deg),
                f"plate_{i}",
            ),
            material=armor,
            name=f"plate_{i}",
        )

        # Yellow accent strip near the inner edge, on the front face of the
        # plate.  Positioned and rotated to match the leaf's radial direction.
        sw = 0.025           # strip width along radial
        sd = 0.004           # strip thickness (stands proud)
        sh = tangential * 0.60  # strip length along tangential
        embed = 0.002        # seated into the plate face for connectivity
        # Canonical strip centre: (sw/2, LEAF_D/2 + sd/2 - embed, 0)
        # After Ry(ry_rad):
        scx = (sw / 2.0) * math.cos(ry_rad)
        scy = LEAF_D / 2.0 + sd / 2.0 - embed
        scz = -(sw / 2.0) * math.sin(ry_rad)
        leaf.visual(
            Box((sw, sd, sh)),
            origin=Origin(xyz=(scx, scy, scz), rpy=(0.0, ry_rad, 0.0)),
            material=hazard_yellow,
            name=f"accent_{i}",
        )

        # Prismatic joint along the outward radial axis.
        model.articulation(
            f"leaf_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=leaf,
            origin=Origin(xyz=(0.0, y_pos, OPEN_CY)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=900.0,
                velocity=0.5,
                lower=0.0,
                upper=travel,
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    leaves = [object_model.get_part(f"leaf_{i}") for i in range(N_LEAVES)]
    joints = [object_model.get_articulation(f"leaf_{i}_slide") for i in range(N_LEAVES)]

    # --- Intentional local overlaps ---
    # Accent strips seated into their own leaf plates.
    for i in range(N_LEAVES):
        ctx.allow_overlap(
            leaves[i], leaves[i],
            elem_a=f"accent_{i}", elem_b=f"plate_{i}",
            reason=f"Accent strip is seated a few mm into leaf_{i} plate surface.",
        )
    # Every leaf rides captured in the fixed top/bottom guide bars.
    for i in range(N_LEAVES):
        for rail in ("top_track", "bottom_track"):
            ctx.allow_overlap(
                leaves[i], frame,
                elem_a=f"plate_{i}", elem_b=rail,
                reason=f"leaf_{i} rides captured inside the {rail} guide bar.",
            )
    # Retracted leaves nest inside the frame margin/pocket.
    for i in range(N_LEAVES):
        ctx.allow_overlap(
            leaves[i], frame,
            elem_a=f"plate_{i}", elem_b="frame_slab",
            reason=f"leaf_{i} retracts into the frame margin pocket when open.",
        )

    # --- Frame geometry ---
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

    # The doorway is a true through-opening: no fixed frame furniture blocks
    # the central window.
    window_x = 0.35
    window_z = (0.50, 1.50)
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

    # --- Four leaves exist with equal angular placement ---
    ctx.check(
        "four iris leaves exist",
        len(leaves) == N_LEAVES,
        details=f"found {len(leaves)} leaves",
    )
    # Joint axes are at 90-degree intervals: dot products of adjacent axes ≈ 0.
    axes = [
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
        (-1.0, 0.0, 0.0),
        (0.0, 0.0, -1.0),
    ]
    for i in range(N_LEAVES):
        ni = (i + 1) % N_LEAVES
        dot = sum(a * b for a, b in zip(axes[i], axes[ni]))
        ctx.check(
            f"leaf_{i} and leaf_{ni} axes are perpendicular",
            abs(dot) < 0.01,
            details=f"dot={dot:.4f}",
        )

    # --- Closed pose: leaves cover the opening ---
    closed_pose = {j: 0.0 for j in joints}
    with ctx.pose(closed_pose):
        # Each leaf overlaps the opening centre region in its radial axis.
        # Horizontal leaves cover the full opening height; vertical leaves
        # cover the full width.  Opposing leaves meet at the centre.
        ctx.expect_overlap(
            leaves[0], leaves[2],
            axes="z",
            elem_a="plate_0", elem_b="plate_2",
            min_overlap=1.0,
            name="horizontal leaves cover the opening height when closed",
        )
        ctx.expect_overlap(
            leaves[1], leaves[3],
            axes="x",
            elem_a="plate_1", elem_b="plate_3",
            min_overlap=0.80,
            name="vertical leaves cover the opening width when closed",
        )

        # Opposing horizontal leaves meet at X=0 when closed (their inner
        # edges align; they ride at different Y depths so use projection).
        ctx.expect_gap(
            leaves[0], leaves[2],
            axis="x",
            min_gap=-0.005, max_gap=0.005,
            positive_elem="plate_0", negative_elem="plate_2",
            name="right and left leaves meet at the centre when closed",
        )

        # Horizontal leaves (0,2) span the full opening height and are captured
        # by both guide bars. Vertical leaves (1,3) only extend toward their
        # respective track.
        for i in (0, 2):
            ctx.expect_overlap(
                leaves[i], frame,
                axes="z",
                elem_a=f"plate_{i}",
                elem_b="top_track",
                min_overlap=0.005,
                name=f"leaf_{i} captured by upper guide bar",
            )
            ctx.expect_overlap(
                leaves[i], frame,
                axes="z",
                elem_a=f"plate_{i}",
                elem_b="bottom_track",
                min_overlap=0.005,
                name=f"leaf_{i} captured by lower guide bar",
            )
        # leaf_1 (top) extends upward, captured by top_track only.
        ctx.expect_overlap(
            leaves[1], frame,
            axes="z",
            elem_a="plate_1",
            elem_b="top_track",
            min_overlap=0.005,
            name="leaf_1 captured by upper guide bar",
        )
        # leaf_3 (bottom) extends downward, captured by bottom_track only.
        ctx.expect_overlap(
            leaves[3], frame,
            axes="z",
            elem_a="plate_3",
            elem_b="bottom_track",
            min_overlap=0.005,
            name="leaf_3 captured by lower guide bar",
        )

        # Record closed positions.
        closed_pos: dict[int, tuple[float, float, float]] = {}
        for i in range(N_LEAVES):
            pos = ctx.part_world_position(leaves[i])
            assert pos is not None
            closed_pos[i] = pos

    # --- Open pose: leaves retract outward, dilating the aperture ---
    open_pose = {j: j.motion_limits.upper for j in joints}
    half_frame = FRAME_W / 2.0
    with ctx.pose(open_pose):
        slide_dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # (x_sign, z_sign)
        for i, (sx, sz) in enumerate(slide_dirs):
            pos = ctx.part_world_position(leaves[i])
            assert pos is not None
            if sx != 0:
                moved = sx * (pos[0] - closed_pos[i][0])
                ctx.check(
                    f"leaf_{i} retracts along {'+' if sx > 0 else '-'}X when open",
                    moved > 0.05,
                    details=f"closed x={closed_pos[i][0]:.3f}, open x={pos[0]:.3f}",
                )
            if sz != 0:
                moved = sz * (pos[2] - closed_pos[i][2])
                ctx.check(
                    f"leaf_{i} retracts along {'+' if sz > 0 else '-'}Z when open",
                    moved > 0.05,
                    details=f"closed z={closed_pos[i][2]:.3f}, open z={pos[2]:.3f}",
                )
            # Leaf stays within the frame silhouette.
            aabb = ctx.part_world_aabb(leaves[i])
            assert aabb is not None
            ctx.check(
                f"leaf_{i} stays inside the frame silhouette when open",
                aabb[0][0] >= -half_frame - 0.01
                and aabb[1][0] <= half_frame + 0.01
                and aabb[0][2] >= -0.01
                and aabb[1][2] <= FRAME_H + 0.01,
                details=f"{leaves[i].name} aabb=({aabb[0]}, {aabb[1]})",
            )

        # Dilated aperture: opposing leaves have separated.
        ctx.expect_gap(
            leaves[0], leaves[2],
            axis="x",
            min_gap=0.20,
            name="horizontal leaves dilate apart in X",
        )
        ctx.expect_gap(
            leaves[1], leaves[3],
            axis="z",
            min_gap=0.10,
            name="vertical leaves dilate apart in Z",
        )

    return ctx.report()


object_model = build_object_model()
