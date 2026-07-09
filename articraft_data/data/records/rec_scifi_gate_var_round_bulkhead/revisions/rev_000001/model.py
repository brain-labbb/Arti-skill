from __future__ import annotations

# Sci-fi blast / security door — circular-bulkhead variant.
#
# Two dark gunmetal halves meet along a yellow-edged zig-zag interlocking seam
# down the middle and retract sideways into side housings to open. Each half is
# a two-stage telescoping pair (inner seam panel + outer panel in a second
# plane) so the whole door pockets fully inside its side housing when open.
#
# VARIANT: the rectangular header-sill-housing surround of the parent has been
# replaced by a circular armoured bulkhead ring (CadQuery lathe-style disk with
# rectangular doorway cutout, trimmed at the deck plane). The bi-parting
# telescoping leaf mechanism, keypad, and hazard blocks are identical.
#
# Articraft brief:
# - Object: armoured sci-fi blast door, ~1.62 m clear width x ~2.05 m tall,
#   standing on the ground plane (geometry from z=0 upward). Viewer looks from
#   -Y; the doorway is a true through-opening (no back wall).
# - Root/support: `frame` is the fixed circular bulkhead ring (armoured plate
#   with raised outer rim lip and inner door-stop frame, deck-trimmed at the
#   floor), plus the top/bottom guide rails behind it. It carries the two
#   fixed side housings, the wall keypad on the left jamb, and the hazard
#   blocks.
# - Parts: frame (root, fixed); left_housing / right_housing (FIXED) are hollow
#   side pockets (front cowl + rear rail) bolted through the ring; keypad,
#   hazard_left, hazard_right (FIXED); door_0 / door_1, the two inner armoured
#   seam panels with complementary zig-zag leading edges; door_0_outer /
#   door_1_outer, the outer telescoping panels riding in a second plane behind
#   the inner ones.
# - Articulations: door_0_slide (PRISMATIC, +X) drives the right inner panel;
#   door_1_slide (-X), door_0_outer_slide (+X, 0.5x) and door_1_outer_slide
#   (-X, 0.5x) are mimic-coupled so one travel value telescopes all four
#   panels open symmetrically.
# - Visible geometry: circular armoured bulkhead ring (CadQuery) with outer
#   rim lip, inner door-stop frame, perimeter bolt heads, top trim arc, and
#   warning lamps; one shared zig-zag seam line splits the inner panel pair;
#   yellow accent strips, keypad with screen + keys + status lamp, yellow/
#   black hazard chevrons, riveted cowl plates, panel grooves, kick plates.
# - Support/fit: every panel is captured top and bottom in fixed guide rails
#   for its whole travel; housings bolt through the ring plate.
# - Intentional overlaps: closed zig-zag teeth interleave (allowed + proven);
#   panels ride captured in the guide rails (allowed + proven); housing cowls
#   and mounted furniture seat against the ring front plate (allowed + proven).
# - Tests: ring has circular proportions and extends past the opening; panels
#   span the opening when closed, zig-zag interlocks, doorway unobstructed,
#   all panels retract into housings when open, fixed furniture mounted, door
#   stands on the floor.
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
# Overall dimensions (metres). X = width, Y = depth (door thickness), Z = up.
# ---------------------------------------------------------------------------
OPENING_W = 1.62  # clear doorway width (jamb-inner to jamb-inner)
OPENING_H = 2.05  # clear doorway height
DOOR_THK = 0.06  # armoured panel thickness (Y extent), one telescoping stage

FRAME_DEPTH = 0.34  # reference depth for the original rectangular frame (Y)
RING_OUTER_R = 1.40  # circular bulkhead ring outer radius
RING_THK = 0.08  # ring plate thickness (Y)

HOUSING_W = 0.51  # side housing / pocket width (X): swallows the open panels
REAR_RAIL_THK = 0.05  # rear rail thickness (Y)

# Leaf geometry.
HALF_W = OPENING_W / 2.0  # 0.81 : half the clear opening
TOOTH = 0.09  # zig-zag tooth half-amplitude (X) — sharp angular interlock
SEAM_CLEAR = 0.002  # running clearance per side of the shared seam line

# Telescoping split per side.
A_OUT = 0.42  # outer edge of the inner seam panel
B_IN = 0.40  # inner edge of the outer panel (0.02 lap behind the inner one)

# Door travel.
TRAVEL_A = 0.84
TRAVEL_B = TRAVEL_A / 2.0

# Two sliding planes inside the housing channel.
A_PLANE_Y = -0.045  # inner seam panels (front plane)
B_PLANE_Y = 0.035  # outer panels (rear plane)

# Housing channel: front cowl ahead of the panel planes, rear rail behind.
COWL_FRONT_Y = -0.115
COWL_THK = 0.05
COWL_PLATE_THK = 0.025
COWL_PLATE_Y = COWL_FRONT_Y - COWL_THK / 2.0 - COWL_PLATE_THK / 2.0
COWL_PLATE_FRONT_Y = COWL_PLATE_Y - COWL_PLATE_THK / 2.0
REAR_RAIL_Y = 0.11

# Top/bottom guide rails (fixed): capture every panel for its whole travel.
GUIDE_HALF_LEN = HALF_W + HOUSING_W - 0.03
GUIDE_DEPTH = 0.16
GUIDE_CY = (A_PLANE_Y + B_PLANE_Y) / 2.0
GUIDE_CAPTURE = 0.012
LEAF_BOT_Z = 0.05  # sill height — bottom of leaf panels
LEAF_TOP_Z = LEAF_BOT_Z + OPENING_H
TOP_GUIDE_H = 0.06
TOP_GUIDE_CZ = LEAF_TOP_Z - GUIDE_CAPTURE + TOP_GUIDE_H / 2.0
BOT_GUIDE_H = 0.036
BOT_GUIDE_CZ = LEAF_BOT_Z + GUIDE_CAPTURE - BOT_GUIDE_H / 2.0

# Vertical centre of the leaf panels.
LEAF_CZ = LEAF_BOT_Z + OPENING_H / 2.0

# Ring placement.
RING_CZ = LEAF_BOT_Z + OPENING_H / 2.0  # ring centre height (world z)
RING_FRONT_Y = -FRAME_DEPTH / 2.0  # ring front face flush with original frame
RING_Y_OFFSET = RING_FRONT_Y + RING_THK / 2.0  # ring local-origin Y offset

# Hazard chevron blocks (shared constants for placement and bolt clearance).
HAZ_H = 0.30
HAZ_W = 0.26
HAZ_CZ = LEAF_BOT_Z + HAZ_H / 2.0 + 0.22


# ---------------------------------------------------------------------------
# Zig-zag leaf geometry (CadQuery).
# ---------------------------------------------------------------------------
def _seam_points() -> list[tuple[float, float]]:
    """Shared zig-zag seam-line (x, z) knots over the opening height."""
    h = OPENING_H
    a = TOOTH
    return [
        (+a, 0.0),
        (-a, h * 0.25),
        (+a, h * 0.50),
        (-a, h * 0.75),
        (+a, h),
    ]


def _make_inner_panel(side: float) -> cq.Workplane:
    """One inner seam panel as a CadQuery solid."""
    h = OPENING_H
    edge_x = side * A_OUT
    pts = [(x + side * SEAM_CLEAR, z) for (x, z) in _seam_points()]
    pts.append((edge_x, h))
    pts.append((edge_x, 0.0))
    profile = cq.Workplane("XZ").polyline(pts).close()
    leaf = profile.extrude(DOOR_THK)
    leaf = leaf.translate((0.0, DOOR_THK / 2.0, 0.0))
    return leaf


def _make_inner_accent(side: float) -> cq.Workplane:
    """Thin yellow ribbon hugging the seam edge of one inner panel."""
    strip = 0.06
    outer = [(x + side * SEAM_CLEAR, z) for (x, z) in _seam_points()]
    inner = [(x + side * strip, z) for (x, z) in outer]
    pts = outer + list(reversed(inner))
    ribbon_thk = DOOR_THK * 0.5
    ribbon = cq.Workplane("XZ").polyline(pts).close().extrude(ribbon_thk)
    ribbon = ribbon.translate((0.0, -DOOR_THK / 2.0 + ribbon_thk - 0.006, 0.0))
    return ribbon


# ---------------------------------------------------------------------------
# Hazard chevron stripes (CadQuery).
# ---------------------------------------------------------------------------
def _make_hazard_stripes(block_w: float, block_h: float) -> cq.Workplane:
    band = 0.05
    gap = band
    thk = 0.008
    diag = block_w + block_h
    slashes: cq.Workplane | None = None
    step = band + gap
    n = int(diag / step) + 2
    for k in range(-n, n + 1):
        c = k * step * math.sqrt(2.0)
        bar = (
            cq.Workplane("XZ")
            .center(c / 2.0, 0.0)
            .rect(band, diag * 1.6)
            .extrude(thk)
            .rotate((0, 0, 0), (0, 1, 0), 45.0)
        )
        slashes = bar if slashes is None else slashes.union(bar)
    assert slashes is not None
    clip = cq.Workplane("XZ").rect(block_w, block_h).extrude(thk * 2.0)
    clip = clip.translate((0.0, thk * 0.5, 0.0))
    stripes = slashes.intersect(clip)
    return stripes


# ---------------------------------------------------------------------------
# Circular armoured bulkhead ring (CadQuery).
# A thick disk with a circular outer profile and a rectangular doorway bore,
# built in local coordinates centred on the doorway centre (the placement
# origin is at world (0, RING_Y_OFFSET, RING_CZ)). An outer rim lip and an
# inner door-stop frame add visible manufactured detail. The bottom is cut
# flat at the deck plane (world z = 0).
# ---------------------------------------------------------------------------
def _make_bulkhead_ring() -> cq.Workplane:
    outer_r = RING_OUTER_R
    thk = RING_THK
    centre_z = RING_CZ  # equals LEAF_BOT_Z + OPENING_H/2 in local coords

    # --- Main plate: circular disk extruded along Y, centred at origin. ---
    ring = (
        cq.Workplane("XZ")
        .circle(outer_r)
        .extrude(thk)
        .translate((0.0, -thk / 2.0, 0.0))
    )

    # --- Rectangular doorway bore (through the full plate thickness). ---
    door_bore = (
        cq.Workplane("XZ")
        .rect(OPENING_W, OPENING_H)
        .extrude(thk + 0.04)
        .translate((0.0, -(thk + 0.04) / 2.0, 0.0))
    )
    ring = ring.cut(door_bore)

    # --- Outer rim lip: raised annular band on the front face. ---
    lip_w = 0.035
    lip_t = 0.015
    lip_back_y = -thk / 2.0 - lip_t
    lip = (
        cq.Workplane("XZ")
        .circle(outer_r + 0.001)
        .circle(outer_r - lip_w)
        .extrude(lip_t)
        .translate((0.0, lip_back_y, 0.0))
    )
    ring = ring.union(lip)

    # --- Inner door-stop frame: thin rectangular surround on the front face. ---
    stop_w = 0.025
    stop_t = 0.012
    stop_back_y = -thk / 2.0 - stop_t
    stop = (
        cq.Workplane("XZ")
        .rect(OPENING_W + 2.0 * stop_w, OPENING_H + 2.0 * stop_w)
        .extrude(stop_t)
        .translate((0.0, stop_back_y, 0.0))
    )
    stop_hole = (
        cq.Workplane("XZ")
        .rect(OPENING_W + 0.004, OPENING_H + 0.004)
        .extrude(stop_t + 0.02)
        .translate((0.0, stop_back_y - 0.01, 0.0))
    )
    stop = stop.cut(stop_hole)
    ring = ring.union(stop)

    # --- Deck (floor) cut: remove everything below the ground plane. ---
    # In local coords the floor is at z = -centre_z; cut a big box below that.
    deck_box = (
        cq.Workplane("XY")
        .rect(2.0 * outer_r + 0.4, thk + lip_t + 0.4)
        .extrude(3.0)
        .translate((0.0, 0.0, -centre_z - 3.0))
    )
    ring = ring.cut(deck_box)

    return ring


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scifi_blast_door")

    gunmetal = model.material("gunmetal", rgba=(0.18, 0.20, 0.22, 1.0))
    gunmetal_deep = model.material("gunmetal_deep", rgba=(0.14, 0.16, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    olive_frame = model.material("olive_frame", rgba=(0.27, 0.30, 0.26, 1.0))
    accent_yellow = model.material("accent_yellow", rgba=(0.92, 0.74, 0.06, 1.0))
    hazard_black = model.material("hazard_black", rgba=(0.07, 0.07, 0.07, 1.0))
    keypad_body = model.material("keypad_body", rgba=(0.16, 0.17, 0.19, 1.0))
    keypad_screen = model.material("keypad_screen", rgba=(0.12, 0.42, 0.62, 1.0))
    keypad_key = model.material("keypad_key", rgba=(0.30, 0.32, 0.34, 1.0))
    status_green = model.material("status_green", rgba=(0.18, 0.78, 0.34, 1.0))

    # ----- Root: fixed structural frame (circular bulkhead ring + rails) -----
    frame = model.part("frame")

    # Circular armoured bulkhead ring (CadQuery mesh): circular outer profile,
    # rectangular doorway bore, outer rim lip, inner door-stop frame, deck-
    # trimmed at the floor.
    frame.visual(
        mesh_from_cadquery(_make_bulkhead_ring(), "bulkhead_ring"),
        origin=Origin(xyz=(0.0, RING_Y_OFFSET, RING_CZ)),
        material=olive_frame,
        name="bulkhead_ring",
    )

    # Perimeter bolt heads on the ring front face.
    bolt_circle_r = RING_OUTER_R - 0.065
    n_bolts = 20
    for i in range(n_bolts):
        angle = 2.0 * math.pi * i / n_bolts
        bx = bolt_circle_r * math.cos(angle)
        bz_local = bolt_circle_r * math.sin(angle)
        bz_world = bz_local + RING_CZ
        # Skip bolts inside the rectangular doorway bore.
        if abs(bx) < HALF_W + 0.04 and abs(bz_local) < OPENING_H / 2.0 + 0.04:
            continue
        # Skip bolts below the floor.
        if bz_world < 0.05:
            continue
        # Skip bolts that would collide with the hazard chevron blocks.
        skip = False
        for hs in (-1.0, 1.0):
            hcx = hs * (HALF_W + HOUSING_W / 2.0)
            if abs(bx - hcx) < HAZ_W / 2.0 + 0.02 and abs(bz_world - HAZ_CZ) < HAZ_H / 2.0 + 0.02:
                skip = True
                break
        if skip:
            continue
        frame.visual(
            Box((0.028, 0.014, 0.028)),
            origin=Origin(xyz=(bx, RING_FRONT_Y - 0.007, bz_world)),
            material=dark_steel,
            name=f"ring_bolt_{i}",
        )

    # Worn metal trim arc across the top of the ring front face.
    # Extended slightly into the ring face for visual connectivity.
    trim_z = LEAF_BOT_Z + OPENING_H + 0.18
    trim_y = RING_FRONT_Y - 0.010  # embeds 0.0025 into the ring front face
    frame.visual(
        Box((0.80, 0.025, 0.09)),
        origin=Origin(xyz=(0.0, trim_y, trim_z)),
        material=dark_steel,
        name="header_trim",
    )

    # Top guide rail: captures every panel top for its whole travel.
    # Extended in Y to bridge through the ring back face for visual connectivity.
    top_guide_cy = -0.05  # shifted back to reach the ring
    top_guide_depth = 0.25  # deepened to overlap with ring back face
    frame.visual(
        Box((2.0 * GUIDE_HALF_LEN, top_guide_depth, TOP_GUIDE_H)),
        origin=Origin(xyz=(0.0, top_guide_cy, TOP_GUIDE_CZ)),
        material=dark_steel,
        name="top_guide",
    )
    # Bottom guide rail: captures every panel bottom.
    bot_guide_cy = -0.05  # shifted back to reach the ring
    bot_guide_depth = 0.25  # deepened to overlap with ring back face
    frame.visual(
        Box((2.0 * GUIDE_HALF_LEN, bot_guide_depth, BOT_GUIDE_H)),
        origin=Origin(xyz=(0.0, bot_guide_cy, BOT_GUIDE_CZ)),
        material=dark_steel,
        name="bottom_guide",
    )

    # Twin amber warning lamps on the ring trim arc.
    # Positioned to overlap with the header trim for visual connectivity.
    lamp_y = trim_y - 0.006  # embeds into the trim face
    for s, lamp_name in ((-1.0, "warning_lamp_l"), (1.0, "warning_lamp_r")):
        frame.visual(
            Box((0.10, 0.012, 0.05)),
            origin=Origin(xyz=(s * 0.35, lamp_y, trim_z)),
            material=accent_yellow,
            name=lamp_name,
        )

    # ----- Fixed side housings: hollow jamb pockets (front cowl + rear rail) --
    left_housing = model.part("left_housing")
    right_housing = model.part("right_housing")
    housing_cz = LEAF_BOT_Z + OPENING_H / 2.0
    housing_h = OPENING_H
    cowl_h = OPENING_H - 0.02
    pocket_inner_x = HALF_W
    pocket_outer_x = HALF_W + HOUSING_W
    for part, s in ((left_housing, -1.0), (right_housing, 1.0)):
        cx = s * (pocket_inner_x + HOUSING_W / 2.0)
        part.visual(
            Box((HOUSING_W, COWL_THK, cowl_h)),
            origin=Origin(xyz=(cx, COWL_FRONT_Y, housing_cz)),
            material=olive_frame,
            name="cowl_front",
        )
        part.visual(
            Box((HOUSING_W * 0.86, COWL_PLATE_THK, cowl_h * 0.8)),
            origin=Origin(xyz=(cx, COWL_PLATE_Y, housing_cz)),
            material=dark_steel,
            name="cowl_plate",
        )
        part.visual(
            Box((HOUSING_W, REAR_RAIL_THK, housing_h)),
            origin=Origin(xyz=(cx, REAR_RAIL_Y, housing_cz)),
            material=olive_frame,
            name="rear_rail",
        )
        end_x = s * (pocket_outer_x - 0.015)
        part.visual(
            Box((0.03, REAR_RAIL_Y - COWL_FRONT_Y + REAR_RAIL_THK, housing_h)),
            origin=Origin(xyz=(end_x, (COWL_FRONT_Y + REAR_RAIL_Y) / 2.0, housing_cz)),
            material=olive_frame,
            name="pocket_end_wall",
        )
        for rz, row in ((housing_cz - 0.56, "lo"), (housing_cz + 0.56, "hi")):
            for dx, col in ((-0.10, "in"), (0.10, "out")):
                part.visual(
                    Box((0.022, 0.012, 0.022)),
                    origin=Origin(xyz=(cx + dx, COWL_PLATE_FRONT_Y - 0.006, rz)),
                    material=dark_steel,
                    name=f"rivet_{row}_{col}",
                )

    # ----- Fixed wall keypad on the left cowl front face -----
    keypad = model.part("keypad")
    kp_cx = -(pocket_inner_x + HOUSING_W / 2.0)
    kp_box_thk = 0.05
    kp_box_y = COWL_PLATE_FRONT_Y - kp_box_thk / 2.0
    kp_z = LEAF_BOT_Z + OPENING_H * 0.6
    keypad.visual(
        Box((0.12, kp_box_thk, 0.18)),
        origin=Origin(xyz=(kp_cx, kp_box_y, kp_z)),
        material=keypad_body,
        name="keypad_box",
    )
    kp_face_y = kp_box_y - kp_box_thk / 2.0
    keypad.visual(
        Box((0.085, 0.014, 0.045)),
        origin=Origin(xyz=(kp_cx, kp_face_y + 0.004, kp_z + 0.05)),
        material=keypad_screen,
        name="keypad_screen",
    )
    for r in range(3):
        for c in range(3):
            keypad.visual(
                Box((0.02, 0.018, 0.02)),
                origin=Origin(
                    xyz=(
                        kp_cx + (c - 1) * 0.028,
                        kp_face_y + 0.006,
                        kp_z - 0.02 - r * 0.028,
                    )
                ),
                material=keypad_key,
                name=f"key_{r}_{c}",
            )
    keypad.visual(
        Box((0.05, 0.014, 0.025)),
        origin=Origin(xyz=(kp_cx, kp_face_y + 0.004, kp_z + 0.09)),
        material=status_green,
        name="status_lamp",
    )

    # ----- Fixed hazard chevrons at the base corners -----
    haz_left = model.part("hazard_left")
    haz_right = model.part("hazard_right")
    haz_thk = 0.02
    haz_back_y = COWL_PLATE_FRONT_Y - haz_thk / 2.0
    haz_face_y = haz_back_y - haz_thk / 2.0
    haz_stripes_mesh = _make_hazard_stripes(HAZ_W * 0.94, HAZ_H * 0.94)
    for part, s, name in ((haz_left, -1.0, "L"), (haz_right, 1.0, "R")):
        cx = s * (pocket_inner_x + HOUSING_W / 2.0)
        part.visual(
            Box((HAZ_W, haz_thk, HAZ_H)),
            origin=Origin(xyz=(cx, haz_back_y, HAZ_CZ)),
            material=hazard_black,
            name="hazard_back",
        )
        yaw = 0.0 if s < 0 else math.pi
        part.visual(
            mesh_from_cadquery(haz_stripes_mesh, f"hazard_stripes_{name}"),
            origin=Origin(xyz=(cx, haz_face_y, HAZ_CZ), rpy=(0.0, 0.0, yaw)),
            material=accent_yellow,
            name="hazard_stripes",
        )

    # ----- Telescoping armoured panels (two stages per side) -----
    door_0 = model.part("door_0")
    door_1 = model.part("door_1")
    door_0_outer = model.part("door_0_outer")
    door_1_outer = model.part("door_1_outer")

    inner_front_y = A_PLANE_Y - DOOR_THK / 2.0
    outer_front_y = B_PLANE_Y - DOOR_THK / 2.0

    groove_heights = ((0.30, "lo"), (0.55, "mid"), (0.80, "hi"))

    for part, side, tag in ((door_0, 1.0, "0"), (door_1, -1.0, "1")):
        part.visual(
            mesh_from_cadquery(_make_inner_panel(side), f"leaf_solid_{tag}"),
            origin=Origin(xyz=(0.0, A_PLANE_Y, LEAF_BOT_Z)),
            material=gunmetal,
            name="leaf_panel",
        )
        part.visual(
            mesh_from_cadquery(_make_inner_accent(side), f"leaf_accent_{tag}"),
            origin=Origin(xyz=(0.0, A_PLANE_Y, LEAF_BOT_Z)),
            material=accent_yellow,
            name="leaf_seam_accent",
        )
        groove_x0 = TOOTH + 0.09
        groove_x1 = A_OUT - 0.04
        groove_cx = side * (groove_x0 + groove_x1) / 2.0
        groove_w = groove_x1 - groove_x0
        for fz, gtag in groove_heights:
            part.visual(
                Box((groove_w, 0.008, 0.024)),
                origin=Origin(
                    xyz=(groove_cx, inner_front_y - 0.004, LEAF_BOT_Z + OPENING_H * fz)
                ),
                material=dark_steel,
                name=f"groove_{gtag}",
            )
        part.visual(
            Box((groove_w, 0.008, 0.16)),
            origin=Origin(xyz=(groove_cx, inner_front_y - 0.004, LEAF_BOT_Z + 0.10)),
            material=dark_steel,
            name="kick_plate",
        )

    for part, side, tag in ((door_0_outer, 1.0, "0"), (door_1_outer, -1.0, "1")):
        b_w = HALF_W - B_IN
        b_cx = side * (B_IN + HALF_W) / 2.0
        part.visual(
            Box((b_w, DOOR_THK, OPENING_H)),
            origin=Origin(xyz=(b_cx, B_PLANE_Y, LEAF_CZ)),
            material=gunmetal_deep,
            name="leaf_panel",
        )
        og_w = b_w - 0.08
        for fz, gtag in groove_heights:
            part.visual(
                Box((og_w, 0.008, 0.024)),
                origin=Origin(
                    xyz=(b_cx, outer_front_y - 0.004, LEAF_BOT_Z + OPENING_H * fz)
                ),
                material=dark_steel,
                name=f"groove_{gtag}",
            )
        part.visual(
            Box((og_w, 0.008, 0.16)),
            origin=Origin(xyz=(b_cx, outer_front_y - 0.004, LEAF_BOT_Z + 0.10)),
            material=dark_steel,
            name="kick_plate",
        )

    # ----- Fixed furniture attached rigidly to the frame -----
    for fixed_part, joint_name in (
        (left_housing, "frame_to_left_housing"),
        (right_housing, "frame_to_right_housing"),
        (keypad, "frame_to_keypad"),
        (haz_left, "frame_to_hazard_left"),
        (haz_right, "frame_to_hazard_right"),
    ):
        model.articulation(
            joint_name,
            ArticulationType.FIXED,
            parent=frame,
            child=fixed_part,
        )

    # ----- Articulations: telescoping prismatic slides -----
    model.articulation(
        "door_0_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.6, lower=0.0, upper=TRAVEL_A),
    )
    model.articulation(
        "door_1_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.6, lower=0.0, upper=TRAVEL_A),
        mimic=Mimic(joint="door_0_slide", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "door_0_outer_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_0_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.3, lower=0.0, upper=TRAVEL_B),
        mimic=Mimic(joint="door_0_slide", multiplier=0.5, offset=0.0),
    )
    model.articulation(
        "door_1_outer_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_1_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.3, lower=0.0, upper=TRAVEL_B),
        mimic=Mimic(joint="door_0_slide", multiplier=0.5, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_housing = object_model.get_part("left_housing")
    right_housing = object_model.get_part("right_housing")
    keypad = object_model.get_part("keypad")
    haz_left = object_model.get_part("hazard_left")
    haz_right = object_model.get_part("hazard_right")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    door_0_outer = object_model.get_part("door_0_outer")
    door_1_outer = object_model.get_part("door_1_outer")
    all_panels = (door_0, door_1, door_0_outer, door_1_outer)

    door_0_slide = object_model.get_articulation("door_0_slide")

    # --- Circular bulkhead ring: proportions and placement. ---
    ring_aabb = ctx.part_element_world_aabb(frame, elem="bulkhead_ring")
    assert ring_aabb is not None
    ring_dx = ring_aabb[1][0] - ring_aabb[0][0]
    ring_dz = ring_aabb[1][2] - ring_aabb[0][2]
    ctx.check(
        "bulkhead ring has circular proportions (not a rectangular frame)",
        abs(ring_dx - ring_dz) < 0.45,
        details=f"dx={ring_dx:.3f}, dz={ring_dz:.3f}",
    )
    ctx.check(
        "bulkhead ring extends past opening width on both sides",
        ring_aabb[0][0] < -HALF_W - 0.05 and ring_aabb[1][0] > HALF_W + 0.05,
        details=f"ring x=({ring_aabb[0][0]:.3f}, {ring_aabb[1][0]:.3f})",
    )
    ctx.check(
        "bulkhead ring extends above the opening",
        ring_aabb[1][2] > LEAF_BOT_Z + OPENING_H + 0.10,
        details=f"ring max z={ring_aabb[1][2]:.3f}",
    )
    ctx.check(
        "bulkhead ring sits on the floor (deck-trimmed)",
        ring_aabb[0][2] >= -0.005,
        details=f"ring min z={ring_aabb[0][2]:.3f}",
    )

    # --- Intentional interlock at the centre seam (closed pose). ---
    for ea in ("leaf_panel", "leaf_seam_accent"):
        for eb in ("leaf_panel", "leaf_seam_accent"):
            ctx.allow_overlap(
                door_0,
                door_1,
                elem_a=ea,
                elem_b=eb,
                reason=(
                    "The two armoured halves close along an interlocking zig-zag "
                    "seam; the complementary teeth and their yellow seam accents "
                    "interleave across the doorway centreline."
                ),
            )

    # Every panel rides captured in the fixed top/bottom guide rails.
    for panel in all_panels:
        for rail in ("top_guide", "bottom_guide"):
            ctx.allow_overlap(
                panel,
                frame,
                elem_a="leaf_panel",
                elem_b=rail,
                reason=f"The {panel.name} panel rides captured inside the {rail} rail.",
            )
    # Inner seam-panel accents also overlap the guide rails (captured channel).
    for inner_panel in (door_0, door_1):
        for rail in ("top_guide", "bottom_guide"):
            ctx.allow_overlap(
                inner_panel,
                frame,
                elem_a="leaf_seam_accent",
                elem_b=rail,
                reason=f"The {inner_panel.name} seam accent rides captured inside the {rail} rail.",
            )

    # Housing cowls and plates bolt through the bulkhead ring front plate.
    for housing in (left_housing, right_housing):
        for ha_elem in ("cowl_front", "cowl_plate", "pocket_end_wall"):
            ctx.allow_overlap(
                housing,
                frame,
                elem_a=ha_elem,
                elem_b="bulkhead_ring",
                reason="Housing structural pod is bolted through the bulkhead ring plate.",
            )
        for row in ("lo", "hi"):
            for col in ("in", "out"):
                ctx.allow_overlap(
                    housing,
                    frame,
                    elem_a=f"rivet_{row}_{col}",
                    elem_b="bulkhead_ring",
                    reason="Rivet heads sit flush on the cowl plate near the ring front face.",
                )

    # Keypad and hazard blocks seat against the ring front plate.
    ctx.allow_overlap(
        keypad,
        frame,
        elem_a="keypad_box",
        elem_b="bulkhead_ring",
        reason="Keypad box mounts flush on the cowl plate, seating against the ring.",
    )
    ctx.allow_overlap(
        keypad,
        frame,
        elem_a="status_lamp",
        elem_b="bulkhead_ring",
        reason="Status lamp sits proud on the cowl plate, near the ring front face.",
    )
    for haz, housing in (
        (haz_left, left_housing),
        (haz_right, right_housing),
    ):
        ctx.allow_overlap(
            haz,
            housing,
            reason="Hazard chevron block seats flush on the cowl plate near the lower rivet row.",
        )
    for haz in (haz_left, haz_right):
        ctx.allow_overlap(
            frame,
            haz,
            reason="Hazard chevron stripes sit proud of the cowl plate, near the ring front face.",
        )
    for haz in (haz_left, haz_right):
        ctx.allow_overlap(
            haz,
            frame,
            elem_a="hazard_back",
            elem_b="bulkhead_ring",
            reason="Hazard backing block mounts on the cowl plate near the ring face.",
        )

    # Each inner panel spans (almost) the full opening height when closed.
    ctx.expect_overlap(
        door_0,
        frame,
        axes="z",
        min_overlap=OPENING_H * 0.9,
        name="door_0 covers the opening height when closed",
    )
    ctx.expect_overlap(
        door_1,
        frame,
        axes="z",
        min_overlap=OPENING_H * 0.9,
        name="door_1 covers the opening height when closed",
    )

    # The two inner panels interlock across the full seam height.
    ctx.expect_overlap(
        door_0,
        door_1,
        axes="z",
        min_overlap=OPENING_H * 0.9,
        name="leaves interlock along the full seam height",
    )
    ctx.expect_overlap(
        door_0,
        door_1,
        axes="x",
        elem_a="leaf_panel",
        elem_b="leaf_panel",
        min_overlap=0.02,
        name="zig-zag teeth interlock across the centreline",
    )
    ctx.expect_overlap(
        door_0,
        door_0_outer,
        axes="x",
        elem_a="leaf_panel",
        elem_b="leaf_panel",
        min_overlap=0.01,
        name="right inner and outer stages lap with no see-through slit",
    )
    ctx.expect_overlap(
        door_1,
        door_1_outer,
        axes="x",
        elem_a="leaf_panel",
        elem_b="leaf_panel",
        min_overlap=0.01,
        name="left inner and outer stages lap with no see-through slit",
    )

    # Closed panels stand on the floor and ride the rails.
    for panel in all_panels:
        aabb = ctx.part_world_aabb(panel)
        assert aabb is not None
        ctx.check(
            f"{panel.name} stands on the floor (not below z=0)",
            aabb[0][2] >= -0.001,
            details=f"{panel.name} min z = {aabb[0][2]:.4f}",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="leaf_panel",
            elem_b="top_guide",
            min_overlap=0.008,
            name=f"{panel.name} is captured by the top guide rail",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="leaf_panel",
            elem_b="bottom_guide",
            min_overlap=0.008,
            name=f"{panel.name} is captured by the bottom guide rail",
        )

    # The doorway is a true through-opening: remaining frame elements stay
    # clear of the central window (the bulkhead ring has a rectangular bore
    # that exposes the full opening).
    window_x = 0.5
    window_z = (0.4, 1.6)
    for elem in ("header_trim", "top_guide", "bottom_guide"):
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

    # Fixed furniture is mounted to the frame/housings, not floating.
    ctx.expect_contact(keypad, left_housing, name="keypad mounted on left cowl")
    ctx.expect_contact(left_housing, frame, name="left housing mounted to frame")
    ctx.expect_contact(right_housing, frame, name="right housing mounted to frame")
    ctx.expect_contact(haz_left, left_housing, name="left hazard on left cowl")
    ctx.expect_contact(haz_right, right_housing, name="right hazard on right cowl")

    # Closed-pose panel centre positions for the open-pose comparison.
    closed_c = {}
    for panel in all_panels:
        pos = ctx.part_world_position(panel)
        assert pos is not None
        closed_c[panel.name] = pos

    # --- Open pose: drive the source joint; mimics telescope the rest. ---
    travel = door_0_slide.motion_limits.upper
    assert travel is not None
    pocket_outer_x = HALF_W + HOUSING_W
    with ctx.pose({door_0_slide: travel}):
        for panel, sign in (
            (door_0, +1.0),
            (door_1, -1.0),
            (door_0_outer, +1.0),
            (door_1_outer, -1.0),
        ):
            pos = ctx.part_world_position(panel)
            assert pos is not None
            moved = sign * (pos[0] - closed_c[panel.name][0])
            ctx.check(
                f"{panel.name} retracts toward {'+' if sign > 0 else '-'}X when open",
                moved > 0.3,
                details=f"closed x={closed_c[panel.name][0]:.3f}, open x={pos[0]:.3f}",
            )
            aabb = ctx.part_world_aabb(panel)
            assert aabb is not None
            ctx.check(
                f"{panel.name} stays inside its housing pocket when open",
                aabb[0][0] >= -pocket_outer_x - 0.001
                and aabb[1][0] <= pocket_outer_x + 0.001,
                details=f"{panel.name} open aabb x=({aabb[0][0]:.3f}, {aabb[1][0]:.3f})",
            )

        ctx.expect_gap(
            door_0,
            door_1,
            axis="x",
            min_gap=0.80,
            name="leaves open to a wide clear doorway",
        )

    return ctx.report()


object_model = build_object_model()
