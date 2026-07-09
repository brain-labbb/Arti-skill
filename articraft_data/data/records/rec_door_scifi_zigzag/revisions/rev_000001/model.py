from __future__ import annotations

# Sci-fi blast / security door.
#
# Two dark gunmetal halves meet along a yellow-edged zig-zag interlocking seam
# down the middle and retract sideways into side housings to open. Each half is
# a two-stage telescoping pair (inner seam panel + outer panel in a second
# plane) so the whole door pockets fully inside its side housing when open.
#
# Articraft brief:
# - Object: armoured sci-fi blast door, ~1.62 m clear width x ~2.05 m tall,
#   standing on the ground plane (geometry from z=0 upward). Viewer looks from
#   -Y; the doorway is a true through-opening (no back wall).
# - Root/support: `frame` is the fixed structural surround (lintel header beam,
#   floor sill, top/bottom guide rails). It carries the two fixed side
#   housings, the wall keypad on the left jamb, and the hazard blocks.
# - Parts: frame (root, fixed); left_housing / right_housing (FIXED) are hollow
#   side pockets (front cowl + rear rail) that double as the jambs and as the
#   cavities the leaves retract into; keypad, hazard_left, hazard_right
#   (FIXED); door_0 / door_1, the two inner armoured seam panels with
#   complementary zig-zag leading edges; door_0_outer / door_1_outer, the outer
#   telescoping panels riding in a second plane behind the inner ones.
# - Articulations: door_0_slide (PRISMATIC, +X) drives the right inner panel;
#   door_1_slide (-X), door_0_outer_slide (+X, 0.5x) and door_1_outer_slide
#   (-X, 0.5x) are mimic-coupled so one travel value telescopes all four
#   panels open symmetrically.
# - Visible geometry: one shared zig-zag seam line splits the inner panel pair
#   into complementary halves (CadQuery), so the closed door mates
#   tooth-into-notch with only a thin running clearance; yellow accent strips
#   hug the seam; lintel header with trim band and warning lamps, keypad with
#   screen + keys + status lamp, yellow/black hazard chevrons, riveted cowl
#   plates, panel grooves, and kick plates.
# - Support/fit: every panel is captured top and bottom in fixed guide rails
#   (small proven overlap) for its whole travel, and nests fully inside its
#   hollow side housing when open (proven by pose).
# - Intentional overlaps: the closed zig-zag teeth interleave across the
#   centreline (AABB overlap, allowed + proven); panel tops/bottoms ride
#   captured in the guide rails (allowed + proven).
# - Tests: the panel pairs span the opening when closed, the zig-zag teeth
#   interlock, the doorway window is unobstructed (no back wall), all panels
#   retract into their housings when open without poking out, fixed furniture
#   stays mounted, and the door stands on the floor.
# - Assumptions: industrial gunmetal armour, yellow safety accents, single
#   centre seam, no powered internals modelled.
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

FRAME_DEPTH = 0.34  # full wall surround depth (along Y)
HEADER_H = 0.22  # lintel header height
SILL_H = 0.05  # floor sill height

HOUSING_W = 0.51  # side housing / pocket width (X): swallows the open panels
REAR_RAIL_THK = 0.05  # rear rail thickness (Y)

# Leaf geometry.
HALF_W = OPENING_W / 2.0  # 0.81 : half the clear opening
TOOTH = 0.09  # zig-zag tooth half-amplitude (X) — sharp angular interlock
SEAM_CLEAR = 0.002  # running clearance per side of the shared seam line

# Telescoping split per side: the inner (seam) panel spans the centre seam out
# to A_OUT; the outer panel spans B_IN..HALF_W in a second plane, lapping the
# inner panel by 0.02 so no see-through slit opens between the stages.
A_OUT = 0.42  # outer edge of the inner seam panel
B_IN = 0.40  # inner edge of the outer panel (0.02 lap behind the inner one)

# Door travel. The inner panel moves TRAVEL_A; the outer panel mimics at half
# rate, so both finish nested together inside the housing pocket.
TRAVEL_A = 0.84
TRAVEL_B = TRAVEL_A / 2.0

# Two sliding planes inside the housing channel.
A_PLANE_Y = -0.045  # inner seam panels (front plane)
B_PLANE_Y = 0.035  # outer panels (rear plane)

# Housing channel: front cowl ahead of the panel planes, rear rail behind.
COWL_FRONT_Y = -0.115  # cowl sits in front of the leaf channel
COWL_THK = 0.05
COWL_PLATE_THK = 0.025
COWL_PLATE_Y = COWL_FRONT_Y - COWL_THK / 2.0 - COWL_PLATE_THK / 2.0
COWL_PLATE_FRONT_Y = COWL_PLATE_Y - COWL_PLATE_THK / 2.0  # most -Y face
REAR_RAIL_Y = 0.11  # rear rail sits behind the leaf channel

# Top/bottom guide rails (fixed): capture every panel for its whole travel.
GUIDE_HALF_LEN = HALF_W + HOUSING_W - 0.03  # reaches into both pockets
GUIDE_DEPTH = 0.16  # Y extent, enclosing both sliding planes
GUIDE_CY = (A_PLANE_Y + B_PLANE_Y) / 2.0
GUIDE_CAPTURE = 0.012  # how deep the panels ride into each rail
LEAF_BOT_Z = SILL_H
LEAF_TOP_Z = SILL_H + OPENING_H
TOP_GUIDE_H = 0.06
TOP_GUIDE_CZ = LEAF_TOP_Z - GUIDE_CAPTURE + TOP_GUIDE_H / 2.0
BOT_GUIDE_H = 0.036
BOT_GUIDE_CZ = LEAF_BOT_Z + GUIDE_CAPTURE - BOT_GUIDE_H / 2.0

# Vertical centre of the leaf panels.
LEAF_CZ = SILL_H + OPENING_H / 2.0


# ---------------------------------------------------------------------------
# Zig-zag leaf geometry (CadQuery). One shared seam line x_s(z) splits the
# inner panel pair into two complementary halves: the right panel occupies
# x >= x_s(z) + SEAM_CLEAR and the left panel x <= x_s(z) - SEAM_CLEAR, so the
# closed halves mate tooth-into-notch and read as one continuous armoured door
# with a thin seam.
# ---------------------------------------------------------------------------
def _seam_points() -> list[tuple[float, float]]:
    """Shared zig-zag seam-line (x, z) knots over the opening height (5 knots)."""
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
    """One inner seam panel as a CadQuery solid.

    ``side=+1`` builds the right panel (body toward +X, edge x_s + clearance);
    ``side=-1`` builds the left panel (body toward -X, edge x_s - clearance).
    """
    h = OPENING_H
    edge_x = side * A_OUT
    pts = [(x + side * SEAM_CLEAR, z) for (x, z) in _seam_points()]  # bottom -> top
    pts.append((edge_x, h))  # top outer corner
    pts.append((edge_x, 0.0))  # bottom outer corner
    profile = cq.Workplane("XZ").polyline(pts).close()
    leaf = profile.extrude(DOOR_THK)
    # "XZ" workplane normal is -Y; recentre thickness to straddle Y=0.
    leaf = leaf.translate((0.0, DOOR_THK / 2.0, 0.0))
    return leaf


def _make_inner_accent(side: float) -> cq.Workplane:
    """Thin yellow ribbon hugging the seam edge of one inner panel."""
    strip = 0.06  # bold yellow accent line hugging the zig-zag seam
    outer = [(x + side * SEAM_CLEAR, z) for (x, z) in _seam_points()]
    inner = [(x + side * strip, z) for (x, z) in outer]
    pts = outer + list(reversed(inner))
    # A thin ribbon straddling the panel front face so the bright seam line
    # reads clearly from the viewer (-Y) side and stands slightly proud.
    ribbon_thk = DOOR_THK * 0.5
    ribbon = cq.Workplane("XZ").polyline(pts).close().extrude(ribbon_thk)
    # "XZ" extrude runs toward -Y -> Y in [-ribbon_thk, 0]; nudge so it crosses
    # the panel front face (front is at Y=-DOOR_THK/2) and stands 6 mm proud.
    ribbon = ribbon.translate((0.0, -DOOR_THK / 2.0 + ribbon_thk - 0.006, 0.0))
    return ribbon


# ---------------------------------------------------------------------------
# Hazard chevron stripes (CadQuery). A set of diagonal yellow bands clipped to
# the rectangular block face so they read as proper hazard tape with no ragged
# overshoot. Built in a local frame: rectangle on the XZ plane centred on the
# origin, slashes running bottom-left -> top-right; thin extrusion toward -Y so
# the stripes stand proud of the block front face.
# ---------------------------------------------------------------------------
def _make_hazard_stripes(block_w: float, block_h: float) -> cq.Workplane:
    band = 0.05  # band width across the slash
    gap = band  # equal yellow/black banding
    thk = 0.008  # proud thickness of the yellow bands
    diag = (block_w + block_h)  # long enough to cross the whole rectangle
    # Build the diagonal slashes as long rotated bars, unioned together.
    slashes: cq.Workplane | None = None
    step = band + gap
    n = int(diag / step) + 2
    for k in range(-n, n + 1):
        # A long bar centred on the line x - z = c (45 deg), offset by c.
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
    # Clip to the block face rectangle (a thin slab) so nothing overshoots.
    clip = cq.Workplane("XZ").rect(block_w, block_h).extrude(thk * 2.0)
    clip = clip.translate((0.0, thk * 0.5, 0.0))
    stripes = slashes.intersect(clip)
    return stripes


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

    header_w = OPENING_W + 2.0 * HOUSING_W

    # ----- Root: fixed structural frame (header + sill + guide rails) -----
    frame = model.part("frame")

    # Lintel header beam across the top.
    frame.visual(
        Box((header_w, FRAME_DEPTH, HEADER_H)),
        origin=Origin(xyz=(0.0, 0.0, SILL_H + OPENING_H + HEADER_H / 2.0)),
        material=olive_frame,
        name="lintel_header",
    )
    # Worn metal trim band on the header front face.
    frame.visual(
        Box((header_w * 0.96, 0.03, HEADER_H * 0.5)),
        origin=Origin(xyz=(0.0, -FRAME_DEPTH / 2.0 - 0.015, SILL_H + OPENING_H + HEADER_H * 0.55)),
        material=dark_steel,
        name="header_trim",
    )
    # Floor sill across the bottom.
    frame.visual(
        Box((header_w, FRAME_DEPTH, SILL_H)),
        origin=Origin(xyz=(0.0, 0.0, SILL_H / 2.0)),
        material=olive_frame,
        name="sill",
    )
    # No back plate: the doorway is a true through-opening, so an open door
    # reveals empty space behind it.
    # Top guide rail recessed into the lintel underside: captures every panel
    # top for its whole travel (spans the opening and both pockets).
    frame.visual(
        Box((2.0 * GUIDE_HALF_LEN, GUIDE_DEPTH, TOP_GUIDE_H)),
        origin=Origin(xyz=(0.0, GUIDE_CY, TOP_GUIDE_CZ)),
        material=dark_steel,
        name="top_guide",
    )
    # Bottom guide rail recessed into the sill top: captures every panel bottom.
    frame.visual(
        Box((2.0 * GUIDE_HALF_LEN, GUIDE_DEPTH, BOT_GUIDE_H)),
        origin=Origin(xyz=(0.0, GUIDE_CY, BOT_GUIDE_CZ)),
        material=dark_steel,
        name="bottom_guide",
    )
    # Twin amber warning lamps seated flush on the header trim band.
    trim_front_y = -FRAME_DEPTH / 2.0 - 0.03  # front face of the header trim
    trim_cz = SILL_H + OPENING_H + HEADER_H * 0.55
    for s, lamp_name in ((-1.0, "warning_lamp_l"), (1.0, "warning_lamp_r")):
        frame.visual(
            Box((0.10, 0.012, 0.05)),
            origin=Origin(xyz=(s * 0.55, trim_front_y - 0.006, trim_cz)),
            material=accent_yellow,
            name=lamp_name,
        )

    # ----- Fixed side housings: hollow jamb pockets (front cowl + rear rail) --
    # Each housing is a pocket spanning the side; its inner X edge sits at the
    # opening face so the closed leaf does not collide, and its hollow channel
    # (between cowl and rear rail) swallows the telescoped panels when open.
    left_housing = model.part("left_housing")
    right_housing = model.part("right_housing")
    housing_cz = SILL_H + OPENING_H / 2.0
    housing_h = OPENING_H
    cowl_h = OPENING_H - 0.02
    pocket_inner_x = HALF_W  # inner face at the opening edge
    pocket_outer_x = HALF_W + HOUSING_W
    for part, s in ((left_housing, -1.0), (right_housing, 1.0)):
        cx = s * (pocket_inner_x + HOUSING_W / 2.0)
        # Front cowl (the broad dark cowl pod in the reference).
        part.visual(
            Box((HOUSING_W, COWL_THK, cowl_h)),
            origin=Origin(xyz=(cx, COWL_FRONT_Y, housing_cz)),
            material=olive_frame,
            name="cowl_front",
        )
        # Darker proud cowl face plate.
        part.visual(
            Box((HOUSING_W * 0.86, COWL_PLATE_THK, cowl_h * 0.8)),
            origin=Origin(xyz=(cx, COWL_PLATE_Y, housing_cz)),
            material=dark_steel,
            name="cowl_plate",
        )
        # Rear rail closing the back of the pocket.
        part.visual(
            Box((HOUSING_W, REAR_RAIL_THK, housing_h)),
            origin=Origin(xyz=(cx, REAR_RAIL_Y, housing_cz)),
            material=olive_frame,
            name="rear_rail",
        )
        # Outer end wall tying cowl and rail together (closes the pocket end).
        end_x = s * (pocket_outer_x - 0.015)
        part.visual(
            Box((0.03, REAR_RAIL_Y - COWL_FRONT_Y + REAR_RAIL_THK, housing_h)),
            origin=Origin(xyz=(end_x, (COWL_FRONT_Y + REAR_RAIL_Y) / 2.0, housing_cz)),
            material=olive_frame,
            name="pocket_end_wall",
        )
        # Rivet studs seated flush on the cowl plate front face (clear of the
        # keypad band around z=1.2-1.4 and the hazard blocks below z=0.35).
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
    # Box back face seats flush on the cowl plate front face.
    kp_box_y = COWL_PLATE_FRONT_Y - kp_box_thk / 2.0
    kp_z = SILL_H + OPENING_H * 0.6
    keypad.visual(
        Box((0.12, kp_box_thk, 0.18)),
        origin=Origin(xyz=(kp_cx, kp_box_y, kp_z)),
        material=keypad_body,
        name="keypad_box",
    )
    kp_face_y = kp_box_y - kp_box_thk / 2.0  # front face of the keypad box
    # Screen near the top, embedded into the box front face.
    keypad.visual(
        Box((0.085, 0.014, 0.045)),
        origin=Origin(xyz=(kp_cx, kp_face_y + 0.004, kp_z + 0.05)),
        material=keypad_screen,
        name="keypad_screen",
    )
    # 3x3 key grid, each key embedded into the box front face.
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
    # Green door-status lamp just above the keypad, flush on the cowl plate.
    keypad.visual(
        Box((0.05, 0.014, 0.025)),
        origin=Origin(xyz=(kp_cx, COWL_PLATE_FRONT_Y - 0.007, kp_z + 0.13)),
        material=status_green,
        name="status_lamp",
    )

    # ----- Fixed hazard chevrons at the base corners (on the cowl fronts) -----
    # Reference: bold yellow/black DIAGONAL hazard striping on a square black
    # block at each lower corner. Stripes are tilted ~45 deg (about the depth
    # axis Y) so they read as proper diagonal hazard banding rather than bars.
    haz_left = model.part("hazard_left")
    haz_right = model.part("hazard_right")
    haz_h = 0.30
    haz_w = 0.26
    haz_thk = 0.02
    # Hazard backing block seats flush on the cowl plate front face.
    haz_back_y = COWL_PLATE_FRONT_Y - haz_thk / 2.0
    haz_face_y = haz_back_y - haz_thk / 2.0  # block front face
    haz_cz = SILL_H + haz_h / 2.0 + 0.22
    # Diagonal yellow hazard bands, clipped to the block rectangle (no overshoot).
    haz_stripes_mesh = _make_hazard_stripes(haz_w * 0.94, haz_h * 0.94)
    for part, s, name in ((haz_left, -1.0, "L"), (haz_right, 1.0, "R")):
        cx = s * (pocket_inner_x + HOUSING_W / 2.0)
        part.visual(
            Box((haz_w, haz_thk, haz_h)),
            origin=Origin(xyz=(cx, haz_back_y, haz_cz)),
            material=hazard_black,
            name="hazard_back",
        )
        # Mirror the slash slant on the right block (yaw 180) so both corners
        # chevron toward the doorway centre. Stripes sit proud of the block face.
        yaw = 0.0 if s < 0 else math.pi
        part.visual(
            mesh_from_cadquery(haz_stripes_mesh, f"hazard_stripes_{name}"),
            origin=Origin(xyz=(cx, haz_face_y, haz_cz), rpy=(0.0, 0.0, yaw)),
            material=accent_yellow,
            name="hazard_stripes",
        )

    # ----- Telescoping armoured panels (two stages per side) -----
    door_0 = model.part("door_0")  # right inner seam panel (front plane)
    door_1 = model.part("door_1")  # left inner seam panel (front plane)
    door_0_outer = model.part("door_0_outer")  # right outer panel (rear plane)
    door_1_outer = model.part("door_1_outer")  # left outer panel (rear plane)

    inner_front_y = A_PLANE_Y - DOOR_THK / 2.0  # inner panel front face plane
    outer_front_y = B_PLANE_Y - DOOR_THK / 2.0  # outer panel front face plane

    groove_heights = ((0.30, "lo"), (0.55, "mid"), (0.80, "hi"))

    for part, side, tag in ((door_0, 1.0, "0"), (door_1, -1.0, "1")):
        part.visual(
            mesh_from_cadquery(_make_inner_panel(side), f"leaf_solid_{tag}"),
            origin=Origin(xyz=(0.0, A_PLANE_Y, SILL_H)),
            material=gunmetal,
            name="leaf_panel",
        )
        part.visual(
            mesh_from_cadquery(_make_inner_accent(side), f"leaf_accent_{tag}"),
            origin=Origin(xyz=(0.0, A_PLANE_Y, SILL_H)),
            material=accent_yellow,
            name="leaf_seam_accent",
        )
        # Recessed-looking horizontal panel grooves on the panel front face,
        # kept outboard of the zig-zag accent zone (|x| > TOOTH + accent strip).
        groove_x0 = TOOTH + 0.09
        groove_x1 = A_OUT - 0.04
        groove_cx = side * (groove_x0 + groove_x1) / 2.0
        groove_w = groove_x1 - groove_x0
        for fz, gtag in groove_heights:
            part.visual(
                Box((groove_w, 0.008, 0.024)),
                origin=Origin(
                    xyz=(groove_cx, inner_front_y - 0.004, SILL_H + OPENING_H * fz)
                ),
                material=dark_steel,
                name=f"groove_{gtag}",
            )
        # Dark kick plate band along the panel bottom.
        part.visual(
            Box((groove_w, 0.008, 0.16)),
            origin=Origin(xyz=(groove_cx, inner_front_y - 0.004, SILL_H + 0.10)),
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
        # Matching horizontal grooves so the stage joint reads as panel lines.
        og_w = b_w - 0.08
        for fz, gtag in groove_heights:
            part.visual(
                Box((og_w, 0.008, 0.024)),
                origin=Origin(
                    xyz=(b_cx, outer_front_y - 0.004, SILL_H + OPENING_H * fz)
                ),
                material=dark_steel,
                name=f"groove_{gtag}",
            )
        part.visual(
            Box((og_w, 0.008, 0.16)),
            origin=Origin(xyz=(b_cx, outer_front_y - 0.004, SILL_H + 0.10)),
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

    # ----- Articulations: telescoping prismatic slides (one driver + mimics) --
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

    # --- Intentional interlock at the centre seam (closed pose). ---
    # The complementary zig-zag teeth interleave across the centreline (their
    # AABBs overlap even though the solids only share a thin running
    # clearance); allow the seam pairing between the two inner panels.
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

    # Every panel rides captured in the fixed top/bottom guide rails for its
    # whole travel (a small intentional overlap models the captured channel).
    for panel in all_panels:
        for rail in ("top_guide", "bottom_guide"):
            ctx.allow_overlap(
                panel,
                frame,
                elem_a="leaf_panel",
                elem_b=rail,
                reason=f"The {panel.name} panel rides captured inside the {rail} rail.",
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
    # The interlocking teeth actually meet across the centreline (X overlap).
    ctx.expect_overlap(
        door_0,
        door_1,
        axes="x",
        elem_a="leaf_panel",
        elem_b="leaf_panel",
        min_overlap=0.02,
        name="zig-zag teeth interlock across the centreline",
    )
    # The telescoping stages lap each other in X so no slit opens between them.
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

    # Closed panels stand on the floor (not below z=0) and ride the rails.
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

    # The doorway is a true through-opening: no fixed frame geometry (e.g. a
    # back wall) blocks the central window of the opening.
    window_x = 0.5
    window_z = (0.4, 1.6)
    for elem in ("lintel_header", "header_trim", "sill", "top_guide", "bottom_guide"):
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

    # Fixed furniture is mounted to the frame, not floating.
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
            # The open panel nests fully inside its housing pocket: it never
            # pokes out past the pocket end wall.
            aabb = ctx.part_world_aabb(panel)
            assert aabb is not None
            ctx.check(
                f"{panel.name} stays inside its housing pocket when open",
                aabb[0][0] >= -pocket_outer_x - 0.001
                and aabb[1][0] <= pocket_outer_x + 0.001,
                details=f"{panel.name} open aabb x=({aabb[0][0]:.3f}, {aabb[1][0]:.3f})",
            )

        # Open panels no longer meet: a wide clear doorway opens at the centre.
        ctx.expect_gap(
            door_0,
            door_1,
            axis="x",
            min_gap=0.80,
            name="leaves open to a wide clear doorway",
        )

    return ctx.report()


object_model = build_object_model()
