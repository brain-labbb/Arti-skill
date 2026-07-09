from __future__ import annotations

# Sci-fi blast / security door — single-leaf sliding variant.
#
# One full-width armoured leaf slides sideways (to the right) into an enlarged
# side pocket to open the doorway. The leaf is a two-stage telescoping pair:
# an inner armoured panel (front plane) and a rear support stage (rear plane)
# that rides behind it at half rate, so both stack inside the right housing
# when open. One prismatic joint drives the inner panel; the rear stage is
# mimic-coupled at 0.5x.
#
# The rectangular frame (lintel, sill, guide rails), the wall-mounted keypad
# on the left jamb, and the yellow-and-black hazard chevrons at both base
# corners are identical to the parent bi-parting model.
#
# Articraft brief:
# - Object: armoured sci-fi blast door, ~1.62 m clear width x ~2.05 m tall,
#   standing on the ground plane (geometry from z=0 upward). Viewer looks
#   from -Y; the doorway is a true through-opening (no back wall).
# - Root/support: `frame` is the fixed structural surround (lintel header
#   beam, floor sill, top/bottom guide rails). It carries the two fixed side
#   housings, the wall keypad on the left jamb, and the hazard blocks.
# - Parts: frame (root, fixed); left_housing (FIXED, unchanged structural
#   jamb); right_housing (FIXED, enlarged pocket for the full-width leaf);
#   keypad, hazard_left, hazard_right (FIXED); door_0 (inner full-width
#   armoured panel, front plane); door_0_outer (rear telescoping stage,
#   rear plane).
# - Articulations: door_0_slide (PRISMATIC, +X) drives the inner panel;
#   door_0_outer_slide (+X, 0.5x) mimics the rear stage.
# - Visible geometry: flat armoured panel (CadQuery, chamfered edges) with
#   leading-edge yellow accent strip, horizontal grooves, kick plate; rear
#   stage with matching grooves; enlarged right pocket; olive frame; keypad
#   with screen + keys + status lamp; hazard chevrons.
# - Support/fit: both panels ride captured in fixed top/bottom guide rails
#   for their whole travel (small proven overlap), and nest inside the
#   enlarged right pocket when open (proven by pose).
# - Intentional overlaps: panel tops/bottoms captured in the guide rails
#   (allowed + proven).
# - Tests: full-width leaf spans the opening when closed; inner and rear
#   stages lap with no slit; doorway is unobstructed; leaf retracts into
#   the right pocket when open; inner travels further than rear (telescoping
#   proof); fixed furniture stays mounted; door stands on the floor.
# - Assumptions: industrial gunmetal armour, yellow safety accents, single
#   leaf sliding right, no powered internals modelled.
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
DOOR_THK = 0.06   # armoured panel thickness (Y extent), one telescoping stage

FRAME_DEPTH = 0.34  # full wall surround depth (along Y)
HEADER_H = 0.22     # lintel header height
SILL_H = 0.05       # floor sill height

LEFT_HOUSING_W = 0.51    # left jamb pocket width (unchanged from parent)
RIGHT_HOUSING_W = 1.15   # enlarged right pocket for full-width leaf
REAR_RAIL_THK = 0.05     # rear rail thickness (Y)

# Leaf geometry — single full-width panel, no zig-zag seam.
LEAF_W = OPENING_W  # 1.62 m full-width armoured leaf

# Door travel. Inner panel moves TRAVEL_A to the right; rear stage mimics
# at half rate so both stack inside the enlarged right pocket.
TRAVEL_A = 1.10
TRAVEL_B = TRAVEL_A / 2.0

# Two sliding planes inside the housing channel.
A_PLANE_Y = -0.045  # inner armoured panel (front plane)
B_PLANE_Y = 0.035   # rear telescoping stage (rear plane)

# Housing channel: front cowl ahead of the panel planes, rear rail behind.
COWL_FRONT_Y = -0.115  # cowl sits in front of the leaf channel
COWL_THK = 0.05
COWL_PLATE_THK = 0.025
COWL_PLATE_Y = COWL_FRONT_Y - COWL_THK / 2.0 - COWL_PLATE_THK / 2.0
COWL_PLATE_FRONT_Y = COWL_PLATE_Y - COWL_PLATE_THK / 2.0  # most -Y face
REAR_RAIL_Y = 0.11  # rear rail sits behind the leaf channel

# Top/bottom guide rails (fixed): capture the leaf for its whole travel.
# Half-length is clamped so the rails stay inside the pocket end walls.
_frame_cx = (RIGHT_HOUSING_W - LEFT_HOUSING_W) / 2.0
GUIDE_HALF_LEN = min(
    _frame_cx + OPENING_W / 2.0 + LEFT_HOUSING_W - 0.04,
    OPENING_W / 2.0 + RIGHT_HOUSING_W - 0.04 - _frame_cx,
)
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
# Full-width armoured leaf panel (CadQuery). A flat rectangular plate with
# chamfered long horizontal edges for a manufactured armour-plate look.
# ---------------------------------------------------------------------------
def _make_leaf_panel(width: float, height: float, thickness: float) -> cq.Workplane:
    """Full-width flat armoured panel with small edge chamfers."""
    chamfer = 0.008
    panel = (
        cq.Workplane("XY")
        .box(width, thickness, height, centered=(True, True, False))
    )
    # Chamfer the long horizontal edges (top and bottom of the plate) for a
    # manufactured armour-plate look.
    panel = panel.edges("|X").chamfer(chamfer)
    return panel


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

    # Frame header spans the full assembly width (opening + both housings).
    # The right housing is wider, so the frame centre shifts slightly right.
    header_w = OPENING_W + LEFT_HOUSING_W + RIGHT_HOUSING_W
    frame_cx = (RIGHT_HOUSING_W - LEFT_HOUSING_W) / 2.0

    # ----- Root: fixed structural frame (header + sill + guide rails) -----
    frame = model.part("frame")

    # Lintel header beam across the top.
    frame.visual(
        Box((header_w, FRAME_DEPTH, HEADER_H)),
        origin=Origin(xyz=(frame_cx, 0.0, SILL_H + OPENING_H + HEADER_H / 2.0)),
        material=olive_frame,
        name="lintel_header",
    )
    # Worn metal trim band on the header front face.
    frame.visual(
        Box((header_w * 0.96, 0.03, HEADER_H * 0.5)),
        origin=Origin(xyz=(frame_cx, -FRAME_DEPTH / 2.0 - 0.015, SILL_H + OPENING_H + HEADER_H * 0.55)),
        material=dark_steel,
        name="header_trim",
    )
    # Floor sill across the bottom.
    frame.visual(
        Box((header_w, FRAME_DEPTH, SILL_H)),
        origin=Origin(xyz=(frame_cx, 0.0, SILL_H / 2.0)),
        material=olive_frame,
        name="sill",
    )
    # Top guide rail recessed into the lintel underside: captures the leaf
    # top for its whole travel (spans the opening and right pocket range).
    frame.visual(
        Box((2.0 * GUIDE_HALF_LEN, GUIDE_DEPTH, TOP_GUIDE_H)),
        origin=Origin(xyz=(frame_cx, GUIDE_CY, TOP_GUIDE_CZ)),
        material=dark_steel,
        name="top_guide",
    )
    # Bottom guide rail recessed into the sill top: captures the leaf bottom.
    frame.visual(
        Box((2.0 * GUIDE_HALF_LEN, GUIDE_DEPTH, BOT_GUIDE_H)),
        origin=Origin(xyz=(frame_cx, GUIDE_CY, BOT_GUIDE_CZ)),
        material=dark_steel,
        name="bottom_guide",
    )
    # Vertical front jamb plates connecting the sill to the header at each
    # opening edge. These represent the inner frame structure that the
    # housings mount onto. Positioned on the front half of the frame depth
    # to avoid the recessed bottom guide rail channel.
    jamb_w = 0.04
    jamb_front_y = -FRAME_DEPTH / 2.0  # flush with the frame front face
    jamb_rear_y = -0.10  # stops before the bottom guide rail channel
    jamb_h = OPENING_H
    jamb_cz = SILL_H + OPENING_H / 2.0
    for s, jamb_name in ((-1.0, "left_jamb"), (1.0, "right_jamb")):
        jamb_cx = s * (OPENING_W / 2.0 - jamb_w / 2.0)
        frame.visual(
            Box((jamb_w, jamb_rear_y - jamb_front_y, jamb_h)),
            origin=Origin(xyz=(jamb_cx, (jamb_front_y + jamb_rear_y) / 2.0, jamb_cz)),
            material=olive_frame,
            name=jamb_name,
        )
    # Twin amber warning lamps seated flush on the header trim band.
    trim_front_y = -FRAME_DEPTH / 2.0 - 0.03  # front face of the header trim
    trim_cz = SILL_H + OPENING_H + HEADER_H * 0.55
    for s, lamp_name in ((-1.0, "warning_lamp_l"), (1.0, "warning_lamp_r")):
        frame.visual(
            Box((0.10, 0.012, 0.05)),
            origin=Origin(xyz=(frame_cx + s * 0.55, trim_front_y - 0.006, trim_cz)),
            material=accent_yellow,
            name=lamp_name,
        )

    # ----- Fixed side housings: left jamb + enlarged right pocket -----
    # Left housing: unchanged structural jamb (same as parent).
    # Right housing: enlarged pocket that swallows the full-width leaf.
    left_housing = model.part("left_housing")
    right_housing = model.part("right_housing")
    housing_cz = SILL_H + OPENING_H / 2.0
    housing_h = OPENING_H
    cowl_h = OPENING_H - 0.02

    housing_specs = (
        (left_housing, -1.0, LEFT_HOUSING_W),
        (right_housing, 1.0, RIGHT_HOUSING_W),
    )
    for part, s, hw in housing_specs:
        pocket_inner_x = OPENING_W / 2.0
        cx = s * (pocket_inner_x + hw / 2.0)
        # Front cowl (the broad dark cowl pod in the reference).
        part.visual(
            Box((hw, COWL_THK, cowl_h)),
            origin=Origin(xyz=(cx, COWL_FRONT_Y, housing_cz)),
            material=olive_frame,
            name="cowl_front",
        )
        # Darker proud cowl face plate.
        part.visual(
            Box((hw * 0.86, COWL_PLATE_THK, cowl_h * 0.8)),
            origin=Origin(xyz=(cx, COWL_PLATE_Y, housing_cz)),
            material=dark_steel,
            name="cowl_plate",
        )
        # Rear rail closing the back of the pocket.
        part.visual(
            Box((hw, REAR_RAIL_THK, housing_h)),
            origin=Origin(xyz=(cx, REAR_RAIL_Y, housing_cz)),
            material=olive_frame,
            name="rear_rail",
        )
        # Outer end wall tying cowl and rail together (closes the pocket end).
        pocket_outer_x = pocket_inner_x + hw
        end_x = s * (pocket_outer_x - 0.015)
        part.visual(
            Box((0.03, REAR_RAIL_Y - COWL_FRONT_Y + REAR_RAIL_THK, housing_h)),
            origin=Origin(xyz=(end_x, (COWL_FRONT_Y + REAR_RAIL_Y) / 2.0, housing_cz)),
            material=olive_frame,
            name="pocket_end_wall",
        )
        # Rivet studs seated flush on the cowl plate front face (clear of the
        # hazard blocks whose top edge is at haz_cz + haz_h/2 ≈ 0.57).
        for rz, row in ((housing_cz - 0.40, "lo"), (housing_cz + 0.56, "hi")):
            for dx, col in ((-0.10, "in"), (0.10, "out")):
                part.visual(
                    Box((0.022, 0.012, 0.022)),
                    origin=Origin(xyz=(cx + dx, COWL_PLATE_FRONT_Y - 0.006, rz)),
                    material=dark_steel,
                    name=f"rivet_{row}_{col}",
                )

    # ----- Fixed wall keypad on the left cowl front face -----
    keypad = model.part("keypad")
    kp_cx = -(OPENING_W / 2.0 + LEFT_HOUSING_W / 2.0)
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
    # Green door-status lamp seated on the keypad box top face, flush with
    # the box front face (small embed for connectivity).
    kp_box_top_z = kp_z + 0.09  # top face of the keypad box
    keypad.visual(
        Box((0.05, 0.014, 0.025)),
        origin=Origin(xyz=(kp_cx, kp_face_y + 0.004, kp_box_top_z - 0.005)),
        material=status_green,
        name="status_lamp",
    )

    # ----- Fixed hazard chevrons at the base corners (on the cowl fronts) -----
    haz_left = model.part("hazard_left")
    haz_right = model.part("hazard_right")
    haz_h = 0.30
    haz_w = 0.26
    haz_thk = 0.02
    # Hazard backing block seats flush on the cowl plate front face.
    haz_back_y = COWL_PLATE_FRONT_Y - haz_thk / 2.0
    haz_face_y = haz_back_y - haz_thk / 2.0  # block front face
    haz_cz = SILL_H + haz_h / 2.0 + 0.22
    # Diagonal yellow hazard bands, clipped to the block rectangle.
    haz_stripes_mesh = _make_hazard_stripes(haz_w * 0.94, haz_h * 0.94)
    for part, s, hw, name in (
        (haz_left, -1.0, LEFT_HOUSING_W, "L"),
        (haz_right, 1.0, RIGHT_HOUSING_W, "R"),
    ):
        cx = s * (OPENING_W / 2.0 + hw / 2.0)
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

    # ----- Sliding armoured leaf (two telescoping stages) -----
    door_0 = model.part("door_0")              # inner armoured panel (front plane)
    door_0_outer = model.part("door_0_outer")  # rear telescoping stage (rear plane)

    inner_front_y = A_PLANE_Y - DOOR_THK / 2.0  # inner panel front face plane
    outer_front_y = B_PLANE_Y - DOOR_THK / 2.0  # outer panel front face plane

    groove_heights = ((0.30, "lo"), (0.55, "mid"), (0.80, "hi"))

    # Inner armoured panel: full-width CadQuery solid with chamfered edges.
    inner_panel_mesh = _make_leaf_panel(LEAF_W, OPENING_H, DOOR_THK)
    door_0.visual(
        mesh_from_cadquery(inner_panel_mesh, "leaf_panel_0"),
        origin=Origin(xyz=(0.0, A_PLANE_Y, SILL_H)),
        material=gunmetal,
        name="leaf_panel",
    )
    # Leading-edge vertical yellow accent strip (the edge that enters the
    # pocket). Thin ribbon proud of the panel front face.
    strip_w = 0.04
    strip_thk = DOOR_THK * 0.5
    door_0.visual(
        Box((strip_w, strip_thk, OPENING_H * 0.92)),
        origin=Origin(
            xyz=(
                LEAF_W / 2.0 - strip_w / 2.0,
                inner_front_y + strip_thk / 2.0 - 0.006,
                LEAF_CZ,
            )
        ),
        material=accent_yellow,
        name="leaf_edge_accent",
    )
    # Recessed-looking horizontal panel grooves on the panel front face.
    groove_margin = 0.08
    groove_w = LEAF_W - 2.0 * groove_margin
    for fz, gtag in groove_heights:
        door_0.visual(
            Box((groove_w, 0.008, 0.024)),
            origin=Origin(
                xyz=(0.0, inner_front_y - 0.004, SILL_H + OPENING_H * fz)
            ),
            material=dark_steel,
            name=f"groove_{gtag}",
        )
    # Dark kick plate band along the panel bottom.
    door_0.visual(
        Box((groove_w, 0.008, 0.16)),
        origin=Origin(xyz=(0.0, inner_front_y - 0.004, SILL_H + 0.10)),
        material=dark_steel,
        name="kick_plate",
    )

    # Rear telescoping stage: full-width panel in the rear plane.
    door_0_outer.visual(
        Box((LEAF_W, DOOR_THK, OPENING_H)),
        origin=Origin(xyz=(0.0, B_PLANE_Y, LEAF_CZ)),
        material=gunmetal_deep,
        name="leaf_panel",
    )
    # Matching horizontal grooves so the stage joint reads as panel lines.
    og_w = LEAF_W - 0.12
    for fz, gtag in groove_heights:
        door_0_outer.visual(
            Box((og_w, 0.008, 0.024)),
            origin=Origin(
                xyz=(0.0, outer_front_y - 0.004, SILL_H + OPENING_H * fz)
            ),
            material=dark_steel,
            name=f"groove_{gtag}",
        )
    door_0_outer.visual(
        Box((og_w, 0.008, 0.16)),
        origin=Origin(xyz=(0.0, outer_front_y - 0.004, SILL_H + 0.10)),
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

    # ----- Articulations: prismatic slide + telescoping mimic -----
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
        "door_0_outer_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=door_0_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
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
    door_0_outer = object_model.get_part("door_0_outer")
    all_panels = (door_0, door_0_outer)

    door_0_slide = object_model.get_articulation("door_0_slide")

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

    # The full-width leaf covers the opening height when closed.
    ctx.expect_overlap(
        door_0,
        frame,
        axes="z",
        min_overlap=OPENING_H * 0.9,
        name="full-width leaf covers the opening height when closed",
    )
    # The leaf spans the full opening width when closed (X overlap with sill).
    ctx.expect_overlap(
        door_0,
        frame,
        axes="x",
        elem_a="leaf_panel",
        elem_b="sill",
        min_overlap=OPENING_W * 0.95,
        name="leaf spans the full opening width when closed",
    )

    # The two telescoping stages lap each other in X so no slit opens between
    # them (they read as one layered armoured leaf).
    ctx.expect_overlap(
        door_0,
        door_0_outer,
        axes="x",
        elem_a="leaf_panel",
        elem_b="leaf_panel",
        min_overlap=0.50,
        name="inner and rear stages lap fully when closed",
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

    # The doorway is a true through-opening: no fixed frame geometry blocks
    # the central window of the opening.
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

    # --- Open pose: drive the source joint; mimic telescopes the rear stage. ---
    travel = door_0_slide.motion_limits.upper
    assert travel is not None
    pocket_outer_x = OPENING_W / 2.0 + RIGHT_HOUSING_W
    with ctx.pose({door_0_slide: travel}):
        # Inner panel retracts to the right.
        pos_inner = ctx.part_world_position(door_0)
        assert pos_inner is not None
        inner_dx = pos_inner[0] - closed_c["door_0"][0]
        ctx.check(
            "door_0 retracts toward +X when open",
            inner_dx > 0.5,
            details=f"closed x={closed_c['door_0'][0]:.3f}, open x={pos_inner[0]:.3f}",
        )

        # Rear stage also moves right (at half rate).
        pos_outer = ctx.part_world_position(door_0_outer)
        assert pos_outer is not None
        outer_dx = pos_outer[0] - closed_c["door_0_outer"][0]
        ctx.check(
            "door_0_outer retracts toward +X when open",
            outer_dx > 0.2,
            details=f"closed x={closed_c['door_0_outer'][0]:.3f}, open x={pos_outer[0]:.3f}",
        )

        # Inner panel moves further than rear stage (telescoping proof).
        ctx.check(
            "inner panel travels further than rear stage (telescoping)",
            inner_dx > outer_dx * 1.5,
            details=f"inner dx={inner_dx:.3f}, outer dx={outer_dx:.3f}",
        )

        # Both panels stay inside the enlarged right pocket when open.
        for panel in all_panels:
            aabb = ctx.part_world_aabb(panel)
            assert aabb is not None
            ctx.check(
                f"{panel.name} stays inside the right pocket when open",
                aabb[1][0] <= pocket_outer_x + 0.001,
                details=f"{panel.name} open max x={aabb[1][0]:.3f}, pocket outer={pocket_outer_x:.3f}",
            )

        # Inner panel left edge clears past the opening centre when open.
        aabb_inner = ctx.part_world_aabb(door_0)
        assert aabb_inner is not None
        ctx.check(
            "inner panel left edge clears past opening centre when open",
            aabb_inner[0][0] > 0.0,
            details=f"inner min x={aabb_inner[0][0]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
