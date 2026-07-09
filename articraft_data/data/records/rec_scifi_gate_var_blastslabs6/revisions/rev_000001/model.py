from __future__ import annotations

# Sci-fi blast / security door — vertical-parting fork.
#
# Six horizontal armoured blast slabs part vertically: the lower three drop into
# a deep sill pocket and the upper three lift into a tall lintel pocket. Each
# slab rides on its own prismatic joint along Z with a uniform joint policy.
# The rectangular frame, side housings, keypad, hazard chevrons, warning lamps,
# guide rails, and all greebling are identical to the parent bi-parting door.
#
# Articraft brief:
# - Object: armoured sci-fi blast door, ~1.62 m clear width x ~2.05 m tall,
#   standing on the ground plane (geometry from z=0 upward). Viewer looks from
#   -Y; the doorway is a true through-opening (no back wall).
# - Root/support: `frame` is the fixed structural surround (lintel header beam
#   with pocket, floor sill with pocket, top/bottom guide rails). It carries
#   the two fixed side housings, the wall keypad on the left jamb, and the
#   hazard blocks.
# - Parts: frame (root, fixed); left_housing / right_housing (FIXED); keypad,
#   hazard_left, hazard_right (FIXED); slab_0..slab_5, six horizontal armoured
#   panels with equal vertical pitch.
# - Articulations: slab_0_slide..slab_5_slide, all PRISMATIC along Z. Lower
#   slabs (0,1,2) axis=(0,0,-1) drop into sill; upper slabs (3,4,5)
#   axis=(0,0,+1) lift into lintel. Uniform travel = OPENING_H/2 so the full
#   doorway clears.
# - Visible geometry: six gunmetal slab panels with dark steel edge strips,
#   central reinforcing ribs, and yellow meeting-edge accents; olive frame with
#   deep sill/lintel pockets; lintel header trim band and warning lamps; keypad
#   with screen + keys + status lamp; yellow/black hazard chevrons; riveted
#   cowl plates.
# - Support/fit: slabs ride captured in the fixed top/bottom guide rails at
#   rest (small proven overlap); retract fully into sill/lintel pockets when
#   open.
# - Intentional overlaps: slab panels and front-face details captured in the
#   guide rail channels (allowed + proven).
# - Tests: all six slabs exist, span the opening when closed, lower slabs drop
#   and upper slabs lift when open, doorway clears, fixed furniture stays
#   mounted, door stands on the floor.
# - Assumptions: industrial gunmetal armour, yellow safety accents, no powered
#   internals modelled.
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
# Overall dimensions (metres). X = width, Y = depth (door thickness), Z = up.
# ---------------------------------------------------------------------------
OPENING_W = 1.62  # clear doorway width (jamb-inner to jamb-inner)
OPENING_H = 2.05  # clear doorway height
DOOR_THK = 0.06  # armoured panel thickness (Y extent)

FRAME_DEPTH = 0.34  # full wall surround depth (along Y)
HEADER_H = 0.22  # visible lintel header height (above opening)
SILL_H = 0.05  # visible floor sill lip height (above floor)

HOUSING_W = 0.51  # side housing / pocket width (X)
REAR_RAIL_THK = 0.05  # rear rail thickness (Y)

# ---------------------------------------------------------------------------
# Slab configuration.
# ---------------------------------------------------------------------------
N_SLABS = 6
SLAB_GAP = 0.002  # running clearance between adjacent slabs
SLAB_H = (OPENING_H - (N_SLABS - 1) * SLAB_GAP) / N_SLABS
GUIDE_EMBED = 0.012  # how far slab edges embed into the jamb guide channels
SLAB_W = OPENING_W + 2.0 * GUIDE_EMBED  # wider than opening for guide capture
SLAB_THK = DOOR_THK

# Vertical guide channel dimensions (on the jambs, capturing slab edges).
CHANNEL_THK_X = 0.030  # channel wall thickness along X
CHANNEL_DEPTH_Y = 0.10  # channel depth along Y (wider than slab thickness)
CHANNEL_CZ = SILL_H + OPENING_H / 2.0  # centred on the opening

# Uniform travel: all slabs move by OPENING_H/2 so lower slabs fully retract
# below the sill line and upper slabs fully retract above the lintel line.
SLAB_TRAVEL = OPENING_H / 2.0

# Pocket dimensions (extra margin beyond the travel envelope).
POCKET_MARGIN = 0.025
SILL_POCKET_DEPTH = SLAB_TRAVEL + POCKET_MARGIN  # below the visible sill lip
LINTEL_POCKET_DEPTH = SLAB_TRAVEL + POCKET_MARGIN  # above the visible header

# ---------------------------------------------------------------------------
# Housing channel geometry (identical to parent).
# ---------------------------------------------------------------------------
COWL_FRONT_Y = -0.115
COWL_THK = 0.05
COWL_PLATE_THK = 0.025
COWL_PLATE_Y = COWL_FRONT_Y - COWL_THK / 2.0 - COWL_PLATE_THK / 2.0
COWL_PLATE_FRONT_Y = COWL_PLATE_Y - COWL_PLATE_THK / 2.0
REAR_RAIL_Y = 0.11

# ---------------------------------------------------------------------------
# Guide rails (identical to parent).
# ---------------------------------------------------------------------------
GUIDE_HALF_LEN = OPENING_W / 2.0 + HOUSING_W - 0.03
GUIDE_DEPTH = 0.16
GUIDE_CY = -0.005
GUIDE_CAPTURE = 0.012
LEAF_BOT_Z = SILL_H
LEAF_TOP_Z = SILL_H + OPENING_H
TOP_GUIDE_H = 0.06
BOT_GUIDE_H = 0.036
TOP_GUIDE_CZ = LEAF_TOP_Z - GUIDE_CAPTURE + TOP_GUIDE_H / 2.0
BOT_GUIDE_CZ = LEAF_BOT_Z + GUIDE_CAPTURE - BOT_GUIDE_H / 2.0

# Vertical guide channel height: spans from the bottom guide centre through
# the top guide centre so every slab edge is captured for its whole travel.
CHANNEL_H = OPENING_H + TOP_GUIDE_H + BOT_GUIDE_H


# ---------------------------------------------------------------------------
# Slab rest position.
# ---------------------------------------------------------------------------
def slab_center_z(i: int) -> float:
    """World-Z centre of slab *i* at rest (q=0)."""
    return SILL_H + i * (SLAB_H + SLAB_GAP) + SLAB_H / 2.0


# ---------------------------------------------------------------------------
# Hazard chevron stripes (CadQuery) — identical to parent.
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
    return slashes.intersect(clip)


# ---------------------------------------------------------------------------
# Shared slab visual helper.
# ---------------------------------------------------------------------------
def _add_slab_visuals(
    part,
    *,
    gunmetal,
    dark_steel,
    accent_yellow,
    slab_index: int,
) -> None:
    """Add the armoured-panel visuals for one blast slab.

    All geometry is in the part-local frame (centred on the slab rest
    position).
    """
    front_y = -SLAB_THK / 2.0  # slab front face

    # Main armoured panel.
    part.visual(
        Box((SLAB_W, SLAB_THK, SLAB_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gunmetal,
        name="slab_panel",
    )

    # Dark steel edge strip along the top edge (front face, slightly proud).
    strip_h = 0.015
    strip_thk = 0.008
    part.visual(
        Box((SLAB_W * 0.96, strip_thk, strip_h)),
        origin=Origin(
            xyz=(0.0, front_y - strip_thk / 2.0, SLAB_H / 2.0 - strip_h / 2.0)
        ),
        material=dark_steel,
        name="edge_strip_top",
    )

    # Dark steel edge strip along the bottom edge.
    part.visual(
        Box((SLAB_W * 0.96, strip_thk, strip_h)),
        origin=Origin(
            xyz=(0.0, front_y - strip_thk / 2.0, -SLAB_H / 2.0 + strip_h / 2.0)
        ),
        material=dark_steel,
        name="edge_strip_bottom",
    )

    # Central reinforcing rib (wider raised strip across the panel centre).
    rib_h = SLAB_H * 0.18
    rib_thk = 0.012
    part.visual(
        Box((SLAB_W * 0.88, rib_thk, rib_h)),
        origin=Origin(xyz=(0.0, front_y - rib_thk / 2.0, 0.0)),
        material=dark_steel,
        name="center_rib",
    )

    # Yellow accent strip at the upper meeting edge (where this slab meets the
    # one above). Omitted on the topmost slab (nothing above to mate with).
    if slab_index < N_SLABS - 1:
        accent_h = 0.008
        accent_thk = 0.006
        part.visual(
            Box((SLAB_W * 0.92, accent_thk, accent_h)),
            origin=Origin(
                xyz=(
                    0.0,
                    front_y - accent_thk / 2.0,
                    SLAB_H / 2.0 - accent_h / 2.0,
                )
            ),
            material=accent_yellow,
            name="meeting_accent",
        )


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

    # ----- Root: fixed structural frame (lintel + sill + guide rails) -----
    frame = model.part("frame")

    # Lintel header beam — tall enough to pocket the retracted upper slabs.
    lintel_total_h = HEADER_H + LINTEL_POCKET_DEPTH
    lintel_cz = LEAF_TOP_Z + lintel_total_h / 2.0
    frame.visual(
        Box((header_w, FRAME_DEPTH, lintel_total_h)),
        origin=Origin(xyz=(0.0, 0.0, lintel_cz)),
        material=olive_frame,
        name="lintel_header",
    )
    # Worn metal trim band on the header front face (identical placement).
    frame.visual(
        Box((header_w * 0.96, 0.03, HEADER_H * 0.5)),
        origin=Origin(
            xyz=(
                0.0,
                -FRAME_DEPTH / 2.0 - 0.015,
                SILL_H + OPENING_H + HEADER_H * 0.55,
            )
        ),
        material=dark_steel,
        name="header_trim",
    )
    # Floor sill — deep enough to pocket the retracted lower slabs.
    sill_total_h = SILL_H + SILL_POCKET_DEPTH
    sill_cz = (SILL_H - SILL_POCKET_DEPTH) / 2.0
    frame.visual(
        Box((header_w, FRAME_DEPTH, sill_total_h)),
        origin=Origin(xyz=(0.0, 0.0, sill_cz)),
        material=olive_frame,
        name="sill",
    )
    # Top guide rail (identical to parent).
    frame.visual(
        Box((2.0 * GUIDE_HALF_LEN, GUIDE_DEPTH, TOP_GUIDE_H)),
        origin=Origin(xyz=(0.0, GUIDE_CY, TOP_GUIDE_CZ)),
        material=dark_steel,
        name="top_guide",
    )
    # Bottom guide rail (identical to parent).
    frame.visual(
        Box((2.0 * GUIDE_HALF_LEN, GUIDE_DEPTH, BOT_GUIDE_H)),
        origin=Origin(xyz=(0.0, GUIDE_CY, BOT_GUIDE_CZ)),
        material=dark_steel,
        name="bottom_guide",
    )
    # Vertical guide channels on the left and right jambs: capture every slab
    # edge for its whole travel (connects the sill and lintel groups and
    # provides the physical support path for the vertically-moving slabs).
    for s, ch_name in ((-1.0, "left_guide"), (1.0, "right_guide")):
        ch_cx = s * (OPENING_W / 2.0 + CHANNEL_THK_X / 2.0)
        frame.visual(
            Box((CHANNEL_THK_X, CHANNEL_DEPTH_Y, CHANNEL_H)),
            origin=Origin(xyz=(ch_cx, 0.0, CHANNEL_CZ)),
            material=dark_steel,
            name=ch_name,
        )

    # Twin amber warning lamps (identical to parent).
    trim_front_y = -FRAME_DEPTH / 2.0 - 0.03
    trim_cz = SILL_H + OPENING_H + HEADER_H * 0.55
    for s, lamp_name in ((-1.0, "warning_lamp_l"), (1.0, "warning_lamp_r")):
        frame.visual(
            Box((0.10, 0.012, 0.05)),
            origin=Origin(xyz=(s * 0.55, trim_front_y - 0.006, trim_cz)),
            material=accent_yellow,
            name=lamp_name,
        )

    # ----- Fixed side housings: jamb pods (identical to parent) -----
    left_housing = model.part("left_housing")
    right_housing = model.part("right_housing")
    housing_cz = SILL_H + OPENING_H / 2.0
    housing_h = OPENING_H
    cowl_h = OPENING_H - 0.02
    pocket_inner_x = OPENING_W / 2.0
    pocket_outer_x = pocket_inner_x + HOUSING_W
    for hpart, s in ((left_housing, -1.0), (right_housing, 1.0)):
        cx = s * (pocket_inner_x + HOUSING_W / 2.0)
        hpart.visual(
            Box((HOUSING_W, COWL_THK, cowl_h)),
            origin=Origin(xyz=(cx, COWL_FRONT_Y, housing_cz)),
            material=olive_frame,
            name="cowl_front",
        )
        hpart.visual(
            Box((HOUSING_W * 0.86, COWL_PLATE_THK, cowl_h * 0.8)),
            origin=Origin(xyz=(cx, COWL_PLATE_Y, housing_cz)),
            material=dark_steel,
            name="cowl_plate",
        )
        hpart.visual(
            Box((HOUSING_W, REAR_RAIL_THK, housing_h)),
            origin=Origin(xyz=(cx, REAR_RAIL_Y, housing_cz)),
            material=olive_frame,
            name="rear_rail",
        )
        end_x = s * (pocket_outer_x - 0.015)
        hpart.visual(
            Box(
                (0.03, REAR_RAIL_Y - COWL_FRONT_Y + REAR_RAIL_THK, housing_h)
            ),
            origin=Origin(
                xyz=(
                    end_x,
                    (COWL_FRONT_Y + REAR_RAIL_Y) / 2.0,
                    housing_cz,
                )
            ),
            material=olive_frame,
            name="pocket_end_wall",
        )
        for rz, row in (
            (housing_cz - 0.56, "lo"),
            (housing_cz + 0.56, "hi"),
        ):
            for dx, col in ((-0.10, "in"), (0.10, "out")):
                hpart.visual(
                    Box((0.022, 0.012, 0.022)),
                    origin=Origin(
                        xyz=(cx + dx, COWL_PLATE_FRONT_Y - 0.006, rz)
                    ),
                    material=dark_steel,
                    name=f"rivet_{row}_{col}",
                )

    # ----- Fixed wall keypad on the left cowl front face (identical) -----
    keypad = model.part("keypad")
    kp_cx = -(pocket_inner_x + HOUSING_W / 2.0)
    kp_box_thk = 0.05
    kp_box_y = COWL_PLATE_FRONT_Y - kp_box_thk / 2.0
    kp_z = SILL_H + OPENING_H * 0.6
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
    # Status lamp sits proud on the keypad front face, embedded into the box.
    keypad.visual(
        Box((0.05, 0.014, 0.025)),
        origin=Origin(
            xyz=(kp_cx, kp_face_y + 0.004, kp_z + 0.07)
        ),
        material=status_green,
        name="status_lamp",
    )

    # ----- Fixed hazard chevrons at the base corners (identical) -----
    haz_left = model.part("hazard_left")
    haz_right = model.part("hazard_right")
    haz_h = 0.30
    haz_w = 0.26
    haz_thk = 0.02
    haz_back_y = COWL_PLATE_FRONT_Y - haz_thk / 2.0
    haz_face_y = haz_back_y - haz_thk / 2.0
    haz_cz = SILL_H + haz_h / 2.0 + 0.22
    haz_stripes_mesh = _make_hazard_stripes(haz_w * 0.94, haz_h * 0.94)
    for hpart, s, stag in (
        (haz_left, -1.0, "L"),
        (haz_right, 1.0, "R"),
    ):
        cx = s * (pocket_inner_x + HOUSING_W / 2.0)
        hpart.visual(
            Box((haz_w, haz_thk, haz_h)),
            origin=Origin(xyz=(cx, haz_back_y, haz_cz)),
            material=hazard_black,
            name="hazard_back",
        )
        yaw = 0.0 if s < 0 else math.pi
        hpart.visual(
            mesh_from_cadquery(haz_stripes_mesh, f"hazard_stripes_{stag}"),
            origin=Origin(xyz=(cx, haz_face_y, haz_cz), rpy=(0.0, 0.0, yaw)),
            material=accent_yellow,
            name="hazard_stripes",
        )

    # ----- Vertical-parting blast slabs (6 panels) -----
    for i in range(N_SLABS):
        slab = model.part(f"slab_{i}")
        cz = slab_center_z(i)

        _add_slab_visuals(
            slab,
            gunmetal=gunmetal,
            dark_steel=dark_steel,
            accent_yellow=accent_yellow,
            slab_index=i,
        )

        # Lower half drops into sill (-Z), upper half lifts into lintel (+Z).
        if i < N_SLABS // 2:
            axis = (0.0, 0.0, -1.0)
        else:
            axis = (0.0, 0.0, 1.0)

        model.articulation(
            f"slab_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=slab,
            origin=Origin(xyz=(0.0, 0.0, cz)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=400.0,
                velocity=0.5,
                lower=0.0,
                upper=SLAB_TRAVEL,
            ),
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_housing = object_model.get_part("left_housing")
    right_housing = object_model.get_part("right_housing")
    keypad = object_model.get_part("keypad")
    haz_left = object_model.get_part("hazard_left")
    haz_right = object_model.get_part("hazard_right")

    slabs = [object_model.get_part(f"slab_{i}") for i in range(N_SLABS)]
    slab_joints = [
        object_model.get_articulation(f"slab_{i}_slide") for i in range(N_SLABS)
    ]

    # --- Overlap allowances: slab edges captured in jamb guide channels. ---
    # Every slab edge embeds GUIDE_EMBED into the left/right jamb guide
    # channels (small intentional overlap models the captured channel fit).
    for slab in slabs:
        for ch in ("left_guide", "right_guide"):
            ctx.allow_overlap(
                slab,
                frame,
                elem_a="slab_panel",
                elem_b=ch,
                reason=(
                    f"The {slab.name} panel edges ride captured inside the "
                    f"{ch} jamb channel for the slab's whole travel."
                ),
            )
    # At rest, slab_0 bottom edge strip overlaps the bottom guide and slab_5
    # top edge strip overlaps the top guide (captured in the rail channels).
    ctx.allow_overlap(
        slabs[0],
        frame,
        elem_a="edge_strip_bottom",
        elem_b="bottom_guide",
        reason=(
            "The slab_0 bottom edge strip sits captured inside the "
            "bottom_guide channel at rest."
        ),
    )
    ctx.allow_overlap(
        slabs[N_SLABS - 1],
        frame,
        elem_a="edge_strip_top",
        elem_b="top_guide",
        reason=(
            f"The slab_{N_SLABS - 1} top edge strip sits captured inside the "
            "top_guide channel at rest."
        ),
    )
    # Slab panels may also pass through the guide capture zones during travel.
    for slab in slabs:
        for rail in ("top_guide", "bottom_guide"):
            ctx.allow_overlap(
                slab,
                frame,
                elem_a="slab_panel",
                elem_b=rail,
                reason=(
                    f"The {slab.name} panel may pass through the {rail} "
                    f"capture zone at rest or during travel."
                ),
            )

    # --- Overlap allowances: hazard backing blocks on housing rivets. ---
    # The hazard backing block seats flush on the cowl plate and slightly
    # overlaps the nearby rivet studs (a small local embed at the mounting
    # surface). Target the specific rivet elements that overlap.
    ctx.allow_overlap(
        haz_left,
        left_housing,
        elem_a="hazard_back",
        elem_b="rivet_lo_in",
        reason=(
            "The hazard backing block seats flush on the left cowl plate "
            "and slightly overlaps the rivet_lo_in stud at the mounting face."
        ),
    )
    ctx.allow_overlap(
        haz_left,
        left_housing,
        elem_a="hazard_back",
        elem_b="rivet_lo_out",
        reason=(
            "The hazard backing block seats flush on the left cowl plate "
            "and slightly overlaps the rivet_lo_out stud at the mounting face."
        ),
    )
    ctx.allow_overlap(
        haz_right,
        right_housing,
        elem_a="hazard_back",
        elem_b="rivet_lo_in",
        reason=(
            "The hazard backing block seats flush on the right cowl plate "
            "and slightly overlaps the rivet_lo_in stud at the mounting face."
        ),
    )
    ctx.allow_overlap(
        haz_right,
        right_housing,
        elem_a="hazard_back",
        elem_b="rivet_lo_out",
        reason=(
            "The hazard backing block seats flush on the right cowl plate "
            "and slightly overlaps the rivet_lo_out stud at the mounting face."
        ),
    )

    # --- Closed-pose checks: slabs span the full opening. ---
    # Each slab spans the doorway width (X overlap with the guide channels).
    for i, slab in enumerate(slabs):
        ctx.expect_overlap(
            slab,
            frame,
            axes="x",
            elem_a="slab_panel",
            elem_b="left_guide",
            min_overlap=GUIDE_EMBED * 0.8,
            name=f"slab_{i} left edge captured in left guide channel",
        )
        ctx.expect_overlap(
            slab,
            frame,
            axes="x",
            elem_a="slab_panel",
            elem_b="right_guide",
            min_overlap=GUIDE_EMBED * 0.8,
            name=f"slab_{i} right edge captured in right guide channel",
        )

    # Adjacent slabs meet at their seam (gap <= SLAB_GAP + tolerance).
    for i in range(N_SLABS - 1):
        ctx.expect_gap(
            slabs[i + 1],
            slabs[i],
            axis="z",
            max_gap=SLAB_GAP + 0.001,
            max_penetration=0.001,
            positive_elem="slab_panel",
            negative_elem="slab_panel",
            name=f"slab_{i} and slab_{i + 1} meet at their horizontal seam",
        )

    # Each slab covers its expected vertical band: prove Z overlap with the
    # jamb guide channels (which span the full opening height).
    for i, slab in enumerate(slabs):
        ctx.expect_overlap(
            slab,
            frame,
            axes="z",
            elem_a="slab_panel",
            elem_b="left_guide",
            min_overlap=SLAB_H * 0.9,
            name=f"slab_{i} covers its vertical band of the opening",
        )

    # Slabs stand within the frame at rest (not below the sill pocket or
    # above the lintel pocket).
    sill_bottom_z = (SILL_H - SILL_POCKET_DEPTH) / 2.0 - (
        SILL_H + SILL_POCKET_DEPTH
    ) / 2.0
    lintel_top_z = LEAF_TOP_Z + HEADER_H + LINTEL_POCKET_DEPTH
    for slab in slabs:
        aabb = ctx.part_world_aabb(slab)
        assert aabb is not None
        ctx.check(
            f"{slab.name} stays within the frame envelope at rest",
            aabb[0][2] >= sill_bottom_z - 0.01
            and aabb[1][2] <= lintel_top_z + 0.01,
            details=f"{slab.name} z=({aabb[0][2]:.4f}, {aabb[1][2]:.4f})",
        )

    # --- Open-pose checks: all slabs retract, doorway clears. ---
    travel = SLAB_TRAVEL
    closed_centers = [slab_center_z(i) for i in range(N_SLABS)]

    with ctx.pose({joint: travel for joint in slab_joints}):
        # Lower slabs (0,1,2) drop into the sill.
        for i in range(N_SLABS // 2):
            pos = ctx.part_world_position(slabs[i])
            assert pos is not None
            ctx.check(
                f"slab_{i} drops into sill when open",
                pos[2] < closed_centers[i] - 0.30,
                details=(
                    f"closed z={closed_centers[i]:.3f}, "
                    f"open z={pos[2]:.3f}"
                ),
            )

        # Upper slabs (3,4,5) lift into the lintel.
        for i in range(N_SLABS // 2, N_SLABS):
            pos = ctx.part_world_position(slabs[i])
            assert pos is not None
            ctx.check(
                f"slab_{i} lifts into lintel when open",
                pos[2] > closed_centers[i] + 0.30,
                details=(
                    f"closed z={closed_centers[i]:.3f}, "
                    f"open z={pos[2]:.3f}"
                ),
            )

        # The gap between the lowest upper slab (slab_3) and the highest
        # lower slab (slab_2) clears the full opening height.
        ctx.expect_gap(
            slabs[N_SLABS // 2],  # slab_3 (lowest upper)
            slabs[N_SLABS // 2 - 1],  # slab_2 (highest lower)
            axis="z",
            min_gap=OPENING_H * 0.80,
            positive_elem="slab_panel",
            negative_elem="slab_panel",
            name="slabs part vertically to clear the doorway",
        )

        # All retracted slabs stay within the frame envelope (not poking out).
        for slab in slabs:
            aabb = ctx.part_world_aabb(slab)
            assert aabb is not None
            ctx.check(
                f"{slab.name} stays within the frame pocket when open",
                aabb[0][2] >= sill_bottom_z - 0.01
                and aabb[1][2] <= lintel_top_z + 0.01,
                details=(
                    f"{slab.name} open z=({aabb[0][2]:.3f}, {aabb[1][2]:.3f})"
                ),
            )

    # --- Fixed furniture is mounted, not floating. ---
    ctx.expect_contact(keypad, left_housing, name="keypad mounted on left cowl")
    ctx.expect_contact(
        left_housing, frame, name="left housing mounted to frame"
    )
    ctx.expect_contact(
        right_housing, frame, name="right housing mounted to frame"
    )
    ctx.expect_contact(haz_left, left_housing, name="left hazard on left cowl")
    ctx.expect_contact(
        haz_right, right_housing, name="right hazard on right cowl"
    )

    # --- Doorway is a true through-opening (no back wall). ---
    window_x = 0.5
    window_z = (0.4, 1.6)
    for elem in (
        "lintel_header",
        "header_trim",
        "sill",
        "top_guide",
        "bottom_guide",
    ):
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

    return ctx.report()


object_model = build_object_model()
