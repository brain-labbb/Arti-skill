from __future__ import annotations

# Sci-fi blast / security door — overhead-retracting variant.
#
# Four dark gunmetal horizontal blast slabs span the doorway at equal vertical
# pitch and retract straight up into the lintel header to open.  Each slab rides
# on its own independent prismatic joint along +Z with a uniform effort/velocity
# policy and staggered travel so the four bars nest tightly inside the header
# cavity when fully retracted, leaving the full opening clear.
#
# Articraft brief:
# - Object: armoured sci-fi blast door, ~1.62 m clear width x ~2.05 m tall,
#   standing on the ground plane (geometry from z=0 upward).  Viewer looks from
#   -Y; the doorway is a true through-opening (no back wall).
# - Root/support: `frame` is the fixed structural surround (lintel header beam,
#   floor sill, top/bottom structural cross-members).  It carries the two fixed
#   side housings, the wall keypad on the left jamb, and the hazard blocks.
# - Parts: frame (root, fixed); left_housing / right_housing (FIXED) are hollow
#   side pockets (front cowl + rear rail) that double as the jambs and as the
#   vertical guide channels the slab ends ride in; keypad, hazard_left,
#   hazard_right (FIXED); slab_0 .. slab_3, the four horizontal armoured bars.
# - Articulations: slab_0_slide .. slab_3_slide (each PRISMATIC, +Z), one
#   independent joint per slab.  Uniform effort and velocity; only the upper
#   travel limit differs so the bottom slab travels farthest and the top slab
#   barely moves, giving a tight nested stack inside the header.
# - Visible geometry: four gunmetal horizontal bars evenly spaced across the
#   opening, each with a yellow safety accent stripe on the front face; lintel
#   header with trim band and warning lamps, keypad with screen + keys + status
#   lamp, yellow/black hazard chevrons, riveted cowl plates.
# - Support/fit: slab ends seat flush against the housing inner faces (the
#   housing cowl pods form the vertical guide channels).
# - Intentional overlaps: at the fully-open pose every slab retracts inside the
#   lintel header cavity (allowed + proven by expect_within/expect_overlap);
#   slab_0 also passes through the structural top cross-member zone (allowed +
#   proven).
# - Tests: four slabs exist, closed pose shows even vertical spacing spanning
#   the opening, open pose nests every slab inside the header with the full
#   doorway clear below, each slab actually moves upward, staggered travel
#   ordering is correct, and fixed furniture stays mounted.
# - Assumptions: industrial gunmetal armour, yellow safety accents, four
#   horizontal bars, no powered internals modelled.
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
# Overall dimensions (metres).  X = width, Y = depth (door thickness), Z = up.
# ---------------------------------------------------------------------------
OPENING_W = 1.62   # clear doorway width (jamb-inner to jamb-inner)
OPENING_H = 2.05   # clear doorway height
DOOR_THK = 0.06    # armoured slab thickness (Y extent)

FRAME_DEPTH = 0.34 # full wall surround depth (along Y)
HEADER_H = 0.22    # lintel header height (also the slab pocket depth)
SILL_H = 0.05      # floor sill height

HOUSING_W = 0.51   # side housing / pocket width (X)
REAR_RAIL_THK = 0.05

HALF_W = OPENING_W / 2.0

# ---------------------------------------------------------------------------
# Slab geometry.  Four horizontal blast bars at equal vertical pitch across the
# opening, retracting upward into the lintel header pocket.
# ---------------------------------------------------------------------------
N_SLABS = 4
SLAB_W = OPENING_W           # slabs span the full clear opening
SLAB_H = 0.050               # bar height (substantial armoured louver)
SLAB_THK = DOOR_THK          # bar depth
SLAB_PITCH = OPENING_H / N_SLABS  # equal vertical spacing

# Nesting inside the header when fully open.  The top slab parks just under the
# header cap; each lower slab stacks beneath it with a small air gap.
NEST_GAP = 0.002
NEST_STEP = SLAB_H + NEST_GAP
NEST_TOP_Z = SILL_H + OPENING_H + HEADER_H - 0.010  # just under the header cap

# Housing channel geometry (unchanged from parent).
COWL_FRONT_Y = -0.115
COWL_THK = 0.05
COWL_PLATE_THK = 0.025
COWL_PLATE_Y = COWL_FRONT_Y - COWL_THK / 2.0 - COWL_PLATE_THK / 2.0
COWL_PLATE_FRONT_Y = COWL_PLATE_Y - COWL_PLATE_THK / 2.0
REAR_RAIL_Y = 0.11

# Structural top/bottom cross-members (the original guide rails, retained as
# structural members of the frame).
GUIDE_HALF_LEN = HALF_W + HOUSING_W - 0.03
GUIDE_DEPTH = 0.16
A_PLANE_Y = -0.045
B_PLANE_Y = 0.035
GUIDE_CY = (A_PLANE_Y + B_PLANE_Y) / 2.0
GUIDE_CAPTURE = 0.012
LEAF_BOT_Z = SILL_H
LEAF_TOP_Z = SILL_H + OPENING_H
TOP_GUIDE_H = 0.06
TOP_GUIDE_CZ = LEAF_TOP_Z - GUIDE_CAPTURE + TOP_GUIDE_H / 2.0
BOT_GUIDE_H = 0.036
BOT_GUIDE_CZ = LEAF_BOT_Z + GUIDE_CAPTURE - BOT_GUIDE_H / 2.0


# ---------------------------------------------------------------------------
# Slab position helpers.  Rest = closed (evenly spaced across the opening);
# open = nested inside the header pocket.
# ---------------------------------------------------------------------------
def _slab_rest_cz(i: int) -> float:
    """Centre-z of slab *i* in the closed pose."""
    return SILL_H + (i + 0.5) * SLAB_PITCH


def _slab_open_cz(i: int) -> float:
    """Centre-z of slab *i* when fully retracted into the header."""
    # slab N-1 (topmost) parks highest; each lower slab stacks beneath it.
    return NEST_TOP_Z - SLAB_H / 2.0 - (N_SLABS - 1 - i) * NEST_STEP


def _slab_travel(i: int) -> float:
    """Prismatic travel (metres along +Z) for slab *i*."""
    return _slab_open_cz(i) - _slab_rest_cz(i)


# Shared slab geometry helper: one Box used by every slab_i in the for-loop.
_SLAB_BODY = Box((SLAB_W, SLAB_THK, SLAB_H))


# ---------------------------------------------------------------------------
# Hazard chevron stripes (CadQuery).  Unchanged from the parent.
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
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scifi_blast_door")

    gunmetal = model.material("gunmetal", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    olive_frame = model.material("olive_frame", rgba=(0.27, 0.30, 0.26, 1.0))
    accent_yellow = model.material("accent_yellow", rgba=(0.92, 0.74, 0.06, 1.0))
    hazard_black = model.material("hazard_black", rgba=(0.07, 0.07, 0.07, 1.0))
    keypad_body = model.material("keypad_body", rgba=(0.16, 0.17, 0.19, 1.0))
    keypad_screen = model.material("keypad_screen", rgba=(0.12, 0.42, 0.62, 1.0))
    keypad_key = model.material("keypad_key", rgba=(0.30, 0.32, 0.34, 1.0))
    status_green = model.material("status_green", rgba=(0.18, 0.78, 0.34, 1.0))

    header_w = OPENING_W + 2.0 * HOUSING_W

    # ----- Root: fixed structural frame (header + sill + cross-members) -----
    frame = model.part("frame")

    # Lintel header beam across the top (acts as the slab pocket enclosure).
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
    # Top structural cross-member (retained from the parent guide rail).
    frame.visual(
        Box((2.0 * GUIDE_HALF_LEN, GUIDE_DEPTH, TOP_GUIDE_H)),
        origin=Origin(xyz=(0.0, GUIDE_CY, TOP_GUIDE_CZ)),
        material=dark_steel,
        name="top_guide",
    )
    # Bottom structural cross-member.
    frame.visual(
        Box((2.0 * GUIDE_HALF_LEN, GUIDE_DEPTH, BOT_GUIDE_H)),
        origin=Origin(xyz=(0.0, GUIDE_CY, BOT_GUIDE_CZ)),
        material=dark_steel,
        name="bottom_guide",
    )
    # Twin amber warning lamps seated flush on the header trim band.
    trim_front_y = -FRAME_DEPTH / 2.0 - 0.03
    trim_cz = SILL_H + OPENING_H + HEADER_H * 0.55
    for s, lamp_name in ((-1.0, "warning_lamp_l"), (1.0, "warning_lamp_r")):
        frame.visual(
            Box((0.10, 0.012, 0.05)),
            origin=Origin(xyz=(s * 0.55, trim_front_y - 0.006, trim_cz)),
            material=accent_yellow,
            name=lamp_name,
        )

    # Vertical guide channels at each jamb: dark steel tracks running from the
    # sill top to the header underside.  The slab ends ride captured inside
    # these channels for the full travel, providing both lateral guidance and
    # the structural support path from every slab back to the fixed frame.
    guide_ch_w = 0.06   # channel width along X
    guide_ch_d = 0.08   # channel depth along Y (envelopes the slab thickness)
    guide_ch_h = OPENING_H
    guide_ch_cz = SILL_H + OPENING_H / 2.0
    for s, ch_name in ((-1.0, "left_guide"), (1.0, "right_guide")):
        frame.visual(
            Box((guide_ch_w, guide_ch_d, guide_ch_h)),
            origin=Origin(xyz=(s * HALF_W, 0.0, guide_ch_cz)),
            material=dark_steel,
            name=ch_name,
        )

    # ----- Fixed side housings: hollow jamb pockets (front cowl + rear rail) --
    left_housing = model.part("left_housing")
    right_housing = model.part("right_housing")
    housing_cz = SILL_H + OPENING_H / 2.0
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
    keypad.visual(
        Box((0.05, 0.014, 0.025)),
        origin=Origin(xyz=(kp_cx, COWL_PLATE_FRONT_Y - 0.007, kp_z + 0.13)),
        material=status_green,
        name="status_lamp",
    )

    # ----- Fixed hazard chevrons at the base corners -----
    haz_left = model.part("hazard_left")
    haz_right = model.part("hazard_right")
    haz_h = 0.30
    haz_w = 0.26
    haz_thk = 0.02
    haz_back_y = COWL_PLATE_FRONT_Y - haz_thk / 2.0
    haz_face_y = haz_back_y - haz_thk / 2.0
    haz_cz = SILL_H + haz_h / 2.0 + 0.22
    haz_stripes_mesh = _make_hazard_stripes(haz_w * 0.94, haz_h * 0.94)
    for part, s, name in ((haz_left, -1.0, "L"), (haz_right, 1.0, "R")):
        cx = s * (pocket_inner_x + HOUSING_W / 2.0)
        part.visual(
            Box((haz_w, haz_thk, haz_h)),
            origin=Origin(xyz=(cx, haz_back_y, haz_cz)),
            material=hazard_black,
            name="hazard_back",
        )
        yaw = 0.0 if s < 0 else math.pi
        part.visual(
            mesh_from_cadquery(haz_stripes_mesh, f"hazard_stripes_{name}"),
            origin=Origin(xyz=(cx, haz_face_y, haz_cz), rpy=(0.0, 0.0, yaw)),
            material=accent_yellow,
            name="hazard_stripes",
        )

    # ----- Horizontal blast slabs (four bars, one prismatic joint each) -----
    # Each slab part frame sits at the slab centre in the closed pose.  The
    # shared _SLAB_BODY box is centred on the part origin, so the visual centre
    # coincides with the joint origin at q=0.
    #
    # Accent-line geometry (shared across all slabs): a thin yellow stripe on
    # the front face near the bottom edge of each bar.
    accent_w = SLAB_W * 0.88
    accent_thk = 0.006
    accent_h = 0.010
    accent_y = -SLAB_THK / 2.0 + 0.001  # embedded 1 mm so it bonds to the body
    accent_z = -SLAB_H / 2.0 + accent_h / 2.0 + 0.002  # near the bottom edge

    slabs = []
    for i in range(N_SLABS):
        slab = model.part(f"slab_{i}")
        slabs.append(slab)
        # Main armoured bar (shared box helper).
        slab.visual(
            _SLAB_BODY,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=gunmetal,
            name="slab_body",
        )
        # Yellow safety accent stripe on the front face, near the bar bottom.
        slab.visual(
            Box((accent_w, accent_thk, accent_h)),
            origin=Origin(xyz=(0.0, accent_y, accent_z)),
            material=accent_yellow,
            name="accent_line",
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

    # ----- Slab articulations: independent prismatic joints along +Z ----------
    # Uniform joint policy: same effort and velocity; only the upper travel
    # limit varies (staggered so the bottom slab travels farthest).
    joint_effort = 400.0
    joint_velocity = 0.4
    for i in range(N_SLABS):
        model.articulation(
            f"slab_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=slabs[i],
            origin=Origin(xyz=(0.0, 0.0, _slab_rest_cz(i))),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=joint_effort,
                velocity=joint_velocity,
                lower=0.0,
                upper=_slab_travel(i),
            ),
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

    # --- Intentional overlaps at rest: slab ends captured in guide channels. ---
    for i, slab in enumerate(slabs):
        for ch in ("left_guide", "right_guide"):
            ctx.allow_overlap(
                slab,
                frame,
                elem_a="slab_body",
                elem_b=ch,
                reason=(
                    f"slab_{i} rides captured inside the {ch} channel for "
                    "its full vertical travel."
                ),
            )

    # Hazard backing blocks seat flush on the cowl plates and share the
    # mounting surface with the cowl-plate rivets (small local overlap).
    # The hazard backing blocks seat flush on the cowl plates and share the
    # mounting surface with both lower cowl-plate rivets on each housing.
    for haz_part, housing_part in (
        (haz_left, left_housing),
        (haz_right, right_housing),
    ):
        for rivet_name in ("rivet_lo_in", "rivet_lo_out"):
            ctx.allow_overlap(
                haz_part,
                housing_part,
                elem_a="hazard_back",
                elem_b=rivet_name,
                reason=(
                    "The hazard backing block and the cowl-plate rivets share "
                    "the same cowl-plate mounting surface; the small local "
                    "overlap is the rivet head seated into the block back face."
                ),
            )

    # --- Intentional overlaps at the fully-open pose. ---
    # Every slab retracts inside the lintel header pocket; slab_0 also passes
    # through the structural top cross-member zone.
    for i, slab in enumerate(slabs):
        for elem in ("slab_body", "accent_line"):
            ctx.allow_overlap(
                slab,
                frame,
                elem_a=elem,
                elem_b="lintel_header",
                reason=(
                    f"slab_{i} retracts into the lintel header cavity when "
                    "fully open; the header pocket encloses the nested stack."
                ),
            )
    # slab_0 (the bottom slab) passes through the top cross-member zone on its
    # way into the header pocket.
    for elem in ("slab_body", "accent_line"):
        ctx.allow_overlap(
            slabs[0],
            frame,
            elem_a=elem,
            elem_b="top_guide",
            reason=(
                "slab_0 passes through the structural top cross-member zone "
                "when fully retracted into the header pocket."
            ),
        )

    # --- Closed-pose checks: four evenly-spaced horizontal bars. ---
    # Each slab spans the full opening width.
    for i, slab in enumerate(slabs):
        ctx.expect_overlap(
            slab,
            frame,
            axes="x",
            elem_a="slab_body",
            elem_b="sill",
            min_overlap=OPENING_W * 0.95,
            name=f"slab_{i} spans the opening width",
        )

    # Slabs are evenly spaced vertically across the opening (each slab overlaps
    # its expected vertical band).
    for i, slab in enumerate(slabs):
        band_lo = SILL_H + i * SLAB_PITCH
        band_hi = SILL_H + (i + 1) * SLAB_PITCH
        aabb = ctx.part_world_aabb(slab)
        assert aabb is not None
        ctx.check(
            f"slab_{i} sits in its vertical band [{band_lo:.3f}..{band_hi:.3f}]",
            aabb[0][2] >= band_lo - 0.001 and aabb[1][2] <= band_hi + 0.001,
            details=f"slab_{i} z=({aabb[0][2]:.4f}, {aabb[1][2]:.4f})",
        )

    # The lowest slab sits on (just above) the sill; the highest slab is just
    # below the header bottom.
    ctx.expect_gap(
        slabs[0],
        frame,
        axis="z",
        positive_elem="slab_body",
        negative_elem="sill",
        max_gap=SLAB_PITCH * 0.6,
        name="slab_0 bottom is near the sill",
    )
    ctx.expect_gap(
        frame,
        slabs[N_SLABS - 1],
        axis="z",
        positive_elem="lintel_header",
        negative_elem="slab_body",
        max_gap=SLAB_PITCH * 0.6,
        name="slab_3 top is near the header bottom",
    )

    # Closed slabs stand on the floor (not below z=0).
    for i, slab in enumerate(slabs):
        aabb = ctx.part_world_aabb(slab)
        assert aabb is not None
        ctx.check(
            f"slab_{i} stands above the floor",
            aabb[0][2] >= -0.001,
            details=f"slab_{i} min z = {aabb[0][2]:.4f}",
        )

    # The doorway is a true through-opening: no fixed frame geometry blocks the
    # central window.
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

    # Fixed furniture stays mounted.
    ctx.expect_contact(keypad, left_housing, name="keypad mounted on left cowl")
    ctx.expect_contact(left_housing, frame, name="left housing mounted to frame")
    ctx.expect_contact(right_housing, frame, name="right housing mounted to frame")
    ctx.expect_contact(haz_left, left_housing, name="left hazard on left cowl")
    ctx.expect_contact(haz_right, right_housing, name="right hazard on right cowl")

    # --- Closed-pose centres for the open-pose comparison. ---
    closed_z = {}
    for i, slab in enumerate(slabs):
        pos = ctx.part_world_position(slab)
        assert pos is not None
        closed_z[i] = pos[2]

    # --- Open pose: drive all four independent joints to full travel. ---
    open_pose = {slab_joints[i]: _slab_travel(i) for i in range(N_SLABS)}
    with ctx.pose(open_pose):
        # Every slab moves upward.
        for i, slab in enumerate(slabs):
            pos = ctx.part_world_position(slab)
            assert pos is not None
            rise = pos[2] - closed_z[i]
            ctx.check(
                f"slab_{i} retracts upward when open",
                rise > 0.10,
                details=f"closed z={closed_z[i]:.4f}, open z={pos[2]:.4f}, rise={rise:.4f}",
            )

        # Staggered travel ordering: slab_0 travels more than slab_1, etc.
        travels = [_slab_travel(i) for i in range(N_SLABS)]
        for i in range(N_SLABS - 1):
            ctx.check(
                f"slab_{i} travels farther than slab_{i + 1} (staggered nesting)",
                travels[i] > travels[i + 1] + 0.05,
                details=f"travel[{i}]={travels[i]:.4f}, travel[{i + 1}]={travels[i + 1]:.4f}",
            )

        # Every slab nests inside the header pocket: XY within the header and
        # Z overlapping the header zone.
        for i, slab in enumerate(slabs):
            ctx.expect_within(
                slab,
                frame,
                axes="xy",
                inner_elem="slab_body",
                outer_elem="lintel_header",
                name=f"slab_{i} XY nests inside the header pocket",
            )
            ctx.expect_overlap(
                slab,
                frame,
                axes="z",
                elem_a="slab_body",
                elem_b="lintel_header",
                min_overlap=SLAB_H * 0.8,
                name=f"slab_{i} Z is inside the header pocket",
            )

        # The full doorway is clear below the nested stack: slab_0 bottom is
        # above the opening top (the header bottom line).
        aabb0 = ctx.part_world_aabb(slabs[0])
        assert aabb0 is not None
        ctx.check(
            "full doorway clear below the nested slab stack",
            aabb0[0][2] >= SILL_H + OPENING_H - 0.005,
            details=(
                f"slab_0 min z = {aabb0[0][2]:.4f}, "
                f"opening top = {SILL_H + OPENING_H:.4f}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
