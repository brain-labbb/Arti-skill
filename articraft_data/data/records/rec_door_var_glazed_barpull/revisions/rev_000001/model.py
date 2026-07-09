from __future__ import annotations

# Single hinged door, matte dark-graphite finish.
# - FIXED root: slim dark door frame (two side jambs + head jamb).
# - Door leaf: dark-graphite blank (CadQuery) with a tall, narrow vertical glass
#   vision strip cut through it, set off-center toward the latch side.
# - Separate translucent glass pane filling the vision opening.
# - Long vertical tubular bar pull on two standoffs, mounted on the room-side
#   face near the latch edge, spanning the mid-height of the leaf.
# - PRIMARY articulation: the leaf swings on its hinge edge (REVOLUTE, vertical).
#
# Frame convention:
#   X = door width (hinge edge at small X, latch edge at large X)
#   Y = door thickness (room side at +Y)
#   Z = height (floor at z=0)

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

# --- Real-world dimensions (meters) ---
LEAF_W = 0.900
LEAF_H = 2.040
LEAF_T = 0.042

JAMB_W = 0.038          # slim jamb face width (along X for side jambs)
JAMB_D = 0.120          # jamb depth (along Y)
HEAD_GAP = 0.004
SIDE_GAP = 0.003
HINGE_LAP = 0.004       # hinge jamb laps the leaf hinge edge

# Vision strip: tall narrow vertical glass, off-center toward the latch side.
GLASS_W = 0.110         # strip width
GLASS_H = 1.520         # strip height (tall, nearly full leaf height)
GLASS_CX = 0.580        # strip center X from hinge edge — off-center toward latch (+X)
GLASS_CZ = LEAF_H / 2.0 + 0.04
GLASS_T = 0.008         # glass pane thickness (sits inside the leaf)

OPENING_H = LEAF_H + HEAD_GAP
SILL_Z = 0.0

# --- Bar pull dimensions ---
BAR_LENGTH = 0.800      # total bar length (vertical span)
BAR_OD = 0.019          # outer diameter (3/4" tube)
BAR_ID = 0.014          # inner diameter (hollow bore)
BAR_END_CAP = 0.003     # solid end-cap disk thickness

STANDOFF_BASE_D = 0.040 # base plate diameter
STANDOFF_BASE_H = 0.006 # base plate thickness
STANDOFF_POST_D = BAR_OD  # post diameter matches bar OD for flush profile
STANDOFF_POST_H = 0.038   # post height

STANDOFF_REACH = STANDOFF_BASE_H + STANDOFF_POST_H  # 0.044 m from leaf face
STANDOFF_SPAN = BAR_LENGTH - 0.080  # 0.720 m between standoff centers

# Bar centerline sits at standoff reach: the standoff post enters the bar lower
# half (like a real saddle mount), ensuring geometric connectivity.
BAR_Y_OFFSET = STANDOFF_REACH  # bar centerline offset from leaf face

HANDLE_X = LEAF_W - 0.060  # 0.840, near latch edge
BAR_CENTER_Z = LEAF_H / 2.0  # 1.020, mid-height of the leaf


def _build_leaf_cq() -> cq.Workplane:
    """Dark-graphite leaf with a through vision opening.

    Local frame: hinge edge at local x=0, leaf extends along +X (0..LEAF_W),
    thickness centered on Y, height 0..LEAF_H.
    """
    blank = cq.Workplane("XY").box(LEAF_W, LEAF_T, LEAF_H, centered=(False, True, False))

    # Through-cut the vision opening (full thickness in Y).
    opening = (
        cq.Workplane("XZ")
        .workplane(offset=-LEAF_T)  # start well behind the leaf
        .center(GLASS_CX, GLASS_CZ)
        .rect(GLASS_W, GLASS_H)
        .extrude(2 * LEAF_T)        # cut all the way through
    )
    leaf = blank.cut(opening)

    # Soften the front face edges of the opening so it reads as a glazed bead.
    try:
        leaf = leaf.edges(">Y").edges("|Z").fillet(0.003)
    except Exception:
        pass
    return leaf


def _build_glass_cq() -> cq.Workplane:
    """Translucent glass pane filling the vision opening, in its own local frame.

    Local frame: pane centered at origin, thin along Y, in the XZ plane.
    """
    pane = cq.Workplane("XZ").rect(GLASS_W + 0.012, GLASS_H + 0.012).extrude(GLASS_T)
    return pane


def _build_pull_bar_cq() -> cq.Workplane:
    """Hollow tubular pull bar along Z, centered at origin, with integral end caps.

    Built as a solid cylinder with an interior bore, leaving solid end caps
    at each end. This produces a single connected mesh (no boolean-union
    artifacts between separate cap and tube pieces).

    Local frame: bar axis along Z, centered at XY origin. The tube runs from
    z=-BAR_LENGTH/2 to z=+BAR_LENGTH/2 with closed end caps.
    """
    half = BAR_LENGTH / 2.0
    r_out = BAR_OD / 2.0
    r_in = BAR_ID / 2.0

    # Solid outer cylinder (single connected shape)
    solid = cq.Workplane("XY").workplane(offset=-half).circle(r_out).extrude(BAR_LENGTH)

    # Bore out the interior, leaving end caps at each end
    bore_length = BAR_LENGTH - 2.0 * BAR_END_CAP
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-half + BAR_END_CAP)
        .circle(r_in)
        .extrude(bore_length)
    )
    return solid.cut(bore)


def _build_standoff_cq() -> cq.Workplane:
    """Mounting standoff: base plate + cylindrical post, along +Y from origin.

    Uses the "-XZ" workplane whose normal is +Y, so extrusion goes outward
    from the door face. Base plate bottom sits at y=0; post tip at y=REACH.
    """
    # "ZX" workplane has normal=+Y, so extrusion protrudes outward (room side).
    base = (
        cq.Workplane("ZX")
        .circle(STANDOFF_BASE_D / 2.0)
        .extrude(STANDOFF_BASE_H)
    )
    post = (
        cq.Workplane("ZX")
        .workplane(offset=STANDOFF_BASE_H)
        .circle(STANDOFF_POST_D / 2.0)
        .extrude(STANDOFF_POST_H)
    )
    standoff = base.union(post)
    # Fillet the base plate bottom edge for a finished look
    try:
        standoff = standoff.edges("<Y").fillet(0.002)
    except Exception:
        pass
    return standoff


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_graphite_glass_door")
    model.material(name="graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material(name="graphite_frame", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material(name="vision_glass", rgba=(0.78, 0.85, 0.88, 0.35))
    model.material(name="brushed_steel", rgba=(0.74, 0.76, 0.78, 1.0))

    # ---------------- FIXED FRAME (root) ----------------
    frame = model.part("door_frame")

    hinge_jamb_x = -JAMB_W / 2.0 + HINGE_LAP
    frame.visual(
        Box((JAMB_W, JAMB_D, OPENING_H)),
        origin=Origin(xyz=(hinge_jamb_x, 0.0, OPENING_H / 2.0)),
        material="graphite_frame",
        name="hinge_jamb",
    )
    latch_jamb_x = LEAF_W + SIDE_GAP + JAMB_W / 2.0
    frame.visual(
        Box((JAMB_W, JAMB_D, OPENING_H)),
        origin=Origin(xyz=(latch_jamb_x, 0.0, OPENING_H / 2.0)),
        material="graphite_frame",
        name="latch_jamb",
    )
    head_z = OPENING_H + JAMB_W / 2.0
    head_len = (latch_jamb_x + JAMB_W / 2.0) - (hinge_jamb_x - JAMB_W / 2.0)
    head_cx = (latch_jamb_x + hinge_jamb_x) / 2.0
    frame.visual(
        Box((head_len, JAMB_D, JAMB_W)),
        origin=Origin(xyz=(head_cx, 0.0, head_z)),
        material="graphite_frame",
        name="head_jamb",
    )

    # ---------------- DOOR LEAF (swings) ----------------
    leaf = model.part("door_leaf")
    leaf_mesh = mesh_from_cadquery(_build_leaf_cq(), "door_leaf")
    leaf.visual(
        leaf_mesh,
        origin=Origin(xyz=(0.0, 0.0, SILL_Z)),
        material="graphite",
        name="leaf_body",
    )
    # Glass pane filling the vision opening. It rides with the leaf.
    glass_mesh = mesh_from_cadquery(_build_glass_cq(), "vision_glass")
    leaf.visual(
        glass_mesh,
        origin=Origin(xyz=(GLASS_CX, 0.0, GLASS_CZ)),
        material="vision_glass",
        name="vision_pane",
    )

    # ---------------- BAR PULL (fixed to leaf, latch side, room face) ----------------
    bar_pull = model.part("bar_pull")

    # Tubular bar: vertical, offset outward from the leaf face by standoff reach.
    bar_mesh = mesh_from_cadquery(_build_pull_bar_cq(), "pull_bar")
    bar_pull.visual(
        bar_mesh,
        origin=Origin(xyz=(0.0, BAR_Y_OFFSET, 0.0)),
        material="brushed_steel",
        name="pull_bar",
    )

    # Two standoffs: symmetrically placed along the bar length, mounting the
    # bar to the leaf face. Created via shared geometry helper + for-loop.
    standoff_mesh = mesh_from_cadquery(_build_standoff_cq(), "pull_standoff")
    for i in range(2):
        z_offset = (STANDOFF_SPAN / 2.0) * (1 - 2 * i)  # +span/2 for i=0, -span/2 for i=1
        bar_pull.visual(
            standoff_mesh,
            origin=Origin(xyz=(0.0, 0.0, z_offset)),
            material="brushed_steel",
            name=f"standoff_{i}",
        )

    # ---------------- ARTICULATIONS ----------------
    # Primary: leaf swings on vertical hinge axis at the hinge edge.
    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, SILL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.6),
    )
    # Bar pull is rigidly mounted to the leaf face.
    model.articulation(
        "leaf_to_pull",
        ArticulationType.FIXED,
        parent=leaf,
        child=bar_pull,
        origin=Origin(xyz=(HANDLE_X, LEAF_T / 2.0, BAR_CENTER_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("door_frame")
    leaf = object_model.get_part("door_leaf")
    bar_pull = object_model.get_part("bar_pull")
    hinge = object_model.get_articulation("frame_to_leaf")
    pull_joint = object_model.get_articulation("leaf_to_pull")

    # --- Hero size: real door leaf (~0.9 x 2.0 m). ---
    leaf_aabb = ctx.part_world_aabb(leaf)
    if leaf_aabb is not None:
        (lx0, ly0, lz0), (lx1, ly1, lz1) = leaf_aabb
        ctx.check("leaf_width_realistic", 0.85 <= (lx1 - lx0) <= 0.95, details=f"w={lx1 - lx0:.3f}")
        ctx.check("leaf_height_realistic", 1.95 <= (lz1 - lz0) <= 2.10, details=f"h={lz1 - lz0:.3f}")
        ctx.check(
            "leaf_thickness_realistic",
            0.035 <= (ly1 - ly0) <= 0.060,
            details=f"t={ly1 - ly0:.3f}",
        )

    # --- Vision glass present as a separate translucent pane. ---
    glass = leaf.get_visual("vision_pane")
    glass_mat = next((m for m in object_model.materials if m.name == "vision_glass"), None)
    ctx.check(
        "vision_glass_translucent",
        glass.name == "vision_pane"
        and glass_mat is not None
        and glass_mat.rgba is not None
        and glass_mat.rgba[3] < 0.6,
        details=f"glass_mat={getattr(glass_mat, 'rgba', None)}",
    )
    pane_aabb = ctx.part_element_world_aabb(leaf, elem="vision_pane")
    if pane_aabb is not None and leaf_aabb is not None:
        (px0, _, pz0), (px1, _, pz1) = pane_aabb
        # Tall narrow strip.
        ctx.check(
            "vision_strip_tall_narrow",
            (pz1 - pz0) > (px1 - px0) * 4.0 and (pz1 - pz0) > 1.0,
            details=f"strip w={px1 - px0:.3f} h={pz1 - pz0:.3f}",
        )
        # Off-center toward the latch side (center X past leaf mid).
        leaf_mid_x = (leaf_aabb[0][0] + leaf_aabb[1][0]) / 2.0
        ctx.check(
            "vision_strip_offcenter_latch",
            (px0 + px1) / 2.0 > leaf_mid_x,
            details=f"strip_cx={(px0 + px1) / 2.0:.3f} leaf_mid={leaf_mid_x:.3f}",
        )

    # --- Bar pull: vertical tubular bar spanning mid-height on two standoffs. ---
    pull_aabb = ctx.part_world_aabb(bar_pull)
    if pull_aabb is not None and leaf_aabb is not None:
        (bx0, by0, bz0), (bx1, by1, bz1) = pull_aabb
        bar_height = bz1 - bz0
        bar_center_z = (bz0 + bz1) / 2.0
        leaf_center_z = (leaf_aabb[0][2] + leaf_aabb[1][2]) / 2.0

        # Bar pull spans the mid-height of the leaf.
        ctx.check(
            "bar_pull_mid_height",
            abs(bar_center_z - leaf_center_z) < 0.15,
            details=f"bar_cz={bar_center_z:.3f} leaf_cz={leaf_center_z:.3f}",
        )
        ctx.check(
            "bar_pull_long_vertical",
            bar_height > 0.60,
            details=f"bar_height={bar_height:.3f}",
        )

        # Bar protrudes from the room-side leaf face.
        ctx.check(
            "bar_pull_protrudes_room_side",
            by1 > leaf_aabb[1][1],
            details=f"pull_y1={by1:.3f} leaf_y1={leaf_aabb[1][1]:.3f}",
        )

        # Bar is near the latch edge.
        ctx.check(
            "bar_pull_near_latch",
            bx1 > leaf_aabb[1][0] - 0.20,
            details=f"pull_x1={bx1:.3f} latch_x={leaf_aabb[1][0]:.3f}",
        )

    # Pull bar visual: tubular proportions and vertical orientation.
    bar_elem_aabb = ctx.part_element_world_aabb(bar_pull, elem="pull_bar")
    if bar_elem_aabb is not None:
        (bar_x0, bar_y0, bar_z0), (bar_x1, bar_y1, bar_z1) = bar_elem_aabb
        bar_dx = bar_x1 - bar_x0
        bar_dz = bar_z1 - bar_z0
        ctx.check(
            "bar_diameter_realistic",
            0.015 <= bar_dx <= 0.025,
            details=f"bar_dx={bar_dx:.4f}",
        )
        ctx.check(
            "bar_length_realistic",
            0.70 <= bar_dz <= 0.90,
            details=f"bar_dz={bar_dz:.3f}",
        )
        # Vertical: Z extent dominates.
        ctx.check(
            "bar_is_vertical",
            bar_dz > bar_dx * 10.0,
            details=f"dz={bar_dz:.3f} dx={bar_dx:.4f}",
        )

    # Two standoffs exist and are symmetrically placed about the bar center.
    s0 = bar_pull.get_visual("standoff_0")
    s1 = bar_pull.get_visual("standoff_1")
    ctx.check(
        "standoffs_exist",
        s0 is not None and s1 is not None,
        details="bar pull must have two mounting standoffs",
    )
    s0_aabb = ctx.part_element_world_aabb(bar_pull, elem="standoff_0")
    s1_aabb = ctx.part_element_world_aabb(bar_pull, elem="standoff_1")
    if s0_aabb is not None and s1_aabb is not None and pull_aabb is not None:
        s0_cz = (s0_aabb[0][2] + s0_aabb[1][2]) / 2.0
        s1_cz = (s1_aabb[0][2] + s1_aabb[1][2]) / 2.0
        pull_cz = (pull_aabb[0][2] + pull_aabb[1][2]) / 2.0
        ctx.check(
            "standoffs_symmetric_about_bar_center",
            abs((s0_cz - pull_cz) + (s1_cz - pull_cz)) < 0.010,
            details=f"s0_cz={s0_cz:.3f} s1_cz={s1_cz:.3f} pull_cz={pull_cz:.3f}",
        )
        # Standoffs span a significant portion of the bar length.
        standoff_span = abs(s0_cz - s1_cz)
        ctx.check(
            "standoffs_well_spaced",
            standoff_span > 0.50,
            details=f"standoff_span={standoff_span:.3f}",
        )

    # Bar pull joint is FIXED (handle does not move independently).
    ctx.check(
        "bar_pull_fixed_to_leaf",
        pull_joint.articulation_type == ArticulationType.FIXED,
        details=f"joint type={pull_joint.articulation_type}",
    )

    # Hinge joint is REVOLUTE (primary swing).
    ctx.check(
        "hinge_is_revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"hinge type={hinge.articulation_type}",
    )

    # --- Closed pose: hinge edge seats against jamb. ---
    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            leaf,
            frame,
            axis="x",
            max_gap=0.001,
            max_penetration=0.006,
            positive_elem="leaf_body",
            negative_elem="hinge_jamb",
            name="hinge_edge_seats_against_jamb",
        )

    # --- Open pose: leaf swings into room; hinge edge stays connected. ---
    rest_pos = ctx.part_world_position(leaf)
    with ctx.pose({hinge: 1.4}):
        open_pos = ctx.part_world_position(leaf)
        open_aabb = ctx.part_world_aabb(leaf)
        ctx.check(
            "leaf_swings_into_room",
            open_aabb is not None and open_aabb[1][1] > LEAF_W * 0.5,
            details=f"open_y1={open_aabb[1][1] if open_aabb else None}",
        )
        ctx.expect_contact(
            leaf,
            frame,
            elem_a="leaf_body",
            elem_b="hinge_jamb",
            contact_tol=0.02,
            name="hinge_edge_stays_connected_when_open",
        )

    ctx.check(
        "leaf_actually_moves",
        rest_pos is not None and open_pos is not None,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    # --- Overlap allowances ---
    ctx.allow_overlap(
        bar_pull,
        leaf,
        reason="Standoff base plates are seated against and fastened through the leaf face like real pull hardware.",
    )
    ctx.allow_overlap(
        leaf,
        frame,
        elem_a="leaf_body",
        elem_b="hinge_jamb",
        reason="Hinge jamb intentionally laps the leaf hinge edge so the leaf is supported by the frame at the hinge line.",
    )

    return ctx.report()


object_model = build_object_model()
