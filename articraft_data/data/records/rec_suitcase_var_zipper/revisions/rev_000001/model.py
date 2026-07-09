from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.46          # width  (X)
D = 0.34          # depth  (Y)
HB = 0.085        # body half height (Z)
HL = 0.075        # lid half height (Z)
T = 0.012         # shell wall thickness
SEAM_GAP = 0.0015  # closed clearance between body rim and lid rim
H = HB + HL       # total closed height

# Zipper track dimensions
ZIP_TAPE_W = 0.016     # tape strip height on wall face (Z extent)
ZIP_TAPE_T = 0.002     # tape thickness proud of wall surface (outward)
ZIP_TEETH_W = 0.008    # teeth row height (Z extent)
ZIP_TEETH_T = 0.005    # teeth thickness proud of tape (outward)

# Zipper slider dimensions
SLIDER_W = 0.028       # slider body width (X)
SLIDER_D = 0.018       # slider body depth (Y, away from suitcase face)
SLIDER_H = 0.016       # slider body height (Z)
TAB_W = 0.014          # pull tab width (X)
TAB_T = 0.003          # pull tab thickness (Y)
TAB_L = 0.032          # pull tab hanging length (Z)

# Zipper travel along the front face
ZIP_TRAVEL = W - 0.10   # about 0.36 m of travel


def _add_box_shell(part, *, width, depth, height, wall, material, z0=0.0, open_top=True):
    """Add a hollow rectangular shell: floor/ceiling panel + four walls.

    Pieces overlap at the edges so the part reads as one connected island.
    """
    cap_z = z0 + (wall / 2.0) if open_top else z0 + height - wall / 2.0
    part.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, cap_z)),
        material=material,
        name="shell_panel",
    )
    wall_h = height
    cz = z0 + height / 2.0
    part.visual(
        Box((width, wall, wall_h)),
        origin=Origin(xyz=(0.0, -(depth / 2.0 - wall / 2.0), cz)),
        material=material,
        name="wall_front",
    )
    part.visual(
        Box((width, wall, wall_h)),
        origin=Origin(xyz=(0.0, (depth / 2.0 - wall / 2.0), cz)),
        material=material,
        name="wall_back",
    )
    part.visual(
        Box((wall, depth, wall_h)),
        origin=Origin(xyz=(-(width / 2.0 - wall / 2.0), 0.0, cz)),
        material=material,
        name="wall_left",
    )
    part.visual(
        Box((wall, depth, wall_h)),
        origin=Origin(xyz=((width / 2.0 - wall / 2.0), 0.0, cz)),
        material=material,
        name="wall_right",
    )


def _add_corner_caps(part, *, width, depth, z_lo, z_hi, material, prefix):
    cap = 0.045
    capz = (z_lo + z_hi) / 2.0
    caph = z_hi - z_lo
    i = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            part.visual(
                Box((cap, cap, caph)),
                origin=Origin(
                    xyz=(
                        sx * (width / 2.0 - cap / 2.0 + 0.004),
                        sy * (depth / 2.0 - cap / 2.0 + 0.004),
                        capz,
                    )
                ),
                material=material,
                name=f"{prefix}_corner_{i}",
            )
            i += 1


def _add_zipper_track(
    part,
    *,
    width,
    depth,
    seam_z,
    tape_w,
    tape_t,
    teeth_w,
    teeth_t,
    mat_tape,
    mat_teeth,
    wall_outer_front,
    wall_outer_left,
    wall_outer_right,
    y_center_side,
    teeth_dir,
):
    """Add zipper tape and teeth along three sides (front, left, right).

    seam_z: z coordinate of the seam line in this part's local frame.
    wall_outer_front: y coordinate of the front wall outer face.
    wall_outer_left/right: x coordinate of side wall outer faces.
    y_center_side: y center for side-wall segments.
    teeth_dir: +1 if teeth go below the seam (body), -1 if above (lid).
    """
    # Tape z: centered just to one side of the seam so it doesn't cross it.
    # Body: tape sits below seam (tape center = seam_z - tape_w/2)
    # Lid:  tape sits above seam (tape center = seam_z + tape_w/2)
    tape_cz = seam_z + teeth_dir * (-tape_w / 2.0)
    # Teeth are entirely on the seam side, not crossing the seam line.
    # Body: teeth from seam_z - teeth_w to seam_z
    # Lid:  teeth from seam_z to seam_z + teeth_w
    teeth_cz = seam_z + teeth_dir * (-teeth_w / 2.0)

    # Segment definitions: (tape_pos, tape_size, teeth_pos, teeth_size)
    # Front segment: runs along X, on the front wall outer face
    front_tape_y = wall_outer_front - tape_t / 2.0  # proud of outer face (further -Y)
    front_teeth_y = wall_outer_front - tape_t - teeth_t / 2.0  # on top of tape

    segments = [
        # Front
        (
            (0.0, front_tape_y, tape_cz),
            (width, tape_t, tape_w),
            (0.0, front_teeth_y, teeth_cz),
            (width * 0.96, teeth_t, teeth_w),
        ),
        # Left
        (
            (wall_outer_left - tape_t / 2.0, y_center_side, tape_cz),
            (tape_t, depth, tape_w),
            (wall_outer_left - tape_t - teeth_t / 2.0, y_center_side, teeth_cz),
            (teeth_t, depth * 0.96, teeth_w),
        ),
        # Right
        (
            (wall_outer_right + tape_t / 2.0, y_center_side, tape_cz),
            (tape_t, depth, tape_w),
            (wall_outer_right + tape_t + teeth_t / 2.0, y_center_side, teeth_cz),
            (teeth_t, depth * 0.96, teeth_w),
        ),
    ]
    for i, (tape_pos, tape_size, teeth_pos, teeth_size) in enumerate(segments):
        part.visual(
            Box(tape_size),
            origin=Origin(xyz=tape_pos),
            material=mat_tape,
            name=f"zip_tape_{i}",
        )
        part.visual(
            Box(teeth_size),
            origin=Origin(xyz=teeth_pos),
            material=mat_teeth,
            name=f"zip_teeth_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_travel_suitcase")

    leather = model.material("leather_body", rgba=(0.32, 0.15, 0.10, 1.0))
    trim = model.material("leather_trim", rgba=(0.18, 0.09, 0.06, 1.0))
    corner = model.material("corner_cap", rgba=(0.22, 0.11, 0.07, 1.0))
    wood = model.material("wood_handle", rgba=(0.55, 0.27, 0.07, 1.0))
    metal = model.material("metal_hardware", rgba=(0.58, 0.58, 0.62, 1.0))
    zip_tape_mat = model.material("zipper_tape", rgba=(0.10, 0.07, 0.05, 1.0))
    zip_teeth_mat = model.material("zipper_teeth", rgba=(0.72, 0.58, 0.25, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("suitcase_body")
    _add_box_shell(
        body, width=W, depth=D, height=HB, wall=T, material=leather, z0=0.0, open_top=True
    )
    _add_corner_caps(
        body, width=W, depth=D, z_lo=0.0, z_hi=HB - 0.02, material=corner, prefix="body"
    )
    # vertical leather trim straps on the long faces
    for sx in (-0.16, 0.16):
        body.visual(
            Box((0.03, D + 0.004, 0.012)),
            origin=Origin(xyz=(sx, 0.0, HB - 0.018)),
            material=trim,
            name=f"body_strap_{'l' if sx < 0 else 'r'}",
        )

    # carry handle on the front (-Y) face: wooden grip held by two loops
    grip_y = -(D / 2.0) - 0.022
    grip_z = HB - 0.006
    body.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(xyz=(0.0, grip_y, grip_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wood,
        name="handle_grip",
    )
    for sx in (-0.06, 0.06):
        body.visual(
            Box((0.02, 0.030, 0.022)),
            origin=Origin(xyz=(sx, -(D / 2.0) - 0.008, grip_z)),
            material=trim,
            name=f"handle_loop_{'l' if sx < 0 else 'r'}",
        )

    # Zipper track on body side (three segments: front, left, right)
    _add_zipper_track(
        body,
        width=W,
        depth=D,
        seam_z=HB,
        tape_w=ZIP_TAPE_W,
        tape_t=ZIP_TAPE_T,
        teeth_w=ZIP_TEETH_W,
        teeth_t=ZIP_TEETH_T,
        mat_tape=zip_tape_mat,
        mat_teeth=zip_teeth_mat,
        wall_outer_front=-(D / 2.0),
        wall_outer_left=-(W / 2.0),
        wall_outer_right=(W / 2.0),
        y_center_side=0.0,
        teeth_dir=1.0,  # body teeth below seam (tape below, teeth approach seam from below)
    )

    # --- Lid -----------------------------------------------------------------
    lid = model.part("suitcase_lid")
    lid_panel_z = HL - T / 2.0
    wall_h = HL - T - SEAM_GAP
    wall_cz = SEAM_GAP + wall_h / 2.0
    # top panel
    lid.visual(
        Box((W, D, T)),
        origin=Origin(xyz=(0.0, -D / 2.0, lid_panel_z)),
        material=leather,
        name="shell_panel",
    )
    # front wall
    lid.visual(
        Box((W, T, wall_h)),
        origin=Origin(xyz=(0.0, -(D - T / 2.0), wall_cz)),
        material=leather,
        name="wall_front",
    )
    # back wall (near hinge)
    lid.visual(
        Box((W, T, wall_h)),
        origin=Origin(xyz=(0.0, -(T / 2.0), wall_cz)),
        material=leather,
        name="wall_back",
    )
    lid.visual(
        Box((T, D, wall_h)),
        origin=Origin(xyz=(-(W / 2.0 - T / 2.0), -D / 2.0, wall_cz)),
        material=leather,
        name="wall_left",
    )
    lid.visual(
        Box((T, D, wall_h)),
        origin=Origin(xyz=((W / 2.0 - T / 2.0), -D / 2.0, wall_cz)),
        material=leather,
        name="wall_right",
    )
    # lid corner caps (local frame)
    capm = 0.045
    for i, (sx, ly) in enumerate(
        [
            (-1.0, -(T / 2.0)),
            (1.0, -(T / 2.0)),
            (-1.0, -(D - T / 2.0)),
            (1.0, -(D - T / 2.0)),
        ]
    ):
        lid.visual(
            Box((capm, capm, HL - 0.02)),
            origin=Origin(
                xyz=(sx * (W / 2.0 - capm / 2.0 + 0.004), ly, (SEAM_GAP + HL - 0.02) / 2.0)
            ),
            material=corner,
            name=f"lid_corner_{i}",
        )
    # lid trim straps
    for sx in (-0.16, 0.16):
        lid.visual(
            Box((0.03, D + 0.004, 0.012)),
            origin=Origin(xyz=(sx, -D / 2.0, SEAM_GAP + 0.010)),
            material=trim,
            name=f"lid_strap_{'l' if sx < 0 else 'r'}",
        )

    # Zipper track on lid side (three segments: front, left, right)
    _add_zipper_track(
        lid,
        width=W,
        depth=D,
        seam_z=SEAM_GAP,
        tape_w=ZIP_TAPE_W,
        tape_t=ZIP_TAPE_T,
        teeth_w=ZIP_TEETH_W,
        teeth_t=ZIP_TEETH_T,
        mat_tape=zip_tape_mat,
        mat_teeth=zip_teeth_mat,
        wall_outer_front=-D,
        wall_outer_left=-(W / 2.0),
        wall_outer_right=(W / 2.0),
        y_center_side=-D / 2.0,
        teeth_dir=-1.0,  # lid teeth above seam (tape above, teeth approach seam from above)
    )

    # Lid hinge (revolute, at rear top edge)
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    # --- Zipper slider -------------------------------------------------------
    # The slider travels along the front face on a prismatic joint.
    # q=0: slider at the left end of travel.
    # q=ZIP_TRAVEL: slider at the right end of travel.
    slider = model.part("zipper_slider")

    # Slider body: sits on the outside of the front face, at the seam line.
    # Back face contacts the suitcase front face; body extends outward (-Y).
    slider.visual(
        Box((SLIDER_W, SLIDER_D, SLIDER_H)),
        origin=Origin(xyz=(0.0, -SLIDER_D / 2.0, 0.0)),
        material=metal,
        name="slider_body",
    )
    # Small channel ridge on the back face (touches the teeth track)
    slider.visual(
        Box((SLIDER_W * 0.6, 0.003, SLIDER_H * 0.5)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=metal,
        name="slider_channel",
    )
    # Pull tab bail (wire loop connecting tab to slider body)
    slider.visual(
        Cylinder(radius=0.004, length=SLIDER_W * 0.5),
        origin=Origin(
            xyz=(0.0, -SLIDER_D, -SLIDER_H * 0.3),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="slider_bail",
    )
    # Pull tab: flat metal tab hanging below the slider
    slider.visual(
        Box((TAB_W, TAB_T, TAB_L)),
        origin=Origin(xyz=(0.0, -SLIDER_D, -(SLIDER_H * 0.3 + TAB_L / 2.0))),
        material=metal,
        name="pull_tab",
    )
    # Small end stop bump at the bottom of the pull tab
    slider.visual(
        Box((TAB_W + 0.004, TAB_T + 0.002, 0.004)),
        origin=Origin(xyz=(0.0, -SLIDER_D, -(SLIDER_H * 0.3 + TAB_L - 0.002))),
        material=metal,
        name="tab_end_stop",
    )

    # Prismatic joint: slider travels along +X on the front face at seam height.
    # Articulation origin at the left end of travel, on the suitcase front face.
    model.articulation(
        "zipper_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(-ZIP_TRAVEL / 2.0, -(D / 2.0), HB)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.15, lower=0.0, upper=ZIP_TRAVEL),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("suitcase_body")
    lid = object_model.get_part("suitcase_lid")
    slider = object_model.get_part("zipper_slider")
    lid_hinge = object_model.get_articulation("lid_hinge")
    zip_slide = object_model.get_articulation("zipper_slide")

    # --- Closed seam check ---------------------------------------------------
    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.006,
            positive_elem="wall_front",
            negative_elem="wall_front",
            name="lid seats on body at the closed seam",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            name="closed lid footprint matches body footprint",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)

    # --- Lid opens upward ----------------------------------------------------
    with ctx.pose({lid_hinge: 1.8}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid swings up when opened",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.05,
        details=f"closed_top={closed_lid_aabb}, open_top={open_lid_aabb}",
    )

    # --- Handle exists -------------------------------------------------------
    grip_aabb = ctx.part_element_world_aabb(body, elem="handle_grip")
    ctx.check(
        "carry handle grip is present on the front face",
        grip_aabb is not None and grip_aabb[0][1] < -(D / 2.0),
        details=f"grip_aabb={grip_aabb}",
    )

    # --- Zipper track exists on both body and lid ----------------------------
    ctx.check(
        "zipper tape on body (front segment)",
        body.get_visual("zip_tape_0") is not None,
    )
    ctx.check(
        "zipper teeth on body (front segment)",
        body.get_visual("zip_teeth_0") is not None,
    )
    ctx.check(
        "zipper tape on lid (front segment)",
        lid.get_visual("zip_tape_0") is not None,
    )
    ctx.check(
        "zipper teeth on lid (front segment)",
        lid.get_visual("zip_teeth_0") is not None,
    )
    # Side segments exist
    ctx.check(
        "zipper tape on body (left segment)",
        body.get_visual("zip_tape_1") is not None,
    )
    ctx.check(
        "zipper tape on body (right segment)",
        body.get_visual("zip_tape_2") is not None,
    )

    # --- Zipper slider exists and has pull tab -------------------------------
    ctx.check("zipper slider part exists", slider is not None)
    ctx.check(
        "slider has pull tab visual",
        slider.get_visual("pull_tab") is not None,
    )

    # --- Slider wraps around teeth track (intentional embedding) -------------
    # The slider body mechanically encloses the zipper teeth on both body and
    # lid sides. This is captured-hardware overlap, scoped to the teeth/slider
    # elements only.
    ctx.allow_overlap(
        body,
        slider,
        elem_a="zip_teeth_0",
        elem_b="slider_body",
        reason="The zipper slider body wraps around the teeth track; the teeth pass through the slider channel.",
    )
    ctx.allow_overlap(
        lid,
        slider,
        elem_a="zip_teeth_0",
        elem_b="slider_body",
        reason="The zipper slider body wraps around the lid-side teeth track; the teeth pass through the slider channel.",
    )

    # Proof: slider body contacts the body-side teeth (seated on the track)
    ctx.expect_contact(
        slider,
        body,
        elem_a="slider_body",
        elem_b="zip_tape_0",
        name="slider is seated on body zipper tape",
    )

    # --- Slider travels along +X on the front face ---------------------------
    zip_upper = zip_slide.motion_limits.upper
    with ctx.pose({zip_slide: 0.0}):
        pos_start = ctx.part_world_position(slider)
    with ctx.pose({zip_slide: zip_upper}):
        pos_end = ctx.part_world_position(slider)

    ctx.check(
        "zipper slider travels along +X on the front face",
        pos_start is not None
        and pos_end is not None
        and pos_end[0] > pos_start[0] + 0.1,
        details=f"start_x={pos_start}, end_x={pos_end}",
    )

    # Slider stays at seam height during travel
    ctx.check(
        "zipper slider stays at seam height during travel",
        pos_start is not None
        and pos_end is not None
        and abs(pos_end[2] - pos_start[2]) < 0.002,
        details=f"start_z={pos_start[2] if pos_start else None}, end_z={pos_end[2] if pos_end else None}",
    )

    # Slider stays on the front face (y near -D/2)
    ctx.check(
        "zipper slider stays on the front face",
        pos_start is not None
        and pos_start[1] < -(D / 2.0) + 0.005,
        details=f"start_y={pos_start[1] if pos_start else None}",
    )

    return ctx.report()
