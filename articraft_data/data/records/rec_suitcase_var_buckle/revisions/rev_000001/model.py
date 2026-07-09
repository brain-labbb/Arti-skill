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

# Buckle strap dimensions
STRAP_X = 0.13           # X offset of each strap from center
STRAP_W = 0.025          # strap width
STRAP_T = 0.003          # strap leather thickness
BODY_FRONT = -D / 2.0    # front face Y coordinate

BUCKLE_FW = 0.038        # buckle frame outer width (X)
BUCKLE_FH = 0.032        # buckle frame outer height (Z)
BUCKLE_BAR = 0.004       # bar cross-section size
BUCKLE_DEPTH = 0.005     # bar depth (Y, proud direction)
BUCKLE_STANDOFF = 0.004  # gap between body front face and buckle inner face

LOWER_STRAP_L = 0.045    # lower anchor strap length (below buckle)
UPPER_STRAP_L = 0.065    # upper strap length (on lid, crosses seam)
TONGUE_L = 0.055         # strap tongue length (hanging below buckle)

# Y layering (proud of body front face toward -Y)
# Lower strap embeds slightly into the body wall for connectivity
STRAP_EMBED = 0.001
LOWER_STRAP_CY = BODY_FRONT - STRAP_T / 2.0 + STRAP_EMBED
BUCKLE_CY = BODY_FRONT - BUCKLE_STANDOFF - BUCKLE_DEPTH / 2.0
TONGUE_CY = BUCKLE_CY - BUCKLE_DEPTH / 2.0 - STRAP_T / 2.0


def _add_box_shell(part, *, width, depth, height, wall, material, z0=0.0, open_top=True):
    """Add a hollow rectangular shell: floor/ceiling panel + four walls."""
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
    caph = (z_hi - z_lo)
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


def _add_buckle_frame(part, *, cx, cy, cz, material, prefix):
    """Add a rectangular buckle frame as inline bar visuals.

    The frame lies in the XZ plane at Y = cy.  The center bar (prong bar)
    sits at Z = cz, which is the strap-tongue hinge line.
    """
    fw = BUCKLE_FW
    fh = BUCKLE_FH
    bar = BUCKLE_BAR
    dep = BUCKLE_DEPTH
    # Top bar
    part.visual(
        Box((fw, dep, bar)),
        origin=Origin(xyz=(cx, cy, cz + fh / 2.0 - bar / 2.0)),
        material=material,
        name=f"{prefix}_top_bar",
    )
    # Bottom bar
    part.visual(
        Box((fw, dep, bar)),
        origin=Origin(xyz=(cx, cy, cz - fh / 2.0 + bar / 2.0)),
        material=material,
        name=f"{prefix}_bottom_bar",
    )
    # Left bar (between top and bottom bars)
    part.visual(
        Box((bar, dep, fh - 2.0 * bar)),
        origin=Origin(xyz=(cx - fw / 2.0 + bar / 2.0, cy, cz)),
        material=material,
        name=f"{prefix}_left_bar",
    )
    # Right bar
    part.visual(
        Box((bar, dep, fh - 2.0 * bar)),
        origin=Origin(xyz=(cx + fw / 2.0 - bar / 2.0, cy, cz)),
        material=material,
        name=f"{prefix}_right_bar",
    )
    # Center bar (prong bar — the tongue wraps around this bar)
    part.visual(
        Box((fw - 2.0 * bar, dep, bar)),
        origin=Origin(xyz=(cx, cy, cz)),
        material=material,
        name=f"{prefix}_center_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_travel_suitcase")

    leather = model.material("leather_body", rgba=(0.32, 0.15, 0.10, 1.0))
    trim = model.material("leather_trim", rgba=(0.18, 0.09, 0.06, 1.0))
    corner = model.material("corner_cap", rgba=(0.22, 0.11, 0.07, 1.0))
    wood = model.material("wood_handle", rgba=(0.55, 0.27, 0.07, 1.0))
    metal = model.material("metal_buckle", rgba=(0.60, 0.56, 0.45, 1.0))
    strap = model.material("strap_leather", rgba=(0.24, 0.11, 0.07, 1.0))

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

    # --- Buckle strap assemblies (body-side: lower strap + buckle frame) -----
    n_straps = 2
    for i in range(n_straps):
        sx = -STRAP_X + i * 2.0 * STRAP_X  # -0.13, +0.13

        # Lower anchor strap: riveted to the body front below the buckle
        lower_z_top = HB - BUCKLE_FH / 2.0  # just below buckle bottom bar
        lower_cz = lower_z_top - LOWER_STRAP_L / 2.0
        body.visual(
            Box((STRAP_W, STRAP_T, LOWER_STRAP_L)),
            origin=Origin(xyz=(sx, LOWER_STRAP_CY, lower_cz)),
            material=strap,
            name=f"lower_strap_{i}",
        )

        # Mounting bosses: bridge body front wall to the buckle frame so the
        # buckle reads as mechanically grounded on the body.
        boss_embed = 0.002  # penetration into the body wall for connectivity
        boss_inner_y = BODY_FRONT + boss_embed
        boss_outer_y = BUCKLE_CY + BUCKLE_DEPTH / 2.0  # meets buckle inner face
        boss_depth = boss_inner_y - boss_outer_y
        boss_cy = (boss_inner_y + boss_outer_y) / 2.0
        for bz_off in (BUCKLE_FH / 2.0 - BUCKLE_BAR / 2.0,
                       -(BUCKLE_FH / 2.0 - BUCKLE_BAR / 2.0)):
            body.visual(
                Box((0.018, boss_depth, 0.014)),
                origin=Origin(xyz=(sx, boss_cy, HB + bz_off)),
                material=metal,
                name=f"buckle_{i}_boss_{'top' if bz_off > 0 else 'bot'}",
            )

        # Buckle frame mounted at the seam on the front face
        _add_buckle_frame(
            body, cx=sx, cy=BUCKLE_CY, cz=HB, material=metal, prefix=f"buckle_{i}"
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
    # lid corner caps
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
    # lid trim straps (decorative vertical bands)
    for sx in (-0.16, 0.16):
        lid.visual(
            Box((0.03, D + 0.004, 0.012)),
            origin=Origin(xyz=(sx, -D / 2.0, SEAM_GAP + 0.010)),
            material=trim,
            name=f"lid_strap_{'l' if sx < 0 else 'r'}",
        )

    # Upper buckle straps on lid (cross the seam into the buckle)
    # In lid local frame the seam is at Z ≈ 0; the strap extends below the
    # seam into the buckle and above onto the lid front face.
    upper_strap_ly = -(D - T / 2.0) - T / 2.0 - STRAP_T / 2.0  # proud of lid front wall
    upper_z_lo = -0.020   # extends 20 mm below seam into the buckle slot
    upper_cz = upper_z_lo + UPPER_STRAP_L / 2.0
    for i in range(n_straps):
        sx = -STRAP_X + i * 2.0 * STRAP_X
        lid.visual(
            Box((STRAP_W, STRAP_T, UPPER_STRAP_L)),
            origin=Origin(xyz=(sx, upper_strap_ly, upper_cz)),
            material=strap,
            name=f"upper_strap_{i}",
        )

    # Lid hinge articulation
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    # --- Strap tongues (hinged at each buckle center bar) --------------------
    for i in range(n_straps):
        sx = -STRAP_X + i * 2.0 * STRAP_X

        tongue = model.part(f"strap_tongue_{i}")
        # Tongue leather strip: extends downward from the hinge bar.
        # Y offset places the tongue in front of the buckle outer face.
        tongue_y = TONGUE_CY - BUCKLE_CY  # local offset from joint to tongue center
        tongue.visual(
            Box((STRAP_W, STRAP_T, TONGUE_L)),
            origin=Origin(xyz=(0.0, tongue_y, -TONGUE_L / 2.0)),
            material=strap,
            name="tongue_strip",
        )
        # Metal tip at the free end of the tongue
        tongue.visual(
            Box((STRAP_W + 0.004, STRAP_T + 0.001, 0.008)),
            origin=Origin(xyz=(0.0, tongue_y, -TONGUE_L + 0.004)),
            material=metal,
            name="tongue_tip",
        )

        model.articulation(
            f"strap_tongue_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=tongue,
            # Joint at the buckle center bar on the body front face
            origin=Origin(xyz=(sx, BUCKLE_CY, HB)),
            # axis = -X: positive q swings the tongue bottom toward -Y (outward)
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.4),
        )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("suitcase_body")
    lid = object_model.get_part("suitcase_lid")
    tongue_0 = object_model.get_part("strap_tongue_0")
    tongue_1 = object_model.get_part("strap_tongue_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    tongue_0_hinge = object_model.get_articulation("strap_tongue_0_hinge")

    # --- Closed seam: lid seats on body at the front wall ---
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

    # --- Lid opens upward ---
    with ctx.pose({lid_hinge: 1.8}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid swings up when opened",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.05,
        details=f"closed_top={closed_lid_aabb}, open_top={open_lid_aabb}",
    )

    # --- Carry handle present on the front face ---
    grip_aabb = ctx.part_element_world_aabb(body, elem="handle_grip")
    ctx.check(
        "carry handle grip is present on the front face",
        grip_aabb is not None and grip_aabb[0][1] < -(D / 2.0),
        details=f"grip_aabb={grip_aabb}",
    )

    # --- Two buckle strap tongues exist and are distinct ---
    ctx.check(
        "two buckle strap tongues are present",
        tongue_0 is not None and tongue_1 is not None,
        details="expected strap_tongue_0 and strap_tongue_1",
    )

    # --- Buckle frames are visible on the body front face ---
    for i in range(2):
        bar_aabb = ctx.part_element_world_aabb(body, elem=f"buckle_{i}_center_bar")
        ctx.check(
            f"buckle frame {i} is present on the body front",
            bar_aabb is not None and bar_aabb[0][1] < BODY_FRONT,
            details=f"buckle_{i}_center_bar aabb={bar_aabb}",
        )

    # --- Upper straps visible on the lid front face ---
    with ctx.pose({lid_hinge: 0.0}):
        for i in range(2):
            us_aabb = ctx.part_element_world_aabb(lid, elem=f"upper_strap_{i}")
            ctx.check(
                f"upper strap {i} crosses the seam on the lid front",
                us_aabb is not None and us_aabb[0][2] < HB and us_aabb[1][2] > HB - 0.01,
                details=f"upper_strap_{i} aabb={us_aabb}",
            )

    # --- Tongue swings outward when released ---
    with ctx.pose({tongue_0_hinge: 0.0}):
        closed_tongue_aabb = ctx.part_world_aabb(tongue_0)
    with ctx.pose({tongue_0_hinge: 1.2}):
        open_tongue_aabb = ctx.part_world_aabb(tongue_0)
    ctx.check(
        "strap tongue swings outward to release the buckle",
        closed_tongue_aabb is not None
        and open_tongue_aabb is not None
        and open_tongue_aabb[0][1] < closed_tongue_aabb[0][1] - 0.01,
        details=f"closed={closed_tongue_aabb}, open={open_tongue_aabb}",
    )

    # --- Tongue at rest hangs in front of the body (not inside) ---
    with ctx.pose({tongue_0_hinge: 0.0}):
        tongue_pos = ctx.part_world_position(tongue_0)
    ctx.check(
        "strap tongue rests in front of the body front face",
        tongue_pos is not None and tongue_pos[1] < BODY_FRONT - 0.005,
        details=f"tongue_pos={tongue_pos}",
    )

    return ctx.report()
