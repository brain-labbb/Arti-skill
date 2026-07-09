from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ── Table dimensions ───────────────────────────────────────────────
TABLE_X = 0.80
TABLE_Y = 0.54
TOP_Z = 0.51
HALF_X = TABLE_X / 2.0

# Butterfly tabletop: 4 slats per half with a centre hinge gap
SLAT_THICKNESS = 0.024
SLATS_PER_HALF = 4
SLAT_GAP = 0.006
SLAT_W = 0.090
CENTER_GAP = TABLE_X - 2.0 * (SLATS_PER_HALF * SLAT_W + (SLATS_PER_HALF - 1) * SLAT_GAP)
# CENTER_GAP ≈ 0.044 m

# Legs
LEG_TOP_Z = TOP_Z - 0.055
LEG_LENGTH = 0.455
LEG_RADIUS = 0.017
LEG_X = HALF_X - 0.070
LEG_Y = TABLE_Y / 2.0 - 0.060
LOWER_BRACE_Z = -0.255
CONTACT_POST_RADIUS = 0.0065

# Shortened brace reach (to centre instead of crossing full width)
BRACE_DX = LEG_X - CENTER_GAP / 2.0

LEG_POSITIONS = [
    (-LEG_X, -LEG_Y),  # 0: left-front
    (LEG_X, -LEG_Y),   # 1: right-front
    (LEG_X, LEG_Y),    # 2: right-rear
    (-LEG_X, LEG_Y),   # 3: left-rear
]

LEG_HALF = ["left", "right", "right", "left"]

# Brace specs: (name, anchor_x, anchor_y, dx, dz, axis_sign, half)
BRACE_SPECS = [
    ("brace_0", -LEG_X, -LEG_Y - 0.070, BRACE_DX, -0.245, 1.0, "left"),
    ("brace_1", LEG_X, -LEG_Y - 0.045, -BRACE_DX, -0.245, -1.0, "right"),
    ("brace_2", LEG_X, LEG_Y + 0.070, -BRACE_DX, -0.245, -1.0, "right"),
    ("brace_3", -LEG_X, LEG_Y + 0.045, BRACE_DX, -0.245, 1.0, "left"),
]


# ── Mesh helpers ───────────────────────────────────────────────────
def _slat_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(SLAT_W, TABLE_Y, radius=0.010, corner_segments=8),
            SLAT_THICKNESS,
            center=True,
        ),
        "rounded_aluminum_slat",
    )


def _capsule_x_mesh(name: str, radius: float, mid_length: float):
    geom = CapsuleGeometry(radius=radius, length=mid_length, radial_segments=18)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _brace_mesh(name: str, dx: float, dz: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            [(0.0, 0.0, 0.0), (dx, 0.0, dz)],
            radius=0.0085,
            samples_per_segment=8,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _half_slat_xs(sign: int) -> list[float]:
    """Slat centre-X positions for one half (sign=-1 left, +1 right)."""
    return [
        sign * (CENTER_GAP / 2.0 + SLAT_W / 2.0 + k * (SLAT_W + SLAT_GAP))
        for k in range(SLATS_PER_HALF)
    ]


# ── Half-tabletop builder ──────────────────────────────────────────
def _build_half_tabletop(
    part,
    sign: int,
    z_delta: float,
    *,
    slat_mesh,
    end_cap_mesh,
    aluminum_mat,
    dark_aluminum,
    black_plastic,
    screw_metal,
    leg_indices,
    brace_indices,
    add_hinge_barrel: bool = False,
):
    """Build one butterfly half.

    *sign*: -1 left (negative X), +1 right (positive X).
    *z_delta*: 0.0 for the root half (world coords), -TOP_Z for the child
    half whose part frame sits at the fold-joint origin.
    """

    def z(wz: float) -> float:
        return wz + z_delta

    # ── Slats ──
    slat_xs = _half_slat_xs(sign)
    for k, sx in enumerate(slat_xs):
        part.visual(
            slat_mesh,
            origin=Origin(xyz=(sx, 0.0, z(TOP_Z))),
            material=aluminum_mat,
            name=f"slat_{k}",
        )
        for ys, suffix in [(-1, "front"), (1, "rear")]:
            part.visual(
                end_cap_mesh,
                origin=Origin(
                    xyz=(sx, ys * (TABLE_Y / 2.0 + 0.012), z(TOP_Z - 0.003))
                ),
                material=black_plastic,
                name=f"{suffix}_cap_{k}",
            )

    # ── Dark gap strips between slats ──
    for k in range(1, SLATS_PER_HALF):
        prev = CENTER_GAP / 2.0 + SLAT_W / 2.0 + (k - 1) * (SLAT_W + SLAT_GAP)
        curr = CENTER_GAP / 2.0 + SLAT_W / 2.0 + k * (SLAT_W + SLAT_GAP)
        gc = sign * (prev + curr) / 2.0
        part.visual(
            Box((SLAT_GAP * 0.65, TABLE_Y + 0.012, 0.026)),
            origin=Origin(xyz=(gc, 0.0, z(TOP_Z + 0.0005))),
            material=dark_aluminum,
            name=f"slat_gap_{k}",
        )

    # ── Front / rear rails (half width) ──
    half_rail_w = HALF_X + 0.015 - CENTER_GAP / 2.0
    rail_cx = sign * (HALF_X + 0.015 + CENTER_GAP / 2.0) / 2.0
    for yv, nm in [
        (-TABLE_Y / 2.0 - 0.016, "front_rail"),
        (TABLE_Y / 2.0 + 0.016, "rear_rail"),
    ]:
        part.visual(
            Box((half_rail_w, 0.030, 0.034)),
            origin=Origin(xyz=(rail_cx, yv, z(TOP_Z - 0.012))),
            material=black_plastic,
            name=nm,
        )

    # ── Outer side rail ──
    part.visual(
        Box((0.030, TABLE_Y + 0.026, 0.034)),
        origin=Origin(xyz=(sign * (HALF_X + 0.016), 0.0, z(TOP_Z - 0.012))),
        material=black_plastic,
        name="side_rail",
    )

    # ── Corner connectors (outer-front, outer-rear) ──
    for ys, suf in [(-1, "front"), (1, "rear")]:
        part.visual(
            Sphere(radius=0.030),
            origin=Origin(
                xyz=(
                    sign * (HALF_X + 0.004),
                    ys * (TABLE_Y / 2.0 + 0.004),
                    z(TOP_Z - 0.012),
                )
            ),
            material=black_plastic,
            name=f"corner_connector_{suf}",
        )

    # ── Hinge connection at centre edge ──
    if add_hinge_barrel:
        # Piano-hinge barrel on the left half
        part.visual(
            Cylinder(radius=0.014, length=TABLE_Y - 0.08),
            origin=Origin(
                xyz=(0.0, 0.0, z(TOP_Z - 0.012)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=screw_metal,
            name="hinge_barrel",
        )
        # Mount plate bridges barrel to the innermost slat and rails
        part.visual(
            Box((0.028, TABLE_Y - 0.10, 0.020)),
            origin=Origin(xyz=(-0.020, 0.0, z(TOP_Z - 0.012))),
            material=black_plastic,
            name="hinge_mount",
        )
    else:
        # Hinge leaf on the right half wraps around the left barrel
        # and overlaps the innermost right slat, bridging the centre gap.
        part.visual(
            Box((0.028, TABLE_Y - 0.10, 0.020)),
            origin=Origin(xyz=(0.020, 0.0, z(TOP_Z - 0.012))),
            material=screw_metal,
            name="hinge_leaf",
        )

    # ── Hinge sockets for legs on this half ──
    for i in leg_indices:
        lx, ly = LEG_POSITIONS[i]
        part.visual(
            Box((0.056, 0.052, 0.070)),
            origin=Origin(xyz=(lx, ly, z(LEG_TOP_Z + 0.015))),
            material=black_plastic,
            name=f"hinge_socket_{i}",
        )

    # ── Brace anchors for braces on this half ──
    for bi in brace_indices:
        _, bx, by, _, _, _, _ = BRACE_SPECS[bi]
        part.visual(
            Box((0.038, 0.036, 0.065)),
            origin=Origin(xyz=(bx, by, z(LEG_TOP_Z + 0.010))),
            material=black_plastic,
            name=f"brace_anchor_{bi}",
        )

    # ── Underside cross-rails ──
    cr_w = half_rail_w - 0.060
    for yv, suf in [(-0.16, "front"), (0.16, "rear")]:
        part.visual(
            Box((cr_w, 0.022, 0.030)),
            origin=Origin(xyz=(rail_cx, yv, z(TOP_Z - 0.018))),
            material=black_plastic,
            name=f"underside_crossrail_{suf}",
        )

    # ── Front-rail oval inserts and screw heads ──
    for k, sx in enumerate(slat_xs):
        if k % 2 == 0:
            part.visual(
                Box((0.042, 0.006, 0.014)),
                origin=Origin(
                    xyz=(sx, -TABLE_Y / 2.0 - 0.033, z(TOP_Z - 0.006))
                ),
                material=black_plastic,
                name=f"front_oval_insert_{k}",
            )
        for dx_s in (-0.020, 0.020):
            part.visual(
                Cylinder(radius=0.006, length=0.003),
                origin=Origin(
                    xyz=(
                        sx + dx_s,
                        -TABLE_Y / 2.0 - 0.031,
                        z(TOP_Z - 0.003),
                    ),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=screw_metal,
                name=f"front_screw_{k}_{0 if dx_s < 0 else 1}",
            )


# ═══════════════════════════════════════════════════════════════════
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="butterfly_folding_camp_table",
        meta={
            "run_notes": (
                "Butterfly / suitcase folding camp table variant. "
                "The tabletop splits into two halves (tabletop_left, "
                "tabletop_right) connected by a central revolute fold "
                "hinge along Y. Each half retains the slatted aluminum "
                "deck, perimeter rails, hinge sockets, and brace anchors. "
                "Legs fold against their respective half and the two "
                "halves close book-style. Two-tone half panels visually "
                "distinguish the folding halves."
            )
        },
    )

    # ── Materials ──
    aluminum_left = model.material("aluminum_left", rgba=(0.46, 0.48, 0.50, 1.0))
    aluminum_right = model.material("aluminum_right", rgba=(0.58, 0.57, 0.53, 1.0))
    dark_aluminum = model.material("dark_gap_shadow", rgba=(0.025, 0.026, 0.026, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.02, 0.019, 0.018, 1.0))
    screw_metal = model.material("screw_heads", rgba=(0.83, 0.82, 0.78, 1.0))

    slat_mesh = _slat_mesh()
    end_cap_mesh = _capsule_x_mesh(
        "rounded_slat_end_cap",
        radius=0.012,
        mid_length=max(0.001, SLAT_W - 0.024),
    )

    # ═══════════════════════════════════════════════════════════════
    # TABLETOP LEFT  (root part – world coordinates)
    # ═══════════════════════════════════════════════════════════════
    tabletop_left = model.part("tabletop_left")
    _build_half_tabletop(
        tabletop_left,
        sign=-1,
        z_delta=0.0,
        slat_mesh=slat_mesh,
        end_cap_mesh=end_cap_mesh,
        aluminum_mat=aluminum_left,
        dark_aluminum=dark_aluminum,
        black_plastic=black_plastic,
        screw_metal=screw_metal,
        leg_indices=[0, 3],
        brace_indices=[0, 3],
        add_hinge_barrel=True,
    )

    # ═══════════════════════════════════════════════════════════════
    # TABLETOP RIGHT  (child of fold hinge – local coordinates)
    # Part frame coincides with fold joint at (0, 0, TOP_Z) when q=0.
    # ═══════════════════════════════════════════════════════════════
    tabletop_right = model.part("tabletop_right")
    _build_half_tabletop(
        tabletop_right,
        sign=+1,
        z_delta=-TOP_Z,
        slat_mesh=slat_mesh,
        end_cap_mesh=end_cap_mesh,
        aluminum_mat=aluminum_right,
        dark_aluminum=dark_aluminum,
        black_plastic=black_plastic,
        screw_metal=screw_metal,
        leg_indices=[1, 2],
        brace_indices=[1, 2],
        add_hinge_barrel=False,
    )

    # ═══════════════════════════════════════════════════════════════
    # CENTRAL FOLD HINGE  (book-style, axis along Y)
    # Positive q raises the right half upward and over.
    # ═══════════════════════════════════════════════════════════════
    model.articulation(
        "tabletop_left_to_right",
        ArticulationType.REVOLUTE,
        parent=tabletop_left,
        child=tabletop_right,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0, velocity=1.5, lower=0.0, upper=math.pi
        ),
    )

    # ═══════════════════════════════════════════════════════════════
    # LEGS  (same geometry, parented to the correct half)
    # ═══════════════════════════════════════════════════════════════
    for i in range(4):
        x, y = LEG_POSITIONS[i]
        parent = tabletop_left if LEG_HALF[i] == "left" else tabletop_right
        z_off = 0.0 if LEG_HALF[i] == "left" else -TOP_Z
        leg_alu = aluminum_left if LEG_HALF[i] == "left" else aluminum_right

        leg = model.part(f"leg_{i}")
        leg.visual(
            Cylinder(radius=0.011, length=0.058),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="hinge_pin",
        )
        leg.visual(
            Cylinder(radius=LEG_RADIUS, length=LEG_LENGTH - 0.022),
            origin=Origin(xyz=(0.0, 0.0, -(LEG_LENGTH - 0.022) / 2.0 - 0.003)),
            material=leg_alu,
            name="straight_tube",
        )
        leg.visual(
            Cylinder(radius=0.023, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
            material=black_plastic,
            name="upper_collar",
        )
        leg.visual(
            Cylinder(radius=0.020, length=0.036),
            origin=Origin(xyz=(0.0, 0.0, -LEG_LENGTH + 0.003)),
            material=rubber,
            name="rubber_foot",
        )
        # Brace contact post (vestigial hardware detail)
        brace_y_off = 0.070 if i in (1, 3) else 0.045
        outer_ys = 1.0 if y > 0.0 else -1.0
        post_len = brace_y_off - LEG_RADIUS - 0.012
        leg.visual(
            Cylinder(radius=CONTACT_POST_RADIUS, length=post_len),
            origin=Origin(
                xyz=(
                    0.0,
                    outer_ys * (LEG_RADIUS + post_len / 2.0),
                    LOWER_BRACE_Z,
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=screw_metal,
            name="brace_contact_post",
        )

        axis_y = 1.0 if x > 0.0 else -1.0
        model.articulation(
            f"tabletop_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=leg,
            origin=Origin(xyz=(x, y, LEG_TOP_Z + z_off)),
            axis=(0.0, axis_y, 0.0),
            motion_limits=MotionLimits(
                effort=16.0, velocity=2.2, lower=0.0, upper=1.45
            ),
        )

    # ═══════════════════════════════════════════════════════════════
    # BRACES  (shortened – each stays within its own half)
    # ═══════════════════════════════════════════════════════════════
    for idx, (name, ax, ay, dx, dz, axis_sign, half) in enumerate(BRACE_SPECS):
        parent = tabletop_left if half == "left" else tabletop_right
        z_off = 0.0 if half == "left" else -TOP_Z
        brace_alu = aluminum_left if half == "left" else aluminum_right

        brace = model.part(name)
        brace.visual(
            _brace_mesh(f"{name}_tube", dx, dz),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=brace_alu,
            name="diagonal_tube",
        )
        brace.visual(
            Cylinder(radius=0.014, length=0.024),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="upper_eye",
        )
        brace.visual(
            Cylinder(radius=0.012, length=0.024),
            origin=Origin(xyz=(dx, 0.0, dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="lower_eye",
        )
        brace.visual(
            Cylinder(radius=0.009, length=0.012),
            origin=Origin(
                xyz=(dx / 2.0, 0.0, dz / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=screw_metal,
            name="center_rivet",
        )

        model.articulation(
            f"tabletop_to_{name}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=brace,
            origin=Origin(xyz=(ax, ay, LEG_TOP_Z - 0.010 + z_off)),
            axis=(0.0, axis_sign, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-0.25, upper=1.05
            ),
        )

    return model


# ═══════════════════════════════════════════════════════════════════
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tabletop_left = object_model.get_part("tabletop_left")
    tabletop_right = object_model.get_part("tabletop_right")
    fold_joint = object_model.get_articulation("tabletop_left_to_right")

    # ── Hinge barrel / leaf inter-part overlap ──
    ctx.allow_overlap(
        tabletop_left,
        tabletop_right,
        elem_a="hinge_barrel",
        elem_b="hinge_leaf",
        reason="The hinge leaf wraps around the barrel to form the pivot connection between halves.",
    )
    ctx.expect_overlap(
        tabletop_right,
        tabletop_left,
        axes="xy",
        elem_a="hinge_leaf",
        elem_b="hinge_barrel",
        min_overlap=0.005,
        name="hinge_leaf_engages_barrel",
    )

    # ── Butterfly structure: each half has its own slats ──
    ctx.check(
        "tabletop_left_has_slats",
        all(
            tabletop_left.get_visual(f"slat_{k}") is not None
            for k in range(SLATS_PER_HALF)
        ),
        "Expected four slats on the left half.",
    )
    ctx.check(
        "tabletop_right_has_slats",
        all(
            tabletop_right.get_visual(f"slat_{k}") is not None
            for k in range(SLATS_PER_HALF)
        ),
        "Expected four slats on the right half.",
    )

    # ── Fold hinge raises right half (book-style close) ──
    rest_aabb = ctx.part_world_aabb(tabletop_right)
    with ctx.pose({fold_joint: 1.2}):
        folded_aabb = ctx.part_world_aabb(tabletop_right)
    if rest_aabb is not None and folded_aabb is not None:
        rest_max_z = float(rest_aabb[1][2])
        fold_max_z = float(folded_aabb[1][2])
        ctx.check(
            "tabletop_left_to_right_fold_raises_right_half",
            fold_max_z > rest_max_z + 0.10,
            f"rest_max_z={rest_max_z:.4f}, folded_max_z={fold_max_z:.4f}",
        )
    else:
        ctx.fail("fold_aabb_available", "Could not compute tabletop_right AABB.")

    # ── Combined open-table width is realistic ──
    left_aabb = ctx.part_world_aabb(tabletop_left)
    right_aabb = ctx.part_world_aabb(tabletop_right)
    if left_aabb is not None and right_aabb is not None:
        total_w = float(right_aabb[1][0] - left_aabb[0][0])
        ctx.check(
            "open_table_width_realistic",
            0.76 <= total_w <= 0.92,
            f"total_width={total_w:.4f}",
        )
    else:
        ctx.fail("half_aabb_available", "Could not compute half-tabletop AABBs.")

    # ── Leg hinge-pin / socket overlaps ──
    for i in range(4):
        parent_part = tabletop_left if LEG_HALF[i] == "left" else tabletop_right
        leg = object_model.get_part(f"leg_{i}")

        ctx.allow_overlap(
            parent_part,
            leg,
            elem_a=f"hinge_socket_{i}",
            elem_b="hinge_pin",
            reason="Folding-leg hinge pin is captured inside the black socket.",
        )
        ctx.expect_overlap(
            leg,
            parent_part,
            axes="xy",
            elem_a="hinge_pin",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.018,
            name=f"leg_{i}_pin_seated_in_socket",
        )
        ctx.allow_overlap(
            parent_part,
            leg,
            elem_a=f"hinge_socket_{i}",
            elem_b="straight_tube",
            reason="Leg tube has a short hidden insertion into the moulded socket.",
        )
        ctx.expect_overlap(
            leg,
            parent_part,
            axes="xy",
            elem_a="straight_tube",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.020,
            name=f"leg_{i}_tube_inserted_in_socket",
        )

    # ── Brace anchor / upper-eye overlaps ──
    for i in range(4):
        half = BRACE_SPECS[i][6]
        parent_part = tabletop_left if half == "left" else tabletop_right
        brace = object_model.get_part(f"brace_{i}")

        ctx.allow_overlap(
            parent_part,
            brace,
            elem_a=f"brace_anchor_{i}",
            elem_b="upper_eye",
            reason="Diagonal brace upper eye is a captured pivot seated in its anchor.",
        )
        ctx.expect_overlap(
            brace,
            parent_part,
            axes="xy",
            elem_a="upper_eye",
            elem_b=f"brace_anchor_{i}",
            min_overlap=0.014,
            name=f"brace_{i}_upper_eye_seated",
        )
        ctx.allow_overlap(
            parent_part,
            brace,
            elem_a=f"brace_anchor_{i}",
            elem_b="diagonal_tube",
            reason="Brace tube begins inside the hinged anchor, representing the captured rod end.",
        )
        ctx.expect_overlap(
            brace,
            parent_part,
            axes="xy",
            elem_a="diagonal_tube",
            elem_b=f"brace_anchor_{i}",
            min_overlap=0.010,
            name=f"brace_{i}_tube_enters_anchor",
        )

    # ── Leg fold: leg_0 reaches ground and folds inward ──
    leg0 = object_model.get_part("leg_0")
    leg0_joint = object_model.get_articulation("tabletop_to_leg_0")
    rest_leg0_aabb = ctx.part_world_aabb(leg0)
    if rest_leg0_aabb is not None:
        rmin, rmax = rest_leg0_aabb
        ctx.check(
            "open_leg_reaches_ground",
            float(rmin[2]) < 0.010 and float(rmax[2]) > LEG_TOP_Z - 0.020,
            details=str(rest_leg0_aabb),
        )
    with ctx.pose({leg0_joint: 1.20}):
        folded_aabb = ctx.part_world_aabb(leg0)
    if rest_leg0_aabb is not None and folded_aabb is not None:
        rmin, rmax = rest_leg0_aabb
        fmin, fmax = folded_aabb
        ctx.check(
            "leg_folds_inward_and_up",
            float(fmin[2]) > float(rmin[2]) + 0.12
            and abs((float(fmin[0]) + float(fmax[0])) / 2.0)
            < abs((float(rmin[0]) + float(rmax[0])) / 2.0),
            details=f"rest={rest_leg0_aabb}, folded={folded_aabb}",
        )

    # ── Brace hinge motion ──
    brace0 = object_model.get_part("brace_0")
    brace0_joint = object_model.get_articulation("tabletop_to_brace_0")
    rest_brace_aabb = ctx.part_world_aabb(brace0)
    with ctx.pose({brace0_joint: 0.70}):
        moved_brace_aabb = ctx.part_world_aabb(brace0)
    if rest_brace_aabb is not None and moved_brace_aabb is not None:
        rmin, rmax = rest_brace_aabb
        mmin, mmax = moved_brace_aabb
        ctx.check(
            "diagonal_brace_hinges",
            abs(float(mmax[2] - mmin[2]) - float(rmax[2] - rmin[2])) > 0.020,
            details=f"rest={rest_brace_aabb}, moved={moved_brace_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
