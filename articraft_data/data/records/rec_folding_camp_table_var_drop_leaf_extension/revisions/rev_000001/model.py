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


TABLE_X = 0.80
TABLE_Y = 0.54
TOP_Z = 0.51
SLAT_THICKNESS = 0.024
SLAT_COUNT = 9
SLAT_GAP = 0.006
SLAT_W = (TABLE_X - (SLAT_COUNT - 1) * SLAT_GAP) / SLAT_COUNT
LEG_TOP_Z = TOP_Z - 0.055
LEG_LENGTH = 0.455
LEG_RADIUS = 0.017
LEG_X = TABLE_X / 2.0 - 0.070
LEG_Y = TABLE_Y / 2.0 - 0.060
LOWER_BRACE_Z = -0.255
CONTACT_POST_RADIUS = 0.0065

# Drop-leaf extension along the rear (+Y) long edge.
DROP_LEAF_X = TABLE_X - 0.08  # slightly narrower than main table to clear corners
DROP_LEAF_DEPTH = 0.22
DROP_LEAF_THICKNESS = SLAT_THICKNESS
DROP_LEAF_SLAT_COUNT = 4
DROP_LEAF_SLAT_GAP = 0.006
DROP_LEAF_SLAT_D = (
    DROP_LEAF_DEPTH - (DROP_LEAF_SLAT_COUNT - 1) * DROP_LEAF_SLAT_GAP
) / DROP_LEAF_SLAT_COUNT
DROP_LEAF_SLAT_X = DROP_LEAF_X / 1.0  # each slat spans the full leaf width
HINGE_Y = TABLE_Y / 2.0 + 0.034  # hinge line just outside the rear rail
HINGE_BARREL_RADIUS = 0.010
HINGE_BARREL_X_POSITIONS = [-0.28, -0.09, 0.09, 0.28]
LEAF_KNUCKLE_X_POSITIONS = [-0.185, 0.0, 0.185]


def _slat_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(SLAT_W, TABLE_Y, radius=0.010, corner_segments=8),
            SLAT_THICKNESS,
            center=True,
        ),
        "rounded_aluminum_slat",
    )


def _drop_leaf_slat_mesh():
    """One long narrow slat for the drop-leaf extension (runs in X, narrow in Y)."""
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(
                DROP_LEAF_SLAT_X, DROP_LEAF_SLAT_D, radius=0.005, corner_segments=6
            ),
            DROP_LEAF_THICKNESS,
            center=True,
        ),
        "drop_leaf_aluminum_slat",
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="compact_folding_camp_table",
        meta={
            "run_notes": (
                "Primary image/category agree: a compact folding camp table. "
                "The carry bag in the reference is omitted so the single asset "
                "focuses on the articulated table."
            )
        },
    )

    mottled_aluminum = model.material("mottled_aluminum", rgba=(0.54, 0.55, 0.54, 1.0))
    dark_aluminum = model.material("dark_gap_shadow", rgba=(0.025, 0.026, 0.026, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.02, 0.019, 0.018, 1.0))
    screw_metal = model.material("screw_heads", rgba=(0.83, 0.82, 0.78, 1.0))

    slat_mesh = _slat_mesh()
    end_cap_mesh = _capsule_x_mesh(
        "rounded_slat_end_cap", radius=0.012, mid_length=max(0.001, SLAT_W - 0.024)
    )

    tabletop = model.part("tabletop")

    # A roll-up style segmented aluminum tabletop: separate gray slats are tied
    # together by black end rails and underside rails so the assembly reads as
    # one supported manufactured top rather than loose boards.
    x0 = -TABLE_X / 2.0 + SLAT_W / 2.0
    for i in range(SLAT_COUNT):
        x = x0 + i * (SLAT_W + SLAT_GAP)
        tabletop.visual(
            slat_mesh,
            origin=Origin(xyz=(x, 0.0, TOP_Z)),
            material=mottled_aluminum,
            name=f"slat_{i}",
        )
        for y, suffix in [(-1, "front"), (1, "rear")]:
            tabletop.visual(
                end_cap_mesh,
                origin=Origin(xyz=(x, y * (TABLE_Y / 2.0 + 0.012), TOP_Z - 0.003)),
                material=black_plastic,
                name=f"{suffix}_cap_{i}",
            )

    for i in range(1, SLAT_COUNT):
        x = -TABLE_X / 2.0 + i * SLAT_W + (i - 0.5) * SLAT_GAP
        tabletop.visual(
            Box((SLAT_GAP * 0.65, TABLE_Y + 0.012, 0.026)),
            origin=Origin(xyz=(x, 0.0, TOP_Z + 0.0005)),
            material=dark_aluminum,
            name=f"slat_gap_{i}",
        )

    # Black perimeter and underside structure, matching the visible rounded
    # plastic connectors and front/rear rails in the reference image.
    for y, name in [(-TABLE_Y / 2.0 - 0.016, "front_rail"), (TABLE_Y / 2.0 + 0.016, "rear_rail")]:
        tabletop.visual(
            Box((TABLE_X + 0.030, 0.030, 0.034)),
            origin=Origin(xyz=(0.0, y, TOP_Z - 0.012)),
            material=black_plastic,
            name=name,
        )
    for x, name in [
        (-TABLE_X / 2.0 - 0.016, "side_rail_0"),
        (TABLE_X / 2.0 + 0.016, "side_rail_1"),
    ]:
        tabletop.visual(
            Box((0.030, TABLE_Y + 0.026, 0.034)),
            origin=Origin(xyz=(x, 0.0, TOP_Z - 0.012)),
            material=black_plastic,
            name=name,
        )
    for y in (-0.16, 0.16):
        tabletop.visual(
            Box((TABLE_X - 0.070, 0.022, 0.030)),
            origin=Origin(xyz=(0.0, y, TOP_Z - 0.018)),
            material=black_plastic,
            name=f"underside_crossrail_{'front' if y < 0 else 'rear'}",
        )
    for idx, (x, y) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        tabletop.visual(
            Sphere(radius=0.030),
            origin=Origin(
                xyz=(x * (TABLE_X / 2.0 + 0.004), y * (TABLE_Y / 2.0 + 0.004), TOP_Z - 0.012)
            ),
            material=black_plastic,
            name=f"corner_connector_{idx}",
        )

    # Hinge sockets for folding legs and braces.  The small intentional pin
    # embeddings are proven and allowed in run_tests().
    leg_positions = [
        (-LEG_X, -LEG_Y),
        (LEG_X, -LEG_Y),
        (LEG_X, LEG_Y),
        (-LEG_X, LEG_Y),
    ]
    for i, (x, y) in enumerate(leg_positions):
        tabletop.visual(
            Box((0.056, 0.052, 0.070)),
            origin=Origin(xyz=(x, y, LEG_TOP_Z + 0.015)),
            material=black_plastic,
            name=f"hinge_socket_{i}",
        )

    # Small screw heads and oval plastic inserts along the front rail.
    for i, x in enumerate([x0 + k * (SLAT_W + SLAT_GAP) for k in range(SLAT_COUNT)]):
        if i % 2 == 0:
            tabletop.visual(
                Box((0.042, 0.006, 0.014)),
                origin=Origin(xyz=(x, -TABLE_Y / 2.0 - 0.033, TOP_Z - 0.006)),
                material=black_plastic,
                name=f"front_oval_insert_{i}",
            )
        for dx in (-0.020, 0.020):
            tabletop.visual(
                Cylinder(radius=0.006, length=0.003),
                origin=Origin(
                    xyz=(x + dx, -TABLE_Y / 2.0 - 0.031, TOP_Z - 0.003),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=screw_metal,
                name=f"front_screw_{i}_{0 if dx < 0 else 1}",
            )

    for i, (x, y) in enumerate(leg_positions):
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
            material=mottled_aluminum,
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
        brace_y_offset = 0.070 if i in (1, 3) else 0.045
        outer_y_sign = 1.0 if y > 0.0 else -1.0
        post_length = brace_y_offset - LEG_RADIUS - 0.012
        leg.visual(
            Cylinder(radius=CONTACT_POST_RADIUS, length=post_length),
            origin=Origin(
                xyz=(
                    0.0,
                    outer_y_sign * (LEG_RADIUS + post_length / 2.0),
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
            parent=tabletop,
            child=leg,
            origin=Origin(xyz=(x, y, LEG_TOP_Z)),
            axis=(0.0, axis_y, 0.0),
            motion_limits=MotionLimits(effort=16.0, velocity=2.2, lower=0.0, upper=1.45),
        )

    # Four independent hinged diagonal braces form the visible X-frame on the
    # front and rear sides.  They are separated slightly in Y so the crossed
    # rods look like real bypassing members with a central rivet instead of
    # occupying the same volume.
    brace_specs = [
        ("brace_0", -LEG_X, -LEG_Y - 0.070, LEG_X * 2.0, -0.245, 1.0),
        ("brace_1", LEG_X, -LEG_Y - 0.045, -LEG_X * 2.0, -0.245, -1.0),
        ("brace_2", LEG_X, LEG_Y + 0.070, -LEG_X * 2.0, -0.245, -1.0),
        ("brace_3", -LEG_X, LEG_Y + 0.045, LEG_X * 2.0, -0.245, 1.0),
    ]
    for index, (name, x, y, dx, dz, axis_sign) in enumerate(brace_specs):
        tabletop.visual(
            Box((0.038, 0.030, 0.065)),
            origin=Origin(xyz=(x, y, LEG_TOP_Z + 0.010)),
            material=black_plastic,
            name=f"brace_anchor_{index}",
        )
        brace = model.part(name)
        brace.visual(
            _brace_mesh(f"{name}_tube", dx, dz),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=mottled_aluminum,
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
            origin=Origin(xyz=(dx / 2.0, 0.0, dz / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="center_rivet",
        )
        model.articulation(
            f"tabletop_to_{name}",
            ArticulationType.REVOLUTE,
            parent=tabletop,
            child=brace,
            origin=Origin(xyz=(x, y, LEG_TOP_Z - 0.010)),
            axis=(0.0, axis_sign, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-0.25, upper=1.05),
        )

    # ── Drop-leaf extension along the rear (+Y) long edge ──────────────────
    # Hinge barrels on the tabletop side, interleaved with the leaf knuckles.
    leaf_slat_mesh = _drop_leaf_slat_mesh()
    for idx, bx in enumerate(HINGE_BARREL_X_POSITIONS):
        tabletop.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=0.050),
            origin=Origin(
                xyz=(bx, HINGE_Y, TOP_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=screw_metal,
            name=f"leaf_hinge_barrel_{idx}",
        )

    # Two fold-out support brackets under the tabletop that extend rearward
    # past the hinge line to carry the deployed leaf.
    for bx in (-0.22, 0.22):
        tabletop.visual(
            Box((0.022, 0.20, 0.014)),
            origin=Origin(
                xyz=(bx, HINGE_Y + 0.080, TOP_Z - 0.030),
            ),
            material=black_plastic,
            name=f"leaf_support_bracket_{'left' if bx < 0 else 'right'}",
        )

    # Drop-leaf part: origin at the hinge line so that q=0 is deployed (level)
    # and positive q folds the leaf downward.
    drop_leaf = model.part("drop_leaf")

    # Interleaved hinge knuckles on the leaf side (offset from tabletop barrels).
    for idx, kx in enumerate(LEAF_KNUCKLE_X_POSITIONS):
        drop_leaf.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS - 0.001, length=0.040),
            origin=Origin(
                xyz=(kx, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=screw_metal,
            name=f"leaf_knuckle_{idx}",
        )

    # Hinge strip connecting the knuckles to the leaf slats.
    drop_leaf.visual(
        Box((DROP_LEAF_X + 0.02, 0.024, DROP_LEAF_THICKNESS + 0.004)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=black_plastic,
        name="leaf_hinge_strip",
    )

    # Leaf slats extending outward in +Y from the hinge line.
    slat_y_start = 0.030  # first slat starts past the hinge hardware
    for i in range(DROP_LEAF_SLAT_COUNT):
        y = slat_y_start + i * (DROP_LEAF_SLAT_D + DROP_LEAF_SLAT_GAP) + DROP_LEAF_SLAT_D / 2.0
        drop_leaf.visual(
            leaf_slat_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=mottled_aluminum,
            name=f"leaf_slat_{i}",
        )

    # Rear edge rail at the far end of the leaf (Y direction).
    far_y = slat_y_start + DROP_LEAF_SLAT_COUNT * DROP_LEAF_SLAT_D + (
        DROP_LEAF_SLAT_COUNT - 1
    ) * DROP_LEAF_SLAT_GAP
    drop_leaf.visual(
        Box((DROP_LEAF_X + 0.030, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, far_y + 0.013, -0.003)),
        material=black_plastic,
        name="leaf_outer_rail",
    )

    # Side rails at the X ends of the leaf.
    for sx, sname in [(-DROP_LEAF_X / 2.0 - 0.013, "leaf_side_rail_left"),
                       (DROP_LEAF_X / 2.0 + 0.013, "leaf_side_rail_right")]:
        drop_leaf.visual(
            Box((0.026, far_y + 0.010, 0.030)),
            origin=Origin(xyz=(sx, far_y / 2.0, -0.003)),
            material=black_plastic,
            name=sname,
        )

    # Rounded corner caps at the two outer corners of the leaf.
    for cx in (-1, 1):
        drop_leaf.visual(
            Sphere(radius=0.020),
            origin=Origin(
                xyz=(cx * (DROP_LEAF_X / 2.0 + 0.003), far_y + 0.008, -0.003),
            ),
            material=black_plastic,
            name=f"leaf_corner_cap_{0 if cx < 0 else 1}",
        )

    # Articulation: hinge along the rear edge.
    # axis = (-1, 0, 0) so that positive q folds the leaf downward (toward -Z).
    # q=0 → leaf deployed level; q≈π/2 → leaf hanging down for packing.
    model.articulation(
        "tabletop_to_drop_leaf",
        ArticulationType.REVOLUTE,
        parent=tabletop,
        child=drop_leaf,
        origin=Origin(xyz=(0.0, HINGE_Y, TOP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=1.65
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tabletop = object_model.get_part("tabletop")

    for i in range(4):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            tabletop,
            leg,
            elem_a=f"hinge_socket_{i}",
            elem_b="hinge_pin",
            reason="The visible folding-leg hinge pin is intentionally captured inside the black socket.",
        )
        ctx.expect_overlap(
            leg,
            tabletop,
            axes="xy",
            elem_a="hinge_pin",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.018,
            name=f"leg_{i}_pin_seated_in_socket",
        )
        ctx.allow_overlap(
            tabletop,
            leg,
            elem_a=f"hinge_socket_{i}",
            elem_b="straight_tube",
            reason="The straight leg tube has a short hidden insertion into the molded hinge socket.",
        )
        ctx.expect_overlap(
            leg,
            tabletop,
            axes="xy",
            elem_a="straight_tube",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.020,
            name=f"leg_{i}_tube_inserted_in_socket",
        )

    for i in range(4):
        brace = object_model.get_part(f"brace_{i}")
        ctx.allow_overlap(
            tabletop,
            brace,
            elem_a=f"brace_anchor_{i}",
            elem_b="upper_eye",
            reason="The diagonal brace upper eye is a captured pivot seated in its tabletop anchor.",
        )
        ctx.expect_overlap(
            brace,
            tabletop,
            axes="xy",
            elem_a="upper_eye",
            elem_b=f"brace_anchor_{i}",
            min_overlap=0.014,
            name=f"brace_{i}_upper_eye_seated",
        )
        ctx.allow_overlap(
            tabletop,
            brace,
            elem_a=f"brace_anchor_{i}",
            elem_b="diagonal_tube",
            reason="The brace tube begins inside the hinged anchor, representing the captured rod end.",
        )
        ctx.expect_overlap(
            brace,
            tabletop,
            axes="xy",
            elem_a="diagonal_tube",
            elem_b=f"brace_anchor_{i}",
            min_overlap=0.010,
            name=f"brace_{i}_tube_enters_anchor",
        )

    for leg_i, brace_i in [(1, 0), (0, 1), (3, 2), (2, 3)]:
        leg = object_model.get_part(f"leg_{leg_i}")
        brace = object_model.get_part(f"brace_{brace_i}")
        ctx.expect_contact(
            leg,
            brace,
            elem_a="brace_contact_post",
            elem_b="lower_eye",
            contact_tol=1e-5,
            name=f"leg_{leg_i}_contact_post_touches_brace_{brace_i}_lower_eye",
        )

    ctx.check(
        "segmented_slat_tabletop",
        tabletop is not None
        and all(tabletop.get_visual(f"slat_{i}") is not None for i in range(SLAT_COUNT)),
        "Expected nine separate rounded tabletop slats.",
    )

    aabb = ctx.part_world_aabb(tabletop)
    if aabb is not None:
        min_pt, max_pt = aabb
        ctx.check(
            "tabletop_width_realistic",
            0.78 <= float(max_pt[0] - min_pt[0]) <= 0.90,
            details=str(aabb),
        )
        # Depth includes the main slat area plus rear hinge hardware for the drop leaf.
        ctx.check(
            "tabletop_depth_realistic",
            0.54 <= float(max_pt[1] - min_pt[1]) <= 0.85,
            details=str(aabb),
        )
    else:
        ctx.fail("tabletop_aabb_present", "Expected tabletop AABB.")

    leg0 = object_model.get_part("leg_0")
    leg0_joint = object_model.get_articulation("tabletop_to_leg_0")
    rest_leg0_aabb = ctx.part_world_aabb(leg0)
    if rest_leg0_aabb is not None:
        rest_min, rest_max = rest_leg0_aabb
        ctx.check(
            "open_leg_reaches_ground",
            float(rest_min[2]) < 0.010 and float(rest_max[2]) > LEG_TOP_Z - 0.020,
            details=str(rest_leg0_aabb),
        )
    with ctx.pose({leg0_joint: 1.20}):
        folded_aabb = ctx.part_world_aabb(leg0)
    if rest_leg0_aabb is not None and folded_aabb is not None:
        rest_min, rest_max = rest_leg0_aabb
        fold_min, fold_max = folded_aabb
        ctx.check(
            "leg_folds_inward_and_up",
            float(fold_min[2]) > float(rest_min[2]) + 0.12
            and abs((float(fold_min[0]) + float(fold_max[0])) / 2.0)
            < abs((float(rest_min[0]) + float(rest_max[0])) / 2.0),
            details=f"rest={rest_leg0_aabb}, folded={folded_aabb}",
        )

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

    # ── Drop-leaf tests ────────────────────────────────────────────────────
    drop_leaf = object_model.get_part("drop_leaf")
    leaf_joint = object_model.get_articulation("tabletop_to_drop_leaf")

    # Prove the drop_leaf part exists with its named slats.
    ctx.check(
        "drop_leaf_has_slats",
        drop_leaf is not None
        and all(
            drop_leaf.get_visual(f"leaf_slat_{i}") is not None
            for i in range(DROP_LEAF_SLAT_COUNT)
        ),
        f"Expected {DROP_LEAF_SLAT_COUNT} named leaf slats on drop_leaf part.",
    )

    # Prove the hinge strip and outer rail exist.
    ctx.check(
        "drop_leaf_has_rails",
        drop_leaf.get_visual("leaf_hinge_strip") is not None
        and drop_leaf.get_visual("leaf_outer_rail") is not None,
        "Expected leaf_hinge_strip and leaf_outer_rail on drop_leaf.",
    )

    # Allow the hinge knuckle / barrel overlap (captured pivot hardware).
    for idx in range(len(LEAF_KNUCKLE_X_POSITIONS)):
        ctx.allow_overlap(
            tabletop,
            drop_leaf,
            elem_a=f"leaf_hinge_barrel_{idx}",
            elem_b=f"leaf_knuckle_{idx}",
            reason="Interleaved hinge barrel and knuckle represent the captured pivot pin of the drop-leaf hinge.",
        )

    # Allow the hinge strip to contact/overlap all hinge barrels (seated
    # hinge plate on the tabletop underside).
    for idx in range(len(HINGE_BARREL_X_POSITIONS)):
        ctx.allow_overlap(
            tabletop,
            drop_leaf,
            elem_a=f"leaf_hinge_barrel_{idx}",
            elem_b="leaf_hinge_strip",
            reason="The leaf hinge strip is intentionally captured under the hinge barrels, representing a surface-mounted continuous hinge.",
        )

    # Allow the leaf knuckles to contact/overlap the rear rail (hinge hardware
    # sits just past the rail edge).
    for idx in range(len(LEAF_KNUCKLE_X_POSITIONS)):
        ctx.allow_overlap(
            tabletop,
            drop_leaf,
            elem_a="rear_rail",
            elem_b=f"leaf_knuckle_{idx}",
            reason="The leaf hinge knuckle sits just past the rear rail edge, representing exposed hinge hardware at the tabletop perimeter.",
        )

    # At q=0 the deployed leaf should be level with the main tabletop (same Z).
    deployed_aabb = ctx.part_world_aabb(drop_leaf)
    if deployed_aabb is not None:
        dep_min, dep_max = deployed_aabb
        ctx.check(
            "deployed_leaf_level_with_tabletop",
            abs(float(dep_max[2]) - (TOP_Z + SLAT_THICKNESS / 2.0)) < 0.015,
            details=f"leaf top z={float(dep_max[2]):.4f}, expected ~{TOP_Z + SLAT_THICKNESS / 2.0:.4f}",
        )
        # The leaf should extend in +Y past the rear rail.
        ctx.check(
            "deployed_leaf_extends_past_rear_rail",
            float(dep_max[1]) > TABLE_Y / 2.0 + 0.10,
            details=f"leaf max y={float(dep_max[1]):.4f}",
        )

    # Folding the leaf down: at q=1.2 the outer edge should drop below the tabletop.
    with ctx.pose({leaf_joint: 1.20}):
        folded_aabb = ctx.part_world_aabb(drop_leaf)
    if deployed_aabb is not None and folded_aabb is not None:
        dep_min, dep_max = deployed_aabb
        fold_min, fold_max = folded_aabb
        ctx.check(
            "drop_leaf_folds_downward",
            float(fold_min[2]) < float(dep_min[2]) - 0.05,
            details=f"deployed_min_z={float(dep_min[2]):.4f}, folded_min_z={float(fold_min[2]):.4f}",
        )

    return ctx.report()


object_model = build_object_model()
