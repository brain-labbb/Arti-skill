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
LEG_LENGTH = 0.475
LEG_RADIUS = 0.017
LEG_X = TABLE_X / 2.0 - 0.070
LEG_Y = TABLE_Y / 2.0 - 0.060
LOWER_BRACE_Z = -0.270
CONTACT_POST_RADIUS = 0.0065

# Leg hinge positions on the tabletop underside.
LEG_POSITIONS = [
    (-LEG_X, -LEG_Y),
    (LEG_X, -LEG_Y),
    (LEG_X, LEG_Y),
    (-LEG_X, LEG_Y),
]

# A-frame outward splay angles (from vertical).
SPLAY_X = math.radians(14)  # outward tilt in the table-width direction
SPLAY_Y = math.radians(8)   # outward tilt in the table-depth direction

# Precompute trig for the combined two-axis splay.
_SX = math.sin(SPLAY_X)
_CX = math.cos(SPLAY_X)
_SY = math.sin(SPLAY_Y)
_CY = math.cos(SPLAY_Y)


def _splay_rpy(x_sign: float, y_sign: float) -> tuple[float, float, float]:
    """Return (roll, pitch, 0) that tilts a Z-axis cylinder outward."""
    return (y_sign * SPLAY_Y, -x_sign * SPLAY_X, 0.0)


def _splayed_xyz(
    dist: float,
    x_sign: float,
    y_sign: float,
    lateral_y: float = 0.0,
) -> tuple[float, float, float]:
    """World-frame offset for a point at *dist* along the splayed tube axis.

    The rotation order is ``R = Rz(0) * Ry(pitch) * Rx(roll)``, so a point
    originally at ``(0, 0, -dist)`` ends up at:

    * x =  dist * cos(rx) * (-sin(ry))   → outward in X
    * y =  dist * sin(rx)                 → outward in Y
    * z = -dist * cos(rx) * cos(ry)       → slightly shortened vertically
    """
    cx = dist * _SX * _CY * x_sign
    cy = dist * _SY * y_sign + lateral_y
    cz = -dist * _CX * _CY
    return (cx, cy, cz)


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


def _brace_mesh(name: str, dx: float, dy: float, dz: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            [(0.0, 0.0, 0.0), (dx, dy, dz)],
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
                "A-frame variant: legs splay outward (14° X / 8° Y) for a wider "
                "ground footprint. Olive powder-coat frame colorway applied to tubes "
                "and braces. All fold hinges and X-braces preserved."
            )
        },
    )

    # --- Materials ---------------------------------------------------------
    mottled_aluminum = model.material("mottled_aluminum", rgba=(0.54, 0.55, 0.54, 1.0))
    olive_coat = model.material("olive_powder_coat", rgba=(0.32, 0.38, 0.22, 1.0))
    dark_aluminum = model.material("dark_gap_shadow", rgba=(0.025, 0.026, 0.026, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.02, 0.019, 0.018, 1.0))
    screw_metal = model.material("screw_heads", rgba=(0.83, 0.82, 0.78, 1.0))

    slat_mesh = _slat_mesh()
    end_cap_mesh = _capsule_x_mesh(
        "rounded_slat_end_cap", radius=0.012, mid_length=max(0.001, SLAT_W - 0.024)
    )

    # --- Tabletop ----------------------------------------------------------
    tabletop = model.part("tabletop")

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

    # Hinge sockets for folding legs.
    for i, (x, y) in enumerate(LEG_POSITIONS):
        tabletop.visual(
            Box((0.056, 0.052, 0.070)),
            origin=Origin(xyz=(x, y, LEG_TOP_Z + 0.015)),
            material=black_plastic,
            name=f"hinge_socket_{i}",
        )

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

    # --- Legs (A-frame splayed tubes) --------------------------------------
    tube_length = LEG_LENGTH - 0.022
    tube_half = tube_length / 2.0
    tube_dist = tube_half + 0.003
    collar_dist = 0.055
    foot_dist = LEG_LENGTH - 0.003

    for i, (x, y) in enumerate(LEG_POSITIONS):
        x_sign = 1.0 if x > 0.0 else -1.0
        y_sign = 1.0 if y > 0.0 else -1.0
        rpy = _splay_rpy(x_sign, y_sign)

        leg = model.part(f"leg_{i}")

        # Hinge pin stays horizontal at the pivot.
        leg.visual(
            Cylinder(radius=0.011, length=0.058),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="hinge_pin",
        )

        # Splayed tube (olive powder-coat).
        leg.visual(
            Cylinder(radius=LEG_RADIUS, length=tube_length),
            origin=Origin(xyz=_splayed_xyz(tube_dist, x_sign, y_sign), rpy=rpy),
            material=olive_coat,
            name="straight_tube",
        )

        # Upper collar follows splay.
        leg.visual(
            Cylinder(radius=0.023, length=0.060),
            origin=Origin(xyz=_splayed_xyz(collar_dist, x_sign, y_sign), rpy=rpy),
            material=black_plastic,
            name="upper_collar",
        )

        # Rubber foot at end of splayed tube.
        leg.visual(
            Cylinder(radius=0.020, length=0.036),
            origin=Origin(xyz=_splayed_xyz(foot_dist, x_sign, y_sign), rpy=rpy),
            material=rubber,
            name="rubber_foot",
        )

        # Brace contact post — positioned along the splayed tube at the
        # brace crossing height, protruding laterally.  The offset is wider
        # than the parent baseline so the X-brace clears the splayed leg tube.
        brace_y_offset = 0.088 if i in (1, 3) else 0.062
        outer_y_sign = 1.0 if y > 0.0 else -1.0
        post_length = brace_y_offset - LEG_RADIUS - 0.012
        d_brace = abs(LOWER_BRACE_Z) / (_CX * _CY)
        post_lateral = outer_y_sign * (LEG_RADIUS + post_length / 2.0)
        post_cx, post_cy_raw, _ = _splayed_xyz(d_brace, x_sign, y_sign)
        post_cy = post_cy_raw + post_lateral
        leg.visual(
            Cylinder(radius=CONTACT_POST_RADIUS, length=post_length),
            origin=Origin(
                xyz=(post_cx, post_cy, LOWER_BRACE_Z),
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

    # --- X-frame diagonal braces -------------------------------------------
    # Each brace reaches from its tabletop anchor to the opposite leg's
    # splayed contact post.  Compute the 3-D endpoint per brace.
    brace_anchor_specs = [
        (0, -LEG_X, -LEG_Y - 0.088, 1.0),   # brace_0 anchor, axis_sign
        (1, LEG_X, -LEG_Y - 0.062, -1.0),    # brace_1
        (2, LEG_X, LEG_Y + 0.088, -1.0),     # brace_2
        (3, -LEG_X, LEG_Y + 0.062, 1.0),     # brace_3
    ]
    # Which leg each brace's lower eye contacts.
    brace_target_leg = [1, 0, 3, 2]

    for bi, (anchor_leg_i, anchor_x, anchor_y, axis_sign) in enumerate(brace_anchor_specs):
        anchor_z = LEG_TOP_Z - 0.010

        # Target leg contact-post world position.
        ti = brace_target_leg[bi]
        tx, ty = LEG_POSITIONS[ti]
        tx_sign = 1.0 if tx > 0.0 else -1.0
        ty_sign = 1.0 if ty > 0.0 else -1.0
        t_outer_y = 1.0 if ty > 0.0 else -1.0
        t_brace_y_off = 0.088 if ti in (1, 3) else 0.062
        t_post_len = t_brace_y_off - LEG_RADIUS - 0.012
        t_lateral = t_outer_y * (LEG_RADIUS + t_post_len / 2.0)

        d_brace = abs(LOWER_BRACE_Z) / (_CX * _CY)
        t_cx, t_cy_raw, _ = _splayed_xyz(d_brace, tx_sign, ty_sign)
        target_world_x = tx + t_cx
        target_world_y = ty + t_cy_raw + t_lateral
        target_world_z = LEG_TOP_Z + LOWER_BRACE_Z

        dx = target_world_x - anchor_x
        dy = target_world_y - anchor_y
        dz = target_world_z - anchor_z

        # Anchor block on tabletop underside.
        tabletop.visual(
            Box((0.038, 0.030, 0.065)),
            origin=Origin(xyz=(anchor_x, anchor_y, LEG_TOP_Z + 0.010)),
            material=black_plastic,
            name=f"brace_anchor_{bi}",
        )

        brace = model.part(f"brace_{bi}")
        brace.visual(
            _brace_mesh(f"brace_{bi}_tube", dx, dy, dz),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=olive_coat,
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
            origin=Origin(xyz=(dx, dy, dz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="lower_eye",
        )
        brace.visual(
            Cylinder(radius=0.009, length=0.012),
            origin=Origin(
                xyz=(dx / 2.0, dy / 2.0, dz / 2.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=screw_metal,
            name="center_rivet",
        )
        model.articulation(
            f"tabletop_to_brace_{bi}",
            ArticulationType.REVOLUTE,
            parent=tabletop,
            child=brace,
            origin=Origin(xyz=(anchor_x, anchor_y, LEG_TOP_Z - 0.010)),
            axis=(0.0, axis_sign, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-0.25, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tabletop = object_model.get_part("tabletop")

    # --- Leg hinge pin / socket seating -----------------------------------
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
            reason="The splayed leg tube has a short hidden insertion into the molded hinge socket.",
        )
        ctx.expect_overlap(
            leg,
            tabletop,
            axes="xy",
            elem_a="straight_tube",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.012,
            name=f"leg_{i}_tube_inserted_in_socket",
        )

    # --- Brace anchor / eye seating ----------------------------------------
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

    # --- Brace contact post / lower eye contact ----------------------------
    for leg_i, brace_i in [(1, 0), (0, 1), (3, 2), (2, 3)]:
        leg = object_model.get_part(f"leg_{leg_i}")
        brace = object_model.get_part(f"brace_{brace_i}")
        ctx.allow_overlap(
            leg,
            brace,
            elem_a="brace_contact_post",
            elem_b="lower_eye",
            reason="The brace lower eye is a captured pivot seated against the leg contact post.",
        )
        ctx.allow_overlap(
            leg,
            brace,
            elem_a="brace_contact_post",
            elem_b="diagonal_tube",
            reason="The brace tube end is seated against the leg contact post at the X-frame crossing.",
        )
        ctx.expect_contact(
            leg,
            brace,
            elem_a="brace_contact_post",
            elem_b="lower_eye",
            contact_tol=1e-5,
            name=f"leg_{leg_i}_contact_post_touches_brace_{brace_i}_lower_eye",
        )

    # --- Tabletop slat check -----------------------------------------------
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
        ctx.check(
            "tabletop_depth_realistic",
            0.54 <= float(max_pt[1] - min_pt[1]) <= 0.72,
            details=str(aabb),
        )
    else:
        ctx.fail("tabletop_aabb_present", "Expected tabletop AABB.")

    # --- Leg ground reach and fold -----------------------------------------
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

    # --- Brace hinge motion ------------------------------------------------
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

    # --- A-frame splay: foot lands wider than hinge socket -----------------
    # Prove the straight_tube foot is further from the table centerline than
    # the hinge socket, confirming the outward A-frame stance.
    for i in range(4):
        leg = object_model.get_part(f"leg_{i}")
        foot_aabb = ctx.part_element_world_aabb(leg, elem="rubber_foot")
        tube_aabb = ctx.part_element_world_aabb(leg, elem="straight_tube")
        hinge_x = LEG_POSITIONS[i][0]
        if foot_aabb is not None and tube_aabb is not None:
            foot_min, foot_max = foot_aabb
            foot_center_x = (float(foot_min[0]) + float(foot_max[0])) / 2.0
            # Foot center should be further from x=0 than the hinge (outward splay).
            ctx.check(
                f"leg_{i}_aframe_splay_x",
                abs(foot_center_x) > abs(hinge_x) + 0.030,
                details=f"foot_x={foot_center_x:.4f}, hinge_x={hinge_x:.4f}",
            )
            # Foot should be near ground level.
            ctx.check(
                f"leg_{i}_foot_near_ground",
                float(foot_min[2]) < 0.030,
                details=f"foot_min_z={foot_min[2]:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
