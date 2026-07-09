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


# ---------------------------------------------------------------------------
# Dimensions — slightly taller stance than the parent (companion variation ⑤).
# ---------------------------------------------------------------------------
TABLE_X = 0.80
TABLE_Y = 0.54
TOP_Z = 0.53
SLAT_THICKNESS = 0.024
SLAT_COUNT = 9
SLAT_GAP = 0.006
SLAT_W = (TABLE_X - (SLAT_COUNT - 1) * SLAT_GAP) / SLAT_COUNT
LEG_TOP_Z = TOP_Z - 0.055
LEG_RADIUS = 0.017
LEG_X = TABLE_X / 2.0 - 0.070
LEG_Y = TABLE_Y / 2.0 - 0.060
# Each X-leg tube spans this Y distance from its pivot toward the opposite side.
LEG_SPREAD_Y = 0.38
# Vertical drop from the pivot to the foot end of each tube.
LEG_DROP = 0.46
LEG_TUBE_LENGTH = math.sqrt(LEG_SPREAD_Y ** 2 + LEG_DROP ** 2)
# Parameter along the tube where the two legs in an X-pair cross.
CROSS_T = LEG_Y / LEG_SPREAD_Y
CROSS_LOCAL_Y = CROSS_T * LEG_SPREAD_Y  # == LEG_Y
CROSS_LOCAL_Z = -CROSS_T * LEG_DROP
FOOT_LENGTH = 0.030


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


def _xleg_tube_mesh(name: str, dy: float, dz: float):
    """Build a straight diagonal tube from (0,0,0) to (0, dy, dz)."""
    return mesh_from_geometry(
        tube_from_spline_points(
            [(0.0, 0.0, 0.0), (0.0, dy, dz)],
            radius=LEG_RADIUS,
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
                "Trestle/X-leg variant of the compact folding camp table. "
                "Each short end carries a crossed-leg pair pinned at a central "
                "pivot, replacing the four independent straight legs. "
                "The slatted roll-top, hinge sockets, and upper collars are "
                "preserved. Stance is slightly taller than the baseline."
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

    # ---- tabletop (root) ----
    tabletop = model.part("tabletop")

    # Segmented aluminum slats with front/rear black end caps.
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

    # Dark shadow strips between slats.
    for i in range(1, SLAT_COUNT):
        x = -TABLE_X / 2.0 + i * SLAT_W + (i - 0.5) * SLAT_GAP
        tabletop.visual(
            Box((SLAT_GAP * 0.65, TABLE_Y + 0.012, 0.026)),
            origin=Origin(xyz=(x, 0.0, TOP_Z + 0.0005)),
            material=dark_aluminum,
            name=f"slat_gap_{i}",
        )

    # Black perimeter rails matching the rounded plastic connectors.
    for y, name in [
        (-TABLE_Y / 2.0 - 0.016, "front_rail"),
        (TABLE_Y / 2.0 + 0.016, "rear_rail"),
    ]:
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

    # Underside cross-rails.
    for y in (-0.16, 0.16):
        tabletop.visual(
            Box((TABLE_X - 0.070, 0.022, 0.030)),
            origin=Origin(xyz=(0.0, y, TOP_Z - 0.018)),
            material=black_plastic,
            name=f"underside_crossrail_{'front' if y < 0 else 'rear'}",
        )

    # Corner connectors.
    for idx, (x, y) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        tabletop.visual(
            Sphere(radius=0.030),
            origin=Origin(
                xyz=(
                    x * (TABLE_X / 2.0 + 0.004),
                    y * (TABLE_Y / 2.0 + 0.004),
                    TOP_Z - 0.012,
                )
            ),
            material=black_plastic,
            name=f"corner_connector_{idx}",
        )

    # End support rails: horizontal members at each short end connecting the
    # two X-leg pivot sockets, providing visible structural support for the
    # crossed-leg pairs.
    for x_sign, end_label in [(-1, "left"), (1, "right")]:
        tabletop.visual(
            Box((0.034, 2.0 * LEG_Y + 0.040, 0.038)),
            origin=Origin(
                xyz=(x_sign * LEG_X, 0.0, LEG_TOP_Z + 0.018),
            ),
            material=black_plastic,
            name=f"end_rail_{end_label}",
        )

    # Hinge sockets for the X-leg pivots (one per corner, same positions as
    # the parent).
    leg_positions = [
        (-LEG_X, -LEG_Y),  # leg_0: left-front pivot
        (LEG_X, -LEG_Y),   # leg_1: right-front pivot
        (LEG_X, LEG_Y),    # leg_2: right-rear pivot
        (-LEG_X, LEG_Y),   # leg_3: left-rear pivot
    ]
    for i, (x, y) in enumerate(leg_positions):
        tabletop.visual(
            Box((0.056, 0.052, 0.070)),
            origin=Origin(xyz=(x, y, LEG_TOP_Z + 0.015)),
            material=black_plastic,
            name=f"hinge_socket_{i}",
        )

    # Small screw heads and oval plastic inserts along the front rail.
    for i, x in enumerate(
        [x0 + k * (SLAT_W + SLAT_GAP) for k in range(SLAT_COUNT)]
    ):
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

    # ---- X-leg members ----
    # Each X-pair has two diagonal tubes crossing near mid-height.  Legs at
    # -Y pivot toward +Y; legs at +Y pivot toward -Y.  The rotation axis is
    # along X so positive q folds the leg upward against the tabletop.
    #
    #   leg_0 & leg_3 → left end pair   (x = -LEG_X)
    #   leg_1 & leg_2 → right end pair  (x = +LEG_X)

    for i, (px, py) in enumerate(leg_positions):
        # Determine tube direction in the leg's local frame.
        y_sign = 1.0 if py < 0.0 else -1.0
        dy = y_sign * LEG_SPREAD_Y
        dz = -LEG_DROP

        leg = model.part(f"leg_{i}")

        # Hinge pin at the pivot (short transverse cylinder).
        leg.visual(
            Cylinder(radius=0.011, length=0.058),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name="hinge_pin",
        )

        # Diagonal X-leg tube from pivot to foot position.
        leg.visual(
            _xleg_tube_mesh(f"xleg_tube_{i}", dy, dz),
            origin=Origin(),
            material=mottled_aluminum,
            name="xleg_tube",
        )

        # Upper collar wrapping the tube near the pivot.
        collar_along = 0.055
        unit_y = dy / LEG_TUBE_LENGTH
        unit_z = dz / LEG_TUBE_LENGTH
        leg.visual(
            Cylinder(radius=0.023, length=0.054),
            origin=Origin(
                xyz=(0.0, collar_along * unit_y, collar_along * unit_z),
                rpy=(math.atan2(-unit_z, unit_y), 0.0, 0.0),
            ),
            material=black_plastic,
            name="upper_collar",
        )

        # Rubber foot at the bottom end of the tube.
        leg.visual(
            Cylinder(radius=0.020, length=FOOT_LENGTH),
            origin=Origin(xyz=(0.0, dy, dz)),
            material=rubber,
            name="rubber_foot",
        )

        # Central pivot bolt where the two X-leg tubes cross.  Added to every
        # leg so each member shows its own half of the crossing hardware.
        # The Y sign matches the tube direction so the bolt sits on the tube
        # centreline at the crossing point.
        leg.visual(
            Cylinder(radius=0.012, length=0.030),
            origin=Origin(
                xyz=(0.0, y_sign * CROSS_LOCAL_Y, CROSS_LOCAL_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=screw_metal,
            name="cross_pivot",
        )

        # Articulation: revolute around X so positive q folds upward.
        axis_x_sign = 1.0 if py < 0.0 else -1.0
        model.articulation(
            f"tabletop_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=tabletop,
            child=leg,
            origin=Origin(xyz=(px, py, LEG_TOP_Z)),
            axis=(axis_x_sign, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=16.0, velocity=2.2, lower=0.0, upper=1.05
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tabletop = object_model.get_part("tabletop")

    # --- hinge socket / pin seating ---
    for i in range(4):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            tabletop,
            leg,
            elem_a=f"hinge_socket_{i}",
            elem_b="hinge_pin",
            reason="The X-leg hinge pin is intentionally captured inside the black socket.",
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
            elem_b="xleg_tube",
            reason="The X-leg tube begins inside the molded hinge socket at the pivot.",
        )
        ctx.expect_overlap(
            leg,
            tabletop,
            axes="xy",
            elem_a="xleg_tube",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.010,
            name=f"leg_{i}_tube_starts_in_socket",
        )

    # --- X-pair tube crossing overlap allowances ---
    # The two tubes in each X-pair intentionally cross and interpenetrate at
    # the pivot point, representing the real bolted crossing joint.
    for pair_name, a, b in [("left", 0, 3), ("right", 1, 2)]:
        leg_a = object_model.get_part(f"leg_{a}")
        leg_b = object_model.get_part(f"leg_{b}")
        ctx.allow_overlap(
            leg_a,
            leg_b,
            elem_a="xleg_tube",
            elem_b="xleg_tube",
            reason=(
                f"The {pair_name} X-pair tubes intentionally cross and share "
                "a central pivot bolt; the local tube interpenetration "
                "represents the real bypassing members pinned together."
            ),
        )
        ctx.allow_overlap(
            leg_a,
            leg_b,
            elem_a="cross_pivot",
            elem_b="cross_pivot",
            reason=(
                f"The {pair_name} X-pair cross-pivot bolts from both legs "
                "meet at the same crossing point, representing one shared "
                "pivot hardware."
            ),
        )
        ctx.allow_overlap(
            leg_a,
            leg_b,
            elem_a="xleg_tube",
            elem_b="cross_pivot",
            reason=(
                f"The {pair_name} X-pair cross-pivot bolt on one leg passes "
                "through the other leg's tube at the crossing point."
            ),
        )
        ctx.allow_overlap(
            leg_b,
            leg_a,
            elem_a="xleg_tube",
            elem_b="cross_pivot",
            reason=(
                f"The {pair_name} X-pair cross-pivot bolt on one leg passes "
                "through the other leg's tube at the crossing point."
            ),
        )

    # --- upper collar / hinge socket nesting ---
    for i in range(4):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            tabletop,
            leg,
            elem_a=f"hinge_socket_{i}",
            elem_b="upper_collar",
            reason=(
                "The upper collar wraps around the tube where it emerges "
                "from the hinge socket; a small local embed represents the "
                "real seated collar-to-socket transition."
            ),
        )

    # --- hinge pin and tube pass through end support rail ---
    # The hinge pin at each pivot is a transverse cylinder that passes through
    # the end support rail (the horizontal member connecting the two sockets
    # at each short end). The X-leg tube also emerges from the same pivot
    # area, briefly sharing space with the rail near the socket.
    end_rail_names = {0: "end_rail_left", 3: "end_rail_left",
                      1: "end_rail_right", 2: "end_rail_right"}
    for i in range(4):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            tabletop,
            leg,
            elem_a=end_rail_names[i],
            elem_b="hinge_pin",
            reason=(
                "The X-leg hinge pin passes through the end support rail "
                "at the pivot, representing the real captured-pin mount."
            ),
        )
        ctx.allow_overlap(
            tabletop,
            leg,
            elem_a=end_rail_names[i],
            elem_b="xleg_tube",
            reason=(
                "The X-leg tube emerges from the pivot socket area and "
                "briefly shares AABB space with the end support rail near "
                "the mounting point."
            ),
        )

    # --- X-pair crossing: each pair's two tubes must cross near mid-height ---
    # The cross_pivot visuals on the two legs in a pair should be close to
    # each other in world space at rest, proving the crossed topology.
    for pair_name, a, b in [("left", 0, 3), ("right", 1, 2)]:
        leg_a = object_model.get_part(f"leg_{a}")
        leg_b = object_model.get_part(f"leg_{b}")
        ctx.expect_contact(
            leg_a,
            leg_b,
            elem_a="cross_pivot",
            elem_b="cross_pivot",
            contact_tol=0.005,
            name=f"{pair_name}_xpair_cross_pivots_near_each_other",
        )

    # --- Proof checks for remaining overlap allowances ---
    # Upper collar nests near the hinge socket (seated transition).
    for i in range(4):
        leg = object_model.get_part(f"leg_{i}")
        ctx.expect_overlap(
            leg,
            tabletop,
            axes="xy",
            elem_a="upper_collar",
            elem_b=f"hinge_socket_{i}",
            min_overlap=0.010,
            name=f"leg_{i}_upper_collar_xy_overlaps_socket",
        )

    # Hinge pin passes through the end support rail at each pivot.
    for i in range(4):
        leg = object_model.get_part(f"leg_{i}")
        ctx.expect_overlap(
            leg,
            tabletop,
            axes="xy",
            elem_a="hinge_pin",
            elem_b=end_rail_names[i],
            min_overlap=0.010,
            name=f"leg_{i}_hinge_pin_xy_overlaps_end_rail",
        )

    # --- X-pair feet must be on opposite Y sides from their pivots ---
    # This proves the diagonal crossing topology (feet cross to the other side).
    for i, (px, py) in enumerate(
        [(-LEG_X, -LEG_Y), (LEG_X, -LEG_Y), (LEG_X, LEG_Y), (-LEG_X, LEG_Y)]
    ):
        leg = object_model.get_part(f"leg_{i}")
        foot_aabb = ctx.part_element_world_aabb(leg, elem="rubber_foot")
        if foot_aabb is not None:
            fmin, fmax = foot_aabb
            foot_center_y = (float(fmin[1]) + float(fmax[1])) / 2.0
            # Foot should be on the opposite side of center from the pivot.
            if py < 0.0:
                ctx.check(
                    f"leg_{i}_foot_crosses_to_positive_y",
                    foot_center_y > 0.0,
                    f"foot center y={foot_center_y:.3f}, pivot y={py:.3f}",
                )
            else:
                ctx.check(
                    f"leg_{i}_foot_crosses_to_negative_y",
                    foot_center_y < 0.0,
                    f"foot center y={foot_center_y:.3f}, pivot y={py:.3f}",
                )

    # --- tabletop geometry ---
    ctx.check(
        "segmented_slat_tabletop",
        tabletop is not None
        and all(
            tabletop.get_visual(f"slat_{i}") is not None for i in range(SLAT_COUNT)
        ),
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
            0.50 <= float(max_pt[1] - min_pt[1]) <= 0.64,
            details=str(aabb),
        )
    else:
        ctx.fail("tabletop_aabb_present", "Expected tabletop AABB.")

    # --- leg fold behavior: leg_0 must fold upward when joint opens ---
    leg0 = object_model.get_part("leg_0")
    leg0_joint = object_model.get_articulation("tabletop_to_leg_0")
    rest_leg0_aabb = ctx.part_world_aabb(leg0)
    if rest_leg0_aabb is not None:
        rest_min, rest_max = rest_leg0_aabb
        ctx.check(
            "open_xleg_reaches_ground",
            float(rest_min[2]) < 0.020 and float(rest_max[2]) > LEG_TOP_Z - 0.020,
            details=str(rest_leg0_aabb),
        )
    with ctx.pose({leg0_joint: 0.90}):
        folded_aabb = ctx.part_world_aabb(leg0)
    if rest_leg0_aabb is not None and folded_aabb is not None:
        rest_min, rest_max = rest_leg0_aabb
        fold_min, fold_max = folded_aabb
        ctx.check(
            "xleg_folds_upward_against_tabletop",
            float(fold_min[2]) > float(rest_min[2]) + 0.12,
            details=f"rest={rest_leg0_aabb}, folded={folded_aabb}",
        )

    # --- X-pair support: each leg's cross_pivot must sit between ground and
    # the tabletop, confirming the trestle structure. ---
    for i in range(4):
        leg = object_model.get_part(f"leg_{i}")
        pivot_aabb = ctx.part_element_world_aabb(leg, elem="cross_pivot")
        if pivot_aabb is not None:
            pmin, pmax = pivot_aabb
            ctx.check(
                f"leg_{i}_cross_pivot_between_ground_and_top",
                0.05 < float(pmin[2]) < TOP_Z - 0.05,
                f"cross_pivot z range: [{pmin[2]:.3f}, {pmax[2]:.3f}]",
            )

    return ctx.report()


object_model = build_object_model()
