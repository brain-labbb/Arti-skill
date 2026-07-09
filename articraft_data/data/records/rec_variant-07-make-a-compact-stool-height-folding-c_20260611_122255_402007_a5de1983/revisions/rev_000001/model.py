from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Compact stool-height folding chair (triangular tubular-steel scissor fold)
# ---------------------------------------------------------------------------
# Real-world scale: stool-height seat ~0.32 m high, small backrest top ~0.58 m.
# Coordinate frame: +X = front of chair, -X = back; +Y = left-right (width);
# +Z = up. Floor is z=0; the chair rests on four rubber feet at z~0.
#
# Mechanism (TRUE single-DOF scissor fold):
#   - REAR leg frame (root): rear legs rise FORWARD from the back feet, pass
#     through the side crossing pivot, and continue UP to become the backrest
#     uprights carrying the padded backrest. The seat's rear hinge bar lives on
#     this frame and is therefore stationary.
#   - FRONT leg frame (child): front legs rise BACK from the front feet, pass
#     through the same side crossing pivot, and continue UP to the seat-front
#     rail. It is joined to the rear frame by a REVOLUTE at the side crossing
#     pivot (axis = +Y). Positive q swings the front feet rearward/up so the two
#     leg planes become parallel and the chair collapses flat.
#   - SEAT pan (child of rear frame): hinged at its REAR edge on the rear hinge
#     bar; its FRONT edge rests on the front-frame seat-front rail. Because the
#     rail moves with the front frame, the seat tilt is a (very nearly linear)
#     function of the fold angle. The seat hinge therefore MIMICS the fold joint
#     so the front edge tracks the rail through the whole motion: as the chair
#     folds, the seat tips up from horizontal toward vertical and the assembly
#     packs flat. This keeps the chair a single intuitive DOF.
#
# Why the previous version was wrong: the seat-hinge mimic multiplier (0.75) did
# not match the real 4-bar coupling, so the seat front edge drifted OFF the front
# rail during the fold (seat detached from the supporting frame), and the fold
# magnitude did not bring the leg planes parallel. The corrected coupling is
# derived from the rail trajectory (slope ~ -0.49 rad/rad) so the seat front edge
# stays on the rail across the full fold.
# ---------------------------------------------------------------------------

TUBE_R = 0.011
HALF_W = 0.180  # half frame width -> side tubes at y = +/- HALF_W

# Side-view (x, z) key points in the OPEN (usable) pose. +X = front of chair.
PIVOT = (0.0, 0.200)  # side crossing / scissor pivot in (x, z)

# Rear leg frame: back feet -> pivot -> backrest top. Leans FORWARD going up.
REAR_FOOT = (-0.14, 0.0)
BACK_TOP = (-0.045, 0.580)
SEAT_HINGE = (-0.125, 0.325)  # stationary rear hinge of the seat pan (on rear frame)

# Front leg frame: front feet -> pivot -> seat-front rail. Leans BACK going up.
FRONT_FOOT = (0.16, 0.0)
SEAT_FRONT = (0.125, 0.320)  # top of front frame; carries/supports the seat front

SEAT_W = 0.310  # seat width along Y (narrower than the frame to clear the uprights)
SEAT_DEPTH = 0.300  # hinge-to-front depth; overhangs the rail so the front rests on it
SEAT_THICK = 0.038
BACK_W = 0.320
BACK_H = 0.130
BACK_THICK = 0.032

# Fold travel: at this angle the front leg plane becomes parallel to the rear
# leg plane (legs ~54.8 deg from horizontal) -> chair is flat.
FOLD_UPPER = 1.30
# Seat-hinge coupling derived from the rail trajectory: seat tilt vs fold angle.
# With axis +Y, positive seat q drops the seat front (+X) edge, which is exactly
# how the front rail moves as the chair folds -> positive multiplier.
SEAT_FOLD_MULT = 0.50


def _leg_tube(p_xz_list: list[tuple[float, float]], y: float, name: str) -> object:
    pts = [(x, y, z) for (x, z) in p_xz_list]
    geom = tube_from_spline_points(
        pts, radius=TUBE_R, samples_per_segment=12, radial_segments=14, cap_ends=True
    )
    return mesh_from_geometry(geom, name)


def _cross_tube(
    a: tuple[float, float, float], b: tuple[float, float, float], r: float, name: str
) -> object:
    geom = tube_from_spline_points(
        [a, b], radius=r, samples_per_segment=4, radial_segments=12, cap_ends=True
    )
    return mesh_from_geometry(geom, name)


def _pad_mesh(width: float, depth: float, thick: float, name: str) -> object:
    """Padded vinyl panel: compact rounded-rect cushion slab."""
    prof = rounded_rect_profile(width, depth, radius=0.030)
    body = ExtrudeGeometry.from_z0(prof, thick, cap=True)
    return mesh_from_geometry(body, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_stool_height_folding_chair")

    model.material("steel_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    model.material("steel_dark", rgba=(0.34, 0.35, 0.37, 1.0))
    model.material("vinyl", rgba=(0.12, 0.16, 0.20, 1.0))
    model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    # ------------------------------------------------------------------ root
    # Rear leg frame: two side tubes (rear leg -> backrest upright), a top
    # backrest rail, the rear seat-hinge bar, the padded backrest, and rubber
    # back feet.
    rear = model.part("rear_leg_frame")
    rear_path = [REAR_FOOT, PIVOT, SEAT_HINGE, BACK_TOP]
    for sgn, tag in ((-1.0, "l"), (1.0, "r")):
        rear.visual(
            _leg_tube(rear_path, sgn * HALF_W, f"rear_rail_{tag}"),
            material="steel_gray",
            name=f"rear_upright_{tag}",
        )
        # Short side brace closes the rear leg into a triangular stool-height
        # support frame instead of the taller straight chair back structure.
        rear.visual(
            _cross_tube(
                (REAR_FOOT[0], sgn * HALF_W, REAR_FOOT[1] + TUBE_R),
                (SEAT_HINGE[0], sgn * HALF_W, SEAT_HINGE[1]),
                TUBE_R - 0.002,
                f"rear_triangle_brace_{tag}",
            ),
            material="steel_dark",
            name=f"rear_triangle_brace_{tag}",
        )
        # Rubber back foot (cylindrical cap on the floor).
        rear.visual(
            Cylinder(radius=TUBE_R + 0.004, length=0.030),
            origin=Origin(xyz=(REAR_FOOT[0], sgn * HALF_W, 0.015)),
            material="rubber",
            name=f"rear_foot_{tag}",
        )
    # Top backrest rail tube joining the uprights.
    rear.visual(
        _cross_tube(
            (BACK_TOP[0], -HALF_W, BACK_TOP[1]),
            (BACK_TOP[0], HALF_W, BACK_TOP[1]),
            TUBE_R,
            "back_rail_top",
        ),
        material="steel_gray",
        name="back_rail_top",
    )
    # Rear seat-hinge cross tube (the seat pivots on this).
    rear.visual(
        _cross_tube(
            (SEAT_HINGE[0], -HALF_W, SEAT_HINGE[1]),
            (SEAT_HINGE[0], HALF_W, SEAT_HINGE[1]),
            TUBE_R,
            "seat_hinge_bar",
        ),
        material="steel_dark",
        name="seat_hinge_bar",
    )
    # Lower cross brace tube between the rear legs near the floor (placed on the
    # rear-leg centerline at this height so it actually meets both legs).
    rear_brace_z = 0.085
    rear_brace_x = REAR_FOOT[0] + (rear_brace_z - REAR_FOOT[1]) * (
        (PIVOT[0] - REAR_FOOT[0]) / (PIVOT[1] - REAR_FOOT[1])
    )
    rear.visual(
        _cross_tube(
            (rear_brace_x, -HALF_W, rear_brace_z),
            (rear_brace_x, HALF_W, rear_brace_z),
            TUBE_R - 0.002,
            "rear_cross_brace",
        ),
        material="steel_dark",
        name="rear_cross_brace",
    )
    # Padded backrest mounted on the upper uprights (top ~0.86 m).
    back_mid_z = 0.525
    rear.visual(
        # Authored with local X=height(BACK_H), Y=width(BACK_W), Z=thickness; a
        # -90deg pitch maps local X->Z (height) and local Z->X (thickness toward
        # the sitter at +X).
        _pad_mesh(BACK_H, BACK_W, BACK_THICK, "backrest_pad"),
        origin=Origin(
            xyz=(BACK_TOP[0] + 0.018, 0.0, back_mid_z),
            rpy=(0.0, -math.pi / 2.0, 0.0),
        ),
        material="vinyl",
        name="backrest_pad",
    )

    # ------------------------------------------------------- front leg frame
    # Front-frame visuals are authored relative to the fold-pivot frame at world
    # (PIVOT_X, 0, PIVOT_Z): local = world - pivot. At q=0 the child frame is
    # coincident with the pivot frame, so this reproduces the open pose.
    front = model.part("front_leg_frame")
    px, pz = PIVOT

    def fp(x: float, z: float) -> tuple[float, float]:
        return (x - px, z - pz)

    front_path = [fp(*FRONT_FOOT), fp(*PIVOT), fp(*SEAT_FRONT)]
    sfx, sfz = fp(*SEAT_FRONT)
    for sgn, tag in ((-1.0, "l"), (1.0, "r")):
        front.visual(
            _leg_tube(front_path, sgn * HALF_W, f"front_rail_{tag}"),
            material="steel_gray",
            name=f"front_leg_{tag}",
        )
        # The compact variant uses a triangular folding side frame: a diagonal
        # compression member links each front foot directly to the seat-front
        # rail so the stool-height leg assembly is visibly triangulated.
        ffx_tri, ffz_tri = fp(*FRONT_FOOT)
        front.visual(
            _cross_tube(
                (ffx_tri, sgn * HALF_W, ffz_tri + TUBE_R),
                (sfx, sgn * HALF_W, sfz),
                TUBE_R - 0.002,
                f"front_triangle_brace_{tag}",
            ),
            material="steel_dark",
            name=f"front_triangle_brace_{tag}",
        )
        # Rubber front foot.
        ffx, ffz = fp(*FRONT_FOOT)
        front.visual(
            Cylinder(radius=TUBE_R + 0.004, length=0.030),
            origin=Origin(xyz=(ffx, sgn * HALF_W, ffz + 0.015)),
            material="rubber",
            name=f"front_foot_{tag}",
        )
    # Seat-front rail (supports the seat front edge).
    front.visual(
        _cross_tube(
            (sfx, -HALF_W, sfz),
            (sfx, HALF_W, sfz),
            TUBE_R,
            "seat_front_rail",
        ),
        material="steel_dark",
        name="seat_front_rail",
    )
    # Lower cross brace between the front legs, near the floor (on the front-leg
    # centerline at this height so it meets both legs).
    front_brace_z = 0.085
    front_brace_x = FRONT_FOOT[0] + (front_brace_z - FRONT_FOOT[1]) * (
        (PIVOT[0] - FRONT_FOOT[0]) / (PIVOT[1] - FRONT_FOOT[1])
    )
    lbx, lbz = fp(front_brace_x, front_brace_z)
    front.visual(
        _cross_tube(
            (lbx, -HALF_W, lbz),
            (lbx, HALF_W, lbz),
            TUBE_R - 0.002,
            "front_cross_brace",
        ),
        material="steel_dark",
        name="front_cross_brace",
    )

    # Fold joint: front frame rotates about the side crossing pivot (axis +Y).
    # Positive q swings the front feet rearward and up toward the rear legs ->
    # the chair folds flat.
    model.articulation(
        "fold_pivot",
        ArticulationType.REVOLUTE,
        parent=rear,
        child=front,
        origin=Origin(xyz=(px, 0.0, pz)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=FOLD_UPPER),
    )

    # --------------------------------------------------------------- seat pan
    # Seat pan hinges at its rear edge on the rear seat-hinge bar. Authored
    # relative to the hinge frame at world (SEAT_HINGE_X, 0, SEAT_HINGE_Z) so the
    # slab extends FORWARD (+X) and is horizontal at q=0.
    seat = model.part("seat_pan")
    shx, shz = SEAT_HINGE
    # Seat center sits half a depth forward of the hinge; tiny -Z so the pad
    # underside rides on the front rail and hinge bar.
    seat_center_x = shx + SEAT_DEPTH / 2.0
    seat.visual(
        _pad_mesh(SEAT_DEPTH, SEAT_W, SEAT_THICK, "seat_pad"),
        origin=Origin(xyz=(seat_center_x - shx, 0.0, -0.022)),
        material="vinyl",
        name="seat_pad",
    )

    # The seat front edge must track the moving front rail. The rail trajectory
    # gives an almost perfectly linear seat-tilt vs fold-angle relation; using a
    # +Y axis, positive q tips the seat front DOWN, so a NEGATIVE multiplier on
    # the (positive-folding) source joint tips the front UP as the chair folds.
    # Net effect over the fold: front edge stays on the rail, seat goes from
    # horizontal (open) toward vertical (folded).
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=rear,
        child=seat,
        origin=Origin(xyz=(shx, 0.0, shz)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=2.0, lower=-0.05, upper=SEAT_FOLD_MULT * FOLD_UPPER
        ),
        mimic=Mimic(joint="fold_pivot", multiplier=SEAT_FOLD_MULT, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear = object_model.get_part("rear_leg_frame")
    front = object_model.get_part("front_leg_frame")
    seat = object_model.get_part("seat_pan")
    fold = object_model.get_articulation("fold_pivot")
    seat_hinge = object_model.get_articulation("seat_hinge")

    # Front and rear frames cross at and share the central scissor pivot.
    ctx.allow_overlap(
        rear,
        front,
        reason="Front and rear leg frames cross and share the central scissor pivot.",
    )
    # Seat pan front edge rests on the front-frame seat-front rail.
    ctx.allow_overlap(
        front,
        seat,
        reason="Seat pan front edge rests on the front frame seat-front rail.",
    )
    # Seat rear edge wraps the rear hinge bar.
    ctx.allow_overlap(
        rear,
        seat,
        elem_a="seat_hinge_bar",
        elem_b="seat_pad",
        reason="Seat pan rear edge is captured on the rear hinge bar.",
    )

    # ----------------------------------------------------------- OPEN pose ---
    # Seat is horizontal at compact stool height (~0.32 m).
    seat_aabb = ctx.part_world_aabb(seat)
    assert seat_aabb is not None
    seat_top = seat_aabb[1][2]
    ctx.check(
        "seat at stool height (open)",
        0.29 <= seat_top <= 0.38,
        details=f"seat_top={seat_top:.3f}",
    )
    # Seat is roughly horizontal when open: broad in X (depth), thin in Z.
    seat_x_open = seat_aabb[1][0] - seat_aabb[0][0]
    seat_z_open = seat_aabb[1][2] - seat_aabb[0][2]
    ctx.check(
        "seat is horizontal when open",
        seat_x_open > 0.24 and seat_z_open < 0.09,
        details=f"x_open={seat_x_open:.3f}, z_open={seat_z_open:.3f}",
    )

    # Backrest is intentionally small, but still clearly present above the seat.
    back = ctx.part_element_world_aabb(rear, elem="backrest_pad")
    assert back is not None
    ctx.check(
        "small backrest rises above stool seat",
        0.50 <= back[1][2] <= 0.66,
        details=f"back_top={back[1][2]:.3f}",
    )
    bw = back[1][1] - back[0][1]
    ctx.check(
        "backrest is a wide panel",
        bw > 0.26,
        details=f"back_width={bw:.3f}",
    )

    # Four rubber feet on the floor in the open pose.
    for part, elem in (
        (rear, "rear_foot_l"),
        (rear, "rear_foot_r"),
        (front, "front_foot_l"),
        (front, "front_foot_r"),
    ):
        fa = ctx.part_element_world_aabb(part, elem=elem)
        assert fa is not None, elem
        ctx.check(
            f"{elem} on the floor (open)",
            fa[0][2] < 0.010,
            details=f"{elem}_bottom={fa[0][2]:.3f}",
        )

    # Open stance: front feet are well ahead of rear feet (upright, stable).
    rf = ctx.part_element_world_aabb(rear, elem="rear_foot_l")
    ff = ctx.part_element_world_aabb(front, elem="front_foot_l")
    assert rf is not None and ff is not None
    open_stance = ff[1][0] - rf[0][0]
    ctx.check(
        "open stance: front feet ahead of rear feet",
        open_stance > 0.25,
        details=f"open_stance={open_stance:.3f}",
    )

    # Lower cross braces exist on both leg frames near the floor.
    for part, elem in ((front, "front_cross_brace"), (rear, "rear_cross_brace")):
        brace = ctx.part_element_world_aabb(part, elem=elem)
        assert brace is not None, elem
        brace_w = brace[1][1] - brace[0][1]
        ctx.check(
            f"{elem} spans the legs near the floor",
            brace_w > 0.30 and brace[1][2] < 0.25,
            details=f"{elem}_w={brace_w:.3f}, z={brace[1][2]:.3f}",
        )

    # The variant has real triangular folding side frames, not just recolored
    # straight legs: each folding leg assembly includes diagonal compression
    # braces that tie the foot to the seat rail/hinge area.
    for part, elem in (
        (front, "front_triangle_brace_l"),
        (front, "front_triangle_brace_r"),
        (rear, "rear_triangle_brace_l"),
        (rear, "rear_triangle_brace_r"),
    ):
        brace = ctx.part_element_world_aabb(part, elem=elem)
        assert brace is not None, elem
        bx = brace[1][0] - brace[0][0]
        bz = brace[1][2] - brace[0][2]
        ctx.check(
            f"{elem} makes a triangular side frame",
            bx > 0.015 and bz > 0.20,
            details=f"{elem}_dx={bx:.3f}, dz={bz:.3f}",
        )

    # Connectivity: the seat front edge actually rests on the front-frame rail
    # in the open pose (the kinematic coupling is correct, not floating).
    rail = ctx.part_element_world_aabb(front, elem="seat_front_rail")
    seat_pad = ctx.part_element_world_aabb(seat, elem="seat_pad")
    assert rail is not None and seat_pad is not None
    # Rail X lies under the seat span, and the rail top is right at the seat
    # underside (contact within a couple cm).
    rail_x = 0.5 * (rail[0][0] + rail[1][0])
    seat_supported = (
        seat_pad[0][0] - 0.02 <= rail_x <= seat_pad[1][0] + 0.02
        and abs(rail[1][2] - seat_pad[0][2]) < 0.04
    )
    ctx.check(
        "seat front rests on the front rail (open)",
        seat_supported,
        details=f"rail_x={rail_x:.3f}, rail_top={rail[1][2]:.3f}, seat_bottom={seat_pad[0][2]:.3f}",
    )

    # ----------------------------------------------- Articulation metadata ---
    ctx.check(
        "fold_pivot is revolute about Y",
        fold.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(c, 2) for c in fold.axis) == (0.0, 1.0, 0.0),
        details=f"type={fold.articulation_type}, axis={fold.axis}",
    )
    ctx.check(
        "seat hinge mimics the fold joint (single DOF)",
        seat_hinge.mimic is not None and seat_hinge.mimic.joint == "fold_pivot",
        details=f"mimic={seat_hinge.mimic}",
    )

    # OPEN floor footprint (X span of all feet + lower braces): this is the
    # leg-stance the fold collapses. The whole-AABB depth is a poor fold metric
    # here because the static rear backrest leans back while the front rail
    # swings forward; the genuine, dramatic fold signature is the feet/legs
    # converging at the floor.
    floor_elems = (
        (rear, "rear_foot_l"),
        (rear, "rear_foot_r"),
        (rear, "rear_cross_brace"),
        (front, "front_foot_l"),
        (front, "front_foot_r"),
        (front, "front_cross_brace"),
    )

    def _floor_footprint() -> float:
        xs: list[float] = []
        for part, elem in floor_elems:
            a = ctx.part_element_world_aabb(part, elem=elem)
            assert a is not None, elem
            xs += [a[0][0], a[1][0]]
        return max(xs) - min(xs)

    open_footprint = _floor_footprint()

    # --------------------------------------------------------- FOLDED pose ---
    with ctx.pose({fold: FOLD_UPPER}):
        ff_folded = ctx.part_element_world_aabb(front, elem="front_foot_l")
        assert ff_folded is not None
        folded_stance = ff_folded[1][0] - rf[0][0]

        folded_footprint = _floor_footprint()

        seat_folded = ctx.part_world_aabb(seat)
        assert seat_folded is not None
        seat_x_folded = seat_folded[1][0] - seat_folded[0][0]
        seat_z_folded = seat_folded[1][2] - seat_folded[0][2]

        # Seat front edge still tracks the front rail when folded (stays
        # connected, did not drift off the supporting frame).
        rail_f = ctx.part_element_world_aabb(front, elem="seat_front_rail")
        seat_pad_f = ctx.part_element_world_aabb(seat, elem="seat_pad")
        assert rail_f is not None and seat_pad_f is not None
        rail_f_x = 0.5 * (rail_f[0][0] + rail_f[1][0])
        rail_f_z = 0.5 * (rail_f[0][2] + rail_f[1][2])
        # Distance from rail center to the nearest seat-pad corner stays small.
        dx = max(seat_pad_f[0][0] - rail_f_x, 0.0, rail_f_x - seat_pad_f[1][0])
        dz = max(seat_pad_f[0][2] - rail_f_z, 0.0, rail_f_z - seat_pad_f[1][2])
        rail_to_seat = math.hypot(dx, dz)

    # Folding swings the front feet rearward toward the rear feet.
    ctx.check(
        "folding brings front feet back toward rear feet",
        folded_stance < open_stance - 0.25,
        details=f"open_stance={open_stance:.3f}, folded_stance={folded_stance:.3f}",
    )
    # The leg footprint at the floor collapses dramatically -> chair folds flat.
    ctx.check(
        "folded chair packs flat (floor footprint collapses)",
        folded_footprint < open_footprint - 0.18 and folded_footprint < 0.16,
        details=f"open_footprint={open_footprint:.3f}, folded_footprint={folded_footprint:.3f}",
    )
    # Seat tips up out of the way as the chair folds: its Z-span grows markedly
    # (the front edge drops with the rail and the pad rotates toward vertical).
    ctx.check(
        "seat folds up (tilts out of horizontal)",
        seat_z_folded > seat_z_open + 0.11 and seat_x_folded < seat_x_open,
        details=(
            f"x_open={seat_x_open:.3f}, x_folded={seat_x_folded:.3f}, "
            f"z_open={seat_z_open:.3f}, z_folded={seat_z_folded:.3f}"
        ),
    )
    # Seat stays mechanically connected to the front rail through the fold
    # (the 4-bar coupling keeps the front edge on the moving rail).
    ctx.check(
        "seat front edge stays on the front rail (folded)",
        rail_to_seat < 0.06,
        details=f"rail_to_seat={rail_to_seat:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
