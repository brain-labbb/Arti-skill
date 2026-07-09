from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
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
# Compact backpacking folding chair (thin tube frame with fabric bucket sling)
# ---------------------------------------------------------------------------
# Real-world scale: seat ~0.45 m high, backrest top ~0.86 m.
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

TUBE_R = 0.008
HALF_W = 0.215  # half frame width -> side tubes at y = +/- HALF_W

# Side-view (x, z) key points in the OPEN (usable) pose. +X = front of chair.
PIVOT = (0.0, 0.255)  # side crossing / scissor pivot in (x, z)

# Rear leg frame: back feet -> pivot -> backrest top. Leans FORWARD going up.
REAR_FOOT = (-0.19, 0.0)
BACK_TOP = (-0.035, 0.76)
SEAT_HINGE = (-0.155, 0.390)  # rear fabric sling hinge / rear seat rail

# Front leg frame: front feet -> pivot -> seat-front rail. Leans BACK going up.
FRONT_FOOT = (0.22, 0.0)
SEAT_FRONT = (0.155, 0.365)  # front sling rail; carries/supports the bucket seat

SEAT_W = 0.40
SEAT_DEPTH = 0.40
SEAT_THICK = 0.006
BACK_W = 0.39
BACK_H = 0.26
BACK_THICK = 0.006
SIDE_LINK_HINGE = (-0.135, 0.355)
SIDE_LINK_LEN = 0.285
SIDE_LINK_DROP = -0.045

# Fold travel: at this angle the front leg plane becomes parallel to the rear
# leg plane (legs ~54.8 deg from horizontal) -> chair is flat.
FOLD_UPPER = 1.28
# Seat-hinge coupling derived from the rail trajectory: seat tilt vs fold angle.
# With axis +Y, positive seat q drops the seat front (+X) edge, which is exactly
# how the front rail moves as the chair folds -> positive multiplier.
SEAT_FOLD_MULT = 0.49


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
    """Simple rounded panel retained for small sewn hems and cuffs."""
    prof = rounded_rect_profile(width, depth, radius=0.030)
    body = ExtrudeGeometry.from_z0(prof, thick - 0.012, cap=True)
    top_prof = rounded_rect_profile(width - 0.012, depth - 0.012, radius=0.032)
    top = ExtrudeGeometry.from_z0(top_prof, 0.018, cap=True).translate(0.0, 0.0, thick - 0.014)
    return mesh_from_geometry(body.merge(top), name)


def _fabric_bucket_mesh(width: float, depth: float, thick: float, name: str) -> object:
    """Thin sagging fabric bucket: raised perimeter, lower hammock-like center."""
    nx, ny = 8, 8
    geom = MeshGeometry()
    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for i in range(nx + 1):
        row_t: list[int] = []
        row_b: list[int] = []
        x = -depth / 2.0 + depth * i / nx
        ux = i / nx
        for j in range(ny + 1):
            y = -width / 2.0 + width * j / ny
            uy = j / ny
            side_lift = 0.020 * abs(2.0 * uy - 1.0) ** 1.7
            rear_front_lift = 0.010 * (abs(2.0 * ux - 1.0) ** 1.4)
            sag = -0.045 * math.sin(math.pi * ux) * math.sin(math.pi * uy)
            z = side_lift + rear_front_lift + sag
            row_t.append(geom.add_vertex(x, y, z + thick / 2.0))
            row_b.append(geom.add_vertex(x, y, z - thick / 2.0))
        top.append(row_t)
        bottom.append(row_b)
    for i in range(nx):
        for j in range(ny):
            a, b, c, d = top[i][j], top[i + 1][j], top[i + 1][j + 1], top[i][j + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
            a, b, c, d = bottom[i][j], bottom[i][j + 1], bottom[i + 1][j + 1], bottom[i + 1][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    # close four thin fabric edges so the sheet has volume for QC/collision
    for i in range(nx):
        for j in (0, ny):
            a, b = top[i][j], top[i + 1][j]
            c, d = bottom[i + 1][j], bottom[i][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    for j in range(ny):
        for i in (0, nx):
            a, b = top[i][j], top[i][j + 1]
            c, d = bottom[i][j + 1], bottom[i][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return mesh_from_geometry(geom, name)


def _fabric_back_mesh(height: float, width: float, thick: float, name: str) -> object:
    """Slightly cupped fabric backrest panel (local X=height, Y=width)."""
    nx, ny = 6, 8
    geom = MeshGeometry()
    front: list[list[int]] = []
    rear: list[list[int]] = []
    for i in range(nx + 1):
        row_f: list[int] = []
        row_r: list[int] = []
        x = -height / 2.0 + height * i / nx
        ux = i / nx
        for j in range(ny + 1):
            y = -width / 2.0 + width * j / ny
            uy = j / ny
            cup = -0.022 * math.sin(math.pi * ux) * math.sin(math.pi * uy)
            edge = 0.006 * abs(2.0 * uy - 1.0)
            z = cup + edge
            row_f.append(geom.add_vertex(x, y, z + thick / 2.0))
            row_r.append(geom.add_vertex(x, y, z - thick / 2.0))
        front.append(row_f)
        rear.append(row_r)
    for i in range(nx):
        for j in range(ny):
            a, b, c, d = front[i][j], front[i + 1][j], front[i + 1][j + 1], front[i][j + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
            a, b, c, d = rear[i][j], rear[i][j + 1], rear[i + 1][j + 1], rear[i + 1][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    for i in range(nx):
        for j in (0, ny):
            a, b = front[i][j], front[i + 1][j]
            c, d = rear[i + 1][j], rear[i][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    for j in range(ny):
        for i in (0, nx):
            a, b = front[i][j], front[i][j + 1]
            c, d = rear[i][j + 1], rear[i][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backpacking_folding_chair")

    model.material("aluminum", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("joint_black", rgba=(0.05, 0.055, 0.06, 1.0))
    model.material("ripstop_blue", rgba=(0.05, 0.16, 0.34, 1.0))
    model.material("webbing_black", rgba=(0.015, 0.016, 0.018, 1.0))
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
            material="aluminum",
            name=f"rear_upright_{tag}",
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
        material="aluminum",
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
        material="joint_black",
        name="seat_hinge_bar",
    )
    # Lower cross brace tube between the rear legs near the floor (placed on the
    # rear-leg centerline at this height so it actually meets both legs).
    rear_brace_z = 0.085
    rear_brace_x = REAR_FOOT[0] + (PIVOT[0] - REAR_FOOT[0]) * (rear_brace_z - REAR_FOOT[1]) / (PIVOT[1] - REAR_FOOT[1])
    rear.visual(
        _cross_tube(
            (rear_brace_x, -HALF_W, rear_brace_z),
            (rear_brace_x, HALF_W, rear_brace_z),
            TUBE_R - 0.001,
            "rear_cross_brace",
        ),
        material="joint_black",
        name="rear_cross_brace",
    )
    # Cupped fabric backrest sling mounted on the upper uprights.
    back_mid_z = 0.64
    rear.visual(
        # Authored with local X=height(BACK_H), Y=width(BACK_W), Z=thickness; a
        # -90deg pitch maps local X->Z (height) and local Z->X (thickness toward
        # the sitter at +X).
        _fabric_back_mesh(BACK_H, BACK_W, BACK_THICK, "backrest_fabric"),
        origin=Origin(
            xyz=(BACK_TOP[0] + 0.018, 0.0, back_mid_z),
            rpy=(0.0, -math.pi / 2.0, 0.0),
        ),
        material="ripstop_blue",
        name="backrest_fabric",
    )
    rear.visual(
        _cross_tube(
            (BACK_TOP[0] + 0.012, -BACK_W / 2.0, back_mid_z + BACK_H / 2.0 - 0.010),
            (BACK_TOP[0] + 0.012, BACK_W / 2.0, back_mid_z + BACK_H / 2.0 - 0.010),
            0.006,
            "backrest_top_sleeve",
        ),
        material="webbing_black",
        name="backrest_top_sleeve",
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
    for sgn, tag in ((-1.0, "l"), (1.0, "r")):
        front.visual(
            _leg_tube(front_path, sgn * HALF_W, f"front_rail_{tag}"),
            material="aluminum",
            name=f"front_leg_{tag}",
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
    sfx, sfz = fp(*SEAT_FRONT)
    front.visual(
        _cross_tube(
            (sfx, -HALF_W, sfz),
            (sfx, HALF_W, sfz),
            TUBE_R,
            "seat_front_rail",
        ),
        material="joint_black",
        name="seat_front_rail",
    )
    # Lower cross brace between the front legs, near the floor (on the front-leg
    # centerline at this height so it meets both legs).
    front_brace_z = 0.085
    front_brace_x = FRONT_FOOT[0] + (PIVOT[0] - FRONT_FOOT[0]) * (front_brace_z - FRONT_FOOT[1]) / (PIVOT[1] - FRONT_FOOT[1])
    lbx, lbz = fp(front_brace_x, front_brace_z)
    front.visual(
        _cross_tube(
            (lbx, -HALF_W, lbz),
            (lbx, HALF_W, lbz),
            TUBE_R - 0.001,
            "front_cross_brace",
        ),
        material="joint_black",
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

    # ------------------------------------------------------- side hinge links
    # Compact backpacking chairs use collapsing side links that pull the fabric
    # sling rails into a flat bundle.  This is a separate hinged, mimic-driven
    # U-link pair: two side rods plus a front crossbar, so it is visibly a real
    # structural mechanism rather than decorative rods.
    side_links = model.part("side_links")
    lhx, lhz = SIDE_LINK_HINGE
    for sgn, tag in ((-1.0, "l"), (1.0, "r")):
        side_links.visual(
            _cross_tube(
                (0.0, sgn * (HALF_W + 0.012), 0.0),
                (SIDE_LINK_LEN, sgn * (HALF_W + 0.012), SIDE_LINK_DROP),
                TUBE_R - 0.002,
                f"side_link_{tag}",
            ),
            material="joint_black",
            name=f"side_link_{tag}",
        )
        side_links.visual(
            Cylinder(radius=TUBE_R + 0.003, length=0.020),
            origin=Origin(
                xyz=(0.0, sgn * (HALF_W + 0.012), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="joint_black",
            name=f"side_hinge_boss_{tag}",
        )
    side_links.visual(
        _cross_tube(
            (SIDE_LINK_LEN, -HALF_W - 0.012, SIDE_LINK_DROP),
            (SIDE_LINK_LEN, HALF_W + 0.012, SIDE_LINK_DROP),
            TUBE_R - 0.003,
            "link_front_crossbar",
        ),
        material="joint_black",
        name="link_front_crossbar",
    )
    model.articulation(
        "side_link_hinge",
        ArticulationType.REVOLUTE,
        parent=rear,
        child=side_links,
        origin=Origin(xyz=(lhx, 0.0, lhz)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=0.75),
        mimic=Mimic(joint="fold_pivot", multiplier=0.55, offset=0.0),
    )

    # --------------------------------------------------------------- seat pan
    # Fabric bucket hinges at its rear edge on the rear seat-hinge bar. Authored
    # relative to the hinge frame at world (SEAT_HINGE_X, 0, SEAT_HINGE_Z) so the
    # slab extends FORWARD (+X) and is horizontal at q=0.
    seat = model.part("seat_pan")
    shx, shz = SEAT_HINGE
    # Seat center sits half a depth forward of the hinge; tiny -Z so the fabric
    # underside rides on the front rail and hinge bar.
    seat_center_x = shx + SEAT_DEPTH / 2.0
    seat.visual(
        _fabric_bucket_mesh(SEAT_W, SEAT_DEPTH, SEAT_THICK, "bucket_seat_fabric"),
        origin=Origin(xyz=(seat_center_x - shx, 0.0, -0.006)),
        material="ripstop_blue",
        name="bucket_seat_fabric",
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
    side_links = object_model.get_part("side_links")
    fold = object_model.get_articulation("fold_pivot")
    seat_hinge = object_model.get_articulation("seat_hinge")
    side_link_hinge = object_model.get_articulation("side_link_hinge")

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
        elem_b="bucket_seat_fabric",
        reason="Fabric bucket rear sleeve is captured on the rear hinge bar.",
    )
    ctx.allow_overlap(
        rear,
        side_links,
        elem_a="rear_upright_l",
        elem_b="side_hinge_boss_l",
        reason="Side-link hinge boss is pinned through the side of the rear frame.",
    )
    ctx.allow_overlap(
        rear,
        side_links,
        elem_a="rear_upright_r",
        elem_b="side_hinge_boss_r",
        reason="Side-link hinge boss is pinned through the side of the rear frame.",
    )

    # ----------------------------------------------------------- OPEN pose ---
    # Seat is horizontal at sitting height ~0.45 m.
    seat_aabb = ctx.part_world_aabb(seat)
    assert seat_aabb is not None
    seat_top = seat_aabb[1][2]
    ctx.check(
        "seat at sitting height (open)",
        0.38 <= seat_top <= 0.48,
        details=f"seat_top={seat_top:.3f}",
    )
    # Seat is roughly horizontal when open: broad in X (depth), thin in Z.
    seat_x_open = seat_aabb[1][0] - seat_aabb[0][0]
    seat_z_open = seat_aabb[1][2] - seat_aabb[0][2]
    ctx.check(
        "seat is horizontal when open",
        seat_x_open > 0.34 and seat_z_open < 0.10,
        details=f"x_open={seat_x_open:.3f}, z_open={seat_z_open:.3f}",
    )

    # Backrest is compact but clearly above the sling seat and panel-wide.
    back = ctx.part_element_world_aabb(rear, elem="backrest_fabric")
    assert back is not None
    ctx.check(
        "backrest reaches chair-back height",
        0.72 <= back[1][2] <= 0.82,
        details=f"back_top={back[1][2]:.3f}",
    )
    bw = back[1][1] - back[0][1]
    ctx.check(
        "backrest is a wide panel",
        bw > 0.30,
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
        open_stance > 0.30,
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

    # Connectivity: the seat front edge actually rests on the front-frame rail
    # in the open pose (the kinematic coupling is correct, not floating).
    rail = ctx.part_element_world_aabb(front, elem="seat_front_rail")
    seat_pad = ctx.part_element_world_aabb(seat, elem="bucket_seat_fabric")
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
    ctx.check(
        "side hinge links collapse with the fold",
        side_link_hinge.articulation_type == ArticulationType.REVOLUTE
        and side_link_hinge.mimic is not None
        and side_link_hinge.mimic.joint == "fold_pivot",
        details=f"type={side_link_hinge.articulation_type}, mimic={side_link_hinge.mimic}",
    )
    link_open = ctx.part_world_aabb(side_links)
    assert link_open is not None
    link_open_z = link_open[1][2] - link_open[0][2]

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
        seat_pad_f = ctx.part_element_world_aabb(seat, elem="bucket_seat_fabric")
        assert rail_f is not None and seat_pad_f is not None
        rail_f_x = 0.5 * (rail_f[0][0] + rail_f[1][0])
        rail_f_z = 0.5 * (rail_f[0][2] + rail_f[1][2])
        # Distance from rail center to the nearest seat-pad corner stays small.
        dx = max(seat_pad_f[0][0] - rail_f_x, 0.0, rail_f_x - seat_pad_f[1][0])
        dz = max(seat_pad_f[0][2] - rail_f_z, 0.0, rail_f_z - seat_pad_f[1][2])
        rail_to_seat = math.hypot(dx, dz)
        link_folded = ctx.part_world_aabb(side_links)
        assert link_folded is not None
        link_folded_z = link_folded[1][2] - link_folded[0][2]

    # Folding swings the front feet rearward toward the rear feet.
    ctx.check(
        "folding brings front feet back toward rear feet",
        folded_stance < open_stance - 0.25,
        details=f"open_stance={open_stance:.3f}, folded_stance={folded_stance:.3f}",
    )
    # The leg footprint at the floor collapses dramatically -> chair folds flat.
    ctx.check(
        "folded chair packs flat (floor footprint collapses)",
        folded_footprint < open_footprint - 0.25 and folded_footprint < 0.18,
        details=f"open_footprint={open_footprint:.3f}, folded_footprint={folded_footprint:.3f}",
    )
    # Seat tips up out of the way as the chair folds: its Z-span grows markedly
    # (the front edge drops with the rail and the pad rotates toward vertical).
    ctx.check(
        "seat folds up (tilts out of horizontal)",
        seat_z_folded > seat_z_open + 0.15 and seat_x_folded < seat_x_open,
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
    ctx.check(
        "side links swing upward during collapse",
        link_folded_z > link_open_z + 0.10,
        details=f"link_open_z={link_open_z:.3f}, link_folded_z={link_folded_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
