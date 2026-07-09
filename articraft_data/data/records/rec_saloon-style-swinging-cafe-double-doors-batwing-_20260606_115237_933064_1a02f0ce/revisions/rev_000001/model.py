from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Saloon-style batwing cafe double doors.
#
# Frame (fixed root) is a dark-wood doorway: two side jambs + a top header,
# spanning an opening ~0.95 m wide and ~2.0 m tall. The two short wood leaves
# only cover the middle band of the opening (a gap above and below), and each
# leaf is hung from the frame by a vertical spring pivot so it swings BOTH
# directions (double-acting).
#
# Axes: X = doorway width (left/right), Y = door thickness / swing depth,
#       Z = height. Hinge axis is vertical (+Z). Door faces lie in the XZ plane.
#
# Mirror symmetry: ONE leaf is authored with its hinge edge at part-local x=0,
# the body extending toward the center along +X, and the scalloped crown peaking
# at the INNER (center) edge -- exactly as in the reference, where the pair
# humps up in the middle and sweeps down toward the jambs. door_1 is the EXACT
# mirror of door_0: the slab/louver/pivot are mirrored across X (so its body
# extends along -X toward the center and its crown also peaks at the center) and
# the leaf is placed with NO yaw. This guarantees a true left-right mirror when
# closed (matched scallops, mirrored slat tilt), never rotational symmetry.
# ---------------------------------------------------------------------------

# Opening / frame dimensions.
OPENING_WIDTH = 0.95
OPENING_HEIGHT = 2.0
JAMB_WIDTH = 0.07  # left/right jamb thickness in X
JAMB_DEPTH = 0.12  # frame depth in Y
HEADER_HEIGHT = 0.09  # top header thickness in Z

# Leaf (batwing) dimensions. The two leaves cover only the middle band.
LEAF_BOTTOM_Z = 0.62  # gap below (kick clearance)
LEAF_BODY_HEIGHT = 0.95  # straight body height of one leaf
SCALLOP_RISE = 0.13  # extra rise of the curved/scalloped top crown
LEAF_TOP_Z = LEAF_BOTTOM_Z + LEAF_BODY_HEIGHT + SCALLOP_RISE
LEAF_THICKNESS = 0.035  # door slab thickness along Y
LOUVER_GAP = 0.006  # gap between the two leaves at the center meeting stiles

# Each leaf width: split the clear opening, leaving a small center gap.
CLEAR_WIDTH = OPENING_WIDTH - 2.0 * JAMB_WIDTH
LEAF_WIDTH = (CLEAR_WIDTH - LOUVER_GAP) / 2.0

# Louver band on the leaf (horizontal slats fill most of the body).
STILE_WIDTH = 0.05  # vertical side rails of the leaf
TOP_RAIL = 0.04  # rail just under the scalloped crown
BOTTOM_RAIL = 0.05  # bottom rail
LOUVER_WIDTH = LEAF_WIDTH - 2.0 * STILE_WIDTH
LOUVER_HEIGHT = LEAF_BODY_HEIGHT - TOP_RAIL - BOTTOM_RAIL

# Hinge geometry: spring pivot at the OUTER edge of each leaf, near the jamb.
PIVOT_RADIUS = 0.012


def _scalloped_leaf_profile(width: float, body_h: float, rise: float) -> list[tuple[float, float]]:
    """Closed XY profile of one leaf slab face: flat sides/bottom, scalloped crown.

    Built centered in X around 0, bottom at y=0. The crown is a smooth cyma
    (ogee) curve that is LOWEST at the outer (left, -half) edge and PEAKS at the
    inner (right, +half) edge, so when door_1 is mirrored the two crowns sweep
    up toward the center and meet in a hump -- matching the saloon-door crown in
    the reference image. door_0 uses this profile directly; door_1 mirrors it.
    """
    half = width / 2.0
    pts: list[tuple[float, float]] = []
    # Bottom edge, outer (left) to inner (right).
    pts.append((-half, 0.0))
    pts.append((half, 0.0))
    # Inner (right) side up to the crown peak.
    pts.append((half, body_h))
    # Scalloped crown sampled from the inner (right) edge across to the outer
    # (left) edge. Highest at the inner edge (t=0), sweeping down to a small
    # shoulder at the outer edge so the pair reads as a centered hump.
    n = 28
    for i in range(n + 1):
        t = i / n  # 0 at inner (+half) edge, 1 at outer (-half) edge
        x = half - t * width
        # Cyma: full rise at the inner edge, easing to ~15% rise at the outer
        # shoulder via a smooth cosine, so the crown is tallest toward center.
        crown = rise * (0.15 + 0.85 * (0.5 + 0.5 * math.cos(math.pi * t)))
        pts.append((x, body_h + crown))
    # Close down the outer (left) side.
    pts.append((-half, body_h))
    return pts


def _louver_grille_mesh(name: str):
    """A framed louver panel with horizontal slats, authored in local XY.

    The helper builds slats in local XY extending along +Z (panel normal).
    We rotate it into the door's XZ face when placing the visual.
    """
    return mesh_from_geometry(
        VentGrilleGeometry(
            (LOUVER_WIDTH, LOUVER_HEIGHT),
            frame=0.010,
            face_thickness=0.006,
            duct_depth=0.012,
            duct_wall=0.004,
            slat_pitch=0.030,
            slat_width=0.024,
            slat_angle_deg=28.0,
            corner_radius=0.004,
            slats=VentGrilleSlats(profile="flat", direction="down"),
            sleeve=VentGrilleSleeve(style="none"),
        ),
        name,
    )


def _build_leaf(model: ArticulatedObject, part_name: str, wood, wood_dark, mirror: bool) -> None:
    """Build one batwing leaf as a part whose frame origin is at the OUTER edge.

    Part-local X runs from the outer (hinge) edge at x=0 toward the center. For
    door_0 (``mirror=False``) the body extends along +X; for door_1
    (``mirror=True``) it is the EXACT mirror, extending along -X, with the
    profile/holes/louver all flipped across X. The crown peaks at the inner
    (center) edge of BOTH leaves, so the closed pair forms one symmetric hump.
    The mirror is baked entirely into geometry, so door_1 is placed with NO yaw
    and both leaves keep the same front orientation -- a true mirror pair.
    """
    leaf = model.part(part_name)

    sign = -1.0 if mirror else 1.0  # mirrored leaf body extends along -X.

    # --- Scalloped wood frame slab with a louver opening cut through it ---
    # Profile is authored in XY (X=width centered, Y=height-from-bottom), then
    # rotated so profile-X -> world X, profile-Y -> world Z, extrude (Z) -> Y.
    # The leaf is a frame (stiles + rails + scalloped crown) around an open
    # louver bay; the louver panel below fills that bay and contacts the frame.
    profile = _scalloped_leaf_profile(LEAF_WIDTH, LEAF_BODY_HEIGHT, SCALLOP_RISE)
    if mirror:
        profile = [(-x, y) for (x, y) in profile]

    # Louver bay opening (centered band, inset by stiles/rails), in profile XY.
    bay_cx = (STILE_WIDTH + LOUVER_WIDTH / 2.0) - LEAF_WIDTH / 2.0  # rel. to center
    bay_cy = BOTTOM_RAIL + LOUVER_HEIGHT / 2.0
    # Make the cut a touch smaller than the louver panel so the panel laps onto
    # the surrounding wood frame (a real rabbet), keeping the leaf one solid.
    lap = 0.008
    half_w = LOUVER_WIDTH / 2.0 - lap
    half_h = LOUVER_HEIGHT / 2.0 - lap
    hole = [
        (bay_cx - half_w, bay_cy - half_h),
        (bay_cx + half_w, bay_cy - half_h),
        (bay_cx + half_w, bay_cy + half_h),
        (bay_cx - half_w, bay_cy + half_h),
    ]
    if mirror:
        hole = [(-x, y) for (x, y) in hole]

    slab_geo = ExtrudeWithHolesGeometry(profile, [hole], LEAF_THICKNESS, cap=True, center=True)
    slab_mesh = mesh_from_geometry(slab_geo, f"{part_name}_slab")
    leaf.visual(
        slab_mesh,
        origin=Origin(
            xyz=(sign * LEAF_WIDTH / 2.0, 0.0, LEAF_BOTTOM_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=wood,
        name=f"{part_name}_slab",
    )

    # --- Horizontal louver panel filling the bay; sized to overlap the frame ---
    louver_mesh = _louver_grille_mesh(f"{part_name}_louver")
    louver_cx = sign * (STILE_WIDTH + LOUVER_WIDTH / 2.0)
    louver_cz = LEAF_BOTTOM_Z + BOTTOM_RAIL + LOUVER_HEIGHT / 2.0
    # Rotate panel (local XY -> door XZ): roll +90 maps local +Y -> +Z,
    # local +Z (normal) -> -Y. Slats then run horizontally along X.
    leaf.visual(
        louver_mesh,
        origin=Origin(
            xyz=(louver_cx, 0.0, louver_cz),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=wood_dark,
        name=f"{part_name}_louver",
    )

    # --- Spring pivot barrel along the outer (hinge) edge ---
    leaf.visual(
        Cylinder(radius=PIVOT_RADIUS, length=LEAF_BODY_HEIGHT + SCALLOP_RISE),
        origin=Origin(xyz=(0.0, 0.0, LEAF_BOTTOM_Z + (LEAF_BODY_HEIGHT + SCALLOP_RISE) / 2.0)),
        material=wood_dark,
        name=f"{part_name}_pivot",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saloon_batwing_doors")

    wood = model.material("wood_warm", color=(0.74, 0.46, 0.24))
    wood_dark = model.material("wood_dark", color=(0.40, 0.24, 0.12))

    # --- Fixed frame (root): two jambs + top header ---
    frame = model.part("door_frame")
    half_open = OPENING_WIDTH / 2.0
    jamb_h = OPENING_HEIGHT
    for s, tag in ((-1.0, "left"), (1.0, "right")):
        frame.visual(
            Box((JAMB_WIDTH, JAMB_DEPTH, jamb_h)),
            origin=Origin(xyz=(s * (half_open - JAMB_WIDTH / 2.0), 0.0, jamb_h / 2.0)),
            material=wood_dark,
            name=f"{tag}_jamb",
        )
    # Top header spans across the top of the opening.
    frame.visual(
        Box((OPENING_WIDTH, JAMB_DEPTH, HEADER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT - HEADER_HEIGHT / 2.0)),
        material=wood_dark,
        name="header",
    )

    # --- Two batwing leaves ---
    _build_leaf(model, "door_0", wood, wood_dark, mirror=False)
    _build_leaf(model, "door_1", wood, wood_dark, mirror=True)

    # Hinge positions: just inside each jamb. Each leaf frame origin sits on its
    # pivot (outer edge). door_0 body extends +X toward the center; door_1 is the
    # mirrored leaf whose body extends -X toward the center. Both leaves are
    # placed with NO yaw -- the mirror lives entirely in geometry.
    inner_jamb_x = half_open - JAMB_WIDTH  # inner face of jamb
    # Pivots sit on the inner jamb faces; each leaf body reaches inward by
    # LEAF_WIDTH, leaving LOUVER_GAP at the center: 2*LEAF_WIDTH + LOUVER_GAP
    # == 2*inner_jamb_x, so the free edges meet with the intended center gap.
    pivot0_x = -inner_jamb_x  # door_0 hangs from the LEFT jamb.
    pivot1_x = inner_jamb_x  # door_1 hangs from the RIGHT jamb.

    # Spring pivots are double-acting: each leaf swings BOTH directions far past
    # 90 deg before the spring returns it to rest at 0.
    model.articulation(
        "frame_to_door_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=model.get_part("door_0"),
        origin=Origin(xyz=(pivot0_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "frame_to_door_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=model.get_part("door_1"),
        origin=Origin(xyz=(pivot1_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-1.2, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("door_frame")
    door0 = object_model.get_part("door_0")
    door1 = object_model.get_part("door_1")
    hinge0 = object_model.get_articulation("frame_to_door_0")
    hinge1 = object_model.get_articulation("frame_to_door_1")

    # --- Frame is the fixed root ---
    ctx.check(
        "frame_is_root",
        frame is not None and door0 is not None and door1 is not None,
        "Expected door_frame plus two leaves.",
    )

    # Intentional embeddings: spring pivots captured by the jambs, and the
    # louver panel lapped into the leaf frame rabbet.
    ctx.allow_overlap(
        door0,
        frame,
        elem_a="door_0_pivot",
        elem_b="left_jamb",
        reason="Spring pivot barrel is captured inside the left jamb.",
    )
    ctx.allow_overlap(
        door1,
        frame,
        elem_a="door_1_pivot",
        elem_b="right_jamb",
        reason="Spring pivot barrel is captured inside the right jamb.",
    )
    ctx.allow_overlap(
        door0,
        door0,
        elem_a="door_0_slab",
        elem_b="door_0_louver",
        reason="Louver panel laps into the leaf frame rabbet to read as seated.",
    )
    ctx.allow_overlap(
        door1,
        door1,
        elem_a="door_1_slab",
        elem_b="door_1_louver",
        reason="Louver panel laps into the leaf frame rabbet to read as seated.",
    )

    # --- Hero feature: scalloped louvered leaves present and sized ---
    for d in (door0, door1):
        aabb = ctx.part_world_aabb(d)
        assert aabb is not None
        mn, mx = aabb
        h = float(mx[2] - mn[2])
        w = float(mx[0] - mn[0])
        ctx.check(
            f"{d.name}_height_band",
            abs(h - (LEAF_BODY_HEIGHT + SCALLOP_RISE)) < 0.03,
            details=f"leaf height={h:.3f}",
        )
        ctx.check(
            f"{d.name}_width",
            abs(w - LEAF_WIDTH) < 0.03,
            details=f"leaf width={w:.3f}",
        )
        # Louver hero element is present.
        louver = d.get_visual(f"{d.name}_louver")
        ctx.check(f"{d.name}_has_louver", louver is not None, "Missing louver panel.")

    # --- Mirror symmetry: door_1 is the exact left-right mirror of door_0 ---
    # Matching elements must sit at the negated X and the SAME Y/Z, and BOTH
    # scalloped crowns must peak at the center meeting edge (not the jamb side).
    # This is the user's symmetry requirement for the pair.
    def _centroid(part, elem):
        a = ctx.part_element_world_aabb(part, elem=elem)
        assert a is not None, f"missing element {elem}"
        return (
            0.5 * (a[0][0] + a[1][0]),
            0.5 * (a[0][1] + a[1][1]),
            0.5 * (a[0][2] + a[1][2]),
        )

    with ctx.pose({hinge0: 0.0, hinge1: 0.0}):
        for tag in ("slab", "louver", "pivot"):
            c0 = _centroid(door0, f"door_0_{tag}")
            c1 = _centroid(door1, f"door_1_{tag}")
            ctx.check(
                f"{tag} mirrors across center (X negated)",
                abs(c1[0] + c0[0]) < 0.01,
                details=f"x0={c0[0]:.3f} x1={c1[0]:.3f}",
            )
            ctx.check(
                f"{tag} shares the same Y/Z on both leaves",
                abs(c1[1] - c0[1]) < 0.01 and abs(c1[2] - c0[2]) < 0.01,
                details=f"y0={c0[1]:.3f} y1={c1[1]:.3f} z0={c0[2]:.3f} z1={c1[2]:.3f}",
            )
    # Scalloped crown peaks at the INNER (center) edge: check the authored
    # profile directly (the deterministic geometric contract). The base profile
    # is taller at +half (inner) than at -half (outer); door_1 mirrors it so its
    # crown peaks at its own inner edge -- together the pair humps at the center.
    base_profile = _scalloped_leaf_profile(LEAF_WIDTH, LEAF_BODY_HEIGHT, SCALLOP_RISE)
    crown = [(x, y) for (x, y) in base_profile if y > LEAF_BODY_HEIGHT + 1e-6]
    peak_x = max(crown, key=lambda p: p[1])[0]
    ctx.check(
        "scalloped crown peaks at the inner (center) edge",
        peak_x > LEAF_WIDTH * 0.25,
        details=f"crown peak at x={peak_x:.3f} (half-width={LEAF_WIDTH / 2:.3f})",
    )

    # --- Gap above and below the leaves (batwing) ---
    # The leaves hang partway up the opening: there is an intentional GAP both
    # above (to the header) and below (kick clearance to the floor). This is the
    # defining batwing trait, NOT a floating part -- each leaf is supported by
    # the frame through its spring pivot (verified by expect_contact below), so
    # the gaps are legitimate clearance, not a disconnected assembly.
    aabb0 = ctx.part_world_aabb(door0)
    assert aabb0 is not None
    mn0, mx0 = aabb0
    ctx.check(
        "gap_below_leaf",
        float(mn0[2]) > 0.45,
        details=f"leaf bottom z={float(mn0[2]):.3f}",
    )
    ctx.check(
        "gap_above_leaf",
        float(mx0[2]) < OPENING_HEIGHT - HEADER_HEIGHT - 0.1,
        details=f"leaf top z={float(mx0[2]):.3f}",
    )

    # --- Symmetric double-acting limits ---
    for h in (hinge0, hinge1):
        lim = h.motion_limits
        ctx.check(
            f"{h.name}_double_acting",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and lim.lower < -0.3
            and lim.upper > 0.3
            and abs(lim.lower + lim.upper) < 1e-6,
            details=f"lower={lim.lower}, upper={lim.upper}",
        )

    # --- Closed pose: leaves nearly meet at center with a small gap, no overlap ---
    with ctx.pose({hinge0: 0.0, hinge1: 0.0}):
        ctx.expect_gap(
            door1,
            door0,
            axis="x",
            min_gap=0.0005,
            max_gap=0.04,
            name="leaves_meet_at_center",
        )
        # Hinge edges stay connected to the frame jambs.
        ctx.expect_contact(door0, frame, name="door_0_pivot_on_frame")
        ctx.expect_contact(door1, frame, name="door_1_pivot_on_frame")

    # --- Swings to BOTH sides (double-acting) ---
    rest0 = ctx.part_world_aabb(door0)
    assert rest0 is not None
    with ctx.pose({hinge0: 0.6}):
        pos_y = ctx.part_world_aabb(door0)
    with ctx.pose({hinge0: -0.6}):
        neg_y = ctx.part_world_aabb(door0)
    assert pos_y is not None and neg_y is not None
    # The free (inner) edge swings to +Y in one direction and -Y in the other.
    ctx.check(
        "door_0_swings_positive",
        float(pos_y[1][1]) > float(rest0[1][1]) + 0.05,
        details=f"+swing maxY={float(pos_y[1][1]):.3f} rest={float(rest0[1][1]):.3f}",
    )
    ctx.check(
        "door_0_swings_negative",
        float(neg_y[0][1]) < float(rest0[0][1]) - 0.05,
        details=f"-swing minY={float(neg_y[0][1]):.3f} rest={float(rest0[0][1]):.3f}",
    )

    return ctx.report()


object_model = build_object_model()
