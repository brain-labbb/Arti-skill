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
# Saloon-style arched cafe double doors.
#
# Frame (fixed root) is a dark-wood doorway: two side jambs + a top header,
# spanning an opening ~0.95 m wide and ~2.0 m tall. The two wood leaves cover
# the middle band of the opening (a gap above and below), and each leaf is
# hung from the frame by a vertical spring pivot so it swings BOTH directions
# (double-acting).
#
# Each leaf has a semicircular arched top. When both leaves are closed, their
# quarter-circle arches combine to form one complete round arch spanning the
# clear opening width.
#
# Axes: X = doorway width (left/right), Y = door thickness / swing depth,
#       Z = height. Hinge axis is vertical (+Z). Door faces lie in the XZ plane.
#
# Mirror symmetry: ONE leaf is authored with its hinge edge at part-local x=0,
# the body extending toward the center along +X, and the arch peaking at the
# INNER (center) edge. door_1 is the EXACT mirror of door_0: the slab/louver/
# pivot are mirrored across X (so its body extends along -X toward the center
# and its arch also peaks at the center) and the leaf is placed with NO yaw.
# This guarantees a true left-right mirror when closed, forming one symmetric
# round arch.
# ---------------------------------------------------------------------------

# Opening / frame dimensions.
OPENING_WIDTH = 0.95
OPENING_HEIGHT = 2.0
JAMB_WIDTH = 0.07  # left/right jamb thickness in X
JAMB_DEPTH = 0.12  # frame depth in Y
HEADER_HEIGHT = 0.09  # top header thickness in Z

# Clear opening between the jambs.
CLEAR_WIDTH = OPENING_WIDTH - 2.0 * JAMB_WIDTH
LOUVER_GAP = 0.006  # gap between the two leaves at the center meeting stiles

# Each leaf width: split the clear opening, leaving a small center gap.
LEAF_WIDTH = (CLEAR_WIDTH - LOUVER_GAP) / 2.0

# Leaf dimensions. The two leaves cover the middle band of the opening.
LEAF_BOTTOM_Z = 0.62  # gap below (kick clearance)
LEAF_BODY_HEIGHT = 0.82  # straight rectangular body height of one leaf
ARCH_RISE = CLEAR_WIDTH / 2.0  # semicircular arch radius (= half clear span)
LEAF_TOP_Z = LEAF_BOTTOM_Z + LEAF_BODY_HEIGHT + ARCH_RISE
LEAF_THICKNESS = 0.035  # door slab thickness along Y

# Louver band on the leaf (horizontal slats fill most of the body).
STILE_WIDTH = 0.05  # vertical side rails of the leaf
TOP_RAIL = 0.04  # rail just under the arched crown
BOTTOM_RAIL = 0.05  # bottom rail
LOUVER_WIDTH = LEAF_WIDTH - 2.0 * STILE_WIDTH
LOUVER_HEIGHT = LEAF_BODY_HEIGHT - TOP_RAIL - BOTTOM_RAIL

# Hinge geometry: spring pivot at the OUTER edge of each leaf, near the jamb.
PIVOT_RADIUS = 0.012


def _arched_leaf_profile(width: float, body_h: float, arch_radius: float,
                          louver_gap: float) -> list[tuple[float, float]]:
    """Closed XY profile of one leaf slab face: flat sides/bottom, semicircular arched top.

    Built centered in X around 0, bottom at y=0. The arch is a quarter-circle
    on top of the rectangular body. The arch center sits at the midpoint of the
    full clear opening, which is just past the inner (free) edge of this leaf.
    When both leaves are closed, their quarter-circles join to form one full
    semicircular arch.

    For door_0 (non-mirrored): +half is the inner (center) edge where the arch
    is tallest. For door_1 (mirrored via x -> -x), the arch peaks at -half,
    which is also the inner edge in world coordinates.
    """
    half = width / 2.0
    # Arch center in profile-local X: the midpoint of the clear opening is
    # louver_gap/2 past this leaf's inner edge.
    arch_cx = half + louver_gap / 2.0
    R = arch_radius

    pts: list[tuple[float, float]] = []
    # Bottom edge, outer (left) to inner (right).
    pts.append((-half, 0.0))
    pts.append((half, 0.0))
    # Inner (right) side up to the spring line.
    pts.append((half, body_h))
    # Semicircular arch sampled from the inner edge across to the outer edge.
    # At the inner edge, dx ≈ louver_gap/2 so arch_y ≈ R.
    # At the outer edge, dx = R so arch_y = 0 (meets the spring line).
    n = 36
    for i in range(n + 1):
        t = i / n  # 0 at inner (+half) edge, 1 at outer (-half) edge
        x = half - t * width
        dx = arch_cx - x  # horizontal distance from arch center
        if dx <= R:
            arch_y = math.sqrt(max(0.0, R * R - dx * dx))
        else:
            arch_y = 0.0
        pts.append((x, body_h + arch_y))
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
    """Build one arched leaf as a part whose frame origin is at the OUTER edge.

    Part-local X runs from the outer (hinge) edge at x=0 toward the center. For
    door_0 (``mirror=False``) the body extends along +X; for door_1
    (``mirror=True``) it is the EXACT mirror, extending along -X, with the
    profile/holes/louver all flipped across X. The arch peaks at the inner
    (center) edge of BOTH leaves, so the closed pair forms one round arch.
    The mirror is baked entirely into geometry, so door_1 is placed with NO yaw
    and both leaves keep the same front orientation -- a true mirror pair.
    """
    leaf = model.part(part_name)

    sign = -1.0 if mirror else 1.0  # mirrored leaf body extends along -X.

    # --- Arched wood frame slab with a louver opening cut through it ---
    # Profile is authored in XY (X=width centered, Y=height-from-bottom), then
    # rotated so profile-X -> world X, profile-Y -> world Z, extrude (Z) -> Y.
    # The leaf is a frame (stiles + rails + arched crown) around an open
    # louver bay; the louver panel below fills that bay and contacts the frame.
    profile = _arched_leaf_profile(LEAF_WIDTH, LEAF_BODY_HEIGHT, ARCH_RISE, LOUVER_GAP)
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
    # At the outer (hinge) edge the arch contributes zero extra height (it is
    # at radius R from the arch center), so the pivot spans only the body.
    leaf.visual(
        Cylinder(radius=PIVOT_RADIUS, length=LEAF_BODY_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, LEAF_BOTTOM_Z + LEAF_BODY_HEIGHT / 2.0)),
        material=wood_dark,
        name=f"{part_name}_pivot",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saloon_arched_doors")

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

    # --- Two arched leaves ---
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

    # --- Hero feature: arched louvered leaves present and sized ---
    expected_leaf_height = LEAF_BODY_HEIGHT + ARCH_RISE
    for d in (door0, door1):
        aabb = ctx.part_world_aabb(d)
        assert aabb is not None
        mn, mx = aabb
        h = float(mx[2] - mn[2])
        w = float(mx[0] - mn[0])
        ctx.check(
            f"{d.name}_height_band",
            abs(h - expected_leaf_height) < 0.05,
            details=f"leaf height={h:.3f}, expected~{expected_leaf_height:.3f}",
        )
        ctx.check(
            f"{d.name}_width",
            abs(w - LEAF_WIDTH) < 0.03,
            details=f"leaf width={w:.3f}",
        )
        # Louver hero element is present.
        louver = d.get_visual(f"{d.name}_louver")
        ctx.check(f"{d.name}_has_louver", louver is not None, "Missing louver panel.")

    # --- Arched top profile verification ---
    # The arch is a semicircle: verify the profile peaks at the inner (center)
    # edge and the peak height matches body_h + R.
    base_profile = _arched_leaf_profile(LEAF_WIDTH, LEAF_BODY_HEIGHT, ARCH_RISE, LOUVER_GAP)
    crown = [(x, y) for (x, y) in base_profile if y > LEAF_BODY_HEIGHT + 1e-6]
    peak_pt = max(crown, key=lambda p: p[1])
    peak_x = peak_pt[0]
    peak_y = peak_pt[1]
    ctx.check(
        "arch peaks at the inner (center) edge",
        peak_x > LEAF_WIDTH * 0.25,
        details=f"crown peak at x={peak_x:.3f} (half-width={LEAF_WIDTH / 2:.3f})",
    )
    ctx.check(
        "arch peak height matches semicircular radius",
        abs(peak_y - (LEAF_BODY_HEIGHT + ARCH_RISE)) < 0.02,
        details=f"peak y={peak_y:.3f}, expected={LEAF_BODY_HEIGHT + ARCH_RISE:.3f}",
    )
    # Outer edge of arch approaches the spring line. The profile's last point
    # is exactly at the spring line (y = body_h), but the crown filter only
    # includes points strictly above body_h, so the leftmost crown point is
    # the penultimate sample where arch_y is small but nonzero.
    outermost = min(crown, key=lambda p: p[0])
    ctx.check(
        "arch drops toward spring line at outer edge",
        outermost[1] < LEAF_BODY_HEIGHT + ARCH_RISE * 0.30,
        details=f"outermost crown y={outermost[1]:.3f}, body_h={LEAF_BODY_HEIGHT:.3f}",
    )

    # --- Mirror symmetry: door_1 is the exact left-right mirror of door_0 ---
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

    # --- Closed pair forms one round arch ---
    # When closed, the two leaf tops should span the clear opening width and
    # peak at the center. Verify the combined slab tops reach approximately the
    # same maximum height at the center meeting edge.
    with ctx.pose({hinge0: 0.0, hinge1: 0.0}):
        aabb0 = ctx.part_element_world_aabb(door0, elem="door_0_slab")
        aabb1 = ctx.part_element_world_aabb(door1, elem="door_1_slab")
        assert aabb0 is not None and aabb1 is not None
        # Both slabs reach approximately the same max Z (arch peak at center).
        max_z0 = float(aabb0[1][2])
        max_z1 = float(aabb1[1][2])
        ctx.check(
            "both arches reach similar peak height",
            abs(max_z0 - max_z1) < 0.02,
            details=f"door_0 peak z={max_z0:.3f}, door_1 peak z={max_z1:.3f}",
        )
        # Peak is substantially above the spring line (proves arch exists).
        spring_z = LEAF_BOTTOM_Z + LEAF_BODY_HEIGHT
        ctx.check(
            "arch rises substantially above spring line",
            max_z0 > spring_z + ARCH_RISE * 0.8,
            details=f"peak z={max_z0:.3f}, spring z={spring_z:.3f}, arch_rise={ARCH_RISE:.3f}",
        )

    # --- Gap above and below the leaves ---
    # The leaves hang partway up the opening: there is an intentional GAP both
    # above (to the header) and below (kick clearance to the floor). Each leaf
    # is supported by the frame through its spring pivot (verified by
    # expect_contact below), so the gaps are legitimate clearance.
    aabb0_full = ctx.part_world_aabb(door0)
    assert aabb0_full is not None
    mn0, mx0 = aabb0_full
    ctx.check(
        "gap_below_leaf",
        float(mn0[2]) > 0.45,
        details=f"leaf bottom z={float(mn0[2]):.3f}",
    )
    ctx.check(
        "gap_above_leaf",
        float(mx0[2]) < OPENING_HEIGHT - HEADER_HEIGHT - 0.02,
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
