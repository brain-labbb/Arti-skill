from __future__ import annotations

# Three-segment multi-compartment time-release capsule pill (~32 mm long).
# Three telescoping shell segments stacked end-to-end along the long axis (X),
# each sliding into the next on its own prismatic seam, forming a linear chain.
# segment_0 (root, white, smallest) → segment_1 (red, medium) → segment_2 (white, largest)
# Each seam is a PRISMATIC joint with uniform policy: same axis, limits, and travel.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters). Pharmaceutical time-release capsule.
# ---------------------------------------------------------------------------
N_SEGMENTS = 3
TUBE_LEN = 0.009             # cylindrical tube section per segment
BASE_R = 0.00370             # innermost segment outer radius (~7.4 mm dia)
WALL = 0.00030               # gelatin shell wall thickness
R_STEP = WALL                # radius step = wall thickness so child inner R = parent outer R
SEAT_OVERLAP = 0.003         # telescoping overlap at each seam
SEPARATION_TRAVEL = 0.008    # max prismatic travel per joint


def _segment_radius(i: int) -> float:
    """Outer radius of segment i."""
    return BASE_R + i * R_STEP


# Ground lift: capsule rests on the table supported by the largest segment.
GROUND_LIFT = _segment_radius(N_SEGMENTS - 1)


# ---------------------------------------------------------------------------
# Shared geometry helper: one capsule shell segment (revolved profile).
# ---------------------------------------------------------------------------
def _shell_half(outer_r: float, tube_len: float) -> cq.Workplane:
    """Hollow capsule segment built by revolving a profile around the X axis.

    Open mouth at x=0 (facing -X), tube extends toward +X, dome at the +X end.
    Using a revolved profile guarantees one connected solid of revolution,
    avoiding disconnected mesh islands from boolean tube+sphere unions.
    """
    inner_r = outer_r - WALL

    # Outer capsule solid: revolve a cross-section in the XZ plane around X axis.
    # Profile: mouth rim → tube wall → hemisphere → close along axis.
    outer_solid = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, outer_r)
        .lineTo(tube_len, outer_r)
        .radiusArc((tube_len + outer_r, 0.0), -outer_r)
        .close()
        .revolve(360, (0, 0, 0), (1, 0, 0))
    )

    # Inner cavity: same profile inset by wall thickness, open at mouth.
    # Extends past mouth (-X) so the subtraction leaves a clean open rim.
    cavity = (
        cq.Workplane("XZ")
        .moveTo(-WALL, 0.0)
        .lineTo(-WALL, inner_r)
        .lineTo(tube_len, inner_r)
        .radiusArc((tube_len + inner_r, 0.0), -inner_r)
        .close()
        .revolve(360, (0, 0, 0), (1, 0, 0))
    )

    return outer_solid.cut(cavity)


# ---------------------------------------------------------------------------
# Imprint helper: embossed "82" on the middle segment's tube surface.
# ---------------------------------------------------------------------------
def _imprint_82(outer_r: float, tube_len: float) -> cq.Workplane:
    """Embossed '82' on the tube surface at x=tube_len/2, +Y side.

    Authored in the segment's standard frame: mouth at x=0, tube extends +X.
    """
    x_center = tube_len * 0.5
    surf_y = outer_r
    proud = 0.00012
    t = 0.00018
    depth = WALL + proud

    def bar(cx: float, cz: float, lx: float, lz: float) -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .workplane(offset=surf_y - WALL * 0.5)
            .center(cx, cz)
            .rect(lx, lz)
            .extrude(depth)
        )

    ring_w = 0.0013
    eight_cx = x_center - 0.0013
    bars = [
        bar(eight_cx, 0.0010, ring_w, t),
        bar(eight_cx, 0.0, ring_w, t),
        bar(eight_cx, -0.0010, ring_w, t),
        bar(eight_cx - ring_w / 2 + t / 2, 0.0005, t, 0.0012),
        bar(eight_cx + ring_w / 2 - t / 2, 0.0005, t, 0.0012),
        bar(eight_cx - ring_w / 2 + t / 2, -0.0005, t, 0.0012),
        bar(eight_cx + ring_w / 2 - t / 2, -0.0005, t, 0.0012),
    ]

    two_cx = x_center + 0.0013
    two_w = 0.0013
    bars += [
        bar(two_cx, 0.0010, two_w, t),
        bar(two_cx + two_w / 2 - t / 2, 0.0005, t, 0.0012),
        bar(two_cx, 0.0, two_w, t),
        bar(two_cx - two_w / 2 + t / 2, -0.0005, t, 0.0012),
        bar(two_cx, -0.0010, two_w, t),
    ]

    # Backing plate fuses all bars into one solid island on the shell surface.
    backing = (
        cq.Workplane("XZ")
        .workplane(offset=surf_y - WALL * 0.65)
        .center(x_center, 0.0)
        .rect(0.0042, 0.0026)
        .extrude(WALL * 0.7)
    )

    glyphs = backing
    for b in bars:
        glyphs = glyphs.union(b)
    # XZ extrude grows toward -Y; mirror to +Y surface.
    return glyphs.mirror("XZ")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="capsule_pill")

    gel_white = model.material("gelatin_white", rgba=(0.95, 0.95, 0.94, 1.0))
    gel_red = model.material("gelatin_red", rgba=(0.86, 0.10, 0.10, 1.0))
    imprint_mat = model.material("imprint_ink", rgba=(0.78, 0.05, 0.05, 1.0))

    # Banded red-and-white coloring: white, red, white.
    seg_colors = [gel_white, gel_red, gel_white]

    # --- Build segments via for loop with shared shell helper ---------------
    # All segments: mouth at x=0 (facing -X), tube +X, dome at +X end.
    # Arranged left to right: seg0, seg1, seg2.
    # Each child's mouth telescopes over the parent's dome end.
    segments = []
    for i in range(N_SEGMENTS):
        r = _segment_radius(i)
        seg = model.part(f"segment_{i}")

        shell = _shell_half(r, TUBE_LEN)

        # Root visual needs ground-lift offset; children ride at parent axis height.
        vis_origin = Origin(xyz=(0.0, 0.0, GROUND_LIFT)) if i == 0 else Origin()

        seg.visual(
            mesh_from_cadquery(shell, f"segment_{i}_shell"),
            origin=vis_origin,
            material=seg_colors[i],
            name=f"segment_{i}_shell",
        )

        # Embossed imprint on the middle (red) segment only.
        if i == 1:
            imprint = _imprint_82(r, TUBE_LEN)
            seg.visual(
                mesh_from_cadquery(imprint, f"segment_{i}_imprint"),
                material=imprint_mat,
                name=f"segment_{i}_imprint",
            )

        seg.inertial = Inertial.from_geometry(
            Box((TUBE_LEN + r, 2 * r, 2 * r)),
            mass=0.0003,
            origin=Origin(
                xyz=(
                    (TUBE_LEN + r) / 2.0,
                    0.0,
                    GROUND_LIFT if i == 0 else 0.0,
                )
            ),
        )

        segments.append(seg)

    # --- Prismatic joints (uniform policy: same type, axis, limits) ---------
    # Joint origin at the seam contact surface in the parent's frame.
    # At q=0 the child's mouth (x=0 in child frame) is at the joint origin.
    # The child's tube then overlaps the parent's tube near the dome end.
    for i in range(N_SEGMENTS - 1):
        # In the parent frame, the tube extends from x=0 to x=TUBE_LEN.
        # The child's mouth is placed at x = TUBE_LEN - SEAT_OVERLAP,
        # so the child tube overlaps the parent tube by SEAT_OVERLAP.
        # Z: segment_0 has its visual at GROUND_LIFT; children inherit via joint chain.
        jz = GROUND_LIFT if i == 0 else 0.0

        model.articulation(
            f"segment_{i}_to_segment_{i+1}",
            ArticulationType.PRISMATIC,
            parent=segments[i],
            child=segments[i + 1],
            origin=Origin(xyz=(TUBE_LEN - SEAT_OVERLAP, 0.0, jz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=SEPARATION_TRAVEL,
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    segments = [object_model.get_part(f"segment_{i}") for i in range(N_SEGMENTS)]
    joints = [
        object_model.get_articulation(f"segment_{i}_to_segment_{i+1}")
        for i in range(N_SEGMENTS - 1)
    ]

    # --- Joint type, axis, and limits (uniform policy) ----------------------
    for i, j in enumerate(joints):
        ctx.check(
            f"segment_{i}_to_segment_{i+1} is prismatic",
            j.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={j.articulation_type}",
        )
        ax = tuple(round(v, 6) for v in j.axis)
        ctx.check(
            f"segment_{i}_to_segment_{i+1} slides along capsule axis (X)",
            ax == (1.0, 0.0, 0.0),
            details=f"axis={ax}",
        )
        lim = j.motion_limits
        ctx.check(
            f"segment_{i}_to_segment_{i+1} has travel limits",
            lim is not None and lim.lower == 0.0 and lim.upper == SEPARATION_TRAVEL,
            details=f"limits=({lim.lower if lim else None},{lim.upper if lim else None})",
        )

    # --- 3 segments present with shell visuals ------------------------------
    for i, seg in enumerate(segments):
        ctx.check(
            f"segment_{i}_shell visual present",
            any(v.name == f"segment_{i}_shell" for v in seg.visuals),
            details=str([v.name for v in seg.visuals]),
        )

    ctx.check(
        "imprint present on segment_1",
        any(v.name == "segment_1_imprint" for v in segments[1].visuals),
        details=str([v.name for v in segments[1].visuals]),
    )

    # --- Telescoping overlap allowances (intentional nesting) ---------------
    # Each child (larger radius) telescopes over the parent's dome end.
    for i in range(N_SEGMENTS - 1):
        ctx.allow_overlap(
            segments[i + 1],
            segments[i],
            elem_a=f"segment_{i+1}_shell",
            elem_b=f"segment_{i}_shell",
            reason=(
                f"segment_{i+1} (larger radius) intentionally telescopes over "
                f"segment_{i}'s dome end at the prismatic seam."
            ),
        )

    ctx.allow_overlap(
        segments[1],
        segments[1],
        elem_a="segment_1_imprint",
        elem_b="segment_1_shell",
        reason="The imprint is intentionally embedded into the segment_1 shell surface.",
    )

    # --- Seated pose (q=0): retained insertion and concentricity -----------
    seated_positions = {}
    with ctx.pose({j: 0.0 for j in joints}):
        for i in range(N_SEGMENTS - 1):
            ctx.expect_overlap(
                segments[i + 1],
                segments[i],
                axes="x",
                elem_a=f"segment_{i+1}_shell",
                elem_b=f"segment_{i}_shell",
                min_overlap=0.001,
                name=f"seated segment_{i+1} overlaps segment_{i} along slide axis",
            )
            ctx.expect_within(
                segments[i],
                segments[i + 1],
                axes="yz",
                inner_elem=f"segment_{i}_shell",
                outer_elem=f"segment_{i+1}_shell",
                margin=0.0004,
                name=f"segment_{i} stays concentric inside segment_{i+1} when seated",
            )
        for i in range(N_SEGMENTS):
            seated_positions[i] = ctx.part_world_position(segments[i])

    # --- Extended pose: each child separates rightward (+X) -----------------
    with ctx.pose({j: j.motion_limits.upper for j in joints}):
        for i in range(N_SEGMENTS - 1):
            extended_child = ctx.part_world_position(segments[i + 1])
            seated_child = seated_positions[i + 1]
            ctx.check(
                f"segment_{i+1} extends rightward from segment_{i} at max travel",
                extended_child is not None
                and seated_child is not None
                and extended_child[0] > seated_child[0] + 0.003,
                details=f"seated={seated_child}, extended={extended_child}",
            )

        # Verify segment_1 fully separates from segment_0 along X.
        ctx.expect_gap(
            segments[1],
            segments[0],
            axis="x",
            min_gap=0.0,
            positive_elem="segment_1_shell",
            negative_elem="segment_0_shell",
            name="segment_1 separates from segment_0 at max travel",
        )

    return ctx.report()


object_model = build_object_model()
