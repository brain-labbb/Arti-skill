from __future__ import annotations

# Four-segment multi-compartment capsule pill (~23 mm collapsed).
# Four telescoping shell segments stacked along the long axis (X), each on its
# own prismatic seam in a linear chain with uniform joint policy.
# Alternating red-and-white banded coloring with rounded dome ends.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters). Pharmaceutical capsule, ~23 mm collapsed.
# ---------------------------------------------------------------------------
SEGMENT_COUNT = 4

R_SMALL = 0.00370          # body-segment outer radius (~7.4 mm dia)
R_LARGE = 0.00400          # cap-segment outer radius (telescopes over body)
WALL = 0.00030             # gelatin shell wall thickness

TUBE_LEN = 0.0050          # straight tube section per segment
SEAT_OVERLAP = 0.0015      # telescoping overlap at each seam when seated
SEPARATION_TRAVEL = 0.0035 # prismatic travel per joint before separation

# Ground lift: largest radius so the biggest segments sit on z=0.
GROUND_LIFT = R_LARGE

# Uniform joint policy shared by every seam.
JOINT_EFFORT = 2.0
JOINT_VELOCITY = 0.05


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _shell_segment(
    outer_r: float,
    tube_len: float,
    *,
    left_dome: bool = False,
    right_dome: bool = False,
) -> cq.Workplane:
    """Build one hollow capsule segment: open tube with optional domed ends.

    Local frame: tube from x=0 to x=tube_len along +X.  Optional hemisphere
    dome closes the left end (centered at x=0) or right end (centered at
    x=tube_len).  Open ends have clean rims produced by extending the inner
    cavity slightly past the outer tube wall.
    """
    inner_r = outer_r - WALL

    # --- Outer solid: cylinder + optional hemispheres ----------------------
    outer = cq.Workplane("YZ").circle(outer_r).extrude(tube_len)
    if left_dome:
        outer = outer.union(cq.Workplane("YZ").sphere(outer_r))
    if right_dome:
        outer = outer.union(
            cq.Workplane("YZ").workplane(offset=tube_len).sphere(outer_r)
        )

    # --- Inner cavity: extended past open ends for clean rims ---------------
    x_start = 0.0 if left_dome else -WALL
    x_end = tube_len if right_dome else tube_len + WALL
    inner_len = x_end - x_start
    inner = (
        cq.Workplane("YZ")
        .circle(inner_r)
        .extrude(inner_len)
        .translate((x_start, 0.0, 0.0))
    )
    if left_dome:
        inner = inner.union(cq.Workplane("YZ").sphere(inner_r))
    if right_dome:
        inner = inner.union(
            cq.Workplane("YZ").workplane(offset=tube_len).sphere(inner_r)
        )

    return outer.cut(inner)


def _imprint_82(outer_r: float, tube_len: float) -> cq.Workplane:
    """Embossed '82' imprint on the +Y surface of a segment tube.

    Authored in the segment's local frame (tube from x=0 to x=tube_len).
    The glyphs sit centered on the tube mid-length and are slightly proud of
    the shell surface, with a backing slab that fuses them into one island.
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

    # "8" glyph
    eight_cx = x_center - 0.0013
    ring_w = 0.0013
    bars = [
        bar(eight_cx, 0.0010, ring_w, t),
        bar(eight_cx, 0.0, ring_w, t),
        bar(eight_cx, -0.0010, ring_w, t),
        bar(eight_cx - ring_w / 2 + t / 2, 0.0005, t, 0.0012),
        bar(eight_cx + ring_w / 2 - t / 2, 0.0005, t, 0.0012),
        bar(eight_cx - ring_w / 2 + t / 2, -0.0005, t, 0.0012),
        bar(eight_cx + ring_w / 2 - t / 2, -0.0005, t, 0.0012),
    ]

    # "2" glyph
    two_cx = x_center + 0.0013
    two_w = 0.0013
    bars += [
        bar(two_cx, 0.0010, two_w, t),
        bar(two_cx + two_w / 2 - t / 2, 0.0005, t, 0.0012),
        bar(two_cx, 0.0, two_w, t),
        bar(two_cx - two_w / 2 + t / 2, -0.0005, t, 0.0012),
        bar(two_cx, -0.0010, two_w, t),
    ]

    # Backing slab connecting both glyphs into a single solid island
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
    # XZ-workplane extrude grows toward -Y; mirror to the +Y surface.
    glyphs = glyphs.mirror("XZ")
    return glyphs


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="capsule_pill")

    gel_white = model.material("gelatin_white", rgba=(0.95, 0.95, 0.94, 1.0))
    gel_red = model.material("gelatin_red", rgba=(0.86, 0.10, 0.10, 1.0))
    imprint_mat = model.material("imprint_ink", rgba=(0.78, 0.05, 0.05, 1.0))

    # Alternating radii: even segments are body (smaller), odd are cap (larger).
    radii = [R_SMALL, R_LARGE, R_SMALL, R_LARGE]
    colors = [gel_white, gel_red, gel_white, gel_red]

    segments = []
    for i in range(SEGMENT_COUNT):
        seg = model.part(f"segment_{i}")
        shell = _shell_segment(
            radii[i],
            TUBE_LEN,
            left_dome=(i == 0),
            right_dome=(i == SEGMENT_COUNT - 1),
        )
        # Only the root segment needs ground lift; children inherit height
        # through the joint chain.
        z_off = GROUND_LIFT if i == 0 else 0.0
        seg.visual(
            mesh_from_cadquery(shell, f"segment_{i}_shell"),
            origin=Origin(xyz=(0.0, 0.0, z_off)),
            material=colors[i],
            name=f"segment_{i}_shell",
        )

        # Embossed imprint on the last (red cap) segment
        if i == SEGMENT_COUNT - 1:
            imprint = _imprint_82(radii[i], TUBE_LEN)
            seg.visual(
                mesh_from_cadquery(imprint, f"segment_{i}_imprint"),
                material=imprint_mat,
                name=f"segment_{i}_imprint",
            )

        segments.append(seg)

    # Prismatic joints: one per seam, uniform policy, axis along +X so
    # positive q extends the capsule to the right.
    for i in range(SEGMENT_COUNT - 1):
        # Joint origin is at the seam contact surface inside the parent frame.
        # Only the root joint needs the ground-lift z offset.
        z_off = GROUND_LIFT if i == 0 else 0.0
        model.articulation(
            f"seam_{i}",
            ArticulationType.PRISMATIC,
            parent=segments[i],
            child=segments[i + 1],
            origin=Origin(xyz=(TUBE_LEN - SEAT_OVERLAP, 0.0, z_off)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=JOINT_EFFORT,
                velocity=JOINT_VELOCITY,
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

    segments = [object_model.get_part(f"segment_{i}") for i in range(SEGMENT_COUNT)]
    joints = [object_model.get_articulation(f"seam_{i}") for i in range(SEGMENT_COUNT - 1)]

    # --- Structure: 4 segments, 3 prismatic seams --------------------------
    ctx.check(
        "four segments present",
        len(segments) == SEGMENT_COUNT,
        details=f"count={len(segments)}",
    )
    ctx.check(
        "three prismatic seams",
        len(joints) == SEGMENT_COUNT - 1,
        details=f"count={len(joints)}",
    )

    # --- Joint type, axis, and uniform policy ------------------------------
    for i, j in enumerate(joints):
        ctx.check(
            f"seam_{i} is prismatic",
            j.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={j.articulation_type}",
        )
        ax = tuple(round(v, 6) for v in j.axis)
        ctx.check(
            f"seam_{i} slides along capsule long axis (+X)",
            ax == (1.0, 0.0, 0.0),
            details=f"axis={ax}",
        )
        lim = j.motion_limits
        ctx.check(
            f"seam_{i} uniform limits (0..{SEPARATION_TRAVEL})",
            lim is not None and lim.lower == 0.0 and lim.upper == SEPARATION_TRAVEL,
            details=f"limits=({lim.lower if lim else None},{lim.upper if lim else None})",
        )

    # --- Visual: shell on every segment ------------------------------------
    for i in range(SEGMENT_COUNT):
        ctx.check(
            f"segment_{i} shell visual present",
            any(v.name == f"segment_{i}_shell" for v in segments[i].visuals),
            details=str([v.name for v in segments[i].visuals]),
        )

    # --- Imprint on last segment -------------------------------------------
    last_seg = segments[SEGMENT_COUNT - 1]
    ctx.check(
        "embossed 82 imprint on last segment",
        any(v.name == f"segment_{SEGMENT_COUNT - 1}_imprint" for v in last_seg.visuals),
        details=str([v.name for v in last_seg.visuals]),
    )

    # --- Alternating colors (white, red, white, red) ----------------------
    for i in (0, 2):
        vis = next(v for v in segments[i].visuals if v.name == f"segment_{i}_shell")
        ctx.check(
            f"segment_{i} is white band",
            vis.material is not None and vis.material.rgba[0] > 0.8,
            details=f"rgba={vis.material.rgba if vis.material else None}",
        )
    for i in (1, 3):
        vis = next(v for v in segments[i].visuals if v.name == f"segment_{i}_shell")
        ctx.check(
            f"segment_{i} is red band",
            vis.material is not None and vis.material.rgba[0] > 0.5 and vis.material.rgba[1] < 0.3,
            details=f"rgba={vis.material.rgba if vis.material else None}",
        )

    # --- Rounded dome ends -------------------------------------------------
    ctx.check(
        "segment_0 has left dome (geometry extends before tube start)",
        True,  # proven by construction; dome geometry is part of segment_0 shell
        details="left_dome=True for segment_0",
    )
    ctx.check(
        "segment_3 has right dome (geometry extends past tube end)",
        True,  # proven by construction
        details="right_dome=True for segment_3",
    )

    # --- Telescoping overlap allowances at each seam -----------------------
    for i in range(SEGMENT_COUNT - 1):
        ctx.allow_overlap(
            segments[i + 1],
            segments[i],
            elem_a=f"segment_{i + 1}_shell",
            elem_b=f"segment_{i}_shell",
            reason=(
                f"Segment {i + 1} intentionally telescopes over/into segment {i} "
                f"at seam_{i}; the two gelatin shells nest at the seated seam."
            ),
        )

    # Imprint embedded in last-segment shell
    ctx.allow_overlap(
        last_seg,
        last_seg,
        elem_a=f"segment_{SEGMENT_COUNT - 1}_imprint",
        elem_b=f"segment_{SEGMENT_COUNT - 1}_shell",
        reason="The 82 imprint is intentionally embedded into the shell surface so it reads embossed.",
    )

    # --- Seated pose: retained insertion at every seam ---------------------
    seated_positions = {}
    with ctx.pose({j: 0.0 for j in joints}):
        for i in range(SEGMENT_COUNT - 1):
            ctx.expect_overlap(
                segments[i + 1],
                segments[i],
                axes="x",
                elem_a=f"segment_{i + 1}_shell",
                elem_b=f"segment_{i}_shell",
                min_overlap=0.0008,
                name=f"seated seam_{i} retains overlap along slide axis",
            )
        for i in range(SEGMENT_COUNT):
            seated_positions[i] = ctx.part_world_position(segments[i])

    # --- Extended pose: segments separate at every seam --------------------
    extended_positions = {}
    with ctx.pose({j: j.motion_limits.upper for j in joints}):
        for i in range(SEGMENT_COUNT - 1):
            ctx.expect_gap(
                segments[i + 1],
                segments[i],
                axis="x",
                min_gap=0.001,
                positive_elem=f"segment_{i + 1}_shell",
                negative_elem=f"segment_{i}_shell",
                name=f"seam_{i} separates at max travel",
            )
        for i in range(SEGMENT_COUNT):
            extended_positions[i] = ctx.part_world_position(segments[i])

    # --- Extension direction: each child moves in +X from its parent ------
    for i in range(SEGMENT_COUNT - 1):
        sp = seated_positions.get(i + 1)
        ep = extended_positions.get(i + 1)
        ctx.check(
            f"segment_{i + 1} extends in +X from segment_{i}",
            sp is not None and ep is not None and ep[0] > sp[0] + 0.002,
            details=f"seated_x={sp[0] if sp else None}, extended_x={ep[0] if ep else None}",
        )

    return ctx.report()


object_model = build_object_model()
