from __future__ import annotations

# Bi-fold accordion security gate – flat-panel concertina variant.
#
# A fixed rectangular steel frame (root) carries a row of flat solid bi-fold
# accordion panels (thin rectangular leaves) that zigzag-fold to one side like
# a concertina. Each panel is hinged to its neighbour by a vertical revolute
# pivot at the panel edge; alternating hinges fold in opposite directions so
# the row compresses into a tight zigzag stack against the left stile.
#
# A single REVOLUTE "fold" drive on the first hinge powers the whole gate:
# every subsequent hinge is mimic-coupled with an alternating ±2 multiplier,
# so the absolute panel angles alternate between +q and -q and the panels
# zigzag symmetrically between the frame plane and a forward offset.
#
# Articulation tree:
#   frame --FIXED--> panel_0 --fold(+q)--> panel_1 --hinge_1(-2q)-->
#       panel_2 --hinge_2(+2q)--> panel_3 -- ... -- panel_{N-1}
#
# Visual layout follows the reference: ~10 flat terracotta-salmon panel
# leaves with dark-steel edge rails and hinge barrels, a fixed frame with a
# wide left stile, slim top and bottom rails with guide tracks, and a small
# latch block mid-height on the right stile. q=0 is the fully DEPLOYED gate
# (hero pose) spanning the opening; positive drive folds the panels to the
# left stile in a zigzag stack.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---------------------------------------------------------------------------
# Geometry constants (meters)
# ---------------------------------------------------------------------------

FRAME_W = 1.20          # overall frame width (X)
FRAME_H = 2.00          # overall frame height (Z); gate stands on z=0
FRAME_D = 0.060         # frame depth (Y)
STILE_W = 0.060         # vertical frame member width (X)
RAIL_H = 0.060          # horizontal frame member height (Z)

OPEN_X0 = STILE_W                 # 0.060
OPEN_X1 = FRAME_W - STILE_W       # 1.140
OPEN_Z0 = RAIL_H                  # 0.060
OPEN_Z1 = FRAME_H - RAIL_H        # 1.940

TRACK_T = 0.018

# --- Accordion panels ---
N_PANELS = 10
PANEL_W = 0.105         # panel leaf width (X)
PANEL_T = 0.012         # panel leaf thickness (Y)
PANEL_H = OPEN_Z1 - OPEN_Z0 - 0.040   # 1.840 (clearance for top/bottom tracks)
LEAD_X = 0.075          # x of the leading panel hinge line
PANEL_CZ = (OPEN_Z0 + OPEN_Z1) * 0.5  # 1.0

# Hinge barrel at each inter-panel seam (continuous piano hinge proxy).
HINGE_R = 0.005
HINGE_LEN = PANEL_H * 0.85

# Edge rail dimensions (structural frame on each panel leaf).
RAIL_W = PANEL_W - 0.006
RAIL_T = PANEL_T + 0.006
RAIL_H_PANEL = 0.022

# Fold angle limit (radians). At FOLD_UPPER the absolute panel angle is
# ±FOLD_UPPER and the projected X-width per panel is PANEL_W*cos(FOLD_UPPER).
FOLD_UPPER = 1.1  # ~63°


# ---------------------------------------------------------------------------
# Shared panel geometry helper
# ---------------------------------------------------------------------------


def _add_panel_visuals(
    part,
    index: int,
    *,
    salmon,
    salmon_dk,
    hinge_mat,
) -> None:
    """Add visuals for one bi-fold accordion panel leaf.

    Every panel carries:
    - a flat rectangular leaf
    - top and bottom structural edge rails
    - a hinge barrel at the right edge (except the trailing panel)

    The leading panel (index 0) also gets a mount bracket that bolts it to
    the left stile.
    """
    # Main leaf
    part.visual(
        Box((PANEL_W, PANEL_T, PANEL_H)),
        origin=Origin(xyz=(PANEL_W * 0.5, 0.0, 0.0)),
        material=salmon,
        name="leaf",
    )
    # Top edge rail (structural frame, slightly proud of the leaf)
    part.visual(
        Box((RAIL_W, RAIL_T, RAIL_H_PANEL)),
        origin=Origin(xyz=(PANEL_W * 0.5, 0.0, PANEL_H * 0.5 - RAIL_H_PANEL * 0.5)),
        material=hinge_mat,
        name="top_rail",
    )
    # Bottom edge rail
    part.visual(
        Box((RAIL_W, RAIL_T, RAIL_H_PANEL)),
        origin=Origin(xyz=(PANEL_W * 0.5, 0.0, -PANEL_H * 0.5 + RAIL_H_PANEL * 0.5)),
        material=hinge_mat,
        name="bottom_rail",
    )
    # Hinge barrel at the right edge (except the trailing panel)
    if index < N_PANELS - 1:
        part.visual(
            Cylinder(radius=HINGE_R, length=HINGE_LEN),
            origin=Origin(xyz=(PANEL_W, 0.0, 0.0)),
            material=hinge_mat,
            name="hinge_barrel",
        )
    # Mount bracket for leading panel (bolts to the left stile)
    if index == 0:
        b_lo = (STILE_W - LEAD_X) - 0.002   # embed into stile
        b_hi = 0.002                          # embed into panel
        part.visual(
            Box((b_hi - b_lo, 0.040, 0.080)),
            origin=Origin(xyz=((b_lo + b_hi) * 0.5, 0.0, 0.0)),
            material=salmon_dk,
            name="mount_bracket",
        )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_accordion_gate")

    # Warm terracotta-salmon paint (~RGB 199,153,140) plus dark-steel accents.
    salmon = model.material("salmon", rgba=(0.87, 0.66, 0.59, 1.0))
    salmon_dk = model.material("salmon_dk", rgba=(0.73, 0.52, 0.46, 1.0))
    hinge_mat = model.material("hinge_steel", rgba=(0.35, 0.33, 0.33, 1.0))
    rivet_mat = model.material("rivet", rgba=(0.46, 0.42, 0.42, 1.0))

    # --- Fixed frame (root) -------------------------------------------------
    frame = model.part("frame")
    frame.visual(
        Box((STILE_W, FRAME_D, FRAME_H)),
        origin=Origin(xyz=(STILE_W * 0.5, 0.0, FRAME_H * 0.5)),
        material=salmon,
        name="stile_a",
    )
    frame.visual(
        Box((STILE_W, FRAME_D, FRAME_H)),
        origin=Origin(xyz=(FRAME_W - STILE_W * 0.5, 0.0, FRAME_H * 0.5)),
        material=salmon,
        name="stile_b",
    )
    frame.visual(
        Box((FRAME_W, FRAME_D, RAIL_H)),
        origin=Origin(xyz=(FRAME_W * 0.5, 0.0, FRAME_H - RAIL_H * 0.5)),
        material=salmon,
        name="head_rail",
    )
    frame.visual(
        Box((FRAME_W, FRAME_D, RAIL_H)),
        origin=Origin(xyz=(FRAME_W * 0.5, 0.0, RAIL_H * 0.5)),
        material=salmon,
        name="sill_rail",
    )
    frame.visual(
        Box((OPEN_X1 - OPEN_X0, TRACK_T, TRACK_T)),
        origin=Origin(xyz=(FRAME_W * 0.5, 0.0, OPEN_Z1 - TRACK_T * 0.5)),
        material=salmon_dk,
        name="top_track",
    )
    frame.visual(
        Box((OPEN_X1 - OPEN_X0, TRACK_T, TRACK_T)),
        origin=Origin(xyz=(FRAME_W * 0.5, 0.0, OPEN_Z0 + TRACK_T * 0.5)),
        material=salmon_dk,
        name="bottom_track",
    )

    # Small latch block on the right stile, mid-height (as in the reference).
    latch_z = 1.000
    plate_x = FRAME_W - STILE_W * 0.5
    frame.visual(
        Box((STILE_W * 0.5, FRAME_D + 0.010, 0.090)),
        origin=Origin(xyz=(plate_x, 0.0, latch_z)),
        material=salmon_dk,
        name="latch_plate",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=Origin(
            xyz=(plate_x, FRAME_D * 0.5 + 0.018, latch_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rivet_mat,
        name="latch_knob",
    )

    # --- Accordion panels ---------------------------------------------------
    # Each panel's part frame sits at its LEFT hinge edge, vertically centred
    # on the opening. The leaf extends along local +X; the hinge to the next
    # panel is at local (PANEL_W, 0, 0).
    panels = []
    for i in range(N_PANELS):
        p = model.part(f"panel_{i}")
        _add_panel_visuals(
            p, i,
            salmon=salmon,
            salmon_dk=salmon_dk,
            hinge_mat=hinge_mat,
        )
        panels.append(p)

    # Leading panel is bolted to the left stile (FIXED).
    model.articulation(
        "frame_to_panel_0",
        ArticulationType.FIXED,
        parent=frame,
        child=panels[0],
        origin=Origin(xyz=(LEAD_X, 0.0, PANEL_CZ)),
    )

    # Drive hinge: panel_0 → panel_1. Axis (0,0,1) = vertical; positive q
    # folds panel_1 forward (+Y) away from the frame plane.
    drive_name = "fold"
    model.articulation(
        drive_name,
        ArticulationType.REVOLUTE,
        parent=panels[0],
        child=panels[1],
        origin=Origin(xyz=(PANEL_W, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=0.8, lower=0.0, upper=FOLD_UPPER,
        ),
    )

    # Mimic hinges: panel_i → panel_{i+1} for i = 1 … N_PANELS-2.
    # Multiplier alternates ±2 so the absolute panel angles zigzag between
    # +q and -q and the row compresses symmetrically.
    overlap_pairs: list[tuple[str, str, str]] = [
        ("panel_0", "frame",
         "Leading panel is bolted to the left stile by its mount bracket."),
    ]

    for i in range(1, N_PANELS - 1):
        multiplier = 2.0 * ((-1) ** i)   # -2, +2, -2, +2, ...
        if multiplier < 0:
            lo, hi = multiplier * FOLD_UPPER, 0.0
        else:
            lo, hi = 0.0, multiplier * FOLD_UPPER
        model.articulation(
            f"hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=panels[i],
            child=panels[i + 1],
            origin=Origin(xyz=(PANEL_W, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=120.0, velocity=1.6, lower=lo, upper=hi,
            ),
            mimic=Mimic(joint=drive_name, multiplier=multiplier),
        )

    # Hinge-barrel overlaps at each inter-panel seam.
    for i in range(N_PANELS - 1):
        overlap_pairs.append(
            (f"panel_{i}", f"panel_{i + 1}",
             f"Hinge barrel at the seam between panel_{i} and panel_{i + 1}.")
        )

    model.meta["overlap_pairs"] = overlap_pairs
    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    drive = object_model.get_articulation("fold")

    # --- Frame identity -----------------------------------------------------
    fb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame stands on ground",
        fb is not None and abs(fb[0][2]) < 1e-6,
        details=f"frame z-min={None if fb is None else fb[0][2]}",
    )
    ctx.check(
        "frame ~2.0m tall and ~1.2m wide",
        fb is not None
        and abs((fb[1][2] - fb[0][2]) - FRAME_H) < 1e-3
        and abs((fb[1][0] - fb[0][0]) - FRAME_W) < 1e-3,
        details=f"aabb={fb}",
    )

    # --- All panels exist with correct visuals ------------------------------
    panels = [object_model.get_part(f"panel_{i}") for i in range(N_PANELS)]
    ctx.check(
        "all accordion panels present",
        all(p is not None for p in panels),
        details=f"{N_PANELS} panels expected",
    )
    for i, p in enumerate(panels):
        ctx.check(
            f"panel_{i} has leaf visual",
            p.get_visual("leaf") is not None,
            details=f"panel_{i} missing flat leaf",
        )

    # --- Hero pose (q=0, deployed flat wall) --------------------------------
    def panel_positions() -> list[tuple[float, float, float]]:
        return [ctx.part_world_position(p) for p in panels]

    pos_open = panel_positions()
    ctx.check(
        "leading panel anchored at left of opening",
        abs(pos_open[0][0] - LEAD_X) < 1e-6,
        details=f"panel_0 x={pos_open[0][0]}",
    )
    ctx.check(
        "deployed panels span most of the opening",
        pos_open[-1][0] > 0.95,
        details=f"trailing panel x(open)={pos_open[-1][0]}",
    )

    # All panels coplanar (flat wall) at q=0.
    ctx.check(
        "deployed panels form a flat wall (y≈0)",
        all(abs(p[1]) < 1e-4 for p in pos_open),
        details=f"y-spread={max(abs(p[1]) for p in pos_open):.2e}",
    )

    # Panels ride within the opening height (between the guide tracks).
    pb = ctx.part_world_aabb(panels[0])
    ctx.check(
        "panels ride within the opening height",
        pb is not None and pb[0][2] >= OPEN_Z0 - 1e-4 and pb[1][2] <= OPEN_Z1 + 1e-4,
        details=f"panel z-range={None if pb is None else (pb[0][2], pb[1][2])}",
    )

    # Uniform pitch at deploy: each panel origin is PANEL_W apart in X.
    ctx.check(
        "deployed panel pitch is uniform",
        all(
            abs((pos_open[i + 1][0] - pos_open[i][0]) - PANEL_W) < 1e-5
            for i in range(N_PANELS - 1)
        ),
        details=f"pitches={[round(pos_open[i + 1][0] - pos_open[i][0], 4) for i in range(N_PANELS - 1)]}",
    )

    # --- Folded pose (zigzag accordion contraction) -------------------------
    with ctx.pose({drive: FOLD_UPPER}):
        pos_folded = panel_positions()

        # Gate contracts toward the left stile.
        deployed_span = pos_open[-1][0] - pos_open[0][0]
        folded_span = pos_folded[-1][0] - pos_folded[0][0]
        ctx.check(
            "gate concertinas: span contracts significantly when folded",
            folded_span < deployed_span * 0.65,
            details=f"deployed={deployed_span:.3f}, folded={folded_span:.3f}",
        )

        # Trailing edge moves toward the left.
        ctx.check(
            "trailing edge moves left when folded",
            pos_folded[-1][0] < pos_open[-1][0] - 0.25,
            details=f"open_x={pos_open[-1][0]:.3f}, folded_x={pos_folded[-1][0]:.3f}",
        )

        # Zigzag: odd panels deviate in Y from the frame plane.
        max_y = max(abs(p[1]) for p in pos_folded)
        ctx.check(
            "folded panels zigzag out of the frame plane",
            max_y > 0.04,
            details=f"max |y|={max_y:.3f}",
        )

        # Leading panel stays anchored.
        ctx.check(
            "leading panel stays anchored when folded",
            abs(pos_folded[0][0] - LEAD_X) < 1e-5,
            details=f"panel_0 x(folded)={pos_folded[0][0]}",
        )

    # --- Half-fold intermediate pose ----------------------------------------
    with ctx.pose({drive: FOLD_UPPER * 0.5}):
        pos_half = panel_positions()
        half_span = pos_half[-1][0] - pos_half[0][0]
        ctx.check(
            "half-fold span is between deployed and fully folded",
            folded_span < half_span < deployed_span,
            details=f"half_span={half_span:.3f}",
        )

    # --- Overlap allowances -------------------------------------------------
    # Declare intentional hinge-barrel and bracket overlaps so the compiler
    # overlap pass does not fail, and prove each pair is seated at the hero
    # pose.
    for a, b, reason in object_model.meta.get("overlap_pairs", []):
        ctx.allow_overlap(a, b, reason=reason)
        ctx.expect_contact(
            object_model.get_part(a),
            object_model.get_part(b),
            contact_tol=1e-3,
            name=f"{a} seated on {b}",
        )

    return ctx.report()


object_model = build_object_model()
