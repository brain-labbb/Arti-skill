from __future__ import annotations

# Concertina / accordion trellis security gate – BARE PANTOGRAPH variant.
#
# A fixed rectangular steel frame (root) carries a folding lattice of
# terracotta-salmon steel bars and thin vertical pickets. The decorative
# diamond-lattice straps have been stripped so each scissor cell is now a
# bare two-bar pantograph linkage: plain crossed flat-steel bars between
# adjacent pickets, riveted at their intersection. Four evenly-spaced rows
# of bare X-linkages span the opening height (one row per former trellis
# band). The gate folds and collapses to one side as ONE motion driven by
# a single REVOLUTE "fold" joint (a real scissor pivot); every other pivot
# is mimic-coupled to it, so the whole gate expands / contracts together.
#
# Articulations form a TREE, so the physically closed-loop pantograph is
# approximated as a serial scissor chain. The load path through the drive
# row is a two-bar "lambda" linkage (the lower-V half of that cell's X):
#
#   picket_c --(rev +q)--> lam_up --(elbow rev -2q at the bar CROSSING)-->
#       lam_dn --(rev +q)--> picket_{c+1}
#
# With the 1 : -2 : 1 mimic ratio the net rotation cancels exactly, so
# every picket stays perfectly upright AND perfectly level at every fold
# angle while the cell pitch contracts as 2*S*cos(theta0 + q). The three
# non-drive rows each carry a pair of mimic-coupled plain bars per cell
# (ascending and descending, pivoting on the left picket), completing the
# visible bare-pantograph X at every row.
#
# Visual layout: ~11 thin vertical pickets, four rows of bare crossed
# bars, a fixed frame with a wide left stile, slim top and bottom rails
# with guide tracks, and a small latch block mid-height on the right
# stile. q=0 is the fully DEPLOYED gate (hero pose) spanning the opening;
# positive drive folds the lattice to the left stile.

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

# --- Lattice (scissor chain) ---
N_CELLS = 10
N_VERTS = N_CELLS + 1             # picket count

PITCH_OPEN = 0.105                # picket spacing when deployed (q=0)
PITCH_CLOSED = 0.044              # picket spacing when collapsed
LEAD_X = 0.075                    # x of the anchored leading picket (picket_0)

V_RISE = 0.045                    # half-row rise (crossing height above bar pivot)
ROW_H = 2.0 * V_RISE              # one row height (0.090)

# Four rows of bare crossed bars, one per former trellis band.
N_ROWS = 4
ROW_BOTTOMS = (0.180, 0.634, 1.079, 1.551)
DRIVE_RI = 2                       # drive row index
LOAD_Z = ROW_BOTTOMS[DRIVE_RI]    # 1.079 – drive-row bar pivot height

# Scissor geometry. Each half-bar spans half a cell and one V_RISE; full
# bars are two half-spans long. Pitch follows
# pitch(q) = 2*S_HALF*cos(THETA0 + q) exactly.
S_HALF = math.hypot(PITCH_OPEN * 0.5, V_RISE)
THETA0 = math.atan2(V_RISE, PITCH_OPEN * 0.5)
FOLD_ANGLE = math.acos((PITCH_CLOSED * 0.5) / S_HALF) - THETA0

PICKET_CX = 0.013                 # picket cross-section (X)
PICKET_CY = 0.022                 # picket cross-section (Y / depth)
PICKET_Z0 = 0.082                 # picket bottom (rides just above bottom track)
PICKET_Z1 = 1.918                 # picket top (just below top track)

BAR_W = 0.022                     # bar face width (all rows)
BAR_T = 0.010                     # bar thickness (Y)
BAR_EXT = 0.012                   # bar end extension past its pivot (rivet lap)

RIVET_R = 0.0055
RIVET_LEN = PICKET_CY + 0.008    # through picket depth + proud heads
CROSS_RIVET_LEN = 2.0 * BAR_T + 0.012  # through two stacked bars + proud heads


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_trellis_gate")

    # Warm, lightly-weathered terracotta-salmon paint sampled from the
    # reference (~RGB 199,153,140), brightened slightly for the shaded render.
    salmon = model.material("salmon", rgba=(0.87, 0.66, 0.59, 1.0))
    salmon_dk = model.material("salmon_dk", rgba=(0.73, 0.52, 0.46, 1.0))
    steel = model.material("steel", rgba=(0.48, 0.46, 0.44, 1.0))
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

    # Small latch block on the right stile, mid-height (as in the reference):
    # a compact keeper plate plus a short catch knob standing proud of it.
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

    # --- Pickets ------------------------------------------------------------
    # Picket part frames originate at the drive-row bar pivot (z = LOAD_Z in
    # world at deploy), so the scissor chain attaches at each part's local
    # origin.
    pickets = []
    # Rivet positions: at every bar-pivot level (row bottoms and tops).
    rivet_zs = sorted({z for zb in ROW_BOTTOMS for z in (zb, zb + ROW_H)})
    for i in range(N_VERTS):
        p = model.part(f"picket_{i}")
        p.visual(
            Box((PICKET_CX, PICKET_CY, PICKET_Z1 - PICKET_Z0)),
            origin=Origin(xyz=(0.0, 0.0, (PICKET_Z0 + PICKET_Z1) * 0.5 - LOAD_Z)),
            material=salmon,
            name="picket_bar",
        )
        for k, z in enumerate(rivet_zs):
            p.visual(
                Cylinder(radius=RIVET_R, length=RIVET_LEN),
                origin=Origin(
                    xyz=(0.0, 0.0, z - LOAD_Z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=rivet_mat,
                name=f"rivet_{k}",
            )
        pickets.append(p)

    # The leading picket is bolted to the left stile by a short mounting
    # bracket so the whole lattice is grounded on the fixed frame (small
    # hidden embeds into both members).
    b_lo = (STILE_W - LEAD_X) - 0.002              # embed into stile
    b_hi = (-PICKET_CX * 0.5) + 0.002              # embed into picket
    pickets[0].visual(
        Box((b_hi - b_lo, 0.030, 0.080)),
        origin=Origin(xyz=((b_lo + b_hi) * 0.5, 0.0, 0.0)),
        material=salmon_dk,
        name="mount_bracket",
    )

    model.articulation(
        "frame_to_picket_0",
        ArticulationType.FIXED,
        parent=frame,
        child=pickets[0],
        origin=Origin(xyz=(LEAD_X, 0.0, LOAD_Z)),
    )

    drive_name = "fold"
    rev_limits = MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=FOLD_ANGLE)
    elbow_limits = MotionLimits(
        effort=120.0, velocity=1.6, lower=0.0, upper=2.0 * FOLD_ANGLE
    )

    # Intentional riveted-lap overlaps (recorded for run_tests()).
    lap_pairs: list[tuple[str, str, str]] = [
        ("picket_0", "frame",
         "Leading picket is bolted to the left stile by its mount bracket."),
    ]

    # --- Shared bar geometry helpers ---------------------------------------
    def _lambda_bar(name: str) -> object:
        """Half-length bar for the drive-row serial scissor chain.

        Extends from pivot to crossing (S_HALF) plus rivet-lap extensions.
        """
        bp = model.part(name)
        bp.visual(
            Box((S_HALF + 2.0 * BAR_EXT, BAR_T, BAR_W)),
            origin=Origin(xyz=(S_HALF * 0.5, 0.0, 0.0)),
            material=steel,
            name="bar",
        )
        return bp

    def _crossed_bar(name: str, *, with_cross_rivet: bool = False) -> object:
        """Full-length bar for non-drive-row bare pantograph diagonals.

        Extends the full cell diagonal (2*S_HALF) plus rivet-lap extensions.
        """
        bp = model.part(name)
        bp.visual(
            Box((2.0 * S_HALF + 2.0 * BAR_EXT, BAR_T, BAR_W)),
            origin=Origin(xyz=(S_HALF, 0.0, 0.0)),
            material=steel,
            name="bar",
        )
        if with_cross_rivet:
            bp.visual(
                Cylinder(radius=RIVET_R, length=CROSS_RIVET_LEN),
                origin=Origin(
                    xyz=(S_HALF, 0.0, 0.0),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=rivet_mat,
                name="cross_rivet",
            )
        return bp

    # --- Build scissor chain (drive row) and mimic bars (non-drive rows) ---
    for c in range(N_CELLS):
        left = pickets[c]
        right = pickets[c + 1]

        # --- Drive-row lambda linkage (the cell's serial scissor half) -----
        up = _lambda_bar(f"lam_up_{c}")
        up_kwargs = dict(
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -THETA0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=rev_limits,
        )
        if c == 0:
            model.articulation(
                drive_name, ArticulationType.REVOLUTE, parent=left, child=up,
                **up_kwargs,
            )
        else:
            up_kwargs["mimic"] = Mimic(joint=drive_name, multiplier=1.0)
            model.articulation(
                f"lam_up_{c}_pivot", ArticulationType.REVOLUTE, parent=left,
                child=up, **up_kwargs,
            )

        dn = _lambda_bar(f"lam_dn_{c}")
        # Elbow rivet at the bar crossing, pinning the lambda halves.
        dn.visual(
            Cylinder(radius=RIVET_R, length=CROSS_RIVET_LEN),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rivet_mat,
            name="elbow_rivet",
        )
        model.articulation(
            f"lam_elbow_{c}",
            ArticulationType.REVOLUTE,
            parent=up,
            child=dn,
            origin=Origin(xyz=(S_HALF, 0.0, 0.0), rpy=(0.0, 2.0 * THETA0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=elbow_limits,
            mimic=Mimic(joint=drive_name, multiplier=2.0),
        )

        model.articulation(
            f"lam_dn_{c}_to_picket_{c + 1}",
            ArticulationType.REVOLUTE,
            parent=dn,
            child=right,
            origin=Origin(xyz=(S_HALF, 0.0, 0.0), rpy=(0.0, -THETA0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=rev_limits,
            mimic=Mimic(joint=drive_name, multiplier=1.0),
        )

        lap_pairs.append(
            (f"lam_up_{c}", f"picket_{c}",
             "Lambda bar is riveted (lapped) to the picket it pivots on.")
        )
        lap_pairs.append(
            (f"lam_up_{c}", f"lam_dn_{c}",
             "Lambda halves lap at their shared elbow rivet (the bar crossing).")
        )
        lap_pairs.append(
            (f"lam_dn_{c}", f"picket_{c + 1}",
             "Lambda bar is riveted (lapped) to the picket it carries.")
        )
        if c > 0:
            lap_pairs.append(
                (f"lam_dn_{c - 1}", f"lam_up_{c}",
                 "Adjacent lambda bars share the picket rivet and lap there.")
            )

        # --- Non-drive rows: bare crossed bars (mimic-coupled) -------------
        for ri in range(N_ROWS):
            if ri == DRIVE_RI:
                continue
            z_bot = ROW_BOTTOMS[ri]
            z_top = z_bot + ROW_H

            # Ascending bar: pivots on left picket at row bottom, angles up.
            xu = _crossed_bar(f"xup_{c}_{ri}", with_cross_rivet=True)
            model.articulation(
                f"xup_{c}_{ri}_pivot",
                ArticulationType.REVOLUTE,
                parent=left,
                child=xu,
                origin=Origin(
                    xyz=(0.0, 0.0, z_bot - LOAD_Z),
                    rpy=(0.0, -THETA0, 0.0),
                ),
                axis=(0.0, -1.0, 0.0),
                motion_limits=rev_limits,
                mimic=Mimic(joint=drive_name, multiplier=1.0),
            )

            # Descending bar: pivots on left picket at row top, angles down.
            xd = _crossed_bar(f"xdn_{c}_{ri}")
            model.articulation(
                f"xdn_{c}_{ri}_pivot",
                ArticulationType.REVOLUTE,
                parent=left,
                child=xd,
                origin=Origin(
                    xyz=(0.0, 0.0, z_top - LOAD_Z),
                    rpy=(0.0, THETA0, 0.0),
                ),
                axis=(0.0, 1.0, 0.0),
                motion_limits=rev_limits,
                mimic=Mimic(joint=drive_name, multiplier=1.0),
            )

            lap_pairs.append(
                (f"xup_{c}_{ri}", f"picket_{c}",
                 "Crossed bar is riveted (lapped) to the picket it pivots on.")
            )
            lap_pairs.append(
                (f"xdn_{c}_{ri}", f"picket_{c}",
                 "Crossed bar is riveted (lapped) to the picket it pivots on.")
            )
            lap_pairs.append(
                (f"xup_{c}_{ri}", f"xdn_{c}_{ri}",
                 "Crossed bars lap at their shared crossing rivet.")
            )
            # The bar far ends slide past the neighbouring picket (real gates
            # absorb this in slotted rivet holes).
            lap_pairs.append(
                (f"xup_{c}_{ri}", f"picket_{c + 1}",
                 "Crossed bar slides past the neighbouring picket face as the "
                 "gate folds (slotted-rivet clearance).")
            )
            lap_pairs.append(
                (f"xdn_{c}_{ri}", f"picket_{c + 1}",
                 "Crossed bar slides past the neighbouring picket face as the "
                 "gate folds (slotted-rivet clearance).")
            )
            # Adjacent cells' bars continue the zigzag through a shared
            # picket rivet; their lap extensions overlap there exactly like
            # the riveted bar joints of a real gate.
            if c + 1 < N_CELLS:
                lap_pairs.append(
                    (f"xdn_{c}_{ri}", f"xup_{c + 1}_{ri}",
                     "Adjacent cells' bars share the picket rivet and lap at "
                     "the lower zigzag continuation.")
                )
                lap_pairs.append(
                    (f"xup_{c}_{ri}", f"xdn_{c + 1}_{ri}",
                     "Adjacent cells' bars share the picket rivet and lap at "
                     "the upper zigzag continuation.")
                )

    model.meta["lap_pairs"] = lap_pairs
    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    drive = object_model.get_articulation("fold")

    # --- Hero pose (q=0, deployed) -----------------------------------------
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

    def picket_positions() -> list[tuple[float, float, float]]:
        return [
            ctx.part_world_position(object_model.get_part(f"picket_{i}"))
            for i in range(N_VERTS)
        ]

    pos_open = picket_positions()
    ctx.check(
        "leading picket anchored at left of opening",
        abs(pos_open[0][0] - LEAD_X) < 1e-6,
        details=f"picket_0 x={pos_open[0][0]}",
    )
    ctx.check(
        "deployed lattice spans most of the opening",
        pos_open[-1][0] > 1.05,
        details=f"trailing picket x(open)={pos_open[-1][0]}",
    )
    ctx.check(
        "deployed picket pitch matches PITCH_OPEN",
        all(
            abs((pos_open[i + 1][0] - pos_open[i][0]) - PITCH_OPEN) < 1e-6
            for i in range(N_CELLS)
        ),
        details=f"pitches={[round(pos_open[i + 1][0] - pos_open[i][0], 4) for i in range(N_CELLS)]}",
    )

    # Pickets ride within the opening height (between the guide tracks).
    pb = ctx.part_world_aabb(object_model.get_part("picket_0"))
    ctx.check(
        "pickets ride within the opening height",
        pb is not None and pb[0][2] >= OPEN_Z0 - 1e-4 and pb[1][2] <= OPEN_Z1 + 1e-4,
        details=f"picket z-range={None if pb is None else (pb[0][2], pb[1][2])}",
    )

    # The lambda chain keeps every picket level and upright at EVERY pose --
    # the key realism property of the scissor approximation.
    for frac, label in ((0.5, "half-folded"), (1.0, "fully folded")):
        with ctx.pose({drive: FOLD_ANGLE * frac}):
            pos = picket_positions()
            ctx.check(
                f"pickets stay level when {label}",
                all(abs(p[2] - LOAD_Z) < 1e-5 for p in pos),
                details=f"z-spread={max(abs(p[2] - LOAD_Z) for p in pos):.2e}",
            )
            expected = 2.0 * S_HALF * math.cos(THETA0 + FOLD_ANGLE * frac)
            ctx.check(
                f"picket pitch contracts uniformly when {label}",
                all(
                    abs((pos[i + 1][0] - pos[i][0]) - expected) < 1e-6
                    for i in range(N_CELLS)
                ),
                details=f"expected pitch={expected:.4f}",
            )

    with ctx.pose({drive: FOLD_ANGLE}):
        pos_closed = picket_positions()
        ctx.check(
            "gate collapses toward the left stile",
            pos_closed[-1][0] < pos_open[-1][0] - 0.45,
            details=f"open_x={pos_open[-1][0]:.3f}, closed_x={pos_closed[-1][0]:.3f}",
        )
        ctx.check(
            "collapsed pitch matches PITCH_CLOSED",
            abs((pos_closed[1][0] - pos_closed[0][0]) - PITCH_CLOSED) < 1e-6,
            details=f"closed pitch={pos_closed[1][0] - pos_closed[0][0]:.4f}",
        )

    # Declare the intentional riveted-lap overlaps (scissor pivots and bar
    # crossings) so the compiler-owned overlap pass does not fail on them,
    # and prove each pair is genuinely seated in contact at the hero pose.
    for a, b, reason in object_model.meta.get("lap_pairs", []):
        ctx.allow_overlap(a, b, reason=reason)
        ctx.expect_contact(
            object_model.get_part(a),
            object_model.get_part(b),
            contact_tol=1e-3,
            name=f"{a} seated on {b}",
        )

    # --- Bare pantograph: every cell carries crossed bars at every row -----
    non_drive_ris = [ri for ri in range(N_ROWS) if ri != DRIVE_RI]
    ctx.check(
        "all bare crossed bars present (non-drive rows)",
        all(
            object_model.get_part(f"xup_{c}_{ri}") is not None
            and object_model.get_part(f"xdn_{c}_{ri}") is not None
            for c in range(N_CELLS)
            for ri in non_drive_ris
        ),
        details=f"{N_CELLS} cells x {len(non_drive_ris)} non-drive rows x 2 bars",
    )

    # No diamond-lattice strap parts should exist.
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no diamond lattice straps (bare pantograph only)",
        not any(
            name.startswith("asc_") or name.startswith("desc_")
            for name in part_names
        ),
        details="asc_/desc_ parts must not exist",
    )

    # Drive-row lambda bars present for every cell.
    ctx.check(
        "drive-row lambda bars present for every cell",
        all(
            object_model.get_part(f"lam_up_{c}") is not None
            and object_model.get_part(f"lam_dn_{c}") is not None
            for c in range(N_CELLS)
        ),
        details=f"{N_CELLS} cells x 2 lambda bars",
    )

    # Topmost crossed bar stays below the head rail when fully folded.
    top_ri = max(non_drive_ris)
    with ctx.pose({drive: FOLD_ANGLE}):
        top = ctx.part_world_aabb(
            object_model.get_part(f"xup_{N_CELLS - 1}_{top_ri}")
        )
        ctx.check(
            "steepened bars stay below the head rail when fully folded",
            top is not None and top[1][2] <= OPEN_Z1 + 1e-4,
            details=f"top bar z-max={None if top is None else top[1][2]}",
        )

    return ctx.report()


object_model = build_object_model()
