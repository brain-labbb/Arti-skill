from __future__ import annotations

# Concertina / accordion trellis security gate (an expanding scissor gate).
#
# A fixed rectangular steel frame (root) carries a folding lattice of flat
# terracotta-salmon steel straps and thin vertical pickets. The lattice folds
# and collapses to one side as ONE motion driven by a single REVOLUTE "fold"
# joint (a real scissor pivot); every other pivot is mimic-coupled to it, so
# the whole gate expands / contracts together like an accordion.
#
# Articulations form a TREE, so the physically closed-loop pantograph is
# approximated as a serial scissor chain. The load path through each cell is a
# two-bar "lambda" linkage (the lower-V half of that cell's scissor X):
#
#   picket_c --(rev +q)--> lam_up --(elbow rev -2q at the bar CROSSING)-->
#       lam_dn --(rev +q)--> picket_{c+1}
#
# With the 1 : -2 : 1 mimic ratio the net rotation cancels exactly, so every
# picket stays perfectly upright AND perfectly level at every fold angle while
# the cell pitch contracts as 2*S*cos(theta0 + q). The decorative diamond
# straps are individual flat bars, each pivoting about its own rivet on the
# picket that carries it (mimic +/-1), exactly like the riveted bars of a real
# trellis gate. As the gate folds the diamonds steepen and elongate; the far
# strap ends slide up the neighbouring picket face (real gates absorb this in
# slotted rivets), while the pitch contraction stays exact at every pose.
#
# Visual layout follows the reference photo: ~6 thin vertical pickets, four
# horizontal diamond-lattice bands (top / upper / lower / bottom) with plain
# picket runs between them, a fixed frame with a wide left stile, slim top and
# bottom rails with guide tracks, and a small latch block mid-height on the
# right stile. q=0 is the fully DEPLOYED gate (hero pose) spanning the
# opening; positive drive folds the lattice to the left stile.

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
N_CELLS = 5
N_VERTS = N_CELLS + 1             # picket count

PITCH_OPEN = 0.105                # picket spacing when deployed (q=0)
PITCH_CLOSED = 0.044              # picket spacing when collapsed
LEAD_X = 0.075                    # x of the anchored leading picket (picket_0)

V_RISE = 0.045                    # half-diamond rise (crossing height above rivet)
ROW_H = 2.0 * V_RISE              # one diamond row height (0.090)
ROWS_PER_BAND = 2
BAND_H = ROWS_PER_BAND * ROW_H    # 0.180

# Four diamond bands, matching the reference photo's banded trellis pattern.
BAND_BOTTOMS = (0.180, 0.634, 1.079, 1.551)
LOAD_Z = BAND_BOTTOMS[2]          # load-path lambda row rides in band 2, row 0

# Scissor geometry. Each half-bar spans half a cell and one V_RISE; full
# decorative diagonals are two half-spans long. Pitch follows
# pitch(q) = 2*S_HALF*cos(THETA0 + q) exactly.
S_HALF = math.hypot(PITCH_OPEN * 0.5, V_RISE)
THETA0 = math.atan2(V_RISE, PITCH_OPEN * 0.5)
FOLD_ANGLE = math.acos((PITCH_CLOSED * 0.5) / S_HALF) - THETA0

PICKET_CX = 0.013                 # picket cross-section (X)
PICKET_CY = 0.022                 # picket cross-section (Y / depth)
PICKET_Z0 = 0.082                 # picket bottom (rides just above bottom track)
PICKET_Z1 = 1.918                 # picket top (just below top track)

STRAP_W = 0.020                   # diamond strap face width
STRAP_T = 0.008                   # diamond strap thickness (Y)
LAM_W = 0.022                     # load lambda bar face width
LAM_T = 0.010                     # load lambda bar thickness (Y)
BAR_EXT = 0.012                   # bar end extension past its pivot (rivet lap)

# Front (ascending) and back (descending) strap families sit on opposite
# picket faces, riveted laps embedding ~3.5mm into the picket depth. The load
# lambda runs through the picket centerline plane (y=0) between the families.
Y_FRONT = 0.0115
Y_BACK = -Y_FRONT

RIVET_R = 0.0055
RIVET_LEN = PICKET_CY + 2.0 * STRAP_T + 0.004   # through picket + both straps
CROSS_RIVET_LEN = 0.040                          # through straps + lambda at crossings


def _row_zs() -> list[float]:
    """Bottom z of every diamond row across all bands."""
    zs = []
    for zb in BAND_BOTTOMS:
        for r in range(ROWS_PER_BAND):
            zs.append(zb + r * ROW_H)
    return zs


ROW_BOTTOMS = _row_zs()

# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_trellis_gate")

    # Warm, lightly-weathered terracotta-salmon paint sampled from the
    # reference (~RGB 199,153,140), brightened slightly for the shaded render.
    salmon = model.material("salmon", rgba=(0.87, 0.66, 0.59, 1.0))
    salmon_dk = model.material("salmon_dk", rgba=(0.73, 0.52, 0.46, 1.0))
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
    # Picket part frames originate at the load-row rivet (z = LOAD_Z in world
    # at deploy), so the scissor chain attaches at each part's local origin.
    pickets = []
    for i in range(N_VERTS):
        p = model.part(f"picket_{i}")
        p.visual(
            Box((PICKET_CX, PICKET_CY, PICKET_Z1 - PICKET_Z0)),
            origin=Origin(xyz=(0.0, 0.0, (PICKET_Z0 + PICKET_Z1) * 0.5 - LOAD_Z)),
            material=salmon,
            name="picket_bar",
        )
        # Rivets at every strap pivot level (band row boundaries).
        rivet_zs = sorted({z for zb in ROW_BOTTOMS for z in (zb, zb + ROW_H)})
        for k, z in enumerate(rivet_zs):
            p.visual(
                Cylinder(radius=RIVET_R, length=RIVET_LEN),
                origin=Origin(xyz=(0.0, 0.0, z - LOAD_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
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
        ("picket_0", "frame", "Leading picket is bolted to the left stile by its mount bracket."),
    ]

    def lam_bar(name: str) -> object:
        part = model.part(name)
        part.visual(
            Box((S_HALF + 2.0 * BAR_EXT, LAM_T, LAM_W)),
            origin=Origin(xyz=(S_HALF * 0.5, 0.0, 0.0)),
            material=salmon_dk,
            name="bar",
        )
        return part

    for c in range(N_CELLS):
        left = pickets[c]
        right = pickets[c + 1]

        # --- Load-path lambda linkage (the cell's real scissor half) --------
        up = lam_bar(f"lam_up_{c}")
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

        dn = lam_bar(f"lam_dn_{c}")
        # Elbow rivet at the bar crossing, pinning the lambda halves and
        # visually pinning the strap families that cross there.
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

        # --- Decorative diamond straps (banded trellis mesh) ----------------
        # Each diagonal is its own flat bar pivoting about its OWN rivet on
        # the left picket; ascending bars ride the front face, descending the
        # back face, crossing mid-cell.
        for ri, z_bot in enumerate(ROW_BOTTOMS):
            z_top = z_bot + ROW_H

            asc = model.part(f"asc_{c}_{ri}")
            asc.visual(
                Box((2.0 * S_HALF + 2.0 * BAR_EXT, STRAP_T, STRAP_W)),
                origin=Origin(xyz=(S_HALF, 0.0, 0.0)),
                material=salmon,
                name="bar",
            )
            # Mid-bar rivet marking the crossing with the descending strap.
            asc.visual(
                Cylinder(radius=RIVET_R, length=CROSS_RIVET_LEN),
                origin=Origin(
                    xyz=(S_HALF, -Y_FRONT, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)
                ),
                material=rivet_mat,
                name="cross_rivet",
            )
            model.articulation(
                f"asc_{c}_{ri}_pivot",
                ArticulationType.REVOLUTE,
                parent=left,
                child=asc,
                origin=Origin(
                    xyz=(0.0, Y_FRONT, z_bot - LOAD_Z), rpy=(0.0, -THETA0, 0.0)
                ),
                axis=(0.0, -1.0, 0.0),
                motion_limits=rev_limits,
                mimic=Mimic(joint=drive_name, multiplier=1.0),
            )

            desc = model.part(f"desc_{c}_{ri}")
            desc.visual(
                Box((2.0 * S_HALF + 2.0 * BAR_EXT, STRAP_T, STRAP_W)),
                origin=Origin(xyz=(S_HALF, 0.0, 0.0)),
                material=salmon,
                name="bar",
            )
            model.articulation(
                f"desc_{c}_{ri}_pivot",
                ArticulationType.REVOLUTE,
                parent=left,
                child=desc,
                origin=Origin(
                    xyz=(0.0, Y_BACK, z_top - LOAD_Z), rpy=(0.0, THETA0, 0.0)
                ),
                axis=(0.0, 1.0, 0.0),
                motion_limits=rev_limits,
                mimic=Mimic(joint=drive_name, multiplier=1.0),
            )

            for nm in (f"asc_{c}_{ri}", f"desc_{c}_{ri}"):
                lap_pairs.append(
                    (nm, f"picket_{c}",
                     "Diamond strap is riveted (lapped) to the picket it pivots on.")
                )
                lap_pairs.append(
                    (nm, f"picket_{c + 1}",
                     "Diamond strap is riveted to the neighbouring picket; the "
                     "joint slides along the picket face as the gate folds.")
                )
            lap_pairs.append(
                (f"asc_{c}_{ri}", f"desc_{c}_{ri}",
                 "Crossed straps are pinned by the mid-cell rivet.")
            )
            # Consecutive cells' same-family straps continue the zigzag
            # through a shared picket rivet; their lap extensions overlap
            # there exactly like the riveted bar joints of a real gate.
            in_band_row = ri % ROWS_PER_BAND
            if c + 1 < N_CELLS:
                if in_band_row + 1 < ROWS_PER_BAND:
                    lap_pairs.append(
                        (f"asc_{c}_{ri}", f"asc_{c + 1}_{ri + 1}",
                         "Ascending straps of adjacent cells lap at the shared "
                         "picket rivet (continuous zigzag).")
                    )
                if in_band_row > 0:
                    lap_pairs.append(
                        (f"desc_{c}_{ri}", f"desc_{c + 1}_{ri - 1}",
                         "Descending straps of adjacent cells lap at the shared "
                         "picket rivet (continuous zigzag).")
                    )

    # The load lambda row shares its band row with decorative straps; the
    # lambda runs in the central plane so they only meet at the rivets.
    # The lambda bars only meet the load-row straps through the crossing
    # rivets: the ascending strap's rivet reaches both lambda halves, and the
    # elbow rivet reaches the descending strap. lam_up never touches desc
    # (no bridging rivet there), so that pair is neither allowed nor expected.
    load_ri = ROW_BOTTOMS.index(LOAD_Z)
    for c in range(N_CELLS):
        lap_pairs.append(
            (f"lam_up_{c}", f"asc_{c}_{load_ri}",
             "Load lambda and the load-row straps stack on shared rivets.")
        )
        lap_pairs.append(
            (f"lam_dn_{c}", f"asc_{c}_{load_ri}",
             "Load lambda and the load-row straps stack on shared rivets.")
        )
        lap_pairs.append(
            (f"lam_dn_{c}", f"desc_{c}_{load_ri}",
             "Elbow rivet pins the descending load-row strap at the crossing.")
        )

    model.meta["lap_pairs"] = lap_pairs
    model.meta["load_row_index"] = load_ri
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
        "deployed lattice spans a meaningful portion of the opening",
        pos_open[-1][0] > 0.50,
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
            # Folded lattice must stay inside the frame opening.
            top = ctx.part_world_aabb(
                object_model.get_part(f"asc_{N_CELLS - 1}_{len(ROW_BOTTOMS) - 1}")
            )
            ctx.check(
                f"steepened straps stay below the head rail when {label}",
                top is not None and top[1][2] <= OPEN_Z1 + 1e-4,
                details=f"top strap z-max={None if top is None else top[1][2]}",
            )

    with ctx.pose({drive: FOLD_ANGLE}):
        pos_closed = picket_positions()
        ctx.check(
            "gate collapses toward the left stile",
            pos_closed[-1][0] < pos_open[-1][0] - 0.25,
            details=f"open_x={pos_open[-1][0]:.3f}, closed_x={pos_closed[-1][0]:.3f}",
        )
        ctx.check(
            "collapsed pitch matches PITCH_CLOSED",
            abs((pos_closed[1][0] - pos_closed[0][0]) - PITCH_CLOSED) < 1e-6,
            details=f"closed pitch={pos_closed[1][0] - pos_closed[0][0]:.4f}",
        )

    # Declare the intentional riveted-lap overlaps (scissor pivots and strap
    # crossings) so the compiler-owned overlap pass does not fail on them, and
    # prove each pair is genuinely seated in contact at the hero pose.
    for a, b, reason in object_model.meta.get("lap_pairs", []):
        ctx.allow_overlap(a, b, reason=reason)
        ctx.expect_contact(
            object_model.get_part(a),
            object_model.get_part(b),
            contact_tol=1e-3,
            name=f"{a} seated on {b}",
        )

    # Banded trellis: every cell carries asc+desc straps for every band row.
    n_rows = len(ROW_BOTTOMS)
    ctx.check(
        "all diamond straps present",
        all(
            object_model.get_part(f"asc_{c}_{r}") is not None
            and object_model.get_part(f"desc_{c}_{r}") is not None
            for c in range(N_CELLS)
            for r in range(n_rows)
        ),
        details=f"{N_CELLS} cells x {n_rows} rows x 2 families",
    )

    return ctx.report()


object_model = build_object_model()
