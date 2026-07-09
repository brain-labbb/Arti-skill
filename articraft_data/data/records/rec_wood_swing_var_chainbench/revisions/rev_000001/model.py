from __future__ import annotations

# Metal A-frame swing bench, CAD-style chain-hung variant.
# Reference: picture/Bench/Wood Swing/004.png
#
# Frame convention:
#   X = fore/aft (the bench faces +X and swings in X)
#   Y = left/right (crossbar runs along Y)
#   Z = up
#
# Structure:
#   - Two tubular-steel A-frame end supports (angled legs, apex sleeve,
#     horizontal cross brace, two diagonal braces, flat foot plates).
#   - A top crossbar spanning the apexes, with horizontal bracket arms
#     extending fore/aft to carry the chain attachment points.
#   - Four hanging steel chains (two pairs, left and right), each a vertical
#     run of identical oval links built with a shared geometry helper.
#   - A slatted metal bench (curved rolltop slatted back and flat slatted seat,
#     armrest ends with round cup holders) hanging from the chains.
#
# Articulation: the four chains swing on revolute pivots at the crossbar
# bracket ends. `swing_drive` (front-left chain) is the driver; the other
# three mimic it with multiplier 1.0. The bench is fixed to the driver chain,
# so the whole pendulum swings together.

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# --- Frame dimensions (meters) -------------------------------------------------
APEX_Z = 1.80          # apex sleeve / crossbar axis height
FRAME_Y = 0.80         # A-frame center planes at y = +/-FRAME_Y
FOOT_X = 0.75          # leg foot position in X
LEG_R = 0.018          # leg tube radius (36 mm dia)
BRACE_R = 0.014        # brace tube radius (28 mm dia)
BAR_R = 0.020          # crossbar tube radius (40 mm dia)
PLATE_T = 0.012        # foot plate thickness
BRACKET_R = 0.012      # bracket arm tube radius (24 mm dia)

SWING_LIMIT = 0.45     # rad

# --- Chain dimensions (meters) -------------------------------------------------
CHAIN_Y = 0.665        # chain planes at y = +/-CHAIN_Y (matching armrests)
CHAIN_FRONT_X = 0.26   # front chain x (at the front seat rail)
CHAIN_REAR_X = -0.20   # rear chain x (at the rear arm post)
N_CHAIN_LINKS = 19     # links per chain
LINK_LENGTH = 0.058    # long axis of the oval link (along Z)
LINK_WIDTH = 0.026     # short axis of the oval link (along X or Y)
CHAIN_WIRE_R = 0.004   # wire rod radius (8 mm dia)

# --- Bench dimensions (bench frame origin at world (0, 0, BENCH_Z) at q=0) ----
BENCH_Z = 0.61
SEAT_W = 1.40          # bench width along Y
SEAT_HALF_X = 0.26     # seat rail half depth
N_SEAT_SLATS = 16
N_BACK_SLATS = 8

BENCH_ARMREST_Z = 0.29     # armrest center z in bench frame
BENCH_ARMREST_H = 0.025    # armrest box height

# Pivot and chain geometry (computed from link dimensions)
LINK_HALF = LINK_LENGTH / 2.0
# Link pitch: consecutive links overlap in Z for a realistic interlocked look.
LINK_PITCH = 0.045
# Chain part origin is at the pivot point. Link i center is at local
# z = -(LINK_HALF) - i * LINK_PITCH.  The top of link 0 is at local z = 0.
# The bottom of the last link is at local z = -(LINK_HALF) - (N-1)*LINK_PITCH
#                                          - LINK_HALF = -LINK_LENGTH - (N-1)*LINK_PITCH.
_CHAIN_SPAN = LINK_LENGTH + (N_CHAIN_LINKS - 1) * LINK_PITCH

# PIVOT_Z is at the bottom of the crossbar bracket.
PIVOT_Z = APEX_Z - BRACKET_R  # = 1.788

# The bench anchor pin sits at the center of the last chain link.
# Last link center in chain frame: -(LINK_HALF) - (N-1)*LINK_PITCH
_LAST_LINK_LOCAL_Z = -(LINK_HALF) - (N_CHAIN_LINKS - 1) * LINK_PITCH
# In world: PIVOT_Z + _LAST_LINK_LOCAL_Z
_ANCHOR_WORLD_Z = PIVOT_Z + _LAST_LINK_LOCAL_Z
# In bench frame: _ANCHOR_WORLD_Z - BENCH_Z
ANCHOR_BENCH_Z = _ANCHOR_WORLD_Z - BENCH_Z

# Chain anchor specs: (name_suffix, x, y)
_CHAIN_ANCHORS = (
    ("front_left", CHAIN_FRONT_X, CHAIN_Y),
    ("front_right", CHAIN_FRONT_X, -CHAIN_Y),
    ("rear_left", CHAIN_REAR_X, CHAIN_Y),
    ("rear_right", CHAIN_REAR_X, -CHAIN_Y),
)


# Curved rolltop back: circular arc in the XZ plane (bench frame).
BACK_R = 0.70
BACK_SX, BACK_SZ = -0.27, 0.03
BACK_PHI0 = 0.12
BACK_DPHI = 0.1043
_BCX = BACK_SX - BACK_R * math.cos(BACK_PHI0)
_BCZ = BACK_SZ - BACK_R * math.sin(BACK_PHI0)


def _arc_point(phi: float) -> tuple[float, float]:
    """Point on the back arc (x, z) in the bench frame."""
    return (_BCX + BACK_R * math.cos(phi), _BCZ + BACK_R * math.sin(phi))


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------

def _make_chain_link_geom():
    """Stadium-shaped chain link, long axis along Z, flat face in the XZ plane.

    Built as a closed tube swept along an oval centreline.  All chain links
    across all four chains share this single mesh helper.
    """
    half_l = LINK_LENGTH / 2.0
    half_w = LINK_WIDTH / 2.0
    straight = max(LINK_LENGTH - LINK_WIDTH, 0.0)
    half_s = straight / 2.0
    n_arc = 10

    pts: list[tuple[float, float, float]] = []
    # Right straight (bottom to top)
    pts.append((half_w, 0.0, -half_s))
    pts.append((half_w, 0.0, half_s))
    # Top semicircular arc
    for k in range(1, n_arc):
        a = -math.pi / 2.0 + math.pi * k / n_arc
        pts.append((half_w * math.cos(a), 0.0, half_s + half_w * math.sin(a)))
    # Left straight (top to bottom)
    pts.append((-half_w, 0.0, half_s))
    pts.append((-half_w, 0.0, -half_s))
    # Bottom semicircular arc
    for k in range(1, n_arc):
        a = math.pi / 2.0 + math.pi * k / n_arc
        pts.append((half_w * math.cos(a), 0.0, -half_s + half_w * math.sin(a)))

    return tube_from_spline_points(
        pts,
        radius=CHAIN_WIRE_R,
        samples_per_segment=8,
        closed_spline=True,
        radial_segments=12,
        cap_ends=False,
    )


# ---------------------------------------------------------------------------
# Structural helpers
# ---------------------------------------------------------------------------

def _build_a_frame(part, ys: float) -> None:
    """Two angled legs + apex sleeve + braces + foot plates at plane y=ys."""
    for tag, xf in (("front", FOOT_X), ("rear", -FOOT_X)):
        z0, z1 = 0.013, 1.776
        scale = math.hypot(FOOT_X, APEX_Z) / APEX_Z
        length = (z1 - z0) * scale
        zc = (z0 + z1) / 2.0
        xc = xf * (1.0 - zc / APEX_Z)
        part.visual(
            Cylinder(radius=LEG_R, length=length),
            origin=Origin(xyz=(xc, ys, zc), rpy=(0.0, math.atan2(-xf, APEX_Z), 0.0)),
            material="steel_frame",
            name=f"leg_{tag}",
        )
        part.visual(
            Box((0.15, 0.10, PLATE_T)),
            origin=Origin(xyz=(xf + 0.01 * math.copysign(1.0, xf), ys, PLATE_T / 2.0)),
            material="steel_dark",
            name=f"foot_plate_{tag}",
        )
    part.visual(
        Cylinder(radius=0.032, length=0.11),
        origin=Origin(xyz=(0.0, ys, APEX_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel_dark",
        name="apex_sleeve",
    )
    part.visual(
        Cylinder(radius=BRACE_R, length=1.07),
        origin=Origin(xyz=(0.0, ys, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel_frame",
        name="cross_brace",
    )
    for tag, sgn in (("front", 1.0), ("rear", -1.0)):
        xe = sgn * FOOT_X * (1.0 - 1.15 / APEX_Z)
        dx, dz = xe - 0.0, 1.15 - 0.55
        length = math.hypot(dx, dz) + 0.03
        part.visual(
            Cylinder(radius=BRACE_R, length=length),
            origin=Origin(
                xyz=(dx / 2.0, ys, 0.55 + dz / 2.0),
                rpy=(0.0, math.atan2(dx, dz), 0.0),
            ),
            material="steel_frame",
            name=f"diag_brace_{tag}",
        )


def _build_chain(part, link_mesh) -> None:
    """Emit N_CHAIN_LINKS chain links hanging from the part origin (pivot).

    Each link is placed at a regular pitch below the pivot.  Adjacent links
    alternate between the XZ plane and the YZ plane (rotated 90 deg around Z),
    matching real chain construction where neighbouring links interlock at
    right angles.
    """
    for i in range(N_CHAIN_LINKS):
        z = -(LINK_HALF) - i * LINK_PITCH
        rot_z = 0.0 if i % 2 == 0 else math.pi / 2.0
        part.visual(
            link_mesh,
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, 0.0, rot_z)),
            material="steel_dark",
            name=f"chain_link_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metal_a_frame_swing_bench_chain")
    # Monochrome CAD-render palette: plain light gray metal.
    model.material("steel_light", rgba=(0.86, 0.87, 0.89, 1.0))
    model.material("steel_frame", rgba=(0.79, 0.80, 0.83, 1.0))
    model.material("steel_dark", rgba=(0.62, 0.64, 0.67, 1.0))

    # --------------------------------------------------------- A-frame ends
    a_frame_left = model.part("a_frame_left")
    _build_a_frame(a_frame_left, FRAME_Y)
    a_frame_right = model.part("a_frame_right")
    _build_a_frame(a_frame_right, -FRAME_Y)

    # ----------------------------------------- crossbar + bracket arms + lugs
    crossbar = model.part("crossbar")
    crossbar.visual(
        Cylinder(radius=BAR_R, length=1.78),
        origin=Origin(xyz=(0.0, 0.0, APEX_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel_frame",
        name="bar",
    )
    for tag, sy in (("left", 0.886), ("right", -0.886)):
        crossbar.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(0.0, sy, APEX_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="steel_dark",
            name=f"end_cap_{tag}",
        )

    # Horizontal bracket arms extending fore/aft from the crossbar to each
    # chain attachment point.  Each bracket overlaps the bar at x=0 and
    # carries a lug + pivot pin at its far end.
    for tag, cx, cy in _CHAIN_ANCHORS:
        # Bracket tube along X from inside the bar out to the chain x-position.
        bracket_x_start = -0.02 if cx >= 0 else cx - 0.01
        bracket_x_end = cx + 0.01 if cx >= 0 else 0.02
        bracket_len = bracket_x_end - bracket_x_start
        bracket_cx = (bracket_x_start + bracket_x_end) / 2.0
        crossbar.visual(
            Cylinder(radius=BRACKET_R, length=bracket_len),
            origin=Origin(
                xyz=(bracket_cx, cy, APEX_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="steel_frame",
            name=f"bracket_{tag}",
        )
        # Lug plate hanging below the bracket endpoint (touches the bracket).
        lug_top = APEX_Z + BRACKET_R
        lug_bot = PIVOT_Z - 0.005
        lug_h = lug_top - lug_bot
        lug_cz = (lug_top + lug_bot) / 2.0
        crossbar.visual(
            Box((0.04, 0.006, lug_h)),
            origin=Origin(xyz=(cx, cy, lug_cz)),
            material="steel_dark",
            name=f"lug_{tag}",
        )
        # Pivot pin through the lug at PIVOT_Z.
        crossbar.visual(
            Cylinder(radius=0.005, length=0.04),
            origin=Origin(xyz=(cx, cy, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="steel_dark",
            name=f"pivot_pin_{tag}",
        )

    # ------------------------------------ shared chain link mesh (one build)
    link_mesh = mesh_from_geometry(_make_chain_link_geom(), "chain_link")

    # ---------------------------------------------------- four hanging chains
    chains: dict[str, object] = {}
    for tag, cx, cy in _CHAIN_ANCHORS:
        name = f"chain_{tag}"
        chain = model.part(name)
        _build_chain(chain, link_mesh)
        chains[name] = chain

    # ------------------------------------------------------------ bench seat
    bench_seat = model.part("bench_seat")
    rail_half = SEAT_W / 2.0
    for tag, rx in (("front", SEAT_HALF_X), ("rear", -SEAT_HALF_X)):
        bench_seat.visual(
            Box((0.05, SEAT_W, 0.05)),
            origin=Origin(xyz=(rx, 0.0, 0.0)),
            material="steel_light",
            name=f"rail_{tag}",
        )
    for tag, ry in (("left", 0.675), ("right", -0.675)):
        bench_seat.visual(
            Box((2.0 * SEAT_HALF_X + 0.05, 0.05, 0.05)),
            origin=Origin(xyz=(0.0, ry, 0.0)),
            material="steel_light",
            name=f"rail_{tag}",
        )
    step = (SEAT_W - 0.14) / (N_SEAT_SLATS - 1)
    for k in range(N_SEAT_SLATS):
        sy = -(rail_half - 0.07) + k * step
        bench_seat.visual(
            Box((0.50, 0.054, 0.022)),
            origin=Origin(xyz=(0.01, sy, 0.035)),
            material="steel_light",
            name=f"seat_slat_{k}",
        )
    for tag, sgn in (("left", 1.0), ("right", -1.0)):
        bench_seat.visual(
            Box((0.045, 0.045, 0.27)),
            origin=Origin(xyz=(SEAT_HALF_X, sgn * 0.66, 0.155)),
            material="steel_light",
            name=f"arm_post_front_{tag}",
        )
        bench_seat.visual(
            Box((0.04, 0.04, 0.27)),
            origin=Origin(xyz=(-0.20, sgn * 0.66, 0.155)),
            material="steel_light",
            name=f"arm_post_rear_{tag}",
        )
        bench_seat.visual(
            Box((0.60, 0.06, BENCH_ARMREST_H)),
            origin=Origin(xyz=(0.0, sgn * 0.665, BENCH_ARMREST_Z)),
            material="steel_light",
            name=f"armrest_{tag}",
        )
        bench_seat.visual(
            Cylinder(radius=0.04, length=0.032),
            origin=Origin(xyz=(0.20, sgn * 0.64, BENCH_ARMREST_Z + 0.026)),
            material="steel_dark",
            name=f"cup_holder_{tag}",
        )

    # Chain anchor lugs + pins on the bench armrests.  The lug sits on top
    # of the armrest (touching it) and the horizontal pin passes through the
    # lug at the height of the last chain link centre.
    armrest_top = BENCH_ARMREST_Z + BENCH_ARMREST_H / 2.0
    lug_h = ANCHOR_BENCH_Z - armrest_top + 0.015  # lug extends above pin
    lug_cz = armrest_top + lug_h / 2.0
    for tag, cx, cy in _CHAIN_ANCHORS:
        bench_seat.visual(
            Box((0.035, 0.006, lug_h)),
            origin=Origin(xyz=(cx, cy, lug_cz)),
            material="steel_dark",
            name=f"anchor_lug_{tag}",
        )
        bench_seat.visual(
            Cylinder(radius=0.005, length=0.04),
            origin=Origin(
                xyz=(cx, cy, ANCHOR_BENCH_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="steel_dark",
            name=f"anchor_pin_{tag}",
        )

    # ------------------------------------------------ curved slatted back
    bench_back = model.part("bench_back")
    for k in range(N_BACK_SLATS):
        phi = 0.18 + k * BACK_DPHI
        px, pz = _arc_point(phi)
        bench_back.visual(
            Box((0.022, 1.36, 0.06)),
            origin=Origin(xyz=(px, 0.0, pz), rpy=(0.0, -phi, 0.0)),
            material="steel_light",
            name=f"back_slat_{k}",
        )
    for r, ry in (("center", 0.0), ("left", 0.60), ("right", -0.60)):
        for j in range(9):
            phi = 0.0757 + j * BACK_DPHI
            px, pz = _arc_point(phi)
            nx, nz = math.cos(phi), math.sin(phi)
            bench_back.visual(
                Box((0.02, 0.05, 0.082)),
                origin=Origin(
                    xyz=(px - 0.018 * nx, ry, pz - 0.018 * nz),
                    rpy=(0.0, -phi, 0.0),
                ),
                material="steel_frame",
                name=f"back_rib_{r}_{j}",
            )

    # ---------------------------------------------------------- kinematics
    model.articulation(
        "frame_left_to_right",
        ArticulationType.FIXED,
        parent=a_frame_left,
        child=a_frame_right,
    )
    model.articulation(
        "frame_to_crossbar",
        ArticulationType.FIXED,
        parent=a_frame_left,
        child=crossbar,
    )
    limits = MotionLimits(
        effort=200.0, velocity=2.0, lower=-SWING_LIMIT, upper=SWING_LIMIT,
    )
    # Driver pivot: front-left chain.  Positive q swings the bench rearward.
    model.articulation(
        "swing_drive",
        ArticulationType.REVOLUTE,
        parent=crossbar,
        child=chains["chain_front_left"],
        origin=Origin(xyz=(CHAIN_FRONT_X, CHAIN_Y, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    for jname, cname, cx, cy in (
        ("swing_follow_front_right", "chain_front_right", CHAIN_FRONT_X, -CHAIN_Y),
        ("swing_follow_rear_left", "chain_rear_left", CHAIN_REAR_X, CHAIN_Y),
        ("swing_follow_rear_right", "chain_rear_right", CHAIN_REAR_X, -CHAIN_Y),
    ):
        model.articulation(
            jname,
            ArticulationType.REVOLUTE,
            parent=crossbar,
            child=chains[cname],
            origin=Origin(xyz=(cx, cy, PIVOT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=limits,
            mimic=Mimic(joint="swing_drive", multiplier=1.0),
        )
    # Bench rides rigidly on the driver chain.
    model.articulation(
        "chain_to_bench",
        ArticulationType.FIXED,
        parent=chains["chain_front_left"],
        child=bench_seat,
        origin=Origin(xyz=(-CHAIN_FRONT_X, -CHAIN_Y, BENCH_Z - PIVOT_Z)),
    )
    model.articulation(
        "seat_to_back",
        ArticulationType.FIXED,
        parent=bench_seat,
        child=bench_back,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    af_l = object_model.get_part("a_frame_left")
    af_r = object_model.get_part("a_frame_right")
    crossbar = object_model.get_part("crossbar")
    seat = object_model.get_part("bench_seat")
    back = object_model.get_part("bench_back")
    chain_names = tuple(f"chain_{t}" for t, _, _ in _CHAIN_ANCHORS)
    chains = {n: object_model.get_part(n) for n in chain_names}
    drive = object_model.get_articulation("swing_drive")
    last_link = N_CHAIN_LINKS - 1

    # ------------------------------------------------------- allowances

    # Crossbar captured inside both apex sleeves.
    for af in (af_l, af_r):
        ctx.allow_overlap(
            crossbar, af,
            elem_a="bar", elem_b="apex_sleeve",
            reason="Crossbar tube is captured inside the A-frame apex sleeve.",
        )

    # Top chain link wraps around the crossbar pivot pin and sits against the lug.
    for tag, cx, cy in _CHAIN_ANCHORS:
        cname = f"chain_{tag}"
        ctx.allow_overlap(
            crossbar, chains[cname],
            elem_a=f"pivot_pin_{tag}", elem_b="chain_link_0",
            reason="Top chain link hangs from the crossbar pivot pin.",
        )
        ctx.allow_overlap(
            crossbar, chains[cname],
            elem_a=f"lug_{tag}", elem_b="chain_link_0",
            reason="Top chain link nests against the crossbar lug plate.",
        )

    # Bottom chain link wraps around the bench anchor pin and sits against the lug.
    for tag, cx, cy in _CHAIN_ANCHORS:
        cname = f"chain_{tag}"
        ctx.allow_overlap(
            chains[cname], seat,
            elem_a=f"chain_link_{last_link}", elem_b=f"anchor_pin_{tag}",
            reason="Bottom chain link is anchored to the bench armrest pin.",
        )
        ctx.allow_overlap(
            chains[cname], seat,
            elem_a=f"chain_link_{last_link}", elem_b=f"anchor_lug_{tag}",
            reason="Bottom chain link nests against the bench anchor lug plate.",
        )

    # Lowest back-rib segments bolt onto the rear seat rail.
    for r in ("center", "left", "right"):
        ctx.allow_overlap(
            back, seat,
            elem_a=f"back_rib_{r}_0", elem_b="rail_rear",
            reason="Curved back rib base is bolted to the rear seat rail.",
        )

    # -------------------------------------------------- chain structure

    def count(part, prefix: str) -> int:
        return sum(1 for v in part.visuals if v.name and v.name.startswith(prefix))

    ctx.check("four chains exist", len(chains) == 4)
    for cn in chain_names:
        n_links = count(chains[cn], "chain_link_")
        ctx.check(
            f"{cn} has {N_CHAIN_LINKS} oval chain links",
            n_links == N_CHAIN_LINKS,
            details=f"got {n_links}",
        )

    # Each chain's links use chain_link_i naming from a shared helper (numeric sort).
    for cn in chain_names:
        link_indices = sorted(
            int(v.name.split("_")[-1])
            for v in chains[cn].visuals
            if v.name and v.name.startswith("chain_link_")
        )
        expected = list(range(N_CHAIN_LINKS))
        ctx.check(
            f"{cn} link names follow chain_link_i pattern (0..{N_CHAIN_LINKS - 1})",
            link_indices == expected,
            details=f"got {link_indices[:5]}... expected {expected[:5]}...",
        )

    # ------------------------------------------------ swing drive / mimic

    lim = drive.motion_limits
    ctx.check(
        "driver swing limits ~ +/-0.45 rad",
        lim is not None
        and abs(lim.lower + SWING_LIMIT) < 1e-9
        and abs(lim.upper - SWING_LIMIT) < 1e-9,
    )
    followers = (
        "swing_follow_front_right",
        "swing_follow_rear_left",
        "swing_follow_rear_right",
    )
    for jn in followers:
        j = object_model.get_articulation(jn)
        ctx.check(
            f"{jn} mimics swing_drive with multiplier 1.0",
            j.mimic is not None
            and j.mimic.joint == "swing_drive"
            and abs(j.mimic.multiplier - 1.0) < 1e-9
            and abs(j.mimic.offset) < 1e-9,
        )

    # ----------------------------------------- frame stands on foot plates

    n_plates = 0
    for af, side in ((af_l, "left"), (af_r, "right")):
        for tag in ("front", "rear"):
            bb = ctx.part_element_world_aabb(af, elem=f"foot_plate_{tag}")
            if bb is not None:
                n_plates += 1
                ctx.check(
                    f"{side} {tag} foot plate rests flat on the ground",
                    abs(bb[0][2]) < 0.0005,
                    details=f"zmin={bb[0][2]:.4f}",
                )
    ctx.check("four flat foot plates", n_plates == 4)
    fb = ctx.part_world_aabb(af_l)
    if fb is not None:
        ctx.check(
            "A-frame is ~1.8 m tall with ~1.6 m leg spread",
            1.78 < fb[1][2] < 1.90 and 1.50 < (fb[1][0] - fb[0][0]) < 1.75,
            details=f"h={fb[1][2]:.2f} spread={fb[1][0] - fb[0][0]:.2f}",
        )

    # --------------------------------------------------------- slat counts

    n_seat = count(seat, "seat_slat_")
    ctx.check("seat is slatted (16 fore/aft slats)", n_seat == N_SEAT_SLATS, details=f"{n_seat}")
    n_back = count(back, "back_slat_")
    ctx.check("back is slatted (8 curved rows)", n_back == N_BACK_SLATS, details=f"{n_back}")
    b0 = ctx.part_element_world_aabb(back, elem="back_slat_0")
    b7 = ctx.part_element_world_aabb(back, elem=f"back_slat_{N_BACK_SLATS - 1}")
    if b0 is not None and b7 is not None:
        c0 = ((b0[0][0] + b0[1][0]) / 2.0, (b0[0][2] + b0[1][2]) / 2.0)
        c7 = ((b7[0][0] + b7[1][0]) / 2.0, (b7[0][2] + b7[1][2]) / 2.0)
        ctx.check(
            "back curves up and rolls rearward",
            (c7[1] - c0[1]) > 0.35 and (c0[0] - c7[0]) > 0.15,
            details=f"dz={c7[1] - c0[1]:.2f} dx_back={c0[0] - c7[0]:.2f}",
        )
    ctx.check("two cup holder armrest ends", count(seat, "cup_holder_") == 2)

    # ------------------------------ chain attachment eyes on crossbar + bench

    n_cross_pins = sum(
        1 for v in crossbar.visuals if v.name and v.name.startswith("pivot_pin_")
    )
    ctx.check("four pivot pins on the crossbar brackets", n_cross_pins == 4, details=f"{n_cross_pins}")
    n_anchors = sum(
        1 for v in seat.visuals if v.name and v.name.startswith("anchor_pin_")
    )
    ctx.check("four anchor pins on the bench armrests", n_anchors == 4, details=f"{n_anchors}")
    n_brackets = sum(
        1 for v in crossbar.visuals if v.name and v.name.startswith("bracket_")
    )
    ctx.check("four bracket arms on the crossbar", n_brackets == 4, details=f"{n_brackets}")

    # ---------------------------------- chain-to-crossbar and chain-to-bench

    ctx.expect_contact(
        crossbar, chains["chain_front_left"],
        elem_a="pivot_pin_front_left", elem_b="chain_link_0",
        name="front-left top chain link is seated on the crossbar pivot pin",
    )
    ctx.expect_contact(
        crossbar, chains["chain_rear_right"],
        elem_a="pivot_pin_rear_right", elem_b="chain_link_0",
        name="rear-right top chain link is seated on the crossbar pivot pin",
    )
    ctx.expect_contact(
        chains["chain_front_left"], seat,
        elem_a=f"chain_link_{last_link}", elem_b="anchor_pin_front_left",
        name="front-left bottom chain link is seated on the bench anchor pin",
    )
    ctx.expect_contact(
        crossbar, af_l,
        elem_a="bar", elem_b="apex_sleeve",
        name="crossbar is seated in the apex sleeve",
    )
    ctx.expect_contact(
        back, seat,
        elem_a="back_rib_center_0", elem_b="rail_rear",
        name="curved back is mounted on the rear seat rail",
    )

    # Verify the anchor lug sits on the armrest (not floating).
    ctx.expect_contact(
        seat, seat,
        elem_a="anchor_lug_front_left", elem_b="armrest_left",
        name="front-left anchor lug is seated on the left armrest",
    )

    # Verify brackets touch the crossbar.
    ctx.expect_contact(
        crossbar, crossbar,
        elem_a="bracket_front_left", elem_b="bar",
        name="front-left bracket is welded to the crossbar",
    )

    # ---------------------- swing clearance through the whole motion range

    bar_bb = ctx.part_world_aabb(crossbar)
    bar_zmin = bar_bb[0][2] if bar_bb is not None else APEX_Z - BAR_R
    centers: dict[float, float] = {}
    for q in (-SWING_LIMIT, 0.0, SWING_LIMIT):
        with ctx.pose({drive: q}):
            zmin, ymax = math.inf, 0.0
            for p in (seat, back, *chains.values()):
                bb = ctx.part_world_aabb(p)
                if bb is None:
                    continue
                zmin = min(zmin, bb[0][2])
                ymax = max(ymax, abs(bb[0][1]), abs(bb[1][1]))
            sb = ctx.part_world_aabb(seat)
            kb = ctx.part_world_aabb(back)
            centers[q] = (sb[0][0] + sb[1][0]) / 2.0 if sb is not None else 0.0
            ctx.check(
                f"bench clears the ground at q={q:+.2f}",
                zmin > 0.05,
                details=f"zmin={zmin:.3f}",
            )
            if sb is not None and kb is not None:
                bench_zmax = max(sb[1][2], kb[1][2])
                ctx.check(
                    f"bench hangs below the crossbar at q={q:+.2f}",
                    bench_zmax < bar_zmin - 0.05,
                    details=f"bench_zmax={bench_zmax:.3f} bar_zmin={bar_zmin:.3f}",
                )
            ctx.check(
                f"bench clears the A-frame legs sideways at q={q:+.2f}",
                ymax < FRAME_Y - LEG_R - 0.015,
                details=f"|y|max={ymax:.3f} leg_plane_inner={FRAME_Y - LEG_R:.3f}",
            )
    ctx.check(
        "driving the pivot actually swings the bench fore/aft",
        (centers[-SWING_LIMIT] - centers[SWING_LIMIT]) > 0.6,
        details=f"dx={centers[-SWING_LIMIT] - centers[SWING_LIMIT]:.3f}",
    )

    # Seat at sitting height at rest.
    sb = ctx.part_element_world_aabb(seat, elem="seat_slat_0")
    if sb is not None:
        ctx.check(
            "seat surface at sitting height (~0.65 m)",
            0.55 < sb[1][2] < 0.72,
            details=f"seat_top={sb[1][2]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
