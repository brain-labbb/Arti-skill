from __future__ import annotations

# A-frame log lawn swing with chain-hung seat unit (natural cedar logs,
# red corrugated gable roof).
#
# Frame convention:
#   X = fore/aft (the swing moves in X; the two seats face each other in X)
#   Y = left/right (top beam and roof ridge run along Y)
#   Z = up
#
# Structure: two A-frame ends of crossed round logs (tips poke through the
# roof near the ridge), a round top beam nested in the leg crotches, and a
# shallow red corrugated gable roof. The seat unit (slatted floor platform,
# two facing fan-back log seats, small center table) hangs from the beam on
# two chains (left and right). Each chain is a series of alternating
# oblong chain links. One revolute driver joint on the left chain plus a
# mimic follower on the right chain; the platform is rigidly fixed to the
# driver chain so the whole unit swings as one pendulum.

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

# ----------------------------------------------------------------- constants
LEG_R = 0.06           # A-frame leg log radius
BEAM_R = 0.06          # top beam log radius
BRACE_R = 0.05         # A-frame cross-brace log radius

AY = 1.07              # A-frame center planes at y = +/-AY
FOOT_X = 0.80          # leg foot x position
LEG_LEN = 2.36         # full leg log length (foot to tip past the crossing)
LEG_LIFT = 0.024       # foot end-center height so the tilted rim touches z=0
# Leg unit direction (foot at +x runs up toward -x).
_LEG_DX = 0.4006
_LEG_DZ = 0.9164

BEAM_Z = 1.88          # top beam axis height (also the swing pivot height)
BEAM_HALF_Y = 1.275

ROOF_P = math.radians(14.0)
ROOF_T = 0.022
ROOF_RUN = 0.78        # horizontal half-run of each slope
ROOF_HALF_Y = 1.375
RIDGE_UNDER_Z = 1.99   # roof panel underside height at the ridge
_TAN_P = math.tan(ROOF_P)
_SIN_P = math.sin(ROOF_P)
_COS_P = math.cos(ROOF_P)
ROOF_SLOPE_LEN = ROOF_RUN / _COS_P

CHAIN_Y = 0.60         # chain centerline planes at y = +/-CHAIN_Y
RAIL_Z = -1.24         # platform hanger rail height (pivot-relative)
SWING_LIMIT = 0.40     # rad

DROP = BEAM_Z          # platform geometry is authored pivot-relative (z - DROP)

# Chain link dimensions (oblong loop in XZ plane, long axis along Z).
LINK_LEN = 0.110       # outer length along the hanging axis (Z)
LINK_WID = 0.046       # outer width (X for even links, Y for odd links)
LINK_ROD_R = 0.006     # rod cross-section radius
N_LINKS = 13           # links per chain
LINK_PITCH = abs(RAIL_Z) / (N_LINKS - 1)  # center-to-center spacing


def _log_between(part, name, a, b, r, material, extend_a=0.0, extend_b=0.0):
    """Place a round log (cylinder) between two 3D end-center points."""
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    ux, uy, uz = dx / length, dy / length, dz / length
    a2 = (a[0] - ux * extend_a, a[1] - uy * extend_a, a[2] - uz * extend_a)
    b2 = (b[0] + ux * extend_b, b[1] + uy * extend_b, b[2] + uz * extend_b)
    center = ((a2[0] + b2[0]) / 2.0, (a2[1] + b2[1]) / 2.0, (a2[2] + b2[2]) / 2.0)
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux) if (abs(ux) > 1e-12 or abs(uy) > 1e-12) else 0.0
    part.visual(
        Cylinder(radius=r, length=length + extend_a + extend_b),
        origin=Origin(xyz=center, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _leg_endpoints(foot_sign: float):
    """Foot and tip end centers of one A-frame leg (in its wall plane)."""
    foot = (foot_sign * FOOT_X, 0.0, LEG_LIFT)
    tip = (
        foot[0] - foot_sign * _LEG_DX * LEG_LEN,
        0.0,
        foot[2] + _LEG_DZ * LEG_LEN,
    )
    return foot, tip


def _build_a_frame(part, wy: float) -> None:
    # leg_0 foot at +x (tip crosses to -x), leg_1 foot at -x (tip at +x).
    for i, fs in enumerate((1.0, -1.0)):
        foot, tip = _leg_endpoints(fs)
        _log_between(
            part,
            f"leg_{i}",
            (foot[0], wy, foot[2]),
            (tip[0], wy, tip[2]),
            LEG_R,
            "cedar",
        )
    # Horizontal cross-brace log between the two legs.
    _log_between(
        part,
        "cross_brace",
        (-0.575, wy, 0.72),
        (0.575, wy, 0.72),
        BRACE_R,
        "cedar_dark",
    )


def _build_roof_panel(part, side: float) -> None:
    """One corrugated roof slope. side=+1 covers x>0, side=-1 covers x<0."""
    mid = 0.392  # horizontal distance from ridge to panel center
    zc = RIDGE_UNDER_Z + (ROOF_T / 2.0) / _COS_P - mid * _TAN_P
    pitch = side * ROOF_P
    part.visual(
        Box((ROOF_SLOPE_LEN, 2.0 * ROOF_HALF_Y, ROOF_T)),
        origin=Origin(xyz=(side * mid, 0.0, zc), rpy=(0.0, pitch, 0.0)),
        material="roof_red",
        name="sheet",
    )
    # Corrugation ribs run DOWN the slope, repeated across the width.
    nx, nz = side * _SIN_P, _COS_P  # outward sheet normal
    n_rib = 16
    for j in range(n_rib):
        ry = -ROOF_HALF_Y + (j + 0.5) * (2.0 * ROOF_HALF_Y / n_rib)
        part.visual(
            Box((ROOF_SLOPE_LEN - 0.03, 0.034, 0.016)),
            origin=Origin(
                xyz=(side * mid + 0.016 * nx, ry, zc + 0.016 * nz),
                rpy=(0.0, pitch, 0.0),
            ),
            material="roof_red_dark",
            name=f"rib_{j}",
        )


def _build_seat(part, s: float) -> None:
    """One fan-back log seat facing the center. s=+1 builds at x>0 facing -x."""
    lean = math.radians(14.0)
    sl, cl = math.sin(lean), math.cos(lean)

    # Short legs standing on the deck slats (slats at x = s*0.22 and s*0.59).
    for i, (lx, ly) in enumerate(
        ((0.22, -0.44), (0.22, 0.44), (0.59, -0.44), (0.59, 0.44))
    ):
        _log_between(
            part,
            f"leg_{i}",
            (s * lx, ly, -1.521),
            (s * lx, ly, -1.29),
            0.03,
            "cedar_dark",
        )
    # Seat frame rails along Y plus side stringers along X (so every slat
    # spanning the seat width is carried by the stringers).
    for tag, rx in (("inner", 0.22), ("outer", 0.59)):
        _log_between(
            part,
            f"seat_rail_{tag}",
            (s * rx, -0.49, -1.285),
            (s * rx, 0.49, -1.285),
            0.032,
            "cedar_dark",
        )
    for i, sy in enumerate((-0.43, 0.43)):
        _log_between(
            part,
            f"seat_stringer_{i}",
            (s * 0.19, sy, -1.285),
            (s * 0.62, sy, -1.285),
            0.032,
            "cedar_dark",
        )
    # Slightly dished seat slats.
    for i, (sx, dz) in enumerate(
        ((0.23, -0.002), (0.348, -0.010), (0.466, -0.010), (0.584, -0.002))
    ):
        part.visual(
            Box((0.095, 1.00, 0.024)),
            origin=Origin(xyz=(s * sx, 0.0, -1.241 + dz)),
            material="cedar",
            name=f"seat_slat_{i}",
        )
    # Outward-leaning back posts.
    base_z = -1.285
    for i, py in enumerate((-0.45, 0.45)):
        base = (s * 0.60, py, base_z)
        top = (s * (0.60 + sl * 0.66), py, base_z + cl * 0.66)
        _log_between(part, f"back_post_{i}", base, top, 0.032, "cedar_dark")
    # Two back cross rails on the seat-side face of the posts.
    for i, t in enumerate((0.18, 0.50)):
        xr = 0.60 + sl * t - 0.045
        zr = base_z + cl * t
        _log_between(
            part,
            f"back_rail_{i}",
            (s * xr, -0.48, zr),
            (s * xr, 0.48, zr),
            0.028,
            "cedar_dark",
        )
    # Fan of vertical back slats (center tallest), mounted on the cross rails.
    heights = (0.40, 0.50, 0.58, 0.50, 0.40)
    for i, (sy, h) in enumerate(zip((-0.36, -0.18, 0.0, 0.18, 0.36), heights)):
        tc = 0.10 + h / 2.0
        part.visual(
            Box((0.020, 0.10, h)),
            origin=Origin(
                xyz=(s * (0.519 + sl * tc), sy, base_z + cl * tc),
                rpy=(0.0, s * lean, 0.0),
            ),
            material="cedar",
            name=f"back_slat_{i}",
        )
    # Armrests with front arm posts.
    for i, ay in enumerate((-0.46, 0.46)):
        _log_between(
            part,
            f"armrest_{i}",
            (s * 0.20, ay, -1.13),
            (s * 0.70, ay, -1.13),
            0.028,
            "cedar",
        )
        _log_between(
            part,
            f"arm_post_{i}",
            (s * 0.24, ay, -1.30),
            (s * 0.24, ay, -1.12),
            0.026,
            "cedar_dark",
        )


# ----------------------------------------------------- chain link geometry
def _chain_link_path(half_l: float, half_w: float):
    """Sample an oblong chain-link centerline path in the XZ plane.

    The oblong has straight sides at x = +/-half_w and semicircular end
    arcs of radius half_w centered at z = +/-half_l.
    """
    pts = []
    n_s = 3   # samples per straight side
    n_a = 8   # samples per semicircular arc
    # Right side (bottom to top)
    for i in range(n_s):
        f = i / max(1, n_s - 1)
        pts.append((half_w, 0.0, -half_l + f * 2.0 * half_l))
    # Top arc (right to left, counterclockwise viewed from +Y)
    for i in range(n_a):
        a = math.pi * (i + 1) / (n_a + 1)
        pts.append((half_w * math.cos(a), 0.0, half_l + half_w * math.sin(a)))
    # Left side (top to bottom)
    for i in range(n_s):
        f = i / max(1, n_s - 1)
        pts.append((-half_w, 0.0, half_l - f * 2.0 * half_l))
    # Bottom arc (left to right)
    for i in range(n_a):
        a = math.pi + math.pi * (i + 1) / (n_a + 1)
        pts.append((half_w * math.cos(a), 0.0, -half_l + half_w * math.sin(a)))
    return pts


def _make_chain_link_mesh():
    """Build one oblong chain link as a closed tube mesh in the XZ plane."""
    half_l = (LINK_LEN - LINK_WID) / 2.0
    half_w = LINK_WID / 2.0
    pts = _chain_link_path(half_l, half_w)
    return mesh_from_geometry(
        tube_from_spline_points(
            pts,
            radius=LINK_ROD_R,
            samples_per_segment=8,
            closed_spline=True,
            radial_segments=10,
            cap_ends=False,
        ),
        "chain_link",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a_frame_log_chain_swing")
    model.material("cedar", rgba=(0.85, 0.71, 0.52, 1.0))
    model.material("cedar_dark", rgba=(0.76, 0.61, 0.43, 1.0))
    model.material("roof_red", rgba=(0.72, 0.15, 0.13, 1.0))
    model.material("roof_red_dark", rgba=(0.60, 0.11, 0.10, 1.0))
    model.material("chain_iron", rgba=(0.32, 0.31, 0.33, 1.0))

    # ------------------------------------------------------- top beam (root)
    beam = model.part("top_beam")
    _log_between(
        beam,
        "beam_log",
        (0.0, -BEAM_HALF_Y, BEAM_Z),
        (0.0, BEAM_HALF_Y, BEAM_Z),
        BEAM_R,
        "cedar",
    )
    # Ridge board on top of the beam carries the two roof panels.
    beam.visual(
        Box((0.10, 2.40, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 1.955)),
        material="cedar_dark",
        name="ridge_board",
    )

    # ------------------------------------------------------ A-frame end walls
    a_left = model.part("a_frame_left")
    _build_a_frame(a_left, -AY)
    a_right = model.part("a_frame_right")
    _build_a_frame(a_right, AY)
    model.articulation(
        "beam_to_a_frame_left", ArticulationType.FIXED, parent=beam, child=a_left
    )
    model.articulation(
        "beam_to_a_frame_right", ArticulationType.FIXED, parent=beam, child=a_right
    )

    # -------------------------------------------------------------- red roof
    roof_front = model.part("roof_panel_front")
    _build_roof_panel(roof_front, 1.0)
    roof_rear = model.part("roof_panel_rear")
    _build_roof_panel(roof_rear, -1.0)
    model.articulation(
        "beam_to_roof_front", ArticulationType.FIXED, parent=beam, child=roof_front
    )
    model.articulation(
        "beam_to_roof_rear", ArticulationType.FIXED, parent=beam, child=roof_rear
    )

    # --------------------------------------------------------- hanging chains
    # Build the shared chain-link mesh once, then stamp it into each chain
    # part with regular vertical placement and alternating 90-degree yaw.
    link_mesh = _make_chain_link_mesh()
    half_yaw = math.pi / 2.0

    limits = MotionLimits(
        effort=250.0, velocity=2.0, lower=-SWING_LIMIT, upper=SWING_LIMIT
    )

    chain_specs = (
        ("chain_left", -CHAIN_Y),
        ("chain_right", CHAIN_Y),
    )
    chains = {}
    for chain_name, py in chain_specs:
        chain = model.part(chain_name)
        for i in range(N_LINKS):
            z = -i * LINK_PITCH
            yaw = 0.0 if i % 2 == 0 else half_yaw
            chain.visual(
                link_mesh,
                origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, 0.0, yaw)),
                material="chain_iron",
                name=f"chain_link_{i}",
            )
        chains[chain_name] = chain

    # Driver revolute on the left chain; mimic on the right.
    model.articulation(
        "swing_pivot",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=chains["chain_left"],
        origin=Origin(xyz=(0.0, -CHAIN_Y, BEAM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "chain_right_pivot",
        ArticulationType.REVOLUTE,
        parent=beam,
        child=chains["chain_right"],
        origin=Origin(xyz=(0.0, CHAIN_Y, BEAM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
        mimic=Mimic(joint="swing_pivot", multiplier=1.0, offset=0.0),
    )

    # -------------------------------------------------------- glider platform
    # Platform frame = pivot frame centered between the chains (world 0,0,BEAM_Z
    # at q=0); all platform geometry uses pivot-relative z (world z - DROP).
    plat = model.part("glider_platform")
    for tag, ry in (("left", -0.60), ("right", 0.60)):
        _log_between(
            plat,
            f"hanger_rail_{tag}",
            (-0.75, ry, RAIL_Z),
            (0.75, ry, RAIL_Z),
            0.042,
            "cedar",
        )
    for i, (px, py) in enumerate(
        ((-0.66, -0.55), (-0.66, 0.55), (0.66, -0.55), (0.66, 0.55))
    ):
        _log_between(
            plat,
            f"corner_post_{i}",
            (px, py, -1.56),
            (px, py, -1.215),
            0.035,
            "cedar_dark",
        )
    for i, cx in enumerate((-0.66, 0.66)):
        _log_between(
            plat,
            f"end_cross_{i}",
            (cx, -0.59, -1.555),
            (cx, 0.59, -1.555),
            0.04,
            "cedar_dark",
        )
    for i, jy in enumerate((-0.50, 0.50)):
        _log_between(
            plat,
            f"deck_joist_{i}",
            (-0.72, jy, -1.575),
            (0.72, jy, -1.575),
            0.035,
            "cedar_dark",
        )
    for i, sx in enumerate((-0.59, -0.405, -0.22, 0.0, 0.22, 0.405, 0.59)):
        plat.visual(
            Box((0.085, 1.16, 0.026)),
            origin=Origin(xyz=(sx, 0.0, -1.530)),
            material="cedar",
            name=f"deck_slat_{i}",
        )
    model.articulation(
        "chain_to_platform",
        ArticulationType.FIXED,
        parent=chains["chain_left"],
        child=plat,
        origin=Origin(xyz=(0.0, CHAIN_Y, 0.0)),
    )

    # ------------------------------------------------------- two facing seats
    seat_front = model.part("seat_front")
    _build_seat(seat_front, 1.0)
    seat_rear = model.part("seat_rear")
    _build_seat(seat_rear, -1.0)
    model.articulation(
        "platform_to_seat_front", ArticulationType.FIXED, parent=plat, child=seat_front
    )
    model.articulation(
        "platform_to_seat_rear", ArticulationType.FIXED, parent=plat, child=seat_rear
    )

    # ----------------------------------------------------------- center table
    table = model.part("center_table")
    for i, py in enumerate((-0.16, 0.16)):
        _log_between(
            table, f"post_{i}", (0.0, py, -1.521), (0.0, py, -1.34), 0.030, "cedar_dark"
        )
    for i, ry in enumerate((-0.175, 0.175)):
        _log_between(
            table,
            f"table_rail_{i}",
            (-0.17, ry, -1.345),
            (0.17, ry, -1.345),
            0.022,
            "cedar_dark",
        )
    for i, tx in enumerate((-0.13125, -0.04375, 0.04375, 0.13125)):
        table.visual(
            Box((0.085, 0.38, 0.022)),
            origin=Origin(xyz=(tx, 0.0, -1.314)),
            material="cedar",
            name=f"table_slat_{i}",
        )
    model.articulation(
        "platform_to_center_table", ArticulationType.FIXED, parent=plat, child=table
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("top_beam")
    a_left = object_model.get_part("a_frame_left")
    a_right = object_model.get_part("a_frame_right")
    roof_front = object_model.get_part("roof_panel_front")
    roof_rear = object_model.get_part("roof_panel_rear")
    plat = object_model.get_part("glider_platform")
    seat_front = object_model.get_part("seat_front")
    seat_rear = object_model.get_part("seat_rear")
    table = object_model.get_part("center_table")
    chain_l = object_model.get_part("chain_left")
    chain_r = object_model.get_part("chain_right")
    driver = object_model.get_articulation("swing_pivot")

    # ----------------------------------------------------- intended embeddings
    for a_frame in (a_left, a_right):
        for leg in ("leg_0", "leg_1"):
            ctx.allow_overlap(
                beam,
                a_frame,
                elem_a="beam_log",
                elem_b=leg,
                reason="Top beam log nests in the crotch of the crossed A-frame legs.",
            )
            ctx.allow_overlap(
                beam,
                a_frame,
                elem_a="ridge_board",
                elem_b=leg,
                reason="Crossed leg tips pass beside the ridge board on their way up.",
            )
    # Leg tips pierce the roof right at the ridge seam.
    for panel in (roof_front, roof_rear):
        for a_frame, rib in ((a_left, "rib_1"), (a_right, "rib_14")):
            for leg in ("leg_0", "leg_1"):
                ctx.allow_overlap(
                    panel,
                    a_frame,
                    elem_a="sheet",
                    elem_b=leg,
                    reason="Crossed leg tip emerges through the roof at the ridge seam.",
                )
                ctx.allow_overlap(
                    panel,
                    a_frame,
                    elem_a=rib,
                    elem_b=leg,
                    reason="Emerging leg tip also passes the corrugation rib above it.",
                )
    for panel in (roof_front, roof_rear):
        ctx.allow_overlap(
            panel,
            beam,
            elem_a="sheet",
            elem_b="ridge_board",
            reason="Roof sheet is seated on (slightly let into) the ridge board.",
        )

    # Top chain links wrap over the beam log (captured pivot).
    for chain in (chain_l, chain_r):
        ctx.allow_overlap(
            beam,
            chain,
            elem_a="beam_log",
            elem_b="chain_link_0",
            reason="Top chain link loops over the beam log (captured pivot).",
        )
        ctx.allow_overlap(
            beam,
            chain,
            elem_a="beam_log",
            elem_b="chain_link_1",
            reason="Second chain link grazes the beam as the chain wraps over it.",
        )
        ctx.allow_overlap(
            beam,
            chain,
            elem_a="ridge_board",
            elem_b="chain_link_0",
            reason="Top chain link grazes the ridge board underside as the chain hooks over the beam.",
        )
    # Bottom chain link attaches to the hanger rail.
    last = N_LINKS - 1
    ctx.allow_overlap(
        plat,
        chain_l,
        elem_a="hanger_rail_left",
        elem_b=f"chain_link_{last}",
        reason="Bottom chain link is shackled to the left hanger rail.",
    )
    ctx.allow_overlap(
        plat,
        chain_r,
        elem_a="hanger_rail_right",
        elem_b=f"chain_link_{last}",
        reason="Bottom chain link is shackled to the right hanger rail.",
    )

    # Seat and table mountings.
    for seat, slats in ((seat_front, ("deck_slat_4", "deck_slat_6")),
                        (seat_rear, ("deck_slat_2", "deck_slat_0"))):
        for leg, slat in (
            ("leg_0", slats[0]),
            ("leg_1", slats[0]),
            ("leg_2", slats[1]),
            ("leg_3", slats[1]),
        ):
            ctx.allow_overlap(
                plat,
                seat,
                elem_a=slat,
                elem_b=leg,
                reason="Seat leg foot is screwed down into the deck slat.",
            )
    for post in ("post_0", "post_1"):
        ctx.allow_overlap(
            plat,
            table,
            elem_a="deck_slat_3",
            elem_b=post,
            reason="Table post is screwed down into the center deck slat.",
        )

    # ------------------------------------------------------- mounted contacts
    ctx.expect_contact(
        beam, a_left, elem_a="beam_log", elem_b="leg_0",
        name="beam seated in the left A-frame crotch",
    )
    ctx.expect_contact(
        beam, a_right, elem_a="beam_log", elem_b="leg_0",
        name="beam seated in the right A-frame crotch",
    )
    ctx.expect_contact(
        roof_front, beam, elem_a="sheet", elem_b="ridge_board",
        name="front roof panel seated on the ridge board",
    )
    ctx.expect_contact(
        roof_rear, beam, elem_a="sheet", elem_b="ridge_board",
        name="rear roof panel seated on the ridge board",
    )
    # Top chain links contact the beam.
    for tag, chain in (("left", chain_l), ("right", chain_r)):
        ctx.expect_contact(
            beam, chain, elem_a="beam_log", elem_b="chain_link_0",
            name=f"{tag} chain hooks onto the top beam",
        )
    # Bottom chain links contact the hanger rails.
    ctx.expect_contact(
        chain_l, plat, elem_a=f"chain_link_{last}", elem_b="hanger_rail_left",
        name="left chain bottom link meets the left hanger rail",
    )
    ctx.expect_contact(
        chain_r, plat, elem_a=f"chain_link_{last}", elem_b="hanger_rail_right",
        name="right chain bottom link meets the right hanger rail",
    )
    ctx.expect_contact(
        seat_front, plat, elem_a="leg_0", elem_b="deck_slat_4",
        name="front seat stands on the deck",
    )
    ctx.expect_contact(
        seat_rear, plat, elem_a="leg_0", elem_b="deck_slat_2",
        name="rear seat stands on the deck",
    )
    ctx.expect_contact(
        table, plat, elem_a="post_0", elem_b="deck_slat_3",
        name="center table is supported off the deck",
    )

    # ------------------------------------------------------ static frame tests
    def count(part, prefix: str) -> int:
        return sum(1 for v in part.visuals if v.name and v.name.startswith(prefix))

    for tag, af in (("left", a_left), ("right", a_right)):
        ab = ctx.part_world_aabb(af)
        if ab is not None:
            ctx.check(
                f"{tag} A-frame feet rest on the ground",
                abs(ab[0][2]) < 0.03,
                details=f"zmin={ab[0][2]:.3f}",
            )
        ctx.check(f"{tag} A-frame has two crossed legs", count(af, "leg_") == 2)
        ctx.check(
            f"{tag} A-frame has a cross brace",
            af.get_visual("cross_brace") is not None,
        )

    beam_bb = ctx.part_element_world_aabb(beam, elem="beam_log")
    sheet_f = ctx.part_element_world_aabb(roof_front, elem="sheet")
    sheet_r = ctx.part_element_world_aabb(roof_rear, elem="sheet")
    aleft_bb = ctx.part_world_aabb(a_left)
    if beam_bb is not None and sheet_f is not None and sheet_r is not None:
        ctx.check(
            "roof ridge sits above the top beam",
            sheet_f[1][2] > beam_bb[1][2] and sheet_r[1][2] > beam_bb[1][2],
            details=f"ridge_z={sheet_f[1][2]:.2f} beam_top={beam_bb[1][2]:.2f}",
        )
        ctx.check(
            "two roof slopes pitch down on opposite sides",
            sheet_f[1][0] > 0.7 and sheet_r[0][0] < -0.7,
            details=f"front_xmax={sheet_f[1][0]:.2f} rear_xmin={sheet_r[0][0]:.2f}",
        )
    if sheet_f is not None and aleft_bb is not None:
        ctx.check(
            "roof overhangs the A-frames sideways",
            sheet_f[1][1] > AY + LEG_R + 0.15 and sheet_f[0][1] < -(AY + LEG_R + 0.15),
            details=f"roof_ymax={sheet_f[1][1]:.2f}",
        )
        ctx.check(
            "crossed log tips poke up past the roof (as in the photo)",
            aleft_bb[1][2] > sheet_f[1][2] + 0.10,
            details=f"tip_z={aleft_bb[1][2]:.2f} roof_zmax={sheet_f[1][2]:.2f}",
        )
    n_rib = count(roof_front, "rib_") + count(roof_rear, "rib_")
    ctx.check("roof panels are corrugated (many ribs)", n_rib == 32, details=f"{n_rib}")

    # --------------------------------------------------------- chain structure
    for tag, chain in (("left", chain_l), ("right", chain_r)):
        n_links = count(chain, "chain_link_")
        ctx.check(
            f"{tag} chain has {N_LINKS} links",
            n_links == N_LINKS,
            details=f"found {n_links}",
        )
    # Driver joint is revolute with Y axis.
    ctx.check(
        "swing pivot is a revolute joint on the Y axis",
        driver.articulation_type == ArticulationType.REVOLUTE
        and driver.axis is not None
        and abs(driver.axis[1]) > 0.99,
    )
    # Right chain mimics the driver.
    right_joint = object_model.get_articulation("chain_right_pivot")
    ctx.check(
        "right chain mimics the driver with multiplier 1.0",
        right_joint.mimic is not None
        and right_joint.mimic.joint == "swing_pivot"
        and abs(right_joint.mimic.multiplier - 1.0) < 1e-9
        and abs(right_joint.mimic.offset) < 1e-9,
    )
    # Alternating link orientations: even links in XZ (yaw=0),
    # odd links in YZ (yaw=pi/2).
    for chain in (chain_l, chain_r):
        even_wider_x = True
        odd_wider_y = True
        for i in range(0, min(6, N_LINKS), 2):
            bb = ctx.part_element_world_aabb(chain, elem=f"chain_link_{i}")
            if bb is not None:
                dx = bb[1][0] - bb[0][0]
                dy = bb[1][1] - bb[0][1]
                if dx < dy:
                    even_wider_x = False
        for i in range(1, min(7, N_LINKS), 2):
            bb = ctx.part_element_world_aabb(chain, elem=f"chain_link_{i}")
            if bb is not None:
                dx = bb[1][0] - bb[0][0]
                dy = bb[1][1] - bb[0][1]
                if dy < dx:
                    odd_wider_y = False
        tag = "left" if chain is chain_l else "right"
        ctx.check(
            f"{tag} chain even links lie in the swing plane (XZ)",
            even_wider_x,
        )
        ctx.check(
            f"{tag} chain odd links are rotated 90 degrees (YZ)",
            odd_wider_y,
        )

    # ---------------------------------------------------------- seat unit tests
    sf_bb = ctx.part_world_aabb(seat_front)
    sr_bb = ctx.part_world_aabb(seat_rear)
    if sf_bb is not None and sr_bb is not None:
        ctx.check(
            "two seats sit on opposite sides of the platform",
            sf_bb[0][0] > 0.10 and sr_bb[1][0] < -0.10,
            details=f"front_xmin={sf_bb[0][0]:.2f} rear_xmax={sr_bb[1][0]:.2f}",
        )
    # Backs are outboard of the seat pans -> the seats face each other.
    f_back = ctx.part_element_world_aabb(seat_front, elem="back_slat_2")
    f_pan = ctx.part_element_world_aabb(seat_front, elem="seat_slat_1")
    r_back = ctx.part_element_world_aabb(seat_rear, elem="back_slat_2")
    r_pan = ctx.part_element_world_aabb(seat_rear, elem="seat_slat_1")
    if all(b is not None for b in (f_back, f_pan, r_back, r_pan)):
        f_face = (f_back[0][0] + f_back[1][0]) / 2.0 > (f_pan[0][0] + f_pan[1][0]) / 2.0
        r_face = (r_back[0][0] + r_back[1][0]) / 2.0 < (r_pan[0][0] + r_pan[1][0]) / 2.0
        ctx.check("seat backs are outboard, seats face each other", f_face and r_face)
    for tag, seat in (("front", seat_front), ("rear", seat_rear)):
        ctx.check(f"{tag} seat pan is slatted", count(seat, "seat_slat_") == 4)
        ctx.check(f"{tag} seat back is a slat fan", count(seat, "back_slat_") == 5)
        ctx.check(f"{tag} seat has two armrests", count(seat, "armrest_") == 2)

    t_bb = ctx.part_world_aabb(table)
    if t_bb is not None and sf_bb is not None and sr_bb is not None:
        ctx.check(
            "center table sits between the facing seats",
            t_bb[0][0] > sr_bb[1][0] and t_bb[1][0] < sf_bb[0][0],
            details=f"table_x=({t_bb[0][0]:.2f},{t_bb[1][0]:.2f})",
        )
        ctx.check(
            "table top below seat tops, above the deck",
            0.40 < t_bb[1][2] < sf_bb[1][2],
            details=f"table_top={t_bb[1][2]:.2f}",
        )

    pb = ctx.part_world_aabb(plat)
    if pb is not None:
        ctx.check(
            "platform hangs above the ground",
            pb[0][2] > 0.20,
            details=f"zmin={pb[0][2]:.3f}",
        )

    # Overall silhouette scale.
    full = None
    for p in (beam, a_left, a_right, roof_front, roof_rear):
        bb = ctx.part_world_aabb(p)
        if bb is None:
            continue
        if full is None:
            full = [list(bb[0]), list(bb[1])]
        else:
            full[0] = [min(a, b) for a, b in zip(full[0], bb[0])]
            full[1] = [max(a, b) for a, b in zip(full[1], bb[1])]
    if full is not None:
        w, h, d = full[1][1] - full[0][1], full[1][2], full[1][0] - full[0][0]
        ctx.check(
            "frame is ~2.75 m wide, ~2.2 m tall, ~1.7 m deep",
            2.6 < w < 2.9 and 2.05 < h < 2.35 and 1.55 < d < 1.90,
            details=f"w={w:.2f} h={h:.2f} d={d:.2f}",
        )

    # ----------------------------------------------- swing motion at limits
    swing_parts = (plat, seat_front, seat_rear, table, chain_l, chain_r)

    def swing_aabb():
        lo = [math.inf] * 3
        hi = [-math.inf] * 3
        for p in swing_parts:
            bb = ctx.part_world_aabb(p)
            if bb is None:
                return None
            lo = [min(a, b) for a, b in zip(lo, bb[0])]
            hi = [max(a, b) for a, b in zip(hi, bb[1])]
        return lo, hi

    roof_plane = RIDGE_UNDER_Z
    centers = {}
    for q, tag, high_seat, high_slat in (
        (SWING_LIMIT, "rearward", seat_rear, "back_slat_2"),
        (-SWING_LIMIT, "forward", seat_front, "back_slat_2"),
    ):
        with ctx.pose({driver: q}):
            gb = swing_aabb()
            if gb is not None:
                centers[tag] = (gb[0][0] + gb[1][0]) / 2.0
                ctx.check(
                    f"swing unit clears the ground swung {tag}",
                    gb[0][2] > 0.08,
                    details=f"zmin={gb[0][2]:.3f}",
                )
                ctx.check(
                    f"swing unit stays clear of the A-frame legs swung {tag}",
                    gb[1][1] < AY - LEG_R - 0.05 and gb[0][1] > -(AY - LEG_R - 0.05),
                    details=f"ymax={gb[1][1]:.3f}",
                )
            top = ctx.part_element_world_aabb(high_seat, elem=high_slat)
            if top is not None:
                worst = top[1][2] + _TAN_P * max(abs(top[0][0]), abs(top[1][0]))
                ctx.check(
                    f"rising seat back stays under the roof slope swung {tag}",
                    worst <= roof_plane - 0.02,
                    details=f"plane_sum={worst:.3f} limit={roof_plane:.3f}",
                )
    if "rearward" in centers and "forward" in centers:
        ctx.check(
            "swing unit translates fore/aft as one body",
            centers["forward"] - centers["rearward"] > 0.6,
            details=f"shift={centers['forward'] - centers['rearward']:.2f}",
        )

    return ctx.report()
