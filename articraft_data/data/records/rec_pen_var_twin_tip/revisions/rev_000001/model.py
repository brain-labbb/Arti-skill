from __future__ import annotations

# DUAL-ENDED TWIN-TIP MARKER (twin-tip variant of the STABILO BOSS highlighter).
#
# A rounded-rectangular lime barrel with a chisel nib and pull-off cap at each
# end. Each cap is a separate part with a PRISMATIC joint along the pen's long
# axis (+X for the front cap, -X for the rear cap). At q=0 each cap is seated
# over its nib; positive q draws that cap outward to expose the nib.
#
# Frame convention:
#   +X = pen length (front of pen at +X, rear at -X)
#   cross-section lies in the Y-Z plane (Y = width, Z = height)
#   barrel is centered on the origin

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
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BARREL_LEN = 0.090       # length of the lime body (rear shoulder to front shoulder)
BARREL_W = 0.0170       # body width (Y)
BARREL_H = 0.0120       # body height (Z)
BODY_CORNER_R = 0.0030  # rounded corners of the rectangular section

COLLAR_LEN = 0.0060     # stepped collar where each cap registers
COLLAR_W = 0.0150
COLLAR_H = 0.0105
COLLAR_CORNER_R = 0.0028

NIB_BASE_LEN = 0.0080   # straight black holder after the collar
NIB_BASE_W = 0.0120
NIB_BASE_H = 0.0090
NIB_WEDGE_LEN = 0.0120  # tapered chisel section
NIB_TIP_W = 0.0090
NIB_TIP_H = 0.0018      # thin chisel edge

CAP_LEN = 0.0420        # hollow black cap
CAP_OUTER_W = 0.0182
CAP_OUTER_H = 0.0132
CAP_WALL = 0.0014
CAP_CORNER_R = 0.0034
CAP_CLIP_LEN = 0.0260   # pocket-clip flat along the cap top

CAP_SEAT_OVERLAP = 0.0060  # how far each cap mouth slides onto the barrel

# Materials
LIME = (0.82, 0.93, 0.13, 1.0)
BLACK = (0.07, 0.07, 0.08, 1.0)
FELT = (0.78, 0.90, 0.15, 1.0)

# Derived
HALF_BARREL = BARREL_LEN / 2.0


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _rounded_rect_prism(length: float, width: float, height: float, corner_r: float):
    """Rounded-rect prism: long axis +X, centered on Y/Z, X in [0, length]."""
    return (
        cq.Workplane("YZ")
        .rect(width, height)
        .extrude(length)
        .edges("|X")
        .fillet(corner_r)
    )


def _flip_x(solid):
    """Mirror a CQ solid across the YZ plane by rotating 180° around Z."""
    return solid.rotate((0, 0, 0), (0, 0, 1), 180)


def _build_barrel():
    """Centered lime barrel body with stepped collars at both ends."""
    body = _rounded_rect_prism(
        BARREL_LEN, BARREL_W, BARREL_H, BODY_CORNER_R
    ).translate((-HALF_BARREL, 0.0, 0.0))

    front_collar = _rounded_rect_prism(
        COLLAR_LEN, COLLAR_W, COLLAR_H, COLLAR_CORNER_R
    ).translate((HALF_BARREL, 0.0, 0.0))

    rear_collar = _rounded_rect_prism(
        COLLAR_LEN, COLLAR_W, COLLAR_H, COLLAR_CORNER_R
    ).translate((-HALF_BARREL - COLLAR_LEN, 0.0, 0.0))

    return body.union(front_collar).union(rear_collar)


def _build_nib():
    """Chisel nib pointing +X: straight holder then tapered wedge. Shoulder at X=0."""
    holder = _rounded_rect_prism(NIB_BASE_LEN, NIB_BASE_W, NIB_BASE_H, 0.0018)

    x0 = NIB_BASE_LEN
    wedge = (
        cq.Workplane("YZ")
        .workplane(offset=x0)
        .rect(NIB_BASE_W, NIB_BASE_H)
        .workplane(offset=NIB_WEDGE_LEN)
        .rect(NIB_TIP_W, NIB_TIP_H)
        .loft(combine=True)
    )
    return holder.union(wedge)


def _build_felt_tip():
    """Exposed ink-soaked felt sliver at the extreme chisel edge, pointing +X."""
    x0 = NIB_BASE_LEN + NIB_WEDGE_LEN
    return (
        cq.Workplane("YZ")
        .workplane(offset=x0 - 0.0015)
        .rect(NIB_TIP_W * 0.96, NIB_TIP_H * 1.05)
        .workplane(offset=0.0025)
        .rect(NIB_TIP_W * 0.9, NIB_TIP_H * 0.8)
        .loft(combine=True)
    )


def _build_cap():
    """Hollow black cap: mouth at X=0, closed end at +X. Includes flat pocket clip."""
    outer = _rounded_rect_prism(CAP_LEN, CAP_OUTER_W, CAP_OUTER_H, CAP_CORNER_R)

    bore_w = CAP_OUTER_W - 2 * CAP_WALL
    bore_h = CAP_OUTER_H - 2 * CAP_WALL
    bore_len = CAP_LEN - CAP_WALL
    bore = (
        cq.Workplane("YZ")
        .rect(bore_w, bore_h)
        .extrude(bore_len)
        .edges("|X")
        .fillet(CAP_CORNER_R - CAP_WALL)
    )
    cap = outer.cut(bore)

    # Pocket clip: thin flat rib along the top, anchored at the closed end.
    clip_thick = 0.0016
    clip_w = 0.0070
    clip_x0 = CAP_LEN - CAP_CLIP_LEN
    clip_z = CAP_OUTER_H / 2.0 + clip_thick / 2.0 - 0.0002
    clip = (
        cq.Workplane("YZ")
        .workplane(offset=clip_x0)
        .center(0.0, clip_z)
        .rect(clip_w, clip_thick)
        .extrude(CAP_CLIP_LEN)
        .edges("|X")
        .fillet(0.0006)
    )
    boss = (
        cq.Workplane("YZ")
        .workplane(offset=CAP_LEN - 0.0050)
        .center(0.0, CAP_OUTER_H / 2.0 - 0.0010)
        .rect(clip_w, 0.0040)
        .extrude(0.0045)
    )
    return cap.union(clip).union(boss)


def _build_end_nib_geometry(direction: int):
    """Shared helper: returns (nib_cq, felt_cq) for one end.
    direction=+1 → front (+X), direction=-1 → rear (-X, mirrored)."""
    nib = _build_nib()
    felt = _build_felt_tip()
    if direction < 0:
        nib = _flip_x(nib)
        felt = _flip_x(felt)
    return nib, felt


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_tip_marker")

    lime = model.material("lime_body", rgba=LIME)
    black = model.material("black_plastic", rgba=BLACK)
    felt_mat = model.material("felt_ink", rgba=FELT)

    # --- Barrel (root): lime body with collars at both ends, plus nibs ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel(), "barrel_body"),
        material=lime,
        name="barrel_body",
    )

    # Place a chisel nib + felt tip at each end via shared helper
    for i in range(2):
        direction = 1 if i == 0 else -1
        nib_cq, felt_cq = _build_end_nib_geometry(direction)
        nib_x = direction * (HALF_BARREL + COLLAR_LEN)

        barrel.visual(
            mesh_from_cadquery(nib_cq, f"nib_{i}"),
            origin=Origin(xyz=(nib_x, 0.0, 0.0)),
            material=black,
            name=f"nib_{i}",
        )
        barrel.visual(
            mesh_from_cadquery(felt_cq, f"felt_tip_{i}"),
            origin=Origin(xyz=(nib_x, 0.0, 0.0)),
            material=felt_mat,
            name=f"felt_tip_{i}",
        )

    # Travel needed to fully clear a nib: mouth past the chisel tip + margin
    nib_tip_dist = HALF_BARREL + COLLAR_LEN + NIB_BASE_LEN + NIB_WEDGE_LEN
    seat_dist = HALF_BARREL - CAP_SEAT_OVERLAP
    full_clear = (nib_tip_dist - seat_dist) + 0.006

    # --- Two caps via for loop, mirrored at front (+X) and rear (-X) ---
    for i in range(2):
        direction = 1 if i == 0 else -1

        cap_geom = _build_cap()
        if direction < 0:
            cap_geom = _flip_x(cap_geom)

        cap_part = model.part(f"cap_{i}")
        cap_part.visual(
            mesh_from_cadquery(cap_geom, f"cap_shell_{i}"),
            material=black,
            name=f"cap_shell_{i}",
        )

        # Joint origin at the cap mouth seating surface on the barrel
        seat_x = direction * seat_dist
        axis = (float(direction), 0.0, 0.0)

        model.articulation(
            f"barrel_to_cap_{i}",
            ArticulationType.PRISMATIC,
            parent=barrel,
            child=cap_part,
            origin=Origin(xyz=(seat_x, 0.0, 0.0)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=15.0, velocity=0.25, lower=0.0, upper=full_clear
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")

    nib_tip_dist = HALF_BARREL + COLLAR_LEN + NIB_BASE_LEN + NIB_WEDGE_LEN
    seat_dist = HALF_BARREL - CAP_SEAT_OVERLAP

    for i in range(2):
        direction = 1 if i == 0 else -1
        sign = "+" if direction > 0 else "-"

        cap = object_model.get_part(f"cap_{i}")
        joint = object_model.get_articulation(f"barrel_to_cap_{i}")

        # --- Joint contract: prismatic along X ---
        ctx.check(
            f"cap_{i} joint is prismatic",
            str(joint.joint_type).lower().endswith("prismatic"),
            details=f"joint_type={joint.joint_type}",
        )
        ax = tuple(joint.axis)
        ctx.check(
            f"cap_{i} slides along {sign}X",
            abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6
            and (ax[0] > 0) == (direction > 0),
            details=f"axis={ax}",
        )

        # --- Nib protrudes past its collar ---
        nib_aabb = ctx.part_element_world_aabb(barrel, elem=f"nib_{i}")
        ctx.check(
            f"nib_{i} protrudes past its collar",
            nib_aabb is not None and (
                (direction > 0 and nib_aabb[1][0] > nib_tip_dist - 1e-6) or
                (direction < 0 and nib_aabb[0][0] < -nib_tip_dist + 1e-6)
            ),
            details=f"nib_aabb={nib_aabb}",
        )

        # --- Felt tip at the chisel point ---
        felt_aabb = ctx.part_element_world_aabb(barrel, elem=f"felt_tip_{i}")
        ctx.check(
            f"felt_tip_{i} is at the chisel point",
            felt_aabb is not None and (
                (direction > 0 and felt_aabb[1][0] >= nib_tip_dist - 0.002) or
                (direction < 0 and felt_aabb[0][0] <= -nib_tip_dist + 0.002)
            ),
            details=f"felt_aabb={felt_aabb}",
        )

        # --- Cap is wider and taller than the barrel (wraps it) ---
        cap_aabb = ctx.part_world_aabb(cap)
        if cap_aabb is not None:
            cap_w = cap_aabb[1][1] - cap_aabb[0][1]
            cap_h = cap_aabb[1][2] - cap_aabb[0][2]
            ctx.check(
                f"cap_{i} is a chunky rectangular shell",
                cap_w >= BARREL_W and cap_h >= BARREL_H,
                details=f"cap_w={cap_w:.4f}, cap_h={cap_h:.4f}",
            )

        # --- Closed pose (q=0): cap fully covers the nib ---
        with ctx.pose({joint: 0.0}):
            ctx.expect_within(
                barrel, cap,
                axes="yz",
                inner_elem=f"nib_{i}",
                outer_elem=f"cap_shell_{i}",
                margin=0.001,
                name=f"seated cap_{i} encloses nib_{i} cross-section",
            )

            cap_closed = ctx.part_world_aabb(cap)
            ctx.check(
                f"seated cap_{i} front covers nib tip",
                cap_closed is not None and (
                    (direction > 0 and cap_closed[1][0] >= nib_tip_dist - 1e-4) or
                    (direction < 0 and cap_closed[0][0] <= -nib_tip_dist + 1e-4)
                ),
                details=f"cap_aabb={cap_closed}, nib_tip={nib_tip_dist:.4f}",
            )
            seated_outer = (
                None if cap_closed is None
                else (cap_closed[1][0] if direction > 0 else cap_closed[0][0])
            )

        # --- Open pose (upper limit): cap pulls outward and clears the nib ---
        upper = joint.motion_limits.upper
        with ctx.pose({joint: upper}):
            cap_open = ctx.part_world_aabb(cap)
            ctx.check(
                f"pulled cap_{i} clears its nib (mouth past tip)",
                cap_open is not None and (
                    (direction > 0 and cap_open[0][0] >= nib_tip_dist - 1e-3) or
                    (direction < 0 and cap_open[1][0] <= -nib_tip_dist + 1e-3)
                ),
                details=f"cap_aabb={cap_open}, nib_tip={nib_tip_dist:.4f}",
            )
            open_outer = (
                None if cap_open is None
                else (cap_open[1][0] if direction > 0 else cap_open[0][0])
            )
            ctx.check(
                f"cap_{i} moves outward when pulled off",
                seated_outer is not None and open_outer is not None and (
                    (direction > 0 and open_outer > seated_outer + 0.02) or
                    (direction < 0 and open_outer < seated_outer - 0.02)
                ),
                details=f"seated={seated_outer}, open={open_outer}",
            )

        # --- Allow seated-cap overlaps (friction-fit capture) ---
        ctx.allow_overlap(
            cap, barrel,
            elem_a=f"cap_shell_{i}", elem_b=f"nib_{i}",
            reason=f"Seated cap_{i} is a friction fit that intentionally encloses nib_{i}.",
        )
        ctx.allow_overlap(
            cap, barrel,
            elem_a=f"cap_shell_{i}", elem_b="barrel_body",
            reason=f"Cap_{i} mouth slides over the barrel collar to seat (push-on fit).",
        )
        ctx.allow_overlap(
            cap, barrel,
            elem_a=f"cap_shell_{i}", elem_b=f"felt_tip_{i}",
            reason=f"Seated cap_{i} encloses felt_tip_{i} to keep the marker from drying out.",
        )

    return ctx.report()


object_model = build_object_model()
