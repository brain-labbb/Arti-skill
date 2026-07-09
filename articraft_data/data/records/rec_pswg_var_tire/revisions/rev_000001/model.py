from __future__ import annotations

# Rustic playground double swing built from round timber logs — TIRE SEAT variant.
#
# Frame convention:
#   X = fore/aft (the swings travel in X)
#   Y = left/right (the crossbeam runs along Y)
#   Z = up
#
# Root part = log frame: two pairs of thick cylindrical log posts (each pair
# slightly splayed in X and lashed together near the top with rope bands),
# carrying one horizontal log crossbeam with four small brass finial caps.
# Child parts = two identical rigid pendulum swings. Each swing hangs a
# horizontal tire seat (torus lying flat) from the crossbeam via four short
# ropes that converge to the pivot point. Each swing pivots about the crossbeam
# axis through its own revolute joint, fully independent of the other.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# --- Overall dimensions (meters) ---------------------------------------------
BEAM_Z = 1.95          # crossbeam centerline height
BEAM_R = 0.10          # crossbeam log radius
BEAM_LEN = 2.60        # frame width along Y
POST_R = 0.075         # post log radius
POST_Y = 1.16          # y position of each post pair
POST_TOP_Z = 1.92      # post axis top height
POST_BASE_X = 0.26     # base splay of each post in X
POST_TOP_X = 0.055     # post axis x at the top (pair nearly touching)

SWING_Y = 0.55         # y center of each swing

# Tire seat dimensions
TIRE_R = 0.22          # torus major radius (center of tube to torus center)
TIRE_TUBE = 0.065      # torus tube radius
TIRE_Z_REL = -1.35     # tire center z relative to pivot

# Rope convergence geometry
ROPE_TOP_Z = -(BEAM_R + 0.015)   # just below the beam surface
N_ROPES = 4                       # ropes per swing
ROPE_R_TOP = 0.014                # rope radius at the top (thicker)
ROPE_R_BOT = 0.010                # rope radius at the tire (thinner)

RING_RO = 0.125        # rope wrap ring outer radius
RING_RI = 0.098        # rope wrap ring inner radius (2 mm snug embed on beam)
RING_W = 0.045         # rope wrap ring width along Y

SWING_LO = -1.0        # fore-aft swing limits (rad)
SWING_HI = 1.0


def _lashing_band() -> cq.Workplane:
    """Elliptical rope band lashed around one splayed post pair."""
    outer = cq.Workplane("XY").ellipse(0.175, 0.105).extrude(0.0275, both=True)
    inner = cq.Workplane("XY").ellipse(0.125, 0.058).extrude(0.03, both=True)
    return outer.cut(inner)


def _rope_ring() -> cq.Workplane:
    """Washer-shaped rope wrap that encircles the crossbeam (axis local Z)."""
    outer = cq.Workplane("XY").circle(RING_RO).extrude(RING_W / 2.0, both=True)
    inner = cq.Workplane("XY").circle(RING_RI).extrude(RING_W / 2.0 + 0.002, both=True)
    return outer.cut(inner)


def _tire_attachment_points() -> list[tuple[float, float, float]]:
    """Return 4 equally-spaced points on the top of the tire ring."""
    pts = []
    for i in range(N_ROPES):
        angle = (2.0 * math.pi * i) / N_ROPES + math.pi / 4.0  # offset 45°
        x = TIRE_R * math.cos(angle)
        y = TIRE_R * math.sin(angle)
        z = TIRE_Z_REL + TIRE_TUBE * 0.7  # attach slightly above tube center
        pts.append((x, y, z))
    return pts


def _make_rope_cone(top_pt: tuple[float, float, float],
                    bot_pt: tuple[float, float, float]) -> tuple[cq.Solid, Origin]:
    """Build a tapered rope cone from top_pt to bot_pt, return solid + origin."""
    dx = bot_pt[0] - top_pt[0]
    dy = bot_pt[1] - top_pt[1]
    dz = bot_pt[2] - top_pt[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)

    # Cone along +Z: r_top at z=0, r_bot at z=length
    cone = cq.Solid.makeCone(ROPE_R_TOP, ROPE_R_BOT, length)

    # Orientation: align local +Z with direction (dx, dy, dz)
    pitch = math.acos(max(-1.0, min(1.0, dz / length)))
    yaw = math.atan2(dy, dx)

    return cone, Origin(xyz=top_pt, rpy=(0.0, pitch, yaw))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rustic_log_double_swing")
    model.material("log_wood", rgba=(0.58, 0.42, 0.26, 1.0))
    model.material("log_wood_dark", rgba=(0.50, 0.35, 0.21, 1.0))
    model.material("rope_tan", rgba=(0.78, 0.65, 0.45, 1.0))
    model.material("brass", rgba=(0.80, 0.64, 0.28, 1.0))
    model.material("tire_rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    # ------------------------------------------------------------- log frame
    frame = model.part("log_frame")

    # Two pairs of splayed log posts, one pair per end of the beam. Each post
    # leans inward so the pair converges (and slightly crosses) just under the
    # crossbeam, where the pair is lashed together with rope bands.
    dx = POST_TOP_X - POST_BASE_X
    post_len = math.hypot(dx, POST_TOP_Z)
    for pair_i, py in enumerate((-POST_Y, POST_Y)):
        for post_i, sgn in enumerate((1.0, -1.0)):
            pitch = math.atan2(sgn * dx, POST_TOP_Z)
            # Lift so the lowest rim of the tilted log rests on the ground.
            cz = (post_len / 2.0) * math.cos(pitch) + POST_R * math.sin(abs(pitch))
            cx = sgn * (POST_BASE_X + POST_TOP_X) / 2.0
            frame.visual(
                Cylinder(radius=POST_R, length=post_len),
                origin=Origin(xyz=(cx, py, cz), rpy=(0.0, pitch, 0.0)),
                material="log_wood",
                name=f"post_{pair_i}_{post_i}",
            )
        # Rope lashing bands binding the post pair near the top.
        for band_i, bz in enumerate((1.70, 1.77)):
            frame.visual(
                mesh_from_cadquery(_lashing_band(), f"lashing_band_{pair_i}_{band_i}"),
                origin=Origin(xyz=(0.0, py, bz)),
                material="rope_tan",
                name=f"lashing_{pair_i}_{band_i}",
            )

    # Horizontal log crossbeam along Y, nesting into the post-pair crotches.
    frame.visual(
        Cylinder(radius=BEAM_R, length=BEAM_LEN),
        origin=Origin(xyz=(0.0, 0.0, BEAM_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="log_wood_dark",
        name="crossbeam",
    )

    # Four small brass finial caps on top of the beam, one just beside each
    # rope hanger point (shifted 0.08 m so the rotating rope rings clear them).
    finial_ys: list[float] = []
    for sy in (-SWING_Y, SWING_Y):
        for sr in (-0.12, 0.12):
            shift = 0.08 if sr > 0 else -0.08
            finial_ys.append(sy + sr + shift)
    for fi, fy in enumerate(sorted(finial_ys)):
        beam_top = BEAM_Z + BEAM_R
        frame.visual(
            Cylinder(radius=0.028, length=0.05),
            origin=Origin(xyz=(0.0, fy, beam_top - 0.015 + 0.025)),
            material="brass",
            name=f"finial_stem_{fi}",
        )
        frame.visual(
            Sphere(radius=0.034),
            origin=Origin(xyz=(0.0, fy, beam_top + 0.05)),
            material="brass",
            name=f"finial_ball_{fi}",
        )
        frame.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=(0.0, fy, beam_top + 0.088)),
            material="brass",
            name=f"finial_tip_{fi}",
        )

    # ------------------------------------------------------------- two swings
    # Each swing part frame sits at its pivot: on the crossbeam axis at the
    # swing's y center. The whole swing (ring + ropes + tire) is one rigid
    # pendulum link.
    swings = []
    for swing_i, sy in enumerate((-SWING_Y, SWING_Y)):
        swing = model.part(f"swing_{swing_i}")

        # Rope wrap ring around the crossbeam (washer, axis along Y).
        swing.visual(
            mesh_from_cadquery(_rope_ring(), f"rope_ring_{swing_i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="rope_tan",
            name="rope_ring",
        )

        # Four converging ropes from the beam hanger to the tire.
        top_pt = (0.0, 0.0, ROPE_TOP_Z)
        attach_pts = _tire_attachment_points()
        for rope_i, bot_pt in enumerate(attach_pts):
            cone, cone_origin = _make_rope_cone(top_pt, bot_pt)
            swing.visual(
                mesh_from_cadquery(cone, f"rope_{swing_i}_{rope_i}"),
                origin=cone_origin,
                material="rope_tan",
                name=f"rope_{rope_i}",
            )
            # Small stopper knot where the rope meets the tire.
            swing.visual(
                Sphere(radius=0.022),
                origin=Origin(xyz=bot_pt),
                material="rope_tan",
                name=f"rope_knot_{rope_i}",
            )

        # Decorative knot at the top convergence point.
        swing.visual(
            Sphere(radius=0.028),
            origin=Origin(xyz=(0.0, 0.0, ROPE_TOP_Z - 0.02)),
            material="rope_tan",
            name="hanger_knot",
        )

        # Tire seat: torus lying flat (axis along Z).
        tire_geom = TorusGeometry(TIRE_R, TIRE_TUBE, radial_segments=20, tubular_segments=36)
        swing.visual(
            mesh_from_geometry(tire_geom, f"tire_seat_{swing_i}"),
            origin=Origin(xyz=(0.0, 0.0, TIRE_Z_REL)),
            material="tire_rubber",
            name="tire_seat",
        )

        # Independent fore-aft pendulum pivot on the crossbeam axis.
        model.articulation(
            f"beam_to_swing_{swing_i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=swing,
            origin=Origin(xyz=(0.0, sy, BEAM_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0, velocity=3.0, lower=SWING_LO, upper=SWING_HI
            ),
        )
        swings.append(swing)

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("log_frame")
    swing_0 = object_model.get_part("swing_0")
    swing_1 = object_model.get_part("swing_1")
    joint_0 = object_model.get_articulation("beam_to_swing_0")
    joint_1 = object_model.get_articulation("beam_to_swing_1")

    # Rope wrap ring is snugly wrapped around the crossbeam (2 mm embed).
    for swing in (swing_0, swing_1):
        ctx.allow_overlap(
            frame,
            swing,
            elem_a="crossbeam",
            elem_b="rope_ring",
            reason="Rope wrap ring grips the log crossbeam snugly to hang the swing.",
        )
        ctx.expect_contact(
            frame,
            swing,
            elem_a="crossbeam",
            elem_b="rope_ring",
            name=f"{swing.name} rope ring wraps the crossbeam",
        )

    # Hero frame geometry: two pairs of splayed log posts, lashed near the top.
    n_posts = sum(1 for v in frame.visuals if v.name and v.name.startswith("post_"))
    ctx.check("two pairs of log posts (four posts)", n_posts == 4, details=f"posts={n_posts}")
    n_lash = sum(1 for v in frame.visuals if v.name and v.name.startswith("lashing_"))
    ctx.check("post pairs are rope-lashed near the top", n_lash == 4, details=f"bands={n_lash}")
    ctx.check("log crossbeam present", frame.get_visual("crossbeam") is not None)

    # Four brass finial caps sit on top of the beam.
    n_finial = sum(1 for v in frame.visuals if v.name and v.name.startswith("finial_ball_"))
    ctx.check("four brass finial caps on the beam", n_finial == 4, details=f"finials={n_finial}")
    beam_top = BEAM_Z + BEAM_R
    finial_above = all(
        v.origin.xyz[2] > beam_top
        for v in frame.visuals
        if v.name and v.name.startswith("finial_ball_")
    )
    ctx.check("finial balls sit above the beam top", finial_above)

    # Frame scale: ~2.6 m wide, ~2.1 m tall, resting on the ground.
    fb = ctx.part_world_aabb(frame)
    if fb is not None:
        (fxmn, fymn, fzmn), (fxmx, fymx, fzmx) = fb
        ctx.check(
            "frame is ~2.6 m wide and ~2.1 m tall",
            2.45 < (fymx - fymn) < 2.75 and 2.0 < fzmx < 2.25,
            details=f"width={fymx - fymn:.2f} top={fzmx:.2f}",
        )
        ctx.check("frame rests on the ground (zmin ~ 0)", abs(fzmn) < 0.02, details=f"zmin={fzmn:.3f}")

    # Each swing: tire seat, four converging ropes, hanger knot, rope ring.
    for swing in (swing_0, swing_1):
        # Tire seat present (not a plank).
        ctx.check(
            f"{swing.name} has a tire seat",
            swing.get_visual("tire_seat") is not None,
        )
        # No plank seat, back rail, or corner fittings (those belong to the parent).
        visual_names = {v.name for v in swing.visuals if v.name}
        ctx.check(
            f"{swing.name} has no plank seat",
            "seat_plank" not in visual_names,
        )
        ctx.check(
            f"{swing.name} has no back rail",
            "back_rail" not in visual_names,
        )
        n_fit = sum(1 for v in swing.visuals if v.name and v.name.startswith("corner_fitting_"))
        ctx.check(f"{swing.name} has no corner fittings", n_fit == 0, details=f"fittings={n_fit}")

        # Four converging ropes.
        n_ropes = sum(1 for v in swing.visuals if v.name and v.name.startswith("rope_") and "knot" not in v.name and "ring" not in v.name)
        ctx.check(f"{swing.name} has four converging ropes", n_ropes == 4, details=f"ropes={n_ropes}")

        # Rope ring at the beam.
        ctx.check(f"{swing.name} has a beam wrap ring", swing.get_visual("rope_ring") is not None)

        # Knots at tire attachments plus hanger knot.
        n_knots = sum(1 for v in swing.visuals if v.name and "knot" in v.name)
        ctx.check(f"{swing.name} has rope knots", n_knots >= 5, details=f"knots={n_knots}")

    # Tire geometry: the tire should be wider in XY than thick in Z.
    for swing in (swing_0, swing_1):
        tire_aabb = ctx.part_element_world_aabb(swing, elem="tire_seat")
        if tire_aabb is not None:
            (txmn, tymn, tzmn), (txmx, tymx, tzmx) = tire_aabb
            tire_dx = txmx - txmn
            tire_dy = tymx - tymn
            tire_dz = tzmx - tzmn
            ctx.check(
                f"{swing.name} tire is a flat ring (wider than tall)",
                tire_dx > 0.30 and tire_dy > 0.30 and tire_dz < 0.20,
                details=f"dx={tire_dx:.3f} dy={tire_dy:.3f} dz={tire_dz:.3f}",
            )

    # At rest each tire hangs below the beam, well above the ground.
    for swing in (swing_0, swing_1):
        with ctx.pose({joint_0: 0.0, joint_1: 0.0}):
            sb = ctx.part_world_aabb(swing)
        if sb is not None:
            ctx.check(
                f"{swing.name} tire hangs above the ground",
                sb[0][2] > 0.25,
                details=f"zmin={sb[0][2]:.2f}",
            )
            ctx.check(
                f"{swing.name} tops out at its beam wrap ring",
                sb[1][2] < BEAM_Z + RING_RO + 0.01,
                details=f"zmax={sb[1][2]:.2f}",
            )

    # Both swings clear the floor through the full -1.0..+1.0 rad travel.
    floor_clear = True
    for q in (SWING_LO, 0.0, SWING_HI):
        with ctx.pose({joint_0: q, joint_1: q}):
            for swing in (swing_0, swing_1):
                sb = ctx.part_world_aabb(swing)
                if sb is not None and sb[0][2] <= 0.0:
                    floor_clear = False
    ctx.check("swings clear the floor through their full travel", floor_clear)

    def _cx(aabb):
        return None if aabb is None else (aabb[0][0] + aabb[1][0]) / 2.0

    # Pendulum motion: each swing displaces fore/aft about the beam axis.
    with ctx.pose({joint_0: 0.8, joint_1: 0.8}):
        f0 = _cx(ctx.part_world_aabb(swing_0))
        f1 = _cx(ctx.part_world_aabb(swing_1))
    with ctx.pose({joint_0: -0.8, joint_1: -0.8}):
        b0 = _cx(ctx.part_world_aabb(swing_0))
        b1 = _cx(ctx.part_world_aabb(swing_1))
    if None not in (f0, b0, f1, b1):
        ctx.check(
            "both swings travel fore/aft as pendulums",
            abs(f0 - b0) > 0.5 and abs(f1 - b1) > 0.5,
            details=f"travel0={abs(f0 - b0):.2f} travel1={abs(f1 - b1):.2f}",
        )

    # Independence: driving one swing leaves the other at rest.
    with ctx.pose({joint_0: 0.0, joint_1: 0.0}):
        rest1 = _cx(ctx.part_world_aabb(swing_1))
    with ctx.pose({joint_0: 0.9, joint_1: 0.0}):
        moved0 = _cx(ctx.part_world_aabb(swing_0))
        still1 = _cx(ctx.part_world_aabb(swing_1))
    if None not in (rest1, moved0, still1):
        ctx.check(
            "swings are fully independent",
            abs(moved0) > 0.5 and abs(still1 - rest1) < 0.01,
            details=f"moved0_cx={moved0:.2f} other_drift={abs(still1 - rest1):.4f}",
        )

    return ctx.report()
