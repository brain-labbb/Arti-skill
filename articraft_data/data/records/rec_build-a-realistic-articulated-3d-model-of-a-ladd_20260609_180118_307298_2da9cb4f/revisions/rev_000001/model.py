from __future__ import annotations

# Realistic articulated step ladder (A-frame).
#
# The reference image (picture/Equipment/ladder/001.png) shows a ~5-step orange
# fiberglass/wood step ladder:
#   - a FRONT frame: two tapered orange side rails (stiles) joined by flat
#     aluminum steps (treads) and capped by a dark molded top cap / tool tray.
#   - a REAR support frame: two thinner orange rails joined by metal spreader
#     cross-braces and diagonal X-braces; it carries no steps.
#   - both frames meet at a hinge under the top cap and spread into an "A".
#
# The PRIMARY user-facing mechanism is the top hinge where the rear frame swings
# open relative to the front frame -> REVOLUTE about the left-right (X) axis.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
LADDER_HEIGHT = 1.46  # top of the rails at the hinge line
HALF_WIDTH = 0.215  # half the outside spacing between the two front rails
RAIL_DEPTH_TOP = 0.034  # front rail front-to-back thickness near the top
RAIL_DEPTH_BOT = 0.052  # front rail front-to-back thickness near the bottom
RAIL_WIDTH = 0.040  # rail side-to-side thickness

# The closed (folded) frames are nearly parallel; the open A-frame splays the
# rear feet rearward. We author the geometry in the OPEN pose so the silhouette
# reads as the deployed ladder.
FRONT_LEAN = 0.090  # rear/back offset of the front rail base vs its top (slight rake)
REAR_SPREAD = 0.300  # rear foot offset behind the hinge along +Y when open

N_STEPS = 5  # number of aluminum treads on the front frame
STEP_BOTTOM_Z = 0.110  # height of the lowest tread
STEP_TOP_Z = 1.250  # height of the top tread (just below the cap)


def _rail_solid(
    *,
    top_xy: tuple[float, float],
    bot_xy: tuple[float, float],
    z_top: float,
    z_bot: float,
    depth_top: float,
    depth_bot: float,
    width: float,
) -> cq.Workplane:
    """Build one tapered ladder rail as a lofted bar from a bottom rectangle
    (at z_bot, centered on bot_xy) up to a top rectangle (at z_top, top_xy).

    The rail cross-section is a rectangle (width in X, depth in Y); it tapers
    in depth from bottom to top, matching the wider-at-the-floor stile look.
    """
    bx, by = bot_xy
    tx, ty = top_xy

    return (
        cq.Workplane("XY")
        .workplane(offset=z_bot)
        .center(bx, by)
        .rect(width, depth_bot)
        .workplane(offset=(z_top - z_bot))
        .center(tx - bx, ty - by)
        .rect(width, depth_top)
        .loft(combine=True)
    )


def _front_frame_shape() -> cq.Workplane:
    """Front frame = two tapered rails + the flat aluminum steps between them,
    authored as one welded mesh in the front-frame local frame.

    Local frame: origin at floor center, +Z up. The hinge line sits at
    (y=0, z=LADDER_HEIGHT). The front rails rake slightly forward so their feet
    are at y=-FRONT_LEAN.
    """
    # Two side rails (left at -X, right at +X).
    left = _rail_solid(
        top_xy=(-HALF_WIDTH, 0.0),
        bot_xy=(-HALF_WIDTH, -FRONT_LEAN),
        z_top=LADDER_HEIGHT,
        z_bot=0.0,
        depth_top=RAIL_DEPTH_TOP,
        depth_bot=RAIL_DEPTH_BOT,
        width=RAIL_WIDTH,
    )
    right = _rail_solid(
        top_xy=(HALF_WIDTH, 0.0),
        bot_xy=(HALF_WIDTH, -FRONT_LEAN),
        z_top=LADDER_HEIGHT,
        z_bot=0.0,
        depth_top=RAIL_DEPTH_TOP,
        depth_bot=RAIL_DEPTH_BOT,
        width=RAIL_WIDTH,
    )
    frame = left.add(right)
    return frame


def _steps_shape() -> cq.Workplane:
    """The flat aluminum treads. Each step spans between the two front rails and
    has a small downturned front lip (read as a tread)."""
    step_span = 2.0 * HALF_WIDTH + RAIL_WIDTH * 0.6  # reach into the rails
    step_w = 0.090  # tread depth (front to back)
    step_t = 0.014  # tread thickness
    steps = None
    for i in range(N_STEPS):
        f = i / (N_STEPS - 1)
        z = STEP_BOTTOM_Z + f * (STEP_TOP_Z - STEP_BOTTOM_Z)
        # The front rails rake forward toward the floor, so the rail centerline
        # at this height sits at rail_y. The tread mounts on the climbing (front,
        # -Y) side of the rails: its back edge stays at the rail centerline so it
        # never reaches back into the rear frame.
        rail_y = -FRONT_LEAN * (1.0 - z / LADDER_HEIGHT)
        y = rail_y - step_w / 2.0 - 0.004
        tread = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .center(0.0, y)
            .box(step_span, step_w, step_t, centered=(True, True, True))
            .edges("|X").fillet(0.0035)
        )
        steps = tread if steps is None else steps.add(tread)
    return steps


def _top_cap_shape() -> cq.Workplane:
    """Dark molded top cap / tool tray sitting on the front rails at the top."""
    cap_w = 2.0 * HALF_WIDTH + RAIL_WIDTH + 0.020
    cap_d = 0.150
    cap_t = 0.040
    z = LADDER_HEIGHT + cap_t / 2.0 - 0.006  # seat slightly onto the rail tops
    body = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .box(cap_w, cap_d, cap_t, centered=(True, True, True))
        .edges("|Z").fillet(0.010)
    )
    # Shallow recessed tool tray on top.
    tray = (
        cq.Workplane("XY")
        .workplane(offset=z + cap_t / 2.0)
        .box(cap_w - 0.045, cap_d - 0.045, 0.018, centered=(True, True, True))
    )
    return body.cut(tray)


def _rear_frame_shape() -> cq.Workplane:
    """Rear support frame, authored in the REAR-frame local frame whose origin is
    the hinge line: origin at (0,0,0) = hinge, rails run downward (-Z) and, at
    q=0 (open), splay rearward (+Y) toward the feet.

    The frame = two thinner rails + horizontal spreader bars + a lower diagonal
    X-brace, all welded into one mesh.
    """
    rail_w = 0.034
    rail_d = 0.026
    length = LADDER_HEIGHT  # rail length from hinge to foot

    # Rear rails sit well inboard of the front rails so the two frames nest
    # without colliding away from the hinge.
    rx = 0.165

    def _rail_y(z: float) -> float:
        """The rail centerline Y at a given (negative) hinge-relative height."""
        return REAR_SPREAD * (-z / length)

    # Rail tops stop just below the top cap (rel z = -RAIL_TOP_INSET) so the cap
    # rests on top of the frames; they still reach into the hinge knuckles.
    rail_top_z = -0.016

    # Rail feet land at y=+REAR_SPREAD, z=-length (relative to hinge).
    def _rear_rail(x: float) -> cq.Workplane:
        return _rail_solid(
            top_xy=(x, _rail_y(rail_top_z)),
            bot_xy=(x, REAR_SPREAD),
            z_top=rail_top_z,
            z_bot=-length,
            depth_top=rail_d,
            depth_bot=rail_d,
            width=rail_w,
        )

    frame = _rear_rail(-rx).add(_rear_rail(rx))

    def _strut(p0: tuple[float, float, float], p1: tuple[float, float, float],
               half: float) -> cq.Workplane:
        """Square-section strut from p0 to p1, lofted between two small squares so
        its ends weld into the rails it connects."""
        ax, ay, az = p0
        bx, by, bz = p1
        d = (bx - ax, by - ay, bz - az)
        dlen = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
        ux = (d[0] / dlen, d[1] / dlen, d[2] / dlen)
        # Pick a stable perpendicular reference.
        ref = (0.0, 0.0, 1.0) if abs(ux[2]) < 0.9 else (1.0, 0.0, 0.0)
        v = (
            ux[1] * ref[2] - ux[2] * ref[1],
            ux[2] * ref[0] - ux[0] * ref[2],
            ux[0] * ref[1] - ux[1] * ref[0],
        )
        vlen = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
        v = (v[0] / vlen, v[1] / vlen, v[2] / vlen)
        w = (
            ux[1] * v[2] - ux[2] * v[1],
            ux[2] * v[0] - ux[0] * v[2],
            ux[0] * v[1] - ux[1] * v[0],
        )

        def _sq(center: tuple[float, float, float]) -> list[tuple[float, float, float]]:
            cx, cy, cz = center
            pts = []
            for sv, sw in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
                pts.append(
                    (
                        cx + half * (sv * v[0] + sw * w[0]),
                        cy + half * (sv * v[1] + sw * w[1]),
                        cz + half * (sv * v[2] + sw * w[2]),
                    )
                )
            return pts

        # Extend ends slightly into the rails for a clean weld.
        a_in = (ax - ux[0] * 0.012, ay - ux[1] * 0.012, az - ux[2] * 0.012)
        b_in = (bx + ux[0] * 0.012, by + ux[1] * 0.012, bz + ux[2] * 0.012)
        wp = cq.Workplane("XY")
        wp = wp.polyline(_sq(a_in)).close()
        wp = wp.polyline(_sq(b_in)).close()
        return wp.loft(combine=True)

    # Horizontal spreader cross-braces (flat metal bars) between the rails.
    for zc in (-0.30, -0.70, -1.10):
        yc = _rail_y(zc)
        frame = frame.add(_strut((-rx, yc, zc), (rx, yc, zc), 0.008))

    # Lower diagonal X-brace (two crossing struts) welded rail-to-rail.
    z_hi, z_lo = -0.78, -1.18
    for sign in (1.0, -1.0):
        p0 = (sign * rx, _rail_y(z_hi), z_hi)
        p1 = (-sign * rx, _rail_y(z_lo), z_lo)
        frame = frame.add(_strut(p0, p1, 0.0045))

    return frame


def _hinge_bracket_shape() -> cq.Workplane:
    """A pair of metal hinge knuckles under the top cap that carry the pin the
    rear frame rotates about. Authored in the front-frame local frame at the
    hinge line."""
    knuckle = None
    z = LADDER_HEIGHT - 0.012
    for x in (-HALF_WIDTH, HALF_WIDTH):
        k = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .center(x, 0.012)
            .cylinder(0.060, 0.014, direct=(1, 0, 0))
        )
        knuckle = k if knuckle is None else knuckle.add(k)
    # Cross pin spanning the knuckles.
    pin = (
        cq.Workplane("YZ")
        .center(0.012, z)
        .cylinder(2.0 * (HALF_WIDTH + 0.030), 0.006, direct=(0, 0, 1))
    )
    return knuckle.add(pin)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="step_ladder")

    wood_orange = model.material("wood_orange", rgba=(0.86, 0.45, 0.10, 1.0))
    rear_orange = model.material("rear_orange", rgba=(0.80, 0.40, 0.09, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.75, 0.77, 1.0))
    dark_cap = model.material("dark_cap", rgba=(0.16, 0.16, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    # -------------------------------------------------------------------
    # FRONT FRAME (root) -- rails + steps + top cap + hinge bracket + feet.
    # -------------------------------------------------------------------
    front = model.part("front_frame")
    front.visual(
        mesh_from_cadquery(_front_frame_shape(), "front_rails"),
        material=wood_orange,
        name="front_rails",
    )
    front.visual(
        mesh_from_cadquery(_steps_shape(), "steps"),
        material=aluminum,
        name="steps",
    )
    front.visual(
        mesh_from_cadquery(_top_cap_shape(), "top_cap"),
        material=dark_cap,
        name="top_cap",
    )
    front.visual(
        mesh_from_cadquery(_hinge_bracket_shape(), "hinge_bracket"),
        material=steel,
        name="hinge_bracket",
    )
    # Front rubber feet.
    for sign, nm in ((-1.0, "front_foot_left"), (1.0, "front_foot_right")):
        front.visual(
            Box((RAIL_WIDTH + 0.006, RAIL_DEPTH_BOT + 0.006, 0.018)),
            origin=Origin(xyz=(sign * HALF_WIDTH, -FRONT_LEAN, 0.009)),
            material=rubber,
            name=nm,
        )

    # -------------------------------------------------------------------
    # REAR SUPPORT FRAME (child of the top hinge).
    # Authored in a local frame whose origin is the hinge line; rails go down
    # (-Z) and splay rearward (+Y) at the open pose.
    # -------------------------------------------------------------------
    rear = model.part("rear_frame")
    rear.visual(
        mesh_from_cadquery(_rear_frame_shape(), "rear_frame"),
        material=rear_orange,
        name="rear_frame",
    )
    # The spreader/X-brace metalwork is welded into the single rear mesh; the
    # orange rails dominate the part read. The feet are rubber boots.
    rx = 0.165
    for sign, nm in ((-1.0, "rear_foot_left"), (1.0, "rear_foot_right")):
        rear.visual(
            Box((0.034 + 0.006, 0.026 + 0.006, 0.018)),
            origin=Origin(xyz=(sign * rx, REAR_SPREAD, -LADDER_HEIGHT + 0.009)),
            material=rubber,
            name=nm,
        )

    # -------------------------------------------------------------------
    # HINGE: rear frame swings about the X axis at the top hinge line.
    # The front-frame hinge line is at (0, 0, LADDER_HEIGHT). The rear-frame
    # local origin (its hinge) is coincident with the articulation frame at q=0.
    # At q=0 the rear feet are already splayed to +Y (open A-frame). Positive q
    # rotates the rear frame further open (feet swing rearward / down toward +Y),
    # negative q folds it toward the front frame.
    # -------------------------------------------------------------------
    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, LADDER_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        # q<0 folds the rear frame closed against the front frame;
        # q>0 opens it wider. The deployed pose is q=0.
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-0.34, upper=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("top_hinge")

    # --- Intentional hinge capture ------------------------------------------
    # The steel hinge knuckles/pin under the top cap straddle the top of the
    # rear frame, which is exactly how the pivot is carried. Allow that small,
    # local, mechanically-required nesting and prove the bracket really touches
    # the rear frame top (so the rear frame is carried by the pivot, not float).
    ctx.allow_overlap(
        front,
        rear,
        elem_a="hinge_bracket",
        elem_b="rear_frame",
        reason="Hinge knuckles and pin intentionally capture the rear frame top "
        "to form the pivot; this is the real hinge engagement.",
    )
    ctx.expect_contact(
        front,
        rear,
        elem_a="hinge_bracket",
        elem_b="rear_frame",
        contact_tol=0.002,
        name="hinge bracket carries the rear frame top",
    )

    # --- Mechanism type / axis claims ---------------------------------------
    ctx.check(
        "top hinge is revolute",
        hinge.joint_type == "revolute" or str(hinge.articulation_type).lower().endswith("revolute"),
        details=f"joint_type={hinge.joint_type}",
    )
    ax = tuple(hinge.axis)
    ctx.check(
        "hinge axis is left-right (X)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )
    ctx.check(
        "hinge sits at the top of the front frame",
        hinge.origin.xyz[2] > 1.30,
        details=f"hinge z={hinge.origin.xyz[2]}",
    )

    # --- Hero geometry present & placed -------------------------------------
    # Top cap sits at the very top; steps span the front frame; rear frame
    # reaches the floor behind the front frame.
    front_aabb = ctx.part_world_aabb(front)
    rear_aabb = ctx.part_world_aabb(rear)
    ctx.check(
        "front frame reaches floor",
        front_aabb is not None and front_aabb[0][2] < 0.03,
        details=f"front min z={None if front_aabb is None else front_aabb[0][2]}",
    )
    ctx.check(
        "front frame reaches the top cap height",
        front_aabb is not None and front_aabb[1][2] > LADDER_HEIGHT,
        details=f"front max z={None if front_aabb is None else front_aabb[1][2]}",
    )

    cap_aabb = ctx.part_element_world_aabb(front, elem="top_cap")
    ctx.check(
        "top cap is the highest element",
        cap_aabb is not None
        and front_aabb is not None
        and cap_aabb[1][2] >= front_aabb[1][2] - 1e-6,
        details=f"cap max z={None if cap_aabb is None else cap_aabb[1][2]}",
    )

    steps_aabb = ctx.part_element_world_aabb(front, elem="steps")
    ctx.check(
        "steps span the full ladder height range",
        steps_aabb is not None
        and steps_aabb[0][2] < 0.20
        and steps_aabb[1][2] > 1.20,
        details=f"steps z=[{None if steps_aabb is None else (steps_aabb[0][2], steps_aabb[1][2])}]",
    )
    # Steps should span between the rails in X.
    ctx.check(
        "steps span between the rails",
        steps_aabb is not None and (steps_aabb[1][0] - steps_aabb[0][0]) > 0.30,
        details=f"steps x-span={None if steps_aabb is None else steps_aabb[1][0]-steps_aabb[0][0]}",
    )

    # Rear frame: at rest (open) the feet are behind the front frame (+Y) and on
    # the floor.
    ctx.check(
        "rear frame reaches the floor",
        rear_aabb is not None and rear_aabb[0][2] < 0.05,
        details=f"rear min z={None if rear_aabb is None else rear_aabb[0][2]}",
    )
    ctx.check(
        "rear feet are splayed behind the front frame",
        rear_aabb is not None and rear_aabb[1][1] > 0.20,
        details=f"rear max y={None if rear_aabb is None else rear_aabb[1][1]}",
    )

    # --- Hinge connectivity: rear frame top is near the hinge line -----------
    # The rear frame must remain mounted at the top (no floating part).
    rear_top = rear_aabb[1][2] if rear_aabb is not None else 0.0
    ctx.check(
        "rear frame top reaches the hinge line",
        rear_top > LADDER_HEIGHT - 0.08,
        details=f"rear top z={rear_top}",
    )

    # --- Decisive pose check: folding the ladder draws the rear feet forward --
    # Track an actual rear foot element. At rest it sits behind the front frame
    # (large +Y); folding (q=lower) swings it forward toward the front frame.
    rest_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_left")
    with ctx.pose({hinge: hinge.motion_limits.lower}):
        folded_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_left")
    rest_y = None if rest_foot is None else 0.5 * (rest_foot[0][1] + rest_foot[1][1])
    folded_y = None if folded_foot is None else 0.5 * (folded_foot[0][1] + folded_foot[1][1])
    ctx.check(
        "folding the hinge swings the rear foot toward the front frame",
        rest_y is not None and folded_y is not None and folded_y < rest_y - 0.20,
        details=f"open rear foot Y={rest_y}, folded rear foot Y={folded_y}",
    )

    # Opening wider (q=upper) pushes the rear foot further back (+Y).
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_left")
    open_y = None if open_foot is None else 0.5 * (open_foot[0][1] + open_foot[1][1])
    ctx.check(
        "opening the hinge wider pushes the rear foot rearward",
        open_y is not None and rest_y is not None and open_y > rest_y + 0.05,
        details=f"rest rear foot Y={rest_y}, opened rear foot Y={open_y}",
    )

    return ctx.report()


object_model = build_object_model()
