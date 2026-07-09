from __future__ import annotations

# Adjustable chrome dumbbell with stacked round steel weight plates,
# knurled handle, threaded bar ends, and paired hexagonal jam-nut collars.
# Frame: handle axis along +X, centered at origin (x=0 is bar midpoint).
# Static body  : knurled handle + shoulder sleeves + plate stack per side +
#                threaded bar ends + inner hex jam nut per side (fixed).
# Articulations:
#   - outer_hex_nut_pos : CONTINUOUS spin about +X (bar axis)
#   - outer_hex_nut_neg : CONTINUOUS spin about +X (bar axis)
# Each outer hex nut carries an off-axis marker for spin detection.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
HANDLE_LEN = 0.150        # central grip section length
HANDLE_R = 0.0150         # grip core radius
GRIP_KNURL_R = 0.0162     # knurled grip band radius

SLEEVE_R = 0.022          # shoulder collar between grip and plate stack
SLEEVE_LEN = 0.010

PLATE_R = 0.080           # plate radius (dia ~0.16 m)
PLATE_THICK = 0.012       # single plate thickness
PLATE_GAP = 0.0008        # tiny seam between plates
N_PLATES = 5
PLATE_HUB_R = 0.026       # raised center hub on each plate

THREAD_R = 0.0150         # threaded bar end core radius
THREAD_RIDGE_R = 0.0160   # outer radius of the thread ridges
THREAD_LEN = 0.050        # exposed threaded bar past the plate stack

# Hex jam nut dimensions
HEX_FLAT = 0.040          # flat-to-flat distance across the hex
HEX_THICK = 0.011         # single nut thickness along bar axis
BORE_R = 0.0165           # central bore radius (clearance over thread ridges)
HEX_CIRCUM_R = HEX_FLAT / (2.0 * math.cos(math.pi / 6.0))  # corner radius

# ---- layout along +X (one side; mirrored for the other) ----
GRIP_HALF = HANDLE_LEN / 2.0
SLEEVE_X = GRIP_HALF + SLEEVE_LEN / 2.0
STACK_START = GRIP_HALF + SLEEVE_LEN
STACK_LEN = N_PLATES * PLATE_THICK + (N_PLATES - 1) * PLATE_GAP
STACK_END = STACK_START + STACK_LEN
THREAD_X = STACK_END + THREAD_LEN / 2.0

# Hex nut layout: inner nut seats against plate stack, outer nut jams against inner
INNER_NUT_CENTER = STACK_END + HEX_THICK / 2.0
CONTACT_FACE = STACK_END + HEX_THICK           # face where the two nuts meet
OUTER_NUT_LOCAL = HEX_THICK / 2.0              # outer nut center offset from contact face


def _rot_x(geom):
    # CylinderGeometry / TorusGeometry are built along local Z; rotate to +X.
    return geom.rotate_y(math.pi / 2.0)


def _hex_profile(flat_to_flat):
    """Regular hexagon profile in XY, one vertex at +X."""
    r = flat_to_flat / (2.0 * math.cos(math.pi / 6.0))
    return [
        (r * math.cos(i * math.pi / 3.0), r * math.sin(i * math.pi / 3.0))
        for i in range(6)
    ]


def _circle_profile(radius, n=24):
    """Circular profile in XY (counterclockwise)."""
    return [
        (radius * math.cos(i * 2.0 * math.pi / n),
         radius * math.sin(i * 2.0 * math.pi / n))
        for i in range(n)
    ]


def _hex_nut_geom():
    """Hex jam nut: six-sided prism with central bore, centered at origin, aligned to X."""
    outer = _hex_profile(HEX_FLAT)
    bore = _circle_profile(BORE_R)
    nut = ExtrudeWithHolesGeometry(outer, [bore], HEX_THICK, center=True)
    return nut.rotate_y(math.pi / 2.0)


def _plate_stack_geom(sign: float):
    stack = None
    for i in range(N_PLATES):
        r = PLATE_R - 0.004 * i
        cx = sign * (STACK_START + i * (PLATE_THICK + PLATE_GAP) + PLATE_THICK / 2.0)
        disc = _rot_x(CylinderGeometry(r, PLATE_THICK, radial_segments=56))
        disc.translate(cx, 0.0, 0.0)
        hub = _rot_x(CylinderGeometry(PLATE_HUB_R, PLATE_THICK * 0.5, radial_segments=40))
        hub.translate(cx + sign * (PLATE_THICK * 0.25), 0.0, 0.0)
        disc.merge(hub)
        ring = _rot_x(TorusGeometry(r * 0.62, 0.0022, radial_segments=10, tubular_segments=48))
        ring.translate(cx + sign * (PLATE_THICK * 0.5 - 0.001), 0.0, 0.0)
        disc.merge(ring)
        stack = disc if stack is None else stack.merge(disc)
    return stack


def _thread_geom(sign: float):
    cx = sign * THREAD_X
    core = _rot_x(CylinderGeometry(THREAD_R, THREAD_LEN, radial_segments=32))
    core.translate(cx, 0.0, 0.0)
    n_ridges = 7
    span = THREAD_LEN - 0.006
    start = THREAD_X - span / 2.0
    for j in range(n_ridges):
        rx = sign * (start + j * (span / (n_ridges - 1)))
        ridge = _rot_x(
            TorusGeometry(THREAD_R, THREAD_RIDGE_R - THREAD_R,
                          radial_segments=8, tubular_segments=28)
        )
        ridge.translate(rx, 0.0, 0.0)
        core.merge(ridge)
    return core


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_dumbbell")

    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.71, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.52, 0.54, 0.57, 1.0))
    bright = model.material("bright_chrome", rgba=(0.86, 0.88, 0.90, 1.0))
    zinc = model.material("zinc_plated", rgba=(0.72, 0.73, 0.70, 1.0))

    # ============ BODY (root): grip + sleeves + plates + threads + inner hex nuts ============
    body = model.part("body")

    grip_core = _rot_x(CylinderGeometry(HANDLE_R, HANDLE_LEN + 0.004, radial_segments=40))
    body.visual(mesh_from_geometry(grip_core, "grip_core"), material=chrome, name="grip_core")

    # Knurled grip band over the middle of the grip.
    knurl = KnobGeometry(
        GRIP_KNURL_R * 2.0,
        HANDLE_LEN * 0.62,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=44, depth=0.0009, helix_angle_deg=12.0),
        center=True,
    ).rotate_y(math.pi / 2.0)
    body.visual(mesh_from_geometry(knurl, "grip_knurl"), material=dark_steel, name="grip_knurl")

    for sign in (-1.0, 1.0):
        side = "pos" if sign > 0 else "neg"

        sleeve = _rot_x(CylinderGeometry(SLEEVE_R, SLEEVE_LEN + 0.002, radial_segments=40))
        sleeve.translate(sign * SLEEVE_X, 0.0, 0.0)
        body.visual(mesh_from_geometry(sleeve, f"sleeve_{side}"), material=steel, name=f"sleeve_{side}")

        stack = _plate_stack_geom(sign)
        body.visual(mesh_from_geometry(stack, f"plates_{side}"), material=bright, name=f"plates_{side}")

        thread = _thread_geom(sign)
        body.visual(mesh_from_geometry(thread, f"thread_{side}"), material=steel, name=f"thread_{side}")

        # Inner hex jam nut: fixed against the outermost plate, parent visual on body
        inner_nut = _hex_nut_geom()
        inner_nut.translate(sign * INNER_NUT_CENTER, 0.0, 0.0)
        body.visual(
            mesh_from_geometry(inner_nut, f"inner_hex_nut_{side}"),
            material=zinc,
            name=f"inner_hex_nut_{side}",
        )

    body.inertial = Inertial.from_geometry(
        Box((2.0 * (THREAD_X + THREAD_LEN / 2.0), 2.0 * PLATE_R, 2.0 * PLATE_R)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ============ OUTER HEX JAM NUTS (continuous spin about bar axis) ============
    for sign in (-1.0, 1.0):
        side = "pos" if sign > 0 else "neg"
        part_name = f"outer_hex_nut_{side}"
        nut_part = model.part(part_name)

        # Hex nut mesh: offset outward from the part origin (contact face)
        nut_geom = _hex_nut_geom()
        nut_geom.translate(sign * OUTER_NUT_LOCAL, 0.0, 0.0)
        nut_part.visual(
            mesh_from_geometry(nut_geom, f"{part_name}_mesh"),
            material=zinc,
            name=f"{part_name}_mesh",
        )

        # Off-axis marker embedded into the hex flat face for spin detection
        nut_part.visual(
            Box((0.004, 0.006, 0.004)),
            origin=Origin(xyz=(sign * OUTER_NUT_LOCAL, HEX_FLAT / 2.0 - 0.001, 0.0)),
            material=bright,
            name=f"{part_name}_marker",
        )

        nut_part.inertial = Inertial.from_geometry(
            Box((HEX_THICK, HEX_FLAT, HEX_FLAT)), mass=0.08
        )

        # Joint anchored at the contact face between the two nuts
        model.articulation(
            f"nut_spin_{side}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=nut_part,
            origin=Origin(xyz=(sign * CONTACT_FACE, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _center(aabb):
    mn, mx = aabb
    return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    nut_pos = object_model.get_part("outer_hex_nut_pos")
    nut_neg = object_model.get_part("outer_hex_nut_neg")
    spin_pos = object_model.get_articulation("nut_spin_pos")
    spin_neg = object_model.get_articulation("nut_spin_neg")

    # ---- Plate stacks present on both sides (tall round stacks) ----
    ext_pos = _ext(ctx.part_element_world_aabb(body, elem="plates_pos"))
    ext_neg = _ext(ctx.part_element_world_aabb(body, elem="plates_neg"))
    ctx.check(
        "positive-side plate stack is a tall round stack",
        ext_pos[1] > 0.12 and ext_pos[2] > 0.12 and ext_pos[0] > 0.04,
        details=f"plates_pos extents={ext_pos}",
    )
    ctx.check(
        "negative-side plate stack is a tall round stack",
        ext_neg[1] > 0.12 and ext_neg[2] > 0.12 and ext_neg[0] > 0.04,
        details=f"plates_neg extents={ext_neg}",
    )

    # ---- Left-right symmetry of the plate stacks ----
    cx_pos = _center(ctx.part_element_world_aabb(body, elem="plates_pos"))[0]
    cx_neg = _center(ctx.part_element_world_aabb(body, elem="plates_neg"))[0]
    ctx.check(
        "plate stacks are left-right symmetric about the bar center",
        abs(cx_pos + cx_neg) < 0.002,
        details=f"cx_pos={cx_pos}, cx_neg={cx_neg}",
    )

    # ---- Inner hex nuts have hexagonal cross-section (not cylindrical) ----
    for side in ("pos", "neg"):
        elem = f"inner_hex_nut_{side}"
        e = _ext(ctx.part_element_world_aabb(body, elem=elem))
        yz_min = min(e[1], e[2])
        yz_max = max(e[1], e[2])
        ctx.check(
            f"inner hex nut {side} has hexagonal cross-section (Y!=Z extents)",
            0.035 < yz_min < 0.050 and 0.035 < yz_max < 0.050 and abs(e[1] - e[2]) > 0.003,
            details=f"extents Y={e[1]:.4f} Z={e[2]:.4f}",
        )

    # ---- Outer hex nuts have hexagonal cross-section ----
    for nut_part, side in ((nut_pos, "pos"), (nut_neg, "neg")):
        e = _ext(ctx.part_element_world_aabb(nut_part, elem=f"outer_hex_nut_{side}_mesh"))
        yz_min = min(e[1], e[2])
        yz_max = max(e[1], e[2])
        ctx.check(
            f"outer hex nut {side} has hexagonal cross-section (Y!=Z extents)",
            0.035 < yz_min < 0.050 and 0.035 < yz_max < 0.050 and abs(e[1] - e[2]) > 0.003,
            details=f"extents Y={e[1]:.4f} Z={e[2]:.4f}",
        )

    # ---- Inner hex nuts sit outboard of the plate stacks ----
    for side, sign in (("pos", 1.0), ("neg", -1.0)):
        aabb_plates = ctx.part_element_world_aabb(body, elem=f"plates_{side}")
        plates_outer = aabb_plates[1][0] if sign > 0 else aabb_plates[0][0]
        nut_cx = _center(ctx.part_element_world_aabb(body, elem=f"inner_hex_nut_{side}"))[0]
        ctx.check(
            f"inner hex nut {side} is outboard of plate stack",
            (sign > 0 and nut_cx > plates_outer - 0.004)
            or (sign < 0 and nut_cx < plates_outer + 0.004),
            details=f"nut_cx={nut_cx}, plates_outer={plates_outer}",
        )

    # ---- Outer hex nuts sit outboard of inner hex nuts ----
    for nut_part, side, sign in ((nut_pos, "pos", 1.0), (nut_neg, "neg", -1.0)):
        aabb_inner = ctx.part_element_world_aabb(body, elem=f"inner_hex_nut_{side}")
        inner_outer = aabb_inner[1][0] if sign > 0 else aabb_inner[0][0]
        outer_cx = _center(
            ctx.part_element_world_aabb(nut_part, elem=f"outer_hex_nut_{side}_mesh")
        )[0]
        ctx.check(
            f"outer hex nut {side} is outboard of inner hex nut",
            (sign > 0 and outer_cx > inner_outer - 0.004)
            or (sign < 0 and outer_cx < inner_outer + 0.004),
            details=f"outer_cx={outer_cx}, inner_outer={inner_outer}",
        )

    # ---- Outer hex nuts thread onto the bar (intentional bore/thread overlap) ----
    for nut_part, side in ((nut_pos, "pos"), (nut_neg, "neg")):
        ctx.allow_overlap(
            nut_part, body,
            elem_a=f"outer_hex_nut_{side}_mesh", elem_b=f"thread_{side}",
            reason="Outer hex jam nut threads onto the bar end; bore intentionally overlaps the thread.",
        )
        ctx.expect_overlap(
            nut_part, body, axes="x", min_overlap=0.004,
            elem_a=f"outer_hex_nut_{side}_mesh", elem_b=f"thread_{side}",
            name=f"outer hex nut {side} seated on its thread",
        )

    # ---- Outer hex nuts spin about the bar axis: marker swings off-axis ----
    for nut_part, spin, side in (
        (nut_pos, spin_pos, "pos"),
        (nut_neg, spin_neg, "neg"),
    ):
        m_elem = f"outer_hex_nut_{side}_marker"
        rest_c = _center(ctx.part_element_world_aabb(nut_part, elem=m_elem))
        ctx.check(
            f"{side} marker rests above the bar axis (+Y)",
            rest_c[1] > 0.015 and abs(rest_c[2]) < 0.010,
            details=f"{side} rest marker center={rest_c}",
        )
        with ctx.pose({spin: math.pi / 2.0}):
            turn_c = _center(ctx.part_element_world_aabb(nut_part, elem=m_elem))
        ctx.check(
            f"{side} outer hex nut spin swings marker from +Y toward +Z",
            turn_c[2] > 0.015 and abs(turn_c[1]) < 0.010,
            details=f"{side} rest={rest_c}, quarter-turn={turn_c}",
        )

    # ---- Two hex nut parts exist (one per side) confirming paired jam-nut design ----
    ctx.check(
        "paired jam-nut collar: both outer hex nut parts exist",
        nut_pos is not None and nut_neg is not None,
        details="outer_hex_nut_pos or outer_hex_nut_neg missing",
    )

    return ctx.report()


object_model = build_object_model()
