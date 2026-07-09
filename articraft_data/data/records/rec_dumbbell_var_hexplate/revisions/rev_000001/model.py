from __future__ import annotations

# Adjustable chrome spin-lock dumbbell with hexagonal cast weight plates.
# Frame: handle axis along +X, centered at origin (x=0 is the bar midpoint),
# left side at -X, right side at +X. Plates stack outward from the grip; each
# threaded bar end carries a knurled spin-lock collar (nut) outboard of the
# plate stack.
# Plate shape : regular hexagonal prisms (6-sided) with center bore, raised hub,
#               and embossed ring. Flat bottom edge lets them sit on the floor.
# Static body  : knurled handle + shoulder sleeves + a hex plate stack per side +
#                threaded bar ends.
# Articulations:
#   - left spin-lock collar  : CONTINUOUS rotation about the handle axis (+X)
#   - right spin-lock collar : CONTINUOUS rotation about the handle axis (+X)
# Each collar carries a small off-axis marker notch so its spin is detectable.

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

PLATE_R = 0.080           # plate circumscribed radius (corner-to-corner ~0.16 m)
PLATE_THICK = 0.012       # single plate thickness
PLATE_GAP = 0.0008        # tiny seam between plates
N_PLATES = 5
PLATE_HUB_R = 0.026       # raised center hub on each plate
BORE_R = 0.016            # center bore radius (slips over bar sleeve)

THREAD_R = 0.0150         # threaded bar end core radius
THREAD_RIDGE_R = 0.0160   # outer radius of the thread ridges
THREAD_LEN = 0.050        # exposed threaded bar past the plate stack

COLLAR_DIA = 0.044        # spin-lock collar (knurled nut) outer diameter
COLLAR_LEN = 0.022        # collar length along the bar

# ---- layout along +X (one side; mirrored for the other) ----
GRIP_HALF = HANDLE_LEN / 2.0
SLEEVE_X = GRIP_HALF + SLEEVE_LEN / 2.0
STACK_START = GRIP_HALF + SLEEVE_LEN
STACK_LEN = N_PLATES * PLATE_THICK + (N_PLATES - 1) * PLATE_GAP
STACK_END = STACK_START + STACK_LEN
THREAD_X = STACK_END + THREAD_LEN / 2.0
COLLAR_X = STACK_END + 0.026   # collar sits on the thread, just outboard of plates


def _rot_x(geom):
    # CylinderGeometry / TorusGeometry are built along local Z; rotate to +X.
    return geom.rotate_y(math.pi / 2.0)


def _hex_profile(radius):
    """Regular hexagon inscribed in a circle of *radius*.

    Vertex angles 30°, 90°, 150°, 210°, 270°, 330° give flat edges at
    max/min local X.  After rotate_y(pi/2) (local X → world -Z), the flat
    edges land at the top and bottom of the plate in world space so the
    hex plate sits flat on the floor without rolling.
    """
    pts = []
    for i in range(6):
        a = math.pi / 6.0 + i * math.pi / 3.0
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return pts


def _circle_profile(radius, segments=36):
    """Counter-clockwise circular profile in local XY."""
    pts = []
    for i in range(segments):
        a = i * 2.0 * math.pi / segments
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return pts


def _hex_plate_geom(radius, thickness, sign, cx):
    """Single hex plate with center bore, raised hub, and embossed ring.

    *sign* = +1 / -1 for the +X / -X side (controls hub/ring offset).
    *cx*   = world X center of this plate.
    """
    # Hex body with center bore, extruded along local Z then rotated to X.
    outer = _hex_profile(radius)
    bore = _circle_profile(BORE_R)
    plate = ExtrudeWithHolesGeometry(
        outer, [bore], thickness, cap=True, center=True,
    )
    plate.rotate_y(math.pi / 2.0)
    plate.translate(cx, 0.0, 0.0)

    # Raised embossed center hub on the outward face.
    hub = _rot_x(CylinderGeometry(PLATE_HUB_R, thickness * 0.5, radial_segments=40))
    hub.translate(cx + sign * (thickness * 0.25), 0.0, 0.0)
    plate.merge(hub)

    # Embossed raised rim ring (weight-relief style) on the outer face.
    ring = _rot_x(TorusGeometry(radius * 0.62, 0.0022, radial_segments=10, tubular_segments=6))
    ring.translate(cx + sign * (thickness * 0.5 - 0.001), 0.0, 0.0)
    plate.merge(ring)

    return plate


def _plate_stack_geom(sign: float):
    """One side's hex plate stack merged into a single mesh."""
    stack = None
    for i in range(N_PLATES):
        r = PLATE_R - 0.004 * i  # gentle outward taper on successive plates
        cx = sign * (STACK_START + i * (PLATE_THICK + PLATE_GAP) + PLATE_THICK / 2.0)
        plate = _hex_plate_geom(r, PLATE_THICK, sign, cx)
        stack = plate if stack is None else stack.merge(plate)
    return stack


def _thread_geom(sign: float):
    # Threaded bar end: thin core cylinder + torus ridges that read as a screw thread.
    cx = sign * THREAD_X
    core = _rot_x(CylinderGeometry(THREAD_R, THREAD_LEN, radial_segments=32))
    core.translate(cx, 0.0, 0.0)
    n_ridges = 7
    span = THREAD_LEN - 0.006
    start = THREAD_X - span / 2.0
    for j in range(n_ridges):
        rx = sign * (start + j * (span / (n_ridges - 1)))
        ridge = _rot_x(
            TorusGeometry(THREAD_R, THREAD_RIDGE_R - THREAD_R, radial_segments=8, tubular_segments=28)
        )
        ridge.translate(rx, 0.0, 0.0)
        core.merge(ridge)
    return core


def _collar_geom():
    # Knurled spin-lock collar (nut) built about local +Z, rotated to +X.
    nut = KnobGeometry(
        COLLAR_DIA,
        COLLAR_LEN,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=30, depth=0.0010, helix_angle_deg=18.0),
        center=True,
    )
    return nut.rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_dumbbell")

    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.71, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.52, 0.54, 0.57, 1.0))
    bright = model.material("bright_chrome", rgba=(0.86, 0.88, 0.90, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.38, 0.39, 0.41, 1.0))

    # ============ BODY (root): grip + sleeves + hex plate stacks + threaded ends ============
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
        body.visual(mesh_from_geometry(stack, f"plates_{side}"), material=cast_iron, name=f"plates_{side}")

        thread = _thread_geom(sign)
        body.visual(mesh_from_geometry(thread, f"thread_{side}"), material=steel, name=f"thread_{side}")

    body.inertial = Inertial.from_geometry(
        Box((2.0 * (THREAD_X + THREAD_LEN / 2.0), 2.0 * PLATE_R, 2.0 * PLATE_R)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ============ SPIN-LOCK COLLARS (continuous spin about +X) ============
    for sign in (-1.0, 1.0):
        side = "pos" if sign > 0 else "neg"
        cname = f"spinlock_collar_{side}"
        collar = model.part(cname)

        nut = _collar_geom()
        collar.visual(mesh_from_geometry(nut, f"{cname}_nut"), material=steel, name=f"{cname}_nut")

        # Off-axis marker notch so the spin is detectable by AABB orientation tests.
        collar.visual(
            Box((0.006, 0.006, 0.014)),
            origin=Origin(xyz=(0.0, COLLAR_DIA / 2.0 + 0.001, 0.0)),
            material=bright,
            name=f"{cname}_marker",
        )

        collar.inertial = Inertial.from_geometry(
            Box((COLLAR_LEN, COLLAR_DIA, COLLAR_DIA)), mass=0.10
        )

        model.articulation(
            f"collar_spin_{side}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=collar,
            origin=Origin(xyz=(sign * COLLAR_X, 0.0, 0.0)),
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
    collar_pos = object_model.get_part("spinlock_collar_pos")
    collar_neg = object_model.get_part("spinlock_collar_neg")
    spin_pos = object_model.get_articulation("collar_spin_pos")
    spin_neg = object_model.get_articulation("collar_spin_neg")

    # ---- Hex plate stacks present on both sides ----
    ext_pos = _ext(ctx.part_element_world_aabb(body, elem="plates_pos"))
    ext_neg = _ext(ctx.part_element_world_aabb(body, elem="plates_neg"))
    for ext, tag in ((ext_pos, "pos"), (ext_neg, "neg")):
        ctx.check(
            f"{tag}-side hex plate stack has substantial span",
            ext[1] > 0.12 and ext[2] > 0.10 and ext[0] > 0.04,
            details=f"plates_{tag} extents={ext}",
        )

    # ---- Plates are hexagonal: Z extent < Y extent (flat-to-flat < corner-to-corner) ----
    # For a regular hexagon inscribed in circle R: Y span = 2R, Z span = R*sqrt(3).
    # Ratio Z/Y = sqrt(3)/2 ≈ 0.866.  A round plate would give ~1.0.
    for ext, tag in ((ext_pos, "pos"), (ext_neg, "neg")):
        ratio = ext[2] / ext[1] if ext[1] > 0 else 0
        ctx.check(
            f"{tag}-side plates are hexagonal (Z/Y ratio < 0.95)",
            0.78 < ratio < 0.95,
            details=f"plates_{tag} Y={ext[1]:.4f} Z={ext[2]:.4f} ratio={ratio:.3f}",
        )

    # ---- Left-right symmetry of the static body ----
    cx_pos = _center(ctx.part_element_world_aabb(body, elem="plates_pos"))[0]
    cx_neg = _center(ctx.part_element_world_aabb(body, elem="plates_neg"))[0]
    ctx.check(
        "plate stacks are left-right symmetric about the bar center",
        abs(cx_pos + cx_neg) < 0.002,
        details=f"cx_pos={cx_pos}, cx_neg={cx_neg}",
    )

    # ---- Collars sit outboard of the plate stacks ----
    plates_pos_outer = ctx.part_element_world_aabb(body, elem="plates_pos")[1][0]  # max X
    collar_pos_cx = ctx.part_world_position(collar_pos)[0]
    ctx.check(
        "positive collar is outboard of the positive plate stack",
        collar_pos_cx > plates_pos_outer - 0.004,
        details=f"collar_pos_x={collar_pos_cx}, plates_pos_outer={plates_pos_outer}",
    )
    plates_neg_outer = ctx.part_element_world_aabb(body, elem="plates_neg")[0][0]  # min X
    collar_neg_cx = ctx.part_world_position(collar_neg)[0]
    ctx.check(
        "negative collar is outboard of the negative plate stack",
        collar_neg_cx < plates_neg_outer + 0.004,
        details=f"collar_neg_x={collar_neg_cx}, plates_neg_outer={plates_neg_outer}",
    )

    # ---- Collars seated on the threaded bar ends (intentional thread overlap) ----
    ctx.allow_overlap(
        collar_pos, body,
        elem_a="spinlock_collar_pos_nut", elem_b="thread_pos",
        reason="Spin-lock collar threads onto the bar end; nut bore intentionally overlaps the thread.",
    )
    ctx.allow_overlap(
        collar_neg, body,
        elem_a="spinlock_collar_neg_nut", elem_b="thread_neg",
        reason="Spin-lock collar threads onto the bar end; nut bore intentionally overlaps the thread.",
    )
    ctx.expect_overlap(
        collar_pos, body, axes="x", min_overlap=0.004,
        elem_a="spinlock_collar_pos_nut", elem_b="thread_pos",
        name="positive collar seated on its thread",
    )
    ctx.expect_overlap(
        collar_neg, body, axes="x", min_overlap=0.004,
        elem_a="spinlock_collar_neg_nut", elem_b="thread_neg",
        name="negative collar seated on its thread",
    )

    # ---- Both collars spin about the handle axis: marker swings off-axis ----
    for collar, spin, tag in ((collar_pos, spin_pos, "pos"), (collar_neg, spin_neg, "neg")):
        m_elem = f"spinlock_collar_{tag}_marker"
        rest_c = _center(ctx.part_element_world_aabb(collar, elem=m_elem))
        ctx.check(
            f"{tag} marker rests above the axis (+Y)",
            rest_c[1] > 0.015 and abs(rest_c[2]) < 0.010,
            details=f"{tag} rest marker center={rest_c}",
        )
        with ctx.pose({spin: math.pi / 2.0}):
            turn_c = _center(ctx.part_element_world_aabb(collar, elem=m_elem))
        ctx.check(
            f"{tag} collar spin swings marker from +Y toward +Z",
            turn_c[2] > 0.015 and abs(turn_c[1]) < 0.010,
            details=f"{tag} rest={rest_c}, quarter-turn={turn_c}",
        )

    return ctx.report()


object_model = build_object_model()
