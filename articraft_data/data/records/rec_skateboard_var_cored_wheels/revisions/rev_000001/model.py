from __future__ import annotations

# Wooden skateboard with a natural maple deck and two metal trucks.
# Frame:
#   +X = nose direction, -X = tail (deck length ~0.80 m along X)
#   +Y / -Y = the two sides of the deck (width ~0.20 m along Y)
#   +Z = up. The deck bottom face sits near z=0; the board rides on four
#        wheels below. Nose/tail kick upward in +Z.
# Articulations:
#   - front truck hanger: REVOLUTE lean/steer about the tilted kingpin axis (~+/-15 deg).
#       its two front wheels: each CONTINUOUS roll about the front axle.
#   - rear truck hanger:  REVOLUTE lean about its kingpin axis (~+/-15 deg).
#       its two rear wheels: each CONTINUOUS roll about the rear axle.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---- key dimensions ----
DECK_LEN = 0.80
DECK_WIDTH = 0.205
DECK_THICK = 0.012

# Truck mount X positions (baseplate bolt centers).
FRONT_X = 0.275
REAR_X = -0.275

# Vertical layout. Deck bottom face nominally at z=0.
DECK_BOTTOM = 0.0
BASEPLATE_THICK = 0.010
AXLE_Z = -0.040  # axle / hanger center height below deck

WHEEL_RADIUS = 0.0275
WHEEL_WIDTH = 0.034
WHEEL_Y = 0.092  # half-track: lateral offset of each wheel from centerline

KINGPIN_TILT = math.radians(28.0)  # baseplate pivot angle from vertical


def _deck_mesh():
    # Lofted popsicle deck: long oval, rounded nose & tail, slight upward kick
    # at the very nose and tail. LoftGeometry validates profile area in the XY
    # projection, so each cross-section is authored as a closed loop in the XY
    # plane (width along X, thickness along Y) at a constant Z station, then the
    # whole loft is rotated so the loft axis (Z) maps to the deck length (X).
    half_len = DECK_LEN / 2.0
    # (frac along half-length, half_width, centerline kick rise)
    raw = [
        (-1.00, 0.030, 0.030),  # tail tip (kicked up)
        (-0.965, 0.060, 0.022),
        (-0.92, 0.085, 0.012),
        (-0.85, 0.097, 0.004),
        (-0.70, 0.1015, 0.0),
        (-0.40, 0.1025, 0.0),
        (0.0, 0.1025, 0.0),
        (0.40, 0.1025, 0.0),
        (0.70, 0.1015, 0.0),
        (0.85, 0.097, 0.004),
        (0.92, 0.085, 0.012),
        (0.965, 0.060, 0.022),
        (1.00, 0.030, 0.030),  # nose tip (kicked up)
    ]
    profiles = []
    for frac, half_w, kick in raw:
        z = frac * half_len  # station along the (pre-rotation) loft axis Z
        w = max(half_w * 2.0, 0.006)
        # Closed rounded-rect loop in XY: X = deck width, Y = deck thickness.
        prof2d = rounded_rect_profile(w, DECK_THICK, min(DECK_THICK * 0.45, half_w))
        loop = [(px, py + kick, z) for (px, py) in prof2d]
        profiles.append(loop)
    deck = LoftGeometry(profiles, cap=True, closed=True)
    # Map the loft axis Z -> X (deck length) and the in-plane width/thickness
    # into Y (width) and Z (thickness, kicking upward at the ends).
    deck.rotate_y(math.pi / 2.0)  # old +Z(length) -> +X
    deck.rotate_x(math.pi / 2.0)  # bring width->Y, thickness->Z (kick up +Z)
    deck.translate(0.0, 0.0, DECK_THICK / 2.0)  # flat board bottom near z=0
    return mesh_from_geometry(deck, "deck_top")


# ---- cored longboard wheel geometry ----
# The wheel spins about local Y (axle direction). The radial disk lies in XZ.
NUM_SPOKES = 5
HUB_RADIUS = 0.009
SPOKE_OUTER_R = 0.019
URETHANE_INNER_R = 0.0185  # slightly inside spoke tips for seated connection


def _spoke_geo(inner_r: float, outer_r: float, axial_thick: float, tang_width: float):
    """Shared spoke helper: a single radial bar along +X from inner_r to outer_r.

    Emitted in a for-i-in-range loop at equal angles to form the cored wheel
    inner structure. Returns a BoxGeometry positioned at the correct radial offset.
    """
    length = outer_r - inner_r
    geo = BoxGeometry((length, axial_thick, tang_width))
    geo.translate(inner_r + length / 2.0, 0.0, 0.0)
    return geo


def _hub_core_mesh(name: str):
    """Hard inner hub core with NUM_SPOKES spokes at equal angles around the bore.

    The open windows between spokes are the natural gaps in the merged geometry.
    """
    spoke_axial = WHEEL_WIDTH * 0.72
    spoke_tang = 0.009

    # Central hub cylinder along Y (axle direction)
    hub = CylinderGeometry(HUB_RADIUS, spoke_axial, radial_segments=24)
    hub.rotate_x(math.pi / 2.0)  # align cylinder Z-axis to Y (axle)

    # Bearing seat rings at each face (slightly larger than hub)
    for z_sign in (-1.0, 1.0):
        seat = CylinderGeometry(HUB_RADIUS + 0.003, 0.004, radial_segments=24)
        seat.rotate_x(math.pi / 2.0)
        seat.translate(0.0, z_sign * (spoke_axial / 2.0 - 0.002), 0.0)
        hub.merge(seat)

    # Emit spokes via for-i-in-range loop at equal angles
    for i in range(NUM_SPOKES):
        angle = 2.0 * math.pi * i / NUM_SPOKES
        spoke = _spoke_geo(HUB_RADIUS, SPOKE_OUTER_R, spoke_axial * 0.80, spoke_tang)
        spoke.rotate_y(angle)
        hub.merge(spoke)

    return mesh_from_geometry(hub, name)


def _urethane_ring_mesh(name: str):
    """Softer outer urethane ring as the rolling contact band.

    Built as a lathed annular cross-section with slightly rounded outer edges.
    """
    r_in = URETHANE_INNER_R
    r_out = WHEEL_RADIUS
    hw = WHEEL_WIDTH / 2.0
    edge_r = 0.002  # slight edge rounding

    # Lathe profile in (radius, z): rectangular with rounded outer corners
    profile = [
        (r_in, -hw),
        (r_out - edge_r, -hw),
        (r_out, -hw + edge_r),
        (r_out, hw - edge_r),
        (r_out - edge_r, hw),
        (r_in, hw),
    ]
    ring = LatheGeometry(profile, segments=48, closed=True)
    ring.rotate_x(math.pi / 2.0)  # spin axis from Z to Y (axle)
    return mesh_from_geometry(ring, name)


def _baseplate_mesh(name: str):
    # Flat metal baseplate that bolts to the deck bottom, with a pivot boss
    # sticking down to capture the hanger kingpin neck.
    plate = BoxGeometry((0.075, 0.075, BASEPLATE_THICK))
    plate.translate(0.0, 0.0, -BASEPLATE_THICK / 2.0)
    boss = CylinderGeometry(0.011, 0.022)
    boss.translate(0.0, 0.0, -BASEPLATE_THICK - 0.006)
    plate.merge(boss)
    return mesh_from_geometry(plate, name)


def _hanger_mesh(name: str):
    # Cast hanger body + cross axle (along Y) carrying both wheels, with a
    # pivot neck reaching up toward the baseplate kingpin boss.
    geo = BoxGeometry((0.036, 0.052, 0.030))
    geo.translate(0.0, 0.0, 0.004)
    neck = CylinderGeometry(0.010, 0.026)
    neck.translate(0.0, 0.0, 0.022)
    geo.merge(neck)
    axle = CylinderGeometry(0.0065, WHEEL_Y * 2.0 + WHEEL_WIDTH * 1.2)
    axle.rotate_x(math.pi / 2.0)  # align cylinder (local Z) to Y
    geo.merge(axle)
    for s in (-1.0, 1.0):
        arm = BoxGeometry((0.024, WHEEL_Y, 0.020))
        arm.translate(0.0, s * WHEEL_Y / 2.0, 0.0)
        geo.merge(arm)
    return mesh_from_geometry(geo, name)


def _bolt_mesh(name: str):
    head = CylinderGeometry(0.0045, 0.005)
    return mesh_from_geometry(head, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard")

    maple = model.material("maple_deck", rgba=(0.74, 0.58, 0.36, 1.0))
    metal = model.material("truck_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    hub_core_mat = model.material("hub_core", rgba=(0.88, 0.88, 0.86, 1.0))  # hard white plastic
    urethane_mat = model.material("urethane", rgba=(0.82, 0.48, 0.22, 1.0))  # softer orange PU
    bolt_dark = model.material("bolt_dark", rgba=(0.20, 0.20, 0.22, 1.0))

    # ---------------- deck (root) ----------------
    deck = model.part("deck")
    deck.visual(_deck_mesh(), material=maple, name="deck_board")

    # Mounting bolts: 4 per truck, rectangular pattern, heads proud on the top.
    bolt_dx = 0.030
    bolt_dy = 0.020
    for tx in (FRONT_X, REAR_X):
        for i, sx in enumerate((-1.0, 1.0)):
            for j, sy in enumerate((-1.0, 1.0)):
                tag = f"{tx:+.2f}_{i}_{j}".replace(".", "p").replace("+", "p").replace("-", "m")
                deck.visual(
                    _bolt_mesh(f"bolt_{tag}"),
                    origin=Origin(xyz=(tx + sx * bolt_dx, sy * bolt_dy, DECK_THICK + 0.001)),
                    material=bolt_dark,
                    name=f"deck_bolt_{tag}",
                )

    deck.inertial = Inertial.from_geometry(
        Box((DECK_LEN, DECK_WIDTH, DECK_THICK)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, DECK_THICK / 2.0)),
    )

    # ---------------- two trucks ----------------
    # Each truck = baseplate (fixed to deck) + hanger (revolute kingpin) + 2 wheels.
    truck_defs = [
        ("front", FRONT_X, 1.0),  # boss tilts toward nose (+X)
        ("rear", REAR_X, -1.0),  # boss tilts toward tail (-X)
    ]

    for label, tx, tilt_sign in truck_defs:
        # ---- baseplate: fixed to deck bottom ----
        bp = model.part(f"{label}_baseplate")
        bp.visual(_baseplate_mesh(f"{label}_plate"), material=metal, name="baseplate")
        bp.inertial = Inertial.from_geometry(
            Box((0.075, 0.075, 0.030)), mass=0.10, origin=Origin(xyz=(0.0, 0.0, -0.012))
        )
        model.articulation(
            f"deck_to_{label}_baseplate",
            ArticulationType.FIXED,
            parent=deck,
            child=bp,
            origin=Origin(xyz=(tx, 0.0, DECK_BOTTOM)),
        )

        # ---- hanger: REVOLUTE about the tilted kingpin axis ----
        hanger = model.part(f"{label}_hanger")
        hanger.visual(_hanger_mesh(f"{label}_hanger_body"), material=metal, name="hanger")
        hanger.inertial = Inertial.from_geometry(Box((0.04, WHEEL_Y * 2.0, 0.03)), mass=0.18)
        kp_axis = (tilt_sign * math.sin(KINGPIN_TILT), 0.0, math.cos(KINGPIN_TILT))
        model.articulation(
            f"{label}_baseplate_to_hanger",
            ArticulationType.REVOLUTE,
            parent=bp,
            child=hanger,
            origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
            axis=kp_axis,
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=4.0,
                lower=-math.radians(15.0),
                upper=math.radians(15.0),
            ),
        )

        # ---- two wheels: each CONTINUOUS roll about the axle (Y axis) ----
        # Cored longboard wheel: hard inner hub core with spokes + soft outer urethane ring.
        for s, side in ((-1.0, "0"), (1.0, "1")):
            wheel = model.part(f"{label}_wheel_{side}")
            wheel.visual(
                _hub_core_mesh(f"{label}_wheel_{side}_core"),
                material=hub_core_mat,
                name="hub_core",
            )
            wheel.visual(
                _urethane_ring_mesh(f"{label}_wheel_{side}_ring"),
                material=urethane_mat,
                name="urethane_ring",
            )
            # Roll marker: small pip on the urethane ring face for roll verification
            pip = CylinderGeometry(0.003, 0.005)
            pip.rotate_x(math.pi / 2.0)  # spin axis along Y
            pip.translate(0.0, 0.0, WHEEL_RADIUS * 0.75)
            wheel.visual(
                mesh_from_geometry(pip, f"{label}_wheel_{side}_pip"),
                material=bolt_dark,
                name="roll_marker",
            )
            wheel.inertial = Inertial.from_geometry(
                Box((WHEEL_WIDTH, WHEEL_RADIUS * 2, WHEEL_RADIUS * 2)), mass=0.06
            )
            model.articulation(
                f"{label}_hanger_to_wheel_{side}",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(xyz=(0.0, s * WHEEL_Y, 0.0)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=30.0),
            )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")

    # ---- deck is longer than wide, with nose & tail kick ----
    dext = _ext(ctx.part_world_aabb(deck))
    ctx.check(
        "deck longer than wide",
        dext[0] > dext[1] + 0.3,
        details=f"deck extents={dext}",
    )
    ctx.check(
        "deck has nose/tail kick (vertical rise)",
        dext[2] > DECK_THICK + 0.015,
        details=f"deck z-extent={dext[2]:.4f}",
    )

    # ---- four wheels rest on a common ground plane ----
    wheel_bottoms = []
    for label in ("front", "rear"):
        for side in ("0", "1"):
            w = object_model.get_part(f"{label}_wheel_{side}")
            wheel_bottoms.append(ctx.part_world_aabb(w)[0][2])
    zmin = min(wheel_bottoms)
    zmax = max(wheel_bottoms)
    ctx.check(
        "board rests on four wheels (coplanar wheel bottoms)",
        (zmax - zmin) < 0.006,
        details=f"wheel bottoms={[round(z, 4) for z in wheel_bottoms]}",
    )
    deck_zmin = ctx.part_world_aabb(deck)[0][2]
    ctx.check(
        "wheels are the lowest part of the assembly",
        zmin < deck_zmin - 0.04,
        details=f"wheel zmin={zmin:.4f}, deck zmin={deck_zmin:.4f}",
    )

    # ---- trucks symmetric front/back about the deck center ----
    fbp = ctx.part_world_position(object_model.get_part("front_baseplate"))
    rbp = ctx.part_world_position(object_model.get_part("rear_baseplate"))
    ctx.check(
        "two trucks symmetric front/back",
        abs(fbp[0] + rbp[0]) < 0.01 and abs(fbp[0]) > 0.2,
        details=f"front_x={fbp[0]:.3f}, rear_x={rbp[0]:.3f}",
    )

    # ---- each truck hanger leans about its tilted kingpin axis ----
    for label in ("front", "rear"):
        hanger = object_model.get_part(f"{label}_hanger")
        kp = object_model.get_articulation(f"{label}_baseplate_to_hanger")
        rest_ext = _ext(ctx.part_world_aabb(hanger))
        with ctx.pose({kp: math.radians(15.0)}):
            leaned_ext = _ext(ctx.part_world_aabb(hanger))
        ctx.check(
            f"{label} hanger leans about kingpin (axle tips)",
            leaned_ext[2] > rest_ext[2] + 0.004,
            details=f"{label} rest_z={rest_ext[2]:.4f} leaned_z={leaned_ext[2]:.4f}",
        )

    # ---- each of the 4 wheels rolls about its axle (marker moves) ----
    for label in ("front", "rear"):
        for side in ("0", "1"):
            wheel = object_model.get_part(f"{label}_wheel_{side}")
            roll = object_model.get_articulation(f"{label}_hanger_to_wheel_{side}")
            pip0 = ctx.part_element_world_aabb(wheel, elem="roll_marker")
            c0 = ((pip0[0][0] + pip0[1][0]) / 2.0, (pip0[0][2] + pip0[1][2]) / 2.0)
            with ctx.pose({roll: math.pi / 2.0}):
                pip1 = ctx.part_element_world_aabb(wheel, elem="roll_marker")
                c1 = ((pip1[0][0] + pip1[1][0]) / 2.0, (pip1[0][2] + pip1[1][2]) / 2.0)
            moved = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
            ctx.check(
                f"{label} wheel {side} rolls about axle (marker moves)",
                moved > 0.01,
                details=f"marker moved {moved:.4f} m",
            )

    # ---- cored longboard wheel geometry checks ----
    # Each wheel must have a distinct hub core and urethane ring visual.
    for label in ("front", "rear"):
        for side in ("0", "1"):
            wheel = object_model.get_part(f"{label}_wheel_{side}")
            hub_aabb = ctx.part_element_world_aabb(wheel, elem="hub_core")
            ring_aabb = ctx.part_element_world_aabb(wheel, elem="urethane_ring")
            ctx.check(
                f"{label} wheel {side} has hub core visual",
                hub_aabb is not None,
                details="hub_core AABB is None",
            )
            ctx.check(
                f"{label} wheel {side} has urethane ring visual",
                ring_aabb is not None,
                details="urethane_ring AABB is None",
            )
            if hub_aabb is not None and ring_aabb is not None:
                # Hub core should be smaller radially than the urethane ring
                hub_dx = hub_aabb[1][0] - hub_aabb[0][0]
                hub_dz = hub_aabb[1][2] - hub_aabb[0][2]
                hub_radial = max(hub_dx, hub_dz)
                ring_dx = ring_aabb[1][0] - ring_aabb[0][0]
                ring_dz = ring_aabb[1][2] - ring_aabb[0][2]
                ring_radial = max(ring_dx, ring_dz)
                ctx.check(
                    f"{label} wheel {side} hub core smaller than urethane ring",
                    hub_radial < ring_radial - 0.005,
                    details=f"hub_radial={hub_radial:.4f}, ring_radial={ring_radial:.4f}",
                )

    # ---- intentional seated overlaps ----
    for label in ("front", "rear"):
        bp = object_model.get_part(f"{label}_baseplate")
        hanger = object_model.get_part(f"{label}_hanger")
        ctx.allow_overlap(
            bp,
            deck,
            elem_a="baseplate",
            elem_b="deck_board",
            reason="Baseplate is bolted flush against the maple deck bottom.",
        )
        ctx.expect_contact(bp, deck, name=f"{label} baseplate seated on deck")
        ctx.allow_overlap(
            hanger,
            bp,
            elem_a="hanger",
            elem_b="baseplate",
            reason="Hanger pivot neck is captured by the baseplate kingpin boss.",
        )
        for side in ("0", "1"):
            wheel = object_model.get_part(f"{label}_wheel_{side}")
            ctx.allow_overlap(
                wheel,
                hanger,
                elem_a="hub_core",
                elem_b="hanger",
                reason="Hub core bore seats on the hanger cross-axle.",
            )
            # The urethane ring wraps the hub core; its inner radius slightly
            # overlaps the spoke tips where it is molded/seated onto the core.
            ctx.allow_overlap(
                wheel,
                wheel,
                elem_a="urethane_ring",
                elem_b="hub_core",
                reason="Soft urethane ring is molded/seated onto the hard hub core spoke tips.",
            )
            ctx.expect_contact(
                wheel,
                wheel,
                elem_a="urethane_ring",
                elem_b="hub_core",
                name=f"{label} wheel {side} urethane ring seated on hub core",
            )

    return ctx.report()


object_model = build_object_model()
