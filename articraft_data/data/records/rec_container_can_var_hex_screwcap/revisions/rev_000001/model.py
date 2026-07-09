from __future__ import annotations

# Hexagonal brushed-metal tea tin with a round screw cap on a threaded neck.
# Fork of the straight-lift-off skirt lid variant: the closure is now a round
# threaded neck rising from the center of the hexagonal top plus a knurled
# screw cap that spins (CONTINUOUS) then lifts off (PRISMATIC through a
# massless carrier). The hexagonal-prism body footprint is unchanged.
#
# Frame: hexagonal prism axis along +Z, centered on the world origin in XY.
#   - body: hexagonal prism (closed top with access hole) + threaded neck.
#   - cap_carrier: kinematic intermediary (prismatic lift), carries gasket.
#   - cap: knurled round screw cap (continuous spin on carrier).
#
# Scale: ~0.07 m across flats, body ~0.13 m tall, neck ~0.015 m visible.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Hexagon sizing (identical to parent) ---
ACROSS_FLATS = 0.070
BODY_CIRCUM = ACROSS_FLATS / (2.0 * math.cos(math.radians(30.0)))
BODY_DIAM = BODY_CIRCUM * 2.0

WALL = 0.0025
BODY_INNER_CIRCUM = BODY_CIRCUM - WALL / math.cos(math.radians(30.0))
BODY_INNER_DIAM = BODY_INNER_CIRCUM * 2.0

BODY_HEIGHT = 0.130
BODY_FLOOR = 0.004

# --- Round threaded neck ---
NECK_OD = 0.028
NECK_ID = 0.022
NECK_HEIGHT = 0.015           # visible height above hex top
NECK_BASE_OD = 0.029          # base flange OD (< cap bore for removal)
NECK_BASE_H = 0.003           # base flange height (embeds into top plate)
NECK_TOTAL_H = NECK_HEIGHT + NECK_BASE_H  # total neck geometry height
THREAD_RIDGE = 0.0008
THREAD_WIDTH = 0.0015
THREAD_COUNT = 3

# --- Knurled screw cap ---
CAP_OD = 0.035
CAP_TOP_THICK = 0.005
CAP_SKIRT = 0.012
CAP_HEIGHT = CAP_TOP_THICK + CAP_SKIRT  # 0.017
CAP_BORE = NECK_OD + 0.002             # 0.030

# --- Carrier gasket ---
GASKET_OD = CAP_BORE                    # contacts cap bore wall
GASKET_ID = NECK_ID
GASKET_THICK = 0.001

# --- Prismatic travel ---
LIFT_TRAVEL = 0.025


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _hex_body() -> cq.Workplane:
    """Hollow hexagonal prism with integrated top plate and neck access hole.

    The cavity stops WALL below the body top, leaving a solid top plate.
    A round access hole (matching the neck bore) passes through the plate.
    """
    outer = (
        cq.Workplane("XY")
        .polygon(6, BODY_DIAM)
        .extrude(BODY_HEIGHT)
    )
    # Cavity: from floor to just below top plate
    cavity_depth = BODY_HEIGHT - BODY_FLOOR - WALL
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BODY_FLOOR)
        .polygon(6, BODY_INNER_DIAM)
        .extrude(cavity_depth)
    )
    body = outer.cut(cavity)
    # Access hole through top plate for neck bore alignment
    hole = (
        cq.Workplane("XY")
        .workplane(offset=BODY_HEIGHT - WALL - 0.001)
        .circle(NECK_ID / 2.0)
        .extrude(WALL + 0.002)
    )
    return body.cut(hole)


def _neck_tube() -> cq.Workplane:
    """Neck with base flange and threaded tube, base at z=0.

    The base flange embeds into the body top plate for connectivity.
    The tube rises above the hex top for the screw cap engagement.
    """
    base = (
        cq.Workplane("XY")
        .circle(NECK_BASE_OD / 2.0)
        .circle(NECK_ID / 2.0)
        .extrude(NECK_BASE_H)
    )
    tube = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BASE_H)
        .circle(NECK_OD / 2.0)
        .circle(NECK_ID / 2.0)
        .extrude(NECK_HEIGHT)
    )
    return base.union(tube)


def _thread_ridge_ring() -> cq.Workplane:
    """One circumferential thread ridge ring, base at z=0."""
    return (
        cq.Workplane("XY")
        .circle(NECK_OD / 2.0 + THREAD_RIDGE)
        .circle(NECK_OD / 2.0 - 0.0003)
        .extrude(THREAD_WIDTH)
    )


def _carrier_gasket() -> cq.Workplane:
    """Thin annular sealing gasket, base at z=0."""
    return (
        cq.Workplane("XY")
        .circle(GASKET_OD / 2.0)
        .circle(GASKET_ID / 2.0)
        .extrude(GASKET_THICK)
    )


def _screw_cap_body() -> cq.Workplane:
    """Round screw cap with blind bore and knurling ridges, base at z=0.

    The blind bore extends slightly past the skirt height so the solid
    top disc clears the neck top face without overlap.
    """
    body_r = CAP_OD / 2.0 - 0.0004
    cap = cq.Workplane("XY").circle(body_r).extrude(CAP_HEIGHT)

    # Blind bore: extends past skirt to ensure disc clears the neck top
    bore_depth = CAP_SKIRT + 0.0005
    bore = cq.Workplane("XY").circle(CAP_BORE / 2.0).extrude(bore_depth)
    cap = cap.cut(bore)

    # Knurling: raised vertical ridges on the outer surface
    n_ridges = 18
    ridge_w = 0.0012
    ridge_d = 0.0008
    for i in range(n_ridges):
        angle = 2.0 * math.pi * i / n_ridges
        r = body_r + ridge_d / 2.0
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        ridge = (
            cq.Workplane("XY")
            .center(x, y)
            .rect(ridge_w, ridge_d)
            .extrude(CAP_SKIRT + CAP_TOP_THICK * 0.4)
        )
        cap = cap.union(ridge)

    return cap


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hexagonal_tea_tin_screw_cap")

    brushed_metal = model.material("brushed_metal", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.42, 0.33, 0.24, 1.0))
    gasket_rubber = model.material("gasket_rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    # ---- body (root): hexagonal prism with integrated top + threaded neck ----
    body = model.part("body")

    body.visual(
        mesh_from_cadquery(_hex_body(), "tin_body"),
        material=brushed_metal,
        name="tin_body",
    )

    # Neck with base flange; origin shifted down so flange embeds into plate
    body.visual(
        mesh_from_cadquery(_neck_tube(), "neck"),
        material=brushed_metal,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - NECK_BASE_H)),
        name="neck",
    )

    # Thread ridges: repeated sub-parts via for-i-in-range loop,
    # shared geometry helper, regular spacing, uniform visual policy.
    ridge_pitch = (NECK_HEIGHT - THREAD_WIDTH - 0.003) / max(THREAD_COUNT - 1, 1)
    for i in range(THREAD_COUNT):
        z = BODY_HEIGHT + 0.002 + i * ridge_pitch
        body.visual(
            mesh_from_cadquery(_thread_ridge_ring(), f"thread_ridge_{i}"),
            material=brushed_metal,
            origin=Origin(xyz=(0.0, 0.0, z)),
            name=f"thread_ridge_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((ACROSS_FLATS, ACROSS_FLATS, BODY_HEIGHT + NECK_HEIGHT)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT + NECK_HEIGHT) / 2.0)),
    )

    # ---- cap_carrier: kinematic intermediary for prismatic lift ----
    carrier = model.part("cap_carrier")
    carrier.visual(
        mesh_from_cadquery(_carrier_gasket(), "carrier_gasket"),
        material=gasket_rubber,
        # Gasket sits below the carrier frame so at rest it engages
        # the cap skirt bore region.
        origin=Origin(xyz=(0.0, 0.0, -GASKET_THICK)),
        name="carrier_gasket",
    )

    # ---- cap: knurled round screw cap ----
    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_screw_cap_body(), "screw_cap"),
        material=dark_bronze,
        # Shift so the skirt extends below the part frame origin and the
        # top disc sits above it. At rest the origin is at the neck top.
        origin=Origin(xyz=(0.0, 0.0, -CAP_SKIRT)),
        name="screw_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_OD / 2.0, length=CAP_HEIGHT),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP_THICK - CAP_SKIRT) / 2.0)),
    )

    # ---- Articulations ----
    neck_top_z = BODY_HEIGHT + NECK_HEIGHT

    # Prismatic: body -> carrier (cap lifts straight up off the neck)
    model.articulation(
        "body_to_carrier",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, neck_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=10.0,
            velocity=0.2,
        ),
    )

    # Continuous: carrier -> cap (screw / unscrew rotation)
    model.articulation(
        "carrier_to_cap",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("cap_carrier")
    cap = object_model.get_part("cap")
    lift = object_model.get_articulation("body_to_carrier")
    spin = object_model.get_articulation("carrier_to_cap")

    # --- Body footprint unchanged from parent (hex prism) ---
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is taller than wide (tall hex prism)",
        body_ext[2] > max(body_ext[0], body_ext[1]) + 0.03,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "body across-flats footprint near 0.07 m",
        0.06 < max(body_ext[0], body_ext[1]) < 0.085,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "cross-section is hexagonal, not square/round",
        abs(body_ext[0] - body_ext[1]) > 0.003,
        details=f"x={body_ext[0]:.4f}, y={body_ext[1]:.4f}",
    )

    # --- Cap is round (not hexagonal) ---
    cap_ext = _ext(ctx.part_world_aabb(cap))
    ctx.check(
        "cap XY extents nearly equal (round, not hex)",
        abs(cap_ext[0] - cap_ext[1]) < 0.003,
        details=f"cap x={cap_ext[0]:.4f}, y={cap_ext[1]:.4f}",
    )

    # --- Cap sits concentric over the neck region at rest ---
    ctx.expect_overlap(
        cap, body,
        axes="xy",
        min_overlap=0.020,
        name="cap sits concentric over neck region at rest",
    )

    # --- Cap skirt wraps around the neck (Z overlap with body) ---
    neck_top_z = BODY_HEIGHT + NECK_HEIGHT
    ctx.expect_overlap(
        cap, body,
        axes="z",
        min_overlap=0.008,
        name="cap skirt wraps around neck height at rest",
    )

    # Allow carrier gasket to contact/overlap cap bore wall (press-fit seal)
    ctx.allow_overlap(
        carrier, cap,
        elem_a="carrier_gasket",
        elem_b="screw_cap",
        reason="Gasket press-fits inside the cap skirt bore as a sealing element.",
    )
    ctx.expect_contact(
        carrier, cap,
        elem_a="carrier_gasket",
        elem_b="screw_cap",
        name="carrier gasket contacts cap bore wall at rest",
    )

    # Allow thread ridges to embed into the neck wall (surface features)
    for i in range(THREAD_COUNT):
        ctx.allow_overlap(
            body, body,
            elem_a=f"thread_ridge_{i}",
            elem_b="neck",
            reason=f"Thread ridge {i} is a raised feature on the neck outer wall.",
        )

    # Allow neck base flange to embed into body top plate
    ctx.allow_overlap(
        body, body,
        elem_a="neck",
        elem_b="tin_body",
        reason="Neck base flange is seated into the body top plate for structural mounting.",
    )
    ctx.expect_overlap(
        body, body,
        axes="z",
        elem_a="neck",
        elem_b="tin_body",
        min_overlap=0.001,
        name="neck base flange overlaps body top plate in Z",
    )

    # --- Prismatic lift: cap rises straight up off the neck ---
    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({lift: LIFT_TRAVEL}):
        lifted_pos = ctx.part_world_position(cap)
        lifted_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "cap lifts straight up off the neck (prismatic +Z)",
        lifted_pos[2] > rest_pos[2] + 0.020
        and abs(lifted_pos[0] - rest_pos[0]) < 1e-5
        and abs(lifted_pos[1] - rest_pos[1]) < 1e-5,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )
    ctx.check(
        "lifted cap clears the neck top",
        lifted_aabb[0][2] > neck_top_z - 0.002,
        details=f"cap bottom={lifted_aabb[0][2]:.4f}, neck top={neck_top_z:.4f}",
    )

    # --- Continuous rotation: cap spins without translating ---
    rest_pos_spin = ctx.part_world_position(cap)
    with ctx.pose({spin: 1.5708}):
        spun_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap spins in place without translating (continuous)",
        abs(spun_pos[0] - rest_pos_spin[0]) < 1e-5
        and abs(spun_pos[1] - rest_pos_spin[1]) < 1e-5
        and abs(spun_pos[2] - rest_pos_spin[2]) < 1e-5,
        details=f"rest={rest_pos_spin}, spun={spun_pos}",
    )

    # --- Non-fixed joints exist ---
    ctx.check(
        "model has non-fixed joints (prismatic + continuous)",
        lift.articulation_type != ArticulationType.FIXED
        and spin.articulation_type != ArticulationType.FIXED,
    )

    return ctx.report()


object_model = build_object_model()
