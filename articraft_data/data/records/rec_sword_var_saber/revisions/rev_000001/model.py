from __future__ import annotations

"""Curved single-edged saber sheathed in an ornate scabbard.

Forked from the Roman gladius: the straight double-edged blade is replaced
by a gently curved single-edged saber blade that bows laterally along its
length.  The scabbard body, chape, and cavity all follow the same curve so
the blade nests inside and draws out prismatically.

Layout: the sheathed sword lies horizontally along +X, resting on the brass
throat fitting and chape (flat undersides at z≈0).  The chape ball tip is
near x=0, the scabbard mouth is at x=0.50, and the hilt extends to about
x=0.74 so the sheathed assembly is ~0.75 m long.

Articulation:
- `sword_draw`     PRISMATIC along +X, 0 → 0.50 m (sheathed → drawn).
- `front_ring_pivot` REVOLUTE about +Y mounting-pin axis, ±60°.
- `rear_ring_pivot`  REVOLUTE about −Y mounting-pin axis, ±60°.
"""

import math

import cadquery as cq
from cadquery.func import segment as _seg, wire as _wire, loft as _cq_loft

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- constants
ZC = 0.0155            # scabbard centreline height
MOUTH_X = 0.50         # scabbard mouth plane
BOW_MAX = 0.018        # max lateral bow at blade midpoint (18 mm)

# Blade (sword-local frame: base at +x, tip at −x)
BLADE_X_BASE = 0.005
BLADE_X_TIP = -0.45
BLADE_LEN = BLADE_X_BASE - BLADE_X_TIP        # 0.455
BLADE_THICK = 0.006
BLADE_CUT_HW = 0.026    # cutting-edge side half-width at base
BLADE_SPINE_HW = 0.014  # spine side half-width at base

# Scabbard body
BODY_X0, BODY_W0, BODY_T0 = 0.085, 0.046, 0.024
BODY_X1, BODY_W1, BODY_T1 = 0.500, 0.062, 0.028

# Chape outer sections (x, width, thickness)
CHAPE_SECS_XWT = [
    (0.016, 0.015, 0.011),
    (0.055, 0.036, 0.023),
    (0.095, 0.054, 0.033),
]

# Cavity sections (x, width, thickness) – wider than blade at each station
CAV_SECS_XWT = [
    (0.048, 0.010, 0.010),
    (0.100, 0.030, 0.011),
    (0.250, 0.044, 0.011),
    (0.505, 0.054, 0.011),
]

# Throat band
THROAT_X0, THROAT_X1 = 0.43, 0.50
THROAT_W, THROAT_T = 0.066, 0.032
THROAT_HOLE_W, THROAT_HOLE_T = 0.058, 0.014

# Suspension rings
RING_R = 0.0105
RING_TUBE = 0.0022
PIN_R = 0.0022
RING_HANG = RING_R - RING_TUBE - PIN_R + 0.0004
PIN_Z = 0.02275
FRONT_PIN_X, FRONT_PIN_SY = 0.49, 1.0
REAR_PIN_X, REAR_PIN_SY = 0.445, -1.0
PIN_BASE_Y = 0.038       # distance from centreline to pin
RING_LIMIT = math.radians(60.0)
SWING_AXIS_Y = (0.0, 1.0, 0.0)


# ---------------------------------------------------------- curve helpers
def _bow(x: float, x_base: float, x_tip: float, length: float) -> float:
    """Parabolic bow amplitude at position *x* between x_base and x_tip."""
    if x >= x_base or x <= x_tip:
        return 0.0
    t = (x_base - x) / length
    return BOW_MAX * 4.0 * t * (1.0 - t)


def _scabbard_bow(x: float) -> float:
    return _bow(x, MOUTH_X + BLADE_X_BASE, MOUTH_X + BLADE_X_TIP, BLADE_LEN)


def _sword_bow(x: float) -> float:
    return _bow(x, BLADE_X_BASE, BLADE_X_TIP, BLADE_LEN)


# ---------------------------------------------------------- geometry helpers
def _offset_loft(sections_xwt: list[tuple[float, float, float, float]]) -> cq.Workplane:
    """Loft rectangles in YZ planes at (x, y_off, ZC).

    *sections_xwt*: list of (x, y_offset, width, thickness).
    Returns a Workplane wrapping a Solid.
    """
    profiles = []
    for x, y_off, w, t in sections_xwt:
        hw, ht = w / 2.0, t / 2.0
        e1 = _seg((x, y_off - hw, ZC - ht), (x, y_off + hw, ZC - ht))
        e2 = _seg((x, y_off + hw, ZC - ht), (x, y_off + hw, ZC + ht))
        e3 = _seg((x, y_off + hw, ZC + ht), (x, y_off - hw, ZC + ht))
        e4 = _seg((x, y_off - hw, ZC + ht), (x, y_off - hw, ZC - ht))
        profiles.append(_wire(e1, e2, e3, e4))
    solid = _cq_loft(*profiles, cap=True)
    return cq.Workplane("XY").newObject([solid])


def _curved_sections(
    base_xwt: list[tuple[float, float, float]],
    bow_fn,
) -> list[tuple[float, float, float, float]]:
    """Add Y offsets from *bow_fn* to (x, width, thickness) tuples."""
    return [(x, bow_fn(x), w, t) for x, w, t in base_xwt]


def _curved_blade_solid() -> cq.Workplane:
    """Curved single-edged saber blade in sword-local frame.

    Cutting edge on the +Y (convex/outer) side, spine on the −Y side.
    """
    n = 32
    edge_pts: list[tuple[float, float]] = []
    spine_pts: list[tuple[float, float]] = []
    for i in range(n + 1):
        t = i / n
        x = BLADE_X_BASE - t * BLADE_LEN
        bow = _sword_bow(x)
        taper = max(1.0 - 0.88 * t ** 0.7, 0.0)
        edge_y = bow + BLADE_CUT_HW * taper
        spine_y = bow - BLADE_SPINE_HW * taper
        edge_pts.append((x, edge_y))
        spine_pts.append((x, spine_y))
    outline = edge_pts + list(reversed(spine_pts))
    return (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(BLADE_THICK / 2.0, both=True)
    )


def _ring_solid() -> cq.Shape:
    return cq.Solid.makeTorus(
        RING_R, RING_TUBE, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0)
    )


def _pommel_solid() -> cq.Shape:
    sphere = cq.Workplane().sphere(1.0).val()
    mat = cq.Matrix([
        [0.024, 0.0, 0.0, 0.0],
        [0.0, 0.024, 0.0, 0.0],
        [0.0, 0.0, 0.0145, 0.0],
    ])
    return sphere.transformGeometry(mat)


def _lerp_xwt(
    secs: list[tuple[float, float, float]], x: float
) -> tuple[float, float]:
    for (xa, wa, ta), (xb, wb, tb) in zip(secs, secs[1:]):
        if xa <= x <= xb:
            f = (x - xa) / (xb - xa)
            return wa + (wb - wa) * f, ta + (tb - ta) * f
    raise ValueError(f"x={x} outside section range")


# ========================================================= build
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sheathed_curved_saber")

    steel = model.material("steel", rgba=(0.62, 0.63, 0.66, 1.0))
    brass = model.material("brass", rgba=(0.71, 0.54, 0.20, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.52, 0.38, 0.13, 1.0))
    amber = model.material("amber_wood", rgba=(0.80, 0.42, 0.10, 1.0))
    burl = model.material("burl_wood", rgba=(0.42, 0.22, 0.12, 1.0))
    tan = model.material("tan_strap", rgba=(0.79, 0.63, 0.44, 1.0))
    gold = model.material("gold", rgba=(0.87, 0.68, 0.24, 1.0))

    # ------------------------------------------------------- scabbard
    scabbard = model.part("scabbard")

    # --- curved cavity cutter
    cavity = _offset_loft(_curved_sections(CAV_SECS_XWT, _scabbard_bow))

    # --- curved wood body
    n_body = 16
    body_secs: list[tuple[float, float, float, float]] = []
    for i in range(n_body + 1):
        t = i / n_body
        x = BODY_X0 + t * (BODY_X1 - BODY_X0)
        w = BODY_W0 + (BODY_W1 - BODY_W0) * t
        th = BODY_T0 + (BODY_T1 - BODY_T0) * t
        body_secs.append((x, _scabbard_bow(x), w, th))
    body = _offset_loft(body_secs).cut(cavity)
    scabbard.visual(
        mesh_from_cadquery(body, "scabbard_body"), material=burl, name="body"
    )

    # --- curved brass chape
    chape_secs = _curved_sections(CHAPE_SECS_XWT, _scabbard_bow)
    chape = _offset_loft(chape_secs).cut(cavity)
    scabbard.visual(
        mesh_from_cadquery(chape, "scabbard_chape"), material=brass, name="chape"
    )
    scabbard.visual(
        Sphere(0.009),
        origin=Origin(xyz=(0.009, _scabbard_bow(0.009), ZC)),
        material=brass,
        name="chape_ball",
    )
    for i, bx in enumerate((0.030, 0.042)):
        w, t = _lerp_xwt(CHAPE_SECS_XWT, bx)
        scabbard.visual(
            Box((0.004, w + 0.003, t + 0.003)),
            origin=Origin(xyz=(bx, _scabbard_bow(bx), ZC)),
            material=brass_dark,
            name=f"chape_ridge_{i}",
        )

    # --- curved brass throat fitting
    n_throat = 6
    throat_outer_secs: list[tuple[float, float, float, float]] = []
    throat_inner_secs: list[tuple[float, float, float, float]] = []
    for i in range(n_throat + 1):
        t = i / n_throat
        x = THROAT_X0 + t * (THROAT_X1 - THROAT_X0)
        y_off = _scabbard_bow(x)
        throat_outer_secs.append((x, y_off, THROAT_W, THROAT_T))
        throat_inner_secs.append((x, y_off, THROAT_HOLE_W, THROAT_HOLE_T))
    throat = _offset_loft(throat_outer_secs).cut(_offset_loft(throat_inner_secs))
    scabbard.visual(
        mesh_from_cadquery(throat, "scabbard_throat"), material=brass, name="throat"
    )
    for i, px in enumerate((0.448, 0.480)):
        scabbard.visual(
            Box((0.020, 0.044, 0.0016)),
            origin=Origin(xyz=(px, _scabbard_bow(px), ZC + THROAT_T / 2.0 + 0.0003)),
            material=brass_dark,
            name=f"throat_relief_{i}",
        )

    # --- tan straps (X patterns on top and bottom faces)
    for xi, xc in enumerate((0.19, 0.33)):
        f = (xc - BODY_X0) / (BODY_X1 - BODY_X0)
        bw = BODY_W0 + (BODY_W1 - BODY_W0) * f
        bt = BODY_T0 + (BODY_T1 - BODY_T0) * f
        y_off = _scabbard_bow(xc)
        for face, zs in (("top", 1.0), ("bottom", -1.0)):
            for di, yaw in enumerate((0.42, -0.42)):
                scabbard.visual(
                    Box((0.075, 0.012, 0.0024)),
                    origin=Origin(
                        xyz=(xc, y_off, ZC + zs * (bt / 2.0 + 0.0002)),
                        rpy=(0.0, 0.0, yaw),
                    ),
                    material=tan,
                    name=f"strap_x{xi}_{face}_{di}",
                )

    # --- suspension-ring mounting lugs (adjusted for curve)
    for tag, (px, sy) in (
        ("front", (FRONT_PIN_X, FRONT_PIN_SY)),
        ("rear", (REAR_PIN_X, REAR_PIN_SY)),
    ):
        bow = _scabbard_bow(px)
        scabbard.visual(
            Cylinder(radius=0.005, length=0.003),
            origin=Origin(
                xyz=(px, sy * 0.032 + bow, PIN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=brass_dark,
            name=f"{tag}_ring_flange",
        )
        scabbard.visual(
            Cylinder(radius=PIN_R, length=0.0155),
            origin=Origin(
                xyz=(px, sy * 0.03675 + bow, PIN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=brass,
            name=f"{tag}_ring_pin",
        )
        scabbard.visual(
            Cylinder(radius=0.0048, length=0.0035),
            origin=Origin(
                xyz=(px, sy * 0.04625 + bow, PIN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            material=brass_dark,
            name=f"{tag}_ring_head",
        )

    # ------------------------------------------------------- sword
    sword = model.part("sword")

    sword.visual(
        mesh_from_cadquery(_curved_blade_solid(), "blade"),
        material=steel,
        name="blade",
    )

    # Guard
    sword.visual(
        Box((0.050, 0.060, 0.028)),
        origin=Origin(xyz=(0.0255, 0.0, 0.0)),
        material=brass,
        name="guard",
    )
    for i, gx in enumerate((0.014, 0.037)):
        sword.visual(
            Box((0.016, 0.040, 0.0016)),
            origin=Origin(xyz=(gx, 0.0, 0.0143)),
            material=brass_dark,
            name=f"guard_relief_{i}",
        )

    # Grip
    sword.visual(
        Cylinder(radius=0.009, length=0.127),
        origin=Origin(xyz=(0.1085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber,
        name="grip_core",
    )
    sword.visual(
        Cylinder(radius=0.0148, length=0.029),
        origin=Origin(xyz=(0.0635, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber,
        name="grip_lower",
    )
    for i in range(4):
        ang = math.radians(30.0 + i * 120.0)
        sword.visual(
            Sphere(0.0125),
            origin=Origin(
                xyz=(0.082 + i * 0.016, 0.0012 * math.cos(ang), 0.0012 * math.sin(ang))
            ),
            material=gold,
            name=f"grip_collar_bead_{i}",
        )
    sword.visual(
        Cylinder(radius=0.0148, length=0.034),
        origin=Origin(xyz=(0.151, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber,
        name="grip_upper",
    )

    # Pommel
    sword.visual(
        mesh_from_cadquery(_pommel_solid(), "pommel"),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=amber,
        name="pommel",
    )

    # Finial
    sword.visual(
        Cylinder(radius=0.0055, length=0.021),
        origin=Origin(xyz=(0.2205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="finial_stem",
    )
    sword.visual(
        Sphere(0.0078),
        origin=Origin(xyz=(0.236, 0.0, 0.0)),
        material=brass,
        name="finial_ball",
    )

    # ------------------------------------------------------- articulations
    model.articulation(
        "sword_draw",
        ArticulationType.PRISMATIC,
        parent=scabbard,
        child=sword,
        origin=Origin(xyz=(MOUTH_X, 0.0, ZC)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=0.0, upper=0.50),
    )

    # Suspension rings (pin Y adjusted for curve offset)
    for tag, (px, sy) in (
        ("front", (FRONT_PIN_X, FRONT_PIN_SY)),
        ("rear", (REAR_PIN_X, REAR_PIN_SY)),
    ):
        ring = model.part(f"{tag}_suspension_ring")
        ring.visual(
            mesh_from_cadquery(_ring_solid(), f"suspension_ring_{tag}"),
            origin=Origin(xyz=(0.0, 0.0, -RING_HANG)),
            material=brass,
            name="ring",
        )
        pin_y = sy * PIN_BASE_Y + _scabbard_bow(px)
        model.articulation(
            f"{tag}_ring_pivot",
            ArticulationType.REVOLUTE,
            parent=scabbard,
            child=ring,
            origin=Origin(xyz=(px, pin_y, PIN_Z)),
            axis=SWING_AXIS_Y,
            motion_limits=MotionLimits(
                effort=2.0, velocity=3.0, lower=-RING_LIMIT, upper=RING_LIMIT
            ),
        )

    return model


# ========================================================= tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    scabbard = object_model.get_part("scabbard")
    sword = object_model.get_part("sword")
    front_ring = object_model.get_part("front_suspension_ring")
    rear_ring = object_model.get_part("rear_suspension_ring")
    draw = object_model.get_articulation("sword_draw")
    front_pivot = object_model.get_articulation("front_ring_pivot")
    rear_pivot = object_model.get_articulation("rear_ring_pivot")

    # --- joint plan
    ctx.check(
        "sword slides on a prismatic joint along the scabbard long axis",
        draw.articulation_type == ArticulationType.PRISMATIC
        and draw.axis == (1.0, 0.0, 0.0),
        details=f"type={draw.articulation_type}, axis={draw.axis}",
    )
    ctx.check(
        "sword draw travel is ~0.5 m",
        draw.motion_limits is not None
        and abs(draw.motion_limits.lower) < 1e-9
        and abs(draw.motion_limits.upper - 0.50) < 0.02,
        details=f"limits=({draw.motion_limits.lower}, {draw.motion_limits.upper})",
    )
    for name, pivot in (("front", front_pivot), ("rear", rear_pivot)):
        ctx.check(
            f"{name} suspension ring is revolute about the lug pin axis (Y, "
            "perpendicular to scabbard length), ~±60°",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and pivot.axis == (0.0, 1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + RING_LIMIT) < 0.05
            and abs(pivot.motion_limits.upper - RING_LIMIT) < 0.05,
            details=f"type={pivot.articulation_type}, axis={pivot.axis}",
        )

    # --- scale and grounding
    scab_aabb = ctx.part_world_aabb(scabbard)
    sword_aabb = ctx.part_world_aabb(sword)
    ctx.check(
        "scabbard rests on the ground (brass fittings at z≈0)",
        scab_aabb is not None and -0.0015 <= scab_aabb[0][2] <= 0.003,
        details=f"scabbard zmin={scab_aabb[0][2] if scab_aabb else None}",
    )
    ctx.check(
        "sword never dips below the ground plane",
        sword_aabb is not None and sword_aabb[0][2] >= -0.0015,
        details=f"sword zmin={sword_aabb[0][2] if sword_aabb else None}",
    )
    ctx.check(
        "sheathed assembly is ~0.75 m long overall",
        scab_aabb is not None
        and sword_aabb is not None
        and 0.71
        <= max(scab_aabb[1][0], sword_aabb[1][0])
        - min(scab_aabb[0][0], sword_aabb[0][0])
        <= 0.78,
        details=f"x span={(scab_aabb, sword_aabb)}",
    )
    throat_aabb = ctx.part_element_world_aabb(scabbard, elem="throat")
    ctx.check(
        "scabbard mouth fitting is ~0.06 m wide",
        throat_aabb is not None
        and 0.05 <= throat_aabb[1][1] - throat_aabb[0][1] <= 0.080,
        details=f"throat aabb={throat_aabb}",
    )

    # --- curved blade geometry
    blade_aabb = ctx.part_element_world_aabb(sword, elem="blade")
    ctx.check(
        "blade is ~0.45 m long",
        blade_aabb is not None
        and 0.43 <= blade_aabb[1][0] - blade_aabb[0][0] <= 0.47,
        details=f"blade aabb={blade_aabb}",
    )
    blade_y_center = (
        (blade_aabb[0][1] + blade_aabb[1][1]) / 2.0 if blade_aabb else 0.0
    )
    ctx.check(
        "blade is single-edged (Y centre shifted toward cutting edge, >0)",
        blade_y_center > 0.002,
        details=f"blade Y centre={blade_y_center}",
    )

    # --- curved scabbard geometry
    body_aabb = ctx.part_element_world_aabb(scabbard, elem="body")
    body_y_center = (
        (body_aabb[0][1] + body_aabb[1][1]) / 2.0 if body_aabb else 0.0
    )
    ctx.check(
        "scabbard body curves laterally to match the saber blade",
        body_y_center > 0.002,
        details=f"body Y centre={body_y_center}",
    )

    # --- sheathed nesting
    ctx.check(
        "sheathed blade is hidden inside the scabbard (tip near the chape)",
        blade_aabb is not None
        and blade_aabb[1][0] <= 0.512
        and blade_aabb[0][0] >= 0.03,
        details=f"blade aabb={blade_aabb}",
    )
    ctx.expect_within(
        sword,
        scabbard,
        axes="yz",
        inner_elem="blade",
        outer_elem="body",
        margin=0.0005,
        name="sheathed blade nests inside the curved hollow body",
    )
    ctx.expect_overlap(
        sword,
        scabbard,
        axes="x",
        elem_a="blade",
        elem_b="body",
        min_overlap=0.35,
        name="sheathed blade is inserted along nearly the full scabbard length",
    )
    ctx.expect_gap(
        sword,
        scabbard,
        axis="x",
        positive_elem="guard",
        negative_elem="throat",
        min_gap=0.0,
        max_gap=0.003,
        name="guard block seats against the throat fitting when sheathed",
    )

    # --- drawing the sword
    with ctx.pose({draw: 0.25}):
        ctx.expect_within(
            sword,
            scabbard,
            axes="yz",
            inner_elem="blade",
            outer_elem="body",
            margin=0.0005,
            name="half-drawn blade stays centered in the curved cavity",
        )
        ctx.expect_overlap(
            sword,
            scabbard,
            axes="x",
            elem_a="blade",
            elem_b="body",
            min_overlap=0.15,
            name="half-drawn blade retains insertion in the scabbard",
        )
    with ctx.pose({draw: 0.50}):
        drawn_blade = ctx.part_element_world_aabb(sword, elem="blade")
        ctx.check(
            "fully drawn blade clears the scabbard mouth completely",
            drawn_blade is not None and drawn_blade[0][0] >= MOUTH_X + 0.01,
            details=f"drawn blade aabb={drawn_blade}",
        )
        # prove the drawn blade shows its curve
        drawn_y_span = (
            drawn_blade[1][1] - drawn_blade[0][1] if drawn_blade else 0.0
        )
        ctx.check(
            "drawn blade has visible lateral curve (Y span > straight blade width)",
            drawn_y_span > 0.035,
            details=f"drawn blade Y span={drawn_y_span}",
        )

    # --- hilt composition
    pommel_aabb = ctx.part_element_world_aabb(sword, elem="pommel")
    ctx.check(
        "rounded flattened amber pommel sits at the hilt end",
        pommel_aabb is not None
        and 0.66 <= 0.5 * (pommel_aabb[0][0] + pommel_aabb[1][0]) <= 0.72
        and 0.024 <= pommel_aabb[1][2] - pommel_aabb[0][2] <= 0.034,
        details=f"pommel aabb={pommel_aabb}",
    )
    bead_aabb = ctx.part_element_world_aabb(sword, elem="grip_collar_bead_1")
    ctx.check(
        "twisted gold spiral collar occupies the middle of the grip",
        bead_aabb is not None
        and 0.57 <= 0.5 * (bead_aabb[0][0] + bead_aabb[1][0]) <= 0.63,
        details=f"bead aabb={bead_aabb}",
    )
    finial_aabb = ctx.part_element_world_aabb(sword, elem="finial_ball")
    ctx.check(
        "brass finial ball caps the pommel",
        finial_aabb is not None and finial_aabb[1][0] >= 0.73,
        details=f"finial aabb={finial_aabb}",
    )

    # --- suspension rings
    ctx.expect_contact(
        front_ring,
        scabbard,
        elem_a="ring",
        elem_b="front_ring_pin",
        contact_tol=0.0015,
        name="front ring is threaded onto its mounting pin",
    )
    ctx.expect_contact(
        rear_ring,
        scabbard,
        elem_a="ring",
        elem_b="rear_ring_pin",
        contact_tol=0.0015,
        name="rear ring is threaded onto its mounting pin",
    )
    for name, ring, pivot in (
        ("front", front_ring, front_pivot),
        ("rear", rear_ring, rear_pivot),
    ):
        closed = ctx.part_world_aabb(ring)
        with ctx.pose({pivot: 1.0}):
            swung = ctx.part_world_aabb(ring)
        ctx.check(
            f"{name} ring swings about its pin (off-axis hang proves rotation)",
            closed is not None
            and swung is not None
            and swung[0][0] < closed[0][0] - 0.003
            and swung[1][2] > closed[1][2] + 0.003,
            details=f"closed={closed}, swung={swung}",
        )

    return ctx.report()


object_model = build_object_model()
