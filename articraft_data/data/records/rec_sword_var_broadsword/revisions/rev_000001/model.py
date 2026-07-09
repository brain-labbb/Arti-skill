from __future__ import annotations

"""Roman gladius-style short sword with a broad triangular broadsword blade,
sheathed in an ornate scabbard.

Reference image: picture/Military/sword/001.png

Layout: the sheathed sword lies horizontally along +X, resting on the brass
throat fitting and chape (their flat undersides sit at z=0).  The chape ball
tip is near x=0, the scabbard mouth is at x=0.50, and the hilt extends to
about x=0.744 so the sheathed assembly is ~0.75 m long.

The blade is a broad flat triangle: wide (~0.076 m) at the guard and tapering
steadily to a point.  The hollow scabbard cavity, body, and throat fitting are
sized to the wider profile so the blade still sheathes and slides out.

Articulation:
- `sword_draw`     PRISMATIC along +X, 0 -> 0.50 m (fully sheathed -> drawn).
- `front_ring_pivot` REVOLUTE about the +Y mounting-pin axis, -60..+60 deg.
- `rear_ring_pivot`  REVOLUTE about the -Y mounting-pin axis, -60..+60 deg.

The scabbard is genuinely hollow: a tapering blade cavity is boolean-cut
through the burl-wood body, the brass chape and the brass throat band, so the
blade visually nests inside it and slides out through the open mouth.
"""

import math

import cadquery as cq
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
ZC = 0.01475          # scabbard centerline height (throat half-thickness)
MOUTH_X = 0.50        # scabbard mouth plane

# wood body outer cross sections (full width, full thickness) along x
# BODY_W1 widened to accommodate the broad blade cavity at the mouth end.
BODY_X0, BODY_W0, BODY_T0 = 0.085, 0.042, 0.0215
BODY_X1, BODY_W1, BODY_T1 = 0.500, 0.094, 0.0255

# chape outer cross sections (widened slightly for the broader cavity)
CHAPE_SECS = [
    (0.016, 0.018, 0.009),
    (0.055, 0.044, 0.020),
    (0.095, 0.060, 0.0295),
]

# blade cavity cross sections (cut from chape + wood + open through the mouth)
# Sized for the broad triangular broadsword blade (0.076 m wide at guard).
CAV_SECS = [
    (0.045, 0.016, 0.009),
    (0.115, 0.048, 0.0105),
    (0.300, 0.066, 0.0105),
    (0.505, 0.082, 0.0105),
]

# throat band (widened for the broad blade to pass through)
THROAT_X0, THROAT_X1 = 0.43, 0.50
THROAT_W, THROAT_T = 0.100, 0.0295
THROAT_HOLE_W, THROAT_HOLE_T = 0.086, 0.025

# blade (sword-local x: base near 0, tip at -0.45)
# Broad flat triangular broadsword blade: wide at the guard, tapering steadily
# to the point.
BLADE_LEN = 0.455           # from local +0.005 to -0.45
BLADE_HALF_W_BASE = 0.038   # 0.076 m full width at the guard
BLADE_TIP_X = -0.45
BLADE_THICK = 0.006

# suspension rings
RING_R = 0.0105      # torus major radius
RING_TUBE = 0.0022   # torus tube radius
PIN_R = 0.0022
# Ring hangs on its pin: the torus inner-top surface seats 0.4 mm INTO the pin
# top so the hanging contact is a real captured support (declared via
# expect_contact below), preserved at every swing angle because the pin is
# axisymmetric about the joint axis.
RING_HANG = RING_R - RING_TUBE - PIN_R + 0.0004   # 0.0065 ring-center drop
PIN_Z = 0.02275
FRONT_PIN = (0.49, 0.038, PIN_Z)    # +Y side, near the mouth
REAR_PIN = (0.445, -0.038, PIN_Z)   # -Y side, near the body
RING_LIMIT = math.radians(60.0)

SWING_AXIS_Y = (0.0, 1.0, 0.0)


def _lerp_sections(secs: list[tuple[float, float, float]], x: float) -> tuple[float, float]:
    """Linear width/thickness interpolation between loft sections."""
    for (xa, wa, ta), (xb, wb, tb) in zip(secs, secs[1:]):
        if xa <= x <= xb:
            f = (x - xa) / (xb - xa)
            return wa + (wb - wa) * f, ta + (tb - ta) * f
    raise ValueError(f"x={x} outside section range")


def _loft(secs: list[tuple[float, float, float]]) -> cq.Workplane:
    """Ruled loft of centered rectangles in YZ planes along +X at height ZC."""
    wp = cq.Workplane("YZ", origin=(secs[0][0], 0.0, ZC)).rect(secs[0][1], secs[0][2])
    prev_x = secs[0][0]
    for x, w, t in secs[1:]:
        wp = wp.workplane(offset=x - prev_x).rect(w, t)
        prev_x = x
    return wp.loft(ruled=True)


def _blade_solid() -> cq.Workplane:
    """Broad flat triangular broadsword blade, sword-local frame.

    Wide at the guard (full width ~0.076 m) and tapering steadily to the point.
    """
    pts = [
        (0.005, BLADE_HALF_W_BASE),
        (BLADE_TIP_X, 0.0),
        (0.005, -BLADE_HALF_W_BASE),
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(BLADE_THICK / 2.0, both=True)


def _blade_spine_solid() -> cq.Workplane:
    """Central ridge spine running along the blade, also triangular."""
    spine_hw = 0.008  # half-width at base
    spine_t = 0.0015  # proud of blade surface
    pts = [
        (0.005, spine_hw),
        (BLADE_TIP_X + 0.02, 0.0),
        (0.005, -spine_hw),
    ]
    return (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(BLADE_THICK / 2.0 + spine_t, both=True)
    )


def _ring_solid() -> cq.Shape:
    """Suspension ring torus with its hole axis along Y."""
    return cq.Solid.makeTorus(RING_R, RING_TUBE, cq.Vector(0, 0, 0), cq.Vector(0, 1, 0))


def _pommel_solid() -> cq.Shape:
    """Flattened amber pommel: oblate ellipsoid, 48 mm wide x 29 mm tall."""
    sphere = cq.Workplane().sphere(1.0).val()
    mat = cq.Matrix([
        [0.024, 0.0, 0.0, 0.0],
        [0.0, 0.024, 0.0, 0.0],
        [0.0, 0.0, 0.0145, 0.0],
    ])
    return sphere.transformGeometry(mat)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sheathed_roman_gladius")

    steel = model.material("steel", rgba=(0.62, 0.63, 0.66, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.46, 0.48, 0.53, 1.0))
    brass = model.material("brass", rgba=(0.71, 0.54, 0.20, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.52, 0.38, 0.13, 1.0))
    amber = model.material("amber_wood", rgba=(0.80, 0.42, 0.10, 1.0))
    burl = model.material("burl_wood", rgba=(0.42, 0.22, 0.12, 1.0))
    tan = model.material("tan_strap", rgba=(0.79, 0.63, 0.44, 1.0))
    gold = model.material("gold", rgba=(0.87, 0.68, 0.24, 1.0))

    # ------------------------------------------------------------- scabbard
    scabbard = model.part("scabbard")

    cavity = _loft(CAV_SECS)

    body = _loft([(BODY_X0, BODY_W0, BODY_T0), (BODY_X1, BODY_W1, BODY_T1)]).cut(cavity)
    scabbard.visual(mesh_from_cadquery(body, "scabbard_body"), material=burl, name="body")

    chape = _loft(CHAPE_SECS).cut(cavity)
    scabbard.visual(mesh_from_cadquery(chape, "scabbard_chape"), material=brass, name="chape")
    scabbard.visual(
        Sphere(0.009), origin=Origin(xyz=(0.009, 0.0, ZC)), material=brass, name="chape_ball"
    )
    # engraved ridge bands wrapping the chape (clear of the cavity, x < 0.045)
    for i, bx in enumerate((0.030, 0.042)):
        w, t = _lerp_sections(CHAPE_SECS, bx)
        scabbard.visual(
            Box((0.004, w + 0.0028, t + 0.0028)),
            origin=Origin(xyz=(bx, 0.0, ZC)),
            material=brass_dark,
            name=f"chape_ridge_{i}",
        )

    throat = (
        cq.Workplane("YZ", origin=((THROAT_X0 + THROAT_X1) / 2.0, 0.0, ZC))
        .rect(THROAT_W, THROAT_T)
        .rect(THROAT_HOLE_W, THROAT_HOLE_T)
        .extrude((THROAT_X1 - THROAT_X0) / 2.0, both=True)
    )
    scabbard.visual(mesh_from_cadquery(throat, "scabbard_throat"), material=brass, name="throat")
    # embossed relief panels on the throat top face
    for i, px in enumerate((0.448, 0.480)):
        scabbard.visual(
            Box((0.020, 0.044, 0.0016)),
            origin=Origin(xyz=(px, 0.0, ZC + THROAT_T / 2.0 + 0.0003)),
            material=brass_dark,
            name=f"throat_relief_{i}",
        )

    # tan straps crossing in two X patterns (top and bottom broad faces)
    for xi, xc in enumerate((0.19, 0.33)):
        bw, bt = _lerp_sections(
            [(BODY_X0, BODY_W0, BODY_T0), (BODY_X1, BODY_W1, BODY_T1)], xc
        )
        for face, zs in (("top", 1.0), ("bottom", -1.0)):
            for di, yaw in enumerate((0.42, -0.42)):
                scabbard.visual(
                    Box((0.075, 0.012, 0.0024)),
                    origin=Origin(
                        xyz=(xc, 0.0, ZC + zs * (bt / 2.0 + 0.0002)),
                        rpy=(0.0, 0.0, yaw),
                    ),
                    material=tan,
                    name=f"strap_x{xi}_{face}_{di}",
                )

    # suspension-ring mounting lugs: flange + pin + retaining head on each side
    for tag, (px, py, pz) in (("front", FRONT_PIN), ("rear", REAR_PIN)):
        sy = 1.0 if py > 0 else -1.0
        scabbard.visual(
            Cylinder(radius=0.005, length=0.003),
            origin=Origin(xyz=(px, sy * 0.032, pz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass_dark,
            name=f"{tag}_ring_flange",
        )
        scabbard.visual(
            Cylinder(radius=PIN_R, length=0.0155),
            origin=Origin(xyz=(px, sy * 0.03675, pz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"{tag}_ring_pin",
        )
        scabbard.visual(
            Cylinder(radius=0.0048, length=0.0035),
            origin=Origin(xyz=(px, sy * 0.04625, pz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass_dark,
            name=f"{tag}_ring_head",
        )

    # ---------------------------------------------------------------- sword
    sword = model.part("sword")

    sword.visual(mesh_from_cadquery(_blade_solid(), "blade"), material=steel, name="blade")
    sword.visual(
        mesh_from_cadquery(_blade_spine_solid(), "blade_spine"),
        material=steel_dark,
        name="blade_spine",
    )
    sword.visual(
        Box((0.050, 0.088, 0.028)),
        origin=Origin(xyz=(0.0255, 0.0, 0.0)),
        material=brass,
        name="guard",
    )
    for i, gx in enumerate((0.014, 0.037)):
        sword.visual(
            Box((0.016, 0.068, 0.0016)),
            origin=Origin(xyz=(gx, 0.0, 0.0143)),
            material=brass_dark,
            name=f"guard_relief_{i}",
        )
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
    # twisted gold spiral collar: stack of offset beads threaded on the core
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
    sword.visual(
        mesh_from_cadquery(_pommel_solid(), "pommel"),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=amber,
        name="pommel",
    )
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

    model.articulation(
        "sword_draw",
        ArticulationType.PRISMATIC,
        parent=scabbard,
        child=sword,
        origin=Origin(xyz=(MOUTH_X, 0.0, ZC)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=0.0, upper=0.50),
    )

    # ------------------------------------------------------ suspension rings
    for tag, pin in (("front", FRONT_PIN), ("rear", REAR_PIN)):
        ring = model.part(f"{tag}_suspension_ring")
        ring.visual(
            mesh_from_cadquery(_ring_solid(), f"suspension_ring_{tag}"),
            origin=Origin(xyz=(0.0, 0.0, -RING_HANG)),
            material=brass,
            name="ring",
        )
        model.articulation(
            f"{tag}_ring_pivot",
            ArticulationType.REVOLUTE,
            parent=scabbard,
            child=ring,
            origin=Origin(xyz=pin),
            axis=SWING_AXIS_Y,
            motion_limits=MotionLimits(
                effort=2.0, velocity=3.0, lower=-RING_LIMIT, upper=RING_LIMIT
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    scabbard = object_model.get_part("scabbard")
    sword = object_model.get_part("sword")
    front_ring = object_model.get_part("front_suspension_ring")
    rear_ring = object_model.get_part("rear_suspension_ring")
    draw = object_model.get_articulation("sword_draw")
    front_pivot = object_model.get_articulation("front_ring_pivot")
    rear_pivot = object_model.get_articulation("rear_ring_pivot")

    # --- joint plan: types, axes, ranges
    ctx.check(
        "sword slides on a prismatic joint along the scabbard long axis",
        draw.articulation_type == ArticulationType.PRISMATIC and draw.axis == (1.0, 0.0, 0.0),
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
            "perpendicular to scabbard length), ~+/-60 deg",
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
        "scabbard rests on the ground (brass fittings at z~0)",
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
        and 0.71 <= max(scab_aabb[1][0], sword_aabb[1][0]) - min(scab_aabb[0][0], sword_aabb[0][0]) <= 0.78,
        details=f"x span={(scab_aabb, sword_aabb)}",
    )
    throat_aabb = ctx.part_element_world_aabb(scabbard, elem="throat")
    ctx.check(
        "scabbard mouth fitting is wide enough for the broad blade (~0.09-0.11 m)",
        throat_aabb is not None and 0.088 <= throat_aabb[1][1] - throat_aabb[0][1] <= 0.115,
        details=f"throat aabb={throat_aabb}",
    )

    # --- blade dimensions and sheathed nesting
    blade_aabb = ctx.part_element_world_aabb(sword, elem="blade")
    ctx.check(
        "blade is ~0.45 m long",
        blade_aabb is not None and 0.43 <= blade_aabb[1][0] - blade_aabb[0][0] <= 0.47,
        details=f"blade aabb={blade_aabb}",
    )
    ctx.check(
        "broad blade is ~0.076 m wide at the guard end",
        blade_aabb is not None and 0.068 <= blade_aabb[1][1] - blade_aabb[0][1] <= 0.085,
        details=f"blade aabb={blade_aabb}",
    )
    ctx.check(
        "blade tapers to a point (narrow at tip end)",
        blade_aabb is not None
        and (blade_aabb[1][1] - blade_aabb[0][1]) > 0.06
        and blade_aabb[0][0] < 0.12,
        details=f"blade aabb={blade_aabb}",
    )
    ctx.check(
        "sheathed blade is hidden inside the scabbard (tip near the chape)",
        blade_aabb is not None and blade_aabb[1][0] <= 0.512 and blade_aabb[0][0] >= 0.03,
        details=f"blade aabb={blade_aabb}",
    )
    ctx.expect_within(
        sword,
        scabbard,
        axes="yz",
        inner_elem="blade",
        outer_elem="body",
        margin=0.0005,
        name="sheathed blade nests inside the hollow wood body cross-section",
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
            name="half-drawn blade stays centered in the scabbard cavity",
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
        bead_aabb is not None and 0.57 <= 0.5 * (bead_aabb[0][0] + bead_aabb[1][0]) <= 0.63,
        details=f"bead aabb={bead_aabb}",
    )
    finial_aabb = ctx.part_element_world_aabb(sword, elem="finial_ball")
    ctx.check(
        "brass finial ball caps the pommel",
        finial_aabb is not None and finial_aabb[1][0] >= 0.73,
        details=f"finial aabb={finial_aabb}",
    )

    # --- suspension rings: captured on their pins and actually swinging
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
        # the hanging ring center is offset RING_HANG below the pin (off-axis),
        # so a positive swing visibly shifts the ring toward -X and raises its
        # top edge (AABB bounds are conservative under rotation, so compare the
        # bounds that strictly grow: xmin decreases, zmax increases).
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
