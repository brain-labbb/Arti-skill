from __future__ import annotations

"""M84-style stun (flashbang) grenade with single-pin bail-ring fuze variant.

Articraft brief:
- Object: M84 stun grenade standing upright, ~0.131 m tall, body tube d=0.048,
  chunky hex end caps ~0.056 across corners.
- Root/support: `body` (root) = bottom hex cap + perforated olive shell + tan
  charge tube + brown mid band + red insert + top hex cap with white stencil +
  silver fuze (collar, cylinder, lever lugs, bail lugs). Grounded at z=0.
- Articulations:
  1. lever_pivot   REVOLUTE  body -> safety_lever, pivot (0.010, 0, 0.128),
     axis (0,-1,0), 0..1.57 rad: positive q swings the folded lever outward/up.
  2. safety_pin_slide  PRISMATIC body -> safety_pin, axis +Y, 0..0.02 m.
  3. bail_swivel  REVOLUTE body -> bail_ring, pivot (0, 0, 0.122) above the
     collar, axis (-1,0,0), 0..2.8 rad: bail swings from -Y rest upward
     through +Z to +Y.
- Visible geometry: 3 rows x 5 boolean vent holes (d 0.012) through the green
  shell revealing the tan charge tube; one lower hole has a red insert; brown
  band wraps the middle row (holes cut); '01-0009' stencil dashes on the top
  hex cap; weathered-silver fuze; single triangular bent-wire bail ring on a
  revolute swivel above the collar.
- Intentional overlaps: lever pivot pin in body lugs, pin shaft seated through
  fuze bore, bail pivot pin through the fuze body swivel mount (allowances).
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
BODY_R_OUT = 0.024
BODY_R_IN = 0.0195
TUBE_R = 0.0193

HEX_R = 0.028  # circumradius; across-flats 0.0485, across-corners 0.056
CAP_BOT_Z0, CAP_BOT_Z1 = 0.0, 0.020
SHELL_Z0, SHELL_Z1 = 0.019, 0.091  # embeds 1 mm into each cap
CAP_TOP_Z0, CAP_TOP_Z1 = 0.090, 0.112

HOLE_R = 0.006
ROW_Z = (0.0325, 0.055, 0.0775)
HOLES_PER_ROW = 5

BAND_R_IN, BAND_R_OUT = 0.0238, 0.0252
BAND_Z0, BAND_Z1 = 0.046, 0.064

FUZE_COLLAR_R, FUZE_COLLAR_Z0, FUZE_COLLAR_Z1 = 0.013, 0.112, 0.1155
FUZE_BODY_R, FUZE_BODY_Z0, FUZE_BODY_Z1 = 0.0115, 0.1155, 0.1265

LEVER_PIVOT = (0.010, 0.0, 0.128)

PIN_SHAFT_R = 0.0016
PIN_TRAVEL = 0.02
PIN_Z = 0.1215
PIN_MOUNT_R = 0.0135  # pin frame offset from the body axis

BAIL_PIVOT_Z = 0.122  # above the collar (collar ends at 0.1155)
BAIL_WIRE_R = 0.0018
BAIL_PIVOT_PIN_R = 0.0014
BAIL_PIVOT_PIN_LEN = 0.028

# ---------------------------------------------------------------- materials
OLIVE = Material(name="olive_steel", rgba=(0.33, 0.36, 0.20, 1.0))
OLIVE_DARK = Material(name="olive_cap", rgba=(0.29, 0.32, 0.18, 1.0))
TAN = Material(name="charge_tan", rgba=(0.76, 0.65, 0.48, 1.0))
BROWN = Material(name="band_brown", rgba=(0.35, 0.24, 0.15, 1.0))
RED = Material(name="insert_red", rgba=(0.62, 0.18, 0.12, 1.0))
SILVER = Material(name="fuze_silver", rgba=(0.62, 0.63, 0.65, 1.0))
STEEL = Material(name="stainless", rgba=(0.72, 0.73, 0.75, 1.0))
LEVER_GREEN = Material(name="lever_green", rgba=(0.15, 0.18, 0.11, 1.0))
WHITE = Material(name="stencil_white", rgba=(0.92, 0.92, 0.90, 1.0))


# --------------------------------------------------------- geometry helpers
def _hex_prism(height: float) -> cq.Workplane:
    """Hex prism, flats facing +X/-X (vertices at 30 + 60k degrees)."""
    pts = [
        (HEX_R * math.cos(math.radians(30 + 60 * k)),
         HEX_R * math.sin(math.radians(30 + 60 * k)))
        for k in range(6)
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(height)


def _perforated_shell() -> cq.Workplane:
    """Annular olive shell with 3 rows x 5 radial vent holes (local z from 0)."""
    h = SHELL_Z1 - SHELL_Z0
    shell = cq.Workplane("XY").circle(BODY_R_OUT).circle(BODY_R_IN).extrude(h)
    for row_z in ROW_Z:
        zl = row_z - SHELL_Z0
        for k in range(HOLES_PER_ROW):
            a = 2.0 * math.pi * k / HOLES_PER_ROW
            cutter = cq.Solid.makeCylinder(
                HOLE_R,
                0.035,
                cq.Vector(0.0, 0.0, zl),
                cq.Vector(math.cos(a), math.sin(a), 0.0),
            )
            shell = shell.cut(cutter)
    return shell


def _mid_band() -> cq.Workplane:
    """Brown wrap band over the middle hole row (local z from 0)."""
    h = BAND_Z1 - BAND_Z0
    band = cq.Workplane("XY").circle(BAND_R_OUT).circle(BAND_R_IN).extrude(h)
    zl = ROW_Z[1] - BAND_Z0
    for k in range(HOLES_PER_ROW):
        a = 2.0 * math.pi * k / HOLES_PER_ROW
        cutter = cq.Solid.makeCylinder(
            HOLE_R + 0.0003,
            0.035,
            cq.Vector(0.0, 0.0, zl),
            cq.Vector(math.cos(a), math.sin(a), 0.0),
        )
        band = band.cut(cutter)
    return band


def _bail_ring_mesh(name: str):
    """Triangular bent-wire bail loop in the local YZ plane, extending in -Y.

    Three straight wire segments form a triangle with the near vertex at the
    swivel pivot and the 0.035 m base at the far end.  The closed spline
    naturally rounds the corners into plausible wire bends.
    """
    verts = [
        (0.0, 0.0, 0.0),          # at pivot (overlaps the pivot pin)
        (0.0, -0.033, -0.0175),   # far lower
        (0.0, -0.033, 0.0175),    # far upper
    ]
    pts: list[tuple[float, float, float]] = []
    n_per_side = 6
    for i in range(3):
        v_a = verts[i]
        v_b = verts[(i + 1) % 3]
        for j in range(n_per_side):
            t = j / n_per_side
            pts.append((
                v_a[0] + t * (v_b[0] - v_a[0]),
                v_a[1] + t * (v_b[1] - v_a[1]),
                v_a[2] + t * (v_b[2] - v_a[2]),
            ))
    geo = tube_from_spline_points(
        pts,
        radius=BAIL_WIRE_R,
        closed_spline=True,
        samples_per_segment=4,
        radial_segments=10,
    )
    return mesh_from_geometry(geo, name)


# ------------------------------------------------------------- build model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="m84_stun_grenade")

    # ============================================================ body (root)
    body = model.part("body")

    # -- bottom hex cap --
    body.visual(
        mesh_from_cadquery(_hex_prism(CAP_BOT_Z1 - CAP_BOT_Z0), "hex_cap_bottom"),
        origin=Origin(xyz=(0.0, 0.0, CAP_BOT_Z0)),
        material=OLIVE_DARK,
        name="hex_cap_bottom",
    )

    # -- perforated olive shell --
    body.visual(
        mesh_from_cadquery(_perforated_shell(), "perforated_shell", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, SHELL_Z0)),
        material=OLIVE,
        name="perforated_shell",
    )

    # -- tan charge tube (visible through vent holes) --
    body.visual(
        Cylinder(radius=TUBE_R, length=SHELL_Z1 - SHELL_Z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (SHELL_Z0 + SHELL_Z1))),
        material=TAN,
        name="charge_tube",
    )

    # -- brown mid band --
    body.visual(
        mesh_from_cadquery(_mid_band(), "mid_band", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, BAND_Z0)),
        material=BROWN,
        name="mid_band",
    )

    # -- reddish insert visible inside one lower vent hole (+X column) --
    body.visual(
        Cylinder(radius=0.0052, length=0.0038),
        origin=Origin(xyz=(0.0205, 0.0, ROW_Z[0]), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=RED,
        name="flash_insert",
    )

    # -- top hex cap --
    body.visual(
        mesh_from_cadquery(_hex_prism(CAP_TOP_Z1 - CAP_TOP_Z0), "hex_cap_top"),
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z0)),
        material=OLIVE_DARK,
        name="hex_cap_top",
    )

    # -- '01-0009' white stencil dashes on the -X flat of the top hex cap --
    apothem = HEX_R * math.cos(math.radians(30.0))
    for i in range(7):
        y = 0.0085 - i * (0.017 / 6.0)
        body.visual(
            Box((0.0008, 0.0016, 0.005)),
            origin=Origin(xyz=(-(apothem + 0.0002), y, 0.103)),
            material=WHITE,
            name=f"stencil_char_{i}",
        )

    # -- weathered silver fuze stack --
    body.visual(
        Cylinder(radius=FUZE_COLLAR_R, length=FUZE_COLLAR_Z1 - FUZE_COLLAR_Z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (FUZE_COLLAR_Z0 + FUZE_COLLAR_Z1))),
        material=SILVER,
        name="fuze_collar",
    )
    body.visual(
        Cylinder(radius=FUZE_BODY_R, length=FUZE_BODY_Z1 - FUZE_BODY_Z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (FUZE_BODY_Z0 + FUZE_BODY_Z1))),
        material=SILVER,
        name="fuze_body",
    )

    # -- lever pivot lug ears (±Y flanking the lever plate) --
    for i, sy in enumerate((1.0, -1.0)):
        body.visual(
            Box((0.006, 0.0025, 0.006)),
            origin=Origin(xyz=(LEVER_PIVOT[0], sy * 0.00775, LEVER_PIVOT[2])),
            material=SILVER,
            name=f"lever_lug_{i}",
        )

    # -- bail pivot lug ears (±X flanking the bail swivel above the collar) --
    for i, sx in enumerate((1.0, -1.0)):
        body.visual(
            Box((0.003, 0.006, 0.005)),
            origin=Origin(xyz=(sx * 0.013, 0.0, BAIL_PIVOT_Z)),
            material=SILVER,
            name=f"bail_lug_{i}",
        )

    # ========================================================= safety lever
    # Local frame at the pivot; folded plate runs over the fuze top (+X) then
    # down the +X side of the body to mid-height.
    lever = model.part("safety_lever")
    lever.visual(
        Cylinder(radius=0.0016, length=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="pivot_pin",
    )
    lever.visual(
        Box((0.019, 0.012, 0.002)),
        origin=Origin(xyz=(0.0075, 0.0, 0.0)),
        material=LEVER_GREEN,
        name="top_plate",
    )
    lever.visual(
        Box((0.002, 0.012, 0.081)),
        origin=Origin(xyz=(0.017, 0.0, -0.0395)),
        material=LEVER_GREEN,
        name="side_strap",
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=LEVER_PIVOT),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=6.0, lower=0.0, upper=1.57),
    )

    # ========================================================= safety pin
    # Single pull pin on the +Y side of the fuze body, prismatic extraction.
    pin = model.part("safety_pin")
    pin.visual(
        Cylinder(radius=PIN_SHAFT_R, length=0.029),
        origin=Origin(xyz=(0.0, -0.0145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="shaft",
    )
    pin.visual(
        Cylinder(radius=0.0033, length=0.003),
        origin=Origin(xyz=(0.0, 0.0005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="eye_head",
    )
    model.articulation(
        "safety_pin_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pin,
        origin=Origin(xyz=(0.0, PIN_MOUNT_R, PIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=PIN_TRAVEL),
    )

    # ========================================================= bail ring
    # Triangular bent-wire bail on a revolute swivel above the fuze collar.
    # Pivot axis along X between the bail lugs; bail extends in -Y at rest.
    # Positive q (about -X) swings the bail from -Y upward through +Z.
    bail = model.part("bail_ring")
    bail.visual(
        _bail_ring_mesh("bail_wire"),
        origin=Origin(),
        material=STEEL,
        name="bail_wire",
    )
    bail.visual(
        Cylinder(radius=BAIL_PIVOT_PIN_R, length=BAIL_PIVOT_PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="pivot_pin",
    )
    model.articulation(
        "bail_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, BAIL_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=0.0, upper=2.8),
    )

    return model


# ------------------------------------------------------------------ tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lever = object_model.get_part("safety_lever")
    pin = object_model.get_part("safety_pin")
    bail = object_model.get_part("bail_ring")

    j_lever = object_model.get_articulation("lever_pivot")
    j_pin = object_model.get_articulation("safety_pin_slide")
    j_bail = object_model.get_articulation("bail_swivel")

    # ----------------------------------------- intentional overlaps
    # Lever pivot pin captured by the lug ears.
    for i in range(2):
        ctx.allow_overlap(
            body, lever,
            elem_a=f"lever_lug_{i}", elem_b="pivot_pin",
            reason="Hinge pin intentionally captured inside the fuze lug ear.",
        )
    # Safety pin shaft seated through the fuze block bore.
    ctx.allow_overlap(
        body, pin,
        elem_a="fuze_body", elem_b="shaft",
        reason="Safety pin shaft seated through the fuze block bore proxy.",
    )
    # Bail pivot pin passes through the fuze body swivel mount.
    ctx.allow_overlap(
        body, bail,
        elem_a="fuze_body", elem_b="pivot_pin",
        reason="Bail pivot pin passes through the fuze body swivel mount.",
    )
    for i in range(2):
        ctx.allow_overlap(
            body, bail,
            elem_a=f"bail_lug_{i}", elem_b="pivot_pin",
            reason="Bail pivot pin captured by the swivel lug ear.",
        )

    # ------------------------------------------ size and grounding
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body grounded at z=0",
        aabb is not None and abs(aabb[0][2]) < 0.002,
        details=f"aabb={aabb}",
    )
    ctx.check(
        "overall height about 0.13 m",
        aabb is not None and 0.120 <= aabb[1][2] <= 0.140,
        details=f"aabb={aabb}",
    )
    shell_aabb = ctx.part_element_world_aabb(body, elem="perforated_shell")
    ctx.check(
        "body tube about 0.05 m across",
        shell_aabb is not None
        and 0.044 <= (shell_aabb[1][0] - shell_aabb[0][0]) <= 0.052
        and 0.044 <= (shell_aabb[1][1] - shell_aabb[0][1]) <= 0.052,
        details=f"shell aabb={shell_aabb}",
    )

    # ------------------------------------------ key part layout
    insert = ctx.part_element_world_aabb(body, elem="flash_insert")
    ctx.check(
        "red insert sits in a lower +X vent hole",
        insert is not None
        and insert[0][0] > 0.017
        and abs(0.5 * (insert[0][2] + insert[1][2]) - 0.0325) < 0.004,
        details=f"insert aabb={insert}",
    )
    band = ctx.part_element_world_aabb(body, elem="mid_band")
    ctx.check(
        "brown band wraps the middle hole row",
        band is not None and band[0][2] < 0.055 < band[1][2],
        details=f"band aabb={band}",
    )
    stencil = ctx.part_element_world_aabb(body, elem="stencil_char_0")
    ctx.check(
        "stencil sits proud on the -X hex flat of the top cap",
        stencil is not None and stencil[0][0] < -0.024 and 0.090 < stencil[0][2] < 0.112,
        details=f"stencil aabb={stencil}",
    )

    # ------------------------------------------ contact / support
    ctx.expect_contact(
        lever, body, elem_a="pivot_pin", elem_b="lever_lug_0", contact_tol=1e-6,
    )
    ctx.expect_contact(
        pin, body, elem_a="shaft", elem_b="fuze_body", contact_tol=1e-6,
    )
    ctx.expect_contact(
        bail, body, elem_a="pivot_pin", elem_b="bail_lug_0", contact_tol=1e-6,
    )

    # ------------------------------------------ joint contracts
    # Lever: revolute about -Y, ~90 deg swing.
    ctx.check(
        "lever joint is revolute about -Y, ~90 deg",
        j_lever.articulation_type == ArticulationType.REVOLUTE
        and j_lever.axis[1] < -0.9
        and j_lever.motion_limits is not None
        and abs(j_lever.motion_limits.upper - 1.57) < 0.05
        and abs(j_lever.motion_limits.lower) < 1e-9,
    )

    # Safety pin: prismatic with 0.02 m travel.
    ctx.check(
        "safety pin is prismatic with 0.02 m travel",
        j_pin.articulation_type == ArticulationType.PRISMATIC
        and j_pin.motion_limits is not None
        and abs(j_pin.motion_limits.upper - PIN_TRAVEL) < 1e-6
        and abs(j_pin.motion_limits.lower) < 1e-9,
    )

    # Bail swivel: revolute, parent is body (not a pin), above the collar.
    ctx.check(
        "bail swivel is revolute, parent is body",
        j_bail.articulation_type == ArticulationType.REVOLUTE
        and j_bail.parent == "body",
    )
    ctx.check(
        "bail swivel origin is above the fuze collar",
        j_bail.origin.xyz[2] > FUZE_COLLAR_Z1,
        details=f"bail_z={j_bail.origin.xyz[2]}, collar_top={FUZE_COLLAR_Z1}",
    )
    ctx.check(
        "bail swivel has meaningful angular range (~2.8 rad)",
        j_bail.motion_limits is not None
        and abs(j_bail.motion_limits.upper - 2.8) < 0.1
        and abs(j_bail.motion_limits.lower) < 1e-9,
    )

    # Confirm exactly 4 parts: body, safety_lever, safety_pin, bail_ring.
    all_part_names = sorted(p.name for p in object_model.parts)
    ctx.check(
        "exactly four parts: body, safety_lever, safety_pin, bail_ring",
        all_part_names == ["bail_ring", "body", "safety_lever", "safety_pin"],
        details=f"parts={all_part_names}",
    )

    # ------------------------------------------ bail triangular shape
    # At rest the bail triangle lies in the YZ plane extending in -Y.
    # X extent should be tiny (wire thickness), Z extent ~0.035 m (base).
    bail_aabb = ctx.part_element_world_aabb(bail, elem="bail_wire")
    if bail_aabb is not None:
        dx = bail_aabb[1][0] - bail_aabb[0][0]
        dz = bail_aabb[1][2] - bail_aabb[0][2]
        ctx.check(
            "bail wire is thin in X (flat triangular loop in YZ plane)",
            dx < 0.010,
            details=f"dx={dx:.4f}",
        )
        ctx.check(
            "bail Z extent confirms triangular base (~0.035 m)",
            dz > 0.025,
            details=f"dz={dz:.4f}",
        )

    # ------------------------------------------ pose checks
    # Lever swings outward and up.
    lever_rest = ctx.part_world_aabb(lever)
    with ctx.pose({j_lever: 1.4}):
        lever_open = ctx.part_world_aabb(lever)
    ctx.check(
        "lever swings outward and up off the body side",
        lever_rest is not None
        and lever_open is not None
        and lever_open[0][2] > lever_rest[0][2] + 0.05
        and lever_open[1][0] > lever_rest[1][0] + 0.03,
        details=f"rest={lever_rest}, open={lever_open}",
    )

    # Safety pin extracts along +Y carrying its eye head.
    pin_rest = ctx.part_world_position(pin)
    with ctx.pose({j_pin: PIN_TRAVEL}):
        pin_out = ctx.part_world_position(pin)
    ctx.check(
        "safety pin extracts +Y from the fuze block",
        pin_rest is not None and pin_out is not None and pin_out[1] - pin_rest[1] > 0.018,
        details=f"rest={pin_rest}, out={pin_out}",
    )

    # Bail swings from rest (-Y) upward; wire AABB center rises.
    bail_wire_rest = ctx.part_element_world_aabb(bail, elem="bail_wire")
    with ctx.pose({j_bail: 1.4}):
        bail_wire_swung = ctx.part_element_world_aabb(bail, elem="bail_wire")
    if bail_wire_rest is not None and bail_wire_swung is not None:
        rest_cz = 0.5 * (bail_wire_rest[0][2] + bail_wire_rest[1][2])
        swung_cz = 0.5 * (bail_wire_swung[0][2] + bail_wire_swung[1][2])
        ctx.check(
            "bail wire swings upward (AABB center rises) from rest position",
            swung_cz > rest_cz + 0.01,
            details=f"rest_z={rest_cz:.4f}, swung_z={swung_cz:.4f}",
        )
    else:
        ctx.fail("bail wire AABB unavailable", f"rest={bail_wire_rest}, swung={bail_wire_swung}")

    return ctx.report()


object_model = build_object_model()
