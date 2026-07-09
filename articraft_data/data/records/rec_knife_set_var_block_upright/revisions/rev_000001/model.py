from __future__ import annotations

"""Kitchen knife block set (upright vertical variant).

An upright light-oak block (~0.13 x 0.12 x 0.24 m) standing on four dark
rubber feet, with an engraved logo panel on the front face. Six knives ride
in near-vertical top slots (independent prismatic joints along the vertical
slot axis, emitted via a for-i-in-range loop); a pair of kitchen shears
slides out of a horizontal front slot (prismatic) and opens at its central
rivet (revolute).
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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- constants
LIFT = 0.008  # block bottom raised on rubber feet

# Block dimensions (upright vertical form)
BLOCK_W = 0.130   # width  (Y)
BLOCK_D = 0.120   # depth  (X)
BLOCK_H = 0.240   # height (Z)
TOP_Z = LIFT + BLOCK_H       # top face Z
FRONT_X = BLOCK_D / 2.0      # front face X

SLOT_THICK = 0.013  # knife-slot opening in Y (must clear the bolster width 0.011)

# Six knife specs, emitted via for-i-in-range loop
KNIFE_SPECS = [
    dict(blade_len=0.175, blade_w=0.040, travel=0.18, grip_len=0.110, slot_w=0.048),
    dict(blade_len=0.155, blade_w=0.028, travel=0.16, grip_len=0.105, slot_w=0.046),
    dict(blade_len=0.145, blade_w=0.040, travel=0.15, grip_len=0.105, slot_w=0.048),
    dict(blade_len=0.125, blade_w=0.024, travel=0.13, grip_len=0.090, slot_w=0.034),
    dict(blade_len=0.092,  blade_w=0.020, travel=0.10, grip_len=0.085, slot_w=0.032),
    dict(blade_len=0.092,  blade_w=0.020, travel=0.10, grip_len=0.085, slot_w=0.032),
]
NUM_KNIVES = len(KNIFE_SPECS)

# Slot (X, Y) positions on the block top face:
# back row (knives 0-2) toward -X, front row (knives 3-5) toward +X
KNIFE_POSITIONS: list[tuple[float, float]] = [
    (-0.020, -0.041), (-0.020, 0.000), (-0.020, 0.041),
    ( 0.020, -0.041), ( 0.020, 0.000), ( 0.020, 0.041),
]

# Shears
SHEARS_TRAVEL = 0.10
SHEARS_OPEN = 0.70  # rad (~40 deg)
PIVOT_X = 0.006     # pivot-rivet X in shears local frame (along blade axis)
SHEARS_SLOT_Z = LIFT + 0.080  # height of front shears-slot centre

GRIP_Z0 = 0.018  # knife grip loft start above the slot mouth


# ============================================================ geometry helpers

def _build_block_solid() -> cq.Workplane:
    """Upright oak body with vertical knife slots and horizontal shears slot."""
    body = (
        cq.Workplane("XY")
        .box(BLOCK_D, BLOCK_W, BLOCK_H)
        .translate((0.0, 0.0, LIFT + BLOCK_H / 2.0))
    )

    # One vertical slot per knife, cut from the top face downward
    # Slot X = slot_w (accommodates blade width), Y = SLOT_THICK (clears bolster)
    # Extend 5 mm above the top face for a clean through-cut
    _SLOT_EXTRA = 0.005
    for i in range(NUM_KNIVES):
        spec = KNIFE_SPECS[i]
        kx, ky = KNIFE_POSITIONS[i]
        depth = spec["blade_len"] + 0.020
        slot = (
            cq.Workplane("XY")
            .box(spec["slot_w"], SLOT_THICK, depth + _SLOT_EXTRA)
            .translate((kx, ky, TOP_Z - depth / 2.0 + _SLOT_EXTRA / 2.0))
        )
        body = body.cut(slot)

    # Horizontal shears slot cut from the front face inward
    sh_w, sh_h, sh_d = 0.060, 0.016, 0.105
    shears_slot = (
        cq.Workplane("XY")
        .box(sh_d, sh_w, sh_h)
        .translate((FRONT_X - sh_d / 2.0 + 0.001, 0.0, SHEARS_SLOT_Z))
    )
    body = body.cut(shears_slot)

    return body


def _grip_loft(sections: list[tuple[float, float, float, float, float]]) -> cq.Workplane:
    """Loft elliptical cross-sections stacked along +Z: (z, x, y, rx, ry)."""
    z0, x0, y0, a0, b0 = sections[0]
    wp = cq.Workplane("XY").workplane(offset=z0).center(x0, y0).ellipse(a0, b0)
    pz, px, py = z0, x0, y0
    for z, x, y, a, b in sections[1:]:
        wp = wp.workplane(offset=z - pz).center(x - px, y - py).ellipse(a, b)
        pz, px, py = z, x, y
    return wp.loft(combine=True)


def _knife_steel(blade_len: float, blade_w: float) -> cq.Workplane:
    """Tapered blade (down -Z into the slot) plus bolster at the mouth."""
    half = blade_w / 2.0
    pts = [
        (-half, 0.0),
        (half, 0.0),
        (half, -0.45 * blade_len),
        (-half + 0.003, -blade_len),
    ]
    blade = cq.Workplane("XZ").polyline(pts).close().extrude(0.00125, both=True)
    bolster = cq.Workplane("XY").box(0.013, 0.011, 0.018).translate((0.0, 0.0, 0.009))
    return blade.union(bolster)


def _knife_grip(grip_len: float) -> cq.Workplane:
    """Curved walnut handle sweeping back as it rises along +Z."""
    return _grip_loft([
        (GRIP_Z0, 0.000, 0.0, 0.0110, 0.0085),
        (GRIP_Z0 + 0.35 * grip_len, -0.004, 0.0, 0.0130, 0.0100),
        (GRIP_Z0 + 0.75 * grip_len, -0.010, 0.0, 0.0125, 0.0095),
        (GRIP_Z0 + grip_len, -0.016, 0.0, 0.0090, 0.0070),
    ])


def _knife_rivet_x(grip_len: float, frac: float) -> float:
    """Interpolated handle-curve X offset for a rivet at frac of grip length."""
    stations = [(0.0, 0.0), (0.35, -0.004), (0.75, -0.010), (1.0, -0.016)]
    for (f0, x0), (f1, x1) in zip(stations, stations[1:]):
        if frac <= f1:
            t = (frac - f0) / (f1 - f0)
            return x0 + t * (x1 - x0)
    return stations[-1][1]


def _shears_steel(inner: bool) -> cq.Workplane:
    """One shears half (vertical frame): tang + tapered blade along Z."""
    if inner:
        pts = [(-0.008, 0.016), (0.008, 0.016), (0.008, -0.025),
               (0.001, -0.094), (-0.004, -0.094), (-0.008, -0.025)]
        xoff = -0.00225
    else:
        pts = [(-0.008, 0.010), (0.008, 0.010), (0.008, -0.031),
               (0.001, -0.100), (-0.004, -0.100), (-0.008, -0.031)]
        xoff = 0.00225
    half = (
        cq.Workplane("YZ")
        .polyline(pts)
        .close()
        .extrude(0.00175, both=True)
        .translate((xoff, 0.0, 0.0))
    )
    if not inner:
        hole = cq.Workplane("YZ").circle(0.0045).extrude(0.02, both=True)
        half = half.cut(hole)
    return half


def _shears_grip(inner: bool) -> cq.Workplane:
    """One shears grip (vertical frame): loft along +Z."""
    if inner:
        sections = [
            (0.012, -0.00425, 0.0050, 0.00375, 0.0060),
            (0.040, -0.00425, 0.0145, 0.00375, 0.0070),
            (0.070, -0.00425, 0.0235, 0.00375, 0.0070),
            (0.092, -0.00425, 0.0190, 0.00350, 0.0055),
        ]
    else:
        sections = [
            (0.006, 0.00425, -0.0050, 0.00375, 0.0060),
            (0.034, 0.00425, -0.0145, 0.00375, 0.0070),
            (0.064, 0.00425, -0.0235, 0.00375, 0.0070),
            (0.086, 0.00425, -0.0190, 0.00350, 0.0055),
        ]
    return _grip_loft(sections)


def _shears_steel_h(inner: bool) -> cq.Workplane:
    """Shears half rotated so blades point along -X for the horizontal slot."""
    return _shears_steel(inner).rotate((0, 0, 0), (0, 1, 0), 90)


def _shears_grip_h(inner: bool) -> cq.Workplane:
    """Shears grip rotated for horizontal orientation."""
    return _shears_grip(inner).rotate((0, 0, 0), (0, 1, 0), 90)


# ============================================================ build

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_knife_block_set")

    oak = model.material("oak", rgba=(0.80, 0.64, 0.42, 1.0))
    engrave = model.material("engraved_oak", rgba=(0.42, 0.28, 0.15, 1.0))
    walnut = model.material("walnut", rgba=(0.32, 0.19, 0.11, 1.0))
    steel = model.material("stainless_steel", rgba=(0.78, 0.79, 0.82, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    # -------------------------------------------------------- block (root)
    block = model.part("knife_block")
    block.visual(
        mesh_from_cadquery(_build_block_solid(), "block_shell"),
        origin=Origin(),
        material=oak,
        name="block_shell",
    )

    # Four dark rubber feet (1 mm embedded into the block bottom)
    foot_xy = [(0.045, -0.050), (0.045, 0.050),
               (-0.045, -0.050), (-0.045, 0.050)]
    for i, (fx, fy) in enumerate(foot_xy):
        block.visual(
            Cylinder(radius=0.009, length=0.009),
            origin=Origin(xyz=(fx, fy, 0.0045)),
            material=rubber,
            name=f"foot_{i}",
        )

    # Engraved logo on the front face above the shears slot
    logo_z = LIFT + 0.175
    block.visual(
        Box((0.002, 0.052, 0.052)),
        origin=Origin(xyz=(FRONT_X + 0.001, 0.0, logo_z)),
        material=engrave,
        name="logo_seal",
    )
    block.visual(
        Box((0.002, 0.064, 0.007)),
        origin=Origin(xyz=(FRONT_X + 0.001, 0.0, logo_z - 0.040)),
        material=engrave,
        name="logo_text",
    )

    # -------------------------------------------------------- knives (loop)
    for i in range(NUM_KNIVES):
        spec = KNIFE_SPECS[i]
        kx, ky = KNIFE_POSITIONS[i]
        knife = model.part(f"knife_{i}")

        knife.visual(
            mesh_from_cadquery(
                _knife_steel(spec["blade_len"], spec["blade_w"]),
                f"knife_{i}_steel",
            ),
            material=steel,
            name=f"knife_{i}_steel",
        )
        knife.visual(
            mesh_from_cadquery(
                _knife_grip(spec["grip_len"]),
                f"knife_{i}_grip",
            ),
            material=walnut,
            name=f"knife_{i}_grip",
        )
        for j, frac in enumerate((0.30, 0.70)):
            knife.visual(
                Cylinder(radius=0.0032, length=0.022),
                origin=Origin(
                    xyz=(_knife_rivet_x(spec["grip_len"], frac), 0.0,
                         GRIP_Z0 + frac * spec["grip_len"]),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel,
                name=f"knife_{i}_rivet_{j}",
            )

        model.articulation(
            f"knife_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=block,
            child=knife,
            origin=Origin(xyz=(kx, ky, TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=20.0, velocity=0.5,
                lower=0.0, upper=spec["travel"],
            ),
        )

    # -------------------------------------------------------- shears
    inner = model.part("shears_inner_half")
    inner.visual(
        mesh_from_cadquery(_shears_steel_h(inner=True), "shears_inner_steel"),
        material=steel,
        name="shears_inner_steel",
    )
    inner.visual(
        mesh_from_cadquery(_shears_grip_h(inner=True), "shears_inner_grip"),
        material=walnut,
        name="shears_inner_grip",
    )
    # Central pivot rivet (spans through the outer half clearance hole)
    inner.visual(
        Cylinder(radius=0.0035, length=0.013),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0)),
        material=steel,
        name="shears_pivot_rivet",
    )
    # Grip rivets (positions in horizontal frame: blade axis is X)
    for j, (rx, ry, rz) in enumerate(((0.030, 0.011, 0.00425),
                                       (0.065, 0.022, 0.00425))):
        inner.visual(
            Cylinder(radius=0.0022, length=0.0085),
            origin=Origin(xyz=(rx, ry, rz)),
            material=steel,
            name=f"shears_inner_rivet_{j}",
        )

    outer = model.part("shears_outer_half")
    outer.visual(
        mesh_from_cadquery(_shears_steel_h(inner=False), "shears_outer_steel"),
        material=steel,
        name="shears_outer_steel",
    )
    outer.visual(
        mesh_from_cadquery(_shears_grip_h(inner=False), "shears_outer_grip"),
        material=walnut,
        name="shears_outer_grip",
    )
    for j, (rx, ry, rz) in enumerate(((0.024, -0.011, -0.00425),
                                       (0.059, -0.022, -0.00425))):
        outer.visual(
            Cylinder(radius=0.0022, length=0.0085),
            origin=Origin(xyz=(rx, ry, rz)),
            material=steel,
            name=f"shears_outer_rivet_{j}",
        )

    # Shears prismatic: slide out of the front slot along +X
    model.articulation(
        "shears_slide",
        ArticulationType.PRISMATIC,
        parent=block,
        child=inner,
        origin=Origin(xyz=(FRONT_X, 0.0, SHEARS_SLOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.5,
            lower=0.0, upper=SHEARS_TRAVEL,
        ),
    )
    # Shears revolute: pivot opens handles in the X-Y plane
    model.articulation(
        "shears_pivot",
        ArticulationType.REVOLUTE,
        parent=inner,
        child=outer,
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0,
            lower=0.0, upper=SHEARS_OPEN,
        ),
    )

    return model


# ============================================================ tests

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    block = object_model.get_part("knife_block")
    inner = object_model.get_part("shears_inner_half")
    outer = object_model.get_part("shears_outer_half")
    slide = object_model.get_articulation("shears_slide")
    pivot = object_model.get_articulation("shears_pivot")

    # Gravity-seated clearance fits: knives and shears hang loosely in their
    # slots without touching the slot walls.
    for i in range(NUM_KNIVES):
        ctx.allow_isolated_part(
            object_model.get_part(f"knife_{i}"),
            reason=(
                "Knife rests by gravity in its vertical block slot; the slot "
                "is a loose clearance fit so the blade never touches the walls."
            ),
        )
    ctx.allow_isolated_part(
        inner,
        reason=(
            "Shears inner half rests by gravity in the horizontal front slot; "
            "loose clearance fit against the slot walls."
        ),
    )
    ctx.allow_isolated_part(
        outer,
        reason=(
            "Outer shears half rides the pivot rivet through its clearance "
            "hole and rests in the same gravity-seated front slot."
        ),
    )

    # ---- block: grounded, upright vertical form, true scale
    bb = ctx.part_world_aabb(block)
    ctx.check(
        "block stands on its feet at z=0",
        bb is not None and abs(bb[0][2]) < 0.0015,
        details=f"block aabb={bb}",
    )
    ctx.check(
        "block is ~0.24 m tall",
        bb is not None and 0.230 < bb[1][2] < 0.260,
        details=f"zmax={bb[1][2] if bb else None}",
    )
    ctx.check(
        "block is ~0.13 m wide (Y)",
        bb is not None and 0.125 < (bb[1][1] - bb[0][1]) < 0.145,
        details=f"width={(bb[1][1] - bb[0][1]) if bb else None}",
    )
    ctx.check(
        "block is ~0.12 m deep (X) — upright form",
        bb is not None and 0.115 < (bb[1][0] - bb[0][0]) < 0.135,
        details=f"depth={(bb[1][0] - bb[0][0]) if bb else None}",
    )
    # Upright: height >> depth
    ctx.check(
        "block height exceeds depth (vertical orientation)",
        bb is not None
        and (bb[1][2] - bb[0][2]) > 1.5 * (bb[1][0] - bb[0][0]),
        details="height/depth ratio check",
    )

    seal = ctx.part_element_world_aabb(block, elem="logo_seal")
    ctx.check(
        "engraved logo sits on the front face above the shears slot",
        seal is not None and seal[1][0] > FRONT_X - 0.005 and seal[0][2] > 0.10,
        details=f"logo aabb={seal}",
    )

    # ---- knives: six independent prismatic slides along vertical axis
    for i in range(NUM_KNIVES):
        spec = KNIFE_SPECS[i]
        kname = f"knife_{i}"
        knife = object_model.get_part(kname)
        joint = object_model.get_articulation(f"{kname}_slide")
        lim = joint.motion_limits

        ctx.check(
            f"{kname} slide is prismatic with travel {spec['travel']:.2f} m",
            joint.articulation_type == ArticulationType.PRISMATIC
            and lim is not None
            and abs(lim.lower) < 1e-9
            and abs(lim.upper - spec["travel"]) < 1e-6,
            details=f"limits=({lim.lower}, {lim.upper})" if lim else "no limits",
        )

        # Stowed: blade hidden within the block footprint
        ctx.expect_within(
            knife, block, axes="xy",
            inner_elem=f"{kname}_steel",
            outer_elem="block_shell",
            margin=0.001,
            name=f"{kname} blade stays inside block footprint when stowed",
        )
        ctx.expect_overlap(
            knife, block, axes="z",
            elem_a=f"{kname}_steel", elem_b="block_shell",
            min_overlap=0.8 * spec["blade_len"],
            name=f"{kname} blade is inserted deep into slot when stowed",
        )

        rest_blade = ctx.part_element_world_aabb(knife, elem=f"{kname}_steel")
        grip_rest = ctx.part_element_world_aabb(knife, elem=f"{kname}_grip")
        ctx.check(
            f"{kname} handle protrudes above the block when stowed",
            grip_rest is not None and bb is not None
            and grip_rest[1][2] > bb[1][2] + 0.03,
            details=f"grip zmax={grip_rest[1][2] if grip_rest else None}",
        )

        with ctx.pose({joint: spec["travel"]}):
            drawn_blade = ctx.part_element_world_aabb(knife, elem=f"{kname}_steel")
            ctx.check(
                f"{kname} draws upward along the vertical slot axis",
                rest_blade is not None and drawn_blade is not None
                and (drawn_blade[0][2] - rest_blade[0][2]) > 0.95 * spec["travel"],
                details=(
                    f"rest zmin={rest_blade[0][2] if rest_blade else None}, "
                    f"drawn zmin={drawn_blade[0][2] if drawn_blade else None}"
                ),
            )
            ctx.check(
                f"{kname} blade clears the slot mouth at full draw",
                drawn_blade is not None and drawn_blade[0][2] > TOP_Z - 0.006,
                details=(
                    f"blade zmin={drawn_blade[0][2] if drawn_blade else None}, "
                    f"mouth z={TOP_Z}"
                ),
            )

    # Riveted handle detail (chef knife = knife_0)
    chef = object_model.get_part("knife_0")
    rivet = ctx.part_element_world_aabb(chef, elem="knife_0_rivet_0")
    ctx.check(
        "knife_0 rivet spans through the walnut grip",
        rivet is not None and (rivet[1][1] - rivet[0][1]) > 0.0205,
        details=f"rivet aabb={rivet}",
    )

    # ---- shears: slide out of front slot along +X, then open at pivot
    slim = slide.motion_limits
    ctx.check(
        "shears slide is prismatic with ~0.10 m travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slim is not None and abs(slim.upper - SHEARS_TRAVEL) < 1e-6,
        details=f"limits=({slim.lower}, {slim.upper})" if slim else "no limits",
    )
    plim = pivot.motion_limits
    ctx.check(
        "shears pivot is revolute opening 0..~40 deg",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and plim is not None and abs(plim.lower) < 1e-9
        and 0.6 < plim.upper < 0.8,
        details=f"limits=({plim.lower}, {plim.upper})" if plim else "no limits",
    )

    # Stowed: blades inside the block along X
    ctx.expect_within(
        inner, block, axes="yz",
        inner_elem="shears_inner_steel",
        outer_elem="block_shell",
        margin=0.001,
        name="stowed shears blade sits inside front slot footprint (YZ)",
    )
    ctx.expect_overlap(
        inner, block, axes="x",
        elem_a="shears_inner_steel", elem_b="block_shell",
        min_overlap=0.05,
        name="stowed shears blades are inserted into the front slot",
    )

    with ctx.pose({slide: SHEARS_TRAVEL}):
        out_pos = ctx.part_world_position(inner)
        rest_pos = ctx.part_world_position(inner)
        # After exiting pose, we get rest. Let me restructure:

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({slide: SHEARS_TRAVEL}):
        out_pos = ctx.part_world_position(inner)
        closed_handle = ctx.part_element_world_aabb(
            outer, elem="shears_outer_grip",
        )
    with ctx.pose({slide: SHEARS_TRAVEL, pivot: 0.6}):
        open_handle = ctx.part_element_world_aabb(
            outer, elem="shears_outer_grip",
        )

    ctx.check(
        "shears slide out of the front slot along +X",
        rest_pos is not None and out_pos is not None
        and (out_pos[0] - rest_pos[0]) > 0.95 * SHEARS_TRAVEL,
        details=f"rest={rest_pos}, out={out_pos}",
    )
    ctx.check(
        "pivot swings the outer shears handle open (Y sweep)",
        closed_handle is not None and open_handle is not None
        and abs(open_handle[0][1] - closed_handle[0][1]) > 0.005,
        details=(
            f"closed ymin={closed_handle[0][1] if closed_handle else None}, "
            f"open ymin={open_handle[0][1] if open_handle else None}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
