from __future__ import annotations

"""Horizontal countertop knife holder.

A long low light-oak holder (~0.46 x 0.22 m, wedge profile 0.065 m tall at the
back tapering to 0.04 m at the front) on four dark rubber feet, with an
engraved logo panel on the front face. Six knives lie at a shallow 15-degree
angle in top slots (independent prismatic joints along the tilted slot axis);
a pair of kitchen shears sits in a wide end slot (prismatic slide + revolute
pivot between the two halves).
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

SLOT_ANGLE_DEG = 15.0  # slot tilt from horizontal
SLOT_ANGLE = math.radians(SLOT_ANGLE_DEG)
SA = math.sin(SLOT_ANGLE)   # ~0.259
CA = math.cos(SLOT_ANGLE)   # ~0.966

# Joint frame rotation: rotate child +Z to the withdrawal direction
# (0, -CA, SA) via a roll about X.
# Rx(alpha) maps (0,0,1) -> (0, -sin(alpha), cos(alpha)).
# sin(alpha) = CA, cos(alpha) = SA -> alpha = 90 - SLOT_ANGLE_DEG = 75 deg.
JOINT_ROLL = math.radians(90.0 - SLOT_ANGLE_DEG)

# Slot cutter rotation: rotate the cutter +Z to the insertion direction
# (0, CA, -SA).  Rx(beta) maps (0,0,1) -> (0, -sin(beta), cos(beta)).
# -sin(beta) = CA -> sin(beta) = -CA; cos(beta) = -SA -> beta = -(90 + 15).
SLOT_CUT_DEG = -(90.0 + SLOT_ANGLE_DEG)

LIFT = 0.005  # rubber-foot lift

BASE_LEN = 0.46   # X (long axis)
BASE_DEPTH = 0.22  # Y (front-to-back)
BASE_HB = 0.065    # height at back  (-Y)
BASE_HF = 0.040    # height at front (+Y)
HALF_D = BASE_DEPTH / 2.0
HALF_L = BASE_LEN / 2.0

SLOT_MOUTH_Y = -0.07
SLOT_THICK = 0.014   # wide enough for the bolster
SLOT_ABOVE = 0.030   # slot extension above the mouth for bolster clearance

N_KNIVES = 6
KNIFE_SPACING = 0.058

# Per-knife data: blade length/width, prismatic travel, grip length, slot width
KNIFE_SPECS = [
    dict(blade_len=0.175, blade_w=0.040, travel=0.18, grip_len=0.110, slot_w=0.048),
    dict(blade_len=0.155, blade_w=0.028, travel=0.16, grip_len=0.105, slot_w=0.046),
    dict(blade_len=0.145, blade_w=0.040, travel=0.15, grip_len=0.105, slot_w=0.048),
    dict(blade_len=0.125, blade_w=0.024, travel=0.13, grip_len=0.090, slot_w=0.034),
    dict(blade_len=0.092, blade_w=0.020, travel=0.10, grip_len=0.085, slot_w=0.032),
    dict(blade_len=0.092, blade_w=0.020, travel=0.10, grip_len=0.085, slot_w=0.032),
]

SHEARS_X = 0.20
SHEARS_SLOT_W = 0.022
SHEARS_TRAVEL = 0.10
SHEARS_OPEN = 0.70   # rad (~40 deg)
PIVOT_Z = 0.006


# ---------------------------------------------------------------- geometry helpers

def _z_top(y: float) -> float:
    """Top-surface local height (before LIFT) at position y along depth."""
    t = (y + HALF_D) / BASE_DEPTH
    return BASE_HB + (BASE_HF - BASE_HB) * t


def _mouth_z_world() -> float:
    return _z_top(SLOT_MOUTH_Y) + LIFT


def _knife_x(i: int) -> float:
    return -(N_KNIVES - 1) * KNIFE_SPACING / 2.0 + i * KNIFE_SPACING


def _make_slot_cutter(x_center: float, width: float,
                      blade_depth: float) -> cq.Workplane:
    """Build a tilted box cutter for one slot."""
    mouth_z = _z_top(SLOT_MOUTH_Y)
    total = blade_depth + SLOT_ABOVE
    offset = (blade_depth - SLOT_ABOVE) / 2.0  # center shift into the base

    cy = SLOT_MOUTH_Y + offset * CA
    cz = mouth_z - offset * SA

    return (
        cq.Workplane("XY")
        .box(width, SLOT_THICK, total)
        .rotate((0, 0, 0), (1, 0, 0), SLOT_CUT_DEG)
        .translate((x_center, cy, cz))
    )


def _build_holder_solid() -> cq.Workplane:
    """Wedge-profile oak body with knife and shears slots."""
    # Build the wedge using XY workplane: extrude a YZ profile along X.
    # Use a workplane at the YZ plane, extrude along its normal (X).
    wp = cq.Workplane("YZ", origin=(0, 0, 0))
    prof = [
        (-HALF_D, 0.0),
        (-HALF_D, BASE_HB),
        (HALF_D, BASE_HF),
        (HALF_D, 0.0),
    ]
    body = wp.polyline(prof).close().extrude(HALF_L, both=True)

    # Collect all slot cutters and subtract them one by one.
    cutters: list[cq.Workplane] = []
    for i in range(N_KNIVES):
        spec = KNIFE_SPECS[i]
        cutters.append(
            _make_slot_cutter(_knife_x(i), spec["slot_w"],
                              spec["blade_len"] + 0.005))
    cutters.append(_make_slot_cutter(SHEARS_X, SHEARS_SLOT_W, 0.105))

    for cutter in cutters:
        body = body.cut(cutter)

    return body


# ---- knife geometry (local frame: +Z = withdrawal, -Z = blade) ----

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
    """Tapered blade (down -Z) plus a bolster block."""
    half = blade_w / 2.0
    pts = [
        (-half, 0.015),
        (half, 0.015),
        (half, -0.45 * blade_len),
        (-half + 0.003, -blade_len),
    ]
    blade = cq.Workplane("XZ").polyline(pts).close().extrude(0.00125, both=True)
    bolster = cq.Workplane("XY").box(0.013, 0.011, 0.020).translate((0, 0, 0.023))
    return blade.union(bolster)


def _knife_grip(grip_len: float) -> cq.Workplane:
    """Curved walnut handle sweeping back (-X) as it rises along +Z."""
    z0 = 0.031
    return _grip_loft([
        (z0, 0.000, 0.0, 0.0110, 0.0085),
        (z0 + 0.35 * grip_len, -0.004, 0.0, 0.0130, 0.0100),
        (z0 + 0.75 * grip_len, -0.010, 0.0, 0.0125, 0.0095),
        (z0 + grip_len, -0.016, 0.0, 0.0090, 0.0070),
    ])


def _knife_rivet_x(grip_len: float, frac: float) -> float:
    stations = [(0.0, 0.0), (0.35, -0.004), (0.75, -0.010), (1.0, -0.016)]
    for (f0, x0), (f1, x1) in zip(stations, stations[1:]):
        if frac <= f1:
            t = (frac - f0) / (f1 - f0)
            return x0 + t * (x1 - x0)
    return stations[-1][1]


# ---- shears geometry (same local frame as knives) ----

def _shears_steel(inner: bool) -> cq.Workplane:
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


# ============================================================= build / tests

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_knife_holder")

    oak = model.material("oak", rgba=(0.80, 0.64, 0.42, 1.0))
    engrave = model.material("engraved_oak", rgba=(0.42, 0.28, 0.15, 1.0))
    walnut = model.material("walnut", rgba=(0.32, 0.19, 0.11, 1.0))
    steel = model.material("stainless_steel", rgba=(0.78, 0.79, 0.82, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    # --------------------------------------------------------- holder base
    holder = model.part("holder")
    holder.visual(
        mesh_from_cadquery(_build_holder_solid(), "holder_shell"),
        origin=Origin(xyz=(0.0, 0.0, LIFT)),
        material=oak,
        name="holder_shell",
    )

    # Four dark rubber feet near the corners
    foot_xy = [
        (-0.19, -0.08), (-0.19, 0.08),
        (0.19, -0.08), (0.19, 0.08),
    ]
    for i, (fx, fy) in enumerate(foot_xy):
        holder.visual(
            Cylinder(radius=0.009, length=0.009),
            origin=Origin(xyz=(fx, fy, 0.0045)),
            material=rubber,
            name=f"foot_{i}",
        )

    # Engraved logo panel + text bar on the front face (y = +HALF_D)
    front_y = HALF_D + 0.001
    logo_z = LIFT + BASE_HF * 0.60
    holder.visual(
        Box((0.090, 0.002, 0.018)),
        origin=Origin(xyz=(0.0, front_y, logo_z)),
        material=engrave,
        name="logo_panel",
    )
    holder.visual(
        Box((0.060, 0.002, 0.006)),
        origin=Origin(xyz=(0.0, front_y, logo_z - 0.016)),
        material=engrave,
        name="logo_text",
    )

    # --------------------------------------------------------- knives (loop)
    mzw = _mouth_z_world()
    for i in range(N_KNIVES):
        spec = KNIFE_SPECS[i]
        kname = f"knife_{i}"
        xi = _knife_x(i)

        knife = model.part(kname)
        knife.visual(
            mesh_from_cadquery(_knife_steel(spec["blade_len"], spec["blade_w"]),
                               f"{kname}_steel"),
            material=steel,
            name=f"{kname}_steel",
        )
        knife.visual(
            mesh_from_cadquery(_knife_grip(spec["grip_len"]), f"{kname}_grip"),
            material=walnut,
            name=f"{kname}_grip",
        )
        for j, frac in enumerate((0.30, 0.70)):
            knife.visual(
                Cylinder(radius=0.0032, length=0.022),
                origin=Origin(
                    xyz=(_knife_rivet_x(spec["grip_len"], frac), 0.0,
                         0.031 + frac * spec["grip_len"]),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel,
                name=f"{kname}_rivet_{j}",
            )

        model.articulation(
            f"{kname}_slide",
            ArticulationType.PRISMATIC,
            parent=holder,
            child=knife,
            origin=Origin(xyz=(xi, SLOT_MOUTH_Y, mzw),
                          rpy=(JOINT_ROLL, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=0.5,
                                       lower=0.0, upper=spec["travel"]),
        )

    # --------------------------------------------------------- shears
    inner = model.part("shears_inner_half")
    inner.visual(
        mesh_from_cadquery(_shears_steel(inner=True), "shears_inner_steel"),
        material=steel,
        name="shears_inner_steel",
    )
    inner.visual(
        mesh_from_cadquery(_shears_grip(inner=True), "shears_inner_grip"),
        material=walnut,
        name="shears_inner_grip",
    )
    inner.visual(
        Cylinder(radius=0.0035, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shears_pivot_rivet",
    )
    for j, (rz, ry) in enumerate(((0.030, 0.011), (0.065, 0.022))):
        inner.visual(
            Cylinder(radius=0.0022, length=0.0085),
            origin=Origin(xyz=(-0.00425, ry, rz),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"shears_inner_rivet_{j}",
        )

    outer = model.part("shears_outer_half")
    outer.visual(
        mesh_from_cadquery(_shears_steel(inner=False), "shears_outer_steel"),
        material=steel,
        name="shears_outer_steel",
    )
    outer.visual(
        mesh_from_cadquery(_shears_grip(inner=False), "shears_outer_grip"),
        material=walnut,
        name="shears_outer_grip",
    )
    for j, (rz, ry) in enumerate(((0.024, -0.011), (0.059, -0.022))):
        outer.visual(
            Cylinder(radius=0.0022, length=0.0085),
            origin=Origin(xyz=(0.00425, ry, rz),
                          rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"shears_outer_rivet_{j}",
        )

    model.articulation(
        "shears_slide",
        ArticulationType.PRISMATIC,
        parent=holder,
        child=inner,
        origin=Origin(xyz=(SHEARS_X, SLOT_MOUTH_Y, mzw),
                      rpy=(JOINT_ROLL, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.5,
                                   lower=0.0, upper=SHEARS_TRAVEL),
    )
    model.articulation(
        "shears_pivot",
        ArticulationType.REVOLUTE,
        parent=inner,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0,
                                   lower=0.0, upper=SHEARS_OPEN),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    holder = object_model.get_part("holder")
    inner = object_model.get_part("shears_inner_half")
    outer = object_model.get_part("shears_outer_half")
    slide = object_model.get_articulation("shears_slide")
    pivot = object_model.get_articulation("shears_pivot")

    # Knives and shears rest by gravity in their clearance-fit slots.
    # The blade is intentionally nested inside the holder body (slot proxy);
    # scope overlap allowances to the blade/holder element pair.
    for i in range(N_KNIVES):
        kname = f"knife_{i}"
        knife_part = object_model.get_part(kname)
        ctx.allow_isolated_part(
            knife_part,
            reason="Knife rests by gravity in its angled slot; loose clearance fit.",
        )
        ctx.allow_overlap(
            holder, knife_part,
            elem_a="holder_shell",
            elem_b=f"{kname}_steel",
            reason=(
                f"Knife blade is intentionally nested inside the holder slot "
                f"(solid-proxy sleeve); the slot cavity is represented by the "
                f"holder body and the blade slides within it."
            ),
        )
        ctx.allow_overlap(
            holder, knife_part,
            elem_a="holder_shell",
            elem_b=f"{kname}_grip",
            reason=(
                f"Knife grip base seats into the slot mouth surface; small local "
                f"embed at the bolster-grip junction for a gravity-seated fit."
            ),
        )
    ctx.allow_isolated_part(
        inner,
        reason="Shears inner half rests by gravity in its slot; loose clearance fit.",
    )
    ctx.allow_overlap(
        holder, inner,
        elem_a="holder_shell",
        elem_b="shears_inner_steel",
        reason="Shears blade is nested inside the holder shears slot (solid-proxy sleeve).",
    )
    ctx.allow_isolated_part(
        outer,
        reason="Outer shears half rides the pivot rivet through its clearance hole.",
    )
    ctx.allow_overlap(
        holder, outer,
        elem_a="holder_shell",
        elem_b="shears_outer_grip",
        reason=(
            "Outer shears grip sits at the slot mouth when stowed; small local "
            "contact with the holder top surface at the mouth edge."
        ),
    )

    # ---- holder: grounded, long and low, correct scale
    bb = ctx.part_world_aabb(holder)
    ctx.check(
        "holder sits on feet near z=0",
        bb is not None and abs(bb[0][2]) < 0.002,
        details=f"holder aabb={bb}",
    )
    ctx.check(
        "holder is ~0.07 m tall (low profile)",
        bb is not None and 0.058 < bb[1][2] < 0.082,
        details=f"zmax={bb[1][2] if bb else None}",
    )
    ctx.check(
        "holder is ~0.46 m long (horizontal bar)",
        bb is not None and 0.43 < (bb[1][0] - bb[0][0]) < 0.49,
        details=f"length={(bb[1][0] - bb[0][0]) if bb else None}",
    )
    ctx.check(
        "holder is ~0.22 m deep",
        bb is not None and 0.20 < (bb[1][1] - bb[0][1]) < 0.24,
        details=f"depth={(bb[1][1] - bb[0][1]) if bb else None}",
    )

    # Logo on the front face
    logo = ctx.part_element_world_aabb(holder, elem="logo_panel")
    ctx.check(
        "engraved logo sits on the front face",
        logo is not None and logo[0][1] > 0.10,
        details=f"logo aabb={logo}",
    )

    # ---- six knives: independent prismatic slides at shallow angle
    mzw = _mouth_z_world()
    for i in range(N_KNIVES):
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

        # Blade inside the holder footprint when stowed
        ctx.expect_within(
            knife, holder,
            axes="xy",
            inner_elem=f"{kname}_steel",
            outer_elem="holder_shell",
            margin=0.002,
            name=f"{kname} blade stays inside holder footprint when stowed",
        )

        # Handle protrudes above the low base
        grip_rest = ctx.part_element_world_aabb(knife, elem=f"{kname}_grip")
        ctx.check(
            f"{kname} handle protrudes above holder when stowed",
            grip_rest is not None and bb is not None
            and grip_rest[1][2] > bb[1][2] + 0.008,
            details=f"grip zmax={grip_rest[1][2] if grip_rest else None}",
        )

        # Draw-out test: knife moves toward user (-Y) and upward (+Z)
        rest = ctx.part_element_world_aabb(knife, elem=f"{kname}_steel")
        with ctx.pose({joint: spec["travel"]}):
            drawn = ctx.part_element_world_aabb(knife, elem=f"{kname}_steel")
            ctx.check(
                f"{kname} draws out along the shallow slot axis",
                rest is not None and drawn is not None
                and (drawn[1][2] - rest[1][2]) > 0.80 * spec["travel"] * SA
                and (drawn[0][1] - rest[0][1]) < -0.50 * spec["travel"] * CA,
                details=f"rest zmax={rest[1][2] if rest else None}, "
                        f"drawn zmax={drawn[1][2] if drawn else None}",
            )
            ctx.check(
                f"{kname} blade clears its slot at full draw",
                drawn is not None and drawn[0][2] > mzw - 0.008,
                details=f"blade zmin={drawn[0][2] if drawn else None}, "
                        f"mouth z={mzw}",
            )

    # Rivet spans through grip (compressed in world Y by the 75-deg joint roll)
    k0 = object_model.get_part("knife_0")
    rivet = ctx.part_element_world_aabb(k0, elem="knife_0_rivet_0")
    ctx.check(
        "knife_0 rivet spans through the walnut grip",
        rivet is not None and (rivet[1][1] - rivet[0][1]) > 0.010,
        details=f"rivet aabb={rivet}",
    )

    # ---- shears: slide + pivot
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

    # Stowed shears blade inside holder
    ctx.expect_within(
        inner, holder,
        axes="xy",
        inner_elem="shears_inner_steel",
        outer_elem="holder_shell",
        margin=0.003,
        name="stowed shears blade sits inside the holder footprint",
    )

    rest_pos = ctx.part_world_position(inner)
    rest_handle = ctx.part_element_world_aabb(outer, elem="shears_outer_grip")
    with ctx.pose({slide: SHEARS_TRAVEL}):
        out_pos = ctx.part_world_position(inner)
        ctx.check(
            "shears slide out of the slot along the tilted axis",
            rest_pos is not None and out_pos is not None
            and (out_pos[2] - rest_pos[2]) > 0.80 * SHEARS_TRAVEL * SA,
            details=f"rest={rest_pos}, out={out_pos}",
        )
        with ctx.pose({slide: SHEARS_TRAVEL, pivot: 0.6}):
            open_handle = ctx.part_element_world_aabb(outer, elem="shears_outer_grip")
            ctx.check(
                "pivot swings the outer shears handle open",
                rest_handle is not None and open_handle is not None
                and open_handle[0][1] < rest_handle[0][1] - 0.015,
                details=f"rest ymin={rest_handle[0][1] if rest_handle else None}, "
                        f"open ymin={open_handle[0][1] if open_handle else None}",
            )

    return ctx.report()


object_model = build_object_model()
