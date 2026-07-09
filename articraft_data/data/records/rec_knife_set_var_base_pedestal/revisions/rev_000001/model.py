from __future__ import annotations

"""Kitchen knife block set on sculptural pedestal.

A slanted light-oak block (~0.13 x 0.12 x 0.24 m, leaning back 12 deg) mounted
on a turned sculptural pedestal base (dark bronze column with wide foot, barrel-
entasis shaft, and moulded capital). Six knives ride in angled top slots via
independent prismatic joints along the tilted slot axis; a pair of kitchen
shears slides out of a wide front pocket (prismatic) and opens at its central
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
TILT = math.radians(12.0)  # block lean-back angle
S = math.sin(TILT)
C = math.cos(TILT)
TAN = math.tan(TILT)
DEG_TILT = math.degrees(TILT)

LIFT = 0.008  # block bottom height above its support surface

# Pedestal
PEDESTAL_TOP = 0.130  # z-height of pedestal top surface (where feet sit)

# Block side profile in the XZ plane (front toward +X), before the LIFT.
FRONT_BOTTOM = (0.062, 0.0)
BACK_BOTTOM = (-0.058, 0.0)
FRONT_TOP = (0.062 - 0.205 * TAN, 0.205)
BACK_TOP = (-0.058 - 0.235 * TAN, 0.235)
BLOCK_WIDTH = 0.130

# Front pocket (shears bay): gap between body front face and the logo panel.
POCKET_GAP = 0.020
PANEL_THICK = 0.010
PANEL_LEN = 0.150  # along the front face from the bottom
SHEARS_PLANE_OFF = 0.010  # shears center plane offset from the body front face

SLOT_THICK = 0.0066  # slot opening width across the blade (y)

# Per-knife data: row, lateral position, blade length/width, prismatic travel,
# grip length and slot width across the blade plane.
KNIFE_SPECS = [
    dict(name="chef_knife",     row="back",  y= 0.000, blade_len=0.175, blade_w=0.040,
         travel=0.18, grip_len=0.110, slot_w=0.048),
    dict(name="bread_knife",    row="back",  y=-0.041, blade_len=0.155, blade_w=0.028,
         travel=0.16, grip_len=0.105, slot_w=0.046),
    dict(name="santoku_knife",  row="back",  y= 0.041, blade_len=0.145, blade_w=0.040,
         travel=0.15, grip_len=0.105, slot_w=0.048),
    dict(name="utility_knife",  row="front", y= 0.000, blade_len=0.125, blade_w=0.024,
         travel=0.13, grip_len=0.090, slot_w=0.034),
    dict(name="paring_knife_0", row="front", y=-0.041, blade_len=0.092, blade_w=0.020,
         travel=0.10, grip_len=0.085, slot_w=0.032),
    dict(name="paring_knife_1", row="front", y= 0.041, blade_len=0.092, blade_w=0.020,
         travel=0.10, grip_len=0.085, slot_w=0.032),
]

SHEARS_TRAVEL = 0.10
SHEARS_OPEN = 0.70  # rad (~40 deg)
PIVOT_Z = 0.006  # pivot rivet height above the pocket mouth (local slide frame)


# ---------------------------------------------------------- geometry helpers

def _top_point(s: float) -> tuple[float, float]:
    """Point on the slanted top face, s in [0, 1] front->back, in (x, z)."""
    return (
        FRONT_TOP[0] + s * (BACK_TOP[0] - FRONT_TOP[0]),
        FRONT_TOP[1] + s * (BACK_TOP[1] - FRONT_TOP[1]),
    )


BACK_ROW_M = _top_point(0.70)
FRONT_ROW_M = _top_point(0.26)


def _face_pt(l: float, off: float, y: float = 0.0) -> tuple[float, float, float]:
    """Point at arc length l along the front face, pushed off outward (normal)."""
    return (FRONT_BOTTOM[0] - l * S + off * C, y, l * C + off * S)


def _tilted_box(sx: float, sy: float, sz: float,
                center: tuple[float, float, float]) -> cq.Workplane:
    """Axis-aligned box whose local +Z is rotated onto the block axis."""
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz)
        .rotate((0, 0, 0), (0, 1, 0), -DEG_TILT)
        .translate(center)
    )


def _build_block_solid() -> cq.Workplane:
    """Slanted oak body + front pocket panel/cheeks/floor, minus knife slots."""
    prof = [FRONT_BOTTOM, FRONT_TOP, BACK_TOP, BACK_BOTTOM]
    body = (
        cq.Workplane("XZ")
        .polyline(prof)
        .close()
        .extrude(BLOCK_WIDTH / 2.0, both=True)
    )

    panel = _tilted_box(
        PANEL_THICK, BLOCK_WIDTH, PANEL_LEN,
        _face_pt(PANEL_LEN / 2.0, POCKET_GAP + PANEL_THICK / 2.0),
    )
    cheek_a = _tilted_box(0.028, 0.012, PANEL_LEN, _face_pt(PANEL_LEN / 2.0, 0.010, -0.059))
    cheek_b = _tilted_box(0.028, 0.012, PANEL_LEN, _face_pt(PANEL_LEN / 2.0, 0.010, 0.059))
    floor = _tilted_box(0.028, 0.118, 0.018, _face_pt(0.013, 0.010))

    block = body.union(panel).union(cheek_a).union(cheek_b).union(floor)

    # Cut one angled slot per knife, parallel to the block axis.
    for spec in KNIFE_SPECS:
        mx, mz = BACK_ROW_M if spec["row"] == "back" else FRONT_ROW_M
        depth = spec["blade_len"] + 0.020
        clen = depth + 0.050
        coff = (0.050 - depth) / 2.0  # cutter center along the axis, rel. mouth
        center = (mx - coff * S, spec["y"], mz + coff * C)
        block = block.cut(_tilted_box(spec["slot_w"], SLOT_THICK, clen, center))
    return block


def _build_pedestal() -> cq.Workplane:
    """Sculptural turned pedestal via CadQuery revolve.

    Profile in the XZ workplane (2D x = radius, 2D y = height), revolved
    around the vertical axis to produce a lathe-turned column with:
      - wide base plinth with chamfered top edge
      - cove/scotia transition to the shaft
      - barrel-entasis column shaft
      - astragal neck and flared capital
      - flat top plate (abacus)
    """
    # --- half-profile points (radius, height) ---
    pts: list[tuple[float, float]] = []

    # Base plinth
    pts.append((0.000, 0.000))
    pts.append((0.100, 0.000))
    pts.append((0.100, 0.016))
    pts.append((0.094, 0.020))   # chamfer

    # Cove transition from plinth to shaft
    pts.append((0.072, 0.026))
    pts.append((0.054, 0.032))
    pts.append((0.046, 0.038))

    # Column shaft with barrel entasis (8 intermediate stations)
    col_z0, col_z1 = 0.040, 0.110
    col_r0, col_r1 = 0.044, 0.036
    n_col = 8
    for i in range(n_col + 1):
        t = i / n_col
        z = col_z0 + t * (col_z1 - col_z0)
        bulge = 0.005 * math.sin(t * math.pi)
        r = col_r0 + t * (col_r1 - col_r0) + bulge
        pts.append((round(r, 4), round(z, 4)))

    # Astragal neck moulding
    pts.append((0.033, 0.113))
    pts.append((0.038, 0.116))
    pts.append((0.052, 0.119))

    # Capital / top plate
    pts.append((0.074, 0.122))
    pts.append((0.082, 0.124))
    pts.append((0.082, PEDESTAL_TOP))
    pts.append((0.000, PEDESTAL_TOP))

    # Revolve around the vertical axis (2D y-axis in the XZ workplane = 3D Z)
    return (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .revolve(360, (0, 0), (0, 1))
    )


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
    """Tapered blade (down -Z) plus a bolster block, in the knife local frame."""
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
    """Curved walnut handle, sweeping back (-X) as it rises along +Z."""
    z0 = 0.031
    return _grip_loft([
        (z0, 0.000, 0.0, 0.0110, 0.0085),
        (z0 + 0.35 * grip_len, -0.004, 0.0, 0.0130, 0.0100),
        (z0 + 0.75 * grip_len, -0.010, 0.0, 0.0125, 0.0095),
        (z0 + grip_len, -0.016, 0.0, 0.0090, 0.0070),
    ])


def _knife_rivet_x(grip_len: float, frac: float) -> float:
    """Interpolated handle-curve x offset for a rivet at frac of grip length."""
    stations = [(0.0, 0.0), (0.35, -0.004), (0.75, -0.010), (1.0, -0.016)]
    for (f0, x0), (f1, x1) in zip(stations, stations[1:]):
        if frac <= f1:
            t = (frac - f0) / (f1 - f0)
            return x0 + t * (x1 - x0)
    return stations[-1][1]


def _shears_steel(inner: bool) -> cq.Workplane:
    """One shears half: tang + tapered blade, flats stacked along x."""
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
        # Clearance hole around the pivot rivet (rivet r=0.0035, hole r=0.0045).
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


# -------------------------------------------------------- build object model

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_knife_block_set")

    # --- materials ---
    oak = model.material("oak", rgba=(0.80, 0.64, 0.42, 1.0))
    engrave = model.material("engraved_oak", rgba=(0.42, 0.28, 0.15, 1.0))
    walnut = model.material("walnut", rgba=(0.32, 0.19, 0.11, 1.0))
    steel = model.material("stainless_steel", rgba=(0.78, 0.79, 0.82, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    bronze = model.material("bronze", rgba=(0.28, 0.24, 0.19, 1.0))

    # ============================================================ pedestal (root)
    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_build_pedestal(), "pedestal_column"),
        material=bronze,
        name="pedestal_column",
    )

    # Anti-slip pads on pedestal base bottom
    pad_angles = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]
    pad_r = 0.078  # radial position on base plate
    for i in range(4):
        px = pad_r * math.cos(pad_angles[i])
        py = pad_r * math.sin(pad_angles[i])
        pedestal.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(px, py, -0.0015)),
            material=rubber,
            name=f"base_pad_{i}",
        )

    # ============================================================ knife block (FIXED on pedestal)
    block = model.part("knife_block")
    block.visual(
        mesh_from_cadquery(_build_block_solid(), "block_shell"),
        origin=Origin(xyz=(0.0, 0.0, LIFT)),
        material=oak,
        name="block_shell",
    )

    # Four dark rubber feet (bottom at z=0 in block frame → contact pedestal top)
    foot_xy = [(0.045, -0.046), (0.045, 0.046), (-0.040, -0.046), (-0.040, 0.046)]
    for i, (fx, fy) in enumerate(foot_xy):
        block.visual(
            Cylinder(radius=0.009, length=0.009),
            origin=Origin(xyz=(fx, fy, 0.0045)),
            material=rubber,
            name=f"foot_{i}",
        )

    # Engraved logo: dark square seal + text bar on the pocket panel face.
    seal_c = _face_pt(0.075, POCKET_GAP + PANEL_THICK)
    block.visual(
        Box((0.002, 0.052, 0.052)),
        origin=Origin(xyz=(seal_c[0], seal_c[1], seal_c[2] + LIFT), rpy=(0.0, -TILT, 0.0)),
        material=engrave,
        name="logo_seal",
    )
    text_c = _face_pt(0.035, POCKET_GAP + PANEL_THICK)
    block.visual(
        Box((0.002, 0.064, 0.007)),
        origin=Origin(xyz=(text_c[0], text_c[1], text_c[2] + LIFT), rpy=(0.0, -TILT, 0.0)),
        material=engrave,
        name="logo_text",
    )

    # FIXED joint: block sits on pedestal top plate, feet contact the surface
    model.articulation(
        "pedestal_to_block",
        ArticulationType.FIXED,
        parent=pedestal,
        child=block,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOP)),
    )

    # ============================================================ knives (loop-emitted prismatic slides)
    for i, spec in enumerate(KNIFE_SPECS):
        kname = spec["name"]
        mx, mz = BACK_ROW_M if spec["row"] == "back" else FRONT_ROW_M

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
            parent=block,
            child=knife,
            origin=Origin(xyz=(mx, spec["y"], mz + LIFT), rpy=(0.0, -TILT, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=0.5,
                                       lower=0.0, upper=spec["travel"]),
        )

    # ============================================================ shears (prismatic slide + revolute pivot)
    mouth = _face_pt(PANEL_LEN, SHEARS_PLANE_OFF)

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
    # Central pivot rivet, spanning through the outer half's clearance hole.
    inner.visual(
        Cylinder(radius=0.0035, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shears_pivot_rivet",
    )
    for j, (rz, ry) in enumerate(((0.030, 0.011), (0.065, 0.022))):
        inner.visual(
            Cylinder(radius=0.0022, length=0.0085),
            origin=Origin(xyz=(-0.00425, ry, rz), rpy=(0.0, math.pi / 2.0, 0.0)),
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
            origin=Origin(xyz=(0.00425, ry, rz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"shears_outer_rivet_{j}",
        )

    model.articulation(
        "shears_slide",
        ArticulationType.PRISMATIC,
        parent=block,
        child=inner,
        origin=Origin(xyz=(mouth[0], 0.0, mouth[2] + LIFT), rpy=(0.0, -TILT, 0.0)),
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


# -------------------------------------------------------------------- tests

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    block = object_model.get_part("knife_block")
    inner = object_model.get_part("shears_inner_half")
    outer = object_model.get_part("shears_outer_half")
    mount = object_model.get_articulation("pedestal_to_block")
    slide = object_model.get_articulation("shears_slide")
    pivot = object_model.get_articulation("shears_pivot")

    # Gravity-seated clearance fits: knives and shears hang in their slots
    # without touching the slot walls.
    for spec in KNIFE_SPECS:
        ctx.allow_isolated_part(
            object_model.get_part(spec["name"]),
            reason=(
                "Knife rests by gravity in its angled block slot; the slot is a "
                "loose clearance fit so the blade never touches the slot walls."
            ),
        )
    ctx.allow_isolated_part(
        inner,
        reason=(
            "Shears half rests by gravity in the wide front pocket slot; "
            "loose clearance fit against the pocket walls."
        ),
    )
    ctx.allow_isolated_part(
        outer,
        reason=(
            "Outer shears half rides the pivot rivet through its clearance "
            "hole and rests in the same gravity-seated pocket slot."
        ),
    )

    # --- pedestal: grounded at z=0, sculptural turned column
    ped_bb = ctx.part_world_aabb(pedestal)
    ctx.check(
        "pedestal base is grounded at z=0",
        ped_bb is not None and abs(ped_bb[0][2]) < 0.005,
        details=f"pedestal aabb={ped_bb}",
    )
    ctx.check(
        "pedestal is ~0.13 m tall",
        ped_bb is not None and 0.120 < ped_bb[1][2] < 0.140,
        details=f"zmax={ped_bb[1][2] if ped_bb else None}",
    )
    # Pedestal has a wide base (sculptural, not a thin pole)
    ped_dx = (ped_bb[1][0] - ped_bb[0][0]) if ped_bb else 0.0
    ped_dy = (ped_bb[1][1] - ped_bb[0][1]) if ped_bb else 0.0
    ctx.check(
        "pedestal base spans ~0.19+ m (wide sculptural foot)",
        ped_dx > 0.18 and ped_dy > 0.18,
        details=f"dx={ped_dx:.4f}, dy={ped_dy:.4f}",
    )

    # --- FIXED mount: pedestal_to_block
    ctx.check(
        "pedestal_to_block is a FIXED articulation",
        mount.articulation_type == ArticulationType.FIXED,
        details=f"type={mount.articulation_type}",
    )

    # --- block: elevated on pedestal, true scale, slanted form
    bb = ctx.part_world_aabb(block)
    ctx.check(
        "block is elevated on pedestal (feet above z=0.12)",
        bb is not None and bb[0][2] > 0.12,
        details=f"block zmin={bb[0][2] if bb else None}",
    )
    ctx.check(
        "block top is ~0.37 m above ground (pedestal + block height)",
        bb is not None and 0.35 < bb[1][2] < 0.39,
        details=f"zmax={bb[1][2] if bb else None}",
    )
    ctx.check(
        "block is ~0.13 m wide",
        bb is not None and 0.125 < (bb[1][1] - bb[0][1]) < 0.165,
        details=f"width={(bb[1][1] - bb[0][1]) if bb else None}",
    )
    seal = ctx.part_element_world_aabb(block, elem="logo_seal")
    ctx.check(
        "engraved logo sits on the front panel face",
        seal is not None and seal[1][0] > 0.080 and seal[0][2] > 0.16,
        details=f"logo aabb={seal}",
    )

    # --- knives: six independent prismatic slides along the tilted slot axis
    for spec in KNIFE_SPECS:
        kname = spec["name"]
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
        # Stowed: blade+bolster steel hidden within the block footprint.
        ctx.expect_within(
            knife,
            block,
            axes="xy",
            inner_elem=f"{kname}_steel",
            outer_elem="block_shell",
            margin=0.001,
            name=f"{kname} blade stays inside the block footprint when stowed",
        )
        ctx.expect_overlap(
            knife,
            block,
            axes="z",
            elem_a=f"{kname}_steel",
            elem_b="block_shell",
            min_overlap=0.8 * spec["blade_len"],
            name=f"{kname} blade is inserted deep into its slot when stowed",
        )
        rest = ctx.part_element_world_aabb(knife, elem=f"{kname}_steel")
        grip_rest = ctx.part_element_world_aabb(knife, elem=f"{kname}_grip")
        ctx.check(
            f"{kname} handle protrudes above the block when stowed",
            grip_rest is not None and bb is not None and grip_rest[1][2] > bb[1][2] + 0.03,
            details=f"grip zmax={grip_rest[1][2] if grip_rest else None}",
        )
        with ctx.pose({joint: spec["travel"]}):
            drawn = ctx.part_element_world_aabb(knife, elem=f"{kname}_steel")
            ctx.check(
                f"{kname} draws out along the tilted slot axis",
                rest is not None
                and drawn is not None
                and (drawn[0][2] - rest[0][2]) > 0.95 * spec["travel"] * C
                and (drawn[0][0] - rest[0][0]) < -0.5 * spec["travel"] * S,
                details=f"rest zmin={rest[0][2] if rest else None}, "
                        f"drawn zmin={drawn[0][2] if drawn else None}",
            )
            mouth_z = (BACK_ROW_M[1] if spec["row"] == "back" else FRONT_ROW_M[1]) + LIFT + PEDESTAL_TOP
            ctx.check(
                f"{kname} blade clears its slot at full draw",
                drawn is not None and drawn[0][2] > mouth_z - 0.006,
                details=f"blade zmin={drawn[0][2] if drawn else None}, mouth z={mouth_z}",
            )

    # Off-axis riveted handle detail (visible rivet heads poke out of the grip).
    chef = object_model.get_part("chef_knife")
    rivet = ctx.part_element_world_aabb(chef, elem="chef_knife_rivet_0")
    ctx.check(
        "chef knife rivet spans through the walnut grip",
        rivet is not None and (rivet[1][1] - rivet[0][1]) > 0.0205,
        details=f"rivet aabb={rivet}",
    )

    # --- shears: slide out of the wide front pocket, then open at the pivot
    lim = slide.motion_limits
    ctx.check(
        "shears slide is prismatic with ~0.10 m travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and lim is not None and abs(lim.upper - SHEARS_TRAVEL) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})" if lim else "no limits",
    )
    plim = pivot.motion_limits
    ctx.check(
        "shears pivot is revolute opening 0..~40 deg",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and plim is not None and abs(plim.lower) < 1e-9
        and 0.6 < plim.upper < 0.8,
        details=f"limits=({plim.lower}, {plim.upper})" if plim else "no limits",
    )
    ctx.expect_within(
        inner,
        block,
        axes="xy",
        inner_elem="shears_inner_steel",
        outer_elem="block_shell",
        margin=0.001,
        name="stowed shears blade sits inside the front pocket footprint",
    )
    ctx.expect_overlap(
        inner,
        block,
        axes="z",
        elem_a="shears_inner_steel",
        elem_b="block_shell",
        min_overlap=0.05,
        name="stowed shears blades are inserted down into the pocket",
    )

    rest_pos = ctx.part_world_position(inner)
    rest_handle = ctx.part_element_world_aabb(outer, elem="shears_outer_grip")
    with ctx.pose({slide: SHEARS_TRAVEL}):
        out_pos = ctx.part_world_position(inner)
        ctx.check(
            "shears slide out of the pocket along the tilted axis",
            rest_pos is not None and out_pos is not None
            and (out_pos[2] - rest_pos[2]) > 0.95 * SHEARS_TRAVEL * C,
            details=f"rest={rest_pos}, out={out_pos}",
        )
        with ctx.pose({slide: SHEARS_TRAVEL, pivot: 0.6}):
            open_handle = ctx.part_element_world_aabb(outer, elem="shears_outer_grip")
            ctx.check(
                "pivot swings the outer shears handle open (off-axis sweep)",
                rest_handle is not None and open_handle is not None
                and open_handle[0][1] < rest_handle[0][1] - 0.02,
                details=f"rest ymin={rest_handle[0][1] if rest_handle else None}, "
                        f"open ymin={open_handle[0][1] if open_handle else None}",
            )

    return ctx.report()


object_model = build_object_model()
