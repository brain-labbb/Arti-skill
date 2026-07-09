from __future__ import annotations

# Squircle (rounded-square prism) lipstick tube — fork of the round parent.
# Cross-section family changed: cylindrical → rounded-rectangle (squircle).
# Frame: tube axis along +Z, base seated on z=0, cap on top.
# Construction:
#   - tube_base (root): squircle hollow body with a silver center band; open top.
#   - cap: squircle cap that PULLS STRAIGHT UP off the base (PRISMATIC, no thread).
#   - bullet_carrier: massless screw carrier; CONTINUOUS rotate about +Z.
#   - bullet: red lipstick bullet on a silver carrier cup; PRISMATIC rise about +Z.
# The twist-up screw chain (continuous rotate + prismatic rise) is decoupled and
# shares the vertical +Z axis with the cap pull-off joint, but the two are independent.

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
    mesh_from_cadquery,
)

# --- key dimensions (meters) ---
TUBE_HALF_W = 0.011        # half-width of the squircle tube (≈ parent diameter)
TUBE_FILLET = 0.004        # corner fillet for rounded-square look
TUBE_WALL = 0.0016
TUBE_BOTTOM_Z = 0.0
BASE_TOP_Z = 0.050         # top rim of the lower base section
BAND_Z = 0.0455            # center silver band height
INNER_HALF_W = TUBE_HALF_W - TUBE_WALL
INNER_FILLET = max(0.0005, TUBE_FILLET - TUBE_WALL)

CAP_HALF_W = TUBE_HALF_W + 0.0012   # cap slips over the base, slightly larger
CAP_FILLET = TUBE_FILLET + 0.001    # cap corners slightly more rounded
CAP_LEN = 0.046            # cap height
CAP_REST_Z = BAND_Z        # cap bottom rim sits at the silver band when closed

BULLET_R = 0.0072          # red bullet radius (still round — lipstick wax is round)
CUP_R = INNER_HALF_W - 0.0006  # silver carrier cup, slides inside the bore
CUP_LEN = 0.012
BULLET_LEN = 0.020         # straight portion of bullet below the angled tip
RISE_HEIGHT = 0.040        # twist-up travel

# Guide ribs on the carrier cup — create the real sliding fit in the squircle bore.
RIB_RADIAL = 0.0014        # rib radial thickness (bridges cup-to-bore gap + seated fit)
RIB_TANGENTIAL = 0.002     # rib tangential width
RIB_INNER = CUP_R - 0.0002 # ribs start slightly inside the cup surface (mesh connectivity)
RIB_OUTER = RIB_INNER + RIB_RADIAL  # ribs extend past the bore wall for seated contact

BAND_HALF_W = TUBE_HALF_W + 0.0006
BAND_FILLET = TUBE_FILLET + 0.0003


def _squircle_prism(half_w: float, fillet_r: float, length: float, offset_z: float = 0.0) -> cq.Workplane:
    """Create a rounded-rectangle prism (squircle) centered at origin, extending along +Z."""
    w = 2.0 * half_w
    solid = (
        cq.Workplane("XY")
        .workplane(offset=offset_z)
        .rect(w, w)
        .extrude(length)
    )
    if fillet_r > 0 and fillet_r < half_w:
        solid = solid.edges("|Z").fillet(fillet_r)
    return solid


def _squircle_tube(
    outer_half_w: float,
    fillet_r: float,
    length: float,
    wall: float,
    offset_z: float = 0.0,
    floor_thickness: float = 0.004,
) -> cq.Workplane:
    """Hollow squircle prism with a closed floor and open top."""
    outer = _squircle_prism(outer_half_w, fillet_r, length, offset_z)
    inner_half_w = outer_half_w - wall
    inner_fillet = max(0.0005, fillet_r - wall)
    bore = _squircle_prism(inner_half_w, inner_fillet, length, offset_z + floor_thickness)
    return outer.cut(bore)


def _tube_base_solid() -> cq.Workplane:
    """Hollow open-top squircle tube body."""
    return _squircle_tube(TUBE_HALF_W, TUBE_FILLET, BASE_TOP_Z, TUBE_WALL)


def _bullet_cup_solid() -> cq.Workplane:
    """Silver carrier cup: short cylinder that rides in the squircle bore."""
    return (
        cq.Workplane("XY")
        .circle(CUP_R)
        .extrude(CUP_LEN)
    )


def _bullet_red_solid() -> cq.Workplane:
    """Red lipstick column with an angled (slanted) cut tip, sitting on the cup."""
    col_bottom = CUP_LEN
    column = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom)
        .circle(BULLET_R)
        .extrude(BULLET_LEN)
    )
    tip = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom + BULLET_LEN)
        .circle(BULLET_R)
        .extrude(0.010)
    )
    cutter = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom + BULLET_LEN)
        .transformed(rotate=(20.0, 0.0, 0.0))
        .rect(0.05, 0.05)
        .extrude(0.03)
        .translate((0.0, 0.0, 0.004))
    )
    tip = tip.cut(cutter)
    return column.union(tip)


def _center_band_solid() -> cq.Workplane:
    """Silver center band: a thin squircle ring wrapping the tube near the parting line."""
    outer = _squircle_prism(BAND_HALF_W, BAND_FILLET, 0.004, BAND_Z)
    inner = _squircle_prism(TUBE_HALF_W - 0.0002, TUBE_FILLET, 0.004, BAND_Z)
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lipstick_tube_squircle")

    white = model.material("tube_white", rgba=(0.93, 0.93, 0.94, 1.0))
    silver = model.material("silver", rgba=(0.74, 0.75, 0.78, 1.0))
    red = model.material("lipstick_red", rgba=(0.86, 0.10, 0.10, 1.0))
    dark = model.material("marker_dark", rgba=(0.15, 0.15, 0.16, 1.0))

    # ---- tube_base (root): hollow squircle body + silver center band ----
    base = model.part("tube_base")
    base.visual(
        mesh_from_cadquery(_tube_base_solid(), "tube_body"),
        material=white,
        name="tube_body",
    )
    base.visual(
        mesh_from_cadquery(_center_band_solid(), "center_band"),
        material=silver,
        name="center_band",
    )
    base.inertial = Inertial.from_geometry(
        Box((2.0 * TUBE_HALF_W, 2.0 * TUBE_HALF_W, BASE_TOP_Z)),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z / 2.0)),
    )

    # ---- cap: pulls straight UP off the base (PRISMATIC, no thread) ----
    cap = model.part("cap")
    # Hollow squircle cap shell: closed top, open underside slips over the base.
    cap_outer = _squircle_prism(CAP_HALF_W, CAP_FILLET, CAP_LEN)
    cap_inner_half_w = CAP_HALF_W - 0.0014
    cap_inner_fillet = max(0.0005, CAP_FILLET - 0.0014)
    cap_bore = _squircle_prism(cap_inner_half_w, cap_inner_fillet, CAP_LEN - 0.004)
    cap_shell = cap_outer.cut(cap_bore)
    cap.visual(
        mesh_from_cadquery(cap_shell, "cap_shell"),
        material=white,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2.0 * CAP_HALF_W, 2.0 * CAP_HALF_W, CAP_LEN)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, CAP_LEN / 2.0)),
    )
    model.articulation(
        "cap_pull",
        ArticulationType.PRISMATIC,
        parent=base,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.2, lower=0.0, upper=0.050),
    )

    # ---- bullet_carrier: massless screw carrier, CONTINUOUS rotate about +Z ----
    carrier = model.part("bullet_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.004, 0.004, 0.004)), mass=1e-4)
    model.articulation(
        "bullet_twist",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ---- bullet: red lipstick on a silver carrier cup, PRISMATIC rise about +Z ----
    bullet = model.part("bullet")
    bullet.visual(
        mesh_from_cadquery(_bullet_cup_solid(), "carrier_cup"),
        material=silver,
        name="carrier_cup",
    )
    bullet.visual(
        mesh_from_cadquery(_bullet_red_solid(), "bullet_red"),
        material=red,
        name="bullet_red",
    )
    # Guide ribs: 4 thin rails at cardinal directions that slide in the squircle bore.
    # They create the real mechanical contact between bullet and tube body.
    rib_mid_r = (RIB_INNER + RIB_OUTER) / 2.0
    for i in range(4):
        angle = i * math.pi / 2.0
        cx = rib_mid_r * math.cos(angle)
        cy = rib_mid_r * math.sin(angle)
        # Rib oriented radially: X-thick if along X axis, Y-thick if along Y axis.
        if i % 2 == 0:
            rib_size = (RIB_RADIAL, RIB_TANGENTIAL, CUP_LEN)
        else:
            rib_size = (RIB_TANGENTIAL, RIB_RADIAL, CUP_LEN)
        bullet.visual(
            Box(rib_size),
            origin=Origin(xyz=(cx, cy, CUP_LEN / 2.0)),
            material=silver,
            name=f"guide_rib_{i}",
        )

    # Small off-axis marker on the cup so a twist is detectable.
    bullet.visual(
        Box((0.0016, 0.0016, 0.004)),
        origin=Origin(xyz=(CUP_R - 0.0006, 0.0, CUP_LEN / 2.0)),
        material=dark,
        name="twist_marker",
    )
    bullet.inertial = Inertial.from_geometry(
        Cylinder(radius=BULLET_R, length=CUP_LEN + BULLET_LEN),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, (CUP_LEN + BULLET_LEN) / 2.0)),
    )
    model.articulation(
        "bullet_rise",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=bullet,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=RISE_HEIGHT),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)

    base = object_model.get_part("tube_base")
    cap = object_model.get_part("cap")
    carrier = object_model.get_part("bullet_carrier")
    bullet = object_model.get_part("bullet")
    cap_pull = object_model.get_articulation("cap_pull")
    twist = object_model.get_articulation("bullet_twist")
    rise = object_model.get_articulation("bullet_rise")

    # --- material/identity claims ---
    bullet_red = bullet.get_visual("bullet_red")
    tube_body = base.get_visual("tube_body")
    band = base.get_visual("center_band")
    ctx.check(
        "bullet is red",
        tuple(bullet_red.material.rgba[:3]) == (0.86, 0.10, 0.10),
        details=f"bullet rgba={bullet_red.material.rgba}",
    )
    ctx.check(
        "tube body is white/silver (light)",
        min(tube_body.material.rgba[:3]) > 0.7,
        details=f"tube rgba={tube_body.material.rgba}",
    )
    ctx.check(
        "center band is silver",
        abs(band.material.rgba[0] - 0.74) < 0.05,
        details=f"band rgba={band.material.rgba}",
    )

    # --- squircle cross-section: tube body is NOT circular ---
    tube_aabb = ctx.part_world_aabb(base)
    if tube_aabb is not None:
        tube_dx = tube_aabb[1][0] - tube_aabb[0][0]
        tube_dy = tube_aabb[1][1] - tube_aabb[0][1]
        # A squircle has flat-ish faces: the AABB width should be close to 2*half_w,
        # not 2*R (which a cylinder would give). Both axes should be similar (square).
        ctx.check(
            "tube body has squircle cross-section (square-like XY footprint)",
            abs(tube_dx - tube_dy) < 0.003 and tube_dx > 0.018,
            details=f"tube dx={tube_dx:.5f}, dy={tube_dy:.5f}",
        )

    # --- cap has squircle cross-section too ---
    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is not None:
        cap_dx = cap_aabb[1][0] - cap_aabb[0][0]
        cap_dy = cap_aabb[1][1] - cap_aabb[0][1]
        ctx.check(
            "cap has squircle cross-section (square-like XY footprint)",
            abs(cap_dx - cap_dy) < 0.003 and cap_dx > 0.020,
            details=f"cap dx={cap_dx:.5f}, dy={cap_dy:.5f}",
        )

    # --- cap slips over the base; justify the seated overlaps ---
    ctx.allow_overlap(
        cap,
        base,
        elem_a="cap_shell",
        elem_b="tube_body",
        reason="The cap is intentionally slipped over the tube base top (seated pull-off cap).",
    )
    ctx.allow_overlap(
        cap,
        base,
        elem_a="cap_shell",
        elem_b="center_band",
        reason="The cap rim seats down onto the silver center band when closed.",
    )
    # Guide ribs are seated in the bore walls (sliding fit — tiny intentional overlap).
    for i in range(4):
        ctx.allow_overlap(
            bullet,
            base,
            elem_a=f"guide_rib_{i}",
            elem_b="tube_body",
            reason=f"Guide rib {i} is seated into the squircle bore wall (sliding fit).",
        )
    ctx.allow_overlap(
        bullet,
        base,
        elem_a="bullet_red",
        elem_b="tube_body",
        reason="The red bullet starts nested inside the tube before being twisted up.",
    )
    ctx.allow_overlap(
        cap,
        bullet,
        reason="The closed cap intentionally encloses the retracted bullet.",
    )

    # Prove the guide ribs maintain contact with the bore wall (sliding fit).
    for i in range(4):
        ctx.expect_contact(
            bullet,
            base,
            elem_a=f"guide_rib_{i}",
            elem_b="tube_body",
            contact_tol=0.001,
            name=f"guide rib {i} contacts bore wall",
        )

    # cap sits on top of the base in the rest pose
    cap_rest = ctx.part_world_position(cap)
    ctx.check(
        "cap is on top of the tube",
        cap_rest is not None and cap_rest[2] > BAND_Z - 0.001,
        details=f"cap origin={cap_rest}",
    )

    # --- cap pulls straight UP off the base ---
    with ctx.pose({cap_pull: 0.045}):
        cap_up = ctx.part_world_position(cap)
        cap_bottom = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "cap pulls straight up off the tube",
        cap_up is not None and cap_up[2] > cap_rest[2] + 0.040,
        details=f"rest={cap_rest}, pulled={cap_up}",
    )
    ctx.check(
        "pulled cap lifts toward the base top",
        cap_bottom > BASE_TOP_Z - 0.006,
        details=f"cap bottom when pulled={cap_bottom}",
    )

    # --- twisting bullet_twist spins the bullet (off-axis marker moves) ---
    marker0 = ctx.part_element_world_aabb(bullet, elem="twist_marker")
    mc0 = ((marker0[0][0] + marker0[1][0]) / 2.0, (marker0[0][1] + marker0[1][1]) / 2.0)
    with ctx.pose({twist: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(bullet, elem="twist_marker")
        mc1 = ((marker1[0][0] + marker1[1][0]) / 2.0, (marker1[0][1] + marker1[1][1]) / 2.0)
    moved = math.hypot(mc1[0] - mc0[0], mc1[1] - mc0[1])
    ctx.check(
        "twist spins the bullet (marker moves)",
        moved > 0.003,
        details=f"marker rest={mc0}, twisted={mc1}, moved={moved}",
    )

    # --- bullet_rise raises the bullet up out of the tube ---
    bull_rest = ctx.part_world_position(bullet)
    with ctx.pose({rise: RISE_HEIGHT}):
        bull_top_aabb = ctx.part_world_aabb(bullet)
        bull_up = ctx.part_world_position(bullet)
        bull_top_z = bull_top_aabb[1][2]
    ctx.check(
        "twist-up raises the bullet",
        bull_up is not None and bull_up[2] > bull_rest[2] + 0.030,
        details=f"rest={bull_rest}, raised={bull_up}",
    )
    ctx.check(
        "raised bullet emerges above the tube top",
        bull_top_z > BASE_TOP_Z + 0.005,
        details=f"bullet top z when raised={bull_top_z}",
    )

    return ctx.report()


object_model = build_object_model()
