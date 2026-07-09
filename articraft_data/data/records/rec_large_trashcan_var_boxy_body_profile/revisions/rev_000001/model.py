from __future__ import annotations

# Tall boxy commercial wheeled container (~1100 L profile).
#
# Coordinate convention:
#   - up is +Z; the wheels touch the ground at z=0.
#   - the bin "front" looks toward +X.
#   - the hinge / handle / wheels are at the rear (-X).
#
# Structure:
#   - body (root, static): near-vertical hollow plastic shell with a thick top
#     rim, horizontal corrugation ribs wrapping all four faces (emitted via a
#     for loop over RIB_Z_LIST through a shared rib-geometry helper), the
#     molded rear axle housing, and a fixed steel axle bar.
#   - lid (REVOLUTE about +Y at the rear top rim): slightly domed flip lid with
#     a front grab lip; opens upward/rearward.
#   - left_wheel / right_wheel (CONTINUOUS about +Y): molded plastic road
#     wheels on the rear axle.

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
    TireGeometry,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# --- key dimensions (meters) for ~1100 L commercial container ---
BODY_H = 1.100       # height of the molded body shell
TOP_W = 1.000        # outside width (Y) at the top rim
TOP_D = 1.200        # outside depth (X) at the top rim
BOT_W = 0.980        # near-vertical walls (slight taper only)
BOT_D = 1.180
WALL = 0.020         # plastic wall thickness
RIM_H = 0.050        # thick reinforcing top rim band
CORNER_R = 0.040     # exterior corner radius

# Horizontal corrugation ribs: regular spacing, shared helper, single loop.
RIB_H = 0.025        # rib band height
RIB_PROUD = 0.010    # protrusion beyond the shell outer surface
RIB_INSET = 0.004    # collar overlap into the shell wall (physical connection)
RIB_COUNT = 7
RIB_SPACING = 0.130
RIB_START = 0.160
RIB_Z_LIST = [RIB_START + i * RIB_SPACING for i in range(RIB_COUNT)]

# Wheels
WHEEL_DIA = 0.200
WHEEL_R = WHEEL_DIA / 2.0
WHEEL_W = 0.050
AXLE_R = 0.012
WHEEL_GAP = 0.006

# Body placement: bottom clears the floor by the wheel diameter minus inset.
BODY_BOTTOM_Z = WHEEL_DIA - 0.040
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_H

# Hinge at the rear top rim.
HINGE_X = -TOP_D / 2.0 + 0.022
HINGE_Z = BODY_TOP_Z + 0.010

# Axle at the rear bottom.
AXLE_X = -BOT_D / 2.0 + 0.020
AXLE_Z = WHEEL_R
AXLE_Z_LOCAL = AXLE_Z - BODY_BOTTOM_Z
HALF_AXLE_Y = BOT_W / 2.0 + WHEEL_GAP + WHEEL_W / 2.0


# ---------- geometry helpers ----------


def _rrect(dx: float, dy: float, r: float) -> cq.Sketch:
    """Centered rounded-rectangle sketch in the workplane XY."""
    r = min(r, dx / 2.0 - 1e-4, dy / 2.0 - 1e-4)
    return cq.Sketch().rect(dx, dy).vertices().fillet(r)


def _outer_d_at_z(z: float) -> float:
    """Interpolate shell outer depth at local body height z."""
    frac = z / BODY_H
    return BOT_D + (TOP_D - BOT_D) * frac


def _outer_w_at_z(z: float) -> float:
    """Interpolate shell outer width at local body height z."""
    frac = z / BODY_H
    return BOT_W + (TOP_W - BOT_W) * frac


def _build_rib_mesh(d_outer: float, w_outer: float, name: str):
    """One horizontal corrugation rib: a thin rectangular collar proud of the
    shell wall.  Centered at local z = 0 so the visual origin places it at the
    desired height.  The inner cut is slightly smaller than d_outer / w_outer
    so the collar physically overlaps with the shell (no floating islands)."""
    z_bot = -RIB_H / 2.0
    outer = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(
                d_outer + 2.0 * RIB_PROUD,
                w_outer + 2.0 * RIB_PROUD,
                CORNER_R + RIB_PROUD,
            ).moved(cq.Location(cq.Vector(0, 0, z_bot)))
        )
        .extrude(RIB_H)
    )
    inner = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(
                d_outer - 2.0 * RIB_INSET,
                w_outer - 2.0 * RIB_INSET,
                max(CORNER_R - RIB_INSET, 0.003),
            ).moved(cq.Location(cq.Vector(0, 0, z_bot - 0.001)))
        )
        .extrude(RIB_H + 0.002)
    )
    rib = outer.cut(inner)
    return mesh_from_cadquery(rib, name, unit_scale=1.0)


def _build_body_shell():
    """Near-vertical hollow shell with thick top rim and rear axle housing."""
    # Outer tapered loft (minimal taper → near-vertical walls).
    outer = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(BOT_D, BOT_W, CORNER_R),
            _rrect(TOP_D, TOP_W, CORNER_R).moved(
                cq.Location(cq.Vector(0, 0, BODY_H))
            ),
        )
        .loft()
    )
    # Hollow interior: leaves floor and walls.
    inner = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(BOT_D - 2 * WALL, BOT_W - 2 * WALL, max(CORNER_R - WALL, 0.006)).moved(
                cq.Location(cq.Vector(0, 0, WALL))
            ),
            _rrect(TOP_D - 2 * WALL, TOP_W - 2 * WALL, max(CORNER_R - WALL, 0.006)).moved(
                cq.Location(cq.Vector(0, 0, BODY_H + 0.002))
            ),
        )
        .loft()
    )
    shell = outer.cut(inner)

    # Thick reinforcing top rim band.
    rim = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(TOP_D + 0.014, TOP_W + 0.014, CORNER_R).moved(
                cq.Location(cq.Vector(0, 0, BODY_H - RIM_H))
            )
        )
        .extrude(RIM_H)
    )
    rim_inner = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - RIM_H - 0.001)
        .rect(TOP_D - 2 * WALL, TOP_W - 2 * WALL)
        .extrude(RIM_H + 0.004)
    )
    rim = rim.cut(rim_inner)
    shell = shell.union(rim)

    # Molded rear axle housing descending from the shell bottom.
    leg_top = 0.030
    leg_bot = AXLE_Z_LOCAL - 0.018
    housing = (
        cq.Workplane("XY")
        .center(AXLE_X + 0.018, 0.0)
        .box(0.090, 0.400, leg_top - leg_bot, centered=(True, True, False))
        .edges("|Y")
        .fillet(0.018)
        .translate((0, 0, leg_bot))
    )
    shell = shell.union(housing)

    return mesh_from_cadquery(shell, "bin_body", unit_scale=1.0)


def _build_lid_mesh():
    """Slightly domed flip lid with a front grab lip, hinged at the rear edge."""
    lid_d = TOP_D + 0.018
    lid_w = TOP_W + 0.018
    plate_t = 0.022

    plate = (
        cq.Workplane("XY")
        .placeSketch(_rrect(lid_d, lid_w, CORNER_R + 0.004))
        .extrude(plate_t)
    )
    crown = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(lid_d - 0.10, lid_w - 0.10, CORNER_R).moved(
                cq.Location(cq.Vector(0, 0, plate_t))
            )
        )
        .extrude(0.012)
    )
    lid = plate.union(crown)

    # Down-turned skirt so the lid laps just over the rim outer lip.
    skirt_drop = 0.014
    skirt_outer = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(lid_d, lid_w, CORNER_R + 0.004).moved(
                cq.Location(cq.Vector(0, 0, -skirt_drop))
            )
        )
        .extrude(skirt_drop)
    )
    skirt_inner = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(lid_d - 0.024, lid_w - 0.024, CORNER_R).moved(
                cq.Location(cq.Vector(0, 0, -skirt_drop - 0.002))
            )
        )
        .extrude(skirt_drop + 0.004)
    )
    skirt = skirt_outer.cut(skirt_inner)
    lid = lid.union(skirt)

    # Front grab lip.
    lip = (
        cq.Workplane("XY")
        .center(lid_d / 2.0 - 0.006, 0.0)
        .box(0.034, 0.200, 0.032, centered=(True, True, False))
        .edges("|Y")
        .fillet(0.008)
        .translate((0, 0, -0.008))
    )
    lid = lid.union(lip)

    return mesh_from_cadquery(lid, "bin_lid", unit_scale=1.0)


def _wheel_mesh(name: str):
    wheel = WheelGeometry(
        WHEEL_R - 0.018,
        WHEEL_W - 0.014,
        rim=WheelRim(inner_radius=WHEEL_R - 0.040, flange_height=0.006, flange_thickness=0.004),
        hub=WheelHub(radius=0.020, width=0.030, cap_style="domed"),
        face=WheelFace(dish_depth=0.006, front_inset=0.003),
        spokes=WheelSpokes(style="split_y", count=6, thickness=0.004, window_radius=0.009),
        bore=WheelBore(style="round", diameter=2 * AXLE_R - 0.004),
    )
    return mesh_from_geometry(wheel, name)


def _tire_mesh(name: str):
    tire = TireGeometry(
        WHEEL_R,
        WHEEL_W,
        inner_radius=WHEEL_R - 0.018,
        tread=TireTread(style="block", depth=0.005, count=22, land_ratio=0.6),
        sidewall=TireSidewall(style="rounded", bulge=0.03),
    )
    return mesh_from_geometry(tire, name)


# ---------- model ----------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_wheeled_container")

    body_gray = model.material("body_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    lid_gray = model.material("lid_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.12, 0.12, 0.13, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("axle_steel", rgba=(0.55, 0.56, 0.58, 1.0))

    # --- body (root) ---
    body = model.part("body")
    body.visual(
        _build_body_shell(),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z)),
        material=body_gray,
        name="shell",
    )

    # Horizontal corrugation ribs: single for loop over RIB_Z_LIST,
    # shared geometry helper (_build_rib_mesh), regular spacing, name_i naming.
    for i, z in enumerate(RIB_Z_LIST):
        d = _outer_d_at_z(z)
        w = _outer_w_at_z(z)
        body.visual(
            _build_rib_mesh(d, w, f"rib_{i}"),
            origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + z)),
            material=body_gray,
            name=f"rib_{i}",
        )

    # Fixed steel axle bar.
    body.visual(
        Cylinder(radius=AXLE_R, length=2 * HALF_AXLE_Y - 0.010),
        origin=Origin(xyz=(AXLE_X, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    body.inertial = Inertial.from_geometry(
        Box((TOP_D, TOP_W, BODY_H)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_H / 2.0)),
    )

    # --- lid ---
    lid = model.part("lid")
    lid.visual(
        _build_lid_mesh(),
        origin=Origin(xyz=((TOP_D + 0.018) / 2.0 - 0.022, 0.0, 0.0)),
        material=lid_gray,
        name="lid_plate",
    )
    lid.inertial = Inertial.from_geometry(Box((TOP_D, TOP_W, 0.04)), mass=3.5)

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # Lid extends along local +X from the hinge; -Y lifts the free edge up.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=1.95),
    )

    # --- wheels ---
    for name, sign in (("left_wheel", 1.0), ("right_wheel", -1.0)):
        wheel = model.part(name)
        rpy = (0.0, 0.0, -math.pi / 2.0) if sign > 0 else (0.0, 0.0, math.pi / 2.0)
        wheel.visual(
            _wheel_mesh(f"{name}_wheel"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=rpy),
            material=wheel_black,
            name="wheel",
        )
        wheel.visual(
            _tire_mesh(f"{name}_tire"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=rpy),
            material=tire_black,
            name="tire",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_R, length=WHEEL_W), mass=0.9
        )
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(AXLE_X, sign * HALF_AXLE_Y, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=20.0),
        )

    return model


# ---------- tests ----------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left = object_model.get_part("left_wheel")
    right = object_model.get_part("right_wheel")
    hinge = object_model.get_articulation("lid_hinge")
    left_spin = object_model.get_articulation("left_wheel_spin")

    # --- hero parts present ---
    ctx.check("has_body", body is not None, "Expected a body part.")
    ctx.check("has_lid", lid is not None, "Expected a lid part.")
    ctx.check("has_wheels", left is not None and right is not None, "Expected two wheels.")

    # --- joint types and axes ---
    ctx.check(
        "lid_is_revolute",
        str(hinge.articulation_type).endswith("REVOLUTE"),
        f"type={hinge.articulation_type}",
    )
    ctx.check(
        "lid_axis_y",
        abs(abs(hinge.axis[1]) - 1.0) < 1e-6
        and abs(hinge.axis[0]) < 1e-6
        and abs(hinge.axis[2]) < 1e-6,
        f"axis={hinge.axis}",
    )
    ctx.check(
        "wheel_is_continuous",
        str(left_spin.articulation_type).endswith("CONTINUOUS"),
        f"type={left_spin.articulation_type}",
    )
    ctx.check(
        "wheel_axis_y",
        abs(abs(left_spin.axis[1]) - 1.0) < 1e-6,
        f"axis={left_spin.axis}",
    )

    # --- boxy commercial container profile ---
    full = ctx.part_world_aabb(body)
    if full is not None:
        size = tuple(full[1][i] - full[0][i] for i in range(3))
        ctx.check("body_height_commercial", 1.05 <= size[2] <= 1.45, f"height={size[2]:.3f}")
        ctx.check("body_width_commercial", 0.90 <= size[1] <= 1.15, f"width={size[1]:.3f}")
        ctx.check("body_depth_commercial", 1.10 <= size[0] <= 1.40, f"depth={size[0]:.3f}")
        # Boxy: width is at least 75 % of depth (not a narrow tapered cart).
        if size[0] > 0:
            ctx.check(
                "body_boxy_ratio",
                size[1] >= 0.75 * size[0],
                f"w/d={size[1] / size[0]:.3f}",
            )

    # --- horizontal corrugation ribs present as body visuals ---
    rib_visuals = [v for v in body.visuals if v.name and v.name.startswith("rib_")]
    ctx.check(
        "horizontal_ribs_present",
        len(rib_visuals) == len(RIB_Z_LIST),
        f"expected {len(RIB_Z_LIST)} rib visuals, found {len(rib_visuals)}",
    )

    # --- wheels touch the ground ---
    for w in (left, right):
        aabb = ctx.part_world_aabb(w)
        if aabb is not None:
            ctx.check(
                f"{w.name}_on_ground",
                abs(aabb[0][2]) < 0.012,
                f"min_z={aabb[0][2]:.4f}",
            )

    # --- left/right wheel symmetry ---
    lp = ctx.part_world_position(left)
    rp = ctx.part_world_position(right)
    if lp is not None and rp is not None:
        ctx.check(
            "wheels_symmetric_y",
            abs(lp[1] + rp[1]) < 0.01 and abs(lp[0] - rp[0]) < 0.01,
            f"lp={lp}, rp={rp}",
        )

    # --- lid closed: caps the body ---
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.50, name="lid_caps_body")
    ctx.allow_overlap(
        lid,
        body,
        reason="The lid skirt laps a few mm over the proud rim outer lip when closed.",
    )
    ctx.expect_contact(lid, body, contact_tol=0.012, name="lid_seated_on_rim")

    # --- lid opens upward ---
    rest_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.6}):
        open_aabb = ctx.part_world_aabb(lid)
    if rest_aabb is not None and open_aabb is not None:
        ctx.check(
            "lid_opens_upward",
            open_aabb[1][2] > rest_aabb[1][2] + 0.15,
            f"rest_top={rest_aabb[1][2]:.3f}, open_top={open_aabb[1][2]:.3f}",
        )
        ctx.check(
            "lid_swings_rearward",
            open_aabb[1][0] < rest_aabb[1][0],
            f"rest_maxx={rest_aabb[1][0]:.3f}, open_maxx={open_aabb[1][0]:.3f}",
        )

    # --- axle captured inside each wheel hub bore ---
    for w in (left, right):
        ctx.allow_overlap(
            body,
            w,
            elem_a="axle",
            elem_b="wheel",
            reason="The fixed axle bar is captured inside the wheel hub bore.",
        )
        ctx.expect_overlap(
            body,
            w,
            axes="y",
            elem_a="axle",
            elem_b="wheel",
            min_overlap=0.006,
            name=f"axle_engages_{w.name}",
        )

    return ctx.report()


object_model = build_object_model()
