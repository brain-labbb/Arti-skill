from __future__ import annotations

# Gray two-wheel curbside wheelie trash bin (Lavex-style ~240 L cart)
# with SPLIT LID: two independent half-lids that meet at the centerline.
#
# Coordinate convention:
#   - up is +Z; the wheels touch the ground at z=0.
#   - the bin "front" (the flat labelled face) looks toward +X.
#   - the hinge / handle / wheels are at the rear (-X).
#
# Structure:
#   - body (root, static): tapered hollow plastic shell, slightly wider at the
#     top than the bottom, with a thick top rim, vertical reinforcing ribs near
#     the upper sides, the molded wheel-axle housing at the rear-bottom, and a
#     fixed steel axle bar across the back.
#   - half_lid_0 (left, REVOLUTE about +Y at the rear top rim): left half-lid
#     covering the +Y half of the opening; opens upward/rearward.
#   - half_lid_1 (right, REVOLUTE about +Y at the rear top rim): right half-lid
#     covering the -Y half of the opening; opens upward/rearward.
#   - left_wheel / right_wheel (CONTINUOUS about +Y): the two molded plastic
#     road wheels on the rear axle.

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

# --- key dimensions (meters) ---
BODY_H = 0.940  # height of the molded body shell (top rim above the floor)
TOP_W = 0.580  # outside width (Y) at the top rim
TOP_D = 0.610  # outside depth (X) at the top rim
BOT_W = 0.480  # outside width (Y) at the bottom (tapered in)
BOT_D = 0.520  # outside depth (X) at the bottom
WALL = 0.018  # plastic wall thickness
RIM_H = 0.045  # thick reinforcing top rim band
CORNER_R = 0.045  # exterior corner radius

WHEEL_DIA = 0.200
WHEEL_R = WHEEL_DIA / 2.0
WHEEL_W = 0.050
AXLE_R = 0.012  # fills the wheel bore so the axle is captured (no float gap)
WHEEL_GAP = 0.006  # axle/wheel sits just outboard of the body wall

# Body sits so its bottom is just above the wheel contact patch.
BODY_BOTTOM_Z = WHEEL_DIA - 0.040  # bottom of the shell clears the floor a bit
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_H

# Hinge axis is along the rear top edge of the rim.
HINGE_X = -TOP_D / 2.0 + 0.022
HINGE_Z = BODY_TOP_Z + 0.010  # lid plate rests just above the rim top

# Axle is at the rear-bottom of the body.
AXLE_X = -BOT_D / 2.0 + 0.020
AXLE_Z = WHEEL_R  # world height of the axle center
AXLE_Z_LOCAL = AXLE_Z - BODY_BOTTOM_Z  # axle height in the body-mesh local frame
HALF_AXLE_Y = BOT_W / 2.0 + WHEEL_GAP + WHEEL_W / 2.0

# --- split lid dimensions ---
LID_D = TOP_D + 0.018  # full lid depth (same for each half)
CENTER_GAP = 0.008  # gap between the two half-lids at the centerline
HALF_LID_W = (TOP_W + 0.018) / 2.0 - CENTER_GAP / 2.0  # width of each half-lid
HINGE_Y_OFFSET = HALF_LID_W / 2.0 + CENTER_GAP / 2.0  # hinge Y from centerline

# Number of half-lid segments driven by the sign list.
HALF_LID_SIGNS = (+1.0, -1.0)
N_HALF_LIDS = len(HALF_LID_SIGNS)


def _rrect(dx: float, dy: float, r: float) -> cq.Sketch:
    """Centered rounded-rectangle sketch in the workplane XY."""
    r = min(r, dx / 2.0 - 1e-4, dy / 2.0 - 1e-4)
    return cq.Sketch().rect(dx, dy).vertices().fillet(r)


def _build_body_mesh():
    """Tapered hollow bin shell with thick rim, ribs and the rear axle housing."""
    # Outer tapered shell: loft between rounded-rect bottom and top sketches so
    # the vertical corners are already rounded (no fragile edge fillet needed).
    outer = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(BOT_D, BOT_W, CORNER_R),
            _rrect(TOP_D, TOP_W, CORNER_R).moved(cq.Location(cq.Vector(0, 0, BODY_H))),
        )
        .loft()
    )

    # Hollow it out: subtract an inner loft, leaving the floor and walls.
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

    # Thick reinforcing top rim band (slightly proud of the wall on the outside).
    rim = (
        cq.Workplane("XY")
        .placeSketch(_rrect(TOP_D + 0.014, TOP_W + 0.014, CORNER_R).moved(
            cq.Location(cq.Vector(0, 0, BODY_H - RIM_H))
        ))
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

    # Vertical reinforcing ribs proud of the upper front face, spread across Y.
    # These sit on the OUTSIDE of the front (+X) wall only; they must never
    # protrude into the hollow cavity.
    rib_w = 0.018
    rib_proud = 0.012
    rib_top = BODY_H - RIM_H - 0.014
    rib_bot = rib_top - 0.230
    rib_h = rib_top - rib_bot
    rib_cz = (rib_top + rib_bot) / 2.0
    # Front-face X at the rib-band mid height (taper interpolation).
    frac = rib_cz / BODY_H
    face_x = (BOT_D + (TOP_D - BOT_D) * frac) / 2.0
    for yy in (-0.18, -0.06, 0.06, 0.18):
        # Box grows from just inside the wall (face_x - 0.006) outward in +X,
        # so it laps onto the wall and stands proud of the front face.
        rib = (
            cq.Workplane("XY")
            .center(face_x - 0.006, yy)
            .box(rib_proud + 0.010, rib_w, rib_h, centered=(False, True, True))
            .edges("|X")
            .fillet(0.004)
            .translate((0.0, 0.0, rib_cz))
        )
        shell = shell.union(rib)

    # Molded rear axle support: a leg block descending from the shell bottom
    # (local z=0) down past the axle center (local AXLE_Z_LOCAL < 0), spanning
    # the rear width so the axle is captured in it. This connects shell to axle.
    leg_top = 0.030  # overlaps up into the shell bottom
    leg_bot = AXLE_Z_LOCAL - 0.018  # a bit below the axle
    housing = (
        cq.Workplane("XY")
        .center(AXLE_X + 0.018, 0.0)
        .box(
            0.085,
            BOT_W - 0.060,  # stays inboard of the wheels
            leg_top - leg_bot,
            centered=(True, True, False),
        )
        .edges("|Y")
        .fillet(0.018)
        .translate((0, 0, leg_bot))
    )
    shell = shell.union(housing)

    return mesh_from_cadquery(shell, "bin_body", unit_scale=1.0)


def _build_half_lid_mesh(mesh_name: str, outer_sign: float = 1.0) -> object:
    """Shared half-lid geometry helper.

    Builds one half-width flip lid panel with a front grab lip and a 3-sided
    dropped skirt (front, rear, and outer side only — no skirt on the
    centerline edge). The hinge edge is at local X ≈ -LID_D/2, and the plate
    extends toward +X.

    Args:
        mesh_name: logical mesh asset name.
        outer_sign: +1.0 places the outer skirt at +Y (left half);
                    -1.0 places it at -Y (right half).
    """
    plate_t = 0.022
    cr = max(CORNER_R - 0.006, 0.006)

    # Main plate: a rounded-corner half-width slab.
    plate = (
        cq.Workplane("XY")
        .placeSketch(_rrect(LID_D, HALF_LID_W, cr))
        .extrude(plate_t)
    )

    # Shallow raised crown on the inner region.
    crown = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(LID_D - 0.10, HALF_LID_W - 0.06, max(cr - 0.008, 0.004)).moved(
                cq.Location(cq.Vector(0, 0, plate_t))
            )
        )
        .extrude(0.010)
    )
    lid = plate.union(crown)

    # Dropped skirt bars on 3 sides only (no inner/centerline side).
    skirt_drop = 0.014
    skirt_thick = 0.010

    # Front skirt bar (at +X edge).
    front_skirt = (
        cq.Workplane("XY")
        .center(LID_D / 2.0 - skirt_thick / 2.0, 0.0)
        .box(skirt_thick, HALF_LID_W - 2 * skirt_thick, skirt_drop, centered=(True, True, False))
        .translate((0, 0, -skirt_drop))
    )
    # Rear skirt bar (at -X edge, near hinge).
    rear_skirt = (
        cq.Workplane("XY")
        .center(-LID_D / 2.0 + skirt_thick / 2.0, 0.0)
        .box(skirt_thick, HALF_LID_W - 2 * skirt_thick, skirt_drop, centered=(True, True, False))
        .translate((0, 0, -skirt_drop))
    )
    # Outer side skirt bar (at outer_sign * +Y or -Y edge).
    outer_y = outer_sign * (HALF_LID_W / 2.0 - skirt_thick / 2.0)
    outer_skirt = (
        cq.Workplane("XY")
        .center(0.0, outer_y)
        .box(LID_D - 2 * skirt_thick, skirt_thick, skirt_drop, centered=(True, True, False))
        .translate((0, 0, -skirt_drop))
    )
    skirt = front_skirt.union(rear_skirt).union(outer_skirt)
    lid = lid.union(skirt)

    # Front grab lip (overhang at the +X front edge for lifting).
    lip = (
        cq.Workplane("XY")
        .center(LID_D / 2.0 - 0.006, 0.0)
        .box(0.030, min(0.120, HALF_LID_W - 0.040), 0.028, centered=(True, True, False))
        .edges("|Y")
        .fillet(0.006)
        .translate((0, 0, -0.006))
    )
    lid = lid.union(lip)

    return mesh_from_cadquery(lid, mesh_name, unit_scale=1.0)


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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_trash_bin_split_lid")

    body_gray = model.material("bin_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    lid_left_mat = model.material("lid_left_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    lid_right_mat = model.material("lid_right_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.12, 0.12, 0.13, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("axle_steel", rgba=(0.55, 0.56, 0.58, 1.0))

    # --- body (root) ---
    body = model.part("body")
    body.visual(
        _build_body_mesh(),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z)),
        material=body_gray,
        name="shell",
    )
    # Fixed steel axle bar running across the rear-bottom housing.
    body.visual(
        Cylinder(radius=AXLE_R, length=2 * HALF_AXLE_Y - 0.010),
        origin=Origin(xyz=(AXLE_X, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    body.inertial = Inertial.from_geometry(
        Box((TOP_D, TOP_W, BODY_H)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_H / 2.0)),
    )

    # --- split half-lids (generated by for loop over the sign list) ---
    lid_materials = (lid_left_mat, lid_right_mat)

    for i in range(N_HALF_LIDS):
        sign = HALF_LID_SIGNS[i]
        lid_name = f"half_lid_{i}"
        half_lid = model.part(lid_name)

        # Each half-lid uses the shared geometry helper with its outer_sign.
        half_lid.visual(
            _build_half_lid_mesh(f"{lid_name}_plate", outer_sign=sign),
            # Offset the mesh so the hinge edge (rear) sits at the part origin.
            origin=Origin(xyz=(LID_D / 2.0 - 0.022, 0.0, 0.0)),
            material=lid_materials[i],
            name=f"{lid_name}_plate",
        )
        half_lid.inertial = Inertial.from_geometry(
            Box((LID_D, HALF_LID_W, 0.04)), mass=1.2
        )

        # Uniform revolute hinge policy for every half-lid segment.
        model.articulation(
            f"{lid_name}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=half_lid,
            origin=Origin(xyz=(HINGE_X, sign * HINGE_Y_OFFSET, HINGE_Z)),
            # Plate extends along local +X from the hinge; -Y lifts the free
            # front edge up and back as q increases.
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0, velocity=2.0, lower=0.0, upper=1.95
            ),
        )

    # --- wheels ---
    for name, sign in (("left_wheel", 1.0), ("right_wheel", -1.0)):
        wheel = model.part(name)
        # WheelGeometry/TireGeometry spin about local X; rotate so the spin axis
        # aligns with world +Y, and the outer face points outboard.
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


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_spin = object_model.get_articulation("left_wheel_spin")

    # Collect the half-lid parts and hinges from the for-loop-generated names.
    half_lids = []
    half_lid_hinges = []
    for i in range(N_HALF_LIDS):
        lid_name = f"half_lid_{i}"
        half_lids.append(object_model.get_part(lid_name))
        half_lid_hinges.append(object_model.get_articulation(f"{lid_name}_hinge"))

    # --- hero parts present ---
    ctx.check("has_body", body is not None, "Expected a body part.")
    ctx.check(
        "has_two_half_lids",
        len(half_lids) == N_HALF_LIDS and all(p is not None for p in half_lids),
        f"Expected {N_HALF_LIDS} half-lid parts.",
    )
    ctx.check(
        "has_wheels",
        left_wheel is not None and right_wheel is not None,
        "Expected two wheels.",
    )

    # --- joint types and axes for every half-lid hinge ---
    for i, hinge in enumerate(half_lid_hinges):
        ctx.check(
            f"half_lid_{i}_is_revolute",
            str(hinge.articulation_type).endswith("REVOLUTE"),
            f"type={hinge.articulation_type}",
        )
        ctx.check(
            f"half_lid_{i}_axis_y",
            abs(abs(hinge.axis[1]) - 1.0) < 1e-6
            and abs(hinge.axis[0]) < 1e-6
            and abs(hinge.axis[2]) < 1e-6,
            f"axis={hinge.axis}",
        )

    # --- wheel joint checks ---
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

    # --- wheels touch the ground (z approx 0) ---
    for w in (left_wheel, right_wheel):
        aabb = ctx.part_world_aabb(w)
        if aabb is not None:
            ctx.check(
                f"{w.name}_on_ground",
                abs(aabb[0][2]) < 0.012,
                f"min_z={aabb[0][2]:.4f}",
            )

    # --- overall scale sanity ---
    full = ctx.part_world_aabb(body)
    if full is not None:
        size = tuple(full[1][i] - full[0][i] for i in range(3))
        ctx.check("body_height", 0.85 <= size[2] <= 1.15, f"size={size!r}")
        ctx.check("body_width", 0.50 <= size[1] <= 0.70, f"size={size!r}")

    # --- left/right wheel symmetry across Y ---
    lp = ctx.part_world_position(left_wheel)
    rp = ctx.part_world_position(right_wheel)
    if lp is not None and rp is not None:
        ctx.check(
            "wheels_symmetric_y",
            abs(lp[1] + rp[1]) < 0.01 and abs(lp[0] - rp[0]) < 0.01,
            f"lp={lp}, rp={rp}",
        )

    # --- each half-lid closed: caps its half of the body opening ---
    for i, half_lid in enumerate(half_lids):
        ctx.expect_overlap(
            half_lid, body, axes="xy", min_overlap=0.10,
            name=f"half_lid_{i}_caps_body",
        )
        # The skirt laps a few mm over the proud rim outer lip when closed.
        ctx.allow_overlap(
            half_lid,
            body,
            reason=f"Half-lid {i} skirt laps over the proud rim outer lip when closed.",
        )
        ctx.expect_contact(
            half_lid, body, contact_tol=0.012,
            name=f"half_lid_{i}_seated_on_rim",
        )

    # --- the two half-lids together span the full body width ---
    ctx.expect_overlap(
        half_lids[0], half_lids[1], axes="x", min_overlap=0.20,
        name="half_lids_overlap_in_depth",
    )

    # --- each half-lid opens upward independently ---
    for i, (half_lid, hinge) in enumerate(zip(half_lids, half_lid_hinges)):
        rest_aabb = ctx.part_world_aabb(half_lid)
        with ctx.pose({hinge: 1.6}):
            open_aabb = ctx.part_world_aabb(half_lid)
        if rest_aabb is not None and open_aabb is not None:
            ctx.check(
                f"half_lid_{i}_opens_upward",
                open_aabb[1][2] > rest_aabb[1][2] + 0.15,
                f"rest_top={rest_aabb[1][2]:.3f}, open_top={open_aabb[1][2]:.3f}",
            )
            ctx.check(
                f"half_lid_{i}_swings_rearward",
                open_aabb[1][0] < rest_aabb[1][0],
                f"rest_maxx={rest_aabb[1][0]:.3f}, open_maxx={open_aabb[1][0]:.3f}",
            )

    # --- independent opening: open only lid 0, lid 1 stays closed ---
    rest_aabb_1 = ctx.part_world_aabb(half_lids[1])
    with ctx.pose({half_lid_hinges[0]: 1.4}):
        open_aabb_0 = ctx.part_world_aabb(half_lids[0])
        still_aabb_1 = ctx.part_world_aabb(half_lids[1])
    if rest_aabb_1 is not None and still_aabb_1 is not None and open_aabb_0 is not None:
        ctx.check(
            "half_lid_1_stays_closed_when_0_opens",
            abs(still_aabb_1[1][2] - rest_aabb_1[1][2]) < 0.005,
            f"rest_top={rest_aabb_1[1][2]:.4f}, still_top={still_aabb_1[1][2]:.4f}",
        )
        ctx.check(
            "half_lid_0_actually_opens_alone",
            open_aabb_0[1][2] > rest_aabb_1[1][2] + 0.10,
            f"lid0_open_top={open_aabb_0[1][2]:.3f}",
        )

    # --- the two half-lids are on opposite sides of centerline (Y symmetry) ---
    pos_0 = ctx.part_world_position(half_lids[0])
    pos_1 = ctx.part_world_position(half_lids[1])
    if pos_0 is not None and pos_1 is not None:
        ctx.check(
            "half_lids_symmetric_y",
            abs(pos_0[1] + pos_1[1]) < 0.02,
            f"lid0_y={pos_0[1]:.4f}, lid1_y={pos_1[1]:.4f}",
        )
        ctx.check(
            "half_lids_on_opposite_sides",
            pos_0[1] > 0.0 and pos_1[1] < 0.0,
            f"lid0_y={pos_0[1]:.4f}, lid1_y={pos_1[1]:.4f}",
        )

    # The axle pin is intentionally captured inside each wheel hub bore.
    for w in (left_wheel, right_wheel):
        ctx.allow_overlap(
            body,
            w,
            elem_a="axle",
            elem_b="wheel",
            reason="The fixed axle bar is captured inside the wheel hub bore.",
        )
        # Prove the axle actually reaches into each wheel hub (retained capture).
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
