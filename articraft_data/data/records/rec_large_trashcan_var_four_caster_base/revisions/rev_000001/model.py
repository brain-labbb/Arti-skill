from __future__ import annotations

# Four-caster mobile trash bin (Lavex-style ~240 L cart on four ground casters).
#
# Coordinate convention:
#   - up is +Z; casters touch the ground at z=0.
#   - the bin "front" (the flat labelled face) looks toward +X.
#   - the hinge / handle are at the rear (-X).
#
# Structure:
#   - body (root, static): tapered hollow plastic shell with thick top rim,
#     vertical reinforcing ribs, and four caster-mount boss pads at the bottom
#     corners; four caster fork+stem assemblies as inline body visuals.
#   - lid (REVOLUTE about +Y at the rear top rim): domed flip lid with grab lip.
#   - caster_0..caster_3 (CONTINUOUS about +Y): four caster wheels at corners,
#     emitted by a for loop over the corner position list with one shared wheel
#     geometry helper and a uniform joint policy.

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

# ─── key dimensions (meters) ───────────────────────────────────────
BODY_H = 0.940
TOP_W = 0.580
TOP_D = 0.610
BOT_W = 0.480
BOT_D = 0.520
WALL = 0.018
RIM_H = 0.045
CORNER_R = 0.045

# Caster dimensions
CASTER_DIA = 0.075
CASTER_R = CASTER_DIA / 2.0
CASTER_W = 0.028
AXLE_PIN_R = 0.005
STEM_R = 0.012
STEM_H = 0.030
FORK_TOP_ABOVE_AXLE = CASTER_R + 0.015
FORK_PLATE_T = 0.004
FORK_CLEARANCE = 0.003
FORK_HALF_SPAN = CASTER_W / 2.0 + FORK_CLEARANCE + FORK_PLATE_T
FORK_PLATE_DEPTH = CASTER_DIA * 0.45
BOSS_R = 0.022
BOSS_DROP = 0.025

# Body sits above the caster assemblies
BODY_BOTTOM_Z = CASTER_R + FORK_TOP_ABOVE_AXLE + STEM_H + 0.005
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_H

# Hinge at the rear top rim
HINGE_X = -TOP_D / 2.0 + 0.022
HINGE_Z = BODY_TOP_Z + 0.010

# Caster corner positions: list drives the count (for-i-in-range(n) pattern)
CASTER_INSET = 0.055
CASTER_CORNERS = [
    (+(BOT_D / 2.0 - CASTER_INSET), +(BOT_W / 2.0 - CASTER_INSET)),  # front-left
    (+(BOT_D / 2.0 - CASTER_INSET), -(BOT_W / 2.0 - CASTER_INSET)),  # front-right
    (-(BOT_D / 2.0 - CASTER_INSET), +(BOT_W / 2.0 - CASTER_INSET)),  # rear-left
    (-(BOT_D / 2.0 - CASTER_INSET), -(BOT_W / 2.0 - CASTER_INSET)),  # rear-right
]


# ─── geometry helpers ──────────────────────────────────────────────


def _rrect(dx: float, dy: float, r: float) -> cq.Sketch:
    """Centered rounded-rectangle sketch in workplane XY."""
    r = min(r, dx / 2.0 - 1e-4, dy / 2.0 - 1e-4)
    return cq.Sketch().rect(dx, dy).vertices().fillet(r)


def _build_body_mesh():
    """Tapered hollow bin shell with thick rim, ribs, and four caster-mount bosses."""
    # Outer tapered shell: loft from bottom to top rounded-rect sections.
    outer = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(BOT_D, BOT_W, CORNER_R),
            _rrect(TOP_D, TOP_W, CORNER_R).moved(cq.Location(cq.Vector(0, 0, BODY_H))),
        )
        .loft()
    )

    # Hollow interior (leaves floor and walls).
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

    # Thick reinforcing top rim band (proud of the wall on the outside).
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

    # Vertical reinforcing ribs proud of the front (+X) face.
    rib_w = 0.018
    rib_proud = 0.012
    rib_top = BODY_H - RIM_H - 0.014
    rib_bot = rib_top - 0.230
    rib_h = rib_top - rib_bot
    rib_cz = (rib_top + rib_bot) / 2.0
    frac = rib_cz / BODY_H
    face_x = (BOT_D + (TOP_D - BOT_D) * frac) / 2.0
    for yy in (-0.18, -0.06, 0.06, 0.18):
        rib = (
            cq.Workplane("XY")
            .center(face_x - 0.006, yy)
            .box(rib_proud + 0.010, rib_w, rib_h, centered=(False, True, True))
            .edges("|X")
            .fillet(0.004)
            .translate((0.0, 0.0, rib_cz))
        )
        shell = shell.union(rib)

    # Caster-mount boss pads at the four bottom corners (cylindrical protrusions
    # extending below the body floor to receive the caster stem).
    for cx, cy in CASTER_CORNERS:
        boss = (
            cq.Workplane("XY")
            .workplane(offset=-BOSS_DROP)
            .center(cx, cy)
            .circle(BOSS_R)
            .extrude(BOSS_DROP + 0.010)  # from -BOSS_DROP up into the floor
        )
        shell = shell.union(boss)

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

    # Down-turned skirt around the perimeter.
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

    # Front grab lip for lifting.
    lip = (
        cq.Workplane("XY")
        .center(lid_d / 2.0 - 0.006, 0.0)
        .box(0.034, 0.180, 0.032, centered=(True, True, False))
        .edges("|Y")
        .fillet(0.008)
        .translate((0, 0, -0.008))
    )
    lid = lid.union(lip)

    return mesh_from_cadquery(lid, "bin_lid", unit_scale=1.0)


def _build_caster_fork_mesh(name: str):
    """Caster fork + stem assembly. Origin at the wheel axle center."""
    plate_bottom = -CASTER_R * 0.4  # extends slightly below axle for retention
    plate_h = FORK_TOP_ABOVE_AXLE - plate_bottom

    # Left fork plate
    left_plate = (
        cq.Workplane("XY")
        .workplane(offset=plate_bottom)
        .center(0, FORK_HALF_SPAN)
        .rect(FORK_PLATE_DEPTH, FORK_PLATE_T)
        .extrude(plate_h)
    )
    # Right fork plate
    right_plate = (
        cq.Workplane("XY")
        .workplane(offset=plate_bottom)
        .center(0, -FORK_HALF_SPAN)
        .rect(FORK_PLATE_DEPTH, FORK_PLATE_T)
        .extrude(plate_h)
    )
    # Crossbar connecting the plates at the top
    crossbar_span = 2 * FORK_HALF_SPAN + FORK_PLATE_T
    crossbar = (
        cq.Workplane("XY")
        .workplane(offset=FORK_TOP_ABOVE_AXLE - 0.008)
        .rect(FORK_PLATE_DEPTH, crossbar_span)
        .extrude(0.014)
    )
    # Stem cylinder going up from the crossbar into the body boss
    stem = (
        cq.Workplane("XY")
        .workplane(offset=FORK_TOP_ABOVE_AXLE - 0.002)
        .circle(STEM_R)
        .extrude(STEM_H + 0.002)
    )

    # Axle pin: cylinder along Y spanning between the fork plates, capturing
    # the wheel bore so the caster is physically supported by the fork.
    axle_pin = (
        cq.Workplane("XY")
        .circle(AXLE_PIN_R)
        .extrude(2 * FORK_HALF_SPAN)
        .translate((0, 0, -FORK_HALF_SPAN))
        .rotate((0, 0, 0), (1, 0, 0), -90)  # align Z cylinder to Y axis
    )

    fork = left_plate.union(right_plate).union(crossbar).union(stem).union(axle_pin)
    # Fillet the bottom edges of the fork plates for a caster-like rounded look.
    try:
        fork = fork.edges("|Z and >Y").fillet(0.002)
    except Exception:
        pass  # fillet is cosmetic; skip if topology rejects it

    return mesh_from_cadquery(fork, name, unit_scale=1.0)


def _caster_wheel_mesh(name: str):
    """Small caster wheel using the shared WheelGeometry helper."""
    wheel = WheelGeometry(
        CASTER_R - 0.010,
        CASTER_W - 0.008,
        rim=WheelRim(
            inner_radius=CASTER_R - 0.020, flange_height=0.004, flange_thickness=0.003
        ),
        hub=WheelHub(radius=0.012, width=0.018, cap_style="domed"),
        face=WheelFace(dish_depth=0.004, front_inset=0.002),
        spokes=WheelSpokes(
            style="split_y", count=5, thickness=0.003, window_radius=0.006
        ),
        bore=WheelBore(style="round", diameter=2 * AXLE_PIN_R - 0.004),
    )
    return mesh_from_geometry(wheel, name)


def _caster_tire_mesh(name: str):
    """Small caster tire using TireGeometry."""
    tire = TireGeometry(
        CASTER_R,
        CASTER_W,
        inner_radius=CASTER_R - 0.010,
        tread=TireTread(style="block", depth=0.003, count=16, land_ratio=0.6),
        sidewall=TireSidewall(style="rounded", bulge=0.02),
    )
    return mesh_from_geometry(tire, name)


# ─── build ─────────────────────────────────────────────────────────


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_caster_trash_bin")

    # Materials
    body_gray = model.material("bin_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    lid_gray = model.material("lid_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    fork_steel = model.material("fork_steel", rgba=(0.52, 0.53, 0.55, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.12, 0.12, 0.13, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))

    # ── body (root) ──
    body = model.part("body")
    body.visual(
        _build_body_mesh(),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z)),
        material=body_gray,
        name="shell",
    )

    # Caster fork+stem assemblies: inline body visuals, one per corner
    n_casters = len(CASTER_CORNERS)
    for i in range(n_casters):
        cx, cy = CASTER_CORNERS[i]
        body.visual(
            _build_caster_fork_mesh(f"caster_mount_{i}"),
            origin=Origin(xyz=(cx, cy, CASTER_R)),
            material=fork_steel,
            name=f"caster_mount_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((TOP_D, TOP_W, BODY_H)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_H / 2.0)),
    )

    # ── lid ──
    lid = model.part("lid")
    lid.visual(
        _build_lid_mesh(),
        origin=Origin(xyz=((TOP_D + 0.018) / 2.0 - 0.022, 0.0, 0.0)),
        material=lid_gray,
        name="lid_plate",
    )
    lid.inertial = Inertial.from_geometry(Box((TOP_D, TOP_W, 0.04)), mass=2.4)

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # Lid plate extends along local +X from the hinge; -Y lifts the free
        # front edge up and back as q increases.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=1.95),
    )

    # ── caster wheels (for loop over corner position list) ──
    for i in range(n_casters):
        cx, cy = CASTER_CORNERS[i]
        caster_name = f"caster_{i}"

        caster = model.part(caster_name)
        # WheelGeometry spins about local X; rotate so spin axis aligns with
        # world Y, matching the uniform joint axis convention.
        caster.visual(
            _caster_wheel_mesh(f"{caster_name}_wheel"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -math.pi / 2.0)),
            material=wheel_black,
            name="wheel",
        )
        caster.visual(
            _caster_tire_mesh(f"{caster_name}_tire"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, -math.pi / 2.0)),
            material=tire_black,
            name="tire",
        )
        caster.inertial = Inertial.from_geometry(
            Cylinder(radius=CASTER_R, length=CASTER_W), mass=0.3
        )

        # Uniform continuous-spin joint policy for every caster
        model.articulation(
            f"{caster_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=caster,
            origin=Origin(xyz=(cx, cy, CASTER_R)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=15.0),
        )

    return model


# ─── tests ─────────────────────────────────────────────────────────


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    n_casters = len(CASTER_CORNERS)
    casters = [object_model.get_part(f"caster_{i}") for i in range(n_casters)]
    caster_spins = [
        object_model.get_articulation(f"caster_{i}_spin") for i in range(n_casters)
    ]

    # ── hero parts present ──
    ctx.check("has_body", body is not None, "Expected a body part.")
    ctx.check("has_lid", lid is not None, "Expected a lid part.")
    ctx.check(
        "has_four_casters",
        all(c is not None for c in casters) and len(casters) == 4,
        f"Expected 4 casters, got {sum(1 for c in casters if c is not None)}.",
    )

    # ── lid hinge: revolute, Y axis ──
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

    # ── caster joints: all CONTINUOUS, all Y axis, uniform policy ──
    for i, spin in enumerate(caster_spins):
        ctx.check(
            f"caster_{i}_is_continuous",
            str(spin.articulation_type).endswith("CONTINUOUS"),
            f"type={spin.articulation_type}",
        )
        ctx.check(
            f"caster_{i}_axis_horizontal_y",
            abs(abs(spin.axis[1]) - 1.0) < 1e-6,
            f"axis={spin.axis}",
        )

    # ── all four casters touch the ground (z ≈ 0) ──
    for i, c in enumerate(casters):
        aabb = ctx.part_world_aabb(c)
        if aabb is not None:
            ctx.check(
                f"caster_{i}_on_ground",
                abs(aabb[0][2]) < 0.012,
                f"min_z={aabb[0][2]:.4f}",
            )

    # ── casters span four distinct corners (X and Y spread) ──
    positions = [ctx.part_world_position(c) for c in casters]
    if all(p is not None for p in positions):
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        ctx.check(
            "casters_span_corners_x",
            max(xs) - min(xs) > 0.20,
            f"x_range={max(xs) - min(xs):.3f}",
        )
        ctx.check(
            "casters_span_corners_y",
            max(ys) - min(ys) > 0.20,
            f"y_range={max(ys) - min(ys):.3f}",
        )
        # Front/rear pair should share similar X; left/right share similar Y
        ctx.check(
            "casters_front_rear_split",
            max(xs) > 0.05 and min(xs) < -0.05,
            f"min_x={min(xs):.3f}, max_x={max(xs):.3f}",
        )

    # ── body scale sanity ──
    full = ctx.part_world_aabb(body)
    if full is not None:
        size = tuple(full[1][i] - full[0][i] for i in range(3))
        ctx.check("body_height", 0.85 <= size[2] <= 1.20, f"size={size!r}")
        ctx.check("body_width", 0.50 <= size[1] <= 0.70, f"size={size!r}")

    # ── lid closed: caps the body ──
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.30, name="lid_caps_body")
    ctx.allow_overlap(
        lid,
        body,
        reason="The lid skirt laps a few mm over the proud rim outer lip when closed.",
    )
    ctx.expect_contact(lid, body, contact_tol=0.012, name="lid_seated_on_rim")

    # ── lid opens upward: free front edge rises when hinge opens ──
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

    # ── caster axle pin captured in wheel bore (intentional local overlap) ──
    for i, c in enumerate(casters):
        ctx.allow_overlap(
            body,
            c,
            elem_a=f"caster_mount_{i}",
            elem_b="wheel",
            reason="The fork axle pin is captured inside the wheel hub bore.",
        )
        # Prove the axle pin actually reaches into the wheel (retained capture).
        ctx.expect_overlap(
            body,
            c,
            axes="y",
            elem_a=f"caster_mount_{i}",
            elem_b="wheel",
            min_overlap=0.010,
            name=f"axle_pin_engages_caster_{i}",
        )

    # ── caster count driven by the position list length ──
    ctx.check(
        "caster_count_matches_corner_list",
        len(casters) == len(CASTER_CORNERS),
        f"parts={len(casters)}, positions={len(CASTER_CORNERS)}",
    )

    return ctx.report()


object_model = build_object_model()
