from __future__ import annotations

# Gray two-wheel curbside wheelie trash bin (Lavex-style ~240 L cart).
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
#   - lid (REVOLUTE about +Y at the rear top rim): a slightly domed flip lid
#     with a front grab lip; opens upward/rearward.
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

# --- front trunnion bar / DIN comb-lift interface ---
TRUNNION_R = 0.014          # 28 mm diameter cross bar
TRUNNION_LENGTH = 0.38      # spans across the front face
GUSSET_T = 0.008            # gusset plate thickness (Y)
GUSSET_H = 0.12             # gusset plate height (Z)
GUSSET_D = 0.055            # gusset depth, proud of face (X)
GUSSET_SPACING = 0.26       # centre-to-centre Y spacing

# Bar height: ~10 cm below the top rim band, on the front (+X) face.
BAR_Z_LOCAL = BODY_H - RIM_H - 0.10
BAR_Z_WORLD = BODY_BOTTOM_Z + BAR_Z_LOCAL
_BAR_FRAC = BAR_Z_LOCAL / BODY_H
FRONT_FACE_X_AT_BAR = (BOT_D + (TOP_D - BOT_D) * _BAR_FRAC) / 2.0
# Bar centre sits near the outer tip of the gussets so the truck arm can hook it.
BAR_CENTER_X = FRONT_FACE_X_AT_BAR + GUSSET_D - TRUNNION_R


def _rounded_rect_wire(wp: cq.Workplane, dx: float, dy: float, r: float) -> cq.Workplane:
    return wp.rect(dx, dy).val()


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


def _build_gusset_mesh(name: str):
    """Trapezoidal gusset plate for the front trunnion bar.

    The plate lies in the XZ plane (thin along Y).  Local origin is at
    the plate geometric centre.  Profile is full height at the mounting
    face (x = -GUSSET_D/2) and tapers to ~60 % at the outer edge where
    the trunnion bar passes through.
    """
    h = GUSSET_H
    d = GUSSET_D
    plate = (
        cq.Workplane("XZ")
        .moveTo(-d / 2, -h / 2)
        .lineTo(d / 2, -h * 0.30)
        .lineTo(d / 2, h * 0.30)
        .lineTo(-d / 2, h / 2)
        .close()
        .extrude(GUSSET_T)
    )
    # Centre in Y so the plate straddles its placement origin.
    # (CadQuery "XZ" workplane extrudes in −Y; shift +Y/2 to centre.)
    plate = plate.translate((0, GUSSET_T / 2, 0))
    # Slight edge break so the plate reads as manufactured steel.
    plate = plate.edges("|Y").fillet(0.003)
    return mesh_from_cadquery(plate, name, unit_scale=1.0)


def _build_lid_mesh():
    """Slightly domed flip lid with a front grab lip, hinged at the rear edge."""
    lid_d = TOP_D + 0.018
    lid_w = TOP_W + 0.018
    plate_t = 0.022
    # Main plate: a rounded-corner slab; the top is gently crowned by adding a
    # shallow second slab on top.
    plate = (
        cq.Workplane("XY")
        .placeSketch(_rrect(lid_d, lid_w, CORNER_R + 0.004))
        .extrude(plate_t)
    )
    crown = (
        cq.Workplane("XY")
        .placeSketch(_rrect(lid_d - 0.10, lid_w - 0.10, CORNER_R).moved(
            cq.Location(cq.Vector(0, 0, plate_t))
        ))
        .extrude(0.012)
    )
    lid = plate.union(crown)

    # Down-turned skirt around the perimeter so the lid laps just over the rim
    # outer lip. Kept shallow so it only grazes the proud rim edge.
    skirt_drop = 0.014
    skirt_outer = (
        cq.Workplane("XY")
        .placeSketch(_rrect(lid_d, lid_w, CORNER_R + 0.004).moved(
            cq.Location(cq.Vector(0, 0, -skirt_drop))
        ))
        .extrude(skirt_drop)
    )
    skirt_inner = (
        cq.Workplane("XY")
        .placeSketch(_rrect(lid_d - 0.024, lid_w - 0.024, CORNER_R).moved(
            cq.Location(cq.Vector(0, 0, -skirt_drop - 0.002))
        ))
        .extrude(skirt_drop + 0.004)
    )
    skirt = skirt_outer.cut(skirt_inner)
    lid = lid.union(skirt)

    # Front grab lip (overhang at the +X front edge for lifting).
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
    model = ArticulatedObject(name="wheelie_trash_bin")

    body_gray = model.material("bin_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    lid_gray = model.material("lid_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.12, 0.12, 0.13, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("axle_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    trunnion_steel = model.material("trunnion_steel", rgba=(0.48, 0.49, 0.52, 1.0))

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

    # --- front trunnion bar / DIN comb-lift interface ---
    # Horizontal cylindrical cross bar standing proud of the front (+X) face
    # on two trapezoidal gusset tie-plates.
    body.visual(
        Cylinder(radius=TRUNNION_R, length=TRUNNION_LENGTH),
        origin=Origin(xyz=(BAR_CENTER_X, 0.0, BAR_Z_WORLD), rpy=(math.pi / 2, 0, 0)),
        material=trunnion_steel,
        name="trunnion_bar",
    )
    # Two gusset plates, symmetric about Y=0, each slightly lapping into
    # the shell wall so the mesh reads as welded to the front face.
    gusset_lap = 0.004  # small embed into the shell for visual seating
    for i in range(2):
        sign = 1.0 if i == 0 else -1.0
        gx = FRONT_FACE_X_AT_BAR + GUSSET_D / 2.0 - gusset_lap
        gy = sign * GUSSET_SPACING / 2.0
        body.visual(
            _build_gusset_mesh(f"gusset_{i}"),
            origin=Origin(xyz=(gx, gy, BAR_Z_WORLD)),
            material=trunnion_steel,
            name=f"gusset_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((TOP_D, TOP_W, BODY_H)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_H / 2.0)),
    )

    # --- lid ---
    lid = model.part("lid")
    # Lid mesh frame: hinge edge at local origin, plate extends toward +X.
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
    lid = object_model.get_part("lid")
    left = object_model.get_part("left_wheel")
    right = object_model.get_part("right_wheel")
    hinge = object_model.get_articulation("lid_hinge")
    left_spin = object_model.get_articulation("left_wheel_spin")

    # --- hero parts present ---
    ctx.check("has_body", body is not None, "Expected a body part.")
    ctx.check("has_lid", lid is not None, "Expected a lid part.")
    ctx.check("has_wheels", left is not None and right is not None, "Expected two wheels.")

    # --- joint types/axes ---
    ctx.check(
        "lid_is_revolute",
        str(hinge.articulation_type).endswith("REVOLUTE"),
        f"type={hinge.articulation_type}",
    )
    ctx.check(
        "lid_axis_y",
        abs(abs(hinge.axis[1]) - 1.0) < 1e-6 and abs(hinge.axis[0]) < 1e-6 and abs(hinge.axis[2]) < 1e-6,
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

    # --- wheels touch the ground (z approx 0) ---
    for w in (left, right):
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
    lp = ctx.part_world_position(left)
    rp = ctx.part_world_position(right)
    if lp is not None and rp is not None:
        ctx.check(
            "wheels_symmetric_y",
            abs(lp[1] + rp[1]) < 0.01 and abs(lp[0] - rp[0]) < 0.01,
            f"lp={lp}, rp={rp}",
        )

    # --- lid closed: caps the body, footprint covers the rim ---
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.30, name="lid_caps_body")
    # The lid plate sits right at the rim top; its skirt laps a few mm over the
    # proud rim lip. Allow that shallow intentional nesting and prove contact.
    ctx.allow_overlap(
        lid,
        body,
        reason="The lid skirt laps a few mm over the proud rim outer lip when closed.",
    )
    ctx.expect_contact(lid, body, contact_tol=0.012, name="lid_seated_on_rim")

    # --- lid opens upward: free front edge rises when the hinge opens ---
    rest_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.6}):
        open_aabb = ctx.part_world_aabb(lid)
    if rest_aabb is not None and open_aabb is not None:
        ctx.check(
            "lid_opens_upward",
            open_aabb[1][2] > rest_aabb[1][2] + 0.15,
            f"rest_top={rest_aabb[1][2]:.3f}, open_top={open_aabb[1][2]:.3f}",
        )
        # When open, the lid swings rearward (its front edge moves toward -X).
        ctx.check(
            "lid_swings_rearward",
            open_aabb[1][0] < rest_aabb[1][0],
            f"rest_maxx={rest_aabb[1][0]:.3f}, open_maxx={open_aabb[1][0]:.3f}",
        )

    # --- trunnion bar / comb-lift interface ---
    bar_aabb = ctx.part_element_world_aabb(body, elem="trunnion_bar")
    ctx.check(
        "has_trunnion_bar",
        bar_aabb is not None,
        "Expected a trunnion_bar visual on the body.",
    )
    if bar_aabb is not None:
        bar_size = tuple(bar_aabb[1][i] - bar_aabb[0][i] for i in range(3))
        bar_cx = (bar_aabb[0][0] + bar_aabb[1][0]) / 2.0
        bar_cz = (bar_aabb[0][2] + bar_aabb[1][2]) / 2.0
        ctx.check(
            "trunnion_on_front_face",
            bar_aabb[1][0] > 0.20,
            f"bar max x = {bar_aabb[1][0]:.3f}",
        )
        ctx.check(
            "trunnion_above_midheight",
            bar_cz > BODY_BOTTOM_Z + BODY_H * 0.5,
            f"bar center z = {bar_cz:.3f}",
        )
        ctx.check(
            "trunnion_horizontal_span",
            bar_size[1] > 0.30 and bar_size[1] > bar_size[2] * 5,
            f"bar size = {bar_size!r}",
        )

    # Two gusset plates, symmetric about Y=0
    g0 = ctx.part_element_world_aabb(body, elem="gusset_0")
    g1 = ctx.part_element_world_aabb(body, elem="gusset_1")
    ctx.check(
        "has_two_gussets",
        g0 is not None and g1 is not None,
        "Expected two gusset plate visuals on the body.",
    )
    if g0 is not None and g1 is not None:
        g0_cy = (g0[0][1] + g0[1][1]) / 2.0
        g1_cy = (g1[0][1] + g1[1][1]) / 2.0
        ctx.check(
            "gussets_symmetric_y",
            abs(g0_cy + g1_cy) < 0.01,
            f"g0_cy={g0_cy:.3f}, g1_cy={g1_cy:.3f}",
        )
        # Gussets stand proud of the front face
        ctx.check(
            "gussets_proud_of_face",
            g0[1][0] > 0.20 and g1[1][0] > 0.20,
            f"g0_max_x={g0[1][0]:.3f}, g1_max_x={g1[1][0]:.3f}",
        )

    # The axle pin is intentionally captured inside each wheel hub bore.
    for w in (left, right):
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
