from __future__ import annotations

# Large commercial 1100 L four-caster waste container (steel dumpster) with
# two split rear-hinged flip-up lids.
#
# Coordinate convention:
#   - up is +Z; the casters touch the ground at z=0.
#   - the container "front" (the lifting comb / handles) faces +X.
#   - the two lids split left/right across +/-Y and both hinge at the rear (-X)
#     top edge.
#
# Structure:
#   - body (root, static): tall ribbed/corrugated hollow steel shell with a
#     thick top rim, horizontal corrugation ribs on the side faces, side grab
#     handles, a front lifting comb (DIN trunnion bar), and four caster stem
#     bosses under the base.
#   - lid_left / lid_right (REVOLUTE about +Y at the rear top rim): the two
#     split flat flip lids; each covers half the top and opens upward/rearward.
#   - caster_fl / caster_fr / caster_rl / caster_rr (CONTINUOUS about +Y):
#     four swivel-style ground wheels mounted on short stems under the base.

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
BODY_W = 1.330  # outside width (Y) of the container body
BODY_D = 1.000  # outside depth (X)
BODY_H = 0.980  # body shell height
TAPER = 0.060  # how much narrower the bottom is (per side) than the top
WALL = 0.022
RIM_H = 0.055
CORNER_R = 0.050

CASTER_DIA = 0.200
CASTER_R = CASTER_DIA / 2.0
CASTER_W = 0.060
STEM_R = 0.028  # vertical swivel-stem radius
STEM_H = 0.090  # swivel stem height between the base boss and the fork
CASTER_TRAIL = 0.070  # wheel center trails the swivel axis (real caster offset)

# Body bottom sits clearly above the wheel tops; the swivel stem bridges the gap.
BODY_BOTTOM_Z = 0.300
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_H

# Lid hinge: rear top edge.
HINGE_X = -BODY_D / 2.0 + 0.030
HINGE_Z = BODY_TOP_Z + 0.012

# Swivel-stem axis placement (under the four bottom corners).
STEM_X = BODY_D / 2.0 - TAPER - 0.080
STEM_Y = BODY_W / 2.0 - TAPER - 0.100
# The wheel/axle sits trailing toward +X (front) of the stem axis.
CASTER_X = STEM_X + CASTER_TRAIL
CASTER_Y = STEM_Y
CASTER_Z = CASTER_R


def _rrect(dx: float, dy: float, r: float) -> cq.Sketch:
    r = min(r, dx / 2.0 - 1e-4, dy / 2.0 - 1e-4)
    return cq.Sketch().rect(dx, dy).vertices().fillet(r)


def _build_body_mesh():
    """Ribbed hollow container shell with rim, corrugations, comb and bosses."""
    bw_b = BODY_W - 2 * TAPER
    bd_b = BODY_D - 2 * TAPER
    # Outer tapered shell.
    outer = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(bd_b, bw_b, CORNER_R),
            _rrect(BODY_D, BODY_W, CORNER_R).moved(cq.Location(cq.Vector(0, 0, BODY_H))),
        )
        .loft()
    )
    # Hollow interior.
    inner = (
        cq.Workplane("XY")
        .placeSketch(
            _rrect(bd_b - 2 * WALL, bw_b - 2 * WALL, max(CORNER_R - WALL, 0.008)).moved(
                cq.Location(cq.Vector(0, 0, WALL))
            ),
            _rrect(BODY_D - 2 * WALL, BODY_W - 2 * WALL, max(CORNER_R - WALL, 0.008)).moved(
                cq.Location(cq.Vector(0, 0, BODY_H + 0.002))
            ),
        )
        .loft()
    )
    shell = outer.cut(inner)

    # Thick reinforcing top rim band.
    rim = (
        cq.Workplane("XY")
        .placeSketch(_rrect(BODY_D + 0.018, BODY_W + 0.018, CORNER_R).moved(
            cq.Location(cq.Vector(0, 0, BODY_H - RIM_H))
        ))
        .extrude(RIM_H)
    )
    rim_inner = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - RIM_H - 0.001)
        .rect(BODY_D - 2 * WALL, BODY_W - 2 * WALL)
        .extrude(RIM_H + 0.004)
    )
    rim = rim.cut(rim_inner)
    shell = shell.union(rim)

    # Horizontal corrugation ribs proud of the two long (front/back) faces.
    rib_t = 0.022
    rib_proud = 0.014
    for zc in (0.30, 0.45, 0.60, 0.75):
        face_d = bd_b + (BODY_D - bd_b) * (zc / BODY_H)
        rib_w = bw_b + (BODY_W - bw_b) * (zc / BODY_H) - 0.10
        rib = (
            cq.Workplane("XY")
            .workplane(offset=zc)
            .rect(face_d + 2 * rib_proud, rib_w)
            .extrude(rib_t)
        )
        rib_cut = (
            cq.Workplane("XY")
            .workplane(offset=zc - 0.002)
            .rect(face_d - 0.04, rib_w + 0.04)
            .extrude(rib_t + 0.004)
        )
        rib = rib.cut(rib_cut)
        shell = shell.union(rib)

    # Front lifting comb: a horizontal trunnion bar across the front face.
    comb_z = 0.34
    face_x = (bd_b + (BODY_D - bd_b) * (comb_z / BODY_H)) / 2.0
    comb = (
        cq.Workplane("YZ")
        .workplane(offset=face_x)
        .center(0.0, comb_z)
        .circle(0.026)
        .extrude(0.060)
    )
    shell = shell.union(comb)
    # Comb gusset plates that tie the bar to the body.
    for yy in (-0.34, 0.34):
        gus = (
            cq.Workplane("XY")
            .center(face_x + 0.02, yy)
            .box(0.080, 0.030, 0.12, centered=(True, True, False))
            .translate((0, 0, comb_z - 0.06))
        )
        shell = shell.union(gus)

    # Side grab handles (one on each side face, +/-Y). Start the slab a little
    # inside the wall so it fuses with the shell, then stand proud outward.
    for ys in (-1.0, 1.0):
        face_y = (bw_b + (BODY_W - bw_b) * (0.62 / BODY_H)) / 2.0
        handle = (
            cq.Workplane("XZ")
            .workplane(offset=ys * (face_y - 0.012))
            .center(0.0, 0.62)
            .rect(0.18, 0.044)
            .extrude(ys * 0.046)
        )
        shell = shell.union(handle)

    # Four caster assemblies: boss + swivel pillar + fork, authored in the
    # body-local frame so the metalwork is one connected solid with the shell.
    # (z=0 here is the body base; it lifts to BODY_BOTTOM_Z in world.)
    # Every descending member is kept clear of the tire envelope: above it (Z),
    # outside it (Y), or behind it (X).
    def lz(world_z: float) -> float:
        return world_z - BODY_BOTTOM_Z

    axle_z_l = lz(CASTER_Z)  # local axle height (negative)
    tire_top_l = lz(CASTER_DIA)  # local tire top
    crown_z_l = tire_top_l + 0.024  # fork crown sits just above the tire top
    cheek_half_y = CASTER_W / 2.0 + 0.016  # fork legs just outside the tire width
    tire_back_dx = CASTER_R + 0.052  # how far behind the wheel center the pillar is
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            syy = sy * STEM_Y
            wxx = sx * CASTER_X  # trailing wheel/fork X
            # The swivel pillar drops behind the tire (toward body center).
            pillar_x = wxx - sx * tire_back_dx
            # Mounting boss flush under the base, above the pillar.
            boss = (
                cq.Workplane("XY")
                .center(pillar_x, syy)
                .box(0.120, 0.130, 0.060, centered=(True, True, False))
                .edges("|Z")
                .fillet(0.016)
                .translate((0, 0, -0.030))
            )
            shell = shell.union(boss)
            # Vertical swivel pillar from the boss down to the crown level,
            # positioned behind the tire so it never enters the tire envelope.
            pillar = (
                cq.Workplane("XY")
                .center(pillar_x, syy)
                .box(0.070, 0.080, abs(crown_z_l) + 0.030, centered=(True, True, True))
                .edges("|Z")
                .fillet(0.012)
                .translate((0, 0, crown_z_l / 2.0))
            )
            shell = shell.union(pillar)
            # Fork crown plate above the tire, from the pillar forward over the
            # wheel center, linking the pillar to the two fork legs.
            crown = (
                cq.Workplane("XY")
                .center((pillar_x + wxx) / 2.0, syy)
                .box(abs(wxx - pillar_x) + 0.070, 2 * cheek_half_y + 0.020, 0.036,
                     centered=(True, True, True))
                .edges("|Y")
                .fillet(0.012)
                .translate((0, 0, crown_z_l))
            )
            shell = shell.union(crown)
            # Two fork legs straddling the wheel (outside the tire width) down to
            # the axle, in a narrow X band at the wheel center.
            leg_top = crown_z_l + 0.018
            leg_bot = axle_z_l - 0.018
            for cy in (-1.0, 1.0):
                leg = (
                    cq.Workplane("XY")
                    .center(wxx, syy + cy * cheek_half_y)
                    .box(0.046, 0.020, leg_top - leg_bot, centered=(True, True, False))
                    .translate((0, 0, leg_bot))
                )
                shell = shell.union(leg)

    return mesh_from_cadquery(shell, "container_body", unit_scale=1.0)


def _build_lid_mesh(half_w: float):
    """One split flip lid covering half the top, hinged at the rear edge."""
    lid_d = BODY_D + 0.016
    plate_t = 0.026
    plate = (
        cq.Workplane("XY")
        .placeSketch(_rrect(lid_d, half_w, CORNER_R + 0.004))
        .extrude(plate_t)
    )
    # Gentle crown rib down the middle (front-to-back) of the plate.
    crown = (
        cq.Workplane("XY")
        .workplane(offset=plate_t)
        .center(0.0, 0.0)
        .rect(lid_d - 0.12, 0.10)
        .extrude(0.014)
    )
    lid = plate.union(crown)

    # Shallow perimeter skirt so the lid laps the rim lip.
    skirt_drop = 0.016
    skirt_outer = (
        cq.Workplane("XY")
        .placeSketch(_rrect(lid_d, half_w, CORNER_R + 0.004).moved(
            cq.Location(cq.Vector(0, 0, -skirt_drop))
        ))
        .extrude(skirt_drop)
    )
    skirt_inner = (
        cq.Workplane("XY")
        .placeSketch(_rrect(lid_d - 0.028, half_w - 0.028, CORNER_R).moved(
            cq.Location(cq.Vector(0, 0, -skirt_drop - 0.002))
        ))
        .extrude(skirt_drop + 0.004)
    )
    lid = lid.union(skirt_outer.cut(skirt_inner))

    # Front grab lip on this lid.
    lip = (
        cq.Workplane("XY")
        .center(lid_d / 2.0 - 0.006, 0.0)
        .box(0.034, half_w * 0.45, 0.030, centered=(True, True, False))
        .edges("|Y")
        .fillet(0.008)
        .translate((0, 0, -0.006))
    )
    lid = lid.union(lip)

    return mesh_from_cadquery(lid, f"lid_half_{int(half_w * 1000)}", unit_scale=1.0)


def _caster_wheel_mesh(name: str):
    wheel = WheelGeometry(
        CASTER_R - 0.016,
        CASTER_W - 0.018,
        rim=WheelRim(inner_radius=CASTER_R - 0.040, flange_height=0.006, flange_thickness=0.004),
        hub=WheelHub(radius=0.022, width=0.034, cap_style="flat"),
        face=WheelFace(dish_depth=0.004, front_inset=0.002),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.005, window_radius=0.008),
        bore=WheelBore(style="round", diameter=2 * 0.012 - 0.002),
    )
    return mesh_from_geometry(wheel, name)


def _caster_tire_mesh(name: str):
    tire = TireGeometry(
        CASTER_R,
        CASTER_W,
        inner_radius=CASTER_R - 0.016,
        tread=TireTread(style="block", depth=0.004, count=24, land_ratio=0.62),
        sidewall=TireSidewall(style="square", bulge=0.02),
    )
    return mesh_from_geometry(tire, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_waste_container")

    body_green = model.material("body_green", rgba=(0.18, 0.33, 0.22, 1.0))
    lid_green = model.material("lid_green", rgba=(0.13, 0.26, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    wheel_dark = model.material("caster_wheel", rgba=(0.16, 0.16, 0.18, 1.0))
    tire_black = model.material("caster_tire", rgba=(0.08, 0.08, 0.09, 1.0))

    casters_layout = (
        ("fl", 1.0, 1.0),
        ("fr", 1.0, -1.0),
        ("rl", -1.0, 1.0),
        ("rr", -1.0, -1.0),
    )

    # --- body (root) ---
    body = model.part("body")
    body.visual(
        _build_body_mesh(),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z)),
        material=body_green,
        name="shell",
    )
    # The boss + swivel stem + trailing fork are modeled in the body mesh; each
    # caster axle is a short steel pin captured between the fork cheeks and the
    # wheel hub bore.
    for tag, sx, sy in casters_layout:
        body.visual(
            Cylinder(radius=0.012, length=CASTER_W + 0.040),
            origin=Origin(
                xyz=(sx * CASTER_X, sy * CASTER_Y, CASTER_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"axle_{tag}",
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_D, BODY_W, BODY_H)),
        mass=60.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_H / 2.0)),
    )

    # --- two split lids (parametric, instantiated mirrored across Y) ---
    half_w = BODY_W / 2.0 + 0.006  # each lid slightly overlaps the centerline
    for tag, sign in (("lid_left", 1.0), ("lid_right", -1.0)):
        lid = model.part(tag)
        lid.visual(
            _build_lid_mesh(half_w - 0.008),
            # Lid mesh frame: hinge edge at local origin, plate extends toward +X;
            # the half plate is centered on its own +/-Y half.
            origin=Origin(xyz=((BODY_D + 0.016) / 2.0 - 0.030, sign * half_w / 2.0, 0.0)),
            material=lid_green,
            name="lid_plate",
        )
        lid.inertial = Inertial.from_geometry(
            Box((BODY_D, half_w, 0.05)),
            mass=3.6,
            origin=Origin(xyz=(0.0, sign * half_w / 2.0, 0.0)),
        )
        model.articulation(
            f"{tag}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=lid,
            origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
            # Plate extends along local +X from the hinge; -Y lifts the free
            # front edge up and back for both lids (same convention).
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=1.9),
        )

    # --- four casters ---
    for tag, sx, sy in casters_layout:
        cname = f"caster_{tag}"
        caster = model.part(cname)
        rpy = (0.0, 0.0, -math.pi / 2.0) if sy > 0 else (0.0, 0.0, math.pi / 2.0)
        caster.visual(
            _caster_wheel_mesh(f"{cname}_wheel"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=rpy),
            material=wheel_dark,
            name="wheel",
        )
        caster.visual(
            _caster_tire_mesh(f"{cname}_tire"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=rpy),
            material=tire_black,
            name="tire",
        )
        caster.inertial = Inertial.from_geometry(
            Cylinder(radius=CASTER_R, length=CASTER_W), mass=0.7
        )
        model.articulation(
            f"{cname}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=caster,
            origin=Origin(xyz=(sx * CASTER_X, sy * CASTER_Y, CASTER_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid_l = object_model.get_part("lid_left")
    lid_r = object_model.get_part("lid_right")
    hinge_l = object_model.get_articulation("lid_left_hinge")
    hinge_r = object_model.get_articulation("lid_right_hinge")
    casters = [
        object_model.get_part(n)
        for n in ("caster_fl", "caster_fr", "caster_rl", "caster_rr")
    ]
    spin_fl = object_model.get_articulation("caster_fl_spin")

    # --- hero parts present ---
    ctx.check("has_body", body is not None, "Expected a body part.")
    ctx.check("has_two_lids", lid_l is not None and lid_r is not None, "Expected two lids.")
    ctx.check("has_four_casters", all(c is not None for c in casters), "Expected four casters.")

    # --- joint types/axes ---
    for h in (hinge_l, hinge_r):
        ctx.check(
            f"{h.name}_revolute",
            str(h.articulation_type).endswith("REVOLUTE"),
            f"type={h.articulation_type}",
        )
        ctx.check(
            f"{h.name}_axis_y",
            abs(abs(h.axis[1]) - 1.0) < 1e-6 and abs(h.axis[0]) < 1e-6 and abs(h.axis[2]) < 1e-6,
            f"axis={h.axis}",
        )
    ctx.check(
        "caster_continuous",
        str(spin_fl.articulation_type).endswith("CONTINUOUS"),
        f"type={spin_fl.articulation_type}",
    )
    ctx.check("caster_axis_y", abs(abs(spin_fl.axis[1]) - 1.0) < 1e-6, f"axis={spin_fl.axis}")

    # --- casters on the ground ---
    for c in casters:
        aabb = ctx.part_world_aabb(c)
        if aabb is not None:
            ctx.check(f"{c.name}_on_ground", abs(aabb[0][2]) < 0.012, f"min_z={aabb[0][2]:.4f}")

    # --- overall scale sanity ---
    full = ctx.part_world_aabb(body)
    if full is not None:
        size = tuple(full[1][i] - full[0][i] for i in range(3))
        ctx.check("body_height", 1.05 <= size[2] <= 1.40, f"size={size!r}")
        ctx.check("body_width", 1.20 <= size[1] <= 1.55, f"size={size!r}")

    # --- caster footprint: four corners, two distinct X and two distinct Y ---
    pos = [ctx.part_world_position(c) for c in casters]
    if all(p is not None for p in pos):
        xs = sorted(set(round(p[0], 2) for p in pos))
        ys = sorted(set(round(p[1], 2) for p in pos))
        ctx.check("casters_two_x", len(xs) == 2, f"xs={xs}")
        ctx.check("casters_two_y", len(ys) == 2, f"ys={ys}")

    # --- lids: each caps its half of the body, split across the centerline ---
    ctx.expect_overlap(lid_l, body, axes="xy", min_overlap=0.25, name="left_lid_caps_body")
    ctx.expect_overlap(lid_r, body, axes="xy", min_overlap=0.25, name="right_lid_caps_body")
    # Use the geometry AABB centers (part origins both sit on the shared hinge).
    al = ctx.part_world_aabb(lid_l)
    ar = ctx.part_world_aabb(lid_r)
    if al is not None and ar is not None:
        cyl = (al[0][1] + al[1][1]) / 2.0
        cyr = (ar[0][1] + ar[1][1]) / 2.0
        ctx.check("lids_split_y", cyl > 0.10 and cyr < -0.10, f"cyl={cyl:.3f}, cyr={cyr:.3f}")

    # The lids lap the rim lip; allow that shallow nesting and prove contact.
    for lid in (lid_l, lid_r):
        ctx.allow_overlap(
            lid,
            body,
            reason="The lid skirt laps a few mm over the proud rim outer lip when closed.",
        )
        ctx.expect_contact(lid, body, contact_tol=0.016, name=f"{lid.name}_seated_on_rim")

    # --- both lids open upward and swing rearward ---
    for lid, hinge in ((lid_l, hinge_l), (lid_r, hinge_r)):
        rest = ctx.part_world_aabb(lid)
        with ctx.pose({hinge: 1.6}):
            opened = ctx.part_world_aabb(lid)
        if rest is not None and opened is not None:
            ctx.check(
                f"{lid.name}_opens_upward",
                opened[1][2] > rest[1][2] + 0.15,
                f"rest_top={rest[1][2]:.3f}, open_top={opened[1][2]:.3f}",
            )
            ctx.check(
                f"{lid.name}_swings_rearward",
                opened[1][0] < rest[1][0],
                f"rest_maxx={rest[1][0]:.3f}, open_maxx={opened[1][0]:.3f}",
            )

    # --- axle stubs captured inside each caster hub ---
    axle_names = {
        "caster_fl": "axle_fl",
        "caster_fr": "axle_fr",
        "caster_rl": "axle_rl",
        "caster_rr": "axle_rr",
    }
    for c in casters:
        ctx.allow_overlap(
            body,
            c,
            elem_a=axle_names[c.name],
            elem_b="wheel",
            reason="The caster stub axle is captured inside the wheel hub bore.",
        )
        ctx.expect_overlap(
            body,
            c,
            axes="y",
            elem_a=axle_names[c.name],
            elem_b="wheel",
            min_overlap=0.003,
            name=f"axle_engages_{c.name}",
        )

    return ctx.report()


object_model = build_object_model()
