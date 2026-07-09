from __future__ import annotations

# Small ground-level steel utility junction box on four short legs, with a flat
# top-opening hinged lid secured by a front hasp clasp.
#
# Reference: a compact grey-painted steel box sitting low to the ground on short
# stubby legs/feet (set into a gravel field). The top is a flat lid hinged along
# the REAR top edge so it swings up and back; a hasp/staple clasp on the FRONT
# top edge latches it shut, with a folded-over corner tab near the hinge. A short
# conduit cable stub exits one side near the base.
#
# Coordinate convention (Z-up world):
#   +X : box WIDTH  (~0.36 m)
#   +Y : box DEPTH  (~0.30 m); FRONT (clasp) at -Y, REAR (hinge) at +Y
#   +Z : HEIGHT; the feet stand on the ground at z=0, box top ~0.27 m
#
# Parts / articulations:
#   - box (ROOT)  : hollow steel enclosure on four short legs, open at the top
#   - lid         : flat top panel, REVOLUTE about the REAR (+Y) horizontal edge,
#                   opens upward/back; carries a folded corner tab
#   - clasp       : front hasp lever, REVOLUTE about a small horizontal pin,
#                   flips up to release the lid
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- overall dimensions (metres) ----
W = 0.36  # width (X)
D = 0.30  # depth (Y)
BOX_H = 0.18  # enclosure shell height (above the legs)
LEG_H = 0.06  # short leg height (box stands this far off the ground)
WALL = 0.010

LID_T = 0.016
LID_OVER = 0.012  # lid overhangs the box rim on the sides/front
LID_W = W + 2 * LID_OVER
LID_D = D + 2 * LID_OVER
RIVET_R = 0.0032
RIVET_H = 0.0022

REAR_Y = +D / 2.0
FRONT_Y = -D / 2.0
RIM_Z = LEG_H + BOX_H  # top rim plane of the box (where the lid seats)


def _hollow_box(outer, wall, *, open_face):
    sx, sy, sz = outer
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    pieces = []
    pieces.append((Box((sx, wall, sz)), Origin(xyz=(0.0, hy - wall / 2.0, 0.0))))  # back +Y
    pieces.append((Box((sx, wall, sz)), Origin(xyz=(0.0, -hy + wall / 2.0, 0.0))))  # front -Y
    pieces.append((Box((wall, sy, sz)), Origin(xyz=(hx - wall / 2.0, 0.0, 0.0))))  # +X
    pieces.append((Box((wall, sy, sz)), Origin(xyz=(-hx + wall / 2.0, 0.0, 0.0))))  # -X
    if open_face != "+z":
        pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, 0.0, hz - wall / 2.0))))
    pieces.append((Box((sx, sy, wall)), Origin(xyz=(0.0, 0.0, -hz + wall / 2.0))))  # floor
    return pieces


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="junction_box")

    paint = model.material("grey_paint", rgba=(0.70, 0.71, 0.72, 1.0))
    lid_paint = model.material("lid_paint", rgba=(0.75, 0.76, 0.77, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.33, 0.34, 0.36, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    shadow = model.material("shadowed_inside", rgba=(0.42, 0.43, 0.44, 1.0))

    # ---- box (ROOT) : enclosure on four short legs, open at the top ----
    box = model.part("box")
    box_cz = LEG_H + BOX_H / 2.0
    for i, (geom, org) in enumerate(_hollow_box((W, D, BOX_H), WALL, open_face="+z")):
        box.visual(
            geom,
            origin=Origin(xyz=(org.xyz[0], org.xyz[1], org.xyz[2] + box_cz)),
            material=paint,
            name=f"shell_{i}",
        )
    # rolled top rim and shadowed inner ledge, matching the folded steel lip in
    # the reference rather than a plain open cube.
    rim_z = RIM_Z + 0.004
    box.visual(
        Box((W + 0.018, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.002, rim_z)),
        material=paint,
        name="front_rolled_rim",
    )
    box.visual(
        Box((W + 0.018, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, REAR_Y + 0.002, rim_z)),
        material=paint,
        name="rear_rolled_rim",
    )
    for sx, name in [(-1, "left"), (1, "right")]:
        box.visual(
            Box((0.012, D + 0.018, 0.012)),
            origin=Origin(xyz=(sx * (W / 2.0 + 0.002), 0.0, rim_z)),
            material=paint,
            name=f"{name}_rolled_rim",
        )
    box.visual(
        Box((W - 0.040, D - 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + 0.014)),
        material=shadow,
        name="dark_recessed_floor",
    )
    for sx, name in [(-1, "left"), (1, "right")]:
        box.visual(
            Box((0.007, D - 0.048, 0.020)),
            origin=Origin(xyz=(sx * (W / 2.0 - WALL - 0.0035), 0.0, RIM_Z - 0.030)),
            material=shadow,
            name=f"{name}_inner_shadow_lip",
        )
    # four short legs (angle-iron stubs) standing on the ground, base at z=0
    leg_inset = 0.03
    for li, (sx, sy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        lx = sx * (W / 2.0 - leg_inset)
        ly = sy * (D / 2.0 - leg_inset)
        box.visual(
            Box((0.028, 0.028, LEG_H + 0.02)),
            origin=Origin(xyz=(lx, ly, (LEG_H + 0.02) / 2.0)),
            material=steel,
            name=f"leg_{li}",
        )
        box.visual(
            Box((0.046, 0.036, 0.008)),
            origin=Origin(xyz=(lx, ly, 0.004)),
            material=steel,
            name=f"foot_pad_{li}",
        )
    # exterior stiffening ribs and rivets make the small grey enclosure read as
    # painted steel sheet metal.
    for sx, name in [(-1, "left"), (1, "right")]:
        box.visual(
            Box((0.008, D - 0.050, 0.018)),
            origin=Origin(xyz=(sx * (W / 2.0 + 0.002), 0.0, LEG_H + 0.115)),
            material=steel,
            name=f"{name}_side_horizontal_rib",
        )
        for yy in (-0.085, 0.085):
            box.visual(
                Box((0.009, 0.014, BOX_H - 0.035)),
                origin=Origin(xyz=(sx * (W / 2.0 + 0.002), yy, LEG_H + 0.095)),
                material=steel,
                name=f"{name}_side_vertical_rib_{yy:+.2f}",
            )
    for x in (-0.135, 0.135):
        box.visual(
            Box((0.012, 0.008, BOX_H - 0.030)),
            origin=Origin(xyz=(x, FRONT_Y - 0.004, LEG_H + 0.100)),
            material=steel,
            name=f"front_corner_seam_{x:+.2f}",
        )
    for x in (-0.130, -0.065, 0.065, 0.130):
        for z in (LEG_H + 0.070, LEG_H + 0.145):
            box.visual(
                Cylinder(radius=RIVET_R, length=RIVET_H),
                origin=Origin(
                    xyz=(x, FRONT_Y - RIVET_H / 2.0 + 0.0004, z), rpy=(-math.pi / 2.0, 0.0, 0.0)
                ),
                material=dark_metal,
                name=f"front_rivet_{x:+.2f}_{z:.2f}",
            )
    # short conduit cable stub exiting the -X side near the base
    box.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-W / 2.0 - 0.03, 0.0, LEG_H + 0.05), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="conduit_stub",
    )
    box.visual(
        Cylinder(radius=0.023, length=0.010),
        origin=Origin(xyz=(-W / 2.0 - 0.004, 0.0, LEG_H + 0.05), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="conduit_mount_collar",
    )
    # exposed barrel hinge seated on the REAR rim (pin axis along X = horizontal)
    rear_hinge = BarrelHingeGeometry(
        0.20,
        leaf_width_a=0.018,
        leaf_width_b=0.018,
        leaf_thickness=0.0024,
        pin_diameter=0.006,
        knuckle_count=7,
    )
    # BarrelHinge pin axis is local Z; a Visual rpy pitch of +pi/2 maps local +Z
    # -> +X so the pin lies horizontally along the rear edge.
    box.visual(
        mesh_from_geometry(rear_hinge, "rear_hinge"),
        origin=Origin(xyz=(0.0, REAR_Y - 0.006, RIM_Z + 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_hinge",
    )
    for x in (-0.085, -0.030, 0.030, 0.085):
        box.visual(
            Cylinder(radius=RIVET_R, length=RIVET_H),
            origin=Origin(
                xyz=(x, REAR_Y + RIVET_H / 2.0 - 0.0004, RIM_Z - 0.015),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"rear_hinge_rivet_{x:+.2f}",
        )
    # hasp staple loop on the front rim (the fixed half of the clasp)
    box.visual(
        Box((0.064, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.003, RIM_Z - 0.034)),
        material=steel,
        name="hasp_fixed_plate",
    )
    for x in (-0.034, 0.034):
        box.visual(
            Box((0.009, 0.018, 0.030)),
            origin=Origin(xyz=(x, FRONT_Y - 0.0145, RIM_Z - 0.030)),
            material=steel,
            name=f"hasp_staple_ear_{x:+.2f}",
        )
    for x in (-0.034, 0.034):
        box.visual(
            Cylinder(radius=0.004, length=0.012),
            origin=Origin(xyz=(x, FRONT_Y - 0.0145, RIM_Z - 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"hasp_outer_pin_end_{x:+.2f}",
        )
    for x in (-0.024, 0.024):
        for z in (RIM_Z - 0.018, RIM_Z - 0.048):
            box.visual(
                Cylinder(radius=RIVET_R, length=RIVET_H),
                origin=Origin(xyz=(x, FRONT_Y - 0.006, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=dark_metal,
                name=f"hasp_plate_rivet_{x:+.2f}_{z:.2f}",
            )
    box.inertial = Inertial.from_geometry(
        Box((W, D, BOX_H + LEG_H)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, box_cz)),
    )

    # ---- lid : flat top panel hinged on the REAR (+Y) horizontal edge ----
    # Authored so the LOCAL ORIGIN is the rear hinge edge: the lid plate extends
    # from local y=0 toward -Y (local y in [-LID_D, 0]) and sits at local z~0.
    # This makes the hinge pivot coincide with the joint origin at the rear rim.
    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_T)),
        origin=Origin(xyz=(0.0, -LID_D / 2.0, 0.0)),
        material=lid_paint,
        name="lid_plate",
    )
    lid.visual(
        Box((LID_W - 0.038, LID_D - 0.046, 0.006)),
        origin=Origin(xyz=(0.0, -LID_D / 2.0 - 0.006, LID_T / 2.0 + 0.003)),
        material=lid_paint,
        name="raised_lid_panel",
    )
    for x in (-0.145, 0.145):
        lid.visual(
            Box((0.010, LID_D - 0.030, 0.007)),
            origin=Origin(xyz=(x, -LID_D / 2.0 - 0.006, LID_T / 2.0 + 0.006)),
            material=steel,
            name=f"lid_side_pressing_{x:+.2f}",
        )
    for y, name in [(-0.060, "rear"), (-LID_D + 0.052, "front")]:
        lid.visual(
            Box((LID_W - 0.070, 0.008, 0.007)),
            origin=Origin(xyz=(0.0, y, LID_T / 2.0 + 0.006)),
            material=steel,
            name=f"lid_{name}_pressing",
        )
    # folded-over corner tab on the front edge (small downturned lip)
    lid.visual(
        Box((0.06, 0.012, 0.03)),
        origin=Origin(xyz=(LID_W / 2.0 - 0.05, -LID_D + 0.006, -0.02)),
        material=lid_paint,
        name="corner_tab",
    )
    # front drip lip running along the front edge of the lid
    lid.visual(
        Box((LID_W, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, -LID_D + 0.005, -0.012)),
        material=lid_paint,
        name="front_lip",
    )
    for x in (-LID_W / 2.0 + 0.006, LID_W / 2.0 - 0.006):
        lid.visual(
            Box((0.010, LID_D - 0.018, 0.022)),
            origin=Origin(xyz=(x, -LID_D / 2.0 - 0.004, -0.011)),
            material=lid_paint,
            name=f"side_lid_lip_{x:+.2f}",
        )
    for x in (-0.145, -0.075, 0.0, 0.075, 0.145):
        lid.visual(
            Cylinder(radius=RIVET_R, length=RIVET_H),
            origin=Origin(xyz=(x, -0.025, LID_T / 2.0 + RIVET_H / 2.0 - 0.0004)),
            material=dark_metal,
            name=f"lid_rear_rivet_{x:+.2f}",
        )
        lid.visual(
            Cylinder(radius=RIVET_R, length=RIVET_H),
            origin=Origin(xyz=(x, -LID_D + 0.032, LID_T / 2.0 + RIVET_H / 2.0 - 0.0004)),
            material=dark_metal,
            name=f"lid_front_rivet_{x:+.2f}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_T)),
        mass=2.0,
        origin=Origin(xyz=(0.0, -LID_D / 2.0, 0.0)),
    )

    # Lid REVOLUTE about the rear (+Y) edge, pin axis along X. The joint origin is
    # the rear hinge edge on the box (Y = rear lid edge = REAR_Y + LID_OVER),
    # at the lid mid-plane just above the rim. The lid plate extends from the
    # hinge toward -Y; positive q about -X lifts the front (-Y) edge up and back.
    hinge_y = REAR_Y + LID_OVER
    model.articulation(
        "box_to_lid",
        ArticulationType.REVOLUTE,
        parent=box,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, RIM_Z + LID_T / 2.0 + 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(105.0), effort=10.0, velocity=2.0),
    )

    # ---- clasp : front hasp lever, flips up on a small horizontal pin ----
    # Mounted on the front face of the box just below the rim. Authored with its
    # pivot at the local origin; the lever hangs down (-Z) closed, flips up open.
    clasp = model.part("clasp")
    clasp.visual(
        Box((0.036, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.003, -0.004)),
        material=steel,
        name="hasp_pivot_leaf",
    )
    clasp.visual(
        Cylinder(radius=0.0045, length=0.056),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hasp_hinge_sleeve",
    )
    for x in (-0.012, 0.012):
        clasp.visual(
            Box((0.008, 0.006, 0.052)),
            origin=Origin(xyz=(x, -0.004, -0.036)),
            material=steel,
            name=f"hasp_side_strap_{x:+.2f}",
        )
    clasp.visual(
        Box((0.038, 0.005, 0.020)),
        origin=Origin(xyz=(0.0, -0.005, -0.038)),
        material=steel,
        name="hasp_center_plate",
    )
    # small hook lip at the bottom that grabs the staple
    clasp.visual(
        Box((0.042, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.014, -0.066)),
        material=steel,
        name="hasp_hook",
    )
    clasp.visual(
        Cylinder(radius=0.0045, length=0.038),
        origin=Origin(xyz=(0.0, -0.017, -0.059), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hasp_hook_rolled_edge",
    )
    for x in (-0.013, 0.013):
        clasp.visual(
            Cylinder(radius=0.0028, length=0.002),
            origin=Origin(xyz=(x, -0.0075, -0.038), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"hasp_lever_rivet_{x:+.2f}",
        )
    clasp.inertial = Inertial.from_geometry(
        Box((0.04, 0.03, 0.07)),
        mass=0.15,
        origin=Origin(xyz=(0.0, -0.008, -0.03)),
    )
    model.articulation(
        "box_to_clasp",
        ArticulationType.REVOLUTE,
        parent=box,
        child=clasp,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0145, RIM_Z - 0.024)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(110.0), effort=2.0, velocity=2.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    box = object_model.get_part("box")
    lid = object_model.get_part("lid")
    clasp = object_model.get_part("clasp")
    lid_joint = object_model.get_articulation("box_to_lid")
    clasp_joint = object_model.get_articulation("box_to_clasp")

    # ---- feet stand on the ground at z=0 ----
    bmn, bmx = ctx.part_world_aabb(box)
    ctx.check("box feet at z=0", abs(bmn[2]) < 1e-3, details=f"box min z={bmn[2]:.4f}")

    # ---- it's a small low box (wider/deeper than tall) ----
    ext = _ext((bmn, bmx))
    ctx.check(
        "box is low (footprint larger than height)",
        ext[0] > ext[2] and ext[1] > ext[2],
        details=f"extents (x,y,z)={tuple(round(e, 3) for e in ext)}",
    )

    # ---- lid joint is REVOLUTE about a HORIZONTAL (X) axis ----
    ax = lid_joint.axis
    ctx.check(
        "lid hinge axis is horizontal (X)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )
    ctx.check(
        "lid joint is revolute",
        str(lid_joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={lid_joint.articulation_type}",
    )

    # ---- lid lifts UP and the front edge swings back when opened ----
    closed_aabb = ctx.part_world_aabb(lid)
    closed_top = closed_aabb[1][2]
    closed_front_y = closed_aabb[0][1]
    with ctx.pose({lid_joint: math.radians(90.0)}):
        open_aabb = ctx.part_world_aabb(lid)
        open_top = open_aabb[1][2]
        open_front_y = open_aabb[0][1]
    ctx.check(
        "lid rises when opened",
        open_top > closed_top + 0.08,
        details=f"closed_top={closed_top:.3f}, open_top={open_top:.3f}",
    )
    ctx.check(
        "lid front edge swings back toward the rear hinge",
        open_front_y > closed_front_y + 0.05,
        details=f"closed_front_y={closed_front_y:.3f}, open_front_y={open_front_y:.3f}",
    )

    # ---- clasp flips up and outward about its front pin ----
    cax = clasp_joint.axis
    ctx.check(
        "front clasp hinge axis opens outward",
        cax[0] < -0.99 and abs(cax[1]) < 1e-6 and abs(cax[2]) < 1e-6,
        details=f"axis={cax}",
    )
    rest_aabb = ctx.part_world_aabb(clasp)
    rest_bottom = rest_aabb[0][2]
    rest_front_y = rest_aabb[0][1]
    with ctx.pose({clasp_joint: math.radians(100.0)}):
        flip_aabb = ctx.part_world_aabb(clasp)
        flip_bottom = flip_aabb[0][2]
        flip_front_y = flip_aabb[0][1]
    ctx.check(
        "front clasp flips up to release",
        flip_bottom > rest_bottom + 0.02,
        details=f"rest_bottom={rest_bottom:.3f}, flip_bottom={flip_bottom:.3f}",
    )
    ctx.check(
        "front clasp swings outward from the box",
        flip_front_y < rest_front_y - 0.010,
        details=f"rest_front_y={rest_front_y:.3f}, flip_front_y={flip_front_y:.3f}",
    )

    # ---- closed lid seats on the box rim (small seam overlap allowed) ----
    ctx.allow_overlap(
        lid,
        box,
        reason="Closed lid overhangs and seats on the box rim; hinge knuckles meet the rear edge.",
    )
    return ctx.report()


object_model = build_object_model()
