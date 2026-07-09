"""Two-tone hard plastic pet travel kennel.

Tan upper shell with rows of oval vent holes, black lower tub, hinged black
wire-grate front door, molded top carry handle, and side latch clips joining
the two shells along the rim flange.

Layout: +X = front (door), +Y = left, +Z = up. Ground at z = 0.
"""

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
RIM_L = 0.62  # carrier length at the rim parting line (X)
RIM_W = 0.42  # carrier width at the rim parting line (Y)
RIM_Z = 0.20  # height of the tub / parting line
TOP_L = 0.50  # roof footprint (upper shell tapers inward)
TOP_W = 0.30
SHELL_H = 0.24  # upper shell height (rim -> roof)
TUB_BOT_L = 0.56  # tub bottom footprint (slight outward taper going up)
TUB_BOT_W = 0.36
WALL = 0.006  # molded plastic wall thickness

# rim flange (both shells flare into a bolt flange at the parting line)
FLANGE_L = 0.66
FLANGE_W = 0.46
FLANGE_T = 0.023  # each flange band thickness (they overlap 2 mm at the rim)

# front door opening (cut into the upper shell front wall, starting at the rim)
DOOR_OPEN_W = 0.26  # full width of the opening (Y)
DOOR_OPEN_H = 0.195  # opening height in shell-local Z above the rim

# front wall slope of the tapered upper shell
SHELL_TILT = math.atan2((RIM_L - TOP_L) / 2.0, SHELL_H)  # ~0.245 rad

# wire door (built in its own hinge-local frame, tilted to match the shell)
DOOR_H = 0.19  # door height along the sloped front face
DOOR_W = 0.252  # door width (hangs from the +Y jamb, extends toward -Y)
BORDER_R = 0.0055  # heavy border wire radius
BAR_R = 0.0035  # inner grid wire radius
N_VBARS = 6  # inner vertical wires
N_HBARS = 3  # inner horizontal wires

HINGE_X = 0.316  # hinge pivot, just proud of the sloped front wall
HINGE_Y = 0.132  # at the +Y jamb of the opening
HINGE_Z = 0.206  # just above the rim

# pull-out cleaning tray (slides along +X through a front-wall slot)
TRAY_L = 0.44  # tray floor pan length (X)
TRAY_W = 0.34  # tray floor pan width (Y, wide enough to rest on rails)
TRAY_T = 0.004  # tray plate thickness
TRAY_LIP_H = 0.028  # front pull-lip height above plate
TRAY_EDGE_H = 0.008  # raised side/rear edge height
RAIL_TOP_Z = 0.030  # z height of rail top surface from tub floor
RAIL_H = 0.008  # rail vertical height
RAIL_W = 0.015  # rail protrusion width from inner wall
RAIL_L = 0.40  # rail length along X
TRAY_TRAVEL = 0.18  # prismatic extension (meters)
SLOT_W = 0.36  # front wall slot width (clearance for tray)
SLOT_H = 0.020  # front wall slot height (clears plate + edges)
TRAY_REST_X = 0.060  # tray center X at rest (lip near front wall)

# ---------------------------------------------------------------- materials
TAN = Material(name="tan_plastic", rgba=(0.80, 0.76, 0.69, 1.0))
BLACK_PLASTIC = Material(name="black_plastic", rgba=(0.13, 0.13, 0.14, 1.0))
BLACK_WIRE = Material(name="black_wire", rgba=(0.08, 0.08, 0.09, 1.0))
DARK_GREY = Material(name="dark_grey_plastic", rgba=(0.20, 0.20, 0.21, 1.0))
TRAY_GREY = Material(name="tray_grey_plastic", rgba=(0.35, 0.34, 0.33, 1.0))


# ---------------------------------------------------------------- cq helpers
def _upper_shell_solid() -> cq.Workplane:
    """Tapered hollow tan shell with the door opening and oval vent holes.

    Built in shell-local coords: z=0 at the rim, z=SHELL_H at the roof.
    """
    shell = (
        cq.Workplane("XY")
        .rect(RIM_L, RIM_W)
        .workplane(offset=SHELL_H)
        .rect(TOP_L, TOP_W)
        .loft()
        .faces("<Z")
        .shell(-WALL)
    )

    # front door opening: box cut through the sloped front wall
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(0.31, 0.0, (DOOR_OPEN_H - 0.01) / 2.0 - 0.005))
        .box(0.24, DOOR_OPEN_W, DOOR_OPEN_H + 0.01)
    )
    shell = shell.cut(opening)

    # oval side vents: vertical stadium slots cut straight through both side
    # walls (three staggered rows, upper rows shorter as the shell narrows)
    rows = (
        (0.048, 7, 0.0),  # (row height above rim, slot count, x stagger)
        (0.106, 6, 0.029),
        (0.164, 5, 0.0),
    )
    pitch = 0.058
    pts = []
    for z_row, count, stagger in rows:
        x0 = -(count - 1) * pitch / 2.0 + stagger
        for i in range(count):
            pts.append((x0 + i * pitch, z_row))
    side_cutters = (
        cq.Workplane("XZ")
        .pushPoints(pts)
        .slot2D(0.046, 0.023, 90)
        .extrude(0.6, both=True)
    )
    shell = shell.cut(side_cutters)

    # rear vents: two rows of four, cut rearward only (front wall untouched)
    rear_pts = []
    for z_row in (0.060, 0.120):
        for i in range(4):
            rear_pts.append((-0.0825 + i * 0.055, z_row))
    rear_cutters = (
        cq.Workplane("YZ")
        .pushPoints(rear_pts)
        .slot2D(0.046, 0.023, 90)
        .extrude(-0.6)
    )
    return shell.cut(rear_cutters)


def _tub_solid() -> cq.Workplane:
    """Black lower tub: outward-tapered open-top hollow tub with front tray slot."""
    tub = (
        cq.Workplane("XY")
        .rect(TUB_BOT_L, TUB_BOT_W)
        .workplane(offset=RIM_Z)
        .rect(RIM_L, RIM_W)
        .loft()
        .faces(">Z")
        .shell(-WALL)
    )
    # front wall slot for the pull-out cleaning tray
    slot_z = RAIL_TOP_Z + (TRAY_T + TRAY_EDGE_H) / 2.0 + 0.001
    slot_cutter = (
        cq.Workplane("XY")
        .transformed(offset=(TUB_BOT_L / 2.0, 0.0, slot_z))
        .box(0.040, SLOT_W, SLOT_H)
    )
    return tub.cut(slot_cutter)


def _tub_rails() -> cq.Workplane:
    """Two horizontal inner side rails with a rear cross-bar for the tray to slide on."""
    rail_z_center = RAIL_TOP_Z - RAIL_H / 2.0
    # account for the tub taper at rail height
    taper_factor = rail_z_center / RIM_Z
    outer_half_w = TUB_BOT_W / 2.0 + (RIM_W - TUB_BOT_W) / 2.0 * taper_factor
    inner_half_w = outer_half_w - WALL
    # rail outer face seats into the inner wall (small embed for connectivity)
    rail_cy = inner_half_w - RAIL_W / 2.0 + 0.001
    left = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, -rail_cy, rail_z_center))
        .box(RAIL_L, RAIL_W, RAIL_H)
    )
    right = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, rail_cy, rail_z_center))
        .box(RAIL_L, RAIL_W, RAIL_H)
    )
    # rear cross-bar connecting the two rails (structural support member)
    crossbar = (
        cq.Workplane("XY")
        .transformed(offset=(-(RAIL_L / 2.0 - 0.010), 0.0, rail_z_center))
        .box(0.020, 2.0 * rail_cy + RAIL_W, RAIL_H)
    )
    return left.union(right).union(crossbar)


def _tray_solid() -> cq.Workplane:
    """Pull-out cleaning tray: thin flat pan with raised edges and front pull lip."""
    # base plate centered at origin
    plate = cq.Workplane("XY").box(TRAY_L, TRAY_W, TRAY_T)
    # front pull tab: tall bar at the +X edge for grabbing
    tab = (
        cq.Workplane("XY")
        .transformed(offset=(TRAY_L / 2.0 - 0.005, 0.0, TRAY_LIP_H / 2.0))
        .box(0.010, TRAY_W * 0.55, TRAY_LIP_H + TRAY_T)
    )
    result = plate.union(tab)
    # rear raised edge
    rear = (
        cq.Workplane("XY")
        .transformed(offset=(-(TRAY_L / 2.0 - 0.003), 0.0, TRAY_T / 2.0 + TRAY_EDGE_H / 2.0))
        .box(0.006, TRAY_W - 0.012, TRAY_EDGE_H)
    )
    result = result.union(rear)
    # side raised edges (left and right)
    for sy in (-1.0, 1.0):
        side = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, sy * (TRAY_W / 2.0 - 0.003), TRAY_T / 2.0 + TRAY_EDGE_H / 2.0))
            .box(TRAY_L - 0.022, 0.006, TRAY_EDGE_H)
        )
        result = result.union(side)
    return result


def _flange_ring() -> cq.Workplane:
    """Rim flange ring (rect frame), interrupted across the door opening."""
    ring = (
        cq.Workplane("XY")
        .rect(FLANGE_L, FLANGE_W)
        .rect(RIM_L - 0.02, RIM_W - 0.02)
        .extrude(FLANGE_T)
    )
    front_cut = (
        cq.Workplane("XY")
        .transformed(offset=(0.315, 0.0, FLANGE_T / 2.0))
        .box(0.10, 0.28, FLANGE_T + 0.01)
    )
    return ring.cut(front_cut)


# ---------------------------------------------------------------- the model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plastic_pet_travel_kennel")

    body = model.part("kennel_body")

    # black lower tub
    body.visual(
        mesh_from_cadquery(_tub_solid(), "lower_tub"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=BLACK_PLASTIC,
        name="lower_tub",
    )

    # tan tapered upper shell with vents + door opening
    body.visual(
        mesh_from_cadquery(_upper_shell_solid(), "upper_shell"),
        origin=Origin(xyz=(0.0, 0.0, RIM_Z)),
        material=TAN,
        name="kennel_upper_shell",
    )

    # rim flange bands: black tub lip below, tan shell lip above (overlap 2 mm)
    flange = _flange_ring()
    body.visual(
        mesh_from_cadquery(flange, "flange_black"),
        origin=Origin(xyz=(0.0, 0.0, RIM_Z - FLANGE_T + 0.001)),
        material=BLACK_PLASTIC,
        name="tub_rim_flange",
    )
    body.visual(
        mesh_from_cadquery(flange, "flange_tan"),
        origin=Origin(xyz=(0.0, 0.0, RIM_Z - 0.001)),
        material=TAN,
        name="shell_rim_flange",
    )

    # dark latch clips clamping the two flange bands together
    for i, cx in enumerate((-0.15, 0.01, 0.17)):  # three per side
        for j, sy in enumerate((-1.0, 1.0)):
            body.visual(
                Box((0.036, 0.024, 0.065)),
                origin=Origin(xyz=(cx, sy * (FLANGE_W / 2.0 - 0.005), RIM_Z)),
                material=DARK_GREY,
                name=f"side_latch_clip_{i}_{j}",
            )
    body.visual(  # one clip centered on the rear flange
        Box((0.024, 0.036, 0.065)),
        origin=Origin(xyz=(-(FLANGE_L / 2.0 - 0.005), 0.0, RIM_Z)),
        material=DARK_GREY,
        name="rear_latch_clip",
    )
    for j, sy in enumerate((-1.0, 1.0)):  # two flanking the door opening
        body.visual(
            Box((0.024, 0.036, 0.065)),
            origin=Origin(xyz=(FLANGE_L / 2.0 - 0.005, sy * 0.17, RIM_Z)),
            material=DARK_GREY,
            name=f"front_latch_clip_{j}",
        )

    # molded black carry handle on the roof (base plate, two risers, grip bar)
    roof_z = RIM_Z + SHELL_H
    body.visual(
        Box((0.16, 0.08, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, roof_z + 0.0045)),
        material=BLACK_PLASTIC,
        name="handle_base_plate",
    )
    for i, sx in enumerate((-1.0, 1.0)):
        body.visual(
            Box((0.016, 0.05, 0.075)),
            origin=Origin(xyz=(sx * 0.062, 0.0, roof_z + 0.047)),
            material=BLACK_PLASTIC,
            name=f"handle_riser_{i}",
        )
    body.visual(
        Box((0.14, 0.048, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, roof_z + 0.0745)),
        material=BLACK_PLASTIC,
        name="handle_grip_bar",
    )

    # inner side rails molded into the tub walls for the pull-out tray
    body.visual(
        mesh_from_cadquery(_tub_rails(), "tub_rails"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=BLACK_PLASTIC,
        name="tub_tray_rails",
    )

    # ---------------------------------------------------------- wire door
    # Local frame: origin at the hinge pivot, +z up the sloped door plane,
    # door hangs from the +Y jamb and extends toward -y. The joint origin rpy
    # tilts the whole frame to match the shell's front-wall slope.
    door = model.part("wire_door")

    yb0, yb1 = -0.008, -(DOOR_W - 0.004)  # border wire centerlines (Y)
    zb0, zb1 = 0.008, DOOR_H - 0.008  # border wire centerlines (Z)
    y_mid = (yb0 + yb1) / 2.0
    z_mid = (zb0 + zb1) / 2.0

    # heavy border wires
    for i, yb in enumerate((yb0, yb1)):
        door.visual(
            Cylinder(radius=BORDER_R, length=(zb1 - zb0) + 2 * BORDER_R),
            origin=Origin(xyz=(0.0, yb, z_mid)),
            material=BLACK_WIRE,
            name=f"door_border_vertical_{i}",
        )
    for i, zb in enumerate((zb0, zb1)):
        door.visual(
            Cylinder(radius=BORDER_R, length=(yb0 - yb1) + 2 * BORDER_R),
            origin=Origin(xyz=(0.0, y_mid, zb), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=BLACK_WIRE,
            name=f"door_border_horizontal_{i}",
        )

    # inner wire grid
    for i in range(N_VBARS):
        yv = yb0 + (i + 1) * (yb1 - yb0) / (N_VBARS + 1)
        door.visual(
            Cylinder(radius=BAR_R, length=(zb1 - zb0)),
            origin=Origin(xyz=(0.0, yv, z_mid)),
            material=BLACK_WIRE,
            name=f"door_wire_vertical_{i}",
        )
    for i in range(N_HBARS):
        zh = zb0 + (i + 1) * (zb1 - zb0) / (N_HBARS + 1)
        door.visual(
            Cylinder(radius=BAR_R, length=(yb0 - yb1)),
            origin=Origin(xyz=(0.0, y_mid, zh), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=BLACK_WIRE,
            name=f"door_wire_horizontal_{i}",
        )

    # hinge knuckles: bridge the hinge border wire into the tan jamb
    for i, zk in enumerate((0.030, 0.155)):
        door.visual(
            Box((0.012, 0.016, 0.022)),
            origin=Origin(xyz=(-0.005, 0.002, zk)),
            material=DARK_GREY,
            name=f"door_hinge_knuckle_{i}",
        )

    # squeeze-latch grip on the free edge
    door.visual(
        Box((0.014, 0.018, 0.05)),
        origin=Origin(xyz=(0.010, yb1 - 0.004, z_mid)),
        material=DARK_GREY,
        name="door_latch_grip",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        # pitch the door frame back so it parallels the sloped front wall;
        # positive q swings the free edge outward (+X), away from the carrier
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z), rpy=(0.0, -SHELL_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.6),
    )

    # ---------------------------------------------------------- pull-out tray
    tray = model.part("slide_out_tray")
    tray_z = RAIL_TOP_Z + TRAY_T / 2.0  # tray center height (seated on rails)
    tray.visual(
        mesh_from_cadquery(_tray_solid(), "tray_pan"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=TRAY_GREY,
        name="tray_pan",
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        # tray rests inside the tub with the pull lip near the front wall slot;
        # positive q slides the tray outward along +X through the front slot
        origin=Origin(xyz=(TRAY_REST_X, 0.0, tray_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.3, lower=0.0, upper=TRAY_TRAVEL),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("kennel_body")
    door = object_model.get_part("wire_door")
    tray = object_model.get_part("slide_out_tray")
    hinge = object_model.get_articulation("body_to_door")
    slide = object_model.get_articulation("body_to_tray")

    # the hinge knuckles intentionally embed into the tan jamb wall
    for i in range(2):
        ctx.allow_overlap(
            body,
            door,
            elem_a="kennel_upper_shell",
            elem_b=f"door_hinge_knuckle_{i}",
            reason="door hinge knuckles seat into molded sockets in the shell jamb",
        )

    # the tray pan rides on the tub rails inside the lower tub
    ctx.allow_overlap(
        body,
        tray,
        elem_a="tub_tray_rails",
        elem_b="tray_pan",
        reason="tray pan rests on the inner side rails molded into the tub walls",
    )

    # closed door: grate covers the front opening, flush with the front face
    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(door, body, axes="yz", min_overlap=0.10)
        ctx.expect_within(door, body, axes="y", margin=0.0)
        aabb = ctx.part_world_aabb(door)
        ctx.check(
            "closed_door_sits_at_front_face",
            aabb is not None and 0.24 < aabb[1][0] < 0.35,
            f"door aabb={aabb}",
        )
        body_aabb = ctx.part_world_aabb(body)
        ctx.check(
            "closed_door_not_proud_of_flange",
            aabb is not None and body_aabb is not None and aabb[1][0] <= body_aabb[1][0] + 0.002,
            f"door_x_max={None if aabb is None else aabb[1][0]}",
        )

    # open door: free edge swings forward and clears the opening
    with ctx.pose({hinge: 1.5}):
        aabb = ctx.part_world_aabb(door)
        ctx.check(
            "open_door_swings_forward",
            aabb is not None and aabb[1][0] > 0.45,
            f"door aabb={aabb}",
        )
        ctx.check(
            "open_door_clears_opening_width",
            aabb is not None and aabb[0][1] > 0.0,
            f"door aabb={aabb}",
        )

    # sanity on the wire grate composition
    ctx.check(
        "wire_grate_bar_count",
        N_VBARS >= 5 and N_HBARS >= 3,
        f"vbars={N_VBARS} hbars={N_HBARS}",
    )

    # -------------------------------------------------- slide-out tray tests
    # tray at rest stays inside the tub footprint
    with ctx.pose({slide: 0.0}):
        tray_aabb = ctx.part_world_aabb(tray)
        body_aabb = ctx.part_world_aabb(body)
        ctx.check(
            "tray_at_rest_inside_tub_xy",
            tray_aabb is not None
            and body_aabb is not None
            and tray_aabb[0][0] > body_aabb[0][0] - 0.01
            and tray_aabb[0][1] > body_aabb[0][1] - 0.01
            and tray_aabb[1][1] < body_aabb[1][1] + 0.01,
            f"tray_aabb={tray_aabb}, body_aabb={body_aabb}",
        )
        # tray is near the tub floor (above the rails)
        ctx.check(
            "tray_sits_on_rails_height",
            tray_aabb is not None and 0.025 < tray_aabb[0][2] < 0.045,
            f"tray z_min={None if tray_aabb is None else tray_aabb[0][2]}",
        )

    # tray at full extension slides forward through the slot
    rest_pos = ctx.part_world_position(tray)
    with ctx.pose({slide: TRAY_TRAVEL}):
        ext_aabb = ctx.part_world_aabb(tray)
        ext_pos = ctx.part_world_position(tray)
        ctx.check(
            "tray_extends_forward_through_slot",
            ext_aabb is not None and ext_aabb[1][0] > 0.38,
            f"tray front x at extension={None if ext_aabb is None else ext_aabb[1][0]}",
        )
        ctx.check(
            "tray_prismatic_moves_in_plus_x",
            rest_pos is not None
            and ext_pos is not None
            and ext_pos[0] > rest_pos[0] + TRAY_TRAVEL - 0.005,
            f"rest={rest_pos}, extended={ext_pos}",
        )
        # rear of tray still inside the tub at full extension
        ctx.check(
            "tray_rear_still_inside_at_extension",
            ext_aabb is not None and ext_aabb[0][0] < 0.20,
            f"tray rear x={None if ext_aabb is None else ext_aabb[0][0]}",
        )

    # the tray pan has a visible front pull lip (geometry extends upward)
    tray_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "tray_has_pull_lip_height",
        tray_aabb is not None and (tray_aabb[1][2] - tray_aabb[0][2]) > TRAY_LIP_H * 0.7,
        f"tray height={None if tray_aabb is None else tray_aabb[1][2] - tray_aabb[0][2]}",
    )

    return ctx.report()


object_model = build_object_model()
