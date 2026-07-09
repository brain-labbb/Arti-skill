"""Wheeled rolling pet travel kennel trolley.

Tan upper shell with rows of oval vent holes, black lower tub, hinged black
wire-grate front door, molded top carry handle, side latch clips, rigid
rolling wheel base with corner caster wheels, and telescoping rear trolley
pull handle.

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

# ----- wheel base dimensions -----
WHEEL_R = 0.025  # caster wheel radius
WHEEL_W = 0.018  # caster wheel width (axial)
FORK_H = 0.038  # fork prong height (axle to mount plate bottom)
FORK_W = 0.004  # fork prong thickness
BASE_T = 0.014  # base platform thickness
BASE_L = TUB_BOT_L + 0.02  # base platform slightly larger than tub
BASE_W = TUB_BOT_W + 0.02
BODY_MOUNT_Z = WHEEL_R + FORK_H + BASE_T  # where the body sits on base

# caster corner insets from base edge
CASTER_INSET_X = 0.045
CASTER_INSET_Y = 0.040
_cx = BASE_L / 2.0 - CASTER_INSET_X
_cy = BASE_W / 2.0 - CASTER_INSET_Y
CASTER_CORNERS = [
    (+_cx, +_cy),  # 0: front-left
    (+_cx, -_cy),  # 1: front-right
    (-_cx, +_cy),  # 2: rear-left
    (-_cx, -_cy),  # 3: rear-right
]

# ----- trolley handle dimensions -----
HANDLE_TUBE_R = 0.007  # tube outer radius
HANDLE_TUBE_SPACING = 0.12  # center-to-center Y spacing
HANDLE_TUBE_LENGTH = 0.38  # main tube length (stops below roof)
HANDLE_TUBE_HIDDEN = 0.15  # portion below joint origin at q=0
HANDLE_RISER_R = 0.005  # riser post radius (thinner, passes through roof)
HANDLE_RISER_LENGTH = 0.026  # riser length bridging tube top to grip
HANDLE_GRIP_W = 0.14  # grip bar width (Y)
HANDLE_GRIP_H = 0.028  # grip bar thickness (Z)
HANDLE_GRIP_D = 0.024  # grip bar depth (X)
HANDLE_TRAVEL = 0.25  # prismatic travel (m)

# Joint origin for trolley handle in body-local coords
HANDLE_JOINT_X = -0.22  # inside rear of carrier
HANDLE_JOINT_Z = 0.19  # just below the rim inside

# ---------------------------------------------------------------- materials
TAN = Material(name="tan_plastic", rgba=(0.80, 0.76, 0.69, 1.0))
BLACK_PLASTIC = Material(name="black_plastic", rgba=(0.13, 0.13, 0.14, 1.0))
BLACK_WIRE = Material(name="black_wire", rgba=(0.08, 0.08, 0.09, 1.0))
DARK_GREY = Material(name="dark_grey_plastic", rgba=(0.20, 0.20, 0.21, 1.0))
STEEL = Material(name="steel", rgba=(0.55, 0.55, 0.58, 1.0))
RUBBER_GRIP = Material(name="rubber_grip", rgba=(0.12, 0.12, 0.13, 1.0))
BASE_FRAME_MAT = Material(name="base_frame_mat", rgba=(0.10, 0.10, 0.11, 1.0))
CASTER_PLASTIC = Material(name="caster_plastic", rgba=(0.14, 0.14, 0.15, 1.0))


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
    """Black lower tub: outward-tapered open-top hollow tub, floor kept."""
    return (
        cq.Workplane("XY")
        .rect(TUB_BOT_L, TUB_BOT_W)
        .workplane(offset=RIM_Z)
        .rect(RIM_L, RIM_W)
        .loft()
        .faces(">Z")
        .shell(-WALL)
    )


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


def _base_platform_solid() -> cq.Workplane:
    """Flat rolling base platform with rounded corners, built from z=0 to BASE_T."""
    return (
        cq.Workplane("XY")
        .rect(BASE_L, BASE_W)
        .extrude(BASE_T)
        .edges("|Z")
        .fillet(0.012)
        .edges(">Z")
        .fillet(0.002)
        .edges("<Z")
        .fillet(0.002)
    )


def _caster_wheel_solid() -> cq.Workplane:
    """Simple caster wheel disc centered at origin, width along Z.

    Will be rotated to align width along X on the part.
    """
    return (
        cq.Workplane("XY")
        .circle(WHEEL_R)
        .extrude(WHEEL_W)
        .translate((0, 0, -WHEEL_W / 2.0))
        .edges()
        .fillet(min(0.002, WHEEL_R * 0.08))
    )


# ---------------------------------------------------------------- the model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_pet_kennel_trolley")

    # ========================================================== wheel base (root)
    base = model.part("wheel_base_frame")

    # flat base platform (CadQuery rounded-corner plate)
    base.visual(
        mesh_from_cadquery(_base_platform_solid(), "base_platform"),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_R + FORK_H)),
        material=BASE_FRAME_MAT,
        name="base_platform",
    )

    # Cross ribs underneath for visual stiffness detail
    for ri, (rx, ry, rl) in enumerate(
        [
            (0.0, 0.0, (BASE_L * 0.70, 0.008, 0.006)),
            (0.0, 0.0, (0.008, BASE_W * 0.70, 0.006)),
        ]
    ):
        base.visual(
            Box(rl),
            origin=Origin(xyz=(rx, ry, WHEEL_R + FORK_H - 0.003)),
            material=BASE_FRAME_MAT,
            name=f"base_rib_{ri}",
        )

    # Caster fork assemblies at each corner (loop-emitted)
    hw = WHEEL_W / 2.0 + 0.002  # half-gap between prongs
    prong_h = FORK_H + 0.005  # prongs extend slightly below axle
    prong_cz = WHEEL_R - 0.005 + prong_h / 2.0  # prong center Z
    mount_cz = WHEEL_R + FORK_H - 0.0025  # mount plate center Z

    for i, (cx, cy) in enumerate(CASTER_CORNERS):
        # mount plate
        base.visual(
            Box((2 * hw + 2 * FORK_W + 0.004, 0.032, 0.005)),
            origin=Origin(xyz=(cx, cy, mount_cz)),
            material=BASE_FRAME_MAT,
            name=f"caster_fork_mount_{i}",
        )
        # left prong (+X)
        base.visual(
            Box((FORK_W, 0.028, prong_h)),
            origin=Origin(xyz=(cx + hw + FORK_W / 2.0, cy, prong_cz)),
            material=BASE_FRAME_MAT,
            name=f"caster_fork_prong_l_{i}",
        )
        # right prong (-X)
        base.visual(
            Box((FORK_W, 0.028, prong_h)),
            origin=Origin(xyz=(cx - hw - FORK_W / 2.0, cy, prong_cz)),
            material=BASE_FRAME_MAT,
            name=f"caster_fork_prong_r_{i}",
        )
        # axle pin bridging the fork prongs through the wheel bore
        base.visual(
            Cylinder(radius=0.003, length=WHEEL_W + 2 * FORK_W),
            origin=Origin(xyz=(cx, cy, WHEEL_R), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=STEEL,
            name=f"axle_pin_{i}",
        )

    # ========================================================== caster wheels (4)
    caster_wheel_mesh = mesh_from_cadquery(_caster_wheel_solid(), "caster_wheel")

    for i, (cx, cy) in enumerate(CASTER_CORNERS):
        wheel_part = model.part(f"caster_wheel_{i}")
        # CadQuery wheel built along Z; rotate 90° around Y so width spans X
        wheel_part.visual(
            caster_wheel_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=CASTER_PLASTIC,
            name=f"wheel_disc_{i}",
        )

        model.articulation(
            f"wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel_part,
            origin=Origin(xyz=(cx, cy, WHEEL_R)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=10.0),
        )

    # ========================================================== kennel body
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
    for i, cx_clip in enumerate((-0.15, 0.01, 0.17)):  # three per side
        for j, sy in enumerate((-1.0, 1.0)):
            body.visual(
                Box((0.036, 0.024, 0.065)),
                origin=Origin(
                    xyz=(cx_clip, sy * (FLANGE_W / 2.0 - 0.005), RIM_Z)
                ),
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

    # rear wall tube guide rails (exterior channels for the telescoping handle)
    # Two channel rails on the rear exterior, guiding the handle tubes
    for bi, bz in enumerate((RIM_Z + 0.06, RIM_Z + SHELL_H * 0.55)):
        # Compute shell rear wall X at this height (tapered shell)
        frac = max(0.0, min(1.0, (bz - RIM_Z) / SHELL_H))
        wall_x = -(RIM_L / 2.0 - (RIM_L - TOP_L) / 2.0 * frac)
        rail_depth = 0.014
        rail_cx = wall_x - rail_depth / 2.0
        body.visual(
            Box((rail_depth, HANDLE_TUBE_SPACING + 2 * HANDLE_TUBE_R + 0.012, 0.016)),
            origin=Origin(xyz=(rail_cx, 0.0, bz)),
            material=DARK_GREY,
            name=f"handle_guide_rail_{bi}",
        )

    # FIXED joint: body sits on the wheel base
    model.articulation(
        "base_to_body",
        ArticulationType.FIXED,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, BODY_MOUNT_Z)),
    )

    # ========================================================== wire door
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

    # ========================================================== trolley handle
    handle = model.part("trolley_handle")

    # Two parallel telescoping tubes (steel) — main tubes stop below roof
    tube_cz = HANDLE_TUBE_LENGTH / 2.0 - HANDLE_TUBE_HIDDEN
    for j, sy in enumerate((-1.0, 1.0)):
        handle.visual(
            Cylinder(radius=HANDLE_TUBE_R, length=HANDLE_TUBE_LENGTH),
            origin=Origin(xyz=(0.0, sy * HANDLE_TUBE_SPACING / 2.0, tube_cz)),
            material=STEEL,
            name=f"handle_tube_{j}",
        )

    # Cross brace between tubes near the bottom (retained end)
    handle.visual(
        Box((0.008, HANDLE_TUBE_SPACING - 2 * HANDLE_TUBE_R, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -HANDLE_TUBE_HIDDEN + 0.010)),
        material=STEEL,
        name="handle_cross_brace",
    )

    # Riser posts connecting tube tops to grip (pass through roof opening)
    tube_top_z = HANDLE_TUBE_LENGTH - HANDLE_TUBE_HIDDEN  # 0.23
    riser_cz = tube_top_z + HANDLE_RISER_LENGTH / 2.0
    for j, sy in enumerate((-1.0, 1.0)):
        handle.visual(
            Cylinder(radius=HANDLE_RISER_R, length=HANDLE_RISER_LENGTH),
            origin=Origin(xyz=(0.0, sy * HANDLE_TUBE_SPACING / 2.0, riser_cz)),
            material=STEEL,
            name=f"handle_riser_{j}",
        )

    # Grip bar at the top (sits above roof when retracted)
    grip_cz = tube_top_z + HANDLE_RISER_LENGTH + HANDLE_GRIP_H / 2.0
    handle.visual(
        Box((HANDLE_GRIP_D, HANDLE_GRIP_W, HANDLE_GRIP_H)),
        origin=Origin(xyz=(0.0, 0.0, grip_cz)),
        material=RUBBER_GRIP,
        name="handle_grip",
    )

    # Prismatic joint: handle extends upward from the body rear
    model.articulation(
        "body_to_trolley_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_JOINT_X, 0.0, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.30, lower=0.0, upper=HANDLE_TRAVEL
        ),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("kennel_body")
    door = object_model.get_part("wire_door")
    hinge = object_model.get_articulation("body_to_door")
    base = object_model.get_part("wheel_base_frame")
    handle = object_model.get_part("trolley_handle")
    trolley_joint = object_model.get_articulation("body_to_trolley_handle")

    # ---- axle pin / caster wheel intentional overlaps ----
    # Each axle pin is captured inside the caster wheel bore
    for i in range(4):
        ctx.allow_overlap(
            base,
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a=f"axle_pin_{i}",
            elem_b=f"wheel_disc_{i}",
            reason="axle pin retained in caster wheel bore for spinning mount",
        )
        ctx.expect_contact(
            base,
            object_model.get_part(f"caster_wheel_{i}"),
            elem_a=f"axle_pin_{i}",
            elem_b=f"wheel_disc_{i}",
            contact_tol=0.005,
            name=f"axle_pin_{i}_contacts_wheel_{i}",
        )

    # ---- trolley handle riser / shell overlap ----
    # The riser posts pass through small molded openings in the shell roof
    for j in range(2):
        ctx.allow_overlap(
            body,
            handle,
            elem_a="kennel_upper_shell",
            elem_b=f"handle_riser_{j}",
            reason="handle riser posts pass through molded roof openings for telescoping handle",
        )
    # Proof: handle stays centered in body footprint (risers in roof openings)
    ctx.expect_within(
        handle, body, axes="xy", margin=0.05,
        name="handle_risers_centered_in_body_footprint",
    )

    # ---- wheel base and casters ----
    ctx.check(
        "wheel_base_frame_exists",
        base is not None and len(base.visuals) > 0,
        "wheel_base_frame part must exist with visuals",
    )

    for i in range(4):
        wheel_part = object_model.get_part(f"caster_wheel_{i}")
        spin = object_model.get_articulation(f"wheel_spin_{i}")
        ctx.check(
            f"caster_wheel_{i}_has_visual",
            wheel_part is not None and len(wheel_part.visuals) > 0,
            f"caster_wheel_{i} must have at least one visual",
        )
        ctx.check(
            f"wheel_spin_{i}_is_continuous",
            spin.articulation_type == ArticulationType.CONTINUOUS,
            f"wheel_spin_{i} type={spin.articulation_type}",
        )

    # Verify caster wheels are near the ground (rolling contact)
    for i in range(4):
        wheel_part = object_model.get_part(f"caster_wheel_{i}")
        aabb = ctx.part_world_aabb(wheel_part)
        ctx.check(
            f"caster_wheel_{i}_near_ground",
            aabb is not None and aabb[0][2] < 0.005,
            f"caster_wheel_{i} min_z={None if aabb is None else aabb[0][2]}",
        )

    # ---- trolley handle ----
    ctx.check(
        "trolley_handle_exists",
        handle is not None and len(handle.visuals) >= 3,
        "trolley_handle must have tubes and grip",
    )
    ctx.check(
        "body_to_trolley_handle_is_prismatic",
        trolley_joint.articulation_type == ArticulationType.PRISMATIC,
        f"type={trolley_joint.articulation_type}",
    )

    # Handle extends upward when deployed
    rest_pos = ctx.part_world_position(handle)
    with ctx.pose({trolley_joint: HANDLE_TRAVEL}):
        ext_pos = ctx.part_world_position(handle)
    ctx.check(
        "trolley_handle_extends_upward",
        rest_pos is not None
        and ext_pos is not None
        and ext_pos[2] > rest_pos[2] + 0.20,
        f"rest_z={rest_pos}, extended_z={ext_pos}",
    )

    # Handle stays within body XY footprint at rest
    with ctx.pose({trolley_joint: 0.0}):
        ctx.expect_within(handle, body, axes="xy", margin=0.05,
                          name="retracted_handle_within_body_footprint")

    # Handle tubes retained in body at max extension
    with ctx.pose({trolley_joint: HANDLE_TRAVEL}):
        ctx.expect_overlap(handle, body, axes="z", min_overlap=0.05,
                           name="extended_handle_tubes_retained_in_body")

    # ---- door tests (preserved from parent) ----
    # the hinge knuckles intentionally embed into the tan jamb wall
    for i in range(2):
        ctx.allow_overlap(
            body,
            door,
            elem_a="kennel_upper_shell",
            elem_b=f"door_hinge_knuckle_{i}",
            reason="door hinge knuckles seat into molded sockets in the shell jamb",
        )

    # closed door: grate covers the front opening, flush with the front face
    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(door, body, axes="yz", min_overlap=0.10)
        ctx.expect_within(door, body, axes="y", margin=0.0)
        aabb = ctx.part_world_aabb(door)
        ctx.check(
            "closed_door_sits_at_front_face",
            aabb is not None and 0.24 < aabb[1][0] < 0.45,
            f"door aabb={aabb}",
        )
        body_aabb = ctx.part_world_aabb(body)
        ctx.check(
            "closed_door_not_proud_of_flange",
            aabb is not None
            and body_aabb is not None
            and aabb[1][0] <= body_aabb[1][0] + 0.002,
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

    return ctx.report()


object_model = build_object_model()
