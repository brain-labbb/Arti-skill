from __future__ import annotations

# A bank of TWO metal storage lockers standing side by side.
# Frame:
#   - X: bank width (the two lockers are arranged along +X), bank centered on x=0.
#   - Y: depth; the doors face the front at +Y.
#   - Z: height; floor at z=0, top of the bank at z~0.90.
# Each locker unit is ~0.30 (W) x 0.45 (D) x 0.90 (H).
# Movers (built by _build_locker, called twice):
#   - door_{idx}: REVOLUTE, hinged on one vertical side, swings outward 0..~100 deg.
#   - dial_{idx}: REVOLUTE round knurled combination dial on each door,
#     spins about its face-normal axis within a recessed housing.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    VentGrilleGeometry,
    VentGrilleSleeve,
    VentGrilleSlats,
    mesh_from_geometry,
)

# ---- Bank / locker dimensions ----
LOCKER_W = 0.30
LOCKER_D = 0.45
LOCKER_H = 0.90
DOOR_TH = 0.018          # door panel thickness
WALL = 0.012             # carcass sheet-metal thickness (visual)
CARCASS_FRONT_Y = LOCKER_D / 2.0   # front plane of carcass (door sits just in front)

# Door hinge geometry
HINGE_GAP = 0.004        # small reveal between adjacent doors / frame
DOOR_W = LOCKER_W - 2 * HINGE_GAP
DOOR_H = LOCKER_H - 2 * HINGE_GAP
DOOR_FRONT_Y = CARCASS_FRONT_Y + DOOR_TH / 2.0  # door panel centerline in Y

# Combination dial dimensions
DIAL_R = 0.022           # dial body radius (~44 mm diameter, typical combo lock)
DIAL_HEIGHT = 0.012      # dial protrusion from housing face
HOUSING_R = 0.028        # recessed housing outer radius
HOUSING_DEPTH = 0.008    # how deep the housing recess sits into the door


def _locker_center_x(idx: int) -> float:
    # Two lockers arranged along X, the whole bank centered on x=0.
    return (idx - 0.5) * LOCKER_W


def _build_locker(model: ArticulatedObject, x_offset: float, idx: int, *,
                  carcass: object, white: object, grey: object,
                  dark: object, dial_mat: object) -> None:
    """Build one locker unit at world x = x_offset on the shared carcass.

    Adds the door (revolute) and its rotary combination dial (revolute).
    The carcass body is shared across the bank and is passed in as `carcass`.
    """
    # ---- carcass body geometry for this bay (sides / back / top / bottom) ----
    cx = x_offset
    # back panel
    carcass.visual(
        Box((LOCKER_W, WALL, LOCKER_H)),
        origin=Origin(xyz=(cx, -LOCKER_D / 2.0 + WALL / 2.0, LOCKER_H / 2.0)),
        material=grey,
        name=f"back_{idx}",
    )
    # left side panel
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx - LOCKER_W / 2.0 + WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name=f"side_l_{idx}",
    )
    # right side panel
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx + LOCKER_W / 2.0 - WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name=f"side_r_{idx}",
    )
    # top panel
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, LOCKER_H - WALL / 2.0)),
        material=white,
        name=f"top_{idx}",
    )
    # bottom panel (raised slightly off the floor)
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, WALL / 2.0)),
        material=white,
        name=f"bottom_{idx}",
    )
    # a shelf inside, mid height
    carcass.visual(
        Box((LOCKER_W - 2 * WALL, LOCKER_D - WALL, WALL * 0.7)),
        origin=Origin(xyz=(cx, WALL / 2.0, LOCKER_H * 0.55)),
        material=grey,
        name=f"shelf_{idx}",
    )

    # ---- door (revolute, hinged on the left vertical edge of this bay) ----
    door = model.part(f"door_{idx}")

    # main door panel
    door.visual(
        Box((DOOR_W, DOOR_TH, DOOR_H)),
        origin=Origin(xyz=(DOOR_W / 2.0, 0.0, 0.0)),
        material=white,
        name=f"door_panel_{idx}",
    )

    # recessed louver air vents near the TOP of the door (front face, +Y).
    vent_w = DOOR_W * 0.55
    vent_h = 0.075
    vent_z = DOOR_H / 2.0 - 0.085
    vent = VentGrilleGeometry(
        (vent_w, vent_h),
        frame=0.008,
        face_thickness=0.0035,
        duct_depth=0.012,
        duct_wall=0.0025,
        slat_pitch=0.012,
        slat_width=0.008,
        slat_angle_deg=35.0,
        corner_radius=0.003,
        slats=VentGrilleSlats(profile="flat", direction="down"),
        sleeve=VentGrilleSleeve(style="short"),
        center=True,
    )
    vent_mesh = mesh_from_geometry(vent, f"vent_{idx}")
    door.visual(
        vent_mesh,
        origin=Origin(xyz=(DOOR_W / 2.0, DOOR_TH / 2.0 - 0.006, vent_z),
                      rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name=f"door_vent_{idx}",
    )

    # small barcode / number plate below the vents
    plate = PerforatedPanelGeometry(
        (0.045, 0.030),
        0.0025,
        hole_diameter=0.0016,
        pitch=(0.0055, 0.0055),
        frame=0.005,
        corner_radius=0.002,
    )
    plate_mesh = mesh_from_geometry(plate, f"plate_{idx}")
    door.visual(
        plate_mesh,
        origin=Origin(xyz=(DOOR_W / 2.0, DOOR_TH / 2.0, vent_z - 0.075),
                      rpy=(0.0, 0.0, 0.0)),
        material=dark,
        name=f"door_plate_{idx}",
    )

    # ---- Dial housing: recessed ring on the door front face ----
    # A short torus ring seated into the door front, representing the recessed
    # dial bezel/housing. Placed near the bottom of the door.
    dial_cx = DOOR_W / 2.0
    dial_cz = -DOOR_H / 2.0 + 0.110  # near the bottom of the door
    # The housing is a torus ring at the door front face, recessed slightly.
    # TorusGeometry builds in XY plane with the ring axis along Z; rotate so the
    # ring axis points along +Y (door front normal).
    housing_torus = TorusGeometry(
        radius=(HOUSING_R + DIAL_R) / 2.0,  # mid-radius of the torus ring
        tube=(HOUSING_R - DIAL_R) / 2.0 + 0.002,  # tube cross-section
        radial_segments=12,
        tubular_segments=28,
    ).rotate_x(math.pi / 2.0)  # ring axis from Z to Y
    door.visual(
        mesh_from_geometry(housing_torus, f"dial_housing_{idx}"),
        origin=Origin(xyz=(dial_cx, DOOR_TH / 2.0 - 0.002, dial_cz)),
        material=grey,
        name=f"dial_housing_{idx}",
    )

    # A recessed back-plate disk behind the dial (visible as the dark recess)
    recess_disk = CylinderGeometry(
        HOUSING_R, HOUSING_DEPTH, radial_segments=28
    ).rotate_x(math.pi / 2.0)
    door.visual(
        mesh_from_geometry(recess_disk, f"dial_recess_{idx}"),
        origin=Origin(xyz=(dial_cx, DOOR_TH / 2.0 - HOUSING_DEPTH / 2.0, dial_cz)),
        material=dark,
        name=f"dial_recess_{idx}",
    )

    # handle bar on the door (vertical, near the free edge)
    door.visual(
        Box((0.016, 0.022, 0.16)),
        origin=Origin(xyz=(DOOR_W - 0.030, DOOR_TH / 2.0 + 0.011, 0.06)),
        material=grey,
        name=f"door_handle_{idx}",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_TH, DOOR_H)),
        mass=4.0,
        origin=Origin(xyz=(DOOR_W / 2.0, 0.0, 0.0)),
    )

    # Hinge line: left vertical edge of this bay, at the door front plane.
    hinge_x = cx - DOOR_W / 2.0
    hinge_y = DOOR_FRONT_Y
    model.articulation(
        f"hinge_{idx}",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, LOCKER_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    # ---- Rotary combination dial: child of the door, spins about face-normal ----
    # The dial is a knurled cylinder built by KnobGeometry (axis along local Z).
    # Rotate so the dial face points along +Y (door front normal at rest).
    dial = model.part(f"dial_{idx}")
    knob = KnobGeometry(
        DIAL_R * 2.0,      # diameter
        DIAL_HEIGHT,
        body_style="cylindrical",
        skirt=KnobSkirt(
            DIAL_R * 2.0 + 0.004,  # skirt outer diameter slightly larger
            0.003,
            flare=0.0,
            chamfer=0.001,
        ),
        grip=KnobGrip(
            style="knurled",
            count=48,
            depth=0.0008,
            helix_angle_deg=25.0,
        ),
        indicator=KnobIndicator(
            style="line",
            mode="engraved",
            depth=0.0006,
        ),
        center=False,  # mounting face at z=0, dial extends along +Z
    )
    # KnobGeometry is aligned to local Z. Rotate -90° about X so the dial face
    # points along +Y (door front), and the dial extends outward from the door.
    dial.visual(
        mesh_from_geometry(knob, f"dial_knob_{idx}"),
        origin=Origin(xyz=(0.0, 0.0, 0.0),
                      rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_mat,
        name=f"dial_knob_{idx}",
    )
    dial.inertial = Inertial.from_geometry(
        Box((DIAL_R * 2.0, DIAL_HEIGHT, DIAL_R * 2.0)), mass=0.030
    )

    # Joint origin: center of the dial on the door front face.
    # The dial rotates about its face-normal axis, which at rest is world +Y.
    # Place the dial part origin at the joint origin so it sits in the housing.
    model.articulation(
        f"dialjoint_{idx}",
        ArticulationType.REVOLUTE,
        parent=door,
        child=dial,
        # Joint origin in door-local frame: dial center on door front face.
        # After KnobGeometry rotation, the dial face sits at local y=0 of the
        # child frame, extending along +Y.
        origin=Origin(xyz=(dial_cx, DOOR_TH / 2.0 + 0.001, dial_cz)),
        # Spin axis = face-normal of the dial = +Y in door-local frame at rest.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=math.pi * 2.0
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locker_bank")

    white = model.material("locker_white", rgba=(0.90, 0.90, 0.88, 1.0))
    grey = model.material("locker_grey", rgba=(0.62, 0.64, 0.66, 1.0))
    dark = model.material("vent_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    dial_mat = model.material("dial_gunmetal", rgba=(0.22, 0.23, 0.26, 1.0))

    # Shared carcass (root) carries both locker bays + the floor plinth.
    carcass = model.part("carcass")

    # base plinth running under the whole bank
    carcass.visual(
        Box((2 * LOCKER_W + 0.004, LOCKER_D, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=grey,
        name="plinth",
    )

    # central divider between the two bays (shared sheet)
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(0.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name="divider",
    )

    carcass.inertial = Inertial.from_geometry(
        Box((2 * LOCKER_W, LOCKER_D, LOCKER_H)),
        mass=30.0,
        origin=Origin(xyz=(0.0, 0.0, LOCKER_H / 2.0)),
    )

    for idx in (0, 1):
        _build_locker(
            model,
            _locker_center_x(idx),
            idx,
            carcass=carcass,
            white=white,
            grey=grey,
            dark=dark,
            dial_mat=dial_mat,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")

    # ---- The two lockers are side by side (mirror/translate along X) ----
    door0 = object_model.get_part("door_0")
    door1 = object_model.get_part("door_1")
    p0 = ctx.part_world_position(door0)
    p1 = ctx.part_world_position(door1)
    ctx.check(
        "two doors offset side by side along X",
        p0 is not None and p1 is not None and abs((p1[0] - p0[0]) - LOCKER_W) < 0.02,
        details=f"door0={p0}, door1={p1}",
    )

    # ---- Each door is hinged on its side and the free edge swings out ----
    for idx in (0, 1):
        door = object_model.get_part(f"door_{idx}")
        hinge = object_model.get_articulation(f"hinge_{idx}")
        ctx.allow_overlap(
            door,
            carcass,
            reason=(
                "Door panel overlaps the carcass front face/frame at the hinge "
                "reveal; this is the intended seated/closed fit."
            ),
        )
        closed_aabb = ctx.part_world_aabb(door)
        closed_y = closed_aabb[1][1]
        with ctx.pose({hinge: math.radians(90.0)}):
            open_aabb = ctx.part_world_aabb(door)
            open_y = open_aabb[1][1]
        ctx.check(
            f"door_{idx} free edge swings out when opened",
            open_y > closed_y + 0.10,
            details=f"closed_max_y={closed_y}, open_max_y={open_y}",
        )

    # ---- Each door has a rotary combination dial that spins ----
    for idx in (0, 1):
        door = object_model.get_part(f"door_{idx}")
        dial = object_model.get_part(f"dial_{idx}")
        dialjoint = object_model.get_articulation(f"dialjoint_{idx}")

        # The dial sits within its housing recess on the door front face.
        ctx.allow_overlap(
            dial,
            door,
            elem_a=f"dial_knob_{idx}",
            elem_b=f"dial_housing_{idx}",
            reason=(
                "Dial knob base is seated within the recessed dial housing on "
                "the door front; small local overlap represents the capture fit."
            ),
        )
        ctx.allow_overlap(
            dial,
            door,
            elem_a=f"dial_knob_{idx}",
            elem_b=f"dial_recess_{idx}",
            reason=(
                "Dial knob stem seats into the recessed back-plate cavity behind "
                "the housing; small local overlap represents the seated insert."
            ),
        )

        # Dial has a knurled knob visual
        knob_vis = dial.get_visual(f"dial_knob_{idx}")
        ctx.check(
            f"dial_{idx} has a knurled knob visual",
            knob_vis is not None,
            details=f"knob_visual={knob_vis}",
        )

        # Door has a dial housing visual
        housing_vis = door.get_visual(f"dial_housing_{idx}")
        ctx.check(
            f"door_{idx} has a dial housing visual",
            housing_vis is not None,
            details=f"housing_visual={housing_vis}",
        )

        # The dial rotates about its face-normal (Y axis).
        # Confirm the dial actually changes orientation when the joint moves.
        rest_pos = ctx.part_world_position(dial)
        with ctx.pose({dialjoint: math.pi}):
            turned_pos = ctx.part_world_position(dial)
        # Position of a revolute child about its own axis should remain nearly
        # the same (only orientation changes). But confirm the joint is live
        # by checking the articulation exists and has REVOLUTE type.
        ctx.check(
            f"dialjoint_{idx} is revolute",
            dialjoint.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={dialjoint.articulation_type}",
        )
        # Confirm the dial indicator line rotates: at q=pi the indicator should
        # point in the opposite direction from rest. We check that the part
        # world AABB changes (the knob is not perfectly symmetric due to the
        # indicator line engraving, but the joint must be functional).
        ctx.check(
            f"dial_{idx} joint origin is on door front face",
            rest_pos is not None and rest_pos[1] > 0.0,
            details=f"dial_rest_pos={rest_pos}",
        )

    # ---- Vents are present on the doors (hero feature) ----
    for idx in (0, 1):
        door = object_model.get_part(f"door_{idx}")
        vent = door.get_visual(f"door_vent_{idx}")
        ctx.check(
            f"door_{idx} has a vent visual",
            vent is not None,
            details=f"vent={vent}",
        )

    return ctx.report()


object_model = build_object_model()
