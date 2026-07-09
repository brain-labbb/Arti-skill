from __future__ import annotations

# A bank of TWO metal storage lockers standing side by side.
# VARIANT: padlock hasp mechanism instead of push-button combination locks.
# Frame:
#   - X: bank width (the two lockers are arranged along +X), bank centered on x=0.
#   - Y: depth; the doors face the front at +Y.
#   - Z: height; floor at z=0, top of the bank at z~0.90.
# Each locker unit is ~0.30 (W) x 0.45 (D) x 0.90 (H).
# Movers (built by _build_locker, called twice):
#   - door_{idx}: REVOLUTE, hinged on one vertical side, swings outward 0..~100 deg.
#   - hasp_{idx}: REVOLUTE, hinged on the door near the free edge, swings from
#     q=0 (arm hanging down, slot engaged with staple) to q~90 deg (arm swung
#     outward, disengaged).

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
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

# ---- Hasp arm dimensions ----
HASP_W = 0.028           # arm width (X direction)
HASP_L = 0.065           # arm length (from hinge to far end)
HASP_TH = 0.003          # arm thickness
HASP_SETBACK = 0.025     # distance from door free edge to hasp hinge

# Slot in the hasp arm (near the far end)
SLOT_W = 0.020           # slot width (X)
SLOT_L = 0.022           # slot length (along arm)
SLOT_OFFSET = 0.015      # distance from arm far end to slot center

# ---- Staple dimensions (on carcass) ----
STAPLE_PLATE_W = 0.038
STAPLE_PLATE_H = 0.038
STAPLE_PLATE_TH = 0.003
STAPLE_LEG_SPAN = 0.014  # center-to-center distance between legs
STAPLE_LEG_W = 0.004     # leg width (X)
STAPLE_LEG_D = 0.004     # leg depth (Z)
STAPLE_LEG_PROTRUSION = 0.025  # leg protrusion from carcass front (Y)
STAPLE_BAR_H = 0.004     # crossbar height (Z)
STAPLE_BAR_D = 0.004     # crossbar depth (Y)

# ---- Padlock dimensions ----
PADLOCK_W = 0.038        # body width
PADLOCK_D = 0.015        # body depth (front to back, Y)
PADLOCK_H = 0.030        # body height
SHACKLE_ROD = 0.004      # shackle rod diameter
SHACKLE_SPAN = 0.008     # shackle inner span (X, fits between staple legs)
SHACKLE_H = 0.015        # shackle leg height


def _locker_center_x(idx: int) -> float:
    return (idx - 0.5) * LOCKER_W


def _hasp_arm_mesh(idx: int):
    """Build the hasp arm plate with a rectangular slot near the far end."""
    hw = HASP_W / 2.0
    hl = HASP_L / 2.0
    outer = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]

    slot_cy = hl - SLOT_OFFSET
    shw = SLOT_W / 2.0
    shl = SLOT_L / 2.0
    hole = [
        (-shw, slot_cy - shl),
        (shw, slot_cy - shl),
        (shw, slot_cy + shl),
        (-shw, slot_cy + shl),
    ]

    geom = ExtrudeWithHolesGeometry(
        outer, [hole], HASP_TH, cap=True, center=True,
    )
    return mesh_from_geometry(geom, f"hasp_arm_{idx}")


def _staple_world_z() -> float:
    """World Z of the staple center, aligned with the hasp arm slot at q=0."""
    return LOCKER_H / 2.0 - HASP_L + SLOT_OFFSET


def _staple_world_x(cx: float) -> float:
    """World X of the staple, aligned with the hasp hinge on the door."""
    return cx + DOOR_W / 2.0 - HASP_SETBACK


def _build_locker(model: ArticulatedObject, x_offset: float, idx: int, *,
                  carcass: object, white: object, grey: object,
                  dark: object, steel: object, brass: object) -> None:
    """Build one locker unit at world x = x_offset on the shared carcass.

    Adds the door (revolute), its hasp arm (revolute), staple, and padlock.
    The carcass body is shared across the bank and is passed in as `carcass`.
    """
    cx = x_offset

    # ---- carcass body geometry for this bay ----
    carcass.visual(
        Box((LOCKER_W, WALL, LOCKER_H)),
        origin=Origin(xyz=(cx, -LOCKER_D / 2.0 + WALL / 2.0, LOCKER_H / 2.0)),
        material=grey,
        name=f"back_{idx}",
    )
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx - LOCKER_W / 2.0 + WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name=f"side_l_{idx}",
    )
    carcass.visual(
        Box((WALL, LOCKER_D, LOCKER_H)),
        origin=Origin(xyz=(cx + LOCKER_W / 2.0 - WALL / 2.0, 0.0, LOCKER_H / 2.0)),
        material=white,
        name=f"side_r_{idx}",
    )
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, LOCKER_H - WALL / 2.0)),
        material=white,
        name=f"top_{idx}",
    )
    carcass.visual(
        Box((LOCKER_W, LOCKER_D, WALL)),
        origin=Origin(xyz=(cx, 0.0, WALL / 2.0)),
        material=white,
        name=f"bottom_{idx}",
    )
    carcass.visual(
        Box((LOCKER_W - 2 * WALL, LOCKER_D - WALL, WALL * 0.7)),
        origin=Origin(xyz=(cx, WALL / 2.0, LOCKER_H * 0.55)),
        material=grey,
        name=f"shelf_{idx}",
    )

    # ---- door (revolute, hinged on the left vertical edge of this bay) ----
    door = model.part(f"door_{idx}")

    door.visual(
        Box((DOOR_W, DOOR_TH, DOOR_H)),
        origin=Origin(xyz=(DOOR_W / 2.0, 0.0, 0.0)),
        material=white,
        name=f"door_panel_{idx}",
    )

    # recessed louver air vents near the TOP of the door
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

    # hasp bracket plate on the door front face near the free edge
    bracket_w = HASP_W + 0.012
    bracket_h = 0.028
    door.visual(
        Box((bracket_w, 0.006, bracket_h)),
        origin=Origin(xyz=(DOOR_W - HASP_SETBACK, DOOR_TH / 2.0 + 0.003, 0.0)),
        material=grey,
        name=f"hasp_bracket_{idx}",
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

    # Door hinge: left vertical edge of this bay, at the door front plane.
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

    # ---- hasp arm (revolute, child of door, swings down over staple) ----
    hasp = model.part(f"hasp_{idx}")

    # The arm plate visual: built in XY with slot near +Y, then rotated so it
    # hangs in -Z from the hinge point.  Rx(-pi/2) maps Y->-Z and Z->Y.
    arm_mesh = _hasp_arm_mesh(idx)
    hasp.visual(
        arm_mesh,
        origin=Origin(xyz=(0.0, 0.0, -HASP_L / 2.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name=f"hasp_arm_{idx}",
    )

    hasp.inertial = Inertial.from_geometry(
        Box((HASP_W, HASP_TH, HASP_L)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -HASP_L / 2.0)),
    )

    # Hasp articulation: parent=door, child=hasp.
    # Origin in door-local frame at the hasp hinge point (on the bracket).
    hasp_hinge_local_x = DOOR_W - HASP_SETBACK
    hasp_hinge_local_y = DOOR_TH / 2.0 + 0.006  # at bracket outer face
    hasp_hinge_local_z = 0.0  # at door vertical center

    # Axis (1,0,0) in door-local: horizontal, parallel to door face.
    # At q=0 the arm hangs in -Z (down, engaged).
    # Positive q rotates -Z toward +Y (arm swings outward/upward = disengaged).
    model.articulation(
        f"hasp_{idx}",
        ArticulationType.REVOLUTE,
        parent=door,
        child=hasp,
        origin=Origin(xyz=(hasp_hinge_local_x, hasp_hinge_local_y, hasp_hinge_local_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=1.5,
            lower=0.0,
            upper=math.radians(90.0),
        ),
    )

    # ---- staple plate + loop on the carcass (near the door free edge) ----
    staple_x = _staple_world_x(cx)
    staple_z = _staple_world_z()
    staple_front_y = CARCASS_FRONT_Y  # carcass front face

    # base plate
    carcass.visual(
        Box((STAPLE_PLATE_W, STAPLE_PLATE_TH, STAPLE_PLATE_H)),
        origin=Origin(xyz=(staple_x, staple_front_y + STAPLE_PLATE_TH / 2.0, staple_z)),
        material=steel,
        name=f"staple_plate_{idx}",
    )

    # left leg
    leg_base_y = staple_front_y + STAPLE_PLATE_TH
    carcass.visual(
        Box((STAPLE_LEG_W, STAPLE_LEG_PROTRUSION, STAPLE_LEG_D)),
        origin=Origin(xyz=(staple_x - STAPLE_LEG_SPAN / 2.0,
                           leg_base_y + STAPLE_LEG_PROTRUSION / 2.0,
                           staple_z)),
        material=steel,
        name=f"staple_leg_l_{idx}",
    )
    # right leg
    carcass.visual(
        Box((STAPLE_LEG_W, STAPLE_LEG_PROTRUSION, STAPLE_LEG_D)),
        origin=Origin(xyz=(staple_x + STAPLE_LEG_SPAN / 2.0,
                           leg_base_y + STAPLE_LEG_PROTRUSION / 2.0,
                           staple_z)),
        material=steel,
        name=f"staple_leg_r_{idx}",
    )
    # crossbar connecting the leg tips
    bar_span = STAPLE_LEG_SPAN + STAPLE_LEG_W
    carcass.visual(
        Box((bar_span, STAPLE_BAR_D, STAPLE_BAR_H)),
        origin=Origin(xyz=(staple_x,
                           leg_base_y + STAPLE_LEG_PROTRUSION - STAPLE_BAR_D / 2.0,
                           staple_z)),
        material=steel,
        name=f"staple_bar_{idx}",
    )

    # ---- padlock body + shackle (visuals on carcass, hangs from staple) ----
    # The padlock body overlaps the staple crossbar to maintain carcass
    # connectivity (padlock → crossbar → legs → plate → carcass shell).
    pad_x = staple_x
    crossbar_outer_y = leg_base_y + STAPLE_LEG_PROTRUSION
    # Padlock body center Y placed so inner face penetrates the crossbar slightly
    pad_y = crossbar_outer_y + PADLOCK_D / 2.0 - 0.002
    # Body top at crossbar Z, body hangs below
    pad_z = staple_z - PADLOCK_H / 2.0

    # padlock body
    carcass.visual(
        Box((PADLOCK_W, PADLOCK_D, PADLOCK_H)),
        origin=Origin(xyz=(pad_x, pad_y, pad_z)),
        material=brass,
        name=f"padlock_body_{idx}",
    )

    # shackle: two legs + crown bar above the crossbar
    # Legs extend from inside the padlock body top up through/above the crossbar
    shackle_leg_len = SHACKLE_H + 0.006  # extra for overlap into body and crown
    shackle_leg_z = staple_z + SHACKLE_H / 2.0  # legs straddle the crossbar height

    # shackle left leg (vertical cylinder)
    carcass.visual(
        Cylinder(SHACKLE_ROD / 2.0, shackle_leg_len),
        origin=Origin(xyz=(pad_x - SHACKLE_SPAN / 2.0, pad_y, shackle_leg_z)),
        material=steel,
        name=f"shackle_leg_l_{idx}",
    )
    # shackle right leg (vertical cylinder)
    carcass.visual(
        Cylinder(SHACKLE_ROD / 2.0, shackle_leg_len),
        origin=Origin(xyz=(pad_x + SHACKLE_SPAN / 2.0, pad_y, shackle_leg_z)),
        material=steel,
        name=f"shackle_leg_r_{idx}",
    )
    # shackle crown (horizontal bar connecting legs at top)
    shackle_crown_z = staple_z + SHACKLE_H + 0.003
    carcass.visual(
        Cylinder(SHACKLE_ROD / 2.0, SHACKLE_SPAN + SHACKLE_ROD),
        origin=Origin(xyz=(pad_x, pad_y, shackle_crown_z),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name=f"shackle_crown_{idx}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locker_bank")

    white = model.material("locker_white", rgba=(0.90, 0.90, 0.88, 1.0))
    grey = model.material("locker_grey", rgba=(0.62, 0.64, 0.66, 1.0))
    dark = model.material("vent_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel_dark", rgba=(0.42, 0.44, 0.48, 1.0))
    brass = model.material("padlock_brass", rgba=(0.72, 0.58, 0.22, 1.0))

    # Shared carcass (root) carries both locker bays + the floor plinth.
    carcass = model.part("carcass")

    # base plinth running under the whole bank
    carcass.visual(
        Box((2 * LOCKER_W + 0.004, LOCKER_D, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=grey,
        name="plinth",
    )

    # central divider between the two bays
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
            steel=steel,
            brass=brass,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carcass = object_model.get_part("carcass")

    # ---- The two lockers are side by side ----
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

    # ---- Hasp arms exist and swing correctly ----
    for idx in (0, 1):
        hasp = object_model.get_part(f"hasp_{idx}")
        hasp_joint = object_model.get_articulation(f"hasp_{idx}")
        door = object_model.get_part(f"door_{idx}")

        # Hasp arm overlaps the carcass (staple posts pass through the slot)
        ctx.allow_overlap(
            hasp,
            carcass,
            reason=(
                "Hasp arm slot intentionally fits over the staple legs on the "
                "carcass; this is the engaged-lock mechanical interface."
            ),
        )

        # Hasp hangs from the door
        ctx.expect_contact(
            hasp, door,
            name=f"hasp_{idx} mounted on door_{idx}",
        )

        # At q=0 the arm hangs down; at q=upper the arm swings outward/upward.
        # Use AABB (part_world_position returns the fixed hinge origin).
        rest_aabb = ctx.part_world_aabb(hasp)
        with ctx.pose({hasp_joint: math.radians(80.0)}):
            swung_aabb = ctx.part_world_aabb(hasp)

        # When swung outward, the max Y should increase (arm extends in +Y)
        ctx.check(
            f"hasp_{idx} arm swings outward when disengaged",
            rest_aabb is not None and swung_aabb is not None
            and swung_aabb[1][1] > rest_aabb[1][1] + 0.01,
            details=f"rest_max_y={rest_aabb[1][1]}, swung_max_y={swung_aabb[1][1]}",
        )

        # When swung, the min Z should rise (arm no longer hangs down)
        ctx.check(
            f"hasp_{idx} arm minimum Z rises when disengaged",
            rest_aabb is not None and swung_aabb is not None
            and swung_aabb[0][2] > rest_aabb[0][2] + 0.02,
            details=f"rest_min_z={rest_aabb[0][2]}, swung_min_z={swung_aabb[0][2]}",
        )

    # ---- Staple visuals exist on the carcass ----
    for idx in (0, 1):
        staple_plate = carcass.get_visual(f"staple_plate_{idx}")
        ctx.check(
            f"staple_plate_{idx} exists on carcass",
            staple_plate is not None,
        )
        staple_bar = carcass.get_visual(f"staple_bar_{idx}")
        ctx.check(
            f"staple_bar_{idx} (crossbar) exists on carcass",
            staple_bar is not None,
        )

    # ---- Padlock visuals exist on the carcass ----
    for idx in (0, 1):
        padlock = carcass.get_visual(f"padlock_body_{idx}")
        ctx.check(
            f"padlock_body_{idx} exists on carcass",
            padlock is not None,
        )
        crown = carcass.get_visual(f"shackle_crown_{idx}")
        ctx.check(
            f"shackle_crown_{idx} exists on carcass",
            crown is not None,
        )

    # ---- Vents are present on the doors ----
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
