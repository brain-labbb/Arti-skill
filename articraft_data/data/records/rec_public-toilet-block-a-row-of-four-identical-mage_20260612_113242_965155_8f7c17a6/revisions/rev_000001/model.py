from __future__ import annotations

# Public toilet block: a row of four portable toilet cabins (porta-potties)
# standing side by side on a shared base, as in a festival / construction-site
# sanitation bank.
#
# Coordinate convention:
#   - up is +Z; the shared base sits on the ground at z = 0.
#   - cabins are arrayed along Y; every door faces the front (+X).
#   - the four cabins are centered about y = 0.
#
# Structure: one rigid root part `block` carries the shared base slab and the
# four ribbed cabin shells (each with a sloped translucent roof and a corner
# vent stack). The four moving parts are the four front doors `door_0..door_3`,
# each hinged on its own VERTICAL axis at its cabin's +Y front corner and each
# opening independently.

import math

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
)

# ---- single-cabin dimensions (meters) ---------------------------------------
W = 1.16               # front-back footprint (x)
DC = 1.12              # per-cabin width (y)
GAP = 0.05             # gap between adjacent cabins
PITCH = DC + GAP       # cabin-to-cabin spacing along y
N_CABINS = 4

WALL_T = 0.035
FLOOR_H = 0.10
WALL_BASE = FLOOR_H
WALL_TOP = 2.18
ROOF_T = 0.06
ROOF_RISE = 0.14
ROOF_PITCH = math.atan2(ROOF_RISE, W)
ROOF_MID_Z = WALL_TOP + 0.02
TOTAL_H = WALL_TOP + ROOF_RISE * 0.5 + ROOF_T
HEADER_H = 0.12

RIB_COUNT = 6
DOOR_GAP = 0.006
OPEN_ANGLE = math.radians(100.0)

# row layout
ROW_Y0 = -(N_CABINS - 1) * 0.5 * PITCH     # y-center of cabin 0
ROW_WIDTH = (N_CABINS - 1) * PITCH + DC


def _cabin_centers():
    return [ROW_Y0 + i * PITCH for i in range(N_CABINS)]


def _curved_roof(part, *, cy, idx, span_y, mat):
    """Molded convex (barrel-vault) roof cap for one cabin, replacing a flat
    tilted slab. A smooth circular arc curves front (+X) to back (-X) with the
    crown biased toward the back, so the roof reads higher at the rear and rounds
    gently down to the front eave, matching the rounded plastic reference roofs."""
    n = 12
    a = W * 0.5
    sagitta = 0.18
    radius = (a * a + sagitta * sagitta) / (2.0 * sagitta)
    xc = -0.12
    front_dx = abs(W * 0.5 - xc)
    front_drop = radius - math.sqrt(radius * radius - front_dx * front_dx)
    z_top = WALL_TOP + front_drop + 0.04
    step = W / n
    crown_z = z_top
    for s in range(n):
        x = -W * 0.5 + (s + 0.5) * step
        dx = x - xc
        root = math.sqrt(radius * radius - dx * dx)
        z = z_top - (radius - root)
        beta = math.atan2(dx, root)
        part.visual(
            Box((step / math.cos(beta) * 1.18, span_y, ROOF_T)),
            origin=Origin(xyz=(x, cy, z + ROOF_T * 0.5 * math.cos(beta)),
                          rpy=(0.0, beta, 0.0)),
            material=mat,
            name=f"c{idx}_roof_cap_seg_{s}",
        )
        if abs(dx) < step:
            crown_z = z
    return crown_z


def _ribbed_wall(part, *, cy, idx, face, span, mat, rib_mat):
    """One ribbed wall panel for cabin `idx`, centered at y=cy, on the given
    outer face (+/-x relative to the cabin, or +/-y in row coordinates)."""
    z0, z1 = WALL_BASE, WALL_TOP
    h = z1 - z0
    zc = 0.5 * (z0 + z1)
    if face in ("+x", "-x"):
        sign = 1.0 if face == "+x" else -1.0
        x = sign * (W * 0.5 - WALL_T * 0.5)
        part.visual(
            Box((WALL_T, span, h)),
            origin=Origin(xyz=(x, cy, zc)),
            material=mat,
            name=f"c{idx}_wall_{'px' if sign > 0 else 'back'}_panel",
        )
        for r in range(RIB_COUNT):
            y = cy - span * 0.5 + span * (r + 0.5) / RIB_COUNT
            part.visual(
                Box((0.016, 0.022, h * 0.94)),
                origin=Origin(xyz=(sign * (W * 0.5 + 0.004), y, zc)),
                material=rib_mat,
                name=f"c{idx}_wall_{'px' if sign > 0 else 'back'}_rib_{r}",
            )
    else:
        sign = 1.0 if face == "+y" else -1.0
        y = cy + sign * (DC * 0.5 - WALL_T * 0.5)
        part.visual(
            Box((span, WALL_T, h)),
            origin=Origin(xyz=(0.0, y, zc)),
            material=mat,
            name=f"c{idx}_wall_{'yp' if sign > 0 else 'ym'}_panel",
        )
        for r in range(RIB_COUNT):
            x = -span * 0.5 + span * (r + 0.5) / RIB_COUNT
            part.visual(
                Box((0.022, 0.016, h * 0.94)),
                origin=Origin(xyz=(x, cy + sign * (DC * 0.5 + 0.004), zc)),
                material=rib_mat,
                name=f"c{idx}_wall_{'yp' if sign > 0 else 'ym'}_rib_{r}",
            )


def _add_cabin_body(part, cy, idx, mats):
    """Add cabin `idx`'s static shell (3 ribbed walls, front frame, roof, vent)
    to the shared root `part`, centered at y=cy."""
    magenta, magenta_dk, roof_white, dark, vent_white = mats

    # side walls (+Y, -Y) and back wall (-X); front (+X) is the door
    _ribbed_wall(part, cy=cy, idx=idx, face="+y", span=DC - 2 * WALL_T, mat=magenta, rib_mat=magenta_dk)
    _ribbed_wall(part, cy=cy, idx=idx, face="-y", span=DC - 2 * WALL_T, mat=magenta, rib_mat=magenta_dk)
    _ribbed_wall(part, cy=cy, idx=idx, face="-x", span=DC - 2 * WALL_T, mat=magenta, rib_mat=magenta_dk)

    # front header above the door opening
    part.visual(
        Box((WALL_T, DC - 2 * WALL_T, HEADER_H)),
        origin=Origin(xyz=(W * 0.5 - WALL_T * 0.5, cy, WALL_TOP - HEADER_H * 0.5)),
        material=magenta,
        name=f"c{idx}_front_header",
    )
    # front corner jambs (door seats against the +Y jamb hinge side)
    jamb_h = WALL_TOP - WALL_BASE
    for sy in (1.0, -1.0):
        part.visual(
            Box((WALL_T, 0.05, jamb_h)),
            origin=Origin(xyz=(W * 0.5 - WALL_T * 0.5, cy + sy * (DC * 0.5 - WALL_T - 0.025), WALL_BASE + jamb_h * 0.5)),
            material=magenta,
            name=f"c{idx}_front_jamb_{'p' if sy > 0 else 'm'}",
        )

    # molded convex translucent roof cap (curves up, higher at back): thin eave
    # lip overhanging the walls, then the curved barrel-vault cap on top
    part.visual(
        Box((W + 0.06, DC + 0.04, 0.03)),
        origin=Origin(xyz=(0.0, cy, WALL_TOP - 0.005), rpy=(0.0, ROOF_PITCH, 0.0)),
        material=roof_white,
        name=f"c{idx}_roof_eave",
    )
    crown_z = _curved_roof(part, cy=cy, idx=idx, span_y=DC + 0.02, mat=roof_white)
    part.visual(
        Box((0.18, 0.18, 0.06)),
        origin=Origin(xyz=(-W * 0.22, cy, crown_z + 0.05)),
        material=vent_white,
        name=f"c{idx}_roof_vent",
    )
    # vent stack up the rear-left corner of the cabin
    stack_x = -W * 0.5 + 0.12
    stack_y = cy + DC * 0.5 - 0.12
    stack_top = WALL_TOP + 0.26
    part.visual(
        Cylinder(radius=0.045, length=stack_top),
        origin=Origin(xyz=(stack_x, stack_y, stack_top * 0.5)),
        material=dark,
        name=f"c{idx}_vent_stack",
    )
    part.visual(
        Cylinder(radius=0.060, length=0.04),
        origin=Origin(xyz=(stack_x, stack_y, stack_top + 0.01)),
        material=dark,
        name=f"c{idx}_vent_stack_cap",
    )


def _door_params():
    door_w = (DC - 2 * WALL_T) - 2 * DOOR_GAP
    door_z0 = WALL_BASE + DOOR_GAP
    door_z1 = WALL_TOP - HEADER_H - DOOR_GAP
    # door_face_x = 0: each door is authored in a LOCAL frame whose origin is its
    # own vertical hinge (the cabin's +Y front corner), leaf face at local x = 0
    # and leaf along -Y. The joint origin places that hinge at the real front-face
    # corner, so the leaf swings about its own edge instead of the cabin center.
    door_face_x = 0.0
    return door_w, door_z0, door_z1, door_face_x


HINGE_X = W * 0.5 - WALL_T - 0.004    # body-frame x of each door's front-face hinge


def _build_door(part, idx, mats):
    """Build door leaf + hardware for cabin `idx`. Authored in a LOCAL frame with
    the hinge at local y=0 and the leaf extending along -Y (door faces +X)."""
    magenta, magenta_dk, roof_white, dark, vent_white = mats
    indicator_mat = vent_white  # overwritten by caller-added indicator material

    door_w, door_z0, door_z1, door_face_x = _door_params()
    leaf_cy = -door_w * 0.5
    door_h = door_z1 - door_z0
    door_zc = 0.5 * (door_z0 + door_z1)

    part.visual(
        Box((0.045, door_w, door_h)),
        origin=Origin(xyz=(door_face_x, leaf_cy, door_zc)),
        material=magenta,
        name=f"door{idx}_leaf",
    )
    part.visual(
        Cylinder(radius=0.024, length=door_h),
        origin=Origin(xyz=(door_face_x, 0.010, door_zc)),
        material=dark,
        name=f"door{idx}_hinge_stile",
    )
    for r in range(3):
        y = leaf_cy - door_w * 0.3 + door_w * 0.6 * r / 2.0
        part.visual(
            Box((0.014, 0.020, door_h * 0.9)),
            origin=Origin(xyz=(door_face_x + 0.026, y, door_zc)),
            material=magenta_dk,
            name=f"door{idx}_rib_{r}",
        )
    for s in range(4):
        part.visual(
            Box((0.016, door_w * 0.62, 0.013)),
            origin=Origin(xyz=(door_face_x + 0.027, leaf_cy, door_z1 - 0.06 - s * 0.025)),
            material=dark,
            name=f"door{idx}_vent_slat_{s}",
        )
    return door_w, door_z0, door_z1, door_face_x, leaf_cy, door_zc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="public_toilet_block")

    magenta = model.material("cabin_magenta", rgba=(0.74, 0.10, 0.30, 1.0))
    magenta_dk = model.material("cabin_magenta_dark", rgba=(0.57, 0.07, 0.22, 1.0))
    roof_white = model.material("roof_white", rgba=(0.90, 0.90, 0.88, 0.92))
    dark = model.material("dark_plastic", rgba=(0.11, 0.11, 0.13, 1.0))
    vent_white = model.material("vent_white", rgba=(0.85, 0.85, 0.84, 1.0))
    indicator_mat = model.material("occupancy_indicator", rgba=(0.85, 0.18, 0.16, 1.0))
    mats = (magenta, magenta_dk, roof_white, dark, vent_white)

    centers = _cabin_centers()

    # ===================== BLOCK (root) ==================================
    block = model.part("block")
    # shared base slab spanning all cabins
    block.visual(
        Box((W + 0.04, ROW_WIDTH + 0.10, FLOOR_H)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_H * 0.5)),
        material=magenta_dk,
        name="base_slab",
    )
    # dark skids front and back under the shared base
    for sx in (1.0, -1.0):
        block.visual(
            Box((0.12, ROW_WIDTH + 0.10, 0.04)),
            origin=Origin(xyz=(sx * (W * 0.5 - 0.10), 0.0, 0.02)),
            material=dark,
            name=f"skid_{'front' if sx > 0 else 'back'}",
        )
    for i, cy in enumerate(centers):
        _add_cabin_body(block, cy, i, mats)
    block.inertial = Inertial.from_geometry(
        Box((W, ROW_WIDTH, TOTAL_H)),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, TOTAL_H * 0.45)),
    )

    # ===================== DOORS (children) ==============================
    door_w, door_z0, door_z1, door_face_x = _door_params()
    leaf_cy = -door_w * 0.5
    door_zc = 0.5 * (door_z0 + door_z1)
    latch_y = -door_w + 0.05

    for i, cy in enumerate(centers):
        door = model.part(f"door_{i}")
        _build_door(door, i, mats)
        door.visual(
            Box((0.03, 0.05, 0.09)),
            origin=Origin(xyz=(door_face_x + 0.030, latch_y, door_zc + 0.18)),
            material=indicator_mat,
            name=f"door{i}_occupancy_indicator",
        )
        door.visual(
            Box((0.05, 0.026, 0.12)),
            origin=Origin(xyz=(door_face_x + 0.036, latch_y, door_zc)),
            material=dark,
            name=f"door{i}_handle",
        )
        door.inertial = Inertial.from_geometry(
            Box((0.05, door_w, door_z1 - door_z0)),
            mass=13.0,
            origin=Origin(xyz=(door_face_x, leaf_cy, door_zc)),
        )
        # hinge at the cabin's +Y front corner, in row coordinates
        hinge_y = cy + DC * 0.5 - WALL_T - DOOR_GAP
        model.articulation(
            f"block_to_door_{i}",
            ArticulationType.REVOLUTE,
            parent=block,
            child=door,
            origin=Origin(xyz=(HINGE_X, hinge_y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=OPEN_ANGLE),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    block = object_model.get_part("block")
    centers = _cabin_centers()

    # block rests on the ground
    baabb = ctx.part_world_aabb(block)
    ctx.check(
        "block base rests at z=0",
        baabb is not None and abs(baabb[0][2]) < 1e-3,
        details=f"block_min_z={None if baabb is None else baabb[0][2]}",
    )
    ctx.check(
        "block is roughly cabin height (2.2-2.6 m)",
        baabb is not None and 2.1 < baabb[1][2] < 2.7,
        details=f"block_top_z={None if baabb is None else baabb[1][2]}",
    )
    # row is wide (several cabins side by side)
    ctx.check(
        "row spans the four cabins in y",
        baabb is not None and (baabb[1][1] - baabb[0][1]) > 3.5,
        details=f"row_width={None if baabb is None else baabb[1][1] - baabb[0][1]}",
    )

    # exactly four doors, each an independent revolute about vertical, each opening
    for i, cy in enumerate(centers):
        door = object_model.get_part(f"door_{i}")
        hinge = object_model.get_articulation(f"block_to_door_{i}")
        ctx.check(
            f"door_{i} hinge is revolute about vertical (Z)",
            str(hinge.articulation_type).endswith("REVOLUTE")
            and abs(hinge.axis[0]) < 1e-6 and abs(hinge.axis[1]) < 1e-6
            and abs(abs(hinge.axis[2]) - 1.0) < 1e-6,
            details=f"type={hinge.articulation_type}, axis={hinge.axis}",
        )
        # each door sits in its own cabin band (centered near cy) when closed
        daabb = ctx.part_world_aabb(door)
        if daabb is not None:
            dyc = 0.5 * (daabb[0][1] + daabb[1][1])
            ctx.check(
                f"door_{i} is on the front face and in its cabin band",
                daabb[1][0] > W * 0.5 - 0.12 and abs(dyc - cy) < DC,
                details=f"door_max_x={daabb[1][0]}, door_yc={dyc}, cabin_cy={cy}",
            )
        # opening swings this door's free edge outward (+X)
        rest = ctx.part_world_aabb(door)
        rest_max_x = rest[1][0] if rest else None
        with ctx.pose({hinge: OPEN_ANGLE}):
            oa = ctx.part_world_aabb(door)
            open_max_x = oa[1][0] if oa else None
        ctx.check(
            f"door_{i} opens outward (+X)",
            rest_max_x is not None and open_max_x is not None and open_max_x > rest_max_x + 0.18,
            details=f"rest_max_x={rest_max_x}, open_max_x={open_max_x}",
        )

    # doors are independent: opening door_0 leaves door_1 closed
    d0 = object_model.get_articulation("block_to_door_0")
    d1 = object_model.get_part("door_1")
    d1_rest = ctx.part_world_aabb(d1)
    with ctx.pose({d0: OPEN_ANGLE}):
        d1_when_d0_open = ctx.part_world_aabb(d1)
    ctx.check(
        "doors move independently (opening door_0 does not move door_1)",
        d1_rest is not None and d1_when_d0_open is not None
        and abs(d1_rest[1][0] - d1_when_d0_open[1][0]) < 1e-6,
        details="door_1 must stay put while door_0 opens",
    )

    # each closed door seats into its cabin opening (small jamb embed)
    for i in range(len(centers)):
        ctx.allow_overlap(
            block,
            object_model.get_part(f"door_{i}"),
            reason="Each closed door seats into its cabin's front opening between the corner jambs.",
        )

    return ctx.report()


object_model = build_object_model()
