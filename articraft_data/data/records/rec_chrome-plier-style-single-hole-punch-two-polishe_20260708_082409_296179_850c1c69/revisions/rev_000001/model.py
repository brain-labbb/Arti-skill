from __future__ import annotations

"""Chrome plier-style single hole punch.

Two polished steel arms cross at a central rivet pivot (scissor topology):
- the FIXED arm (root "frame") carries the upper handle and the lower jaw with
  the perforated die block that the paper rests on;
- the MOVING arm ("punch_lever") carries the lower handle and the upper jaw
  with the cylindrical punch pin.

Squeezing the lower handle up toward the upper handle rotates the lever about
the rivet (revolute joint "punch_stroke", axis +Y) and drives the punch pin
down through the paper slot into the die bore.

Orientation: in-use pose. Squeeze plane = XZ, hinge axis = Y, jaws point +X,
handles extend toward -X. The two flat arms are stacked in thin Y layers like
real stamped-steel punch pliers.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
ARM_T = 0.006  # arm plate thickness (Y)
FRAME_Y = 0.0033  # fixed-arm layer center (+Y side)
LEVER_Y = -0.0033  # moving-arm layer center (-Y side)

PIVOT_Z = 0.032  # rivet pivot height (world)
BOSS_R = 0.0095  # pivot boss disc radius
BORE_R = 0.0036  # lever boss bore; rivet shaft (r=0.0038) is a captured
RIVET_R = 0.0038  # interference fit so the pivot is physically connected

HANDLE_W = 0.011  # handle bar width (Z, in the squeeze plane)

# fixed lower jaw / die block
DIE_X0, DIE_X1 = 0.026, 0.050
DIE_Z0, DIE_Z1 = 0.002, 0.019  # die top = paper table
DIE_Y_HALF = 0.0075

PIN_R = 0.0030
PIN_X = 0.038  # punch pin / die bore axis
DIE_BORE_R = 0.0036
PIN_Z0, PIN_Z1 = 0.0220, 0.0330  # world, at rest (paper slot = 3 mm)

STROKE_MAX = 0.14  # rad; pin drop at PIN_X ~= 5.3 mm -> enters the die bore


def _bar(p0: tuple[float, float], p1: tuple[float, float], width: float, y_center: float):
    """Flat bar in the XZ squeeze plane between two (x, z) points."""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dz)
    # rotation about +Y sends +X toward -Z: theta = -atan2(dz, dx)
    theta = -math.degrees(math.atan2(dz, dx))
    cx, cz = (p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0
    return (
        cq.Workplane("XY")
        .box(length, ARM_T, width)
        .rotate((0, 0, 0), (0, 1, 0), theta)
        .translate((cx, y_center, cz))
    )


def _boss(y_center: float):
    """Pivot boss disc, axis along Y, centered on the pivot at (0, PIVOT_Z)."""
    return (
        cq.Workplane("XZ")
        .circle(BOSS_R)
        .extrude(ARM_T, both=True)
        .translate((0.0, y_center, PIVOT_Z))
        .intersect(
            cq.Workplane("XY")
            .box(0.06, ARM_T, 0.06)
            .translate((0.0, y_center, PIVOT_Z))
        )
    )


def _frame_arm_solid():
    """Fixed arm: upper handle + boss + jaw bar reaching down to the die."""
    handle = _bar((0.0, PIVOT_Z), (-0.100, PIVOT_Z + 0.024), HANDLE_W, FRAME_Y)
    jaw = _bar((0.0, PIVOT_Z), (0.036, 0.014), 0.009, FRAME_Y)
    return handle.union(_boss(FRAME_Y)).union(jaw)


def _die_block_solid():
    """Perforated die block: paper table with punch bore and chip windows."""
    blk = (
        cq.Workplane("XY")
        .box(DIE_X1 - DIE_X0, 2 * DIE_Y_HALF, DIE_Z1 - DIE_Z0)
        .translate(((DIE_X0 + DIE_X1) / 2.0, 0.0, (DIE_Z0 + DIE_Z1) / 2.0))
    )
    # vertical punch bore, aligned with the pin, through the whole block
    bore = (
        cq.Workplane("XY")
        .circle(DIE_BORE_R)
        .extrude(0.030)
        .translate((PIN_X, LEVER_Y, DIE_Z0 - 0.005))
    )
    blk = blk.cut(bore)
    # two rows of small chip-drain confetti holes through the block (Y axis)
    for hz in (0.0065, 0.0115):
        for xi in range(4):
            hx = 0.030 + 0.0055 * xi
            hole = (
                cq.Workplane("XZ")
                .circle(0.0011)
                .extrude(0.030, both=True)
                .translate((hx, 0.0, hz))
            )
            blk = blk.cut(hole)
    return blk


def _lever_arm_solid():
    """Moving arm: lower handle + bored boss + upper jaw bar (lever-local
    frame, origin at the pivot)."""
    handle = _bar((0.0, 0.0), (-0.100, -0.024), HANDLE_W, LEVER_Y)
    jaw = _bar((-0.002, 0.0), (0.042, -0.001), 0.008, LEVER_Y)
    boss = _boss(LEVER_Y).translate((0.0, 0.0, -PIVOT_Z))
    solid = handle.union(boss).union(jaw)
    # clearance bore for the rivet shaft
    bore = (
        cq.Workplane("XZ")
        .circle(BORE_R)
        .extrude(0.020, both=True)
    )
    return solid.cut(bore)


def _grip_solid(p_pivot: tuple[float, float], p_tip: tuple[float, float], y_center: float):
    """Knurled grip sleeve over the outer third of a handle."""
    dx, dz = p_tip[0] - p_pivot[0], p_tip[1] - p_pivot[1]
    a = (p_pivot[0] + dx * 0.62, p_pivot[1] + dz * 0.62)
    b = p_tip
    theta = -math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    cx, cz = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
    seg = math.hypot(b[0] - a[0], b[1] - a[1])
    return (
        cq.Workplane("XY")
        .box(seg, ARM_T + 0.0014, HANDLE_W + 0.002)
        .rotate((0, 0, 0), (0, 1, 0), theta)
        .translate((cx, y_center, cz))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plier_single_hole_punch")

    chrome = model.material("polished_chrome", rgba=(0.80, 0.82, 0.86, 1.0))
    steel = model.material("die_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    knurl = model.material("knurled_grip", rgba=(0.52, 0.54, 0.57, 1.0))
    rivet_mat = model.material("rivet_steel", rgba=(0.86, 0.88, 0.91, 1.0))

    # ------------------------------------------------------------ fixed arm
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_arm_solid(), "frame_arm"),
        name="frame_arm",
        material=chrome,
    )
    frame.visual(
        mesh_from_cadquery(_die_block_solid(), "die_block"),
        name="die_block",
        material=steel,
    )
    frame.visual(
        mesh_from_cadquery(
            _grip_solid((0.0, PIVOT_Z), (-0.100, PIVOT_Z + 0.024), FRAME_Y),
            "frame_grip",
        ),
        name="frame_grip",
        material=knurl,
    )
    # rivet: shaft through both arm layers + two proud dome caps
    frame.visual(
        Cylinder(radius=RIVET_R, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="rivet_shaft",
        material=rivet_mat,
    )
    for i, y_cap in enumerate((-0.0089, 0.0089)):
        frame.visual(
            Cylinder(radius=0.0060, length=0.0028),
            origin=Origin(xyz=(0.0, y_cap, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            name=f"rivet_cap_{i}",
            material=rivet_mat,
        )

    # ----------------------------------------------------------- moving arm
    lever = model.part("punch_lever")
    lever.visual(
        mesh_from_cadquery(_lever_arm_solid(), "lever_arm"),
        name="lever_arm",
        material=chrome,
    )
    lever.visual(
        mesh_from_cadquery(_grip_solid((0.0, 0.0), (-0.100, -0.024), LEVER_Y), "lever_grip"),
        name="lever_grip",
        material=knurl,
    )
    # punch pin hanging from the upper jaw, aligned with the die bore
    pin_len = PIN_Z1 - PIN_Z0
    lever.visual(
        Cylinder(radius=PIN_R, length=pin_len),
        origin=Origin(xyz=(PIN_X, LEVER_Y, (PIN_Z0 + PIN_Z1) / 2.0 - PIVOT_Z)),
        name="punch_pin",
        material=steel,
    )

    model.articulation(
        "punch_stroke",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=STROKE_MAX),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    lever = object_model.get_part("punch_lever")
    stroke = object_model.get_articulation("punch_stroke")
    pin = lever.get_visual("punch_pin")
    die = frame.get_visual("die_block")
    lever_body = lever.get_visual("lever_arm")
    rivet = frame.get_visual("rivet_shaft")

    # rivet shaft is a captured interference-fit pin inside the lever boss
    # bore: this is the physical pivot connection, intentional by design
    ctx.allow_overlap(
        frame,
        lever,
        elem_a=rivet,
        elem_b=lever_body,
        reason="rivet pivot pin is press-fit through the lever boss bore",
    )
    ctx.expect_contact(frame, lever, elem_a=rivet, elem_b=lever_body, name="rivet_captures_lever")
    ctx.expect_overlap(frame, lever, axes="xz", min_overlap=0.004, elem_a=rivet, elem_b=lever_body)

    # rest pose: punch pin hangs over the die with an open paper slot
    ctx.expect_gap(
        lever,
        frame,
        axis="z",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem=pin,
        negative_elem=die,
        name="paper_slot_open_at_rest",
    )
    pin_rest = ctx.part_element_world_aabb(lever, elem=pin)
    die_ab = ctx.part_element_world_aabb(frame, elem=die)
    assert pin_rest is not None and die_ab is not None
    # pin hangs centered over the die bore in XY
    pin_cx = (pin_rest[0][0] + pin_rest[1][0]) / 2.0
    pin_cy = (pin_rest[0][1] + pin_rest[1][1]) / 2.0
    assert abs(pin_cx - PIN_X) < 1e-4, "punch pin not over die bore (x)"
    assert abs(pin_cy - LEVER_Y) < 1e-4, "punch pin not over die bore (y)"

    grip = lever.get_visual("lever_grip")
    grip_rest = ctx.part_element_world_aabb(lever, elem=grip)
    assert grip_rest is not None

    # squeezed pose: the pin actually plunges below the die top into the bore
    with ctx.pose({stroke: STROKE_MAX}):
        pin_closed = ctx.part_element_world_aabb(lever, elem=pin)
        assert pin_closed is not None
        die_top = die_ab[1][2]
        assert pin_closed[0][2] < die_top - 0.0015, (
            f"punch pin does not enter die: pin_zmin={pin_closed[0][2]:.4f} die_top={die_top:.4f}"
        )
        # pin still centered on the bore while inside it
        cx = (pin_closed[0][0] + pin_closed[1][0]) / 2.0
        assert abs(cx - PIN_X) < 0.0012, "pin drifts off the bore when squeezed"
        # squeezing raises the lower handle grip toward the fixed handle
        grip_closed = ctx.part_element_world_aabb(lever, elem=grip)
        assert grip_closed is not None
        lift = grip_closed[0][2] - grip_rest[0][2]
        assert lift > 0.006, f"lower handle barely moves when squeezed: lift={lift:.4f}"

    return ctx.report()


object_model = build_object_model()
