from __future__ import annotations

# Multi-disc (4-disc) fat CD jewel case.
#
# Coordinate convention:
#   - Case lies flat in the XY plane; "up" is +Z.
#   - X is case width (~0.142 m), Y is case depth (~0.125 m).
#   - Rear hinge runs left-right along the +Y edge.
#   - z=0 is the underside of the base frame; the assembly stacks upward in +Z.
#
# Structure (for-loop over NUM_DISCS):
#   - base (root): tall clear rectangular frame (open top).
#   - leaf_0 .. leaf_3: dark tray leaves, each hinged at the inner rear wall.
#     Each leaf carries a raised hub, rosette teeth, and front finger notches.
#     REVOLUTE page-flip about axis=(-1,0,0) so positive q lifts the front edge.
#   - disc_0 .. disc_3: silver CDs, each on a CONTINUOUS spin joint about the
#     leaf hub axis (+Z in leaf frame). Seated on the hub rosette.
#   - lid: clear hinged cover capping the top. REVOLUTE about the rear top edge.

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
    mesh_from_cadquery,
)

NUM_DISCS = 4

# ---- key dimensions (meters) -------------------------------------------------
CASE_W = 0.142          # X extent
CASE_D = 0.125          # Y extent
BASE_FLOOR_T = 0.004    # base bottom plate thickness
WALL = 0.003            # frame wall thickness
WALL_H = 0.034          # wall height (top of walls above z=0)
LID_T = 0.005           # lid cap total thickness
LID_WALL = 0.0016       # lid shell wall thickness

LEAF_T = 0.0015         # leaf floor plate thickness
LEAF_SPACING = 0.007    # vertical spacing between leaf hinge lines
LEAF_FIRST_Z = 0.005    # first leaf hinge height above z=0

# Leaf plate slightly smaller than the case interior so it nests inside.
LEAF_W = CASE_W - 2 * WALL - 0.002
LEAF_D = CASE_D - 2 * WALL - 0.002

DISC_R = 0.060          # CD outer radius (120 mm disc)
DISC_T = 0.0012         # CD thickness
DISC_HOLE_R = 0.0075    # CD center hole radius

HUB_R = 0.009           # raised tray hub radius
HUB_H = 0.004           # hub height above leaf floor

# Hinge Y positions
LEAF_HINGE_Y = CASE_D / 2.0 - WALL   # inner face of rear wall
LID_HINGE_Y = CASE_D / 2.0            # outer rear edge (top corner)

# Hinge barrel detail on each leaf
BARREL_R = 0.0018       # hinge barrel radius
BARREL_W = 0.028        # hinge barrel length along X


def _leaf_hinge_z(i: int) -> float:
    """Return the Z height of the i-th leaf hinge line."""
    return LEAF_FIRST_Z + i * LEAF_SPACING


# ---- geometry helpers --------------------------------------------------------

def _base_frame() -> cq.Workplane:
    """Tall clear base frame: open-topped rectangular shell."""
    outer = cq.Workplane("XY").box(CASE_W, CASE_D, WALL_H, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .workplane(offset=BASE_FLOOR_T)
        .box(CASE_W - 2 * WALL, CASE_D - 2 * WALL, WALL_H, centered=(True, True, False))
    )
    return outer.cut(inner)


def _leaf_plate() -> cq.Workplane:
    """Tray leaf in its local frame.

    Local origin at the rear hinge edge (0, 0, 0). The plate extends toward -Y
    (front of case). Floor at z=[0, LEAF_T]. Hub, rosette teeth, and front
    finger notches on top.
    """
    # Floor plate: centered in X, from y=0 to y=-LEAF_D.
    plate = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_D, LEAF_T, centered=(True, True, False))
        .translate((0.0, -LEAF_D / 2.0, 0.0))
    )

    # Raised hub boss at plate center.
    hub_cy = -LEAF_D / 2.0
    hub = (
        cq.Workplane("XY")
        .workplane(offset=LEAF_T)
        .center(0.0, hub_cy)
        .circle(HUB_R)
        .extrude(HUB_H)
    )
    plate = plate.union(hub)

    # Rosette teeth: small radial cylinders around the hub.
    n_teeth = 8
    for j in range(n_teeth):
        a = 2.0 * math.pi * j / n_teeth
        tx = (HUB_R + 0.0015) * math.cos(a)
        ty = hub_cy + (HUB_R + 0.0015) * math.sin(a)
        tooth = (
            cq.Workplane("XY")
            .workplane(offset=LEAF_T)
            .center(tx, ty)
            .circle(0.0016)
            .extrude(HUB_H * 0.7)
        )
        plate = plate.union(tooth)

    # Front finger-notch cutouts: two half-circle scoops at the -Y edge.
    for nx in (-0.026, 0.026):
        notch = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(nx, -LEAF_D)
            .circle(0.013)
            .extrude(LEAF_T + 0.003)
        )
        plate = plate.cut(notch)

    # Hinge barrel at the rear edge (pivot cylinder along X).
    barrel = cq.Solid.makeCylinder(
        BARREL_R, BARREL_W,
        pnt=cq.Vector(-BARREL_W / 2.0, 0.0, LEAF_T / 2.0),
        dir=cq.Vector(1.0, 0.0, 0.0),
    )
    plate = plate.union(cq.Workplane("XY").add(barrel))

    return plate


def _lid_shell() -> cq.Workplane:
    """Clear lid cap in its local frame.

    Local origin at the hinge point (outer rear top corner). The cap extends
    toward -Y (covering the case) and upward in +Z (above the hinge).
    It is a shallow shell open on the bottom with a thin top plate and walls.
    """
    w = CASE_W
    d = CASE_D
    # Outer solid plate: from y=0 to y=-d, z=[0, LID_T].
    outer = (
        cq.Workplane("XY")
        .box(w, d, LID_T, centered=(True, True, False))
        .translate((0.0, -d / 2.0, 0.0))
    )
    # Inner hollow: smaller box from z=0 to z=(LID_T - LID_WALL), same XY range.
    inner = (
        cq.Workplane("XY")
        .box(w - 2 * LID_WALL, d - 2 * LID_WALL, LID_T - LID_WALL,
             centered=(True, True, False))
        .translate((0.0, -d / 2.0, 0.0))
    )
    return outer.cut(inner)


def _disc_solid() -> cq.Workplane:
    """Silver CD: thin cylinder with a central hole."""
    return (
        cq.Workplane("XY")
        .circle(DISC_R)
        .extrude(DISC_T)
        .faces(">Z")
        .workplane()
        .hole(DISC_HOLE_R * 2.0)
    )


# ---- build -------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cd_jewel_case_multi")

    clear = model.material("clear_plastic", rgba=(0.55, 0.60, 0.68, 0.30))
    silver = model.material("disc_silver", rgba=(0.80, 0.82, 0.85, 1.0))

    # Slightly varied dark tints per leaf for visual variety.
    leaf_tints = [
        (0.10, 0.10, 0.12, 1.0),
        (0.12, 0.08, 0.11, 1.0),
        (0.08, 0.11, 0.14, 1.0),
        (0.11, 0.12, 0.09, 1.0),
    ]

    # ---- base (root): tall clear frame ---------------------------------------
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_frame(), "base_frame"),
        material=clear,
        name="base_frame",
    )
    base.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, WALL_H)),
        mass=0.090,
        origin=Origin(xyz=(0.0, 0.0, WALL_H / 2.0)),
    )

    # ---- lid: clear hinged cap, revolute about the rear top edge -------------
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell"),
        material=clear,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, LID_T)),
        mass=0.040,
        origin=Origin(xyz=(0.0, -CASE_D / 2.0, LID_T / 2.0)),
    )
    # Lid hinge at the outer rear top corner. Body extends -Y (forward).
    # axis=(-1,0,0): positive q lifts the front edge up.
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, WALL_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0,
            lower=0.0, upper=math.radians(70.0),
        ),
    )

    # ---- shared meshes for loop-built leaves and discs -----------------------
    leaf_mesh = mesh_from_cadquery(_leaf_plate(), "leaf_plate")
    disc_mesh = mesh_from_cadquery(_disc_solid(), "disc_body")

    # Hub center Y in the leaf local frame.
    hub_cy = -LEAF_D / 2.0
    # Disc Z in the leaf local frame: sits on the rosette teeth top.
    disc_z_in_leaf = LEAF_T + HUB_H * 0.7

    for i in range(NUM_DISCS):
        leaf_z = _leaf_hinge_z(i)
        leaf_mat = model.material(f"leaf_dark_{i}", rgba=leaf_tints[i])

        # ---- leaf_i: hinged tray leaf ----------------------------------------
        leaf = model.part(f"leaf_{i}")
        leaf.visual(leaf_mesh, material=leaf_mat, name=f"leaf_plate_{i}")

        # Page-flip hinge at the inner rear wall face, at the leaf stacking Z.
        # axis=(-1,0,0): positive q lifts the front edge upward.
        model.articulation(
            f"base_to_leaf_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leaf,
            origin=Origin(xyz=(0.0, LEAF_HINGE_Y, leaf_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=3.0,
                lower=0.0, upper=math.radians(120.0),
            ),
        )

        # ---- disc_i: silver CD spinning on the leaf hub ----------------------
        disc = model.part(f"disc_{i}")
        disc.visual(disc_mesh, material=silver, name=f"disc_body_{i}")
        # Off-center marker so spin is visually/numerically detectable.
        disc.visual(
            Cylinder(radius=0.004, length=DISC_T),
            origin=Origin(xyz=(0.030, 0.0, DISC_T / 2.0)),
            material=leaf_mat,
            name=f"disc_marker_{i}",
        )
        disc.inertial = Inertial.from_geometry(
            Cylinder(radius=DISC_R, length=DISC_T),
            mass=0.016,
            origin=Origin(xyz=(0.0, 0.0, DISC_T / 2.0)),
        )

        # Continuous spin about the leaf-local +Z (perpendicular to leaf face).
        model.articulation(
            f"leaf_{i}_to_disc_{i}",
            ArticulationType.CONTINUOUS,
            parent=leaf,
            child=disc,
            origin=Origin(xyz=(0.0, hub_cy, disc_z_in_leaf)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.2, velocity=8.0),
        )

    return model


# ---- tests -------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_joint = object_model.get_articulation("base_to_lid")

    # --- clear materials (alpha < 1) ---
    lid_alpha = lid.get_visual("lid_shell").material.rgba[3]
    ctx.check("lid is transparent", lid_alpha < 0.6, details=f"alpha={lid_alpha}")
    base_alpha = base.get_visual("base_frame").material.rgba[3]
    ctx.check("base frame is clear plastic", base_alpha < 0.6, details=f"alpha={base_alpha}")

    # --- 4 leaves and 4 discs exist ---
    leaves = [object_model.get_part(f"leaf_{i}") for i in range(NUM_DISCS)]
    discs = [object_model.get_part(f"disc_{i}") for i in range(NUM_DISCS)]
    leaf_joints = [object_model.get_articulation(f"base_to_leaf_{i}") for i in range(NUM_DISCS)]
    disc_joints = [object_model.get_articulation(f"leaf_{i}_to_disc_{i}") for i in range(NUM_DISCS)]

    ctx.check("4 leaves present", len(leaves) == NUM_DISCS)
    ctx.check("4 discs present", len(discs) == NUM_DISCS)

    # --- lid opens upward (front edge lifts) ---
    closed_lid_top = ctx.part_world_aabb(lid)[1][2]
    closed_lid_front_y = ctx.part_world_aabb(lid)[0][1]
    with ctx.pose({lid_joint: math.radians(60.0)}):
        open_lid = ctx.part_world_aabb(lid)
        open_lid_top = open_lid[1][2]
        open_lid_front_y = open_lid[0][1]
    ctx.check(
        "lid swings open and lifts upward",
        open_lid_top > closed_lid_top + 0.02,
        details=f"closed_top={closed_lid_top:.4f}, open_top={open_lid_top:.4f}",
    )
    # Front edge Y moves rearward as the lid opens (swings about the rear hinge).
    ctx.check(
        "lid front edge swings rearward as it opens",
        open_lid_front_y > closed_lid_front_y + 0.02,
        details=f"closed_front_y={closed_lid_front_y:.4f}, open_front_y={open_lid_front_y:.4f}",
    )

    # --- lid hinge axis runs left-right (X) ---
    ctx.check(
        "lid hinge axis runs along X",
        abs(lid_joint.axis[0]) > 0.9,
        details=f"axis={lid_joint.axis}",
    )

    # --- each leaf flips up, each disc spins, all within footprint ---
    for i in range(NUM_DISCS):
        leaf = leaves[i]
        disc = discs[i]
        lj = leaf_joints[i]
        dj = disc_joints[i]

        # Leaf page-flip lifts the front edge upward.
        closed_top = ctx.part_world_aabb(leaf)[1][2]
        with ctx.pose({lj: math.radians(90.0)}):
            open_top = ctx.part_world_aabb(leaf)[1][2]
        ctx.check(
            f"leaf_{i} flips upward",
            open_top > closed_top + 0.02,
            details=f"closed_top={closed_top:.4f}, open_top={open_top:.4f}",
        )

        # Leaf hinge axis runs left-right (X direction).
        ctx.check(
            f"leaf_{i} hinge axis along X",
            abs(lj.axis[0]) > 0.9,
            details=f"axis={lj.axis}",
        )

        # Disc spin moves the off-center marker around the hub axis.
        marker_rest = ctx.part_element_world_aabb(disc, elem=f"disc_marker_{i}")
        rest_cx = 0.5 * (marker_rest[0][0] + marker_rest[1][0])
        rest_cy = 0.5 * (marker_rest[0][1] + marker_rest[1][1])
        with ctx.pose({dj: math.radians(90.0)}):
            marker_q = ctx.part_element_world_aabb(disc, elem=f"disc_marker_{i}")
            q_cx = 0.5 * (marker_q[0][0] + marker_q[1][0])
            q_cy = 0.5 * (marker_q[0][1] + marker_q[1][1])
        ctx.check(
            f"disc_{i} spin moves marker around hub",
            abs(q_cx - rest_cx) > 0.015 and abs(q_cy - rest_cy) > 0.015,
            details=f"rest=({rest_cx:.4f},{rest_cy:.4f}) q90=({q_cx:.4f},{q_cy:.4f})",
        )

        # Disc spin axis is vertical (leaf-local Z = world Z when leaf is flat).
        ctx.check(
            f"disc_{i} spin axis is Z",
            abs(dj.axis[2]) > 0.9,
            details=f"axis={dj.axis}",
        )

        # Leaf fits within the base footprint.
        ctx.expect_within(
            leaf, base, axes="xy",
            inner_elem=f"leaf_plate_{i}", outer_elem="base_frame",
            margin=0.005,
            name=f"leaf_{i} within case footprint",
        )

        # Disc sits within the case footprint.
        ctx.expect_within(
            disc, base, axes="xy",
            inner_elem=f"disc_body_{i}", outer_elem="base_frame",
            margin=0.005,
            name=f"disc_{i} within case footprint",
        )

        # Hub-through-disc intentional overlap (scoped).
        ctx.allow_overlap(
            disc, leaf,
            elem_a=f"disc_body_{i}", elem_b=f"leaf_plate_{i}",
            reason=(
                f"The CD center hole drops over leaf_{i}'s raised hub; the hub "
                "intentionally pokes through the disc bore so the disc is captured."
            ),
        )

        # Hinge barrel intentionally embedded in the rear wall (scoped).
        ctx.allow_overlap(
            leaf, base,
            elem_a=f"leaf_plate_{i}", elem_b="base_frame",
            reason=(
                f"leaf_{i}'s hinge barrel wraps around the pivot at the inner "
                "rear wall face; a small local embed represents the hinge capture."
            ),
        )

        # Disc rests near its leaf hub height (not floating high).
        disc_bottom = ctx.part_world_aabb(disc)[0][2]
        expected_z = _leaf_hinge_z(i) + LEAF_T + HUB_H * 0.7
        ctx.check(
            f"disc_{i} rests on leaf_{i} hub",
            abs(disc_bottom - expected_z) < 0.002,
            details=f"disc_bottom={disc_bottom:.4f}, expected={expected_z:.4f}",
        )

    # --- leaves are stacked (each hinge Z is above the previous) ---
    for i in range(1, NUM_DISCS):
        prev_z = ctx.part_world_position(leaves[i - 1])[2]
        curr_z = ctx.part_world_position(leaves[i])[2]
        ctx.check(
            f"leaf_{i} hinge is above leaf_{i-1}",
            curr_z > prev_z + 0.003,
            details=f"leaf_{i-1}_z={prev_z:.4f}, leaf_{i}_z={curr_z:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
