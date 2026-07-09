from __future__ import annotations

# European four-gang wall switch plate — a horizontal rounded-rectangle
# faceplate (~0.315 x 0.09 m, ~0.012 m proud of the wall) with a thin
# polished-chrome trim ring around a matte ivory plastic plate. It carries
# four wide rocker light switches in a row. Each rocker is an independent
# see-saw REVOLUTE about a horizontal axis (parallel to the plate width)
# through its mid-height, range -0.10..+0.10 rad. A warm-beige wall slab is
# the root so the plate reads as wall-mounted.
#
# Coordinate convention: +Z up, plate width along X, the plate face points
# toward +Y. The wall surface is the plane y = 0; the plate protrudes to +Y.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
# Wall backing slab
WALL_W = 0.48
WALL_T = 0.018
WALL_H = 0.24

# Number of gangs (rocker modules)
GANG_COUNT = 4

# Chrome trim ring (outermost rounded rectangle)
TRIM_W = 0.315
TRIM_H = 0.090
TRIM_R = 0.012  # outer corner radius
TRIM_DEPTH = 0.011  # front of the chrome lip (y, from the wall plane)
TRIM_IN_W = 0.305  # inner opening of the ring
TRIM_IN_H = 0.080
TRIM_IN_R = 0.0095

# Ivory faceplate field (slightly larger than the ring opening so the two
# visuals interlock and read as one assembly; chrome lip sits 1 mm proud)
PLATE_W = 0.307
PLATE_H = 0.082
PLATE_R = 0.0095
PLATE_DEPTH = 0.010  # front face of the ivory field

# Four ~0.055 m rocker modules evenly spaced
MODULE_PITCH = 0.075
ROCKER_X = tuple((i - (GANG_COUNT - 1) / 2.0) * MODULE_PITCH for i in range(GANG_COUNT))

# Rocker pockets cut into the plate (square openings with a thin back floor)
POCKET_SIZE = 0.054
POCKET_R = 0.004
POCKET_FLOOR_Y = 0.0025  # back floor of the pocket (thin ivory wall remains)

# Rocker paddles (slightly convex square, sits proud of the chrome lip)
ROCKER_SIZE = 0.050
ROCKER_BASE_T = 0.0045  # straight skirt thickness
ROCKER_CAP_T = 0.0035  # convex lofted cap thickness
ROCKER_T = ROCKER_BASE_T + ROCKER_CAP_T  # 0.008 total
ROCKER_BACK_Y = 0.0055  # rear face of the paddle (inside the pocket)
ROCKER_PIVOT_Y = ROCKER_BACK_Y + ROCKER_T / 2.0  # see-saw pivot at mid-depth
ROCKER_RANGE = 0.10  # +- rad

# Rocker pivot axle: a thin steel hinge pin along X through the paddle
# mid-height whose tips are captured in the pocket side walls.
AXLE_R = 0.002
AXLE_LEN = POCKET_SIZE + 0.003  # tips embed ~1.5 mm into each pocket wall

# Visuals built in CadQuery's XY plane (extruded along +Z) are rotated into
# the wall frame with roll = -pi/2, which maps cq +Z -> world +Y.
_CQ_TO_WALL = (-math.pi / 2.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Geometry builders (cq frame: x = world x, +z = out of the wall)
# ---------------------------------------------------------------------------
def _rounded_rect(w: float, h: float, r: float) -> cq.Sketch:
    return cq.Sketch().rect(w, h).vertices().fillet(r)


def _trim_ring_mesh() -> object:
    """Thin raised polished-chrome border around the ivory field."""
    ring = (
        cq.Workplane("XY")
        .placeSketch(_rounded_rect(TRIM_W, TRIM_H, TRIM_R))
        .extrude(TRIM_DEPTH)
    )
    opening = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .placeSketch(_rounded_rect(TRIM_IN_W, TRIM_IN_H, TRIM_IN_R))
        .extrude(TRIM_DEPTH + 0.002)
    )
    ring = ring.cut(opening)
    # Soft fillet on the outer front edge so the trim reads polished, not sharp.
    try:
        ring = ring.edges(">Z").fillet(0.0015)
    except Exception:
        pass
    return mesh_from_cadquery(ring, "trim_ring")


def _plate_mesh() -> object:
    """Matte ivory field with four square rocker pockets (thin back floor kept)."""
    plate = (
        cq.Workplane("XY")
        .placeSketch(_rounded_rect(PLATE_W, PLATE_H, PLATE_R))
        .extrude(PLATE_DEPTH)
    )

    # Square rocker pockets: open the front, keep a thin floor at the back.
    for xc in ROCKER_X:
        pocket = (
            cq.Workplane("XY", origin=(xc, 0.0, POCKET_FLOOR_Y))
            .placeSketch(_rounded_rect(POCKET_SIZE, POCKET_SIZE, POCKET_R))
            .extrude(PLATE_DEPTH)
        )
        plate = plate.cut(pocket)

    return mesh_from_cadquery(plate, "plate_field")


def _rocker_mesh() -> object:
    """Slightly convex square ivory paddle, authored in a LOCAL frame centered
    on its see-saw pivot: x = plate width, +z = outward, thickness centered."""
    half = ROCKER_T / 2.0
    base = (
        cq.Workplane("XY", origin=(0.0, 0.0, -half))
        .placeSketch(_rounded_rect(ROCKER_SIZE, ROCKER_SIZE, 0.005))
        .extrude(ROCKER_BASE_T)
    )
    cap = (
        cq.Workplane("XY")
        .placeSketch(
            _rounded_rect(ROCKER_SIZE, ROCKER_SIZE, 0.005).moved(
                cq.Location(cq.Vector(0.0, 0.0, -half + ROCKER_BASE_T))
            ),
            _rounded_rect(ROCKER_SIZE - 0.008, ROCKER_SIZE - 0.008, 0.008).moved(
                cq.Location(cq.Vector(0.0, 0.0, half))
            ),
        )
        .loft()
    )
    paddle = base.union(cap)
    return mesh_from_cadquery(paddle, "rocker_paddle")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_gang_wall_switch_plate")

    wall_beige = model.material("wall_beige", rgba=(0.80, 0.70, 0.60, 1.0))
    ivory = model.material("ivory_plastic", rgba=(0.93, 0.90, 0.83, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.78, 0.79, 0.82, 1.0))
    steel = model.material("clip_steel", rgba=(0.60, 0.62, 0.65, 1.0))

    # Root: small warm-beige wall slab the plate mounts onto.
    wall = model.part("wall_panel")
    wall.visual(
        Box((WALL_W, WALL_T, WALL_H)),
        origin=Origin(xyz=(0.0, -WALL_T / 2.0, 0.0)),
        material=wall_beige,
        name="wall_slab",
    )

    # Faceplate: ivory field + chrome trim ring.
    faceplate = model.part("faceplate")
    faceplate.visual(
        _plate_mesh(),
        origin=Origin(rpy=_CQ_TO_WALL),
        material=ivory,
        name="plate_field",
    )
    faceplate.visual(
        _trim_ring_mesh(),
        origin=Origin(rpy=_CQ_TO_WALL),
        material=chrome,
        name="trim_ring",
    )

    model.articulation(
        "wall_to_faceplate",
        ArticulationType.FIXED,
        parent=wall,
        child=faceplate,
        origin=Origin(),
    )

    # Four independent see-saw rockers about a horizontal X axis through the
    # paddle mid-height/mid-depth. Positive q presses the top edge inward.
    rocker_mesh = _rocker_mesh()
    for idx, xc in enumerate(ROCKER_X):
        rocker = model.part(f"rocker_{idx}")
        rocker.visual(
            rocker_mesh,
            origin=Origin(rpy=_CQ_TO_WALL),
            material=ivory,
            name="rocker_paddle",
        )
        # Steel hinge pin on the see-saw axis; its tips are captured in the
        # pocket side walls so the paddle is physically mounted.
        rocker.visual(
            Cylinder(radius=AXLE_R, length=AXLE_LEN),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="pivot_axle",
        )
        model.articulation(
            f"rocker_{idx}_pivot",
            ArticulationType.REVOLUTE,
            parent=faceplate,
            child=rocker,
            origin=Origin(xyz=(xc, ROCKER_PIVOT_Y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.0,
                lower=-ROCKER_RANGE,
                upper=ROCKER_RANGE,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_panel")
    faceplate = object_model.get_part("faceplate")
    rockers = [object_model.get_part(f"rocker_{i}") for i in range(GANG_COUNT)]
    pivots = [object_model.get_articulation(f"rocker_{i}_pivot") for i in range(GANG_COUNT)]

    # --- Faceplate proportions: horizontal rounded rectangle ~0.315 x 0.09,
    # protruding ~0.012 m from the wall plane (y = 0). ---
    fp_aabb = ctx.part_world_aabb(faceplate)
    ctx.check("faceplate present", fp_aabb is not None, details=f"aabb={fp_aabb}")
    if fp_aabb is not None:
        (mn, mx) = fp_aabb
        ctx.check(
            "faceplate ~0.315 m wide x ~0.09 m tall",
            abs((mx[0] - mn[0]) - TRIM_W) < 0.004 and abs((mx[2] - mn[2]) - TRIM_H) < 0.004,
            details=f"w={mx[0] - mn[0]:.4f}, h={mx[2] - mn[2]:.4f}",
        )
        ctx.check(
            "faceplate protrudes ~0.012 m from the wall",
            -0.0005 < mn[1] < 0.0015 and 0.010 < mx[1] < 0.016,
            details=f"y_min={mn[1]:.4f}, y_max={mx[1]:.4f}",
        )

    # --- Mounted flat against the wall: rear of the plate seats on the wall
    # surface with no gap and no penetration. ---
    ctx.expect_gap(
        faceplate,
        wall,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0001,
        name="faceplate seated flush on the wall",
    )
    ctx.expect_within(
        faceplate,
        wall,
        axes="xz",
        margin=0.0,
        name="faceplate inside the wall slab footprint",
    )

    # --- Thin raised chrome trim ring surrounds the ivory field. ---
    trim_aabb = ctx.part_element_world_aabb(faceplate, elem="trim_ring")
    field_aabb = ctx.part_element_world_aabb(faceplate, elem="plate_field")
    ctx.check(
        "chrome trim and ivory field present",
        trim_aabb is not None and field_aabb is not None,
        details=f"trim={trim_aabb}, field={field_aabb}",
    )
    if trim_aabb is not None and field_aabb is not None:
        ctx.check(
            "chrome trim is the outermost border",
            trim_aabb[1][0] > field_aabb[1][0] + 0.002
            and trim_aabb[0][0] < field_aabb[0][0] - 0.002
            and trim_aabb[1][2] > field_aabb[1][2] + 0.002
            and trim_aabb[0][2] < field_aabb[0][2] - 0.002,
            details=f"trim={trim_aabb}, field={field_aabb}",
        )
        ctx.check(
            "chrome lip proud of the ivory field surface",
            PLATE_DEPTH + 0.0003 < trim_aabb[1][1] < PLATE_DEPTH + 0.0025,
            details=f"trim_front={trim_aabb[1][1]:.4f}, field_face={PLATE_DEPTH}",
        )

    # --- Four gang variant: exactly 4 rockers, evenly spaced. ---
    ctx.check(
        "four-gang variant has exactly 4 rockers",
        len(rockers) == GANG_COUNT and len(pivots) == GANG_COUNT,
        details=f"rockers={len(rockers)}, pivots={len(pivots)}",
    )

    # --- Each rocker module: centered on its pocket, inside the plate, proud
    # of the chrome lip, mounted on a captured axle. ---
    for idx, (rocker, xc) in enumerate(zip(rockers, ROCKER_X)):
        pos = ctx.part_world_position(rocker)
        ctx.check(
            f"rocker_{idx} centered on its module (x={xc:+.3f})",
            pos is not None and abs(pos[0] - xc) < 0.002 and abs(pos[2]) < 0.002,
            details=f"pos={pos}",
        )
        ctx.expect_within(
            rocker,
            faceplate,
            axes="xz",
            margin=0.0,
            name=f"rocker_{idx} stays inside the plate outline",
        )
        r_aabb = ctx.part_world_aabb(rocker)
        if r_aabb is not None and trim_aabb is not None:
            ctx.check(
                f"rocker_{idx} paddle sits proud of the chrome lip",
                r_aabb[1][1] > trim_aabb[1][1] + 0.0010,
                details=f"rocker_front={r_aabb[1][1]:.4f}, trim_front={trim_aabb[1][1]:.4f}",
            )

    # --- Rockers are evenly spaced along X ---
    if len(rockers) >= 2:
        positions_x = []
        for rocker in rockers:
            p = ctx.part_world_position(rocker)
            if p is not None:
                positions_x.append(p[0])
        if len(positions_x) == GANG_COUNT:
            spacings = [positions_x[i + 1] - positions_x[i] for i in range(len(positions_x) - 1)]
            avg_spacing = sum(spacings) / len(spacings)
            ctx.check(
                "rockers evenly spaced along X",
                all(abs(s - avg_spacing) < 0.002 for s in spacings),
                details=f"spacings={[f'{s:.4f}' for s in spacings]}",
            )
            ctx.check(
                "rocker spacing matches MODULE_PITCH",
                abs(avg_spacing - MODULE_PITCH) < 0.002,
                details=f"avg_spacing={avg_spacing:.4f}, expected={MODULE_PITCH}",
            )

    # --- Rocker hinge pins: each paddle is mounted on a steel axle whose tips
    # are intentionally captured in the pocket side walls. ---
    for idx, rocker in enumerate(rockers):
        ctx.allow_overlap(
            rocker,
            faceplate,
            elem_a="pivot_axle",
            elem_b="plate_field",
            reason=(
                "Hinge-pin tips are intentionally captured in the rocker "
                "pocket side walls so the see-saw paddle is physically mounted."
            ),
        )
        ctx.expect_contact(
            rocker,
            faceplate,
            elem_a="pivot_axle",
            elem_b="plate_field",
            name=f"rocker_{idx} axle engages the pocket walls",
        )
        ctx.expect_overlap(
            rocker,
            faceplate,
            axes="x",
            elem_a="pivot_axle",
            elem_b="plate_field",
            min_overlap=0.0005,
            name=f"rocker_{idx} axle tips retained in the pocket walls",
        )

    # --- See-saw articulation: symmetric +-0.10 rad range; pressing tips the
    # paddle (bottom edge swings outward at +q) without hitting the pocket
    # floor or the wall. ---
    for idx, (rocker, pivot) in enumerate(zip(rockers, pivots)):
        limits = pivot.motion_limits
        ctx.check(
            f"rocker_{idx} see-saw range is +-{ROCKER_RANGE:.2f} rad",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower + ROCKER_RANGE) < 1e-6
            and abs(limits.upper - ROCKER_RANGE) < 1e-6,
            details=f"limits=({limits.lower}, {limits.upper})",
        )
        rest_aabb = ctx.part_world_aabb(rocker)
        with ctx.pose({pivot: ROCKER_RANGE}):
            tip_aabb = ctx.part_world_aabb(rocker)
            ctx.check(
                f"rocker_{idx} pressed pose tips the paddle outward",
                rest_aabb is not None
                and tip_aabb is not None
                and tip_aabb[1][1] > rest_aabb[1][1] + 0.0015,
                details=f"rest_front={rest_aabb}, tipped_front={tip_aabb}",
            )
            ctx.check(
                f"rocker_{idx} pressed pose clears the pocket floor",
                tip_aabb is not None and tip_aabb[0][1] > POCKET_FLOOR_Y + 0.0002,
                details=f"tipped_back={None if tip_aabb is None else tip_aabb[0][1]}",
            )
            ctx.expect_gap(
                rocker,
                wall,
                axis="y",
                max_penetration=0.0,
                name=f"rocker_{idx} pressed pose stays clear of the wall",
            )
        with ctx.pose({pivot: -ROCKER_RANGE}):
            low_aabb = ctx.part_world_aabb(rocker)
            ctx.check(
                f"rocker_{idx} opposite press also clears the pocket floor",
                low_aabb is not None and low_aabb[0][1] > POCKET_FLOOR_Y + 0.0002,
                details=f"tipped_back={None if low_aabb is None else low_aabb[0][1]}",
            )

    return ctx.report()


object_model = build_object_model()
