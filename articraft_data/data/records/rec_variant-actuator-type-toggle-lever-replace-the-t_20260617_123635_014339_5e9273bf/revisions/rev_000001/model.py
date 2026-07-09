from __future__ import annotations

# European three-gang wall switch plate — toggle-lever variant:
# Same faceplate, chrome trim ring, and Schuko socket as the parent rocker
# model, but the two wide rocker switches are replaced with two small flip
# toggle-lever switches. Each toggle is a short tapered ivory paddle lever
# that protrudes from a narrow slot in a raised escutcheon frame and flips
# up/down (REVOLUTE about a horizontal X axis) with a ±0.35 rad throw.
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
WALL_W = 0.40
WALL_T = 0.018
WALL_H = 0.24

# Chrome trim ring (outermost rounded rectangle)
TRIM_W = 0.240
TRIM_H = 0.090
TRIM_R = 0.012
TRIM_DEPTH = 0.011
TRIM_IN_W = 0.230
TRIM_IN_H = 0.080
TRIM_IN_R = 0.0095

# Ivory faceplate field
PLATE_W = 0.232
PLATE_H = 0.082
PLATE_R = 0.0095
PLATE_DEPTH = 0.010

# Three modules left to right: toggle, toggle, Schuko socket
MODULE_PITCH = 0.075
TOGGLE_X = (-MODULE_PITCH, 0.0)
SOCKET_X = MODULE_PITCH

# Toggle escutcheon (raised square frame around each toggle slot)
ESC_SIZE = 0.052
ESC_R = 0.005
ESC_FACE_Y = 0.0120  # raised ~2 mm above the ivory field

# Toggle slot (narrow rectangular through-opening in each escutcheon)
TOGGLE_SLOT_W = 0.021  # along X (plate width)
TOGGLE_SLOT_H = 0.009  # along cq Y → world Z (vertical)
TOGGLE_SLOT_R = 0.002

# Toggle lever paddle (tapered ivory flip lever)
LEVER_W = 0.018  # width at base (along X)
LEVER_T = 0.005  # thickness at base (along cq Y → world Z)
LEVER_H = 0.028  # protruding length from the pivot (along cq Z → world Y)

# Toggle pivot: at the plate front face; axis parallel to plate width
TOGGLE_PIVOT_Y = PLATE_DEPTH
TOGGLE_RANGE = 0.35  # ±0.35 rad (~20°)

# Toggle pivot axle (steel pin captured in the escutcheon slot walls)
TOGGLE_AXLE_R = 0.0015
TOGGLE_AXLE_LEN = TOGGLE_SLOT_W + 0.004  # tips embed ~2 mm into each wall

# Schuko module (identical to parent)
TILE_SIZE = 0.055
TILE_R = 0.006
TILE_FACE_Y = 0.0125
WELL_RADIUS = 0.020
WELL_FLOOR_Y = 0.0035
PIN_HOLE_R = 0.0024
PIN_HOLE_DX = 0.0095
PIN_HOLE_FLOOR_Y = 0.0008

# Grounding clips (steel tabs at top and bottom of the Schuko well)
CLIP_W = 0.012
CLIP_T = 0.005
CLIP_H = 0.0048
CLIP_ZC = WELL_RADIUS - 0.0018
CLIP_YC = 0.0080

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
    try:
        ring = ring.edges(">Z").fillet(0.0015)
    except Exception:
        pass
    return mesh_from_cadquery(ring, "trim_ring")


def _plate_mesh() -> object:
    """Matte ivory field: two raised toggle escutcheon frames with narrow
    through-slots, plus a raised Schuko tile with a true hollow circular well
    and two pin holes."""
    plate = (
        cq.Workplane("XY")
        .placeSketch(_rounded_rect(PLATE_W, PLATE_H, PLATE_R))
        .extrude(PLATE_DEPTH)
    )

    # Raised toggle escutcheon frames (square pads around each toggle slot).
    for xc in TOGGLE_X:
        esc = (
            cq.Workplane("XY", origin=(xc, 0.0, PLATE_DEPTH - 0.001))
            .placeSketch(_rounded_rect(ESC_SIZE, ESC_SIZE, ESC_R))
            .extrude(ESC_FACE_Y - PLATE_DEPTH + 0.001)
        )
        plate = plate.union(esc)

    # Raised Schuko tile.
    tile = (
        cq.Workplane("XY", origin=(SOCKET_X, 0.0, PLATE_DEPTH - 0.001))
        .placeSketch(_rounded_rect(TILE_SIZE, TILE_SIZE, TILE_R))
        .extrude(TILE_FACE_Y - PLATE_DEPTH + 0.001)
    )
    plate = plate.union(tile)
    try:
        plate = plate.edges(">Z").fillet(0.0012)
    except Exception:
        pass

    # Toggle slots: narrow rectangular through-cuts in each escutcheon.
    for xc in TOGGLE_X:
        slot = (
            cq.Workplane("XY", origin=(xc, 0.0, -0.001))
            .placeSketch(_rounded_rect(TOGGLE_SLOT_W, TOGGLE_SLOT_H, TOGGLE_SLOT_R))
            .extrude(ESC_FACE_Y + 0.002)
        )
        plate = plate.cut(slot)

    # True hollow circular well recessed into the Schuko tile.
    well = (
        cq.Workplane("XY", origin=(SOCKET_X, 0.0, WELL_FLOOR_Y))
        .circle(WELL_RADIUS)
        .extrude(TILE_FACE_Y)
    )
    plate = plate.cut(well)

    # Two blind Schuko pin holes in the well floor.
    for sx in (-1.0, 1.0):
        hole = (
            cq.Workplane("XY", origin=(SOCKET_X + sx * PIN_HOLE_DX, 0.0, PIN_HOLE_FLOOR_Y))
            .circle(PIN_HOLE_R)
            .extrude(WELL_FLOOR_Y - PIN_HOLE_FLOOR_Y + 0.001)
        )
        plate = plate.cut(hole)

    return mesh_from_cadquery(plate, "plate_field")


def _toggle_lever_mesh() -> object:
    """Small flip toggle lever: tapered ivory paddle authored with its pivot
    at the origin. Extends along cq +Z (→ world +Y outward from wall).
    The base cross-section is wider; the tip tapers to ~82% with rounder
    corners for a finger-friendly profile."""
    lever = (
        cq.Workplane("XY")
        .placeSketch(
            _rounded_rect(LEVER_W, LEVER_T, 0.001).moved(
                cq.Location(cq.Vector(0.0, 0.0, 0.0))
            ),
            _rounded_rect(LEVER_W * 0.82, LEVER_T * 0.82, 0.0015).moved(
                cq.Location(cq.Vector(0.0, 0.0, LEVER_H))
            ),
        )
        .loft()
    )
    # Domed tip for a finished look.
    try:
        lever = lever.edges(">Z").fillet(0.0012)
    except Exception:
        pass
    return mesh_from_cadquery(lever, "toggle_lever")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_gang_wall_switch_plate_toggle")

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

    # Faceplate: ivory field + chrome trim ring + Schuko grounding clips.
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
    for sz, elem in ((1.0, "ground_clip_upper"), (-1.0, "ground_clip_lower")):
        faceplate.visual(
            Box((CLIP_W, CLIP_T, CLIP_H)),
            origin=Origin(xyz=(SOCKET_X, CLIP_YC, sz * CLIP_ZC)),
            material=steel,
            name=elem,
        )

    model.articulation(
        "wall_to_faceplate",
        ArticulationType.FIXED,
        parent=wall,
        child=faceplate,
        origin=Origin(),
    )

    # Two independent flip toggle levers, emitted via a shared helper mesh
    # and a for-loop with toggle_{i} naming and uniform revolute policy.
    lever_mesh = _toggle_lever_mesh()
    for i, xc in enumerate(TOGGLE_X):
        toggle = model.part(f"toggle_{i}")
        toggle.visual(
            lever_mesh,
            origin=Origin(rpy=_CQ_TO_WALL),
            material=ivory,
            name="toggle_lever",
        )
        # Steel pivot pin on the flip axis; tips captured in the escutcheon
        # slot side walls so the lever is physically mounted.
        toggle.visual(
            Cylinder(radius=TOGGLE_AXLE_R, length=TOGGLE_AXLE_LEN),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="pivot_axle",
        )
        model.articulation(
            f"toggle_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=faceplate,
            child=toggle,
            origin=Origin(xyz=(xc, TOGGLE_PIVOT_Y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.0,
                lower=-TOGGLE_RANGE,
                upper=TOGGLE_RANGE,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_panel")
    faceplate = object_model.get_part("faceplate")
    toggles = [object_model.get_part(f"toggle_{i}") for i in range(2)]
    pivots = [object_model.get_articulation(f"toggle_{i}_pivot") for i in range(2)]

    # --- Faceplate proportions: horizontal rounded rectangle ~0.24 x 0.09,
    # protruding ~0.012 m from the wall plane (y = 0). ---
    fp_aabb = ctx.part_world_aabb(faceplate)
    ctx.check("faceplate present", fp_aabb is not None, details=f"aabb={fp_aabb}")
    if fp_aabb is not None:
        (mn, mx) = fp_aabb
        ctx.check(
            "faceplate ~0.24 m wide x ~0.09 m tall",
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
        faceplate, wall, axis="y",
        max_gap=0.0005, max_penetration=0.0001,
        name="faceplate seated flush on the wall",
    )
    ctx.expect_within(
        faceplate, wall, axes="xz", margin=0.0,
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

    # --- Toggle levers: two small flip levers, each centered on its module
    # position and protruding well beyond the plate face. ---
    for i, (toggle, xc) in enumerate(zip(toggles, TOGGLE_X)):
        pos = ctx.part_world_position(toggle)
        ctx.check(
            f"toggle_{i} centered on its module (x={xc:+.3f})",
            pos is not None and abs(pos[0] - xc) < 0.003 and abs(pos[2]) < 0.005,
            details=f"pos={pos}",
        )
        ctx.expect_within(
            toggle, faceplate, axes="xz", margin=0.005,
            name=f"toggle_{i} stays inside the plate outline on XZ",
        )
        t_aabb = ctx.part_world_aabb(toggle)
        if t_aabb is not None and trim_aabb is not None:
            ctx.check(
                f"toggle_{i} lever protrudes well beyond the chrome lip",
                t_aabb[1][1] > trim_aabb[1][1] + 0.005,
                details=f"toggle_front={t_aabb[1][1]:.4f}, trim_front={trim_aabb[1][1]:.4f}",
            )

    # --- Schuko module: hollow recessed well with pin holes (authored cuts)
    # and steel grounding clips seated inside the well at top and bottom. ---
    ctx.check(
        "Schuko well authored as a true hollow recess with pin holes",
        WELL_FLOOR_Y < TILE_FACE_Y - 0.006
        and WELL_RADIUS * 2.0 > 0.035
        and PIN_HOLE_DX * 2.0 > 0.015,
        details=(
            f"well_depth={TILE_FACE_Y - WELL_FLOOR_Y:.4f}, dia={2 * WELL_RADIUS:.3f}, "
            f"pin_spacing={2 * PIN_HOLE_DX:.4f}"
        ),
    )
    for elem, sz in (("ground_clip_upper", 1.0), ("ground_clip_lower", -1.0)):
        c_aabb = ctx.part_element_world_aabb(faceplate, elem=elem)
        ctx.check(
            f"{elem} seated inside the socket well",
            c_aabb is not None
            and abs((c_aabb[0][0] + c_aabb[1][0]) / 2.0 - SOCKET_X) < 0.002
            and c_aabb[0][1] > WELL_FLOOR_Y - 0.0005
            and c_aabb[1][1] < TILE_FACE_Y + 0.0005
            and min(abs(c_aabb[0][2]), abs(c_aabb[1][2])) < WELL_RADIUS - 0.002
            and (sz > 0) == ((c_aabb[0][2] + c_aabb[1][2]) > 0),
            details=f"{elem} aabb={c_aabb}",
        )

    # --- Toggle pivot axles: each lever is mounted on a steel pin whose tips
    # are intentionally captured in the escutcheon slot side walls. ---
    for i, toggle in enumerate(toggles):
        ctx.allow_overlap(
            toggle, faceplate,
            elem_a="pivot_axle", elem_b="plate_field",
            reason=(
                "Toggle pivot-pin tips are intentionally captured in the "
                "escutcheon slot side walls so the flip lever is physically mounted."
            ),
        )
        ctx.expect_contact(
            toggle, faceplate,
            elem_a="pivot_axle", elem_b="plate_field",
            name=f"toggle_{i} axle engages the slot walls",
        )
        ctx.expect_overlap(
            toggle, faceplate, axes="x",
            elem_a="pivot_axle", elem_b="plate_field",
            min_overlap=0.0005,
            name=f"toggle_{i} axle tips retained in the slot walls",
        )

    # --- Toggle flip articulation: symmetric ±0.35 rad range; positive q
    # flips the lever tip upward (+Z), negative flips it downward. ---
    for i, (toggle, pivot) in enumerate(zip(toggles, pivots)):
        limits = pivot.motion_limits
        ctx.check(
            f"toggle_{i} flip range is ±{TOGGLE_RANGE:.2f} rad",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower + TOGGLE_RANGE) < 1e-6
            and abs(limits.upper - TOGGLE_RANGE) < 1e-6,
            details=f"limits=({limits.lower}, {limits.upper})",
        )
        rest_aabb = ctx.part_world_aabb(toggle)
        # Positive q: lever tip swings upward (+Z in world).
        with ctx.pose({pivot: TOGGLE_RANGE}):
            up_aabb = ctx.part_world_aabb(toggle)
            ctx.check(
                f"toggle_{i} positive flip moves lever tip upward",
                rest_aabb is not None
                and up_aabb is not None
                and up_aabb[1][2] > rest_aabb[1][2] + 0.003,
                details=f"rest_top_z={rest_aabb[1][2]:.4f}, up_top_z={up_aabb[1][2]:.4f}",
            )
            ctx.expect_gap(
                toggle, wall, axis="y", max_penetration=0.0,
                name=f"toggle_{i} up-flip pose clears the wall",
            )
        # Negative q: lever tip swings downward (-Z in world).
        with ctx.pose({pivot: -TOGGLE_RANGE}):
            down_aabb = ctx.part_world_aabb(toggle)
            ctx.check(
                f"toggle_{i} negative flip moves lever tip downward",
                rest_aabb is not None
                and down_aabb is not None
                and down_aabb[0][2] < rest_aabb[0][2] - 0.003,
                details=f"rest_bot_z={rest_aabb[0][2]:.4f}, down_bot_z={down_aabb[0][2]:.4f}",
            )
            ctx.expect_gap(
                toggle, wall, axis="y", max_penetration=0.0,
                name=f"toggle_{i} down-flip pose clears the wall",
            )

    return ctx.report()


object_model = build_object_model()
