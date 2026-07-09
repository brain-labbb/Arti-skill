from __future__ import annotations

# European three-gang wall switch plate — ROTARY DIMMER VARIANT:
# A horizontal rounded-rectangle faceplate (~0.24 x 0.09 m, ~0.012 m proud of
# the wall) with a thin polished-chrome trim ring around a matte ivory plastic
# plate. Left to right it carries two round rotary dimmer knobs and one
# round-recessed Schuko socket. Each dimmer is an independent CONTINUOUS joint
# about the front-normal +Y axis, spinning freely. The Schuko module is static:
# a raised square ivory tile with a true hollow circular well (~0.04 m dia)
# containing two pin holes and top/bottom steel grounding clips. A warm-beige
# wall slab is the root so the plate reads as wall-mounted.
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
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
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
TRIM_R = 0.012  # outer corner radius
TRIM_DEPTH = 0.011  # front of the chrome lip (y, from the wall plane)
TRIM_IN_W = 0.230  # inner opening of the ring
TRIM_IN_H = 0.080
TRIM_IN_R = 0.0095

# Ivory faceplate field (slightly larger than the ring opening so the two
# visuals interlock and read as one assembly; chrome lip sits 1 mm proud)
PLATE_W = 0.232
PLATE_H = 0.082
PLATE_R = 0.0095
PLATE_DEPTH = 0.010  # front face of the ivory field

# Three ~0.055 m modules, left to right: dimmer, dimmer, Schuko socket
MODULE_PITCH = 0.075
DIMMER_X = (-MODULE_PITCH, 0.0)
SOCKET_X = MODULE_PITCH

# Circular shaft holes in the plate for the dimmer modules
SHAFT_HOLE_R = 0.012  # 24 mm diameter opening for the dimmer mechanism

# Rotary dimmer knobs: round skirted knobs that protrude from the plate face
DIMMER_DIAMETER = 0.044  # ~44 mm knob diameter
DIMMER_HEIGHT = 0.018  # ~18 mm total height
DIMMER_SKIRT_D = 0.050  # skirt slightly wider than the knob body
DIMMER_SKIRT_H = 0.004  # thin skirt ring at the base
DIMMER_BASE_Y = PLATE_DEPTH  # knob base seated on the plate face

# Schuko module: raised square tile with a hollow circular well
TILE_SIZE = 0.055
TILE_R = 0.006
TILE_FACE_Y = 0.0125  # raised ~2.5 mm above the ivory field
WELL_RADIUS = 0.020  # ~0.04 m diameter recessed well
WELL_FLOOR_Y = 0.0035  # true hollow recess, ~9 mm deep
PIN_HOLE_R = 0.0024  # Schuko pin holes, ~4.8 mm dia
PIN_HOLE_DX = 0.0095  # +-9.5 mm => 19 mm pin spacing
PIN_HOLE_FLOOR_Y = 0.0008  # blind hole bottom

# Grounding clips (steel tabs at the top and bottom of the well)
CLIP_W = 0.012  # along X
CLIP_T = 0.005  # along Y (depth inside the well)
CLIP_H = 0.0048  # radial extent (along Z)
CLIP_ZC = WELL_RADIUS - 0.0018  # embeds into the well wall, protrudes inward
CLIP_YC = 0.0080  # sits between the well floor and the mouth

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
    """Matte ivory field: two circular dimmer shaft holes and a raised Schuko
    tile with a true hollow circular well plus two pin holes."""
    plate = (
        cq.Workplane("XY")
        .placeSketch(_rounded_rect(PLATE_W, PLATE_H, PLATE_R))
        .extrude(PLATE_DEPTH)
    )

    # Raised square Schuko tile.
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

    # Circular dimmer shaft holes through the plate.
    for xc in DIMMER_X:
        hole = (
            cq.Workplane("XY", origin=(xc, 0.0, -0.001))
            .circle(SHAFT_HOLE_R)
            .extrude(PLATE_DEPTH + 0.002)
        )
        plate = plate.cut(hole)

    # True hollow circular well recessed into the tile.
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


def _dimmer_knob_mesh() -> object:
    """Round rotary dimmer knob: tapered body with ribbed grip and a raised
    pointer indicator. Built in local Z (knob axis), center=False so the
    mounting face sits at z=0 and the knob extends toward +Z."""
    knob = KnobGeometry(
        DIMMER_DIAMETER,
        DIMMER_HEIGHT,
        body_style="tapered",
        top_diameter=DIMMER_DIAMETER - 0.008,
        edge_radius=0.001,
        grip=KnobGrip(style="ribbed", count=12, depth=0.0008, width=0.002),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0006),
        center=False,
    )
    return mesh_from_geometry(knob, "dimmer_knob")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_gang_wall_switch_plate_dimmer")

    wall_beige = model.material("wall_beige", rgba=(0.80, 0.70, 0.60, 1.0))
    ivory = model.material("ivory_plastic", rgba=(0.93, 0.90, 0.83, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.78, 0.79, 0.82, 1.0))
    steel = model.material("clip_steel", rgba=(0.60, 0.62, 0.65, 1.0))
    knob_ivory = model.material("knob_ivory", rgba=(0.91, 0.88, 0.80, 1.0))

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

    # Two rotary dimmer knobs: round skirted knobs that spin CONTINUOUS about
    # the front-normal +Y axis. Each knob is mounted at the plate face with a
    # small shaft stub penetrating the circular hole so it reads as physically
    # attached to the plate.
    knob_mesh = _dimmer_knob_mesh()
    n_dimmers = len(DIMMER_X)
    for i in range(n_dimmers):
        xc = DIMMER_X[i]
        dimmer = model.part(f"dimmer_{i}")

        # Knob body: KnobGeometry is built in local Z (center=False, base at
        # z=0, extends +Z). Rotate to wall frame so +Z -> +Y (outward).
        dimmer.visual(
            knob_mesh,
            origin=Origin(rpy=_CQ_TO_WALL),
            material=knob_ivory,
            name="knob_body",
        )

        # Shaft stub: a short steel cylinder that penetrates the knob base and
        # the plate hole, physically connecting the knob to the faceplate.
        # Cylinder is along local Z; roll=+pi/2 maps Z to -Y so the shaft
        # extends from inside the knob down through the plate.
        shaft_len = PLATE_DEPTH + 0.004  # 0.014 m total
        shaft_offset_y = -0.005  # centers shaft from y=-0.012 to y=+0.002 in part frame
        dimmer.visual(
            Cylinder(radius=0.004, length=shaft_len),
            origin=Origin(xyz=(0.0, shaft_offset_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="shaft_stub",
        )

        # Joint origin at the plate surface where the knob mounts; axis = +Y
        # (front-normal, pointing outward from the wall).
        model.articulation(
            f"dimmer_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=faceplate,
            child=dimmer,
            origin=Origin(xyz=(xc, DIMMER_BASE_Y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_panel")
    faceplate = object_model.get_part("faceplate")
    n_dimmers = len(DIMMER_X)
    dimmers = [object_model.get_part(f"dimmer_{i}") for i in range(n_dimmers)]
    spins = [object_model.get_articulation(f"dimmer_{i}_spin") for i in range(n_dimmers)]

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

    # --- Dimmer knobs: two round rotary knobs on the left modules, evenly
    # spaced, protruding from the plate face. ---
    for i, (dimmer, xc) in enumerate(zip(dimmers, DIMMER_X)):
        pos = ctx.part_world_position(dimmer)
        ctx.check(
            f"dimmer_{i} centered on its module (x={xc:+.3f})",
            pos is not None and abs(pos[0] - xc) < 0.003 and abs(pos[2]) < 0.003,
            details=f"pos={pos}",
        )
        ctx.expect_within(
            dimmer,
            faceplate,
            axes="xz",
            margin=0.005,
            name=f"dimmer_{i} stays inside the plate outline",
        )
        d_aabb = ctx.part_world_aabb(dimmer)
        if d_aabb is not None and trim_aabb is not None:
            ctx.check(
                f"dimmer_{i} knob protrudes beyond the chrome lip",
                d_aabb[1][1] > trim_aabb[1][1] + 0.002,
                details=f"knob_front={d_aabb[1][1]:.4f}, trim_front={trim_aabb[1][1]:.4f}",
            )

    # --- Dimmer articulations: CONTINUOUS about +Y (front-normal). ---
    for i, spin in enumerate(spins):
        ctx.check(
            f"dimmer_{i}_spin is CONTINUOUS",
            spin.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={spin.articulation_type}",
        )
        # Axis should be +Y (front-normal, out from wall).
        axis = spin.axis
        ctx.check(
            f"dimmer_{i}_spin axis is front-normal (+Y)",
            axis is not None
            and abs(axis[0]) < 0.01
            and abs(axis[1] - 1.0) < 0.01
            and abs(axis[2]) < 0.01,
            details=f"axis={axis}",
        )

    # --- Knob bases seat on the plate face around the circular holes. The
    # shaft stubs pass through the holes (inside the void) to connect the knob
    # to the plate internally. ---
    for i, dimmer in enumerate(dimmers):
        ctx.allow_overlap(
            dimmer,
            faceplate,
            elem_a="shaft_stub",
            elem_b="plate_field",
            reason=(
                "The dimmer shaft stub passes through the circular hole in the "
                "plate to physically mount the knob on the faceplate."
            ),
        )
        ctx.expect_contact(
            dimmer,
            faceplate,
            elem_a="knob_body",
            elem_b="plate_field",
            name=f"dimmer_{i} knob base seats on the plate face",
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

    # --- Rotary dimmer spin check: a rotated pose should move the indicator
    # dot without displacing the knob center (pure rotation about Y). ---
    for i, (dimmer, spin) in enumerate(zip(dimmers, spins)):
        rest_pos = ctx.part_world_position(dimmer)
        with ctx.pose({spin: math.pi / 2.0}):
            spun_pos = ctx.part_world_position(dimmer)
            ctx.check(
                f"dimmer_{i} spin preserves position (pure rotation)",
                rest_pos is not None
                and spun_pos is not None
                and abs(spun_pos[0] - rest_pos[0]) < 0.002
                and abs(spun_pos[1] - rest_pos[1]) < 0.002
                and abs(spun_pos[2] - rest_pos[2]) < 0.002,
                details=f"rest={rest_pos}, spun={spun_pos}",
            )
        # Verify the knob actually rotates (AABB changes under rotation).
        rest_aabb = ctx.part_world_aabb(dimmer)
        with ctx.pose({spin: math.pi / 4.0}):
            rot_aabb = ctx.part_world_aabb(dimmer)
            ctx.check(
                f"dimmer_{i} rotated pose changes the bounding box",
                rest_aabb is not None
                and rot_aabb is not None
                and (
                    abs(rot_aabb[1][0] - rest_aabb[1][0]) > 0.0005
                    or abs(rot_aabb[1][2] - rest_aabb[1][2]) > 0.0005
                    or abs(rot_aabb[0][0] - rest_aabb[0][0]) > 0.0005
                    or abs(rot_aabb[0][2] - rest_aabb[0][2]) > 0.0005
                ),
                details=f"rest_aabb={rest_aabb}, rot_aabb={rot_aabb}",
            )

    return ctx.report()


object_model = build_object_model()
