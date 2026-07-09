from __future__ import annotations

# European three-gang wall switch plate — push-button variant:
# Same faceplate, trim ring, Schuko socket, and wall panel as the parent.
# The two rocker modules are replaced by two square push-button actuators that
# press straight into the plate (PRISMATIC along -Y). Each button is an
# independent prismatic joint with a 3 mm travel range.
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

# Three ~0.055 m modules, left to right: button, button, Schuko socket
MODULE_PITCH = 0.075
BUTTON_X = (-MODULE_PITCH, 0.0)
SOCKET_X = MODULE_PITCH

# Module pockets cut into the plate (square openings with a thin back floor)
POCKET_SIZE = 0.054
POCKET_R = 0.004
POCKET_FLOOR_Y = 0.0025  # back floor of the pocket (thin ivory wall remains)

# Push-button actuators: square ivory caps with a stepped guide section.
# The cap face is wider than the pocket so it rests on the pocket rim (real
# push-button design: the wider front lip prevents the button from falling in).
N_BUTTONS = 2
BUTTON_SIZE = 0.056  # cap face, wider than pocket (0.054) to seat on rim
BUTTON_CAP_T = 0.006  # cap thickness (front face to back face)
BUTTON_BACK_DEPTH = 0.002  # portion of the cap behind the joint origin
BUTTON_FRONT_PROUD = BUTTON_CAP_T - BUTTON_BACK_DEPTH  # 0.004 in front
BUTTON_GUIDE_SIZE = 0.050  # narrower guide section inside the pocket
BUTTON_GUIDE_L = 0.002  # guide section length
BUTTON_TRAVEL = 0.003  # 3 mm press travel

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
    """Matte ivory field: two square module pockets (thin back floor kept) and
    a raised Schuko tile with a true hollow circular well plus two pin holes."""
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

    # Square module pockets: open the front, keep a thin floor at the back.
    for xc in BUTTON_X:
        pocket = (
            cq.Workplane("XY", origin=(xc, 0.0, POCKET_FLOOR_Y))
            .placeSketch(_rounded_rect(POCKET_SIZE, POCKET_SIZE, POCKET_R))
            .extrude(PLATE_DEPTH)
        )
        plate = plate.cut(pocket)

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


def _button_mesh() -> object:
    """Square ivory push-button cap with a stepped guide section.
    Built in CQ frame with z=0 at the plate front face (joint origin):
    the cap front extends +z (outward from wall) and the guide section
    extends -z (into the pocket)."""
    # Main cap face: from z = -BUTTON_BACK_DEPTH to z = +BUTTON_FRONT_PROUD
    cap = (
        cq.Workplane("XY", origin=(0.0, 0.0, -BUTTON_BACK_DEPTH))
        .placeSketch(_rounded_rect(BUTTON_SIZE, BUTTON_SIZE, 0.004))
        .extrude(BUTTON_CAP_T)
    )
    # Narrower guide section extending into the pocket behind the cap
    guide = (
        cq.Workplane("XY", origin=(0.0, 0.0, -BUTTON_BACK_DEPTH - BUTTON_GUIDE_L))
        .placeSketch(_rounded_rect(BUTTON_GUIDE_SIZE, BUTTON_GUIDE_SIZE, 0.003))
        .extrude(BUTTON_GUIDE_L)
    )
    button = cap.union(guide)
    # Soft fillet on the front edges for a finished look
    try:
        button = button.edges(">Z").fillet(0.0008)
    except Exception:
        pass
    return mesh_from_cadquery(button, "button_cap")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_gang_wall_switch_plate")

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

    # Two independent push-button actuators: PRISMATIC along -Y (into the
    # plate). Positive q presses the button inward.
    button_mesh = _button_mesh()
    for i in range(N_BUTTONS):
        xc = BUTTON_X[i]
        button = model.part(f"button_{i}")
        button.visual(
            button_mesh,
            origin=Origin(rpy=_CQ_TO_WALL),
            material=ivory,
            name="button_cap",
        )
        model.articulation(
            f"button_{i}_press",
            ArticulationType.PRISMATIC,
            parent=faceplate,
            child=button,
            origin=Origin(xyz=(xc, PLATE_DEPTH, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.5,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_panel")
    faceplate = object_model.get_part("faceplate")
    buttons = [object_model.get_part(f"button_{i}") for i in range(N_BUTTONS)]
    presses = [object_model.get_articulation(f"button_{i}_press") for i in range(N_BUTTONS)]

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

    # --- Push-button actuators: each centered on its module position, sitting
    # proud of the chrome trim at rest. The wider cap face seats on the pocket
    # rim (intentional local overlap at the flange seating surface). ---
    for i, (button, xc) in enumerate(zip(buttons, BUTTON_X)):
        pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} centered on its module (x={xc:+.3f})",
            pos is not None and abs(pos[0] - xc) < 0.002 and abs(pos[2]) < 0.002,
            details=f"pos={pos}",
        )
        ctx.expect_within(
            button,
            faceplate,
            axes="xz",
            margin=0.002,
            name=f"button_{i} stays inside the plate outline",
        )
        b_aabb = ctx.part_world_aabb(button)
        if b_aabb is not None and trim_aabb is not None:
            ctx.check(
                f"button_{i} cap sits proud of the chrome lip at rest",
                b_aabb[1][1] > trim_aabb[1][1] + 0.001,
                details=f"button_front={b_aabb[1][1]:.4f}, trim_front={trim_aabb[1][1]:.4f}",
            )

        # The wider cap flange seats on the pocket rim: small intentional
        # overlap at the seating surface, scoped to the cap/plate interface.
        ctx.allow_overlap(
            button,
            faceplate,
            elem_a="button_cap",
            elem_b="plate_field",
            reason=(
                "Button cap flange is intentionally wider than the pocket "
                "opening and seats on the pocket rim, creating a thin local "
                "overlap at the contact surface."
            ),
        )
        ctx.expect_contact(
            button,
            faceplate,
            elem_a="button_cap",
            elem_b="plate_field",
            name=f"button_{i} cap seats on the pocket rim",
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

    # --- Push-button prismatic articulation: correct limits, pressing moves
    # the button inward (-Y), and the button stays clear of the wall. ---
    for i, (button, press) in enumerate(zip(buttons, presses)):
        limits = press.motion_limits
        ctx.check(
            f"button_{i} travel range is 0..{BUTTON_TRAVEL:.3f} m",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower) < 1e-6
            and abs(limits.upper - BUTTON_TRAVEL) < 1e-6,
            details=f"limits=({limits.lower}, {limits.upper})",
        )

        # Prismatic joint is PRISMATIC type
        ctx.check(
            f"button_{i} articulation is PRISMATIC",
            press.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={press.articulation_type}",
        )

        # At rest: button front is proud of the plate
        rest_aabb = ctx.part_world_aabb(button)

        # Fully pressed: button moves inward
        with ctx.pose({press: BUTTON_TRAVEL}):
            pressed_aabb = ctx.part_world_aabb(button)
            ctx.check(
                f"button_{i} pressing moves it inward (-Y)",
                rest_aabb is not None
                and pressed_aabb is not None
                and pressed_aabb[1][1] < rest_aabb[1][1] - 0.001,
                details=f"rest_front={rest_aabb[1][1] if rest_aabb else None}, "
                        f"pressed_front={pressed_aabb[1][1] if pressed_aabb else None}",
            )
            # Pressed button should not penetrate the wall
            ctx.expect_gap(
                button,
                wall,
                axis="y",
                max_penetration=0.0,
                name=f"button_{i} pressed stays clear of the wall",
            )
            # Pressed button back should stay clear of the pocket floor
            ctx.check(
                f"button_{i} pressed back stays clear of pocket floor",
                pressed_aabb is not None and pressed_aabb[0][1] > POCKET_FLOOR_Y + 0.0002,
                details=f"pressed_back={pressed_aabb[0][1] if pressed_aabb else None}, "
                        f"pocket_floor={POCKET_FLOOR_Y}",
            )

    return ctx.report()


object_model = build_object_model()
