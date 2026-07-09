from __future__ import annotations

# Freestanding stainless-steel gas range — French double door variant
# reference: picture/Other/stove/002.png
#
# Layout (meters, Z up, +X is the front of the range):
# - cabinet 0.55 wide (Y), 0.60 deep (X), cooktop surface at z = 0.90
# - four gas burners with silver caps and thin wire pan supports on the deck
# - rear-hinged tempered-glass lid (revolute, 0 .. ~100 deg)
# - forward-slanted control panel with five round black knobs (revolute, 0 .. 270 deg)
# - French double oven door: two mirrored half-doors on vertical hinges at the
#   outer edges, meeting in the middle (each revolute, 0 .. 90 deg)
# - two interior wire racks; the upper rack slides out (prismatic, 0 .. 0.30 m)

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
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
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Global dimensions
# ----------------------------------------------------------------------------
WIDTH = 0.55  # Y
DEPTH = 0.60  # X
DECK_TOP = 0.90  # cooktop surface height
DECK_THICK = 0.025

HALF_W = WIDTH / 2.0
HALF_D = DEPTH / 2.0

# Control panel face line (in the XZ plane): top edge tucked under the deck,
# bottom edge slightly proud above the oven door (forward slant).
PANEL_TOP = (0.255, DECK_TOP - DECK_THICK)  # (x, z) of face top edge
PANEL_BOT = (0.305, 0.66)  # (x, z) of face bottom edge
PANEL_TILT = math.atan2(PANEL_BOT[0] - PANEL_TOP[0], PANEL_TOP[1] - PANEL_BOT[1])
PANEL_NORMAL = (math.cos(PANEL_TILT), 0.0, math.sin(PANEL_TILT))
PANEL_FACE_CENTER = (
    (PANEL_TOP[0] + PANEL_BOT[0]) / 2.0,
    0.0,
    (PANEL_TOP[1] + PANEL_BOT[1]) / 2.0,
)
PANEL_SLOPE_LEN = math.hypot(PANEL_BOT[0] - PANEL_TOP[0], PANEL_TOP[1] - PANEL_BOT[1])
PANEL_THICK = 0.02

KNOB_YS = (-0.19, -0.095, 0.0, 0.095, 0.19)

# Burners (centers on the deck)
BURNER_XS = (-0.09, 0.14)
BURNER_YS = (-0.13, 0.13)
BURNER_BASE_R = 0.048
BURNER_BASE_H = 0.012
BURNER_CAP_R = 0.038
BURNER_CAP_H = 0.008

GRATE_HALF = 0.105  # half size of the square wire pan support
GRATE_WIRE_R = 0.0035
GRATE_TOP_Z = 0.026  # wire centerline height above deck in grate-local frame

# Lid (rear hinge)
LID_HINGE_X = -0.295
LID_HINGE_Z = 0.937
LID_LEN = 0.62
LID_HALF_W = 0.25
LID_GLASS_T = 0.006
LID_OPEN_MAX = math.radians(100.0)

# French double oven door
DOOR_HINGE_X = 0.305
DOOR_HINGE_Z = 0.225  # bottom of the door opening
DOOR_H = 0.42  # door height
DOOR_W = 0.50  # total door width
HALF_DOOR_W = DOOR_W / 2.0  # 0.25 per half
DOOR_FRAME_T = 0.022
DOOR_CENTER_Z = DOOR_HINGE_Z + DOOR_H / 2.0  # 0.435

# Oven cavity / racks
CAVITY_FLOOR_TOP = 0.26
CAVITY_CEIL_BOT = 0.60
RACK_LEN = 0.45  # rack depth along X
RACK_HALF_W = 0.205
RACK_WIRE_R = 0.004
UPPER_RACK_Z = 0.4055
LOWER_RACK_Z = 0.3055
RACK_X_CENTER = -0.025  # rest position of rack center along X
RACK_TRAVEL = 0.30


def _cq_cylinder(radius: float, length: float) -> cq.Workplane:
    """Cylinder along +Z from z=0 to z=length."""
    return cq.Workplane("XY").circle(radius).extrude(length)


def _build_grate_mesh():
    """Square wire pan support: perimeter frame, 4 legs, 4 inward finger bars.

    Local frame: centered on the burner axis, z=0 at the deck surface.
    """
    r = GRATE_WIRE_R
    h = GRATE_HALF
    z = GRATE_TOP_Z
    frame_len = 2.0 * h

    grate = None

    def _union(shape: cq.Workplane):
        nonlocal grate
        grate = shape if grate is None else grate.union(shape)

    # Perimeter frame wires (two along X, two along Y) at height z.
    for sy in (-h, h):
        bar = (
            _cq_cylinder(r, frame_len)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .translate((-h, sy, z))
        )
        _union(bar)
    for sx in (-h, h):
        bar = (
            _cq_cylinder(r, frame_len)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((sx, -h, z))
        )
        _union(bar)
    # Corner legs down to the deck (slightly below z=0 so they seat in the deck).
    for sx in (-h, h):
        for sy in (-h, h):
            leg = _cq_cylinder(r, z + r + 0.001).translate((sx, sy, -0.001))
            _union(leg)
    # Finger bars from each frame side toward the burner center.
    finger_inner = 0.030
    finger_len = h - finger_inner
    for sgn in (-1.0, 1.0):
        fx = (
            _cq_cylinder(r, finger_len)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .translate((sgn * h if sgn < 0 else finger_inner, 0.0, z))
        )
        _union(fx)
        fy = (
            _cq_cylinder(r, finger_len)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((0.0, sgn * h if sgn < 0 else finger_inner, z))
        )
        _union(fy)
    return mesh_from_cadquery(grate, "pan_support_grate")


def _build_rack_mesh():
    """Oven wire rack: perimeter frame + cross wires, centered, wires at z=0."""
    r = RACK_WIRE_R
    half_len = RACK_LEN / 2.0
    rack = None

    def _union(shape: cq.Workplane):
        nonlocal rack
        rack = shape if rack is None else rack.union(shape)

    # Side wires along X.
    for sy in (-RACK_HALF_W, RACK_HALF_W):
        bar = (
            _cq_cylinder(r, RACK_LEN)
            .rotate((0, 0, 0), (0, 1, 0), 90)
            .translate((-half_len, sy, 0.0))
        )
        _union(bar)
    # Cross wires along Y (front, back, and interior).
    n_cross = 8
    for i in range(n_cross):
        x = -half_len + r + i * (RACK_LEN - 2.0 * r) / (n_cross - 1)
        bar = (
            _cq_cylinder(r, 2.0 * RACK_HALF_W)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((x, -RACK_HALF_W, 0.0))
        )
        _union(bar)
    return mesh_from_cadquery(rack, "oven_wire_rack")


def _build_lid_glass_mesh():
    """Tempered-glass lid panel with rounded far (top-when-open) corners.

    Lid-local frame: hinge axis along Y at the origin; the closed panel extends
    along +X from x=0.012 to x=0.012+LID_LEN, thickness along Z.
    """
    panel = (
        cq.Workplane("XY")
        .box(LID_LEN, 2.0 * LID_HALF_W, LID_GLASS_T)
        .translate((0.012 + LID_LEN / 2.0, 0.0, 0.0))
        .edges("|Z and >X")
        .fillet(0.06)
    )
    return mesh_from_cadquery(panel, "lid_glass_panel")


def _build_half_door_frame_mesh():
    """Half-width door frame bezel for one French door leaf."""
    return mesh_from_geometry(
        BezelGeometry(
            (0.17, 0.26),
            (HALF_DOOR_W, DOOR_H),
            DOOR_FRAME_T,
            opening_shape="rounded_rect",
            outer_shape="rounded_rect",
            opening_corner_radius=0.018,
            outer_corner_radius=0.012,
        ),
        "half_door_frame",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_gas_range")

    steel = model.material("stainless_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    brushed = model.material("brushed_panel", rgba=(0.65, 0.66, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.36, 0.38, 1.0))
    black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))
    cap_silver = model.material("burner_silver", rgba=(0.80, 0.81, 0.82, 1.0))
    chrome = model.material("chrome_wire", rgba=(0.62, 0.63, 0.65, 1.0))
    lid_glass = model.material("lid_glass", rgba=(0.55, 0.60, 0.62, 0.45))
    door_glass = model.material("door_glass", rgba=(0.13, 0.14, 0.16, 0.55))
    enamel = model.material("cavity_enamel", rgba=(0.22, 0.22, 0.23, 1.0))

    # ------------------------------------------------------------------
    # body (root): plinth, cabinet shell, deck, burners, grates, panel
    # ------------------------------------------------------------------
    body = model.part("body")

    # Plinth base (recessed kick).
    body.visual(
        Box((0.56, 0.51, 0.08)),
        origin=Origin(xyz=(-0.01, 0.0, 0.04)),
        material=dark_steel,
        name="plinth",
    )
    # Side walls (double as oven cavity side walls).
    for sgn, nm in ((-1.0, "side_wall_0"), (1.0, "side_wall_1")):
        body.visual(
            Box((DEPTH, 0.06, 0.81)),
            origin=Origin(xyz=(0.0, sgn * 0.245, 0.475)),
            material=steel,
            name=nm,
        )
    # Back wall.
    body.visual(
        Box((0.04, 0.45, 0.81)),
        origin=Origin(xyz=(-0.28, 0.0, 0.475)),
        material=steel,
        name="back_wall",
    )
    # Oven cavity floor and ceiling.
    body.visual(
        Box((0.55, 0.45, 0.03)),
        origin=Origin(xyz=(0.005, 0.0, 0.245)),
        material=enamel,
        name="cavity_floor",
    )
    body.visual(
        Box((0.55, 0.45, 0.03)),
        origin=Origin(xyz=(0.005, 0.0, 0.615)),
        material=enamel,
        name="cavity_ceiling",
    )
    # Front rail above the oven opening (behind the control panel bottom).
    body.visual(
        Box((0.03, 0.50, 0.06)),
        origin=Origin(xyz=(0.285, 0.0, 0.65)),
        material=steel,
        name="front_top_rail",
    )
    # Lower front panel below the oven door.
    body.visual(
        Box((0.03, 0.50, 0.155)),
        origin=Origin(xyz=(0.285, 0.0, 0.1575)),
        material=steel,
        name="lower_front_panel",
    )
    # Cooktop deck.
    body.visual(
        Box((DEPTH, WIDTH, DECK_THICK)),
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP - DECK_THICK / 2.0)),
        material=steel,
        name="cooktop_deck",
    )
    # Forward-slanted brushed-metal control panel.
    half_th = PANEL_THICK / 2.0
    panel_center = (
        PANEL_FACE_CENTER[0] - half_th * PANEL_NORMAL[0],
        0.0,
        PANEL_FACE_CENTER[2] - half_th * PANEL_NORMAL[2],
    )
    body.visual(
        Box((PANEL_THICK, WIDTH, PANEL_SLOPE_LEN + 0.016)),
        origin=Origin(xyz=panel_center, rpy=(0.0, -PANEL_TILT, 0.0)),
        material=brushed,
        name="control_panel",
    )

    # Burners (base + silver cap) and wire pan supports.
    grate_mesh = _build_grate_mesh()
    idx = 0
    for bx in BURNER_XS:
        for by in BURNER_YS:
            body.visual(
                Cylinder(radius=BURNER_BASE_R, length=BURNER_BASE_H),
                origin=Origin(xyz=(bx, by, DECK_TOP + BURNER_BASE_H / 2.0)),
                material=dark_steel,
                name=f"burner_base_{idx}",
            )
            body.visual(
                Cylinder(radius=BURNER_CAP_R, length=BURNER_CAP_H),
                origin=Origin(
                    xyz=(bx, by, DECK_TOP + BURNER_BASE_H + BURNER_CAP_H / 2.0)
                ),
                material=cap_silver,
                name=f"burner_cap_{idx}",
            )
            body.visual(
                grate_mesh,
                origin=Origin(xyz=(bx, by, DECK_TOP - 0.0005)),
                material=chrome,
                name=f"pan_support_{idx}",
            )
            idx += 1

    # Lid hinge brackets + barrels at the rear edge of the deck.
    for i, sy in enumerate((-0.20, 0.20)):
        body.visual(
            Box((0.03, 0.04, 0.035)),
            origin=Origin(xyz=(-0.285, sy, 0.9125)),
            material=steel,
            name=f"lid_hinge_bracket_{i}",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.05),
            origin=Origin(xyz=(LID_HINGE_X, sy, LID_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"lid_hinge_barrel_{i}",
        )

    # French door hinge barrels (vertical, at outer edges of door opening).
    for i, sy in enumerate((-HALF_DOOR_W, HALF_DOOR_W)):
        body.visual(
            Cylinder(radius=0.009, length=0.40),
            origin=Origin(xyz=(DOOR_HINGE_X, sy, DOOR_CENTER_Z)),
            material=dark_steel,
            name=f"door_hinge_barrel_{i}",
        )

    # Rack support rails on the cavity side walls (two levels).
    for level, rail_top in (("upper", UPPER_RACK_Z - RACK_WIRE_R + 0.0005),
                            ("lower", LOWER_RACK_Z - RACK_WIRE_R + 0.0005)):
        for i, sy in enumerate((-1.0, 1.0)):
            body.visual(
                Box((0.51, 0.012, 0.008)),
                origin=Origin(xyz=(0.01, sy * 0.209, rail_top - 0.004)),
                material=enamel,
                name=f"{level}_rack_rail_{i}",
            )

    # ------------------------------------------------------------------
    # lid: tempered-glass panel hinged at the rear edge of the cooktop
    # ------------------------------------------------------------------
    lid = model.part("lid")
    lid.visual(
        _build_lid_glass_mesh(),
        origin=Origin(),
        material=lid_glass,
        name="lid_glass",
    )
    for i, sy in enumerate((-0.20, 0.20)):
        lid.visual(
            Box((0.034, 0.018, 0.011)),
            origin=Origin(xyz=(0.012, sy, 0.0)),
            material=dark_steel,
            name=f"hinge_leaf_{i}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, LID_HINGE_Z)),
        # Closed panel extends along +X from the hinge; -Y lifts the free edge up.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=LID_OPEN_MAX),
    )

    # ------------------------------------------------------------------
    # French double oven door: two mirrored half-doors on vertical hinges
    # ------------------------------------------------------------------
    door_frame_mesh = _build_half_door_frame_mesh()

    # hinge_y positions and axis z-signs for left (door_0) and right (door_1)
    # door_0: hinge at left outer edge (y=-0.25), axis (0,0,-1) opens outward
    # door_1: hinge at right outer edge (y=+0.25), axis (0,0,+1) opens outward
    door_configs = (
        (-HALF_DOOR_W, -1.0),  # door_0: left
        (HALF_DOOR_W, 1.0),    # door_1: right
    )

    for i, (hinge_y, axis_z) in enumerate(door_configs):
        door = model.part(f"door_{i}")

        # Direction from hinge toward meeting edge in door-local Y
        y_sign = 1.0 if hinge_y < 0 else -1.0
        y_center = y_sign * HALF_DOOR_W / 2.0  # center of the half-door panel

        # Bezel frame: local +Z (depth) -> door +X, +X (width) -> door +Y, +Y (height) -> door +Z
        door.visual(
            door_frame_mesh,
            origin=Origin(
                xyz=(DOOR_FRAME_T / 2.0, y_center, 0.0),
                rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
            ),
            material=black,
            name="door_frame",
        )

        # Glass window (slightly larger than bezel opening, seated inside)
        glass_half_w = 0.19  # slightly wider than 0.17 opening
        glass_h = 0.28       # slightly taller than 0.26 opening
        door.visual(
            Box((0.006, glass_half_w, glass_h)),
            origin=Origin(xyz=(0.008, y_center, 0.0)),
            material=door_glass,
            name="door_glass",
        )

        # Vertical handle near the meeting edge, on two standoffs
        handle_y = y_sign * (HALF_DOOR_W - 0.035)
        handle_x = DOOR_FRAME_T + 0.035
        standoff_len = 0.030
        standoff_cx = DOOR_FRAME_T + standoff_len / 2.0
        for j, dz in enumerate((-0.09, 0.09)):
            door.visual(
                Cylinder(radius=0.007, length=standoff_len),
                origin=Origin(
                    xyz=(standoff_cx, handle_y, dz),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=steel,
                name=f"handle_standoff_{j}",
            )
        door.visual(
            Cylinder(radius=0.009, length=0.22),
            origin=Origin(xyz=(handle_x, handle_y, 0.0)),
            material=steel,
            name="door_handle",
        )

        # Hinge pin (vertical cylinder captured in the body barrel)
        door.visual(
            Cylinder(radius=0.006, length=0.38),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark_steel,
            name="hinge_pin",
        )

        model.articulation(
            f"door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(DOOR_HINGE_X, hinge_y, DOOR_CENTER_Z)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(
                effort=30.0, velocity=1.5, lower=0.0, upper=math.pi / 2.0
            ),
        )

    # ------------------------------------------------------------------
    # knobs: five round black knobs on the slanted panel
    # ------------------------------------------------------------------
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.024,
            body_style="skirted",
            top_diameter=0.036,
            skirt=KnobSkirt(0.054, 0.006, flare=0.08),
            grip=KnobGrip(style="fluted", count=18, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            center=False,
        ),
        "range_knob",
    )
    knob_pitch = math.pi / 2.0 - PANEL_TILT
    for i, ky in enumerate(KNOB_YS):
        knob = model.part(f"knob_{i}")
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.0015)),
            material=knob_black,
            name="knob_shell",
        )
        model.articulation(
            f"knob_{i}_turn",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(
                xyz=(PANEL_FACE_CENTER[0], ky, PANEL_FACE_CENTER[2]),
                rpy=(0.0, knob_pitch, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=4.0, lower=0.0, upper=math.radians(270.0)
            ),
        )

    # ------------------------------------------------------------------
    # oven racks: upper rack slides out, lower rack fixed
    # ------------------------------------------------------------------
    rack_mesh = _build_rack_mesh()

    upper_rack = model.part("upper_rack")
    upper_rack.visual(rack_mesh, origin=Origin(), material=chrome, name="rack_wires")
    model.articulation(
        "upper_rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(RACK_X_CENTER, 0.0, UPPER_RACK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.5, lower=0.0, upper=RACK_TRAVEL),
    )

    lower_rack = model.part("lower_rack")
    lower_rack.visual(rack_mesh, origin=Origin(), material=chrome, name="rack_wires")
    model.articulation(
        "lower_rack_mount",
        ArticulationType.FIXED,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(RACK_X_CENTER, 0.0, LOWER_RACK_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    doors = [object_model.get_part(f"door_{i}") for i in range(2)]
    upper_rack = object_model.get_part("upper_rack")
    lower_rack = object_model.get_part("lower_rack")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(5)]

    lid_hinge = object_model.get_articulation("lid_hinge")
    door_hinges = [object_model.get_articulation(f"door_{i}_hinge") for i in range(2)]
    rack_slide = object_model.get_articulation("upper_rack_slide")

    # ----- intentional local embeddings -------------------------------
    for i, knob in enumerate(knobs):
        ctx.allow_overlap(
            knob,
            body,
            elem_a="knob_shell",
            elem_b="control_panel",
            reason="Knob skirt is intentionally seated 1.5 mm into the slanted panel.",
        )
    for i in range(2):
        ctx.allow_overlap(
            lid,
            body,
            elem_a=f"hinge_leaf_{i}",
            elem_b=f"lid_hinge_barrel_{i}",
            reason="Lid hinge leaf intentionally wraps the captured hinge barrel.",
        )
    # French door hinge pins captured inside body barrels
    for i in range(2):
        ctx.allow_overlap(
            doors[i],
            body,
            elem_a="hinge_pin",
            elem_b=f"door_hinge_barrel_{i}",
            reason=f"Door {i} hinge pin is captured inside the vertical hinge barrel.",
        )
    # French door hinge barrels captured at the outer edge of each door frame
    for i in range(2):
        ctx.allow_overlap(
            doors[i],
            body,
            elem_a="door_frame",
            elem_b=f"door_hinge_barrel_{i}",
            reason=f"Door {i} frame wraps the vertical hinge barrel at its outer edge.",
        )
    for i in range(2):
        ctx.allow_overlap(
            upper_rack,
            body,
            elem_a="rack_wires",
            elem_b=f"upper_rack_rail_{i}",
            reason="Rack side wires seat 0.5 mm into their support rail.",
        )
        ctx.allow_overlap(
            lower_rack,
            body,
            elem_a="rack_wires",
            elem_b=f"lower_rack_rail_{i}",
            reason="Rack side wires seat 0.5 mm into their support rail.",
        )

    # ----- cooktop: four burners with silver caps under wire grates ----
    deck = ctx.part_element_world_aabb(body, elem="cooktop_deck")
    ctx.check(
        "cooktop surface at 0.90 m",
        deck is not None and abs(deck[1][2] - DECK_TOP) < 1e-6,
        details=f"deck aabb={deck!r}",
    )
    for i in range(4):
        cap = ctx.part_element_world_aabb(body, elem=f"burner_cap_{i}")
        grate = ctx.part_element_world_aabb(body, elem=f"pan_support_{i}")
        ctx.check(
            f"burner cap {i} sits on its burner base above the deck",
            cap is not None and cap[0][2] > DECK_TOP and cap[1][2] < DECK_TOP + 0.03,
            details=f"cap aabb={cap!r}",
        )
        ctx.check(
            f"pan support {i} wires span above burner cap {i}",
            cap is not None
            and grate is not None
            and grate[1][2] > cap[1][2] + 0.001
            and grate[0][2] < DECK_TOP,  # legs reach down into the deck surface
            details=f"cap={cap!r} grate={grate!r}",
        )

    # ----- glass lid ----------------------------------------------------
    glass = ctx.part_element_world_aabb(lid, elem="lid_glass")
    ctx.check(
        "lid glass is slightly deeper than the cooktop and full width",
        glass is not None
        and 0.60 < (glass[1][0] - glass[0][0]) < 0.66
        and 0.46 < (glass[1][1] - glass[0][1]) < 0.52,
        details=f"lid glass aabb={glass!r}",
    )
    # Closed lid hovers just above the pan supports, never embedded in them.
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_glass",
        negative_elem="pan_support_0",
        min_gap=0.001,
        max_gap=0.03,
        name="closed lid clears the pan supports",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.20,
        name="closed lid covers the cooktop footprint",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="hinge_leaf_0",
        elem_b="lid_hinge_barrel_0",
        name="lid hinge leaf engages the rear hinge barrel",
    )
    with ctx.pose({lid_hinge: 1.70}):
        open_glass = ctx.part_element_world_aabb(lid, elem="lid_glass")
        ctx.check(
            "raised lid stands nearly vertical above the cooktop",
            open_glass is not None
            and open_glass[1][2] > 1.40
            and open_glass[1][0] < 0.10,
            details=f"open lid aabb={open_glass!r}",
        )

    # ----- French double oven door ------------------------------------
    # Both doors are closed at q=0: verify they meet in the middle
    for i, door in enumerate(doors):
        # Each door hinge pin contacts its barrel
        ctx.expect_contact(
            door,
            body,
            elem_a="hinge_pin",
            elem_b=f"door_hinge_barrel_{i}",
            name=f"door_{i} hinge pin engages the body barrel",
        )
        # Each door has a frame and glass window
        frame_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
        glass_aabb = ctx.part_element_world_aabb(door, elem="door_glass")
        ctx.check(
            f"door_{i} has a framed glass window",
            frame_aabb is not None
            and glass_aabb is not None
            and glass_aabb[0][0] > frame_aabb[0][0] - 0.001
            and glass_aabb[1][0] < frame_aabb[1][0] + 0.001,
            details=f"frame={frame_aabb!r} glass={glass_aabb!r}",
        )
        # Handle is forward of the door frame
        handle_aabb = ctx.part_element_world_aabb(door, elem="door_handle")
        ctx.check(
            f"door_{i} handle is forward of the door face",
            frame_aabb is not None
            and handle_aabb is not None
            and handle_aabb[0][0] > frame_aabb[1][0] - 0.005,
            details=f"frame={frame_aabb!r} handle={handle_aabb!r}",
        )

    # Doors meet at center: left door right edge contacts right door left edge
    ctx.expect_contact(
        doors[0],
        doors[1],
        elem_a="door_frame",
        elem_b="door_frame",
        contact_tol=0.005,
        name="French doors meet at the center when closed",
    )

    # Each door opens outward: positive q moves the meeting edge forward (+X)
    for i, hinge in enumerate(door_hinges):
        rest_aabb = ctx.part_world_aabb(doors[i])
        with ctx.pose({hinge: math.pi / 2.0}):
            open_aabb = ctx.part_world_aabb(doors[i])
            ctx.check(
                f"door_{i} opens outward (meeting edge swings forward)",
                rest_aabb is not None
                and open_aabb is not None
                and open_aabb[1][0] > rest_aabb[1][0] + 0.10,
                details=f"rest={rest_aabb!r} open={open_aabb!r}",
            )

    # Verify each hinge is vertical (axis Z component dominates)
    for i, hinge in enumerate(door_hinges):
        ax = hinge.axis
        ctx.check(
            f"door_{i} hinge axis is vertical",
            ax is not None and abs(ax[2]) > 0.99,
            details=f"axis={ax!r}",
        )

    # Hinges are at outer edges (symmetric about center)
    d0_aabb = ctx.part_world_aabb(doors[0])
    d1_aabb = ctx.part_world_aabb(doors[1])
    ctx.check(
        "French doors span the full oven opening width",
        d0_aabb is not None
        and d1_aabb is not None
        and d1_aabb[1][1] - d0_aabb[0][1] > 0.40,
        details=f"door_0={d0_aabb!r} door_1={d1_aabb!r}",
    )

    # ----- knobs --------------------------------------------------------
    for i, knob in enumerate(knobs):
        ctx.expect_contact(
            knob,
            body,
            elem_a="knob_shell",
            elem_b="control_panel",
            name=f"knob {i} is seated on the slanted control panel",
        )
        joint = object_model.get_articulation(f"knob_{i}_turn")
        limits = joint.motion_limits
        ctx.check(
            f"knob {i} turns about 270 degrees",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.upper - limits.lower - math.radians(270.0)) < 1e-6,
            details=f"limits={limits!r}",
        )
    k0 = ctx.part_world_aabb(knobs[0])
    k4 = ctx.part_world_aabb(knobs[4])
    ctx.check(
        "five knobs form a row across the panel front",
        k0 is not None and k4 is not None and k4[0][1] - k0[1][1] > 0.25,
        details=f"k0={k0!r} k4={k4!r}",
    )

    # ----- oven racks ----------------------------------------------------
    for rack, level in ((upper_rack, "upper"), (lower_rack, "lower")):
        ctx.expect_within(
            rack,
            body,
            axes="y",
            margin=0.0,
            name=f"{level} rack stays between the cavity walls",
        )
        ctx.expect_contact(
            rack,
            body,
            elem_a="rack_wires",
            elem_b=f"{level}_rack_rail_0",
            name=f"{level} rack rests on its side rails",
        )
    up = ctx.part_world_aabb(upper_rack)
    low = ctx.part_world_aabb(lower_rack)
    ctx.check(
        "two racks stacked inside the oven cavity",
        up is not None
        and low is not None
        and up[0][2] > low[1][2] + 0.05
        and low[0][2] > CAVITY_FLOOR_TOP,
        details=f"upper={up!r} lower={low!r}",
    )
    rest_rack = ctx.part_world_aabb(upper_rack)
    # Open both French doors to pull out the rack
    with ctx.pose({
        door_hinges[0]: math.pi / 2.0,
        door_hinges[1]: math.pi / 2.0,
        rack_slide: RACK_TRAVEL,
    }):
        out_rack = ctx.part_world_aabb(upper_rack)
        ctx.check(
            "upper rack pulls out 0.30 m through the open French doors",
            rest_rack is not None
            and out_rack is not None
            and abs((out_rack[1][0] - rest_rack[1][0]) - RACK_TRAVEL) < 1e-6
            and out_rack[1][0] > 0.45,
            details=f"rest={rest_rack!r} out={out_rack!r}",
        )
        ctx.expect_overlap(
            upper_rack,
            body,
            axes="x",
            elem_a="rack_wires",
            elem_b="upper_rack_rail_0",
            min_overlap=0.15,
            name="pulled-out rack retains insertion on its rails",
        )

    return ctx.report()


object_model = build_object_model()
