from __future__ import annotations

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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _rounded_plate(
    length: float,
    width: float,
    thickness: float,
    *,
    radius: float = 0.002,
    round_vertical_edges: bool = True,
    round_holes: tuple[tuple[float, float, float], ...] = (),
    slots: tuple[tuple[float, float, float, float], ...] = (),
    rect_holes: tuple[tuple[float, float, float, float], ...] = (),
) -> cq.Workplane:
    """Thin XY plate extruded upward from z=0 with optional through holes."""

    wp = cq.Workplane("XY").rect(length, width).extrude(thickness)
    if round_vertical_edges and radius > 0:
        wp = wp.edges("|Z").fillet(min(radius, width * 0.45, length * 0.45))

    if round_holes:
        wp = (
            wp.faces(">Z")
            .workplane(centerOption="CenterOfBoundBox")
            .pushPoints([(x, y) for x, y, _r in round_holes])
        )
        for x, y, r in round_holes:
            cutter = (
                cq.Workplane("XY")
                .center(x, y)
                .circle(r)
                .extrude(thickness * 4.0)
                .translate((0.0, 0.0, -thickness * 1.5))
            )
            wp = wp.cut(cutter)

    for x, y, slot_len, slot_dia in slots:
        cutter = (
            cq.Workplane("XY")
            .center(x, y)
            .slot2D(slot_len, slot_dia, 0)
            .extrude(thickness * 4.0)
            .translate((0.0, 0.0, -thickness * 1.5))
        )
        wp = wp.cut(cutter)

    for x, y, sx, sy in rect_holes:
        cutter = (
            cq.Workplane("XY")
            .center(x, y)
            .rect(sx, sy)
            .extrude(thickness * 4.0)
            .translate((0.0, 0.0, -thickness * 1.5))
        )
        wp = wp.cut(cutter)

    return wp


def _tooth_bar(
    count: int,
    pitch: float,
    tooth_width: float,
    bar_width: float,
    height: float,
) -> cq.Workplane:
    """One connected row of small rectangular serrations on a thin back strip.

    The backing strip is oriented so that:
    - X is the depth direction (thin, ``bar_width``)
    - Y is the tooth-spacing direction (``total`` span)
    - Z is the tooth protrusion direction (``height``)
    Teeth always overlap with the backing strip in both X and Y.
    """

    total = (count - 1) * pitch + tooth_width
    base_depth = bar_width  # depth of backing strip in X
    wp = cq.Workplane("XY").rect(base_depth, total + 0.001).extrude(height * 0.35)
    for i in range(count):
        y = -total / 2.0 + tooth_width / 2.0 + i * pitch
        tooth = (
            cq.Workplane("XY")
            .center(0.0, y)
            .rect(base_depth * 0.85, tooth_width)
            .extrude(height)
        )
        wp = wp.union(tooth)
    return wp


def _bent_lever(
    width: float,
    base_len: float,
    tab_len: float,
    thickness: float,
    bend_angle: float,
) -> cq.Workplane:
    """Sheet-metal finger lever with one bend in XZ, extruded along Y.

    The base lies flat along +X at z=0.  At x = base_len the sheet bends
    upward by *bend_angle* (radians from horizontal) for *tab_len*.
    The solid is centered in Y (width/2 on each side).
    """
    tab_dx = tab_len * math.cos(bend_angle)
    tab_dz = tab_len * math.sin(bend_angle)
    # outward normal of the tab direction (for thickness offset)
    nx = -math.sin(bend_angle)
    nz = math.cos(bend_angle)

    # Cross-section vertices in XZ plane (clockwise outer → inner)
    pts = [
        (0.0, 0.0),                                           # base start, bottom
        (base_len, 0.0),                                      # bend point, bottom
        (base_len + tab_dx, tab_dz),                          # tab tip, bottom
        (base_len + tab_dx + thickness * nx,
         tab_dz + thickness * nz),                            # tab tip, top
        (base_len, thickness),                                # bend point, top
        (0.0, thickness),                                     # base start, top
    ]
    wp = cq.Workplane("XZ").moveTo(*pts[0])
    for p in pts[1:]:
        wp = wp.lineTo(*p)
    wp = wp.close()
    return wp.extrude(width).translate((0.0, width / 2.0, 0.0))


def _ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


# ---------------------------------------------------------------------------
# Key layout dimensions (bulldog clip badge holder)
# ---------------------------------------------------------------------------
_MOUTH_X = -0.013        # front edge of clip (mouth)
_BACK_X = 0.013          # back edge
_CLIP_DEPTH = _BACK_X - _MOUTH_X  # 0.026
_CLIP_WIDTH = 0.038      # 38mm across
_PLATE_THICK = 0.0012    # 1.2mm stamped metal
_HINGE_X = 0.005         # hinge line position
_CHEEK_HEIGHT = 0.013    # side wall height
_CHEEK_THICK = 0.0020    # side wall thickness
_CHEEK_DEPTH = 0.013     # side wall depth (hinge to back)
_HINGE_Z = _PLATE_THICK + _CHEEK_HEIGHT - 0.004  # 0.0102, pin height

_SWIVEL_X = 0.009        # swivel mount on body
_SWIVEL_POST_H = 0.0110  # swivel post height above plate


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="badge_id_holder_clip",
        meta={
            "reference_note": (
                "Bulldog-style badge/ID holder clip: short wide folded-leaf "
                "clamp with twin finger levers and clear swivel connector."
            ),
        },
    )

    chrome = model.material("polished_chrome", rgba=(0.82, 0.82, 0.78, 1.0))
    dark_chrome = model.material("shadowed_chrome", rgba=(0.30, 0.31, 0.31, 1.0))
    clear_vinyl = model.material("clear_translucent_vinyl", rgba=(0.78, 0.92, 1.0, 0.38))
    hole_shadow = model.material("dark_hole_shadow", rgba=(0.02, 0.025, 0.03, 1.0))

    # ------------------------------------------------------------------
    # clip_body  (lower leaf + cheeks + hinge pin + swivel mount)
    # ------------------------------------------------------------------
    body = model.part("clip_body")

    # Lower stamped plate — short, wide bulldog leaf
    lower_plate = _rounded_plate(
        _CLIP_DEPTH, _CLIP_WIDTH, _PLATE_THICK,
        radius=0.003,
    )
    body.visual(
        mesh_from_cadquery(lower_plate, "lower_stamped_plate", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, _PLATE_THICK / 2.0)),
        material=chrome,
        name="lower_stamped_plate",
    )

    # Front gripping lip — slightly upturned edge at the mouth
    body.visual(
        Box((0.005, _CLIP_WIDTH - 0.004, 0.0022)),
        origin=Origin(xyz=(_MOUTH_X + 0.003, 0.0, _PLATE_THICK + 0.0009)),
        material=chrome,
        name="lower_jaw_lip",
    )

    # Lower jaw teeth — serrated gripping bar at the mouth (teeth point up)
    _lower_bar_depth = 0.006
    body.visual(
        mesh_from_cadquery(
            _tooth_bar(8, 0.0038, 0.0025, _lower_bar_depth, 0.0015),
            "lower_jaw_teeth",
        ),
        origin=Origin(xyz=(_MOUTH_X + _lower_bar_depth / 2.0, 0.0,
                            _PLATE_THICK - 0.0002)),
        material=dark_chrome,
        name="lower_jaw_teeth",
    )

    # Folded side cheeks — two stamped side walls carry the hinge pin
    cheek_center_x = (_HINGE_X + _BACK_X) / 2.0
    cheek_center_z = _PLATE_THICK + _CHEEK_HEIGHT / 2.0
    for y, suffix in ((-1, "0"), (1, "1")):
        cheek_y = y * (_CLIP_WIDTH / 2.0 - _CHEEK_THICK / 2.0)
        body.visual(
            Box((_CHEEK_DEPTH, _CHEEK_THICK, _CHEEK_HEIGHT)),
            origin=Origin(xyz=(cheek_center_x, cheek_y, cheek_center_z)),
            material=chrome,
            name=f"side_cheek_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.0030, length=0.0007),
            origin=Origin(xyz=(_HINGE_X, cheek_y * 1.01, _HINGE_Z),
                           rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hole_shadow,
            name=f"cheek_hole_{suffix}",
        )

    # Hinge pin — through-pin captured by the cheeks
    pin_span = _CLIP_WIDTH - 2.0 * _CHEEK_THICK + 0.006
    body.visual(
        Cylinder(radius=0.0020, length=pin_span),
        origin=Origin(xyz=(_HINGE_X, 0.0, _HINGE_Z),
                       rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_chrome,
        name="hinge_pin",
    )

    # Torsion spring — visible coil on the pin
    body.visual(
        mesh_from_geometry(
            TorusGeometry(0.0043, 0.0022, radial_segments=28, tubular_segments=8),
            "torsion_spring",
        ),
        origin=Origin(xyz=(_HINGE_X, -0.009, _HINGE_Z),
                       rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_chrome,
        name="torsion_spring",
    )

    # Swivel post and boss — mount for the badge connector
    body.visual(
        Cylinder(radius=0.0031, length=_SWIVEL_POST_H),
        origin=Origin(xyz=(_SWIVEL_X, 0.0, _PLATE_THICK + _SWIVEL_POST_H / 2.0)),
        material=dark_chrome,
        name="swivel_post",
    )
    body.visual(
        Cylinder(radius=0.0066, length=0.0009),
        origin=Origin(xyz=(_SWIVEL_X, 0.0, _PLATE_THICK + _SWIVEL_POST_H + 0.00045)),
        material=chrome,
        name="swivel_boss",
    )

    # ------------------------------------------------------------------
    # spring_jaw  (upper leaf + finger levers + teeth)
    # ------------------------------------------------------------------
    jaw = model.part("spring_jaw")

    # Hinge barrel — wraps around the pin
    jaw.visual(
        Cylinder(radius=0.0034, length=_CLIP_WIDTH - 2.0 * _CHEEK_THICK - 0.002),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )

    # Barrel wrap — transition from barrel to upper plate
    barrel_wrap_depth = 0.008
    jaw.visual(
        Box((barrel_wrap_depth, _CLIP_WIDTH - 0.006, 0.008)),
        origin=Origin(xyz=(-barrel_wrap_depth / 2.0, 0.0, -0.003)),
        material=chrome,
        name="barrel_wrap",
    )

    # Upper spring plate — short, wide bulldog leaf (mirrors lower plate)
    upper_depth = abs(_HINGE_X - _MOUTH_X)  # 0.018
    upper_plate = _rounded_plate(
        upper_depth, _CLIP_WIDTH - 0.003, 0.0011,
        radius=0.0025,
        round_holes=((0.005, 0.0, 0.0032),),
    )
    upper_center_x = (_MOUTH_X + _HINGE_X) / 2.0 - _HINGE_X  # relative to jaw origin
    upper_center_z = _PLATE_THICK + 0.0015 - _HINGE_Z  # just above lower plate
    jaw.visual(
        mesh_from_cadquery(upper_plate, "upper_spring_plate", tolerance=0.0003),
        origin=Origin(xyz=(upper_center_x, 0.0, upper_center_z)),
        material=chrome,
        name="upper_spring_plate",
    )

    # Pressed rivet — decorative detail on upper plate
    jaw.visual(
        Cylinder(radius=0.0048, length=0.0012),
        origin=Origin(xyz=(upper_center_x + 0.005, 0.0,
                            upper_center_z + 0.0011 / 2.0 + 0.0006)),
        material=dark_chrome,
        name="pressed_rivet",
    )

    # Finger levers — two upturned sheet-metal tabs behind the hinge
    lever_base_len = 0.007
    lever_tab_len = 0.011
    lever_width = 0.010
    lever_thick = 0.0012
    lever_bend = 1.15  # ~66°
    lever_y_offsets = (-0.012, 0.012)
    # Lever base overlaps with the upper plate for connectivity
    lever_base_z = upper_center_z + 0.0011 / 2.0 - 0.0003

    for i, y_off in enumerate(lever_y_offsets):
        lever_geom = _bent_lever(
            lever_width, lever_base_len, lever_tab_len,
            lever_thick, lever_bend,
        )
        jaw.visual(
            mesh_from_cadquery(lever_geom, f"finger_lever_{i}", tolerance=0.0003),
            origin=Origin(xyz=(-0.001, y_off, lever_base_z)),
            material=chrome,
            name=f"finger_lever_{i}",
        )

    # Upper front teeth — serrated gripping bar at the mouth (on jaw)
    _upper_bar_depth = 0.005
    teeth_x_rel = _MOUTH_X + _upper_bar_depth / 2.0 - _HINGE_X
    teeth_z_rel = upper_center_z + 0.0003
    jaw.visual(
        mesh_from_cadquery(
            _tooth_bar(7, 0.0038, 0.0024, _upper_bar_depth, 0.0014),
            "upper_front_teeth",
        ),
        origin=Origin(xyz=(teeth_x_rel, 0.0, teeth_z_rel),
                       rpy=(math.pi, 0.0, 0.0)),
        material=dark_chrome,
        name="upper_front_teeth",
    )

    # ------------------------------------------------------------------
    # badge_connector  (perforated clear strap + swivel button)
    # ------------------------------------------------------------------
    connector = model.part("badge_connector")
    grid_holes = tuple(
        (x, y, 0.0016, 0.0028)
        for x in (0.012, 0.019, 0.026, 0.033)
        for y in (-0.006, 0.0, 0.006)
    )
    connector_panel = _rounded_plate(
        0.115,
        0.024,
        0.0012,
        radius=0.007,
        round_holes=((0.0, 0.0, 0.0045),),
        slots=((0.064, 0.0, 0.029, 0.0065),),
        rect_holes=grid_holes,
    )
    connector.visual(
        mesh_from_cadquery(connector_panel, "perforated_clear_strap", tolerance=0.00025),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=clear_vinyl,
        name="perforated_clear_strap",
    )
    connector.visual(
        mesh_from_cadquery(_ring(0.0076, 0.0046, 0.0013), "swivel_button_ring", tolerance=0.0002),
        origin=Origin(xyz=(0.0, 0.0, -0.00015)),
        material=chrome,
        name="swivel_button_ring",
    )
    connector.visual(
        Cylinder(radius=0.0048, length=0.0010),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=clear_vinyl,
        name="button_window",
    )

    # ------------------------------------------------------------------
    # Articulations
    # ------------------------------------------------------------------
    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(_HINGE_X, 0.0, _HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=0.45),
    )

    swivel_top_z = _PLATE_THICK + _SWIVEL_POST_H + 0.0009
    model.articulation(
        "body_to_connector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=connector,
        origin=Origin(xyz=(_SWIVEL_X, 0.0, swivel_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("clip_body")
    jaw = object_model.get_part("spring_jaw")
    connector = object_model.get_part("badge_connector")
    jaw_hinge = object_model.get_articulation("body_to_jaw")
    swivel = object_model.get_articulation("body_to_connector")

    # -- intentional overlaps -------------------------------------------
    ctx.allow_overlap(
        body,
        jaw,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The hinge pin is intentionally captured inside the moving barrel.",
    )
    ctx.allow_overlap(
        body,
        connector,
        elem_a="swivel_boss",
        elem_b="swivel_button_ring",
        reason="The swivel button ring is intentionally seated with a tiny crimped overlap on the boss.",
    )

    # -- hinge retention ------------------------------------------------
    ctx.expect_overlap(
        jaw,
        body,
        axes="y",
        elem_a="hinge_barrel",
        elem_b="hinge_pin",
        min_overlap=0.015,
        name="hinge barrel is retained on pin across clip width",
    )

    # -- swivel seating -------------------------------------------------
    ctx.expect_contact(
        connector,
        body,
        elem_a="swivel_button_ring",
        elem_b="swivel_boss",
        contact_tol=0.0010,
        name="swivel button is seated on the boss",
    )
    ctx.expect_gap(
        connector,
        body,
        axis="z",
        positive_elem="swivel_button_ring",
        negative_elem="swivel_boss",
        max_gap=0.0002,
        max_penetration=0.0004,
        name="swivel crimp has only local seated penetration",
    )
    ctx.expect_overlap(
        connector,
        body,
        axes="xy",
        elem_a="swivel_button_ring",
        elem_b="swivel_boss",
        min_overlap=0.006,
        name="swivel ring is centered over the rivet boss",
    )

    # -- bulldog form: finger levers are behind the hinge line ----------
    lever_0 = ctx.part_element_world_aabb(jaw, elem="finger_lever_0")
    lever_1 = ctx.part_element_world_aabb(jaw, elem="finger_lever_1")
    hinge_barrel = ctx.part_element_world_aabb(jaw, elem="hinge_barrel")
    ctx.check(
        "finger_lever tabs extend behind the hinge (bulldog clip form)",
        lever_0 is not None and lever_1 is not None and hinge_barrel is not None
        and lever_0[1][0] > hinge_barrel[0][0]  # lever_0 max-X past barrel min-X
        and lever_1[1][0] > hinge_barrel[0][0],
        details=f"lever0={lever_0}, lever1={lever_1}, barrel={hinge_barrel}",
    )

    # -- bulldog form: wide mouth (clip width > clip depth) -------------
    lower_plate_aabb = ctx.part_element_world_aabb(body, elem="lower_stamped_plate")
    ctx.check(
        "bulldog clip body is wider than deep",
        lower_plate_aabb is not None
        and (lower_plate_aabb[1][1] - lower_plate_aabb[0][1])
            > (lower_plate_aabb[1][0] - lower_plate_aabb[0][0]),
        details=f"lower_plate_aabb={lower_plate_aabb}",
    )

    # -- jaw hinge articulation -----------------------------------------
    rest_teeth = ctx.part_element_world_aabb(jaw, elem="upper_front_teeth")
    with ctx.pose({jaw_hinge: 0.35}):
        open_teeth = ctx.part_element_world_aabb(jaw, elem="upper_front_teeth")
    ctx.check(
        "positive hinge pose opens the jaw teeth",
        rest_teeth is not None
        and open_teeth is not None
        and open_teeth[1][2] > rest_teeth[1][2] + 0.004,
        details=f"rest={rest_teeth}, opened={open_teeth}",
    )

    # -- connector swivel -----------------------------------------------
    rest_panel = ctx.part_element_world_aabb(connector, elem="perforated_clear_strap")
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_panel = ctx.part_element_world_aabb(connector, elem="perforated_clear_strap")
    if rest_panel is not None and turned_panel is not None:
        rest_dx = rest_panel[1][0] - rest_panel[0][0]
        rest_dy = rest_panel[1][1] - rest_panel[0][1]
        turn_dx = turned_panel[1][0] - turned_panel[0][0]
        turn_dy = turned_panel[1][1] - turned_panel[0][1]
    else:
        rest_dx = rest_dy = turn_dx = turn_dy = 0.0
    ctx.check(
        "badge connector swivels ninety degrees about the button",
        rest_dx > rest_dy * 2.0 and turn_dy > turn_dx * 2.0,
        details=f"rest_dx={rest_dx}, rest_dy={rest_dy}, turn_dx={turn_dx}, turn_dy={turn_dy}",
    )

    return ctx.report()


object_model = build_object_model()
