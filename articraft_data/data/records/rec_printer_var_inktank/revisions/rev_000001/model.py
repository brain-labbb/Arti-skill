from __future__ import annotations

# White Brother supertank all-in-one inkjet printer.
# Frame: width along X, depth along Y (front = -Y, rear = +Y), height along Z
#   (body bottom near z = 0). Body ~0.40 (X) x 0.36 (Y) x 0.17 (Z) m.
# Supertank variant: external ink-tank reservoir housing bulges from the right (+X)
#   side of the body with translucent CMYK ink-level windows and fill caps.
# Articulations:
#   - scanner_lid: REVOLUTE about the rear hinge line; front edge lifts up (0..70 deg).
#   - output_tray: PRISMATIC pull-out toward the front (-Y), ~0.12 m travel.
#   - paper_stopper: REVOLUTE flip-up on the front of the tray (0..80 deg).
#   - five control-panel buttons: PRISMATIC press down (-Z), ~1.5 mm.

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

# ---- key dimensions ----
BODY_W = 0.400  # X
BODY_D = 0.360  # Y
BODY_H = 0.150  # Z (main box; lid adds a bit on top)
BODY_Z0 = 0.0
BODY_ZTOP = BODY_Z0 + BODY_H

FRONT_Y = -BODY_D / 2.0  # -0.180
REAR_Y = BODY_D / 2.0  # +0.180

LID_H = 0.022  # scanner lid thickness
LID_HINGE_Y = REAR_Y - 0.010  # hinge a touch inboard of the rear edge
LID_TOP_Z = BODY_ZTOP + LID_H

# Control panel band on the upper front face.
PANEL_W = 0.300
PANEL_H = 0.062  # vertical extent of the panel band
PANEL_Y = FRONT_Y + 0.006  # panel face slightly proud of recess
PANEL_Z = BODY_ZTOP - 0.040  # center of the panel band

# Output slot / tray.
SLOT_W = 0.260
SLOT_Z = 0.060  # center height of the output slot
TRAY_W = 0.250
TRAY_T = 0.014  # tray plate thickness
TRAY_LEN = 0.150  # tray depth (Y), much hides inside the slot when retracted
TRAY_TRAVEL = 0.120

# Ink-tank reservoir (supertank variant). Sits on the right (+X) side of the body.
TANK_W = 0.065          # protrusion from body right wall
TANK_D = 0.160          # depth (Y) — narrower than body
TANK_H = 0.110          # height (Z)
TANK_Z0 = 0.010         # sits slightly above body bottom
TANK_X_CENTER = BODY_W / 2.0 + TANK_W / 2.0 - 0.005  # overlaps body wall by 5mm
TANK_Z_CENTER = TANK_Z0 + TANK_H / 2.0


def _fillet_box(w: float, d: float, h: float, r: float, cz: float) -> cq.Workplane:
    # Axis-aligned rounded box centered at (0,0,cz), filleted on the vertical edges.
    b = (
        cq.Workplane("XY")
        .box(w, d, h, centered=(True, True, True))
        .edges("|Z")
        .fillet(r)
    )
    return b.translate((0.0, 0.0, cz))


def _body_solid() -> cq.Workplane:
    # Main printer body: a tall lower box plus a slightly inset upper deck so the
    # scanner area reads as a separate stage; front gets a recessed control area.
    cz = BODY_Z0 + BODY_H / 2.0
    lower = _fillet_box(BODY_W, BODY_D, BODY_H, 0.018, cz)

    # Recess the upper-front: carve a shallow notch where the control panel + slot live.
    # Output slot: rectangular cut through the front face.
    slot = (
        cq.Workplane("XY")
        .box(SLOT_W, 0.090, 0.052, centered=(True, True, True))
        .translate((0.0, FRONT_Y + 0.030, SLOT_Z))
    )
    body = lower.cut(slot)

    # Panel recess: a shallow pocket on the upper front face so the panel sits inset.
    recess = (
        cq.Workplane("XY")
        .box(PANEL_W + 0.004, 0.022, PANEL_H + 0.004, centered=(True, True, True))
        .translate((0.0, FRONT_Y + 0.010, PANEL_Z))
    )
    body = body.cut(recess)
    return body


def _tank_housing_solid() -> cq.Workplane:
    """External ink-tank reservoir housing that bulges from the right side of the body.
    Centered at local origin; caller places it at (TANK_X_CENTER, 0, TANK_Z_CENTER)."""
    housing = (
        cq.Workplane("XY")
        .box(TANK_W, TANK_D, TANK_H, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.008)
        .edges("|X")
        .fillet(0.004)
    )
    return housing


def _tank_window_solid() -> cq.Workplane:
    """A single translucent tank window panel (thin slab inset into the housing front face)."""
    win_w = (TANK_W - 0.020) / 4.0 - 0.004  # 4 windows side by side with gaps
    win_h = TANK_H - 0.030
    win_d = 0.004  # thin in Y (depth direction)
    return cq.Workplane("XY").box(win_w, win_d, win_h, centered=(True, True, True))


def _scanner_lid_solid() -> cq.Workplane:
    # Flat scanner lid (off-white frame) sized to cover the top deck. Authored so
    # the hinge line at the rear is at the part origin; geometry extends toward -Y.
    # Bottom of the lid frame dips a hair below the body top so it seats on the
    # body (contact for support); the hinge line is at the part origin (y=0).
    # LOCAL frame: hinge at origin (z=0 == body top). Lid bottom dips slightly
    # below z=0 so it seats on the body for support.
    cz = LID_H / 2.0 - 0.003
    span = BODY_D - 0.014  # front edge sits just inside the body front
    cy = -span / 2.0  # extend from hinge (y=0) toward the front (-Y)
    lid = (
        cq.Workplane("XY")
        .box(BODY_W - 0.014, span, LID_H, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.012)
    )
    return lid.translate((0.0, cy, cz))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brother_inkjet_printer")

    off_white = model.material("off_white", rgba=(0.91, 0.90, 0.87, 1.0))
    lid_gray = model.material("lid_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.52, 0.53, 0.55, 1.0))
    dark = model.material("display_dark", rgba=(0.12, 0.12, 0.14, 1.0))
    btn_gray = model.material("button_gray", rgba=(0.40, 0.40, 0.43, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.78, 0.78, 0.77, 1.0))
    # Supertank ink materials
    tank_body = model.material("tank_body", rgba=(0.18, 0.18, 0.20, 1.0))
    ink_black = model.material("ink_black", rgba=(0.08, 0.08, 0.10, 0.55))
    ink_cyan = model.material("ink_cyan", rgba=(0.05, 0.75, 0.90, 0.55))
    ink_magenta = model.material("ink_magenta", rgba=(0.85, 0.10, 0.55, 0.55))
    ink_yellow = model.material("ink_yellow", rgba=(0.95, 0.85, 0.05, 0.55))
    cap_dark = model.material("cap_dark", rgba=(0.10, 0.10, 0.12, 1.0))

    # ---------------- body (root) ----------------
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=off_white,
        name="body_shell",
    )

    # "brother" branding plate on the upper-left front.
    body.visual(
        Box((0.060, 0.004, 0.012)),
        origin=Origin(xyz=(-0.150, FRONT_Y + 0.001, BODY_ZTOP - 0.018)),
        material=lid_gray,
        name="brand_plate",
    )

    # ---- Ink-tank reservoir housing (supertank variant) ----
    # External tank block bulging from the right (+X) side of the body.
    body.visual(
        mesh_from_cadquery(_tank_housing_solid(), "tank_housing"),
        origin=Origin(xyz=(TANK_X_CENTER, 0.0, TANK_Z_CENTER)),
        material=tank_body,
        name="tank_housing",
    )

    # Translucent ink-level windows on the front face of the tank housing.
    # Each window is a thin panel showing the ink color level inside.
    ink_mats = [ink_black, ink_cyan, ink_magenta, ink_yellow]
    win_h = TANK_H - 0.030
    win_w_each = (TANK_W - 0.020) / 4.0 - 0.004
    tank_front_y = -TANK_D / 2.0 + 0.002  # just proud of the housing front face
    for i, mat in enumerate(ink_mats):
        # Distribute 4 windows across the tank width (along X)
        wx = TANK_X_CENTER - TANK_W / 2.0 + 0.012 + (win_w_each + 0.004) * i + win_w_each / 2.0
        body.visual(
            mesh_from_cadquery(_tank_window_solid(), f"tank_window_{i}"),
            origin=Origin(xyz=(wx, tank_front_y, TANK_Z_CENTER)),
            material=mat,
            name=f"tank_window_{i}",
        )

    # Fill caps on top of the tank housing — 4 round caps for CMYK inks.
    cap_r = 0.008
    cap_t = 0.006
    cap_z = TANK_Z0 + TANK_H + cap_t / 2.0
    cap_xs = [TANK_X_CENTER - 0.018 + 0.012 * k for k in range(4)]
    for i, cx in enumerate(cap_xs):
        cyl = Cylinder(radius=cap_r, length=cap_t)
        body.visual(
            cyl,
            origin=Origin(xyz=(cx, 0.0, cap_z)),
            material=cap_dark,
            name=f"tank_cap_{i}",
        )

    # Level-window label strip — thin dark label band across the top of the windows.
    body.visual(
        Box((TANK_W - 0.014, 0.004, 0.008)),
        origin=Origin(xyz=(TANK_X_CENTER, tank_front_y, TANK_Z_CENTER + win_h / 2.0 + 0.006)),
        material=cap_dark,
        name="tank_label_strip",
    )

    # Control panel face (recessed band) holding display + icons. Thick enough to
    # bridge from the recess back wall (y=-0.159) out to a face proud at PANEL_Y,
    # so the panel and its controls are physically supported by the body.
    PANEL_BACK_Y = FRONT_Y + 0.021  # = recess back wall
    panel_thk = PANEL_BACK_Y - (PANEL_Y - 0.003)
    body.visual(
        Box((PANEL_W, panel_thk, PANEL_H)),
        origin=Origin(xyz=(0.0, (PANEL_BACK_Y + (PANEL_Y - 0.003)) / 2.0, PANEL_Z)),
        material=panel_gray,
        name="control_panel",
    )
    # Central rectangular display.
    body.visual(
        Box((0.060, 0.004, 0.040)),
        origin=Origin(xyz=(0.010, PANEL_Y + 0.003, PANEL_Z)),
        material=dark,
        name="display",
    )
    # A couple of printed icons to the right of the display.
    for i, ix in enumerate((0.058, 0.082)):
        body.visual(
            Box((0.014, 0.003, 0.014)),
            origin=Origin(xyz=(ix, PANEL_Y + 0.003, PANEL_Z)),
            material=dark,
            name=f"panel_icon_{i}",
        )
    # Slider / extra control on the far right of the panel.
    body.visual(
        Box((0.010, 0.005, 0.024)),
        origin=Origin(xyz=(0.118, PANEL_Y + 0.003, PANEL_Z)),
        material=btn_gray,
        name="panel_slider",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0 + BODY_H / 2.0)),
    )

    # ---------------- scanner lid (revolute, rear hinge) ----------------
    lid = model.part("scanner_lid")
    # Off-white lid frame.
    lid.visual(
        mesh_from_cadquery(_scanner_lid_solid(), "lid_frame"),
        material=off_white,
        name="lid_frame",
    )
    # Gray document-cover top inset onto the lid (LOCAL frame; z=0 == body top).
    span = BODY_D - 0.014
    lid.visual(
        Box((BODY_W - 0.040, span - 0.026, 0.004)),
        origin=Origin(xyz=(0.0, -span / 2.0, LID_H - 0.003)),
        material=lid_gray,
        name="cover_top",
    )
    lid.inertial = Inertial.from_geometry(
        Box((BODY_W - 0.014, span, LID_H + 0.005)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -span / 2.0, LID_H / 2.0)),
    )
    # Hinge at the rear, axis along X. Lid extends toward -Y; with axis -X the
    # right-hand rule lifts the front (-Y) edge upward (+Z) for positive q.
    model.articulation(
        "body_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, BODY_ZTOP)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(70.0)
        ),
    )

    # ---------------- output tray (prismatic, pulls out the front) ----------------
    tray = model.part("output_tray")
    # Tray plate. Authored so most of its length is hidden inside the slot at rest;
    # the joint origin sits at the slot lip and travel moves it out along -Y.
    tray.visual(
        Box((TRAY_W, TRAY_LEN, TRAY_T)),
        origin=Origin(xyz=(0.0, TRAY_LEN / 2.0, 0.0)),
        material=tray_gray,
        name="tray_plate",
    )
    # Low side rails to read as a real catch tray.
    for sx in (-(TRAY_W / 2.0) + 0.006, (TRAY_W / 2.0) - 0.006):
        tray.visual(
            Box((0.008, TRAY_LEN, 0.010)),
            origin=Origin(xyz=(sx, TRAY_LEN / 2.0, 0.007)),
            material=tray_gray,
            name=f"tray_rail_{'l' if sx < 0 else 'r'}",
        )
    tray.inertial = Inertial.from_geometry(
        Box((TRAY_W, TRAY_LEN, TRAY_T + 0.010)),
        mass=0.20,
        origin=Origin(xyz=(0.0, TRAY_LEN / 2.0, 0.0)),
    )
    # Joint origin at the slot lip; +(-Y) axis -> tray slides forward out of the slot.
    # Child frame at q=0 sits at the lip; tray geometry (centered toward +Y of its
    # own origin) is thus mostly retained inside the body slot when collapsed.
    SLOT_LIP_Y = FRONT_Y + 0.012
    model.articulation(
        "body_to_output_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, SLOT_LIP_Y, SLOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.20, lower=0.0, upper=TRAY_TRAVEL
        ),
    )

    # ---------------- paper stopper (revolute, flips up on tray front) ----------------
    stopper = model.part("paper_stopper")
    STOP_W = TRAY_W - 0.020
    STOP_LEN = 0.045
    # Authored with hinge at part origin; flap extends forward toward -Y and lies
    # flat (in the tray plane) at q=0. The tray's exposed front edge is its local
    # y=0 end (the joint lip), so the stopper hinges there.
    stopper.visual(
        Box((STOP_W, STOP_LEN, 0.006)),
        origin=Origin(xyz=(0.0, -STOP_LEN / 2.0, 0.0)),
        material=tray_gray,
        name="stopper_flap",
    )
    stopper.inertial = Inertial.from_geometry(
        Box((STOP_W, STOP_LEN, 0.006)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -STOP_LEN / 2.0, 0.0)),
    )
    # Hinge at the tray's exposed front edge (tray-local y=0). Flap extends toward
    # -Y; axis -X lifts the free (-Y) edge upward (+Z) for positive q.
    model.articulation(
        "tray_to_paper_stopper",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=stopper,
        origin=Origin(xyz=(0.0, 0.0, TRAY_T / 2.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=math.radians(80.0)
        ),
    )

    # ---------------- control-panel buttons (prismatic press-down) ----------------
    # Four round buttons on the left of the panel, plus the right slider-button.
    BTN_R = 0.0095
    BTN_T = 0.010
    # Panel front face is at PANEL_Y - 0.003. Seat each button so it pokes proud of
    # that face while its base stays embedded in the panel (a real button bore).
    PANEL_FACE_Y = PANEL_Y - 0.003
    btn_top_z = PANEL_Z  # buttons centered on the panel band
    btn_y = PANEL_FACE_Y - 0.002  # cap center just in front of the panel face
    btn_xs = (-0.118, -0.094, -0.070, -0.046)
    for i, bx in enumerate(btn_xs):
        btn = model.part(f"button_{i}")
        # Cylinder axis along Y (faces the user); presses inward toward +Y.
        cyl = Cylinder(radius=BTN_R, length=BTN_T)
        btn.visual(
            cyl,
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=btn_gray,
            name=f"button_{i}_cap",
        )
        btn.inertial = Inertial.from_geometry(
            Box((2 * BTN_R, BTN_T, 2 * BTN_R)), mass=0.004
        )
        # Press direction: into the panel = +Y (toward the body interior).
        model.articulation(
            f"body_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            origin=Origin(xyz=(bx, btn_y, btn_top_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=0.05, lower=0.0, upper=0.0015
            ),
        )

    # Fifth button: the right-side slider/extra control presses down (-Z).
    btn5 = model.part("button_4")
    btn5.visual(
        Box((0.012, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=btn_gray,
        name="button_4_cap",
    )
    btn5.inertial = Inertial.from_geometry(Box((0.012, 0.008, 0.010)), mass=0.004)
    model.articulation(
        "body_to_button_4",
        ArticulationType.PRISMATIC,
        parent=body,
        child=btn5,
        origin=Origin(xyz=(0.140, btn_y, PANEL_Z + 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.0015),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("scanner_lid")
    tray = object_model.get_part("output_tray")
    stopper = object_model.get_part("paper_stopper")
    btn0 = object_model.get_part("button_0")

    lid_hinge = object_model.get_articulation("body_to_scanner_lid")
    tray_joint = object_model.get_articulation("body_to_output_tray")
    stopper_hinge = object_model.get_articulation("tray_to_paper_stopper")
    btn0_joint = object_model.get_articulation("body_to_button_0")

    # ---- lid covers the top and is seated on the body at rest ----
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_frame",
        elem_b="body_shell",
        reason="Scanner lid rests flush on the top deck; the hinge edge seats on the body.",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.20, name="lid covers the top deck"
    )
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits above the body top",
        lid_pos is not None and lid_pos[2] >= BODY_ZTOP - 1e-4,
        details=f"lid origin={lid_pos}",
    )

    # ---- lid opens: front edge lifts up about the rear hinge ----
    front_z_rest = ctx.part_world_aabb(lid)[0][2]  # min-Z corner ~ front edge at rest
    rest_top = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_hinge: math.radians(70.0)}):
        open_top = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "scanner lid opens upward about the rear hinge",
        open_top > rest_top + 0.10,
        details=f"rest_top={rest_top}, open_top={open_top}",
    )

    # ---- control panel is on the front face ----
    panel_aabb = ctx.part_element_world_aabb(body, elem="control_panel")
    ctx.check(
        "control panel on the front of the body",
        panel_aabb is not None and panel_aabb[0][1] < FRONT_Y + 0.020,
        details=f"panel aabb={panel_aabb}",
    )

    # ---- output tray seated in the slot, slides out the front (-Y) ----
    ctx.allow_overlap(
        tray,
        body,
        reason="The tray (plate and side rails) is retained inside the body output slot when collapsed.",
    )
    ctx.expect_within(
        tray,
        body,
        axes="x",
        inner_elem="tray_plate",
        outer_elem="body_shell",
        margin=0.005,
        name="tray stays within body width",
    )
    rest_y = ctx.part_world_position(tray)[1]
    with ctx.pose({tray_joint: TRAY_TRAVEL}):
        out_y = ctx.part_world_position(tray)[1]
    ctx.check(
        "output tray slides out the front",
        out_y < rest_y - 0.10,
        details=f"rest_y={rest_y}, out_y={out_y}",
    )

    # ---- paper stopper flips up off the tray front ----
    stop_rest_top = ctx.part_world_aabb(stopper)[1][2]
    with ctx.pose({stopper_hinge: math.radians(80.0)}):
        stop_open_top = ctx.part_world_aabb(stopper)[1][2]
    ctx.check(
        "paper stopper flips up",
        stop_open_top > stop_rest_top + 0.025,
        details=f"rest_top={stop_rest_top}, open_top={stop_open_top}",
    )

    # ---- a sampled control button presses inward (into the panel) ----
    for i in range(5):
        b = object_model.get_part(f"button_{i}")
        ctx.allow_overlap(
            b,
            body,
            elem_a=f"button_{i}_cap",
            elem_b="control_panel",
            reason="Button cap base is seated in its panel bore and presses inward.",
        )
    btn_rest_y = ctx.part_world_position(btn0)[1]
    with ctx.pose({btn0_joint: 0.0015}):
        btn_press_y = ctx.part_world_position(btn0)[1]
    ctx.check(
        "control button presses inward",
        btn_press_y > btn_rest_y + 0.0009,
        details=f"rest_y={btn_rest_y}, press_y={btn_press_y}",
    )

    # ---- ink-tank reservoir housing bulges from the right side (+X) of the body ----
    tank_aabb = ctx.part_element_world_aabb(body, elem="tank_housing")
    body_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "tank_housing exists on body",
        tank_aabb is not None,
        details="tank_housing visual not found",
    )
    if tank_aabb is not None and body_aabb is not None:
        # Tank must extend beyond the body's right (+X) edge
        ctx.check(
            "tank housing protrudes beyond body right side (+X)",
            tank_aabb[1][0] > body_aabb[1][0] - 0.005,
            details=f"tank_max_x={tank_aabb[1][0]:.4f}, body_max_x={body_aabb[1][0]:.4f}",
        )
    # Verify translucent ink windows exist alongside the tank
    for i in range(4):
        win_aabb = ctx.part_element_world_aabb(body, elem=f"tank_window_{i}")
        ctx.check(
            f"tank_window_{i} visible on tank housing",
            win_aabb is not None,
            details=f"tank_window_{i} visual not found",
        )

    return ctx.report()


object_model = build_object_model()
