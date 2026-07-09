from __future__ import annotations

# Walnut keepsake box with finger-joint (dovetail-style) corners,
# lighter accent splines, and a flat lid hinged at the rear top edge.
#
# Frame:
#   +X = box width (0.24 m), +Y = box depth (0.15 m), +Z = up (height 0.10 m).
#   Rear of the box is +Y; front (where the lid free edge sits) is -Y.
#   Everything is centered on x=0, y=0; the base sits on z=0 and grows up.
#
# Articulation:
#   - lid: REVOLUTE about the rear-top hinge line (axis = +X). The closed lid
#     geometry extends from the rear hinge toward -Y, so positive q lifts the
#     front edge up and swings it back over the rear (~0 to 100 degrees).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
)

# ---- Box dimensions ----
W = 0.240  # outer width (X)
D = 0.150  # outer depth (Y)
WALL = 0.012  # wood wall thickness
BASE_T = 0.012  # bottom panel thickness
BODY_H = 0.090  # outer height of the box base (without lid)
INNER_H = BODY_H - BASE_T  # interior cavity height

LID_T = 0.014  # lid slab thickness
LID_PANEL_INSET = 0.010  # recessed lighter inner panel inset

# Hinge sits at the rear top edge of the box base.
HINGE_Y = D / 2.0 - WALL / 2.0  # centered on the rear wall top
HINGE_Z = BODY_H  # at the top rim of the base

FINGER = 0.020  # finger-joint pitch (height of each interlocking tab)
SPLINE_W = 0.006  # accent spline width
SPLINE_T = 0.004  # accent spline protrusion / inset

# ---- Divider dimensions ----
# Two thin removable panels drop into the cavity, splitting it into three
# equal-width compartments along X.
IW = W - 2 * WALL  # inner cavity width (X)
ID = D - 2 * WALL  # inner cavity depth (Y)
DIV_T = 0.004  # divider panel thickness
DIV_D = ID - 0.004  # panel depth (slight clearance from walls)
DIV_H = INNER_H - 0.004  # panel height (slight clearance from floor to rim)
N_DIVIDERS = 2
COMPARTMENT_W = IW / (N_DIVIDERS + 1)  # equal compartment width
DIVIDER_XS = [
    -IW / 2.0 + (i + 1) * COMPARTMENT_W for i in range(N_DIVIDERS)
]  # [-0.036, 0.036]


def _box_base_solid() -> cq.Workplane:
    # Hollow rectangular shell: outer block minus the interior cavity, leaving
    # four walls and a solid floor.
    outer = (
        cq.Workplane("XY")
        .box(W, D, BODY_H, centered=(True, True, False))
    )
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BASE_T)
        .box(W - 2 * WALL, D - 2 * WALL, INNER_H + 0.01, centered=(True, True, False))
    )
    shell = outer.cut(cavity)

    # Carve finger-joint notches at the four vertical corners. At each corner the
    # two meeting walls interlock with alternating tabs; we cut small notches out
    # of one wall face at alternating heights so the corner reads as a box joint.
    notch_depth = WALL + 0.001
    half_w = W / 2.0
    half_d = D / 2.0
    n_fingers = max(2, int(INNER_H // FINGER))

    corners = [
        (half_w, half_d, 1, 1),
        (-half_w, half_d, -1, 1),
        (half_w, -half_d, 1, -1),
        (-half_w, -half_d, -1, -1),
    ]
    for cx, cy, sx, sy in corners:
        for i in range(n_fingers):
            z0 = BASE_T + i * FINGER
            if z0 + FINGER > BODY_H:
                break
            if i % 2 == 0:
                # Notch into the X-facing wall end (cut a slot on the side face).
                notch = (
                    cq.Workplane("XY")
                    .workplane(offset=z0)
                    .center(cx - sx * notch_depth / 2.0, cy - sy * (FINGER / 2.0))
                    .box(notch_depth, FINGER, FINGER * 0.5, centered=(True, True, False))
                )
                shell = shell.cut(notch)
    return shell


def _accent_splines_solid() -> cq.Workplane:
    # Lighter maple accent splines set diagonally across each top corner of the
    # box base, the classic reinforcing keys seen on fine boxes.
    splines = None
    half_w = W / 2.0
    half_d = D / 2.0
    corners = [
        (half_w - WALL / 2.0, half_d - WALL / 2.0),
        (-half_w + WALL / 2.0, half_d - WALL / 2.0),
        (half_w - WALL / 2.0, -half_d + WALL / 2.0),
        (-half_w + WALL / 2.0, -half_d + WALL / 2.0),
    ]
    for cx, cy in corners:
        for k in range(3):
            zc = BASE_T + 0.012 + k * 0.024
            if zc + SPLINE_W / 2.0 > BODY_H:
                break
            key = (
                cq.Workplane("XY")
                .workplane(offset=zc)
                .center(cx, cy)
                .rect(WALL * 1.6, SPLINE_W)
                .extrude(SPLINE_W)
                .rotate((cx, cy, zc), (cx, cy, zc + 1), 45.0)
            )
            splines = key if splines is None else splines.union(key)
    return splines


def _lid_solid() -> cq.Workplane:
    # Flat lid slab spanning the full footprint. Authored in a LOCAL frame whose
    # origin is the hinge line: the lid extends from y=0 (hinge) to y=-(D) and
    # is centered on x. Top of slab at z=0, bottom at z=-LID_T.
    slab = (
        cq.Workplane("XY")
        .center(0.0, -D / 2.0)
        .box(W, D, LID_T, centered=(True, True, False))
        .translate((0.0, 0.0, -LID_T))
    )
    return slab


def _lid_panel_solid() -> cq.Workplane:
    # Lighter recessed inner panel on the underside of the lid (visible when open).
    pw = W - 2 * WALL - 2 * LID_PANEL_INSET
    pd = D - 2 * WALL - 2 * LID_PANEL_INSET
    panel = (
        cq.Workplane("XY")
        .center(0.0, -D / 2.0)
        .box(pw, pd, 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, -LID_T - 0.004))
    )
    return panel


def _divider_solid() -> cq.Workplane:
    # Thin removable wooden divider panel. Authored in a LOCAL frame whose
    # origin is the bottom edge center of the panel, standing upright along +Z.
    # The panel spans Y (depth of cavity) and X (thickness), growing up in Z.
    panel = (
        cq.Workplane("XY")
        .center(0.0, 0.0)
        .box(DIV_T, DIV_D, DIV_H, centered=(True, True, False))
    )
    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="keepsake_box")

    walnut = model.material("walnut", rgba=(0.36, 0.22, 0.13, 1.0))
    maple = model.material("maple_accent", rgba=(0.82, 0.66, 0.42, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.58, 0.26, 1.0))

    # ---- box base (root) ----
    body = model.part("box_base")
    body.visual(
        mesh_from_cadquery(_box_base_solid(), "box_shell"),
        material=walnut,
        name="box_shell",
    )
    body.visual(
        mesh_from_cadquery(_accent_splines_solid(), "corner_splines"),
        material=maple,
        name="corner_splines",
    )

    # Two brass hinge knuckles on the rear top edge of the base.
    for i, kx in enumerate((-0.055, 0.055)):
        knuckle = (
            cq.Workplane("YZ")
            .workplane(offset=kx - 0.010)
            .center(HINGE_Y, HINGE_Z)
            .circle(0.006)
            .extrude(0.020)
        )
        body.visual(
            mesh_from_cadquery(knuckle, f"base_knuckle_{i}"),
            material=brass,
            name=f"base_knuckle_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Box((W, D, BODY_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- lid ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_slab"),
        material=walnut,
        name="lid_slab",
    )
    lid.visual(
        mesh_from_cadquery(_lid_panel_solid(), "lid_panel"),
        material=maple,
        name="lid_panel",
    )
    # Lid-side hinge knuckles, interleaved with the base knuckles on the hinge line.
    for i, kx in enumerate((-0.020, 0.085)):
        knuckle = (
            cq.Workplane("YZ")
            .workplane(offset=kx - 0.010)
            .center(0.0, 0.0)
            .circle(0.006)
            .extrude(0.020)
        )
        lid.visual(
            mesh_from_cadquery(knuckle, f"lid_knuckle_{i}"),
            material=brass,
            name=f"lid_knuckle_{i}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((W, D, LID_T)),
        mass=0.25,
        origin=Origin(xyz=(0.0, -D / 2.0, -LID_T / 2.0)),
    )

    # Hinge: axis along the rear top edge. Closed lid extends toward -Y; with
    # axis -X, positive q (right-hand rule) lifts the front edge up and swings
    # it rearward over the box.
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    # ---- removable compartment dividers ----
    # Two thin wooden panels that drop into the cavity and split it into three
    # equal-width compartments along the X axis. Each divider has a PRISMATIC
    # lift-out joint so it can be removed.
    cedar = model.material("cedar_divider", rgba=(0.62, 0.46, 0.30, 1.0))
    div_lift = model.articulation  # shorthand

    for i in range(N_DIVIDERS):
        divider = model.part(f"divider_{i}")
        divider.visual(
            mesh_from_cadquery(_divider_solid(), f"divider_panel_{i}"),
            material=cedar,
            name=f"divider_panel_{i}",
        )
        divider.inertial = Inertial.from_geometry(
            Box((DIV_T, DIV_D, DIV_H)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, DIV_H / 2.0)),
        )

        # PRISMATIC lift-out joint: origin at the cavity floor surface where the
        # divider seats. Axis +Z so positive q lifts the panel straight out.
        model.articulation(
            f"base_to_divider_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=divider,
            origin=Origin(xyz=(DIVIDER_XS[i], 0.0, BASE_T)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=0.5, lower=0.0, upper=0.10
            ),
        )

    return model


def run_tests() -> "TestReport":
    from sdk import TestContext

    ctx = TestContext(object_model)

    body = object_model.get_part("box_base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")

    # The lid and base hinge knuckles intentionally share the hinge axis line.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_knuckle_0",
        elem_b="base_knuckle_0",
        reason="Interleaved hinge knuckles share the rear hinge pin line.",
    )
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_knuckle_1",
        elem_b="base_knuckle_1",
        reason="Interleaved hinge knuckles share the rear hinge pin line.",
    )
    # The closed lid seats onto the box top rim (small intentional contact embed).
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_slab",
        elem_b="box_shell",
        reason="Closed lid slab seats flush onto the box top rim.",
    )
    # Lid hinge knuckles mount on the rear wall top, slightly embedding the shell.
    for i in (0, 1):
        ctx.allow_overlap(
            lid,
            body,
            elem_a=f"lid_knuckle_{i}",
            elem_b="box_shell",
            reason="Lid hinge knuckles are mounted onto the rear wall top edge.",
        )
    # The closed lid slab covers the rear hinge knuckles on the box.
    for i in (0, 1):
        ctx.allow_overlap(
            lid,
            body,
            elem_a="lid_slab",
            elem_b=f"base_knuckle_{i}",
            reason="Closed lid slab rests over the rear hinge knuckles.",
        )

    # 1) Box base is a hollow wooden rectangle: footprint spans full width/depth,
    #    and the interior cavity is open (lid covers the full footprint).
    base_aabb = ctx.part_world_aabb(body)
    bmn, bmx = base_aabb
    ctx.check(
        "box base spans full footprint",
        (bmx[0] - bmn[0]) > 0.22 and (bmx[1] - bmn[1]) > 0.13 and (bmx[2] - bmn[2]) > 0.08,
        details=f"base extents={(bmx[0]-bmn[0], bmx[1]-bmn[1], bmx[2]-bmn[2])}",
    )

    # 2) Closed lid covers the box footprint and sits at the top.
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.10, name="closed lid covers the box footprint"
    )
    lid_aabb = ctx.part_world_aabb(lid)
    lmn, lmx = lid_aabb
    ctx.check(
        "closed lid sits at the box top",
        lmx[2] >= bmx[2] - 0.002,
        details=f"lid_top_z={lmx[2]}, base_top_z={bmx[2]}",
    )

    # 3) Hinge is at the rear top edge.
    hz = HINGE_Z
    ctx.check(
        "hinge at rear top edge",
        HINGE_Y > 0.0 and abs(hz - bmx[2]) < 0.01,
        details=f"hinge_y={HINGE_Y}, hinge_z={hz}, base_top={bmx[2]}",
    )

    # 4) Opening the lid: the FRONT edge (-Y, min y) lifts up and swings back.
    front_y_closed = lmn[1]
    top_z_closed = lmx[2]
    with ctx.pose({hinge: math.radians(100.0)}):
        op_mn, op_mx = ctx.part_world_aabb(lid)
        front_y_open = op_mn[1]
        top_z_open = op_mx[2]
    ctx.check(
        "lid front edge lifts when opened",
        top_z_open > top_z_closed + 0.05,
        details=f"top_z closed={top_z_closed}, open={top_z_open}",
    )
    ctx.check(
        "lid front edge swings rearward when opened",
        front_y_open > front_y_closed + 0.02,
        details=f"front_y closed={front_y_closed}, open={front_y_open}",
    )

    # 5) Dividers: two removable vertical panels inside the cavity.
    dividers = [object_model.get_part(f"divider_{i}") for i in range(N_DIVIDERS)]
    div_joints = [object_model.get_articulation(f"base_to_divider_{i}") for i in range(N_DIVIDERS)]

    # Dividers sit inside the cavity, so they intentionally overlap with the box base.
    for i in range(N_DIVIDERS):
        ctx.allow_overlap(
            dividers[i],
            body,
            elem_a=f"divider_panel_{i}",
            elem_b="box_shell",
            reason=f"Divider {i} drops into the cavity and rests on the floor.",
        )

    # Each divider is within the cavity footprint (XY).
    for i in range(N_DIVIDERS):
        ctx.expect_within(
            dividers[i],
            body,
            axes="xy",
            margin=0.005,
            name=f"divider_{i} stays within cavity footprint",
        )

    # Each divider sits within the cavity height (above floor, below rim).
    for i in range(N_DIVIDERS):
        div_mn, div_mx = ctx.part_world_aabb(dividers[i])
        ctx.check(
            f"divider_{i} seated at cavity floor",
            abs(div_mn[2] - BASE_T) < 0.005,
            details=f"divider_bottom_z={div_mn[2]}, cavity_floor_z={BASE_T}",
        )

    # Lift-out motion: positive q raises each divider upward.
    for i in range(N_DIVIDERS):
        rest_pos = ctx.part_world_position(dividers[i])
        with ctx.pose({div_joints[i]: 0.08}):
            lifted_pos = ctx.part_world_position(dividers[i])
        ctx.check(
            f"divider_{i} lifts out when joint is positive",
            rest_pos is not None and lifted_pos is not None and lifted_pos[2] > rest_pos[2] + 0.07,
            details=f"rest_z={rest_pos}, lifted_z={lifted_pos}",
        )

    return ctx.report()


object_model = build_object_model()
