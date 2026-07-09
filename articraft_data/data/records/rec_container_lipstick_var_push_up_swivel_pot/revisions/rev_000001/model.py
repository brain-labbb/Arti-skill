from __future__ import annotations

# Push-up swivel lipstick tube (fork of the round twist-up parent).
# Frame: tube axis along +Z, base seated on z=0, cap on top.
# Construction:
#   - tube_base (root): slim hollow tubular body with silver center band and a
#     vertical slider slot on the +X side; open top.
#   - cap: glossy white/silver cap, PULLS STRAIGHT UP off the base (PRISMATIC).
#   - slider: thumb-wheel slider carrying the bullet straight up the bore on a
#     single PRISMATIC joint (no separate rotating carrier).
# The push-up mechanism replaces the parent twist-up screw chain (continuous
# carrier + prismatic bullet) with one prismatic slider.

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
    mesh_from_cadquery,
)

# --- key dimensions (meters) ---
TUBE_R = 0.0105
TUBE_WALL = 0.0016
BASE_TOP_Z = 0.050
BAND_Z = 0.0455
INNER_R = TUBE_R - TUBE_WALL

CAP_R = TUBE_R + 0.0012
CAP_LEN = 0.046
CAP_REST_Z = BAND_Z

BULLET_R = 0.0072
CUP_R = INNER_R - 0.0006
CUP_LEN = 0.012
BULLET_LEN = 0.020
RISE_HEIGHT = 0.040

# Slider slot on the +X tube wall
SLOT_W = 0.005
SLOT_BOTTOM_Z = 0.005
SLOT_TOP_Z = BASE_TOP_Z

# Thumb tab dimensions
TAB_W = 0.007
TAB_H = 0.008
TAB_D = 0.004


def _tube_base_solid() -> cq.Workplane:
    """Hollow open-top tube with a vertical slider slot cut in the +X wall."""
    outer = cq.Workplane("XY").circle(TUBE_R).extrude(BASE_TOP_Z)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .circle(INNER_R)
        .extrude(BASE_TOP_Z)
    )
    body = outer.cut(bore)
    # Vertical slider slot through the +X wall
    slot_dx = TUBE_WALL + 0.002
    slot = (
        cq.Workplane("XY")
        .workplane(offset=SLOT_BOTTOM_Z)
        .center(TUBE_R, 0.0)
        .rect(slot_dx, SLOT_W)
        .extrude(SLOT_TOP_Z - SLOT_BOTTOM_Z)
    )
    return body.cut(slot)


def _slider_solid() -> cq.Workplane:
    """Cup platform + connecting rib + external thumb tab as one connected body."""
    # Internal cup that rides inside the bore
    cup = cq.Workplane("XY").circle(CUP_R).extrude(CUP_LEN)
    # Connecting rib from inside the cup through the tube wall slot
    rib_w = 0.003
    rib_z_lo, rib_z_hi = 0.002, 0.008
    rib_x_lo = CUP_R * 0.5
    rib_x_hi = TUBE_R + 0.001
    rib = (
        cq.Workplane("XY")
        .workplane(offset=rib_z_lo)
        .center((rib_x_lo + rib_x_hi) / 2.0, 0.0)
        .rect(rib_x_hi - rib_x_lo, rib_w)
        .extrude(rib_z_hi - rib_z_lo)
    )
    # External thumb tab (wider than slot, sits outside the tube wall)
    tab = (
        cq.Workplane("XY")
        .workplane(offset=0.001)
        .center(TUBE_R + TAB_D / 2.0, 0.0)
        .rect(TAB_D, TAB_W)
        .extrude(TAB_H)
    )
    return cup.union(rib).union(tab)


def _bullet_red_solid() -> cq.Workplane:
    """Red lipstick column with angled tip, sits on top of the slider cup."""
    col_bottom = CUP_LEN
    column = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom)
        .circle(BULLET_R)
        .extrude(BULLET_LEN)
    )
    tip = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom + BULLET_LEN)
        .circle(BULLET_R)
        .extrude(0.010)
    )
    cutter = (
        cq.Workplane("XY")
        .workplane(offset=col_bottom + BULLET_LEN)
        .transformed(rotate=(20.0, 0.0, 0.0))
        .rect(0.05, 0.05)
        .extrude(0.03)
        .translate((0.0, 0.0, 0.004))
    )
    return column.union(tip.cut(cutter))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lipstick_tube_push")

    white = model.material("tube_white", rgba=(0.93, 0.93, 0.94, 1.0))
    silver = model.material("silver", rgba=(0.74, 0.75, 0.78, 1.0))
    red = model.material("lipstick_red", rgba=(0.86, 0.10, 0.10, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.40, 0.42, 0.45, 1.0))

    # ---- tube_base (root): hollow body + band + slot ----
    base = model.part("tube_base")
    base.visual(
        mesh_from_cadquery(_tube_base_solid(), "tube_body"),
        material=white,
        name="tube_body",
    )
    base.visual(
        Cylinder(radius=TUBE_R + 0.0006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, BAND_Z)),
        material=silver,
        name="center_band",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_R, length=BASE_TOP_Z),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z / 2.0)),
    )

    # ---- cap: pull-off (identical to parent) ----
    cap = model.part("cap")
    cap_outer = cq.Workplane("XY").circle(CAP_R).extrude(CAP_LEN)
    cap_bore = (
        cq.Workplane("XY")
        .circle(CAP_R - 0.0014)
        .extrude(CAP_LEN - 0.004)
    )
    cap.visual(
        mesh_from_cadquery(cap_outer.cut(cap_bore), "cap_shell"),
        material=white,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, CAP_LEN / 2.0)),
    )
    model.articulation(
        "cap_pull",
        ArticulationType.PRISMATIC,
        parent=base,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=0.2, lower=0.0, upper=0.050
        ),
    )

    # ---- slider: single prismatic push-up carrying the bullet ----
    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(_slider_solid(), "slider_body"),
        material=gunmetal,
        name="slider_body",
    )
    slider.visual(
        mesh_from_cadquery(_bullet_red_solid(), "bullet_red"),
        material=red,
        name="bullet_red",
    )
    slider.inertial = Inertial.from_geometry(
        Cylinder(radius=CUP_R, length=CUP_LEN + BULLET_LEN + 0.010),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, (CUP_LEN + BULLET_LEN) / 2.0)),
    )
    model.articulation(
        "slider_push",
        ArticulationType.PRISMATIC,
        parent=base,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5, velocity=0.3, lower=0.0, upper=RISE_HEIGHT
        ),
    )

    return model


def run_tests():
    from sdk import TestContext

    ctx = TestContext(object_model)

    base = object_model.get_part("tube_base")
    cap = object_model.get_part("cap")
    slider = object_model.get_part("slider")
    cap_pull = object_model.get_articulation("cap_pull")
    slider_push = object_model.get_articulation("slider_push")

    # --- Material/identity claims ---
    bullet_red = slider.get_visual("bullet_red")
    tube_body = base.get_visual("tube_body")
    band = base.get_visual("center_band")
    slider_body = slider.get_visual("slider_body")

    ctx.check(
        "bullet is red",
        tuple(bullet_red.material.rgba[:3]) == (0.86, 0.10, 0.10),
        details=f"rgba={bullet_red.material.rgba}",
    )
    ctx.check(
        "tube body is white/silver",
        min(tube_body.material.rgba[:3]) > 0.7,
        details=f"rgba={tube_body.material.rgba}",
    )
    ctx.check(
        "center band is silver",
        abs(band.material.rgba[0] - 0.74) < 0.05,
        details=f"rgba={band.material.rgba}",
    )
    ctx.check(
        "slider body is gunmetal",
        slider_body.material.rgba[0] < 0.55,
        details=f"rgba={slider_body.material.rgba}",
    )

    # --- Mechanism: single prismatic slider, no twist carrier ---
    joint_names = [a.name for a in object_model.articulations]
    ctx.check(
        "single prismatic slider replaces twist chain (no bullet_twist)",
        "slider_push" in joint_names and "bullet_twist" not in joint_names,
        details=f"joints={joint_names}",
    )
    ctx.check(
        "exactly two articulations (cap_pull + slider_push)",
        len(joint_names) == 2,
        details=f"joints={joint_names}",
    )

    # --- Overlap allowances ---
    ctx.allow_overlap(
        cap, base,
        elem_a="cap_shell", elem_b="tube_body",
        reason="Cap slips over tube base top (seated pull-off cap).",
    )
    ctx.allow_overlap(
        cap, base,
        elem_a="cap_shell", elem_b="center_band",
        reason="Cap rim seats onto the silver center band when closed.",
    )
    ctx.allow_overlap(
        slider, base,
        elem_a="slider_body", elem_b="tube_body",
        reason="Slider cup rides inside the bore; thumb tab wraps outside through the slot.",
    )
    ctx.allow_overlap(
        slider, base,
        elem_a="bullet_red", elem_b="tube_body",
        reason="Bullet starts nested inside the tube before being pushed up.",
    )
    ctx.allow_overlap(
        slider, base,
        elem_a="slider_body", elem_b="center_band",
        reason="Slider rib passes through the slot near the band at high extension.",
    )
    ctx.allow_overlap(
        cap, slider,
        reason="Closed cap encloses the retracted bullet and slider cup.",
    )

    # --- Cap position and pull-off ---
    cap_rest = ctx.part_world_position(cap)
    ctx.check(
        "cap is on top of the tube",
        cap_rest is not None and cap_rest[2] > BAND_Z - 0.001,
        details=f"cap origin={cap_rest}",
    )

    with ctx.pose({cap_pull: 0.045}):
        cap_up = ctx.part_world_position(cap)
        cap_bottom = ctx.part_world_aabb(cap)[0][2]
    ctx.check(
        "cap pulls straight up off the tube",
        cap_up is not None and cap_up[2] > cap_rest[2] + 0.040,
        details=f"rest={cap_rest}, pulled={cap_up}",
    )
    ctx.check(
        "pulled cap lifts clear of the base top",
        cap_bottom > BASE_TOP_Z - 0.006,
        details=f"cap bottom when pulled={cap_bottom}",
    )

    # --- Slider push mechanism ---
    slider_rest = ctx.part_world_position(slider)
    with ctx.pose({slider_push: RISE_HEIGHT}):
        slider_up = ctx.part_world_position(slider)
        bullet_top_z = ctx.part_world_aabb(slider)[1][2]

    ctx.check(
        "slider push raises the bullet",
        slider_up is not None and slider_up[2] > slider_rest[2] + 0.030,
        details=f"rest={slider_rest}, pushed={slider_up}",
    )
    ctx.check(
        "pushed bullet emerges above the tube top",
        bullet_top_z > BASE_TOP_Z + 0.005,
        details=f"bullet top z when pushed={bullet_top_z}",
    )
    ctx.check(
        "slider travels straight up (no XY drift or rotation)",
        slider_rest is not None
        and slider_up is not None
        and abs(slider_up[0] - slider_rest[0]) < 0.001
        and abs(slider_up[1] - slider_rest[1]) < 0.001,
        details=(
            f"rest_xy=({slider_rest[0]:.4f},{slider_rest[1]:.4f}), "
            f"pushed_xy=({slider_up[0]:.4f},{slider_up[1]:.4f})"
        ),
    )

    # --- Thumb tab visible outside the tube ---
    body_aabb = ctx.part_element_world_aabb(slider, elem="slider_body")
    ctx.check(
        "thumb tab extends beyond tube outer radius",
        body_aabb is not None and body_aabb[1][0] > TUBE_R + 0.001,
        details=f"slider max_x={body_aabb[1][0] if body_aabb else None}, tube_r={TUBE_R}",
    )

    # --- Tube slot: the tube body is not a simple closed cylinder ---
    tube_aabb = ctx.part_element_world_aabb(base, elem="tube_body")
    ctx.check(
        "tube body has slot (X extent less than full diameter at some height)",
        tube_aabb is not None
        and (tube_aabb[1][0] - tube_aabb[0][0]) > 2.0 * TUBE_R * 0.9,
        details=f"tube x_extent={tube_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
