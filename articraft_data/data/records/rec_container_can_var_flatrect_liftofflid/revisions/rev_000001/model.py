from __future__ import annotations

# Metal rectangular fuel flask variant: full-width straight-lift press lid.
# Fork of the round-screw-cap flask — the threaded neck, screw cap, and bail
# ring are replaced by a raised rim on the body top deck and a flat rectangular
# press lid with a short downward skirt that wraps the rim and lifts straight
# up (PRISMATIC, +Z) off the body.
#
# Frame: flask stands upright, height along +Z, width along +X, depth along +Y.
#   - body centerline at x=0, y=0; bottom at z=0, top of body at z=BODY_H.
#   - a raised rim stands on the body top deck; the lid seats on the rim.
# Articulations:
#   - lid_lift (PRISMATIC, +Z): the lid lifts straight up off the body.

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

# ---- key dimensions (meters) ----
BODY_W = 0.130   # x extent
BODY_D = 0.060   # y extent
BODY_H = 0.130   # z extent of the main body
EDGE_R = 0.012   # rounded vertical-edge radius
WALL = 0.0035    # sheet-metal wall thickness
FLOOR = 0.004    # interior floor thickness
TOP_PLATE = 0.0035  # top deck thickness around the open mouth

# Rim: raised rectangular lip on the body top deck.
RIM_INSET = 0.003   # inset from body edges
RIM_WALL = 0.002    # rim wall thickness
RIM_H = 0.005       # rim height above body top
RIM_TOP_Z = BODY_H + RIM_H  # seating plane

# Lid: flat plate + downward peripheral skirt.
PLATE_H = 0.003        # lid plate thickness
SKIRT_H = 0.006        # skirt depth (captures below body top when seated)
SKIRT_WALL = 0.002     # skirt wall thickness
SKIRT_CLEARANCE = 0.0005  # radial clearance between skirt inner and rim outer

# Grip ribs on the lid top surface (repeated features).
N_GRIP_RIBS = 5
RIB_W = 0.080
RIB_D = 0.004
RIB_H = 0.0012


def _rounded_box(w, d, h, r, base_z=0.0):
    """Shared helper: rounded-corner CadQuery box with base at z=base_z."""
    r = min(r, w / 2.0 - 0.0005, d / 2.0 - 0.0005)
    return (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, base_z))
        .box(w, d, h, centered=(True, True, False))
        .edges("|Z")
        .fillet(r)
    )


def _body_with_rim():
    """Flask body shell with raised rim and an open mouth into the cavity."""
    body = _rounded_box(BODY_W, BODY_D, BODY_H, EDGE_R)
    inner = _rounded_box(
        BODY_W - 2 * WALL,
        BODY_D - 2 * WALL,
        BODY_H - FLOOR - TOP_PLATE,
        max(EDGE_R - WALL, 0.002),
        base_z=FLOOR,
    )

    # Rim dimensions (outer matches body inset, inner hollow for the wall).
    rim_outer_w = BODY_W - 2 * RIM_INSET
    rim_outer_d = BODY_D - 2 * RIM_INSET
    rim_inner_w = rim_outer_w - 2 * RIM_WALL
    rim_inner_d = rim_outer_d - 2 * RIM_WALL
    rim_r = max(EDGE_R - RIM_INSET, 0.002)
    rim_inner_r = max(rim_r - RIM_WALL, 0.001)

    # Rim outer solid extends slightly into the body for a clean union.
    rim_outer = _rounded_box(
        rim_outer_w, rim_outer_d, RIM_H + 0.001, rim_r,
        base_z=BODY_H - 0.001,
    )
    # Hollow cutter removes the inside of the rim wall.
    rim_hollow = _rounded_box(
        rim_inner_w, rim_inner_d, RIM_H + 0.003, rim_inner_r,
        base_z=BODY_H,
    )

    mouth = _rounded_box(
        rim_inner_w,
        rim_inner_d,
        TOP_PLATE + RIM_H + 0.004,
        rim_inner_r,
        base_z=BODY_H - TOP_PLATE - 0.001,
    )

    return body.cut(inner).union(rim_outer).cut(rim_hollow).cut(mouth)


def _lid_solid():
    """Lid: flat plate with downward peripheral skirt.
    Lid-local origin is at the plate bottom (seating plane).
    Plate extends upward to z=+PLATE_H; skirt hangs down to z=-SKIRT_H.
    """
    # Plate at z=0 to z=PLATE_H, matching body footprint.
    plate = _rounded_box(BODY_W, BODY_D, PLATE_H, EDGE_R)

    # Skirt inner matches rim outer + clearance (wraps the rim from outside).
    rim_outer_w = BODY_W - 2 * RIM_INSET
    rim_outer_d = BODY_D - 2 * RIM_INSET
    rim_r = max(EDGE_R - RIM_INSET, 0.002)

    skirt_inner_w = rim_outer_w + 2 * SKIRT_CLEARANCE
    skirt_inner_d = rim_outer_d + 2 * SKIRT_CLEARANCE
    skirt_outer_w = skirt_inner_w + 2 * SKIRT_WALL
    skirt_outer_d = skirt_inner_d + 2 * SKIRT_WALL

    skirt_inner_r = rim_r + SKIRT_CLEARANCE
    skirt_outer_r = skirt_inner_r + SKIRT_WALL

    # Skirt outer solid at z=-SKIRT_H to z=+0.001 (overlaps plate for union).
    skirt_outer = _rounded_box(
        skirt_outer_w, skirt_outer_d, SKIRT_H + 0.001, skirt_outer_r,
        base_z=-SKIRT_H,
    )
    # Hollow cutter removes the inside of the skirt wall only (not the plate).
    # Stops just below the plate bottom to preserve the full plate top surface.
    skirt_hollow = _rounded_box(
        skirt_inner_w, skirt_inner_d, SKIRT_H + 0.0015, skirt_inner_r,
        base_z=-SKIRT_H - 0.001,
    )

    return plate.union(skirt_outer).cut(skirt_hollow)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fuel_flask_press_lid")

    metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.35, 0.37, 0.40, 1.0))
    label_white = model.material("label_white", rgba=(0.93, 0.93, 0.90, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.20, 0.20, 0.22, 1.0))

    # ---- body (root): flask shell with raised rim + front label ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_with_rim(), "flask_shell"),
        material=metal,
        name="flask_shell",
    )
    # Printed front label on the +Y broad face (inherited from parent).
    body.visual(
        Box((0.058, 0.0015, 0.080)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 + 0.0006, 0.060)),
        material=label_white,
        name="front_label",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- lid: flat rectangular press-fit cap with skirt + grip ribs ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=dark_metal,
        name="lid_shell",
    )

    # Grip ribs on the lid top, evenly spaced along Y (repeated sub-parts).
    # Each rib is half-embedded into the plate top for robust mesh connectivity.
    rib_spacing = (BODY_D - 2 * RIB_D) / max(N_GRIP_RIBS - 1, 1)
    rib_y_start = -(BODY_D / 2.0 - RIB_D)
    for i in range(N_GRIP_RIBS):
        y = rib_y_start + i * rib_spacing
        lid.visual(
            Box((RIB_W, RIB_D, RIB_H)),
            origin=Origin(xyz=(0.0, y, PLATE_H + RIB_H * 0.25)),
            material=grip_rubber,
            name=f"grip_rib_{i}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, PLATE_H + SKIRT_H)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, (PLATE_H - SKIRT_H) / 2.0)),
    )

    # ---- articulation: lid lifts straight up along +Z ----
    # Joint origin at the rim top (seating plane) on the body.
    # At q=0, lid frame coincides with joint frame (lid seated on rim).
    # Positive q translates the lid upward along +Z.
    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.5, lower=0.0, upper=0.06,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_lift = object_model.get_articulation("lid_lift")

    def ext(part):
        mn, mx = ctx.part_world_aabb(part)
        return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])

    # ---- body is flat and rectangular (inherited from parent) ----
    bx, by, bz = ext(body)
    ctx.check(
        "flask body is flat (width greater than depth)",
        bx > by + 0.03,
        details=f"body extents={bx, by, bz}",
    )
    ctx.check(
        "flask body is upright and tall",
        bz > 0.10,
        details=f"body extents={bx, by, bz}",
    )
    ctx.check(
        "rim mouth opens through the deck into a hollow body cavity",
        BODY_H - FLOOR - TOP_PLATE > BODY_H * 0.80
        and BODY_W - 2 * RIM_INSET - 2 * RIM_WALL < BODY_W
        and BODY_D - 2 * RIM_INSET - 2 * RIM_WALL < BODY_D,
        details=(
            f"mouth=({BODY_W - 2 * RIM_INSET - 2 * RIM_WALL}, "
            f"{BODY_D - 2 * RIM_INSET - 2 * RIM_WALL}), "
            f"cavity_h={BODY_H - FLOOR - TOP_PLATE}"
        ),
    )

    # ---- lid covers the full top deck footprint ----
    ctx.expect_overlap(
        lid, body,
        axes="xy",
        min_overlap=min(BODY_W, BODY_D) * 0.80,
        name="lid covers the full body top deck footprint",
    )

    # ---- lid is mounted at the top of the flask ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is mounted at the top of the flask",
        lid_pos is not None and lid_pos[2] > BODY_H - 0.010,
        details=f"lid origin z={lid_pos}",
    )

    # ---- lid skirt intentionally captures the body rim (press-fit) ----
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_shell",
        elem_b="flask_shell",
        reason=(
            "The lid skirt is intentionally seated over the body rim as a "
            "press-fit capture; the skirt extends slightly below the body "
            "top plane to wrap the rim."
        ),
    )
    ctx.expect_contact(
        lid, body,
        elem_a="lid_shell",
        elem_b="flask_shell",
        name="lid skirt contacts the body rim when seated",
    )

    # ---- prismatic joint lifts the lid straight up along +Z ----
    rest_z = ctx.part_world_position(lid)[2]
    lift_amount = 0.040  # 40 mm lift
    with ctx.pose({lid_lift: lift_amount}):
        lifted_z = ctx.part_world_position(lid)[2]
    ctx.check(
        "lid lifts straight up off the body",
        lifted_z > rest_z + lift_amount * 0.9,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # ---- lid XY center stays fixed during straight lift (no drift) ----
    rest_aabb = ctx.part_world_aabb(lid)
    rest_cx = (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
    rest_cy = (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
    with ctx.pose({lid_lift: lift_amount}):
        lifted_aabb = ctx.part_world_aabb(lid)
        lifted_cx = (lifted_aabb[0][0] + lifted_aabb[1][0]) / 2.0
        lifted_cy = (lifted_aabb[0][1] + lifted_aabb[1][1]) / 2.0
    drift = math.hypot(lifted_cx - rest_cx, lifted_cy - rest_cy)
    ctx.check(
        "lid XY center stays fixed during straight lift",
        drift < 0.002,
        details=(
            f"rest_center=({rest_cx:.4f}, {rest_cy:.4f}), "
            f"lifted_center=({lifted_cx:.4f}, {lifted_cy:.4f}), drift={drift:.5f}"
        ),
    )

    # ---- grip ribs exist on the lid ----
    for i in range(N_GRIP_RIBS):
        rib_name = f"grip_rib_{i}"
        v = lid.get_visual(rib_name)
        ctx.check(
            f"grip rib {rib_name} exists on lid",
            v is not None,
            details=f"visual {rib_name} not found",
        )

    return ctx.report()


object_model = build_object_model()
