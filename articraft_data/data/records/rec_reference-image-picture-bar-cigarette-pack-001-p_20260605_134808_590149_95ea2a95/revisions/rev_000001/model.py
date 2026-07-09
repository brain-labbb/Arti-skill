from __future__ import annotations

# Black-and-red FLIP-TOP cigarette HARDPACK (reference: BILL'S CIGS :)).
# No cigarettes are modeled - this is the pack only.
#
# Frame: Z up (height), X width, Y depth. The printed FRONT face looks toward
# -Y, the hinge is at the +Y back-top edge. Base sits at z=0.
#
# Construction (CadQuery, meters):
#   - pack_body: a hollow shelled box, open at the top, whose rim is cut on a
#     diagonal - LOW at the front (the mouth) and HIGH at the back (the hinge).
#   - collar: a bright-orange inner frame glued inside the body that PROTRUDES
#     above the front mouth (the iconic bright band you see when the lid is up).
#   - flip_lid: the complementary hollow cap, hinged at the back-top edge. This
#     is the primary articulation (REVOLUTE flip).
# Printed graphics (front label panels, brand band, side barcode) are thin flush
# decals seated on the relevant face so they read as print, not floating tiles.

import cadquery as cq
from math import atan2, degrees

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters), king-size hardpack ----
W = 0.055          # width (X)
D = 0.022          # depth (Y)
H = 0.088          # total closed height (Z)
T = 0.0013         # card wall thickness
R_SIDE = 0.0030    # rounded vertical edges
R_TB = 0.0016      # rounded top/bottom edges

Z_FRONT = 0.058    # rim cut height at the FRONT (the mouth) - lower
Z_BACK = 0.069     # rim cut height at the BACK (the hinge) - higher

HINGE = (0.0, D / 2.0 - T / 2.0, Z_BACK)   # lid pivot line (along X), at back-top


def _outer_box() -> cq.Workplane:
    # Full rounded outer block, base on z=0.
    box = (
        cq.Workplane("XY")
        .box(W, D, H, centered=(True, True, False))
        .edges("|Z")
        .fillet(R_SIDE)
        .edges("#Z")
        .fillet(R_TB)
    )
    return box


def _cavity(z0: float, z1: float) -> cq.Workplane:
    # Interior block used to hollow the shell; walls/floor left at thickness T.
    return (
        cq.Workplane("XY")
        .box(W - 2.0 * T, D - 2.0 * T, z1 - z0, centered=(True, True, False))
        .translate((0.0, 0.0, z0))
    )


def _lid_cut_tool() -> cq.Workplane:
    # Big half-space sitting ABOVE the diagonal rim plane (front-low, back-high).
    # Rotating a z>=0 block about +X by phi maps its +Z onto the rim normal, so
    # the block fills the lid region; cut keeps the body, intersect keeps the lid.
    z_mid = (Z_FRONT + Z_BACK) / 2.0
    phi = atan2(Z_BACK - Z_FRONT, D)
    L = 0.4
    return (
        cq.Workplane("XY")
        .box(L, L, L, centered=(True, True, False))
        .rotate((0, 0, 0), (1, 0, 0), degrees(phi))
        .translate((0.0, 0.0, z_mid))
    )


def _body_solid() -> cq.Workplane:
    hollow = _outer_box().cut(_cavity(T, H + 0.01))   # open-top hollow shell
    return hollow.cut(_lid_cut_tool())                # trim off the lid region


def _lid_solid_world() -> cq.Workplane:
    cap = _outer_box().intersect(_lid_cut_tool())     # solid top cap (closed top)
    return cap.cut(_cavity(-0.01, H - T))             # hollow it, open bottom


def _lid_solid_local() -> cq.Workplane:
    # Re-express the lid in the hinge-local frame so articulation origin = HINGE.
    return _lid_solid_world().translate((-HINGE[0], -HINGE[1], -HINGE[2]))


def _collar_solid() -> cq.Workplane:
    # Bright inner frame: U-shape (front + two sides, open at back). The side
    # walls overlap the body's inner walls (same part) for a solid join and stop
    # flush at the mouth; only the FRONT strip rises above the mouth -> the bright
    # band that shows when the lid is open.
    tc = 0.0010
    z_bot = Z_FRONT - 0.014
    z_side_top = Z_FRONT
    z_front_top = Z_FRONT + 0.0070
    x_out = W / 2.0 - T + 0.0003       # bites into the side walls (same part)
    y_front = -D / 2.0 + T + 0.0006    # clear of the closed lid's front wall
    y_back = D / 2.0 - T - 0.002       # leave the back open

    front = (
        cq.Workplane("XY")
        .box(2.0 * x_out, tc, z_front_top - z_bot, centered=(True, True, False))
        .translate((0.0, y_front + tc / 2.0, z_bot))
    )
    side_len = y_back - y_front
    sides = front
    for sx in (-1.0, 1.0):
        side = (
            cq.Workplane("XY")
            .box(tc, side_len, z_side_top - z_bot, centered=(True, True, False))
            .translate((sx * (x_out - tc / 2.0), y_front + side_len / 2.0, z_bot))
        )
        sides = sides.union(side)
    return sides


def _to_lid_local(xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    return (xyz[0] - HINGE[0], xyz[1] - HINGE[1], xyz[2] - HINGE[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_top_cigarette_pack")

    charcoal = model.material("charcoal_card", rgba=(0.055, 0.048, 0.044, 1.0))
    orange = model.material("inner_frame_orange", rgba=(0.95, 0.30, 0.09, 1.0))
    red = model.material("brand_red", rgba=(0.80, 0.12, 0.07, 1.0))
    photo = model.material("photo_panel", rgba=(0.55, 0.45, 0.36, 1.0))
    white = model.material("white_ink", rgba=(0.90, 0.88, 0.83, 1.0))
    dark_red = model.material("side_dark_red", rgba=(0.32, 0.045, 0.035, 1.0))
    bar_dark = model.material("barcode_dark", rgba=(0.08, 0.08, 0.08, 1.0))
    bar_white = model.material("barcode_white", rgba=(0.86, 0.85, 0.81, 1.0))

    # ---------- pack body (root) ----------
    body = model.part("pack_body")
    body.visual(mesh_from_cadquery(_body_solid(), "pack_body"), material=charcoal, name="pack_body")
    body.visual(mesh_from_cadquery(_collar_solid(), "inner_frame"), material=orange, name="inner_frame")

    def front_decal(name, mat, w, h, cz, cx=0.0):
        body.visual(
            Box((w, 0.0006, h)),
            origin=Origin(xyz=(cx, -D / 2.0 + 0.0001, cz)),
            material=mat,
            name=name,
        )

    # Front label, top -> bottom (reads as the printed BILL'S CIGS face).
    front_decal("front_brand_band", red, 0.049, 0.0050, 0.0545)
    front_decal("front_smoking_word", white, 0.030, 0.0040, 0.0492)
    front_decal("front_photo_panel", photo, 0.040, 0.0200, 0.0330, cx=0.002)
    front_decal("front_kills_word", white, 0.022, 0.0040, 0.0195)
    front_decal("front_maker_text", white, 0.032, 0.0028, 0.0110)
    # Dark-red vertical stripe down the left of the front face.
    body.visual(
        Box((0.0040, 0.0006, 0.046)),
        origin=Origin(xyz=(-0.0228, -D / 2.0 + 0.0001, 0.030)),
        material=dark_red,
        name="front_red_stripe",
    )

    # Side barcode on the +X face: a light backing with dark vertical bars.
    body.visual(
        Box((0.0007, 0.0130, 0.0140)),
        origin=Origin(xyz=(W / 2.0 - 0.0002, 0.0, 0.0275)),
        material=bar_white,
        name="side_barcode_backing",
    )
    for i, cy in enumerate((-0.0048, -0.0028, -0.0010, 0.0012, 0.0032, 0.0050)):
        body.visual(
            Box((0.0012, 0.0007, 0.0110)),
            origin=Origin(xyz=(W / 2.0 + 0.0002, cy, 0.0275)),
            material=bar_dark,
            name=f"side_barcode_bar_{i}",
        )
    # Echo of the printed side text on the -X face.
    body.visual(
        Box((0.0006, 0.0070, 0.045)),
        origin=Origin(xyz=(-W / 2.0 - 0.0001, 0.0, 0.028)),
        material=dark_red,
        name="side_text_stripe",
    )

    # ---------- flip lid (revolute) ----------
    lid = model.part("flip_lid")
    lid.visual(mesh_from_cadquery(_lid_solid_local(), "flip_lid"), material=charcoal, name="flip_lid")
    # Thin orange seam line on the lid front edge (the inner-frame lip, closed pose).
    lid.visual(
        Box((0.049, 0.0006, 0.0018)),
        origin=Origin(xyz=_to_lid_local((0.0, -D / 2.0 + 0.0001, 0.0592))),
        material=orange,
        name="lid_seam_lip",
    )

    model.articulation(
        "lid_flip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=HINGE),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=2.40),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("pack_body")
    lid = object_model.get_part("flip_lid")
    hinge = object_model.get_articulation("lid_flip")

    # The lid seam seats on the box mouth (complementary diagonal cut).
    ctx.allow_overlap(
        body,
        lid,
        reason="The flip-top lid rim seats flush on the box mouth along the diagonal seam.",
    )

    # --- proportions: a tall, slim hardpack ---
    bmn, bmx = ctx.part_world_aabb(body)
    lmn, lmx = ctx.part_world_aabb(lid)
    full_mn = tuple(min(bmn[i], lmn[i]) for i in range(3))
    full_mx = tuple(max(bmx[i], lmx[i]) for i in range(3))
    height = full_mx[2] - full_mn[2]
    width = full_mx[0] - full_mn[0]
    depth = full_mx[1] - full_mn[1]
    ctx.check(
        "pack is a tall slim box (~0.088 x 0.055 x 0.022 m)",
        0.082 < height < 0.094 and 0.050 < width < 0.060 and 0.020 < depth < 0.026,
        details=f"h={height:.3f}, w={width:.3f}, d={depth:.3f}",
    )

    # --- the body is an open-top tray: its rim is lower at the front than back ---
    front_rim = ctx.part_element_world_aabb(body, elem="pack_body")
    ctx.check(
        "body rim is below the closed top (open mouth)",
        front_rim is not None and front_rim[1][2] < H - 0.015,
        details=f"body_top_z={None if front_rim is None else front_rim[1][2]:.4f}",
    )

    # --- bright inner frame protrudes above the front mouth ---
    collar = ctx.part_element_world_aabb(body, elem="inner_frame")
    ctx.check(
        "orange inner frame rises above the front mouth",
        collar is not None and Z_FRONT + 0.003 < collar[1][2] < Z_FRONT + 0.012,
        details=f"collar_top_z={None if collar is None else collar[1][2]:.4f}",
    )

    # --- closed lid caps the very top and covers the mouth footprint ---
    lid_box = ctx.part_element_world_aabb(lid, elem="flip_lid")
    ctx.check(
        "closed lid forms the top of the pack",
        lid_box is not None and lid_box[1][2] > H - 0.002,
        details=f"lid_top_z={None if lid_box is None else lid_box[1][2]:.4f}",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.018,
        elem_a="flip_lid",
        elem_b="pack_body",
        name="closed lid covers the open mouth footprint",
    )

    # --- the flip lid flips back off the mouth about the rear hinge ---
    closed = ctx.part_element_world_aabb(lid, elem="flip_lid")
    with ctx.pose({hinge: 2.0}):
        opened = ctx.part_element_world_aabb(lid, elem="flip_lid")
    # Closed, the lid front reaches the -Y face; open, it swings well behind it.
    ctx.check(
        "hinge flips the lid back and clear of the mouth",
        closed is not None
        and opened is not None
        and opened[0][1] > closed[0][1] + 0.008
        and opened[1][2] > Z_BACK,
        details=f"closed_front_y={closed[0][1]:.4f}, opened_front_y={opened[0][1]:.4f}, opened_top_z={opened[1][2]:.4f}",
    )

    # --- asset omits individual cigarettes ---
    forbidden = ("stick", "filter", "cigarette_rod", "loose_roll", "tobacco")
    bad: list[str] = []
    for part in object_model.parts:
        for visual in part.visuals:
            if visual.name and any(tok in visual.name for tok in forbidden):
                bad.append(f"{part.name}.{visual.name}")
    ctx.check("asset omits individual cigarettes", not bad, details=", ".join(bad))

    return ctx.report()
