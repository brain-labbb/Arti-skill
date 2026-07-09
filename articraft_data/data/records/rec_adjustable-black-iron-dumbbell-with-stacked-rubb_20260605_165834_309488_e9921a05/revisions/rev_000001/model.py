from __future__ import annotations

# Adjustable black iron dumbbell.
# Frame: handle axis along +X, centerline at y=z=0, midpoint at the origin.
#   Static body  = steel handle (knurled grip) + stacked black weight plates on
#                  each side (more plates on the +X side, as in the reference).
#   Threaded ends poke past the plate stacks; a knurled STAR-LOCK spin collar
#   seats on each exposed thread.
# Articulations:
#   - left  star-lock collar (-X end): CONTINUOUS spin about the handle axis.
#   - right star-lock collar (+X end): CONTINUOUS spin about the handle axis.
# Each collar carries a small off-axis star point + index dot so its spin is
# detectable by AABB-based spin tests.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions --
HANDLE_LEN = 0.32  # overall steel bar length
BAR_R = 0.0140  # smooth bar radius (between collars / under plates)
GRIP_R = 0.0155  # knurled grip radius (center of bar)
GRIP_LEN = 0.115  # knurled center grip length
THREAD_R = 0.0135  # threaded end radius
SLEEVE_LEN = 0.085  # length of each plate-carrying sleeve from grip end

PLATE_R = 0.090  # plate outer radius (dia 0.18 m)
PLATE_HUB_R = 0.024  # raised center hub radius
PLATE_BORE_R = 0.0150  # center bore radius (slips over the bar/sleeve)
PLATE_THK = 0.0145  # single plate thickness

# Plate counts per side (right side shows more, matching the photo).
LEFT_PLATES = 3
RIGHT_PLATES = 5

COLLAR_R = 0.0265  # star-lock nut body radius
COLLAR_LEN = 0.030  # collar body length
STAR_POINTS = 6  # star-lock lobe count


def _knurled_cylinder(radius: float, length: float, *, teeth: int = 28, depth: float = 0.0011):
    """A cylinder (axis +X) with shallow axial knurl flutes cut into the side."""
    body = cq.Workplane("YZ").circle(radius).extrude(length)
    # cut a ring of shallow grooves around the side
    for i in range(teeth):
        ang = 2.0 * math.pi * i / teeth
        cy = (radius + depth * 0.2) * math.cos(ang)
        cz = (radius + depth * 0.2) * math.sin(ang)
        groove = (
            cq.Workplane("YZ")
            .center(cy, cz)
            .circle(depth)
            .extrude(length)
        )
        body = body.cut(groove)
    return body.translate((0.0, 0.0, 0.0))


def _handle_solid() -> cq.Workplane:
    """Steel bar along +X: threaded ends, smooth load sleeves, knurled center."""
    x0 = -HANDLE_LEN / 2.0

    # full-length core bar
    bar = cq.Workplane("YZ").workplane(offset=x0).circle(BAR_R).extrude(HANDLE_LEN)

    # threaded end tips (slightly reduced) -> reads as the exposed thread
    tip_len = 0.034
    for sign in (-1.0, 1.0):
        cx = sign * (HANDLE_LEN / 2.0 - tip_len)
        tip = (
            cq.Workplane("YZ")
            .workplane(offset=cx if sign > 0 else cx - tip_len + tip_len)
            .circle(THREAD_R)
            .extrude(sign * tip_len)
        )
        bar = bar.union(tip)
        # thread ridges: stack of thin torus-like rings via small cylinders
        n_rings = 7
        for k in range(n_rings):
            rx = sign * (HANDLE_LEN / 2.0 - tip_len + (k + 0.5) * (tip_len / n_rings))
            ring = (
                cq.Workplane("YZ")
                .workplane(offset=rx)
                .circle(THREAD_R + 0.0009)
                .extrude(0.0016)
            )
            bar = bar.union(ring)

    # knurled center grip (a thicker sleeve over the middle of the bar)
    grip = _knurled_cylinder(GRIP_R, GRIP_LEN, teeth=30, depth=0.0012)
    grip = grip.translate((-GRIP_LEN / 2.0, 0.0, 0.0))
    bar = bar.union(grip)

    # collar shoulders where the grip meets the sleeves (small fillet rings)
    for sign in (-1.0, 1.0):
        sx = sign * GRIP_LEN / 2.0
        shoulder = (
            cq.Workplane("YZ")
            .workplane(offset=sx - 0.004 if sign > 0 else sx)
            .circle(GRIP_R - 0.001)
            .extrude(sign * 0.004)
        )
        bar = bar.union(shoulder)

    return bar


def _plate_mesh(name: str):
    """A single black weight plate: thick disc + raised center hub + bore."""
    outer = CylinderGeometry(PLATE_R, PLATE_THK, radial_segments=48).rotate_y(math.pi / 2.0)
    # raised hub on each face
    hub = CylinderGeometry(PLATE_HUB_R, PLATE_THK + 0.010, radial_segments=36).rotate_y(
        math.pi / 2.0
    )
    outer.merge(hub)
    # bevel the rim slightly with a thin cone-ish lip (use a smaller disc inset)
    rim = CylinderGeometry(PLATE_R - 0.006, PLATE_THK + 0.0015, radial_segments=48).rotate_y(
        math.pi / 2.0
    )
    outer.merge(rim)
    return mesh_from_geometry(outer, name)


def _collar_solid() -> cq.Workplane:
    """Knurled spin collar: a knurled nut body + protruding star/lever lobes."""
    body = _knurled_cylinder(COLLAR_R, COLLAR_LEN, teeth=24, depth=0.0014)
    body = body.translate((-COLLAR_LEN / 2.0, 0.0, 0.0))

    # star-lock lever lobes radiating outward (the spin handle of the lock nut)
    lobe_len = 0.018
    for i in range(STAR_POINTS):
        ang = 2.0 * math.pi * i / STAR_POINTS
        # a tapered finger sticking out radially, centered on the collar mid
        finger = (
            cq.Workplane("XY")
            .rect(COLLAR_LEN * 0.7, 0.010)
            .workplane(offset=lobe_len)
            .rect(COLLAR_LEN * 0.45, 0.006)
            .loft(ruled=True)
        )
        # the loft above extrudes along +Z; rotate so it points along +Y, then
        # spin around the X axis to distribute the lobes.
        finger = finger.rotate((0, 0, 0), (1, 0, 0), -90.0)  # +Z -> +Y
        finger = finger.translate((0.0, COLLAR_R - 0.002, 0.0))
        finger = finger.rotate((0, 0, 0), (1, 0, 0), math.degrees(ang))
        body = body.union(finger)

    # central hub cap so the nut reads closed on the outboard face
    cap = cq.Workplane("YZ").workplane(offset=COLLAR_LEN / 2.0 - 0.003).circle(
        COLLAR_R * 0.55
    ).extrude(0.005)
    body = body.union(cap)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_dumbbell")

    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    matte_black = model.material("plate_black", rgba=(0.07, 0.07, 0.08, 1.0))
    collar_steel = model.material("collar_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    marker_red = model.material("index_marker", rgba=(0.80, 0.12, 0.10, 1.0))

    # =========================================================== static body ==
    body = model.part("dumbbell_body")

    # steel handle
    body.visual(
        mesh_from_cadquery(_handle_solid(), "handle"),
        material=steel,
        name="handle",
    )

    # plate stacks. Plates seat against the grip shoulders and stack outward.
    # The first plate's inner face sits at the end of the grip; plates butt up.
    def add_stack(sign: float, count: int, side: str):
        inner_x = sign * GRIP_LEN / 2.0  # plate stack starts at grip end
        for j in range(count):
            cx = inner_x + sign * (PLATE_THK / 2.0 + j * PLATE_THK)
            body.visual(
                _plate_mesh(f"{side}_plate_{j}"),
                origin=Origin(xyz=(cx, 0.0, 0.0)),
                material=matte_black,
                name=f"{side}_plate_{j}",
            )

    add_stack(-1.0, LEFT_PLATES, "left")
    add_stack(1.0, RIGHT_PLATES, "right")

    body.inertial = Inertial.from_geometry(
        Box((HANDLE_LEN, 2 * PLATE_R, 2 * PLATE_R)), mass=12.0
    )

    # Collar seat X positions: the nut body (half-length COLLAR_LEN/2) must seat
    # outboard of the last plate, clearing the raised hub (HUB_PROUD past the
    # plate face) so the only intentional overlap is the nut sitting on the
    # threaded handle end.
    hub_proud = 0.005  # hub protrudes this far past the plate face
    left_stack_end = -(GRIP_LEN / 2.0 + LEFT_PLATES * PLATE_THK)
    right_stack_end = GRIP_LEN / 2.0 + RIGHT_PLATES * PLATE_THK
    left_collar_x = left_stack_end - (hub_proud + COLLAR_LEN / 2.0 + 0.002)
    right_collar_x = right_stack_end + (hub_proud + COLLAR_LEN / 2.0 + 0.002)

    # =================================================== star-lock collars ====
    def add_collar(name: str, sign: float, joint: str):
        collar = model.part(name)
        collar.visual(
            mesh_from_cadquery(_collar_solid(), f"{name}_nut"),
            material=collar_steel,
            name=f"{name}_nut",
        )
        # off-axis index marker on one star lobe so spin is detectable by AABB.
        collar.visual(
            Box((0.006, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, COLLAR_R + 0.014, 0.0)),
            material=marker_red,
            name=f"{name}_marker",
        )
        collar.inertial = Inertial.from_geometry(
            Box((COLLAR_LEN, 2 * (COLLAR_R + 0.018), 2 * (COLLAR_R + 0.018))), mass=0.12
        )
        seat_x = left_collar_x if sign < 0 else right_collar_x
        model.articulation(
            joint,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=collar,
            origin=Origin(xyz=(seat_x, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=8.0),
        )

    add_collar("left_collar", -1.0, "body_to_left_collar")
    add_collar("right_collar", 1.0, "body_to_right_collar")

    return model


# ---------------------------------------------------------------------- tests --
def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("dumbbell_body")
    left_collar = object_model.get_part("left_collar")
    right_collar = object_model.get_part("right_collar")
    left_joint = object_model.get_articulation("body_to_left_collar")
    right_joint = object_model.get_articulation("body_to_right_collar")

    # --- plate stacks present on both sides --------------------------------
    li = body.get_visual("left_plate_0")
    lo = body.get_visual(f"left_plate_{LEFT_PLATES - 1}")
    ri = body.get_visual("right_plate_0")
    ro = body.get_visual(f"right_plate_{RIGHT_PLATES - 1}")
    ctx.check(
        "left and right plate stacks exist",
        all(v is not None for v in (li, lo, ri, ro)),
        details="plate visuals must exist on both sides",
    )

    # right stack has more plates than left (matches reference)
    ctx.check(
        "right stack has more plates than left",
        RIGHT_PLATES > LEFT_PLATES,
        details=f"left={LEFT_PLATES}, right={RIGHT_PLATES}",
    )

    # plates are mounted on the handle: inner plate seats against the body.
    li_aabb = ctx.part_element_world_aabb(body, elem="left_plate_0")
    ro_aabb = ctx.part_element_world_aabb(body, elem=f"right_plate_{RIGHT_PLATES - 1}")
    ctx.check(
        "plate stacks span outward on each side of center",
        li_aabb is not None and ro_aabb is not None
        and li_aabb[0][0] < 0.0 < ro_aabb[1][0],
        details=f"left_inner={li_aabb}, right_outer={ro_aabb}",
    )

    # --- plates are black, handle/collars metallic -------------------------
    ctx.check(
        "plates use matte-black material",
        body.get_visual("left_plate_0").material.name == "plate_black"
        and body.get_visual("right_plate_0").material.name == "plate_black",
        details="plate material name should be plate_black",
    )
    ctx.check(
        "handle is steel/metallic",
        body.get_visual("handle").material.name == "steel",
        details="handle material name should be steel",
    )
    ctx.check(
        "collars are metallic steel",
        left_collar.get_visual("left_collar_nut").material.name == "collar_steel"
        and right_collar.get_visual("right_collar_nut").material.name == "collar_steel",
        details="collar nut material should be collar_steel",
    )

    # --- collars outboard of plates ----------------------------------------
    lc_pos = ctx.part_world_position(left_collar)
    rc_pos = ctx.part_world_position(right_collar)
    left_outer = ctx.part_element_world_aabb(body, elem=f"left_plate_{LEFT_PLATES - 1}")
    right_outer = ctx.part_element_world_aabb(body, elem=f"right_plate_{RIGHT_PLATES - 1}")
    ctx.check(
        "left collar is outboard of left plate stack",
        lc_pos is not None and left_outer is not None and lc_pos[0] < left_outer[0][0],
        details=f"left_collar_x={lc_pos}, left_stack_min_x={left_outer[0][0] if left_outer else None}",
    )
    ctx.check(
        "right collar is outboard of right plate stack",
        rc_pos is not None and right_outer is not None and rc_pos[0] > right_outer[1][0],
        details=f"right_collar_x={rc_pos}, right_stack_max_x={right_outer[1][0] if right_outer else None}",
    )

    # collars seat on the exposed thread (intentional small overlap with body)
    ctx.allow_overlap(
        left_collar,
        body,
        elem_a="left_collar_nut",
        elem_b="handle",
        reason="Star-lock collar is threaded onto the exposed handle end (seated).",
    )
    ctx.allow_overlap(
        right_collar,
        body,
        elem_a="right_collar_nut",
        elem_b="handle",
        reason="Star-lock collar is threaded onto the exposed handle end (seated).",
    )
    ctx.expect_overlap(
        left_collar, body, axes="x", elem_a="left_collar_nut", elem_b="handle",
        min_overlap=0.004, name="left collar seated on thread",
    )
    ctx.expect_overlap(
        right_collar, body, axes="x", elem_a="right_collar_nut", elem_b="handle",
        min_overlap=0.004, name="right collar seated on thread",
    )

    # --- both collars spin about the handle axis (marker rotates) ----------
    def marker_yz(part):
        a = ctx.part_element_world_aabb(
            part, elem=f"{part.name}_marker"
        )
        if a is None:
            return None
        cy = (a[0][1] + a[1][1]) / 2.0
        cz = (a[0][2] + a[1][2]) / 2.0
        return (cy, cz)

    for collar, joint in ((left_collar, left_joint), (right_collar, right_joint)):
        rest = marker_yz(collar)
        with ctx.pose({joint: math.pi / 2.0}):
            turned = marker_yz(collar)
        ok = (
            rest is not None
            and turned is not None
            and abs(turned[1] - rest[0]) < 0.004  # new z ~= old y
            and abs(rest[1]) < 0.004  # marker starts on +Y (z~0)
            and abs(turned[1]) > 0.010  # marker now has significant z
        )
        ctx.check(
            f"{collar.name} marker rotates about handle axis",
            ok,
            details=f"rest_yz={rest}, quarter_turn_yz={turned}",
        )

    return ctx.report()


object_model = build_object_model()
