from __future__ import annotations

"""Short, stout blacksmith pincers (nipper-style tongs) in aged dark gray steel.

Two forged halves cross at a broad flattened riveted boss. Above the boss two
thick claw-like jaws hook inward and meet at a narrow biting edge at the top,
leaving a visible open throat between the jaw backs. Below the boss two robust
handles bow gently outward, taper, and end in slightly flared rounded tips.
Worn, polished highlights live on the biting edges and handle tips. A single
revolute joint at the rivet (axis normal to the tool plane) opens the jaws by
about 0.35 rad when the handles are released / spread.

Coordinate convention: the tool lies in the world XZ plane (Z up along the
tool, X across it). Y is the thickness direction and the rivet/pivot axis.
The pivot is at the world origin. The rear arm (-Y lap layer at the boss) is
the root and carries the rivet; the front arm (+Y lap layer) is bored at the
boss and rotates about the rivet. The arms cross at the boss: the rear arm
owns the left jaw and the right handle, the front arm the right jaw and the
left handle, so squeezing the handles together closes the jaws.

All moving-front-arm geometry keeps its XZ radius from the pivot larger than
the boss-plate radius (jaws) or smaller than the lap-layer thickness reach
(handle roots), so rotation about the rivet never collides with the static
rear half at any joint angle.
"""

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
# Boss / rivet lap joint: broad flattened discs, ~0.045 m diameter.
BOSS_R = 0.0225
BOSS_T = 0.016  # each arm's boss plate thickness
# The rivet clamps the lap joint, so the two boss faces ride in true sliding
# contact on the y=0 lap plane. Rotation about the rivet axis preserves Y, so
# this contact never becomes penetration at any joint angle.
LAYER_HALF_GAP = 0.0
Y_REAR = -LAYER_HALF_GAP  # rear lap layer spans [Y_REAR - BOSS_T, Y_REAR]
Y_FRONT = LAYER_HALF_GAP  # front lap layer spans [Y_FRONT, Y_FRONT + BOSS_T]

BORE_R = 0.0060  # rivet bore through the front (moving) boss
RIVET_SHANK_R = 0.0055

# Front domed rivet head: large dome, ~0.024 m across.
DOME_F_BASE_R = 0.0120
DOME_F_H = 0.0049
DOME_F_GAP = 0.0001  # dome base floats 0.1 mm off the front boss face
# Rear (peened) head: smaller, embedded 0.4 mm into the rear boss face.
DOME_R_BASE_R = 0.0105
DOME_R_H = 0.0045
DOME_R_EMBED = 0.0004

# Jaws: thick inward-curving claws, full thickness centered on the tool plane.
JAW_T = 0.026  # head thickness (both jaws share this Y band)
JAW_BITE_X = 0.0020  # jaw body stops here; the worn bite strip goes closer
BITE_X_IN = 0.0006  # worn bite edge inner face (closed gap = 2 * 0.0006)
HEAD_TOP_Z = 0.0718

# Necks: lap-layer webs that carry each jaw down onto its boss plate.
NECK_Z_LO = 0.0080
NECK_Z_HI = 0.0330

# Handles: bowed outward, tapering, slightly flared rounded tips.
# (z, center_x, width, thickness) sections, lofted top to bottom.
HANDLE_SECTIONS = (
    (-0.0080, 0.0000, 0.0200, 0.0150),
    (-0.0600, 0.0200, 0.0190, 0.0145),
    (-0.1300, 0.0330, 0.0165, 0.0125),
    (-0.1900, 0.0330, 0.0140, 0.0105),
    (-0.2200, 0.0300, 0.0150, 0.0115),
)
HANDLE_TIP_R = 0.0055
HANDLE_TIP_C = (0.0295, -0.2230)  # (x, z) of the rounded tip ball center
HANDLE_YC_REAR = Y_REAR - BOSS_T / 2.0
HANDLE_YC_FRONT = Y_FRONT + BOSS_T / 2.0

PIVOT_UPPER = 0.35  # joint range 0 .. 0.35 rad of jaw opening


def _jaw(side: float) -> cq.Workplane:
    """Thick claw jaw, full thickness centered on y=0, on one side of x=0.

    Built as the +X (right) jaw and mirrored for side < 0. Every profile
    point keeps an XZ radius from the pivot above the boss radius, so the
    rotating jaw can never sweep into the static boss plates.
    """
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.0085, 0.0265)
        .lineTo(0.0240, 0.0270)
        .spline(
            [
                (0.0298, 0.0400),
                (0.0290, 0.0520),
                (0.0180, 0.0650),
                (0.0060, 0.0705),
                (JAW_BITE_X, HEAD_TOP_Z),
            ],
            includeCurrent=True,
        )
        .lineTo(JAW_BITE_X, 0.0640)
        .spline(
            [
                (0.0080, 0.0560),
                (0.0118, 0.0450),
                (0.0105, 0.0340),
            ],
            includeCurrent=True,
        )
        .close()
        .extrude(JAW_T)
        .translate((0.0, JAW_T / 2.0, 0.0))
    )
    if side < 0:
        profile = profile.mirror("YZ")
    return profile


def _bite_edge(side: float) -> cq.Workplane:
    """Narrow worn biting edge strip at the top of one jaw."""
    strip = (
        cq.Workplane("XZ")
        .moveTo(BITE_X_IN, 0.0625)
        .lineTo(0.0045, 0.0585)
        .lineTo(0.0045, HEAD_TOP_Z)
        .lineTo(BITE_X_IN, HEAD_TOP_Z)
        .close()
        .extrude(JAW_T)
        .translate((0.0, JAW_T / 2.0, 0.0))
    )
    if side < 0:
        strip = strip.mirror("YZ")
    return strip


def _neck(side: float, y_top: float) -> cq.Workplane:
    """Lap-layer web joining one jaw root to its boss plate."""
    web = (
        cq.Workplane("XZ")
        .moveTo(0.0040, NECK_Z_LO)
        .lineTo(0.0200, NECK_Z_LO)
        .lineTo(0.0240, NECK_Z_HI)
        .lineTo(0.0060, NECK_Z_HI)
        .close()
        .extrude(BOSS_T)
        .translate((0.0, y_top, 0.0))
    )
    if side < 0:
        web = web.mirror("YZ")
    return web


def _boss_plate(y_top: float, bored: bool) -> cq.Workplane:
    """Broad flattened hinge disc centered on the pivot, in one lap layer."""
    plate = (
        cq.Workplane("XZ")
        .circle(BOSS_R)
        .extrude(BOSS_T)
        .translate((0.0, y_top, 0.0))
    )
    if bored:
        bore = (
            cq.Workplane("XZ")
            .circle(BORE_R)
            .extrude(BOSS_T * 3.0)
            .translate((0.0, y_top + BOSS_T, 0.0))
        )
        plate = plate.cut(bore)
    return plate


def _handle(side: float, y_c: float) -> cq.Workplane:
    """Robust bowed handle: lofted rounded-rectangle sections along -Z."""
    sketches = []
    for z, cx, w, t in HANDLE_SECTIONS:
        rr = 0.30 * min(w, t)
        sk = (
            cq.Sketch()
            .rect(w, t)
            .vertices()
            .fillet(rr)
            .moved(cq.Location(cq.Vector(side * cx, y_c, z)))
        )
        sketches.append(sk)
    return cq.Workplane("XY").placeSketch(*sketches).loft()


def _handle_tip(side: float, y_c: float) -> cq.Workplane:
    """Rounded, worn handle tip ball seated in the flared handle end."""
    tx, tz = HANDLE_TIP_C
    return (
        cq.Workplane("XY")
        .sphere(HANDLE_TIP_R)
        .translate((side * tx, y_c, tz))
    )


def _rivet_shank() -> cq.Workplane:
    """Rivet shank: buried in the rear boss, through the front bore, into the dome."""
    y_lo = Y_REAR - BOSS_T + 0.003  # inside the rear boss plate
    y_hi = Y_FRONT + BOSS_T + DOME_F_GAP + 0.0015  # reaches into the front dome
    return (
        cq.Workplane("XZ")
        .circle(RIVET_SHANK_R)
        .extrude(y_hi - y_lo)
        .translate((0.0, y_hi, 0.0))
    )


def _dome(base_y: float, outward: float, base_r: float, h: float) -> cq.Workplane:
    """Spherical-cap rivet head: base circle at base_y, doming along outward Y."""
    sphere_r = (base_r * base_r + h * h) / (2.0 * h)
    center_y = base_y - outward * (sphere_r - h)
    cap = (
        cq.Workplane("XY")
        .sphere(sphere_r)
        .translate((0.0, center_y, 0.0))
    )
    cutter = (
        cq.Workplane("XY")
        .box(0.2, 0.2, 0.2)
        .translate((0.0, base_y - outward * 0.1, 0.0))
    )
    return cap.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blacksmith_pincers")

    aged_steel = model.material("aged_steel", rgba=(0.27, 0.28, 0.30, 1.0))
    boss_steel = model.material("boss_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.55, 0.57, 0.59, 1.0))
    rivet_steel = model.material("rivet_steel", rgba=(0.34, 0.35, 0.37, 1.0))

    # ---------------------------------------------------------- rear arm
    # Root half: left claw jaw, -Y lap layer at the boss, right handle,
    # and the rivet (shank plus both heads).
    rear = model.part("rear_arm")
    rear.visual(
        mesh_from_cadquery(_jaw(-1.0), "rear_jaw"),
        name="jaw",
        material=aged_steel,
    )
    rear.visual(
        mesh_from_cadquery(_bite_edge(-1.0), "rear_bite_edge"),
        name="bite_edge",
        material=worn_steel,
    )
    rear.visual(
        mesh_from_cadquery(_neck(-1.0, Y_REAR), "rear_neck"),
        name="neck",
        material=aged_steel,
    )
    rear.visual(
        mesh_from_cadquery(_boss_plate(Y_REAR, bored=False), "rear_boss_plate"),
        name="boss_plate",
        material=boss_steel,
    )
    rear.visual(
        mesh_from_cadquery(_handle(1.0, HANDLE_YC_REAR), "rear_handle"),
        name="handle",
        material=aged_steel,
    )
    rear.visual(
        mesh_from_cadquery(_handle_tip(1.0, HANDLE_YC_REAR), "rear_handle_tip"),
        name="handle_tip",
        material=worn_steel,
    )
    rear.visual(
        mesh_from_cadquery(_rivet_shank(), "rivet_shank"),
        name="rivet_shank",
        material=rivet_steel,
    )
    rear.visual(
        mesh_from_cadquery(
            _dome(Y_FRONT + BOSS_T + DOME_F_GAP, 1.0, DOME_F_BASE_R, DOME_F_H),
            "rivet_dome",
        ),
        name="rivet_dome",
        material=rivet_steel,
    )
    rear.visual(
        mesh_from_cadquery(
            _dome(Y_REAR - BOSS_T + DOME_R_EMBED, -1.0, DOME_R_BASE_R, DOME_R_H),
            "rivet_head_rear",
        ),
        name="rivet_head_rear",
        material=rivet_steel,
    )

    # ---------------------------------------------------------- front arm
    # Moving half: right claw jaw, bored boss plate in the +Y lap layer,
    # left handle. The arms cross at the boss so squeezing the handles
    # together closes the curved jaws.
    front = model.part("front_arm")
    front.visual(
        mesh_from_cadquery(_jaw(1.0), "front_jaw"),
        name="jaw",
        material=aged_steel,
    )
    front.visual(
        mesh_from_cadquery(_bite_edge(1.0), "front_bite_edge"),
        name="bite_edge",
        material=worn_steel,
    )
    front.visual(
        mesh_from_cadquery(_neck(1.0, Y_FRONT + BOSS_T), "front_neck"),
        name="neck",
        material=aged_steel,
    )
    front.visual(
        mesh_from_cadquery(_boss_plate(Y_FRONT + BOSS_T, bored=True), "front_boss_plate"),
        name="boss_plate",
        material=boss_steel,
    )
    front.visual(
        mesh_from_cadquery(_handle(-1.0, HANDLE_YC_FRONT), "front_handle"),
        name="handle",
        material=aged_steel,
    )
    front.visual(
        mesh_from_cadquery(_handle_tip(-1.0, HANDLE_YC_FRONT), "front_handle_tip"),
        name="handle_tip",
        material=worn_steel,
    )

    # ---------------------------------------------------------- articulation
    # Single revolute joint at the rivet, axis normal to the tool plane (+Y).
    # Positive q swings the front arm's right jaw toward +X (opening the bite
    # at the top) while its left handle splays further toward -X.
    model.articulation(
        "boss_rivet_pivot",
        ArticulationType.REVOLUTE,
        parent=rear,
        child=front,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=PIVOT_UPPER, effort=60.0, velocity=2.0
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_arm")
    rear = object_model.get_part("rear_arm")
    pivot = object_model.get_articulation("boss_rivet_pivot")

    # --- closed rest pose: bite edges nearly touch at the top center ---
    ctx.expect_gap(
        front,
        rear,
        axis="x",
        positive_elem="bite_edge",
        negative_elem="bite_edge",
        min_gap=0.0005,
        max_gap=0.003,
        name="closed jaws bring the worn bite edges nearly together",
    )
    # The jaw backs stay apart: only the narrow bite edges approach.
    ctx.expect_gap(
        front,
        rear,
        axis="x",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.002,
        max_gap=0.007,
        name="visible gap remains between the jaw backs behind the bite",
    )
    # Both claw jaws share the same Y band so the head reads as one claw.
    ctx.expect_overlap(
        front,
        rear,
        axes="y",
        elem_a="jaw",
        elem_b="jaw",
        min_overlap=0.02,
        name="jaws are coplanar full-thickness halves of one claw head",
    )

    # --- head proportions: claw ~0.06 m wide, ~0.05 m tall, above the boss ---
    f_jaw = ctx.part_element_world_aabb(front, elem="jaw")
    r_jaw = ctx.part_element_world_aabb(rear, elem="jaw")
    if f_jaw is not None and r_jaw is not None:
        width = f_jaw[1][0] - r_jaw[0][0]
        height = max(f_jaw[1][2], r_jaw[1][2]) - min(f_jaw[0][2], r_jaw[0][2])
        ctx.check(
            "claw head is ~0.06 m wide and ~0.05 m tall above the boss",
            abs(width - 0.060) < 0.008
            and abs(height - 0.050) < 0.008
            and min(f_jaw[0][2], r_jaw[0][2]) > BOSS_R,
            details=f"width={width:.4f}, height={height:.4f}",
        )
    else:
        ctx.fail("jaw aabbs available", "missing jaw element aabb")

    # --- boss: broad flattened lap joint ~0.045 m across, faces stacked ---
    f_boss = ctx.part_element_world_aabb(front, elem="boss_plate")
    if f_boss is not None:
        boss_w = f_boss[1][0] - f_boss[0][0]
        boss_h = f_boss[1][2] - f_boss[0][2]
        ctx.check(
            "flattened boss is ~0.045 m in diameter",
            abs(boss_w - 0.045) < 0.004 and abs(boss_h - 0.045) < 0.004,
            details=f"boss_w={boss_w:.4f}, boss_h={boss_h:.4f}",
        )
    else:
        ctx.fail("boss aabb available", "missing boss_plate element aabb")
    ctx.expect_gap(
        front,
        rear,
        axis="y",
        positive_elem="boss_plate",
        negative_elem="boss_plate",
        min_gap=0.0,
        max_gap=0.002,
        name="boss plates stack face to face at the rivet lap joint",
    )

    # --- rivet: shank through the bored boss, large dome capping the front ---
    ctx.expect_within(
        rear,
        front,
        axes="xz",
        inner_elem="rivet_shank",
        outer_elem="boss_plate",
        margin=0.0,
        name="rivet shank stays centered inside the front boss bore",
    )
    ctx.expect_overlap(
        rear,
        front,
        axes="y",
        elem_a="rivet_shank",
        elem_b="boss_plate",
        min_overlap=0.014,
        name="rivet shank passes through the full front boss thickness",
    )
    ctx.expect_gap(
        rear,
        front,
        axis="y",
        positive_elem="rivet_dome",
        negative_elem="boss_plate",
        min_gap=0.0,
        max_gap=0.001,
        name="large domed rivet head seats on the front boss face",
    )
    dome = ctx.part_element_world_aabb(rear, elem="rivet_dome")
    if dome is not None:
        dome_w = dome[1][0] - dome[0][0]
        ctx.check(
            "rivet dome is large (~0.024 m across) and bulges off the boss",
            0.018 <= dome_w <= 0.030 and dome[1][1] > Y_FRONT + BOSS_T + 0.003,
            details=f"dome_w={dome_w:.4f}, dome_max_y={dome[1][1]:.4f}",
        )
    else:
        ctx.fail("rivet dome aabb available", "missing rivet_dome element aabb")

    # --- handles: robust, ~0.22 m long, bowed outward, rounded worn tips ---
    f_handle = ctx.part_element_world_aabb(front, elem="handle")
    r_handle = ctx.part_element_world_aabb(rear, elem="handle")
    f_tip = ctx.part_element_world_aabb(front, elem="handle_tip")
    r_tip = ctx.part_element_world_aabb(rear, elem="handle_tip")
    handles_ok = all(v is not None for v in (f_handle, r_handle, f_tip, r_tip))
    if handles_ok:
        drop = -min(f_tip[0][2], r_tip[0][2]) - 0.008
        ctx.check(
            "handles extend ~0.22 m below the boss",
            0.20 <= drop <= 0.24,
            details=f"drop={drop:.4f}",
        )
        ctx.check(
            "handles bow outward on opposite sides",
            r_handle[1][0] > 0.035 and f_handle[0][0] < -0.035,
            details=(
                f"rear_max_x={r_handle[1][0]:.4f}, front_min_x={f_handle[0][0]:.4f}"
            ),
        )
        ctx.check(
            "rounded tip balls sit at the flared handle ends",
            f_tip[0][2] < f_handle[0][2] and r_tip[0][2] < r_handle[0][2],
            details=(
                f"front_tip_min_z={f_tip[0][2]:.4f}, "
                f"front_handle_min_z={f_handle[0][2]:.4f}"
            ),
        )
    else:
        ctx.fail("handle aabbs available", "missing handle/handle_tip element aabb")

    # --- overall: short stout tool ~0.30 m end to end ---
    if f_jaw is not None and r_jaw is not None and handles_ok:
        total = max(f_jaw[1][2], r_jaw[1][2]) - min(f_tip[0][2], r_tip[0][2])
        ctx.check(
            "overall length is about 0.30 m",
            0.28 <= total <= 0.32,
            details=f"total={total:.4f}",
        )

    # --- open pose: positive q separates the bite and splays the handle ---
    with ctx.pose({pivot: PIVOT_UPPER}):
        ctx.expect_gap(
            front,
            rear,
            axis="x",
            positive_elem="bite_edge",
            negative_elem="bite_edge",
            min_gap=0.015,
            name="open pose separates the curved jaws at the bite",
        )
        f_handle_open = ctx.part_element_world_aabb(front, elem="handle")
        if handles_ok and f_handle_open is not None:
            ctx.check(
                "open pose swings the front handle further outward",
                f_handle_open[0][0] < f_handle[0][0] - 0.04,
                details=(
                    f"closed_min_x={f_handle[0][0]:.4f}, "
                    f"open_min_x={f_handle_open[0][0]:.4f}"
                ),
            )
        else:
            ctx.fail("open handle aabb available", "missing open-pose handle aabb")

    return ctx.report()


object_model = build_object_model()
